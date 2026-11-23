"""One autonomous drone: EKF + mapping + planner + controller + energy.

Which measurements the drone has is set by its NavConfig (config.NAV_CONFIGS):
N-point camera fixes, VIO, swarm. Whatever it has, every measurement
first goes through the drone's EKF (estimator.py); guidance, control, mapping
and deconfliction only use the EKF's outputs. Ground truth is used only to
SIMULATE the sensors and to SCORE the run (metrics), never to fly.

State machine
  PREVIEW  -> on the pad, motors off: the drone plans its reference route with A* on its own map
              (the green line in Gazebo; path deviation is scored against it), and VIO drones
              calibrate their velocity bias
  TAKEOFF  -> vertical climb to cruise altitude
  SCAN     -> 360 deg yaw scan with the depth camera
  CRUISE   -> follow own path, planned in flight with RRT* or A* (config.PLANNER; its trail in
              Gazebo shows the route flown), re-planned every second (rolling) and when the map blocks it
  ESCAPE   -> fail-safe: climb above everything and go straight to the goal
  LANDING  -> descend on the goal pad
  LANDED   -> motors off
"""
import math
import numpy as np

from . import config as C
from . import downwash as DW
from .estimator import DroneEKF
from .sensors import SensorSuite
from .energy import PowerModel, Battery
from .grid_planner import polyline_blocked, line_free, PLANNER_NAMES, known_unknowns
from .geometry import closest_on_polyline, point_at_s, cumulative_s, wrap_angle
from .onboard_planner import InlinePlanner, compute_route

INFLATION = C.DRONE_RADIUS + C.PLAN_MARGIN
PLAN_INFLATION = INFLATION + C.PLAN_HYSTERESIS
AIRBORNE = ('TAKEOFF', 'SCAN', 'CRUISE', 'ESCAPE', 'LANDING')
MAP_SIGMA_MAX = 0.35        # a drone writes into the SHARED map only while its EKF sigma is below this
MAP_BARRIER_SIGMA = 0.5     # above this EKF sigma the map-based safety barrier is not trusted (depth only)
REACQUIRE_SIGMA = 0.3       # above this EKF sigma a camera fix is fused as one joint iterated update
DEPTH_MEMORY = 1.5          # s: depth hits kept for the reactive safety barrier
WIND_COMP_MAX = 1.0         # never trust a wind estimate above this (m/s) - at the default wind
GIVE_WAY_BUFFER = 0.5       # m added to the separation standard when a drone plans to give way
GIVE_WAY_HORIZON = 10.0     # s of intent broadcast over the swarm radio (where a drone will fly)
GIVE_WAY_MOVE = 3.0         # s a candidate give-way velocity is assumed to be flown (then the drone holds)
GIVE_WAY_LATERAL = 3.0      # extra cost of moving sideways off the own path (waiting is preferred)
GIVE_WAY_T = np.arange(0.0, GIVE_WAY_HORIZON + 1e-9, 0.5)
APF_RANGE = 2.7             # reactive avoidance acts this far beyond the rotor footprint (m)
LAND_RECENTER = 0.25        # m off the pad (EKF estimate) at which a landing drone stops descending to re-centre
GIVE_WAY_CLIMB = 0.2        # m/s: the climb / descent give-way manoeuvres (0.6 m over GIVE_WAY_MOVE)
GIVE_WAY_Z = (0.9, 2.5)     # m: heights a vertical give-way manoeuvre may lead to
BOXED_COST = 3000.0         # J: price of a passage turn the drone cannot take safely (no safe manoeuvre)
GIVE_WAY_KINDS = ('hold', 'slow', 'left', 'right', 'up', 'down')
LAND_HOLD_MAX = 10.0        # s a landing drone may hold its height to re-centre before it lands anyway
# rolling re-planning in flight: every REPLAN_PERIOD s (and at once when the map changes) the drone plans
# again from where it is; it switches to the new route only if that is clearly shorter than what is left
# of its current route (so RRT*'s random variation does not make it wander), or the current one is blocked
FORCED_REPLANS = ('initial', 'obstacle on path', 'off path', 'no progress')


def _clip_norm(v, vmax):
    n = float(np.linalg.norm(v))
    return v if n <= vmax or n < 1e-9 else v * (vmax / n)


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class PeerTrack:
    """Kalman filter (constant velocity) for a drone seen by the on-board depth
    camera. Drones without swarm learn about other drones only this way."""

    def __init__(self, z, t, r):
        self.x = np.array([z[0], z[1], z[2], 0.0, 0.0])
        self.P = np.diag([r, r, r, 1.0, 1.0])
        self.t = t
        self.seen = t

    def predict(self, t):
        dt = t - self.t
        if dt <= 0:
            return
        self.x[0] += self.x[3] * dt
        self.x[1] += self.x[4] * dt
        F = np.eye(5)
        F[0, 3] = F[1, 4] = dt
        q = 0.8 ** 2 * dt
        self.P = F @ self.P @ F.T + np.diag([1e-4, 1e-4, 1e-3, q, q])
        self.t = t

    def update(self, z, r):
        Hm = np.zeros((3, 5))
        Hm[0, 0] = Hm[1, 1] = Hm[2, 2] = 1.0
        S = Hm @ self.P @ Hm.T + np.eye(3) * r
        K = self.P @ Hm.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(z, float) - Hm @ self.x)
        self.P = (np.eye(5) - K @ Hm) @ self.P

    @property
    def pos(self):
        return self.x[:3].copy()

    @property
    def vel(self):
        return np.array([self.x[3], self.x[4], 0.0])


class DroneAgent:
    def __init__(self, spec, nav, world, grid, rng, speed_scale=1.0, shared_map=False, log=None, plan_rng=None):
        self.spec = spec
        self.id = spec.id
        self.name = spec.name
        self.nav = nav
        self.swarm = nav.swarm
        self.world = world
        self.map = grid
        self.shared_map = shared_map
        self.rng = rng
        # RRT* samples from its own stream, so planning never shifts the sensor noise of a mission
        self.plan_rng = plan_rng if plan_rng is not None else np.random.default_rng([spec.id, 0x525254])
        self.log = log or (lambda t, msg: None)
        self.sensors = SensorSuite(world, rng)
        sp = C.SENSORS
        self.ekf = DroneEKF(spec.spawn[0], spec.spawn[1], C.REST_Z, 0.0, wind_sigma0=sp.wind_init,
                            bias_sigma0=sp.odom_bias_init if nav.sensor else 0.0)
        self.pm = PowerModel(nav)
        self.battery = Battery()
        self.cruise = spec.cruise_speed * speed_scale
        # fixed altitude layers (no radio): every drone always cruises on its own layer
        self.cruise_alt = spec.layer_alt if nav.layers else spec.cruise_alt     # layers: one per drone
        # largest wind the drone compensates: 1 m/s, or more when it is set to fly in stronger wind
        self.wind_comp_max = max(WIND_COMP_MAX, C.WIND.mean_speed + 3.0 * C.WIND.gust_sigma)
        self.goal = np.array(spec.goal, float)
        # reference route: A* on the drone's own map, planned on the pad before take-off (scoring)
        self.ref = None
        self.ref_cum = None
        self.ref_len = 0.0
        self.ref_ms = None
        self.plan_version = 0          # +1 whenever the flown path is (re-)planned
        self.plan_ms = []              # wall-clock time of every plan (on-board computation)
        self.scans_out = []            # shared map: this drone's latest scans, sent over the radio
        self.scan_seq = 0

        self.state = 'PREVIEW'
        self.state_t0 = 0.0
        self.path = None
        self.path_cum = None
        self.path_len = 0.0
        self.s_prog = 0.0
        self.planned_version = -1
        self.plan_fail_since = None
        # the on-board route planner (onboard_planner): inline in the headless twin, a separate
        # process next to the flight loop in the ROS drone node
        self.planner = InlinePlanner()
        self.plan_job = None           # the route being computed: (reason, request time, map version, remaining m)
        self.plan_wanted = None        # a route asked for while another was being computed
        self.plan_req_t = -1e9         # when the last route was asked for (rolling re-planning)

        self.v_cmd = np.zeros(3)       # AIR-velocity command (world frame) sent to the airframe
        self.v_gnd = np.zeros(3)       # ground-velocity setpoint of the guidance (acceleration-limited)
        self.w_ff = np.zeros(2)        # wind feed-forward currently applied (v_cmd = v_gnd - w_ff)
        self.v_des = np.zeros(3)       # desired velocity over the ground
        self.acc = np.zeros(3)
        self.yaw_rate = 0.0
        self.z_int = 0.0
        self.power = 0.0
        self.trader = None             # swarm: this drone's part of the turn trading (ttc.TurnTrader)
        self.v_want = None             # velocity the drone wants before giving way (prices its passage turns)
        self.gw_alt = None             # height of a vertical give-way manoeuvre in progress
        self.gw_kind = None            # current give-way manoeuvre (GIVE_WAY_KINDS) or None
        self.yielding = False          # swarm: currently giving way to another drone
        self.yield_to = None
        self.boxed_in = False          # swarm: cannot give way (no safe velocity) -> others go round it
        self.yield_until = -1.0
        self.avoiding = False

        self.sprog_ref = 0.0
        self.sprog_ref_t = 0.0
        self.stuck_count = 0
        self.slow_time = 0.0

        self.t_cam = -1e9
        self.t_depth = -1e9
        self.t_peer = -1e9
        self.t_compass = -1e9
        self.depth_mem = []            # recent depth hits: (time, drone->obstacle vectors in world axes, EKF heading)
        self.last_markers = None
        self.look_offset = 0.0         # camera configs: yaw offset to keep landmarks in view
        self.look_t = -1e9
        self.lm_xy = np.array(C.LANDMARK_TREES, float)
        self.tracks = {}               # depth-camera peer tracks (Kalman filters)
        self.flags = {'failed': False}
        self.m = {
            'replans': 0, 'takeoff_t': None, 'land_t': None, 'cruise_t': None,
            'dist': 0.0, 'dist_h': 0.0, 'cte_sq': 0.0, 'cte_n': 0, 'cte_on': 0, 'cte_max': 0.0, 'cte_sum': 0.0,
            'loc_sq': 0.0, 'loc_n': 0, 'loc_max': 0.0,
            'yield_time': 0.0, 'avoid_time': 0.0, 'escapes': 0, 'stuck': 0,
            'gw_time': {k: 0.0 for k in GIVE_WAY_KINDS},
            'contacts': 0, 'in_contact': False,
            'landing_err': None, 'energy_parts': {'profile': 0.0, 'induced': 0.0, 'parasite': 0.0,
                                                  'climb': 0.0, 'electronics': 0.0, 'navigation': 0.0},
            'peak_power': 0.0, 'max_speed': 0.0, 'obstacles_found': 0,
            'cam_frames': 0, 'cam_fix_frames': 0, 'cam_markers': 0, 'cam_visible_sum': 0,
            'peer_updates': 0, 'sigma_sum': 0.0, 'loc_final': None, 'safety_hits': 0,
            'depth_brakes': 0, 'time_no_fix': 0.0, 'last_fix_t': None, 'max_no_fix': 0.0, 'rejected': 0,
            'hard_brakes': 0, 'nees_sum': 0.0,
            'trk_sq': 0.0, 'trk_n': 0, 'trk_max': 0.0,
        }
        # separation, drone contacts and downwash are scored by the world (mission.World), which sees
        # every drone
        self.last_cte = 0.0
        self.last_loc_err = 0.0
        self.truth = None

    # ------------------------------------------------------------------ utils
    @property
    def airborne(self):
        return self.state in AIRBORNE

    def _set_state(self, s, t, why=''):
        if s != self.state:
            self.log(t, f'{self.name}: {self.state} -> {s}' + (f' ({why})' if why else ''))
            self.state = s
            self.state_t0 = t

    @staticmethod
    def _due(t, last, rate, dt):
        """True when a sensor running at `rate` Hz produces a new sample this tick."""
        return t - last >= 1.0 / rate - 0.5 * dt

    def in_air(self):
        """Motors on and clear of the ground - according to the EKF."""
        return self.airborne and self.ekf.s[2] > C.REST_Z + 0.05

    # ------------------------------------------------------------ perception
    def sense(self, t, dt, truth, peer_truth, peer_bcast):
        """Simulate this tick's measurements and fuse ALL of them in the EKF.

        truth (p, v, yaw, roll, pitch) only feeds the sensor models and the metrics.
        peer_bcast: the other swarm drones' broadcast estimates (swarm configs only)."""
        self.truth = truth
        p, v = truth['p'], truth['v']
        sp = C.SENSORS
        nav = self.nav
        ekf = self.ekf
        tick = dt if dt > 0 else 1.0 / C.CONTROL_RATE

        # ---- prediction (sub-stepped if the simulator stalled)
        gyro = self.sensors.gyro(truth['yaw'], dt)
        n_sub = max(1, int(math.ceil(dt / 0.1)))
        for _ in range(n_sub):
            ekf.predict(self.v_cmd, gyro, dt / n_sub, self.airborne, self.in_air(), sp.track_sigma, sp.wind_rw,
                        sp.odom_bias_rw, sp.gyro_sigma)

        # ---- always on: ToF altimeter + IMU heading
        roll_m, pitch_m = self.sensors.attitude(truth['roll'], truth['pitch'])
        ekf.update_altitude(self.sensors.tof(p[2], truth['roll'], truth['pitch']), roll_m, pitch_m, sp.tof_sigma)
        if self._due(t, self.t_compass, sp.compass_rate, tick):
            self.t_compass = t
            ekf.update_heading(self.sensors.compass(truth['yaw']), math.radians(sp.compass_sigma_deg))

        # ---- VIO: measured velocity (on the pad: bias calibration)
        if nav.sensor:
            ekf.update_sensor_data(self.sensors.odometry(v, dt), sp.odom_sigma)

        if not self.airborne:
            return
        rej0 = ekf.n_rejected

        # ---- camera N-point landmark fix
        if nav.cam_points and self._due(t, self.t_cam, sp.cam_rate, tick):
            self.t_cam = t
            dets = self.sensors.camera_markers(p, truth['yaw'])
            self.last_markers = len(dets)
            self.m['cam_frames'] += 1
            self.m['cam_visible_sum'] += len(dets)
            if len(dets) >= nav.cam_points:
                used = 0
                use = dets[:nav.cam_points]
                if ekf.sigma_xy() > REACQUIRE_SIGMA:
                    # very uncertain (long gap): one joint iterated update, so the filter does not
                    # lock onto a wrong, over-confident position
                    used = ekf.update_landmarks([d['xyz'] for d in use], [d['bearing'] for d in use],
                                                [d['range'] for d in use], [d['sb'] for d in use],
                                                [d['sr'] for d in use])
                else:
                    for d in use:
                        ok_b = ekf.update_bearing(d['xyz'], d['bearing'], d['sb'])
                        ok_r = ekf.update_range(d['xyz'], d['range'], d['sr'])
                        used += int(ok_b and ok_r)
                self.m['cam_markers'] += used
                if used >= nav.cam_points:
                    self.m['cam_fix_frames'] += 1
                    self.m['last_fix_t'] = t

        # ---- swarm: UWB ranges to peers, fused at the peers' broadcast EKF estimates
        if self.swarm and peer_bcast and self._due(t, self.t_peer, sp.uwb_rate, tick):
            self.t_peer = t
            own = ekf.pos
            for pid, b in peer_bcast.items():
                if pid == self.id or pid not in peer_truth or b['state'] == 'PREVIEW':
                    continue
                r = self.sensors.uwb_peer_range(p, peer_truth[pid])
                if r is None:
                    continue
                # only learn from CLEARLY better-localised peers: drones with similar uncertainty
                # usually share the same error (same wind, no absolute reference), so ranging
                # between them would only make both over-confident
                own_sig = ekf.sigma_xy()
                if b['sigma'] >= 0.5 * own_sig:
                    continue
                q = b['pos'] + b['vel'] * tick            # broadcast is one tick old
                # NLOS rejection: skip pairs whose line of sight crosses a known obstacle
                if not line_free(self.map.obstacles_dilated(0.35), self.map, own[:2], q[:2]):
                    continue
                u = own - q
                nu = float(np.linalg.norm(u))
                if nu < 0.3:
                    continue
                u = u / nu
                # consider the peer's uncertainty; similar uncertainty usually means a COMMON error
                # (same wind, same missing references), so inflate it the more the closer they are
                extra = (1.0 + 9.0 * (b['sigma'] / own_sig) ** 2) * float(u @ b['cov'] @ u)
                sig = sp.uwb_sigma + sp.peer_sigma_per_m * r
                if ekf.update_range(q, r, sig, extra_var=extra, gate=4.0):
                    self.m['peer_updates'] += 1
        self.m['rejected'] += ekf.n_rejected - rej0          # gated camera / peer outliers

        # ---- depth image: map unknown obstacles + keep the hits for reactive braking
        if ekf.s[2] > 0.5 and self._due(t, self.t_depth, sp.depth_rate, tick):
            self.t_depth = t
            ang_b, rng, hit = self.sensors.depth_scan(p, truth['yaw'])
            ang = ekf.yaw + ang_b                       # body -> world with the EKF heading
            est = ekf.pos
            on_layer = abs(ekf.s[2] - self.cruise_alt) < 0.35          # the 2-D map is the cruise layer
            if on_layer and (not self.shared_map or ekf.sigma_xy() < MAP_SIGMA_MAX):
                r_end = rng + np.where(hit, 0.05, 0.0)
                ends = np.stack([est[0] + r_end * np.cos(ang), est[1] + r_end * np.sin(ang)], axis=1)
                newly = self.map.integrate_scan(est[:2], ends, hit)
                if self.shared_map:                      # the other drones add it to their copy of the map
                    self.scan_seq += 1
                    self.scans_out = (self.scans_out + [(self.scan_seq, est[:2].copy(), ends, hit)])[-3:]
                if newly:
                    self.m['obstacles_found'] += newly
            # short obstacle memory for the safety barrier: the hits of the last DEPTH_MEMORY seconds
            # (moved along with the EKF velocity), plus the 360 deg short-range ToF ring - it sees the
            # side of a tree the drone is flying round, which the forward camera never saw
            ra, rr, rh = self.sensors.tof_ring(p, truth['yaw'])
            ang_all = np.concatenate([ang[hit], ekf.yaw + ra[rh]])
            rng_all = np.concatenate([rng[hit], rr[rh]])
            rel = np.stack([rng_all * np.cos(ang_all), rng_all * np.sin(ang_all)], axis=1)
            self.depth_mem = [(tm, r) for (tm, r) in self.depth_mem if t - tm < DEPTH_MEMORY]
            if len(rel):
                self.depth_mem.append((t, rel))

    def track_peers(self, t, detections):
        """Depth-camera detections of other drones (body frame) -> Kalman-filtered peer tracks."""
        own = self.ekf.pos
        R = _rot(self.ekf.yaw)
        r_meas = 0.08 ** 2 + self.ekf.sigma_xy() ** 2
        for pid, rel_b in detections:
            z = own + np.array([*(R @ rel_b[:2]), rel_b[2]])
            tr = self.tracks.get(pid)
            if tr is None or t - tr.seen > 1.0:
                self.tracks[pid] = PeerTrack(z, t, r_meas)
            else:
                tr.predict(t)
                tr.update(z, r_meas)
                tr.seen = t
        out = []
        for pid, tr in list(self.tracks.items()):
            if t - tr.seen > 1.0:
                del self.tracks[pid]
                continue
            tr.predict(t)
            out.append({'id': pid, 'pos': tr.pos, 'vel': tr.vel, 'radio': False, 'airborne': True})
        return out

    # --------------------------------------------------------------- planning
    def _reference_plan(self, t):
        """On the pad, before take-off: the shortest route with A* on what the drone knows (the known
        trees). The green line in Gazebo; path deviation is scored against it. Computed on the drone,
        while it stands on its pad (no flight energy), and not counted as in-flight planning."""
        path, self.ref_ms = compute_route(self.map, tuple(self.spec.spawn), tuple(self.goal), 'astar', 0,
                                          (PLAN_INFLATION, INFLATION))
        if path is None:                             # cannot happen in a feasible forest
            path = [tuple(self.spec.spawn), tuple(self.spec.goal)]
        self.ref = np.array(path, float)
        self.ref_cum = cumulative_s(self.ref)
        self.ref_len = float(self.ref_cum[-1])
        self.log(t, f'{self.name}: reference route (A*, before take-off) {self.ref_len:.1f} m')

    def _remaining(self):
        """Length of what is left of the current route, from where the drone is (m)."""
        est = self.ekf.pos
        p = point_at_s(self.path, self.s_prog, self.path_cum)
        return float(np.linalg.norm(est[:2] - p)) + max(0.0, self.path_len - self.s_prog)

    def _route_blocked(self):
        return polyline_blocked(self.map.inflated(INFLATION), self.map, self.path, from_s=self.s_prog + 0.3)

    def _request_route(self, t, reason):
        """Ask the on-board planner for a route from where the drone is to its pad. One route is computed
        at a time; a request while one is being computed is kept (a forced reason wins) and sent next."""
        if self.planner.busy():
            if self.plan_wanted is None or reason in FORCED_REPLANS:
                self.plan_wanted = reason
            return
        est = self.ekf.pos
        seed = int(self.plan_rng.integers(0, 2 ** 63 - 1))       # the drone's planning stream
        remaining = self._remaining() if self.path is not None else None
        self.plan_job = (reason, t, self.map.version, remaining)
        self.plan_req_t = t
        self.plan_wanted = None
        self.planner.submit(self.map, (float(est[0]), float(est[1])), tuple(self.goal), C.PLANNER, seed,
                            (PLAN_INFLATION, INFLATION))
        self._receive_route(t)                       # inline planner: the route is ready at once

    def _receive_route(self, t):
        """Take a route from the planner when it is ready and decide: adopt it, keep the current route,
        or ask again (the map changed while it was computed and the new route crosses something)."""
        res = self.planner.poll()
        if res is None:
            return
        path, ms = res
        reason, t_req, map_version, remaining = self.plan_job
        self.plan_job = None
        self.plan_ms.append(ms)
        if self.state not in ('SCAN', 'CRUISE'):     # escaping or landing meanwhile: not needed any more
            self.plan_wanted = None
            return
        forced = reason in FORCED_REPLANS or self.path is None
        if path is not None and self.map.version != map_version and polyline_blocked(
                self.map.inflated(INFLATION), self.map, np.asarray(path, float), from_s=0.3):
            path = None                              # computed on an older map: crosses a newly found tree
            self.plan_wanted = self.plan_wanted or reason
        if path is None:
            if (forced or self._route_blocked()) and self.plan_fail_since is None:
                self.plan_fail_since = t
            return
        self.plan_fail_since = None                  # a route exists (adopted or not)
        path = np.array(path, float)
        cum = cumulative_s(path)
        if not forced and not self._route_blocked():
            # keep the current route unless the new one is clearly shorter than what is left of it
            if float(cum[-1]) > remaining - max(C.REPLAN_GAIN_M, C.REPLAN_GAIN * remaining):
                return
            reason = reason + ', shorter'
        first = self.path is None
        self.path, self.path_cum, self.path_len = path, cum, float(cum[-1])
        self.s_prog = 0.0
        if self.state == 'CRUISE' and t > t_req:      # computed while the drone flew on (planner process)
            _, s_near, _ = closest_on_polyline(self.ekf.pos, self.path, self.path_cum)
            if s_near <= 2.0:
                self.s_prog = s_near
        self.planned_version = map_version
        if map_version == self.map.version:
            self.plan_wanted = None                  # planned on the current map: nothing left to ask for
        self.sprog_ref, self.sprog_ref_t = self.s_prog, t
        self.plan_version += 1
        if not first:
            self.m['replans'] += 1
            self.log(t, f'{self.name}: re-planned ({reason}), new path {self.path_len:.1f} m')

    def _check_replan(self, t):
        """In flight, every control step: take a finished route, and ask for a new one at once when a
        newly found obstacle blocks the route or the drone has lost it, and otherwise every
        REPLAN_PERIOD s (rolling re-planning, which also takes in every other change of the map)."""
        self._receive_route(t)
        if self.path is None:
            self._request_route(t, 'initial')
            return
        if self.map.version != self.planned_version:
            self.planned_version = self.map.version
            if self._route_blocked():                # a newly found obstacle on the route: at once
                self._request_route(t, 'obstacle on path')
                return
        est = self.ekf.pos
        d, s_near, _ = closest_on_polyline(est, self.path, self.path_cum)
        if d > 1.3:
            # pushed off the path (e.g. by another drone): fly straight back to it if nothing is
            # in the way, so the drone keeps its route; re-plan only when the way back is blocked
            back = point_at_s(self.path, max(s_near, self.s_prog) + 1.0, self.path_cum)
            if d > 3.0 or not line_free(self.map.inflated(INFLATION), self.map, est[:2], back):
                self._request_route(t, 'off path')
                return
        if self.plan_wanted is not None:
            self._request_route(t, self.plan_wanted)
        elif t - self.plan_req_t >= C.REPLAN_PERIOD - 1e-9:
            self._request_route(t, 'rolling')

    # ---------------------------------------------------------------- control
    def _peer_avoidance(self, est, peers):
        """Reactive avoidance of nearby drones (APF repulsion + tangential curl), at ANY height:
        with downwash a drone above or below is as much in the way as one at the same height,
        only the clearance differs (downwash.required_separation). The field acts within
        APF_RANGE of the required clearance and has the same shape at every height.
        Swarm drones use the give-way instead (_give_way); they only go round a drone that reports
        it cannot get out of the way and does not pass before them (lower priority, or a later turn
        in the turn trading)."""
        v_rep = np.zeros(2)
        v_curl = np.zeros(2)
        k_rep, k_curl = 1.6, 1.0
        active = False
        base = 2 * C.DRONE_RADIUS
        for pr in peers:
            buf = C.REACTIVE_BUFFER
            if self.swarm and pr.get('radio'):                 # a radio peer (not only a sensor track)
                if C.GIVE_WAY == 'priority':
                    if not (pr.get('boxed_in') and pr['priority'] > self.spec.priority):
                        continue
                elif not self._covered(pr['id']):              # its round message did not arrive in time:
                    buf = GIVE_WAY_BUFFER                       # avoid it like a drone without the radio
                elif not (pr.get('boxed_in') and pr['id'] not in self._ahead_ids()):
                    continue
            rel = est[:2] - pr['pos'][:2]
            dh = max(float(np.linalg.norm(rel)), 0.01)
            sep = DW.required_separation(est[2] - pr['pos'][2]) + buf
            de = max(dh - (sep - base), 0.05)          # distance measured from the clearance boundary
            if de < APF_RANGE:
                f = (1.0 / de - 1.0 / APF_RANGE) ** 2
                u = rel / dh
                v_rep += k_rep * f * u
                v_curl += k_curl * f * np.array([-u[1], u[0]])
                active = True
        self.avoiding = active
        return _clip_norm(v_rep + v_curl, 1.8)

    def intent(self):
        """Where this drone intends to be over the next GIVE_WAY_HORIZON s (swarm radio broadcast),
        sampled every 0.5 s: along its own path at its current speed while cruising, otherwise
        where it is now (escaping: straight on)."""
        est = self.ekf.pos[:2]
        if self.state == 'CRUISE' and self.path is not None and len(self.path) > 1:
            V = max(0.3, float(np.linalg.norm(self.v_des[:2])))
            pts = [point_at_s(self.path, self.s_prog + V * tk, self.path_cum) for tk in GIVE_WAY_T]
            out = np.array(pts, float)
            out += (est - out[0]) * np.exp(-GIVE_WAY_T / 1.0)[:, None]     # from where it is now
            return out
        v = self.ekf.vel[:2] if self.state == 'ESCAPE' else np.zeros(2)
        return est[None, :] + GIVE_WAY_T[:, None] * v[None, :]

    def route_intent(self):
        """Where this drone will be over the next GIVE_WAY_HORIZON s if it simply follows its route at
        its planned speed (swarm radio broadcast): what the turn trading checks for conflicts. A drone
        that is waiting still announces where it wants to go, so its conflict stays known."""
        est = self.ekf.pos[:2]
        if self.state == 'CRUISE' and self.path is not None and len(self.path) > 1:
            V = max(0.3, self._path_speed())
            pts = [point_at_s(self.path, self.s_prog + V * tk, self.path_cum) for tk in GIVE_WAY_T]
            out = np.array(pts, float)
            out += (est - out[0]) * np.exp(-GIVE_WAY_T / 1.0)[:, None]     # from where it is now
            return out
        if self.state == 'ESCAPE':
            return est[None, :] + GIVE_WAY_T[:, None] * self.ekf.vel[None, :2]
        return None

    def _covered(self, pid):
        """Turn trading: did this drone's last round include the message of drone pid (or is there no round
        to check)? A drone that is not covered is avoided reactively, with the give-way buffer (ttc.py)."""
        tr = self.trader
        return tr is None or tr.covered is None or pid in tr.covered

    def _ahead_ids(self):
        """Drones that pass before this one in its cluster's turn order (turn trading)."""
        turn = self.trader.turn if self.trader is not None else None
        return turn['ahead'] if turn else set()

    def _give_way(self, t, est, v_xy, peers):
        """Swarm give-way: by turn trading (C.GIVE_WAY == 'ttc', ttc.py) or by fixed priority."""
        self.v_want = np.asarray(v_xy, float).copy()
        if C.GIVE_WAY == 'priority':
            self.gw_alt = None
            out = self._give_way_priority(t, est, v_xy, peers)
            self.gw_kind = self._kind(out, v_xy) if self.yielding else None
            return out
        return self._give_way_ttc(t, est, v_xy, peers)

    def _give_way_ttc(self, t, est, v_xy, peers):
        """Give way to the drones that pass before this one in its cluster's turn order (ttc.py),
        with the manoeuvre of least energy that keeps the separation standard + GIVE_WAY_BUFFER to
        their broadcast intents: continue, slow down, hold, step left or right (the 32 horizontal
        candidates), climb or descend 0.6 m. The drones after it give way to it; it only makes sure
        not to cut into them within 1.5 s. A drone with no drone before it flies on. If no manoeuvre
        is safe it reports itself boxed in, and the others go round it."""
        self.boxed_in = False
        ahead = self._ahead_ids() if self.swarm else set()
        near = [pr for pr in peers if pr.get('airborne', True) and pr.get('radio')
                and float(np.linalg.norm(est[:2] - pr['pos'][:2])) < 10.0]
        hp = [pr for pr in near if pr['id'] in ahead]
        if not hp:
            self._set_yield(t, None)
            self.gw_alt, self.gw_kind = None, None
            return v_xy
        lp = [pr for pr in near if pr['id'] not in ahead]
        cands = self._give_way_candidates3(v_xy, est)
        slack, culprit, worst0 = self._gw_slack(est, cands, hp, lp)
        if slack[0] >= 0.0:
            self._set_yield(t, None)
            self.gw_alt, self.gw_kind = None, None
            return v_xy
        energy = self._gw_energy(v_xy, cands)
        ok = slack >= 0.0
        k = int(np.argmin(np.where(ok, energy, np.inf))) if ok.any() else int(np.argmax(slack))
        self.boxed_in = not ok.any()
        self._set_yield(t, culprit if worst0 < 0 else None)
        c = cands[k]
        self.gw_alt = float(est[2] + c[2] * GIVE_WAY_MOVE) if abs(c[2]) > 1e-9 else None
        self.gw_kind = self._kind(c, v_xy) if self.yielding else None
        return c[:2].copy()

    def turn_cost(self, ahead_peers):
        """Price (J) of a passage turn behind the drones `ahead_peers` (their broadcasts; they fly on along
        their route intents): the extra energy of the cheapest way to keep the separation standard +
        GIVE_WAY_BUFFER to all of them over the next GIVE_WAY_HORIZON s -
          hold or slow down: the route intent delayed by the shortest sufficient delay (0.5 s steps),
          step left or right: the route offset sideways by 0.6-2.4 m (reached within 2 s, then held),
          climb or descend 0.6 m on the route (the downwash-aware standard only grows with the height
          difference, so on its own this never resolves a conflict);
        0 if the drone can simply fly on, BOXED_COST if nothing works. Energy = flight time lost x the
        drone's power (+ the climb). Computed on the drone from its own route, map and energy model
        (turn trading, ttc.py)."""
        route = self.route_intent()
        if not ahead_peers or route is None or not self.airborne:
            return 0.0
        z = float(self.ekf.pos[2])
        others = [(np.asarray(pr['route'] if pr.get('route') is not None else pr['intent'], float),
                   float(pr['pos'][2])) for pr in ahead_peers]
        K = len(route)

        def ok(track, zz=z):
            for tr, zp in others:
                k = min(K, len(tr))
                if (np.linalg.norm(track[:k] - tr[:k], axis=1) - DW.required_separation(zz - zp)
                        - GIVE_WAY_BUFFER).min() < 0.0:
                    return False
            return True

        if ok(route):
            return 0.0
        p_fly = self.pm.hover_power()
        v_ref = max(self.cruise, 0.3)
        best = np.inf
        idx = np.arange(K)
        for k in range(1, K):                                    # hold / slow down: wait k x 0.5 s
            if ok(route[np.maximum(idx - k, 0)]):
                best = p_fly * k * float(GIVE_WAY_T[1])
                break
        tang = np.gradient(route, axis=0)
        nrm = np.linalg.norm(tang, axis=1, keepdims=True)
        if float(nrm.max()) > 1e-6:                              # step left / right (needs a moving route)
            tang = np.where(nrm > 1e-6, tang / np.maximum(nrm, 1e-9), tang[np.argmax(nrm[:, 0])] / nrm.max())
            left = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
            ramp = np.minimum(1.0, GIVE_WAY_T / 2.0)[:, None]
            blk = self.map.inflated(INFLATION) if self.ekf.sigma_xy() < MAP_BARRIER_SIGMA else None
            a = v_ref * 2.0
            for off in (0.6, 1.2, 1.8, 2.4):
                e = p_fly * 2.0 * (math.hypot(a, off) - a) / v_ref
                if e >= best:
                    break
                for side in (1.0, -1.0):
                    tr = route + side * off * ramp * left
                    if blk is not None:
                        gm = self.map
                        ii = np.clip(np.floor((tr[:, 0] - gm.origin) / gm.res).astype(int), 0, gm.n - 1)
                        jj = np.clip(np.floor((tr[:, 1] - gm.origin) / gm.res).astype(int), 0, gm.n - 1)
                        if blk[ii, jj].any():
                            continue
                    if ok(tr):
                        best = e
                        break
                if best <= e:
                    break
        climb = self.pm.W * GIVE_WAY_CLIMB * GIVE_WAY_MOVE / C.ENERGY.eta_climb
        if climb < best:                                         # climb / descend 0.6 m on the route
            for dz in (GIVE_WAY_CLIMB * GIVE_WAY_MOVE, -GIVE_WAY_CLIMB * GIVE_WAY_MOVE):
                if GIVE_WAY_Z[0] <= z + dz <= GIVE_WAY_Z[1] and ok(route, z + dz):
                    best = climb
                    break
        return float(best) if np.isfinite(best) else BOXED_COST

    def _give_way_candidates3(self, v_xy, est):
        """The horizontal give-way candidates (vz = 0; cands[0] = the wanted velocity) plus climbing and
        descending 0.6 m, flying on or holding position, where that height stays within GIVE_WAY_Z."""
        out = [np.append(c, 0.0) for c in self._give_way_candidates(v_xy)]
        for vz in (GIVE_WAY_CLIMB, -GIVE_WAY_CLIMB):
            z1 = est[2] + vz * GIVE_WAY_MOVE
            if GIVE_WAY_Z[0] <= z1 <= GIVE_WAY_Z[1]:
                out += [np.array([v_xy[0], v_xy[1], vz]), np.array([0.0, 0.0, vz])]
        return np.array(out)

    def _gw_slack(self, est, cands, hp, lp):
        """Smallest (distance - required distance) of each candidate over the look-ahead, against the
        intents of the drones it gives way to (hp: + GIVE_WAY_BUFFER, 10 s) and the current velocity of
        the others (lp: 1.5 s); candidates leading into an obstacle get -5. Returns (slack, the drone it
        gives way to first, slack of the wanted velocity)."""
        T = GIVE_WAY_T
        tm = np.minimum(T, GIVE_WAY_MOVE)
        own = est[None, None, :2] + cands[:, None, :2] * tm[None, :, None]
        vert = np.nonzero(np.abs(cands[:, 2]) > 1e-9)[0]
        own_z = est[2] + cands[:, 2:3] * tm[None, :]
        slack = np.full(len(cands), np.inf)
        culprit, worst0 = None, np.inf
        for pr, buf, horizon in [(q, GIVE_WAY_BUFFER, len(T)) for q in hp] + [(q, 0.0, 4) for q in lp]:
            pos = np.asarray(pr['pos'], float)
            vel = np.asarray(pr['vel'], float)[:2]
            req0 = DW.required_separation(est[2] - pos[2]) + buf
            req = np.full((len(cands), len(T)), req0)
            for r in vert:                                  # climbing / descending: the standard per height
                req[r] = [DW.required_separation(z - pos[2]) + buf for z in own_z[r]]
            tracks = [pos[None, :2] + T[:4, None] * vel[None, :]]       # current velocity, 1.5 s
            it = pr.get('intent')
            if it is not None and buf > 0:
                tracks.append(np.asarray(it, float)[:horizon])
            s_pr = np.full(len(cands), np.inf)
            for tr in tracks:
                k = min(len(tr), horizon)
                d = np.linalg.norm(own[:, :k, :] - tr[None, :k, :], axis=-1)
                s_pr = np.minimum(s_pr, (d - req[:, :k]).min(axis=1))
            d0 = float(np.linalg.norm(est[:2] - pos[:2]))
            if d0 < req0:                                   # already too close: open the distance
                u = (est[:2] - pos[:2]) / max(d0, 1e-6)
                s_pr = ((cands[:, :2] - vel[None, :]) @ u) - 0.2
            slack = np.minimum(slack, s_pr)
            if buf > 0 and s_pr[0] < worst0:
                worst0, culprit = float(s_pr[0]), pr['id']
        if self.ekf.sigma_xy() < MAP_BARRIER_SIGMA:
            blk = self.map.inflated(C.DRONE_RADIUS + 0.1)
            gm = self.map
            pts = est[None, None, :2] + cands[:, None, :2] * np.array([0.5, 1.0, 1.5])[None, :, None]
            ii = np.clip(np.floor((pts[..., 0] - gm.origin) / gm.res).astype(int), 0, gm.n - 1)
            jj = np.clip(np.floor((pts[..., 1] - gm.origin) / gm.res).astype(int), 0, gm.n - 1)
            hit = blk[ii, jj].any(axis=1) & (np.linalg.norm(cands[:, :2], axis=1) > 0.05)
            slack = np.where(hit, np.minimum(slack, -5.0), slack)
        return slack, culprit, worst0

    def _gw_energy(self, v_xy, cands):
        """Extra energy (J) of each give-way candidate, flown for GIVE_WAY_MOVE s: the route it loses
        (slower progress along the route, plus the longer way out to the side and back) costs flight
        time at the drone's power, and a climb or descent costs the climb (now, or on the way back up).
        Energy follows flight time (the rotors draw over 90 % of the power), so waiting costs about
        what it delays the drone."""
        V = float(np.linalg.norm(v_xy))
        u = v_xy / V if V > 0.05 else np.array([math.cos(self.ekf.yaw), math.sin(self.ekf.yaw)])
        along = cands[:, :2] @ u
        a = np.maximum(along, 0.0) * GIVE_WAY_MOVE                     # route covered during the move
        off = np.linalg.norm(cands[:, :2] - along[:, None] * u[None, :], axis=1) * GIVE_WAY_MOVE
        lost = np.maximum(0.0, V - along) * GIVE_WAY_MOVE + 2.0 * (np.hypot(a, off) - a)   # out and back
        climb = self.pm.W * np.abs(cands[:, 2]) * GIVE_WAY_MOVE / C.ENERGY.eta_climb
        return self.pm.hover_power() * lost / max(self.cruise, 0.3) + climb

    @staticmethod
    def _kind(c, v_xy):
        """The give-way manoeuvre a velocity stands for: up / down, hold, slow (along the route), left / right."""
        if len(c) > 2 and abs(c[2]) > 1e-9:
            return 'up' if c[2] > 0 else 'down'
        if float(np.linalg.norm(c[:2])) < 0.05:
            return 'hold'
        V = float(np.linalg.norm(v_xy))
        if V < 0.05:
            return 'slow'
        u = v_xy / V
        cross = u[0] * c[1] - u[1] * c[0]
        along = float(c[:2] @ u)
        if along > 0 and abs(cross) < 0.35 * along:
            return 'slow'
        return 'left' if cross > 0 else 'right'

    def _give_way_priority(self, t, est, v_xy, peers):
        """Swarm priority give-way, kept for comparison (C.GIVE_WAY == 'priority'): fixed priorities, the
        drones in drone order, drone1 first (replaces climbing to a yield layer, which downwash rules out).
        A lower-priority drone checks candidate velocities (flown for GIVE_WAY_MOVE s, then holding)
        against the INTENT every higher-priority drone broadcasts (its next GIVE_WAY_HORIZON s along
        its own path, plus its current velocity for the next 1.5 s) and flies the one closest to the
        velocity it wants that keeps the separation standard + GIVE_WAY_BUFFER and does not lead
        into an obstacle: waiting first, stepping aside only when it must. It never parks on a
        route a higher-priority drone is about to fly, never cuts into a lower-priority drone
        (1.5 s look-ahead), and a drone already too close must open the distance. If no velocity
        is safe it reports itself boxed in, and higher-priority drones go round it."""
        self.boxed_in = False
        if not self.swarm or self.spec.priority == 0:
            self._set_yield(t, None)
            return v_xy
        near = [pr for pr in peers if pr.get('airborne', True) and pr.get('radio')
                and float(np.linalg.norm(est[:2] - pr['pos'][:2])) < 10.0]
        hp = [pr for pr in near if pr['priority'] < self.spec.priority]
        if not hp:
            self._set_yield(t, None)
            return v_xy
        lp = [pr for pr in near if pr['priority'] > self.spec.priority]
        T = GIVE_WAY_T
        cands = self._give_way_candidates(v_xy)                       # (m, 2), cands[0] = v_xy
        # own track: the candidate velocity for GIVE_WAY_MOVE s, then holding position
        own = est[None, None, :2] + cands[:, None, :] * np.minimum(T, GIVE_WAY_MOVE)[None, :, None]
        slack = np.full(len(cands), np.inf)
        culprit, worst0 = None, np.inf
        for pr, buf, horizon in [(q, GIVE_WAY_BUFFER, len(T)) for q in hp] + [(q, 0.0, 4) for q in lp]:
            pos = np.asarray(pr['pos'], float)[:2]
            vel = np.asarray(pr['vel'], float)[:2]
            req = DW.required_separation(est[2] - pr['pos'][2]) + buf
            tracks = [pos[None, :] + T[:4, None] * vel[None, :]]       # current velocity, 1.5 s
            it = pr.get('intent')
            if it is not None and buf > 0:
                tracks.append(np.asarray(it, float)[:horizon])
            s_pr = np.full(len(cands), np.inf)
            for tr in tracks:
                k = min(len(tr), horizon)
                d = np.linalg.norm(own[:, :k, :] - tr[None, :k, :], axis=-1).min(axis=1)
                s_pr = np.minimum(s_pr, d - req)
            d0 = float(np.linalg.norm(est[:2] - pos))
            if d0 < req:                                              # already too close: open up
                # feasibility: open the distance at >= 0.2 m/s. Note: this slack is a speed (m/s) while the
                # others are distances (m); the difference matters only when no candidate is safe (argmax
                # below), where the move that opens the distance fastest is taken.
                u = (est[:2] - pos) / max(d0, 1e-6)
                s_pr = ((cands - vel[None, :]) @ u) - 0.2
            slack = np.minimum(slack, s_pr)
            if buf > 0 and s_pr[0] < worst0:
                worst0, culprit = float(s_pr[0]), pr['id']
        # a give-way move must not lead into an obstacle (the safety filter would cancel it):
        # where it takes the drone in 0.5 / 1 / 1.5 s must be free in its own map
        if self.ekf.sigma_xy() < MAP_BARRIER_SIGMA:
            blk = self.map.inflated(C.DRONE_RADIUS + 0.1)
            gm = self.map
            pts = est[None, None, :2] + cands[:, None, :] * np.array([0.5, 1.0, 1.5])[None, :, None]
            ii = np.clip(np.floor((pts[..., 0] - gm.origin) / gm.res).astype(int), 0, gm.n - 1)
            jj = np.clip(np.floor((pts[..., 1] - gm.origin) / gm.res).astype(int), 0, gm.n - 1)
            hit = blk[ii, jj].any(axis=1) & (np.linalg.norm(cands, axis=1) > 0.05)
            slack = np.where(hit, np.minimum(slack, -5.0), slack)
        if slack[0] >= 0.0:
            self._set_yield(t, None)
            return v_xy
        # closest to the wanted velocity; moving sideways off the own path costs extra, so the drone
        # waits (time separation) when that is enough and steps aside only when it must
        V = float(np.linalg.norm(v_xy))
        u = v_xy / V if V > 0.05 else np.zeros(2)
        lateral = cands - (cands @ u)[:, None] * u[None, :]
        cost = ((cands - v_xy[None]) ** 2).sum(-1) + GIVE_WAY_LATERAL * (lateral ** 2).sum(-1)
        ok = slack >= 0.0
        k = int(np.argmin(np.where(ok, cost, np.inf))) if ok.any() else int(np.argmax(slack))
        self.boxed_in = not ok.any()
        self._set_yield(t, culprit if worst0 < 0 else None)
        return cands[k]

    def _give_way_candidates(self, v_xy):
        V = float(np.linalg.norm(v_xy))
        u = v_xy / V if V > 0.05 else np.array([math.cos(self.ekf.yaw), math.sin(self.ekf.yaw)])
        sp = max(V, 0.6)
        out = [v_xy]
        out += [s * v_xy for s in (0.7, 0.45, 0.2)]
        for ang in (30, -30, 60, -60, 90, -90, 135, -135, 180):
            R = _rot(math.radians(ang))
            for s in (1.0, 0.6, 0.3):
                out.append(s * sp * (R @ u))
        out.append(np.zeros(2))
        return np.array(out)

    def _set_yield(self, t, peer_id):
        if peer_id is not None:
            if not self.yielding:
                self.log(t, f'{self.name}: giving way to drone{peer_id}')
            self.yielding, self.yield_to, self.yield_until = True, peer_id, t + 0.5
        elif self.yielding and t > self.yield_until:
            self.yielding, self.yield_to = False, None
            self.log(t, f'{self.name}: way clear')

    @staticmethod
    def _barrier(v_xy, n, gap, a_brake, v_other=None, share=1.0, push=True):
        """Braking-distance barrier: closing speed towards an obstacle (along -n) may not
        exceed sqrt(2 a gap). Tangential motion is untouched.
        v_other: velocity of a moving obstacle (another drone): the CLOSING speed is limited,
        and with share=0.5 each of the two drones removes half of the excess.
        push: inside the margin (gap <= 0) also back off."""
        v_rel = v_xy if v_other is None else v_xy - v_other
        v_in = -float(v_rel @ n)
        v_allow = math.sqrt(2.0 * a_brake * max(0.0, gap))
        out = v_xy
        if v_in > v_allow:
            out = out + share * (v_in - v_allow) * n
        if push and gap <= 0.0:
            out = out + min(0.4, 0.3 - 2.0 * gap) * n      # already too close: back off
        return out

    def _safety_filter(self, t, est, v_xy, peers, count=True):
        """Safety layer on the drone's OWN knowledge (EKF pose, map, tracked peers,
        latest depth hits). Keeps reactive peer avoidance from shoving a drone into a
        tree and stops head-on encounters before rotors touch."""
        a_brake = 0.6 * C.ACC_MAX_H
        out = v_xy.copy()
        hit = False
        static = []                                          # (direction, gap) of every nearby obstacle
        above_all = est[2] > C.OBSTACLE_TOP + 0.5            # nothing to hit up here except drones
        # unknown wind can push the drone on after it brakes: keep a margin of 1 s of
        # the EKF's wind uncertainty (tiny when the wind is well estimated)
        wind_margin = 1.0 * math.sqrt(max(0.0, 0.5 * (self.ekf.P[4, 4] + self.ekf.P[5, 5])))
        # peers at any height: keep the downwash-aware separation standard
        for pr in peers:
            rel = est[:2] - pr['pos'][:2]
            d = float(np.linalg.norm(rel))
            if pr.get('radio'):
                buf = 0.0 if self._covered(pr.get('id')) else GIVE_WAY_BUFFER
            else:
                buf = C.REACTIVE_BUFFER
            sep = DW.required_separation(est[2] - pr['pos'][2]) + buf
            if d < sep + 1.8 and d > 1e-3:
                gap = d - sep
                before = out.copy()
                out = self._barrier(out, rel / d, gap, a_brake, np.asarray(pr['vel'], float)[:2], share=0.5)
                hit |= not np.allclose(before, out)
        # obstacles known in the map - only while the EKF says the drone knows where it is
        # (with a drifted estimate the map would put the trees in the wrong place)
        gm = self.map
        occ = gm.occupied()
        i0, j0 = gm.to_cell(est[0], est[1])
        r = int(math.ceil(1.8 / gm.res))
        a0, a1 = max(0, i0 - r), min(gm.n, i0 + r + 1)
        b0, b1 = max(0, j0 - r), min(gm.n, j0 + r + 1)
        sub = occ[a0:a1, b0:b1]
        if not above_all and self.ekf.sigma_xy() < MAP_BARRIER_SIGMA and np.any(sub):
            cx = gm.cx[a0:a1, b0:b1][sub]
            cy = gm.cy[a0:a1, b0:b1][sub]
            rel = np.stack([est[0] - cx, est[1] - cy], axis=1)
            dist = np.linalg.norm(rel, axis=1)
            # nearest cell in each of 8 directions -> handles corridors / two trees
            ang = np.floor((np.arctan2(rel[:, 1], rel[:, 0]) + math.pi) / (math.pi / 4)).astype(int) % 8
            for sector in np.unique(ang):
                m = np.where(ang == sector)[0]
                k = m[np.argmin(dist[m])]
                gap = dist[k] - 0.5 * gm.res - C.DRONE_RADIUS - 0.08 - wind_margin
                if gap < 1.0:
                    before = out.copy()
                    n = rel[k] / max(dist[k], 1e-6)
                    out = self._barrier(out, n, gap, a_brake)
                    static.append((n, gap))
                    hit |= not np.allclose(before, out)
        # recent depth hits relative to the drone (moved on with the EKF velocity):
        # still protects the drone when its absolute position estimate has drifted
        mem = [(tm, r) for (tm, r) in self.depth_mem if t - tm < DEPTH_MEMORY]
        if mem and not above_all:
            v = self.ekf.vel[:2]
            rel = -np.concatenate([r - v * (t - tm) for (tm, r) in mem])         # obstacle -> drone
            dist = np.linalg.norm(rel, axis=1)
            ang = np.floor((np.arctan2(rel[:, 1], rel[:, 0]) + math.pi) / (math.pi / 4)).astype(int) % 8
            for sector in np.unique(ang):
                m = np.where(ang == sector)[0]
                k = m[np.argmin(dist[m])]
                gap = dist[k] - C.DRONE_RADIUS - 0.10 - wind_margin
                if gap < 1.0:
                    before = out.copy()
                    n = rel[k] / max(dist[k], 1e-6)
                    out = self._barrier(out, n, gap, a_brake)
                    static.append((n, gap))
                    if not np.allclose(before, out):
                        hit = True
                        if count:
                            self.m['depth_brakes'] += 1
        # backing off from one obstacle must not drive the drone into another (a narrow gap between
        # two trees): enforce every obstacle's closing-speed limit again until they all hold
        for _ in range(3):
            before = out.copy()
            for n, gap in static:
                out = self._barrier(out, n, gap, a_brake, push=False)
            if np.allclose(before, out):
                break
        if hit and count:
            self.m['safety_hits'] += 1
        return out

    def _look_offset(self, t, est, travel_yaw):
        """Active perception for a narrow camera: choose a heading offset (within
        +-cam_look_offset_deg of the travel direction, so the depth camera still sees
        ahead) that keeps the most known landmarks inside the camera view.
        Not needed with the default 360 deg camera ring."""
        if t - self.look_t < 0.5:
            return self.look_offset
        self.look_t = t
        sp = C.SENSORS
        rel = self.lm_xy - est[:2]
        d = np.hypot(rel[:, 0], rel[:, 1])
        brg = np.arctan2(rel[:, 1], rel[:, 0])
        ok = (d > 0.6) & (d < 0.85 * sp.cam_range)
        half = math.radians(sp.cam_fov_deg) / 2.0 - math.radians(6.0)
        lim = math.radians(sp.cam_look_offset_deg)
        best, best_score = 0.0, -1e9
        for off in np.linspace(-lim, lim, 7):
            a = np.abs((brg - (travel_yaw + off) + math.pi) % (2 * math.pi) - math.pi)
            inview = ok & (a < half)
            n = int(np.sum(inview))
            score = 10.0 * min(n, self.nav.cam_points + 1) + float(np.sum(1.0 / d[inview])) - 2.0 * abs(off)
            if score > best_score:
                best, best_score = float(off), score
        self.look_offset = 0.6 * self.look_offset + 0.4 * best     # smooth changes
        return self.look_offset

    def _path_speed(self):
        V = self.cruise
        # slow down before sharp corners
        ahead = np.where(self.path_cum > self.s_prog + 1e-3)[0]
        for k in ahead:
            if k >= len(self.path) - 1:
                break
            dist = self.path_cum[k] - self.s_prog
            if dist > 3.0:
                break
            a = self.path[k] - self.path[k - 1]
            b = self.path[k + 1] - self.path[k]
            na, nb = np.linalg.norm(a), np.linalg.norm(b)
            if na < 1e-6 or nb < 1e-6:
                continue
            th = math.acos(max(-1.0, min(1.0, float(a @ b) / (na * nb))))
            v_c = self.cruise * max(0.35, 1.0 - 0.85 * th / math.pi)
            V = min(V, math.sqrt(v_c ** 2 + 2 * 0.6 * C.ACC_MAX_H * max(0.0, dist)))
        remaining = self.path_len - self.s_prog
        V = min(V, max(0.25, math.sqrt(2 * 0.5 * C.ACC_MAX_H * max(0.0, remaining))))
        return V

    def control(self, t, dt, peers):
        """Guidance + control from the EKF estimate only.
        Returns (air-velocity command in the world frame (3,), yaw rate)."""
        est = self.ekf.pos
        yaw = self.ekf.yaw
        st = self.state
        v_t = np.zeros(3)              # desired velocity over the ground
        yaw_rate = 0.0
        z_target = None

        if st == 'PREVIEW':
            if self.ref is None:
                self._reference_plan(t)
            if t >= C.PREVIEW_TIME:
                self.m['takeoff_t'] = t
                self._set_state('TAKEOFF', t)
            self.v_cmd[:] = 0.0
            self.v_gnd[:] = 0.0
            self.w_ff[:] = 0.0
            self.v_des[:] = 0.0
            self.acc[:] = 0.0
            self.yaw_rate = 0.0
            self.power = 0.0
            return self.v_cmd.copy(), 0.0

        if st == 'LANDED':
            self.v_cmd[:] = 0.0
            self.v_gnd[:] = 0.0
            self.w_ff[:] = 0.0
            self.v_des[:] = 0.0
            self.acc[:] = 0.0
            self.power = 0.0
            return self.v_cmd.copy(), 0.0

        spawn = np.array(self.spec.spawn, float)
        if st == 'TAKEOFF':
            v_t[:2] = _clip_norm(1.0 * (spawn - est[:2]), 0.3)
            v_t[2] = C.TAKEOFF_SPEED
            if est[2] >= self.cruise_alt - 0.12:
                self._set_state('SCAN', t)
        elif st == 'SCAN':
            v_t[:2] = _clip_norm(1.0 * (spawn - est[:2]), 0.3)
            z_target = self.cruise_alt
            yaw_rate = C.SCAN_YAW_RATE
            if t - self.state_t0 >= C.SCAN_TIME:
                # the first route, planned on board now that the drone has looked around; it hovers
                # (still scanning) until the route is ready
                self._receive_route(t)
                if self.path is None and not self.planner.busy():
                    self._request_route(t, 'initial')
                if self.path is not None:
                    self.m['cruise_t'] = t
                    self.log(t, f'{self.name}: path planned with {PLANNER_NAMES[C.PLANNER]} ({self.path_len:.1f} m, '
                                f'reference {self.ref_len:.1f} m)')
                    self._set_state('CRUISE', t)
                    self.sprog_ref, self.sprog_ref_t = 0.0, t
                elif self.plan_fail_since is not None and t - self.plan_fail_since > 3.0:
                    self._set_state('ESCAPE', t, 'no path')
                    self.m['escapes'] += 1
        elif st == 'CRUISE':
            self._check_replan(t)
            if self.plan_fail_since is not None and t - self.plan_fail_since > 3.0:
                self.m['escapes'] += 1
                self._set_state('ESCAPE', t, 'no feasible path')
            else:
                d, s_near, _ = closest_on_polyline(est, self.path, self.path_cum)
                if self.s_prog - 0.5 <= s_near <= self.s_prog + 2.0:
                    self.s_prog = max(self.s_prog, s_near)
                carrot = point_at_s(self.path, self.s_prog + C.LOOKAHEAD, self.path_cum)
                V = self._path_speed()
                z_target = self.cruise_alt
                to_c = carrot - est[:2]
                nc = float(np.linalg.norm(to_c))
                if nc > 1e-6:
                    v_t[:2] = V * to_c / max(nc, C.LOOKAHEAD * 0.5)
                v_t[:2] = self._give_way(t, est, v_t[:2], peers)
                if self.gw_alt is not None:                  # giving way by climbing / descending
                    z_target = self.gw_alt
                v_t[:2] += self._peer_avoidance(est, peers)
                v_t[:2] = self._safety_filter(t, est, _clip_norm(v_t[:2], C.V_MAX), peers)
                # progress / stuck watchdog: no progress along the path, or the safety layer has
                # kept the drone (almost) still for a long time. Paused while the drone gives way:
                # waiting for another drone is not being stuck
                slow = float(np.linalg.norm(v_t[:2])) < 0.25 and not self.yielding
                self.slow_time = self.slow_time + dt if slow else 0.0
                if self.yielding:
                    self.sprog_ref_t = t
                if self.s_prog > self.sprog_ref + 0.3 and self.slow_time < 8.0:
                    self.sprog_ref, self.sprog_ref_t = self.s_prog, t
                    self.stuck_count = 0
                elif t - self.sprog_ref_t > 8.0 or self.slow_time >= 8.0:
                    self.slow_time = 0.0
                    self.stuck_count += 1
                    self.m['stuck'] += 1
                    self.sprog_ref_t = t
                    if self.stuck_count >= 2:
                        self.m['escapes'] += 1
                        self._set_state('ESCAPE', t, 'no progress')
                    else:
                        self._request_route(t, 'no progress')
                # arrival
                dg = float(np.linalg.norm(self.goal - est[:2]))
                if self.s_prog >= self.path_len - 0.35 and dg < 0.35:
                    self._set_state('LANDING', t)
        elif st == 'ESCAPE':
            z_target = C.ESCAPE_ALT
            if est[2] > C.ESCAPE_ALT - 0.3:
                to_g = self.goal - est[:2]
                dg = float(np.linalg.norm(to_g))
                V = min(self.cruise, max(0.25, math.sqrt(2 * 0.5 * C.ACC_MAX_H * dg)))
                if dg > 1e-6:
                    v_t[:2] = V * to_g / dg
                if dg < 0.3:
                    self._set_state('LANDING', t)
            v_t[:2] = self._give_way(t, est, v_t[:2], peers)
            v_t[:2] += self._peer_avoidance(est, peers)
            v_t[:2] = self._safety_filter(t, est, _clip_norm(v_t[:2], C.V_MAX), peers)
        elif st == 'LANDING':
            to_pad = self.goal - est[:2]
            v_t[:2] = _clip_norm(1.2 * to_pad, 0.4)
            # pushed off the pad on the way down (another drone passing close): stop descending and
            # re-centre first, unless already near the ground or after LAND_HOLD_MAX s of holding
            hold = (float(np.linalg.norm(to_pad)) > LAND_RECENTER and est[2] > 0.4
                    and t - self.state_t0 < LAND_HOLD_MAX)
            v_t[2] = 0.0 if hold else (-C.LAND_SPEED if est[2] > 0.55 else -0.25)
            if est[2] <= C.REST_Z + 0.06:
                self._set_state('LANDED', t)
                self.m['land_t'] = t
                tp = self.truth['p']                       # scoring only
                self.m['landing_err'] = float(np.linalg.norm(tp[:2] - self.goal))
                self.m['loc_final'] = float(np.linalg.norm(tp[:2] - est[:2]))
                self.log(t, f'{self.name}: LANDED, landing error {self.m["landing_err"]:.2f} m')
                self.v_cmd[:] = 0.0
                self.v_gnd[:] = 0.0
                self.w_ff[:] = 0.0
                self.v_des[:] = 0.0
                return self.v_cmd.copy(), 0.0

        # timeout fail-safe
        if self.m['takeoff_t'] is not None and t - self.m['takeoff_t'] > C.MISSION_TIMEOUT \
                and self.state not in ('LANDING', 'LANDED'):
            self.flags['failed'] = True
            self.goal = est[:2].copy()
            self._set_state('LANDING', t, 'timeout')

        # altitude hold
        if z_target is not None:
            ez = z_target - est[2]
            self.z_int = float(np.clip(self.z_int + ez * dt, -0.5, 0.5))
            v_t[2] = float(np.clip(C.KP_Z * ez + C.KI_Z * self.z_int, -0.9, 1.0))

        # keep-in boundary
        lim = C.ARENA_HALF - C.BOUNDARY_KEEP_IN * 0.6
        for k in range(2):
            if est[k] > lim:
                v_t[k] = min(v_t[k], -0.5 * (est[k] - lim))
            elif est[k] < -lim:
                v_t[k] = max(v_t[k], 0.5 * (-lim - est[k]))
        v_t[:2] = _clip_norm(v_t[:2], C.V_MAX)
        # don't fly fast where the forward depth camera cannot see: turn first
        sp_h = float(np.linalg.norm(v_t[:2]))
        if st in ('CRUISE', 'ESCAPE') and sp_h > 0.3:
            half = math.radians(C.SENSORS.depth_fov_deg) / 2.0
            off = abs(wrap_angle(math.atan2(v_t[1], v_t[0]) - yaw))
            if off > half:
                v_t[:2] *= max(0.15, 1.0 - (off - half) / half)
        self.v_des = v_t.copy()

        # guidance: the GROUND-velocity setpoint changes smoothly (acceleration limits) ...
        dg = v_t - self.v_gnd
        dg[:2] = _clip_norm(dg[:2], C.ACC_MAX_H * dt)
        dg[2] = float(np.clip(dg[2], -C.ACC_MAX_V * dt, C.ACC_MAX_V * dt))
        self.v_gnd = self.v_gnd + dg
        # ... and the wind feed-forward from the EKF (fly v_air = v_ground - wind) follows the wind
        # estimate with the airframe's full tilt authority, as the attitude loop of a real drone does
        w_t = _clip_norm(self.ekf.wind, self.wind_comp_max) if self.in_air() else np.zeros(2)
        self.w_ff = self.w_ff + _clip_norm(w_t - self.w_ff, C.ACC_MAX_WIND * dt)
        v_new = self.v_gnd.copy()
        v_new[:2] -= self.w_ff
        dv = v_new - self.v_cmd
        if st in ('CRUISE', 'ESCAPE', 'LANDING') and self.in_air():
            # emergency brake: the acceleration limit must never carry the drone into another
            # drone. Re-check the velocity it will really fly (command + wind) against close peers.
            w_hat = _clip_norm(self.ekf.wind, self.wind_comp_max)
            v_g = v_new[:2] + w_hat
            v_safe = v_g.copy()
            for pr in peers:
                rel = est[:2] - pr['pos'][:2]
                d = float(np.linalg.norm(rel))
                # hard floor: the rotor / downwash footprint without the separation margin
                gap = d - (DW.required_separation(est[2] - pr['pos'][2]) - C.SEP_MARGIN) - 0.1
                if gap < 0.4 and d > 1e-3:
                    v_safe = self._barrier(v_safe, rel / d, gap, C.ACC_MAX_H, np.asarray(pr['vel'], float)[:2],
                                           share=0.5)
            if not np.allclose(v_safe, v_g):
                v_safe = self._safety_filter(t, est, v_safe, [], count=False)     # never into an obstacle
                v_new[:2] = v_safe - w_hat
                self.v_gnd[:2] = v_new[:2] + self.w_ff
                dv = v_new - self.v_cmd
                self.m['hard_brakes'] += 1
        self.acc = dv / dt if dt > 0 else np.zeros(3)
        self.v_cmd = v_new

        # heading (EKF yaw): face the direction of travel (narrow camera: look towards landmarks)
        yaw_max = C.SCAN_YAW_RATE if st == 'SCAN' else C.YAW_RATE_MAX
        if st != 'SCAN':
            if sp_h > 0.3:
                travel = math.atan2(v_t[1], v_t[0])
                if self.nav.cam_points and st == 'CRUISE' and C.SENSORS.cam_fov_deg < 300:
                    travel += self._look_offset(t, est, travel)
                yaw_rate = 2.5 * wrap_angle(travel - yaw)
        self.yaw_rate = float(np.clip(yaw_rate, -yaw_max, yaw_max))
        return self.v_cmd.copy(), self.yaw_rate

    # ----------------------------------------------------------------- energy
    def integrate_energy(self, dt):
        """Rotor power from the AIR velocity command (headwinds cost energy)."""
        if not self.airborne or dt <= 0:
            self.power = 0.0
            return
        p, parts = self.pm.power(self.v_cmd, self.acc)
        self.power = p
        self.battery.draw(p, dt)
        for k, v in parts.items():
            self.m['energy_parts'][k] += v * dt
        self.m['peak_power'] = max(self.m['peak_power'], p)
        if self.yielding:
            self.m['yield_time'] += dt
            if self.gw_kind:
                self.m['gw_time'][self.gw_kind] += dt
        if self.avoiding:
            self.m['avoid_time'] += dt

    # ---------------------------------------------------------------- metrics
    def record_truth_metrics(self, t, dt, prev_p):
        """Scoring against ground truth (never fed back into the drone)."""
        p = self.truth['p']
        if self.airborne and prev_p is not None:
            self.m['dist'] += float(np.linalg.norm(p - prev_p))
            if self.state in ('CRUISE', 'ESCAPE'):
                self.m['dist_h'] += float(np.linalg.norm(p[:2] - prev_p[:2]))
            self.m['max_speed'] = max(self.m['max_speed'], float(np.linalg.norm(self.truth['v'])))
        if self.airborne:
            le = float(np.linalg.norm(self.ekf.pos[:2] - p[:2]))
            self.last_loc_err = le
            self.m['sigma_sum'] += self.ekf.sigma_xy()
            e = self.ekf.pos[:2] - p[:2]
            try:
                self.m['nees_sum'] += float(e @ np.linalg.solve(self.ekf.cov_xy, e))
            except np.linalg.LinAlgError:
                pass
            if self.nav.cam_points:
                lf = self.m['last_fix_t'] if self.m['last_fix_t'] is not None else self.m['takeoff_t']
                gap = t - lf
                self.m['max_no_fix'] = max(self.m['max_no_fix'], gap)
                if gap > 1.0:
                    self.m['time_no_fix'] += dt
            self.m['loc_sq'] += le * le
            self.m['loc_n'] += 1
            self.m['loc_max'] = max(self.m['loc_max'], le)
        if self.state in ('CRUISE', 'ESCAPE') and self.ref is not None:
            cte, _, _ = closest_on_polyline(p, self.ref, self.ref_cum)
            self.last_cte = cte
            self.m['cte_sq'] += cte * cte
            self.m['cte_sum'] += cte
            self.m['cte_n'] += 1
            self.m['cte_on'] += int(cte <= 0.5)
            self.m['cte_max'] = max(self.m['cte_max'], cte)
            # tracking error: distance from the drone's OWN current plan (control + localisation
            # quality, independent of which way round an obstacle the plan goes)
            if self.state == 'CRUISE' and self.path is not None and len(self.path) > 1:
                trk, _, _ = closest_on_polyline(p, self.path, self.path_cum)
                self.m['trk_sq'] += trk * trk
                self.m['trk_n'] += 1
                self.m['trk_max'] = max(self.m['trk_max'], trk)
        # physical contact with any obstacle (ground truth, same shapes as the Gazebo collision model)
        if self.airborne and p[2] > 0.3:
            gap, lim = self.world.contact_gap(p)
            touching = bool(np.any((lim > 0) & (gap < 0.01)))
            if touching and not self.m['in_contact']:
                self.m['contacts'] += 1
                self.log(t, f'{self.name}: !! CONTACT with obstacle at ({p[0]:.1f},{p[1]:.1f})')
            self.m['in_contact'] = touching

    def progress(self):
        """Progress along this drone's reference route (0..100 %)."""
        if self.state == 'LANDED':
            return 100.0
        if self.truth is None or self.ref is None or self.state in ('PREVIEW', 'TAKEOFF', 'SCAN'):
            return 0.0
        _, s, _ = closest_on_polyline(self.truth['p'], self.ref, self.ref_cum)
        return 100.0 * s / max(self.ref_len, 1e-6)

    def summary(self):
        m = self.m
        rms = lambda sq, n: math.sqrt(sq / n) if n else 0.0
        flight_time = (m['land_t'] - m['takeoff_t']) if (m['land_t'] is not None and m['takeoff_t'] is not None) else None
        return {
            'id': self.id, 'name': self.name, 'color': self.spec.color,
            'nav': self.nav.key, 'nav_label': self.nav.label,
            'state': self.state, 'failed': self.flags['failed'],
            'goal_reached': bool(m['landing_err'] is not None and m['landing_err'] <= 0.5 and not self.flags['failed']),
            'flight_time_s': flight_time,
            'energy_kj': self.battery.used_j / 1000.0,
            'energy_wh': self.battery.used_j / 3600.0,
            'battery_soc': self.battery.soc,
            'avg_power_w': (self.battery.used_j / flight_time) if flight_time else None,
            'peak_power_w': m['peak_power'],
            'distance_m': m['dist'],
            'horizontal_distance_m': m['dist_h'],
            'ref_length_m': self.ref_len,
            'length_ratio': (m['dist_h'] / self.ref_len) if self.ref_len > 0 else None,
            # None (not 0) for a drone that never got en route, so it does not flatter the fleet's mean
            'cte_rms_m': rms(m['cte_sq'], m['cte_n']) if m['cte_n'] else None,
            'track_rms_m': rms(m['trk_sq'], m['trk_n']) if m['trk_n'] else None,
            'track_max_m': m['trk_max'] if m['trk_n'] else None,
            'cte_mean_m': (m['cte_sum'] / m['cte_n']) if m['cte_n'] else None,
            'cte_max_m': m['cte_max'] if m['cte_n'] else None,
            'on_path_pct': (100.0 * m['cte_on'] / m['cte_n']) if m['cte_n'] else None,
            'loc_rms_m': rms(m['loc_sq'], m['loc_n']) if m['loc_n'] else None,
            'loc_max_m': m['loc_max'] if m['loc_n'] else None,
            'landing_err_m': m['landing_err'],
            'replans': m['replans'], 'stuck_events': m['stuck'], 'escapes': m['escapes'],
            'yield_time_s': m['yield_time'], 'avoid_time_s': m['avoid_time'],
            'give_way_s': {k: round(v, 2) for k, v in m['gw_time'].items()},
            'contacts': m['contacts'],
            'max_speed_ms': m['max_speed'],
            'plans': len(self.plan_ms),                  # in flight (the reference plan: ref_plan_ms)
            'ref_plan_ms': self.ref_ms,
            'plan_time_ms_mean': (sum(self.plan_ms) / len(self.plan_ms)) if self.plan_ms else None,
            'plan_time_ms_max': max(self.plan_ms) if self.plan_ms else None,
            'ttc_trades': self.trader.trades if self.trader is not None else 0.0,
            'energy_parts_kj': {k: v / 1000.0 for k, v in m['energy_parts'].items()},
            'nav_power_w': self.pm.p_nav,
            'energy_per_m_j': (self.battery.used_j / m['dist_h']) if m['dist_h'] > 0 else None,
            'loc_final_m': m['loc_final'],
            'sigma_mean_m': (m['sigma_sum'] / m['loc_n']) if m['loc_n'] else None,
            'consistency': (m['nees_sum'] / m['loc_n'] / 2.0) if m['loc_n'] else None,     # NEES / dof
            'cam_frames': m['cam_frames'],
            'cam_fix_pct': (100.0 * m['cam_fix_frames'] / m['cam_frames']) if m['cam_frames'] else None,
            'cam_markers_visible_mean': (m['cam_visible_sum'] / m['cam_frames']) if m['cam_frames'] else None,
            'cam_markers_used': m['cam_markers'],
            'longest_no_fix_s': m['max_no_fix'] if self.nav.cam_points else None,
            'peer_updates': m['peer_updates'],
            'meas_updates': self.ekf.n_updates, 'meas_rejected': m['rejected'],
            'safety_interventions': m['safety_hits'], 'depth_brakes': m['depth_brakes'],
            'hard_brakes': m['hard_brakes'],
            'unknown_known': known_unknowns(self.map),   # hidden pines in its map (scoring only)
        }
