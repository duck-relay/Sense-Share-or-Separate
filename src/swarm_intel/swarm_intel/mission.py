"""A mission: five drones, each with its own on-board computer, in a world that simulates the physics.

DroneRuntime is ONE drone's on-board computer (an edge node): it simulates the drone's own sensors,
fuses every measurement in its EKF (estimator.py), keeps its own map, plans on board - the reference
route with A* on the pad before take-off, the flown route with RRT* or A* in flight - takes part in
the swarm's turn trading (ttc.py), and computes its command. It talks to the other drones only by
radio messages (DroneRuntime.message). In ROS 2 every drone runs it in its own node (drone_node.py).

WorldSim is everything that is not on a drone: wind, rotor downwash and the drone's real heading act
on the commands (Gazebo's VelocityControl ignores forces, so they are added here, identically for
Gazebo and the headless plant), and the scoring that needs every drone at once (separation, drone
contacts, time in another drone's downwash). In ROS 2 it runs in the world node (world_node.py).

MissionCore is the headless twin: five DroneRuntimes and a WorldSim in one process, with an
in-process radio, stepped together. The ROS 2 nodes run exactly the same per-drone and world code.

Navigation components (config.NAV_CONFIGS; the whole fleet, or a mixed fleet, config.FLEETS):
  camera N-point  ArUco fixes on the known trees (baseline = 3-point + VIO)
  VIO             on-board IMU + visual-inertial odometry (measured velocity)
  swarm           * a shared occupancy map: every drone keeps its own copy and sends its depth scans
                    over the radio, the others add them to theirs
                  * cooperative localisation: UWB peer ranges + broadcast estimates
                  * give-way by turn trading (ttc.py): where routes would come closer than the
                    separation standard, the drones trade passage turns (along trading cycles) into the
                    order of lowest energy for the whole fleet; no drone has priority. The fixed-priority
                    give-way (fixed priorities in drone order) is kept for comparison (config.GIVE_WAY = 'priority')
Without swarm a drone has only its own map and notices other drones only with its depth camera /
ToF ring within 3 m (Kalman-tracked, reactive APF + curl avoidance). Every drone keeps the
downwash-aware separation standard (downwash.required_separation).
"""
import math

import numpy as np

from . import config as C
from . import downwash as DW
from . import layouts
from .geometry import WorldObstacles
from .grid_planner import GridMap, PLANNER_NAMES, known_unknowns
from .agent import DroneAgent, GIVE_WAY_KINDS
from .ttc import TurnTrader, GIVE_WAY_NAMES

GROUND_Z = 0.02     # the drawn reference route starts and ends on the ground, at the pads


def _r(p):
    return [round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)]


def known_map():
    """What every drone knows before take-off: the landmark trees (the map of the markers)."""
    gm = GridMap()
    for (x, y) in C.LANDMARK_TREES:
        gm.seed_disk(x, y, C.TREE_CROWN_R)
    return gm


def mission_streams(seed, layout):
    """Seeds of a mission's random streams, the same in every process: one per drone (its sensors), the
    wind, the wake turbulence. They depend on the seed AND the forest, so the missions of a study are
    independent, and all configurations of one mission share them (paired comparisons)."""
    master = np.random.default_rng([int(seed), int(layout)])
    drones = {d.id: int(master.integers(1 << 31)) for d in C.DRONES}
    wind = int(master.integers(1 << 31))
    wake = int(master.integers(1 << 31))
    return drones, wind, wake


def fleet_navs(nav):
    """nav: a configuration key (the whole fleet uses it) or a mixed-fleet key (config.FLEETS).
    Returns ({drone id: NavConfig}, key, label, fleet or None)."""
    fleet = C.FLEETS.get(str(nav))
    if fleet:
        return ({d.id: C.nav_config(k) for d, k in zip(C.DRONES, fleet.navs)}, fleet.key, fleet.label, fleet)
    one = C.nav_config(nav)
    return {d.id: one for d in C.DRONES}, one.key, one.label, None


def _as_array(v):
    return None if v is None else np.asarray(v, float)


def decode_message(msg):
    """A radio message as received over ROS (JSON: lists) -> the form the drones use (numpy arrays)."""
    if msg is None:
        return None
    out = dict(msg)
    for k in ('pos', 'vel', 'cov', 'intent'):
        out[k] = _as_array(out.get(k))
    out['scans'] = [(int(s[0]), np.asarray(s[1], float), np.asarray(s[2], float), np.asarray(s[3], bool))
                    for s in out.get('scans') or ()]
    ttc = out.get('ttc')
    if ttc and ttc.get('snap'):
        ttc = dict(ttc)
        ttc['snap'] = {'route': np.asarray(ttc['snap']['route'], float), 'z': float(ttc['snap']['z'])}
        out['ttc'] = ttc
    if ttc and ttc.get('cluster') is not None:
        out['ttc'] = dict(out['ttc'], cluster=tuple(ttc['cluster']))
    if ttc and ttc.get('order') is not None:
        out['ttc'] = dict(out['ttc'], order=tuple(ttc['order']))
    return out


# ---------------------------------------------------------------------------
# one drone's on-board computer
# ---------------------------------------------------------------------------
class DroneRuntime:
    """One drone's on-board computer (see the module docstring). truth = the true state of every drone
    {id: {'p', 'v', 'yaw', 'roll', 'pitch'}}: it only feeds this drone's simulated sensors (its own
    pose, other drones in reach of its proximity sensors and UWB) and its scoring, never its decisions.
    inbox = the latest radio messages of the other drones {id: message}."""

    def __init__(self, spec, nav, seed, layout, speed_scale=1.0, log=None):
        self.spec = spec
        self.id = spec.id
        drone_seeds, _, _ = mission_streams(seed, layout)
        self.agent = DroneAgent(spec, nav, WorldObstacles(), known_map(), np.random.default_rng(drone_seeds[spec.id]),
                                speed_scale=speed_scale, shared_map=nav.swarm, log=log,
                                plan_rng=np.random.default_rng([int(seed), int(layout), spec.id, 0x525254]))
        if nav.swarm and C.GIVE_WAY == 'ttc':
            self.agent.trader = TurnTrader(spec.id, log)
        self.scan_seen = {}            # shared map: last scan number added from each other drone
        self.prev_p = None

    def receive_scans(self, inbox):
        """Shared map: add the other drones' new depth scans to this drone's copy (in drone order)."""
        a = self.agent
        if not a.shared_map:
            return
        for j in sorted(inbox):
            msg = inbox[j]
            if not msg:
                continue
            for seq, origin, ends, hits in msg.get('scans') or ():
                if seq > self.scan_seen.get(j, 0):
                    a.map.integrate_scan(origin, ends, hits)
                    self.scan_seen[j] = seq

    def sense(self, t, dt, truth, inbox):
        self.t_sense = t                   # the send time of this step's radio message
        a = self.agent
        self.receive_scans(inbox)
        peer_truth = {i: q['p'] for i, q in truth.items()}
        a.sense(t, dt, truth[self.id], peer_truth, inbox if a.swarm else {})

    def message(self):
        """This drone's radio message (drones with the swarm radio only): its EKF estimate, intent,
        latest depth scans (shared map) and its part of the turn trading."""
        a = self.agent
        if not a.swarm:
            return None
        msg = {'id': self.id, 'radio': True, 'pos': a.ekf.pos, 'vel': a.ekf.vel, 'cov': a.ekf.cov_pos,
               'sigma': a.ekf.sigma_xy(), 'airborne': a.airborne, 'state': a.state, 'yielding': a.yielding,
               'boxed_in': a.boxed_in, 'intent': a.intent(), 'scans': list(a.scans_out),
               'ts': getattr(self, 't_sense', None)}
        if C.GIVE_WAY == 'priority':                 # only the fixed-priority comparison sends a rank
            msg['priority'] = a.spec.priority
        if a.trader is not None:
            msg['ttc'] = a.trader.message()
        return msg

    def control(self, t, dt, truth, inbox):
        """Decisions and command. Returns {'v_body': air velocity in the drone's own frame (from its EKF
        heading), 'yaw_rate', 'airborne', 'state'}."""
        a = self.agent
        i = self.id
        peers = {j: dict(m) for j, m in inbox.items() if m is not None and m['airborne']} if a.swarm else {}
        if C.TTC_ADAPTIVE:
            # a late message: its position is moved on by its age; a message older than RADIO_MAX_AGE is not
            # used (an ideal link delivers every message in the step it is sent: nothing changes)
            for j in list(peers):
                age = t - (peers[j].get('ts') if peers[j].get('ts') is not None else t)
                if age > C.RADIO_MAX_AGE:
                    del peers[j]
                elif age > 1e-9:
                    peers[j]['pos'] = np.asarray(peers[j]['pos'], float) + age * np.asarray(peers[j]['vel'], float)
        if a.trader is not None:
            a.trader.step(t, a, inbox)
        if a.airborne:
            # proximity sensing (every configuration): other drones physically flying close by. Its
            # relative measurement is better than a broadcast estimate, so it wins for position/velocity
            visible = {k: q['p'] for k, q in truth.items() if k != i and q['p'][2] > 0.3}
            dets = a.sensors.proximity(truth[i]['p'], truth[i]['yaw'], visible)
            for trk in a.track_peers(t, dets):
                if trk['id'] in peers:
                    peers[trk['id']].update(pos=trk['pos'], vel=trk['vel'])
                else:
                    peers[trk['id']] = trk
        v_cmd, yaw_rate = a.control(t, dt if dt > 0 else 1.0 / C.CONTROL_RATE, list(peers.values()))
        a.integrate_energy(dt)
        a.record_truth_metrics(t, dt, self.prev_p)
        self.prev_p = truth[i]['p'].copy()
        c, s = math.cos(a.ekf.yaw), math.sin(a.ekf.yaw)
        v = np.asarray(v_cmd, float)
        v_body = np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]])
        return {'v_body': v_body, 'yaw_rate': float(yaw_rate), 'airborne': bool(a.airborne), 'state': a.state}

    def plans(self):
        """This drone's routes, for the world node: its reference route (A*, before take-off), also as the
        3-D line Gazebo draws in green - from its pad up to its cruise altitude, along the route and down
        to its landing pad - and the route it now follows (RRT* or A*, re-planned in flight), with a
        version that changes with it. The trail of the route actually flown (in the drone's colour) is drawn by the world
        node from Gazebo's positions (world_node.draw_trails)."""
        a = self.agent
        z = a.cruise_alt
        (sx, sy), (gx, gy) = a.spec.spawn, a.goal
        ref3d = None
        if a.ref is not None:
            ref3d = ([_r((sx, sy, GROUND_Z)), _r((sx, sy, z))] + [_r((x, y, z)) for x, y in a.ref]
                     + [_r((gx, gy, z)), _r((gx, gy, GROUND_Z))])
        return {'id': self.id, 'version': a.plan_version, 'ref': None if a.ref is None else a.ref.tolist(),
                'plan': None if a.path is None else np.asarray(a.path, float).tolist(), 'z': z, 'ref3d': ref3d}

    def status(self, truth_i=None):
        """Telemetry of this drone (dashboard, logs) and its summary (report)."""
        a = self.agent
        tr = truth_i if truth_i is not None else a.truth
        out = {'id': self.id, 'name': a.name, 'color': a.spec.color, 'state': a.state,
               'yielding': a.yielding, 'avoiding': a.avoiding, 'airborne': a.airborne,
               'est': [round(float(v), 3) for v in a.ekf.pos],
               'power_w': round(a.power, 1), 'energy_kj': round(a.battery.used_j / 1000.0, 3),
               'energy_j': round(a.battery.used_j, 2), 'soc': round(a.battery.soc, 2),
               'loc_err': round(a.last_loc_err, 3), 'sigma': round(a.ekf.sigma_xy(), 3), 'markers': a.last_markers,
               'cte': round(a.last_cte, 3), 'goal': list(a.spec.goal), 'progress': round(a.progress(), 1),
               'replans': a.m['replans'], 'contacts': a.m['contacts'], 'unknown_known': known_unknowns(a.map)}
        if tr is not None:
            out.update({'pos': [round(float(v), 3) for v in tr['p']],
                        'speed': round(float(np.linalg.norm(tr['v'])), 3),
                        'hspeed': round(float(np.linalg.norm(tr['v'][:2])), 3),
                        'vz': round(float(tr['v'][2]), 3), 'height': round(float(tr['p'][2]), 3),
                        'yaw_deg': round(math.degrees(tr['yaw']), 1),
                        'goal_dist': round(float(np.linalg.norm(tr['p'][:2] - a.goal)), 2)})
        return out


# ---------------------------------------------------------------------------
# the world: physics and the scoring that needs every drone
# ---------------------------------------------------------------------------
class WindField:
    """PHYSICS: steady wind (random direction per seed) + per-drone gusts (Ornstein-Uhlenbeck)."""

    def __init__(self, rng, ids, wp=None):
        self.wp = wp or C.WIND
        a = rng.uniform(0.0, 2.0 * math.pi)
        self.mean = self.wp.mean_speed * np.array([math.cos(a), math.sin(a)])
        self.rng = rng
        self.gust = {i: np.zeros(2) for i in ids}

    def step(self, dt):
        if dt <= 0:
            return
        tau, sig = self.wp.gust_tau, self.wp.gust_sigma
        for i in self.gust:
            self.gust[i] += -self.gust[i] / tau * dt + sig * math.sqrt(2.0 * dt / tau) * self.rng.normal(size=2)

    def velocity(self, i):
        return self.mean + self.gust[i]


class WorldSim:
    """Everything that is not on a drone (see the module docstring)."""

    def __init__(self, seed, layout, ids, wind=True, log=None):
        _, wind_seed, wake_seed = mission_streams(seed, layout)
        self.log = log or (lambda t, msg: None)
        self.ids = list(ids)
        self.wind_on = wind
        self.wind = WindField(np.random.default_rng(wind_seed), self.ids)
        # downwash turbulence (unit Ornstein-Uhlenbeck noise per drone, scaled by the local downdraft)
        self.wake_rng = np.random.default_rng(wake_seed)
        self.wake_turb = {i: np.zeros(2) for i in self.ids}
        self.downdraft = {i: 0.0 for i in self.ids}
        self.in_wake = set()
        self.pair_contact = set()
        self.safety = {i: {'min_sep': math.inf, 'min_sep3d': math.inf, 'min_margin': math.inf, 'drone_contacts': 0,
                           'wake_time': 0.0, 'wake_max': 0.0} for i in self.ids}

    def _downwash(self, truth, airborne, dt):
        """PHYSICS: downdraft (m/s) each drone feels from the wakes of the drones above it."""
        p = C.DOWNWASH
        out = {}
        if dt > 0:
            k = math.sqrt(2.0 * dt / p.turb_tau)
            for i in self.wake_turb:
                self.wake_turb[i] += -self.wake_turb[i] / p.turb_tau * dt + k * self.wake_rng.normal(size=2)
        for i in self.ids:
            w = 0.0
            if p.on and i in truth and airborne.get(i) and truth[i]['p'][2] > C.REST_Z + 0.05:
                pi = truth[i]['p']
                for j in self.ids:
                    if j == i or j not in truth or not airborne.get(j):
                        continue
                    pj = truth[j]['p']
                    dz = float(pj[2] - pi[2])
                    if dz <= 0.0 or dz > p.max_depth:
                        continue
                    v_air = truth[j]['v'][:2] - (self.wind.velocity(j) if self.wind_on else 0.0)
                    ox, oy = DW.axis_offset(dz, v_air)
                    w += DW.speed(dz, math.hypot(pi[0] - pj[0] - ox, pi[1] - pj[1] - oy))
            out[i] = w
        return out

    def physics(self, t, dt, truth, cmds):
        """cmds: {id: DroneRuntime.control output}. Returns {id: (velocity the drone really flies in the
        world frame, yaw rate)}: its command turned by its TRUE heading (it flew it in its believed
        frame), plus the wind once it has lifted off, plus the downdraft and turbulence of the wakes."""
        self.wind.step(dt)
        airborne = {i: c['airborne'] for i, c in cmds.items()}
        self.downdraft = self._downwash(truth, airborne, dt)
        out = {}
        for i, c in cmds.items():
            vb = np.asarray(c['v_body'], float)
            yaw = truth[i]['yaw']
            cy, sy = math.cos(yaw), math.sin(yaw)
            v = np.array([cy * vb[0] - sy * vb[1], sy * vb[0] + cy * vb[1], vb[2]])
            z = truth[i]['p'][2]
            if self.wind_on and c['airborne'] and z > C.REST_Z + 0.05:
                v[:2] += self.wind.velocity(i)
            w = self.downdraft.get(i, 0.0)
            if w > 0.0:
                v[2] -= w
                v[:2] += C.DOWNWASH.turb * w * self.wake_turb[i]
            if c['airborne'] and z > 0.4:
                s = self.safety[i]
                s['wake_max'] = max(s['wake_max'], w)
                if w > C.DOWNWASH.w_tol:
                    s['wake_time'] += dt
                    if i not in self.in_wake:
                        self.log(t, f'!! drone{i} in the downwash of a drone above ({w:.1f} m/s down)')
                        self.in_wake.add(i)
                elif w < 0.5 * C.DOWNWASH.w_tol:
                    self.in_wake.discard(i)
            out[i] = (v, c['yaw_rate'])
        return out

    def score(self, t, truth, cmds):
        """Safety between flying drones (ground truth, scoring only):
           min_sep    horizontal distance while at the same altitude (|dz| < 0.3 m)
           min_sep3d  plain 3-D distance
           min_margin horizontal distance minus the downwash-aware separation standard (negative =
                      rotor discs or downwash closer than allowed)
        all only while BOTH drones are en route (CRUISE / ESCAPE): the simultaneous climb from the pads
        would otherwise set the value in every configuration; drone contacts: rotor discs touching
        (same collision size as in Gazebo)."""
        ids = [i for i in self.ids if i in truth and cmds.get(i, {}).get('airborne') and truth[i]['p'][2] > 0.4]
        for x in range(len(ids)):
            for y in range(x + 1, len(ids)):
                pa, pb = truth[ids[x]]['p'], truth[ids[y]]['p']
                d3 = float(np.linalg.norm(pa - pb))
                dh = float(np.linalg.norm(pa[:2] - pb[:2]))
                same = abs(pa[2] - pb[2]) < 0.3
                en_route = all(cmds[k]['state'] in ('CRUISE', 'ESCAPE') for k in (ids[x], ids[y]))
                margin = dh - DW.required_separation(pa[2] - pb[2])
                for k in (ids[x], ids[y]):
                    s = self.safety[k]
                    if en_route:
                        s['min_sep3d'] = min(s['min_sep3d'], d3)
                        s['min_margin'] = min(s['min_margin'], margin)
                        if same:
                            s['min_sep'] = min(s['min_sep'], dh)
                pair = (ids[x], ids[y])
                touching = dh < 2 * C.DRONE_COLLISION_R and abs(pa[2] - pb[2]) < 0.12
                if pair in self.pair_contact:
                    # the collision pushes the rotor discs exactly apart: one contact lasts until the
                    # drones are clearly apart again (otherwise a drone pressing on counts every tick)
                    touching = dh < 2 * C.DRONE_COLLISION_R + 0.05 and abs(pa[2] - pb[2]) < 0.2
                if touching and pair not in self.pair_contact:
                    for k in pair:
                        self.safety[k]['drone_contacts'] += 1
                    self.log(t, f'!! drone{pair[0]} and drone{pair[1]} touched (rotor discs overlap)')
                if touching:
                    self.pair_contact.add(pair)
                else:
                    self.pair_contact.discard(pair)

    def drone_safety(self, i):
        """The world-scored part of drone i's summary."""
        s = self.safety[i]
        fin = lambda v: v if math.isfinite(v) else None
        return {'min_separation_m': fin(s['min_sep']), 'min_separation_3d_m': fin(s['min_sep3d']),
                'min_sep_margin_m': fin(s['min_margin']), 'wake_time_s': s['wake_time'],
                'wake_max_ms': s['wake_max'], 'drone_contacts': s['drone_contacts']}


def build_report(meta, summaries, sim, events):
    """The mission report from every drone's summary (on board) and the world's scoring: every
    metric covers the whole fleet (metrics.fleet_metrics), and 'fleet' lists every drone.
    meta: model_version, nav, nav_label, nav_parts, layout, seed, speed_scale, mission_time_s,
    all_landed_at_s, wind_mean_ms, reference_paths."""
    from .metrics import fleet_metrics
    fleet = []
    for i in sorted(summaries):
        f = dict(summaries[i])
        f.update(sim.drone_safety(i))
        fleet.append(f)
    rep = dict(meta)
    rep.update({'planner': C.PLANNER, 'give_way': C.GIVE_WAY, 'fleet': fleet})
    rep.update(fleet_metrics(fleet, nav_of=lambda f: C.NAV[f['nav']],
                             mission_time=meta.get('all_landed_at_s') or meta.get('mission_time_s'),
                             give_way_kinds=GIVE_WAY_KINDS))
    rep.update({'unknown_obstacles_total': len(C.UNKNOWN_PINES), 'events': events})
    return rep


# ---------------------------------------------------------------------------
# the headless twin
# ---------------------------------------------------------------------------
class Radio:
    """In-process radio of the headless twin. inbox(me): the latest message this drone has received from
    every other drone. With the default (ideal) link every message reaches every drone at once (the drones
    read it in the next tick's sensing and in this tick's decisions). config.RADIO makes the link realistic
    for the control experiments: each message is lost for each receiver with probability RADIO.loss, arrives
    RADIO.delay_ticks control steps later (drawn uniformly), and only reaches drones within RADIO.range_m.
    A drone keeps the newest message it received from each sender, so a lost message leaves an older one in
    its inbox. The link draws from its own random stream, so it never shifts the sensor noise."""

    def __init__(self, seed=0, layout=0, params=None):
        self.p = params or C.RADIO
        self.ideal = self.p.loss <= 0.0 and max(self.p.delay_ticks) <= 0 and self.p.range_m <= 0.0
        self.rng = np.random.default_rng([int(seed), int(layout), 0x52414449])
        self.tick = 0
        self.recv = {}          # receiver -> {sender: (send tick, message)}
        self.queue = []         # (due tick, send tick, receiver, sender, message)
        self.msgs = {}          # ideal link: the latest message of every drone
        self.sent = 0
        self.lost = 0

    def publish(self, i, msg, receivers=(), positions=None):
        if msg is None:
            return
        if self.ideal:
            self.msgs[i] = msg
            return
        for r in receivers:
            if r == i:
                continue
            self.sent += 1
            if self.p.range_m > 0.0 and positions is not None and float(
                    np.linalg.norm(np.asarray(positions[i]) - np.asarray(positions[r]))) > self.p.range_m:
                self.lost += 1
                continue
            if self.p.loss > 0.0 and self.rng.random() < self.p.loss:
                self.lost += 1
                continue
            lo, hi = self.p.delay_ticks
            d = int(self.rng.integers(lo, hi + 1)) if hi > lo else int(lo)
            self.queue.append((self.tick + d, self.tick, r, i, msg))

    def deliver(self):
        """End of a tick's sending: hand over every message that is due, keeping the newest per sender."""
        if self.ideal:
            return
        keep = []
        for due, sent, r, i, msg in self.queue:
            if due <= self.tick:
                box = self.recv.setdefault(r, {})
                if i not in box or box[i][0] <= sent:
                    box[i] = (sent, msg)
            else:
                keep.append((due, sent, r, i, msg))
        self.queue = keep
        self.tick += 1

    def inbox(self, me):
        if self.ideal:
            return {j: m for j, m in self.msgs.items() if j != me}
        return {j: m for j, (_, m) in self.recv.get(me, {}).items()}


class MissionCore:
    """The headless twin: five DroneRuntimes, a WorldSim and an in-process radio, stepped together.
    Every tick each drone senses (with the radio messages of the last tick) and sends its message,
    then each decides with the messages of this tick, then the world applies the physics."""

    def __init__(self, nav=C.BASELINE, seed=0, speed_scale=1.0, wind=True):
        navs, self.key, self.label, self.fleet = fleet_navs(nav)
        self.navs = navs
        self.nav = navs[C.DRONES[0].id]              # the configuration (of every drone; mixed fleets: see navs)
        self.seed = seed
        self.speed_scale = speed_scale
        self.layout = layouts.current()
        self.world = WorldObstacles()                # the true obstacles (the plant's collisions)
        self.events = []
        self.drones = {d.id: DroneRuntime(d, navs[d.id], seed, self.layout, speed_scale, log=self._log)
                       for d in C.DRONES}
        self.agents = {i: rt.agent for i, rt in self.drones.items()}
        self.sim = WorldSim(seed, self.layout, list(self.drones), wind=wind, log=self._log)
        self.radio = Radio(seed, self.layout)
        self.t = None
        self.t0 = None
        self.all_landed_t = None
        if self.fleet:
            self._log(0.0, f'Mixed fleet: {self.label}  ({", ".join(self.fleet.navs)})  seed={seed}')
        else:
            self._log(0.0, f'Navigation (all 5 drones): {self.nav.label}  [{self.nav.parts}]  seed={seed}')
        self._log(0.0, f'Path planner: {PLANNER_NAMES[C.PLANNER]} (reference route: A*, on board before '
                       f'take-off); swarm give-way: {GIVE_WAY_NAMES[C.GIVE_WAY]}')
        if wind:
            w = self.sim.wind.mean
            self._log(0.0, f'Wind {np.linalg.norm(w):.2f} m/s towards {math.degrees(math.atan2(w[1], w[0])):.0f} deg + gusts')

    def _log(self, t, msg):
        self.events.append((round(float(t), 2), msg))

    @property
    def downdraft(self):
        return self.sim.downdraft

    def step(self, t, truth):
        """truth: {id: {'p': (3,), 'v': (3,), 'yaw', 'roll', 'pitch'}}  ->  {id: (v_world(3), yaw_rate)}"""
        if self.t is None:
            self.t = t
            self.t0 = t
        dt = float(np.clip(t - self.t, 0.0, 1.0))      # the EKF sub-steps long gaps
        self.t = t
        tm = t - self.t0                      # mission clock (starts at 0)
        live = [i for i in self.drones if i in truth]
        for i in live:                        # 1) sensing (radio messages of the last tick)
            self.drones[i].sense(tm, dt, truth, self.radio.inbox(i))
        pos = {i: truth[i]['p'] for i in live}
        for i in live:                        # 2) every drone sends its message
            self.radio.publish(i, self.drones[i].message(), receivers=live, positions=pos)
        self.radio.deliver()
        cmds = {i: self.drones[i].control(tm, dt, truth, self.radio.inbox(i)) for i in live}   # 3) decisions
        out = self.sim.physics(tm, dt, truth, cmds)                                         # 4) physics
        self.sim.score(tm, truth, cmds)
        if self.all_landed_t is None and all(a.state == 'LANDED' for a in self.agents.values()):
            self.all_landed_t = tm
            self._log(tm, 'All drones landed.')
        return out

    @property
    def mission_time(self):
        return 0.0 if self.t is None else self.t - self.t0

    def telemetry(self):
        drones = []
        for i, rt in self.drones.items():
            if rt.agent.truth is None:
                continue
            d = rt.status()
            d['downdraft'] = round(float(self.sim.downdraft.get(i, 0.0)), 2)
            drones.append(d)
        return {'nav': self.key, 'nav_label': self.label, 'nav_parts': self.nav.parts,
                't': round(self.mission_time, 2), 'drones': drones,
                'unknown_total': len(C.UNKNOWN_PINES),
                'all_landed': self.all_landed_t is not None,
                'events': self.events[-8:]}

    def report(self):
        meta = {'model_version': C.MODEL_VERSION, 'nav': self.key, 'nav_label': self.label, 'nav_parts': self.nav.parts,
                'fleet_navs': [self.navs[d.id].key for d in C.DRONES], 'layout': self.layout,
                'wind_mean_ms': float(np.linalg.norm(self.sim.wind.mean)) if self.sim.wind_on else 0.0,
                'seed': self.seed, 'speed_scale': self.speed_scale, 'mission_time_s': self.mission_time,
                'all_landed_at_s': self.all_landed_t,
                'reference_paths': {str(i): (None if a.ref is None else a.ref.tolist()) for i, a in self.agents.items()}}
        summaries = {i: a.summary() for i, a in self.agents.items()}
        return build_report(meta, summaries, self.sim, self.events)
