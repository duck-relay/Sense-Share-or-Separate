#!/usr/bin/env python3
"""ROS 2 world node: everything that is not on a drone. The drones run in their own nodes (drone_node.py).

  Gazebo      --/model/<drone>/odometry (nav_msgs/Odometry, true pose)-->        world node
  world node  --/swarm/truth  (std_msgs/String JSON, every tick)-->               drone nodes (sensors, scoring)
  drone nodes --/<drone>/cmd, /<drone>/status, /<drone>/plans (JSON)-->           world node
  world node  --/model/<drone>/cmd_vel  (geometry_msgs/Twist, BODY frame)-->      Gazebo
  world node  --/swarm/telemetry (JSON, 5 Hz), /swarm/report (JSON, latched)-->   dashboard

Each tick it reads the true pose of every drone, sends it to the drone nodes, and turns each drone's
latest command into the velocity it really flies (mission.WorldSim: its true heading, the wind once it
has lifted off, the downdraft and turbulence of the wakes above it). It scores what needs every drone
(separation, drone contacts, downwash), draws every drone's routes in Gazebo as the drones send them
(gz_lines.py: green = reference route, A* before take-off; a trail in the drone's own colour = the
route it has flown, drawn as it flies), and writes the
run's report from the drones' own summaries and its scoring.

Runs entirely on SIM time (/clock), so the mission behaves the same whether Gazebo runs at 100 % or
10 % real-time speed.
"""
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from . import config as C
from .grid_planner import PLANNERS, PLANNER_NAMES
from .ttc import GIVE_WAYS, GIVE_WAY_NAMES
from .mission import WorldSim, fleet_navs, build_report
from .report import format_report, dumps
from .paths import default_results_dir, baseline_report
from .gz_lines import GzLines, GROUND

TRAIL_STEP = 0.15        # m: a new point of a drone's trail (its flown route, in its colour) every TRAIL_STEP
TRAIL_DRAW = 0.5         # s: new trail segments are drawn this often
from .stop_signal import StopSignal, is_shutdown_error


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class DroneLink:
    """Latest true pose of one drone from Gazebo, velocity by finite differences."""

    def __init__(self, spec):
        self.spec = spec
        self.p = None
        self.q = (0.0, 0.0, 0.0, 1.0)
        self.v = np.zeros(3)
        self.stamp = None
        self.offset = np.zeros(3)
        self.offset_checked = False
        self.n = 0
        self.jumps = 0             # physically impossible jumps: odometry from TWO simulations

    def update(self, msg):
        pp = msg.pose.pose.position
        p = np.array([pp.x, pp.y, pp.z], float)
        if not self.offset_checked:
            # Some OdometryPublisher builds report pose relative to spawn -> detect & fix
            sx, sy = self.spec.spawn
            if math.hypot(p[0] - sx, p[1] - sy) > 1.0 and math.hypot(p[0], p[1]) < 0.3:
                self.offset = np.array([sx, sy, 0.0])
            self.offset_checked = True
        p = p + self.offset
        o = msg.pose.pose.orientation
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.p is not None and self.stamp is not None:
            dt = t - self.stamp
            # a 1.5 kg drone cannot move 2 m between two odometry messages, nor can its clock run
            # backwards: both happen when a second (leftover) Gazebo publishes the same topics
            if float(np.linalg.norm(p - self.p)) > 2.0 and abs(dt) < 0.5 or dt < -0.05:
                self.jumps += 1
            if 1e-4 < dt < 0.5:
                v = (p - self.p) / dt
                self.v = 0.5 * self.v + 0.5 * v
        self.p = p
        self.q = (o.x, o.y, o.z, o.w)
        self.stamp = t
        self.n += 1


def _idle_cmd():
    return {'v_body': np.zeros(3), 'yaw_rate': 0.0, 'airborne': False, 'state': 'PREVIEW', 'tel': {}}


class WorldNode(Node):
    def __init__(self):
        super().__init__('swarm_world')
        for name, default in (('nav', C.BASELINE), ('seed', 0), ('planner', C.PLANNER), ('give_way', C.GIVE_WAY),
                              ('speed_scale', 1.0), ('results_dir', ''), ('telemetry_rate', 5.0),
                              ('exit_when_done', False), ('lines', True), ('gui', False), ('study_run', False)):
            self.declare_parameter(name, default)
        planner = str(self.get_parameter('planner').value).strip().lower()
        if planner in PLANNERS:
            C.PLANNER = planner
        else:
            self.get_logger().error(f'unknown planner {planner!r} -> using {C.PLANNER}')
        give_way = str(self.get_parameter('give_way').value).strip().lower()
        if give_way in GIVE_WAYS:
            C.GIVE_WAY = give_way
        else:
            self.get_logger().error(f'unknown give_way {give_way!r} -> using {C.GIVE_WAY}')
        try:
            self.navs, self.key, self.label, self.fleet = fleet_navs(str(self.get_parameter('nav').value))
        except ValueError as e:
            self.get_logger().error(f'{e} -> using the baseline {C.BASELINE}')
            self.navs, self.key, self.label, self.fleet = fleet_navs(C.BASELINE)
        self.nav = self.navs[C.DRONES[0].id]        # the configuration (mixed fleets: navs per drone)
        self.exit_when_done = bool(self.get_parameter('exit_when_done').value)
        self.finished = False
        self.seed = int(self.get_parameter('seed').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        rd = os.path.expanduser(str(self.get_parameter('results_dir').value)) or default_results_dir()
        self.results_root = rd
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(rd, f'{self.key}_{stamp}')
        os.makedirs(self.run_dir, exist_ok=True)

        self.events = []
        self.sim = WorldSim(self.seed, 0, [d.id for d in C.DRONES], wind=True, log=self._log)
        self.links = {d.id: DroneLink(d) for d in C.DRONES}
        self.cmds = {d.id: _idle_cmd() for d in C.DRONES}
        self.status = {}
        self.plans = {}
        self.cmd_pubs = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        for d in C.DRONES:
            self.create_subscription(Odometry, f'/model/{d.name}/odometry',
                                     lambda m, i=d.id: self.links[i].update(m), 10)
            self.cmd_pubs[d.id] = self.create_publisher(Twist, f'/model/{d.name}/cmd_vel', 10)
            self.create_subscription(String, f'/{d.name}/cmd', lambda m, i=d.id: self.on_cmd(i, m), 10)
            self.create_subscription(String, f'/{d.name}/status', lambda m, i=d.id: self.on_status(i, m), latched)
            self.create_subscription(String, f'/{d.name}/plans', lambda m, i=d.id: self.on_plans(i, m), latched)
        self.truth_pub = self.create_publisher(String, '/swarm/truth', 10)
        self.tel_pub = self.create_publisher(String, '/swarm/telemetry', 10)
        self.rep_pub = self.create_publisher(String, '/swarm/report', latched)

        self.csv_f = open(os.path.join(self.run_dir, 'telemetry.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(['t', 'id', 'state', 'x', 'y', 'z', 'est_x', 'est_y', 'est_z', 'vx', 'vy', 'vz',
                           'speed', 'power_w', 'energy_j', 'soc', 'loc_err', 'cte', 'yielding', 'avoiding',
                           'downdraft'])

        self.z_fallback = False
        self.z_dr = {d.id: C.REST_Z for d in C.DRONES}
        self.t0 = None
        self.last_t = None
        self.tm = 0.0
        self.last_tel_t = -1e9
        self.wall0 = time.time()
        self.rtf_ref = None
        self.rtf = 0.0
        self.all_landed_t = None
        self.reported_all = False
        self.waiting_since = time.time()
        self.warned = False
        self.invalid = None            # set when the run cannot be trusted (two simulations at once)

        self.lines = GzLines(log=lambda m: self.get_logger().info(m)) if bool(self.get_parameter('lines').value) else None
        self.trails = {d.id: {'pts': [], 'sent': 0, 'done': False} for d in C.DRONES}   # trails: flown routes
        self.trail_t = -1e9
        self.timer = self.create_timer(1.0 / C.CONTROL_RATE, self.tick)
        # wall-clock heartbeat so the dashboard can say what we are waiting for
        self.wait_timer = self.create_timer(1.0, self.heartbeat, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.wait_reason = 'starting'
        self._log(0.0, f'Navigation: {self.label} [{self.nav.parts}] for all 5 drones; planner '
                       f'{PLANNER_NAMES[C.PLANNER]} (reference route: A*, on board before take-off); swarm give-way: '
                       f'{GIVE_WAY_NAMES[C.GIVE_WAY]}; seed {self.seed}')
        self.get_logger().info(f'World node up: results -> {self.run_dir}')
        self.get_logger().info('Waiting for /clock and /model/droneX/odometry from Gazebo ...')

    # ------------------------------------------------------------------
    def _log(self, t, msg):
        self.events.append((round(float(t), 2), msg))
        self.get_logger().info(f'[t={t:6.2f}s] {msg}')

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, i, msg):
        m = json.loads(msg.data)
        m['v_body'] = np.asarray(m['v_body'], float)
        self.cmds[i] = m

    def on_status(self, i, msg):
        self.status[i] = json.loads(msg.data)

    def on_plans(self, i, msg):
        m = json.loads(msg.data)
        old = self.plans.get(i) or {}
        self.plans[i] = m
        if self.lines is not None:                    # green: the drone's reference route (gz_lines)
            if m.get('ref3d') is not None and m.get('ref3d') != old.get('ref3d'):
                self.lines.set('ref', i, m['ref3d'])

    def draw_trails(self, tm, truth):
        """Trails, each in its drone's colour: the route each drone has actually flown (Gazebo's true position), from its take-off pad
        on the ground to where it is, a point every TRAIL_STEP m, ending on the ground at its landing pad.
        Only the new segments are sent (every TRAIL_DRAW s, and at once when a drone has landed)."""
        flush = False
        for i, tr in truth.items():
            T = self.trails[i]
            st = self.cmds[i].get('state', 'PREVIEW')
            if T['done'] or st == 'PREVIEW':
                continue
            p = [round(float(v), 3) for v in tr['p']]
            if not T['pts']:
                T['pts'].append([p[0], p[1], GROUND])          # from the ground at the take-off pad
            last = T['pts'][-1]
            if st == 'LANDED':
                T['pts'] += [p, [p[0], p[1], GROUND]]
                T['done'] = flush = True
            elif math.dist(p, last) >= TRAIL_STEP:
                T['pts'].append(p)
        if flush or tm - self.trail_t >= TRAIL_DRAW:
            self.trail_t = tm
            for i, T in self.trails.items():
                n = len(T['pts'])
                if n > 1 and n - 1 > T['sent']:                  # segments 1..sent are drawn
                    self.lines.extend('trail', i, T['pts'][T['sent']:], first_id=T['sent'] + 1)
                    T['sent'] = n - 1

    def heartbeat(self):
        """Wall-clock timer: runs even while sim time stands still (Gazebo paused / no /clock)."""
        if self.last_t is not None:
            return
        t = self.sim_now()
        missing = [lk.spec.name for lk in self.links.values() if lk.p is None]
        no_drone = [d.name for d in C.DRONES if d.id not in self.status]
        if t <= 0.0:
            self.wait_reason = ('Gazebo is paused (or /clock is not bridged) - press the orange Play button '
                                '(bottom-left of the Gazebo window)')
        elif missing:
            self.wait_reason = f'no odometry yet from {", ".join(missing)}'
        elif no_drone:
            self.wait_reason = f'drone nodes not up yet: {", ".join(no_drone)}'
        else:
            self.wait_reason = 'starting'
        if not self.warned and time.time() - self.waiting_since > 15.0 and (t <= 0.0 or missing):
            self.warned = True
            self.get_logger().warn(
                f'Still waiting: {self.wait_reason} (sim time {t:.1f} s). If Gazebo is already playing, check:\n'
                '  1) ign topic -l | grep odometry   2) ros2 topic hz /clock   3) ros2 topic hz /model/drone1/odometry\n'
                '  4) numpy<2 is installed (pip3 install "numpy<2")')
        self.tel_pub.publish(String(data=dumps({'waiting': True, 'reason': self.wait_reason, 'nav': self.key,
                                                 'run_dir': self.run_dir})))

    def tick(self):
        t = self.sim_now()
        if t <= 0.0 or any(lk.p is None for lk in self.links.values()):
            return
        if len(self.status) < len(self.links):        # the mission starts once every drone's computer is up
            return
        if self.last_t is not None and t <= self.last_t:
            return
        if self.t0 is None:
            self.t0 = t
        dt = 0.0 if self.last_t is None else float(np.clip(t - self.last_t, 0.0, 1.0))
        self.last_t = t
        tm = t - self.t0
        self.tm = tm
        wall = time.time()
        if self.rtf_ref is None:
            self.rtf_ref = (t, wall)
        elif wall - self.rtf_ref[1] > 2.0:
            self.rtf = (t - self.rtf_ref[0]) / (wall - self.rtf_ref[1])
            self.rtf_ref = (t, wall)

        truth = {}
        for i, lk in self.links.items():
            r, p, y = quat_to_rpy(*lk.q)
            pos = lk.p.copy()
            vel = lk.v.copy()
            if self.z_fallback:
                pos[2] = self.z_dr[i]
            truth[i] = {'p': pos, 'v': vel, 'yaw': y, 'roll': r, 'pitch': p}
        # guard: an OdometryPublisher without 3-D support reports z == 0.000 exactly
        # (a parked drone really sits at z = 0.127 m). Then use integrated altitude.
        if not self.z_fallback and all(abs(lk.p[2]) < 1e-4 for lk in self.links.values()) \
                and any(c.get('state') == 'TAKEOFF' for c in self.cmds.values()):
            self.z_fallback = True
            self.get_logger().error('Odometry has no altitude (OdometryPublisher in 2-D mode). Using '
                                    'integrated altitude instead. Updating Gazebo Fortress fixes this.')
            for i in truth:
                truth[i]['p'][2] = self.z_dr[i]
        # the true state of this tick to the drone nodes (their sensors and scoring) ...
        self.truth_pub.publish(String(data=dumps({'tm': tm, 'dt': dt, 'truth': truth})))
        # ... and their latest commands, turned into what they really fly
        out = self.sim.physics(tm, dt, truth, self.cmds)
        self.sim.score(tm, truth, self.cmds)
        for i, (v_w, yaw_rate) in out.items():
            if dt > 0:   # always integrated, so the fallback is ready if needed
                self.z_dr[i] = max(C.REST_Z, self.z_dr[i] + float(v_w[2]) * dt)
            self.publish_cmd(i, v_w, yaw_rate, truth[i])
        self.log_csv(tm, truth)
        if self.lines is not None:
            self.draw_trails(tm, truth)
        if self.invalid is None and any(lk.jumps >= 5 for lk in self.links.values()):
            bad = ', '.join(lk.spec.name for lk in self.links.values() if lk.jumps >= 5)
            self.invalid = 'odometry from two Gazebo simulations at once (another Gazebo was still running)'
            self.get_logger().error(
                f'THIS RUN IS INVALID: the odometry of {bad} jumps between two places - a second Gazebo '
                'simulation (left over from an earlier run) is publishing the same topics.\n'
                '  Stop everything:  pkill -f "ign gazebo"; pkill -f parameter_bridge; pkill -f swarm_intel\n'
                '  then start this run again. The run is saved, marked invalid, and left out of every study.')
        if t - self.last_tel_t >= 1.0 / float(self.get_parameter('telemetry_rate').value):
            self.last_tel_t = t
            self.tel_pub.publish(String(data=dumps(self.telemetry(tm))))
        self.check_done(tm)

    def publish_cmd(self, i, v_world, yaw_rate, tr):
        """v_world is the velocity the drone really flies (see WorldSim.physics). VelocityControl expects it
        in the BODY frame of Gazebo's model."""
        lk = self.links[i]
        R = quat_to_R(*lk.q)
        v_b = R.T @ np.asarray(v_world, float)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = float(v_b[0]), float(v_b[1]), float(v_b[2])
        k_att = 2.0 if self.cmds[i].get('airborne') else 0.0
        # PHYSICS: VelocityControl has no attitude dynamics; this stands in for the autopilot's own
        # inner attitude loop, which keeps the airframe level after a bump
        msg.angular.x = float(-k_att * tr['roll'])
        msg.angular.y = float(-k_att * tr['pitch'])
        msg.angular.z = float(yaw_rate)
        self.cmd_pubs[i].publish(msg)

    def log_csv(self, tm, truth):
        tm = round(tm, 3)
        for i in sorted(truth):
            tr = truth[i]
            c = self.cmds[i]
            tel = c.get('tel') or {}
            p, v = tr['p'], tr['v']
            e = tel.get('est') or [float('nan')] * 3
            self.csv.writerow([tm, i, c.get('state', 'PREVIEW'), *(round(float(x), 4) for x in p),
                               *(round(float(x), 4) for x in e), *(round(float(x), 4) for x in v),
                               round(float(np.linalg.norm(v)), 4), round(float(tel.get('power_w', 0.0)), 2),
                               round(float(tel.get('energy_j', 0.0)), 2), round(float(tel.get('soc', 100.0)), 3),
                               round(float(tel.get('loc_err', 0.0)), 4), round(float(tel.get('cte', 0.0)), 4),
                               int(bool(tel.get('yielding'))), int(bool(tel.get('avoiding'))),
                               round(self.sim.downdraft.get(i, 0.0), 3)])

    def telemetry(self, tm):
        drones = []
        for i in sorted(self.status):
            d = dict(self.status[i])
            d.pop('summary', None)
            d.pop('events', None)
            d['downdraft'] = round(float(self.sim.downdraft.get(i, 0.0)), 2)
            drones.append(d)
        return {'nav': self.key, 'nav_label': self.label, 'nav_parts': self.nav.parts, 't': round(tm, 2),
                'drones': drones, 'unknown_total': len(C.UNKNOWN_PINES),
                'all_landed': self.all_landed_t is not None,
                'events': self.all_events()[-8:], 'rtf': round(self.rtf, 3), 'run_dir': self.run_dir}

    def all_events(self):
        ev = list(self.events)
        for st in self.status.values():
            ev += [tuple(e) for e in st.get('events') or ()]
        return sorted(ev, key=lambda e: e[0])

    def landed(self, i):
        """Landed, and the drone's own summary says so (its last status is in)."""
        return (self.cmds[i].get('state') == 'LANDED'
                and (self.status.get(i) or {}).get('summary', {}).get('state') == 'LANDED')

    def report(self, final):
        summaries = {i: st['summary'] for i, st in self.status.items() if st.get('summary')}
        meta = {'model_version': C.MODEL_VERSION, 'nav': self.key, 'nav_label': self.label, 'nav_parts': self.nav.parts,
                'fleet_navs': [self.navs[d.id].key for d in C.DRONES], 'layout': 0,
                'wind_mean_ms': float(np.linalg.norm(self.sim.wind.mean)), 'seed': self.seed,
                'speed_scale': self.speed_scale, 'mission_time_s': self.tm,
                'all_landed_at_s': self.all_landed_t,
                'reference_paths': {str(i): (self.plans.get(i) or {}).get('ref') for i in sorted(self.links)}}
        rep = build_report(meta, summaries, self.sim, self.all_events())
        rep.update({'rtf_last': self.rtf, 'wall_time_s': time.time() - self.wall0, 'final': final,
                    'source': 'gazebo', '_run': os.path.basename(self.run_dir),
                    # made by the Gazebo study (gazebo_study), or watched (menu 1-12 / a launch by hand): only
                    # study runs enter the Gazebo study and the validation (paths.study_run)
                    'gui': bool(self.get_parameter('gui').value),
                    'study_run': bool(self.get_parameter('study_run').value)})
        if self.invalid:
            rep['invalid'] = self.invalid
        return rep

    def write_report(self, final):
        rep = self.report(final)
        rep['baseline'] = baseline_report(self.results_root, exclude=self.run_dir, planner=C.PLANNER,
                                          give_way=C.GIVE_WAY)
        with open(os.path.join(self.run_dir, 'summary.json'), 'w') as f:
            f.write(dumps(rep, indent=2))
        self.rep_pub.publish(String(data=dumps(rep)))
        self.csv_f.flush()
        return rep

    def check_done(self, tm):
        if len(self.status) < len(self.links):
            return
        if self.all_landed_t is None and all(self.landed(i) for i in self.links):
            self.all_landed_t = tm
            self._log(tm, 'All drones landed - mission report generated.')
        if not self.reported_all and self.all_landed_t is not None:
            self.reported_all = True
            rep = self.write_report(final=True)
            print('\n' + format_report(rep, width=104, baseline=rep.get('baseline')) + '\n', flush=True)
            self.get_logger().info(f'Fleet report saved to {self.run_dir}/summary.json. '
                                   'Compare all configurations with:  ros2 run swarm_intel study --gazebo')
            if self.exit_when_done:
                self.finished = True

    def shutdown(self, interrupted=False):
        """Save everything first (each step on its own, so one failure cannot lose the report)."""
        if not self.reported_all and self.status:
            try:
                self.write_report(final=self.all_landed_t is not None)
                if interrupted:
                    print(f'[swarm_intel] stopped before every drone had landed: partial report saved to '
                          f'{self.run_dir}/summary.json (left out of every study)', flush=True)
            except Exception as e:
                print(f'[swarm_intel] could not write summary.json: {e}', file=sys.stderr)
        try:
            self.csv_f.close()
        except Exception:
            pass
        try:
            for i in self.cmd_pubs:
                self.cmd_pubs[i].publish(Twist())
        except Exception:
            pass
        if self.lines is not None:
            self.lines.close()


def main(args=None):
    try:
        import numpy
        if int(numpy.__version__.split('.')[0]) >= 2:
            print('\033[93m[swarm_intel] WARNING: numpy %s detected. ROS 2 Humble binaries are built '
                  'for numpy 1.x; if you see "_ARRAY_API not found" run:  pip3 install "numpy<2"\033[0m'
                  % numpy.__version__, file=sys.stderr)
    except Exception:
        pass
    try:       # handle Ctrl+C ourselves, so the context is still valid while the report is saved
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    stop = StopSignal()                          # SIGINT/SIGTERM end the loop between callbacks
    node = WorldNode()
    interrupted = False
    try:
        while rclpy.ok() and not node.finished and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
        interrupted = bool(stop) and not node.finished
    except KeyboardInterrupt:
        interrupted = True
    except Exception as e:                       # e.g. ExternalShutdownException
        interrupted = True
        if not is_shutdown_error(e, stop, rclpy):
            raise
    finally:
        node.shutdown(interrupted)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
