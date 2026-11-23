#!/usr/bin/env python3
"""ROS 2 drone node: ONE drone's on-board computer (an edge node). The launch file starts one per drone.

  world node --/swarm/truth  (std_msgs/String JSON, every world tick)-->  every drone node
  drone node --/swarm/radio  (JSON, every tick, drones with the swarm radio)-->  the other drone nodes
  drone node --/<drone>/cmd    (JSON: air velocity in its own frame, yaw rate, state)-->  world node
  drone node --/<drone>/status (JSON, 5 Hz and on landing: telemetry, summary, new log lines)-->  world node
  drone node --/<drone>/plans  (JSON, latched, whenever its route changes)-->  world node (draws the
                               reference route in green; the trail of the flown route, in the
                               drone's colour, it
                               draws itself from Gazebo's positions)

Everything that decides how the drone flies runs here, on the drone: its EKF, its map (with the radio:
its copy of the shared map, kept up to date with the other drones' scans), path planning - the
reference route with A* on the pad before take-off and the flown route with RRT* or A* in flight, both
computed here and nowhere else - the turn trading with the other drones (ttc.py) and the control.
In flight the drone plans again every second from where it is (rolling re-planning, agent.py). The
routes are computed by a separate planner process of this computer (onboard_planner.ProcessPlanner),
like a companion computer next to the autopilot: the flight loop keeps running at full rate while a
route is computed and takes the new route when it is ready, so the real planning time of the drone's
computer shows in the flight (planner_process:=false computes it inside the flight loop instead).
The true state in /swarm/truth only feeds the drone's simulated sensors (its own pose, other drones
in reach of its proximity sensors and UWB) and its scoring, like the sensor plugins of a simulator.

Each tick runs when the world's truth for that tick arrives (simulation time), so the drone follows
the simulation at any real-time factor. The drone node is the same code as one drone of the headless
twin (mission.DroneRuntime).
"""
import json
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from . import config as C
from .grid_planner import PLANNERS
from .ttc import GIVE_WAYS
from .mission import DroneRuntime, fleet_navs, decode_message
from .report import dumps
from .stop_signal import StopSignal, is_shutdown_error
from .onboard_planner import ProcessPlanner

STATUS_RATE = 5.0      # Hz


def decode_truth(data):
    return {int(i): {'p': np.asarray(q['p'], float), 'v': np.asarray(q['v'], float), 'yaw': float(q['yaw']),
                     'roll': float(q['roll']), 'pitch': float(q['pitch'])} for i, q in data.items()}


class DroneNode(Node):
    def __init__(self):
        super().__init__('drone')
        for name, default in (('drone_id', 1), ('nav', C.BASELINE), ('seed', 0), ('planner', C.PLANNER),
                              ('give_way', C.GIVE_WAY), ('speed_scale', 1.0), ('layout', 0),
                              ('planner_process', True)):
            self.declare_parameter(name, default)
        planner = str(self.get_parameter('planner').value).strip().lower()
        give_way = str(self.get_parameter('give_way').value).strip().lower()
        if planner in PLANNERS:
            C.PLANNER = planner
        if give_way in GIVE_WAYS:
            C.GIVE_WAY = give_way
        did = int(self.get_parameter('drone_id').value)
        spec = next(d for d in C.DRONES if d.id == did)
        try:
            navs = fleet_navs(str(self.get_parameter('nav').value))[0]
        except ValueError as e:
            self.get_logger().error(f'{e} -> using the baseline {C.BASELINE}')
            navs = fleet_navs(C.BASELINE)[0]
        layout = int(self.get_parameter('layout').value)
        if layout:
            from . import layouts
            layouts.apply(layout)
        self.spec = spec
        self.events = []
        self.last_tm = None            # mission time of the last tick this computer processed
        self.rt = DroneRuntime(spec, navs[did], int(self.get_parameter('seed').value), layout,
                               float(self.get_parameter('speed_scale').value), log=self._log)
        if bool(self.get_parameter('planner_process').value):
            self.rt.agent.planner = ProcessPlanner(log=lambda msg: self.get_logger().warn(f'{spec.name}: {msg}'))
        self.inbox = {}
        self.state = None
        self.last_status = -1e9
        self.plan_sent = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.radio_pub = self.create_publisher(String, '/swarm/radio', 20)
        self.cmd_pub = self.create_publisher(String, f'/{spec.name}/cmd', 10)
        self.status_pub = self.create_publisher(String, f'/{spec.name}/status', latched)
        self.plans_pub = self.create_publisher(String, f'/{spec.name}/plans', latched)
        self.create_subscription(String, '/swarm/truth', self.on_truth, 10)
        if self.rt.agent.swarm:
            self.create_subscription(String, '/swarm/radio', self.on_radio, 50)
        self.get_logger().info(f'{spec.name}: on-board computer up ({navs[did].label}; planner {C.PLANNER}, '
                               f'give-way {C.GIVE_WAY})')
        self.publish_status(0.0)           # tells the world node that this drone's computer is up

    def _log(self, t, msg):
        self.events.append((round(float(t), 2), msg))
        self.get_logger().info(f'[t={t:6.2f}s] {msg}')

    def on_radio(self, msg):
        m = json.loads(msg.data)
        if int(m.get('id', -1)) != self.spec.id:
            self.inbox[int(m['id'])] = decode_message(m)

    def on_truth(self, msg):
        data = json.loads(msg.data)
        tm, dt = float(data['tm']), float(data['dt'])
        truth = decode_truth(data['truth'])
        if self.spec.id not in truth:
            return
        rt = self.rt
        # the time since the last tick THIS computer processed: while it was busy (planning) it may have
        # missed ticks, and its filter and its energy count must cover the whole gap, not one world step
        if self.last_tm is not None and abs((tm - self.last_tm) - dt) > 1e-6:
            dt = float(np.clip(tm - self.last_tm, 0.0, 1.0))
        self.last_tm = tm
        inbox = dict(self.inbox)
        rt.sense(tm, dt, truth, inbox)
        out = rt.message()
        if out is not None:
            self.radio_pub.publish(String(data=dumps(out)))
        cmd = rt.control(tm, dt, truth, inbox)
        a = rt.agent
        tel = {'est': [float(v) for v in a.ekf.pos], 'power_w': a.power, 'energy_j': a.battery.used_j,
               'soc': a.battery.soc, 'loc_err': a.last_loc_err, 'cte': a.last_cte, 'yielding': a.yielding,
               'avoiding': a.avoiding}
        self.cmd_pub.publish(String(data=dumps({'id': self.spec.id, 'tm': tm, **cmd, 'tel': tel})))
        key = (a.plan_version, a.ref is not None)
        if key != self.plan_sent:
            self.plan_sent = key
            self.plans_pub.publish(String(data=dumps(rt.plans())))
        if a.state != self.state or tm - self.last_status >= 1.0 / STATUS_RATE:
            self.state = a.state
            self.last_status = tm
            self.publish_status(tm, truth[self.spec.id])

    def publish_status(self, tm, truth_i=None):
        a = self.rt.agent
        st = self.rt.status(truth_i)
        st.update({'tm': tm, 'summary': a.summary(), 'events': self.events})    # the whole log: latched, depth 1
        self.status_pub.publish(String(data=dumps(st)))


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    stop = StopSignal()                          # SIGINT/SIGTERM end the loop between callbacks
    node = DroneNode()
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:                       # e.g. ExternalShutdownException
        if not is_shutdown_error(e, stop, rclpy):
            raise
    finally:
        try:
            node.publish_status(node.last_tm or 0.0)
        except Exception:
            pass
        node.rt.agent.planner.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
