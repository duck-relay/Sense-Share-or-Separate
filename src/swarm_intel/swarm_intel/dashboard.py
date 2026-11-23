#!/usr/bin/env python3
"""Live terminal dashboard for the swarm mission.

    ros2 run swarm_intel dashboard

Shows every drone's state, position, height, speed, power, energy, battery,
localisation error (EKF estimate vs truth) and deviation from its reference route, and the
fleet's progress; prints the fleet's mission summary when every drone has landed.
"""
import json
import os
import shutil
import sys

from . import config as C
from . import report as R
from .report import col, format_summary

STATE_COL = {'PREVIEW': R.C_GREY, 'TAKEOFF': R.C_YELLOW, 'SCAN': R.C_YELLOW, 'CRUISE': R.C_GREEN,
             'ESCAPE': R.C_RED, 'LANDING': R.C_CYAN, 'LANDED': R.C_BLUE}


def bar(frac, n=30):
    frac = max(0.0, min(1.0, frac))
    k = int(round(frac * n))
    return '█' * k + '░' * (n - k)


# (header, unit, width, value formatter) - header, units and rows all come from this one table
COLUMNS = [
    ('X', 'm', 7, lambda d: f"{d['pos'][0]:.2f}"),
    ('Y', 'm', 7, lambda d: f"{d['pos'][1]:.2f}"),
    ('HEIGHT', 'm', 7, lambda d: f"{d['height']:.2f}"),
    ('SPEED', 'm/s', 6, lambda d: f"{d['speed']:.2f}"),
    ('Vz', 'm/s', 6, lambda d: f"{d['vz']:.2f}"),
    ('POWER', 'W', 7, lambda d: f"{d['power_w']:.1f}"),
    ('ENERGY', 'kJ', 7, lambda d: f"{d['energy_kj']:.3f}"),
    ('BATT', '%', 6, lambda d: f"{d['soc']:.1f}"),
    ('LOC.ERR', 'm', 8, lambda d: f"{d['loc_err']:.3f}"),
    ('EKF 1σ', 'm', 7, lambda d: f"{d.get('sigma') or 0:.3f}"),
    ('MRK', '#', 4, lambda d: '-' if d.get('markers') is None else str(d['markers'])),
    ('PATH.ERR', 'm', 9, lambda d: f"{d['cte']:.3f}"),
    ('GOAL', 'm', 6, lambda d: f"{d['goal_dist']:.1f}"),
    ('REPLAN', '#', 7, lambda d: str(d['replans'])),
    ('DOWN', 'm/s', 6, lambda d: f"{d.get('downdraft', 0.0):.1f}"),
]


def drone_col(d):
    """The drone's display colour as a 256-colour terminal code (its colour in Gazebo and the plots)."""
    h = str(d.get('color') or '#888888').lstrip('#')
    if len(h) != 6:
        return ''
    r, g, b = (int(h[k:k + 2], 16) for k in (0, 2, 4))
    q = lambda v: min(5, max(0, int(round(v / 255 * 5))))
    return f'\033[38;5;{16 + 36 * q(r) + 6 * q(g) + q(b)}m'


def render(tel, rep=None, width=None):
    width = width or shutil.get_terminal_size((120, 40)).columns
    width = max(100, min(width, 150))
    L = []
    nav = tel.get('nav_label') or tel.get('mode', '?')
    mcol = R.C_YELLOW if tel.get('nav') == C.BASELINE else R.C_GREEN
    head = (f" DRONE NAVIGATION STUDY │ {col(nav, mcol + R.C_BOLD)} │ sim t = {tel.get('t', 0):6.1f} s"
            f" │ RTF {tel.get('rtf', 0):4.2f} ")
    L.append(col('━' * width, R.C_CYAN))
    L.append(head)
    if tel.get('nav_parts'):
        L.append(col(f" all 5 drones use: {tel['nav_parts']}  -  each drone fuses everything in its EKF", R.C_GREY))
    L.append(col('━' * width, R.C_CYAN))
    drones = tel.get('drones', [])
    if drones:
        prog = sum(d.get('progress', 0.0) for d in drones) / len(drones)
        landed = sum(1 for d in drones if d['state'] == 'LANDED')
        if all(d['state'] == 'PREVIEW' for d in drones):
            info = col(f"PREVIEW: every drone plans its green reference route (A*, on board). Take-off in "
                       f"{max(0.0, C.PREVIEW_TIME - tel.get('t', 0)):.1f} s", R.C_CYAN)
        else:
            info = f"{landed} of {len(drones)} landed"
        L.append(f" {col('FLEET', R.C_BOLD)} {col(bar(prog / 100.0), R.C_CYAN)} {prog:5.1f} % of the routes flown  {info}")
    L.append('')
    hdr = ' DRONE    STATE     ' + ''.join(f'{h:>{w}}' for h, _, w, _ in COLUMNS)
    units = ' ' * len(' DRONE    STATE     ') + ''.join(f'{u:>{w}}' for _, u, w, _ in COLUMNS)
    L.append(col(hdr, R.C_BOLD))
    L.append(col(units, R.C_GREY))
    for d in drones:
        c = drone_col(d)
        st = d['state'] + ('▼G' if d.get('yielding') else '') + ('~A' if d.get('avoiding') else '')
        row = (f" {col(('● ' + d.get('name', str(d['id']))).ljust(8), c + R.C_BOLD)}"
               f" {col(st.ljust(10)[:10], STATE_COL.get(d['state'], ''))}"
               + ''.join(f'{fn(d):>{w}}' for _, _, w, fn in COLUMNS))
        L.append(row)
    L.append('')
    fleet_e = sum(d['energy_kj'] for d in drones)
    fleet_p = sum(d['power_w'] for d in drones)
    known = [d.get('unknown_known', 0) for d in drones]
    L.append(f" Fleet power {fleet_p:7.1f} W │ fleet energy {fleet_e:6.2f} kJ │ hidden pines mapped (mean of the drones) "
             f"{(sum(known) / len(known)) if known else 0:.1f}/{tel.get('unknown_total', 0)}"
             f" │ ▼G giving way  ~A avoiding  MRK markers  DOWN downwash felt")
    L.append(col(' Gazebo: green = each drone\'s reference route (A* on board before take-off); a trail in the drone\'s '
                 'colour = the route it has flown (RRT* or A*, re-planned every second)', R.C_GREY))
    L.append(col('─' * width, R.C_GREY))
    L.append(col(' EVENTS', R.C_BOLD))
    for (t, m) in tel.get('events', [])[-7:]:
        L.append(f"  {col(f't={t:6.2f}s', R.C_GREY)}  {m[:width - 14]}")
    if rep is not None:
        L.append('')
        L.append(format_summary(rep, baseline=rep.get('baseline'), width=width))
        L.append(col(f" Full report: in the launch terminal and in {tel.get('run_dir', 'the run folder')}/summary.json",
                     R.C_GREY))
        L.append(col(' Next: run another configuration (nav:=...), then  ros2 run swarm_intel study --gazebo', R.C_GREY))
    return '\n'.join(L)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String

    if os.environ.get('NO_COLOR'):
        R.USE_COLOR = False

    class Dashboard(Node):
        def __init__(self):
            super().__init__('swarm_dashboard')
            self.tel = None
            self.rep = None
            self.dirty = False
            self.create_subscription(String, '/swarm/telemetry', self.on_tel, 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(String, '/swarm/report', self.on_rep, latched)
            self.create_timer(0.25, self.draw)          # wall-clock refresh
            sys.stdout.write('\033[2J\033[H Waiting for /swarm/telemetry ... (start the launch file)\n')
            sys.stdout.flush()

        def on_tel(self, msg):
            self.tel = json.loads(msg.data)
            self.dirty = True

        def on_rep(self, msg):
            self.rep = json.loads(msg.data)
            self.dirty = True

        def draw(self):
            if not self.dirty or self.tel is None:
                return
            self.dirty = False
            if self.tel.get('waiting'):
                out = (col(' DRONE NAVIGATION STUDY', R.C_BOLD)
                       + f"  config: {self.tel.get('nav', self.tel.get('mode', '?'))}\n\n"
                       + col(' Waiting: ', R.C_YELLOW) + self.tel.get('reason', '') + '\n'
                       + '\n Green in Gazebo: each drone\'s reference route (A*, planned on board before take-off);'
                         ' in the drone\'s colour: the route it has flown.\n')
            else:
                out = render(self.tel, self.rep)
            # repaint in place (no full clear -> no flicker): home, each line + clear-to-EOL, clear below
            sys.stdout.write('\033[H' + '\n'.join(line + '\033[K' for line in out.split('\n')) + '\n\033[J')
            sys.stdout.flush()

    rclpy.init(args=args)
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:                        # ExternalShutdownException when ROS shuts down first
        if type(e).__name__ != 'ExternalShutdownException':
            raise
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
