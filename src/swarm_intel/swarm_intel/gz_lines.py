"""Every drone's lines, drawn live in the Gazebo window by the world node.

  GREEN   the drone's reference route: A*, planned on board on the pad before take-off (the drone
          sends it, mission.DroneRuntime.plans), drawn as the whole flight: up from the take-off pad,
          along the route at cruise altitude, down to the landing pad
  TRAIL   in the drone's own colour (config.DRONE_COLORS, the colour of its body and landing pad):
          the route it has actually flown, from the take-off pad on the ground to where it is now,
          drawn as it flies (Gazebo's true position, a point every 0.15 m, new segments added every
          half second) and ending on the ground at its landing pad

The routes the drones fly are planned on board in flight (RRT*, or A* with planner:=astar, re-planned
every second); each trail shows where that took its drone, next to the green route it is measured
against. Both lines are drawn a little below the drone (the trail just above green, and shifted 6 cm
sideways, so both stay visible where they agree, also on the vertical climb and descent). None of the
drone colours is close to that green (config.py).

Drawing uses the marker service of the Gazebo Fortress GUI (/marker_array, ignition.msgs.Marker_V),
called with the `ign service` command line tool from a background thread, so no node ever waits for
it. A reference route is drawn whole (only its newest version is sent); a trail only gets its new
segments, so drawing it costs the same however long it is. Without the GUI (gui:=false) it stays off.
"""
import math
import shutil
import subprocess
import threading
import time

import numpy as np

from . import config as C

GREEN = (0.05, 0.80, 0.20)
BELOW = {'ref': 0.12, 'trail': 0.07}                  # m below the drone
SHIFT = {'ref': (0.0, 0.0), 'trail': (0.06, 0.0)}     # m sideways: the vertical parts side by side
THICK = {'ref': 0.07, 'trail': 0.05}
GROUND = 0.02                                         # m: lowest drawing height (the pads)


def hex_rgb(h):
    h = h.lstrip('#')
    return tuple(int(h[k:k + 2], 16) / 255.0 for k in (0, 2, 4))


def line_color(kind, drone_id):
    """Green for every reference route; a trail in its drone's own colour."""
    return GREEN if kind == 'ref' else hex_rgb(C.DRONE_COLORS[drone_id])


def _color(name, rgb, a=1.0):
    return f'{name} {{ r: {rgb[0]:.3f} g: {rgb[1]:.3f} b: {rgb[2]:.3f} a: {a:.2f} }}'


def segment_marker(ns, mid, a, b, thick, rgb):
    """Protobuf text of one BOX marker spanning the segment a -> b (3-D points)."""
    d = b - a
    L = float(np.linalg.norm(d))
    yaw = math.atan2(d[1], d[0])
    pitch = -math.atan2(d[2], math.hypot(d[0], d[1]))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    q = (-sy * sp, cy * sp, sy * cp, cy * cp)                 # Rz(yaw) * Ry(pitch)
    c = (a + b) / 2.0
    mat = ' '.join([_color('ambient', rgb), _color('diffuse', rgb), _color('emissive', rgb)])
    return (f'marker {{ ns: "{ns}" id: {mid} action: ADD_MODIFY type: BOX visibility: GUI '
            f'pose {{ position {{ x: {c[0]:.3f} y: {c[1]:.3f} z: {c[2]:.3f} }} '
            f'orientation {{ x: {q[0]:.5f} y: {q[1]:.5f} z: {q[2]:.5f} w: {q[3]:.5f} }} }} '
            f'scale {{ x: {L + thick:.3f} y: {thick:.3f} z: {thick:.3f} }} '
            f'material {{ {mat} lighting: false }} }}')


def delete_all_marker(ns):
    return f'marker {{ ns: "{ns}" action: DELETE_ALL }}'


def segment_markers(kind, drone_id, pts3d, first_id=1):
    """(namespace, markers) of the segments between consecutive 3-D points, ids first_id, first_id+1, ..."""
    ns = f'{kind}_drone{drone_id}'
    if pts3d is None or len(pts3d) < 2:
        return ns, []
    pts = np.array(pts3d, float)
    pts[:, 2] = np.maximum(GROUND, pts[:, 2] - BELOW[kind])
    pts[:, :2] += SHIFT[kind]
    rgb = line_color(kind, drone_id)
    out = []
    for k in range(len(pts) - 1):
        if float(np.linalg.norm(pts[k + 1] - pts[k])) > 1e-3:
            out.append(segment_marker(ns, first_id + k, pts[k], pts[k + 1], THICK[kind], rgb))
    return ns, out


class GzLines:
    """Draws from a background thread. set() replaces a whole line (only its newest version is sent);
    extend() adds segments to a line (all of them are sent, in order)."""

    def __init__(self, log=print):
        self.log = log
        self.cli = shutil.which('ign') or shutil.which('gz')
        prefix = 'ignition' if self.cli and self.cli.endswith('ign') else 'gz'
        self.cmd = [self.cli or 'ign', 'service', '-s', '/marker_array', '--reqtype', f'{prefix}.msgs.Marker_V',
                    '--reptype', f'{prefix}.msgs.Boolean', '--timeout', '3000', '--req']
        self.replace = {}                    # ns -> markers of the newest version (drawn after a DELETE_ALL)
        self.adds = {}                       # ns -> markers still to add
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.idle = threading.Event()
        self.idle.set()
        self.stop = False
        self.ok_once = False
        self.fails = 0
        self.enabled = self.cli is not None
        if not self.enabled:
            self.log('lines: no ign/gz command found - the routes are not drawn')
            return
        self.thread = threading.Thread(target=self._run, name='gz_lines', daemon=True)
        self.thread.start()

    def set(self, kind, drone_id, pts3d):
        if not self.enabled:
            return
        ns, markers = segment_markers(kind, drone_id, pts3d)
        with self.lock:
            self.replace[ns] = markers
            self.idle.clear()
        self.wake.set()

    def extend(self, kind, drone_id, pts3d, first_id):
        """Add the segments between pts3d[0], pts3d[1], ... with marker ids first_id, first_id+1, ..."""
        if not self.enabled:
            return
        ns, markers = segment_markers(kind, drone_id, pts3d, first_id)
        if not markers:
            return
        with self.lock:
            self.adds.setdefault(ns, []).extend(markers)
            self.idle.clear()
        self.wake.set()

    def _next_job(self):
        with self.lock:
            if self.replace:
                ns = next(iter(self.replace))
                return 'replace', ns, self.replace.pop(ns)
            if self.adds:
                ns = next(iter(self.adds))
                return 'add', ns, self.adds.pop(ns)
            self.idle.set()
            return None

    def _run(self):
        while not self.stop:
            self.wake.wait(timeout=1.0)
            self.wake.clear()
            job = self._next_job()
            if job is None:
                continue
            kind, ns, markers = job
            ok = self._send([delete_all_marker(ns)]) if kind == 'replace' else True
            for k in range(0, len(markers), 150):
                ok = ok and self._send(markers[k:k + 150])
            if not ok:
                with self.lock:                  # retry later (the GUI may still be starting)
                    if kind == 'replace':
                        self.replace.setdefault(ns, markers)
                    else:
                        self.adds[ns] = markers + self.adds.get(ns, [])
                self.fails += 1
                if self.fails == 5 and not self.ok_once:
                    self.log('lines: Gazebo GUI marker service not answering yet (/marker_array) - will keep trying')
                time.sleep(min(10.0, 1.0 + self.fails))
            else:
                if not self.ok_once:
                    self.log('lines: drawing every drone\'s reference route (green, A* before take-off) and the '
                             'route it has flown (its trail, in the drone\'s colour, drawn as it flies)')
                self.ok_once = True
                self.fails = 0
            self.wake.set()                      # look for more work at once

    def _send(self, markers):
        try:
            r = subprocess.run(self.cmd + [' '.join(markers)], capture_output=True, text=True, timeout=8.0)
        except Exception:
            return False
        return r.returncode == 0 and 'true' in (r.stdout or '').lower()

    def close(self, flush_s=3.0):
        """Stop drawing; first give the last segments (the landings) up to flush_s seconds to go out."""
        if self.enabled and self.ok_once:
            self.idle.wait(timeout=flush_s)
        self.stop = True
        self.wake.set()
