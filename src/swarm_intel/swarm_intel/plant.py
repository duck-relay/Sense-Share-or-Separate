"""Headless stand-in for Gazebo used by the batch runner and self-tests.

Mimics what Gazebo's VelocityControl plugin does to the drone: the commanded
velocity is tracked almost instantly (small first-order lag), gravity leaks
in a little every physics step, the ground stops the drone, and obstacles
physically block it (same collision radius as the Gazebo model).
"""
import math
import numpy as np

from . import config as C


class KinematicPlant:
    def __init__(self, world, tau=0.06, sag=0.02):
        self.world = world
        self.tau = tau
        self.sag = sag
        self.s = {}
        for d in C.DRONES:
            self.s[d.id] = {'p': np.array([d.spawn[0], d.spawn[1], C.REST_Z], float),
                            'v': np.zeros(3), 'yaw': 0.0}

    def truth(self):
        return {i: {'p': s['p'].copy(), 'v': s['v'].copy(), 'yaw': s['yaw'], 'roll': 0.0, 'pitch': 0.0}
                for i, s in self.s.items()}

    def step(self, cmds, dt):
        a = 1.0 - math.exp(-dt / self.tau)
        for i, (v_cmd, yaw_rate) in cmds.items():
            s = self.s[i]
            target = np.array(v_cmd, float)
            if s['p'][2] > C.REST_Z + 0.005 or target[2] > 0:
                target[2] -= self.sag
            s['v'] = s['v'] + a * (target - s['v'])
            p = s['p'] + s['v'] * dt
            if p[2] < C.REST_Z:
                p[2] = C.REST_Z
                s['v'][2] = max(0.0, s['v'][2])
            # obstacles push the drone out (rigid contact of the rotor disc or the feet)
            free = p.copy()
            _, lims = self.world.contact_gap(p)
            for (cx, cy), lim in zip(self.world.xy, lims):
                if lim <= 0:
                    continue
                dx, dy = p[0] - cx, p[1] - cy
                d = math.hypot(dx, dy)
                if d < lim and d > 1e-6:
                    p[0] = cx + dx / d * lim
                    p[1] = cy + dy / d * lim
            p[0] = float(np.clip(p[0], -C.ARENA_HALF + 0.3, C.ARENA_HALF - 0.3))
            p[1] = float(np.clip(p[1], -C.ARENA_HALF + 0.3, C.ARENA_HALF - 0.3))
            if dt > 0 and not np.allclose(free[:2], p[:2]):
                s['v'][:2] = (p[:2] - s['p'][:2]) / dt      # a blocked drone really stops
            s['p'] = p
            s['yaw'] = (s['yaw'] + yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        # drones are solid too: overlapping rotor discs push each other apart
        ids = list(self.s)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                sa, sb = self.s[ids[a]], self.s[ids[b]]
                d = sa['p'][:2] - sb['p'][:2]
                dh = float(np.hypot(d[0], d[1]))
                lim = 2 * C.DRONE_COLLISION_R
                if dh < lim and abs(sa['p'][2] - sb['p'][2]) < 0.1 and dh > 1e-6:
                    n = d / dh
                    push = 0.5 * (lim - dh)
                    sa['p'][:2] += push * n
                    sb['p'][:2] -= push * n
                    for sv, sgn in ((sa, 1.0), (sb, -1.0)):
                        vn = float(sv['v'][:2] @ n) * sgn
                        if vn < 0:                        # moving into the other drone: stop that part
                            sv['v'][:2] -= vn * n * sgn


def run_headless(nav, seed=0, speed_scale=1.0, dt=None, max_t=320.0, record=False):
    """Run one full mission without Gazebo (until every drone has landed or given up). Returns
    (report, trajectory list)."""
    from .mission import MissionCore
    dt = dt or 1.0 / C.CONTROL_RATE
    core = MissionCore(nav, seed=seed, speed_scale=speed_scale)
    plant = KinematicPlant(core.world)
    traj = []
    k = 0
    t = 0.0
    while t < max_t:
        truth = plant.truth()
        cmds = core.step(t, truth)
        if record:
            for i, a in core.agents.items():
                tr = truth[i]
                traj.append((round(t, 3), i, *tr['p'], *a.ekf.pos, a.power, a.battery.used_j, a.state, a.last_cte))
        plant.step(cmds, dt)
        k += 1
        t = k * dt
        if core.all_landed_t is not None:
            break
    return core.report(), traj
