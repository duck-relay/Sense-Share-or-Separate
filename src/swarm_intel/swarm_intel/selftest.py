#!/usr/bin/env python3
"""Self-test of the package: checks the code against what the study relies on.

    ros2 run swarm_intel selftest            # everything (about 3 min)
    ros2 run swarm_intel selftest --quick    # without the mission checks (about 20 s)

  geometry   the segment-vs-grid check agrees with an exact (rational) reference, in both directions;
             the connectivity check agrees with a plain flood fill
  planners   on random maps both planners return only collision-free routes from start to goal,
             A* finds a route exactly when one exists, RRT* never "finds" one that does not exist,
             RRT* replays exactly for the same random stream, and the drone node's planner process
             returns exactly the route the headless twin computes inline
  ttc        turn trading picks the lowest-energy order of all orders; its trading cycles are
             consistent; ties favour no drone number
  mission    (headless twin) every drone has its own computer state; every drone plans its reference
             route (A*) on its pad before take-off, which costs no drone energy and is not counted as
             in-flight planning; the flown route is planned in flight and re-planned every second
             (rolling); a mission replays exactly;
             without the fixed-priority comparison the drones' priority numbers change nothing;
             the A* and fixed-priority cases fly as separate cases

Exit code 0 when every check passes. The same checks run under pytest (test/test_selftest.py).
"""
import argparse
import itertools
import sys
from fractions import Fraction

import numpy as np

from . import config as C
from . import grid_planner as G
from . import ttc

SKIP_WALL_CLOCK = ('fleet_plan_time_ms_mean', 'fleet_plan_time_ms_max')     # depend on the computer


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
def _segment_reference(bl, ax, ay, bx, by):
    """segment_cells_free in exact arithmetic: the cells of the end points, of every open piece between
    two grid-line crossings, and the two cells beside every corner passed on the way."""
    n0, n1 = bl.shape
    ax, ay, bx, by = Fraction(ax), Fraction(ay), Fraction(bx), Fraction(by)

    def fl(v):
        return v.numerator // v.denominator

    def free(i, j):
        return 0 <= i < n0 and 0 <= j < n1 and not bl[i, j]

    if not (free(fl(ax), fl(ay)) and free(fl(bx), fl(by))):
        return False
    dx, dy = bx - ax, by - ay
    tx, ty = set(), set()
    for d, a, b, out in ((dx, ax, bx, tx), (dy, ay, by, ty)):
        if d != 0:
            for g in range(min(fl(a), fl(b)), max(fl(a), fl(b)) + 2):
                t = (g - a) / d
                if 0 <= t <= 1:
                    out.add(t)
    ev = sorted({Fraction(0), Fraction(1)} | tx | ty)
    for t0, t1 in zip(ev[:-1], ev[1:]):
        tm = (t0 + t1) / 2
        if not free(fl(ax + tm * dx), fl(ay + tm * dy)):
            return False
    for t in tx & ty:
        if 0 < t < 1:
            ci, cj = fl(ax + t * dx), fl(ay + t * dy)
            bi, bj = (ci - 1 if dx > 0 else ci), (cj - 1 if dy > 0 else cj)        # the cell before the corner
            if not (free(bi + (1 if dx > 0 else -1), bj) and free(bi, bj + (1 if dy > 0 else -1))):
                return False
    return True


def _flood_connected(bl, s, g):
    seen = np.zeros_like(bl, bool)
    seen[s] = True
    stack = [s]
    n0, n1 = bl.shape
    while stack:
        i, j = stack.pop()
        if (i, j) == g:
            return True
        for a, b in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)):
            if 0 <= a < n0 and 0 <= b < n1 and not bl[a, b] and not seen[a, b]:
                seen[a, b] = True
                stack.append((a, b))
    return False


def check_geometry(n_grids=800):
    rng = np.random.default_rng(7)
    wrong = asym = total = conn_wrong = 0
    for _ in range(n_grids):
        n = int(rng.integers(4, 12))
        bl = rng.random((n, n)) < rng.choice([0.0, 0.1, 0.25, 0.4])
        flat = G.flat_cells(bl)

        def point():                         # also exactly on grid lines and corners
            q = int(rng.choice([64, 2, 1, 4]))
            return int(rng.integers(0, n * q)) / q, int(rng.integers(0, n * q)) / q

        for _ in range(25):
            (ax, ay), (bx, by) = point(), point()
            got = G.segment_cells_free(flat, n, n, 0.0, 1.0, ax, ay, bx, by)
            total += 1
            asym += got != G.segment_cells_free(flat, n, n, 0.0, 1.0, bx, by, ax, ay)
            wrong += got != _segment_reference(bl, ax, ay, bx, by)
        free = np.argwhere(~bl)
        if len(free) >= 2:
            s, g = (tuple(free[int(k)]) for k in rng.integers(0, len(free), 2))
            truth = _flood_connected(bl, s, g)
            full = G.cells_connected(flat, n, n, s[0] * n + s[1], g[0] * n + g[1])
            part = G.cells_connected(flat, n, n, s[0] * n + s[1], g[0] * n + g[1], 8)
            conn_wrong += (full != truth) or (part is not None and part != truth)
    return [('segment check equals the exact reference', wrong == 0, f'{total} segments, {wrong} differ'),
            ('segment check gives the same answer in both directions', asym == 0, f'{asym} differ'),
            ('connectivity check equals a plain flood fill', conn_wrong == 0, f'{n_grids} grids, {conn_wrong} differ')]


# ---------------------------------------------------------------------------
# planners
# ---------------------------------------------------------------------------
def _random_map(rng):
    half = float(rng.choice([6.0, 10.0]))
    gm = G.GridMap(half=half, res=float(rng.choice([0.2, 0.1])))
    for _ in range(int(rng.integers(5, 45))):
        x, y = rng.uniform(-half + 1, half - 1, 2)
        gm.seed_disk(x, y, float(rng.uniform(0.2, 0.9)))
    if rng.random() < 0.3:                   # sometimes the goal is walled in
        for a in np.linspace(0, 2 * np.pi, 20, endpoint=False):
            gm.seed_disk(half - 2 + 1.9 * np.cos(a), half - 2 + 1.9 * np.sin(a), 0.45)
    start = tuple(float(v) for v in rng.uniform(-half + 1.2, -half + 3, 2))
    return gm, start, (half - 2.0, half - 2.0)


def _path_error(path, bl, gm, start, goal):
    if path[0] != start:
        return 'does not start at the start'
    if not bl[gm.to_cell(*goal)] and path[-1] != goal:
        return 'does not end on the goal'
    flat = G.flat_cells(bl)
    for k in range(len(path) - 1):
        a, b = path[k], path[k + 1]
        if (k == 0 and bl[gm.to_cell(*a)]) or (k == len(path) - 2 and bl[gm.to_cell(*b)]):
            continue                         # leaving / reaching a pad inside an inflated obstacle
        if not G.segment_cells_free(flat, gm.n, gm.n, gm.origin, 1.0 / gm.res, *a, *b):
            return f'segment {k} crosses an obstacle'
    return None


def check_planners(n_maps=60):
    from .agent import PLAN_INFLATION, INFLATION
    rng = np.random.default_rng(2026)
    bad_path = astar_wrong = rrt_ghost = rrt_missed = reachable = maps = replay = 0
    for n in range(n_maps):
        gm, start, goal = _random_map(rng)
        infl = PLAN_INFLATION if rng.random() < 0.5 else INFLATION
        bl = gm.inflated(infl)
        s, g = G.nearest_free(bl, gm.to_cell(*start)), G.nearest_free(bl, gm.to_cell(*goal))
        if s is None or g is None:
            continue
        maps += 1
        truth = _flood_connected(bl, s, g)
        reachable += truth
        pa = G.plan_path(gm, start, goal, infl, planner='astar')
        pr = G.plan_path(gm, start, goal, infl, planner='rrtstar', rng=np.random.default_rng(n))
        astar_wrong += (pa is not None) != truth
        rrt_ghost += pr is not None and not truth
        rrt_missed += pr is None and truth
        bad_path += sum(1 for p in (pa, pr) if p is not None and _path_error(p, bl, gm, start, goal))
        if n < 10:
            replay += pr != G.plan_path(gm, start, goal, infl, planner='rrtstar', rng=np.random.default_rng(n))
    return [('every planned route is collision-free and runs from start to goal', bad_path == 0,
             f'{maps} random maps, {reachable} with a route'),
            ('A* finds a route exactly when one exists', astar_wrong == 0, ''),
            ('RRT* never returns a route to an unreachable goal', rrt_ghost == 0,
             f'within its sample budget it missed {rrt_missed} of {reachable} existing routes'),
            ('RRT* replays exactly for the same random stream', replay == 0, '')] + check_planner_process()


def check_planner_process():
    """The ROS drone node computes its routes in a separate process (onboard_planner.ProcessPlanner); the
    headless twin computes them inline. Same map, start, goal and seed must give the same route."""
    from .agent import PLAN_INFLATION, INFLATION
    from .onboard_planner import InlinePlanner, ProcessPlanner
    rng = np.random.default_rng(77)
    jobs = []
    while len(jobs) < 4:
        gm, start, goal = _random_map(rng)
        jobs.append((gm, start, goal, ('rrtstar', 'astar')[len(jobs) % 2], int(rng.integers(1 << 62))))
    inline, proc = InlinePlanner(), ProcessPlanner()
    same = 0
    try:
        for gm, start, goal, planner, seed in jobs:
            inline.submit(gm, start, goal, planner, seed, (PLAN_INFLATION, INFLATION))
            a = inline.poll()[0]
            proc.submit(gm, start, goal, planner, seed, (PLAN_INFLATION, INFLATION))
            while proc.busy():
                r = proc.poll()
                if r is None:
                    import time
                    time.sleep(0.01)
            same += a == r[0]
    finally:
        proc.close()
    return [('the drone node\'s planner process returns exactly the inline route (same map and seed)',
             same == len(jobs), f'{same} of {len(jobs)} routes (RRT* and A*)')]


# ---------------------------------------------------------------------------
# turn trading
# ---------------------------------------------------------------------------
def check_ttc(n_clusters=300):
    rng = np.random.default_rng(5)
    wrong = cyc_wrong = 0
    for _ in range(n_clusters):
        ids = [int(i) for i in rng.permutation(np.arange(1, 6))[:int(rng.integers(2, 6))]]
        table = {}

        def cost(i, ahead):
            if not ahead:
                return 0.0
            return table.setdefault((i, ahead), float(rng.choice([0.0, 0.0, rng.uniform(50, 900)])))

        def total(order):
            return sum(cost(i, tuple(sorted(order[:k]))) for k, i in enumerate(order))

        best, e_best, e_endow = ttc.best_order(tuple(ids), cost)
        wrong += abs(min(total(p) for p in itertools.permutations(ids)) - e_best) > 1e-9 or abs(total(ids) - e_endow) > 1e-9
        cycles = ttc.cycles_between(list(ids), list(best))
        moved = {i for i in ids if ids.index(i) != list(best).index(i)}
        cyc_wrong += {i for c in cycles for i in c} != moved or any(len(c) < 2 for c in cycles)
    first = sum(min((1, 2), key=lambda i: ttc.tie_rank(r, i)) == 1 for r in range(10000))
    return [('turn trading picks the lowest-energy order of all orders', wrong == 0, f'{n_clusters} random clusters'),
            ('its trading cycles turn the old order into the new one', cyc_wrong == 0, ''),
            ('ties favour no drone number', 4700 < first < 5300, f'drone1 first in {first} of 10000 ties with drone2')]


# ---------------------------------------------------------------------------
# missions (headless twin)
# ---------------------------------------------------------------------------
def check_mission():
    from . import layouts
    from . import metrics as M
    from .mission import MissionCore
    from .plant import KinematicPlant, run_headless
    planner0, give_way0 = C.PLANNER, C.GIVE_WAY
    priorities = [d.priority for d in C.DRONES]
    out = []
    try:
        C.PLANNER, C.GIVE_WAY = C.DEFAULT_PLANNER, C.DEFAULT_GIVE_WAY
        layouts.apply(0)
        core = MissionCore('cam3_sensor_swarm', seed=0)
        plant = KinematicPlant(core.world)
        agents = list(core.agents.values())
        own = all(len({id(getattr(a, name)) for a in agents}) == len(agents)
                  for name in ('map', 'ekf', 'plan_rng', 'trader', 'sensors', 'battery'))
        out.append(('every drone has its own map, EKF, planner stream, turn trader, sensors and battery', own, ''))
        dt = 1.0 / C.CONTROL_RATE
        ref_at, pad_energy, pad_plans, k = {}, None, None, 0
        t_end = C.PREVIEW_TIME + C.SCAN_TIME + 8.0
        while k * dt < t_end:
            t = k * dt
            cmds = core.step(t, plant.truth())
            plant.step(cmds, dt)
            k += 1
            for a in agents:
                if a.ref is not None and a.id not in ref_at:
                    ref_at[a.id] = (t, a.state)
            if all(a.state == 'PREVIEW' for a in agents):
                pad_energy = max(a.battery.used_j for a in agents)
                pad_plans = sum(len(a.plan_ms) for a in agents)
        out.append(('every drone plans its reference route (A*) on its pad, before take-off',
                    len(ref_at) == len(agents) and all(st == 'PREVIEW' and t < C.PREVIEW_TIME for t, st in ref_at.values()),
                    f'at t = {max(t for t, _ in ref_at.values()):.2f} s' if ref_at else ''))
        out.append(('the reference route costs no drone energy (none is drawn on the pad)', pad_energy == 0.0,
                    f'{pad_energy} J'))
        out.append(('the reference plan is not counted as in-flight planning',
                    all(a.ref_ms is not None for a in agents) and pad_plans == 0, ''))
        out.append((f'the flown route is planned in flight ({G.PLANNER_NAMES[C.PLANNER]}), after take-off',
                    all(a.path is not None and a.m['cruise_t'] is not None and a.m['cruise_t'] > C.PREVIEW_TIME
                        for a in agents), ''))
        t_last = (k - 1) * dt
        rolling = [(len(a.plan_ms), int((t_last - a.m['cruise_t']) / C.REPLAN_PERIOD + 1e-9) + 1, a.plan_version)
                   for a in agents if a.m['cruise_t'] is not None]
        out.append((f'in flight every drone plans again every {C.REPLAN_PERIOD:g} s (rolling re-planning) and '
                    'switches routes only when it gains',
                    len(rolling) == len(agents) and all(n >= need and v <= n for n, need, v in rolling),
                    'routes computed / switched to per drone: ' + ', '.join(f'{n}/{v}' for n, _, v in rolling)))

        def flown(key, seed):
            rep, _ = run_headless(key, seed=seed)
            return rep, {k_: v for k_, v in M.extract(rep).items() if k_ not in SKIP_WALL_CLOCK}

        _, a1 = flown('all', 3)
        _, a2 = flown('all', 3)
        out.append(('a mission replays exactly (same seed, same numbers)', a1 == a2, ''))
        base = {key: flown(key, 1)[1] for key in ('cam3_sensor_swarm', 'cam3_sensor')}
        for d, p in zip(C.DRONES, (3, 0, 4, 1, 2)):
            d.priority = p
        shuffled = {key: flown(key, 1)[1] for key in base}
        for d, p in zip(C.DRONES, priorities):
            d.priority = p
        out.append(('turn trading: the drones\' priority numbers change nothing (no drone has priority)',
                    shuffled['cam3_sensor_swarm'] == base['cam3_sensor_swarm'], ''))
        out.append(('no radio: the drones\' priority numbers change nothing', shuffled['cam3_sensor'] == base['cam3_sensor'], ''))
        C.GIVE_WAY = 'priority'
        rep_p, m_p = flown('cam3_sensor_swarm', 1)
        C.GIVE_WAY = C.DEFAULT_GIVE_WAY
        out.append(('the fixed-priority give-way flies as a separate case',
                    rep_p['give_way'] == 'priority' and m_p != base['cam3_sensor_swarm'],
                    f"fleet energy {m_p['fleet_energy_kj']:.2f} kJ vs {base['cam3_sensor_swarm']['fleet_energy_kj']:.2f} kJ with turn trading"))
        C.PLANNER = 'astar'
        rep_a, m_a = flown('cam3_sensor', 1)
        C.PLANNER = C.DEFAULT_PLANNER
        out.append(('the A* case flies as a separate case, scored against the same kind of reference route',
                    rep_a['planner'] == 'astar' and rep_a['fleet_goals_reached'] == len(C.DRONES)
                    and all(rep_a['reference_paths'].get(str(d.id)) for d in C.DRONES),
                    f"fleet energy {m_a['fleet_energy_kj']:.2f} kJ vs {base['cam3_sensor']['fleet_energy_kj']:.2f} kJ with RRT*"))
    finally:
        C.PLANNER, C.GIVE_WAY = planner0, give_way0
        for d, p in zip(C.DRONES, priorities):
            d.priority = p
    return out


CHECKS = [('geometry', check_geometry), ('planners', check_planners), ('ttc', check_ttc), ('mission', check_mission)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--quick', action='store_true', help='skip the mission checks (they fly 9 missions)')
    a = ap.parse_args(argv)
    failed = 0
    print(f'swarm_intel self-test (package {C.MODEL_VERSION})')
    for name, fn in CHECKS:
        if a.quick and name == 'mission':
            continue
        print(f'\n {name}', flush=True)
        for what, ok, detail in fn():
            failed += not ok
            print(f"   {'PASS' if ok else 'FAIL'}  {what}" + (f'   [{detail}]' if detail else ''), flush=True)
    print('\n' + ('All checks passed.' if not failed else f'{failed} CHECK(S) FAILED.'))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
