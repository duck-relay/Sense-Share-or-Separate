"""Forest layouts for the study.

Layout 0 is the hand-made forest of config.py, the one Gazebo shows. Layouts 1, 2, ...
are random forests generated from their number, so every run of the study sees exactly
the same forests on every computer:

* 20 known landmark trees spread over the arena (ArUco markers, as in layout 0)
* 6 unknown pines: 3 near the straight routes of three different drones (drawn at random), between
  20 % and 70 % of the way along (so those drones have to discover and avoid them), and 3 anywhere
* the same spacing rules as layout 0 (trees >= 1.8 m apart, pines >= 2.5 m, tree-pine
  >= 1.7 m, every take-off / landing pad >= 1.4 m from any tree and >= 2.2 m from any pine)
* accepted only if every drone can reach its goal (a collision-free route exists with every
  obstacle known) and every drone's shortest route is at most 1.6 x its straight line
  (otherwise the next candidate is drawn). This check belongs to the design of the experiment,
  done once per forest; the drones never see these routes (they plan on board, see agent.py).

Use apply(n) before building a MissionCore. It replaces the obstacle lists in config
in place, so every module that reads them (planner, sensors, world model, plots)
sees the new forest.
"""
import math

import numpy as np

from . import config as C

_LAYOUT0 = (list(C.LANDMARK_TREES), list(C.UNKNOWN_PINES))
_CACHE = {}
_current = 0

TREE_GAP = 1.8
PINE_GAP = 2.5
TREE_PINE_GAP = 1.7
PAD_GAP = 1.4                # trees (layout 0: nearest tree 1.41 m from a pad)
PAD_GAP_PINE = 2.2           # pines (layout 0: nearest pine 2.28 m from a pad)
EDGE = 9.2
N_TREES = len(_LAYOUT0[0])
N_PINES = len(_LAYOUT0[1])
N_ROUTE_PINES = 3
MAX_DETOUR = 1.6
ROUTE_SPAN = (0.2, 0.7)       # route pines: this far along a drone's straight route


def _pads():
    return np.array([d.spawn for d in C.DRONES] + [d.goal for d in C.DRONES], float)


def _ok(p, pts, gap):
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) >= gap for q in pts)


def _candidate(rng):
    pads = _pads()
    pines, trees = [], []
    tries = 0
    # pines on the routes of three different drones first, then anywhere
    for k in rng.choice(len(C.DRONES), N_ROUTE_PINES, replace=False):
        d = C.DRONES[int(k)]
        s, g = np.array(d.spawn, float), np.array(d.goal, float)
        u = (g - s) / np.linalg.norm(g - s)
        nrm = np.array([-u[1], u[0]])
        L = float(np.linalg.norm(g - s))
        for _ in range(200):
            tries += 1
            p = s + u * L * rng.uniform(*ROUTE_SPAN) + nrm * rng.uniform(-1.0, 1.0)
            if _ok(p, pines, PINE_GAP) and _ok(p, pads, PAD_GAP_PINE) and max(abs(p)) < EDGE:
                pines.append(p)
                break
        else:
            return None
    while len(pines) < N_PINES:
        tries += 1
        if tries > 5000:
            return None
        p = rng.uniform(-EDGE, EDGE, size=2)
        if _ok(p, pines, PINE_GAP) and _ok(p, pads, PAD_GAP_PINE):
            pines.append(p)
    while len(trees) < N_TREES:
        tries += 1
        if tries > 20000:
            return None
        p = rng.uniform(-EDGE, EDGE, size=2)
        if _ok(p, trees, TREE_GAP) and _ok(p, pines, TREE_PINE_GAP) and _ok(p, pads, PAD_GAP):
            trees.append(p)
    r = lambda pts: [(round(float(x), 2), round(float(y), 2)) for x, y in pts]
    return r(trees), r(pines)


_SCENARIO_CACHE = {}


def scenario_paths():
    """Shortest route of every drone with EVERY obstacle known (A*, pruned), for the current forest.
    Experiment design only: the forest generator checks with it that every drone can reach its goal.
    The drones never see it, and nothing is scored against it."""
    from .grid_planner import GridMap, plan_path
    from .agent import INFLATION, PLAN_INFLATION
    key = (tuple(C.UNKNOWN_PINES), tuple(C.LANDMARK_TREES), tuple((d.spawn, d.goal) for d in C.DRONES))
    if key not in _SCENARIO_CACHE:
        gm = GridMap()
        for (x, y) in C.LANDMARK_TREES:
            gm.seed_disk(x, y, C.TREE_CROWN_R)
        for (x, y) in C.UNKNOWN_PINES:
            gm.seed_disk(x, y, C.PINE_PLAN_R)
        out = {}
        for d in C.DRONES:
            p = (plan_path(gm, d.spawn, d.goal, PLAN_INFLATION, planner='astar')
                 or plan_path(gm, d.spawn, d.goal, INFLATION, planner='astar'))
            if p is None:
                raise RuntimeError(f'{d.name} cannot reach its goal in this forest')
            out[d.id] = p
        _SCENARIO_CACHE[key] = out
    return _SCENARIO_CACHE[key]


def _feasible(trees, pines):
    """Every drone can reach its goal, and no drone's shortest route is absurdly long."""
    from .geometry import cumulative_s
    old = (list(C.LANDMARK_TREES), list(C.UNKNOWN_PINES))
    C.LANDMARK_TREES[:] = trees
    C.UNKNOWN_PINES[:] = pines
    try:
        paths = scenario_paths()
    except RuntimeError:
        return False
    finally:
        C.LANDMARK_TREES[:] = old[0]
        C.UNKNOWN_PINES[:] = old[1]
    for d in C.DRONES:
        straight = math.hypot(d.goal[0] - d.spawn[0], d.goal[1] - d.spawn[1])
        if float(cumulative_s(np.asarray(paths[d.id], float))[-1]) > MAX_DETOUR * straight:
            return False
    return True


def generate(n):
    """(trees, pines) of layout n. Layout 0 is the hand-made forest."""
    n = int(n)
    if n == 0:
        return list(_LAYOUT0[0]), list(_LAYOUT0[1])
    if n not in _CACHE:
        rng = np.random.default_rng(7919 * n + 17)
        for _ in range(200):
            cand = _candidate(rng)
            if cand and _feasible(*cand):
                _CACHE[n] = cand
                break
        else:
            raise RuntimeError(f'could not generate a feasible forest for layout {n}')
    trees, pines = _CACHE[n]
    return list(trees), list(pines)


def apply(n):
    """Make layout n the current forest (config lists replaced in place)."""
    global _current
    trees, pines = generate(n)
    C.LANDMARK_TREES[:] = trees
    C.UNKNOWN_PINES[:] = pines
    _current = int(n)
    return _current


def current():
    return _current
