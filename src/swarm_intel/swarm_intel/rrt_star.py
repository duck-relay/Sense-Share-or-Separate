"""RRT* path planner on the inflated occupancy grid (the drones' planner).

RRT* (Karaman and Frazzoli, 2011) grows a tree of collision-free straight segments by random
sampling. Each new node is connected to the neighbour that gives it the cheapest path from the
root, and the neighbours are then rewired through the new node when that shortens their path, so
the tree converges towards the shortest path as it grows.

The drones grow one tree from their position. Once a route to the goal exists, samples are drawn
only from the ellipse of points that can still shorten it (informed sampling, Gammell et al.,
2014), so the rest of the budget goes into shortening it.

bidirectional=True gives the two-tree form (B-RRT* / RRT*-Connect: Jordan and Perez, 2013; Klemm
et al., 2015): one tree from the start and one from the goal, taking turns; after each new node
the other tree is extended greedily towards it until the two link (a route) or it hits an
obstacle. It was tested as an upgrade for large maps (planner_bench --two-trees) but was slower
than the one-tree form with the same sample budget in this forest (the greedy connection adds many
nodes per sample), so the drones do not use it.

Samples are drawn uniformly over the free space (10 % at the other tree's root): a uniform point
of the map, redrawn while it falls on a blocked cell. Collision checks use the same inflated
occupancy grid as A* and the rest of the mission: a segment is free when every grid cell it
touches is free (grid_planner.segment_cells_free). The caller prunes the returned path with
line-of-sight shortcuts (grid_planner.smooth), as it does for A*.

Normally nothing here visits every cell of the map: the work grows with the number of samples
and the length of the segments checked, not with the number of grid cells. The sample budget was
set on the 20 x 20 m field of the study and grows in proportion to the area of the map
(BUDGET_AREA), so a larger field gets the same sample density.

Sampling alone cannot tell that a goal is unreachable. plan_path first rules out a start or goal
walled into a small pocket (a bounded flood fill, for both planners). If the regular budget ends
without a route, RRT* checks once whether start and goal are connected at all
(grid_planner.cells_connected, two flood fills that stop when they meet): if not, it reports no
route at once; if they are, it keeps sampling up to max_iterations (also scaled with the area).

Planning is deterministic for a given random generator. Every drone has its own planning
stream, seeded from the mission, the forest and the drone, so a mission replays identically,
every configuration of a mission starts from the same stream, and the sensor noise does not
depend on how often a drone plans.

Pure Python on purpose: the trees stay small (thousands of nodes, found through a spatial hash)
and the inner loop is dominated by grid look-ups, which are fastest on a flat bytes object.
"""
import math

import numpy as np

from .grid_planner import segment_cells_free, cells_connected

BUDGET_AREA = 400.0       # m^2: the field (20 x 20 m) the sample budget below was set on

DEFAULTS = {
    'step': 1.5,          # m: longest new edge (steering distance)
    'iterations': 1000,   # samples per plan per BUDGET_AREA of map (more until a path is found)
    'max_iterations': 6000,  # per BUDGET_AREA: give up (no path) after this many samples
    'goal_bias': 0.10,    # share of samples drawn at the other tree's root (the goal / the start)
    'gamma_factor': 1.1,  # rewiring radius gamma = this x the bound of Karaman and Frazzoli (2011)
    'bidirectional': False,  # True: two trees (start and goal) with greedy connection (not used)
}


class _Grid:
    """Flat, read-only view of an inflated occupancy grid for fast look-ups."""

    def __init__(self, blocked, origin, res):
        self.n0, self.n1 = blocked.shape
        self.cells = blocked.astype('uint8').tobytes()
        self.origin = origin
        self.res = res
        self.inv = 1.0 / res

    def blocked(self, x, y):
        i = int(math.floor((x - self.origin) * self.inv))
        j = int(math.floor((y - self.origin) * self.inv))
        if i < 0 or j < 0 or i >= self.n0 or j >= self.n1:
            return True
        return self.cells[i * self.n1 + j] != 0

    def segment_free(self, ax, ay, bx, by):
        return segment_cells_free(self.cells, self.n0, self.n1, self.origin, self.inv, ax, ay, bx, by)


class _Tree:
    """One RRT* tree: nodes, parents, costs from the root, children, and a spatial hash with cells
    of one steering step (nearest / near queries)."""

    def __init__(self, x, y, lo, step, span):
        self.X, self.Y, self.parent, self.cost, self.children = [x], [y], [-1], [0.0], [[]]
        self.lo, self.step = lo, step
        self.nb = int(span / step) + 1
        self.bucket = {self.key(x, y): [0]}

    def key(self, x, y):
        return (int((x - self.lo) / self.step), int((y - self.lo) / self.step))

    def nearest(self, x, y):
        X, Y, bucket, step = self.X, self.Y, self.bucket, self.step
        bx, by = self.key(x, y)
        best, bd = -1, math.inf
        for r in range(self.nb + 1):
            for i in range(bx - r, bx + r + 1):
                edge = i == bx - r or i == bx + r
                for j in ((range(by - r, by + r + 1)) if edge else (by - r, by + r)):
                    for q in bucket.get((i, j), ()):
                        d = (X[q] - x) ** 2 + (Y[q] - y) ** 2
                        if d < bd:
                            bd, best = d, q
            if best >= 0 and bd <= (r * step) ** 2:
                break
        return best

    def near(self, x, y, rad):
        X, Y = self.X, self.Y
        bx, by = self.key(x, y)
        r2 = rad * rad
        out = []
        for i in (bx - 1, bx, bx + 1):
            for j in (by - 1, by, by + 1):
                for q in self.bucket.get((i, j), ()):
                    d = (X[q] - x) ** 2 + (Y[q] - y) ** 2
                    if d <= r2:
                        out.append((q, math.sqrt(d)))
        return out

    def extend(self, grid, qx, qy, gamma):
        """Steer from the nearest node towards (qx, qy) by at most one step; insert the new node with
        the cheapest collision-free parent among its neighbours and rewire them through it.
        Returns (new node or -1, whether it lies on (qx, qy), whether any cost changed)."""
        X, Y, cost = self.X, self.Y, self.cost
        q0 = self.nearest(qx, qy)
        dx, dy = qx - X[q0], qy - Y[q0]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return -1, True, False
        reached = d <= self.step
        if not reached:
            qx, qy = X[q0] + dx / d * self.step, Y[q0] + dy / d * self.step
            d = self.step
        if not grid.segment_free(X[q0], Y[q0], qx, qy):
            return -1, False, False
        n = len(X) + 1
        rad = min(self.step, gamma * math.sqrt(math.log(n) / n))
        cand = self.near(qx, qy, rad)
        cand.sort(key=lambda c: cost[c[0]] + c[1])
        par, pc = q0, cost[q0] + d
        free = {q0: True}
        for q, dq in cand:
            c = cost[q] + dq
            if c >= pc:
                break
            ok = grid.segment_free(X[q], Y[q], qx, qy)
            free[q] = ok
            if ok:
                par, pc = q, c
                break
        new = len(X)
        X.append(qx)
        Y.append(qy)
        self.parent.append(par)
        cost.append(pc)
        self.children.append([])
        self.children[par].append(new)
        self.bucket.setdefault(self.key(qx, qy), []).append(new)
        changed = False
        for q, dq in cand:                       # rewire the neighbours through the new node
            if q == par:
                continue
            c = pc + dq
            if c < cost[q] - 1e-9:
                ok = free.get(q)
                if ok is None:
                    ok = grid.segment_free(qx, qy, X[q], Y[q])
                if ok:
                    self.children[self.parent[q]].remove(q)
                    self.parent[q] = new
                    self.children[new].append(q)
                    delta = c - cost[q]
                    stack = [q]
                    while stack:                  # the whole subtree gets shorter by delta
                        r = stack.pop()
                        cost[r] += delta
                        stack.extend(self.children[r])
                    changed = True
        return new, reached, changed

    def branch(self, q):
        """Node q and its ancestors up to the root, as points (q first)."""
        out = []
        while q >= 0:
            out.append((self.X[q], self.Y[q]))
            q = self.parent[q]
        return out


def rrt_star(blocked, origin, res, start, goal, rng, params=None):
    """Shortest-path search from start to goal (world coordinates) with RRT*.

    blocked: 2-D bool array of the inflated grid (True = not allowed); origin, res: its
    lower-left corner and cell size. rng: a numpy Generator (the drone's planning stream).
    Returns the path as a list of (x, y) from start to goal, or None when no path was found
    within the sample budget.
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    grid = _Grid(blocked, origin, res)
    sx, sy = float(start[0]), float(start[1])
    gx, gy = float(goal[0]), float(goal[1])
    if grid.blocked(sx, sy) or grid.blocked(gx, gy):
        return None
    if grid.segment_free(sx, sy, gx, gy):          # nothing in the way: the straight line is optimal
        return [(sx, sy), (gx, gy)]

    step = p['step']
    lo = origin
    side0, side1 = grid.n0 * res, grid.n1 * res
    n_free = int(np.count_nonzero(~blocked))
    # rewiring radius: min(step, gamma sqrt(ln n / n)) with gamma above the bound for asymptotic
    # optimality in the plane, 2 (1 + 1/2)^(1/2) (free area / pi)^(1/2)
    gamma = p['gamma_factor'] * 2.0 * math.sqrt(1.5) * math.sqrt(n_free * res * res / math.pi)
    scale = max(1.0, side0 * side1 / BUDGET_AREA)
    budget = int(round(p['iterations'] * scale))
    limit = int(round(p['max_iterations'] * scale))
    two = bool(p['bidirectional'])
    trees = [_Tree(sx, sy, lo, step, max(side0, side1))]
    if two:
        trees.append(_Tree(gx, gy, lo, step, max(side0, side1)))
    # routes found: (node of the start tree, node of the goal tree or -1 for the goal itself) -> the
    # length of the free straight link between them
    links = {}
    best_cost, best = math.inf, None
    c_min = math.hypot(gx - sx, gy - sy)
    cx, cy = 0.5 * (sx + gx), 0.5 * (sy + gy)
    ca, sa = (gx - sx) / c_min, (gy - sy) / c_min

    def update_best():
        nonlocal best_cost, best
        best_cost, best = math.inf, None
        c0 = trees[0].cost
        c1 = trees[1].cost if two else None
        for (a, b), d in links.items():
            c = c0[a] + d + (c1[b] if b >= 0 else 0.0)
            if c < best_cost:
                best_cost, best = c, (a, b)

    def try_link(t, node):
        """Link the new node `node` of tree t to the other tree (or, one tree: to the goal) when a
        free straight link of at most one step exists; returns True if a route was added."""
        T = trees[t]
        x, y = T.X[node], T.Y[node]
        if not two:
            dg = math.hypot(gx - x, gy - y)
            if dg <= step and grid.segment_free(x, y, gx, gy):
                links[(node, -1)] = dg
                return True
            return False
        O = trees[1 - t]
        cand = O.near(x, y, step)
        cand.sort(key=lambda c: O.cost[c[0]] + c[1])
        for q, dq in cand:
            if grid.segment_free(x, y, O.X[q], O.Y[q]):
                links[(node, q) if t == 0 else (q, node)] = dq
                return True
        return False

    it = 0
    side = 0
    draws = []
    checked = False
    while True:
        if best is not None and it >= budget:
            break
        if it >= limit:
            return None
        if not checked and it >= budget:           # the regular budget is spent without a route:
            checked = True                         # is there one at all?
            inv = grid.inv
            ca_, cb_ = (math.floor((sx - lo) * inv) * grid.n1 + math.floor((sy - lo) * inv),
                        math.floor((gx - lo) * inv) * grid.n1 + math.floor((gy - lo) * inv))
            if not cells_connected(grid.cells, grid.n0, grid.n1, ca_, cb_):
                return None
        if not draws:
            draws = rng.random((256, 3)).tolist()
        u0, u1, u2 = draws.pop()
        it += 1
        A = trees[side]
        # sample: the other end, the informed ellipse once a route exists, or the free space
        if u0 < p['goal_bias']:
            qx, qy = (gx, gy) if side == 0 else (sx, sy)
        elif best is not None:
            a = 0.5 * best_cost
            b = 0.5 * math.sqrt(max(best_cost * best_cost - c_min * c_min, 0.0))
            rr = math.sqrt(u1)
            ph = 2.0 * math.pi * u2
            ex, ey = a * rr * math.cos(ph), b * rr * math.sin(ph)
            qx, qy = cx + ca * ex - sa * ey, cy + sa * ex + ca * ey
            if grid.blocked(qx, qy):
                if two:
                    side = 1 - side
                continue
        else:
            qx, qy = lo + u1 * side0, lo + u2 * side1
            if grid.blocked(qx, qy):             # redrawn: the budget counts samples of the free space
                it -= 1
                continue
        new, _, changed = A.extend(grid, qx, qy, gamma)
        if new >= 0:
            changed = try_link(side, new) or changed
            if two:
                # connect: extend the other tree greedily towards the new node until it links or
                # is blocked
                O = trees[1 - side]
                tx, ty = A.X[new], A.Y[new]
                while True:
                    m, reached, ch = O.extend(grid, tx, ty, gamma)
                    changed = changed or ch
                    if m < 0:
                        break
                    if try_link(1 - side, m):
                        changed = True
                        break
                    if reached:
                        break
        if changed and links:
            update_best()
        if two:
            side = 1 - side

    a, b = best
    path = trees[0].branch(a)[::-1]                # start ... a
    if two:
        path += trees[1].branch(b)                 # b ... goal
    else:
        path.append((gx, gy))
    return path
