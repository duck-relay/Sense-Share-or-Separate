"""Log-odds occupancy grid and path planning on its inflated cells.

The same GridMap class is used as
  * the CENTRAL shared map in configurations with swarm (every drone writes to it)
  * a PRIVATE map per drone in configurations without swarm.

Planners (plan_path): RRT* (rrt_star.py) is the drones' planner. A* on the 8-connected grid is
kept for the planner comparison study (--planner astar) and computes the full-knowledge ideal
path that path deviation is scored against. Both paths are pruned with the same line-of-sight
shortcuts (smooth).
"""
import heapq
import math
import numpy as np

from . import config as C

L_OCC = 0.85
L_FREE = -0.40
L_MIN = -2.2
L_MAX = 3.5
L_THRESH = 1.2          # >= 2 consistent hits before a cell counts as occupied


def known_unknowns(gm):
    """How many of the real unknown pines are present in a drone's map (scoring only)."""
    occ = (gm.logodds >= L_THRESH) & ~gm.static
    n = 0
    for (x, y) in C.UNKNOWN_PINES:
        m = (gm.cx - x) ** 2 + (gm.cy - y) ** 2 <= (C.PINE_PLAN_R + 0.35) ** 2
        n += int(np.any(occ & m))
    return n


def disk_offsets(r_cells):
    out = []
    for dx in range(-r_cells, r_cells + 1):
        for dy in range(-r_cells, r_cells + 1):
            if dx * dx + dy * dy <= r_cells * r_cells + 1e-9:
                out.append((dx, dy))
    return out


def dilate(mask, r_cells):
    if r_cells <= 0:
        return mask.copy()
    n0, n1 = mask.shape
    out = mask.copy()
    for dx, dy in disk_offsets(r_cells):
        if dx == 0 and dy == 0:
            continue
        xs0, xs1 = max(0, dx), n0 + min(0, dx)
        ys0, ys1 = max(0, dy), n1 + min(0, dy)
        out[xs0:xs1, ys0:ys1] |= mask[xs0 - dx:xs1 - dx, ys0 - dy:ys1 - dy]
    return out


class GridMap:
    def __init__(self, half=C.ARENA_HALF, res=C.GRID_RES):
        self.res = res
        self.half = half
        self.n = int(round(2 * half / res))
        self.origin = -half
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)
        self.static = np.zeros((self.n, self.n), dtype=bool)
        self.version = 0
        self._infl_cache = {}
        # keep-in band along the arena edge
        k = int(math.ceil(C.BOUNDARY_KEEP_IN / res))
        self.border = np.zeros((self.n, self.n), dtype=bool)
        self.border[:k, :] = True
        self.border[-k:, :] = True
        self.border[:, :k] = True
        self.border[:, -k:] = True
        xs = self.origin + (np.arange(self.n) + 0.5) * res
        self.cx, self.cy = np.meshgrid(xs, xs, indexing='ij')

    # -- coordinates -------------------------------------------------------
    def to_cell(self, x, y):
        i = int(math.floor((x - self.origin) / self.res))
        j = int(math.floor((y - self.origin) / self.res))
        return min(max(i, 0), self.n - 1), min(max(j, 0), self.n - 1)

    def to_world(self, i, j):
        return self.origin + (i + 0.5) * self.res, self.origin + (j + 0.5) * self.res

    # -- map building -------------------------------------------------------
    def seed_disk(self, x, y, r):
        m = (self.cx - x) ** 2 + (self.cy - y) ** 2 <= (r + 0.5 * self.res) ** 2
        self.static |= m
        self._bump()

    def _bump(self):
        self.version += 1
        self._infl_cache.clear()

    def occupied(self):
        return self.static | (self.logodds >= L_THRESH)

    def integrate_scan(self, origin, ends, hits):
        """Bresenham ray update. ends: (R,2) end points, hits: bool per ray."""
        i0, j0 = self.to_cell(*origin)
        before = self.logodds >= L_THRESH
        for (ex, ey), hit in zip(ends, hits):
            i1, j1 = self.to_cell(ex, ey)
            cells = _bresenham(i0, j0, i1, j1)
            for (i, j) in cells[:-1]:
                if not self.static[i, j]:
                    self.logodds[i, j] = max(L_MIN, self.logodds[i, j] + L_FREE)
            i, j = cells[-1]
            if hit:
                self.logodds[i, j] = min(L_MAX, self.logodds[i, j] + L_OCC)
            elif not self.static[i, j]:
                self.logodds[i, j] = max(L_MIN, self.logodds[i, j] + L_FREE)
        after = self.logodds >= L_THRESH
        changed = before != after
        if np.any(changed):
            newly = after & ~before & ~self.static
            self._bump()
            return int(np.sum(newly))
        return 0

    def inflated(self, radius):
        key = round(radius, 3)
        if key not in self._infl_cache:
            r_cells = int(math.ceil(radius / self.res))
            self._infl_cache[key] = dilate(self.occupied(), r_cells) | self.border
        return self._infl_cache[key]

    def snapshot(self):
        """A frozen copy for a route computed next to the flight loop (onboard_planner.ProcessPlanner):
        the drone's own map keeps changing meanwhile. Inflated grids already computed come along
        (they are replaced, never changed in place)."""
        s = object.__new__(GridMap)
        s.__dict__.update(self.__dict__)
        s.logodds = self.logodds.copy()
        s.static = self.static.copy()
        s._infl_cache = dict(self._infl_cache)
        return s

    def obstacles_dilated(self, radius):
        """Occupied cells grown by `radius` (without the keep-in border) - for line-of-sight checks."""
        key = ('obs', round(radius, 3))
        if key not in self._infl_cache:
            self._infl_cache[key] = dilate(self.occupied(), int(math.ceil(radius / self.res)))
        return self._infl_cache[key]


POCKET_CELLS = 2000        # plan_path's quick check visits at most this many cells (about 1 ms)


def flat_cells(blocked):
    """The blocked grid as a flat read-only byte view (row i, column j at i * n1 + j), without a copy."""
    return np.ascontiguousarray(blocked, dtype=bool).view(np.uint8).reshape(-1).data


def cells_connected(cells, n0, n1, a, b, max_cells=None):
    """Are the free cells a and b (flat indices) joined by free cells? 4-neighbourhood: the connectivity
    of a path that never slips between two diagonally touching blocked cells (what A* without corner
    cutting and RRT*'s segment check both allow).
    Two flood fills, one from each cell, grown in turn (the smaller front first): True when they meet,
    False when one of them cannot grow any more. A cell walled into a small pocket is therefore
    recognised after visiting only the pocket, whatever the size of the map.
    max_cells: give up (None = undecided) once this many cells have been visited."""
    if a == b:
        return True
    mark = {a: 1, b: 2}
    fronts = [[a], [b]]
    while fronts[0] and fronts[1]:
        k = 0 if len(fronts[0]) <= len(fronts[1]) else 1
        lab = k + 1
        new = []
        for c in fronts[k]:
            i, j = divmod(c, n1)
            for q, inside in ((c - n1, i > 0), (c + n1, i < n0 - 1), (c - 1, j > 0), (c + 1, j < n1 - 1)):
                if inside and not cells[q]:
                    m = mark.get(q)
                    if m is None:
                        mark[q] = lab
                        new.append(q)
                    elif m != lab:
                        return True
        fronts[k] = new
        if max_cells is not None and len(mark) > max_cells:
            return None
    return False


def _bresenham(x0, y0, x1, y1):
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------
_NEIGH = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def nearest_free(blocked, cell, max_r=12):
    i0, j0 = cell
    if not blocked[i0, j0]:
        return cell
    n0, n1 = blocked.shape
    best = None
    for r in range(1, max_r + 1):
        for di in range(-r, r + 1):
            for dj in (-r, r) if abs(di) != r else range(-r, r + 1):
                i, j = i0 + di, j0 + dj
                if 0 <= i < n0 and 0 <= j < n1 and not blocked[i, j]:
                    d = di * di + dj * dj
                    if best is None or d < best[0]:
                        best = (d, (i, j))
        if best is not None:
            return best[1]
    return None


def astar(blocked, start, goal):
    n0, n1 = blocked.shape
    if blocked[goal] or blocked[start]:
        return None
    gx, gy = goal

    def h(i, j):
        dx, dy = abs(i - gx), abs(j - gy)
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    g = {start: 0.0}
    came = {}
    openq = [(h(*start), 0.0, start)]
    closed = set()
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        closed.add(cur)
        ci, cj = cur
        for di, dj, cost in _NEIGH:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < n0 and 0 <= nj < n1) or blocked[ni, nj]:
                continue
            if di != 0 and dj != 0 and (blocked[ci + di, cj] or blocked[ci, cj + dj]):
                continue   # no corner cutting
            ng = gc + cost
            nb = (ni, nj)
            if ng < g.get(nb, 1e18):
                g[nb] = ng
                came[nb] = cur
                heapq.heappush(openq, (ng + h(ni, nj), ng, nb))
    return None


def segment_cells_free(cells, n0, n1, origin, inv_res, ax, ay, bx, by):
    """True if every grid cell the segment a->b touches is free (exact grid traversal after
    Amanatides and Woo): the cells of its two end points (a point on a grid line belongs to the
    cell above / right of it, as in GridMap.to_cell), every cell it passes through, and, where it
    passes exactly through a cell corner on the way, both cells beside the corner - so no segment
    slips between two diagonally touching blocked cells. The result is the same for a->b and b->a.
    cells: the blocked grid as a flat sequence (row i, column j at i * n1 + j); points outside the
    grid count as blocked."""
    x, y = (ax - origin) * inv_res, (ay - origin) * inv_res
    ex, ey = (bx - origin) * inv_res, (by - origin) * inv_res
    i, j = int(math.floor(x)), int(math.floor(y))
    ie, je = int(math.floor(ex)), int(math.floor(ey))
    if not (0 <= i < n0 and 0 <= j < n1 and 0 <= ie < n0 and 0 <= je < n1):
        return False
    if cells[i * n1 + j]:
        return False
    dx, dy = ex - x, ey - y
    si = 1 if dx > 0 else -1
    sj = 1 if dy > 0 else -1
    inf = float('inf')
    tdx = abs(1.0 / dx) if dx != 0 else inf
    tdy = abs(1.0 / dy) if dy != 0 else inf
    # tx, ty: how far along the segment (0 at a, 1 at b) it meets the next grid line in x and in y
    tx = ((i + 1 - x) if dx > 0 else (x - i)) * tdx if dx != 0 else inf
    ty = ((j + 1 - y) if dy > 0 else (y - j)) * tdy if dy != 0 else inf
    while i != ie or j != je:
        more_i, more_j = i != ie, j != je
        if more_i and more_j and abs(tx - ty) <= 1e-12:    # meets a cell corner
            # on the way (not at an end point, which only touches the corner): both cells beside it
            if 1e-12 < tx < 1.0 - 1e-12 and (cells[(i + si) * n1 + j] or cells[i * n1 + j + sj]):
                return False
            tx += tdx
            ty += tdy
            i += si
            j += sj
        elif more_i and (not more_j or tx < ty):
            tx += tdx
            i += si
        else:
            ty += tdy
            j += sj
        if cells[i * n1 + j]:
            return False
    return True


def line_free(blocked, gm, a, b):
    """True if the straight segment a->b (world coords) touches only free cells."""
    return segment_cells_free(blocked.ravel(), gm.n, gm.n, gm.origin, 1.0 / gm.res,
                              float(a[0]), float(a[1]), float(b[0]), float(b[1]))


def smooth(blocked, gm, pts):
    """Greedy string pulling: keep only the vertices needed for line of sight."""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not line_free(blocked, gm, pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


PLANNERS = ('rrtstar', 'astar')
PLANNER_NAMES = {'rrtstar': 'RRT*', 'astar': 'A*'}


def plan_path(gm, start_xy, goal_xy, inflation, blocked=None, planner=None, rng=None):
    """Plan a smoothed collision-free polyline in world coordinates.

    planner: 'rrtstar' (needs rng, the drone's planning stream) or 'astar'; default config.PLANNER.
    Returns a list of (x, y) or None.  The first point is the (possibly
    snapped) start, the last point is exactly the goal when reachable.
    """
    planner = planner or C.PLANNER
    if planner not in PLANNERS:
        raise ValueError(f'unknown planner {planner!r} (use one of {", ".join(PLANNERS)})')
    if blocked is None:
        blocked = gm.inflated(inflation)
    s = nearest_free(blocked, gm.to_cell(*start_xy))
    g = nearest_free(blocked, gm.to_cell(*goal_xy))
    if s is None or g is None:
        return None
    # quick check, the same for both planners: a start or goal walled into a small pocket has no route
    n0, n1 = blocked.shape
    if cells_connected(flat_cells(blocked), n0, n1, s[0] * n1 + s[1], g[0] * n1 + g[1], POCKET_CELLS) is False:
        return None
    if planner == 'astar':
        cells = astar(blocked, s, g)
        if cells is None:
            return None
        pts = [gm.to_world(i, j) for (i, j) in cells]
    else:
        if rng is None:
            raise ValueError('RRT* needs a random generator (the drone\'s planning stream)')
        from .rrt_star import rrt_star
        # the tree grows from where the drone is and ends on the goal (each moved to the nearest
        # free cell centre when it lies inside an inflated obstacle)
        a = tuple(start_xy) if not blocked[gm.to_cell(*start_xy)] else gm.to_world(*s)
        b = tuple(goal_xy) if not blocked[gm.to_cell(*goal_xy)] else gm.to_world(*g)
        pts = rrt_star(blocked, gm.origin, gm.res, a, b, rng)
        if pts is None:
            return None
    # exact end points (start from where the drone really is, finish on the pad)
    pts[0] = tuple(start_xy) if not blocked[gm.to_cell(*start_xy)] else pts[0]
    if not blocked[gm.to_cell(*goal_xy)]:
        pts[-1] = tuple(goal_xy)
    if pts[0] != tuple(start_xy):
        pts.insert(0, tuple(start_xy))
    pts = smooth(blocked, gm, pts)
    return [tuple(map(float, p)) for p in pts]


def polyline_blocked(blocked, gm, pts, from_s=0.0):
    """Is any remaining part of the polyline inside an inflated obstacle?"""
    from .geometry import cumulative_s, point_at_s
    cum = cumulative_s(pts)
    s = from_s
    total = cum[-1] if len(cum) else 0.0
    while s <= total:
        p = point_at_s(pts, s, cum)
        i, j = gm.to_cell(p[0], p[1])
        if blocked[i, j]:
            return True
        s += gm.res * 0.5
    return False
