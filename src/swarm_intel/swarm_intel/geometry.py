"""Geometry helpers: obstacle cross-sections, ray casting, line-of-sight, polylines."""
import math
import numpy as np

from . import config as C


class WorldObstacles:
    """Ground-truth obstacles (known landmark trees + unknown pines).

    Only the simulator / sensor models see this. Drones only get what their
    sensors (or, in swarm mode, their peers) tell them.
    """

    def __init__(self, trees=None, pines=None):
        self.trees = np.array(trees if trees is not None else C.LANDMARK_TREES, dtype=float)
        self.pines = np.array(pines if pines is not None else C.UNKNOWN_PINES, dtype=float)
        self.xy = np.vstack([self.trees, self.pines]) if len(self.pines) else self.trees.copy()
        self.kind = np.array(['tree'] * len(self.trees) + ['pine'] * len(self.pines))
        self._cache = {}

    @staticmethod
    def tree_radius_at(z):
        dz = z - C.TREE_CROWN_Z
        crown = math.sqrt(max(0.0, C.TREE_CROWN_R ** 2 - dz ** 2)) if abs(dz) < C.TREE_CROWN_R else 0.0
        trunk = C.TREE_TRUNK_R if z <= C.TREE_TRUNK_H else 0.0
        return max(crown, trunk)

    @staticmethod
    def pine_radius_at(z):
        for z0, z1, r in C.PINE_TIERS:
            if z0 <= z <= z1:
                return r
        return 0.0

    def radii_matrix(self, zs):
        """Obstacle radius at each height in zs -> array (len(zs), n_obstacles)."""
        zs = np.asarray(zs, float)
        dz = zs - C.TREE_CROWN_Z
        crown = np.sqrt(np.clip(C.TREE_CROWN_R ** 2 - dz ** 2, 0.0, None))
        trunk = np.where(zs <= C.TREE_TRUNK_H, C.TREE_TRUNK_R, 0.0)
        rt = np.maximum(crown, trunk)
        rp = np.zeros_like(zs)
        for z0, z1, r in C.PINE_TIERS:
            rp = np.where((zs >= z0) & (zs <= z1), np.maximum(rp, r), rp)
        return np.hstack([np.repeat(rt[:, None], len(self.trees), axis=1),
                          np.repeat(rp[:, None], len(self.pines), axis=1)])

    @staticmethod
    def tree_radius_max(z0, z1):
        """Largest tree cross-section anywhere in the horizontal slab [z0, z1]."""
        if z1 < 0.0:
            return 0.0
        zc = min(max(C.TREE_CROWN_Z, z0), z1)            # point of the slab closest to the crown centre
        dz = zc - C.TREE_CROWN_Z
        crown = math.sqrt(max(0.0, C.TREE_CROWN_R ** 2 - dz ** 2)) if abs(dz) < C.TREE_CROWN_R else 0.0
        trunk = C.TREE_TRUNK_R if z0 <= C.TREE_TRUNK_H else 0.0
        return max(crown, trunk)

    @staticmethod
    def pine_radius_max(z0, z1):
        """Largest pine cross-section anywhere in the horizontal slab [z0, z1]."""
        return max([r for a, b, r in C.PINE_TIERS if a <= z1 and b >= z0], default=0.0)

    def radii_band(self, z0, z1):
        """Obstacle radii: the widest cross-section of each obstacle in the slab [z0, z1]."""
        key = ('band', round(z0, 2), round(z1, 2))
        if key in self._cache:
            return self._cache[key]
        a, b = key[1], key[2]
        rt = self.tree_radius_max(a, b)
        rp = self.pine_radius_max(a, b)
        r = np.array([rt] * len(self.trees) + [rp] * len(self.pines), dtype=float)
        if len(self._cache) > 400:
            self._cache.clear()
        self._cache[key] = r
        return r

    def contact_gap(self, p):
        """Horizontal gap between the drone's collision shapes and each obstacle (negative = touching):
        the rotor disc (radius DRONE_COLLISION_R, around the body centre) and the feet ring
        (radius DRONE_FEET_R, REST_Z below it) - the same shapes as the Gazebo collision model.
        Returns (gap per obstacle, required centre distance per obstacle; 0 = no obstacle at that height)."""
        r_disc = self.radii_at(p[2])
        zf = p[2] - C.REST_Z
        r_feet = self.radii_band(zf, zf + C.DRONE_FEET_H)
        lim = np.maximum(np.where(r_disc > 0, r_disc + C.DRONE_COLLISION_R, 0.0),
                         np.where(r_feet > 0, r_feet + C.DRONE_FEET_R, 0.0))
        d = np.hypot(self.xy[:, 0] - p[0], self.xy[:, 1] - p[1])
        return d - lim, lim

    def radii_at(self, z, band=0.06):
        """Obstacle radii in the horizontal slice [z-band, z+band] (drone body thickness)."""
        key = round(z, 2)
        if key in self._cache:
            return self._cache[key]
        zs = (key - band, key, key + band)            # computed from the key, so the cache is exact
        rt = max(self.tree_radius_at(zz) for zz in zs)
        rp = max(self.pine_radius_at(zz) for zz in zs)
        r = np.array([rt] * len(self.trees) + [rp] * len(self.pines), dtype=float)
        if len(self._cache) > 400:
            self._cache.clear()
        self._cache[key] = r
        return r


def ray_cast(origin, angles, max_range, cxy, radii):
    """Distance along each ray to the first circle hit (inf if none)."""
    ox, oy = origin
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)          # (R,2)
    m = radii > 1e-6
    if not np.any(m):
        return np.full(len(angles), np.inf)
    c = cxy[m] - np.array([ox, oy])                                  # (K,2)
    r = radii[m]
    b = d @ c.T                                                      # (R,K) projection
    cc = np.sum(c * c, axis=1)                                       # (K,)
    disc = b * b - (cc - r * r)[None, :]
    hit = disc >= 0
    t = b - np.sqrt(np.where(hit, disc, 0.0))
    t = np.where(hit & (t > 0.02) & (t <= max_range), t, np.inf)
    return np.min(t, axis=1)


def segment_blockers(p, q, cxy, radii, exclude=None):
    """Number of circles the 2-D segment p->q passes through."""
    p = np.asarray(p[:2], float)
    q = np.asarray(q[:2], float)
    seg = q - p
    L2 = float(seg @ seg)
    if L2 < 1e-9:
        return 0
    t = np.clip(((cxy - p) @ seg) / L2, 0.0, 1.0)
    closest = p + t[:, None] * seg
    dist = np.linalg.norm(cxy - closest, axis=1)
    blocked = (dist < radii) & (radii > 1e-6)
    if exclude is not None:
        blocked[exclude] = False
    return int(np.sum(blocked))


def cumulative_s(pts):
    pts = np.asarray(pts, float)
    if len(pts) < 2:
        return np.zeros(len(pts))
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1))])


def closest_on_polyline(p, pts, cum=None):
    """Return (distance, s_along, seg_index) of point p to the polyline pts."""
    pts = np.asarray(pts, float)[:, :2]
    p = np.asarray(p[:2], float)
    if cum is None:
        cum = cumulative_s(pts)
    if len(pts) == 1:
        return float(np.linalg.norm(p - pts[0])), 0.0, 0
    a = pts[:-1]
    b = pts[1:]
    ab = b - a
    L2 = np.sum(ab * ab, axis=1)
    L2 = np.where(L2 < 1e-12, 1e-12, L2)
    t = np.clip(np.sum((p - a) * ab, axis=1) / L2, 0.0, 1.0)
    proj = a + t[:, None] * ab
    d = np.linalg.norm(p - proj, axis=1)
    i = int(np.argmin(d))
    s = cum[i] + t[i] * math.sqrt(L2[i])
    return float(d[i]), float(s), i


def point_at_s(pts, s, cum=None):
    pts = np.asarray(pts, float)[:, :2]
    if cum is None:
        cum = cumulative_s(pts)
    if s <= 0:
        return pts[0].copy()
    if s >= cum[-1]:
        return pts[-1].copy()
    i = int(np.searchsorted(cum, s) - 1)
    i = max(0, min(i, len(pts) - 2))
    seg = cum[i + 1] - cum[i]
    t = 0.0 if seg < 1e-9 else (s - cum[i]) / seg
    return pts[i] + t * (pts[i + 1] - pts[i])


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
