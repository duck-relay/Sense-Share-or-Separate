"""Synthetic on-board sensors, generated from Gazebo ground truth + noise.

Gazebo Fortress here runs no ArUco detector, VIO, UWB or IMU plugin (the heavy
RGB-D cameras were what made the old world run at 10 % real time), so the
measurements are generated from ground truth with explicit, editable noise
models. The drone never sees ground truth: every measurement below goes into
its EKF (estimator.py) first.

Always on (every configuration):
  IMU             gyro yaw rate, attitude (roll/pitch) and magnetometer heading
  ToF altimeter   down-looking slant range
  depth camera    forward depth image for OBSTACLE detection (and other drones)
Configuration dependent:
  camera N-point  ArUco markers on the 20 known trees -> body bearing + PnP range
  VIO             IMU + visual-inertial odometry: measured velocity with noise
                  and a slowly drifting bias
  swarm           UWB peer-to-peer ranges (+ the peers' broadcast estimates)
"""
import math
import numpy as np

from . import config as C
from .geometry import ray_cast, segment_blockers, wrap_angle

CAM_TILT = math.radians(15.0)      # camera pitched 15 deg down (as on the original drone model)
CAM_FWD = 0.15                     # camera 15 cm ahead of the body centre


class SensorSuite:
    def __init__(self, world, rng, sp=None):
        self.world = world
        self.sp = sp or C.SENSORS
        # one random stream per sensor: a sensor that is added, removed or used more often (camera
        # frames, peer ranges) never changes another sensor's noise, so configurations flown on the
        # same mission share their noise (common random numbers: tighter paired comparisons)
        (self.r_imu, self.r_alt, self.r_odom, self.r_cam, self.r_uwb, self.r_depth, self.r_ring,
         self.r_prox) = (np.random.default_rng(int(s)) for s in rng.integers(1 << 31, size=8))
        self.bias = self.r_odom.normal(0.0, self.sp.odom_bias_init, size=2)
        self.markers = np.array([[x, y, C.ANCHOR_Z] for (x, y) in C.LANDMARK_TREES])
        self.half_h = math.radians(self.sp.cam_fov_deg) / 2.0     # horizontal coverage
        self.half_v = math.radians(self.sp.cam_vfov_deg) / 2.0    # vertical field of view
        self.prev_yaw = None

    # ------------------------------------------------------------------ IMU
    def gyro(self, yaw_true, dt):
        """Yaw rate over the last tick (rad/s) with gyro noise."""
        if self.prev_yaw is None or dt <= 0:
            rate = 0.0
        else:
            rate = wrap_angle(yaw_true - self.prev_yaw) / dt
        self.prev_yaw = yaw_true
        return rate + self.r_imu.normal(0.0, self.sp.gyro_sigma)

    def attitude(self, roll, pitch):
        s = math.radians(self.sp.attitude_sigma_deg)
        return roll + self.r_imu.normal(0.0, s), pitch + self.r_imu.normal(0.0, s)

    def compass(self, yaw_true):
        return wrap_angle(yaw_true + self.r_imu.normal(0.0, math.radians(self.sp.compass_sigma_deg)))

    def tof(self, z_true, roll, pitch):
        """Down-looking ToF: slant range to the ground."""
        c = max(0.3, math.cos(roll) * math.cos(pitch))
        return max(0.0, z_true / c + self.r_alt.normal(0.0, self.sp.tof_sigma))

    # ---------------------------------------------------------- VIO
    def odometry(self, v_true, dt):
        """VIO: IMU + visual-inertial odometry, horizontal velocity with a drifting bias."""
        sp = self.sp
        if dt > 0:
            self.bias += self.r_odom.normal(0.0, sp.odom_bias_rw * math.sqrt(dt), size=2)
        n = self.r_odom.normal(0.0, sp.odom_sigma, size=2)
        return np.array([v_true[0] + self.bias[0] + n[0], v_true[1] + self.bias[1] + n[1]])

    # --------------------------------------------------------------- camera
    def _los_clear(self, a, b, exclude):
        """3-D line of sight a->b against tree trunks/crowns and pine tiers."""
        L = float(np.linalg.norm(b - a))
        n = max(2, int(L / 0.15))
        t = np.linspace(0.0, 1.0, n + 1)[1:-1]
        t = t[t * L < L - 0.25]                 # ignore the target trunk itself
        if len(t) == 0:
            return True
        pts = a[None, :] + t[:, None] * (b - a)[None, :]
        R = self.world.radii_matrix(pts[:, 2])                       # (S, N)
        d = np.hypot(pts[:, None, 0] - self.world.xy[None, :, 0], pts[:, None, 1] - self.world.xy[None, :, 1])
        blocked = d < R
        if exclude is not None:
            blocked[:, exclude] = False
        return not bool(np.any(blocked))

    def camera_markers(self, p_true, yaw_true):
        """ArUco markers detected in one camera frame.

        Returns a list (nearest first) of dicts: landmark id, landmark xyz,
        measured BODY-frame bearing (rad), measured 3-D range (m) and their sigmas.
        """
        sp = self.sp
        cam = np.array([p_true[0] + CAM_FWD * math.cos(yaw_true),
                        p_true[1] + CAM_FWD * math.sin(yaw_true), p_true[2] - 0.01])
        # geometry for visibility is taken from the camera; the MEASUREMENT is
        # reported relative to the body centre (lever arm already compensated)
        rel = self.markers[:, :2] - cam[:2]
        dh = np.hypot(rel[:, 0], rel[:, 1])
        brg_cam = np.arctan2(rel[:, 1], rel[:, 0])
        relb = self.markers[:, :2] - p_true[:2]
        brg = np.arctan2(relb[:, 1], relb[:, 0])
        off = np.abs((brg_cam - yaw_true + math.pi) % (2 * math.pi) - math.pi)
        elev = np.arctan2(self.markers[:, 2] - cam[2], np.maximum(dh, 1e-3))
        cand = np.where((dh > 0.35) & (dh <= sp.cam_range) & (off <= self.half_h)
                        & (np.abs(elev + CAM_TILT) <= self.half_v))[0]
        out = []
        for i in cand:
            # marker point on the trunk surface facing the camera (4 markers per trunk)
            mpt = self.markers[i].copy()
            mpt[:2] -= rel[i] / dh[i] * C.TREE_TRUNK_R
            if not self._los_clear(cam, mpt, exclude=[i]):
                continue
            d = dh[i]
            p_det = sp.cam_p_near if d <= sp.cam_sure_range else \
                sp.cam_p_near + (sp.cam_p_far - sp.cam_p_near) * (d - sp.cam_sure_range) / (sp.cam_range - sp.cam_sure_range)
            if self.r_cam.random() > p_det:
                continue
            sb = math.radians(sp.cam_bearing_sigma_deg)
            r3 = float(np.linalg.norm(self.markers[i] - p_true))
            b_meas = float(brg[i]) - yaw_true + self.r_cam.normal(0.0, sb)
            r_meas = r3 + self.r_cam.normal(0.0, sp.cam_range_sigma0 + sp.cam_range_sigma_k * r3 * r3)
            if self.r_cam.random() < sp.cam_outlier_prob:              # wrong id / bad corners
                b_meas += self.r_cam.choice([-1, 1]) * math.radians(self.r_cam.uniform(6, 15))
                r_meas *= self.r_cam.uniform(0.6, 1.5)
            r_meas = max(0.2, r_meas)
            sr = sp.cam_range_sigma0 + sp.cam_range_sigma_k * r_meas * r_meas    # from what the drone measured
            out.append({'id': int(i), 'xyz': self.markers[i], 'bearing': wrap_angle(b_meas),
                        'range': r_meas, 'sb': sb, 'sr': sr})
        out.sort(key=lambda m: m['range'])
        return out

    # ---------------------------------------------------------------- swarm
    def _nlos_bias(self, p, q):
        radii = self.world.radii_at(C.TREE_CROWN_Z)
        k = segment_blockers(p, q, self.world.xy, radii)
        b = 0.0
        if k:
            b += float(np.sum(self.r_uwb.uniform(self.sp.nlos_bias_min, self.sp.nlos_bias_max, size=k)))
        if self.r_uwb.random() < self.sp.multipath_prob:
            b += self.r_uwb.uniform(0.1, 0.6)
        return b

    def uwb_peer_range(self, p_true, q_true):
        r = float(np.linalg.norm(p_true - q_true))
        if r > self.sp.peer_max_range:
            return None
        sigma = self.sp.uwb_sigma + self.sp.peer_sigma_per_m * r       # noise grows with distance
        return r + self._nlos_bias(p_true, q_true) + self.r_uwb.normal(0.0, sigma)

    # ------------------------------------------------------- obstacle depth
    def depth_scan(self, p_true, yaw_true):
        """Forward depth camera. Returns ray angles in the BODY frame, measured
        ranges and hit flags. The drone rotates them with its EKF heading to
        (a) map unknown obstacles at its EKF position and (b) brake reactively
        on the latest hits, which works even when its position estimate drifts."""
        sp = self.sp
        half = math.radians(sp.depth_fov_deg) / 2.0
        ang_b = np.linspace(-half, half, sp.depth_rays)
        # everything from OBSTACLE_V_CLEAR below the feet to OBSTACLE_V_CLEAR above the rotors is in
        # the way (the camera's vertical field of view): a drone does not skim over a pine top
        z = p_true[2]
        radii = self.world.radii_band(z - C.REST_Z - C.OBSTACLE_V_CLEAR, z + C.DRONE_DISC_DZ + C.OBSTACLE_V_CLEAR)
        d = ray_cast(p_true[:2], yaw_true + ang_b, sp.depth_range, self.world.xy, radii)
        hit = np.isfinite(d)
        rng = np.where(hit, d + self.r_depth.normal(0.0, sp.depth_sigma, size=len(d)), sp.depth_range)
        return ang_b, rng, hit

    def tof_ring(self, p_true, yaw_true):
        """Short-range ToF ring, 360 deg around the drone: obstacles beside and behind it, which the forward
        depth camera does not see (e.g. the far side of a tree the drone is flying round). Same vertical band
        as the depth camera. Returns ray angles in the BODY frame, measured ranges and hit flags."""
        sp = self.sp
        ang_b = np.linspace(-math.pi, math.pi, sp.ring_rays, endpoint=False)
        z = p_true[2]
        radii = self.world.radii_band(z - C.REST_Z - C.OBSTACLE_V_CLEAR, z + C.DRONE_DISC_DZ + C.OBSTACLE_V_CLEAR)
        d = ray_cast(p_true[:2], yaw_true + ang_b, sp.ring_range, self.world.xy, radii)
        hit = np.isfinite(d)
        rng = np.where(hit, d + self.r_ring.normal(0.0, sp.ring_sigma, size=len(d)), sp.ring_range)
        return ang_b, rng, hit

    def proximity(self, p_true, yaw_true, others_true):
        """Other flying drones within 3 m horizontally, seen by the short-range proximity sensors
        (depth camera + side/rear ToF ring, 360 deg around, +-45 deg up/down) when nothing blocks
        the line of sight. A drone (almost) straight above or below is NOT seen.
        Returns (id, relative position in the BODY frame) pairs."""
        out = []
        c, s = math.cos(yaw_true), math.sin(yaw_true)
        radii = self.world.radii_at(p_true[2])
        tan_el = math.tan(math.radians(self.sp.proximity_elev_deg))
        for oid, q in others_true.items():
            d = float(np.linalg.norm(p_true[:2] - q[:2]))
            if abs(q[2] - p_true[2]) > tan_el * d:          # (almost) straight above / below: not in view
                continue
            if d <= self.sp.proximity_range and segment_blockers(p_true, q, self.world.xy, radii) == 0:
                rel = q - p_true + self.r_prox.normal(0.0, 0.08, size=3)
                out.append((oid, np.array([c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1], rel[2]])))
        return out
