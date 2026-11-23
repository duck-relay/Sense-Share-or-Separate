"""Per-drone navigation filter: ONE extended Kalman filter that every
measurement goes through before the drone uses it (all configurations).

State  s = [x, y, z, yaw, wx, wy, bx, by]
    x, y, z  position (m, world frame)
    yaw      heading (rad)
    wx, wy   wind: velocity of the air over the ground (m/s)
    bx, by   bias of the VIO velocity (IMU + visual-inertial odometry) (m/s)

Motion model (prediction, every control tick)
    flying   : p <- p + (v_air + w) dt     v_air = the drone's own air-velocity
                                           command after the airframe's response lag
                                           (w only acts once the drone has lifted off)
    parked   : p unchanged (motors off, the drone stands on its legs)
    yaw      : yaw <- yaw + gyro_rate dt
    w        : first-order Gauss-Markov (wind changes slowly and stays bounded)
    b        : random walk

Measurement updates (each used only where the configuration has it)
    always      ToF altimeter (slant range, tilt from the IMU) and IMU heading
    VIO measured velocity  = v_air + w + b  (airborne)  /  b  (parked:
                the drone calibrates its bias on the pad before take-off)
    camera      body-frame bearing + 3-D PnP range to each ArUco marker of an
                N-point fix (one joint iterated update when the position is very
                uncertain, e.g. the first fix after a long gap)
    swarm       3-D UWB range to a peer drone, at the peer's broadcast EKF
                estimate; the peer's covariance is added to the measurement
                noise ("consider" update) so it is not double-counted
Every update is chi-square gated, so wrong markers and NLOS ranges are rejected.

The controller only ever uses this filter's outputs: position, heading, and
the wind estimate (it commands  v_air = v_desired - wind  to hold its track).
"""
import math
import numpy as np

CHI2_GATE = 9.0            # 3 sigma for scalar updates
SD_PERSIST = 4             # VIO rejected this many times in a row (0.2 s) while flying: the drone
                           # really moves unlike its model (pushed by downwash, held by an obstacle), so the
                           # disturbance (wind) state takes the difference up instead of the estimate drifting
IX, IY, IZ, IYAW, IWX, IWY, IBX, IBY = range(8)
N = 8


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class DroneEKF:
    def __init__(self, x0, y0, z0, yaw0=0.0, wind_sigma0=0.25, bias_sigma0=0.0, response_tau=0.06, wind_tau=30.0):
        self.s = np.array([x0, y0, z0, yaw0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        self.P = np.diag([0.05 ** 2, 0.05 ** 2, 0.02 ** 2, math.radians(3.0) ** 2,
                          wind_sigma0 ** 2, wind_sigma0 ** 2,
                          max(bias_sigma0, 1e-6) ** 2, max(bias_sigma0, 1e-6) ** 2])
        self.has_bias = bias_sigma0 > 0.0
        self.tau = response_tau
        self.wind_tau = wind_tau
        self.v_air = np.zeros(3)           # expected air velocity (command through the response lag)
        self.airborne = False              # lifted off: the wind moves the drone
        self.tof_rejects = 0
        self.sd_rejects = [0, 0]
        self.n_disturbance = 0             # times the VIO revealed an unmodelled push
        self.heading_rejects = 0
        self.heading_init = False
        self.n_updates = 0
        self.n_rejected = 0

    # ------------------------------------------------------------- outputs
    @property
    def pos(self):
        return self.s[:3].copy()

    @property
    def yaw(self):
        return float(self.s[IYAW])

    @property
    def wind(self):
        return self.s[IWX:IWY + 1].copy()

    @property
    def vel(self):
        """Estimated velocity over the ground (3-D)."""
        if not self.airborne:
            return np.zeros(3)
        return np.array([self.v_air[0] + self.s[IWX], self.v_air[1] + self.s[IWY], self.v_air[2]])

    @property
    def cov_xy(self):
        return self.P[:2, :2].copy()

    @property
    def cov_pos(self):
        return self.P[:3, :3].copy()

    def sigma_xy(self):
        return math.sqrt(max(0.0, 0.5 * (self.P[IX, IX] + self.P[IY, IY])))

    # ---------------------------------------------------------- prediction
    def predict(self, v_cmd, gyro_rate, dt, motors_on, lifted_off, q_track, q_wind, q_bias, gyro_sigma):
        """v_cmd: the air-velocity command sent this tick (world frame, in the EKF heading).
        q_track (m/sqrt(s)), q_wind (m/s/sqrt(s)), q_bias (m/s/sqrt(s)): process noise
        DENSITIES, so the tuning does not depend on the control rate."""
        if dt <= 0:
            return
        P = self.P
        self.airborne = bool(motors_on and lifted_off)
        if motors_on:
            a = 1.0 - math.exp(-dt / self.tau)
            self.v_air = self.v_air + a * (np.asarray(v_cmd, float) - self.v_air)
            wx, wy = (self.s[IWX], self.s[IWY]) if self.airborne else (0.0, 0.0)
            self.s[IX] += (self.v_air[0] + wx) * dt
            self.s[IY] += (self.v_air[1] + wy) * dt
            self.s[IZ] += self.v_air[2] * dt
            # P <- F P F^T with F = I + dt * A. A couples position to the wind (in the air)
            # and to the heading: a heading error rotates the commanded velocity.
            vx, vy = self.v_air[0], self.v_air[1]
            kw = dt if self.airborne else 0.0
            P[IX, :] += kw * P[IWX, :] - vy * dt * P[IYAW, :]
            P[IY, :] += kw * P[IWY, :] + vx * dt * P[IYAW, :]
            P[:, IX] += kw * P[:, IWX] - vy * dt * P[:, IYAW]
            P[:, IY] += kw * P[:, IWY] + vx * dt * P[:, IYAW]
            if self.airborne:
                phi = math.exp(-dt / self.wind_tau)
                self.s[IWX:IWY + 1] *= phi
                P[IWX:IWY + 1, :] *= phi
                P[:, IWX:IWY + 1] *= phi
                P[IWX, IWX] += q_wind ** 2 * dt
                P[IWY, IWY] += q_wind ** 2 * dt
            qp = q_track ** 2 * dt
            P[IX, IX] += qp
            P[IY, IY] += qp
            P[IZ, IZ] += qp
        else:
            self.v_air[:] = 0.0
            P[IZ, IZ] += 1e-6 * dt
        self.s[IYAW] = wrap(self.s[IYAW] + gyro_rate * dt)
        P[IYAW, IYAW] += (gyro_sigma * dt) ** 2 + 1e-7 * dt
        if self.has_bias:
            P[IBX, IBX] += q_bias ** 2 * dt
            P[IBY, IBY] += q_bias ** 2 * dt

    # ------------------------------------------------------------- updates
    def _update(self, h, y, R, gate):
        """Scalar update with measurement row h (N,), innovation y, noise variance R."""
        Ph = self.P @ h
        S = float(h @ Ph) + R
        if S <= 0.0 or y * y / S > gate:
            self.n_rejected += 1
            return False
        K = Ph / S
        self.s += K * y
        self.s[IYAW] = wrap(self.s[IYAW])
        self.P -= np.outer(K, Ph)
        self.P = 0.5 * (self.P + self.P.T)
        self.n_updates += 1
        return True

    def update_altitude(self, tof_range, roll, pitch, sigma):
        """Down-looking ToF: measures the slant range z / (cos(roll) cos(pitch))."""
        c = max(0.3, math.cos(roll) * math.cos(pitch))
        h = np.zeros(N)
        h[IZ] = 1.0 / c
        if self._update(h, tof_range - self.s[IZ] / c, sigma ** 2, 25.0):
            self.tof_rejects = 0
            return True
        self.tof_rejects += 1
        if self.tof_rejects >= 5:          # the altimeter is the only height reference: re-initialise
            self.s[IZ] = tof_range * c
            self.P[IZ, :] = 0.0
            self.P[:, IZ] = 0.0
            self.P[IZ, IZ] = sigma ** 2
            self.tof_rejects = 0
        return False

    def update_heading(self, yaw_meas, sigma):
        """Magnetometer heading. The first reading initialises the heading (the drone
        does not know how it was placed on the pad); persistent rejection re-initialises."""
        if not self.heading_init or self.heading_rejects >= 10:
            self.s[IYAW] = wrap(yaw_meas)
            self.P[IYAW, :] = 0.0
            self.P[:, IYAW] = 0.0
            self.P[IYAW, IYAW] = sigma ** 2
            self.heading_init = True
            self.heading_rejects = 0
            return True
        h = np.zeros(N)
        h[IYAW] = 1.0
        if self._update(h, wrap(yaw_meas - self.s[IYAW]), sigma ** 2, 25.0):
            self.heading_rejects = 0
            return True
        self.heading_rejects += 1
        return False

    def update_sensor_data(self, v_meas, sigma, gate=CHI2_GATE):
        """VIO (IMU + visual-inertial odometry): horizontal velocity = v_air + wind + bias in the air,
        bias only while parked (true velocity 0) - that is the on-pad calibration.
        A velocity that disagrees for SD_PERSIST updates in a row is a real, unmodelled push
        (downwash, contact): the wind state's uncertainty is raised so it absorbs it."""
        ok = 0
        for k, (iw, ib) in enumerate(((IWX, IBX), (IWY, IBY))):
            h = np.zeros(N)
            h[ib] = 1.0
            pred = self.s[ib]
            if self.airborne:
                h[iw] = 1.0
                pred += self.v_air[k] + self.s[iw]
            y = float(v_meas[k]) - pred
            if self._update(h, y, sigma ** 2, gate):
                ok += 1
                self.sd_rejects[k] = 0
            elif self.airborne:
                self.sd_rejects[k] += 1
                if self.sd_rejects[k] >= SD_PERSIST:
                    self.P[iw, iw] += y * y
                    ok += int(self._update(h, y, sigma ** 2, gate))
                    self.sd_rejects[k] = 0
                    self.n_disturbance += 1
        return ok

    def update_bearing(self, point, bearing_body, sigma, gate=CHI2_GATE):
        """Bearing to a known point measured in the BODY frame (camera)."""
        dx = point[0] - self.s[IX]
        dy = point[1] - self.s[IY]
        r2 = dx * dx + dy * dy
        if r2 < 0.09:
            return False
        h = np.zeros(N)
        h[IX] = dy / r2
        h[IY] = -dx / r2
        h[IYAW] = -1.0
        y = wrap(bearing_body - (math.atan2(dy, dx) - self.s[IYAW]))
        return self._update(h, y, sigma ** 2, gate)

    def update_landmarks(self, points, bearings, ranges, s_bear, s_range, iters=8, gate=16.0):
        """One camera frame's N-point fix as a JOINT iterated-EKF update (bearing + range of every
        marker). Used when the position is very uncertain - e.g. the first fix after a long gap -
        where one-at-a-time linearised updates converge to the wrong place and leave the filter
        over-confident. A marker whose residual stays above sqrt(gate) sigma is dropped once
        (wrong marker / bad corners). Returns the number of markers used."""
        x0, P0 = self.s.copy(), self.P.copy()
        pts = [np.asarray(p, float) for p in points]
        keep = list(range(len(pts)))
        for attempt in range(2):
            x = x0.copy()
            Hm = K = None
            for _ in range(iters):
                rows, res, rv = [], [], []
                for i in keep:
                    dx, dy = pts[i][0] - x[IX], pts[i][1] - x[IY]
                    r2 = max(dx * dx + dy * dy, 0.09)
                    h = np.zeros(N)
                    h[IX], h[IY], h[IYAW] = dy / r2, -dx / r2, -1.0
                    rows.append(h)
                    res.append(wrap(bearings[i] - (math.atan2(dy, dx) - x[IYAW])))
                    rv.append(s_bear[i] ** 2)
                    d = x[:3] - pts[i][:3]
                    rh = max(float(np.linalg.norm(d)), 0.2)
                    h = np.zeros(N)
                    h[:3] = d / rh
                    rows.append(h)
                    res.append(ranges[i] - rh)
                    rv.append(s_range[i] ** 2)
                Hm, R = np.array(rows), np.diag(rv)
                y = np.array(res) + Hm @ (x - x0)
                y[0::2] = [wrap(v) for v in y[0::2]]
                S = Hm @ P0 @ Hm.T + R
                K = np.linalg.solve(S, Hm @ P0).T
                x_new = x0 + K @ y
                x_new[IYAW] = wrap(x_new[IYAW])
                done = float(np.linalg.norm(x_new[:2] - x[:2])) < 1e-3
                x = x_new
                if done:
                    break
            # residuals of each marker at the solution
            worst, wi = 0.0, None
            for n, i in enumerate(keep):
                dx, dy = pts[i][0] - x[IX], pts[i][1] - x[IY]
                rb = wrap(bearings[i] - (math.atan2(dy, dx) - x[IYAW])) / s_bear[i]
                rr = (ranges[i] - float(np.linalg.norm(x[:3] - pts[i][:3]))) / s_range[i]
                m = rb * rb + rr * rr
                if m > worst:
                    worst, wi = m, n
            if worst > 2.0 * gate and attempt == 0 and len(keep) > 2:
                del keep[wi]
                self.n_rejected += 1
                continue
            if worst > 2.0 * gate:
                self.n_rejected += len(keep)
                return 0
            break
        P = (np.eye(N) - K @ Hm) @ P0
        self.s = x
        self.P = 0.5 * (P + P.T)
        self.n_updates += 2 * len(keep)
        return len(keep)

    def update_range(self, point, r_meas, sigma, extra_var=0.0, gate=CHI2_GATE):
        """3-D range to a known point (marker) or to a peer's estimate (UWB)."""
        d = self.s[:3] - np.asarray(point, float)[:3]
        r_hat = float(np.linalg.norm(d))
        if r_hat < 0.2:
            return False
        h = np.zeros(N)
        h[:3] = d / r_hat
        return self._update(h, r_meas - r_hat, sigma ** 2 + extra_var, gate)
