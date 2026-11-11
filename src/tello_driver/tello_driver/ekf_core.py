"""Drone state EKF (pure numpy, no ROS: also used offline to replay recorded flights).

State  s = [x, y, z, vx, vy, vz, yaw, b, dx, dy]   (map frame; b = IMU yaw offset: yaw_imu = yaw + b;
                                                    dx, dy = drift velocity: the Tello moves with a zero command)
Input  u = [ux, uy, uz, wz]                   body-frame velocity command really sent to the Tello (m/s, rad/s)

Model, identified on the real drone (Nov 11): each velocity follows the command with a first-order lag and a
pure delay,  tau dv/dt = K R(yaw) u(t - L_u) + d - v,  position = integral of v,  yaw rate = K_w wz(t - L_u).
The drift d (its optical-flow hold does not work here: 5-10 cm/s with zero command, Nov 11) is a slowly varying
velocity learnt while the markers are in view, and kept when they are not.
The total delay command -> ArUco pose was ~0.5 s: L_u for the drone, L_v for the video + detection.

Measurements
  ArUco pose  [x, y, z, yaw]  taken L_v seconds ago: the filter goes back in time (stored states), applies the
              update there, and re-propagates to now with the stored inputs. Chi-square gate.
  ToF height  z  (rejected over furniture by a gate: the bed reads 0.85 m less)
  IMU heading yaw + b  (keeps the heading when the markers are out of view)

Without markers the position is predicted from the commands (dead reckoning): its covariance grows with the
real drift of the Tello (~5-10 cm/s, optical-flow hold not working), so the users know when to stop trusting it.
"""
import math
from collections import deque

import numpy as np

NX = 10
IX, IY, IZ, IVX, IVY, IVZ, IYAW, IB, IDX, IDY = range(NX)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DroneEKF:
    # defaults: identification flight + EKF replay of the real flights of Nov 11 (tools/ekf_replay.py)
    def __init__(self, K_xy=1.15, tau_xy=0.40, K_z=0.9, tau_z=0.45, K_w=1.0, L_u=0.25, L_v=0.25,
                 accel_noise=0.6, yaw_rate_noise=0.15, drift_noise=0.01, history_s=1.5):
        self.K_xy, self.tau_xy, self.K_z, self.tau_z, self.K_w = K_xy, tau_xy, K_z, tau_z, K_w
        self.L_u, self.L_v = L_u, L_v
        self.qa, self.qw, self.qd = accel_noise, yaw_rate_noise, drift_noise
        self.s = np.zeros(NX)
        self.P = np.diag([1e3] * 3 + [1.0] * 3 + [10.0, 10.0] + [0.1 ** 2] * 2)
        self.t = None
        self.initialised = False
        self.inputs = deque()                     # (t, u) commands, to apply them L_u later
        self.hist = deque()                       # (t, s, P, u_applied) for delayed measurements
        self.history_s = history_s
        self.rejected = 0

    # ------------------------------------------------------------------ model
    def command_at(self, t):
        """Command in effect at time t - L_u (zero-order hold)."""
        tq = t - self.L_u
        u = np.zeros(4)
        for tu, uu in self.inputs:
            if tu <= tq:
                u = uu
            else:
                break
        return u

    def add_command(self, t, u):
        self.inputs.append((t, np.asarray(u, float)))
        while len(self.inputs) > 2 and self.inputs[1][0] < t - self.L_u - self.history_s - 1.0:
            self.inputs.popleft()

    def _step(self, s, P, u, dt):
        x, y, z, vx, vy, vz, yaw, b, dx, dy = s
        c, sn = math.cos(yaw), math.sin(yaw)
        cx = self.K_xy * (c * u[0] - sn * u[1])
        cy = self.K_xy * (sn * u[0] + c * u[1])
        cz = self.K_z * u[2]
        a_xy, a_z = dt / self.tau_xy, dt / self.tau_z
        s2 = s.copy()
        s2[IX] += vx * dt
        s2[IY] += vy * dt
        s2[IZ] += vz * dt
        s2[IVX] += a_xy * (cx + dx - vx)
        s2[IVY] += a_xy * (cy + dy - vy)
        s2[IVZ] += a_z * (cz - vz)
        s2[IYAW] = wrap(yaw + self.K_w * u[3] * dt)
        F = np.eye(NX)
        F[IX, IVX] = F[IY, IVY] = F[IZ, IVZ] = dt
        F[IVX, IVX] = F[IVY, IVY] = 1 - a_xy
        F[IVX, IDX] = F[IVY, IDY] = a_xy
        F[IVZ, IVZ] = 1 - a_z
        F[IVX, IYAW] = a_xy * self.K_xy * (-sn * u[0] - c * u[1])
        F[IVY, IYAW] = a_xy * self.K_xy * (c * u[0] - sn * u[1])
        Q = np.zeros((NX, NX))
        Q[IVX, IVX] = Q[IVY, IVY] = (self.qa * dt) ** 2 + 1e-6
        Q[IVZ, IVZ] = (0.5 * self.qa * dt) ** 2 + 1e-6
        Q[IX, IX] = Q[IY, IY] = Q[IZ, IZ] = 1e-6
        Q[IYAW, IYAW] = (self.qw * dt) ** 2
        Q[IB, IB] = (0.002 * dt) ** 2
        Q[IDX, IDX] = Q[IDY, IDY] = self.qd ** 2 * dt  # random walk: the drift changes over seconds
        return s2, F @ P @ F.T + Q

    def predict(self, t):
        """Propagate to time t (small internal steps)."""
        if self.t is None:
            self.t = t
            return
        while self.t < t - 1e-6:
            dt = min(0.02, t - self.t)
            u = self.command_at(self.t)
            self.s, self.P = self._step(self.s, self.P, u, dt)
            self.t += dt
            self.hist.append((self.t, self.s.copy(), self.P.copy()))
        while self.hist and self.hist[0][0] < t - self.history_s:
            self.hist.popleft()

    # ------------------------------------------------------------------ updates
    def _update(self, s, P, z, h, H, R, angle_rows=()):
        y = z - h
        for i in angle_rows:
            y[i] = wrap(y[i])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        s = s + K @ y
        s[IYAW] = wrap(s[IYAW])
        P = (np.eye(NX) - K @ H) @ P
        return s, 0.5 * (P + P.T), float(y @ np.linalg.solve(S, y))

    def update_pose(self, t_recv, pose, sigma_p, sigma_yaw, gate=16.3):
        """ArUco pose (x, y, z, yaw) received at t_recv, image taken L_v earlier."""
        pose = np.asarray(pose, float)
        R = np.diag([max(sigma_p, 0.02) ** 2] * 3 + [max(sigma_yaw, math.radians(1.5)) ** 2])
        H = np.zeros((4, NX))
        H[0, IX] = H[1, IY] = H[2, IZ] = H[3, IYAW] = 1.0
        if not self.initialised:
            self.s[:3] = pose[:3]
            self.s[3:6] = 0.0
            self.s[IYAW] = pose[3]
            self.s[IDX] = self.s[IDY] = 0.0
            self.P = np.diag([0.05 ** 2] * 3 + [0.2 ** 2] * 3 + [0.05 ** 2, 1.0] + [0.08 ** 2] * 2)
            self.t = t_recv
            self.hist.clear()
            self.initialised = True
            self.b_init = False
            return True
        self.predict(t_recv)
        t_meas = t_recv - self.L_v
        # state at the time of the image
        k = None
        for i in range(len(self.hist) - 1, -1, -1):
            if self.hist[i][0] <= t_meas:
                k = i
                break
        if k is None:
            s0, P0, t0 = self.s, self.P, self.t
        else:
            t0, s0, P0 = self.hist[k]
        s1, P1, d2 = self._update(s0.copy(), P0.copy(), pose, s0[[IX, IY, IZ, IYAW]], H, R, angle_rows=(3,))
        if d2 > gate:
            self.rejected += 1
            if self.rejected < 8:
                return False
            # many rejections in a row: the filter is lost (long dead reckoning), restart from the markers
            self.initialised = False
            self.rejected = 0
            return self.update_pose(t_recv, pose, sigma_p, sigma_yaw)
        self.rejected = 0
        # re-propagate from the image time to now with the stored commands
        t_now = self.t
        self.s, self.P, self.t = s1, P1, t0
        self.hist = deque(h for h in self.hist if h[0] <= t0)
        self.predict(t_now)
        return True

    def update_height(self, z_meas, sigma=0.05, gate=6.6):
        """Distance sensor: height above what is below (gated: over the bed it reads ~0.85 m less)."""
        if not self.initialised:
            return False
        H = np.zeros((1, NX))
        H[0, IZ] = 1.0
        s1, P1, d2 = self._update(self.s, self.P, np.array([z_meas]), np.array([self.s[IZ]]), H, np.array([[sigma ** 2]]))
        if d2 > gate:
            return False
        self.s, self.P = s1, P1
        return True

    def update_imu_yaw(self, yaw_imu, sigma=math.radians(1.0)):
        if not self.initialised:
            return False
        if not getattr(self, "b_init", False):  # the offset can be anything in [-pi, pi]: take it directly
            self.s[IB] = wrap(yaw_imu - self.s[IYAW])
            self.P[IB, :] = self.P[:, IB] = 0.0
            self.P[IB, IB] = self.P[IYAW, IYAW] + sigma ** 2
            self.b_init = True
            return True
        H = np.zeros((1, NX))
        H[0, IYAW] = H[0, IB] = 1.0
        h = np.array([wrap(self.s[IYAW] + self.s[IB])])
        self.s, self.P, _ = self._update(self.s, self.P, np.array([yaw_imu]), h, H, np.array([[sigma ** 2]]),
                                         angle_rows=(0,))
        return True

    # ------------------------------------------------------------------ outputs
    def sigma_xy(self):
        return math.sqrt(max(self.P[IX, IX], self.P[IY, IY]))
