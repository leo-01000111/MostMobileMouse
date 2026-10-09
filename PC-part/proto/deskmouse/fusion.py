"""Velocity Kalman filter: accelerometer prediction + camera velocity measurement (notes/DECISIONS.md 2026-10-09).

Smaller than DESIGN.md's EKF. World-frame 2D velocity is v(t) = V(t) + c, where V is the open-loop integral of the
rest-bias-corrected, yaw-rotated accelerometer (500 Hz) and c is a velocity offset the filter estimates. Because c is
the same at every time, a camera measurement that refers to a past frame interval corrects the present directly,
which is how the ~45 ms camera delay is handled. The scale (metres per front-end relative unit) is λ = λ0·exp(ℓ),
with ℓ in the state, so every acceleration the IMU sees against the camera refines it.

State x = [c_x, c_y, ℓ], covariance P (3×3).
  predict (per accel sample): V += a_world·dt, P_cc += q_acc²·dt, P_ℓℓ += q_ℓ²·dt
  camera (per frame, interval [t0, t1]): z = d_rel / (t1 - t0)   (relative units / s)
      h = (V̄ + c) / λ, V̄ = mean of V over the interval; H = [I/λ, -(V̄ + c)/λ]
      rejected when the innovation exceeds `gate` σ (knock ringing, mismatches)
Position: P(t) = PV(t) + C, PV the integral of V, C the integral of c (corrections move the past too).
"""
from __future__ import annotations

from collections import deque

import numpy as np


class VelocityKF:
    def __init__(self, acc_noise: float, scale_noise: float, gate: float, resync_frames: int, learn_scale: bool = False,
                 history: int = 1500):
        self.q_acc = acc_noise
        self.q_l = scale_noise
        self.learn_scale = learn_scale  # False: camera updates leave ℓ alone (scale only from strokes/calibration)
        self.gate = gate
        self.resync_frames = resync_frames
        self.V = np.zeros(2)
        self.PV = np.zeros(2)
        self.C = np.zeros(2)
        self.c = np.zeros(2)
        self.l = 0.0
        self.lam0: float | None = None
        self.P = np.diag([1.0, 1.0, 0.0])
        self.t: int | None = None
        self.hist: deque = deque(maxlen=history)  # (t, V, PV, C) per accel sample
        self.rejected_run = 0
        self.stats = dict(updates=0, rejected=0, resyncs=0)

    @property
    def ready(self) -> bool:
        return self.lam0 is not None and len(self.hist) > 1

    @property
    def lam(self) -> float:
        return float(self.lam0 * np.exp(self.l))

    def set_scale(self, lam: float, sigma: float):
        """(Re)anchor the scale: λ0 = lam, ℓ = 0 with standard deviation `sigma` (log units)."""
        self.lam0 = lam
        self.l = 0.0
        self.P[2, :] = 0
        self.P[:, 2] = 0
        self.P[2, 2] = sigma ** 2

    def scale_measurement(self, lam_meas: float, sigma: float):
        """A direct scale measurement (one stroke: IMU distance / camera distance), log-normal with std `sigma`."""
        y = float(np.log(lam_meas / self.lam))
        S = self.P[2, 2] + sigma ** 2
        K = self.P[:, 2] / S
        self.c = self.c + K[:2] * y
        self.l += float(K[2] * y)
        self.P = self.P - np.outer(K, self.P[2, :])

    def zero_velocity(self, sigma: float = 0.0):
        """Phone known to be at rest (or just put down): v = 0 now."""
        self.c = -self.V.copy()
        self.P[:2, :2] = np.eye(2) * sigma ** 2
        self.P[:2, 2] = 0
        self.P[2, :2] = 0

    def predict(self, t: int, a_world: np.ndarray):
        if self.t is not None:
            dt = (t - self.t) / 1e9
            if 0 < dt < 0.05:
                Vn = self.V + a_world * dt
                self.PV = self.PV + (self.V + Vn) / 2 * dt
                self.C = self.C + self.c * dt
                self.V = Vn
                self.P[:2, :2] += np.eye(2) * self.q_acc ** 2 * dt
                self.P[2, 2] += self.q_l ** 2 * dt
        self.t = t
        self.hist.append((t, self.V.copy(), self.PV.copy(), self.C.copy()))

    def _interval_mean_V(self, t0: int, t1: int) -> np.ndarray | None:
        h = self.hist
        if not h or t1 <= t0 or h[0][0] > t0:
            return None
        ts = np.fromiter((r[0] for r in h), np.int64, len(h))
        PV = np.array([r[2] for r in h])
        if t1 > ts[-1]:
            return None
        p0 = np.array([np.interp(t0, ts, PV[:, k]) for k in range(2)])
        p1 = np.array([np.interp(t1, ts, PV[:, k]) for k in range(2)])
        return (p1 - p0) / ((t1 - t0) / 1e9)

    def position_at(self, t: int) -> np.ndarray:
        """Filter position at a past time t (or now): PV(t) + C(now) - c·(now - t)."""
        h = self.hist
        if t >= h[-1][0]:
            return self.PV + self.C
        ts = np.fromiter((r[0] for r in h), np.int64, len(h))
        PV = np.array([r[2] for r in h])
        pv = np.array([np.interp(t, ts, PV[:, k]) for k in range(2)])
        return pv + self.C - self.c * ((self.t - t) / 1e9)

    def camera(self, t0: int, t1: int, d_rel: np.ndarray, sigma_rel: float) -> bool:
        """Frame interval [t0, t1] (IMU clock), translation d_rel (relative units, world frame), its std per axis.
        Returns True if accepted."""
        if not self.ready:
            return False
        Vbar = self._interval_mean_V(t0, t1)
        if Vbar is None:
            return False
        dt = (t1 - t0) / 1e9
        z = d_rel / dt
        lam = self.lam
        va = Vbar + self.c
        h = va / lam
        H = np.zeros((2, 3))
        H[0, 0] = H[1, 1] = 1 / lam
        if self.learn_scale:
            H[:, 2] = -va / lam
        Rm = np.eye(2) * (sigma_rel / dt) ** 2
        y = z - h
        S = H @ self.P @ H.T + Rm
        Si = np.linalg.inv(S)
        if float(y @ Si @ y) > self.gate ** 2:
            self.stats["rejected"] += 1
            self.rejected_run += 1
            if self.rejected_run >= self.resync_frames:
                # lost lock (e.g. long bump, wrong scale): trust the camera again
                self.c = z * lam - Vbar
                self.P[:2, :2] = np.eye(2) * ((sigma_rel / dt) * lam) ** 2
                self.P[:2, 2] = 0
                self.P[2, :2] = 0
                self.rejected_run = 0
                self.stats["resyncs"] += 1
            return False
        self.rejected_run = 0
        K = self.P @ H.T @ Si
        dx = K @ y
        self.c = self.c + dx[:2]
        self.l += float(dx[2])
        self.P = (np.eye(3) - K @ H) @ self.P
        self.P = (self.P + self.P.T) / 2
        # the corrected offset also applies between the measured interval and now
        self.C = self.C + dx[:2] * ((self.t - (t0 + t1) / 2) / 1e9)
        self.stats["updates"] += 1
        return True
