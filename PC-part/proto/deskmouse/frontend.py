"""Vision front end (DESIGN.md §6.2) with per-track inverse depth (notes/DECISIONS.md).

Planar motion + gyro de-rotation means every static point's image displacement is parallel:
    d_i = rho_i * w + dpsi * J n_i + sigma * n_i
where n_i is the de-rotated normalized position in the previous frame, rho_i the track's inverse
depth (relative units, common unknown scale), w the shared translation term, dpsi a residual
rotation and sigma a scale change (non-zero only when the phone lifts or tilts). J = 90° rotation.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import FrontEndConfig

J = np.array([[0.0, -1.0], [1.0, 0.0]])
_M64 = (1 << 64) - 1


class SplitMix64:
    """Tiny PRNG shared with the C++ core (PC-part/core/src/frontend.cpp) so both draw the same RANSAC samples."""

    def __init__(self, seed: int = 0):
        self.s = seed & _M64

    def next(self) -> int:
        self.s = (self.s + 0x9E3779B97F4A7C15) & _M64
        z = self.s
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & _M64
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & _M64
        return z ^ (z >> 31)

    def pair(self, m: int) -> tuple[int, int]:
        """Two distinct positions in range(m), m >= 2."""
        a = self.next() % m
        b = self.next() % (m - 1)
        return a, b + (b >= a)


@dataclass
class FrameResult:
    t_ns: int                 # mid-exposure time in IMU clock
    w: np.ndarray             # shared translation term (normalized units, relative-rho scale)
    w_cov: np.ndarray         # 2x2
    dpsi_cam: float           # residual rotation seen by the camera after gyro de-rotation (rad)
    sigma: float              # scale change
    n_tracks: int
    n_inliers: int
    q: float
    scale_anomaly: bool
    valid: bool
    rhos: np.ndarray = field(default_factory=lambda: np.zeros(0))  # established track inverse depths


class Track:
    __slots__ = ("id", "rho", "a", "b", "bad", "age")

    def __init__(self, tid: int, rho: float):
        self.id = tid
        self.rho = rho
        self.a = 0.0   # Σ d·w
        self.b = 0.0   # Σ |w|²
        self.bad = 0
        self.age = 0


class FrontEnd:
    def __init__(self, K: np.ndarray, dist: np.ndarray, size: tuple[int, int], cfg: FrontEndConfig | None = None):
        self.cfg = cfg or FrontEndConfig()
        self.K = K.astype(np.float64)
        # Camera2 distortion is [k1,k2,k3,p1,p2]; OpenCV wants [k1,k2,p1,p2,k3]
        k1, k2, k3, p1, p2 = (list(dist) + [0] * 5)[:5]
        self.dist = np.array([k1, k2, p1, p2, k3], np.float64)
        self.w_img, self.h_img = size
        self.f = 0.5 * (K[0, 0] + K[1, 1])
        self.prev = None
        self.pts = np.zeros((0, 2), np.float32)
        self.tracks: list[Track] = []
        self.next_id = 0
        self.bootstrapped = False
        self.clahe = cv2.createCLAHE(3.0, (8, 8))
        c = self.cfg
        self.lk = dict(winSize=(c.lk_win, c.lk_win), maxLevel=c.lk_levels,
                       criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, c.lk_iters, c.lk_eps))
        self.rng = SplitMix64(0)

    # ------------------------------------------------------------------ helpers
    def _prep(self, y: np.ndarray) -> np.ndarray:
        if self.cfg.clahe_always or np.median(y) < self.cfg.clahe_below_median:
            return self.clahe.apply(y)
        return y

    def _normalize(self, p: np.ndarray) -> np.ndarray:
        if len(p) == 0:
            return p.reshape(0, 2).astype(np.float64)
        return cv2.undistortPoints(p.reshape(-1, 1, 2).astype(np.float64), self.K, self.dist).reshape(-1, 2)

    def _rotate_px(self, p: np.ndarray, ang: float) -> np.ndarray:
        cx, cy = self.K[0, 2], self.K[1, 2]
        c, s = np.cos(ang), np.sin(ang)
        q = p - (cx, cy)
        return np.stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]], 1) + (cx, cy)

    def _replenish(self, img: np.ndarray):
        c = self.cfg
        if len(self.pts) >= c.min_tracks:
            return
        mask = np.full(img.shape, 255, np.uint8)
        sat = (img >= c.saturation).astype(np.uint8)
        if sat.any():
            sat = cv2.dilate(sat, np.ones((2 * c.saturation_margin + 1,) * 2, np.uint8))
            mask[sat > 0] = 0
        for x, y in self.pts:
            cv2.circle(mask, (int(x), int(y)), c.gftt_min_dist, 0, -1)
        need = c.max_tracks - len(self.pts)
        # grid: detect per cell so features spread over the image
        gh, gw = img.shape[0] // c.grid, img.shape[1] // c.grid
        counts = np.zeros((c.grid, c.grid), int)
        for x, y in self.pts:
            counts[min(int(y) // gh, c.grid - 1), min(int(x) // gw, c.grid - 1)] += 1
        per_cell = max(1, need // (c.grid * c.grid))
        new = []
        for gy in range(c.grid):
            for gx in range(c.grid):
                if counts[gy, gx] >= per_cell:
                    continue
                sub = mask[gy * gh:(gy + 1) * gh, gx * gw:(gx + 1) * gw]
                p = cv2.goodFeaturesToTrack(img[gy * gh:(gy + 1) * gh, gx * gw:(gx + 1) * gw], per_cell - counts[gy, gx],
                                            c.gftt_quality, c.gftt_min_dist, mask=sub)
                if p is not None:
                    new.append(p.reshape(-1, 2) + (gx * gw, gy * gh))
        if not new:
            return
        new = np.concatenate(new)[:need].astype(np.float32)
        known = [t.rho for t in self.tracks if t.b * self.f**2 >= c.rho_min_motion_px**2]
        rho0 = float(np.median(known)) if known else 1.0
        for _ in new:
            self.tracks.append(Track(self.next_id, rho0))
            self.next_id += 1
        self.pts = np.concatenate([self.pts, new]) if len(self.pts) else new

    # ------------------------------------------------------------------ model fit
    @staticmethod
    def _design(rho: np.ndarray, n: np.ndarray, with_sigma: bool) -> np.ndarray:
        """Rows for unknowns [wx, wy, dpsi(, sigma)] from d_i = rho_i w + dpsi J n_i + sigma n_i."""
        N = len(rho)
        A = np.zeros((2 * N, 4 if with_sigma else 3))
        A[0::2, 0] = rho
        A[1::2, 1] = rho
        A[0::2, 2] = -n[:, 1]
        A[1::2, 2] = n[:, 0]
        if with_sigma:
            A[0::2, 3] = n[:, 0]
            A[1::2, 3] = n[:, 1]
        return A

    def _fit(self, rho, n, d, usable):
        """RANSAC over usable tracks, then LS refine with sigma. Returns params, inlier mask (all tracks)."""
        c = self.cfg
        thr = c.inlier_px / self.f
        idx = np.where(usable)[0]
        if len(idx) < 3:
            return None, np.zeros(len(rho), bool)
        dflat = d.reshape(-1)
        # All hypotheses at once: each from 2 tracks (4 equations, unknowns w, dpsi), batched normal equations.
        H = c.ransac_iters
        s = idx[np.array([self.rng.pair(len(idx)) for _ in range(H)])]  # (H, 2)
        A = np.zeros((H, 4, 3))
        A[:, 0::2, 0] = rho[s]
        A[:, 1::2, 1] = rho[s]
        A[:, 0::2, 2] = -n[s, 1]
        A[:, 1::2, 2] = n[s, 0]
        b = d[s].reshape(H, 4)
        AtA = A.transpose(0, 2, 1) @ A + np.eye(3) * 1e-12
        x = np.linalg.solve(AtA, (A.transpose(0, 2, 1) @ b[:, :, None]))[:, :, 0]  # (H, 3)
        pred = rho[None, :, None] * x[:, None, :2] + x[:, None, 2:3] * (n @ J.T)[None]
        res = np.linalg.norm(pred - d[None], axis=2)  # (H, N)
        inl_all = (res < thr) & usable[None]
        counts = inl_all.sum(1)
        best_n = int(counts.max())
        best = inl_all[int(counts.argmax())]
        if best_n < 3:
            return None, np.zeros(len(rho), bool)
        x = None
        inl = best
        for _ in range(2):  # refine, then recompute inliers once
            A = self._design(rho[inl], n[inl], True)
            x, *_ = np.linalg.lstsq(A, d[inl].reshape(-1), rcond=None)
            res = np.linalg.norm((self._design(rho, n, True) @ x - dflat).reshape(-1, 2), axis=1)
            inl = res < thr
            if inl.sum() < 3:
                return None, inl
        return x, inl

    # ------------------------------------------------------------------ main
    def process(self, y: np.ndarray, t_ns: int, dpsi_gyro: float) -> FrameResult:
        """y: Y plane; t_ns: mid-exposure time (IMU clock); dpsi_gyro: device-gyro z integrated since last frame."""
        c = self.cfg
        img = self._prep(y)
        empty = FrameResult(t_ns, np.zeros(2), np.eye(2), 0.0, 0.0, 0, 0, 0.0, False, False)
        if self.prev is None or len(self.pts) == 0:
            self.prev = img
            self._replenish(img)
            return empty

        ang = c.rot_sign * dpsi_gyro
        guess = self._rotate_px(self.pts.astype(np.float64), ang).astype(np.float32)
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev, img, self.pts.reshape(-1, 1, 2), guess.reshape(-1, 1, 2).copy(),
                                             flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **self.lk)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(img, self.prev, p1, self.pts.reshape(-1, 1, 2).copy(),
                                               flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **self.lk)
        p1 = p1.reshape(-1, 2)
        fb = np.linalg.norm(p0r.reshape(-1, 2) - self.pts, axis=1)
        inside = (p1[:, 0] >= 0) & (p1[:, 1] >= 0) & (p1[:, 0] < self.w_img) & (p1[:, 1] < self.h_img)
        ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < c.fb_max_px) & inside
        tracks = [t for t, k in zip(self.tracks, ok) if k]
        p0, p1 = self.pts[ok], p1[ok]

        n0 = self._normalize(p0)
        n1 = self._normalize(p1)
        cs, sn = np.cos(ang), np.sin(ang)
        n0r = n0 @ np.array([[cs, sn], [-sn, cs]])  # rotate by +ang
        d = n1 - n0r
        rho = np.array([t.rho for t in tracks])
        known = np.array([t.b * self.f**2 >= c.rho_min_motion_px**2 for t in tracks], bool)
        usable = known if known.sum() >= 8 else np.ones(len(tracks), bool)  # bootstrap: all tracks at rho=1

        x, inl = self._fit(rho, n0r, d, usable)
        result = empty
        result.t_ns = t_ns
        result.n_tracks = len(tracks)
        keep = np.ones(len(tracks), bool)
        if x is not None:
            w, dpsi, sigma = x[:2], x[2], x[3]
            dc = d - dpsi * (n0r @ J.T) - sigma * n0r  # displacement explained by translation only
            # per-track inverse depth: least squares over the track's history (only when moving)
            w2 = float(w @ w)
            moving = w2 * self.f**2 > 0.05**2
            for i, t in enumerate(tracks):
                t.age += 1
                if moving:
                    t.a += float(dc[i] @ w)
                    t.b += w2
                    if t.b > 0:
                        t.rho = t.a / t.b
                res = np.linalg.norm(dc[i] - t.rho * w) * self.f
                if known[i] and moving and res > 2 * c.inlier_px:
                    t.bad += 1
                elif res < c.inlier_px:
                    t.bad = 0
                if t.bad >= c.bad_frames_drop or t.rho <= 0 and t.b * self.f**2 > c.rho_min_motion_px**2:
                    keep[i] = False
            n_in = int((inl & usable).sum())
            r = (d[inl] - (self._design(rho[inl], n0r[inl], True) @ x).reshape(-1, 2))
            cov = (np.cov(r.T) if len(r) > 2 else np.eye(2) * 1e-6) / max(n_in, 1)
            cov = np.maximum(cov, np.eye(2) * (c.cov_floor_px / self.f) ** 2)
            q = float(np.clip((n_in - c.q_inliers_zero) / (c.q_inliers_full - c.q_inliers_zero), 0, 1))
            if not self.bootstrapped and moving and n_in >= c.q_inliers_zero:
                self.bootstrapped = True
            result = FrameResult(t_ns, w.copy(), cov, float(dpsi), float(sigma), len(tracks), n_in, q,
                                 bool(abs(sigma) > c.scale_anomaly), True,
                                 np.array([t.rho for t, k in zip(tracks, known) if k]))

        self.tracks = [t for t, k in zip(tracks, keep) if k]
        self.pts = p1[keep].astype(np.float32)
        self.prev = img
        self._replenish(img)
        return result
