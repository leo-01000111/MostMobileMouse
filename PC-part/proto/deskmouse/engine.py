"""Real-time MVP engine: IMU + camera frames in, mouse events out.

Simplified stand-in for the M2 EKF (DESIGN.md §6.3–6.8), built on the M1 front end:
  * motion: front-end translation term w (relative units) × scale λ (m per unit), rotated by gyro yaw;
  * scale λ: ceiling gauge at start, then corrected after every stroke by comparing the stroke's
    vision displacement with an IMU-only (ZUPT at both ends) displacement;
  * stillness (IMU + vision) → deadband; taps (accel-z jerk) → clicks; twist → scroll; gyro tilt → lift.
Inputs are fed in arrival order; call on_imu() for every sample and on_frame() for every frame.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .config import FrontEndConfig
from .frontend import FrontEnd


@dataclass
class EngineConfig:
    # geometry (S24, measured from place-1 recordings)
    M_ci: list = field(default_factory=lambda: [[0.0, -1.0], [1.0, 0.0]])  # image = M_ci · body XY
    rot_sign: float = 1.0
    clock_offset_ns: int = 2_000_000
    r_cam: tuple[float, float] = (-0.022, 0.044)  # main lens from phone centre, body frame (m), S24 spec estimate
    mount_yaw_deg: float = 0.0   # rotate output if the phone lies sideways (90 = top edge pointing left)
    aspect: float = 1.0          # extra gain on the vertical axis (auto-tuned by the game)
    world_aligned: bool = False  # body-aligned like a real mouse; world-aligned drifted with yaw errors after lifts
    instant_left: bool = False   # left click on the tap itself (no double-tap wait); double tap then also sends right
    # output mapping (§6.8)
    dpi: float = 800.0
    g_max: float = 2.5
    v_lo: float = 0.05
    v_hi: float = 0.5
    # stillness (§6.4)
    still_gyro: float = 0.03
    still_acc_std: float = 0.08
    still_ms: float = 60.0
    still_flow_px: float = 0.15
    gyro_bias_tau_s: float = 3.0
    # scale
    ceiling_h: float = 1.9
    ceiling_percentile: float = 15.0
    scale_alpha: float = 0.35
    stroke_min_m: float = 0.03
    stroke_max_s: float = 2.0
    stroke_min_peak_speed: float = 0.08
    # taps (§6.6, threshold lowered for soft pads, notes/DECISIONS.md)
    tap_thr_min: float = 0.4
    tap_mad_k: float = 8.0
    tap_refractory_ms: float = 100.0
    tap_suppress_before_ms: float = 15.0
    tap_suppress_after_ms: float = 120.0
    double_tap_ms: float = 250.0
    # scroll (§6.7)
    scroll_enabled: bool = True   # twist-to-scroll; off = twisting never freezes the cursor
    scroll_omega: float = 0.3
    scroll_enter_ms: float = 40.0
    scroll_exit_omega: float = 0.1
    scroll_exit_ms: float = 150.0
    scroll_step_deg: float = 4.0
    scroll_sign: float = 1.0
    scroll_max_ref_speed: float = 0.03
    # lift (§6.5)
    lift_tilt_deg: float = 4.0
    lift_scale_anomaly: float = 0.015


@dataclass
class Event:
    t_ns: int
    kind: str          # "move", "left", "right", "wheel", "lift", "land"
    dx: int = 0
    dy: int = 0
    wheel: int = 0


class Engine:
    def __init__(self, K, dist, size, cfg: EngineConfig | None = None, fe_cfg: FrontEndConfig | None = None,
                 lam0: float | None = None):
        """lam0: scale (m per relative unit) remembered from a previous session, if any."""
        self.cfg = cfg or EngineConfig()
        c = self.cfg
        fe_cfg = fe_cfg or FrontEndConfig()
        fe_cfg.rot_sign = c.rot_sign
        self.fe = FrontEnd(K, dist, size, fe_cfg)
        self.Minv = np.linalg.inv(np.array(c.M_ci, float))
        self.r_cam = np.array(c.r_cam, float)
        # IMU state
        self.gyro_bias = np.zeros(3)
        self.bias_init: list = []
        self.bias_ok = False
        self.gyro_t = deque(maxlen=1500)   # (t, Z) cumulative integral of bias-corrected gyro z (device)
        self.Zcum = 0.0
        self.last_gyro_t = None
        self.last_gyro = np.zeros(3)
        self.tilt = np.zeros(2)            # integrated roll/pitch since last rest (rad)
        self.recent_gyro = deque(maxlen=64)
        self.recent_acc = deque(maxlen=64)
        self.acc_buf = deque(maxlen=3000)   # (t, body X, body Y) specific force for stroke integration
        self.rest_acc = None
        self.still = False
        self.still_since = None
        self.imu_still_since = None
        self.last_flow_px = 0.0
        # taps
        self.jerk = deque(maxlen=500)
        self.last_az = None
        self.tap_thr = c.tap_thr_min
        self.last_tap_t = -1 << 62
        self.pending_tap_t = None
        self.suppress_until = -1 << 62
        self.suppress_from = 1 << 62
        # scroll
        self.scroll = False
        self.scroll_cand_since = None
        self.scroll_quiet_since = None
        self.scroll_acc = 0.0
        self.scroll_psi_last = 0.0
        # lift
        self.lifted = False
        self.anomaly_frames = 0
        # vision / output
        self.lam = lam0                    # metres per relative unit
        self.lam_gauge: list = []
        self.stroke_vis = np.zeros(2)
        self.stroke_start = None
        self.stroke_peak = 0.0
        self.prev_Z = None
        self.carry = np.zeros(2)
        self.last_ref_speed = 0.0
        self.events: list[Event] = []
        self.stats = dict(frames=0, taps=0, scale_updates=0, lifts=0)
        self.frame_log: list | None = None  # set to [] to log front-end results for replay

    # ------------------------------------------------------------------ IMU
    def psi_at(self, t_ns: int) -> float:
        """Body yaw (rad) at time t from gyro: ψ = -∫gyro_z_dev."""
        if not self.gyro_t:
            return 0.0
        ts, zs = self.gyro_t[-1]
        if t_ns >= ts:
            return -zs
        arr = np.array(self.gyro_t)
        return -float(np.interp(t_ns, arr[:, 0], arr[:, 1]))

    def on_imu(self, t_ns: int, kind: str, x: float, y: float, z: float):
        c = self.cfg
        if kind == "gyro":
            g = np.array([x, y, z])
            # bias: set from the first detected still window, then EMA while still (§6.1)
            self.bias_init.append((t_ns, g))
            if len(self.bias_init) > 100:
                self.bias_init.pop(0)
            gc = g - self.gyro_bias
            if self.last_gyro_t is not None:
                dt = (t_ns - self.last_gyro_t) / 1e9
                self.Zcum += gc[2] * dt
                # tilt: device x/y rates → body roll/pitch (sign irrelevant for magnitude)
                self.tilt += gc[:2] * dt
            self.last_gyro_t = t_ns
            self.last_gyro = gc
            self.gyro_t.append((t_ns, self.Zcum))
            self.recent_gyro.append((t_ns, np.linalg.norm(gc)))
            if self.still:
                if not self.bias_ok:
                    self.gyro_bias = np.mean([v for (_, v) in self.bias_init[-50:]], axis=0)
                    self.bias_ok = True
                    self.Zcum = self.Zcum  # yaw keeps its value; only future integration changes
                else:
                    a = 1 / 500 / c.gyro_bias_tau_s
                    self.gyro_bias = (1 - a) * self.gyro_bias + a * g
            self._scroll_update(t_ns, gc[2])
            self._lift_update(t_ns)
        elif kind == "accel":
            bx, by = -x, y  # body X, Y specific force
            self.acc_buf.append((t_ns, bx, by))
            self.recent_acc.append((t_ns, bx, by))
            self._tap_update(t_ns, z)
            self._still_update(t_ns)

    def _still_update(self, t_ns):
        c = self.cfg
        win = c.still_ms * 1e6
        g = [v for (t, v) in self.recent_gyro if t > t_ns - win]
        if not self.bias_ok:  # bias unknown: judge by the spread of the raw rate instead of its size
            raw = np.array([v for (t, v) in self.bias_init if t > t_ns - win])
            g = [float(np.abs(raw - raw.mean(0)).max())] if len(raw) > 5 else []
        a = np.array([(bx, by) for (t, bx, by) in self.recent_acc if t > t_ns - win])
        imu_still = bool(g) and max(g) < c.still_gyro and len(a) > 5 and a.std(0).max() < c.still_acc_std
        if imu_still:
            if self.imu_still_since is None:
                self.imu_still_since = t_ns
        else:
            self.imu_still_since = None
        was = self.still
        self.still = (self.imu_still_since is not None and t_ns - self.imu_still_since >= win
                      and self.last_flow_px < c.still_flow_px)
        if self.still:
            self.rest_acc = a.mean(0) if len(a) else self.rest_acc
            self.tilt[:] = 0.0
        if self.still and not was:
            self._stroke_end(t_ns)
        if not self.still and was:
            self.stroke_start = t_ns
            self.stroke_vis[:] = 0
            self.stroke_peak = 0.0

    def _tap_update(self, t_ns, az):
        c = self.cfg
        if self.last_az is None:
            self.last_az = az
            return
        j = abs(az - self.last_az)
        self.last_az = az
        if t_ns - self.last_tap_t > 300e6:
            self.jerk.append(j)
            if len(self.jerk) % 50 == 0:
                arr = np.fromiter(self.jerk, float)
                med = np.median(arr)
                self.tap_thr = max(c.tap_thr_min, med + c.tap_mad_k * np.median(np.abs(arr - med)))
        if j > self.tap_thr and t_ns - self.last_tap_t > c.tap_refractory_ms * 1e6 and not self.lifted and not self.scroll:
            self.last_tap_t = t_ns
            self.stats["taps"] += 1
            self.suppress_from = int(t_ns - c.tap_suppress_before_ms * 1e6)
            self.suppress_until = int(t_ns + c.tap_suppress_after_ms * 1e6)
            if c.instant_left:
                if self.pending_tap_t is not None and t_ns - self.pending_tap_t < c.double_tap_ms * 1e6:
                    self.events.append(Event(t_ns, "right"))
                    self.pending_tap_t = None
                else:
                    self.events.append(Event(t_ns, "left"))
                    self.pending_tap_t = t_ns
                return
            if self.pending_tap_t is not None and t_ns - self.pending_tap_t < c.double_tap_ms * 1e6:
                self.events.append(Event(t_ns, "right"))
                self.pending_tap_t = None
            else:
                self.pending_tap_t = t_ns

    def poll(self, t_ns: int):
        """Fire a pending single tap once the double-tap window has passed."""
        if self.cfg.instant_left:
            if self.pending_tap_t is not None and t_ns - self.pending_tap_t >= self.cfg.double_tap_ms * 1e6:
                self.pending_tap_t = None
            return
        if self.pending_tap_t is not None and t_ns - self.pending_tap_t >= self.cfg.double_tap_ms * 1e6:
            self.events.append(Event(t_ns, "left"))
            self.pending_tap_t = None

    def _scroll_update(self, t_ns, wz):
        c = self.cfg
        if not c.scroll_enabled:
            self.scroll = False
            return
        if not self.scroll:
            calibrated = self.stats["scale_updates"] > 0
            if self.bias_ok and abs(wz) > c.scroll_omega and (not calibrated or self.last_ref_speed < c.scroll_max_ref_speed):
                self.scroll_cand_since = self.scroll_cand_since or t_ns
                if t_ns - self.scroll_cand_since > c.scroll_enter_ms * 1e6:
                    self.scroll = True
                    self.scroll_acc = 0.0
                    self.scroll_psi_last = -self.Zcum
                    self.scroll_quiet_since = None
            else:
                self.scroll_cand_since = None
            return
        psi = -self.Zcum
        self.scroll_acc += psi - self.scroll_psi_last
        self.scroll_psi_last = psi
        step = np.radians(c.scroll_step_deg)
        n = int(self.scroll_acc / step)
        if n:
            self.scroll_acc -= n * step
            self.events.append(Event(t_ns, "wheel", wheel=int(c.scroll_sign * n)))
        if abs(wz) < c.scroll_exit_omega:
            self.scroll_quiet_since = self.scroll_quiet_since or t_ns
            if t_ns - self.scroll_quiet_since > c.scroll_exit_ms * 1e6:
                self.scroll = False
                self.scroll_cand_since = None
        else:
            self.scroll_quiet_since = None

    def _lift_update(self, t_ns):
        c = self.cfg
        tilt = np.degrees(np.linalg.norm(self.tilt))
        if not self.lifted and tilt > c.lift_tilt_deg:
            self.lifted = True
            self.stats["lifts"] += 1
            self.events.append(Event(t_ns, "lift"))
        elif self.lifted and self.still:
            self.lifted = False
            self.events.append(Event(t_ns, "land"))

    # ------------------------------------------------------------------ strokes / scale
    def _stroke_end(self, t_ns):
        c = self.cfg
        if self.stroke_start is None or self.rest_acc is None:
            return
        dur = (t_ns - self.stroke_start) / 1e9
        vis = self.stroke_vis.copy()
        self.stroke_start = None
        if dur > c.stroke_max_s or dur < 0.1 or np.linalg.norm(vis) == 0:
            return
        arr = np.array([r for r in self.acc_buf if self.stroke_start_bound(r[0], t_ns, dur)])
        if len(arr) < 20:
            return
        tt = arr[:, 0] / 1e9
        acc = arr[:, 1:] - self.rest_acc
        # rotate body accel into world by yaw
        g = np.array(self.gyro_t)
        psi = -np.interp(arr[:, 0], g[:, 0], g[:, 1])
        cs, sn = np.cos(psi), np.sin(psi)
        aw = np.stack([cs * acc[:, 0] - sn * acc[:, 1], sn * acc[:, 0] + cs * acc[:, 1]], 1)
        v = np.concatenate([[[0, 0]], np.cumsum((aw[1:] + aw[:-1]) / 2 * np.diff(tt)[:, None], 0)])
        v -= np.outer((tt - tt[0]) / (tt[-1] - tt[0]), v[-1])
        p = np.concatenate([[[0, 0]], np.cumsum((v[1:] + v[:-1]) / 2 * np.diff(tt)[:, None], 0)])[-1]
        L = np.linalg.norm(p)
        peak = np.linalg.norm(v, axis=1).max()
        if L < c.stroke_min_m or peak < c.stroke_min_peak_speed:
            return
        cosang = float(p @ vis / (L * np.linalg.norm(vis)))
        if cosang < 0.9:
            return
        lam_meas = L / np.linalg.norm(vis)
        a = 1.0 if self.stats["scale_updates"] == 0 else c.scale_alpha
        self.lam = lam_meas if self.lam is None else (1 - a) * self.lam + a * lam_meas
        self.stats["scale_updates"] += 1

    def stroke_start_bound(self, t, t_end, dur):
        return t_end - (dur + 0.06) * 1e9 <= t <= t_end

    # ------------------------------------------------------------------ frames
    def on_frame(self, y: np.ndarray, t_sensor_ns: int, exposure_ns: int = 0, skew_ns: int = 0):
        c = self.cfg
        t = int(t_sensor_ns + exposure_ns // 2 + max(skew_ns, 0) // 2 + c.clock_offset_ns)
        Z = -self.psi_at(t)
        dZ = 0.0 if self.prev_Z is None else Z - self.prev_Z
        psi_prev = -self.prev_Z if self.prev_Z is not None else -Z
        self.prev_Z = Z
        psi = -Z
        res = self.fe.process(y, t, dZ)
        n_rhos = len(res.rhos)
        rho15 = float(np.percentile(res.rhos, c.ceiling_percentile)) if n_rhos else 0.0
        rho_med = float(np.median(res.rhos)) if n_rhos else 1.0
        rec = (t, psi_prev, psi, float(res.w[0]), float(res.w[1]), res.sigma, res.valid, n_rhos, rho15, rho_med, res.n_inliers)
        if self.frame_log is not None:
            self.frame_log.append(rec)
        self.on_result(*rec)

    def on_result(self, t, psi_prev, psi, wx, wy, sigma, valid, n_rhos, rho15, rho_med, n_inliers=0):
        """Everything after the front end; replayable from logged front-end results (game tuning)."""
        c = self.cfg
        self.stats["frames"] += 1
        if not valid:
            return
        w = np.array([wx, wy])
        self.last_flow_px = float(np.linalg.norm(w) * rho_med * self.fe.f)
        # scale: ceiling gauge until the first stroke calibration
        if n_rhos >= 8 and rho15 > 0:
            self.lam_gauge.append(1.0 / (c.ceiling_h * rho15))
            self.lam_gauge = self.lam_gauge[-120:]
        if self.lam is None and len(self.lam_gauge) >= 30:
            self.lam = 1.0 / float(np.median(self.lam_gauge))
        # lift from vision scale change
        self.anomaly_frames = self.anomaly_frames + 1 if abs(sigma) > c.lift_scale_anomaly else 0
        if self.anomaly_frames >= 2 and not self.lifted:
            self.lifted = True
            self.stats["lifts"] += 1
            self.events.append(Event(t, "lift"))

        d_rel_body = -self.Minv @ w
        cs, sn = np.cos(psi), np.sin(psi)
        d_rel_world = np.array([cs * d_rel_body[0] - sn * d_rel_body[1], sn * d_rel_body[0] + cs * d_rel_body[1]])
        if not self.still:
            self.stroke_vis += d_rel_world
        if self.lam is None:
            return
        lens = self.lam * d_rel_world
        # reference point = lens - R(ψ) r_cam
        c0, s0 = np.cos(psi_prev), np.sin(psi_prev)
        lever = np.array([cs * self.r_cam[0] - sn * self.r_cam[1], sn * self.r_cam[0] + cs * self.r_cam[1]]) - \
            np.array([c0 * self.r_cam[0] - s0 * self.r_cam[1], s0 * self.r_cam[0] + c0 * self.r_cam[1]])
        ref = lens - lever
        dt = 1 / 60
        speed = float(np.linalg.norm(ref) / dt)
        self.last_ref_speed = speed
        if (self.still or self.lifted or self.scroll or self.suppress_from <= t <= self.suppress_until):
            return
        # output frame
        if c.world_aligned:
            out = ref
        else:
            out = np.array([cs * ref[0] + sn * ref[1], -sn * ref[0] + cs * ref[1]])
        m = np.radians(c.mount_yaw_deg)
        out = np.array([np.cos(m) * out[0] - np.sin(m) * out[1], (np.sin(m) * out[0] + np.cos(m) * out[1]) * c.aspect])
        s = np.clip((speed - c.v_lo) / (c.v_hi - c.v_lo), 0, 1)
        gain = 1 + (c.g_max - 1) * s * s * (3 - 2 * s)
        counts = out * c.dpi / 0.0254 * gain + self.carry
        n = np.trunc(counts)
        self.carry = counts - n
        if n.any():
            # +X → right, +Y (forward) → up (screen dy negative)
            self.events.append(Event(t, "move", int(n[0]), int(-n[1])))

    def drain(self) -> list[Event]:
        ev, self.events = self.events, []
        return ev
