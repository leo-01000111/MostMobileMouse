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
from .fusion import VelocityKF


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
    # latency compensation: cursor leads by velocity × latency_comp_ms (0 = off)
    latency_comp_ms: float = 0.0  # off: amplified per-frame noise into visible jitter (user test); see notes/MVP.md
    lead_use_imu: bool = True     # refine the camera velocity with accelerometer samples newer than the frame
    lead_max_m: float = 0.03
    lead_vel_alpha: float = 0.5   # smoothing of the camera velocity (per frame)
    # smoothing: 1€ filter on the cursor position (Casiez et al. 2012): heavy when slow, light when fast
    smooth: bool = True
    oe_min_cutoff_hz: float = 1.0
    oe_beta: float = 0.01          # cutoff grows by beta · speed (counts/s); chosen on replay: shake -28 %, ~10 px lag
    oe_d_cutoff_hz: float = 1.0
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
    # knocks: a sharp in-plane jolt (hand hitting the phone) makes the image ring for a few frames; mute output
    shock_jerk: float = 1.0          # |Δ in-plane accel| between consecutive samples (m/s², 500 Hz) that counts as a knock
    shock_suppress_before_ms: float = 15.0
    shock_suppress_after_ms: float = 70.0
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
    # velocity Kalman filter (fusion.py, notes/DECISIONS.md 2026-10-09); False = the MVP path (still gate, knock hold)
    kf: bool = True
    kf_acc_noise: float = 0.05        # velocity random walk from the accelerometer, m/s per sqrt(s)
    kf_scale_noise: float = 0.002     # log-scale random walk per sqrt(s)
    kf_learn_scale: bool = False      # camera updates also refine the scale (drifted +30 % on the class session)
    kf_scale_sigma0: float = 0.3      # log-scale std when the scale comes from the ceiling gauge
    kf_scale_sigma_cal: float = 0.05  # log-scale std after a calibration run
    kf_stroke_sigma: float = 0.15     # log-scale std of one stroke measurement outside calibration
    kf_cam_sigma_px: float = 0.03     # camera translation noise per frame, image pixels
    kf_cam_sigma_rel: float = 0.1     # plus this fraction of the frame's own flow (motion blur, rolling shutter)
    kf_gate: float = 4.0              # reject a camera frame beyond this many sigma (knock ringing, mismatches)
    kf_resync_frames: int = 6         # after this many rejected frames in a row, trust the camera again
    kf_leash_mm: float = 0.3          # at rest the cursor only follows once the estimate is this far away
    kf_predict_now: bool = False      # output the position at the newest IMU sample instead of the frame time


@dataclass
class Event:
    t_ns: int
    kind: str          # "move", "left", "right", "wheel", "lift", "land"
    dx: int = 0
    dy: int = 0
    wheel: int = 0


class Engine:
    def __init__(self, K, dist, size, cfg: EngineConfig | None = None, fe_cfg: FrontEndConfig | None = None,
                 gauge_ratio: float | None = None):
        """gauge_ratio: calibrated scale ÷ ceiling-gauge scale from a previous session. The front end's relative
        units differ per session, so the raw scale can't be carried over; this ratio can."""
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
        self.last_axy = None
        self.shock_from = 1 << 62
        self.shock_until = -1 << 62
        self.held = np.zeros(2)          # motion held during a knock window (world frame, m)
        self.held_dt = 0.0
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
        self.lam = None                    # metres per relative unit
        self.gauge_ratio0 = gauge_ratio
        self.lam_gauge: list = []
        self.kf = VelocityKF(c.kf_acc_noise, c.kf_scale_noise, c.kf_gate, c.kf_resync_frames, c.kf_learn_scale)
        self.kf_out = None        # output position (world, m): follows the filter, held by the leash at rest
        self.kf_emitted = None    # output position already sent
        self.kf_was_lifted = False
        self.calibrating = False
        self.cal_strokes: list = []
        self.stroke_vis = np.zeros(2)
        self.stroke_start = None
        self.stroke_peak = 0.0
        self.prev_Z = None
        self.carry = np.zeros(2)
        self.last_ref_speed = 0.0
        self.events: list[Event] = []
        self.stats = dict(frames=0, taps=0, scale_updates=0, lifts=0, shocks=0)
        self.frame_log: list | None = None  # set to [] to log front-end results for replay
        self.v_ref = np.zeros(2)   # reference-point velocity (world frame, m/s) for latency compensation
        self.lead = np.zeros(2)    # current lead added to the cursor (m)
        self.last_frame_t = None
        self.pos_raw = np.zeros(2)       # cursor position before smoothing (counts)
        self.pos_f = np.zeros(2)         # smoothed position
        self.pos_sent = np.zeros(2)      # integer counts already sent
        self.speed_f = 0.0
        self.last_emit_t = None

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
            if c.kf:
                a = np.zeros(2)
                if self.rest_acc is not None:
                    ab = np.array([bx, by]) - self.rest_acc
                    psi = -self.Zcum
                    cs, sn = np.cos(psi), np.sin(psi)
                    a = np.array([cs * ab[0] - sn * ab[1], sn * ab[0] + cs * ab[1]])
                self.kf.predict(t_ns, a)
            self.acc_buf.append((t_ns, bx, by))
            self.recent_acc.append((t_ns, bx, by))
            self._shock_update(t_ns, bx, by)
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

    def _shock_update(self, t_ns, bx, by):
        c = self.cfg
        if self.last_axy is not None and np.hypot(bx - self.last_axy[0], by - self.last_axy[1]) > c.shock_jerk:
            if t_ns - c.shock_suppress_before_ms * 1e6 > self.shock_until:  # new window, else extend the open one
                self.shock_from = int(t_ns - c.shock_suppress_before_ms * 1e6)
                self.stats["shocks"] += 1
            self.shock_until = int(t_ns + c.shock_suppress_after_ms * 1e6)
        self.last_axy = (bx, by)

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
        if self.calibrating:
            # calibration (new ceiling): the scale is the median of this run's strokes only, the old prior is ignored
            self.cal_strokes.append(lam_meas)
            self.lam = float(np.median(self.cal_strokes))
            if c.kf:
                self.kf.set_scale(self.lam, c.kf_scale_sigma_cal)
        elif c.kf and self.kf.lam0 is not None:
            self.kf.scale_measurement(lam_meas, c.kf_stroke_sigma)
            self.lam = self.kf.lam
        else:
            a = 1.0 if self.stats["scale_updates"] == 0 else c.scale_alpha
            self.lam = lam_meas if self.lam is None else (1 - a) * self.lam + a * lam_meas
        self.stats["scale_updates"] += 1

    def begin_calibration(self):
        """Start a calibration run: separate strokes with short pauses; each stroke measures the scale."""
        self.calibrating = True
        self.cal_strokes = []

    def end_calibration(self) -> int:
        """End the run; returns the number of strokes it measured (the scale is their median)."""
        self.calibrating = False
        return len(self.cal_strokes)

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
            self.lam = (self.gauge_ratio0 or 1.0) / float(np.median(self.lam_gauge))
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
        if c.kf:
            self._kf_frame(t, psi_prev, psi, d_rel_world, rho_med)
            return
        if self.lam is None:
            return
        lens = self.lam * d_rel_world
        # reference point = lens - R(ψ) r_cam
        c0, s0 = np.cos(psi_prev), np.sin(psi_prev)
        lever = np.array([cs * self.r_cam[0] - sn * self.r_cam[1], sn * self.r_cam[0] + cs * self.r_cam[1]]) - \
            np.array([c0 * self.r_cam[0] - s0 * self.r_cam[1], s0 * self.r_cam[0] + c0 * self.r_cam[1]])
        ref = lens - lever
        dt = 1 / 60 if self.last_frame_t is None else float(np.clip((t - self.last_frame_t) / 1e9, 1 / 120, 0.1))
        self.last_frame_t = t
        speed = float(np.linalg.norm(ref) / dt)
        self.last_ref_speed = speed
        if self.scroll or self.suppress_from <= t <= self.suppress_until:
            return  # lead left untouched while output is suppressed
        if self.shock_from <= t <= self.shock_until and not self.lifted:
            # knock: hold the motion and release the net once the image stops ringing (the ringing cancels, a real
            # stroke survives, delayed by the window)
            self.held += ref
            self.held_dt += dt
            return
        released = False
        if self.held_dt > 0:
            if not self.lifted:
                # a stroke that ended in the knock (phone hits a stop) still gets its motion; a still frame adds none
                ref = self.held.copy() if self.still else ref + self.held
                dt = self.held_dt if self.still else dt + self.held_dt
                speed = float(np.linalg.norm(ref) / dt)
                released = True
            self.held[:] = 0
            self.held_dt = 0.0
        if (self.still or self.lifted) and not released:
            self._emit(t, -self.lead)  # give back whatever lead is left, then nothing
            self.lead[:] = 0
            self.v_ref[:] = 0
            return
        self._output(t, ref, dt, speed, psi)

    def _output(self, t, ref, dt, speed, psi):
        """Reference-point displacement (world, m) -> output frame, pointer acceleration, emit."""
        c = self.cfg
        cs, sn = np.cos(psi), np.sin(psi)
        if c.world_aligned:
            out = ref
        else:
            out = np.array([cs * ref[0] + sn * ref[1], -sn * ref[0] + cs * ref[1]])
        m = np.radians(c.mount_yaw_deg)
        out = np.array([np.cos(m) * out[0] - np.sin(m) * out[1], (np.sin(m) * out[0] + np.cos(m) * out[1]) * c.aspect])
        s = np.clip((speed - c.v_lo) / (c.v_hi - c.v_lo), 0, 1)
        gain = 1 + (c.g_max - 1) * s * s * (3 - 2 * s)
        k = c.dpi / 0.0254 * gain
        counts = out * k
        self._emit(t, counts + self._lead_step(t, counts / dt, psi, k))

    def _kf_frame(self, t, psi_prev, psi, d_rel_world, rho_med):
        """Kalman path: the camera corrects the filter; the cursor follows the filter's position."""
        c = self.cfg
        kf = self.kf
        t_prev = self.last_frame_t
        dt = 1 / 60 if t_prev is None else float(np.clip((t - t_prev) / 1e9, 1 / 120, 0.1))
        self.last_frame_t = t
        if self.lam is None:
            return
        if kf.lam0 is None:
            kf.set_scale(self.lam, c.kf_scale_sigma0)
        if t_prev is not None and not self.lifted:
            sig_px = c.kf_cam_sigma_px + c.kf_cam_sigma_rel * self.last_flow_px
            kf.camera(t_prev, t, d_rel_world, sig_px / (self.fe.f * max(rho_med, 1e-6)))
        self.lam = kf.lam
        if not kf.ready:
            return
        p = kf.position_at(kf.t if c.kf_predict_now else t)
        landed = self.kf_was_lifted and not self.lifted
        self.kf_was_lifted = self.lifted
        if landed:
            kf.zero_velocity()
            p = kf.position_at(kf.t if c.kf_predict_now else t)
        if self.kf_out is None or self.lifted or landed:
            self.kf_out = p.copy()
            self.kf_emitted = p.copy()
            return
        if self.scroll or self.suppress_from <= t <= self.suppress_until:
            self.kf_out = p.copy()  # suppressed motion is dropped, not caught up later
            self.kf_emitted = p.copy()
            return
        if self.still:
            d = p - self.kf_out
            n = float(np.linalg.norm(d))
            leash = c.kf_leash_mm / 1000
            if n > leash:
                self.kf_out = self.kf_out + d * (1 - leash / n)
        else:
            self.kf_out = p.copy()
        move = self.kf_out - self.kf_emitted
        self.kf_emitted = self.kf_out.copy()
        if not move.any():
            return
        cs, sn = np.cos(psi), np.sin(psi)
        c0, s0 = np.cos(psi_prev), np.sin(psi_prev)
        lever = np.array([cs * self.r_cam[0] - sn * self.r_cam[1], sn * self.r_cam[0] + cs * self.r_cam[1]]) - \
            np.array([c0 * self.r_cam[0] - s0 * self.r_cam[1], s0 * self.r_cam[0] + c0 * self.r_cam[1]])
        ref = move if self.still else move - lever
        speed = float(np.linalg.norm(ref) / dt)
        self.last_ref_speed = speed
        self._output(t, ref, dt, speed, psi)

    def _to_output(self, v_world, psi):
        """World-frame vector → output frame (body alignment, mount yaw, aspect), metres."""
        c = self.cfg
        cs, sn = np.cos(psi), np.sin(psi)
        v = v_world if c.world_aligned else np.array([cs * v_world[0] + sn * v_world[1], -sn * v_world[0] + cs * v_world[1]])
        m = np.radians(c.mount_yaw_deg)
        return np.array([np.cos(m) * v[0] - np.sin(m) * v[1], (np.sin(m) * v[0] + np.cos(m) * v[1]) * c.aspect])

    def _emit(self, t, counts):
        c = self.cfg
        self.pos_raw = self.pos_raw + counts
        if c.smooth:
            dt = 1 / 60 if self.last_emit_t is None else float(np.clip((t - self.last_emit_t) / 1e9, 1 / 240, 0.1))

            def alpha(fc):
                return 1.0 / (1.0 + 1.0 / (2 * np.pi * fc * dt))
            v = float(np.linalg.norm(self.pos_raw - self.pos_f) / dt)
            self.speed_f += alpha(c.oe_d_cutoff_hz) * (v - self.speed_f)
            self.pos_f = self.pos_f + alpha(c.oe_min_cutoff_hz + c.oe_beta * self.speed_f) * (self.pos_raw - self.pos_f)
        else:
            self.pos_f = self.pos_raw.copy()
        self.last_emit_t = t
        n = np.trunc(self.pos_f - self.pos_sent)
        self.pos_sent = self.pos_sent + n
        if n.any():
            # +X → right, +Y (forward) → up (screen dy negative)
            self.events.append(Event(t, "move", int(n[0]), int(-n[1])))

    def _lead_step(self, t, v_counts, psi, k) -> np.ndarray:
        """Change of the latency lead, in output counts (after gain), so it always returns exactly to zero."""
        c = self.cfg
        if c.latency_comp_ms <= 0:
            return np.zeros(2)
        self.v_ref = c.lead_vel_alpha * v_counts + (1 - c.lead_vel_alpha) * self.v_ref  # counts/s
        v = self.v_ref.copy()
        if c.lead_use_imu and self.rest_acc is not None and self.acc_buf:
            newer = [(bx, by) for (ts, bx, by) in self.acc_buf if ts > t]
            if newer:
                a_body = np.sum(np.array(newer) - self.rest_acc, axis=0) * 0.002  # Δv since the frame, m/s, body
                cs, sn = np.cos(psi), np.sin(psi)
                a_world = np.array([cs * a_body[0] - sn * a_body[1], sn * a_body[0] + cs * a_body[1]])
                v = v + self._to_output(a_world, psi) * k
        target = v * c.latency_comp_ms / 1000
        lim = c.lead_max_m * c.dpi / 0.0254
        nrm = np.linalg.norm(target)
        if nrm > lim:
            target *= lim / nrm
        d = target - self.lead
        self.lead = target
        return d

    def gauge_ratio(self) -> float | None:
        """Calibrated scale relative to the ceiling gauge (store this between sessions)."""
        if not self.lam or not self.lam_gauge or self.stats["scale_updates"] == 0:
            return None
        return float(self.lam * np.median(self.lam_gauge))

    def drain(self) -> list[Event]:
        ev, self.events = self.events, []
        return ev
