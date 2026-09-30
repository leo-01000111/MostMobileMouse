"""Desk Mouse aim lab: osu-style levels to compare the phone mouse with a real mouse and auto-tune the phone.

usage:  python PC-part/game/aimlab.py --input mouse     (baseline with your normal mouse)
        python PC-part/game/aimlab.py --input phone     (phone face-down, "Mouse mode" on the phone, USB)
SPACE starts each level, ESC quits. After a phone session the tuner runs and saves phone_params.json.
Sessions are saved in PC-part/game/sessions/ (gitignored).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import random
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pygame

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "proto"))
sys.path.insert(0, str(HERE.parent / "receiver"))

W, H = 1600, 900
CX, CY = W // 2, H // 2
BG = (18, 20, 26)
FG = (230, 232, 238)
DIM = (120, 126, 140)
ACC = (255, 170, 60)
GOOD = (90, 210, 140)
BAD = (240, 90, 90)
CYAN = (80, 190, 255)
SESSIONS = HERE / "sessions"
PARAMS = HERE / "phone_params.json"


SIM_T = [None]  # bot mode: simulated clock (ns), advanced by the main loop


def now_ns() -> int:
    return SIM_T[0] if SIM_T[0] is not None else time.perf_counter_ns()


# ================================================================== inputs
class MouseInput:
    name = "mouse"

    def __init__(self):
        pygame.mouse.set_visible(False)
        pygame.event.set_grab(True)
        self.pos = np.array([CX, CY], float)
        pygame.mouse.set_pos((CX, CY))

    def handle(self, ev, out):
        if ev.type == pygame.MOUSEMOTION:
            self.pos = np.array(ev.pos, float)
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            out.append(("click", now_ns()))
        elif ev.type == pygame.MOUSEWHEEL:
            out.append(("wheel", now_ns(), int(ev.y)))

    def poll(self, out):
        pass

    def status(self):
        return ""

    def close(self, session_dir):
        pass


class _FrameCountingQueue(queue.Queue):
    """Queue that knows how many camera frames are waiting (cheap 'is a newer frame queued?' check)."""

    def __init__(self):
        super().__init__()
        self.frames = 0

    def put(self, item, block=True, timeout=None):
        if item[0] == "frame":
            self.frames += 1
        super().put(item, block, timeout)

    def get(self, block=True, timeout=None):
        item = super().get(block, timeout)
        if item[0] == "frame":
            self.frames -= 1
        return item


class PhoneInput:
    """Receives the phone stream, runs the engine in a thread, moves a virtual cursor (1 count = 1 px)."""
    name = "phone"

    def __init__(self, overrides: dict, host=None):
        import mvp_live
        from deskmouse.engine import EngineConfig

        pygame.mouse.set_visible(False)
        self.pos = np.array([CX, CY], float)
        self.cfg = EngineConfig(instant_left=True, **{k: v for k, v in overrides.items() if k in EngineConfig.__dataclass_fields__})
        self.overrides = overrides
        self.events: queue.Queue = queue.Queue()
        self.imu_rows: list = []
        self.hello = None
        self.eng = None
        self.connected = False
        self.ended = None
        self.msg = "waiting for the phone: tap 'Mouse mode' on it"
        self.fps = 0
        self._nf = 0
        self._ns = 0
        self.skipped = 0
        self._t = time.time()
        self._live = mvp_live
        self._host = host
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        from deskmouse.engine import Engine
        from deskmouse.io import intrinsics_from
        lv = self._live
        s = lv.open_stream(self._host)
        self.connected = True
        self.msg = "connected: put the phone face-down (3 beeps), hands off until the long beep"
        q = _FrameCountingQueue()
        threading.Thread(target=lv.reader, args=(s, q, None), daemon=True).start()
        ratio0 = self.overrides.get("gauge_ratio")
        while True:
            m = q.get()
            arr = now_ns()
            if m[0] == "end":
                self.ended = m[1]
                self.msg = f"stream ended: {m[1]}"
                return
            if m[0] == "hello":
                h = self.hello = m[1]
                K, dist = intrinsics_from(h["camera_characteristics"], h["width"])
                self.eng = Engine(K, dist, (h["width"], h["height"]), self.cfg, gauge_ratio=ratio0)
                self.eng.frame_log = []
                self.msg = ""
                continue
            if self.eng is None:
                continue
            if m[0] == "imu":
                _, ts, kind, x, y, z = m
                self.imu_rows.append((ts, 0 if kind == "gyro" else 1, x, y, z, arr))
                self.eng.on_imu(ts, kind, x, y, z)
                self.eng.poll(ts)
            elif m[0] == "frame":
                if q.frames > 0:  # a newer frame is already waiting: skip this one
                    self._ns += 1
                    continue
                self._nf += 1
                if time.time() - self._t > 1:
                    self.fps, self.skipped, self._nf, self._ns, self._t = self._nf, self._ns, 0, 0, time.time()
                _, ts, exp, skew, img = m
                n0 = len(self.eng.frame_log)
                self.eng.on_frame(img, ts, exp, skew)
                if len(self.eng.frame_log) > n0:
                    self.eng.frame_log[-1] = self.eng.frame_log[-1] + (arr,)
            for e in self.eng.drain():
                self.events.put((now_ns(), e))

    def handle(self, ev, out):
        pass

    def poll(self, out):
        while not self.events.empty():
            t, e = self.events.get()
            if e.kind == "move":
                self.pos = np.clip(self.pos + (e.dx, e.dy), (0, 0), (W - 1, H - 1))
            elif e.kind == "left":
                out.append(("click", t))
            elif e.kind == "wheel":
                out.append(("wheel", t, e.wheel))

    def status(self):
        if self.msg:
            return self.msg
        e = self.eng
        if e is None:
            return "waiting for camera data"
        st = "LIFTED" if e.lifted else "scroll" if e.scroll else "still" if e.still else "moving"
        return f"phone: {st}   {self.fps} fps ({self.skipped} skipped/s)   scale calibrations {e.stats['scale_updates']}"

    def close(self, d: Path):
        if self.eng is None:
            return
        np.savetxt(d / "phone_imu.csv", np.array(self.imu_rows, float), delimiter=",",
                   header="t_phone_ns,kind,x,y,z,arrival_pc_ns", comments="", fmt="%.10g")
        rows = [r for r in self.eng.frame_log if len(r) == 12]
        np.savetxt(d / "phone_frames.csv", np.array(rows, float), delimiter=",",
                   header="t_ns,psi_prev,psi,wx,wy,sigma,valid,n_rhos,rho15,rho_med,n_inliers,arrival_pc_ns",
                   comments="", fmt="%.10g")
        (d / "phone_hello.json").write_text(json.dumps(self.hello))
        r = self.eng.gauge_ratio()
        if r:
            self.overrides["gauge_ratio"] = r
        self.overrides.pop("lam", None)


class BotInput:
    """Test player: aims with feedback (like a person) through a distorted mapping (rotation, vertical gain)."""
    name = "bot"

    def __init__(self, rot_deg=0.0, aspect=1.0, seed=1):
        c, n = math.cos(math.radians(rot_deg)), math.sin(math.radians(rot_deg))
        # distortion in 'up is +y' coordinates, applied to hand motion
        self.D = np.diag([1.0, aspect]) @ np.array([[c, -n], [n, c]])
        # apply the tuned correction exactly like the engine does: out = diag(1, aspect) · R(mount_yaw) · raw
        bp = HERE / "bot_params.json"
        if rot_deg or aspect != 1.0:
            if bp.exists():
                q = json.loads(bp.read_text())
                m = math.radians(q.get("mount_yaw_deg", 0.0))
                C = np.diag([1.0, q.get("aspect", 1.0)]) @ np.array([[math.cos(m), -math.sin(m)], [math.sin(m), math.cos(m)]])
                self.D = C @ self.D
        self.pos = np.array([CX, CY], float)
        self.rng = np.random.default_rng(seed)
        self.hist: list = []  # (t, cursor) for reaction delay
        self.v = np.zeros(2)
        self.clicked_for = None
        self.t_last_wheel = 0
        self.move = None       # (t0, duration s, hand displacement, start pos)
        self.next_plan_t = 0

    def handle(self, ev, out):
        pass

    def status(self):
        return "bot"

    def close(self, d):
        pass

    def poll(self, out, lv=None, t=None, dt=1 / 120):
        if lv is None or t is None:
            return
        self.hist.append((t, self.pos.copy()))
        self.hist = self.hist[-40:]
        seen = next((p for (tt, p) in self.hist if tt >= t - 0.12e9), self.pos)  # 120 ms reaction delay
        tgt = lv.target
        if isinstance(lv, Scroll):
            z = lv.zones[min(lv.i, len(lv.zones) - 1)]
            if (t - self.t_last_wheel) > 0.15e9 and abs(lv.marker - z) > 0.5:
                out.append(("wheel", t, int(np.sign(z - lv.marker))))
                self.t_last_wheel = t
            return
        if isinstance(lv, Rhythm):  # aim at the next note due, not one that is already past
            nxt = [n for n in lv.notes if n[3] is None and n[0] >= lv.sec(t) - 0.02]
            tgt = (nxt[0][1], nxt[0][2], lv.r) if nxt else None
        elif isinstance(lv, ClickTargets) and lv.i < len(lv.points):  # current target, no one-frame lag
            tgt = (*lv.points[lv.i], lv.r)
        if tgt is None or not np.isfinite(tgt[0]):
            return
        if isinstance(lv, Trace):  # continuous pursuit: delayed proportional control
            err = np.array(tgt[:2]) - seen
            v_des = 4.0 * np.array([err[0], -err[1]])
            self.v += (v_des - self.v) * min(1.0, dt * 12)
            step = self.D @ (self.v * dt)
            self.pos = np.clip(self.pos + np.array([step[0], -step[1]]), (0, 0), (W - 1, H - 1))
            return
        # aiming: planned minimum-jerk movements (ballistic), then corrections after a reaction delay
        tgt_xy = np.array(tgt[:2])
        if self.move is None and t >= self.next_plan_t:
            e = tgt_xy - self.pos
            dist = float(np.hypot(*e))
            if dist > 0.3 * tgt[2]:
                hand = np.array([e[0], -e[1]]) * self.rng.normal(1.0, 0.05)  # intended, assuming no distortion
                self.move = (t, 0.2 + 0.00035 * dist, hand, self.pos.copy())
        if self.move is not None:
            t0, T, hand, p0 = self.move
            s = min(1.0, (t - t0) / 1e9 / T)
            k = 10 * s**3 - 15 * s**4 + 6 * s**5
            step = self.D @ (hand * k)
            self.pos = np.clip(p0 + np.array([step[0], -step[1]]), (0, 0), (W - 1, H - 1))
            if s >= 1.0:
                self.move = None
                self.next_plan_t = t + 0.12e9  # reaction delay before a correction
        d = float(np.hypot(*(tgt_xy - self.pos)))
        if isinstance(lv, Rhythm):
            live = [n for n in lv.notes if n[3] is None and n[0] >= lv.sec(t) - 0.02]
            if live and abs(lv.sec(t) - live[0][0]) < 0.005 and d < tgt[2]:
                out.append(("click", t))
        elif isinstance(lv, ClickTargets) and d < 0.6 * tgt[2] and self.move is None:
            key = (lv.name, lv.i, lv.misses)
            if self.clicked_for != key:
                self.clicked_for = key
                out.append(("click", t))


# ================================================================== levels
class Level:
    name = "level"
    help = ""

    def __init__(self, rng: random.Random):
        self.rng = rng
        self.done = False
        self.t0 = 0
        self.target = None  # (x, y, r) for logging
        self.res: dict = {}
        self.log: list = []  # (t, kind, data)

    def start(self, t):
        self.t0 = t

    def sec(self, t):
        return (t - self.t0) / 1e9

    def update(self, t, cur, clicks, wheel):
        raise NotImplementedError

    def draw(self, s, t, font):
        pass


def circle(s, col, p, r, w=0):
    pygame.draw.circle(s, col, (int(p[0]), int(p[1])), int(r), w)


class ClickTargets(Level):
    """Click each target (one at a time)."""

    def __init__(self, rng, name, help_, points, radius):
        super().__init__(rng)
        self.name, self.help = name, help_
        self.points, self.r = points, radius
        self.i = 0
        self.hits = self.misses = 0
        self.times: list = []
        self.t_spawn = 0

    def start(self, t):
        super().start(t)
        self.t_spawn = t
        self.log.append((t, "spawn", self.points[0]))

    def update(self, t, cur, clicks, wheel):
        p = self.points[self.i]
        self.target = (p[0], p[1], self.r)
        for tc in clicks:
            if np.hypot(cur[0] - p[0], cur[1] - p[1]) <= self.r:
                self.hits += 1
                self.times.append((tc - self.t_spawn) / 1e9)
                self.log.append((tc, "hit", (p[0], p[1], float(cur[0]), float(cur[1]))))
                self.i += 1
                self.t_spawn = tc
                if self.i >= len(self.points):
                    self.done = True
                    break
                p = self.points[self.i]
                self.log.append((tc, "spawn", p))
            else:
                self.misses += 1
                self.log.append((tc, "miss", (float(cur[0]), float(cur[1]))))
        if self.done:
            self.res = {"hits": self.hits, "misses": self.misses, "mean_time_s": float(np.mean(self.times)),
                        "median_time_s": float(np.median(self.times))}

    def draw(self, s, t, font):
        if self.i < len(self.points):
            p = self.points[self.i]
            circle(s, ACC, p, self.r)
            circle(s, BG, p, self.r * 0.35)
            if self.i + 1 < len(self.points):
                circle(s, DIM, self.points[self.i + 1], self.r, 2)
        s.blit(font.render(f"{self.i}/{len(self.points)}   misses {self.misses}", True, DIM), (20, H - 40))


class Rhythm(Level):
    name = "Rhythm"
    help = "Click (phone: tap) each circle when the shrinking ring touches it. Slow and steady."

    def __init__(self, rng, n=16, interval=1.1, approach=1.5, r=55):
        super().__init__(rng)
        self.r, self.approach = r, approach
        ang = 0.0
        self.notes = []
        for i in range(n):
            ang += rng.uniform(0.6, 1.4) * rng.choice([-1, 1])
            rad = rng.uniform(120, 240)
            self.notes.append([2.0 + i * interval, CX + rad * math.cos(ang) * 1.4, CY + rad * math.sin(ang), None])
        self.score = 0
        self.errors: list = []

    def update(self, t, cur, clicks, wheel):
        now = self.sec(t)
        for tc in clicks:
            tcs = self.sec(tc)
            cand = [n for n in self.notes if n[3] is None and abs(tcs - n[0]) < 0.25]
            if not cand:
                self.log.append((tc, "stray", (float(cur[0]), float(cur[1]))))
                continue
            n = min(cand, key=lambda n: abs(tcs - n[0]))
            inside = np.hypot(cur[0] - n[1], cur[1] - n[2]) <= self.r
            err = tcs - n[0]
            if inside and abs(err) <= 0.2:
                n[3] = 300 if abs(err) < 0.07 else 100 if abs(err) < 0.14 else 50
                self.errors.append(err)
            else:
                n[3] = 0
            self.score += n[3]
            self.log.append((tc, "note", (n[0], n[1], n[2], err, bool(inside), n[3])))
        for n in self.notes:
            if n[3] is None and now > n[0] + 0.25:
                n[3] = 0
                self.log.append((t, "expired", (n[0], n[1], n[2])))
        live = [n for n in self.notes if n[3] is None and n[0] - self.approach <= now]
        self.target = (live[0][1], live[0][2], self.r) if live else None
        if now > self.notes[-1][0] + 0.5:
            self.done = True
            hits = sum(1 for n in self.notes if n[3])
            self.res = {"score": self.score, "max": 300 * len(self.notes), "hits": hits, "notes": len(self.notes),
                        "timing_err_ms_mean": float(1000 * np.mean(self.errors)) if self.errors else None,
                        "timing_err_ms_sd": float(1000 * np.std(self.errors)) if self.errors else None}

    def draw(self, s, t, font):
        now = self.sec(t)
        for n in reversed(self.notes):
            if n[3] is None and n[0] - self.approach <= now <= n[0] + 0.25:
                k = max(0.0, (n[0] - now) / self.approach)
                circle(s, CYAN, (n[1], n[2]), self.r)
                circle(s, FG, (n[1], n[2]), self.r * (1 + 2 * k), 3)
        s.blit(font.render(f"score {self.score}", True, DIM), (20, H - 40))


class Trace(Level):
    name = "Trace"
    help = "Keep the cursor inside the moving ring for 30 seconds."

    def __init__(self, rng, dur=30.0, r=40):
        super().__init__(rng)
        self.dur, self.r = dur, r
        self.inside = 0
        self.total = 0
        self.err2 = 0.0

    def pos(self, sec):
        u = max(0.0, sec - 3.0)  # 3 s lead-in: ring waits at its start position
        return (CX + 520 * math.sin(2 * math.pi * 0.11 * u), CY + 260 * math.sin(2 * math.pi * 0.17 * u + 0.8))

    def update(self, t, cur, clicks, wheel):
        sec = self.sec(t)
        p = self.pos(sec)
        self.target = (p[0], p[1], self.r)
        if sec > 3.0:
            d = math.hypot(cur[0] - p[0], cur[1] - p[1])
            self.total += 1
            self.inside += d <= self.r
            self.err2 += d * d
        if sec > self.dur + 3.0:
            self.done = True
            self.res = {"inside_pct": 100 * self.inside / max(1, self.total), "rms_px": math.sqrt(self.err2 / max(1, self.total))}

    def draw(self, s, t, font):
        sec = self.sec(t)
        p = self.pos(sec)
        circle(s, GOOD if sec > 3 else DIM, p, self.r, 3)
        circle(s, GOOD, p, 4)
        if self.total:
            s.blit(font.render(f"inside {100 * self.inside / self.total:4.1f}%", True, DIM), (20, H - 40))


class Precision(Level):
    name = "Precision"
    help = "Move onto each small dot and hold still for 0.6 s (no click needed)."

    def __init__(self, rng, n=10, r=12, hold=0.6, timeout=10.0):
        super().__init__(rng)
        self.pts = [(rng.uniform(150, W - 150), rng.uniform(120, H - 120)) for _ in range(n)]
        self.r, self.hold, self.timeout = r, hold, timeout
        self.i = 0
        self.t_spawn = 0
        self.t_in = None
        self.times: list = []
        self.timeouts = 0
        self.hover_pos: list = []

    def start(self, t):
        super().start(t)
        self.t_spawn = t

    def update(self, t, cur, clicks, wheel):
        p = self.pts[self.i]
        self.target = (p[0], p[1], self.r)
        inside = math.hypot(cur[0] - p[0], cur[1] - p[1]) <= self.r
        if inside:
            self.t_in = self.t_in or t
            self.hover_pos.append((t, float(cur[0]), float(cur[1])))
        else:
            self.t_in = None
        nxt = False
        if self.t_in and (t - self.t_in) / 1e9 >= self.hold:
            self.times.append((t - self.t_spawn) / 1e9)
            self.log.append((t, "captured", p))
            nxt = True
        elif (t - self.t_spawn) / 1e9 > self.timeout:
            self.timeouts += 1
            self.log.append((t, "timeout", p))
            nxt = True
        if nxt:
            self.i += 1
            self.t_in = None
            self.t_spawn = t
            if self.i >= len(self.pts):
                self.done = True
                hp = np.array(self.hover_pos) if self.hover_pos else np.zeros((0, 3))
                self.res = {"captured": len(self.times), "timeouts": self.timeouts,
                            "mean_time_s": float(np.mean(self.times)) if self.times else None,
                            "hover_jitter_px": float(np.mean(np.hypot(np.diff(hp[:, 1]), np.diff(hp[:, 2])))) if len(hp) > 2 else None}

    def draw(self, s, t, font):
        if self.i < len(self.pts):
            p = self.pts[self.i]
            circle(s, ACC, p, self.r)
            if self.t_in:
                k = min(1.0, (t - self.t_in) / 1e9 / self.hold)
                pygame.draw.arc(s, GOOD, (p[0] - 22, p[1] - 22, 44, 44), 0, 2 * math.pi * k, 3)
        s.blit(font.render(f"{self.i}/{len(self.pts)}   timeouts {self.timeouts}", True, DIM), (20, H - 40))


class Scroll(Level):
    name = "Scroll"
    help = "Scroll (phone: twist in place) until the marker sits in the green zone, hold 0.4 s."

    def __init__(self, rng, n=8, step=30):
        super().__init__(rng)
        self.step = step
        self.marker = 0.0
        self.zones = []
        z = 0
        for i in range(n):
            z += rng.randint(3, 8) * (1 if i % 2 == 0 else -1)
            self.zones.append(z)
        self.i = 0
        self.t_spawn = 0
        self.t_in = None
        self.times: list = []
        self.notches = 0
        self.timeouts = 0

    def start(self, t):
        super().start(t)
        self.t_spawn = t

    def update(self, t, cur, clicks, wheel):
        for w in wheel:
            self.marker += w
            self.notches += abs(w)
            self.log.append((t, "wheel", w))
        z = self.zones[self.i]
        self.target = None
        if abs(self.marker - z) <= 0.5:
            self.t_in = self.t_in or t
        else:
            self.t_in = None
        nxt = False
        if self.t_in and (t - self.t_in) / 1e9 >= 0.4:
            self.times.append((t - self.t_spawn) / 1e9)
            nxt = True
        elif (t - self.t_spawn) / 1e9 > 12:
            self.timeouts += 1
            nxt = True
        if nxt:
            self.log.append((t, "zone", (z, self.marker)))
            self.i += 1
            self.t_in = None
            self.t_spawn = t
            if self.i >= len(self.zones):
                self.done = True
                self.res = {"zones": len(self.times), "timeouts": self.timeouts,
                            "mean_time_s": float(np.mean(self.times)) if self.times else None, "notches": self.notches}

    def draw(self, s, t, font):
        x = CX
        pygame.draw.line(s, DIM, (x, 40), (x, H - 40), 2)
        if self.i < len(self.zones):
            zy = CY - (self.zones[self.i] - self.marker) * self.step
            pygame.draw.rect(s, GOOD, (x - 60, zy - self.step / 2, 120, self.step), 0)
        pygame.draw.polygon(s, ACC, [(x - 40, CY), (x - 70, CY - 12), (x - 70, CY + 12)])
        pygame.draw.polygon(s, ACC, [(x + 40, CY), (x + 70, CY - 12), (x + 70, CY + 12)])
        s.blit(font.render(f"{self.i}/{len(self.zones)}   notches {self.notches}", True, DIM), (20, H - 40))


def make_levels(seed=7):
    rng = random.Random(seed)
    cal = []
    for i in range(8):
        cal.append((CX - 380, CY) if i % 2 == 0 else (CX + 380, CY))
    for i in range(8):
        cal.append((CX, CY - 260) if i % 2 == 0 else (CX, CY + 260))
    pts, prev = [], (CX, CY)
    while len(pts) < 20:
        p = (rng.uniform(90, W - 90), rng.uniform(90, H - 90))
        if math.hypot(p[0] - prev[0], p[1] - prev[1]) > 260:
            pts.append(p)
            prev = p
    return [
        ClickTargets(rng, "Calibrate", "Click the big targets: left/right, then up/down. Quick, confident strokes.", cal, 60),
        ClickTargets(rng, "Targets", "Click each orange target as fast as you can.", pts, 32),
        Rhythm(rng),
        Trace(rng),
        Precision(rng),
        Scroll(rng),
    ]


# ================================================================== main loop
def text(s, font, msg, y, col=FG, center=True):
    img = font.render(msg, True, col)
    s.blit(img, (CX - img.get_width() // 2 if center else 40, y))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", choices=["mouse", "phone", "bot"], required=True)
    ap.add_argument("--bot-rot", type=float, default=0.0, help="bot test: cursor rotation (deg)")
    ap.add_argument("--bot-aspect", type=float, default=1.0, help="bot test: vertical gain")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--levels", help="comma-separated level numbers to play (1-6), default all")
    ap.add_argument("--no-tune", action="store_true")
    ap.add_argument("--host", help="phone IP address: connect over Wi-Fi instead of USB")
    a = ap.parse_args()

    if a.headless:
        import os
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    if a.input == "bot":
        SIM_T[0] = 10**12
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"Desk Mouse aim lab ({a.input})")
    font = pygame.font.SysFont("segoeui", 26)
    big = pygame.font.SysFont("segoeui", 44, bold=True)
    small = pygame.font.SysFont("consolas", 20)
    clock = pygame.time.Clock()

    overrides = json.loads(PARAMS.read_text()) if PARAMS.exists() else {}
    inp = (MouseInput() if a.input == "mouse" else PhoneInput(overrides, a.host) if a.input == "phone"
           else BotInput(a.bot_rot, a.bot_aspect))
    levels = make_levels()
    if a.input == "phone" and not overrides.get("scroll_enabled", True):
        levels = [lv for lv in levels if lv.name != "Scroll"]
    if a.levels:
        keep = {int(x) for x in a.levels.split(",")}
        levels = [lv for i, lv in enumerate(levels, 1) if i in keep]

    d = SESSIONS / f"{datetime.now():%Y%m%d_%H%M%S}_{a.input}"
    d.mkdir(parents=True, exist_ok=True)
    frame_rows = []
    results = {}
    li = 0
    playing = False
    quit_ = False
    while not quit_ and li < len(levels):
        lv = levels[li]
        clicks, wheel, raw = [], [], []
        if a.input == "bot":
            SIM_T[0] += int(1e9 / 120)
            if not playing:
                playing = True
                lv.start(now_ns())
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                quit_ = True
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE and not playing:
                playing = True
                lv.start(now_ns())
            inp.handle(ev, raw)
        if a.input == "bot":
            inp.poll(raw, lv if playing else None, now_ns())
        else:
            inp.poll(raw)
        for r in raw:
            if r[0] == "click":
                clicks.append(r[1])
            else:
                wheel.append(r[2])
        t = now_ns()
        cur = inp.pos.copy()
        screen.fill(BG)
        if playing:
            lv.update(t, cur, clicks, wheel)
            lv.draw(screen, t, font)
            tg = lv.target or (np.nan, np.nan, np.nan)
            frame_rows.append((t, li, cur[0], cur[1], tg[0], tg[1], tg[2]))
            if lv.done:
                results[lv.name] = lv.res
                playing = False
                li += 1
        else:
            text(screen, big, f"Level {li + 1}/{len(levels)}: {lv.name}", CY - 120)
            text(screen, font, lv.help, CY - 40, DIM)
            text(screen, font, "press SPACE to start   (ESC quits)", CY + 40)
            if results:
                last = list(results.items())[-1]
                text(screen, small, f"{last[0]}: " + ", ".join(f"{k} {v:.3g}" if isinstance(v, float) else f"{k} {v}"
                                                              for k, v in last[1].items()), CY + 120, GOOD)
        st = inp.status()
        if st:
            screen.blit(small.render(st, True, ACC), (20, 14))
        circle(screen, FG, cur, 5)
        circle(screen, BG, cur, 2)
        pygame.display.flip()
        if a.input != "bot":
            clock.tick(120)  # leaves CPU for the phone engine thread

    # save session
    with open(d / "game_frames.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_pc_ns", "level", "cx", "cy", "tx", "ty", "tr"])
        w.writerows(frame_rows)
    ev_rows = []
    for i, lv in enumerate(levels):
        ev_rows.append((lv.t0, i, "level_start", lv.name))
        for (t, k, dd) in lv.log:
            ev_rows.append((t, i, k, json.dumps(dd)))
    with open(d / "game_events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_pc_ns", "level", "kind", "data"])
        w.writerows(ev_rows)
    meta = {"input": a.input, "bot_rot": a.bot_rot, "bot_aspect": a.bot_aspect, "date": datetime.now().isoformat(), "window": [W, H], "results": results,
            "levels": [lv.name for lv in levels], "params": overrides if a.input == "phone" else None}
    inp.close(d)
    (d / "session.json").write_text(json.dumps(meta, indent=1))

    # results + (phone) auto-tune
    lines = [f"Session saved: {d.name}"]
    tune_lines: list = []
    if a.input in ("phone", "bot") and not a.no_tune and ((d / "phone_imu.csv").exists() or a.input == "bot"):
        import tune
        try:
            tune_lines = tune.run(d, overrides)
        except Exception as e:  # noqa: BLE001
            tune_lines = [f"tuning failed: {e}"]
    elif a.input == "phone" and "gauge_ratio" in overrides:
        PARAMS.write_text(json.dumps(overrides, indent=1))
    import tune as _t
    base = _t.latest_session("mouse", exclude=d)
    waiting = not a.headless
    if a.headless:
        print("\n".join(lines + tune_lines))
        for name, r in results.items():
            print(name, r)
    while waiting:
        for ev in pygame.event.get():
            if ev.type in (pygame.QUIT, pygame.KEYDOWN):
                waiting = False
        screen.fill(BG)
        text(screen, big, "Results", 40)
        y = 110
        for name, r in results.items():
            b = (json.loads((base / "session.json").read_text())["results"].get(name) if base else None) or {}
            txt = ", ".join(f"{k} {v:.3g}" if isinstance(v, float) else f"{k} {v}" for k, v in r.items())
            screen.blit(font.render(f"{name}: {txt}", True, FG), (40, y))
            if b:
                btxt = ", ".join(f"{k} {v:.3g}" if isinstance(v, float) else f"{k} {v}" for k, v in b.items())
                screen.blit(small.render(f"   mouse baseline: {btxt}", True, DIM), (40, y + 32))
            y += 72
        for ln in lines + tune_lines:
            screen.blit(small.render(ln, True, ACC), (40, y))
            y += 26
        text(screen, font, "press any key to exit", H - 50, DIM)
        pygame.display.flip()
        clock.tick(30)
    pygame.quit()


if __name__ == "__main__":
    main()
