"""Synthetic recordings: textured multi-depth scene above a phone sliding on a desk, plus IMU.

Writes the recorder's directory format (DESIGN.md §5.3) plus gt.csv (ground truth), so every
tool runs on synthetic and real data alike.

Frames (DESIGN.md §2): body B = (X right, Y forward, Z up) when face-down; device frame
x_dev = -X_B, y_dev = +Y_B, z_dev = -Z_B. World W = desk plane, Z up. Camera looks up (+Z_W).
Image mapping used by the simulator: image x = +X_B, image y = -Y_B (pixel rows grow toward the
user), i.e. M_ci = diag(1, -1). Real phones may differ; the calibration step estimates it.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

G = 9.80665


# ---------------------------------------------------------------- scene

@dataclass
class Patch:
    """Horizontal textured rectangle at height z (m above the camera) covering [x0,x1]×[y0,y1] in W."""
    z: float
    x0: float = -1e3
    x1: float = 1e3
    y0: float = -1e3
    y1: float = 1e3
    texture_scale: float = 0.004  # metres per texel
    seed: int = 0
    contrast: float = 1.0
    brightness: float = 150.0
    motion: tuple[float, float, float] | None = None  # (amplitude m, frequency Hz, direction rad): moving distractor
    tex: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        # Periodic 1/f noise (filtered in the frequency domain), so wrapping the texture leaves no seams.
        rng = np.random.default_rng(self.seed)
        n = 1024
        f = np.hypot(*np.meshgrid(np.fft.fftfreq(n), np.fft.fftfreq(n)))
        spectrum = np.fft.fft2(rng.standard_normal((n, n))) / np.maximum(f, 1.0 / n) ** 0.9
        t = np.real(np.fft.ifft2(spectrum)).astype(np.float32)
        t = (t - t.mean()) / (t.std() + 1e-6)
        self.tex = np.clip(self.brightness + 25 * self.contrast * t, 0, 255).astype(np.float32)

    def offset(self, t: float) -> tuple[float, float]:
        if not self.motion:
            return 0.0, 0.0
        a, f, ang = self.motion
        s = a * np.sin(2 * np.pi * f * t)
        return s * np.cos(ang), s * np.sin(ang)


def scene_preset(name: str, ceiling_h: float = 1.9) -> list[Patch]:
    """'plane': textured ceiling only. 'lowtex': faint ceiling. 'multidepth': ceiling + shelf + monitor
    (like place 1). 'distractor': multidepth + a moving head-like patch."""
    ceiling = Patch(ceiling_h, seed=1)
    if name == "plane":
        return [ceiling]
    if name == "lowtex":
        return [Patch(ceiling_h, seed=1, contrast=0.08)]
    near = [
        ceiling,
        Patch(0.45, x0=-1.2, x1=-0.35, y0=-0.6, y1=0.9, seed=2, texture_scale=0.002, brightness=120),  # shelf underside
        Patch(0.30, x0=0.25, x1=0.9, y0=0.35, y1=0.8, seed=3, texture_scale=0.0015, brightness=80),   # monitor edge
    ]
    if name == "multidepth":
        return near
    if name == "distractor":
        return near + [Patch(0.55, x0=0.3, x1=0.65, y0=-0.55, y1=-0.25, seed=4, texture_scale=0.001,
                             brightness=110, motion=(0.05, 0.4, 0.3))]
    raise ValueError(name)


# ---------------------------------------------------------------- trajectories

def _min_jerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def keyframes_to_traj(keys: list[tuple[float, float, float, float]], rate: float):
    """keys = [(t, x, y, psi)], min-jerk between consecutive keys. Returns t, x, y, psi arrays."""
    t = np.arange(0, keys[-1][0], 1 / rate)
    out = np.zeros((len(t), 3))
    k = np.array(keys)
    for i in range(len(k) - 1):
        a, b = k[i], k[i + 1]
        m = (t >= a[0]) & (t < b[0])
        s = _min_jerk((t[m] - a[0]) / max(b[0] - a[0], 1e-9))[:, None]
        out[m] = a[1:] + (b[1:] - a[1:]) * s
    out[t >= k[-1][0]] = k[-1][1:]
    return t, out[:, 0], out[:, 1], out[:, 2]


def protocol_keys(tag: str, speed: float = 1.0, psi0: float = 0.0) -> list[tuple[float, float, float, float]]:
    """Keyframes mirroring the recording protocols (DESIGN.md §5.2). speed scales stroke durations."""
    mv = 0.8 / speed  # seconds per stroke
    keys = [(0.0, 0.0, 0.0, psi0), (2.0, 0.0, 0.0, psi0)]  # 2 s stillness for initialisation

    def add(dt, x, y, psi):
        keys.append((keys[-1][0] + dt, x, y, psi))

    if tag == "still":
        add(20.0, 0, 0, psi0)
    elif tag in ("ruler_x", "ruler_y"):
        d = (0.2, 0.0) if tag == "ruler_x" else (0.0, 0.2)
        for _ in range(5):
            add(mv, *d, psi0); add(1.0, *d, psi0); add(mv, 0, 0, psi0); add(1.0, 0, 0, psi0)
    elif tag in ("square", "square_rot"):
        c, s = np.cos(psi0), np.sin(psi0)
        for _ in range(3):
            for (u, v) in [(0.15, 0), (0.15, 0.15), (0, 0.15), (0, 0)]:
                add(mv, c * u - s * v, s * u + c * v, psi0); add(0.7, c * u - s * v, s * u + c * v, psi0)
    elif tag == "twist":
        for k in range(6):
            a = np.radians(45) * (1 if k % 2 == 0 else -1)
            add(mv, 0, 0, psi0 + a); add(0.7, 0, 0, psi0 + a)
        add(mv, 0, 0, psi0); add(1.0, 0, 0, psi0)
    else:
        raise ValueError(tag)
    add(2.0, *keys[-1][1:])
    return keys


# ---------------------------------------------------------------- rendering

@dataclass
class Camera:
    width: int = 640
    height: int = 480
    fx: float = 428.0
    fy: float = 427.5
    cx: float = 319.3
    cy: float = 237.6
    r_cam: tuple[float, float] = (0.0, 0.0)  # lens offset from the reference point, body frame (m)

    def rays(self):
        u, v = np.meshgrid(np.arange(self.width, dtype=np.float32), np.arange(self.height, dtype=np.float32))
        # image x = +X_B, image y = -Y_B; camera looks along +Z
        return (u - self.cx) / self.fx, -(v - self.cy) / self.fy


def render(scene: list[Patch], cam: Camera, x: float, y: float, psi: float, t: float,
           rays=None, noise: float = 1.0, rng=None) -> np.ndarray:
    bx, by = rays if rays is not None else cam.rays()
    c, s = np.cos(psi), np.sin(psi)
    lx = x + c * cam.r_cam[0] - s * cam.r_cam[1]
    ly = y + s * cam.r_cam[0] + c * cam.r_cam[1]
    wx_dir = c * bx - s * by  # ray direction in W per metre of height
    wy_dir = s * bx + c * by
    img = np.zeros(bx.shape, np.float32)
    depth = np.full(bx.shape, np.inf, np.float32)
    for p in scene:
        ox, oy = p.offset(t)
        X = lx + p.z * wx_dir - ox
        Y = ly + p.z * wy_dir - oy
        m = (X >= p.x0) & (X <= p.x1) & (Y >= p.y0) & (Y <= p.y1) & (p.z < depth)
        if not m.any():
            continue
        n = p.tex.shape[0]
        mapx = np.mod(X / p.texture_scale, n).astype(np.float32)
        mapy = np.mod(Y / p.texture_scale, n).astype(np.float32)
        val = cv2.remap(p.tex, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        img[m] = val[m]
        depth[m] = p.z
    if noise > 0:
        rng = rng or np.random.default_rng()
        img += rng.normal(0, noise, img.shape).astype(np.float32)
    return np.clip(img, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------- IMU

@dataclass
class ImuModel:
    rate: float = 500.0
    gyro_noise: float = 0.0013  # rad/s per sample (measured on S24 still takes)
    accel_noise: float = 0.006  # m/s² per sample (measured)
    gyro_bias: tuple[float, float, float] = (-0.0003, -0.0007, 0.0006)  # device frame, rad/s
    accel_bias: tuple[float, float, float] = (0.01, -0.01, 0.0)  # device frame, m/s²
    tilt: tuple[float, float] = (0.0, 0.006)  # small desk tilt (rad) about body X, Y


def imu_samples(t, x, y, psi, imu: ImuModel, rng):
    """Device-frame gyro and accel for planar motion of the reference point (= body origin)."""
    dt = t[1] - t[0]
    vx, vy = np.gradient(x, dt), np.gradient(y, dt)
    ax, ay = np.gradient(vx, dt), np.gradient(vy, dt)
    wz = np.gradient(np.unwrap(psi), dt)
    c, s = np.cos(psi), np.sin(psi)
    # specific force in body frame: horizontal accel rotated into B, plus the upward reaction to gravity
    fbx = c * ax + s * ay
    fby = -s * ax + c * ay
    tx, ty = imu.tilt
    fbx = fbx + G * np.sin(ty)
    fby = fby - G * np.sin(tx)
    fbz = np.full_like(fbx, G * np.cos(tx) * np.cos(ty))
    # body → device: x_dev = -X_B, y_dev = Y_B, z_dev = -Z_B
    acc = np.stack([-fbx, fby, -fbz], 1) + imu.accel_bias + rng.normal(0, imu.accel_noise, (len(t), 3))
    gyr = np.stack([np.zeros_like(wz), np.zeros_like(wz), -wz], 1) + imu.gyro_bias + rng.normal(0, imu.gyro_noise, (len(t), 3))
    return acc, gyr, vx, vy


# ---------------------------------------------------------------- writer

def simulate(out_dir: str | Path, tag: str, scene: str = "multidepth", speed: float = 1.0, psi0: float | None = None,
             fps: float = 60.0, exposure_ns: int = 2_000_000, cam: Camera | None = None, imu: ImuModel | None = None,
             raw: bool = True, seed: int = 0, t0_ns: int = 10_000_000_000, max_seconds: float | None = None) -> Path:
    cam = cam or Camera()
    imu = imu or ImuModel()
    rng = np.random.default_rng(seed)
    if psi0 is None:
        psi0 = float(np.radians(30)) if tag == "square_rot" else 0.0
    t, x, y, psi = keyframes_to_traj(protocol_keys(tag, speed, psi0), imu.rate)
    if max_seconds:
        m = t < max_seconds
        t, x, y, psi = t[m], x[m], y[m], psi[m]
    acc, gyr, vx, vy = imu_samples(t, x, y, psi, imu, rng)
    patches = scene_preset(scene)

    d = Path(out_dir) / f"rec_sim_{tag}_{scene}_s{speed:g}_{seed}"
    d.mkdir(parents=True, exist_ok=True)
    t_ns = t0_ns + np.round(t * 1e9).astype(np.int64)

    with open(d / "imu.csv", "w", newline="") as f:
        f.write("t_ns,type,x,y,z,a,b,c,d\n")
        rows = []
        for i in range(len(t)):
            rows.append(f"{t_ns[i]},gyro,{gyr[i, 0]:.7g},{gyr[i, 1]:.7g},{gyr[i, 2]:.7g},,,,\n")
            rows.append(f"{t_ns[i] + 50_000},accel,{acc[i, 0]:.7g},{acc[i, 1]:.7g},{acc[i, 2]:.7g},,,,\n")
        f.writelines(rows)
    with open(d / "gt.csv", "w", newline="") as f:
        f.write("t_ns,x,y,psi,vx,vy\n")
        f.writelines(f"{t_ns[i]},{x[i]:.7g},{y[i]:.7g},{psi[i]:.7g},{vx[i]:.7g},{vy[i]:.7g}\n" for i in range(len(t)))

    # SENSOR_TIMESTAMP = start of exposure; render at mid-exposure (no blur, global shutter)
    ft = np.arange(0, t[-1] - exposure_ns / 1e9, 1 / fps)
    frame_ts = t0_ns + np.round(ft * 1e9).astype(np.int64)
    rays = cam.rays()
    if raw:
        sink = open(d / "frames.y8", "wb")
    else:
        sink = cv2.VideoWriter(str(d / "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (cam.width, cam.height), False)
    for tf in ft:
        tm = tf + exposure_ns / 2e9
        img = render(patches, cam, np.interp(tm, t, x), np.interp(tm, t, y), np.interp(tm, t, psi), tm, rays, rng=rng)
        if raw:
            sink.write(img.tobytes())
        else:
            sink.write(img)
    if raw:
        sink.close()
    else:
        sink.release()
    with open(d / "frames.csv", "w", newline="") as f:
        f.write("idx,t_ns,exposure_ns,rolling_shutter_skew_ns,iso,frame_duration_ns,focus_diopters,active_physical_id\n")
        f.writelines(f"{k},{ts},{exposure_ns},0,100,{int(1e9 / fps)},0.526,sim\n" for k, ts in enumerate(frame_ts))
    (d / "labels.csv").write_text("t_ns,label\n")

    s_arr = cam.width / 4080
    meta = {
        "format_version": 1, "app_version": "sim", "protocol": tag, "location": f"sim:{scene}",
        "device": {"model": "simulator"}, "start_elapsed_realtime_ns": int(t0_ns), "duration_s": float(t[-1]),
        "storage": "frames.y8" if raw else "video.mp4",
        "capture": {"camera_choice": "sim", "camera_id": "sim", "zoom_ratio": 1.0, "width": cam.width,
                    "height": cam.height, "requested_fps": fps, "exposure_mode": "manual",
                    "applied_exposure_ns": exposure_ns, "applied_iso": 100,
                    "applied_state": {"ois_mode": 0, "video_stabilization_mode": 0}},
        "camera_characteristics": {
            "intrinsics": [cam.fx / s_arr, cam.fy / s_arr, cam.cx / s_arr, cam.cy / s_arr, 0],
            "distortion": [0, 0, 0, 0, 0], "active_array": "0 0 4080 3060", "timestamp_source": 1,
            "sensor_orientation": 90},
        "measured": {"frames": len(frame_ts), "dropped_frames": 0},
        "sim": {"scene": scene, "speed": speed, "psi0": psi0, "seed": seed, "r_cam": list(cam.r_cam),
                "M_ci": [[1, 0], [0, -1]], "patch_heights": [p.z for p in patches], "imu": asdict(imu)},
    }
    (d / "meta.json").write_text(json.dumps(meta, indent=1))
    return d
