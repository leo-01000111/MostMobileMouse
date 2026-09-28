"""MVP: phone (face-down, "Mouse mode" in the recorder app, USB) -> this script -> Windows cursor.

usage:  python PC-part/receiver/mvp_live.py [--dry-run] [--mount-yaw 0] [--dpi 800] [--save out.bin]
Stop with Ctrl+C, or press both volume keys on the phone.
"""
import argparse
import ctypes
import json
import os
import queue
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "proto"))
from deskmouse.engine import Engine, EngineConfig  # noqa: E402
from deskmouse.io import intrinsics_from  # noqa: E402

PORT = 47475
STATE = Path(__file__).resolve().parent / "state.json"  # remembered scale between sessions


# ------------------------------------------------------------------ Windows mouse output
class _MI(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long), ("dy", ctypes.c_long), ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong), ("time", ctypes.c_ulong), ("dwExtraInfo", ctypes.POINTER(ctypes.c_ulong))]


class _INPUT(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong), ("mi", _MI)]


MOVE, LDOWN, LUP, RDOWN, RUP, WHEEL = 0x1, 0x2, 0x4, 0x8, 0x10, 0x800


def send(flags, dx=0, dy=0, data=0):
    inp = _INPUT(0, _MI(dx, dy, data & 0xFFFFFFFF, flags, 0, None))
    ctypes.windll.user32.SendInput(1, ctypes.byref(inp), ctypes.sizeof(inp))


def apply(ev, dry):
    if dry:
        if ev.kind != "move":
            print(f"\n  {ev.kind} {ev.wheel if ev.kind == 'wheel' else ''}")
        return
    if ev.kind == "move":
        send(MOVE, ev.dx, ev.dy)
    elif ev.kind == "left":
        send(LDOWN)
        send(LUP)
    elif ev.kind == "right":
        send(RDOWN)
        send(RUP)
    elif ev.kind == "wheel":
        send(WHEEL, data=120 * ev.wheel)


# ------------------------------------------------------------------ stream
def adb() -> str:
    return shutil.which("adb") or str(Path(os.environ.get("LOCALAPPDATA", "")) / "Android/Sdk/platform-tools/adb.exe")


def recv_exact(s, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = s.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError("phone closed the stream")
        got += k
    return bytes(buf)


def reader(s, q, save):
    try:
        while True:
            t = recv_exact(s, 1)
            if t == b"\x01":
                raw = recv_exact(s, 4)
                body = recv_exact(s, struct.unpack("<I", raw)[0])
                if save:
                    save.write(t + raw + body)
                q.put(("hello", json.loads(body)))
            elif t == b"\x02":
                b = recv_exact(s, 21)
                if save:
                    save.write(t + b)
                ts, kind, x, y, z = struct.unpack("<qBfff", b)
                q.put(("imu", ts, "gyro" if kind == 0 else "accel", x, y, z))
            elif t == b"\x03":
                h = recv_exact(s, 20)
                ts, exp, skew, w, hh = struct.unpack("<qiiHH", h)
                img = recv_exact(s, w * hh)
                if save:
                    save.write(t + h + img)
                q.put(("frame", ts, exp, skew, np.frombuffer(img, np.uint8).reshape(hh, w)))
            else:
                raise ValueError(f"bad message type {t!r}")
    except Exception as e:  # noqa: BLE001
        q.put(("end", str(e)))


def connect():
    """adb accepts the forwarded connection even when the phone isn't listening; wait for real data."""
    while True:
        s = None
        try:
            s = socket.create_connection(("127.0.0.1", PORT), timeout=2)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
            s.settimeout(30)  # the phone sends its first message after countdown + metering (~5 s)
            if s.recv(1, socket.MSG_PEEK):
                s.settimeout(None)
                return s
        except OSError:
            pass
        if s:
            s.close()
        time.sleep(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="print clicks/scrolls, don't move the cursor")
    ap.add_argument("--mount-yaw", type=float, default=0.0, help="degrees; use if cursor directions are rotated")
    ap.add_argument("--dpi", type=float, default=800.0)
    ap.add_argument("--body-aligned", action="store_true")
    ap.add_argument("--save", help="write the raw stream to this file")
    a = ap.parse_args()

    subprocess.run([adb(), "forward", f"tcp:{PORT}", f"tcp:{PORT}"], check=True)
    print("Open the recorder app on the phone and tap 'Mouse mode'. Waiting for the phone...")
    s = connect()
    print("Connected. Put the phone face-down now (3 beeps, then hands off for 1.5 s).")
    save = open(a.save, "wb") if a.save else None
    q = queue.Queue()
    threading.Thread(target=reader, args=(s, q, save), daemon=True).start()

    cfg = EngineConfig(mount_yaw_deg=a.mount_yaw, dpi=a.dpi, world_aligned=not a.body_aligned)
    lam0 = json.loads(STATE.read_text()).get("lam") if STATE.exists() else None
    eng = None
    n_frames = n_skipped = 0
    t_last = time.time()
    try:
        while True:
            msg = q.get()
            if msg[0] == "end":
                print("\nstream ended:", msg[1])
                break
            if msg[0] == "hello":
                h = msg[1]
                K, dist = intrinsics_from(h["camera_characteristics"], h["width"])
                eng = Engine(K, dist, (h["width"], h["height"]), cfg, lam0=lam0)
                print(f"hello: {h['device'].get('model')} {h['width']}x{h['height']} @ {h['fps']} fps, "
                      f"exposure {h['exposure_ns'] / 1e6:.2f} ms ISO {h['iso']}; remembered scale {lam0}")
                continue
            if eng is None:
                continue
            if msg[0] == "imu":
                _, ts, kind, x, y, z = msg
                eng.on_imu(ts, kind, x, y, z)
                eng.poll(ts)
            elif msg[0] == "frame":
                # never fall behind: if a newer frame is already waiting, skip this one
                if any(m[0] == "frame" for m in list(q.queue)[:400]):
                    n_skipped += 1
                    continue
                _, ts, exp, skew, img = msg
                eng.on_frame(img, ts, exp, skew)
                n_frames += 1
            for ev in eng.drain():
                apply(ev, a.dry_run)
            if time.time() - t_last > 1.0:
                st = eng.stats
                state = "LIFTED" if eng.lifted else "scroll" if eng.scroll else "still" if eng.still else "moving"
                lam = f"{eng.lam:.3f}" if eng.lam else "-"
                print(f"\r{n_frames:3d} fr/s  skipped {n_skipped:3d}  queue {q.qsize():4d}  {state:7s}  "
                      f"scale {lam}  calibrations {st['scale_updates']:3d}  taps {st['taps']:3d}   ", end="", flush=True)
                n_frames = n_skipped = 0
                t_last = time.time()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if eng is not None and eng.lam and eng.stats["scale_updates"] > 0:
            STATE.write_text(json.dumps({"lam": eng.lam}))
            print(f"saved scale {eng.lam:.4f} to {STATE.name}")
        if save:
            save.close()
        s.close()


if __name__ == "__main__":
    main()
