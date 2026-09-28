"""Load recordings in the DESIGN.md §5.3 format (recorder app or synthetic simulator).

imu.csv columns: t_ns,type,x,y,z,a,b,c,d
  gyro/accel: a..d empty; gyro_unc/accel_unc: a,b,c = bias; grv: a = w.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np
import pandas as pd


@dataclass
class Recording:
    path: Path
    meta: dict
    frames: pd.DataFrame
    imu: pd.DataFrame
    labels: pd.DataFrame
    width: int = field(init=False)
    height: int = field(init=False)

    def __post_init__(self):
        cap = self.meta["capture"]
        self.width, self.height = int(cap["width"]), int(cap["height"])

    @property
    def tag(self) -> str:
        return self.meta["protocol"]

    def imu_of(self, kind: str) -> pd.DataFrame:
        """Samples of one sensor type ('gyro', 'accel', ...), sorted by time."""
        return self.imu[self.imu["type"] == kind].sort_values("t_ns").reset_index(drop=True)

    def frames_y(self, start: int = 0, stop: int | None = None) -> Iterator[tuple[int, np.ndarray]]:
        """Yield (idx, Y plane uint8 HxW) in storage order, which matches frames.csv rows."""
        stop = len(self.frames) if stop is None else min(stop, len(self.frames))
        raw = self.path / "frames.y8"
        if raw.exists():
            n = self.width * self.height
            with open(raw, "rb") as f:
                f.seek(start * n)
                for i in range(start, stop):
                    buf = f.read(n)
                    if len(buf) < n:
                        return
                    yield i, np.frombuffer(buf, np.uint8).reshape(self.height, self.width)
            return
        cap = cv2.VideoCapture(str(self.path / "video.mp4"))
        if start:
            cap.set(cv2.CAP_PROP_POS_FRAMES, start)
        for i in range(start, stop):
            ok, img = cap.read()
            if not ok:
                return
            yield i, img[:, :, 0] if img.ndim == 3 else img  # chroma is flat, any channel = Y

    def video_frame_count(self) -> int:
        """Frames actually decodable from the video (or raw file)."""
        raw = self.path / "frames.y8"
        if raw.exists():
            return raw.stat().st_size // (self.width * self.height)
        cap = cv2.VideoCapture(str(self.path / "video.mp4"))
        n = 0
        while cap.grab():
            n += 1
        return n


def load(path: str | Path) -> Recording:
    p = Path(path)
    meta = json.loads((p / "meta.json").read_text(encoding="utf-8"))
    frames = pd.read_csv(p / "frames.csv")
    imu = pd.read_csv(p / "imu.csv", dtype={"type": "category"})
    labels_file = p / "labels.csv"
    labels = pd.read_csv(labels_file) if labels_file.exists() else pd.DataFrame(columns=["t_ns", "label"])
    return Recording(p, meta, frames, imu, labels)
