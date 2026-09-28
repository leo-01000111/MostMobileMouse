"""Tier 0 ceiling check: feature counts on stock-camera photos and frame-to-frame
tracking quality on a stock-camera video (plans/RECORDINGS_NEEDED.md §1).

usage: python tier0_check.py <place_dir>   (writes <place_dir>/analysis/summary.json)
Photos are identified by EXIF focal length (5.4 mm = main, 2.2 mm = ultra-wide).
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import ExifTags, Image

LK = dict(winSize=(21, 21), maxLevel=3, criteria=(3, 30, 0.01))


def features(gray, n=400):
    mask = (gray < 250).astype(np.uint8) * 255
    p = cv2.goodFeaturesToTrack(gray, n, 0.01, 12, mask=mask)
    return np.zeros((0, 1, 2), np.float32) if p is None else p


def grid_cells(p, w, h, k=4):
    return len({(int(x * k / w), int(y * k / h)) for x, y in p.reshape(-1, 2)})


def photo_stats(path):
    im = Image.open(path)
    exif = {ExifTags.TAGS.get(k, k): v for k, v in im.getexif().get_ifd(0x8769).items()}
    focal = float(exif.get("FocalLength", 0))
    cam = "main" if focal > 4 else "ultrawide"
    gray = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2GRAY)
    out = {"camera": cam, "focal_mm": focal}
    for w in (640, 1280):
        h = w * 3 // 4
        s = cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA)
        s_clahe = cv2.createCLAHE(3.0, (8, 8)).apply(s)
        p, pc = features(s), features(s_clahe)
        out[f"{w}"] = dict(n=len(p), grid16=grid_cells(p, w, h), n_clahe=len(pc), grid16_clahe=grid_cells(pc, w, h))
    return out


def video_stats(path, short_side=360):
    cap = cv2.VideoCapture(str(path))
    vw, vh = cap.get(3), cap.get(4)
    scale = short_side / min(vw, vh)
    size = (int(vw * scale), int(vh * scale))
    prev, rows = None, []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        g = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), size, interpolation=cv2.INTER_AREA)
        if prev is not None:
            p = cv2.goodFeaturesToTrack(prev, 200, 0.01, 8)
            if p is not None and len(p) > 10:
                q, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, p, None, **LK)
                r, st2, _ = cv2.calcOpticalFlowPyrLK(g, prev, q, None, **LK)
                good = (st.ravel() & st2.ravel()).astype(bool) & (np.linalg.norm((r - p).reshape(-1, 2), axis=1) < 0.5)
                a, b = p.reshape(-1, 2)[good], q.reshape(-1, 2)[good]
                if len(a) > 10:
                    _, inl = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC, ransacReprojThreshold=0.5)
                    rows.append((len(a), int(inl.sum()) if inl is not None else 0,
                                 float(np.median(np.linalg.norm(b - a, axis=1))), cv2.Laplacian(g, cv2.CV_64F).var()))
        prev = g
    o = np.array(rows)
    moving, still = o[:, 2] > 0.7, o[:, 2] < 0.2

    def summ(m):
        if not m.any():
            return None
        return dict(frames=int(m.sum()), tracks_median=float(np.median(o[m, 0])),
                    rigid_inlier_frac_median=round(float(np.median(o[m, 1] / o[m, 0])), 3),
                    sharpness_median=round(float(np.median(o[m, 3])), 1))

    return dict(size=[vw, vh], fps=cap.get(5), analysed_at=list(size), moving=summ(moving), still=summ(still))


def main(place):
    place = Path(place)
    res = {"photos": {p.name: photo_stats(p) for p in sorted(place.glob("*.jpg"))},
           "videos": {v.name: video_stats(v) for v in sorted(place.glob("*.mp4"))}}
    (place / "analysis").mkdir(exist_ok=True)
    (place / "analysis" / "summary.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main(sys.argv[1])
