"""Run the pipeline on one recording, save trajectory + stats (+ plots).

usage: python scripts/replay.py <recording> [--only-vision] [--plot] [--frames N] [--out DIR]
(M1: only --only-vision is implemented.)
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deskmouse import metrics  # noqa: E402
from deskmouse.io import load  # noqa: E402
from deskmouse.vo import run_vision_only  # noqa: E402


def plot(df, gt, out: Path, title: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = (df.t_ns - df.t_ns.iloc[0]) / 1e9
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(2, 2, 1)
    ax.plot(df.x * 100, df.y * 100, lw=0.8, label="estimate")
    if gt is not None:
        ax.plot(gt.x * 100, gt.y * 100, "k--", lw=0.6, label="ground truth")
    ax.set_aspect("equal"); ax.set_xlabel("X cm"); ax.set_ylabel("Y cm"); ax.legend(fontsize=8); ax.set_title(title, fontsize=9)
    ax = fig.add_subplot(2, 2, 2)
    ax.plot(t, df.x * 100, label="x"); ax.plot(t, df.y * 100, label="y")
    if gt is not None:
        tg = (gt.t_ns - df.t_ns.iloc[0]) / 1e9
        ax.plot(tg, gt.x * 100, "k--", lw=0.5); ax.plot(tg, gt.y * 100, "k:", lw=0.5)
    ax.set_ylabel("cm"); ax.legend(fontsize=8)
    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t, df.n_tracks, label="tracks"); ax.plot(t, df.n_inliers, label="inliers"); ax.legend(fontsize=8); ax.set_xlabel("s")
    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t, 1 / df.lam, label="gauge: 1/λ"); ax.set_xlabel("s"); ax.legend(fontsize=8)
    r = df.attrs.get("final_rhos")
    if r is not None and len(r):
        ax2 = ax.twinx()
        lam = np.nanmedian(df.lam)
        ax2.hist(1 / np.clip(r * lam, 1e-3, None), bins=40, range=(0, 3), alpha=0.3, color="gray", orientation="horizontal")
        ax2.set_ylabel("track depth m (final)")
    plt.tight_layout(); plt.savefig(out, dpi=80); plt.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--only-vision", action="store_true", default=True)
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--frames", type=int)
    ap.add_argument("--out")
    ap.add_argument("--freeze-gauge", type=float, help="freeze the vision-only scale gauge after N seconds")
    a = ap.parse_args()
    rec = load(a.recording)
    out = Path(a.out or Path(__file__).resolve().parents[1] / "out" / Path(a.recording).name)
    out.mkdir(parents=True, exist_ok=True)
    df = run_vision_only(rec, max_frames=a.frames, freeze_gauge_after_s=a.freeze_gauge)
    df.to_csv(out / "trajectory.csv", index=False)
    gt = rec.ground_truth()
    stats = {"recording": rec.path.name, "tag": rec.tag, "frames": len(df),
             "median_tracks": float(df.n_tracks.median()), "median_inliers": float(df.n_inliers.median()),
             "valid_frac": float(df.valid.mean()), "ref_depth_m": float(1 / np.nanmedian(df.lam)) if df.lam.notna().any() else None}
    if rec.tag.startswith("ruler"):
        stats["ruler"] = metrics.ruler_metrics(df)
    if gt is not None:
        stats["gt"] = metrics.gt_errors(df, gt)
    (out / "stats.json").write_text(json.dumps(stats, indent=1))
    print(json.dumps(stats, indent=1))
    if a.plot:
        plot(df, gt, out / "replay.png", rec.path.name)


if __name__ == "__main__":
    main()
