"""Generate a synthetic recording.

usage: python scripts/simulate.py OUT_DIR --tag ruler_x --scene multidepth [--speed 1] [--seed 0] [--mp4] [--seconds N]
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deskmouse.sim import simulate  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--tag", default="ruler_x")
ap.add_argument("--scene", default="multidepth", choices=["plane", "lowtex", "multidepth", "distractor"])
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--mp4", action="store_true")
ap.add_argument("--seconds", type=float)
a = ap.parse_args()
print(simulate(a.out, a.tag, a.scene, a.speed, seed=a.seed, raw=not a.mp4, max_seconds=a.seconds))
