"""Stereo depth accuracy on synthetic scenes (no Webots): python scripts/stereo_accuracy.py

Per range and backend: wall = median signed / p90 absolute depth error of all
points on a textured wall facing the robot; box = detected obstacle distance
vs the true front face of a 0.6 x 0.5 x 0.5 m box on the lane centre; theory =
1 px of disparity at that range (Z^2 / fB). Rig = the stereo Camera nodes of --wbt.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.lane_vision import parse_wbt_cameras  # noqa: E402
from src.stereo_depth import HAVE_CV2, StereoObstacleSensor, points_from_disparity, StereoRig, _model  # noqa: E402
from src.stereo_synth import SceneBox, render_pair  # noqa: E402


def wall_error(rig, dist, backend):
    L, R = render_pair(rig, (0, 0, 0), [SceneBox.at(dist + 0.05, 0.0, 0.1, 4.0, 2.5, seed=3)], supersample=2)
    s = StereoObstacleSensor(rig, backend=backend)
    s.process_gray(L, R)
    pts, _r, _c = points_from_disparity(s.disp, rig, 8.0)
    on = (pts[:, 2] > 0.15) & (np.abs(pts[:, 1]) < 0.8)
    err = pts[on, 0] - dist
    return float(np.median(err)), float(np.percentile(np.abs(err), 90)), s.ms


def box_error(rig, dist, backend):
    L, R = render_pair(rig, (0, 0, 0), [SceneBox.at(dist + 0.3, 0.0, 0.6, 0.5, 0.5, seed=4)], supersample=2)
    rep = StereoObstacleSensor(rig, backend=backend).process_gray(L, R)
    return (rep.distance_m - dist) if rep.detected else None, rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ranges", default="0.6,1,1.5,2,3,4")
    ap.add_argument("--wbt", default=str(ROOT / "webots" / "worlds" / "butlerbot_warehouse_obstacles.wbt"),
                    help="world to read the stereo camera poses from")
    a = ap.parse_args(argv)
    cams = parse_wbt_cameras(a.wbt)
    rig = StereoRig(_model("stereo_left", cams["stereo_left"]), _model("stereo_right", cams["stereo_right"]))
    print(rig.describe())
    backends = (["sgbm"] if HAVE_CV2 else []) + ["numpy"]
    print(f"{'range_m':>7} {'backend':>7} {'wall_med_cm':>11} {'wall_p90_cm':>11} {'box_err_cm':>10} "
          f"{'1px_cm':>6} {'ms':>6}  box report")
    for d in (float(v) for v in a.ranges.split(",")):
        for be in backends:
            med, p90, ms = wall_error(rig, d, be)
            be_err, rep = box_error(rig, d, be)
            box = "miss" if be_err is None else f"{be_err * 100:+.1f}"
            print(f"{d:7.2f} {be:>7} {med * 100:+11.1f} {p90 * 100:11.1f} {box:>10} {d * d / rig.fB * 100:6.1f} "
                  f"{ms:6.0f}  {rep.band_label if rep.detected else '-'} z {rep.z_top_m}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
