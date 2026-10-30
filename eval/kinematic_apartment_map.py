"""Offline exploration of the apartment geometry.

This is a stand-in for Webots: a kinematic robot, a ray-cast depth
camera, and the same mapper the simulator runs. It reads the world
file and the ground-truth JSON. The robot controller does not.

From the repo root::

    python eval/kinematic_apartment_map.py --steps 250 --noise
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from eval.apartment_gt_grid import load_solids, rasterize, spawn_pose  # noqa: E402
from eval.depth_scene import Box  # noqa: E402
from eval.kinematic_runner import run_scene  # noqa: E402
from eval.score_apartment_map import format_scores, score_run  # noqa: E402
from src.mapping.body import OAK_DEPTH, DepthCamera  # noqa: E402
from src.mapping.run_log import default_run_dir, publish_current  # noqa: E402


def obstacle_boxes(solids) -> list[Box]:
    boxes = []
    for solid in solids:
        if solid.name.startswith("APT_FLOOR_"):
            continue
        if solid.zmax <= 0.02:
            continue
        boxes.append(Box(solid.xmin, solid.ymin, solid.zmin, solid.xmax, solid.ymax, solid.zmax))
    return boxes


def test_camera(width: int = 80, height: int = 50) -> DepthCamera:
    """Same mount and field of view as oak_depth, fewer pixels."""
    return DepthCamera(
        name=OAK_DEPTH.name,
        translation=OAK_DEPTH.translation,
        rotation=OAK_DEPTH.rotation,
        width=width,
        height=height,
        fov_rad=OAK_DEPTH.fov_rad,
        min_range_m=OAK_DEPTH.min_range_m,
        max_range_m=OAK_DEPTH.max_range_m,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Kinematic apartment mapping stand-in.")
    parser.add_argument("--steps", type=int, default=180)
    parser.add_argument("--dt", type=float, default=0.2)
    parser.add_argument("--noise", action="store_true")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--width", type=int, default=64)
    parser.add_argument("--height", type=int, default=40)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args(argv)
    solids = load_solids()
    boxes = obstacle_boxes(solids)
    sx, sy, yaw = spawn_pose()
    out = args.out or default_run_dir(ROOT)
    publish_current(ROOT, out)
    result = run_scene(
        boxes,
        start=(sx, sy, yaw),
        steps=args.steps,
        dt_s=args.dt,
        noise=args.noise,
        seed=args.seed,
        camera=test_camera(args.width, args.height),
        stride=2,
        run_dir=out,
    )
    # Raster at the map resolution so the scorer and the rays agree.
    _ = rasterize(solids)
    scores = score_run(out)
    print(f"steps: {result.steps}")
    print(f"done: {result.explorer.done}")
    print(f"stalled: {result.stalled}")
    print(f"true_pose: {result.pose.x:.2f} {result.pose.y:.2f} {result.pose.yaw:.3f}")
    print(format_scores(scores), end="")
    print(f"run: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
