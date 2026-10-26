"""Score an apartment occupancy grid against the measured shell.

The mapper must not import this file. Alignment uses the known spawn
pose (JSON translation, world-file heading), not the robot's estimate.

Reported numbers:

* occupancy IoU — predicted occupied vs walls, doors, and furniture
* free-space IoU — predicted free vs floor that is not occupied
* coverage % — share of free floor cells the map has labelled
* wall error cm — mean distance from a mapped obstacle boundary to the
  nearest ground-truth obstacle boundary (walls and furniture). A
  perfect raster is a fraction of one cell. Furniture that was mapped
  is part of the boundary, so a correct sofa does not look like a
  missed wall.

Usage (from the repo root)::

    python eval/score_apartment_map.py logs/mapping_runs/<run>
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from eval.apartment_gt_grid import GtGrid, rasterize, spawn_pose  # noqa: E402


def load_run(path: Path) -> tuple[np.ndarray, dict]:
    folder = path if path.is_dir() else path.parent
    npy = path if path.suffix == ".npy" else folder / "map.npy"
    meta_path = folder / "map_meta.json"
    labels = np.load(npy)
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    return np.asarray(labels), meta


def warp_to_ground_truth(
    labels: np.ndarray,
    meta: dict,
    gt: GtGrid,
    spawn_xy: tuple[float, float],
    yaw_rad: float,
) -> np.ndarray:
    """Sample the odometry map at each ground-truth cell centre."""
    res = gt.resolution_m
    height, width = gt.labels.shape
    cols = np.arange(width, dtype=np.float64)
    rows = np.arange(height, dtype=np.float64)
    cc, rr = np.meshgrid(cols, rows)
    xw = gt.origin_xy[0] + (cc + 0.5) * res
    yw = gt.origin_xy[1] + (rr + 0.5) * res
    dx = xw - float(spawn_xy[0])
    dy = yw - float(spawn_xy[1])
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    xo = c * dx + s * dy
    yo = -s * dx + c * dy
    origin_x, origin_y = (float(v) for v in meta["origin_xy_m"])
    map_res = float(meta["resolution_m"])
    mc = np.floor((xo - origin_x) / map_res).astype(np.int32)
    mr = np.floor((yo - origin_y) / map_res).astype(np.int32)
    mh, mw = labels.shape
    inside = (mr >= 0) & (mc >= 0) & (mr < mh) & (mc < mw)
    pred = np.full(gt.labels.shape, -1, dtype=np.int8)
    pred[inside] = labels[mr[inside], mc[inside]]
    return pred


def _iou(left: np.ndarray, right: np.ndarray, domain: np.ndarray) -> float:
    a = left & domain
    b = right & domain
    inter = int(np.count_nonzero(a & b))
    union = int(np.count_nonzero(a | b))
    if union == 0:
        return 1.0
    return inter / union


def _boundary(occupied: np.ndarray, free: np.ndarray) -> np.ndarray:
    neighbor = np.zeros_like(occupied)
    neighbor[1:, :] |= free[:-1, :]
    neighbor[:-1, :] |= free[1:, :]
    neighbor[:, 1:] |= free[:, :-1]
    neighbor[:, :-1] |= free[:, 1:]
    return occupied & neighbor


def wall_error_m(gt: np.ndarray, pred: np.ndarray, resolution_m: float) -> float:
    gt_edge = _boundary(gt == 1, gt == 0)
    pred_edge = _boundary(pred == 1, pred == 0)
    gy, gx = np.nonzero(gt_edge)
    py, px = np.nonzero(pred_edge)
    if len(py) == 0 or len(gy) == 0:
        return float("nan")
    gt_pts = np.column_stack((gx.astype(np.float64), gy.astype(np.float64))) * resolution_m
    mins = []
    for start in range(0, len(py), 500):
        block = np.column_stack(
            (px[start : start + 500].astype(np.float64), py[start : start + 500].astype(np.float64))
        )
        delta = block[:, None, :] * resolution_m - gt_pts[None, :, :]
        mins.append(np.hypot(delta[..., 0], delta[..., 1]).min(axis=1))
    return float(np.mean(np.concatenate(mins)))


def score_labels(gt: GtGrid, pred: np.ndarray) -> dict:
    domain = gt.labels >= 0
    gt_free = gt.labels == 0
    gt_occ = gt.labels == 1
    pred_free = pred == 0
    pred_occ = pred == 1
    known = pred >= 0
    free_count = int(np.count_nonzero(gt_free))
    coverage = (int(np.count_nonzero(known & gt_free)) / free_count) if free_count else 0.0
    err_m = wall_error_m(gt.labels, pred, gt.resolution_m)
    return {
        "occupancy_iou": _iou(pred_occ, gt_occ, domain),
        "free_iou": _iou(pred_free, gt_free, domain),
        "coverage_pct": 100.0 * coverage,
        "wall_error_cm": err_m * 100.0,
    }


def score_run(path: Path, gt: GtGrid | None = None) -> dict:
    labels, meta = load_run(path)
    grid = gt if gt is not None else rasterize(resolution_m=float(meta.get("resolution_m", 0.05)))
    sx, sy, yaw = spawn_pose()
    pred = warp_to_ground_truth(labels, meta, grid, (sx, sy), yaw)
    return score_labels(grid, pred)


def _paint(path: Path, gt: GtGrid, pred: np.ndarray, scores: dict) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def _rgb(labels: np.ndarray) -> np.ndarray:
        rgb = np.zeros(labels.shape + (3,), dtype=np.float32)
        rgb[labels < 0] = (0.78, 0.78, 0.76)
        rgb[labels == 0] = (0.96, 0.94, 0.90)
        rgb[labels == 1] = (0.15, 0.15, 0.16)
        return rgb

    disagree = (gt.labels >= 0) & (pred >= 0) & (gt.labels != pred)
    unknown = (gt.labels >= 0) & (pred < 0)
    overlay = _rgb(pred)
    overlay[disagree] = (0.75, 0.22, 0.18)
    overlay[unknown] = (0.45, 0.55, 0.70)
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.4), dpi=120)
    panels = (
        (axes[0], _rgb(gt.labels), "Ground truth"),
        (axes[1], _rgb(pred), "Mapped (spawn-aligned)"),
        (axes[2], overlay, "Red = disagree, blue = unseen"),
    )
    for ax, image, title in panels:
        ax.imshow(image, origin="lower", interpolation="nearest")
        ax.set_title(title, fontsize=10)
        ax.set_xticks([])
        ax.set_yticks([])
    wall = scores["wall_error_cm"]
    wall_txt = "n/a" if wall != wall else f"{wall:.1f} cm"
    fig.suptitle(
        "occupancy IoU {occupancy_iou:.3f}   free IoU {free_iou:.3f}   "
        "coverage {coverage_pct:.1f}%   wall error {wall}".format(wall=wall_txt, **scores),
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def format_scores(scores: dict) -> str:
    wall = scores["wall_error_cm"]
    wall_txt = "nan" if wall != wall else f"{wall:.2f}"
    return (
        f"occupancy_iou: {scores['occupancy_iou']:.4f}\n"
        f"free_iou: {scores['free_iou']:.4f}\n"
        f"coverage_pct: {scores['coverage_pct']:.2f}\n"
        f"wall_error_cm: {wall_txt}\n"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Score an apartment mapping run.")
    parser.add_argument("run", type=Path, help="Run folder with map.npy and map_meta.json")
    parser.add_argument(
        "-o",
        "--png",
        type=Path,
        default=None,
        help="Comparison PNG (default: <run>/map_vs_ground_truth.png)",
    )
    args = parser.parse_args(argv)
    run = args.run
    labels, meta = load_run(run)
    gt = rasterize(resolution_m=float(meta.get("resolution_m", 0.05)))
    sx, sy, yaw = spawn_pose()
    pred = warp_to_ground_truth(labels, meta, gt, (sx, sy), yaw)
    scores = score_labels(gt, pred)
    text = format_scores(scores)
    print(text, end="")
    folder = run if run.is_dir() else run.parent
    (folder / "scores.txt").write_text(text, encoding="utf-8")
    png = args.png or (folder / "map_vs_ground_truth.png")
    _paint(png, gt, pred, scores)
    print(f"png: {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
