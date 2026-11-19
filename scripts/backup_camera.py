"""Draw the existing planner path on a waypoint snapshot.

Harness only. The controller does not import this file. The path is
``_path_on_own_map``: A* on cells the robot has marked free, or a
straight aim when the goal cell is not free. Nothing here is a new
planner weight, and the truth pose is printed for the driver's log
only. It is never written back as a command.
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

from src.mapping.brain_port import latest_stop, read_status, write_png
from src.mapping.driver import _path_on_own_map
from src.mapping.frontiers import clusters, frontier_mask
from src.mapping.odometry import wrap_rad


# Stock spawn. The odometry frame's +x is this heading. Used only when
# comparing a snapshot with the pose log after the fact.
_SPAWN_X = 6.1
_SPAWN_Y = 4.7
_SPAWN_YAW = 1.5708


class _Grid:
    """The snapshot's own grid, with the two lookups the planner uses."""

    def __init__(self, labels: np.ndarray, origin: tuple[float, float], res: float) -> None:
        self._labels = labels
        self.origin = (float(origin[0]), float(origin[1]))
        self.res = float(res)

    def labels(self) -> np.ndarray:
        return self._labels

    def cell_of(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor((float(x) - self.origin[0]) / self.res))
        row = int(math.floor((float(y) - self.origin[1]) / self.res))
        if row < 0 or col < 0 or row >= self._labels.shape[0] or col >= self._labels.shape[1]:
            return None
        return row, col

    def cell_center(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin[0] + (int(col) + 0.5) * self.res
        y = self.origin[1] + (int(row) + 0.5) * self.res
        return x, y


class _Explorer:
    def __init__(self, grid: _Grid, pose: tuple[float, float, float]) -> None:
        self.grid = grid
        self._pose = pose

    def pose(self) -> tuple[float, float, float]:
        return self._pose


def labels_from_map(payload: dict) -> tuple[np.ndarray, tuple[float, float], float]:
    rows = payload["rows"]
    grid = np.full((len(rows), len(rows[0])), -1, dtype=np.int8)
    for r, line in enumerate(rows):
        for c, ch in enumerate(line):
            if ch == ".":
                grid[r, c] = 0
            elif ch == "#":
                grid[r, c] = 1
    origin = payload["origin_xy_m"]
    return grid, (float(origin[0]), float(origin[1])), float(payload["resolution_m"])


def planned_path(labels, origin, res, pose, goal: tuple[float, float]) -> list[tuple[float, float]]:
    """The path ``WaypointDriver`` would follow to ``goal`` on this map."""
    explorer = _Explorer(_Grid(np.asarray(labels), origin, res), pose)
    return [(float(x), float(y)) for x, y in _path_on_own_map(explorer, goal)]


def _line_cells(grid: _Grid, a: tuple[float, float], b: tuple[float, float]) -> list[tuple[int, int]]:
    start = grid.cell_of(a[0], a[1])
    end = grid.cell_of(b[0], b[1])
    if start is None or end is None:
        return []
    r0, c0 = start
    r1, c1 = end
    cells = []
    steps = max(abs(r1 - r0), abs(c1 - c0), 1)
    for i in range(steps + 1):
        t = i / steps
        cells.append((int(round(r0 + (r1 - r0) * t)), int(round(c0 + (c1 - c0) * t))))
    return cells


def path_kind(labels, origin, res, pose, goal, path) -> str:
    """``astar`` when the goal cell is free and a path was found. Else ``straight``."""
    grid = _Grid(np.asarray(labels), origin, res)
    if len(path) != 1 or path[0] != (float(goal[0]), float(goal[1])):
        return "astar"
    # A one-point path is the straight aim, including the case where the
    # robot is already on the goal cell. Distinguish that from a refusal.
    start = grid.cell_of(pose[0], pose[1])
    end = grid.cell_of(goal[0], goal[1])
    if start is not None and start == end:
        return "astar"
    return "straight"


def crosses_occupied(labels, origin, res, path) -> int:
    grid = _Grid(np.asarray(labels), origin, res)
    hits = 0
    seen = set()
    for a, b in zip(path, path[1:]):
        for cell in _line_cells(grid, a, b):
            if cell in seen:
                continue
            seen.add(cell)
            row, col = cell
            if 0 <= row < labels.shape[0] and 0 <= col < labels.shape[1] and int(labels[row, col]) == 1:
                hits += 1
    return hits


def frontiers_near(labels, origin, res, pose, limit: int = 6) -> list[dict]:
    mask = frontier_mask(np.asarray(labels))
    grid = _Grid(np.asarray(labels), origin, res)
    found = []
    for comp in clusters(mask):
        if len(comp) < 2:
            continue
        xs = []
        ys = []
        for row, col in comp:
            x, y = grid.cell_center(row, col)
            xs.append(x)
            ys.append(y)
        cx = float(sum(xs) / len(xs))
        cy = float(sum(ys) / len(ys))
        dist = math.hypot(cx - pose[0], cy - pose[1])
        bearing = wrap_rad(math.atan2(cy - pose[1], cx - pose[0]) - pose[2])
        found.append(
            {
                "x": round(cx, 2),
                "y": round(cy, 2),
                "cells": len(comp),
                "dist_m": round(dist, 2),
                "bearing_rad": round(bearing, 2),
            }
        )
    found.sort(key=lambda item: item["dist_m"])
    return found[:limit]


def _paint(labels, origin, res, pose, goal, path) -> np.ndarray:
    """Map with +x right and +y up. Green is an A* path. Orange is a straight aim."""
    grid = _Grid(np.asarray(labels), origin, float(res))
    rgb = np.full((labels.shape[0], labels.shape[1], 3), 40, dtype=np.uint8)
    rgb[labels == 0] = (230, 230, 230)
    rgb[labels == 1] = (20, 20, 20)
    kind = path_kind(labels, origin, res, pose, goal, path)
    color = (40, 170, 60) if kind == "astar" else (230, 140, 30)
    blocked = crosses_occupied(labels, origin, res, path)
    for a, b in zip(path, path[1:]):
        for row, col in _line_cells(grid, a, b):
            if 0 <= row < rgb.shape[0] and 0 <= col < rgb.shape[1]:
                here = (230, 40, 180) if blocked and int(labels[row, col]) == 1 else color
                rgb[row, col] = here
    goal_cell = grid.cell_of(goal[0], goal[1])
    if goal_cell is not None:
        row, col = goal_cell
        r0, r1 = max(0, row - 1), min(rgb.shape[0], row + 2)
        c0, c1 = max(0, col - 1), min(rgb.shape[1], col + 2)
        rgb[r0:r1, c0:c1] = (40, 90, 220)
    pose_cell = grid.cell_of(pose[0], pose[1])
    if pose_cell is not None:
        row, col = pose_cell
        r0, r1 = max(0, row - 1), min(rgb.shape[0], row + 2)
        c0, c1 = max(0, col - 1), min(rgb.shape[1], col + 2)
        rgb[r0:r1, c0:c1] = (220, 40, 40)
    nose = (pose[0] + 0.45 * math.cos(pose[2]), pose[1] + 0.45 * math.sin(pose[2]))
    for row, col in _line_cells(grid, (pose[0], pose[1]), nose):
        if 0 <= row < rgb.shape[0] and 0 <= col < rgb.shape[1]:
            rgb[row, col] = (240, 210, 40)
    # The live grid is a 20 m square. Crop to what this stop has seen,
    # plus the path, so the doorway is visible. +y stays up.
    interest = np.asarray(labels) >= 0
    for point in (*path, goal, (pose[0], pose[1])):
        cell = grid.cell_of(point[0], point[1])
        if cell is not None:
            interest[cell] = True
    ys, xs = np.nonzero(interest)
    if len(ys):
        margin = 8
        r0 = max(0, int(ys.min()) - margin)
        r1 = min(rgb.shape[0], int(ys.max()) + margin + 1)
        c0 = max(0, int(xs.min()) - margin)
        c1 = min(rgb.shape[1], int(xs.max()) + margin + 1)
        rgb = rgb[r0:r1, c0:c1]
    scale = 4 if rgb.shape[0] < 220 else 2
    rgb = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    return np.flipud(rgb)


def _gt_near(folder: Path, t_s: float) -> tuple[float, float, float, float] | None:
    path = folder / "gt_pose_log.csv"
    if not path.is_file():
        return None
    best = None
    best_dt = 1e9
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        parts = line.split(",")
        if len(parts) < 5:
            continue
        try:
            t = float(parts[0])
            sample = (t, float(parts[1]), float(parts[2]), float(parts[4]))
        except ValueError:
            continue
        dt = abs(t - float(t_s))
        if dt < best_dt:
            best_dt = dt
            best = sample
    return best


def _up_z(folder: Path, t_s: float) -> float | None:
    path = folder / "physics_log.csv"
    if not path.is_file():
        return None
    best = None
    best_dt = 1e9
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        parts = line.split(",")
        if len(parts) < 7:
            continue
        try:
            t = float(parts[0])
            up = float(parts[6])
        except ValueError:
            continue
        dt = abs(t - float(t_s))
        if dt < best_dt:
            best_dt = dt
            best = up
    return best


def belief_vs_truth(pose: dict, gt: tuple[float, float, float, float] | None) -> dict | None:
    """Odometry estimate against the pose log. The spawn is the frame origin."""
    if gt is None:
        return None
    _t, gx, gy, gyaw = gt
    x = float(pose.get("x_m") or 0.0)
    y = float(pose.get("y_m") or 0.0)
    yaw = float(pose.get("yaw_rad") or 0.0)
    c = math.cos(_SPAWN_YAW)
    s = math.sin(_SPAWN_YAW)
    wx = _SPAWN_X + x * c - y * s
    wy = _SPAWN_Y + x * s + y * c
    wyaw = wrap_rad(_SPAWN_YAW + yaw)
    return {
        "world_est": (round(wx, 3), round(wy, 3), round(wyaw, 3)),
        "truth": (round(gx, 3), round(gy, 3), round(gyaw, 3)),
        "pos_err_m": round(math.hypot(wx - gx, wy - gy), 3),
        "yaw_err_rad": round(wrap_rad(wyaw - gyaw), 3),
    }


def show(folder: Path) -> dict | None:
    folder = Path(folder)
    stop = latest_stop(folder)
    if stop is None:
        print(f"no snapshot in {folder / 'driver'}")
        return None
    status = read_status(stop)
    pose = status.get("pose") or {}
    travel = status.get("travel") or {}
    print(f"snapshot {stop}")
    print(
        f"status {status.get('status')}  t={status.get('t_s')}  "
        f"pose=({pose.get('x_m')}, {pose.get('y_m')}, {pose.get('yaw_rad')})  "
        f"detail={status.get('detail')}"
    )
    print(
        f"travel commanded={travel.get('commanded_m')} wheel={travel.get('wheel_m')} "
        f"floor={travel.get('floor_m')} reliable={travel.get('floor_reliable')}"
    )
    gt = _gt_near(folder, float(status.get("t_s") or 0.0))
    compared = belief_vs_truth(pose, gt)
    up = _up_z(folder, float(status.get("t_s") or 0.0))
    if compared is None:
        print("truth none")
    else:
        print(
            f"truth {compared['truth']}  est_world {compared['world_est']}  "
            f"pos_err={compared['pos_err_m']} yaw_err={compared['yaw_err_rad']} up_z={up}"
        )
    payload = json.loads((stop / "map.json").read_text(encoding="utf-8"))
    labels, origin, res = labels_from_map(payload)
    free = int(np.sum(labels == 0))
    occ = int(np.sum(labels == 1))
    print(f"map free={free} occupied={occ} res={res}")
    here = (float(pose.get("x_m") or 0.0), float(pose.get("y_m") or 0.0), float(pose.get("yaw_rad") or 0.0))
    openings = frontiers_near(labels, origin, res, here)
    if openings:
        print("openings " + " | ".join(
            f"({item['x']},{item['y']}) n={item['cells']} d={item['dist_m']} b={item['bearing_rad']}"
            for item in openings
        ))
    else:
        print("openings none")
    return status


def draw(folder: Path, goal: tuple[float, float], out: Path | None) -> Path:
    folder = Path(folder)
    stop = latest_stop(folder)
    if stop is None:
        raise SystemExit(f"no snapshot in {folder / 'driver'}")
    status = read_status(stop)
    pose = status.get("pose") or {}
    here = (float(pose.get("x_m") or 0.0), float(pose.get("y_m") or 0.0), float(pose.get("yaw_rad") or 0.0))
    payload = json.loads((stop / "map.json").read_text(encoding="utf-8"))
    labels, origin, res = labels_from_map(payload)
    path = planned_path(labels, origin, res, here, goal)
    kind = path_kind(labels, origin, res, here, goal, path)
    blocked = crosses_occupied(labels, origin, res, path)
    length = 0.0
    for a, b in zip(path, path[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])
    image = _paint(labels, origin, res, here, goal, path)
    dest = out if out is not None else stop / "backup_camera.png"
    write_png(dest, image)
    note = {
        "goal_xy_m": [goal[0], goal[1]],
        "kind": kind,
        "length_m": round(length, 3),
        "points": [[round(x, 3), round(y, 3)] for x, y in path],
        "occupied_cells_crossed": blocked,
        "pose": {"x_m": here[0], "y_m": here[1], "yaw_rad": here[2]},
    }
    dest.with_suffix(".json").write_text(json.dumps(note, indent=2) + "\n", encoding="utf-8")
    print(f"path {kind} length={length:.2f} points={len(path)} occupied_crossed={blocked}")
    print(f"wrote {dest}")
    return dest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Show a stop and draw the existing planner path.")
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("cmd", nargs="?", choices=["show", "draw"], default="show")
    parser.add_argument("x", nargs="?", type=float)
    parser.add_argument("y", nargs="?", type=float)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    if args.cmd == "draw":
        if args.x is None or args.y is None:
            raise SystemExit("draw needs x y in the robot's odometry frame")
        draw(args.run, (args.x, args.y), args.out)
        return 0
    show(args.run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
