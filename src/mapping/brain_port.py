"""Snapshots and the command file for the waypoint driver.

A stop writes the robot's own camera frames, its own occupancy grid,
its own pose, and the travel it measured. The controller then stops
calling step until ``command.json`` appears. Nothing here reads the
simulator pose.
"""

from __future__ import annotations

import json
import math
import os
import struct
import time
import zlib
from pathlib import Path

import numpy as np

from src.mapping.driver import parse_command


def driver_dir(run_dir: Path | str) -> Path:
    folder = Path(run_dir) / "driver"
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def publish_command(run_dir: Path | str, payload: dict) -> Path:
    """Write ``driver/command.json`` in one rename. Raises if the payload is bad."""
    cmd = parse_command(payload)
    folder = driver_dir(run_dir)
    dest = folder / "command.json"
    tmp = folder / "command.json.tmp"
    tmp.write_text(json.dumps(cmd) + "\n", encoding="utf-8")
    os.replace(tmp, dest)
    return dest


def take_command(run_dir: Path | str) -> dict | None:
    """Parse and remove ``command.json``. None if it is missing or half-written.

    A file that parses as JSON but is not a command is moved aside.
    """
    path = driver_dir(run_dir) / "command.json"
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    try:
        cmd = parse_command(data)
    except ValueError:
        path.replace(path.with_name("command.bad.json"))
        raise
    path.replace(path.with_name("command.accepted.json"))
    return cmd


def wait_for_command(run_dir: Path | str, *, poll_s: float = 0.05) -> dict:
    """Block until a command file parses. This does not step a simulator."""
    while True:
        try:
            cmd = take_command(run_dir)
        except ValueError as exc:
            print(f"MAPPING driver rejected command: {exc}", flush=True)
            cmd = None
        if cmd is not None:
            return cmd
        time.sleep(poll_s)


def write_stop(
    run_dir: Path | str,
    index: int,
    report: dict,
    t_s: float,
    images: dict,
    labels,
    meta: dict,
) -> Path:
    """Write one stop folder and point ``latest.txt`` at it last."""
    folder = driver_dir(run_dir)
    stop = folder / "stops" / f"{int(index):04d}"
    stop.mkdir(parents=True, exist_ok=True)
    files: list[str] = []
    for name, image in images.items():
        if image is None:
            continue
        filename = f"{name}.png"
        write_png(stop / filename, image)
        files.append(filename)
    pose = (report.get("pose") or {})
    pose_xy = (float(pose.get("x_m", 0.0)), float(pose.get("y_m", 0.0)))
    frontier = report.get("frontier") if isinstance(report.get("frontier"), dict) else None
    path = list((frontier or {}).get("path_xy_m") or [])
    goal = (frontier or {}).get("goal_xy_m")
    grid = _grid_payload(labels, meta, pose_xy)
    (stop / "map.json").write_text(json.dumps(grid) + "\n", encoding="utf-8")
    write_png(stop / "map.png", _color_map(labels, meta, pose_xy, path, goal))
    files.extend(["map.json", "map.png"])
    body = dict(report)
    body["t_s"] = round(float(t_s), 4)
    body["index"] = int(index)
    body["files"] = files
    (stop / "status.json").write_text(json.dumps(body, indent=2) + "\n", encoding="utf-8")
    files.append("status.json")
    body["files"] = files
    (stop / "status.json").write_text(json.dumps(body, indent=2) + "\n", encoding="utf-8")
    relative = f"stops/{int(index):04d}"
    tmp = folder / "latest.txt.tmp"
    tmp.write_text(relative + "\n", encoding="utf-8")
    os.replace(tmp, folder / "latest.txt")
    return stop


def latest_stop(run_dir: Path | str) -> Path | None:
    folder = Path(run_dir) / "driver"
    pointer = folder / "latest.txt"
    if not pointer.is_file():
        return None
    relative = pointer.read_text(encoding="utf-8").strip()
    if not relative:
        return None
    stop = folder / relative
    if not (stop / "status.json").is_file():
        return None
    return stop


def read_status(stop: Path) -> dict:
    return json.loads((Path(stop) / "status.json").read_text(encoding="utf-8"))


def write_png(path: Path | str, image) -> None:
    """RGB or gray PNG. Stdlib only, so a headless run needs no image library."""
    arr = np.asarray(image)
    if arr.ndim == 2:
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    elif arr.shape[2] == 4:
        arr = arr[:, :, :3]
    arr = np.ascontiguousarray(np.clip(arr, 0, 255).astype(np.uint8))
    height, width = int(arr.shape[0]), int(arr.shape[1])
    raw = b"".join(b"\x00" + arr[row].tobytes() for row in range(height))
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", zlib.compress(raw, 6)) + _chunk(b"IEND", b"")
    dest = Path(path)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(png)


def _chunk(tag: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def _grid_payload(labels, meta: dict, pose_xy: tuple[float, float]) -> dict:
    grid = np.asarray(labels)
    chars = np.full(grid.shape, "-", dtype="U1")
    chars[grid == 0] = "."
    chars[grid == 1] = "#"
    rows = ["".join(line.tolist()) for line in chars]
    payload = dict(meta or {})
    payload["frame"] = "odometry"
    payload["legend"] = {"unknown": "-", "free": ".", "occupied": "#"}
    payload["pose_xy_m"] = [pose_xy[0], pose_xy[1]]
    payload["rows"] = rows
    return payload


def format_frontier(proposal) -> str:
    """One line for the mapper's frontier proposal. ``none`` when it has no goal."""
    if not isinstance(proposal, dict):
        return "none"
    goal = proposal.get("goal_xy_m")
    if not goal or len(goal) < 2:
        return "none"
    path = proposal.get("path_xy_m") or []
    return f"({float(goal[0]):.2f},{float(goal[1]):.2f}) path={len(path)}"


def _cell_of(origin, res, shape, x: float, y: float) -> tuple[int, int] | None:
    col = int(math.floor((float(x) - float(origin[0])) / float(res)))
    row = int(math.floor((float(y) - float(origin[1])) / float(res)))
    if row < 0 or col < 0 or row >= shape[0] or col >= shape[1]:
        return None
    return row, col


def _segment_cells(origin, res, shape, a, b) -> list[tuple[int, int]]:
    start = _cell_of(origin, res, shape, a[0], a[1])
    end = _cell_of(origin, res, shape, b[0], b[1])
    if start is None or end is None:
        return []
    r0, c0 = start
    r1, c1 = end
    steps = max(abs(r1 - r0), abs(c1 - c0), 1)
    cells = []
    for i in range(steps + 1):
        t = i / steps
        cells.append((int(round(r0 + (r1 - r0) * t)), int(round(c0 + (c1 - c0) * t))))
    return cells


def _color_map(labels, meta: dict, pose_xy: tuple[float, float], path=None, goal_xy=None):
    grid = np.asarray(labels)
    rgb = np.full((grid.shape[0], grid.shape[1], 3), 40, dtype=np.uint8)
    rgb[grid == 0] = (230, 230, 230)
    rgb[grid == 1] = (20, 20, 20)
    origin = (meta or {}).get("origin_xy_m")
    res = (meta or {}).get("resolution_m")
    # Green is the frontier path ``select_goal`` already returned. A cell
    # of that path that is occupied is magenta, same as the harness overlay.
    if origin and res and path:
        points = [(float(p[0]), float(p[1])) for p in path if p and len(p) >= 2]
        for a, b in zip(points, points[1:]):
            for row, col in _segment_cells(origin, res, rgb.shape, a, b):
                if 0 <= row < rgb.shape[0] and 0 <= col < rgb.shape[1]:
                    rgb[row, col] = (230, 40, 180) if int(grid[row, col]) == 1 else (40, 170, 60)
    if origin and res and goal_xy and len(goal_xy) >= 2:
        cell = _cell_of(origin, res, rgb.shape, goal_xy[0], goal_xy[1])
        if cell is not None:
            row, col = cell
            r0, c0 = max(0, row - 1), max(0, col - 1)
            rgb[r0 : row + 2, c0 : col + 2] = (40, 90, 220)
    if origin and res:
        cell = _cell_of(origin, res, rgb.shape, pose_xy[0], pose_xy[1])
        if cell is not None:
            row, col = cell
            r0, c0 = max(0, row - 1), max(0, col - 1)
            rgb[r0 : row + 2, c0 : col + 2] = (220, 40, 40)
    return rgb
