"""Headless loop: four fixed waypoints, then stop.

This process is the harness. It may read the pose logger afterwards to
score whether the shell moved. The driver and the snapshot port do not.

The aims are in the robot's odometry frame. At the stock spawn that frame's
+x is the robot's initial heading. The driver is not given these numbers
except as command files, the same way an outside planner would.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.mapping.brain_port import publish_command, read_status

WEBOTS = ROOT / "webots"


def _webots_bin() -> Path:
    home = os.environ.get("WEBOTS_HOME", "")
    candidates = []
    if home:
        candidates.append(Path(home) / "webots")
    candidates.extend([Path("/opt/webots/webots"), Path("/usr/local/webots/webots")])
    for path in candidates:
        if path.is_file():
            return path
    return Path("webots")
# Short aims on the open floor in front of the stock spawn.
WAYPOINTS = (
    {"cmd": "go_to", "x": 0.6, "y": 0.0},
    {"cmd": "go_to", "x": 1.1, "y": 0.0},
    {"cmd": "turn_to", "yaw": 0.4},
    {"cmd": "look"},
)


def _wait_status(folder: Path, proc: subprocess.Popen, previous: str, timeout_s: float) -> tuple[str, dict]:
    pointer = folder / "driver" / "latest.txt"
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if proc.poll() is not None:
            raise SystemExit(f"webots exited {proc.returncode} while waiting\n{_tail(folder)}")
        if pointer.is_file():
            name = pointer.read_text(encoding="utf-8").strip()
            status_path = folder / "driver" / name / "status.json"
            if name and name != previous and status_path.is_file():
                return name, read_status(status_path.parent)
        time.sleep(0.1)
    raise SystemExit(f"no new snapshot after {previous!r}\n{_tail(folder)}")


def _tail(folder: Path) -> str:
    log = folder / "webots.log"
    if not log.is_file():
        return "(no webots.log)"
    lines = log.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(lines[-80:])


def _print_stop(folder: Path, waypoint: dict, status: dict) -> None:
    pose = status.get("pose") or {}
    travel = status.get("travel") or {}
    gt = _gt_near(folder, float(status.get("t_s") or 0.0))
    target = ""
    if waypoint.get("cmd") == "go_to":
        target = f"target=({waypoint['x']:.2f},{waypoint['y']:.2f}) "
    elif waypoint.get("cmd") == "turn_to":
        target = f"target_yaw={waypoint['yaw']:.2f} "
    gt_txt = "none" if gt is None else f"({gt[0]:.3f},{gt[1]:.3f},{gt[2]:.3f})"
    print(
        f"SMOKE {waypoint['cmd']} {target}"
        f"est=({pose.get('x_m')},{pose.get('y_m')},{pose.get('yaw_rad')}) "
        f"wheel={travel.get('wheel_m')} floor={travel.get('floor_m')} "
        f"gt={gt_txt} status={status.get('status')} remaining={status.get('remaining_m')}",
        flush=True,
    )


def _gt_near(folder: Path, t_s: float) -> tuple[float, float, float] | None:
    """Simulator pose nearest ``t_s``. The harness log only. The driver does not read it."""
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
            sample = (float(parts[1]), float(parts[2]), float(parts[4]))
        except ValueError:
            continue
        dt = abs(t - float(t_s))
        if dt < best_dt:
            best_dt = dt
            best = sample
    return best


def _gt_travel(folder: Path) -> float | None:
    path = folder / "gt_pose_log.csv"
    if not path.is_file():
        return None
    rows = path.read_text(encoding="utf-8").splitlines()
    if len(rows) < 3:
        return None
    def xy(line: str) -> tuple[float, float]:
        parts = line.split(",")
        return float(parts[1]), float(parts[2])
    x0, y0 = xy(rows[1])
    x1, y1 = xy(rows[-1])
    return math.hypot(x1 - x0, y1 - y0)


def main() -> int:
    folder = Path(os.environ.get("RBM_DRIVER_SMOKE_DIR", "/tmp/driver_smoke"))
    if folder.exists():
        for child in sorted(folder.rglob("*"), reverse=True):
            if child.is_file():
                child.unlink()
            elif child.is_dir():
                child.rmdir()
    folder.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    env["PYTHONUNBUFFERED"] = "1"
    env["RBM_DRIVER"] = "1"
    env["RBM_MAP_DIR"] = str(folder)
    env["RBM_MAP_MAX_S"] = "150"
    env["RBM_GYRO_CAL_S"] = "1.5"
    env["RBM_DEPTH_SEED"] = "1"
    env.pop("RBM_ORACLE_POSE", None)
    env.pop("RBM_DRIVE", None)
    env.pop("RBM_MOVIE", None)
    env.pop("RBM_STRAIGHT_S", None)
    binary = _webots_bin()
    if binary.is_file():
        env["WEBOTS_HOME"] = str(binary.parent)
        env["PATH"] = str(binary.parent) + os.pathsep + env.get("PATH", "")
    log = open(folder / "webots.log", "w", encoding="utf-8")
    proc = subprocess.Popen(
        [
            "xvfb-run",
            "-a",
            "-s",
            "-screen 0 1280x720x24",
            str(binary),
            "--mode=fast",
            "--fullscreen",
            "--batch",
            "--stdout",
            "--stderr",
            "--port=1733",
            "worlds/apartment.wbt",
        ],
        cwd=str(WEBOTS),
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    seen: list[dict] = []
    try:
        previous = ""
        previous, status = _wait_status(folder, proc, previous, 180.0)
        seen.append(status)
        if status.get("status") != "ready":
            raise SystemExit(f"first snapshot was {status.get('status')}\n{_tail(folder)}")
        for waypoint in WAYPOINTS:
            publish_command(folder, waypoint)
            previous, status = _wait_status(folder, proc, previous, 120.0)
            seen.append(status)
            _print_stop(folder, waypoint, status)
        publish_command(folder, {"cmd": "stop"})
        previous, status = _wait_status(folder, proc, previous, 60.0)
        seen.append(status)
        print(f"SMOKE stop -> {status.get('status')}", flush=True)
        deadline = time.time() + 60.0
        while time.time() < deadline and proc.poll() is None:
            if (folder / "mapping_done").is_file():
                proc.wait(timeout=30)
                break
            time.sleep(0.2)
        if proc.poll() is None:
            raise SystemExit(f"webots did not exit after stop\n{_tail(folder)}")
    finally:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=10)
        log.close()
    problems = []
    if seen[0].get("status") != "ready":
        problems.append(f"ready was {seen[0].get('status')}")
    if seen[-1].get("status") != "stopped":
        problems.append(f"stop was {seen[-1].get('status')}")
    for item, status in zip(WAYPOINTS, seen[1:-1]):
        if status.get("status") not in (
            "arrived",
            "stuck",
            "blocked",
            "stopped_short",
            "timeout",
            "tilt_warning",
        ):
            problems.append(f"{item['cmd']} status {status.get('status')}")
        if (status.get("pose") or {}).get("frame") != "odometry":
            problems.append(f"{item['cmd']} pose frame")
    for index, (x_m, y_m) in ((1, (0.6, 0.0)), (2, (1.1, 0.0))):
        status = seen[index]
        pose = status.get("pose") or {}
        gap = math.hypot(float(pose.get("x_m") or 0.0) - x_m, float(pose.get("y_m") or 0.0) - y_m)
        if status.get("status") != "arrived" or gap > 0.08:
            problems.append(
                f"go_to ({x_m},{y_m}) {status.get('status')} pose={pose} gap={gap:.3f} "
                f"detail={status.get('detail')}"
            )
    outcome = folder / "driver" / "stops" / "0001"
    for name in ("status.json", "map.json", "map.png", "oak.png"):
        if not (outcome / name).is_file():
            problems.append(f"missing {outcome / name}")
    gt = _gt_travel(folder)
    print(f"SMOKE gt_travel_m={gt}", flush=True)
    if gt is None or gt < 0.9:
        problems.append(f"harness travel {gt}")
    summary = {
        "exit": proc.returncode,
        "statuses": [item.get("status") for item in seen],
        "first_stop": str(outcome),
        "gt_travel_m": gt,
    }
    print("SMOKE " + json.dumps(summary), flush=True)
    if problems:
        print("SMOKE FAIL", flush=True)
        for item in problems:
            print(f"  {item}", flush=True)
        print(_tail(folder), flush=True)
        return 1
    print("SMOKE PASS", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
