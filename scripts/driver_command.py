"""Read the latest waypoint snapshot and, if asked, write the next command.

The pose in the snapshot is the robot's own estimate. This script does
not open the simulator pose log.

Examples, from the repo root::

    python scripts/driver_command.py --run /tmp/driver_smoke
    python scripts/driver_command.py --run /tmp/driver_smoke go_to 0.6 0
    python scripts/driver_command.py --run /tmp/driver_smoke turn_to 0.4
    python scripts/driver_command.py --run /tmp/driver_smoke look
    python scripts/driver_command.py --run /tmp/driver_smoke stop
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.mapping.brain_port import format_frontier, latest_stop, publish_command, read_status


def run_dir_from(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit)
    env = (os.environ.get("RBM_MAP_DIR") or "").strip()
    if env:
        return Path(env)
    pointer = ROOT / "logs" / "mapping_runs" / "CURRENT"
    if pointer.is_file():
        line = pointer.read_text(encoding="utf-8").strip()
        if line:
            return Path(line)
    raise SystemExit("pass --run, or set RBM_MAP_DIR")


def show(folder: Path) -> dict | None:
    stop = latest_stop(folder)
    if stop is None:
        print(f"no snapshot in {folder / 'driver'}")
        return None
    status = read_status(stop)
    pose = status.get("pose") or {}
    travel = status.get("travel") or {}
    battery = status.get("battery") or {}
    print(f"snapshot {stop}")
    print(
        f"status {status.get('status')}  t={status.get('t_s')}  "
        f"pose=({pose.get('x_m')}, {pose.get('y_m')}, {pose.get('yaw_rad')})  "
        f"frame={pose.get('frame')}"
    )
    print(
        f"travel commanded={travel.get('commanded_m')}  "
        f"wheel={travel.get('wheel_m')}  floor={travel.get('floor_m')}"
    )
    print(f"battery {battery.get('pct')}%  detail {status.get('detail')}")
    if "frontier" in status:
        print(f"frontier {format_frontier(status.get('frontier'))}")
    return status


def command_from(args) -> dict | None:
    if not args.cmd:
        return None
    if args.cmd == "go_to":
        if args.a is None or args.b is None:
            raise SystemExit("go_to needs x y, in the robot's odometry frame")
        return {"cmd": "go_to", "x": args.a, "y": args.b}
    if args.cmd == "turn_to":
        if args.a is None:
            raise SystemExit("turn_to needs a heading in radians")
        return {"cmd": "turn_to", "yaw": args.a}
    if args.cmd in ("look", "scan"):
        return {"cmd": "look"}
    if args.cmd == "stop":
        return {"cmd": "stop"}
    raise SystemExit(f"unknown command {args.cmd}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Read a waypoint snapshot and write a command.")
    parser.add_argument("--run", help="Run folder that contains driver/")
    parser.add_argument("cmd", nargs="?", choices=["go_to", "turn_to", "look", "scan", "stop"])
    parser.add_argument("a", nargs="?", type=float)
    parser.add_argument("b", nargs="?", type=float)
    args = parser.parse_args(argv)
    folder = run_dir_from(args.run)
    show(folder)
    payload = command_from(args)
    if payload is None:
        return 0
    path = publish_command(folder, payload)
    print(f"wrote {path}")
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
