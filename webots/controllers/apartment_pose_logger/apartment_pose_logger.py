"""Record the simulator pose for scoring. Not imported by the mapper.

Writes gt_pose_log.csv into the run folder advertised by
logs/mapping_runs/CURRENT (or RBM_MAP_DIR). Columns are simulator time
and the robot node's true pose. The mapper never reads this file.
"""

from __future__ import annotations

import math
import os
import sys

from controller import Supervisor

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_POINTER = os.path.join(_PROJECT_ROOT, "logs", "mapping_runs", "CURRENT")


def _run_dir() -> str:
    override = (os.environ.get("RBM_MAP_DIR") or "").strip()
    if override:
        return override
    try:
        with open(_POINTER, encoding="utf-8") as handle:
            line = handle.readline().strip()
    except OSError:
        return ""
    return line


def _find_robot(supervisor):
    named = supervisor.getFromDef("ButlerBot")
    if named is not None:
        return named
    root = supervisor.getRoot()
    children = root.getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        field = node.getField("name")
        if field is None:
            continue
        try:
            if field.getSFString() == "ButlerBot":
                return node
        except Exception:
            continue
    return None


def _yaw(node) -> float:
    matrix = node.getOrientation()
    return math.atan2(float(matrix[3]), float(matrix[0]))


def main() -> None:
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    robot = None
    handle = None
    folder = ""
    next_try = 0.0
    while supervisor.step(timestep) != -1:
        now = float(supervisor.getTime())
        if robot is None:
            robot = _find_robot(supervisor)
            if robot is None:
                continue
        if handle is None and now >= next_try:
            folder = _run_dir()
            next_try = now + 0.5
            if not folder:
                continue
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, "gt_pose_log.csv")
            handle = open(path, "w", encoding="utf-8", newline="\n")
            handle.write("t_s,x_m,y_m,z_m,yaw_rad\n")
            print(f"POSE LOG {path}")
        if handle is None or robot is None:
            continue
        pos = robot.getPosition()
        yaw = _yaw(robot)
        handle.write(
            f"{now:.4f},{float(pos[0]):.5f},{float(pos[1]):.5f},{float(pos[2]):.5f},{yaw:.6f}\n"
        )
        handle.flush()
        # The mapper writes this file when it finishes. Only a supervisor
        # can stop the world, and the mapper is not one.
        if folder and os.path.isfile(os.path.join(folder, "mapping_done")):
            handle.flush()
            handle.close()
            print(f"POSE LOG mapping finished, quitting t={now:.1f}s")
            supervisor.simulationQuit(0)
            return


if __name__ == "__main__":
    main()
