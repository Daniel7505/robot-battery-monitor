"""Record the simulator pose for scoring. Not imported by the mapper.

Writes gt_pose_log.csv into the run folder advertised by
logs/mapping_runs/CURRENT (or RBM_MAP_DIR). Columns are simulator time
and the robot node's true pose. The mapper never reads this file.

physics_log.csv is the same idea for the contact audit: per-wheel
contact counts, wheel-centre height, body speed, and wall contacts.
Encoder speed is logged by the mapper from its own position sensors.
This file is how the two are compared after the run.
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


def _up_z(node) -> float:
    matrix = node.getOrientation()
    return float(matrix[8])


def _endpoint(robot, def_name: str):
    joint = robot.getFromProtoDef(def_name)
    if joint is None:
        return None
    field = joint.getField("endPoint")
    if field is None:
        return None
    try:
        return field.getSFNode()
    except Exception:
        return None


def _partner_name(supervisor, node_id: int, cache: dict) -> str:
    if node_id in cache:
        return cache[node_id]
    name = ""
    node = supervisor.getFromId(int(node_id))
    if node is not None:
        field = node.getField("name")
        if field is not None:
            try:
                name = field.getSFString() or ""
            except Exception:
                name = ""
    cache[node_id] = name
    return name


def _contact_summary(supervisor, solid, cache: dict) -> tuple[int, int, float]:
    """Return (floor contacts, wall contacts, lowest contact z)."""
    if solid is None:
        return 0, 0, float("nan")
    try:
        points = solid.getContactPoints()
    except Exception:
        return 0, 0, float("nan")
    floor_n = 0
    wall_n = 0
    low = float("nan")
    for point in points:
        z = float(point.point[2])
        low = z if math.isnan(low) else min(low, z)
        name = _partner_name(supervisor, point.node_id, cache).lower()
        if "floor" in name or (name == "" and z < 0.03):
            floor_n += 1
        elif z > 0.03 or "wall" in name or "furn" in name:
            wall_n += 1
        else:
            floor_n += 1
    return floor_n, wall_n, low


def main() -> None:
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    robot = None
    handle = None
    physics = None
    walls = None
    parts = None
    cache: dict = {}
    folder = ""
    next_try = 0.0
    wall_events = 0
    prev = None
    movie = False
    shot = False
    frame_i = 0
    next_frame = 8.0
    record = os.environ.get("RBM_MOVIE") == "1"
    viewpoint = None
    while supervisor.step(timestep) != -1:
        now = float(supervisor.getTime())
        if robot is None:
            robot = _find_robot(supervisor)
            if robot is None:
                continue
        if parts is None and robot is not None:
            parts = {
                "left": _endpoint(robot, "LEFT_WHEEL_JOINT"),
                "right": _endpoint(robot, "RIGHT_WHEEL_JOINT"),
                "fl": _endpoint(robot, "FRONT_LEFT_CASTER"),
                "fr": _endpoint(robot, "FRONT_RIGHT_CASTER"),
                "rl": _endpoint(robot, "REAR_LEFT_CASTER"),
                "rr": _endpoint(robot, "REAR_RIGHT_CASTER"),
            }
            for solid in parts.values():
                if solid is None:
                    continue
                try:
                    solid.enableContactPointsTracking(timestep)
                except Exception:
                    pass
            try:
                robot.enableContactPointsTracking(timestep, True)
            except Exception:
                pass
            missing = [name for name, solid in parts.items() if solid is None]
            print(f"POSE LOG contact parts missing={missing or 'none'}")
        if handle is None and now >= next_try:
            folder = _run_dir()
            next_try = now + 0.5
            if not folder:
                continue
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, "gt_pose_log.csv")
            handle = open(path, "w", encoding="utf-8", newline="\n")
            handle.write("t_s,x_m,y_m,z_m,yaw_rad\n")
            physics = open(os.path.join(folder, "physics_log.csv"), "w", encoding="utf-8", newline="\n")
            physics.write(
                "t_s,x_m,y_m,z_m,yaw_rad,speed_m_s,up_z,"
                "left_floor,right_floor,caster_floor,left_wall,right_wall,caster_wall,"
                "left_z,right_z,contact_z\n"
            )
            walls = open(os.path.join(folder, "wall_contacts.csv"), "w", encoding="utf-8", newline="\n")
            walls.write("t_s,x_m,y_m,z_m,yaw_rad,left_wall,right_wall,caster_wall\n")
            print(f"POSE LOG {path}")
            if record and not movie:
                os.makedirs(os.path.join(folder, "frames"), exist_ok=True)
                movie = True
                print(f"POSE LOG frames {folder}/frames")
        if handle is None or robot is None or physics is None or walls is None:
            continue
        pos = robot.getPosition()
        yaw = _yaw(robot)
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        handle.write(f"{now:.4f},{x:.5f},{y:.5f},{z:.5f},{yaw:.6f}\n")
        speed = 0.0
        if prev is not None:
            dt = now - prev[0]
            if dt > 1e-6:
                speed = math.hypot(x - prev[1], y - prev[2]) / dt
        prev = (now, x, y)
        left_f, left_w, left_cz = _contact_summary(supervisor, parts["left"] if parts else None, cache)
        right_f, right_w, right_cz = _contact_summary(supervisor, parts["right"] if parts else None, cache)
        caster_f = caster_w = 0
        if parts:
            for key in ("fl", "fr", "rl", "rr"):
                cf, cw, _cz = _contact_summary(supervisor, parts[key], cache)
                caster_f += cf
                caster_w += cw
        left_z = right_z = float("nan")
        if parts and parts["left"] is not None:
            left_z = float(parts["left"].getPosition()[2])
        if parts and parts["right"] is not None:
            right_z = float(parts["right"].getPosition()[2])
        lows = [value for value in (left_cz, right_cz) if value == value]
        contact_z = min(lows) if lows else float("nan")
        physics.write(
            f"{now:.4f},{x:.5f},{y:.5f},{z:.5f},{yaw:.6f},{speed:.5f},{_up_z(robot):.5f},"
            f"{left_f},{right_f},{caster_f},{left_w},{right_w},{caster_w},"
            f"{left_z:.5f},{right_z:.5f},{contact_z:.5f}\n"
        )
        if record:
            if viewpoint is None:
                viewpoint = supervisor.getFromDef("VIEWPOINT")
            if viewpoint is not None:
                # Straight down over the shell. The mapper never reads this node.
                # Absolute eye position. follow is off, so this is the world point.
                pos = viewpoint.getField("position")
                if pos is not None:
                    pos.setSFVec3f([x, y, 4.5])
            if now >= next_frame:
                frame = os.path.join(folder, "frames", f"frame_{frame_i:04d}.png")
                supervisor.exportImage(frame, 85)
                frame_i += 1
                next_frame = now + 0.5
                if not shot and now >= 20.0:
                    still = os.path.join(folder, "follow.png")
                    supervisor.exportImage(still, 90)
                    shot = True
                    print(f"POSE LOG screenshot {still}")
        if left_w or right_w or caster_w:
            wall_events += 1
            walls.write(
                f"{now:.4f},{x:.5f},{y:.5f},{z:.5f},{yaw:.6f},{left_w},{right_w},{caster_w}\n"
            )
        if int(now * 4) != int((now - timestep / 1000.0) * 4):
            handle.flush()
            physics.flush()
        # The mapper writes this file when it finishes. Only a supervisor
        # can stop the world, and the mapper is not one.
        if folder and os.path.isfile(os.path.join(folder, "mapping_done")):
            handle.flush()
            physics.flush()
            walls.flush()
            handle.close()
            physics.close()
            walls.close()
            print(f"POSE LOG mapping finished, quitting t={now:.1f}s wall_contact_steps={wall_events}")
            supervisor.simulationQuit(0)
            return


if __name__ == "__main__":
    main()
