"""One waypoint at a time, in the robot's own odometry frame.

The explorer still integrates the wheels, the gyro, the floor camera,
and the depth picture. This module only chooses the wheel command for
that step, and it stops when the command is finished. It does not read
the simulator pose or the floor plan.

``RBM_DRIVER`` stays off unless set. With the flag off, nothing here runs.
"""

from __future__ import annotations

import math

from src.mapping.body import ARRIVE_M, CRUISE_M_S, SPIN_RAD_S, WHEEL_RADIUS_M
from src.mapping.drive import pure_pursuit, wheel_speeds
from src.mapping.explore import PIVOT_EXIT_RAD, STALL_WINDOW_S, STILL_M
from src.mapping.frontiers import astar, motion_blocked, traversable
from src.mapping.odometry import wrap_rad
from src.mapping.tick import choose_motion, live_veto


# Same finish as the survey spin. A look is that turn, then a stop.
_LOOK_DONE_RAD = 2.0 * math.pi - 0.08
# The existing "the heading did move" step. The clock beside it is
# STALL_WINDOW_S, which stays 2 s.
_YAW_STEP_RAD = 0.12
# Two drive motors, 8 W each at cruise, and the 480 Wh pack fallback.
# This is the robot's own book. There is no battery sensor on this proto.
_PACK_WH = 480.0
_MOTOR_W = 8.0
_CRUISE_RAD_S = CRUISE_M_S / WHEEL_RADIUS_M


def parse_command(data: dict) -> dict:
    """``go_to``, ``turn_to``, ``look`` (or ``scan``), or ``stop``.

    ``x`` and ``y`` are metres in the odometry frame. ``yaw`` is radians
    in that same frame. ``heading`` is accepted as another name for yaw.
    """
    if not isinstance(data, dict):
        raise ValueError("command is not an object")
    kind = str(data.get("cmd") or "").strip().lower()
    if kind == "scan":
        kind = "look"
    if kind not in ("go_to", "turn_to", "look", "stop"):
        raise ValueError(f"unknown command {kind!r}")
    if kind == "go_to":
        return {"cmd": "go_to", "x": _finite(data.get("x"), "x"), "y": _finite(data.get("y"), "y")}
    if kind == "turn_to":
        raw = data.get("yaw", data.get("heading"))
        return {"cmd": "turn_to", "yaw": _finite(raw, "yaw")}
    return {"cmd": kind}


def _finite(value, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} is not a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} is not finite")
    return number


class WaypointDriver:
    """Drive the armed command. ``wheels`` returns rad/s for the two hubs."""

    def __init__(self) -> None:
        self.command: dict | None = None
        self.outcome: str | None = None
        self.detail = ""
        self.armed = False
        self.quitting = False
        self.commanded_m = 0.0
        self.wheel_m = 0.0
        self.floor_m = 0.0
        self.wh_used = 0.0
        self.tilt_warning = False
        self._t = 0.0
        self._timeout_s = 15.0
        self._still_from = 0.0
        self._best_dist = 0.0
        self._best_err = 0.0
        self._best_head = math.pi
        self._yaw0 = 0.0
        self._turn_mark = 0.0
        self._win_x = 0.0
        self._win_y = 0.0
        self._win_yaw = 0.0
        self._window_translated = False
        self._window_spun = False
        self._window_moved = False
        self._lifted0 = 0
        self._path: list[tuple[float, float]] | None = None
        self._path_at = -1.0

    def arm(self, command: dict, explorer, *, timeout_s: float | None = None) -> None:
        """Start one command. Travel for the previous command is cleared."""
        cmd = parse_command(command)
        self.command = cmd
        self.outcome = None
        self.detail = ""
        self.armed = cmd["cmd"] != "stop"
        self.commanded_m = 0.0
        self.wheel_m = 0.0
        self.floor_m = 0.0
        self.tilt_warning = False
        self._t = 0.0
        self._path = None
        self._path_at = -1.0
        x, y, yaw = _pose(explorer)
        self._best_dist = math.hypot(float(cmd.get("x", x)) - x, float(cmd.get("y", y)) - y)
        self._best_err = abs(wrap_rad(float(cmd.get("yaw", yaw)) - yaw))
        self._best_head = _head_err(x, y, yaw, cmd) if cmd["cmd"] == "go_to" else self._best_err
        self._yaw0 = _yaw_u(explorer)
        self._turn_mark = 0.0
        self._reset_window(explorer)
        self._lifted0 = int(getattr(explorer, "wheels_lifted", 0))
        self._timeout_s = float(timeout_s) if timeout_s is not None else _timeout_s(cmd, (x, y))
        if cmd["cmd"] == "stop":
            self.outcome = "stopped"
            self.detail = "stop command"
            self.armed = False

    def battery(self) -> dict:
        pct = 100.0 * max(0.0, 1.0 - self.wh_used / _PACK_WH)
        return {
            "pct": round(pct, 2),
            "wh_used": round(self.wh_used, 4),
            "pack_wh": _PACK_WH,
        }

    def report(self, explorer) -> dict:
        """What the snapshot records. The pose is the robot's own estimate."""
        x, y, yaw = _pose(explorer)
        return {
            "status": self.outcome or "ready",
            "detail": self.detail,
            "tilt_warning": bool(self.tilt_warning or self.outcome == "tilt_warning"),
            "pose": {"x_m": x, "y_m": y, "yaw_rad": yaw, "frame": "odometry"},
            "travel": {
                "commanded_m": round(self.commanded_m, 4),
                "wheel_m": round(self.wheel_m, 4),
                "floor_m": round(self.floor_m, 4),
            },
            "battery": self.battery(),
            "command": self.command,
        }

    def wheels(self, explorer, left_clear, right_clear, view, dt) -> tuple[float, float]:
        if self.outcome is not None or not self.armed or self.command is None:
            return 0.0, 0.0
        step = max(0.0, float(dt))
        self._t += step
        self._note_measured(explorer)
        if int(getattr(explorer, "wheels_lifted", 0)) > self._lifted0:
            self.tilt_warning = True
            return self._stop("tilt_warning", "wheels lifted")
        kind = self.command["cmd"]
        if kind == "go_to":
            return self._go(explorer, left_clear, right_clear, view, step)
        if kind == "turn_to":
            return self._turn(explorer, view, step)
        if kind == "look":
            return self._look(explorer, view, step)
        return self._stop("stopped", "stop command")

    def _go(self, explorer, left_clear, right_clear, view, dt: float) -> tuple[float, float]:
        x, y, yaw = _pose(explorer)
        goal = (float(self.command["x"]), float(self.command["y"]))
        dist = math.hypot(goal[0] - x, goal[1] - y)
        if dist <= ARRIVE_M:
            return self._stop("arrived", f"within {ARRIVE_M:.2f} m")
        head = _head_err(x, y, yaw, self.command)
        closer = dist <= self._best_dist - STILL_M or head <= self._best_head - _YAW_STEP_RAD
        if closer:
            self._best_dist = min(self._best_dist, dist)
            self._best_head = min(self._best_head, head)
            self._reset_window(explorer)
        elif self._stall():
            return 0.0, 0.0
        if self._t >= self._timeout_s:
            return self._stop("timeout", "command time")
        why = _stuck(explorer)
        if why:
            return self._stop("stuck", why)
        self._note_window_motion(explorer)
        path = self._path_to(explorer, goal)
        pursuit_v, pursuit_w, arrived = pure_pursuit((x, y, yaw), path, arrive_m=ARRIVE_M)
        if arrived:
            return self._stop("arrived", f"within {ARRIVE_M:.2f} m")
        choice = choose_motion(
            view=view,
            pursuit_v=pursuit_v,
            pursuit_w=pursuit_w,
            heading_err=wrap_rad(math.atan2(goal[1] - y, goal[0] - x) - yaw),
            grid=explorer.grid,
            pose=(x, y, yaw),
            left_clear=float(left_clear),
            right_clear=float(right_clear),
        )
        v = float(choice.v)
        w = float(choice.w)
        if motion_blocked(explorer.grid, (x, y, yaw), v, w):
            v, w = 0.0, 0.0
        self._remember(v, w)
        left, right = wheel_speeds(v, w)
        self._note_command(left, right, dt)
        return left, right

    def _turn(self, explorer, view, dt: float) -> tuple[float, float]:
        _x, _y, yaw = _pose(explorer)
        err = wrap_rad(float(self.command["yaw"]) - yaw)
        if abs(err) <= PIVOT_EXIT_RAD:
            return self._stop("arrived", "heading within 12 deg")
        if abs(err) <= self._best_err - _YAW_STEP_RAD:
            self._best_err = abs(err)
            self._reset_window(explorer)
        elif self._stall():
            return 0.0, 0.0
        if self._t >= self._timeout_s:
            return self._stop("timeout", "command time")
        why = _stuck(explorer)
        if why:
            return self._stop("stuck", why)
        self._note_window_motion(explorer)
        return self._spin(explorer, view, dt, SPIN_RAD_S if err > 0.0 else -SPIN_RAD_S)

    def _look(self, explorer, view, dt: float) -> tuple[float, float]:
        turned = abs(_yaw_u(explorer) - self._yaw0)
        if turned >= _LOOK_DONE_RAD:
            return self._stop("arrived", "full turn")
        if turned >= self._turn_mark + _YAW_STEP_RAD:
            self._turn_mark = turned
            self._reset_window(explorer)
        elif self._stall():
            return 0.0, 0.0
        if self._t >= self._timeout_s:
            return self._stop("timeout", "command time")
        why = _stuck(explorer)
        if why:
            return self._stop("stuck", why)
        self._note_window_motion(explorer)
        return self._spin(explorer, view, dt, SPIN_RAD_S)

    def _spin(self, explorer, view, dt: float, w: float) -> tuple[float, float]:
        pose = _pose(explorer)
        if live_veto(0.0, w, view) is not None or motion_blocked(explorer.grid, pose, 0.0, w):
            self._note_command(0.0, 0.0, dt)
            return 0.0, 0.0
        self._remember(0.0, w)
        left, right = wheel_speeds(0.0, w)
        self._note_command(left, right, dt)
        return left, right

    def _stall(self) -> bool:
        """Stop when this window has lasted the existing 2 s.

        Commanded motion the body did not follow is stuck. A window that
        never moved, or moved without getting closer, is blocked.
        """
        if self._t - self._still_from < STALL_WINDOW_S - 1e-6:
            return False
        sent = self._window_translated or self._window_spun
        if sent and not self._window_moved:
            self._stop("stuck", "no motion for 2 s")
        else:
            self._stop("blocked", "no closer for 2 s")
        return True

    def _remember(self, v: float, w: float) -> None:
        if abs(v) >= 0.02:
            self._window_translated = True
        if abs(w) >= 0.1:
            self._window_spun = True

    def _note_window_motion(self, explorer) -> None:
        x, y, _yaw = _pose(explorer)
        if math.hypot(x - self._win_x, y - self._win_y) >= STILL_M:
            self._window_moved = True
        if abs(_yaw_u(explorer) - self._win_yaw) >= _YAW_STEP_RAD:
            self._window_moved = True

    def _path_to(self, explorer, goal: tuple[float, float]) -> list[tuple[float, float]]:
        if self._path is not None and self._t - self._path_at < 0.5:
            return self._path
        self._path = _path_on_own_map(explorer, goal)
        self._path_at = self._t
        return self._path

    def _reset_window(self, explorer) -> None:
        x, y, _yaw = _pose(explorer)
        self._still_from = self._t
        self._win_x = x
        self._win_y = y
        self._win_yaw = _yaw_u(explorer)
        self._window_translated = False
        self._window_spun = False
        self._window_moved = False

    def _stop(self, status: str, detail: str) -> tuple[float, float]:
        self.outcome = status
        self.detail = detail
        self.armed = False
        if status == "tilt_warning":
            self.tilt_warning = True
        return 0.0, 0.0

    def _note_measured(self, explorer) -> None:
        odom = explorer.odometry
        self.wheel_m += abs(float(getattr(odom, "last_ds_m", 0.0)))
        fusion = getattr(explorer, "_fusion", None)
        sample = getattr(fusion, "scene_floor_m", None) if fusion is not None else None
        if sample is not None:
            self.floor_m += abs(float(sample))

    def _note_command(self, left: float, right: float, dt: float) -> None:
        v = 0.5 * (float(left) + float(right)) * WHEEL_RADIUS_M
        self.commanded_m += abs(v) * float(dt)
        if _CRUISE_RAD_S <= 0.0 or dt <= 0.0:
            return
        scale = (abs(float(left)) + abs(float(right))) / (2.0 * _CRUISE_RAD_S)
        watts = 2.0 * _MOTOR_W * min(1.5, scale)
        self.wh_used += watts * float(dt) / 3600.0


def _timeout_s(cmd: dict, pose: tuple[float, float]) -> float:
    """Longer than the 2 s stall clock. Distance only stretches a go_to."""
    if cmd["cmd"] == "go_to":
        dist = math.hypot(float(cmd["x"]) - pose[0], float(cmd["y"]) - pose[1])
        return max(15.0, dist / 0.08)
    if cmd["cmd"] == "look":
        return 20.0
    return 15.0


def _head_err(x: float, y: float, yaw: float, cmd: dict) -> float:
    bearing = math.atan2(float(cmd["y"]) - y, float(cmd["x"]) - x)
    return abs(wrap_rad(bearing - yaw))


def _pose(explorer) -> tuple[float, float, float]:
    pose = explorer.pose()
    return float(pose[0]), float(pose[1]), float(pose[2])


def _yaw_u(explorer) -> float:
    return float(explorer.odometry.yaw_unwrapped_rad)


def _stuck(explorer) -> str:
    reason = getattr(explorer, "_stuck_reason", None)
    if reason is None:
        return ""
    why = reason()
    return str(why) if why else ""


def _path_on_own_map(explorer, goal: tuple[float, float]) -> list[tuple[float, float]]:
    """A path on cells this robot has marked free. Unknown space is a straight aim.

    The straight aim is what the live range and the footprint veto then accept
    or refuse. The apartment plan is not consulted.
    """
    grid = explorer.grid
    pose = _pose(explorer)
    start = grid.cell_of(pose[0], pose[1])
    end = grid.cell_of(goal[0], goal[1])
    if start is None or end is None:
        return [goal]
    free = traversable(grid.labels(), grid.res)
    if not bool(free[end]):
        return [goal]
    cells = astar(free, start, end, step_cost=float(grid.res))
    if not cells:
        return [goal]
    points = [grid.cell_center(r, c) for r, c in cells]
    points[-1] = (float(goal[0]), float(goal[1]))
    return points
