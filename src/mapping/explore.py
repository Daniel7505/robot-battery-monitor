"""Frontier exploration loop.

The robot spins once to fill the depth camera's blind sides, drives to
the best frontier with pure pursuit, spins again on arrival, and stops
when no reachable frontier is longer than 0.3 m. Depth closer than the
stop range, or a radar flag from the controller, holds the wheels and
turns toward the clearer side.

This module does not know the apartment. Pose is whatever the odometry
object reports.
"""

from __future__ import annotations

import math

from src.mapping.body import (
    ARRIVE_M,
    OAK_DEPTH,
    ROBOT_RADIUS_M,
    SPIN_RAD_S,
    STOP_CLEARANCE_M,
    DepthCamera,
)
from src.mapping.depth_cloud import body_points, forward_clearance, to_odom
from src.mapping.drive import pure_pursuit, wheel_speeds
from src.mapping.frontiers import select_goal
from src.mapping.grid import OccupancyGrid
from src.mapping.odometry import WheelGyroOdometry


class Explorer:
    def __init__(
        self,
        *,
        camera: DepthCamera = OAK_DEPTH,
        stride: int = 4,
        grid: OccupancyGrid | None = None,
        odometry: WheelGyroOdometry | None = None,
    ) -> None:
        self.camera = camera
        self.stride = int(stride)
        self.grid = grid if grid is not None else OccupancyGrid()
        self.odometry = odometry if odometry is not None else WheelGyroOdometry()
        self.phase = "spin"
        self.done = False
        self.t_s = 0.0
        self.goal_xy: tuple[float, float] | None = None
        self.path_xy: list[tuple[float, float]] = []
        self._spin_start: float | None = None
        self._replan_at = -1.0
        self._avoid_until = -1.0
        self._reverse_until = -1.0
        self._turn = SPIN_RAD_S
        self._banned: set[tuple[int, int]] = set()
        self._fails: dict[tuple[int, int], int] = {}
        self._goal_since = 0.0
        self._stuck_anchor: tuple[float, float, float] | None = None
        self._wheels: tuple[float, float] | None = None
        self._commanded = (0.0, 0.0)
        self.frontiers_left = 0

    def pose(self) -> tuple[float, float, float]:
        return self.odometry.pose()

    def step(
        self,
        *,
        dt_s: float,
        left_wheel_rad: float,
        right_wheel_rad: float,
        yaw_rate_rad_s: float,
        roll_rad: float = 0.0,
        pitch_rad: float = 0.0,
        depth=None,
        radar_blocked: bool = False,
    ) -> tuple[float, float]:
        dt = max(0.0, float(dt_s))
        self.t_s += dt
        if self._wheels_stalled(left_wheel_rad, right_wheel_rad, dt):
            self._note_contact()
        self._wheels = (float(left_wheel_rad), float(right_wheel_rad))
        self.odometry.update(left_wheel_rad, right_wheel_rad, yaw_rate_rad_s, dt)
        x, y, yaw = self.odometry.pose()
        self.grid.stamp_footprint(x, y)
        clearance = left_clear = right_clear = 99.0
        if depth is not None:
            hits, floors, sensor = body_points(
                self.camera,
                depth,
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                stride=self.stride,
            )
            origin = to_odom(sensor.reshape(1, 3), (x, y, yaw))[0]
            self.grid.integrate(origin, to_odom(hits, (x, y, yaw)), to_odom(floors, (x, y, yaw)))
            clearance, left_clear, right_clear = forward_clearance(hits)
            prefer = SPIN_RAD_S if left_clear >= right_clear else -SPIN_RAD_S
        else:
            prefer = self._turn
        blocked = radar_blocked or clearance < STOP_CLEARANCE_M
        # One direction for the whole blocked episode. Picking again
        # every frame wags the robot between the two sides of a jamb.
        if self.phase == "go" and blocked:
            if self.t_s >= self._avoid_until:
                self._turn = prefer
                self.path_xy = []
                self._replan_at = self.t_s + 0.45
            self._avoid_until = self.t_s + 0.08
        if self.phase == "spin":
            return self._remember(self._spin())
        if self.phase == "go":
            return self._remember(self._go(blocked, clearance))
        return self._remember((0.0, 0.0))

    def _spin(self) -> tuple[float, float]:
        yaw_u = self.odometry.yaw_unwrapped_rad
        if self._spin_start is None:
            self._spin_start = yaw_u
        if abs(yaw_u - self._spin_start) >= 2.0 * math.pi - 0.08:
            self._spin_start = None
            self.phase = "go"
            self._replan_at = -1.0
            self.goal_xy = None
            return self._go(False, 99.0)
        return wheel_speeds(0.0, SPIN_RAD_S)

    def _remember(self, speeds: tuple[float, float]) -> tuple[float, float]:
        self._commanded = (float(speeds[0]), float(speeds[1]))
        return speeds

    def _wheels_stalled(self, left_rad: float, right_rad: float, dt: float) -> bool:
        """A forward command the hubs did not follow. A spin in place is not a stall."""
        if self._wheels is None or dt <= 0.0:
            return False
        cmd_l, cmd_r = self._commanded
        if abs(cmd_l + cmd_r) < 0.5:
            return False
        expected = 0.5 * (abs(cmd_l) + abs(cmd_r)) * dt
        if expected < 0.02:
            return False
        actual = 0.5 * (abs(left_rad - self._wheels[0]) + abs(right_rad - self._wheels[1]))
        return actual < 0.30 * expected

    def _note_contact(self) -> None:
        x, y, yaw = self.odometry.pose()
        self.grid.mark_contact(x + math.cos(yaw) * 0.28, y + math.sin(yaw) * 0.28)
        # Back off the contact before turning. A spin in place stays in the pinch.
        self._reverse_until = self.t_s + 0.8
        self._avoid_until = self._reverse_until + 0.35
        self.path_xy = []
        self._replan_at = self._avoid_until

    def _go(self, blocked: bool, clearance: float) -> tuple[float, float]:
        if self.done:
            return 0.0, 0.0
        if self._goal_expired() or self._is_stuck():
            self._ban_current()
            self._plan()
        if self.t_s < self._reverse_until:
            return wheel_speeds(-0.14, self._turn * 0.35)
        if self.t_s < self._avoid_until or blocked:
            return wheel_speeds(0.0, self._turn)
        if self.t_s >= self._replan_at or not self.path_xy:
            self._plan()
        if self.done or not self.path_xy:
            return 0.0, 0.0
        v, w, arrived = pure_pursuit(self.odometry.pose(), self.path_xy, arrive_m=ARRIVE_M)
        if clearance < 1.15:
            span = max(0.15, 1.15 - STOP_CLEARANCE_M)
            v *= max(0.30, (clearance - STOP_CLEARANCE_M) / span)
        if arrived:
            self.phase = "spin"
            self._spin_start = None
            self.goal_xy = None
            self.path_xy = []
            return wheel_speeds(0.0, SPIN_RAD_S)
        return wheel_speeds(v, w)

    def _plan(self, depth: int = 0) -> None:
        self._replan_at = self.t_s + 1.0
        x, y, _yaw = self.odometry.pose()
        if depth > 6:
            self.phase = "done"
            self.done = True
            return
        chosen = select_goal(self.grid, (x, y), radius_m=ROBOT_RADIUS_M, banned=self._banned)
        if chosen is None:
            self.frontiers_left = 0
            self.goal_xy = None
            self.path_xy = []
            self.phase = "done"
            self.done = True
            return
        goal, path = chosen
        cell = self.grid.cell_of(goal[0], goal[1])
        # Already standing on it: look once more, then drop it so a speck
        # of unknown next to the robot cannot spin the run forever.
        if cell is not None and math.hypot(goal[0] - x, goal[1] - y) < ARRIVE_M:
            count = self._fails.get(cell, 0) + 1
            self._fails[cell] = count
            if count >= 2:
                self._banned.add(cell)
                self._plan(depth + 1)
                return
            self.phase = "spin"
            self._spin_start = None
            self.goal_xy = None
            self.path_xy = []
            return
        same = self.goal_xy is not None and math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1]) < 0.2
        self.goal_xy, self.path_xy = goal, path
        self.frontiers_left = 1
        if not same:
            self._goal_since = self.t_s
            self._stuck_anchor = (self.t_s, x, y)

    def _goal_key(self) -> tuple[int, int] | None:
        if self.goal_xy is None:
            return None
        cell = self.grid.cell_of(self.goal_xy[0], self.goal_xy[1])
        return cell

    def _ban_current(self) -> None:
        key = self._goal_key()
        self.goal_xy = None
        self.path_xy = []
        self._replan_at = -1.0
        if key is None:
            return
        count = self._fails.get(key, 0) + 1
        self._fails[key] = count
        if count >= 2:
            # Drop the whole nearby edge, not one cell. A doorway frontier
            # is dozens of cells and banning them one at a time never leaves.
            radius = 4
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dy * dy + dx * dx <= radius * radius:
                        self._banned.add((key[0] + dy, key[1] + dx))

    def _goal_expired(self) -> bool:
        return self.goal_xy is not None and (self.t_s - self._goal_since) > 40.0

    def _is_stuck(self) -> bool:
        if self._stuck_anchor is None or self.goal_xy is None:
            return False
        t0, x0, y0 = self._stuck_anchor
        if self.t_s - t0 < 4.0:
            return False
        x, y, _yaw = self.odometry.pose()
        moved = math.hypot(x - x0, y - y0)
        self._stuck_anchor = (self.t_s, x, y)
        return moved < 0.08
