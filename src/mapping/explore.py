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

import numpy as np

from src.mapping.body import (
    ARRIVE_M,
    BLIND_CLEARANCE_M,
    BLIND_VIEW_FRACTION,
    CRUISE_M_S,
    OAK_DEPTH,
    ROBOT_RADIUS_M,
    SPIN_RAD_S,
    REAR_DEPTH,
    STOP_CLEARANCE_M,
    DepthCamera,
)
from src.mapping.bubble import BubbleView, assess_bubble, max_speed_for_gap, pivot_ready, reverse_budget_m
from src.mapping.depth_cloud import (
    blind_fraction,
    body_points,
    depth_scene_motion_m,
    forward_clearance,
    rear_clearance_m,
    to_odom,
)
from src.mapping.drive import pure_pursuit, wheel_speeds
from src.mapping.footprint import fit_motion
from src.mapping.frontiers import select_goal
from src.mapping.fuse import MotionFusion
from src.mapping.grid import OccupancyGrid
from src.mapping.loop_close import LoopCloser
from src.mapping.odometry import WheelGyroOdometry, wrap_rad
from src.mapping.pin import PinWatch
from src.mapping.rooms import segment_map
from src.mapping.scan_match import scan_fit
from src.mapping.scripted import BodyScript, drive_mode
from src.mapping.slip import MotionCheck
from src.mapping.wall_line import wall_yaw_delta

# Through a doorway, not down an open room. Half of cruise, so a jamb
# correction has time to finish before the tire meets the frame.
DOOR_SPEED_M_S = 0.12


class Explorer:
    def __init__(
        self,
        *,
        camera: DepthCamera = OAK_DEPTH,
        stride: int = 4,
        grid: OccupancyGrid | None = None,
        odometry: WheelGyroOdometry | None = None,
        fusion: bool = True,
        floor: bool = True,
        loop: bool = True,
        drive: str | None = None,
    ) -> None:
        self.camera = camera
        self.stride = int(stride)
        self.grid = grid if grid is not None else OccupancyGrid()
        self.odometry = odometry if odometry is not None else WheelGyroOdometry()
        self.fusion_enabled = bool(fusion)
        self.floor_enabled = bool(floor)
        self.loop_enabled = bool(loop)
        mode = drive_mode(drive)
        # A named body-frame pattern. It is not a place in the world.
        self._script: BodyScript | None = BodyScript(mode) if mode else None
        self._fusion = MotionFusion(floor=self.floor_enabled, visual=self.fusion_enabled)
        self._loop = LoopCloser()
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
        self._banned: dict[tuple[int, int], float] = {}
        self._fails: dict[tuple[int, int], int] = {}
        self._goal_since = 0.0
        self._stuck_anchor: tuple[float, float, float] | None = None
        self._wheels: tuple[float, float] | None = None
        self._commanded = (0.0, 0.0)
        self._motion = MotionCheck()
        self._progress_xy = (0.0, 0.0)
        self._progress_t = 0.0
        self._recover_until = -1.0
        self._confident_after = 0.0
        self._last_spin_t = -100.0
        self._nudge_at = 0.0
        self._match_proposal: tuple[float, float, float] | None = None
        self._match_proposal_t = -10.0
        self._close_since: float | None = None
        self._heading_anchor: tuple[float, float] | None = None
        self._view_blind = False
        self._blind_frames = 0
        self._slip_debt_m = 0.0
        self._agree_at = 0.0
        self._scan_xy = None
        self._scan_at = -10.0
        self._spin_then: str | None = None
        self._did_final_scan = False
        self._settle_since: float | None = None
        self._settle_occ: int | None = None
        self._settle_check = 0.0
        self.slip_events = 0
        self.wall_applied_yaw = 0.0
        self._wall_prev = 0.0
        self.frontiers_left = 0
        self._last_clear = (99.0, 99.0, 99.0)
        self._held_clear: tuple[float, float, float] | None = None
        self._bubble_gap: float | None = None
        self._min_gap: float | None = None
        self._path_m = 0.0
        self._reverse_left_m = 0.0
        self._reverse_release = False
        self._reverse_needs_gap = False
        self._bubble_view: BubbleView | None = None
        self._pin = PinWatch()
        self._pin_idle_s = 0.0
        self._pin_until = -1.0
        self._mark_xy = (0.0, 0.0)
        self._prev_depth = None
        self._prev_depth_yaw = 0.0
        self._side_since: float | None = None
        self._side_sign = 0.0
        self._side_escape_sign = 0.0
        self._side_escape_until = -1.0
        self._stall_t = 0.0
        self._stall_exp = 0.0
        self._stall_act = 0.0
        self._stall_trans = 0.0
        self._stall_latched = False
        self.sweep_checks = 0
        self.sweep_rejected = 0
        self.sweep_widened = 0
        self.sweep_aligned = 0
        self.sweep_straight = 0
        self.sweep_stopped = 0
        self._sweep_log_at = -1.0
        self._commit_until = -1.0
        self._commit_heading = 0.0
        self._escape_log_at = -1.0
        self._blocked_since: float | None = None
        self.room_labels = None
        self.room_names: dict[int, str] = {}
        self.room_doors = None
        # Cells the shell has actually covered. A depth ray through a
        # doorway paints the next room free; that is not a visit.
        self.visited = np.zeros((self.grid.n, self.grid.n), dtype=bool)
        self._visited_cell: tuple[int, int] | None = None
        self._opening_since: float | None = None
        self._rear_gap: float | None = None
        self._prev_rear = None
        self._prev_rear_yaw = 0.0

    def pose(self) -> tuple[float, float, float]:
        return self.odometry.pose()

    def observed_room_labels(self):
        """Room ids on cells the shell has covered. Rays through a door do not count."""
        if self.room_labels is None:
            return None
        labels = np.array(self.room_labels, copy=True)
        labels[~self.visited] = 0
        return labels

    def _mark_visited(self) -> None:
        x, y, _yaw = self.odometry.pose()
        cell = self.grid.cell_of(x, y)
        if cell is None or cell == self._visited_cell:
            return
        self._visited_cell = cell
        res = self.grid.res
        reach = int(math.ceil(ROBOT_RADIUS_M / res))
        limit = (ROBOT_RADIUS_M / res) ** 2
        r0, c0 = cell
        n = self.grid.n
        vis = self.visited
        for dr in range(-reach, reach + 1):
            rr = r0 + dr
            if rr < 0 or rr >= n:
                continue
            for dc in range(-reach, reach + 1):
                if dr * dr + dc * dc > limit:
                    continue
                cc = c0 + dc
                if 0 <= cc < n:
                    vis[rr, cc] = True

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
        proximity: dict | None = None,
        floor=None,
        rgb=None,
        rear_depth=None,
        rear_rgb=None,
    ) -> tuple[float, float]:
        dt = max(0.0, float(dt_s))
        self._step_dt = dt
        self.t_s += dt
        stalled = self._wheels_stalled(left_wheel_rad, right_wheel_rad, dt)
        if stalled:
            self._note_contact()
        self._wheels = (float(left_wheel_rad), float(right_wheel_rad))
        ds = self.odometry.pending_translation(left_wheel_rad, right_wheel_rad)
        clearance = left_clear = right_clear = 99.0
        hits = floors = sensor = None
        if depth is not None:
            hits, floors, sensor = body_points(
                self.camera,
                depth,
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                stride=self.stride,
            )
            clearance, left_clear, right_clear = forward_clearance(hits)
            self._held_clear = (clearance, left_clear, right_clear)
            # Two empty frames, not one glitch. Held until a usable frame
            # arrives, including the steps between depth pictures.
            if blind_fraction(self.camera, depth, stride=self.stride) >= BLIND_VIEW_FRACTION:
                self._blind_frames += 1
            else:
                self._blind_frames = 0
                self._view_blind = False
            if self._blind_frames >= 2:
                self._view_blind = True
            prefer = SPIN_RAD_S if left_clear >= right_clear else -SPIN_RAD_S
        elif self._held_clear is not None:
            # Depth arrives at 10 Hz. Dropping it on the steps in between
            # made the shell look open and the speed cap ignored the wall.
            clearance, left_clear, right_clear = self._held_clear
            prefer = SPIN_RAD_S if left_clear >= right_clear else -SPIN_RAD_S
        else:
            prefer = self._turn
        if rear_depth is not None:
            rear_hits, _rear_floors, _rear_sensor = body_points(
                REAR_DEPTH,
                rear_depth,
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                stride=2,
            )
            self._rear_gap = rear_clearance_m(rear_hits)
        if self._view_blind:
            # No return is not an open hallway. The nose is inside minRange.
            clearance = min(clearance, BLIND_CLEARANCE_M)
        # Freeze a forward command the body did not follow, and a forward
        # command into a view the camera cannot see. A reverse that the
        # hubs actually follow is the robot leaving the wall, so it counts.
        # Ordinary braking (a visible wall inside the stop range) still
        # counts too, or the pose would sit a halt short of every wall.
        forward_cmd = (self._commanded[0] + self._commanded[1]) > 0.3
        in_contact = stalled or (self._view_blind and forward_cmd) or self.t_s < self._pin_until
        drop = False
        if depth is not None or self._view_blind or in_contact:
            before = self._motion.events
            slip_drop = self._motion.observe(dt, ds, clearance, blind=self._view_blind)
            if slip_drop:
                self._slip_debt_m = min(2.0, self._slip_debt_m + abs(ds))
            if self._motion.events > before and self.phase == "go":
                self._note_slip()
            drop = slip_drop or in_contact
        # A commanded spin should leave the axle where it is. Encoder
        # translation on that command is one wheel scrubbing, which walked
        # the pose while the body pivoted in a few centimetres.
        cmd_l, cmd_r = self._commanded
        spin_cmd = abs(cmd_l + cmd_r) < 0.4 and abs(cmd_l - cmd_r) > 1.0
        yaw_before = self.odometry.yaw_unwrapped_rad
        self.odometry.update(
            left_wheel_rad,
            right_wheel_rad,
            yaw_rate_rad_s,
            dt,
            trust_translation=not drop and not spin_cmd,
        )
        dyaw = self.odometry.yaw_unwrapped_rad - yaw_before
        self._mark_visited()
        applied = 0.0 if drop or spin_cmd else ds
        self._path_m += abs(applied)
        floor_image = floor if self.floor_enabled else None
        self._fusion.scene_floor_m = None
        self._fusion.scene_visual_m = None
        if self.fusion_enabled or self.floor_enabled:
            fused = self._fusion.observe(
                self.odometry,
                ds_raw_m=ds,
                ds_applied_m=applied,
                dyaw_rad=dyaw,
                floor=floor_image,
                depth=depth if self.fusion_enabled else None,
                camera=self.camera,
                rgb=rgb,
                rear_depth=rear_depth if self.fusion_enabled else None,
                rear_camera=REAR_DEPTH,
                rear_rgb=rear_rgb,
            )
            if fused == "slip":
                self.slip_events += 1
                self._slip_debt_m = min(2.0, self._slip_debt_m + abs(ds))
                self._confident_after = max(self._confident_after, self.t_s + 4.0)
                self._log(f"{self._fusion.last_slip} slip, wheel motion rejected")
        self._watch_pin(depth, ds, rear_depth)
        corrected = False
        x, y, yaw = self.odometry.pose()
        if (
            hits is not None
            and hits.size
            and not self._view_blind
            and not drop
            and abs(dyaw) < 0.04
            and not (dt > 0.0 and abs(dyaw) > 0.18 * dt)
            and self.t_s >= self._nudge_at
        ):
            # Match the wall in this scan to cells the robot already
            # painted. Done before this scan is integrated, or the new
            # points would agree with themselves.
            delta = wall_yaw_delta(self.grid, (x, y, yaw), hits[:, :2])
            if abs(delta) >= 0.004 and delta * self._wall_prev > 0.0 and abs(delta - self._wall_prev) < 0.02:
                self.odometry.nudge_yaw(delta)
                self.wall_applied_yaw += delta
                x, y, yaw = self.odometry.pose()
            self._wall_prev = delta
            self._nudge_at = self.t_s + 0.2
        if hits is not None and hits.size and not self._view_blind:
            scanned = to_odom(hits, (x, y, yaw))
            self._scan_xy = scanned
            self._scan_at = self.t_s
            if not drop and self.t_s >= self._agree_at:
                fit = scan_fit(self.grid, scanned)
                if fit == "support":
                    self._slip_debt_m = max(0.0, self._slip_debt_m - 0.25)
                elif fit == "disagree" and not corrected:
                    self._slip_debt_m = min(2.0, self._slip_debt_m + 0.15)
                elif not corrected:
                    # Trusted driving with nothing to contradict it. A flat
                    # floor often cannot support or disagree.
                    self._slip_debt_m = max(0.0, self._slip_debt_m - 0.05)
                self._agree_at = self.t_s + 0.5
        view = assess_bubble(clearance, blind=self._view_blind, proximity=proximity)
        if self._rear_gap is not None:
            rear = self._rear_gap if view.rear is None else min(view.rear, self._rear_gap)
            known = [gap for gap in (view.forward, view.left, view.right, rear) if gap is not None]
            view = BubbleView(
                view.forward,
                view.left,
                view.right,
                rear,
                min(known) if known else None,
            )
        self._bubble_view = view
        self._bubble_gap = view.forward
        self._min_gap = view.minimum
        self._watch_side(view)
        if self._reverse_left_m > 0.0 and self._reverse_needs_gap:
            # Encoder distance along a jamb spends the budget while the
            # tire is still on the frame. Wait until that side opens.
            if self._side_opened(view) or self.t_s > self._reverse_until:
                self._reverse_left_m = 0.0
                self._reverse_needs_gap = False
                self._side_escape_sign = 0.0
                self._side_since = None
        elif self._reverse_left_m > 0.0:
            if ds >= -1e-4:
                # Hubs have not started going backward. A stall would
                # otherwise burn the timer and the backup would expire
                # before the wheels ever moved.
                self._reverse_until = max(
                    self._reverse_until, self.t_s + self._reverse_left_m / 0.12 + 0.35
                )
            if ds < 0.0:
                self._reverse_left_m = max(0.0, self._reverse_left_m + ds)
            if self._reverse_left_m <= 0.0 or self.t_s > self._reverse_until:
                self._reverse_left_m = 0.0
            elif (
                self._reverse_release
                and ds < -0.002
                and view.minimum is not None
                and pivot_ready(view.minimum)
                and not self._view_blind
            ):
                self._reverse_left_m = 0.0
        if (
            self._script is None
            and self.loop_enabled
            and self.phase != "spin"
            and abs(dyaw) < 0.08
            and hits is not None
            and hits.size
            and sensor is not None
            and floors is not None
            and not drop
        ):
            fixed = self._loop.consider(
                (x, y, yaw),
                self._path_m,
                self._fusion.landmarks,
                hits[:, :2],
                floors[:, :2],
                sensor[:2],
            )
            if fixed is not None:
                self.odometry.x_m, self.odometry.y_m = fixed[0], fixed[1]
                self.odometry.nudge_yaw(wrap_rad(fixed[2] - self.odometry.yaw_rad))
                self._loop.rebuild(self.grid)
                x, y, yaw = self.odometry.pose()
                self._log(f"loop closure pose=({x:.2f},{y:.2f},{yaw:.2f})")
        self.grid.stamp_footprint(x, y)
        if hits is not None and sensor is not None and floors is not None:
            origin = to_odom(sensor.reshape(1, 3), (x, y, yaw))[0]
            self.grid.integrate(origin, to_odom(hits, (x, y, yaw)), to_odom(floors, (x, y, yaw)))
        self._last_clear = (clearance, left_clear, right_clear)
        # A depth hit at 1.5 m is the far side of the room, not a press.
        # Recovery only when the shell itself is close. An open room was
        # backing up every few seconds and never leaving the spawn area.
        shell_gap = self._min_gap if self._min_gap is not None else self._bubble_gap
        near_shell = self._view_blind or (shell_gap is not None and shell_gap < 0.45)
        if near_shell:
            if self._close_since is None:
                self._close_since = self.t_s
        else:
            self._close_since = None
        close_held = self._close_since is not None and (self.t_s - self._close_since) >= 0.4
        # Pure pursuit spins in place when the path is behind the nose.
        # If that spin does not actually change the heading, the body is
        # wedged and an open view must not hide it. A real turn, which
        # does change the heading, is left alone.
        yaw_u = self.odometry.yaw_unwrapped_rad
        # A few degrees resets the timer. Only a heading that is not
        # turning at all counts as wedged. A slow alignment is a real turn.
        if self._heading_anchor is None or abs(yaw_u - self._heading_anchor[1]) > 0.08:
            self._heading_anchor = (self.t_s, yaw_u)
        heading_stuck = (self.t_s - self._heading_anchor[0]) > 3.5
        if math.hypot(x - self._progress_xy[0], y - self._progress_xy[1]) > 0.05:
            self._progress_xy = (x, y)
            self._progress_t = self.t_s
        # Both shoulders on a frame, and the nose still looking through
        # it, is a doorway approach. A few seconds of lining up is not a
        # stall, and the recovery used to blacklist the opening.
        in_mouth = (
            0.12 < left_clear < 0.80
            and 0.12 < right_clear < 0.80
            and clearance > 0.16
        )
        if in_mouth:
            if self._opening_since is None:
                self._opening_since = self.t_s
            if self.t_s - self._opening_since < 10.0:
                self._progress_t = self.t_s
        else:
            self._opening_since = None
        if (
            self._script is None
            and self.phase == "go"
            and not self.done
            and self.t_s >= self._recover_until
            and (self.t_s - self._progress_t) > 3.5
            and (close_held or heading_stuck)
        ):
            self._start_recovery()
        # The bubble is the whole shell. A jamb in the shoulder of the
        # depth view is still a steer (see _go), not a wall by itself.
        cap = self._speed_cap(view)
        shell_blocked = cap < 0.04
        # The forward radar reports furniture inside 0.45 m. That is not
        # the shell touching it. Treating it as a block turned an open
        # bubble into a spin, and the spin reversed whenever the depth
        # shoulders swapped, for minutes.
        if radar_blocked and not shell_blocked:
            cap = min(cap, 0.12)
        blocked = shell_blocked
        # One direction for the whole blocked episode. Picking again
        # every frame wags the robot between the two sides of a jamb.
        if self.phase == "go" and blocked:
            if self._blocked_since is None:
                self._blocked_since = self.t_s
                self._turn = prefer
                if self._replan_at < self.t_s:
                    self._replan_at = self.t_s + 0.45
            self._avoid_until = self.t_s + 0.08
        elif not blocked:
            self._blocked_since = None
        if self._script is not None:
            speeds = self._script.command(
                ds_m=ds,
                yaw_unwrapped=self.odometry.yaw_unwrapped_rad,
                forward_gap_m=view.forward,
                rear_gap_m=view.rear,
                dt_s=dt,
            )
            if self._script.done:
                self.done = True
                self.phase = "done"
            return self._remember(speeds)
        if self.phase == "settle":
            return self._remember(self._settle())
        if self.phase == "spin":
            return self._remember(self._spin())
        if self.phase == "go":
            return self._remember(self._go(blocked, cap, left_clear, right_clear))
        return self._remember((0.0, 0.0))

    def _spin(self) -> tuple[float, float]:
        yaw_u = self.odometry.yaw_unwrapped_rad
        if self._spin_start is None:
            self._spin_start = yaw_u
        if abs(yaw_u - self._spin_start) >= 2.0 * math.pi - 0.08:
            self._spin_start = None
            then = self._spin_then
            self._spin_then = None
            self.phase = "go"
            self._replan_at = -1.0
            self.goal_xy = None
            self._progress_t = self.t_s
            self._progress_xy = self.odometry.pose()[:2]
            if then == "settle":
                self._log("final scan finished")
                self._plan()
                if self.phase != "go":
                    return 0.0, 0.0
            return self._go(False, 99.0)
        return wheel_speeds(0.0, SPIN_RAD_S)

    def _remember(self, speeds: tuple[float, float]) -> tuple[float, float]:
        self._commanded = (float(speeds[0]), float(speeds[1]))
        self._mark_xy = self.odometry.pose()[:2]
        return speeds

    def _wheels_stalled(self, left_rad: float, right_rad: float, dt: float) -> bool:
        """A translation command the hubs did not follow. A spin in place is not a stall.

        One 8 ms step of a backup is about 0.01 rad. The old per-step floor
        of 0.02 rad meant a slow command could never count, so a tire
        scrubbing a door jamb looked like ordinary driving.
        """
        if self._wheels is None or dt <= 0.0:
            return self._stall_latched
        cmd_l, cmd_r = self._commanded
        self._stall_t += dt
        self._stall_exp += 0.5 * (abs(cmd_l) + abs(cmd_r)) * dt
        self._stall_act += 0.5 * (
            abs(left_rad - self._wheels[0]) + abs(right_rad - self._wheels[1])
        )
        self._stall_trans += abs(cmd_l + cmd_r) * dt
        if self._stall_t >= 0.20:
            expected = self._stall_exp
            actual = self._stall_act
            trans = self._stall_trans / self._stall_t
            self._stall_t = 0.0
            self._stall_exp = 0.0
            self._stall_act = 0.0
            self._stall_trans = 0.0
            self._stall_latched = expected >= 0.04 and trans > 0.4 and actual < 0.30 * expected
        return self._stall_latched

    def _watch_pin(self, depth, ds: float, rear_depth=None) -> None:
        """Freeze wheel translation when depth, floor, and visual odometry are all still.

        The forward range can keep changing while the shell yaws against a
        jamb, so the old clearance check never closed. These three measure
        translation. A fast yaw no longer throws their readings away.
        While the tail leads (reverse or a spin), the rear camera is the
        depth vote. The forward picture still shows the room behind the motion.
        """
        cmd_l, cmd_r = self._commanded
        tail_first = (cmd_l + cmd_r) < -0.3 or (
            abs(cmd_l + cmd_r) < 0.4 and abs(cmd_l - cmd_r) > 1.0
        )
        depth_m = None
        if tail_first and rear_depth is not None:
            if self._prev_rear is not None:
                dyaw = self.odometry.yaw_unwrapped_rad - self._prev_rear_yaw
                depth_m = depth_scene_motion_m(self._prev_rear, rear_depth, REAR_DEPTH, dyaw)
            self._prev_rear = rear_depth
            self._prev_rear_yaw = self.odometry.yaw_unwrapped_rad
        else:
            if depth is not None and self._prev_depth is not None:
                dyaw = self.odometry.yaw_unwrapped_rad - self._prev_depth_yaw
                depth_m = depth_scene_motion_m(self._prev_depth, depth, self.camera, dyaw)
            if depth is not None:
                self._prev_depth = depth
                self._prev_depth_yaw = self.odometry.yaw_unwrapped_rad
        translating = abs(self._commanded[0] + self._commanded[1]) > 0.4
        if not translating:
            # A sweep stop or a one-step spin is not the end of the
            # scrub. Wiping the window there dropped the still votes
            # before the wheels had claimed enough travel to call a pin.
            self._pin_idle_s += self._step_dt
            if self._pin_idle_s > 0.80:
                self._pin.reset()
            return
        self._pin_idle_s = 0.0
        if self._pin.wheel_m <= 0.0:
            self._pin.anchor(self._mark_xy[0], self._mark_xy[1])
        self._pin.add(ds, depth_m, self._fusion.scene_floor_m, self._fusion.scene_visual_m)
        if not self._pin.ready():
            return
        pinned = self._pin.pinned()
        if pinned:
            self.odometry.x_m = self._pin.x0
            self.odometry.y_m = self._pin.y0
            self._pin_until = self.t_s + 1.2
            self._log("pinned, depth floor and visual still, wheel translation frozen")
            # A crossing that is already lined up stays the goal. Recovery
            # would reverse and drop it, which is how a 7 s hall visit ended.
            if self.t_s < self._commit_until:
                self._progress_t = self._commit_until
            else:
                self._start_recovery()
        self._pin.reset()

    def _pressing_side(self, view: BubbleView) -> float:
        """+1 when the left shell is on a surface, -1 for the right, else 0."""
        left = 99.0 if view.left is None else view.left
        right = 99.0 if view.right is None else view.right
        if left < 0.09 and left + 0.04 < right:
            return 1.0
        if right < 0.09 and right + 0.04 < left:
            return -1.0
        return 0.0

    def _watch_side(self, view: BubbleView) -> None:
        """A side gap that stays shut is a jamb, not a nose contact.

        The bubble's reverse drives along the heading and used to count
        encoder radians, so a tire skidding on the frame spent the budget
        and the same doorway was tried again. Hold the press, then yaw
        off that side and keep reversing until the gap opens.
        """
        if self.phase != "go" or self.t_s < self._side_escape_until:
            return
        side = self._pressing_side(view)
        if side == 0.0:
            self._side_sign = 0.0
            self._side_since = None
            return
        if side != self._side_sign or self._side_since is None:
            self._side_sign = side
            self._side_since = self.t_s
            return
        if self.t_s - self._side_since >= 0.45:
            self._start_side_escape(side, view)

    def _side_opened(self, view: BubbleView) -> bool:
        if self._side_escape_sign > 0.0:
            return view.left is None or view.left > 0.14
        if self._side_escape_sign < 0.0:
            return view.right is None or view.right > 0.14
        return True

    def _start_side_escape(self, side: float, view: BubbleView) -> None:
        which = "left" if side > 0.0 else "right"
        gap = view.left if side > 0.0 else view.right
        shown = -1.0 if gap is None else float(gap)
        self._log(f"side contact on the {which}, backing off the frame gap={shown:.2f}")
        key = self._goal_key()
        if key is not None:
            self._fails[key] = self._fails.get(key, 0) + 1
            self._blacklist(key, radius=3, ttl_s=8.0)
        self.goal_xy = None
        self.path_xy = []
        self._replan_at = self.t_s + 1.2
        self._side_escape_sign = side
        self._side_escape_until = self.t_s + 1.3
        self._reverse_left_m = 0.22
        self._reverse_until = self._side_escape_until
        self._reverse_needs_gap = True
        self._reverse_release = False
        self._recover_until = max(self._recover_until, self.t_s + 0.8)
        self._progress_t = self._side_escape_until
        self._progress_xy = self.odometry.pose()[:2]
        self._confident_after = max(self._confident_after, self.t_s + 4.0)
        self._turn = -side * SPIN_RAD_S
        self.phase = "go"
        self.done = False

    def _door_yaw(self, left_clear: float, right_clear: float) -> tuple[float, str]:
        """Centre on a doorway. The mode is mouth, approach, or empty.

        Mouth: both side sonars see the frame, so a small yaw holds the
        line while the shell is in the opening. Approach: the depth
        shoulders see a gap the nose has not entered yet, so the drive
        lines up on that centre line and then goes straight. A press
        closer than 5 cm is the side-escape case, not a centering command.
        """
        view = self._bubble_view
        left_s = None if view is None else view.left
        right_s = None if view is None else view.right
        nose = self._last_clear[0]
        if (
            left_s is not None
            and right_s is not None
            and 0.05 < left_s < 0.50
            and 0.05 < right_s < 0.50
        ):
            return self._jamb_yaw(left_s, right_s), "mouth"
        nearer = min(float(left_clear), float(right_clear))
        if left_clear < 1.20 and right_clear < 1.20 and nose > nearer + 0.20:
            return self._jamb_yaw(left_clear, right_clear), "approach"
        return 0.0, ""

    def _log(self, message: str) -> None:
        print(f"MAPPING t={self.t_s:.1f}s {message}", flush=True)

    def _note_contact(self) -> None:
        x, y, yaw = self.odometry.pose()
        fresh = self.t_s >= self._reverse_until
        # Tire outer corners. The old point 0.28 m down the centre line
        # sat in the 0.81 m opening and stayed occupied.
        self._reinforce_corners(x, y, yaw)
        if not fresh:
            return
        # Only as far as the pivot disk needs. A fixed two-second reverse
        # yanked the robot off a doorway it had already lined up with.
        self._arm_reverse(self._min_gap, blind=self._view_blind, force_min_m=0.14, reason="contact")
        self.path_xy = []
        self._replan_at = self._avoid_until
        nose, left_c, right_c = self._last_clear
        self._log(f"contact, backing up nose={nose:.2f} left={left_c:.2f} right={right_c:.2f}")

    def _note_slip(self) -> None:
        self.slip_events += 1
        self._confident_after = max(self._confident_after, self.t_s + 6.0)
        self._start_recovery()

    def _start_recovery(self) -> None:
        """Reverse, drop the frontier ahead, and refuse to call the run finished."""
        if self.t_s < self._recover_until:
            return
        x, y, yaw = self.odometry.pose()
        nose, left_c, right_c = self._last_clear
        self._log(f"backup nose={nose:.2f} left={left_c:.2f} right={right_c:.2f} blind={int(self._view_blind)}")
        # An open nose is not a wall. Banning the goal paints a disk on the
        # room beyond the doorway, which is how the bedroom was abandoned.
        if nose >= 1.50:
            self._log("stall with an open nose, not painting the opening")
            self.goal_xy = None
            self.path_xy = []
            self._replan_at = self.t_s + 0.4
            self._recover_until = self.t_s + 0.8
            self._progress_t = self.t_s + 0.8
            self._progress_xy = (x, y)
            self.phase = "go"
            self.done = False
            return
        self._ban_current(immediate=True)
        # A small, expiring patch on the jamb. Radius 12 sealed the 0.81 m
        # opening and the ban never came back, so the bedroom was abandoned.
        self._blacklist_xy(x + math.cos(yaw) * 0.30, y + math.sin(yaw) * 0.30, radius=4, ttl_s=12.0)
        self._reinforce_corners(x, y, yaw)
        tight = self._tight_face()
        if tight is not None and tight[0] == "rear" and tight[1] < 0.12:
            # The tail is on the wall. A reverse drives further in.
            self._reverse_left_m = 0.0
            self._reverse_needs_gap = False
        else:
            self._arm_reverse(self._min_gap, blind=self._view_blind, force_min_m=0.08, reason="backup")
        self._recover_until = self.t_s + 0.8
        self._progress_t = self._recover_until
        self._progress_xy = (x, y)
        self._confident_after = max(self._confident_after, self.t_s + 6.0)
        self._turn = SPIN_RAD_S if self._turn < 0.0 else -SPIN_RAD_S
        self.path_xy = []
        self.goal_xy = None
        self._replan_at = self._recover_until
        self.phase = "go"
        self.done = False

    def _confident(self) -> bool:
        # Still against a surface the camera cannot see, or the wheels
        # recently claimed travel the scan does not support.
        if self._view_blind or self._slip_debt_m >= 0.35:
            return False
        if self.t_s < self._confident_after:
            return False
        if self._scan_xy is not None and self.t_s - self._scan_at <= 1.5:
            if scan_fit(self.grid, self._scan_xy) == "disagree":
                self._confident_after = max(self._confident_after, self.t_s + 6.0)
                return False
        return True

    def _jamb_yaw(self, left_clear: float, right_clear: float) -> float:
        """Centre in a gap. A wall on only one side is not a doorway.

        Yawing at every nearby shoulder turned the robot parallel to the
        bedroom wall and held it there. Both shoulders have to be close
        before this adds a turn, and a matched pair stays straight.
        """
        if left_clear > 1.3 or right_clear > 1.3:
            return 0.0
        # Positive error: more air on the left, so the body is right of
        # the centre line and should yaw left. A matched pair (a few
        # centimetres) stays straight instead of hunting.
        error = float(left_clear) - float(right_clear)
        if abs(error) < 0.08:
            return 0.0
        return max(-0.45, min(0.45, 1.1 * error))

    def _speed_cap(self, view) -> float:
        cap = CRUISE_M_S
        for gap in (view.forward, view.left, view.right, view.rear):
            if gap is None:
                continue
            cap = min(cap, max_speed_for_gap(gap))
        return cap

    def _arm_reverse(
        self,
        gap: float | None,
        *,
        blind: bool,
        force_min_m: float = 0.0,
        reason: str,
    ) -> None:
        budget = reverse_budget_m(gap, blind=blind, force_min_m=force_min_m)
        if budget < 0.03:
            return
        if self._reverse_left_m >= budget * 0.8 and self.t_s < self._reverse_until:
            return
        self._reverse_left_m = max(self._reverse_left_m, budget)
        self._reverse_release = gap is not None and not pivot_ready(gap)
        self._reverse_until = self.t_s + budget / 0.12 + 0.35
        self._avoid_until = max(self._avoid_until, self._reverse_until + 0.3)
        shown = -1.0 if gap is None else float(gap)
        self._log(f"{reason} backup {budget:.2f}m gap={shown:.2f}")

    def _tight_face(self) -> tuple[str, float] | None:
        """The shell face with the smallest measured gap."""
        view = self._bubble_view
        if view is None:
            return None
        known = [
            (name, gap)
            for name, gap in (
                ("forward", view.forward),
                ("rear", view.rear),
                ("left", view.left),
                ("right", view.right),
            )
            if gap is not None
        ]
        if not known:
            return None
        return min(known, key=lambda item: item[1])

    def _leave_rear(self) -> tuple[float, float] | None:
        """Drive forward when the tail is the face on the wall.

        The pivot backup always reversed. With the nose in an open room
        and the rear sonar on the shared wall, that command held the
        tires on the wall for minutes and the encoders walked the pose.
        """
        tight = self._tight_face()
        if tight is None or tight[0] != "rear" or tight[1] >= 0.10:
            return None
        self._reverse_left_m = 0.0
        self._reverse_needs_gap = False
        if self.t_s >= self._escape_log_at:
            self._log(f"rear contact, driving off the wall gap={tight[1]:.2f}")
            self._escape_log_at = self.t_s + 0.5
        return wheel_speeds(0.12, 0.0)

    def _drive_off_block(self) -> tuple[float, float] | None:
        """Leave a block that a spin has not opened."""
        view = self._bubble_view
        if view is None:
            return None

        def air(gap: float | None) -> float:
            return 9.0 if gap is None else float(gap)

        faces = {
            "forward": air(view.forward),
            "rear": air(view.rear),
            "left": air(view.left),
            "right": air(view.right),
        }
        open_name = max(faces, key=lambda name: faces[name])
        if faces[open_name] < 0.20:
            return None
        self._reverse_left_m = 0.0
        self._reverse_needs_gap = False
        if self.t_s >= self._escape_log_at:
            self._log(f"blocked spin, driving toward {open_name} gap={faces[open_name]:.2f}")
            self._escape_log_at = self.t_s + 0.5
        if open_name == "rear":
            return wheel_speeds(-0.12, 0.0)
        yaw = 0.45 if open_name == "left" else (-0.45 if open_name == "right" else 0.0)
        return wheel_speeds(0.12, yaw)

    def _go(self, blocked: bool, cap: float, left_clear: float = 99.0, right_clear: float = 99.0) -> tuple[float, float]:
        if self.done:
            return 0.0, 0.0
        leaving = self._leave_rear()
        if leaving is not None:
            return leaving
        if self._side_escape_sign != 0.0 and self.t_s <= self._side_escape_until:
            # Yaw away from the jamb while reversing. A straight reverse
            # stays on a frame that is beside the shell.
            return wheel_speeds(-0.10, -0.55 * self._side_escape_sign)
        if self._reverse_left_m > 0.0 and self.t_s <= self._reverse_until:
            # Nearly straight. A hard yaw here swings a tire into the jamb
            # the reverse was supposed to leave.
            return wheel_speeds(-0.12, self._turn * 0.08)
        if self._goal_expired() or (self._is_stuck() and (self._bubble_gap or 99.0) < 1.2):
            self._ban_current()
            self._plan()
        if self.phase != "go":
            return 0.0, 0.0
        if blocked and not pivot_ready(self._min_gap):
            # Nose contact backs up. A rear contact was already returned
            # above. A side gap is the side-escape path, not another reverse
            # along the wall.
            tight = self._tight_face()
            if tight is not None and tight[0] in ("left", "right") and tight[1] < 0.09:
                if self.t_s > self._side_escape_until:
                    self._start_side_escape(1.0 if tight[0] == "left" else -1.0, self._bubble_view)
                return wheel_speeds(-0.10, -0.55 * self._side_escape_sign)
            self._arm_reverse(self._min_gap, blind=self._view_blind, reason="pivot")
            if self._reverse_left_m > 0.0:
                return wheel_speeds(-0.12, self._turn * 0.08)
        if (
            blocked
            and self._blocked_since is not None
            and (self.t_s - self._blocked_since) > 1.2
        ):
            # A spin that does not open the gap is the scrub that walked
            # the encoders. Drive toward the open face instead.
            leaving_block = self._drive_off_block()
            if leaving_block is not None:
                return leaving_block
        if self.t_s < self._avoid_until or blocked:
            return wheel_speeds(0.0, self._turn)
        if self.t_s >= self._replan_at or not self.path_xy:
            self._plan()
        if self.phase != "go" or self.done or not self.path_xy:
            return 0.0, 0.0
        pose = self.odometry.pose()
        v, w, arrived = pure_pursuit(pose, self.path_xy, arrive_m=ARRIVE_M)
        raw_w = w
        door_w, door_mode = self._door_yaw(left_clear, right_clear)
        nose_open = self._bubble_gap is None or self._bubble_gap > 0.12
        through = arrived and left_clear < 1.05 and right_clear < 1.05 and nose_open
        if arrived and not through:
            self._last_spin_t = self.t_s
            self.phase = "spin"
            self._spin_start = None
            self.goal_xy = None
            self.path_xy = []
            return wheel_speeds(0.0, SPIN_RAD_S)
        approach_kind = ""
        if door_mode == "approach":
            # Outside the frame: face the centre line, then drive straight.
            # An arc here is the corner cut that puts a tire on the jamb.
            err = wrap_rad(self._segment_heading() - pose[2])
            if abs(err) > 0.12:
                v = 0.0
                w = max(-0.55, min(0.55, math.copysign(0.55, err)))
                approach_kind = "align"
            else:
                v = DOOR_SPEED_M_S
                w = 0.0
                if abs(raw_w) > 0.15:
                    approach_kind = "straight"
        elif door_mode == "mouth" or through:
            # In the opening, or arriving with both jambs beside the nose.
            w = door_w if through else w + door_w
            v = min(DOOR_SPEED_M_S if through else v, DOOR_SPEED_M_S)
        elif left_clear < 0.90 and right_clear < 0.90:
            v = min(v, DOOR_SPEED_M_S)
        v = min(v, max(0.0, cap))
        v, w = self._apply_sweep(v, w, approach_kind, door_mode)
        return wheel_speeds(v, w)

    def _segment_heading(self) -> float:
        """Heading of the polyline about 0.35 m ahead.

        That is the next segment, not the chord from the robot to a
        later corner. The chord is what cuts the doorway.
        """
        x, y, yaw = self.odometry.pose()
        path = self.path_xy
        if not path:
            return yaw
        nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
        if nearest >= len(path) - 1:
            dx = path[-1][0] - x
            dy = path[-1][1] - y
            if math.hypot(dx, dy) < 0.05:
                return yaw
            return math.atan2(dy, dx)
        travelled = 0.0
        prev = path[nearest]
        heading = yaw
        for nxt in path[nearest + 1 :]:
            dx = nxt[0] - prev[0]
            dy = nxt[1] - prev[1]
            seg = math.hypot(dx, dy)
            if seg > 1e-6:
                heading = math.atan2(dy, dx)
            travelled += seg
            if travelled >= 0.35:
                return heading
            prev = nxt
        return heading

    def _apply_sweep(self, v: float, w: float, approach_kind: str, door_mode: str = "") -> tuple[float, float]:
        """Reject an arc the shell would drive into an occupied cell."""
        pose = self.odometry.pose()
        v2, w2, kind = fit_motion(self.grid, pose, v, w, self._segment_heading())
        nose, left_c, right_c = self._last_clear
        # Live jambs, matched, and air ahead. The occupancy sweep was
        # vetoing this with a smeared cell, and the segment heading kept
        # the body spinning on the threshold instead of driving through.
        centered = (
            door_mode in ("approach", "mouth")
            and 0.18 < left_c < 0.70
            and 0.18 < right_c < 0.70
            and abs(left_c - right_c) < 0.12
            and nose > 0.25
        )
        if centered and (kind != "clear" or abs(w) > 0.15 or v < 0.05):
            v2 = DOOR_SPEED_M_S
            w2 = 0.0
            kind = "clear"
        self.sweep_checks += 1
        if kind != "clear":
            self.sweep_rejected += 1
            if kind == "wider":
                self.sweep_widened += 1
            elif kind == "straight":
                self.sweep_straight += 1
            elif kind == "align":
                self.sweep_aligned += 1
            elif kind == "stop":
                self.sweep_stopped += 1
        elif approach_kind == "align":
            self.sweep_aligned += 1
        elif approach_kind == "straight":
            self.sweep_straight += 1
        modified = kind != "clear" or approach_kind != ""
        if modified and self.t_s >= self._sweep_log_at:
            if kind != "clear":
                self._log(f"sweep {kind}, footprint would clip")
            elif approach_kind == "align":
                self._log("sweep align, doorway off the centre line")
            else:
                self._log("sweep straight, doorway on the centre line")
            self._sweep_log_at = self.t_s + 0.5
        # Lined up, and the straight shell misses the map. Stay on this
        # heading instead of taking the next frontier back through the door.
        if door_mode in ("approach", "mouth") and kind == "clear" and v2 > 0.05 and abs(w2) < 0.08:
            self._commit_until = max(self._commit_until, self.t_s + 16.0)
            # The polyline heading hunts between the jambs. Once the
            # sonars agree, the way through is the way the shell is facing.
            self._commit_heading = pose[2] if centered else self._segment_heading()
        return v2, w2

    def sweep_summary(self) -> str:
        return (
            f"checks={self.sweep_checks} rejected={self.sweep_rejected} "
            f"wide={self.sweep_widened} align={self.sweep_aligned} "
            f"straight={self.sweep_straight} stop={self.sweep_stopped}"
        )

    def _forward_preference(self) -> tuple[float, float] | None:
        if self._commit_until < 0.0 or self.t_s > self._commit_until + 20.0:
            return None
        return (math.cos(self._commit_heading), math.sin(self._commit_heading))

    def _hold_goal(self) -> bool:
        """Keep the path while a lined-up doorway crossing is committed.

        Replanning every few seconds handed the -0.35 seed a frontier back
        in the main room seven seconds after it had entered the hall.
        Holding every young goal did the opposite: the robot kept a
        frontier it could not reach and spun in place.
        """
        if self.goal_xy is None or not self.path_xy:
            return False
        return self.t_s < self._commit_until

    def _plan(self, depth: int = 0) -> None:
        self._replan_at = self.t_s + 3.0
        x, y, yaw = self.odometry.pose()
        if self._hold_goal():
            return
        if depth > 6:
            self._no_frontier()
            return
        banned = self._ban_set()
        prefer = self._forward_preference()
        heading_xy = (math.cos(yaw), math.sin(yaw))
        rooms = segment_map(self.grid.labels(), self.grid.res)
        self.room_labels = rooms["labels"]
        self.room_names = rooms["names"]
        self.room_doors = rooms["doors"]
        chosen = select_goal(
            self.grid,
            (x, y),
            radius_m=ROBOT_RADIUS_M,
            banned=banned,
            prefer_xy=prefer,
            rooms=rooms,
            heading_xy=heading_xy,
        )
        if chosen is None and self._banned:
            self._log("no goal under bans, clearing them")
            self._banned.clear()
            chosen = select_goal(
                self.grid,
                (x, y),
                radius_m=ROBOT_RADIUS_M,
                banned=set(),
                prefer_xy=prefer,
                rooms=rooms,
                heading_xy=heading_xy,
            )
        if chosen is None:
            self._no_frontier()
            return
        goal, path = chosen
        if (
            self.goal_xy is not None
            and self.path_xy
            and math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1]) < 1.2
        ):
            return
        cell = self.grid.cell_of(goal[0], goal[1])
        # Already standing on it: look once more, then drop it so a speck
        # of unknown next to the robot cannot spin the run forever.
        if cell is not None and math.hypot(goal[0] - x, goal[1] - y) < ARRIVE_M:
            # One look. A second spin on the same door is how the Webots run
            # sat against the loveseat for minutes. An open view is not that
            # case: reversing there turned the robot off the bedroom door.
            if self.t_s - self._last_spin_t < 25.0:
                self._ban_current(immediate=True)
                nose = self._last_clear[0]
                if self._view_blind or nose < 1.5:
                    self._start_recovery()
                else:
                    self._log("goal already under the robot, skipping the reverse")
                    self._plan(depth + 1)
                return
            count = self._fails.get(cell, 0) + 1
            self._fails[cell] = count
            if count >= 2:
                self._blacklist(cell, radius=4)
                self._plan(depth + 1)
                return
            self._last_spin_t = self.t_s
            self.phase = "spin"
            self._spin_start = None
            self.goal_xy = None
            self.path_xy = []
            return
        same = self.goal_xy is not None and math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1]) < 0.2
        self.goal_xy, self.path_xy = goal, path
        self.frontiers_left = 1
        self.phase = "go"
        if not same:
            self._goal_since = self.t_s
            self._stuck_anchor = (self.t_s, x, y)
            self._did_final_scan = False
            self._settle_since = None
            self._log(f"goal ({goal[0]:.2f},{goal[1]:.2f})")

    def _goal_key(self) -> tuple[int, int] | None:
        if self.goal_xy is None:
            return None
        cell = self.grid.cell_of(self.goal_xy[0], self.goal_xy[1])
        return cell

    def _no_frontier(self) -> None:
        """No reachable frontier. Done only after a scan and a stopped hold."""
        if not self._confident():
            # A smeared map can hide every frontier. Look again once the
            # pose has stopped slipping instead of declaring the apartment done.
            self._log("frontiers hidden, pose not trusted, not done")
            self.goal_xy = None
            self.path_xy = []
            self.phase = "go"
            self.done = False
            self._did_final_scan = False
            self._reverse_until = max(self._reverse_until, self.t_s + 0.8)
            self._replan_at = self.t_s + 1.5
            self._progress_t = self._replan_at
            return
        if not self._did_final_scan:
            self._did_final_scan = True
            self._log("no frontier, final scan")
            self.goal_xy = None
            self.path_xy = []
            self.phase = "spin"
            self._spin_start = None
            self._spin_then = "settle"
            self.done = False
            return
        self.frontiers_left = 0
        self.goal_xy = None
        self.path_xy = []
        self.phase = "settle"
        self.done = False
        self._settle_since = self.t_s
        self._settle_occ = int((self.grid.labels() == 1).sum())
        self._settle_check = self.t_s
        self._log("stopped, holding for a stable map")

    def _settle(self) -> tuple[float, float]:
        if self._settle_since is None:
            self._settle_since = self.t_s
            self._settle_occ = int((self.grid.labels() == 1).sum())
        if self.t_s < self._settle_check:
            return 0.0, 0.0
        self._settle_check = self.t_s + 0.5
        x, y, yaw = self.odometry.pose()
        heading_xy = (math.cos(yaw), math.sin(yaw))
        rooms = segment_map(self.grid.labels(), self.grid.res)
        self.room_labels = rooms["labels"]
        self.room_names = rooms["names"]
        self.room_doors = rooms["doors"]
        chosen = select_goal(
            self.grid, (x, y), radius_m=ROBOT_RADIUS_M, banned=self._ban_set(), rooms=rooms, heading_xy=heading_xy
        )
        if chosen is None and self._banned:
            self._log("no goal under bans, clearing them")
            self._banned.clear()
            chosen = select_goal(
                self.grid, (x, y), radius_m=ROBOT_RADIUS_M, banned=set(), rooms=rooms, heading_xy=heading_xy
            )
        if chosen is not None:
            self._log("frontier open, resuming")
            self._did_final_scan = False
            self.phase = "go"
            goal, path = chosen
            self.goal_xy, self.path_xy = goal, path
            self._goal_since = self.t_s
            self._stuck_anchor = (self.t_s, x, y)
            self._log(f"goal ({goal[0]:.2f},{goal[1]:.2f})")
            return 0.0, 0.0
        count = int((self.grid.labels() == 1).sum())
        if self._settle_occ is None or abs(count - self._settle_occ) > 8:
            self._settle_occ = count
            self._settle_since = self.t_s
            self._log("map changed, reset done hold")
            return 0.0, 0.0
        if self.t_s - self._settle_since >= 4.0 and self._confident():
            self.frontiers_left = 0
            self.phase = "done"
            self.done = True
            self._log("done: no reachable frontier after a final scan, map stable while stopped")
        return 0.0, 0.0

    def _ban_current(self, *, immediate: bool = False) -> None:
        key = self._goal_key()
        self.goal_xy = None
        self.path_xy = []
        self._replan_at = -1.0
        if key is None:
            return
        count = self._fails.get(key, 0) + 1
        self._fails[key] = count
        if immediate or count >= 2:
            # A short ban, not a permanent disk. The opening has to be
            # reachable again after the robot has backed up and looked.
            self._blacklist(key, radius=5 if immediate else 3, ttl_s=12.0)

    def _ban_set(self) -> set[tuple[int, int]]:
        expired = [cell for cell, until in self._banned.items() if self.t_s >= until]
        for cell in expired:
            del self._banned[cell]
        return set(self._banned)

    def _blacklist(self, cell: tuple[int, int], *, radius: int, ttl_s: float = 12.0) -> None:
        until = self.t_s + float(ttl_s)
        n = 0
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dy * dy + dx * dx <= radius * radius:
                    key = (cell[0] + dy, cell[1] + dx)
                    self._banned[key] = max(self._banned.get(key, 0.0), until)
                    n += 1
        self._log(f"blacklist {n} cells for {ttl_s:.0f}s around ({cell[0]},{cell[1]})")

    def _reinforce_corners(self, x: float, y: float, yaw: float) -> None:
        """Bump the two front tire corners, not the air in front of the nose."""
        c, s = math.cos(yaw), math.sin(yaw)
        for fx, fy in ((0.10, 0.20), (0.10, -0.20)):
            self.grid.mark_contact(x + c * fx - s * fy, y + s * fx + c * fy)

    def _blacklist_xy(self, x: float, y: float, *, radius: int, ttl_s: float = 12.0) -> None:
        cell = self.grid.cell_of(x, y)
        if cell is not None:
            self._blacklist(cell, radius=radius, ttl_s=ttl_s)

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
