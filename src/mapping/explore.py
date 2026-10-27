"""Frontier exploration loop.

The robot spins once to fill the depth camera's blind sides, then picks
a frontier every few seconds and drives the motion live range still
allows. Sweep, doorway, and footprint checks are costs. A command that
does not move the odometry waits once, reverses once, and tries the
next goal. Depth closer than the stop distance is the only veto.

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
from src.mapping.bubble import BubbleView, assess_bubble
from src.mapping.depth_cloud import (
    blind_fraction,
    body_points,
    depth_scene_motion_m,
    forward_clearance,
    rear_clearance_m,
    to_odom,
)
from src.mapping.drive import pure_pursuit, wheel_speeds
from src.mapping.floor_flow import ShortFloorTrack
from src.mapping.frontiers import select_goal
from src.mapping.tick import choose_motion, log_decision
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

# WheelGyroOdometry treats a rate under this as the lookup offset, not a
# turn. A pure pivot sits above it. The floor camera is on the spin axis,
# so that picture stays still and must not freeze the gyro.
_GYRO_REST_RAD_S = 0.02

def _picture_still(prev, curr, *, limit: float, min_std: float) -> bool:
    """True when two frames are the same picture, not a shifted one.

    A blank frame is not evidence. A couple of pixels of noise is.
    Comparing against the frame from the start of the pause, rather than
    the previous 8 ms, is what separates a creep from a body that is sitting
    still: door speed moves the floor by many pixels in a fifth of a second.
    """
    if prev is None or curr is None:
        return False
    before = np.asarray(prev, dtype=np.float64)
    after = np.asarray(curr, dtype=np.float64)
    if before.shape != after.shape or before.ndim != 2 or before.size < 16:
        return False
    finite = np.isfinite(before) & np.isfinite(after)
    if int(np.count_nonzero(finite)) < 16:
        return False
    if float(np.std(before[finite])) < min_std and float(np.std(after[finite])) < min_std:
        return False
    best = float(np.mean(np.abs(before[finite] - after[finite])))
    width = before.shape[1]
    for shift in (1, 2):
        if width <= shift + 2:
            break
        left = before[:, :-shift]
        right = after[:, shift:]
        usable = np.isfinite(left) & np.isfinite(right)
        if int(np.count_nonzero(usable)) < 16:
            continue
        best = min(best, float(np.mean(np.abs(left[usable] - right[usable]))))
        left = before[:, shift:]
        right = after[:, :-shift]
        usable = np.isfinite(left) & np.isfinite(right)
        if int(np.count_nonzero(usable)) < 16:
            continue
        best = min(best, float(np.mean(np.abs(left[usable] - right[usable]))))
    return best < limit


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
        self._goal_since = 0.0
        self._wheels: tuple[float, float] | None = None
        self._commanded = (0.0, 0.0)
        self._motion = MotionCheck()
        self._confident_after = 0.0
        self._last_spin_t = -100.0
        self._nudge_at = 0.0
        self._match_proposal: tuple[float, float, float] | None = None
        self._match_proposal_t = -10.0
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
        self._bubble_view: BubbleView | None = None
        self._pin = PinWatch()
        self._pin_idle_s = 0.0
        self._pin_until = -1.0
        self._mark_xy = (0.0, 0.0)
        self._prev_depth = None
        self._prev_depth_yaw = 0.0
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
        self._attempts: list[tuple[float, float, float, float]] = []
        self._motion_anchor: tuple[float, float, float, float] | None = None
        self._relief: str | None = None
        self._relief_until = -1.0
        self._relief_w = 0.0
        self._body_v = 0.0
        self._body_w = 0.0
        self._last_motion = ""
        self._motion_log_at = -1.0
        self._radar_blocked = False
        self.room_labels = None
        self.room_names: dict[int, str] = {}
        self.room_doors = None
        # Cells the shell has actually covered. A depth ray through a
        # doorway paints the next room free; that is not a visit.
        self.visited = np.zeros((self.grid.n, self.grid.n), dtype=bool)
        self._visited_cell: tuple[int, int] | None = None
        self._rear_gap: float | None = None
        self._prev_rear = None
        self._prev_rear_yaw = 0.0
        # Pose and pictures at the start of a stretch the cameras have not
        # moved. A zero-velocity update puts the pose back there.
        self._zupt_anchor: dict | None = None
        self._zupt_until = -1.0
        # Translation hold from the floor sensor alone. Yaw stays on the
        # gyro until depth and vision agree as well.
        self._zupt_trans_until = -1.0
        self._floor_track = ShortFloorTrack()
        self._zupt_wheels: tuple[float, float] | None = None
        self._pitch_rest: float | None = None
        self.zupt_events = 0
        self.wheels_lifted = 0

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
        elif self._held_clear is not None:
            # Depth arrives at 10 Hz. Dropping it on the steps in between
            # made the shell look open and the speed cap ignored the wall.
            clearance, left_clear, right_clear = self._held_clear
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
        # While the cameras still agree the shell is sitting, do not
        # integrate this step. The latch is refreshed only when a new
        # picture says the same thing.
        # A real gyro rate is a turn even when the floor image is still.
        # Holding yaw there rewound every sample, so the heading never
        # reached the release and the turn stayed frozen.
        gyro_turning = abs(float(yaw_rate_rad_s)) >= _GYRO_REST_RAD_S
        held_yaw = self.t_s < self._zupt_until and not gyro_turning
        held_trans = held_yaw or self.t_s < self._zupt_trans_until
        self.odometry.update(
            left_wheel_rad,
            right_wheel_rad,
            yaw_rate_rad_s,
            dt,
            trust_translation=not drop and not spin_cmd and not held_trans,
            trust_yaw=not held_yaw,
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
                dt_s=dt,
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
        self._watch_zupt(
            depth,
            floor_image,
            rgb,
            yaw_rate_rad_s,
            pitch_rad,
            left_wheel_rad,
            right_wheel_rad,
        )
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
            and self.t_s >= self._zupt_until
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
        self._radar_blocked = bool(radar_blocked)
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
            and self.t_s >= self._zupt_until
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
        self._watch_motion()
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
            return self._remember(self._go(False, 0.0, left_clear, right_clear))
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
        self._pin.reset()

    def _arm_zupt_anchor(self, depth, floor, rgb, left_rad: float, right_rad: float) -> None:
        held_depth = None
        if depth is None and self._zupt_anchor is not None:
            held_depth = self._zupt_anchor.get("depth")
        self._zupt_anchor = {
            "t": self.t_s,
            "x": self.odometry.x_m,
            "y": self.odometry.y_m,
            "yaw": self.odometry.yaw_unwrapped_rad,
            "depth": np.array(depth, copy=True) if depth is not None else held_depth,
            "floor": np.array(floor, copy=True),
            "rgb": None if rgb is None else np.array(rgb, copy=True),
            "left": float(left_rad),
            "right": float(right_rad),
            "logged": False,
            "lifted": False,
            "backed": False,
            "left_spin": False,
            "still_hits": 0,
            "misses": 0,
        }

    def _zupt_travel(self, left_rad: float, right_rad: float) -> float:
        """Wheel travel since the previous floor step, in metres.

        Translation plus spin, so a scrub that nets zero forward still
        closes a floor step. The first sample only arms the baseline.
        """
        prev = self._zupt_wheels
        self._zupt_wheels = (float(left_rad), float(right_rad))
        if prev is None:
            return 0.0
        dl = float(left_rad) - prev[0]
        dr = float(right_rad) - prev[1]
        radius = self.odometry.radius_m
        track = max(self.odometry.track_m, 1e-6)
        return abs(0.5 * (dl + dr)) * radius + abs(dr - dl) * radius / track

    def _watch_zupt(
        self,
        depth,
        floor,
        rgb,
        yaw_rate: float,
        pitch: float,
        left_rad: float,
        right_rad: float,
    ) -> None:
        """Freeze translation when the floor image has not moved.

        The downward camera is the stuck signal. A correlation against an
        anchor from a fifth of a second ago wraps: 32 px is about 3.9 cm,
        and one period comes back as a still picture, which reversed a
        real 0.12 m/s doorway crossing. The stall is a run of short steps,
        each about 1.2 cm, all matching.         Depth and the forward view used
        to have to agree before a still floor counted, and a yaw against
        the jamb changed those pictures while the pile under the shell did
        not. Yaw is frozen only when those pictures agree too, and only
        while the gyro is inside its rest gate. A pure pivot leaves the
        floor image still. The gyro keeps that turn, and only translation
        is put back on the anchor.
        The hubs may be spinning the whole time.
        """
        if self._pitch_rest is None and self.odometry.still_s > 1.0 and abs(float(pitch)) < 0.15:
            self._pitch_rest = float(pitch)
        travel = self._zupt_travel(left_rad, right_rad)
        if floor is None:
            return
        if self._zupt_anchor is None:
            # Wait for a depth frame so a later yaw freeze has a picture
            # to compare. Translation does not need it, but the first
            # arm is the one the forward view is judged against.
            if depth is None:
                return
            self._arm_zupt_anchor(depth, floor, rgb, left_rad, right_rad)
            self._floor_track.reset()
            return
        anchor = self._zupt_anchor
        age = self.t_s - float(anchor["t"])
        status = self._floor_track.push(floor, self._step_dt, travel)
        # "pending" is a step that has not closed yet. A blank face and a
        # real slide both restart the pose anchor, so a weak zero cannot
        # sit on an old pose and delete the crossing.
        if status in ("moving", "blank"):
            self._arm_zupt_anchor(depth, floor, rgb, left_rad, right_rad)
            self._floor_track.reset()
            return
        if not self._floor_track.confident_still() or age < 0.40:
            return
        # The downward camera sits on the spin axis, so a real turn
        # leaves that picture still. Rewinding the gyro to the anchor
        # deleted a pivot in one sample (the door heading jumped 29°).
        # Forward wheel travel with that same still picture is the lip:
        # the shell is yawing and not driving. A pure pivot is not.
        yaw_moved = abs(self.odometry.yaw_unwrapped_rad - float(anchor["yaw"]))
        dl_now = float(left_rad) - float(anchor["left"])
        dr_now = float(right_rad) - float(anchor["right"])
        claimed_now = abs(0.5 * (dl_now + dr_now)) * self.odometry.radius_m
        if yaw_moved >= 0.12 and claimed_now < 0.02:
            self._arm_zupt_anchor(depth, floor, rgb, left_rad, right_rad)
            self._floor_track.reset()
            return
        depth_ok = False
        if depth is not None and anchor.get("depth") is not None:
            depth_ok = _picture_still(anchor["depth"], depth, limit=0.045, min_std=0.0)
        vis_ok = bool(self._fusion.scene_visual_still)
        if rgb is not None and anchor["rgb"] is not None:
            before = np.asarray(anchor["rgb"], dtype=np.float64)
            after = np.asarray(rgb, dtype=np.float64)
            if (
                before.shape == after.shape
                and float(np.std(before)) >= 4.0
                and float(np.std(after)) >= 4.0
            ):
                vis_ok = vis_ok and _picture_still(anchor["rgb"], rgb, limit=8.0, min_std=4.0)
        pictures = bool(depth_ok and vis_ok)
        dl = float(left_rad) - float(anchor["left"])
        dr = float(right_rad) - float(anchor["right"])
        claimed = abs(0.5 * (dl + dr)) * self.odometry.radius_m
        spun = abs(dr - dl) * self.odometry.radius_m / max(self.odometry.track_m, 1e-6)
        self.odometry.x_m = float(anchor["x"])
        self.odometry.y_m = float(anchor["y"])
        # Only the steps until the next picture. A latch that ran on
        # after the shell started moving deleted a real turn.
        self._zupt_trans_until = self.t_s + 0.12
        # A real turn keeps the gyro. The still picture is the axis of
        # that turn, not evidence the heading was wrong. Rewinding while
        # the rate stayed above the rest gate held fused yaw near the
        # anchor for the whole pivot.
        turning = abs(float(yaw_rate)) >= _GYRO_REST_RAD_S
        if pictures and yaw_moved < 0.12 and not turning:
            self.odometry.yaw_unwrapped_rad = float(anchor["yaw"])
            self.odometry.yaw_rad = wrap_rad(self.odometry.yaw_unwrapped_rad)
            self._zupt_until = self.t_s + 0.12
        if not anchor["logged"]:
            anchor["logged"] = True
            self.zupt_events += 1
            if pictures and not turning:
                self.odometry.observe_rest(float(yaw_rate), self._step_dt)
                self._log("zero velocity, depth floor and vision still, translation and yaw frozen")
            else:
                self._log("floor still, translation frozen")
        rest = self._pitch_rest
        # The carpet ramp is about 0.04 rad. A real unload tips past that.
        pitched = rest is not None and abs(float(pitch) - rest) > 0.12
        if pitched and (claimed >= 0.02 or spun >= 0.08) and not anchor["lifted"]:
            anchor["lifted"] = True
            self.wheels_lifted += 1
            self._log("wheels lifted, casters holding, drive wheels spinning free")
        # Hubs that are not claiming motion have nothing to undo and
        # nothing to back away from. The pose freeze above is the update.
        if claimed < 0.02 and spun < 0.08:
            return
        if self.phase == "spin" and age >= 0.80 and not anchor["left_spin"]:
            anchor["left_spin"] = True
            self._spin_start = None
            self._spin_then = None
            self.phase = "go"
            self._replan_at = self.t_s
            self._log("spin did not turn the shell")

    def _log(self, message: str) -> None:
        print(f"MAPPING t={self.t_s:.1f}s {message}", flush=True)


    def _note_slip(self) -> None:
        self.slip_events += 1
        self._confident_after = max(self._confident_after, self.t_s + 6.0)

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

    def _avoid_now(self) -> list[tuple[float, float, float]]:
        live: list[tuple[float, float, float, float]] = []
        out: list[tuple[float, float, float]] = []
        for x, y, t0, weight in self._attempts:
            age = self.t_s - t0
            if age > 36.0:
                continue
            live.append((x, y, t0, weight))
            decayed = weight * math.exp(-age / 12.0)
            if decayed > 0.05:
                out.append((x, y, decayed))
        self._attempts = live
        return out

    def _remember_attempt(self, x: float, y: float, weight: float = 2.0) -> None:
        self._attempts.append((float(x), float(y), self.t_s, float(weight)))

    def _watch_motion(self) -> None:
        """Stuck and contact are the same fact: a command the odometry did not follow."""
        if self._relief is not None or self.phase != "go" or self.done:
            return
        translating = abs(self._body_v) > 0.03
        spinning = abs(self._body_w) > 0.12 and not translating
        if not translating and not spinning:
            self._motion_anchor = None
            return
        x, y, yaw = self.odometry.pose()
        if self._motion_anchor is None:
            self._motion_anchor = (self.t_s, x, y, yaw)
            return
        t0, x0, y0, yaw0 = self._motion_anchor
        moved = math.hypot(x - x0, y - y0)
        turned = abs(wrap_rad(yaw - yaw0))
        if translating and moved >= 0.04:
            self._motion_anchor = (self.t_s, x, y, yaw)
            return
        if spinning and turned >= 0.12:
            self._motion_anchor = (self.t_s, x, y, yaw)
            return
        if self.t_s - t0 < 2.2:
            return
        self._relief = "wait"
        self._relief_until = self.t_s + 0.40
        self._relief_w = 0.0
        log_decision(
            {
                "t": round(self.t_s, 2),
                "kind": "stuck",
                "response": "wait",
                "disp_m": round(moved, 3),
            }
        )

    def _open_reverse(self) -> bool:
        view = self._bubble_view
        if view is None or view.rear is None:
            return True
        return float(view.rear) > 0.08

    def _relief_spin(self) -> float:
        view = self._bubble_view
        left = -1.0 if view is None or view.left is None else float(view.left)
        right = -1.0 if view is None or view.right is None else float(view.right)
        if left < 0.0 and right < 0.0:
            _nose, left_c, right_c = self._last_clear
            return SPIN_RAD_S if left_c >= right_c else -SPIN_RAD_S
        return SPIN_RAD_S if left >= right else -SPIN_RAD_S

    def _relief_command(self) -> tuple[float, float] | None:
        if self._relief is None:
            return None
        if self._relief == "wait":
            if self.t_s <= self._relief_until:
                self._body_v, self._body_w = 0.0, 0.0
                return 0.0, 0.0
            if self._open_reverse():
                self._relief = "reverse"
                self._relief_until = self.t_s + 0.45
                log_decision({"t": round(self.t_s, 2), "kind": "stuck", "response": "reverse"})
            else:
                self._relief = "spin"
                self._relief_w = self._relief_spin()
                self._relief_until = self.t_s + 0.45
                log_decision(
                    {
                        "t": round(self.t_s, 2),
                        "kind": "stuck",
                        "response": "spin",
                        "w": round(self._relief_w, 2),
                    }
                )
        if self.t_s <= self._relief_until:
            if self._relief == "reverse":
                self._body_v, self._body_w = -0.10, 0.0
                return wheel_speeds(-0.10, 0.0)
            self._body_v, self._body_w = 0.0, self._relief_w
            return wheel_speeds(0.0, self._relief_w)
        self._finish_relief()
        return None

    def _finish_relief(self) -> None:
        x, y, _yaw = self.odometry.pose()
        if self.goal_xy is not None:
            self._remember_attempt(self.goal_xy[0], self.goal_xy[1], 2.2)
        else:
            self._remember_attempt(x, y, 2.2)
        self.goal_xy = None
        self.path_xy = []
        self._replan_at = self.t_s
        self._relief = None
        self._motion_anchor = None
        self._body_v, self._body_w = 0.0, 0.0
        self._log("stuck, next goal")

    def _segment_heading(self) -> float:
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

    def _note_motion(self, choice) -> None:
        if choice.name == self._last_motion and self.t_s < self._motion_log_at:
            return
        self._last_motion = choice.name
        self._motion_log_at = self.t_s + 1.0
        costs = {key: value for key, value in choice.costs.items() if key != "total" and abs(value) > 1e-3}
        log_decision(
            {
                "t": round(self.t_s, 2),
                "kind": "motion",
                "chosen": choice.name,
                "v": round(choice.v, 3),
                "w": round(choice.w, 3),
                "why": choice.why(),
                "costs": costs,
                "vetoes": choice.vetoes[:8],
            }
        )

    def _go(self, blocked: bool, cap: float, left_clear: float = 99.0, right_clear: float = 99.0) -> tuple[float, float]:
        del blocked, cap
        if self.done:
            self._body_v, self._body_w = 0.0, 0.0
            return 0.0, 0.0
        relief = self._relief_command()
        if relief is not None:
            return relief
        if self.phase != "go":
            return 0.0, 0.0
        if self.t_s >= self._replan_at or not self.path_xy:
            self._plan()
        if self.phase != "go" or self.done or not self.path_xy:
            self._body_v, self._body_w = 0.0, 0.0
            return 0.0, 0.0
        pose = self.odometry.pose()
        v, w, arrived = pure_pursuit(pose, self.path_xy, arrive_m=ARRIVE_M)
        if arrived:
            if self.goal_xy is not None:
                self._remember_attempt(self.goal_xy[0], self.goal_xy[1], 1.0)
            if self.t_s - self._last_spin_t > 12.0:
                self._last_spin_t = self.t_s
                self.phase = "spin"
                self._spin_start = None
                self.goal_xy = None
                self.path_xy = []
                self._body_v, self._body_w = 0.0, SPIN_RAD_S
                return wheel_speeds(0.0, SPIN_RAD_S)
            self.goal_xy = None
            self.path_xy = []
            self._replan_at = self.t_s
            self._plan()
            if self.phase != "go" or not self.path_xy:
                self._body_v, self._body_w = 0.0, 0.0
                return 0.0, 0.0
            pose = self.odometry.pose()
            v, w, arrived = pure_pursuit(pose, self.path_xy, arrive_m=ARRIVE_M)
            if arrived:
                self._body_v, self._body_w = 0.0, 0.0
                return 0.0, 0.0
        view = self._bubble_view
        if view is None and self._bubble_gap is not None:
            view = BubbleView(self._bubble_gap, None, None, None, self._min_gap)
        choice = choose_motion(
            view=view,
            pursuit_v=v,
            pursuit_w=w,
            heading_err=wrap_rad(self._segment_heading() - pose[2]),
            grid=self.grid,
            pose=pose,
            left_clear=float(left_clear),
            right_clear=float(right_clear),
            radar_blocked=self._radar_blocked,
        )
        self._note_motion(choice)
        self._body_v, self._body_w = choice.v, choice.w
        return wheel_speeds(choice.v, choice.w)

    def sweep_summary(self) -> str:
        return (
            f"checks={self.sweep_checks} rejected={self.sweep_rejected} "
            f"wide={self.sweep_widened} align={self.sweep_aligned} "
            f"straight={self.sweep_straight} stop={self.sweep_stopped}"
        )

    def _log_goal(self, goal: tuple[float, float], report: dict) -> None:
        runner = report.get("runner_up")
        runner_score = report.get("runner_score")
        log_decision(
            {
                "t": round(self.t_s, 2),
                "kind": "goal",
                "chosen": [round(goal[0], 2), round(goal[1], 2)],
                "score": round(float(report.get("score") or 0.0), 3),
                "runner_up": None if runner is None else [round(runner[0], 2), round(runner[1], 2)],
                "runner_score": None if runner_score is None else round(float(runner_score), 3),
            }
        )
        self._log(f"goal ({goal[0]:.2f},{goal[1]:.2f})")

    def _plan(self, depth: int = 0) -> None:
        self._replan_at = self.t_s + 4.0
        x, y, yaw = self.odometry.pose()
        if depth > 6:
            self._no_frontier()
            return
        rooms = segment_map(self.grid.labels(), self.grid.res)
        self.room_labels = rooms["labels"]
        self.room_names = rooms["names"]
        self.room_doors = rooms["doors"]
        report: dict = {}
        chosen = select_goal(
            self.grid,
            (x, y),
            radius_m=ROBOT_RADIUS_M,
            avoid=self._avoid_now(),
            rooms=rooms,
            heading_xy=(math.cos(yaw), math.sin(yaw)),
            report=report,
        )
        if chosen is None:
            self._no_frontier()
            return
        goal, path = chosen
        if math.hypot(goal[0] - x, goal[1] - y) < ARRIVE_M:
            self._remember_attempt(goal[0], goal[1], 1.5)
            if self.t_s - self._last_spin_t > 12.0:
                self._last_spin_t = self.t_s
                self.phase = "spin"
                self._spin_start = None
                self.goal_xy = None
                self.path_xy = []
                self._log_goal(goal, report)
                return
            self._plan(depth + 1)
            return
        if (
            self.goal_xy is not None
            and self.path_xy
            and math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1]) < 1.2
        ):
            return
        same = (
            self.goal_xy is not None
            and math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1]) < 0.2
        )
        self.goal_xy, self.path_xy = goal, path
        self.frontiers_left = 1
        self.phase = "go"
        self.done = False
        if not same:
            self._goal_since = self.t_s
            self._motion_anchor = None
            self._log_goal(goal, report)

    def _no_frontier(self) -> None:
        """No reachable frontier. Done only after a scan and a stopped hold."""
        if not self._confident():
            self._log("frontiers hidden, pose not trusted, not done")
            self.goal_xy = None
            self.path_xy = []
            self.phase = "go"
            self.done = False
            self._did_final_scan = False
            self._replan_at = self.t_s + 1.5
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
        rooms = segment_map(self.grid.labels(), self.grid.res)
        self.room_labels = rooms["labels"]
        self.room_names = rooms["names"]
        self.room_doors = rooms["doors"]
        chosen = select_goal(
            self.grid,
            (x, y),
            radius_m=ROBOT_RADIUS_M,
            avoid=self._avoid_now(),
            rooms=rooms,
            heading_xy=(math.cos(yaw), math.sin(yaw)),
        )
        if chosen is not None:
            self._log("frontier open, resuming")
            self._did_final_scan = False
            self.phase = "go"
            goal, path = chosen
            self.goal_xy, self.path_xy = goal, path
            self._goal_since = self.t_s
            self._motion_anchor = None
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
