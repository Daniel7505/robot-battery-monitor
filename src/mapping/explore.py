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
from collections import deque

import numpy as np

from src.mapping.body import (
    ARRIVE_M,
    BLIND_CLEARANCE_M,
    BLIND_VIEW_FRACTION,
    CRUISE_M_S,
    OAK_DEPTH,
    OAK_FIRM_RANGE_M,
    OAK_SOFT_RANGE_M,
    ROBOT_RADIUS_M,
    SPIN_RAD_S,
    REAR_DEPTH,
    STOP_CLEARANCE_M,
    TURN_AIM_M,
    TURN_CLEAR_M,
    WHEEL_RADIUS_M,
    WHEEL_TRACK_M,
    DepthCamera,
)
from src.mapping.bubble import SIGHT_M, SHELL_HALF_M, BubbleView, assess_bubble, corner_bump, corner_gap
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
from src.mapping.footprint import footprint_clips
from src.mapping.frontiers import (
    _pose_clearance,
    approach_command,
    free_behind_m,
    frontier_mask,
    select_goal,
    spin_fits,
    wide_escape,
)
from src.mapping.tick import (
    TURN_W_RAD_S,
    _ahead_blocked,
    _behind_blocked,
    choose_motion,
    live_veto,
    log_major,
)
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
# Commanded motion whose own depth picture does not change. Two to three
# seconds: long enough that a slow creep moves the ranges, short enough
# that a wedge does not sit for a minute.
_VIEW_STUCK_S = 2.5
_VIEW_RANGE_EPS_M = 0.05
_VIEW_FRAC_EPS = 0.08
# Wheels turning, gyro not. Or wheel speed changing with no accelerometer
# pulse. Same window as the camera check.
_IMU_STUCK_S = 2.5
_IMU_WHEEL_W_MIN = 0.18
_IMU_GYRO_W_MAX = 0.08
_IMU_DV_MIN_M_S = 0.08
_IMU_DV_WINDOW_S = 0.40
_IMU_ACCEL_MAX_M_S2 = 0.20
# After an escape, do not drive back along the same heading while the
# shell is still on that spot. Reverse and a turn away stay legal.
_ESCAPE_BLOCK_S = 18.0
_ESCAPE_BLOCK_M = 0.70
_ESCAPE_BLOCK_YAW = 0.9

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
# Side face plus 2 cm. A creep through a throat the camera and the map
# both see uses this instead of the 0.30 m pivot aim. Cruise still uses
# the full aim, so a fast arc does not take the tighter line.
CREEP_CLEAR_M = SHELL_HALF_M + 0.02
# Leave a pivot once the nose is this close to the path. 12° puts the
# 0.19 m nose 4 cm off the axis, inside one grid cell, so a centred
# crossing still clears the jamb. 15° is a full cell and clips it.
PIVOT_EXIT_RAD = math.radians(12.0)
# Do not start another pivot until the error is past one cell of path
# wobble (about 10° at the 0.28 m turn) on top of that exit. A real
# corner in this planner is above 57°.
PIVOT_ENTER_RAD = math.radians(25.0)
# Same patch of frontier. One bump still fades through the attempt
# weight. The third failure on that patch stays banned until the camera
# sees the unknown beside it or a nearby wall disappears.
FRONTIER_FAILS = 3
FRONTIER_REGION_M = 0.80
FRONTIER_PATCH_M = 1.00
# Occupied cells in the local window. Smaller than a whole-map flicker.
MAP_CHANGE_CELLS = 12
# Past the depth camera's minimum. A return on the minimum itself is still
# the blind edge; the backup keeps going until the wall is this far outside it.
BLIND_EXIT_MARGIN_M = 0.05
# The same goal region is not scored again until the shell has left the stuck pose.
GOAL_HOLD_M = 0.30
# Commanded translation that does not cover that distance in this long is a
# stall, blind or not. It is far under the 60 s a seed is allowed to sit
# inside 0.45 m.
STALL_WINDOW_S = 2.0
PILE_LIMIT_S = 60.0
# Straight reverse out of a blind corner. The 0.45 s relief reverse is shorter
# than the blind zone, so it is not used for this case. 1 m matches the
# ordinary backup cap so a dead camera cannot reverse across the apartment.
BLIND_BACK_CAP_M = 1.0
STILL_M = 0.04
STILL_S = 0.50


def _split_ranges(
    hits: np.ndarray, sensor: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Firm hits, lower-confidence hits, and probable hits.

    Firm is inside ``OAK_FIRM_RANGE_M`` and can occupy at the normal odds.
    Soft is out to ``OAK_SOFT_RANGE_M`` and occupies only after repeated
    views. Farther than that is probable: free space along the ray, and
    the end cell never blocks a path.
    """
    empty = np.zeros((0, 3), dtype=np.float64)
    if hits.size == 0:
        return empty, empty, empty
    origin = np.asarray(sensor, dtype=np.float64).reshape(1, 3)
    delta = np.asarray(hits, dtype=np.float64) - origin
    dist = np.sqrt(delta[:, 0] ** 2 + delta[:, 1] ** 2 + delta[:, 2] ** 2)
    firm = dist <= OAK_FIRM_RANGE_M
    soft = (dist > OAK_FIRM_RANGE_M) & (dist <= OAK_SOFT_RANGE_M)
    probable = dist > OAK_SOFT_RANGE_M
    return hits[firm], hits[soft], hits[probable]


class _FrontierRegion:
    """One failed frontier patch and the unknown cells that would clear it."""

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y
        self.fails = 0
        self.cells: set[tuple[int, int]] = set()
        self.watch: set[tuple[int, int]] = set()
        self.banned = False
        self.logged = False
        self.occ = 0


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
        self._approach_why = ""
        self._approach_log_at = 0.0
        self._motion_log_at = -1.0
        self._commit: tuple[float, float] | None = None
        self._commit_at = -1.0
        self._commit_until = -1.0
        self._backing = False
        self._back_from: tuple[float, float] | None = None
        self._recover = False
        self._recover_from: tuple[float, float] | None = None
        self._recover_need = 0.0
        self._recover_phase = ""
        self._recover_until = -1.0
        self._recover_t0 = -1.0
        self._reverse_failed = False
        self._contact_until = -1.0
        self._proximity: dict = {}
        self._refused: set[tuple] = set()
        self._goal_score = 0.0
        self._goal_cells: list[tuple[int, int]] = []
        self._regions: list[_FrontierRegion] = []
        self._held: set[tuple[int, int]] = set()
        self._prefer_near = False
        self._pivot_hold = False
        self._still_from: tuple[float, float, float] | None = None
        self._blind_back = False
        self._blind_back_from: tuple[float, float] | None = None
        self._blind_back_t0 = -1.0
        self._blind_back_until = -1.0
        self._blind_counted = False
        self._escape_phase = "back"
        self._escape_phase_t = -1.0
        self._escape_yaw = 0.0
        self._escape_spin = 0.70
        # t, x, y of the last time the shell was 0.45 m from here. The pile
        # guard trips at PILE_LIMIT_S and stays tripped until that move.
        self._pile_anchor: tuple[float, float, float] | None = None
        self._pile_forced = False
        self._pile_logged = False
        # Recent odom poses the shell actually drove. The pile leave reverses
        # along these when a straight sweep is closed. Not a map of the rooms.
        self._crumbs: list[tuple[float, float]] = []
        # Depth signature and the time it last changed. The stuck check
        # uses this picture, not the odometry pose.
        self._view_sig: tuple | None = None
        self._view_still_from: float | None = None
        # (t, |wheel yaw rate|, |gyro yaw rate|, wheel speed, horiz accel or None)
        self._imu_samples: deque = deque()
        self._accel_bias = [0.0, 0.0]
        # until, x, y, yaw. The heading that was stuck. Forward along it
        # is refused until the cooldown or until the shell has left.
        self._escape_block: tuple[float, float, float, float] | None = None
        self._block_log_at = -1.0
        self._block_refusals = 0
        self._last_escape_xy: tuple[float, float] | None = None
        self._escape_repeats = 0
        # Back out to here, then take a different goal. The old spot is
        # low priority until the avoid weight fades.
        self._leave_xy: tuple[float, float] | None = None
        self._quarantine: tuple[float, float, float] | None = None
        self._narrow_logged = False
        # t, x, y, commanded translation while still. Resets once the shell moves.
        self._progress_from: tuple[float, float, float, bool] | None = None
        # gx, gy, x0, y0, frontier cells. Held until the shell moves GOAL_HOLD_M.
        self._goal_hold: tuple[float, float, float, float, list] | None = None
        self._pocket = False
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
        accel_m_s2: tuple[float, float, float] | None = None,
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
        soft = probable = None
        if depth is not None:
            hits, floors, sensor = body_points(
                self.camera,
                depth,
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                stride=self.stride,
            )
            # Stopping uses the firm band. A 4–5 m smear must not look like
            # a wall in the stop distance, and a return past 5 m must not
            # either. Both still update the map, at lower confidence.
            firm, soft, probable = _split_ranges(hits, sensor)
            hits = firm
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
        if depth is not None:
            self._note_view(depth)
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
        # A visible picture that has not moved is not travel, including a
        # reverse the hubs followed. A blind reverse still counts: the
        # range comes back once the nose clears minimum range, and that
        # distance is how the pose leaves the jamb.
        if self._view_frozen() and not self._view_blind and abs(ds) > 1e-4:
            drop = True
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
        self._note_imu(dt, yaw_rate_rad_s, accel_m_s2)
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
        self._proximity = proximity or {}
        view = assess_bubble(clearance, blind=self._view_blind, proximity=self._proximity)
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

            def _xy(cloud) -> np.ndarray:
                if cloud is not None and getattr(cloud, "size", 0):
                    return to_odom(cloud, (x, y, yaw))
                return np.zeros((0, 2))

            self.grid.integrate(
                origin,
                to_odom(hits, (x, y, yaw)),
                to_odom(floors, (x, y, yaw)),
                probable_xy=_xy(probable),
                soft_xy=_xy(soft),
            )
        self._last_clear = (clearance, left_clear, right_clear)
        self._note_translation()
        self._note_progress()
        self._note_pile()
        self._note_crumb()
        self._watch_motion()
        # Spin, settle, and go all command motion. A wedge in the survey
        # spin never reached this check when it lived only in _go.
        if not self.done:
            stuck = self._stuck_escape(self.odometry.pose())
            if stuck is not None:
                return self._remember(stuck)
            leaving = self._leaving_command(self.odometry.pose())
            if leaving is not None:
                return self._remember(leaving)
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
            self._goal_cells = []
            if then == "settle":
                self._log("final scan finished")
                self._plan()
                if self.phase != "go":
                    return 0.0, 0.0
            return self._go(False, 99.0)
        return wheel_speeds(0.0, SPIN_RAD_S)

    def _remember(self, speeds: tuple[float, float]) -> tuple[float, float]:
        speeds = self._respect_escape_block(speeds)
        self._commanded = (float(speeds[0]), float(speeds[1]))
        self._mark_xy = self.odometry.pose()[:2]
        return speeds

    def _arm_escape_block(self) -> None:
        """Remember this spot and heading so the next command cannot drive back in."""
        x, y, yaw = self.odometry.pose()
        if self._escape_block is not None:
            until, x0, y0, _yaw0 = self._escape_block
            if self.t_s < until and math.hypot(x - x0, y - y0) < _ESCAPE_BLOCK_M:
                return
        if self._last_escape_xy is not None and math.hypot(x - self._last_escape_xy[0], y - self._last_escape_xy[1]) < 0.45:
            self._escape_repeats += 1
        else:
            self._escape_repeats = 1
            self._block_refusals = 0
        self._last_escape_xy = (x, y)
        self._escape_block = (self.t_s + _ESCAPE_BLOCK_S, x, y, yaw)
        self._remember_attempt(x, y, 4.0)
        if self._escape_repeats >= 2:
            self._begin_pile_leave((x, y, yaw))
        if self.goal_xy is not None:
            cells = list(self._goal_cells)
            self._goal_hold = (float(self.goal_xy[0]), float(self.goal_xy[1]), x, y, cells)
            self.goal_xy = None
            self._goal_cells = []
            self.path_xy = []
        if self.phase == "spin":
            self._spin_start = None
            self._spin_then = None
            self.phase = "go"
            self._replan_at = self.t_s

    def _respect_escape_block(self, speeds: tuple[float, float]) -> tuple[float, float]:
        """A forward command along the stuck heading becomes a reverse."""
        block = self._escape_block
        if block is None:
            return speeds
        until, x0, y0, yaw0 = block
        x, y, yaw = self.odometry.pose()
        if self.t_s >= until or math.hypot(x - x0, y - y0) >= _ESCAPE_BLOCK_M:
            self._escape_block = None
            return speeds
        left, right = float(speeds[0]), float(speeds[1])
        v = 0.5 * (left + right) * WHEEL_RADIUS_M
        if v <= 0.02 or abs(wrap_rad(yaw - yaw0)) > _ESCAPE_BLOCK_YAW:
            return speeds
        if self.t_s >= self._block_log_at:
            self._block_log_at = self.t_s + 2.0
            self._log("blocked heading, not driving back in")
        self._block_refusals += 1
        if self._block_refusals >= 4:
            self._begin_pile_leave(self.odometry.pose())
        self._body_v, self._body_w = -0.10, 0.0
        self._note_approach("back", -0.10, 0.0)
        return wheel_speeds(-0.10, 0.0)

    def _begin_pile_leave(self, pose) -> None:
        """The same spot again. Reverse along the trail and drop that goal."""
        if self._leave_xy is not None:
            return
        x, y = float(pose[0]), float(pose[1])
        self._leave_xy = (x, y)
        self._quarantine = (x, y, self.t_s)
        if self.goal_xy is not None:
            self._remember_attempt(float(self.goal_xy[0]), float(self.goal_xy[1]), 6.0)
        self.goal_xy = None
        self._goal_cells = []
        self.path_xy = []
        self._escape_block = None
        self._block_refusals = 0
        self._major("leave", "pile_leave")
        self._log("pile again, leaving for another goal")

    def _leaving_command(self, pose) -> tuple[float, float] | None:
        """Reverse along the trail until the shell is out of the quarantined spot."""
        if self._leave_xy is None:
            return None
        if math.hypot(float(pose[0]) - self._leave_xy[0], float(pose[1]) - self._leave_xy[1]) >= 0.70:
            self._leave_xy = None
            self._escape_repeats = 0
            self._replan_at = self.t_s
            return None
        along = self._trail_reverse(pose, trust_trail=True)
        if along is not None and along[0] < -0.2 and along[1] < -0.2:
            return along
        if self._rear_measured_close():
            return None
        self._body_v, self._body_w = -0.10, 0.0
        self._note_approach("back", -0.10, 0.0)
        return wheel_speeds(-0.10, 0.0)

    def _narrow_gap(self, pose) -> bool:
        """A throat this robot can see, tighter than the pivot aim and wider than the body.

        Both the map and a live shoulder range have to agree. An open hall
        (both shoulders at the no-return sentinel) is not a doorway. Ground
        truth is not read.
        """
        _nose, left, right = self._last_clear
        if float(left) >= 90.0 and float(right) >= 90.0:
            return False
        if not any(0.15 <= float(side) < 90.0 for side in (left, right)):
            return False
        x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
        for dist in (0.0, 0.35):
            clearance = _pose_clearance(
                self.grid,
                x + dist * math.cos(yaw),
                y + dist * math.sin(yaw),
            )
            if CREEP_CLEAR_M <= clearance < TURN_AIM_M:
                return True
        return False

    def _note_narrow_gap(self) -> None:
        if self._narrow_logged:
            return
        self._narrow_logged = True
        self._major("creep", "narrow_gap", clearance_m=_pose_clearance(self.grid, *self.odometry.pose()[:2]))
        self._log("narrow gap, creeping")

    def _creep_through_gap(self, pose, v: float, w: float, heading_err: float) -> tuple[float, float] | None:
        """Full buffer at cruise. Through a seen throat, creep at the body margin."""
        if v <= 0.02 or abs(heading_err) > 0.50 or not self._narrow_gap(pose):
            self._narrow_logged = False
            return None
        creep_v = min(float(v), DOOR_SPEED_M_S)
        creep_w = max(-0.20, min(0.20, float(w)))
        if self._shell_clips(creep_v, creep_w):
            return None
        self._note_narrow_gap()
        self._body_v, self._body_w = creep_v, creep_w
        self._note_approach("straight", creep_v, creep_w)
        return wheel_speeds(creep_v, creep_w)

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
        # No casters and no carpet ramp. A pitch past 0.12 rad while the
        # hubs claim motion means the shell tipped, not a wheel unload.
        pitched = rest is not None and abs(float(pitch) - rest) > 0.12
        if pitched and (claimed >= 0.02 or spun >= 0.08) and not anchor["lifted"]:
            anchor["lifted"] = True
            self.wheels_lifted += 1
            self._log("wheels lifted, shell pitched, drive wheels spinning free")
        # The floor picture did not shift while the hubs claimed travel.
        # On the hall pile the shell's true pose was still, so this rewind
        # matched the body. A blank, low-texture frame takes the branch
        # above and does not freeze. Leaving is the stall escape, which
        # keeps backing or turns to a clear heading instead of scoring
        # another goal on the frozen pose.
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

    def _note_progress(self) -> None:
        """Clock commanded translation that the odometry did not follow."""
        x, y, _yaw = self.odometry.pose()
        commanded = abs(self._body_v) > 0.03
        if self._progress_from is None:
            self._progress_from = (self.t_s, x, y, commanded)
            return
        t0, x0, y0, saw = self._progress_from
        if math.hypot(x - x0, y - y0) >= 0.04:
            self._progress_from = (self.t_s, x, y, False)
            return
        self._progress_from = (t0, x0, y0, saw or commanded)

    def _translation_stalled(self) -> bool:
        """Commands asked for translation and the shell has not covered 0.30 m."""
        if self._progress_from is None or self._pivoting() or self._doorway_crossing():
            return False
        # Lining up on an opening looks like a stall. Dropping the goal there
        # is how a kitchen or bath target was abandoned on the threshold.
        if self._approach_why in ("straight", "pivot"):
            return False
        t0, x0, y0, saw = self._progress_from
        if not saw or self.t_s - t0 < STALL_WINDOW_S:
            return False
        x, y, _yaw = self.odometry.pose()
        return math.hypot(x - x0, y - y0) < GOAL_HOLD_M

    def _note_translation(self) -> None:
        """Remember the last time the odometry actually moved."""
        x, y, _yaw = self.odometry.pose()
        if self._still_from is None:
            self._still_from = (self.t_s, x, y)
            return
        _t0, x0, y0 = self._still_from
        if math.hypot(x - x0, y - y0) >= STILL_M:
            self._still_from = (self.t_s, x, y)

    def _not_translating(self) -> bool:
        if self._still_from is None:
            return False
        t0, x0, y0 = self._still_from
        x, y, _yaw = self.odometry.pose()
        return (self.t_s - t0) >= STILL_S and math.hypot(x - x0, y - y0) < STILL_M

    def _pivoting(self) -> bool:
        return abs(self._body_v) < 0.02 and abs(self._body_w) > 0.2

    def _doorway_crossing(self) -> bool:
        """A lined-up drive through a mapped doorway. A sit in the opening is not one.

        The place tag alone used to skip every stuck event in a doorway,
        including a robot that was not crossing. The straight approach and
        a forward command are the crossing. An open-floor pivot is separate.
        """
        if self._place_tag() != "doorway":
            return False
        if self._approach_why == "straight":
            return True
        return self._body_v > 0.03

    def _blind_room_still(self) -> bool:
        """Blind, tagged room, and the shell is not translating. A pivot is not this."""
        if not self._view_blind or self._place_tag() != "room" or self._pivoting():
            return False
        return self._not_translating()

    def _depth_real(self) -> bool:
        """A forward return past min range, not the blind placeholder."""
        if self._view_blind:
            return False
        nose = float(self._last_clear[0])
        if not math.isfinite(nose):
            return False
        # No hit inside max range. The corridor is open, which is a real view.
        if nose >= 90.0:
            return True
        sensor = nose - float(self.camera.translation[0])
        return sensor >= float(self.camera.min_range_m) + BLIND_EXIT_MARGIN_M

    def _goal_released(self) -> bool:
        if self._goal_hold is None:
            return True
        x, y, _yaw = self.odometry.pose()
        if math.hypot(x - self._goal_hold[2], y - self._goal_hold[3]) >= GOAL_HOLD_M:
            self._goal_hold = None
            return True
        return False

    def _goal_on_hold(self, goal: tuple[float, float]) -> bool:
        if self._goal_released():
            return False
        assert self._goal_hold is not None
        gx, gy = self._goal_hold[0], self._goal_hold[1]
        return math.hypot(float(goal[0]) - gx, float(goal[1]) - gy) < FRONTIER_REGION_M

    def _held_cells(self) -> set[tuple[int, int]]:
        if self._goal_released() or self._goal_hold is None:
            return set()
        cells = list(self._goal_hold[4])
        if cells:
            return set(cells)
        return self._patch_cells(self._goal_hold[0], self._goal_hold[1], None)

    def _end_blind_back(self, cooldown: float) -> None:
        self._blind_back = False
        self._blind_back_from = None
        self._blind_counted = False
        self._blind_back_until = self.t_s + cooldown

    def _begin_blind_back(self, pose) -> None:
        x, y = float(pose[0]), float(pose[1])
        self._blind_back = True
        self._blind_back_from = (x, y)
        self._blind_back_t0 = self.t_s
        self._backing = False
        self._commit = None
        self._pivot_hold = False
        if self.goal_xy is not None and not self._blind_counted:
            cells = list(self._goal_cells)
            self._goal_hold = (float(self.goal_xy[0]), float(self.goal_xy[1]), x, y, cells)
            self._remember_attempt(self.goal_xy[0], self.goal_xy[1], 2.2)
            if self._frontier_fail_ok():
                self._note_frontier_fail(self.goal_xy[0], self.goal_xy[1], cells)
            self._blind_counted = True
        self.goal_xy = None
        self._goal_cells = []
        self.path_xy = []
        self._pocket = False
        if self._view_blind:
            self._log("blind corner, backing out")
        else:
            self._log("stall, backing out")
        self._major("back", "cant_see_behind_backing_out")

    def _note_pile(self) -> None:
        """Arm the hard stop once the shell has sat inside 0.45 m for 60 s."""
        x, y, _yaw = self.odometry.pose()
        if self._pile_anchor is None:
            self._pile_anchor = (self.t_s, x, y)
            return
        t0, x0, y0 = self._pile_anchor
        if math.hypot(x - x0, y - y0) > 0.45:
            self._pile_anchor = (self.t_s, x, y)
            self._pile_forced = False
            self._pile_logged = False
            return
        if self.t_s - t0 < PILE_LIMIT_S:
            return
        self._pile_forced = True
        if self._pile_logged:
            return
        self._pile_logged = True
        self._log("pile, leaving the spot")
        self._major("back", "cant_see_behind_backing_out")

    def _motion_commanded(self) -> bool:
        if abs(self._body_v) >= 0.03 or abs(self._body_w) >= 0.15:
            return True
        cmd_l, cmd_r = self._commanded
        v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
        w = (cmd_r - cmd_l) * WHEEL_RADIUS_M / WHEEL_TRACK_M
        return abs(v) >= 0.03 or abs(w) >= 0.15

    def _depth_signature(self, depth) -> tuple:
        """Median range, finite fraction, and a coarse block grid.

        An empty frame (no return, or everything inside minimum range) is
        a stable signature. That is the nose-99 spin as well as the
        blind placeholder. Ground truth is not read.
        """
        image = np.asarray(depth, dtype=np.float64)
        if image.ndim != 2 or image.size == 0:
            return (None, 0.0, ())
        sampled = image[::4, ::4]
        lo = self.camera.min_range_m
        hi = self.camera.max_range_m
        finite = np.isfinite(sampled) & (sampled >= lo) & (sampled <= hi)
        count = int(sampled.size)
        frac = 0.0 if count == 0 else float(np.count_nonzero(finite) / float(count))
        med = None if frac < 0.08 else float(np.median(sampled[finite]))
        height, width = sampled.shape
        blocks: list[float | None] = []
        for br in range(3):
            for bc in range(4):
                r0, r1 = br * height // 3, max(br * height // 3 + 1, (br + 1) * height // 3)
                c0, c1 = bc * width // 4, max(bc * width // 4 + 1, (bc + 1) * width // 4)
                tile = sampled[r0:r1, c0:c1]
                ok = np.isfinite(tile) & (tile >= lo) & (tile <= hi)
                if int(np.count_nonzero(ok)) < 3:
                    blocks.append(None)
                else:
                    blocks.append(float(np.median(tile[ok])))
        return (med, frac, tuple(blocks))

    def _signature_changed(self, previous: tuple, current: tuple) -> bool:
        med_a, frac_a, blocks_a = previous
        med_b, frac_b, blocks_b = current
        if abs(float(frac_a) - float(frac_b)) >= _VIEW_FRAC_EPS:
            return True
        if med_a is None and med_b is None:
            return False
        if med_a is None or med_b is None:
            return True
        if abs(float(med_a) - float(med_b)) >= _VIEW_RANGE_EPS_M:
            return True
        diffs = []
        for left, right in zip(blocks_a, blocks_b):
            if left is None and right is None:
                continue
            if left is None or right is None:
                diffs.append(0.2)
                continue
            diffs.append(abs(float(left) - float(right)))
        if diffs and float(np.median(diffs)) >= _VIEW_RANGE_EPS_M:
            return True
        return False

    def _note_view(self, depth) -> None:
        sig = self._depth_signature(depth)
        if self._view_sig is None or self._signature_changed(self._view_sig, sig):
            self._view_sig = sig
            self._view_still_from = self.t_s
            return
        if self._view_still_from is None:
            self._view_still_from = self.t_s

    def _view_frozen(self) -> bool:
        if self._view_sig is None or self._view_still_from is None:
            return False
        return self.t_s - self._view_still_from >= _VIEW_STUCK_S

    def _camera_stuck(self) -> bool:
        """Motion is commanded and this robot's depth picture is not changing.

        Odometry yaw is not a vote. A spin the gyro thinks is happening
        still counts when the picture stays put.
        """
        if not self._view_frozen():
            return False
        if not self._motion_commanded():
            self._view_still_from = self.t_s
            return False
        return True

    def _note_imu(self, dt: float, yaw_rate_rad_s: float, accel) -> None:
        if dt <= 0.0:
            return
        wheel_w = self.odometry.last_wheel_dyaw_rad / dt
        wheel_v = self.odometry.last_ds_m / dt
        gyro_w = float(yaw_rate_rad_s) - self.odometry.bias_rad_s
        horiz = None
        if accel is not None and len(accel) >= 2:
            ax = float(accel[0])
            ay = float(accel[1])
            if abs(wheel_v) < 0.02 and abs(wheel_w) < 0.05:
                self._accel_bias[0] = 0.98 * self._accel_bias[0] + 0.02 * ax
                self._accel_bias[1] = 0.98 * self._accel_bias[1] + 0.02 * ay
            horiz = math.hypot(ax - self._accel_bias[0], ay - self._accel_bias[1])
        self._imu_samples.append((self.t_s, wheel_w, gyro_w, wheel_v, horiz))
        while self._imu_samples and self.t_s - self._imu_samples[0][0] > _IMU_STUCK_S:
            self._imu_samples.popleft()

    def _imu_stuck(self) -> bool:
        """Encoders or the command say the wheels are moving; the IMU does not.

        A pure spin is the gyro. A change in wheel speed is the
        accelerometer. Constant-speed slip has no acceleration to see;
        the camera check covers that. Ground truth is not read.
        """
        samples = self._imu_samples
        if len(samples) < 2:
            return False
        span = samples[-1][0] - samples[0][0]
        if span < _IMU_STUCK_S - 0.3:
            return False
        wheel_w = sum(abs(row[1]) for row in samples) / len(samples)
        gyro_w = sum(abs(row[2]) for row in samples) / len(samples)
        if wheel_w >= _IMU_WHEEL_W_MIN and gyro_w < _IMU_GYRO_W_MAX:
            return True
        if any(row[4] is None for row in samples):
            return False
        peak = max(float(row[4]) for row in samples)
        max_dv = 0.0
        start = 0
        for i, row in enumerate(samples):
            while samples[start][0] < row[0] - _IMU_DV_WINDOW_S:
                start += 1
            max_dv = max(max_dv, abs(row[3] - samples[start][3]))
        return max_dv >= _IMU_DV_MIN_M_S and peak < _IMU_ACCEL_MAX_M_S2

    def _stuck_reason(self) -> str | None:
        camera = self._camera_stuck()
        imu = self._imu_stuck()
        if camera and imu:
            return "camera_imu_stuck"
        if camera:
            return "camera_stuck"
        if imu:
            return "imu_stuck"
        return None

    def _stuck_escape(self, pose) -> tuple[float, float] | None:
        """Reverse along the trail. Do not spin in place: that was the wedge."""
        why = self._stuck_reason()
        if why is None:
            return None
        self._arm_escape_block()
        self._major("stuck", why)
        self._relief = None
        self._blind_back = False
        self._pivot_hold = False
        if not self._rear_measured_close():
            self._body_v, self._body_w = -0.10, 0.0
            self._note_approach("back", -0.10, 0.0)
            return wheel_speeds(-0.10, 0.0)
        along = self._trail_reverse(pose, trust_trail=True)
        if along is not None and along[0] < -0.2 and along[1] < -0.2:
            return along
        if (
            not self._view_blind
            and float(self._last_clear[0]) >= 0.50
            and not self._shell_clips(0.10, 0.0)
        ):
            self._body_v, self._body_w = 0.10, 0.0
            self._note_approach("wide", 0.10, 0.0)
            return wheel_speeds(0.10, 0.0)
        self._body_v, self._body_w = -0.10, 0.0
        self._note_approach("back", -0.10, 0.0)
        return wheel_speeds(-0.10, 0.0)

    def _rear_measured_close(self) -> bool:
        view = self._bubble_view
        return view is not None and view.rear is not None and float(view.rear) <= 0.08

    def _note_crumb(self) -> None:
        x, y, _yaw = self.odometry.pose()
        if self._crumbs and math.hypot(x - self._crumbs[-1][0], y - self._crumbs[-1][1]) < 0.25:
            return
        self._crumbs.append((float(x), float(y)))
        if len(self._crumbs) > 16:
            del self._crumbs[0]

    def _trail_reverse(self, pose, trust_trail: bool = False) -> tuple[float, float] | None:
        """Reverse toward a pose the shell already drove, if the rear sensor is open.

        The map sweep can close both straights while the trail behind is
        still clear. ``trust_trail`` ignores that map smear and still
        refuses a measured rear hit. Reversing into an unseen gap is not this.
        """
        if trust_trail:
            if self._rear_measured_close():
                return None
        elif self._reverse_closed(pose):
            return None
        x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
        back = yaw + math.pi
        best_err = None
        for cx, cy in self._crumbs[:-1]:
            if math.hypot(cx - x, cy - y) < 0.35:
                continue
            err = abs(wrap_rad(math.atan2(cy - y, cx - x) - back))
            if best_err is None or err < best_err:
                best_err = err
        # A crumb off to the side is not a reverse. Turn toward it only when
        # the corner sweep allows, then the next call lines the tail up.
        if best_err is not None and best_err > 0.6:
            turn = math.copysign(0.50, wrap_rad(math.atan2(
                self._crumbs[-2][1] - y, self._crumbs[-2][0] - x
            ) - back)) if len(self._crumbs) >= 2 else 0.50
            if self._spin_safe(turn):
                self._body_v, self._body_w = 0.0, turn
                self._note_approach("pivot", 0.0, turn)
                return wheel_speeds(0.0, turn)
            return None
        self._body_v, self._body_w = -0.10, 0.0
        self._note_approach("back", -0.10, 0.0)
        return wheel_speeds(-0.10, 0.0)

    def _spin_safe(self, w: float) -> bool:
        """A pivot whose corner stays out of the map and the live side gap."""
        if self._shell_clips(0.0, w):
            return False
        view = self._bubble_view
        if view is None:
            return True
        side = view.left if w > 0 else view.right
        return side is None or float(side) >= TURN_CLEAR_M

    def _straight_leave(self, pose) -> tuple[float, float] | None:
        """Creep forward or back. No spin. None when both ways meet the shell."""
        view = self._bubble_view
        nose = float(self._last_clear[0])
        forward_gap = None if view is None else view.forward
        rear_gap = None if view is None else view.rear
        forward_ok = (
            not self._view_blind
            and nose >= 0.50
            and not self._shell_clips(0.12, 0.0)
            and (forward_gap is None or float(forward_gap) >= 0.30)
        )
        back_ok = not self._reverse_closed(pose) and not self._shell_clips(-0.10, 0.0)
        prefer_forward = forward_ok and (
            not back_ok or rear_gap is None or forward_gap is None or float(forward_gap) >= float(rear_gap)
        )
        if prefer_forward:
            self._body_v, self._body_w = 0.12, 0.0
            self._note_approach("wide", 0.12, 0.0)
            return wheel_speeds(0.12, 0.0)
        if back_ok:
            self._body_v, self._body_w = -0.10, 0.0
            self._note_approach("back", -0.10, 0.0)
            return wheel_speeds(-0.10, 0.0)
        if forward_ok:
            self._body_v, self._body_w = 0.12, 0.0
            self._note_approach("wide", 0.12, 0.0)
            return wheel_speeds(0.12, 0.0)
        return None

    def _clear_heading(self) -> bool:
        """The forward camera has a real range, so a turn has found open floor."""
        if self._view_blind:
            return False
        nose = float(self._last_clear[0])
        return nose >= 90.0 or nose >= 0.50

    def _stall_escape(self, pose) -> tuple[float, float] | None:
        """Leave a stall by backing, then turning to a heading the camera can see.

        A blind room starts immediately. Any other commanded translation that
        has not moved the shell 0.30 m in STALL_WINDOW_S starts the same
        escape. The reverse stops on a tail contact, on 0.8 s with no motion,
        or at 1 m, and a turn follows. The goal region stays unscored until
        the shell has moved. A doorway crossing is left to the short relief.
        """
        if self._doorway_crossing() and not self._blind_back:
            return None
        if not self._blind_back:
            if self.t_s < self._blind_back_until:
                return None
            blind = self._blind_room_still()
            stalled = self._translation_stalled()
            if not blind and not stalled:
                return None
            if blind and not stalled and self._reverse_closed(pose):
                return None
            self._begin_blind_back(pose)
            # Back out first. If that does not move the shell, the turn below
            # takes over. Driving starts only after the yaw actually changes,
            # so a frozen picture cannot be integrated as travel.
            self._escape_phase = "turn" if self._reverse_closed(pose) else "back"
            self._escape_phase_t = self.t_s
            self._escape_yaw = float(pose[2])
            self._escape_spin = self._relief_spin()
        origin = self._blind_back_from
        dist = 0.0 if origin is None else math.hypot(float(pose[0]) - origin[0], float(pose[1]) - origin[1])
        # Blind still needs a real range. A visible stall only needs the move.
        seen = (not self._view_blind) or self._depth_real()
        if dist >= GOAL_HOLD_M and seen:
            self._end_blind_back(0.0)
            self._replan_at = self.t_s
            self._progress_from = (self.t_s, float(pose[0]), float(pose[1]), False)
            return None
        if self._escape_phase == "back":
            phase_age = self.t_s - self._escape_phase_t
            if (
                self._reverse_closed(pose)
                or dist >= BLIND_BACK_CAP_M
                or (phase_age > 0.8 and dist < 0.02)
            ):
                self._escape_phase = "turn"
                self._escape_phase_t = self.t_s
                self._escape_yaw = float(pose[2])
                self._escape_spin = self._relief_spin()
            else:
                self._body_v, self._body_w = -0.10, 0.0
                self._note_approach("back", -0.10, 0.0)
                return wheel_speeds(-0.10, 0.0)
        if self._escape_phase == "turn":
            turned = abs(wrap_rad(float(pose[2]) - self._escape_yaw))
            if not self._spin_safe(self._escape_spin):
                self._escape_spin = -float(self._escape_spin)
            # Both pivots meet a corner. Back straight instead of driving
            # the sweep into the wall.
            if not self._spin_safe(self._escape_spin):
                self._escape_phase = "back"
                self._escape_phase_t = self.t_s
                along = self._trail_reverse(pose)
                if along is not None:
                    return along
                if self._reverse_closed(pose):
                    turn = 0.40 if self._spin_safe(0.40) else -0.40
                    if not self._spin_safe(turn):
                        view = self._bubble_view
                        left = 0.0 if view is None or view.left is None else float(view.left)
                        right = 0.0 if view is None or view.right is None else float(view.right)
                        turn = 0.30 if left >= right else -0.30
                    self._body_v, self._body_w = 0.0, turn
                    return wheel_speeds(0.0, turn)
                self._body_v, self._body_w = -0.10, 0.0
                self._note_approach("back", -0.10, 0.0)
                return wheel_speeds(-0.10, 0.0)
            # A spin the gyro did not follow is slip. Keep turning instead of
            # driving the false wheel travel into the pose.
            yaw_stuck = self.t_s - self._escape_phase_t > 4.0 and turned < 0.2
            if yaw_stuck:
                self._escape_phase_t = self.t_s
                self._escape_yaw = float(pose[2])
                self._escape_spin = -float(self._escape_spin)
            elif (turned >= 0.8 and self._clear_heading()) or (
                not self._view_blind and self.t_s - self._escape_phase_t > 4.0
            ):
                self._escape_phase = "drive"
                self._escape_phase_t = self.t_s
            else:
                self._body_v, self._body_w = 0.0, float(self._escape_spin)
                self._note_approach("pivot", 0.0, float(self._escape_spin))
                return wheel_speeds(0.0, float(self._escape_spin))
        if self._view_blind:
            self._escape_phase = "back"
            self._escape_phase_t = self.t_s
            self._body_v, self._body_w = -0.10, 0.0
            self._note_approach("back", -0.10, 0.0)
            return wheel_speeds(-0.10, 0.0)
        if self.t_s - self._escape_phase_t > 0.8 and dist < 0.05:
            self._escape_phase = "turn"
            self._escape_phase_t = self.t_s
            self._escape_yaw = float(pose[2])
            self._escape_spin = -self._relief_spin()
            self._body_v, self._body_w = 0.0, float(self._escape_spin)
            return wheel_speeds(0.0, float(self._escape_spin))
        nose = float(self._last_clear[0])
        if not self._view_blind and nose < 0.40:
            self._escape_phase = "turn"
            self._escape_phase_t = self.t_s
            self._escape_yaw = float(pose[2])
            self._escape_spin = -self._relief_spin()
            self._body_v, self._body_w = 0.0, float(self._escape_spin)
            return wheel_speeds(0.0, float(self._escape_spin))
        self._body_v, self._body_w = 0.12, 0.0
        return wheel_speeds(0.12, 0.0)

    def _frontier_fail_ok(self) -> bool:
        """A stuck look counts. A doorway crossing and an open-floor pivot do not.

        A blind view used to skip every failure, and so did wheel slip. A
        room corner and a visible stall both count: the shell was commanded
        to move and did not. A lined-up doorway crossing still does not.
        The one-bump avoid weight still expires on its own.
        """
        if self._doorway_crossing():
            return False
        if self._blind_room_still() or self._translation_stalled():
            return True
        if self._slip_debt_m >= 0.15 or self._view_blind:
            return False
        if abs(self._body_v) < 0.02 and abs(self._body_w) > 0.2:
            x, y, _yaw = self.odometry.pose()
            if _pose_clearance(self.grid, x, y) >= TURN_AIM_M:
                return False
        return True

    def _ban_set(self) -> set[tuple[int, int]]:
        """Frontier cells on their third failure. One-bump weights are separate."""
        self._lift_regions()
        return set(self._held)

    def _note_frontier_fail(
        self,
        x: float,
        y: float,
        cells: list[tuple[int, int]] | None = None,
    ) -> None:
        """Count a failed approach. The third one bans that frontier patch."""
        region = None
        for item in self._regions:
            if math.hypot(item.x - x, item.y - y) <= FRONTIER_REGION_M:
                region = item
                break
        if region is None:
            region = _FrontierRegion(x, y)
            self._regions.append(region)
        if region.banned:
            return
        region.fails += 1
        region.x = float(x)
        region.y = float(y)
        if region.fails < FRONTIER_FAILS:
            return
        found = self._patch_cells(x, y, cells)
        if not found:
            cell = self.grid.cell_of(x, y)
            found = set() if cell is None else {cell}
        region.cells = found
        region.watch = self._unknown_beside(found)
        region.banned = True
        region.occ = self._occupied_near(x, y)
        self._held.update(found)
        if not region.logged:
            region.logged = True
            self._log("frontier_ban")
            self._major("frontier_ban", "frontier_ban")

    def _patch_cells(
        self,
        x: float,
        y: float,
        saved: list[tuple[int, int]] | None,
    ) -> set[tuple[int, int]]:
        """Frontier cells the failed goal was looking at, not the whole wall.

        The viewpoint stands 1–3 m back, so the patch is the bit of the
        chosen frontier nearest that goal, within ``FRONTIER_PATCH_M``.
        """
        pool = list(saved or [])
        if not pool:
            here = self.grid.cell_of(x, y)
            if here is None:
                return set()
            ys, xs = np.nonzero(frontier_mask(self.grid.labels()))
            if len(ys) == 0:
                return set()
            d2 = (ys - here[0]) ** 2 + (xs - here[1]) ** 2
            i = int(np.argmin(d2))
            ny, nx = int(ys[i]), int(xs[i])
            limit = (FRONTIER_PATCH_M / self.grid.res) ** 2
            return {
                (int(ys[j]), int(xs[j]))
                for j in range(len(ys))
                if (int(ys[j]) - ny) ** 2 + (int(xs[j]) - nx) ** 2 <= limit
            }
        here = self.grid.cell_of(x, y)
        if here is None:
            return set(pool)
        ny, nx = min(pool, key=lambda p: (p[0] - here[0]) ** 2 + (p[1] - here[1]) ** 2)
        limit = (FRONTIER_PATCH_M / self.grid.res) ** 2
        return {p for p in pool if (p[0] - ny) ** 2 + (p[1] - nx) ** 2 <= limit}

    def _unknown_beside(self, cells: set[tuple[int, int]]) -> set[tuple[int, int]]:
        labels = self.grid.labels()
        height, width = labels.shape
        found: set[tuple[int, int]] = set()
        for y, x in cells:
            for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if 0 <= ny < height and 0 <= nx < width and int(labels[ny, nx]) < 0:
                    found.add((ny, nx))
        return found

    def _occupied_near(self, x: float, y: float, radius_m: float = 1.2) -> int:
        here = self.grid.cell_of(x, y)
        if here is None:
            return 0
        labels = self.grid.labels()
        rc = int(math.ceil(radius_m / self.grid.res))
        r0 = max(0, here[0] - rc)
        r1 = min(labels.shape[0], here[0] + rc + 1)
        c0 = max(0, here[1] - rc)
        c1 = min(labels.shape[1], here[1] + rc + 1)
        window = labels[r0:r1, c0:c1]
        return int(np.count_nonzero(window == 1))

    def _lift_regions(self) -> None:
        """Drop a region ban once the camera has seen it, or the wall moved."""
        if not self._held and not any(item.banned for item in self._regions):
            return
        labels = self.grid.labels()
        alive: list[_FrontierRegion] = []
        for region in self._regions:
            if region.banned and self._region_resolved(region, labels):
                for cell in region.cells:
                    self._held.discard(cell)
                if region.logged:
                    self._log("frontier_clear")
                    self._major("frontier_clear", "frontier_clear")
                    region.logged = False
                continue
            alive.append(region)
        self._regions = alive

    def _region_resolved(self, region: _FrontierRegion, labels: np.ndarray) -> bool:
        if region.watch:
            known = 0
            height, width = labels.shape
            for y, x in region.watch:
                if 0 <= y < height and 0 <= x < width and int(labels[y, x]) != -1:
                    known += 1
            if known * 5 >= len(region.watch) * 3:
                return True
        # A wall that disappears (a smear clearing) is a new opening.
        # Fresh occupied cells are the camera doing its job, not a reason
        # to walk back into the same patch.
        if self._occupied_near(region.x, region.y) <= region.occ - MAP_CHANGE_CELLS:
            return True
        return False

    def _watch_motion(self) -> None:
        """Stuck and contact are the same fact: a command the odometry did not follow."""
        if self._blind_back:
            self._motion_anchor = None
            return
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
        self._backing = False
        self._commit = None
        self._major("stuck", "unknown", disp_m=moved)

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
            if (
                self._open_reverse()
                and not self._reverse_failed
                and not _behind_blocked(self.grid, self.odometry.pose())
            ):
                self._relief = "reverse"
                self._relief_until = self.t_s + 0.45
                self._major("stuck", "unknown")
            else:
                self._relief_w = self._relief_spin()
                self._relief_until = self.t_s + 0.45
                if self._shell_clips(0.0, self._relief_w):
                    self._relief = "stop"
                    self._major("stuck", "too_tight_to_fit")
                else:
                    self._reverse_failed = False
                    self._relief = "spin"
                    self._major("stuck", "heading_off", heading_err_deg=math.degrees(self._relief_w))
        if self.t_s <= self._relief_until:
            if self._relief == "reverse":
                self._body_v, self._body_w = -0.10, 0.0
                return wheel_speeds(-0.10, 0.0)
            if self._relief == "stop":
                self._body_v, self._body_w = 0.0, 0.0
                return 0.0, 0.0
            self._body_v, self._body_w = 0.0, self._relief_w
            return wheel_speeds(0.0, self._relief_w)
        self._finish_relief()
        return None

    def _finish_relief(self) -> None:
        x, y, _yaw = self.odometry.pose()
        if self.goal_xy is not None:
            self._remember_attempt(self.goal_xy[0], self.goal_xy[1], 2.2)
            # A doorway crossing and an open-floor pivot are the motion stack,
            # not a bad frontier. A blind room with no translation does count.
            # The attempt weight above still expires.
            if self._frontier_fail_ok():
                self._note_frontier_fail(self.goal_xy[0], self.goal_xy[1], self._goal_cells)
        else:
            self._remember_attempt(x, y, 2.2)
        self.goal_xy = None
        self._goal_cells = []
        self.path_xy = []
        self._replan_at = self.t_s
        self._prefer_near = True
        self._relief = None
        self._motion_anchor = None
        self._backing = False
        self._back_from = None
        self._commit = None
        self._pocket = False
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

    def _place_tag(self) -> str:
        """doorway, hallway, or room from this robot's own map. Not the floor plan."""
        x, y, _yaw = self.odometry.pose()
        cell = self.grid.cell_of(x, y)
        if cell is None:
            return "room"
        row, col = cell
        doors = self.room_doors
        if doors is not None and 0 <= row < doors.shape[0] and 0 <= col < doors.shape[1] and bool(doors[row, col]):
            return "doorway"
        labels = self.room_labels
        if labels is not None and 0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]:
            name = str(self.room_names.get(int(labels[row, col]), ""))
            if name.startswith("hall"):
                return "hallway"
        return "room"

    def _major(self, action: str, reason: str, **numbers: float) -> None:
        x, y, _yaw = self.odometry.pose()
        log_major(self.t_s, x, y, self._place_tag(), action, reason, **numbers)

    def _note_approach(self, why: str, v: float, w: float) -> None:
        # Separate from the motion log. Sharing that throttle let a straight
        # crossing write a line on every tick, because the scored command
        # changed the motion name in between.
        if why == self._approach_why and self.t_s < self._approach_log_at:
            return
        self._approach_why = why
        self._approach_log_at = self.t_s + 1.0
        err = abs(wrap_rad(self._segment_heading() - self.odometry.pose()[2]))
        clear = _pose_clearance(self.grid, *self.odometry.pose()[:2])
        if why == "pivot":
            self._major("pivot", "heading_off", heading_err_deg=math.degrees(err), clearance_m=clear)
        elif why == "back":
            self._major("back", "cant_see_behind_backing_out", clearance_m=clear)
        elif why == "recover":
            self._major("recover", "contact_bump", clearance_m=clear)
        elif why == "wide":
            self._major("wide", "too_tight_to_fit", heading_err_deg=math.degrees(err), clearance_m=clear)
        elif why == "straight":
            self._major("cross", "heading_off", heading_err_deg=math.degrees(err), clearance_m=clear)

    def _note_motion(self, choice) -> None:
        if choice.name == self._last_motion and self.t_s < self._motion_log_at:
            return
        self._last_motion = choice.name
        self._motion_log_at = self.t_s + 1.0
        if choice.name == "doorway_refusal":
            self._major("stop", "too_tight_to_fit", clearance_m=_pose_clearance(self.grid, *self.odometry.pose()[:2]))
        elif choice.name in ("back_out", "doorway_rescan"):
            self._major("back", "cant_see_behind_backing_out", clearance_m=_pose_clearance(self.grid, *self.odometry.pose()[:2]))

    def _shell_clips(self, v: float, w: float) -> bool:
        return footprint_clips(self.grid, self.odometry.pose(), v, w)

    def _remember_command(self, v: float, w: float) -> None:
        if abs(v) < 0.02 and abs(w) < 0.05:
            self._commit = None
            return
        self._commit = (float(v), float(w))
        self._commit_at = self.t_s
        self._commit_until = self.t_s + 0.8

    def _hold_command(self, pose, view) -> tuple[float, float] | None:
        """Keep the last command briefly. A live or mapped contact drops it."""
        if self._commit is None or self.t_s <= self._commit_at or self.t_s >= self._commit_until:
            return None
        v, w = self._commit
        turning = abs(w) > TURN_W_RAD_S
        blocked = live_veto(v, w, view) is not None
        if not blocked and turning:
            blocked = footprint_clips(self.grid, pose, v, w)
        if not blocked and v < -0.02 and not turning and _behind_blocked(self.grid, pose):
            blocked = True
        if not blocked and v > 0.02 and not turning and _ahead_blocked(self.grid, pose):
            blocked = True
        if blocked:
            self._commit = None
            return None
        return v, w

    def _reverse_closed(self, pose) -> bool:
        rear = None if self._bubble_view is None else self._bubble_view.rear
        if rear is not None and float(rear) <= 0.06:
            return True
        return _behind_blocked(self.grid, pose)

    def _begin_back(self, pose) -> tuple[float, float] | None:
        if self._reverse_closed(pose):
            return None
        if not self._backing:
            self._backing = True
            self._back_from = (float(pose[0]), float(pose[1]))
        self._commit = None
        self._body_v, self._body_w = -0.10, 0.0
        self._note_approach("back", -0.10, 0.0)
        return wheel_speeds(-0.10, 0.0)

    def _dead_end(self, pose) -> bool:
        """No half-turn, and the free corridor behind is shorter than the robot."""
        if spin_fits(self.grid, pose):
            return False
        behind = free_behind_m(self.grid, pose)
        # Still a corridor to leave by, but not a robot-length of it.
        if behind < 0.18 or behind >= 0.48:
            return False
        return _pose_clearance(self.grid, float(pose[0]), float(pose[1])) < TURN_AIM_M

    def _goal_ahead(self, pose) -> bool:
        """The adopted goal lies in front of the nose, not behind it."""
        if self.goal_xy is None:
            return False
        bearing = math.atan2(self.goal_xy[1] - float(pose[1]), self.goal_xy[0] - float(pose[0]))
        return abs(wrap_rad(bearing - float(pose[2]))) < 1.0

    def _corner_contact(self) -> float | None:
        """A corner IR inside 0.10 m. A centre sonar on open floor is not this."""
        if self.t_s < self._contact_until:
            return None
        return corner_bump(self._proximity)

    def _begin_recover(self, pose) -> tuple[float, float] | None:
        """Reverse until the corner object can be seen, then stop and look."""
        if self._reverse_closed(pose):
            return None
        gap = self._corner_contact()
        if not self._recover:
            seen = 0.0 if gap is None else gap
            self._recover = True
            self._recover_from = (float(pose[0]), float(pose[1]))
            self._recover_need = max(0.12, SIGHT_M - seen)
            self._recover_phase = "reverse"
            self._recover_t0 = self.t_s
            self._backing = False
            self._commit = None
        return self._recover_command(pose)

    def _recover_command(self, pose) -> tuple[float, float] | None:
        if not self._recover:
            return None
        x, y = float(pose[0]), float(pose[1])
        origin = self._recover_from
        dist = 0.0 if origin is None else math.hypot(x - origin[0], y - origin[1])
        gap = corner_gap(self._proximity)
        if self._recover_phase == "reverse":
            # A reverse that does not move is into the wedge, not out of it.
            if self.t_s - self._recover_t0 > 0.5 and dist < 0.02:
                self._recover = False
                self._contact_until = self.t_s + 2.0
                self._reverse_failed = True
                return None
            in_view = gap is None or gap >= min(SIGHT_M, 0.28)
            if (dist >= self._recover_need and in_view) or dist >= 0.55 or self._reverse_closed(pose):
                self._recover_phase = "look"
                self._recover_until = self.t_s + 0.25
                self._body_v, self._body_w = 0.0, 0.0
                self._note_approach("recover", 0.0, 0.0)
                return 0.0, 0.0
            self._body_v, self._body_w = -0.10, 0.0
            self._note_approach("recover", -0.10, 0.0)
            return wheel_speeds(-0.10, 0.0)
        if self.t_s < self._recover_until:
            self._body_v, self._body_w = 0.0, 0.0
            return 0.0, 0.0
        if corner_bump(self._proximity) is not None and not self._reverse_closed(pose) and dist < 0.55:
            self._recover_phase = "reverse"
            self._recover_need = dist + max(0.12, SIGHT_M - gap)
            self._body_v, self._body_w = -0.10, 0.0
            self._note_approach("recover", -0.10, 0.0)
            return wheel_speeds(-0.10, 0.0)
        self._recover = False
        self._recover_phase = ""
        return None

    def _continue_back(self, pose) -> tuple[float, float] | None:
        """Keep reversing until the centre can turn, the map closes, or 1 m."""
        if not self._backing:
            return None
        # A forward goal is not a reason to keep backing. That was the
        # bedroom exit: the goal pointed out and the shell reversed in.
        if self._goal_ahead(pose) and self._corner_contact() is None:
            self._backing = False
            self._back_from = None
            return None
        x, y = float(pose[0]), float(pose[1])
        origin = self._back_from
        dist = 0.0 if origin is None else math.hypot(x - origin[0], y - origin[1])
        if dist >= 1.0 or _pose_clearance(self.grid, x, y) >= TURN_AIM_M or self._reverse_closed(pose):
            self._backing = False
            self._back_from = None
            return None
        self._body_v, self._body_w = -0.10, 0.0
        self._note_approach("back", -0.10, 0.0)
        return wheel_speeds(-0.10, 0.0)

    def _needs_room(self, pose, heading_err: float) -> bool:
        """A turn is being asked for where the centre cannot pivot."""
        clear = _pose_clearance(self.grid, float(pose[0]), float(pose[1]))
        if clear >= TURN_AIM_M:
            return False
        if abs(heading_err) > 0.55:
            return True
        return bool(self._pocket and clear < TURN_CLEAR_M)

    def _issue_wide(self, command) -> tuple[float, float]:
        v, w, _why = command
        self._backing = False
        self._remember_command(v, w)
        self._note_approach("wide", v, w)
        self._body_v, self._body_w = v, w
        return wheel_speeds(v, w)

    def _go(self, blocked: bool, cap: float, left_clear: float = 99.0, right_clear: float = 99.0) -> tuple[float, float]:
        del blocked, cap
        if self.done:
            self._body_v, self._body_w = 0.0, 0.0
            return 0.0, 0.0
        pose = self.odometry.pose()
        self._note_translation()
        self._note_progress()
        self._note_pile()
        stuck = self._stuck_escape(pose)
        if stuck is not None:
            return stuck
        if self._pile_forced:
            # Sixty seconds in the same 0.45 m ball. Never stop in place.
            self._relief = None
            self._blind_back = False
            leaving = self._straight_leave(pose)
            if leaving is None:
                leaving = self._trail_reverse(pose, trust_trail=True)
            if leaving is None and not self._view_blind and float(self._last_clear[0]) >= 0.50:
                self._body_v, self._body_w = 0.10, 0.0
                self._note_approach("wide", 0.10, 0.0)
                leaving = wheel_speeds(0.10, 0.0)
            if leaving is not None:
                return leaving
            # A wedge does not get another spin. The camera and the IMU
            # already took the reverse above when the picture or the gyro
            # disagreed with the wheels.
            if not self._view_blind and self._stuck_reason() is None:
                view = self._bubble_view
                left = 0.0 if view is None or view.left is None else float(view.left)
                right = 0.0 if view is None or view.right is None else float(view.right)
                turn = 0.30 if left >= right else -0.30
                if self._spin_safe(turn):
                    self._body_v, self._body_w = 0.0, turn
                    self._note_approach("pivot", 0.0, turn)
                    return wheel_speeds(0.0, turn)
            self._body_v, self._body_w = -0.10, 0.0
            self._note_approach("back", -0.10, 0.0)
            return wheel_speeds(-0.10, 0.0)
        blind = self._stall_escape(pose)
        if blind is not None:
            # The timed relief reverse is 0.45 s. A stall replaces it.
            self._relief = None
            return blind
        relief = self._relief_command()
        if relief is not None:
            return relief
        if self.phase != "go":
            return 0.0, 0.0
        pose = self.odometry.pose()
        contact = self._corner_contact()
        if self._recover or (contact is not None and not self._reverse_closed(pose)):
            recover = self._begin_recover(pose) if not self._recover else self._recover_command(pose)
            if recover is not None:
                return recover
        elif contact is not None:
            # Can't back off this corner. Sitting here is not a recovery.
            self._recover = False
            self._contact_until = self.t_s + 1.5
        backing = self._continue_back(pose)
        if backing is not None:
            return backing
        # An empty path with a future replan time is a wait, not another
        # plan. Calling it every step flooded the log and stalled the sim.
        if not self.path_xy and self.t_s < self._replan_at:
            self._body_v, self._body_w = 0.0, 0.0
            return 0.0, 0.0
        if self.t_s >= self._replan_at or not self.path_xy:
            self._plan()
        if self.phase != "go" or self.done or not self.path_xy:
            self._body_v, self._body_w = 0.0, 0.0
            return 0.0, 0.0
        pose = self.odometry.pose()
        approach = approach_command(self.grid, pose, self.path_xy)
        heading_err = wrap_rad(self._segment_heading() - pose[2])
        if approach is not None and approach[2] == "pivot":
            err = abs(heading_err)
            # Already turning: stop once the nose is inside 12° and drive.
            # Not turning: ignore a small leftover so the pivot does not
            # restart on the next cell.
            release = (self._pivot_hold and err < PIVOT_EXIT_RAD) or (
                not self._pivot_hold and err < PIVOT_ENTER_RAD
            )
            if release:
                self._pivot_hold = False
                approach = None
            elif footprint_clips(self.grid, pose, 0.0, approach[1]):
                self._pivot_hold = False
                approach = (-0.10, 0.0, "back")
            else:
                # A pivot is v = 0. Giving up into a forward arc is how the
                # shell drove into a jamb while the log still said pivot.
                self._pivot_hold = True
                self._backing = False
                self._commit = None
                _v, w, _why = approach
                self._body_v, self._body_w = 0.0, w
                self._note_approach("pivot", 0.0, w)
                return wheel_speeds(0.0, w)
        elif abs(heading_err) < PIVOT_ENTER_RAD:
            self._pivot_hold = False
        if approach is not None and approach[2] == "back":
            # Backing here is to make room. It is not the way out when the
            # goal is still in front; a corner contact has its own reverse.
            if self._goal_ahead(pose):
                approach = None
            else:
                cmd = self._begin_back(pose)
                if cmd is not None:
                    return cmd
                approach = None
        if approach is not None and approach[2] == "wide":
            if approach[0] < 0.0 and self._goal_ahead(pose):
                approach = None
            elif footprint_clips(self.grid, pose, approach[0], approach[1]):
                # The arc's corner meets the map. The back-up rule below
                # decides whether reversing is legal.
                approach = None if self._goal_ahead(pose) else (-0.10, 0.0, "back")
            else:
                return self._issue_wide(approach)
        v, w, arrived = pure_pursuit(pose, self.path_xy, arrive_m=ARRIVE_M)
        if arrived:
            if self._pocket:
                if self.goal_xy is not None:
                    self._remember_attempt(self.goal_xy[0], self.goal_xy[1], 1.5)
                    if self._frontier_fail_ok():
                        self._note_frontier_fail(self.goal_xy[0], self.goal_xy[1], self._goal_cells)
                self.goal_xy = None
                self._goal_cells = []
                self.path_xy = []
                self._pocket = False
                self._commit = None
                self._replan_at = self.t_s
                cmd = self._begin_back(pose)
                if cmd is not None:
                    return cmd
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
        if approach is not None and approach[2] == "straight":
            v, w = approach[0], approach[1]
            heading_err = 0.0
            self._note_approach("straight", v, w)
        else:
            heading_err = wrap_rad(self._segment_heading() - pose[2])
        through = self._creep_through_gap(pose, v, w, heading_err)
        if through is not None:
            return through
        if (
            (approach is None or approach[2] != "straight")
            and self._needs_room(pose, heading_err)
            and not self._goal_ahead(pose)
        ):
            wide = wide_escape(self.grid, pose)
            if (
                wide is not None
                and not (wide[0] < 0.0 and self._goal_ahead(pose))
                and not footprint_clips(self.grid, pose, wide[0], wide[1])
            ):
                return self._issue_wide(wide)
            cmd = self._begin_back(pose)
            if cmd is not None:
                return cmd
        if (approach is None or approach[2] != "straight") and self._dead_end(pose):
            spot = self.goal_xy if self.goal_xy is not None else (float(pose[0]), float(pose[1]))
            self._note_refusals(
                {
                    "refused": [
                        {
                            "xy": spot,
                            "reason": "dead end",
                            "clear_m": round(_pose_clearance(self.grid, float(pose[0]), float(pose[1])), 2),
                            "size_m": round(free_behind_m(self.grid, pose), 2),
                        }
                    ]
                }
            )
            cmd = self._begin_back(pose)
            if cmd is not None:
                return cmd
        held = self._hold_command(pose, view)
        if held is not None:
            self._body_v, self._body_w = held
            return wheel_speeds(*held)
        choice = choose_motion(
            view=view,
            pursuit_v=v,
            pursuit_w=w,
            heading_err=heading_err,
            grid=self.grid,
            pose=pose,
            left_clear=float(left_clear),
            right_clear=float(right_clear),
            radar_blocked=self._radar_blocked,
        )
        self._note_motion(choice)
        if choice.name in ("back_out", "doorway_rescan"):
            cmd = self._begin_back(pose)
            if cmd is not None:
                return cmd
        self._remember_command(choice.v, choice.w)
        self._body_v, self._body_w = choice.v, choice.w
        return wheel_speeds(choice.v, choice.w)

    def sweep_summary(self) -> str:
        return (
            f"checks={self.sweep_checks} rejected={self.sweep_rejected} "
            f"wide={self.sweep_widened} align={self.sweep_aligned} "
            f"straight={self.sweep_straight} stop={self.sweep_stopped}"
        )

    def _log_goal(self, goal: tuple[float, float], report: dict) -> None:
        del report
        self._major("goal", "goal_changed", goal_x=float(goal[0]), goal_y=float(goal[1]))
        self._log(f"goal ({goal[0]:.2f},{goal[1]:.2f})")

    def _note_refusals(self, report: dict) -> None:
        """One line per refused spot, not one line per replan."""
        for item in report.get("refused") or []:
            xy = item.get("xy")
            if not xy:
                continue
            key = (round(float(xy[0]), 1), round(float(xy[1]), 1), str(item.get("reason")))
            if key in self._refused:
                continue
            self._refused.add(key)
            raw = str(item.get("reason") or "")
            reason = "dead_end" if raw == "dead end" else "no_room_to_turn" if raw == "no room to turn" else "unknown"
            self._major(
                "refuse",
                reason,
                goal_x=float(xy[0]),
                goal_y=float(xy[1]),
                clearance_m=float(item.get("clear_m") or 0.0),
            )
            self._log(
                f"refuse ({float(xy[0]):.2f},{float(xy[1]):.2f}) "
                f"{item.get('reason')} clear={item.get('clear_m')}m"
            )

    def _plan(self, depth: int = 0) -> None:
        self._replan_at = self.t_s + 4.0
        prefer_near = self._prefer_near
        self._prefer_near = False
        x, y, yaw = self.odometry.pose()
        if depth > 6:
            self._no_frontier()
            return
        rooms = segment_map(self.grid.labels(), self.grid.res)
        self.room_labels = rooms["labels"]
        self.room_names = rooms["names"]
        self.room_doors = rooms["doors"]
        report: dict = {}
        banned = set(self._ban_set()) | self._held_cells()
        aim = CREEP_CLEAR_M if self._narrow_gap((x, y, yaw)) else None
        chosen = select_goal(
            self.grid,
            (x, y),
            radius_m=ROBOT_RADIUS_M,
            banned=banned,
            avoid=self._avoid_now(),
            rooms=rooms,
            heading_xy=(math.cos(yaw), math.sin(yaw)),
            report=report,
            prefer_near=prefer_near,
            visited=self.visited,
            turn_aim_m=aim,
        )
        self._note_refusals(report)
        if chosen is None:
            self._no_frontier()
            return
        goal, path = chosen
        if self._quarantine is not None:
            qx, qy, qt = self._quarantine
            if self.t_s - qt > 30.0:
                self._quarantine = None
            elif math.hypot(goal[0] - qx, goal[1] - qy) < 0.80 and depth < 4:
                self._remember_attempt(float(goal[0]), float(goal[1]), 6.0)
                self._plan(depth + 1)
                return
        pending = list(report.get("cells") or [])
        if self._goal_on_hold(goal):
            # Moved less than GOAL_HOLD_M. Leave it out until the shell does.
            self.goal_xy = None
            self._goal_cells = []
            self.path_xy = []
            self._replan_at = self.t_s + 1.0
            return
        if math.hypot(goal[0] - x, goal[1] - y) < ARRIVE_M:
            self._remember_attempt(goal[0], goal[1], 1.5)
            if self.t_s - self._last_spin_t > 12.0:
                self._last_spin_t = self.t_s
                self.phase = "spin"
                self._spin_start = None
                self.goal_xy = None
                self._goal_cells = []
                self.path_xy = []
                self._log_goal(goal, report)
                return
            # Already looked. A second spin on the same patch is the loop
            # between two pieces of furniture. That is a failure. The
            # one-bump weight above still expires.
            if self._frontier_fail_ok():
                self._note_frontier_fail(goal[0], goal[1], pending)
            self._plan(depth + 1)
            return
        if self.goal_xy is not None and self.path_xy:
            shift = math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1])
            # Same target: refresh the path. A different target waits, so
            # the goal does not flip every replan.
            if shift < 0.45:
                self.path_xy = path
                self._pocket = bool(report.get("tight"))
                if pending:
                    self._goal_cells = pending
                return
            if self.t_s - self._goal_since < 8.0:
                return
            new_score = float(report.get("score") or 0.0)
            if self._goal_score > 0.0 and new_score < self._goal_score * 1.25:
                return
        same = (
            self.goal_xy is not None
            and math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1]) < 0.2
        )
        self.goal_xy, self.path_xy = goal, path
        self._goal_cells = pending
        self._goal_score = float(report.get("score") or 0.0)
        self._pocket = bool(report.get("tight"))
        self.frontiers_left = 1
        self.phase = "go"
        self.done = False
        if not same:
            self._goal_since = self.t_s
            self._motion_anchor = None
            self._commit = None
            self._log_goal(goal, report)

    def _no_frontier(self) -> None:
        """No reachable frontier. Done only after a scan and a stopped hold."""
        self._pocket = False
        if not self._confident():
            self._log("frontiers hidden, pose not trusted, not done")
            self.goal_xy = None
            self._goal_cells = []
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
            self._goal_cells = []
            self.path_xy = []
            self.phase = "spin"
            self._spin_start = None
            self._spin_then = "settle"
            self.done = False
            return
        self.frontiers_left = 0
        self.goal_xy = None
        self._goal_cells = []
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
        report: dict = {}
        chosen = select_goal(
            self.grid,
            (x, y),
            radius_m=ROBOT_RADIUS_M,
            banned=self._ban_set(),
            avoid=self._avoid_now(),
            rooms=rooms,
            heading_xy=(math.cos(yaw), math.sin(yaw)),
            report=report,
            visited=self.visited,
        )
        self._note_refusals(report)
        if chosen is not None:
            self._log("frontier open, resuming")
            self._did_final_scan = False
            self.phase = "go"
            goal, path = chosen
            self.goal_xy, self.path_xy = goal, path
            self._goal_cells = list(report.get("cells") or [])
            self._goal_score = float(report.get("score") or 0.0)
            self._pocket = bool(report.get("tight"))
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
