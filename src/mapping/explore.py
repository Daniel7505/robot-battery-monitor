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
    BLIND_CLEARANCE_M,
    BLIND_VIEW_FRACTION,
    OAK_DEPTH,
    ROBOT_RADIUS_M,
    SPIN_RAD_S,
    STOP_CLEARANCE_M,
    DepthCamera,
)
from src.mapping.depth_cloud import blind_fraction, body_points, forward_clearance, to_odom
from src.mapping.drive import pure_pursuit, wheel_speeds
from src.mapping.frontiers import select_goal
from src.mapping.grid import OccupancyGrid
from src.mapping.odometry import WheelGyroOdometry
from src.mapping.scan_match import pose_correction, scan_fit
from src.mapping.slip import MotionCheck


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
        self.frontiers_left = 0
        self._last_clear = (99.0, 99.0, 99.0)

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
        else:
            prefer = self._turn
        if self._view_blind:
            # No return is not an open hallway. The nose is inside minRange.
            clearance = min(clearance, BLIND_CLEARANCE_M)
        # Freeze a forward command the body did not follow, and a forward
        # command into a view the camera cannot see. A reverse that the
        # hubs actually follow is the robot leaving the wall, so it counts.
        # Ordinary braking (a visible wall inside the stop range) still
        # counts too, or the pose would sit a halt short of every wall.
        forward_cmd = (self._commanded[0] + self._commanded[1]) > 0.3
        in_contact = stalled or (self._view_blind and forward_cmd)
        drop = False
        if depth is not None or self._view_blind or in_contact:
            before = self._motion.events
            slip_drop = self._motion.observe(dt, ds, clearance, blind=self._view_blind)
            if slip_drop:
                self._slip_debt_m = min(2.0, self._slip_debt_m + abs(ds))
            if self._motion.events > before and self.phase == "go":
                self._note_slip()
            drop = slip_drop or in_contact
        self.odometry.update(
            left_wheel_rad,
            right_wheel_rad,
            yaw_rate_rad_s,
            dt,
            trust_translation=not drop,
        )
        corrected = False
        if hits is not None and hits.size and self.phase in ("go", "spin") and not drop and self.t_s >= self._nudge_at:
            # Match before the rays are painted, so this frame lands on the
            # corrected pose. Contact is skipped: a scan into the lens is
            # not a pose measurement.
            dx, dy, dyaw = pose_correction(self.grid, self.odometry.pose(), hits[:, :2])
            self._nudge_at = self.t_s + 0.45
            # One lucky peak is not a pose. Apply only when the next scan
            # agrees, so a match cannot walk back and forth along a wall.
            prev = self._match_proposal
            prev_t = self._match_proposal_t
            self._match_proposal = (dx, dy, dyaw)
            self._match_proposal_t = self.t_s
            if (dx != 0.0 or dy != 0.0 or dyaw != 0.0) and prev is not None and (self.t_s - prev_t) <= 1.6:
                dot = prev[0] * dx + prev[1] * dy + 0.4 * prev[2] * dyaw
                na = math.hypot(dx, dy, 0.4 * dyaw)
                nb = math.hypot(prev[0], prev[1], 0.4 * prev[2])
                if na > 1e-3 and nb > 1e-3 and dot >= 0.65 * na * nb:
                    self.odometry.apply_correction(dx, dy, dyaw)
                    corrected = True
                    self._slip_debt_m = max(0.0, self._slip_debt_m - 0.20)
                    self._log(f"match dx={dx:.3f} dy={dy:.3f} dyaw={dyaw:.3f}")
        x, y, yaw = self.odometry.pose()
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
        self.grid.stamp_footprint(x, y)
        if hits is not None and sensor is not None and floors is not None:
            origin = to_odom(sensor.reshape(1, 3), (x, y, yaw))[0]
            self.grid.integrate(origin, to_odom(hits, (x, y, yaw)), to_odom(floors, (x, y, yaw)))
        self._last_clear = (clearance, left_clear, right_clear)
        if self._view_blind or clearance < 1.5:
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
        if self._heading_anchor is None or abs(yaw_u - self._heading_anchor[1]) > 0.30:
            self._heading_anchor = (self.t_s, yaw_u)
        heading_stuck = (self.t_s - self._heading_anchor[0]) > 3.5
        if math.hypot(x - self._progress_xy[0], y - self._progress_xy[1]) > 0.05:
            self._progress_xy = (x, y)
            self._progress_t = self.t_s
        elif (
            self.phase == "go"
            and not self.done
            and self.t_s >= self._recover_until
            and (self.t_s - self._progress_t) > 3.5
            and (close_held or heading_stuck)
        ):
            self._start_recovery()
        # Nose only. A jamb in the shoulder cone is a steer (see _go),
        # not a wall to spin away from.
        blocked = radar_blocked or clearance < STOP_CLEARANCE_M
        # One direction for the whole blocked episode. Picking again
        # every frame wags the robot between the two sides of a jamb.
        # The path stays: throwing it away on a one-frame block is what
        # swapped the doorway goal for a new one every tenth of a second.
        if self.phase == "go" and blocked:
            if self.t_s >= self._avoid_until:
                self._turn = prefer
                if self._replan_at < self.t_s:
                    self._replan_at = self.t_s + 0.45
            self._avoid_until = self.t_s + 0.08
        if self.phase == "settle":
            return self._remember(self._settle())
        if self.phase == "spin":
            return self._remember(self._spin())
        if self.phase == "go":
            return self._remember(self._go(blocked, clearance, left_clear, right_clear))
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
        # Long enough to leave the depth camera's minimum (~0.35 m past
        # the lens). A short hop stayed blind and the next drive went
        # straight back into the same surface. Then turn, so the next
        # forward command is not the same heading.
        self._reverse_until = self.t_s + 2.0
        self._avoid_until = self._reverse_until + 1.5
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
        self._ban_current(immediate=True)
        # A small, expiring patch on the jamb. Radius 12 sealed the 0.81 m
        # opening and the ban never came back, so the bedroom was abandoned.
        self._blacklist_xy(x + math.cos(yaw) * 0.30, y + math.sin(yaw) * 0.30, radius=4, ttl_s=12.0)
        self._reinforce_corners(x, y, yaw)
        self._reverse_until = self.t_s + 2.2
        self._avoid_until = self._reverse_until + 1.4
        self._recover_until = self._reverse_until + 0.4
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
        if left_clear + 0.08 < right_clear:
            return -0.40
        if right_clear + 0.08 < left_clear:
            return 0.40
        return 0.0

    def _go(self, blocked: bool, clearance: float, left_clear: float = 99.0, right_clear: float = 99.0) -> tuple[float, float]:
        if self.done:
            return 0.0, 0.0
        if self._goal_expired() or (self._is_stuck() and clearance < 1.5):
            self._ban_current()
            self._plan()
        if self.phase != "go":
            return 0.0, 0.0
        if self.t_s < self._reverse_until:
            # Nearly straight. A hard yaw here swings a tire into the jamb
            # the reverse was supposed to leave.
            return wheel_speeds(-0.16, self._turn * 0.12)
        if self.t_s < self._avoid_until or blocked:
            return wheel_speeds(0.0, self._turn)
        if self.t_s >= self._replan_at or not self.path_xy:
            self._plan()
        if self.phase != "go" or self.done or not self.path_xy:
            return 0.0, 0.0
        v, w, arrived = pure_pursuit(self.odometry.pose(), self.path_xy, arrive_m=ARRIVE_M)
        w += self._jamb_yaw(left_clear, right_clear)
        if left_clear < 0.90 and right_clear < 0.90:
            v = min(v, 0.16)
        if clearance < 1.15:
            span = max(0.15, 1.15 - STOP_CLEARANCE_M)
            v *= max(0.30, (clearance - STOP_CLEARANCE_M) / span)
        if arrived and left_clear < 1.05 and right_clear < 1.05 and clearance > STOP_CLEARANCE_M:
            # Both jambs are beside us and the nose is clear. Drive through
            # instead of spinning in the opening.
            return wheel_speeds(0.16, self._jamb_yaw(left_clear, right_clear))
        if arrived:
            self._last_spin_t = self.t_s
            self.phase = "spin"
            self._spin_start = None
            self.goal_xy = None
            self.path_xy = []
            return wheel_speeds(0.0, SPIN_RAD_S)
        return wheel_speeds(v, w)

    def _plan(self, depth: int = 0) -> None:
        self._replan_at = self.t_s + 3.0
        x, y, _yaw = self.odometry.pose()
        if depth > 6:
            self._no_frontier()
            return
        banned = self._ban_set()
        chosen = select_goal(self.grid, (x, y), radius_m=ROBOT_RADIUS_M, banned=banned)
        if chosen is None and self._banned:
            self._log("no goal under bans, clearing them")
            self._banned.clear()
            chosen = select_goal(self.grid, (x, y), radius_m=ROBOT_RADIUS_M, banned=set())
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
        x, y, _yaw = self.odometry.pose()
        chosen = select_goal(self.grid, (x, y), radius_m=ROBOT_RADIUS_M, banned=self._ban_set())
        if chosen is None and self._banned:
            self._log("no goal under bans, clearing them")
            self._banned.clear()
            chosen = select_goal(self.grid, (x, y), radius_m=ROBOT_RADIUS_M, banned=set())
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
