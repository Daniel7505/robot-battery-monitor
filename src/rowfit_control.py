"""Row-fit lane keep: steer + speed from a fitted lane model.

Used by the Webots controller in the default lane mode (``RBM_LANE_MODE``
unset or ``rowfit``). ``RBM_LANE_MODE=gap`` runs the old pixel path
(``lane_keep.lane_keep_command``), which this module does not touch.

Per NEW camera frame (every ~320 ms) the controller:

* picks a look-ahead distance that grows with speed,
* takes the point on the fitted centre line that far ahead and computes
  the pure-pursuit curvature to reach it. On a constant-curvature lane
  with the robot on it, pure pursuit returns exactly the lane curvature,
  so offset, heading and curvature feed-forward are all in one number,
* adds a small damping term on the measured offset rate (frame to frame,
  using the real time between frames, not the 8 ms physics tick),
* sets a target speed from the sharpest curvature it can see (slow-in)
  and lets it climb back when the lane straightens (speed-up-out).

Between frames the curvature command and target speed are HELD. Each
physics tick only the speed ramp and steer slew run, and wheel speeds
come from ``lane_keep.wheels_from_steer`` (same limits as the gap path).
Sign conventions match ``lane_keep``: steer > 0 = yaw right.
"""

from __future__ import annotations

import math

from src.lane_keep import (
    DEFAULT_K_STEER,
    SteerFilter,
    WHEEL_RADIUS_M,
    WHEEL_Y_LEFT_M,
    wheels_from_steer,
)

TRACK_M = 2.0 * WHEEL_Y_LEFT_M  # 0.34 m between drive wheels
V_CRUISE_M_S = 0.44  # = 5.5 rad/s, the gap path cruise
V_MIN_M_S = 0.20
V_HELD_M_S = 0.30  # cap while steering a held (rejected-fit) lane
A_LAT_M_S2 = 0.12  # curve speed: v = sqrt(a_lat / |kappa|)
ACCEL_M_S2 = 0.25  # speed-up-out (gentle)
DECEL_M_S2 = 0.80  # slow-in (quick)
LD_BASE_M = 0.45
LD_GAIN_S = 1.2  # look-ahead = base + gain * v
LD_MIN_M = 0.55  # pursuit-stability floor (not a camera number)
LD_MAX_M = 1.6  # default only; the runtime derives it from the camera coverage
# Look-ahead window from the camera footprint (lookahead_bounds_from_coverage):
# goal at most LD_FAR_MARGIN_M short of the nearest top-row reach (so the far
# fit still has line beyond it), at least LD_NEAR_MARGIN_M past the bottom row.
LD_FAR_MARGIN_M = 0.36  # 1.96 m top row -> 1.6 m, the tuned value
LD_NEAR_MARGIN_M = 0.25
LD_ABS_MIN_M = 0.4
LD_ABS_MAX_M = 2.5
K_D_OFFSET = 0.6  # 1/m per (m/s) of offset rate
KAPPA_MAX = 2.5
STEER_CAP = 0.8
MIN_CONF = 0.15
LOST_FRAMES_STOP = 3  # ~1 s at 320 ms frames

# --- Sharp corners (lane_vision corner cue: "the line widens, then disappears")
# LANE -> CORNER_APPROACH -> PIVOT -> REACQUIRE -> LANE. Distances come from
# the cue and the measured lane width; nothing about the track is known.
STATE_LANE = "LANE"
STATE_APPROACH = "CORNER_APPROACH"
STATE_PIVOT = "PIVOT"
STATE_REACQUIRE = "REACQUIRE"
CORNER_MIN_CONF = 0.6  # act on a persistent cue at least this sure
V_APPROACH_M_S = 0.25  # cap while closing on the pivot point
A_APPROACH_M_S2 = 0.30  # stopping profile v = sqrt(2 a remaining)
V_CREEP_M_S = 0.04  # last centimetres (no zero-speed stall short of the point)
PIVOT_ARRIVE_M = 0.015  # remaining distance that starts the pivot
PIVOT_ANGLE_RAD = math.pi / 2  # a sharp corner of this kind is 90 deg
PIVOT_K_YAW = 2.0  # yaw rate per rad of yaw error (1/s)
PIVOT_W_MAX = 0.9  # rad/s cap on the in-place turn rate
PIVOT_W_MIN = 0.12  # rad/s floor while outside tolerance (beats stiction)
PIVOT_ALPHA = 1.5  # rad/s^2 rate limit (spin-up / spin-down)
PIVOT_TOL_RAD = math.radians(1.0)
PIVOT_W_SETTLED = 0.05  # rad/s: turn rate counted as stopped
PIVOT_SETTLE_TICKS = 4  # inside tolerance this many ticks = done
PIVOT_LANE_EXIT_RAD = math.radians(12.0)  # vision may end the pivot inside this yaw error ...
PIVOT_LANE_ALIGNED_RAD = math.radians(1.5)  # ... if it sees a lane this well aligned
PIVOT_TIMEOUT_S = 8.0
REACQUIRE_RELAX_FRAMES = 6  # tracker gate relaxed for this many frames after a pivot
REACQUIRE_OK_FRAMES = 3  # valid frames in a row back to LANE
V_REACQUIRE_M_S = 0.20
REACQUIRE_MAX_HEADING_RAD = math.radians(20.0)
CORNER_COOLDOWN_M = 0.5  # no new corner until this far into the new leg

# --- Intersections (src.junction_vision + src.route_policy)
# LANE -> JUNCTION_APPROACH -> (turn) CORNER_APPROACH -> PIVOT -> REACQUIRE
#                           -> (straight past a side branch) LANE
#                           -> (straight over a crossing / gap) GAP_CROSS -> REACQUIRE -> LANE
#                                                   GAP_CROSS -> (nothing by max blind) LOOK_AROUND -> STOPPED
STATE_JUNCTION = "JUNCTION_APPROACH"
STATE_GAP = "GAP_CROSS"
STATE_LOOK = "LOOK_AROUND"
STATE_STOPPED = "STOPPED"
V_JUNCTION_M_S = 0.25  # cap from the first sign of an opening until past the junction
V_GAP_M_S = 0.15  # blind creep over a gap / crossing
OPENING_SLOW_M = 0.6  # one side opens (branch or corner inner line): V_JUNCTION cap this far
OPENING_SLOW_NEAR_M = 0.9  # ... once the opening is this close (decel 0.44 -> 0.25 m/s takes ~0.1 m)
MAX_BLIND_M = 4.0  # default RBM_MAX_BLIND_M: blind distance before giving up
JUNCTION_DECIDE_M = 0.35  # decide by this distance to the near crossing line at the latest
JUNCTION_LOST_FRAMES = 3  # no junction this many frames while still far = false alarm
JUNCTION_COOLDOWN_M = 0.8  # after a junction: no new one until this far on
GAP_K_YAW = 1.5  # heading hold: yaw rate per rad of error
GAP_W_MAX = 0.3
GAP_OK_FRAMES = 2  # lane frames in a row that end the blind crossing
GAP_MAX_HEADING_RAD = math.radians(20.0)
GAP_MAX_OFFSET_M = 0.45
GAP_WIDTH_TOL = 0.25  # +/- fraction of the approach lane width
GAP_MIN_BLIND_M = 0.15  # do not 'reacquire' the approach lane's own last metres
GAP_REACQ_NEAR_M = 1.0  # the new lane's lines must start within this distance (not just far rows)
LOOK_W_M_S = 0.5  # rad/s look-around turn rate
LOOK_ANGLES_RAD = (math.pi / 2, -math.pi / 2, 0.0)  # relative to the crossing heading


def lookahead_bounds_from_coverage(coverages) -> tuple[float, float]:
    """(ld_min, ld_max) from ``lane_vision.camera_coverage`` dicts of all lane cameras.

    Uses the shortest top-row reach and the farthest-forward bottom row, so
    the goal point stays inside what every camera sees. Clamped to
    [LD_ABS_MIN_M, LD_ABS_MAX_M]; falls back to LD_MIN_M/LD_MAX_M.
    """
    tops = [c.get("top_row_x_m") for c in coverages if c and c.get("top_row_x_m") is not None]
    bots = [c.get("bottom_row_x_m") for c in coverages if c and c.get("bottom_row_x_m") is not None]
    if not tops:
        return LD_MIN_M, LD_MAX_M
    ld_max = max(LD_ABS_MIN_M, min(LD_ABS_MAX_M, min(tops) - LD_FAR_MARGIN_M))
    ld_min = LD_MIN_M
    if bots:
        ld_min = max(ld_min, max(bots) + LD_NEAR_MARGIN_M)
    ld_min = max(LD_ABS_MIN_M, min(ld_min, ld_max))
    return ld_min, ld_max


def curve_speed(kappa_abs: float, conf: float = 1.0) -> float:
    """Target speed for the sharpest curvature ahead (and fit confidence)."""
    k = abs(float(kappa_abs))
    v = V_CRUISE_M_S if k < 1e-6 else min(V_CRUISE_M_S, math.sqrt(A_LAT_M_S2 / k))
    c = max(0.0, min(1.0, (float(conf) - MIN_CONF) / 0.45))
    v = V_MIN_M_S + (v - V_MIN_M_S) * c
    return max(V_MIN_M_S, min(V_CRUISE_M_S, v))


def steer_for_curvature(kappa: float, v_m_s: float, k_steer: float = DEFAULT_K_STEER) -> float:
    """wheels_from_steer(turn_slow=0): w_R - w_L = -2 k s  ->  yaw = v*kappa."""
    return -float(v_m_s) * float(kappa) * TRACK_M / (2.0 * float(k_steer) * WHEEL_RADIUS_M)


class SpeedGovernor:
    def __init__(self, v0: float = V_MIN_M_S) -> None:
        self.v = float(v0)

    def step(self, target: float, dt: float) -> float:
        dv = float(target) - self.v
        lim = (ACCEL_M_S2 if dv > 0 else DECEL_M_S2) * max(0.0, float(dt))
        self.v += max(-lim, min(lim, dv))
        return self.v


class RowfitController:
    def __init__(
        self,
        *,
        k_steer: float = DEFAULT_K_STEER,
        ld_min: float = LD_MIN_M,
        ld_max: float = LD_MAX_M,
    ) -> None:
        self.k_steer = float(k_steer)
        self.ld_min = float(ld_min)
        self.ld_max = float(ld_max)
        self.filter = SteerFilter()
        self.gov = SpeedGovernor()
        self.kappa_cmd = 0.0
        self.target_speed = V_MIN_M_S
        self.lookahead_used = 0.0
        self.frames_lost = 0
        self.last_offset: float | None = None
        self.n_frames = 0
        self.held_frames = 0
        # corner state machine
        self.tracker = None  # LaneTracker (optional): told to re-acquire after a pivot
        self.state = STATE_LANE
        self.events: list[str] = []  # console lines since the last drain_events()
        self.corner_dir = 0
        self.corner_dist_m: float | None = None  # last cue distance (outer crossbar)
        self.corner_conf = 0.0
        self.remaining_m: float | None = None  # to the pivot point
        self.lane_w_m: float | None = None
        self.lane_yaw: float | None = None  # IMU yaw of the approach lane's direction
        self.yaw_target: float | None = None
        self.pivot_w = 0.0
        self.pivot_t = 0.0
        self.pivot_settle = 0
        self.pivot_start_yaw: float | None = None
        self.last_pivot: dict | None = None  # {"dir", "target", "yaw", "err_deg", "time_s", "exit"}
        self.reacq_ok = 0
        self.reacq_frames = 0
        self.cooldown_m = 0.0
        self.pivots = 0
        self._reacq_fail_logged = False
        # intersections
        from src.route_policy import RoutePolicy

        self.policy = RoutePolicy()
        self.max_blind_m = MAX_BLIND_M
        self.look_around = True
        self.junction = None  # Junction being approached / crossed
        self.jn_near_m: float | None = None  # dead-reckoned distance to its near line
        self.jn_center_m: float | None = None
        self.jn_missing = 0
        self.jn_cooldown_m = 0.0
        self.jn_slow_m = 0.0  # V_JUNCTION cap for this many more metres
        self.jn_count = 0
        self.route_choice = ""
        self.turn_src = "corner"  # what CORNER_APPROACH is driving: "corner" or "junction"
        self.gap_travel_m = 0.0
        self.blind_m = 0.0
        self.gap_ok = 0
        self.gap_yaw: float | None = None  # heading held while crossing
        self.gap_seen: dict = {}
        self.look_i = 0
        self.look_seen: list = []
        self.junctions: list[dict] = []  # one record per decided junction (sim / report)
        # obstacles (src.obstacle_gate): an external forward-speed cap, None = no cap. It only
        # caps speed; the lane / junction state machine underneath keeps its progress.
        self.obstacle_v_cap: float | None = None

    def reset(self) -> None:
        tracker, route, mb, la = self.tracker, self.policy.route, self.max_blind_m, self.look_around
        self.__init__(k_steer=self.k_steer, ld_min=self.ld_min, ld_max=self.ld_max)
        self.tracker = tracker
        self.set_route(route, max_blind_m=mb, look_around=la)

    def set_route(self, route=None, *, max_blind_m: float | None = None, look_around: bool | None = None) -> None:
        from src.route_policy import RoutePolicy

        self.policy = RoutePolicy(route)
        if max_blind_m is not None and max_blind_m > 0:
            self.max_blind_m = float(max_blind_m)
        if look_around is not None:
            self.look_around = bool(look_around)

    def drain_events(self) -> list[str]:
        out, self.events = self.events, []
        return out

    def _event(self, msg: str) -> None:
        self.events.append(msg)

    def _pursuit(self, est, v: float) -> tuple[float, float]:
        """(pure-pursuit curvature, goal distance^2) on the estimate's centre arc."""
        Ld = max(self.ld_min, min(self.ld_max, LD_BASE_M + LD_GAIN_S * v))
        if est.lookahead_m > 0.0:
            Ld = max(self.ld_min, min(Ld, est.lookahead_m - 0.1))
        gx, gy = est.goal_point(Ld)
        d2 = gx * gx + gy * gy
        return (0.0 if d2 < 1e-6 else 2.0 * gy / d2), d2

    def on_frame(self, est, frame_dt: float = 0.32) -> None:
        """Recompute the held command from a NEW frame's LaneEstimate."""
        self.n_frames += 1
        if est is not None and getattr(est, "held", False):
            # Rejected (implausible) fit: steer along the last good lane, moved
            # by odometry/IMU, at reduced speed. Counts toward the lost-lane
            # brake unless the tracker says it is only a short view.
            self.held_frames += 1
            if getattr(est, "counts_as_lost", True):
                self.frames_lost += 1
            self.target_speed = min(self.target_speed, V_HELD_M_S)
            self.last_offset = None
            self.kappa_cmd = max(-KAPPA_MAX, min(KAPPA_MAX, self._pursuit(est, self.gov.v)[0]))
            return
        if est is None or not getattr(est, "valid", False) or est.confidence < MIN_CONF:
            self.frames_lost += 1
            self.target_speed = V_MIN_M_S
            self.last_offset = None
            return
        self.frames_lost = 0
        v = self.gov.v
        k_pp, d2 = self._pursuit(est, v)
        k_d = 0.0
        if self.last_offset is not None and frame_dt > 1e-3:
            k_d = -K_D_OFFSET * (est.offset_m - self.last_offset) / float(frame_dt)
        self.last_offset = est.offset_m
        self.kappa_cmd = max(-KAPPA_MAX, min(KAPPA_MAX, k_pp + k_d))
        self.lookahead_used = math.sqrt(d2)
        k_gov = max(
            abs(est.curvature_1pm),
            abs(getattr(est, "near_curvature_1pm", 0.0)),
            abs(k_pp),
        )
        self.target_speed = curve_speed(k_gov, est.confidence)

    # ------------------------------------------------------------------
    # corner state machine
    # ------------------------------------------------------------------
    def _corner_frame(self, est, yaw) -> None:
        """New frame: corner bookkeeping before the lane logic."""
        corner = getattr(est, "corner", None) if est is not None else None
        if self.state in (STATE_JUNCTION, STATE_GAP, STATE_LOOK) or (
                self.state == STATE_REACQUIRE and self.junction is not None) or self.jn_cooldown_m > 0.0:
            if self.tracker is not None and hasattr(self.tracker, "bar_filter_frames"):
                self.tracker.bar_filter_frames = max(self.tracker.bar_filter_frames, 2)
        if self.state == STATE_LANE:
            if (corner is not None and corner.seen and corner.confidence >= CORNER_MIN_CONF
                    and self.cooldown_m <= 0.0):
                w = self._lane_width(est)
                self.state = STATE_APPROACH
                self.corner_dir = int(corner.direction)
                self.lane_w_m = w
                self._take_corner(corner)
                self.lane_yaw = None
                if yaw is not None and est is not None and (est.valid or getattr(est, "held", False)):
                    self.lane_yaw = float(yaw) - float(est.heading_rad)
                self._event(
                    f"CORNER seen {corner.side} {corner.distance_m:.2f} m conf {corner.confidence:.2f} "
                    f"({corner.cams}) — approach, pivot point in {self.remaining_m:.2f} m "
                    f"(lane {w:.2f} m)" + ("" if yaw is not None else " — NO IMU yaw: will stop there")
                )
                self.turn_src = "corner"
                return
            self._junction_lane_frame(est, yaw)
            return
        if self.state == STATE_JUNCTION:
            self._junction_approach_frame(est, yaw)
            return
        if self.state == STATE_GAP:
            self._gap_frame(est, yaw)
            return
        if self.state == STATE_LOOK:
            self._look_frame(est)
            return
        if self.state == STATE_APPROACH:
            if self.turn_src == "junction":
                j = getattr(est, "junction", None) if est is not None else None
                if (j is not None and self.junction is not None and j.kind in _TURN_KINDS
                        and _family(j.kind) == _family(self.junction.kind)):
                    self.remaining_m = float(j.center_m)
            elif (self.turn_src == "corner" and corner is not None and corner.direction == self.corner_dir
                  and corner.confidence >= 0.4):
                self._take_corner(corner)
            if est is not None and est.valid and yaw is not None:
                # lane direction in IMU yaw, smoothed (the near lane is straight up to the corner)
                ly = float(yaw) - float(est.heading_rad)
                if self.lane_yaw is None:
                    self.lane_yaw = ly
                else:
                    self.lane_yaw += 0.3 * math.atan2(math.sin(ly - self.lane_yaw), math.cos(ly - self.lane_yaw))
            return
        if self.state == STATE_REACQUIRE:
            self.reacq_frames += 1
            ok = (est is not None and est.valid and est.confidence >= MIN_CONF
                  and abs(est.heading_rad) < REACQUIRE_MAX_HEADING_RAD)
            self.reacq_ok = self.reacq_ok + 1 if ok else 0
            if self.reacq_ok >= REACQUIRE_OK_FRAMES:
                self.state = STATE_LANE
                self.cooldown_m = CORNER_COOLDOWN_M
                if self.junction is not None:
                    self.jn_cooldown_m = max(self.jn_cooldown_m, JUNCTION_COOLDOWN_M)
                    self.junction = None
                    self.jn_center_m = None
                if self.tracker is not None and hasattr(self.tracker, "begin_settle"):
                    self.tracker.begin_settle()  # the new leg's fit is still settling
                self._event(
                    f"REACQUIRE ok after {self.reacq_frames} frames — off {est.offset_m * 100:+.0f}cm "
                    f"hd {math.degrees(est.heading_rad):+.1f}deg conf {est.confidence:.2f}, back to LANE"
                )

    def _lane_width(self, est) -> float:
        w = getattr(est, "lane_width_m", 0.0) if est is not None else 0.0
        if self.tracker is not None and getattr(self.tracker, "lane_w", None):
            w = self.tracker.lane_w
        return float(w) if w and 0.4 < w < 3.0 else 1.30

    def _take_corner(self, corner) -> None:
        self.corner_dist_m = float(corner.distance_m)
        self.corner_conf = float(corner.confidence)
        # Pivot point: the corner's centre-line vertex = outer line minus half a
        # lane. Turning 90 deg there puts the axle on the new leg's centre line.
        self.remaining_m = self.corner_dist_m - 0.5 * float(self.lane_w_m or 1.30)

    def _wheels(self, v: float, w: float) -> tuple[float, float]:
        """Wheel rad/s for forward speed v (m/s) and yaw rate w (rad/s, + = left)."""
        half = 0.5 * w * TRACK_M
        return (v - half) / WHEEL_RADIUS_M, (v + half) / WHEEL_RADIUS_M

    def _cmd(self, left, right, *, steer=0.0, reason, phase, v=0.0, brake=False, new_frame=False) -> dict:
        return {
            "left": round(left, 3),
            "right": round(right, 3),
            "brake": bool(brake),
            "error": round(float(steer), 4),
            "steer": round(float(steer), 4),
            "reason": reason,
            "phase": phase,
            "error_source": "rowfit",
            "target_speed": round(self.target_speed, 3),
            "speed_cmd": round(v, 3),
            "kappa_cmd": round(self.kappa_cmd, 4),
            "lookahead_m": round(self.lookahead_used, 3),
            "new_frame": bool(new_frame),
            **self.corner_fields(),
        }

    def corner_fields(self) -> dict:
        return {
            "state": self.state,
            "corner_dir": ("left" if self.corner_dir > 0 else "right") if self.corner_dir else "",
            "corner_m": None if self.corner_dist_m is None else round(self.corner_dist_m, 3),
            "corner_conf": round(self.corner_conf, 2),
            "pivot_remaining_m": None if self.remaining_m is None else round(self.remaining_m, 3),
            "yaw_target_deg": None if self.yaw_target is None else round(math.degrees(self.yaw_target), 1),
            "junction_type": self.junction.kind if self.junction is not None else "",
            "junction_m": None if self.jn_center_m is None else round(self.jn_center_m, 3),
            "openings": self.junction.openings().replace(" ", "") if self.junction is not None else "",
            "route_choice": self.route_choice,
            "blind_m": round(self.blind_m, 3) if self.state in (STATE_GAP, STATE_LOOK) else None,
        }

    def _start_pivot(self, yaw: float) -> None:
        base = self.lane_yaw if self.lane_yaw is not None else float(yaw)
        self.yaw_target = base + self.corner_dir * PIVOT_ANGLE_RAD
        self.pivot_start_yaw = float(yaw)
        self.pivot_w = 0.0
        self.pivot_t = 0.0
        self.pivot_settle = 0
        self.state = STATE_PIVOT
        self.gov.v = 0.0
        self.filter = SteerFilter()
        self._event(
            f"PIVOT start {'left' if self.corner_dir > 0 else 'right'} yaw {math.degrees(yaw):+.1f}deg -> "
            f"target {math.degrees(self.yaw_target):+.1f}deg (lane {math.degrees(base):+.1f}deg, "
            f"remaining {self.remaining_m:+.3f} m)"
        )

    def _end_pivot(self, yaw: float, how: str) -> None:
        err = math.atan2(math.sin(self.yaw_target - yaw), math.cos(self.yaw_target - yaw))
        self.last_pivot = {
            "dir": self.corner_dir, "target_deg": round(math.degrees(self.yaw_target), 2),
            "yaw_deg": round(math.degrees(yaw), 2), "err_deg": round(math.degrees(err), 2),
            "time_s": round(self.pivot_t, 2), "exit": how,
        }
        self.pivots += 1
        self._event(
            f"PIVOT done yaw {math.degrees(yaw):+.1f}deg (target {math.degrees(self.yaw_target):+.1f}, "
            f"err {math.degrees(err):+.1f}deg, {self.pivot_t:.1f} s, {how}) — REACQUIRE"
        )
        self.state = STATE_REACQUIRE
        self.pivot_w = 0.0
        self.reacq_ok = 0
        self.reacq_frames = 0
        self.frames_lost = 0
        self.last_offset = None
        self.kappa_cmd = 0.0
        self.target_speed = V_REACQUIRE_M_S
        self.gov.v = 0.0
        self.remaining_m = None
        if self.tracker is not None and hasattr(self.tracker, "begin_reacquire"):
            self.tracker.begin_reacquire(REACQUIRE_RELAX_FRAMES)

    def _pivot_tick(self, est, new_frame: bool, dt: float, yaw: float) -> dict:
        self.pivot_t += dt
        err = math.atan2(math.sin(self.yaw_target - yaw), math.cos(self.yaw_target - yaw))
        if new_frame and est is not None and est.valid and abs(err) < PIVOT_LANE_EXIT_RAD:
            if (est.n_left >= 4 and est.n_right >= 4 and abs(est.heading_rad) < PIVOT_LANE_ALIGNED_RAD
                    and est.confidence >= 0.4):
                self._end_pivot(yaw, f"lane seen aligned hd {math.degrees(est.heading_rad):+.1f}deg")
                return self._cmd(0.0, 0.0, reason="pivot done (lane)", phase="rowfit_" + self.state.lower(),
                                 new_frame=new_frame)
        if abs(err) <= PIVOT_TOL_RAD and abs(self.pivot_w) <= PIVOT_W_SETTLED:
            self.pivot_settle += 1
        else:
            self.pivot_settle = 0
        if self.pivot_settle >= PIVOT_SETTLE_TICKS:
            self._end_pivot(yaw, "yaw on target")
            return self._cmd(0.0, 0.0, reason="pivot done (yaw)", phase="rowfit_" + self.state.lower(),
                             new_frame=new_frame)
        if self.pivot_t > PIVOT_TIMEOUT_S:
            self.state = STATE_LANE
            self._event(f"PIVOT timeout after {self.pivot_t:.1f} s, yaw err {math.degrees(err):+.1f}deg — braking")
            return self._cmd(0.0, 0.0, reason="pivot timeout", phase="rowfit_stop", brake=True,
                             new_frame=new_frame)
        want = PIVOT_K_YAW * err
        want = max(-PIVOT_W_MAX, min(PIVOT_W_MAX, want))
        if abs(err) > PIVOT_TOL_RAD and abs(want) < PIVOT_W_MIN:
            want = math.copysign(PIVOT_W_MIN, err)
        # never ask for more than we can stop from before the target
        w_stop = math.sqrt(2.0 * PIVOT_ALPHA * abs(err))
        want = max(-w_stop, min(w_stop, want))
        lim = PIVOT_ALPHA * dt
        self.pivot_w += max(-lim, min(lim, want - self.pivot_w))
        left, right = self._wheels(0.0, self.pivot_w)
        return self._cmd(left, right, reason=f"pivot err {math.degrees(err):+.1f}deg",
                         phase="rowfit_pivot", new_frame=new_frame)

    def step(self, est=None, *, new_frame: bool = False, dt: float = 0.008, frame_dt: float = 0.32,
             yaw: float | None = None) -> dict:
        """One physics tick. ``yaw``: IMU yaw (rad, + = left/CCW). Without it a
        corner is still approached, but the robot brakes at the pivot point."""
        if new_frame:
            self._corner_frame(est, yaw)
        if self.state == STATE_PIVOT:
            if yaw is None:
                self.state = STATE_LANE
                self.frames_lost = LOST_FRAMES_STOP
            else:
                return self._pivot_tick(est, new_frame, dt, float(yaw))
        if self.state == STATE_APPROACH:
            return self._approach_tick(est, new_frame, dt, frame_dt, yaw)
        if self.state == STATE_JUNCTION:
            return self._junction_tick(est, new_frame, dt, yaw)
        if self.state == STATE_GAP:
            return self._gap_tick(est, new_frame, dt, yaw)
        if self.state == STATE_LOOK:
            return self._look_tick(est, new_frame, dt, yaw)
        if self.state == STATE_STOPPED:
            return self._cmd(0.0, 0.0, reason="stopped (no lane after the blind crossing)", phase="rowfit_stop",
                             brake=True, new_frame=new_frame)
        if new_frame:
            self.on_frame(est, frame_dt)
        if self.jn_slow_m > 0.0 or self.jn_cooldown_m > 0.0:
            self.target_speed = min(self.target_speed, V_JUNCTION_M_S)
        if self.state == STATE_REACQUIRE:
            self.target_speed = min(self.target_speed, V_REACQUIRE_M_S)
        if self.obstacle_v_cap is not None:
            self.target_speed = min(self.target_speed, max(0.0, float(self.obstacle_v_cap)))
        if self.frames_lost >= LOST_FRAMES_STOP:
            if self.state == STATE_REACQUIRE and not self._reacq_fail_logged:
                self._reacq_fail_logged = True
                self._event(f"REACQUIRE failed — no lane {self.frames_lost} frames after the pivot, braking")
            return {
                "left": 0.0,
                "right": 0.0,
                "brake": True,
                "error": 0.0,
                "steer": 0.0,
                "reason": f"rowfit — lane lost {self.frames_lost} frames",
                "phase": "rowfit_stop",
                "error_source": "rowfit",
                "target_speed": 0.0,
                "kappa_cmd": 0.0,
                "new_frame": bool(new_frame),
                **self.corner_fields(),
            }
        v = self.gov.step(self.target_speed, dt)
        if self.cooldown_m > 0.0:
            self.cooldown_m = max(0.0, self.cooldown_m - v * dt)
        if self.jn_cooldown_m > 0.0:
            self.jn_cooldown_m = max(0.0, self.jn_cooldown_m - v * dt)
        if self.jn_slow_m > 0.0:
            self.jn_slow_m = max(0.0, self.jn_slow_m - v * dt)
        raw = steer_for_curvature(self.kappa_cmd, v, self.k_steer)
        raw = max(-STEER_CAP, min(STEER_CAP, raw))
        steer = self.filter.step(raw, dt)
        left, right = wheels_from_steer(
            steer, cruise=v / WHEEL_RADIUS_M, k_steer=self.k_steer, turn_slow=0.0
        )
        if self.state == STATE_REACQUIRE:
            phase = "rowfit_reacquire"
        else:
            phase = "rowfit" if self.frames_lost == 0 else "rowfit_hold"
        return self._cmd(left, right, steer=steer, reason="rowfit keep", phase=phase, v=v, new_frame=new_frame)

    def _approach_tick(self, est, new_frame: bool, dt: float, frame_dt: float, yaw) -> dict:
        if new_frame:
            # steer on the (clipped) lane fit; missing / held frames do not count
            # toward the lost-lane brake here: the lane is SUPPOSED to end ahead.
            if est is not None and (est.valid or getattr(est, "held", False)) and est.confidence >= MIN_CONF:
                k_pp, d2 = self._pursuit(est, max(self.gov.v, 0.1))
                self.kappa_cmd = max(-KAPPA_MAX, min(KAPPA_MAX, k_pp))
                self.lookahead_used = math.sqrt(d2)
            else:
                self.kappa_cmd = 0.0
            self.last_offset = None
        rem = float(self.remaining_m if self.remaining_m is not None else 0.0)
        if rem <= PIVOT_ARRIVE_M:
            if yaw is None:
                self.state = STATE_LANE
                self.frames_lost = LOST_FRAMES_STOP
                self._event("CORNER reached but no IMU yaw — braking at the corner (no pivot)")
                return self._cmd(0.0, 0.0, reason="corner without IMU", phase="rowfit_stop", brake=True,
                                 new_frame=new_frame)
            self._start_pivot(float(yaw))
            return self._pivot_tick(est, False, dt, float(yaw))
        target = min(V_APPROACH_M_S, math.sqrt(2.0 * A_APPROACH_M_S2 * max(0.0, rem - PIVOT_ARRIVE_M)))
        target = max(V_CREEP_M_S, target)
        if self.obstacle_v_cap is not None:
            target = min(target, max(0.0, float(self.obstacle_v_cap)))
        self.target_speed = target
        # this profile can need more than the governor's slow-in decel: follow it directly
        if self.gov.v > target:
            self.gov.v = max(target, self.gov.v - max(DECEL_M_S2, A_APPROACH_M_S2 * 2.0) * dt)
        else:
            self.gov.step(target, dt)
        v = self.gov.v
        self.remaining_m = rem - v * dt
        raw = steer_for_curvature(self.kappa_cmd, v, self.k_steer)
        raw = max(-STEER_CAP, min(STEER_CAP, raw))
        steer = self.filter.step(raw, dt)
        left, right = wheels_from_steer(steer, cruise=v / WHEEL_RADIUS_M, k_steer=self.k_steer, turn_slow=0.0)
        return self._cmd(left, right, steer=steer, reason=f"corner approach {rem:.2f} m to pivot",
                         phase="rowfit_approach", v=v, new_frame=new_frame)

    # ------------------------------------------------------------------
    # intersections
    # ------------------------------------------------------------------
    def _lane_yaw_update(self, est, yaw, gain: float = 0.3) -> None:
        if est is None or not est.valid or yaw is None:
            return
        ly = float(yaw) - float(est.heading_rad)
        if self.lane_yaw is None:
            self.lane_yaw = ly
        else:
            self.lane_yaw += gain * math.atan2(math.sin(ly - self.lane_yaw), math.cos(ly - self.lane_yaw))

    def _junction_lane_frame(self, est, yaw) -> None:
        j = getattr(est, "junction", None) if est is not None else None
        if j is None or self.jn_cooldown_m > 0.0:
            return
        if j.kind.startswith("opening"):
            # one side opens, the other line still runs straight: a branch or a
            # corner's inner line. Slow down in time for the side-branch decision.
            other = j.events.get("R" if j.kind == "opening_L" else "L", ("", None))[0]
            if other == "CONT" and j.near_m <= OPENING_SLOW_NEAR_M:
                self.jn_slow_m = max(self.jn_slow_m, OPENING_SLOW_M)
            return
        if j.kind not in _TURN_KINDS or not j.seen:
            return
        self.state = STATE_JUNCTION
        self.junction = j
        self.jn_near_m = float(j.near_m)
        self.jn_center_m = float(j.center_m)
        self.jn_missing = 0
        self.jn_count += 1
        self.lane_w_m = self._lane_width(est)
        self.lane_yaw = None
        if yaw is not None and est is not None and (est.valid or getattr(est, "held", False)):
            self.lane_yaw = float(yaw) - float(est.heading_rad)
        far = "" if j.far_m is None else f", far side {j.far_m:.2f} m"
        self._event(
            f"JUNCTION #{self.jn_count} seen {j.kind} near {j.near_m:.2f} m centre {j.center_m:.2f} m{far} "
            f"openings {j.openings()} conf {j.confidence:.2f} — slowing to {V_JUNCTION_M_S:.2f} m/s"
        )
        self._junction_maybe_decide(est, yaw)

    def _junction_approach_frame(self, est, yaw) -> None:
        j = getattr(est, "junction", None) if est is not None else None
        if j is not None and j.kind in _TURN_KINDS and self.junction is not None \
                and _family(j.kind) == _family(self.junction.kind):
            if (j.kind, j.openings()) != (self.junction.kind, self.junction.openings()):
                self._event(f"JUNCTION #{self.jn_count} now {j.kind} openings {j.openings()} "
                            f"near {j.near_m:.2f} m centre {j.center_m:.2f} m")
            self.junction = j
            self.jn_near_m = float(j.near_m)
            self.jn_center_m = float(j.center_m)
            self.jn_missing = 0
        else:
            self.jn_missing += 1
            if self.jn_missing >= JUNCTION_LOST_FRAMES and (self.jn_near_m or 0.0) > 0.6:
                self._event(f"JUNCTION #{self.jn_count} lost before the decision — back to LANE")
                self.state = STATE_LANE
                self.junction = None
                self.jn_center_m = None
                self.jn_count -= 1
                return
        self._lane_yaw_update(est, yaw)
        self._junction_maybe_decide(est, yaw)

    def _junction_maybe_decide(self, est, yaw) -> None:
        j = self.junction
        near = float(self.jn_near_m if self.jn_near_m is not None else 0.0)
        known = j.straight is not None or j.kind in ("side_L", "side_R", "t_end", "dead_end")
        if not known and near > JUNCTION_DECIDE_M:
            return
        straight = j.straight
        if j.kind in ("t_end", "dead_end") and straight is not False:
            # a T-end / dead end has a line across the way ahead: never straight,
            # whatever the openings say (RBM_ROUTE=S falls back to L, else R)
            self._event(f"JUNCTION #{self.jn_count} {j.kind}: straight forced off (was {straight})")
            straight = False
        choice, note = self.policy.choose(j.left, straight, j.right)
        self.route_choice = choice or "-"
        rec = {"n": self.jn_count, "kind": j.kind, "openings": j.openings(), "choice": choice, "note": note,
               "near_m": round(near, 3), "center_m": round(float(self.jn_center_m or 0.0), 3)}
        self.junctions.append(rec)
        self._event(f"ROUTE junction #{self.jn_count} {j.kind} [{j.openings()}] -> {choice or 'STOP'} ({note})")
        if choice is None:
            self.state = STATE_STOPPED
            self._event("JUNCTION dead end (no opening) — STOPPED")
            return
        if choice in ("L", "R"):
            self.state = STATE_APPROACH
            self.turn_src = "junction"
            self.corner_dir = 1 if choice == "L" else -1
            self.corner_dist_m = float(j.near_m)
            self.corner_conf = float(j.confidence)
            self.remaining_m = float(self.jn_center_m)
            self._event(f"TURN {choice} at the junction centre, pivot point in {self.remaining_m:.2f} m"
                        + ("" if yaw is not None else " — NO IMU yaw: will stop there"))
            return
        if j.kind in ("side_L", "side_R"):
            self.state = STATE_LANE
            w_cross = 2.0 * max(0.0, float(self.jn_center_m) - near)
            self.jn_cooldown_m = float(self.jn_center_m) + 0.5 * w_cross + 0.4
            self._event(f"STRAIGHT past the {'left' if j.kind == 'side_L' else 'right'} branch "
                        f"(single-line lane for ~{self.jn_cooldown_m:.1f} m)")
            self.junction = None
            self.jn_center_m = None
            return
        self._start_gap(est, yaw)

    def _start_gap(self, est, yaw) -> None:
        self.state = STATE_GAP
        self.gap_travel_m = 0.0
        self.blind_m = 0.0
        self.gap_ok = 0
        self.gap_seen = {}
        base = self.lane_yaw
        if base is None and yaw is not None:
            base = float(yaw) - (float(est.heading_rad) if est is not None and est.valid else 0.0)
        self.gap_yaw = base
        self._event(
            f"GAP_CROSS start: near line {float(self.jn_near_m or 0.0):.2f} m, holding heading "
            + ("(no IMU: wheels straight)" if base is None else f"{math.degrees(base):+.1f}deg")
            + f", creep {V_GAP_M_S:.2f} m/s, max blind {self.max_blind_m:.2f} m"
        )

    def _junction_tick(self, est, new_frame: bool, dt: float, yaw) -> dict:
        if self.state != STATE_JUNCTION:  # decided this frame
            return self.step(None, new_frame=False, dt=dt, yaw=yaw)
        v = self._drive(est, new_frame, dt, yaw, V_JUNCTION_M_S)
        self.jn_near_m = (self.jn_near_m or 0.0) - v * dt
        self.jn_center_m = (self.jn_center_m or 0.0) - v * dt
        if self.jn_near_m <= JUNCTION_DECIDE_M - 0.1 and self.state == STATE_JUNCTION:
            self._junction_maybe_decide(est, yaw)  # forced by odometry (vision lost the junction)
        return self._last_drive_cmd(f"junction approach {self.junction.kind if self.junction else ''}",
                                    "rowfit_junction", v, new_frame)

    def _drive(self, est, new_frame: bool, dt: float, yaw, v_cap: float, *, hold_yaw: float | None = None) -> float:
        """Lane pursuit on the (clipped) fit if valid, else IMU heading hold; returns v."""
        if self.obstacle_v_cap is not None:
            v_cap = min(v_cap, max(0.0, float(self.obstacle_v_cap)))
        if new_frame:
            if hold_yaw is None and est is not None and est.valid and est.confidence >= MIN_CONF:
                k_pp, d2 = self._pursuit(est, max(self.gov.v, 0.1))
                self.kappa_cmd = max(-KAPPA_MAX, min(KAPPA_MAX, k_pp))
                self.lookahead_used = math.sqrt(d2)
                self._hold = False
            else:
                self._hold = True
            self.last_offset = None
        self.target_speed = v_cap
        if self.gov.v > v_cap:
            self.gov.v = max(v_cap, self.gov.v - DECEL_M_S2 * dt)
        else:
            self.gov.step(v_cap, dt)
        v = self.gov.v
        tgt = hold_yaw if hold_yaw is not None else self.lane_yaw
        if getattr(self, "_hold", False) or hold_yaw is not None:
            w = 0.0
            if yaw is not None and tgt is not None:
                err = math.atan2(math.sin(tgt - yaw), math.cos(tgt - yaw))
                w = max(-GAP_W_MAX, min(GAP_W_MAX, GAP_K_YAW * err))
            self.kappa_cmd = w / max(v, 0.05)
            left, right = self._wheels(v, w)
            self.filter = SteerFilter()
            self._drive_out = (left, right, 0.0)
        else:
            raw = steer_for_curvature(self.kappa_cmd, v, self.k_steer)
            raw = max(-STEER_CAP, min(STEER_CAP, raw))
            steer = self.filter.step(raw, dt)
            left, right = wheels_from_steer(steer, cruise=v / WHEEL_RADIUS_M, k_steer=self.k_steer, turn_slow=0.0)
            self._drive_out = (left, right, steer)
        return v

    def _last_drive_cmd(self, reason: str, phase: str, v: float, new_frame: bool) -> dict:
        left, right, steer = self._drive_out
        return self._cmd(left, right, steer=steer, reason=reason, phase=phase, v=v, new_frame=new_frame)

    def _gap_lane_ok(self, est) -> bool:
        if est is None or not est.valid or getattr(est, "held", False) or est.confidence < MIN_CONF:
            return False
        if est.n_left < 4 or est.n_right < 4:
            return False
        if abs(est.heading_rad) > GAP_MAX_HEADING_RAD or abs(est.offset_m) > GAP_MAX_OFFSET_M:
            return False
        w0 = float(self.lane_w_m or 1.30)
        w = float(getattr(est, "lane_width_m", w0) or w0)
        return abs(w - w0) <= GAP_WIDTH_TOL * w0

    def _gap_frame(self, est, yaw) -> None:
        view = getattr(est, "view", None) if est is not None else None
        if view:
            for k in ("ahead", "left", "right"):
                if view.get(k) is not None and k not in self.gap_seen and self.blind_m > GAP_MIN_BLIND_M:
                    self.gap_seen[k] = round(self.blind_m, 2)
                    self._event(f"GAP sees yellow {k} at {view[k]:.2f} m (after {self.blind_m:.2f} m blind)"
                                + (" — side line = opening" if k != "ahead" else ""))
        if self.blind_m < GAP_MIN_BLIND_M:
            self.gap_ok = 0
            return
        sides = [view.get(k) for k in ("left", "right")] if view else []
        near_ok = bool(sides) and all(v is not None and v <= GAP_REACQ_NEAR_M for v in sides)
        if self._gap_lane_ok(est) and near_ok:
            self.gap_ok += 1
        else:
            self.gap_ok = 0
            if self.tracker is not None and hasattr(self.tracker, "begin_reacquire"):
                self.tracker.begin_reacquire(2)  # no old-lane prior: take the next lane fresh
        if self.gap_ok >= GAP_OK_FRAMES:
            self._event(f"GAP reacquire after {self.blind_m:.2f} m blind — lane ahead off {est.offset_m * 100:+.0f}cm "
                        f"hd {math.degrees(est.heading_rad):+.1f}deg w {est.lane_width_m:.2f} m — REACQUIRE")
            if self.junctions:
                self.junctions[-1]["blind_m"] = round(self.blind_m, 3)
            self.state = STATE_REACQUIRE
            self.reacq_ok = 0
            self.reacq_frames = 0
            self.frames_lost = 0
            self.last_offset = None
            self.target_speed = V_REACQUIRE_M_S
            if self.tracker is not None and hasattr(self.tracker, "begin_reacquire"):
                self.tracker.begin_reacquire(REACQUIRE_RELAX_FRAMES)
            return
        if view and view.get("bar") is not None and not self._gap_lane_ok(est) and self.blind_m > GAP_MIN_BLIND_M:
            # a line straight across the way ahead: this crossing is a T after all
            bar = float(view["bar"])
            want = self.junctions[-1]["note"] if self.junctions else ""
            choice = "R" if ("wanted R" in want) else "L"
            self.state = STATE_APPROACH
            self.turn_src = "gap"
            self.corner_dir = 1 if choice == "L" else -1
            self.lane_yaw = self.gap_yaw
            self.remaining_m = max(0.0, bar - 0.5 * float(self.lane_w_m or 1.30))
            self.route_choice = choice
            self._event(f"GAP line across ahead at {bar:.2f} m after {self.blind_m:.2f} m blind — straight blocked, "
                        f"turning {choice}, pivot in {self.remaining_m:.2f} m")

    def _gap_tick(self, est, new_frame: bool, dt: float, yaw) -> dict:
        if self.state != STATE_GAP:
            return self.step(None, new_frame=False, dt=dt, yaw=yaw)
        near = float(self.jn_near_m or 0.0) - self.gap_travel_m
        cap = V_JUNCTION_M_S if near > 0.15 else V_GAP_M_S
        # before the near line: steer on the clipped lane; past it: IMU heading hold
        hold = self.gap_yaw if (near <= 0.15 and self.gap_yaw is not None) else None
        v = self._drive(est, new_frame, dt, yaw, cap, hold_yaw=hold)
        if hold is None and yaw is not None and new_frame:
            self._lane_yaw_update(est, yaw)
            if self.lane_yaw is not None:
                self.gap_yaw = self.lane_yaw
        self.gap_travel_m += v * dt
        if near <= 0.0:
            self.blind_m += v * dt
        if self.blind_m >= self.max_blind_m:
            self._event(f"GAP no lane after {self.blind_m:.2f} m blind (max {self.max_blind_m:.2f} m)"
                        + (" — LOOK_AROUND" if (self.look_around and yaw is not None) else " — STOPPED"))
            if self.junctions:
                self.junctions[-1]["blind_m"] = round(self.blind_m, 3)
                self.junctions[-1]["gave_up"] = True
            if self.look_around and yaw is not None:
                self.state = STATE_LOOK
                self.look_i = 0
                self.look_seen = []
                self.pivot_w = 0.0
                self.gov.v = 0.0
                base = self.gap_yaw if self.gap_yaw is not None else float(yaw)
                self._look_base = base
                self.yaw_target = base + LOOK_ANGLES_RAD[0]
                return self._cmd(0.0, 0.0, reason="gap: max blind distance — look-around", phase="rowfit_look",
                                 new_frame=new_frame)
            self.state = STATE_STOPPED
            return self._cmd(0.0, 0.0, reason="gap: max blind distance", phase="rowfit_stop", brake=True,
                             new_frame=new_frame)
        return self._last_drive_cmd(f"gap cross blind {self.blind_m:.2f} m", "rowfit_gap", v, new_frame)

    def _look_frame(self, est) -> None:
        view = getattr(est, "view", None) if est is not None else None
        if view and any(view.get(k) is not None for k in ("ahead", "left", "right")):
            near = min(view[k] for k in ("ahead", "left", "right") if view.get(k) is not None)
            self.look_seen.append((self.look_i, round(near, 2)))

    def _look_tick(self, est, new_frame: bool, dt: float, yaw) -> dict:
        if yaw is None:
            self.state = STATE_STOPPED
            return self._cmd(0.0, 0.0, reason="look-around needs IMU", phase="rowfit_stop", brake=True,
                             new_frame=new_frame)
        err = math.atan2(math.sin(self.yaw_target - yaw), math.cos(self.yaw_target - yaw))
        if abs(err) < math.radians(2.0) and abs(self.pivot_w) < 0.08:
            self.look_i += 1
            if self.look_i >= len(LOOK_ANGLES_RAD):
                names = ("left (+90)", "right (-90)", "ahead")
                seen = {names[i]: d for i, d in self.look_seen}
                rep = ", ".join(f"{n}: {'yellow at %.2f m' % seen[n] if n in seen else 'nothing'}" for n in names)
                self._event(f"LOOK_AROUND done — {rep}. STOPPED (no lane within {self.max_blind_m:.2f} m blind); "
                            "drive it on by hand / teleop")
                self.state = STATE_STOPPED
                return self._cmd(0.0, 0.0, reason="look-around done", phase="rowfit_stop", brake=True,
                                 new_frame=new_frame)
            self.yaw_target = self._look_base + LOOK_ANGLES_RAD[self.look_i]
            err = math.atan2(math.sin(self.yaw_target - yaw), math.cos(self.yaw_target - yaw))
        want = max(-LOOK_W_M_S, min(LOOK_W_M_S, PIVOT_K_YAW * err))
        w_stop = math.sqrt(2.0 * PIVOT_ALPHA * abs(err))
        want = max(-w_stop, min(w_stop, want))
        lim = PIVOT_ALPHA * dt
        self.pivot_w += max(-lim, min(lim, want - self.pivot_w))
        left, right = self._wheels(0.0, self.pivot_w)
        return self._cmd(left, right, reason=f"look-around {self.look_i + 1}/{len(LOOK_ANGLES_RAD)}",
                         phase="rowfit_look", new_frame=new_frame)


_TURN_KINDS = ("t_end", "side_L", "side_R", "plus", "gap_straight", "lane_lost", "dead_end")


def _family(kind: str) -> str:
    if kind in ("plus", "t_end", "gap_straight", "lane_lost", "dead_end"):
        return "cross"
    return kind[-1] if kind.startswith(("side", "opening")) else kind
