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

    def reset(self) -> None:
        tracker = self.tracker
        self.__init__(k_steer=self.k_steer, ld_min=self.ld_min, ld_max=self.ld_max)
        self.tracker = tracker

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
            return
        if self.state == STATE_APPROACH:
            if corner is not None and corner.direction == self.corner_dir and corner.confidence >= 0.4:
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
        if new_frame:
            self.on_frame(est, frame_dt)
        if self.state == STATE_REACQUIRE:
            self.target_speed = min(self.target_speed, V_REACQUIRE_M_S)
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
