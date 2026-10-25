"""Row-fit lane keep: steer + speed from a fitted lane model.

Used by the Webots controller. ``RBM_LANE_MODE=gap`` is ignored; the
pixel-gap keeper has been removed.

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
come from ``lane_keep.wheels_from_steer`` (same limits as before).
Sign conventions match ``lane_keep``: steer > 0 = yaw right.

The steps live in sibling modules. This module re-exports what callers
already import. Coast, ``curve_speed``, and ``MAX_WHEEL_RAD_S`` stay
together in ``rowfit_speed``.
"""

from __future__ import annotations

from src.rowfit_speed import *  # noqa: F403
from src.rowfit_speed import _RowfitSpeed
from src.rowfit_lane import _RowfitLane, lookahead_bounds_from_coverage
from src.rowfit_junction import _RowfitJunction, _TURN_KINDS, _family
from src.rowfit_corner import _RowfitCorner


def classify_steer_source(state: str, est, *, brake: bool, map_owns: bool) -> str:
    """Which source commanded the wheels.

    A loaded track does not steer a straight. It owns distance only while
    a turn is already committed (approach or junction) and GPS is on that
    road. A rejected fit is ``held``, not camera and not track. 0.30 m/s
    is that hold cap.
    """
    if brake or state == STATE_STOPPED:
        return STEER_STOPPED
    if state == STATE_PIVOT:
        return STEER_PIVOT
    if state == STATE_RECOVER:
        return STEER_RECOVER
    if state in (STATE_GAP, STATE_LOOK):
        return STEER_BLIND
    if map_owns and state in (STATE_APPROACH, STATE_JUNCTION):
        return STEER_TRACK
    if est is not None and getattr(est, "held", False):
        return STEER_HELD
    if est is not None and getattr(est, "valid", False):
        return STEER_CAMERA
    return STEER_HELD


class RowfitController(_RowfitLane, _RowfitCorner, _RowfitSpeed, _RowfitJunction):
    def __init__(
        self,
        *,
        k_steer: float = DEFAULT_K_STEER,
        ld_min: float = LD_MIN_M,
        ld_max: float = LD_MAX_M,
        v_cruise: float | None = None,
        v_straight: float | None = None,
        pivot_w_max: float | None = None,
        turn_brake=None,
        coast_into_turn=None,
    ) -> None:
        self.k_steer = float(k_steer)
        self.ld_min = float(ld_min)
        self.ld_max = float(ld_max)
        # None keeps the module constants, so existing callers and tests
        # are unchanged. config.yaml's controller section supplies these
        # when the Webots runtime constructs the controller.
        self.v_cruise = V_CRUISE_M_S if v_cruise is None else float(v_cruise)
        self.v_straight = V_STRAIGHT_M_S if v_straight is None else float(v_straight)
        self.pivot_w_max = PIVOT_W_MAX if pivot_w_max is None else float(pivot_w_max)
        self.filter = SteerFilter()
        self.gov = SpeedGovernor()
        self.kappa_cmd = 0.0
        self.target_speed = V_MIN_M_S
        self.lookahead_used = 0.0
        self.frames_lost = 0
        self.last_offset: float | None = None
        self.n_frames = 0
        self.held_frames = 0
        self.straight_frames = 0  # qualifying straightaway frames in a row
        self.straight_boost = False  # target raised to V_STRAIGHT_M_S this frame
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
        self.track = None  # optional TrackGeometry / TrackNetwork (GPS corner distance)
        self.pose = None  # (x, y) world metres when GPS is known
        self._yaw = None
        self._map_dist_logged = False
        self.recover_m = 0.0
        self.recover_t = 0.0
        self._steer_est = None  # last camera frame; held between frames
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
        # Turn-approach braking (``src.physics_speed_limit.TurnBrakeParams``).
        self.turn_brake = turn_brake
        self.coast_into_turn = coast_into_turn
        self._coast_active = False
        self._coast_latched = False  # one arm per approach, cleared at the pivot
        self._coasted_this_approach = False
        self._coast_wh_saved = 0.0
        self._coast_starts = 0
        self._open_turn_id: int | None = None
        self._turn_records: list[str] = []
        self._curve_yaw0: float | None = None
        self._curve_ct = 0.0
        self._seg_n = 0
        self._seg_ct = 0.0
        self._seg_ct_fit = 0.0
        self._seg_len = 0.0
        self._seg_start: tuple[float, float] | None = None
        self._seg_prev: tuple[float, float] | None = None
        self._pivot_lateral_max_m = 0.0
        self._pivot_ct_fit_max_m = 0.0
        self._pivot_entry_v_m_s = 0.0
        self.pivot_integrated_rad = 0.0
        self.pivot_yaw_rate_rad_s = 0.0
        self.pivot_heading_err_rad = 0.0

    def reset(self) -> None:
        tracker, route, mb, la = self.tracker, self.policy.route, self.max_blind_m, self.look_around
        track = self.track
        vc, vs, pw = self.v_cruise, self.v_straight, self.pivot_w_max
        tb = self.turn_brake
        coast = self.coast_into_turn
        self.__init__(
            k_steer=self.k_steer,
            ld_min=self.ld_min,
            ld_max=self.ld_max,
            v_cruise=vc,
            v_straight=vs,
            pivot_w_max=pw,
            turn_brake=tb,
            coast_into_turn=coast,
        )
        self.tracker = tracker
        self.track = track
        self.set_route(route, max_blind_m=mb, look_around=la)

    def set_route(self, route=None, *, max_blind_m: float | None = None, look_around: bool | None = None) -> None:
        from src.route_policy import RoutePolicy

        self.policy = RoutePolicy(route)
        if max_blind_m is not None and max_blind_m > 0:
            self.max_blind_m = float(max_blind_m)
        if look_around is not None:
            self.look_around = bool(look_around)

    def set_track(self, track) -> None:
        """World track for pivot distance. None keeps vision + odometry only."""
        self.track = track

    def drain_events(self) -> list[str]:
        out, self.events = self.events, []
        return out

    def _event(self, msg: str) -> None:
        self.events.append(msg)
        if msg.startswith(("TURN_RECORD", "PIVOT_RECORD", "CURVE_RECORD", "SEGMENT_RECORD")):
            self._turn_records.append(msg)

    def _wheels(self, v: float, w: float) -> tuple[float, float]:
        """Wheel rad/s for forward speed v (m/s) and yaw rate w (rad/s, + = left)."""
        half = 0.5 * w * TRACK_M
        return (v - half) / WHEEL_RADIUS_M, (v + half) / WHEEL_RADIUS_M

    def _map_owns_distance(self) -> bool:
        """True when this tick's remaining distance came from the track."""
        if self.state not in (STATE_APPROACH, STATE_JUNCTION):
            return False
        return self._map_corner_m(self._yaw) is not None

    def _lateral_source(self, est) -> str:
        if est is not None and getattr(est, "held", False):
            return STEER_HELD
        if est is not None and getattr(est, "valid", False):
            return STEER_CAMERA
        return STEER_HELD

    def _stamp_steer(self, cmd: dict) -> dict:
        est = self._steer_est
        source = classify_steer_source(
            self.state,
            est,
            brake=bool(cmd.get("brake")),
            map_owns=self._map_owns_distance(),
        )
        cmd["steer_source"] = source
        cmd["steer_lateral"] = self._lateral_source(est)
        reject = ""
        if est is not None and getattr(est, "held", False):
            reject = str(getattr(est, "reject_reason", "") or "")
        cmd["fit_reject"] = reject
        return cmd

    def _cmd(self, left, right, *, steer=0.0, reason, phase, v=0.0, brake=False, new_frame=False) -> dict:
        return self._stamp_steer({
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
            "straight_boost": bool(self.straight_boost),
            **self.corner_fields(),
        })

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
            "yaw_rate_rad_s": round(self.pivot_yaw_rate_rad_s, 4),
            "pivot_heading_err_rad": round(self.pivot_heading_err_rad, 4),
            "pivot_integrated_deg": round(math.degrees(self.pivot_integrated_rad), 2),
            "coast_active": bool(self._coast_active),
            "coast_wh_saved": round(self._coast_wh_saved, 6),
            "coast_starts": int(self._coast_starts),
        }

    def step(
        self,
        est=None,
        *,
        new_frame: bool = False,
        dt: float = 0.008,
        frame_dt: float = 0.32,
        yaw: float | None = None,
        pose=None,
        yaw_rate_rad_s: float | None = None,
        cross_track_m: float | None = None,
    ) -> dict:
        """One physics tick. ``yaw``: IMU yaw (rad, + = left/CCW). Without it a
        corner is still approached, but the robot brakes at the pivot point.

        ``pose`` is ``(x, y)`` in world metres. With a track set, it is the
        pivot distance while a turn is already committed. It does not start
        a turn by itself.

        ``cross_track_m`` is true position vs the lane centreline (same source
        as the lap ``ct_max_m``). When it is missing, segment and pivot records
        do not fall back to the camera lane-fit offset.
        """
        if pose is not None:
            self.pose = (float(pose[0]), float(pose[1]))
        if yaw is not None:
            self._yaw = yaw
        self._observe_segment(self.pose, est, cross_track_m=cross_track_m)
        self._note_curve_yaw(yaw, est, cross_track_m=cross_track_m)
        if new_frame:
            self._steer_est = est
            self._corner_frame(est, yaw)
        if self.state != STATE_LANE:
            self._straight_reset()  # the bump never outlives plain LANE following
        if self.state == STATE_PIVOT:
            if yaw is None:
                held = float(self._yaw) if self._yaw is not None else 0.0
                self._event("PIVOT yaw lost — closing the pivot record")
                self._end_pivot(held, "yaw lost")
                self.frames_lost = LOST_FRAMES_STOP
                return self._cmd(0.0, 0.0, reason="pivot yaw lost", phase="rowfit_stop", brake=True,
                                 new_frame=new_frame)
            else:
                return self._pivot_tick(
                    est,
                    new_frame,
                    dt,
                    float(yaw),
                    yaw_rate_rad_s=yaw_rate_rad_s,
                    cross_track_m=cross_track_m,
                )
        if self.state == STATE_APPROACH:
            return self._approach_tick(
                est, new_frame, dt, frame_dt, yaw, yaw_rate_rad_s=yaw_rate_rad_s
            )
        if self.state == STATE_JUNCTION:
            return self._junction_tick(est, new_frame, dt, yaw)
        if self.state == STATE_GAP:
            return self._gap_tick(est, new_frame, dt, yaw)
        if self.state == STATE_LOOK:
            return self._look_tick(est, new_frame, dt, yaw)
        if self.state == STATE_RECOVER:
            return self._recover_tick(est, new_frame, dt, yaw)
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
            if self.state == STATE_REACQUIRE:
                if not self._reacq_fail_logged:
                    self._reacq_fail_logged = True
                    self._event(
                        f"REACQUIRE failed — no lane {self.frames_lost} frames after the pivot "
                        "— RECOVER (creep along the exit heading, do not park)"
                    )
                self._enter_recover()
                return self._recover_tick(est, new_frame, dt, yaw)
            return self._stamp_steer({
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
            })
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


# Bases stay on the class. The names are not part of the import surface.
del _RowfitLane, _RowfitCorner, _RowfitSpeed, _RowfitJunction

