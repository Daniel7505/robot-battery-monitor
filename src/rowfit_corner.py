"""Sharp corners, the pivot, and the turn / segment log."""

from __future__ import annotations

from src.rowfit_speed import *  # noqa: F403
from src.rowfit_junction import _TURN_KINDS, _family


class _RowfitCorner:
    def turn_log_line(self) -> str:
        """One checksum line. Live TURN_RECORD / PIVOT_RECORD lines stay singular."""
        turns = sum(1 for row in self._turn_records if row.startswith("TURN_RECORD"))
        pivots = sum(1 for row in self._turn_records if row.startswith("PIVOT_RECORD"))
        curves = sum(1 for row in self._turn_records if row.startswith("CURVE_RECORD"))
        segments = sum(1 for row in self._turn_records if row.startswith("SEGMENT_RECORD"))
        return (
            f"TURN_LOG turns={turns} pivots={pivots} curves={curves} "
            f"segments={segments} controller_pivots={self.pivots}"
        )

    def note_cross_track(self, ct_m: float | None) -> None:
        """Peak |true cross-track| on the open straight (GPS vs centreline).

        Ignored during a pivot. Camera lane-fit offset is tracked separately
        via :meth:`note_cross_track_fit`.
        """
        if ct_m is None or self.state == STATE_PIVOT:
            return
        try:
            val = abs(float(ct_m))
        except (TypeError, ValueError):
            return
        if val > self._seg_ct:
            self._seg_ct = val

    def note_cross_track_fit(self, offset_m: float | None) -> None:
        """Peak |camera lane-fit offset| on the open straight (diagnostic)."""
        if offset_m is None or self.state == STATE_PIVOT:
            return
        try:
            val = abs(float(offset_m))
        except (TypeError, ValueError):
            return
        if val > self._seg_ct_fit:
            self._seg_ct_fit = val

    def _note_pivot_cross_track(self, ct_m: float | None) -> None:
        if ct_m is None:
            return
        try:
            val = abs(float(ct_m))
        except (TypeError, ValueError):
            return
        if val > self._pivot_lateral_max_m:
            self._pivot_lateral_max_m = val

    def _note_pivot_cross_track_fit(self, offset_m: float | None) -> None:
        if offset_m is None:
            return
        try:
            val = abs(float(offset_m))
        except (TypeError, ValueError):
            return
        if val > self._pivot_ct_fit_max_m:
            self._pivot_ct_fit_max_m = val

    @staticmethod
    def _ct_fit_suffix(fit_max: float) -> str:
        return f" ct_fit_max_m={fit_max:.3f}" if fit_max > 1e-4 else ""

    def _observe_segment(self, pose, est, *, cross_track_m: float | None = None) -> None:
        if self.state == STATE_PIVOT:
            return
        if cross_track_m is not None:
            self.note_cross_track(cross_track_m)
        if est is not None:
            off = getattr(est, "offset_m", None)
            if off is not None:
                self.note_cross_track_fit(off)
        if pose is None:
            return
        try:
            xy = (float(pose[0]), float(pose[1]))
        except (TypeError, ValueError, IndexError):
            return
        if self._seg_start is None:
            self._seg_start = xy
        if self._seg_prev is not None:
            self._seg_len += math.hypot(xy[0] - self._seg_prev[0], xy[1] - self._seg_prev[1])
        self._seg_prev = xy

    def _close_segment(self, why: str) -> None:
        """End the straight that leads into a pivot, or the leg after the last one."""
        empty = self._seg_len < 1e-3 and self._seg_ct <= 0.0 and self._seg_start is None
        if empty and (why != "pivot" or self._seg_n > 0):
            return
        start = self._seg_start
        end = self._seg_prev
        start_txt = f"({start[0]:.3f},{start[1]:.3f})" if start is not None else "unknown"
        end_txt = f"({end[0]:.3f},{end[1]:.3f})" if end is not None else "unknown"
        self._seg_n += 1
        self._event(
            f"SEGMENT_RECORD seg={self._seg_n} after_turn={self.pivots} "
            f"ct_max_m={self._seg_ct:.3f}{self._ct_fit_suffix(self._seg_ct_fit)} "
            f"length_m={self._seg_len:.3f} "
            f"start_pos={start_txt} end_pos={end_txt} close={why}"
        )
        self._seg_ct = 0.0
        self._seg_ct_fit = 0.0
        self._seg_len = 0.0
        self._seg_start = end
        self._seg_prev = end

    def finish_turn_log(self) -> list[str]:
        """Close the last straight and return lines to print before process kill."""
        if self.state != STATE_PIVOT:
            self._close_segment("mission")
        fresh = self.drain_events()
        return fresh + [self.turn_log_line()]

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
                if yaw is not None and self._est_usable_for_lane_yaw(est):
                    self.lane_yaw = float(yaw) - float(est.heading_rad)
                self.remaining_m = self._prefer_map_distance(self.remaining_m, yaw)
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
                    # Vision centre may be one frame late: never push remaining back out
                    # (that delayed the pivot ~v*frame_dt ≈ 8 cm at 0.25 m/s).
                    if self._vision_distance_ok(est):
                        prev = self.remaining_m
                        self.remaining_m = float(j.center_m)
                        self._cap_remaining(prev)
            elif (self.turn_src == "corner" and corner is not None and corner.direction == self.corner_dir
                  and corner.confidence >= 0.4 and self._vision_distance_ok(est)):
                prev = self.remaining_m
                self._take_corner(corner)
                self._cap_remaining(prev)
            if yaw is not None and self._est_usable_for_lane_yaw(est):
                # Prefer held / high-conf fits; ignore low-conf fresh fits near the junction.
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
        """Live near-window width when both lines are in this frame.

        The tracker's smoothed width, then 1.30 m, are the fallback for a
        frame that can see only one line (or none).
        """
        if est is not None and getattr(est, "width_live", False):
            w = float(getattr(est, "lane_width_m", 0.0) or 0.0)
            if 0.4 < w < 3.0:
                return w
        if self.tracker is not None and getattr(self.tracker, "lane_w", None):
            w = float(self.tracker.lane_w)
            if 0.4 < w < 3.0:
                return w
        w = float(getattr(est, "lane_width_m", 0.0) or 0.0) if est is not None else 0.0
        return w if 0.4 < w < 3.0 else 1.30


    @staticmethod
    def _est_usable_for_approach_steer(est) -> bool:
        """May this estimate steer while lining up a turn?

        Prefer held (odometry-propagated last-good lane). A fresh fit must clear
        ``APPROACH_STEER_MIN_CONF`` so low-conf cross-aisle contamination is ignored.
        """
        if est is None:
            return False
        conf = float(getattr(est, "confidence", 0.0) or 0.0)
        if getattr(est, "held", False):
            return conf >= MIN_CONF
        return bool(getattr(est, "valid", False)) and conf >= APPROACH_STEER_MIN_CONF

    @staticmethod
    def _est_usable_for_lane_yaw(est) -> bool:
        """May this estimate update ``lane_yaw`` while lining up a turn?"""
        if est is None:
            return False
        conf = float(getattr(est, "confidence", 0.0) or 0.0)
        if getattr(est, "held", False):
            return conf >= MIN_CONF
        return bool(getattr(est, "valid", False)) and conf >= APPROACH_YAW_MIN_CONF

    def _take_corner(self, corner) -> None:
        self.corner_dist_m = float(corner.distance_m)
        self.corner_conf = float(corner.confidence)
        # Pivot point: the corner's centre-line vertex = outer line minus half a
        # lane. Turning 90 deg there puts the axle on the new leg's centre line.
        self.remaining_m = self.corner_dist_m - 0.5 * float(self.lane_w_m or 1.30)

    def _vision_distance_ok(self, est) -> bool:
        """A short or rejected view must not move the pivot point.

        A held fit with a normal lookahead may still refine it (one frame
        late), but only by ``VISION_SHRINK_MAX_M``.
        """
        if est is None:
            return False
        reason = str(getattr(est, "reject_reason", "") or "")
        if reason.startswith("short view"):
            return False
        look = getattr(est, "lookahead_m", None)
        if look is not None and float(look) < 0.50:
            return False
        return True

    def _cap_remaining(self, prev) -> None:
        """Keep a vision update from jumping the pivot forward.

        Never push remaining back out. Never pull it in by more than
        ``VISION_SHRINK_MAX_M`` in one frame. The track, when it owns the
        distance, is applied in the approach tick and is left alone here.
        """
        if self._map_corner_m(self._yaw) is not None:
            return
        new = self.remaining_m
        if prev is None or new is None:
            return
        shrunk = min(float(prev), float(new))
        self.remaining_m = max(float(prev) - VISION_SHRINK_MAX_M, shrunk)

    def _map_corner_m(self, yaw) -> float | None:
        """Track distance to the next corner, or None when pose/track don't apply."""
        if self.track is None or self.pose is None or yaw is None:
            return None
        from src.track_geometry import distance_to_next_corner

        return distance_to_next_corner(self.track, self.pose[0], self.pose[1], float(yaw))

    def _prefer_map_distance(self, vision_m, yaw) -> float:
        """Use the track when the camera distance is early. Otherwise keep vision."""
        vision = 0.0 if vision_m is None else float(vision_m)
        mapped = self._map_corner_m(yaw)
        if mapped is not None and mapped > vision + EARLY_VS_MAP_M:
            self._map_dist_logged = True
            self._event(
                f"TURN distance {mapped:.2f} m from the track "
                f"(camera {vision:.2f} m was early)"
            )
            return float(mapped)
        return vision

    def _fresh_lane(self, est) -> bool:
        return (
            est is not None
            and getattr(est, "valid", False)
            and not getattr(est, "held", False)
            and float(getattr(est, "confidence", 0.0) or 0.0) >= MIN_CONF
            and abs(float(getattr(est, "heading_rad", 9.0))) < REACQUIRE_MAX_HEADING_RAD
        )

    def _note_curve_yaw(self, yaw, est, *, cross_track_m: float | None = None) -> None:
        """A ~90° heading change in LANE is not a pivot. Log it on its own."""
        if self.state == STATE_PIVOT:
            self._curve_yaw0 = None
            return
        if yaw is None or self.state != STATE_LANE:
            return
        if self._curve_yaw0 is None:
            self._curve_yaw0 = float(yaw)
            self._curve_ct = 0.0
            return
        if cross_track_m is not None:
            val = abs(float(cross_track_m))
            if val > self._curve_ct:
                self._curve_ct = val
        delta = math.degrees(
            math.atan2(
                math.sin(float(yaw) - self._curve_yaw0),
                math.cos(float(yaw) - self._curve_yaw0),
            )
        )
        if abs(delta) < 80.0:
            return
        n = sum(1 for row in self._turn_records if "kind=curve" in row) + 1
        self._event(
            f"TURN_RECORD turn={n} kind=curve "
            f"dir={'left' if delta > 0 else 'right'} "
            f"entry_v_m_s={float(self.gov.v):.3f} rem_m=0.000 src=lane"
        )
        self._event(
            f"CURVE_RECORD turn={n} heading_change_deg={delta:+.1f} "
            f"ct_max_m={self._curve_ct:.3f} entry_v_m_s={float(self.gov.v):.3f}"
        )
        self._curve_yaw0 = float(yaw)
        self._curve_ct = 0.0

    def _start_pivot(self, yaw: float, *, entry_v_m_s: float | None = None) -> None:
        self._close_segment("pivot")
        base = self.lane_yaw if self.lane_yaw is not None else float(yaw)
        self.yaw_target = base + self.corner_dir * PIVOT_ANGLE_RAD
        self.pivot_start_yaw = float(yaw)
        self.pivot_w = 0.0
        self.pivot_t = 0.0
        self.pivot_settle = 0
        self.pivot_integrated_rad = 0.0
        self._pivot_lateral_max_m = 0.0
        self._pivot_ct_fit_max_m = 0.0
        pose = self.pose
        pos_txt = (
            f"({pose[0]:.3f},{pose[1]:.3f})"
            if pose is not None
            else "unknown"
        )
        self._pivot_start_pos = pos_txt
        self._pivot_entry_v_m_s = float(entry_v_m_s if entry_v_m_s is not None else self.gov.v)
        self._open_turn_id = self.pivots + 1
        self._event(
            f"TURN_RECORD turn={self._open_turn_id} kind=pivot "
            f"dir={'left' if self.corner_dir > 0 else 'right'} "
            f"entry_v_m_s={self._pivot_entry_v_m_s:.3f} rem_m="
            f"{float(self.remaining_m or 0.0):.3f} src={self.turn_src}"
        )
        coast_was = bool(self._coasted_this_approach or self._coast_latched)
        if self._coast_latched or self._coast_active:
            self._event(
                f"COAST stop — pivot gate entry_v={self._pivot_entry_v_m_s:.3f} m/s "
                f"rem={float(self.remaining_m or 0.0):.3f} m"
            )
        self._coast_active = False
        self._coast_latched = False
        self._coasted_this_approach = False
        self._curve_yaw0 = None
        self.state = STATE_PIVOT
        self.gov.v = 0.0
        self.filter = SteerFilter()
        self._map_dist_logged = False
        self._event(
            f"PIVOT start {'left' if self.corner_dir > 0 else 'right'} yaw {math.degrees(yaw):+.1f}deg -> "
            f"target {math.degrees(self.yaw_target):+.1f}deg (lane {math.degrees(base):+.1f}deg, "
            f"remaining {float(self.remaining_m or 0.0):+.3f} m) entry_v={self._pivot_entry_v_m_s:.3f} m/s "
            f"pos={pos_txt} coast_into_turn={coast_was}"
        )

    def _end_pivot(self, yaw: float, how: str) -> None:
        err = math.atan2(math.sin(self.yaw_target - yaw), math.cos(self.yaw_target - yaw))
        if self.pivot_start_yaw is not None:
            heading_change = math.degrees(
                math.atan2(
                    math.sin(float(yaw) - float(self.pivot_start_yaw)),
                    math.cos(float(yaw) - float(self.pivot_start_yaw)),
                )
            )
        else:
            heading_change = 0.0
        pose = self.pose
        pos_txt = (
            f"({pose[0]:.3f},{pose[1]:.3f})"
            if pose is not None
            else "unknown"
        )
        self.last_pivot = {
            "dir": self.corner_dir, "target_deg": round(math.degrees(self.yaw_target), 2),
            "yaw_deg": round(math.degrees(yaw), 2), "err_deg": round(math.degrees(err), 2),
            "time_s": round(self.pivot_t, 2), "exit": how,
            "entry_v_m_s": round(self._pivot_entry_v_m_s, 3),
            "heading_change_deg": round(heading_change, 1),
            "max_lateral_m": round(self._pivot_lateral_max_m, 3),
            "pos_end": pos_txt,
        }
        self.pivots += 1
        turn_id = self._open_turn_id if self._open_turn_id is not None else self.pivots
        self._open_turn_id = None
        start_pos = getattr(self, "_pivot_start_pos", pos_txt)
        self._event(
            f"PIVOT_RECORD turn={turn_id} dir={'left' if self.corner_dir > 0 else 'right'} "
            f"heading_change_deg={heading_change:+.1f} entry_v_m_s={self._pivot_entry_v_m_s:.3f} "
            f"ct_max_m={self._pivot_lateral_max_m:.3f}"
            f"{self._ct_fit_suffix(self._pivot_ct_fit_max_m)} start_pos={start_pos}"
        )
        self._curve_yaw0 = None
        self._event(
            f"PIVOT done yaw {math.degrees(yaw):+.1f}deg (target {math.degrees(self.yaw_target):+.1f}, "
            f"err {math.degrees(err):+.1f}deg, heading_change {heading_change:+.1f}deg, "
            f"entry_v={self._pivot_entry_v_m_s:.3f} m/s ct_max={self._pivot_lateral_max_m:.3f} m, "
            f"{self.pivot_t:.1f} s, {how}) — REACQUIRE"
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

    def _pivot_tick(
        self,
        est,
        new_frame: bool,
        dt: float,
        yaw: float,
        *,
        yaw_rate_rad_s: float | None = None,
        cross_track_m: float | None = None,
    ) -> dict:
        self.pivot_t += dt
        err = math.atan2(math.sin(self.yaw_target - yaw), math.cos(self.yaw_target - yaw))
        self.pivot_heading_err_rad = float(err)
        self._note_pivot_cross_track(cross_track_m)
        if est is not None and getattr(est, "valid", False):
            self._note_pivot_cross_track_fit(getattr(est, "offset_m", None))
        rate = 0.0 if yaw_rate_rad_s is None else float(yaw_rate_rad_s)
        self.pivot_yaw_rate_rad_s = rate
        from src.gyro_pivot import integrate_heading_rad, pivot_turn_progress_rad, rate_tracking_adjustment

        if yaw_rate_rad_s is not None:
            self.pivot_integrated_rad = integrate_heading_rad(self.pivot_integrated_rad, rate, dt)
        turn_prog = pivot_turn_progress_rad(self.pivot_integrated_rad, self.corner_dir)
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
        if (
            yaw_rate_rad_s is not None
            and turn_prog >= PIVOT_ANGLE_RAD - PIVOT_TOL_RAD
            and abs(rate) <= PIVOT_W_SETTLED
            and abs(err) <= PIVOT_LANE_ALIGNED_RAD
        ):
            self._end_pivot(yaw, "integrated heading")
            return self._cmd(0.0, 0.0, reason="pivot done (gyro heading)", phase="rowfit_" + self.state.lower(),
                             new_frame=new_frame)
        if self.pivot_t > PIVOT_TIMEOUT_S:
            self._event(f"PIVOT timeout after {self.pivot_t:.1f} s, yaw err {math.degrees(err):+.1f}deg — braking")
            self._end_pivot(yaw, "timeout")
            return self._cmd(0.0, 0.0, reason="pivot timeout", phase="rowfit_stop", brake=True,
                             new_frame=new_frame)
        want = PIVOT_K_YAW * err
        want = max(-self.pivot_w_max, min(self.pivot_w_max, want))
        if yaw_rate_rad_s is not None:
            want += rate_tracking_adjustment(want, rate)
            want = max(-self.pivot_w_max, min(self.pivot_w_max, want))
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

    def _enter_recover(self) -> None:
        self.state = STATE_RECOVER
        self.recover_m = 0.0
        self.recover_t = 0.0
        self.frames_lost = 0
        self.gov.v = 0.0
        self.target_speed = V_REACQUIRE_M_S
        if self.tracker is not None and hasattr(self.tracker, "begin_reacquire"):
            self.tracker.begin_reacquire(REACQUIRE_RELAX_FRAMES)

    def _recover_tick(self, est, new_frame: bool, dt: float, yaw) -> dict:
        """Creep along the exit heading until a lane shows up, then reacquire.

        A hard park here latches the ABS brake, and the robot stays off the
        lane. The creep is bounded; past that bound this does stop.
        """
        if new_frame and self._fresh_lane(est):
            self.state = STATE_REACQUIRE
            self.reacq_ok = 0
            self.reacq_frames = 0
            self.frames_lost = 0
            self._reacq_fail_logged = False
            self.target_speed = V_REACQUIRE_M_S
            self._event(
                f"RECOVER found the lane off {est.offset_m * 100:+.0f}cm "
                f"hd {math.degrees(est.heading_rad):+.1f}deg — REACQUIRE"
            )
            if self.tracker is not None and hasattr(self.tracker, "begin_reacquire"):
                self.tracker.begin_reacquire(REACQUIRE_RELAX_FRAMES)
            v = V_REACQUIRE_M_S
            self.gov.v = v
            left, right = self._wheels(v, 0.0)
            return self._cmd(
                left, right, reason="recover found lane", phase="rowfit_reacquire",
                v=v, new_frame=new_frame,
            )
        self.recover_t += max(0.0, float(dt))
        if self.recover_m >= RECOVER_CREEP_M or self.recover_t >= RECOVER_MAX_S:
            self.state = STATE_STOPPED
            self._event(
                f"RECOVER no lane after {self.recover_m:.2f} m / {self.recover_t:.1f} s — STOPPED"
            )
            return self._cmd(
                0.0, 0.0, reason="recover gave up", phase="rowfit_stop", brake=True,
                new_frame=new_frame,
            )
        v_cap = V_REACQUIRE_M_S
        if self.obstacle_v_cap is not None:
            v_cap = min(v_cap, max(0.0, float(self.obstacle_v_cap)))
        self.target_speed = v_cap
        if self.gov.v > v_cap:
            self.gov.v = max(v_cap, self.gov.v - DECEL_M_S2 * dt)
        else:
            self.gov.step(v_cap, dt)
        v = self.gov.v
        w = 0.0
        tgt = self.yaw_target if self.yaw_target is not None else self.lane_yaw
        if yaw is not None and tgt is not None:
            err = math.atan2(math.sin(tgt - float(yaw)), math.cos(tgt - float(yaw)))
            w = max(-GAP_W_MAX, min(GAP_W_MAX, GAP_K_YAW * err))
        self.recover_m += v * dt
        left, right = self._wheels(v, w)
        return self._cmd(
            left, right, reason=f"recover creep {self.recover_m:.2f} m",
            phase="rowfit_recover", v=v, new_frame=new_frame,
        )

