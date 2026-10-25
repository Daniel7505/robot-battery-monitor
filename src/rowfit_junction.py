"""Junction approach, gap cross, and look-around."""

from __future__ import annotations

from src.rowfit_speed import *  # noqa: F403


class _RowfitJunction:
    # ------------------------------------------------------------------
    # intersections
    # ------------------------------------------------------------------
    def _lane_yaw_update(self, est, yaw, gain: float = 0.3) -> None:
        if yaw is None or not self._est_usable_for_lane_yaw(est):
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
        if yaw is not None and self._est_usable_for_lane_yaw(est):
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
            self.remaining_m = self._prefer_map_distance(float(self.jn_center_m), yaw)
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
            # Junction/gap approach: same as CORNER_APPROACH — held ok, low-conf fresh ignored.
            if hold_yaw is None and self._est_usable_for_approach_steer(est):
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
