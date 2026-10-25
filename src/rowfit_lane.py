"""Pure pursuit, look-ahead, and the straightaway cruise check."""

from __future__ import annotations

from src.rowfit_speed import *  # noqa: F403


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


class _RowfitLane:
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
            self._straight_reset()
            if getattr(est, "counts_as_lost", True):
                short = str(getattr(est, "reject_reason", "") or "").startswith("short view")
                # The new leg is often only a short view for a few frames. That
                # is not a lost lane, and it must not park the robot.
                if not (short and self.state == STATE_REACQUIRE):
                    self.frames_lost += 1
            self.target_speed = min(self.target_speed, V_HELD_M_S)
            self.last_offset = None
            self.kappa_cmd = max(-KAPPA_MAX, min(KAPPA_MAX, self._pursuit(est, self.gov.v)[0]))
            return
        if est is None or not getattr(est, "valid", False) or est.confidence < MIN_CONF:
            self.frames_lost += 1
            self._straight_reset()
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
        self.target_speed = curve_speed(k_gov, est.confidence, v_cruise=self.v_cruise)
        if self._straightaway_ok(est, k_gov):
            self.straight_frames += 1
        else:
            self.straight_frames = 0
        self.straight_boost = self.straight_frames >= STRAIGHT_FRAMES
        if self.straight_boost:
            self.target_speed = max(self.target_speed, self.v_straight)

    def _straight_reset(self) -> None:
        self.straight_frames = 0
        self.straight_boost = False

    def _straightaway_ok(self, est, k_gov: float) -> bool:
        """This frame qualifies for the straightaway cruise bump (see V_STRAIGHT_M_S)."""
        if self.state != STATE_LANE or self.frames_lost:
            return False
        if self.obstacle_v_cap is not None:  # SLOW / STOPPED / WAITING or a pick hold
            return False
        if self.cooldown_m > 0.0 or self.jn_cooldown_m > 0.0 or self.jn_slow_m > 0.0:
            return False
        # any junction / opening / corner cue in view, even one not yet persistent
        if getattr(est, "junction", None) is not None or getattr(est, "corner", None) is not None:
            return False
        return (
            est.confidence >= STRAIGHT_MIN_CONF
            and k_gov <= STRAIGHT_MAX_KAPPA
            and abs(self.kappa_cmd) <= STRAIGHT_MAX_KAPPA
            and abs(est.offset_m) <= STRAIGHT_MAX_OFFSET_M
            and abs(est.heading_rad) <= STRAIGHT_MAX_HEADING_RAD
        )
