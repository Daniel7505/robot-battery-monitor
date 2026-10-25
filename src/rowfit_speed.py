"""Coast, curve speed, and the wheel-rate cap.

``curve_speed``, coast-into-turn, and ``MAX_WHEEL_RAD_S`` stay together.
The other row-fit modules import these tunables and keep their own steps.
"""

from __future__ import annotations

import math

# Assigned before the lane_keep import. wheels_from_steer reads this name,
# and lane_keep re-exports it once this module has bound the value.
MAX_WHEEL_RAD_S = 8.0

from src.lane_keep import (
    DEFAULT_K_STEER,
    SteerFilter,
    WHEEL_RADIUS_M,
    WHEEL_Y_LEFT_M,
    wheels_from_steer,
)

TRACK_M = 2.0 * WHEEL_Y_LEFT_M  # 0.34 m between drive wheels
V_CRUISE_M_S = 0.44  # = 5.5 rad/s on the 0.08 m contact wheel
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
# While lining up a junction/corner turn the near paint is often contaminated by
# the cross-aisle. Fresh fits can stay valid=True at low confidence and pull
# heading several degrees (warehouse aisle-8 miss ~6.8–9°). Held estimates carry
# the last good lane moved by odometry — prefer those. Only trust a fresh fit
# once it clears these thresholds:
APPROACH_STEER_MIN_CONF = 0.55  # steer on a fresh (non-held) fit while approaching
APPROACH_YAW_MIN_CONF = 0.55    # update lane_yaw from a fresh fit while approaching
LOST_FRAMES_STOP = 3  # ~1 s at 320 ms frames

# --- Straightaway cruise (light bump on long, clean straights only)
# Plain LANE following tops out at V_CRUISE_M_S. On a straight with a sure fit,
# a small steer, no corner / junction cue in view, no junction slow / cooldown and
# no obstacle or pick cap (``obstacle_v_cap is None``), the target may rise to
# V_STRAIGHT_M_S after STRAIGHT_FRAMES qualifying frames in a row. Any miss drops
# straight back to the normal curve governor (decel 0.8 m/s^2: 0.50 -> 0.44 in
# ~0.08 s). Junction / approach / pivot / reacquire / gap caps are untouched.
V_STRAIGHT_M_S = 0.50  # = 6.25 rad/s hub (MAX_WHEEL_RAD_S 8.0 leaves steer headroom)
STRAIGHT_MIN_CONF = 0.75
STRAIGHT_MAX_KAPPA = 0.10  # 1/m: |pursuit/lane curvature| and |kappa_cmd| ("steer small")
STRAIGHT_MAX_OFFSET_M = 0.08
STRAIGHT_MAX_HEADING_RAD = math.radians(4.0)
STRAIGHT_FRAMES = 3  # ~1 s of clean straight at 320 ms frames before the bump
V_MAX_M_S = max(V_CRUISE_M_S, V_STRAIGHT_M_S)  # top forward speed (sizes the obstacle slow zone)

# --- Sharp corners (lane_vision corner cue: "the line widens, then disappears")
# LANE -> CORNER_APPROACH -> PIVOT -> REACQUIRE -> LANE.
# REACQUIRE that never sees the new lane goes to RECOVER (creep) instead of
# parking. The cue says a turn is coming. How far to the pivot is odometry
# from the first confident distance, and the loaded track when GPS is on
# that road. A later short camera view must not pull the pivot forward.
STATE_LANE = "LANE"
STATE_APPROACH = "CORNER_APPROACH"
STATE_PIVOT = "PIVOT"
STATE_REACQUIRE = "REACQUIRE"
CORNER_MIN_CONF = 0.6  # act on a persistent cue at least this sure
V_APPROACH_M_S = 0.25  # cap on the last metre into the pivot (not the whole leg)
# Beyond this, the approach keeps cruise. A 0.25 m/s cap for the entire
# corner cue (often several metres) was most of the extra lap time.
APPROACH_CRUISE_BEYOND_M = 1.0
A_APPROACH_M_S2 = 0.30  # stopping profile v = sqrt(2 a remaining)
V_CREEP_M_S = 0.04  # last centimetres (no zero-speed stall short of the point)
PIVOT_ARRIVE_M = 0.015  # remaining distance that starts the pivot (axle at the vertex)
# Forced creep only in the last few centimetres, and only when not coasting.
# The pivot gate stays at PIVOT_ARRIVE_M, so a shorter zone does not open
# the spin before the vertex.
PIVOT_BRAKE_ZONE_M = 0.06
# Coast arms only when speed is clearly above the pivot-entry creep.
COAST_ARM_ABOVE_M_S = 0.02
# One fresh fit may pull the pivot closer by this much. A short view
# (lookahead under the lane gate) may not move it at all.
VISION_SHRINK_MAX_M = 0.20
# Camera distance shorter than the track by this much is "early": use the track.
EARLY_VS_MAP_M = 0.25
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
# After a pivot, a missing lane creeps along the exit heading instead of
# latching the park brake. A fresh lane goes back to REACQUIRE.
STATE_RECOVER = "RECOVER"
RECOVER_CREEP_M = 1.5
RECOVER_MAX_S = 12.0

# --- Intersections (src.junction_vision + src.route_policy)
# LANE -> JUNCTION_APPROACH -> (turn) CORNER_APPROACH -> PIVOT -> REACQUIRE
#                           -> (straight past a side branch) LANE
#                           -> (straight over a crossing / gap) GAP_CROSS -> REACQUIRE -> LANE
#                                                   GAP_CROSS -> (nothing by max blind) LOOK_AROUND -> STOPPED
STATE_JUNCTION = "JUNCTION_APPROACH"
STATE_GAP = "GAP_CROSS"
STATE_LOOK = "LOOK_AROUND"
STATE_STOPPED = "STOPPED"

# What actually set the wheel command this tick. The dashboard used to
# re-run the gap lane keeper, see empty nadir pixels, and log a nadir
# stop while these wheels were still turning.
STEER_CAMERA = "camera"  # fresh lane fit
STEER_HELD = "held"  # last fit, moved by odometry; speed cap 0.30 m/s
STEER_TRACK = "track"  # committed turn; remaining distance is the map
STEER_PIVOT = "pivot"  # in-place yaw on the IMU
STEER_RECOVER = "recover"  # creep along the exit heading
STEER_BLIND = "blind"  # gap cross / look-around, heading held
STEER_STOPPED = "stopped"

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


def curve_speed(kappa_abs: float, conf: float = 1.0, v_cruise: float | None = None) -> float:
    """Target speed for the sharpest curvature ahead (and fit confidence).

    ``v_cruise`` defaults to the module constant. The overnight optimizer
    passes the allowlisted ``controller.cruise_m_s`` through the controller
    instance; callers that omit it keep the 0.44 m/s baseline.
    """
    cruise = V_CRUISE_M_S if v_cruise is None else float(v_cruise)
    k = abs(float(kappa_abs))
    v = cruise if k < 1e-6 else min(cruise, math.sqrt(A_LAT_M_S2 / k))
    c = max(0.0, min(1.0, (float(conf) - MIN_CONF) / 0.45))
    v = V_MIN_M_S + (v - V_MIN_M_S) * c
    return max(V_MIN_M_S, min(cruise, v))


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


class _RowfitSpeed:
    def _turn_v_allowed(self, d_remaining_m: float) -> float | None:
        """Physics cap from distance to the pivot (m/s), or None if disabled."""
        tb = self.turn_brake
        if tb is None:
            return None
        from src.physics_speed_limit import effective_turn_distance_m, max_speed_for_distance_m

        d = effective_turn_distance_m(
            d_remaining_m,
            perception_horizon_m=self.ld_max,
        )
        return max_speed_for_distance_m(
            d,
            a_brake_m_s2=tb.a_brake_m_s2,
            reaction_latency_s=tb.reaction_latency_s,
            safety_margin=tb.safety_margin,
            v_target_m_s=tb.approach_target_m_s,
        )

    def coast_summary_fields(self) -> dict:
        return {
            "coast_wh_saved": round(self._coast_wh_saved, 6),
            "coast_starts": int(self._coast_starts),
        }

    def _pivot_entry_speed_cap_m_s(self) -> float:
        coast_cfg = self.coast_into_turn
        v_pivot = V_CREEP_M_S
        if coast_cfg is not None:
            v_pivot = max(V_CREEP_M_S, float(coast_cfg.v_pivot_m_s))
        return v_pivot + 0.01

    def _advance_remaining(self, rem: float, v_cmd: float, dt: float, mapped: float | None) -> None:
        """Shrink vision remaining only while the wheels are actually told to roll.

        A zero speed command used to subtract ``gov.v * dt`` anyway, so the
        pivot gate opened while the axle was still short of the intersection.
        Map distance, when present, already replaced ``remaining_m`` this tick.
        """
        if mapped is not None:
            return
        if float(v_cmd) <= 0.02:
            return
        self.remaining_m = max(0.0, float(rem) - float(v_cmd) * max(0.0, float(dt)))

    def _brake_to_pivot_cap(
        self,
        est,
        new_frame: bool,
        dt: float,
        rem: float,
        mapped: float | None,
        v_cap: float,
        *,
        yaw_rate_rad_s: float | None = None,
    ) -> dict:
        # Do not drop the approach latch here. This creep still rolls; the
        # latch ends in ``_start_pivot`` so the pivot line can say it coasted.
        self._coast_active = False
        self.target_speed = V_CREEP_M_S
        if self.gov.v > V_CREEP_M_S:
            self.gov.v = max(V_CREEP_M_S, self.gov.v - max(DECEL_M_S2, A_APPROACH_M_S2 * 2.0) * dt)
        v = self.gov.v
        self._advance_remaining(rem, v, dt, mapped)
        raw = steer_for_curvature(self.kappa_cmd, v, self.k_steer)
        raw = max(-STEER_CAP, min(STEER_CAP, raw))
        steer = self.filter.step(raw, dt)
        left, right = wheels_from_steer(
            steer, cruise=v / WHEEL_RADIUS_M, k_steer=self.k_steer, turn_slow=0.0
        )
        return self._cmd(
            left,
            right,
            steer=steer,
            reason=f"corner brake-to-pivot {rem:.2f} m v={v:.3f}>{v_cap:.3f}",
            phase="rowfit_approach",
            v=v,
            new_frame=new_frame,
        )

    def _approach_tick(
        self,
        est,
        new_frame: bool,
        dt: float,
        frame_dt: float,
        yaw,
        *,
        yaw_rate_rad_s: float | None = None,
    ) -> dict:
        if new_frame:
            # steer on the (clipped) lane fit; missing / held frames do not count
            # toward the lost-lane brake here: the lane is SUPPOSED to end ahead.
            if self._est_usable_for_approach_steer(est):
                k_pp, d2 = self._pursuit(est, max(self.gov.v, 0.1))
                self.kappa_cmd = max(-KAPPA_MAX, min(KAPPA_MAX, k_pp))
                self.lookahead_used = math.sqrt(d2)
            else:
                self.kappa_cmd = 0.0
            self.last_offset = None
        mapped = self._map_corner_m(yaw)
        if mapped is not None:
            if (
                not self._map_dist_logged
                and self.remaining_m is not None
                and mapped > float(self.remaining_m) + EARLY_VS_MAP_M
            ):
                self._map_dist_logged = True
                self._event(
                    f"TURN distance {mapped:.2f} m from the track "
                    f"(camera {float(self.remaining_m):.2f} m was early)"
                )
            self.remaining_m = mapped
            rem = mapped
        else:
            rem = float(self.remaining_m if self.remaining_m is not None else 0.0)
        if rem <= PIVOT_ARRIVE_M:
            if yaw is None:
                self.state = STATE_LANE
                self.frames_lost = LOST_FRAMES_STOP
                self._event("CORNER reached but no IMU yaw — braking at the corner (no pivot)")
                return self._cmd(0.0, 0.0, reason="corner without IMU", phase="rowfit_stop", brake=True,
                                 new_frame=new_frame)
            v_entry = float(self.gov.v)
            v_cap = self._pivot_entry_speed_cap_m_s()
            if v_entry > v_cap + 1e-3:
                return self._brake_to_pivot_cap(
                    est, new_frame, dt, rem, mapped, v_cap, yaw_rate_rad_s=yaw_rate_rad_s
                )
            self._start_pivot(float(yaw), entry_v_m_s=v_entry)
            return self._pivot_tick(est, False, dt, float(yaw), yaw_rate_rad_s=yaw_rate_rad_s)
        rem_brake = max(0.0, rem - PIVOT_ARRIVE_M)
        v_turn = self._turn_v_allowed(rem_brake)
        profile = math.sqrt(2.0 * A_APPROACH_M_S2 * max(0.0, rem_brake))
        if rem_brake > APPROACH_CRUISE_BEYOND_M:
            target = self.v_cruise
        else:
            target = min(V_APPROACH_M_S, profile)
        target = max(V_CREEP_M_S, target)
        if v_turn is not None:
            target = min(target, v_turn, self.v_cruise)
        else:
            target = min(target, self.v_cruise)
        if self.obstacle_v_cap is not None:
            target = min(target, max(0.0, float(self.obstacle_v_cap)))
        coast_cfg = self.coast_into_turn
        v_now = float(self.gov.v)
        coasting = False
        v_pivot = V_CREEP_M_S
        if coast_cfg is not None and coast_cfg.enabled:
            from src.physics_speed_limit import coast_decel_m_s2, coast_distance_m, effective_turn_distance_m

            v_pivot = float(coast_cfg.v_pivot_m_s)
            if self.v_cruise <= v_pivot + 1e-3:
                if self._coast_latched:
                    self._event(
                        f"COAST stop — cruise {self.v_cruise:.2f} m/s is not above "
                        f"pivot entry {v_pivot:.2f} m/s"
                    )
                    self._coast_latched = False
                    self._coast_active = False
            elif not self._coast_latched and v_now > v_pivot + COAST_ARM_ABOVE_M_S:
                a_coast = coast_decel_m_s2(
                    crr=coast_cfg.crr,
                    mass_kg=coast_cfg.total_mass_kg,
                    v_m_s=v_now,
                    drag_linear_n_s_per_m=coast_cfg.drag_linear_n_s_per_m,
                )
                d_coast = coast_distance_m(
                    v_now,
                    v_target_m_s=v_pivot,
                    a_coast_m_s2=a_coast,
                    margin_m=coast_cfg.margin_m,
                )
                d_avail = effective_turn_distance_m(rem_brake, perception_horizon_m=self.ld_max)
                # The brake-zone floor is not part of this distance. Arming inside
                # 0.32 m at creep speed logged a new COAST start on every tick.
                d_zone = min(d_coast, max(0.0, d_avail))
                if d_coast >= 0.05 and rem_brake <= d_zone + 1e-3:
                    self._coast_starts += 1
                    self._coast_latched = True
                    self._coasted_this_approach = True
                    note = (
                        f"COAST start {v_now:.2f} m/s -> {v_pivot:.2f} m/s over ~{d_coast:.2f} m "
                        f"(a_coast={a_coast:.3f} m/s²)"
                    )
                    if d_coast > d_avail + 1e-3:
                        note += f" [look-ahead {d_avail:.2f} m < coast {d_coast:.2f} m]"
                    self._event(note)
            if self._coast_latched:
                coasting = True
                self._coast_active = True
        elif self._coast_latched or self._coast_active:
            self._event("COAST stop — coasting disabled")
            self._coast_latched = False
            self._coast_active = False
        v_cap = self._pivot_entry_speed_cap_m_s()
        if coasting:
            target = v_pivot
            self.target_speed = target
            a_coast = coast_decel_m_s2(
                crr=coast_cfg.crr,
                mass_kg=coast_cfg.total_mass_kg,
                v_m_s=v_now,
                drag_linear_n_s_per_m=coast_cfg.drag_linear_n_s_per_m,
            )
            self.gov.v = max(v_pivot, v_now - a_coast * max(0.0, float(dt)))
            v = self.gov.v
            if coast_cfg.a_brake_m_s2 > a_coast + 1e-6:
                p_saved = (
                    coast_cfg.total_mass_kg
                    * v
                    * (coast_cfg.a_brake_m_s2 - a_coast)
                    * max(0.0, float(dt))
                )
                self._coast_wh_saved += max(0.0, p_saved) / 3600.0
        elif rem <= PIVOT_BRAKE_ZONE_M and float(self.gov.v) > v_cap + 1e-3:
            return self._brake_to_pivot_cap(
                est, new_frame, dt, rem, mapped, v_cap, yaw_rate_rad_s=yaw_rate_rad_s
            )
        else:
            self.target_speed = target
            # this profile can need more than the governor's slow-in decel: follow it directly
            if self.gov.v > target:
                self.gov.v = max(target, self.gov.v - max(DECEL_M_S2, A_APPROACH_M_S2 * 2.0) * dt)
            else:
                self.gov.step(target, dt)
            v = self.gov.v
        self._advance_remaining(rem, v, dt, mapped)
        raw = steer_for_curvature(self.kappa_cmd, v, self.k_steer)
        raw = max(-STEER_CAP, min(STEER_CAP, raw))
        steer = self.filter.step(raw, dt)
        # Match the coasting speed. A zero hub is a velocity hold at stall,
        # which spiked pack current and froze the axle short of the vertex.
        hub = max(v, v_pivot if coasting else 0.0) / WHEEL_RADIUS_M
        left, right = wheels_from_steer(steer, cruise=hub, k_steer=self.k_steer, turn_slow=0.0)
        reason = (
            f"corner coast {rem:.2f} m to pivot"
            if coasting
            else f"corner approach {rem:.2f} m to pivot"
        )
        return self._cmd(left, right, steer=steer, reason=reason,
                         phase="rowfit_approach", v=v, new_frame=new_frame)

