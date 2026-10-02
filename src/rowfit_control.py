"""Row-fit lane keep: steer + speed from a fitted lane model.

Used by the Webots controller only when ``RBM_LANE_MODE=rowfit``. The
default ``gap`` path (``lane_keep.lane_keep_command``) is untouched.

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
A_LAT_M_S2 = 0.12  # curve speed: v = sqrt(a_lat / |kappa|)
ACCEL_M_S2 = 0.25  # speed-up-out (gentle)
DECEL_M_S2 = 0.80  # slow-in (quick)
LD_BASE_M = 0.45
LD_GAIN_S = 1.2  # look-ahead = base + gain * v
LD_MIN_M = 0.55
LD_MAX_M = 1.6
K_D_OFFSET = 0.6  # 1/m per (m/s) of offset rate
KAPPA_MAX = 2.5
STEER_CAP = 0.8
MIN_CONF = 0.15
LOST_FRAMES_STOP = 3  # ~1 s at 320 ms frames


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
    def __init__(self, *, k_steer: float = DEFAULT_K_STEER) -> None:
        self.k_steer = float(k_steer)
        self.filter = SteerFilter()
        self.gov = SpeedGovernor()
        self.kappa_cmd = 0.0
        self.target_speed = V_MIN_M_S
        self.lookahead_used = 0.0
        self.frames_lost = 0
        self.last_offset: float | None = None
        self.n_frames = 0

    def reset(self) -> None:
        self.__init__(k_steer=self.k_steer)

    def on_frame(self, est, frame_dt: float = 0.32) -> None:
        """Recompute the held command from a NEW frame's LaneEstimate."""
        self.n_frames += 1
        if est is None or not getattr(est, "valid", False) or est.confidence < MIN_CONF:
            self.frames_lost += 1
            self.target_speed = V_MIN_M_S
            self.last_offset = None
            return
        self.frames_lost = 0
        v = self.gov.v
        Ld = max(LD_MIN_M, min(LD_MAX_M, LD_BASE_M + LD_GAIN_S * v))
        if est.lookahead_m > 0.0:
            Ld = max(LD_MIN_M, min(Ld, est.lookahead_m - 0.1))
        gx, gy = est.goal_point(Ld)
        d2 = gx * gx + gy * gy
        k_pp = 0.0 if d2 < 1e-6 else 2.0 * gy / d2
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

    def step(self, est=None, *, new_frame: bool = False, dt: float = 0.008, frame_dt: float = 0.32) -> dict:
        if new_frame:
            self.on_frame(est, frame_dt)
        if self.frames_lost >= LOST_FRAMES_STOP:
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
            }
        v = self.gov.step(self.target_speed, dt)
        raw = steer_for_curvature(self.kappa_cmd, v, self.k_steer)
        raw = max(-STEER_CAP, min(STEER_CAP, raw))
        steer = self.filter.step(raw, dt)
        left, right = wheels_from_steer(
            steer, cruise=v / WHEEL_RADIUS_M, k_steer=self.k_steer, turn_slow=0.0
        )
        return {
            "left": round(left, 3),
            "right": round(right, 3),
            "brake": False,
            "error": round(float(steer), 4),
            "steer": round(float(steer), 4),
            "reason": "rowfit keep",
            "phase": "rowfit" if self.frames_lost == 0 else "rowfit_hold",
            "error_source": "rowfit",
            "target_speed": round(self.target_speed, 3),
            "speed_cmd": round(v, 3),
            "kappa_cmd": round(self.kappa_cmd, 4),
            "lookahead_m": round(self.lookahead_used, 3),
            "new_frame": bool(new_frame),
        }
