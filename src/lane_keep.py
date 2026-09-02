"""Nadir-only lane keep — 2026-09-02 test swap.

Shoulder cameras looking down, pixel gap, 6 cm stripe scale, steer.
Choir (picture-wins, wall bounce, LINE_CAM, forecast, planner) is stashed
at archives/lane_keep_choir_2026-09-02.py. Restore that file to undo.

Other cameras may still exist in the world. They are not on the wheel.
"""

from __future__ import annotations

import math

# Nadir cruise. Controller may still pass this explicitly.
DEFAULT_CRUISE_RAD_S = 5.5
DEFAULT_K_STEER = 2.0
DEFAULT_STEER_DEADBAND = 0.01
DEFAULT_STEER_SLEW_PER_S = 12.0
DEFAULT_STEER_RELEASE_PER_S = 4.0
DEFAULT_TURN_SLOW = 0.24
MAX_WHEEL_RAD_S = 8.0
WHEEL_RADIUS_M = 0.08
STRIPE_W_M = 0.06
LANE_HALF_M = 0.65
WHEEL_Y_LEFT_M = 0.17
PAINT_Y_LEFT_M = 0.65
PAINT_HALF_W_M = 0.03

NADIR_MOUNT_POS = (0.01042, 0.41808, 0.72919)
NADIR_MOUNT_ROT = (0.0, 0.0, 1.0, -math.pi / 2.0)
NADIR_IMAGE = 64
NADIR_FOV_RAD = 1.2
NADIR_AXLE_AHEAD_M = 0.0
NADIR_SPAWN_GAP_PX = 31

NADIR_BASE_L_PX = 32
NADIR_BASE_R_PX = 29
NADIR_AHEAD_L_PX = 32
NADIR_AHEAD_R_PX = 26
NADIR_PX_DEADBAND = 2
NADIR_AHEAD_DEADBAND = 1
NADIR_K_PX = 0.03
NADIR_K_HOLD = 0.03
NADIR_KD_PX = 0.12
NADIR_STEER_CAP = 0.55
# Spatial curve κ = ω/v. Same pixel yank at 2× speed is half the curve
# unless P/HOLD scale with v. D already grows with pixel rate — do not
# scale D. Cap keeps a hairpin from visiting the desert.
NADIR_V_REF_M_S = 0.21
NADIR_V_SCALE_MAX = 2.2

SKY_RGB = (0.72, 0.76, 0.82)
FLOOR_RGB = (0.55, 0.56, 0.58)
FORECAST_IMAGE_ROLL_RAD = 0.0
PLANNER_KAHEAD = 0.0
DEFAULT_ABS_DECEL_M_S2 = 2.5
MIN_STOP_DISTANCE_M = 0.06
DEFAULT_RED_THRESH = 0.99  # red cam is off this test
LOST_PAINT_YELLOW = 0.12
MIN_STRIPE_FILL = 0.03
DEFAULT_PREVIEW_GAIN = 0.0
PREVIEW_SKY_FRAC = 0.80
PREVIEW_RANGE_MIN_M = 0.40
PREVIEW_RANGE_MAX_M = 4.00
PREVIEW_FALLBACK_T_S = 2.0
PLANNER_AHEAD_REF_S = 2.0
DEFAULT_WALL_COMFORT = 0.16
DEFAULT_WALL_PREVIEW = 0.55
DEFAULT_WALL_CONTACT = 0.45
DEFAULT_FILL_COMFORT = 0.08
DEFAULT_FILL_GAIN = 8.0
DEFAULT_WALL_HIT = 0.18
WALL_COMFORT_M = 0.45

# Same S as scripts/s_track.py. GPS y vs world 0 is not "off center."
_TRACK_START_M = 3.0
_TRACK_LOBE_M = 9.0
_TRACK_AMP_M = 1.0
_TRACK_SIGN = -1.0
FINISH_X_M = 24.5


def track_centerline(x: float) -> tuple[float, float]:
    """Lane center (y, heading_rad) at world x."""
    x = float(x)
    if x <= _TRACK_START_M:
        return 0.0, 0.0
    if x <= _TRACK_START_M + _TRACK_LOBE_M:
        u = x - _TRACK_START_M
        y = _TRACK_SIGN * (_TRACK_AMP_M / 2.0) * (
            1.0 - math.cos(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        yp = (
            _TRACK_SIGN
            * (_TRACK_AMP_M / 2.0)
            * (2.0 * math.pi / _TRACK_LOBE_M)
            * math.sin(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        return y, math.atan(yp)
    if x <= _TRACK_START_M + 2 * _TRACK_LOBE_M:
        u = x - _TRACK_START_M - _TRACK_LOBE_M
        y = -_TRACK_SIGN * (_TRACK_AMP_M / 2.0) * (
            1.0 - math.cos(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        yp = (
            -_TRACK_SIGN
            * (_TRACK_AMP_M / 2.0)
            * (2.0 * math.pi / _TRACK_LOBE_M)
            * math.sin(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        return y, math.atan(yp)
    return 0.0, 0.0


def track_cross_track_m(x: float, y: float) -> float:
    """Meters left of centerline (+ = robot left of paint center)."""
    cy, th = track_centerline(x)
    return (float(y) - cy) * math.cos(th)


def yellow_score(rgb: tuple[float, float, float]) -> float:
    """0–1: yellow paint (high R+G, low B) vs gray floor."""
    r, g, b = rgb
    return max(0.0, min(1.0, (r + g) * 0.5 - b))


def red_score(rgb: tuple[float, float, float]) -> float:
    r, g, b = rgb
    return max(0.0, min(1.0, r - max(g, b)))


def _nadir_fan_err(
    left_px: int | None,
    right_px: int | None,
    *,
    base_l: int,
    base_r: int,
    deadband: float,
) -> float | None:
    e_l = None if left_px is None else float(base_l) - float(left_px)
    e_r = None if right_px is None else float(base_r) - float(right_px)
    if e_l is None and e_r is None:
        return None
    if e_l is None:
        err = -float(e_r)
    elif e_r is None:
        err = float(e_l)
    else:
        err = 0.5 * (float(e_l) - float(e_r))
    if abs(err) <= float(deadband):
        return 0.0
    return err


def steer_from_nadir_gaps(
    left_px: int | None,
    right_px: int | None,
    *,
    last_err: float | None = None,
    dt: float = 0.008,
    base_l: int = NADIR_BASE_L_PX,
    base_r: int = NADIR_BASE_R_PX,
    left_ahead_px: int | None = None,
    right_ahead_px: int | None = None,
    ahead_l: int = NADIR_AHEAD_L_PX,
    ahead_r: int = NADIR_AHEAD_R_PX,
    cruise: float | None = None,
) -> tuple[float | None, float]:
    """Fight axle gaps back to 32/29. Forward rows add HOLD. +steer = yaw right.

    P/HOLD scale with v/v_ref so curvature holds when cruise rises.
    D is not scaled. Result still clipped to NADIR_STEER_CAP.
    """
    err = _nadir_fan_err(
        left_px, right_px, base_l=base_l, base_r=base_r, deadband=NADIR_PX_DEADBAND
    )
    hold = _nadir_fan_err(
        left_ahead_px,
        right_ahead_px,
        base_l=ahead_l,
        base_r=ahead_r,
        deadband=NADIR_AHEAD_DEADBAND,
    )
    if err is None and hold is None:
        return None, 0.0
    if err is None:
        err = 0.0
    if hold is None:
        hold = 0.0
    rate = 0.0
    if last_err is not None and float(dt) > 1e-4:
        rate = (err - float(last_err)) / float(dt)
    scale = 1.0
    if cruise is not None:
        v = abs(float(cruise)) * WHEEL_RADIUS_M
        if NADIR_V_REF_M_S > 1e-6:
            scale = max(1.0, min(NADIR_V_SCALE_MAX, v / NADIR_V_REF_M_S))
    raw = (NADIR_K_PX * err + NADIR_K_HOLD * hold) * scale - NADIR_KD_PX * rate
    return max(-NADIR_STEER_CAP, min(NADIR_STEER_CAP, raw)), err


class NadirGuard:
    """Stop if both eyes vanish, or both gaps grow/shrink together."""

    def __init__(self, unison_px: int = 2, unison_frames: int = 4) -> None:
        self.unison_px = int(unison_px)
        self.unison_frames = int(unison_frames)
        self.last_l: int | None = None
        self.last_r: int | None = None
        self._unison_n = 0
        self.abort = False
        self.reason = ""
        self.last_err = 0.0

    def reset(self) -> None:
        self.last_l = None
        self.last_r = None
        self._unison_n = 0
        self.abort = False
        self.reason = ""
        self.last_err = 0.0

    def step(self, left_px: int | None, right_px: int | None) -> dict:
        if left_px is None and right_px is None:
            self.abort = True
            self.reason = "both nadir eyes gone"
            return {"abort": True, "reason": self.reason}
        if (
            left_px is not None
            and right_px is not None
            and self.last_l is not None
            and self.last_r is not None
        ):
            d_l = int(left_px) - int(self.last_l)
            d_r = int(right_px) - int(self.last_r)
            unison = (
                d_l * d_r > 0
                and abs(d_l) >= self.unison_px
                and abs(d_r) >= self.unison_px
            )
            self._unison_n = self._unison_n + 1 if unison else 0
            if self._unison_n >= self.unison_frames:
                self.abort = True
                self.reason = "nadir unison lie (both gaps moved the same way)"
                return {"abort": True, "reason": self.reason}
        if left_px is not None:
            self.last_l = int(left_px)
        if right_px is not None:
            self.last_r = int(right_px)
        return {"abort": False, "reason": ""}


class SteerFilter:
    """Slew the virtual wheel so both hubs ease in together."""

    def __init__(
        self,
        slew_per_s: float = DEFAULT_STEER_SLEW_PER_S,
        release_per_s: float | None = None,
    ) -> None:
        self.value = 0.0
        self.slew_per_s = float(slew_per_s)
        if release_per_s is None:
            self.release_per_s = min(self.slew_per_s, DEFAULT_STEER_RELEASE_PER_S)
        else:
            self.release_per_s = float(release_per_s)

    def step(self, target: float | None, dt: float) -> float:
        want = 0.0 if target is None else max(-1.0, min(1.0, float(target)))
        if abs(want) < 1e-9:
            grabbing = abs(self.value) < 1e-9
        elif self.value * want < 0.0:
            grabbing = True
        else:
            grabbing = abs(want) + 1e-9 >= abs(self.value)
        rate = self.slew_per_s if grabbing else self.release_per_s
        max_step = max(0.0, float(rate) * max(0.0, float(dt)))
        delta = want - self.value
        if abs(delta) > max_step:
            delta = max_step if delta > 0.0 else -max_step
        self.value += delta
        return self.value


def wheels_from_steer(
    steer: float,
    *,
    cruise: float = DEFAULT_CRUISE_RAD_S,
    k_steer: float = DEFAULT_K_STEER,
    turn_slow: float = DEFAULT_TURN_SLOW,
) -> tuple[float, float]:
    """Both hubs stay forward. steer < 0 → left slower (yaw left)."""
    s = float(steer)
    slow = max(0.0, min(0.6, float(turn_slow))) * min(1.0, abs(s))
    cruise_eff = float(cruise) * (1.0 - slow)
    left = cruise_eff + k_steer * s
    right = cruise_eff - k_steer * s
    floor = 0.35 * abs(float(cruise))
    left = max(floor, min(MAX_WHEEL_RAD_S, left))
    right = max(floor, min(MAX_WHEEL_RAD_S, right))
    return left, right


def _bgra_rgb_at(
    image: bytes | bytearray,
    width: int,
    height: int,
    col: int,
    row: int,
) -> tuple[float, float, float] | None:
    x = int(col)
    y = int(row)
    if x < 0 or y < 0 or x >= int(width) or y >= int(height):
        return None
    o = (y * int(width) + x) * 4
    buf = memoryview(image)
    if o + 2 >= len(buf):
        return None
    scale = 1.0 / 255.0
    return (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)


def _longest_run(cols: list[int]) -> tuple[int, int] | None:
    if not cols:
        return None
    cols = sorted(set(int(c) for c in cols))
    best = (cols[0], cols[0])
    start = prev = cols[0]
    for c in cols[1:]:
        if c == prev + 1:
            prev = c
            if prev - start > best[1] - best[0]:
                best = (start, prev)
        else:
            start = prev = c
    return best


def nadir_wheel_to_tape(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    side: str = "left",
    thresh: float = 0.22,
) -> dict | None:
    """Yellow ribbon = 6 cm. Gap from inner tape edge to that side's wheel."""
    w = int(width)
    h = int(height)
    if w < 4 or h < 4 or not image:
        return None
    right = str(side).lower().startswith("r")
    gaps: list[int] = []
    ahead_gaps: list[int] = []
    axle_gaps: list[int] = []
    stripes: list[int] = []
    tape_cols: list[int] = []
    wheel_cols: list[int] = []
    for row in range(h):
        ycols: list[int] = []
        dcols: list[int] = []
        for col in range(w):
            rgb = _bgra_rgb_at(image, w, h, col, row)
            if rgb is None:
                continue
            if yellow_score(rgb) >= float(thresh):
                ycols.append(col)
            elif rgb[0] < 0.20 and rgb[1] < 0.20 and rgb[2] < 0.24:
                dcols.append(col)
        run = _longest_run(ycols)
        if run is None:
            continue
        stripe = run[1] - run[0] + 1
        if stripe < 2 or stripe > 12:
            continue
        if right:
            dark = [c for c in dcols if c < run[0] - 1]
            dark_run = _longest_run(dark)
            if dark_run is None or dark_run[1] - dark_run[0] + 1 < 2:
                continue
            wheel = dark_run[1]
            tape_inner = run[0]
            gap = tape_inner - wheel
        else:
            dark = [c for c in dcols if c > run[1] + 1]
            dark_run = _longest_run(dark)
            if dark_run is None or dark_run[1] - dark_run[0] + 1 < 2:
                continue
            wheel = dark_run[0]
            tape_inner = run[1]
            gap = wheel - tape_inner
        if gap < 2:
            continue
        gaps.append(gap)
        stripes.append(stripe)
        tape_cols.append(tape_inner)
        wheel_cols.append(wheel)
        if row < h // 2:
            ahead_gaps.append(gap)
        if (h // 2 - 6) <= row < (h // 2 + 6):
            axle_gaps.append(gap)
    if not gaps or not stripes:
        return None
    gaps.sort()
    stripes.sort()
    gap_px = gaps[len(gaps) // 2]
    stripe_px = stripes[len(stripes) // 2]
    if stripe_px < 1:
        return None
    m = float(gap_px) * (STRIPE_W_M / float(stripe_px))
    tape_cols.sort()
    wheel_cols.sort()
    ahead_gaps.sort()
    axle_gaps.sort()
    return {
        "gap_px": int(gap_px),
        "gap_ahead_px": None if not ahead_gaps else int(ahead_gaps[len(ahead_gaps) // 2]),
        "gap_axle_px": None if not axle_gaps else int(axle_gaps[len(axle_gaps) // 2]),
        "stripe_px": int(stripe_px),
        "m": m,
        "tape_col": int(tape_cols[len(tape_cols) // 2]),
        "wheel_col": int(wheel_cols[len(wheel_cols) // 2]),
    }


def lane_keep_command(
    left_yellow: float = 0.0,
    right_yellow: float = 0.0,
    finish_red: float = 0.0,
    *,
    cruise: float = DEFAULT_CRUISE_RAD_S,
    k_steer: float = DEFAULT_K_STEER,
    red_thresh: float = DEFAULT_RED_THRESH,
    mark_plan: dict | None = None,
    left_offset: float | None = None,
    right_offset: float | None = None,
    left_curve: float | None = None,
    right_curve: float | None = None,
    left_fill: float | None = None,
    right_fill: float | None = None,
    left_y_m: float | None = None,
    right_y_m: float | None = None,
    left_far_offset: float | None = None,
    right_far_offset: float | None = None,
    z_y_m: float | None = None,
    w_y_m: float | None = None,
    t_ahead: float | None = None,
    preview_ok: bool = False,
    allow_one_eye: bool = False,
    steer_filter: SteerFilter | None = None,
    planner=None,
    watch_plan: dict | None = None,
    lookout=None,
    z_fill: float | None = None,
    w_fill: float | None = None,
    left_wall_dist_m: float | None = None,
    right_wall_dist_m: float | None = None,
    left_gap_px: int | None = None,
    right_gap_px: int | None = None,
    left_ahead_px: int | None = None,
    right_ahead_px: int | None = None,
    nadir_guard: NadirGuard | None = None,
    nadir_primary: bool = True,
    dt: float = 0.008,
) -> dict:
    """Shoulder nadir only. Extra kwargs are accepted and ignored."""
    del (
        left_yellow,
        right_yellow,
        finish_red,
        red_thresh,
        mark_plan,
        left_offset,
        right_offset,
        left_curve,
        right_curve,
        left_fill,
        right_fill,
        left_y_m,
        right_y_m,
        left_far_offset,
        right_far_offset,
        z_y_m,
        w_y_m,
        t_ahead,
        preview_ok,
        allow_one_eye,
        planner,
        watch_plan,
        lookout,
        z_fill,
        w_fill,
        left_wall_dist_m,
        right_wall_dist_m,
        nadir_primary,
    )
    lost = left_gap_px is None and right_gap_px is None
    halt = None
    if nadir_guard is not None:
        halt = nadir_guard.step(left_gap_px, right_gap_px)
    if lost:
        halt = {"abort": True, "reason": "both nadir eyes gone"}
    if halt is not None and halt.get("abort"):
        return {
            "left": 0.0,
            "right": 0.0,
            "brake": True,
            "error": 0.0,
            "steer": 0.0,
            "reason": f"nadir — {halt.get('reason') or 'stop'}",
            "remaining_m": None,
            "phase": "nadir_stop",
            "error_source": "nadir",
            "left_pressure": 0.0,
            "right_pressure": 0.0,
            "metric_ct": None,
            "metric_active": False,
            "preview_ok": False,
            "err_ahead": None,
            "t_ahead": None,
            "plan_mode": None,
            "plan_rate": None,
        }
    last = None if nadir_guard is None else nadir_guard.last_err
    raw, err_px = steer_from_nadir_gaps(
        left_gap_px,
        right_gap_px,
        last_err=last,
        dt=dt,
        left_ahead_px=left_ahead_px,
        right_ahead_px=right_ahead_px,
        cruise=cruise,
    )
    if nadir_guard is not None:
        nadir_guard.last_err = err_px
    if raw is None:
        raw = 0.0
    steer = steer_filter.step(raw, dt) if steer_filter is not None else raw
    left, right = wheels_from_steer(float(steer), cruise=cruise, k_steer=k_steer)
    left_p = 0.0
    right_p = 0.0
    if left_gap_px is not None:
        left_p = max(0.0, (NADIR_BASE_L_PX - float(left_gap_px)) / 20.0)
    if right_gap_px is not None:
        right_p = max(0.0, (NADIR_BASE_R_PX - float(right_gap_px)) / 20.0)
    return {
        "left": round(left, 3),
        "right": round(right, 3),
        "brake": False,
        "error": round(float(steer), 4),
        "steer": round(float(steer), 4),
        "left_pressure": round(float(left_p), 4),
        "right_pressure": round(float(right_p), 4),
        "reason": "nadir keep",
        "remaining_m": None,
        "phase": "seek",
        "plan_mode": None,
        "plan_rate": None,
        "preview_ok": False,
        "err_ahead": None,
        "t_ahead": None,
        "metric_ct": None,
        "metric_active": False,
        "error_source": "nadir",
    }


# --- boot helpers (aim / interlock). Not on the wheel. ---


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: tuple[float, float, float]) -> tuple[float, float, float] | None:
    n = math.sqrt(_dot(v, v))
    if n < 1e-9:
        return None
    return (v[0] / n, v[1] / n, v[2] / n)


def _sub(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def apply_sf_rotation(
    rot: tuple[float, float, float, float],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    ax, ay, az, ang = rot
    axis = _norm((ax, ay, az)) or (0.0, 1.0, 0.0)
    c = math.cos(ang)
    s = math.sin(ang)
    kxv = _cross(axis, vec)
    kdv = _dot(axis, vec)
    return (
        vec[0] * c + kxv[0] * s + axis[0] * kdv * (1.0 - c),
        vec[1] * c + kxv[1] * s + axis[1] * kdv * (1.0 - c),
        vec[2] * c + kxv[2] * s + axis[2] * kdv * (1.0 - c),
    )


def rotation_matrix_to_sf(columns_xyz: tuple) -> tuple[float, float, float, float]:
    x, y, z = columns_xyz
    r00, r01, r02 = x[0], y[0], z[0]
    r10, r11, r12 = x[1], y[1], z[1]
    r20, r21, r22 = x[2], y[2], z[2]
    trace = r00 + r11 + r22
    cos_a = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_a)
    if angle < 1e-8:
        return (0.0, 1.0, 0.0, 0.0)
    ax = r21 - r12
    ay = r02 - r20
    az = r10 - r01
    axis = _norm((ax, ay, az))
    if axis is None:
        return (0.0, 1.0, 0.0, math.pi)
    return (axis[0], axis[1], axis[2], angle)


def _basis_from_z(
    z_axis: tuple[float, float, float],
    up: tuple[float, float, float],
) -> tuple | None:
    z = _norm(z_axis)
    if z is None:
        return None
    up_n = _norm(up) or (0.0, 0.0, 1.0)
    if abs(_dot(z, up_n)) > 0.98:
        up_n = (1.0, 0.0, 0.0) if abs(z[0]) < 0.9 else (0.0, 1.0, 0.0)
    x = _norm(_cross(up_n, z))
    if x is None:
        x = _norm(_cross((0.0, 1.0, 0.0), z))
    if x is None:
        return None
    y = _cross(z, x)
    return (x, y, z)


def look_at_sf_rotation(
    cam_pos: tuple[float, float, float],
    target_pos: tuple[float, float, float],
    up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> tuple[float, float, float, float]:
    z_axis = _sub(cam_pos, target_pos)
    basis = _basis_from_z(z_axis, up)
    if basis is None:
        return (0.0, 1.0, 0.0, 0.0)
    return rotation_matrix_to_sf(basis)


def beam_sf_rotation(
    from_pos: tuple[float, float, float],
    to_pos: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    z_axis = _sub(to_pos, from_pos)
    basis = _basis_from_z(z_axis, (0.0, 1.0, 0.0))
    if basis is None:
        return (0.0, 1.0, 0.0, 0.0)
    return rotation_matrix_to_sf(basis)


def camera_fov_pyramid(
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    fov_rad: float,
    width: int,
    height: int,
    dist_m: float,
) -> list[tuple[float, float, float]]:
    aspect = float(max(1, int(width))) / float(max(1, int(height)))
    half_v = 0.5 * float(fov_rad)
    half_h = math.atan(math.tan(half_v) * aspect)
    dist = max(0.1, float(dist_m))
    corners = (
        (math.tan(half_h), math.tan(half_v), -1.0),
        (-math.tan(half_h), math.tan(half_v), -1.0),
        (-math.tan(half_h), -math.tan(half_v), -1.0),
        (math.tan(half_h), -math.tan(half_v), -1.0),
    )
    pts: list[tuple[float, float, float]] = [
        (float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2]))
    ]
    for c in corners:
        n = _norm(c)
        if n is None:
            continue
        w = apply_sf_rotation(cam_rot, n)
        pts.append(
            (
                cam_pos[0] + w[0] * dist,
                cam_pos[1] + w[1] * dist,
                cam_pos[2] + w[2] * dist,
            )
        )
    return pts


def roll_camera_sf(
    rot: tuple[float, float, float, float],
    angle_rad: float,
) -> tuple[float, float, float, float]:
    local = (0.0, 0.0, 1.0, float(angle_rad))
    x = apply_sf_rotation(rot, apply_sf_rotation(local, (1.0, 0.0, 0.0)))
    y = apply_sf_rotation(rot, apply_sf_rotation(local, (0.0, 1.0, 0.0)))
    z = apply_sf_rotation(rot, apply_sf_rotation(local, (0.0, 0.0, 1.0)))
    return rotation_matrix_to_sf((x, y, z))


def rotate_bgra_90_cw(
    image: bytes | bytearray, width: int, height: int
) -> tuple[bytearray, int, int]:
    w = int(width)
    h = int(height)
    nw, nh = h, w
    src = memoryview(image)
    out = bytearray(nw * nh * 4)
    for r in range(h):
        for c in range(w):
            nc = h - 1 - r
            nr = c
            s = (r * w + c) * 4
            d = (nr * nw + nc) * 4
            if s + 4 <= len(src) and d + 4 <= len(out):
                out[d : d + 4] = src[s : s + 4]
    return out, nw, nh


def rgb_distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def classify_view(
    rgb: tuple[float, float, float],
    *,
    yellow: float | None = None,
    red: float | None = None,
) -> str:
    r, g, b = rgb
    red_v = red_score(rgb) if red is None else float(red)
    yellow_v = yellow_score(rgb) if yellow is None else float(yellow)
    if red_v >= 0.28:
        return "red"
    if yellow_v >= 0.15:
        return "yellow"
    d_sky = rgb_distance(rgb, SKY_RGB)
    d_floor = rgb_distance(rgb, FLOOR_RGB)
    if d_sky < 0.08 and d_sky <= d_floor:
        return "sky"
    if d_floor < 0.18:
        return "ground"
    if b > 0.65 and b >= r - 0.02 and b >= g - 0.02:
        return "sky"
    return "ground"


def ground_hit_ahead_m(
    cam_pos: tuple[float, float, float],
    target_pos: tuple[float, float, float],
    floor_z: float = 0.0,
) -> float:
    dx = target_pos[0] - cam_pos[0]
    dz = target_pos[2] - cam_pos[2]
    if abs(dz) < 1e-9:
        return max(0.0, float(target_pos[0]))
    s = (floor_z - cam_pos[2]) / dz
    if s < 0.0:
        return 0.0
    return max(0.0, cam_pos[0] + s * dx)


def image_row_elevation_rad(row: float, height: int, fov_rad: float) -> float:
    h = max(1, int(height))
    ny = (float(row) + 0.5) / h
    return (0.5 - ny) * float(fov_rad)


def image_row_to_ground_ahead_m(
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    row: float,
    height: int,
    fov_rad: float,
    floor_z: float = 0.0,
) -> float | None:
    ay = image_row_elevation_rad(row, height, fov_rad)
    ray_cam = _norm((0.0, math.tan(ay), -1.0))
    if ray_cam is None:
        return None
    ray = apply_sf_rotation(cam_rot, ray_cam)
    if ray[2] >= -1e-6:
        return None
    t = (floor_z - cam_pos[2]) / ray[2]
    if t < 0.0:
        return None
    return max(0.0, cam_pos[0] + t * ray[0])


def nadir_axle_row(
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    height: int,
    fov_rad: float,
    *,
    axle_ahead_m: float = NADIR_AXLE_AHEAD_M,
) -> int:
    h = max(1, int(height))
    best_row = h // 2
    best_err = 1e9
    for row in range(h):
        ahead = image_row_to_ground_ahead_m(cam_pos, cam_rot, row, h, fov_rad)
        if ahead is None:
            continue
        err = abs(float(ahead) - float(axle_ahead_m))
        if err < best_err:
            best_err = err
            best_row = row
    return best_row


def cruise_speed_m_s(wheel_rad_s: float = DEFAULT_CRUISE_RAD_S) -> float:
    return abs(float(wheel_rad_s)) * WHEEL_RADIUS_M


def stopping_distance_m(
    speed_m_s: float, decel_m_s2: float = DEFAULT_ABS_DECEL_M_S2
) -> float:
    v = max(0.0, float(speed_m_s))
    a = max(0.1, float(decel_m_s2))
    return max(MIN_STOP_DISTANCE_M, (v * v) / (2.0 * a))


def time_to_mark_s(look_ahead_m: float, speed_m_s: float) -> float | None:
    v = float(speed_m_s)
    if v <= 0.05:
        return None
    return max(0.0, float(look_ahead_m) / v)


class MarkStopTracker:
    """Red-mark tracker stub. Off the wheel this test."""

    def reset(self) -> None:
        return None

    def step(self, *args, **kwargs):
        return None


class GapPlanner:
    def reset(self) -> None:
        return None

    def step(self, raw_steer, dt=0.008, **kwargs):
        return {"desired_steer": raw_steer, "mode": None, "rate": 0.0}


class CorridorWatch:
    def reset(self) -> None:
        return None

    def step(self, *args, **kwargs):
        return None


class ForecastLookout:
    def reset(self) -> None:
        return None

    def step(self, *args, **kwargs):
        return {"abort": False, "reason": ""}


def yellow_horizon_row(height: int) -> int:
    return max(1, (int(height) * 2) // 5)


def yellow_look_band(height: int) -> tuple[int, int]:
    h = int(height)
    if h <= 20:
        return 0, h
    return max(1, h // 2), max(h // 2 + 1, (h * 13) // 16)


def yellow_far_band(height: int) -> tuple[int, int]:
    y0, y1 = yellow_look_band(height)
    mid = (y0 + y1) // 2
    return y0, mid


def yellow_near_band(height: int) -> tuple[int, int]:
    y0, y1 = yellow_look_band(height)
    mid = (y0 + y1) // 2
    return mid, y1


def yellow_look_split(height: int) -> int:
    y0, y1 = yellow_look_band(height)
    return (y0 + y1) // 2


def offset_to_column(offset: float, width: int) -> int:
    w = max(1, int(width))
    x = 0.5 + 0.5 * max(-1.0, min(1.0, float(offset)))
    return max(0, min(w - 1, int(x * w)))


def yellow_band_fill(*args, **kwargs) -> float:
    return 0.0


def yellow_near_fill(*args, **kwargs) -> float:
    return 0.0


def yellow_nearest_pixel(*args, **kwargs):
    return None


def yellow_ahead_pixel(*args, **kwargs):
    return None


def yellow_line_offset(*args, **kwargs):
    return None


def yellow_far_offset(*args, **kwargs):
    return None


def yellow_near_offset(*args, **kwargs):
    return None


def yellow_line_curve(*args, **kwargs):
    return None


def band_mean_rgb(*args, **kwargs) -> tuple[float, float, float]:
    return (0.5, 0.5, 0.5)


def band_sky_frac(*args, **kwargs) -> float:
    return 0.0


def preview_band_ok(*args, **kwargs) -> bool:
    return False


def dirt_band_ok(*args, **kwargs) -> bool:
    return True


def preview_look_ahead_m(*args, **kwargs):
    return None


def preview_ahead_weight(*args, **kwargs) -> float:
    return 0.0


def wall_pressure(*args, **kwargs) -> float:
    return 0.0


def steer_from_walls(*args, **kwargs):
    return None


def steer_from_offsets(*args, **kwargs):
    return None


def steer_from_ranges(*args, **kwargs):
    return None


def mean_rgb_bgra(image: bytes | bytearray, width: int, height: int) -> tuple[float, float, float]:
    n = max(1, int(width) * int(height))
    rs = gs = bs = 0
    buf = memoryview(image)
    pixels = min(n, len(buf) // 4)
    for i in range(pixels):
        o = i * 4
        bs += buf[o]
        gs += buf[o + 1]
        rs += buf[o + 2]
    if pixels <= 0:
        return (0.0, 0.0, 0.0)
    s = 1.0 / (255.0 * pixels)
    return (rs * s, gs * s, bs * s)


def peak_score_bgra(
    image: bytes | bytearray,
    width: int,
    height: int,
    score_fn,
    *,
    y0: int | None = None,
    y1: int | None = None,
) -> float:
    return 0.0


def metric_walls_plausible(*args, **kwargs) -> bool:
    return False


def metric_ct_from_walls(*args, **kwargs):
    return None


def line_wall_hit(*args, **kwargs):
    return None


def forecast_wall_hit(*args, **kwargs):
    return None


def forecast_gap_error(*args, **kwargs):
    return None


def image_col_azimuth_rad(col: float, width: int, fov_rad: float) -> float:
    w = max(1, int(width))
    nx = (float(col) + 0.5) / w
    return (nx - 0.5) * float(fov_rad)


def pixel_to_ground_m(
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    col: float,
    row: float,
    width: int,
    height: int,
    fov_rad: float,
    floor_z: float = 0.0,
) -> tuple[float, float] | None:
    ay = image_row_elevation_rad(row, height, fov_rad)
    ax = image_col_azimuth_rad(col, width, fov_rad)
    ray_cam = _norm((math.tan(ax), math.tan(ay), -1.0))
    if ray_cam is None:
        return None
    ray = apply_sf_rotation(cam_rot, ray_cam)
    if ray[2] >= -1e-6:
        return None
    t = (floor_z - cam_pos[2]) / ray[2]
    if t < 0.0:
        return None
    return (cam_pos[0] + t * ray[0], cam_pos[1] + t * ray[1])


def pixel_to_ground_robot_m(
    robot_xy: tuple[float, float],
    yaw_rad: float,
    mount_pos: tuple[float, float, float],
    mount_rot: tuple[float, float, float, float],
    col: float,
    row: float,
    width: int,
    height: int,
    fov_rad: float,
    floor_z: float = 0.0,
) -> tuple[float, float] | None:
    ay = image_row_elevation_rad(row, height, fov_rad)
    ax = image_col_azimuth_rad(col, width, fov_rad)
    ray_cam = _norm((math.tan(ax), math.tan(ay), -1.0))
    if ray_cam is None:
        return None
    ray_body = apply_sf_rotation(mount_rot, ray_cam)
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    ray_w = (
        ray_body[0] * c - ray_body[1] * s,
        ray_body[0] * s + ray_body[1] * c,
        ray_body[2],
    )
    if ray_w[2] >= -1e-6:
        return None
    mx, my, mz = mount_pos
    cam_w = (
        float(robot_xy[0]) + mx * c - my * s,
        float(robot_xy[1]) + mx * s + my * c,
        mz,
    )
    t = (floor_z - cam_w[2]) / ray_w[2]
    if t < 0.0:
        return None
    hit_w = (cam_w[0] + t * ray_w[0], cam_w[1] + t * ray_w[1])
    dx = hit_w[0] - float(robot_xy[0])
    dy = hit_w[1] - float(robot_xy[1])
    return (dx * c + dy * s, -dx * s + dy * c)


def _mat9_mul(
    rot9: tuple[float, ...],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    r = rot9
    return (
        r[0] * vec[0] + r[1] * vec[1] + r[2] * vec[2],
        r[3] * vec[0] + r[4] * vec[1] + r[5] * vec[2],
        r[6] * vec[0] + r[7] * vec[1] + r[8] * vec[2],
    )


def pixel_to_ground_matrix_m(
    cam_world: tuple[float, float, float],
    cam_R9: tuple[float, ...],
    col: float,
    row: float,
    width: int,
    height: int,
    fov_rad: float,
    floor_z: float = 0.0,
) -> tuple[float, float] | None:
    if len(cam_R9) < 9:
        return None
    ay = image_row_elevation_rad(row, height, fov_rad)
    ax = image_col_azimuth_rad(col, width, fov_rad)
    ray_cam = _norm((math.tan(ax), math.tan(ay), -1.0))
    if ray_cam is None:
        return None
    ray = _mat9_mul(tuple(cam_R9[:9]), ray_cam)
    if ray[2] >= -1e-6:
        return None
    t = (floor_z - float(cam_world[2])) / ray[2]
    if t < 0.0:
        return None
    return (
        float(cam_world[0]) + t * ray[0],
        float(cam_world[1]) + t * ray[1],
    )


def world_hit_to_robot_m(
    hit_xy: tuple[float, float],
    robot_xy: tuple[float, float],
    yaw_rad: float,
) -> tuple[float, float]:
    dx = float(hit_xy[0]) - float(robot_xy[0])
    dy = float(hit_xy[1]) - float(robot_xy[1])
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    return (dx * c + dy * s, -dx * s + dy * c)


def yellow_best_pixel(
    image: bytes | bytearray,
    width: int,
    height: int,
    y0: int,
    y1: int,
    *,
    thresh: float = 0.20,
) -> tuple[float, float] | None:
    if width < 1 or height < 1 or not image or y1 <= y0:
        return None
    scale = 1.0 / 255.0
    buf = memoryview(image)
    nbytes = len(buf)
    y0 = max(0, int(y0))
    y1 = min(int(height), int(y1))
    best_s = float(thresh)
    best: tuple[float, float] | None = None
    for y in range(y0, y1):
        row = y * width * 4
        for x in range(width):
            o = row + x * 4
            if o + 2 >= nbytes:
                continue
            rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
            s = yellow_score(rgb)
            if s > best_s:
                best_s = s
                best = (float(x), float(y))
    return best


def yellow_wall_pixel(*args, **kwargs):
    return None


def nadir_lateral_m(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    cam_pos: tuple[float, float, float] = NADIR_MOUNT_POS,
    cam_rot: tuple[float, float, float, float] = NADIR_MOUNT_ROT,
    fov_rad: float = NADIR_FOV_RAD,
    robot_xy: tuple[float, float] = (0.0, 0.0),
    yaw_rad: float = 0.0,
    cam_world: tuple[float, float, float] | None = None,
    cam_R9: tuple[float, ...] | None = None,
    thresh: float = 0.20,
) -> float | None:
    """Robot-frame +Y of the paint ribbon (distance metric)."""
    w = int(width)
    h = int(height)
    if w < 2 or h < 2 or not image:
        return None

    def _hit(col: float, row: float) -> tuple[float, float] | None:
        if cam_world is not None and cam_R9 is not None:
            hit_w = pixel_to_ground_matrix_m(
                cam_world, cam_R9, col, row, w, h, fov_rad
            )
            if hit_w is None:
                return None
            return world_hit_to_robot_m(hit_w, robot_xy, float(yaw_rad))
        return pixel_to_ground_robot_m(
            robot_xy,
            float(yaw_rad),
            cam_pos,
            cam_rot,
            col,
            row,
            w,
            h,
            fov_rad,
        )

    axle = nadir_axle_row(cam_pos, cam_rot, h, fov_rad)
    if cam_world is not None and cam_R9 is not None:
        best_row = h // 2
        best_err = 1e9
        for row in range(h):
            hit = _hit(0.5 * (w - 1), float(row))
            if hit is None:
                continue
            err = abs(float(hit[0]) - float(NADIR_AXLE_AHEAD_M))
            if err < best_err:
                best_err = err
                best_row = row
        axle = best_row
    y0 = max(0, axle - 2)
    y1 = min(h, axle + 3)
    ys: list[float] = []
    for row in range(y0, y1):
        for col in range(w):
            rgb = _bgra_rgb_at(image, w, h, col, row)
            if rgb is None or yellow_score(rgb) < float(thresh):
                continue
            hit = _hit(float(col), float(row))
            if hit is not None:
                ys.append(float(hit[1]))
    if not ys:
        pix = yellow_best_pixel(image, w, h, 0, h, thresh=thresh)
        if pix is None:
            return None
        hit = _hit(pix[0], pix[1])
        if hit is None:
            return None
        return float(hit[1])
    ys.sort()
    return ys[len(ys) // 2]


def wall_dist_from_lateral(y_m: float, *, side: str) -> float:
    if str(side).lower().startswith("l"):
        return float(PAINT_Y_LEFT_M) - float(y_m)
    return float(y_m) - (-float(PAINT_Y_LEFT_M))
