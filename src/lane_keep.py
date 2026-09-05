"""Nadir-only lane keep.

Shoulder cameras looking down. Pixel gap vs 33/33 (128×128 remount), ahead HOLD vs 33/33,
6 cm stripe scale, v-scaled P/HOLD, steer.

Choir (picture-wins, wall bounce, LINE_CAM, forecast, planner stubs) is
archived at archives/lane_keep_stubs_2026-09-02.py. Restore that file
to undo.

Unprojection helpers stay because the 20 cm shove probe uses them.
They do not steer.
"""

from __future__ import annotations

import math

DEFAULT_CRUISE_RAD_S = 5.5
DEFAULT_K_STEER = 2.0
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

# Unprojection / 20 cm shove still uses the old look-down shoulder.
# Live left eye pose is in butlerbot.wbt (Daniel 2026-09-04 remount).
NADIR_MOUNT_POS = (0.01042, 0.41808, 0.72919)
NADIR_MOUNT_ROT = (0.0, 0.0, 1.0, -math.pi / 2.0)
NADIR_IMAGE = 64
NADIR_FOV_RAD = 1.2
NADIR_AXLE_AHEAD_M = 0.0

NADIR_BASE_L_PX = 33
NADIR_BASE_R_PX = 33
NADIR_AHEAD_L_PX = 33
NADIR_AHEAD_R_PX = 33
NADIR_PX_DEADBAND = 2
NADIR_AHEAD_DEADBAND = 1
NADIR_K_PX = 0.03
NADIR_K_HOLD = 0.03
NADIR_KD_PX = 0.12
NADIR_STEER_CAP = 0.55
NADIR_V_REF_M_S = 0.21
NADIR_V_SCALE_MAX = 2.2

_TRACK_START_M = 3.0
_TRACK_LOBE_M = 5.0
_TRACK_AMP_M = 1.0
_TRACK_SIGN = -1.0
FINISH_X_M = 16.5


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
    """Fight axle gaps back to left/right bases. Forward rows add HOLD. +steer = yaw right."""
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


def stripe_m_per_px(stripe_px: float) -> float | None:
    """Local stretch: 6 cm of tape / pixels on this row.

    Pan the camera up-track and the same tape is fewer pixels at the top
    of the frame. ``m_per_px`` grows with row. Do not reuse the axle
    stripe for the far band.
    """
    s = float(stripe_px)
    if s < 1.0:
        return None
    return STRIPE_W_M / s


def _median_int(vals: list[int]) -> int:
    vals = sorted(vals)
    return int(vals[len(vals) // 2])


def _median_float(vals: list[float]) -> float:
    vals = sorted(vals)
    return float(vals[len(vals) // 2])


def nadir_wheel_to_tape(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    side: str = "left",
    thresh: float = 0.22,
) -> dict | None:
    """Yellow ribbon = 6 cm. Gap from inner tape edge to that side's wheel.

    Each row uses its own stripe width as the scale. Pixel counts still
    steer (left/right bases). Per-row meters are for ahead speed later — they do
    not vote on the wheel.
    """
    w = int(width)
    h = int(height)
    if w < 4 or h < 4 or not image:
        return None
    right = str(side).lower().startswith("r")
    gaps: list[int] = []
    ahead_gaps: list[int] = []
    axle_gaps: list[int] = []
    stripes: list[int] = []
    ahead_stripes: list[int] = []
    axle_stripes: list[int] = []
    gap_m_all: list[float] = []
    ahead_m: list[float] = []
    axle_m: list[float] = []
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
        if stripe < 2 or stripe > 24:
            continue
        scale = stripe_m_per_px(stripe)
        if scale is None:
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
        gm = float(gap) * scale
        gaps.append(gap)
        stripes.append(stripe)
        gap_m_all.append(gm)
        tape_cols.append(tape_inner)
        wheel_cols.append(wheel)
        if row < h // 2:
            ahead_gaps.append(gap)
            ahead_stripes.append(stripe)
            ahead_m.append(gm)
        if (h // 2 - 6) <= row < (h // 2 + 6):
            axle_gaps.append(gap)
            axle_stripes.append(stripe)
            axle_m.append(gm)
    if not gaps or not stripes:
        return None
    gap_px = _median_int(gaps)
    stripe_px = _median_int(stripes)
    if stripe_px < 1:
        return None
    m = _median_float(axle_m) if axle_m else _median_float(gap_m_all)
    return {
        "gap_px": int(gap_px),
        "gap_ahead_px": None if not ahead_gaps else _median_int(ahead_gaps),
        "gap_axle_px": None if not axle_gaps else _median_int(axle_gaps),
        "stripe_px": int(stripe_px),
        "stripe_px_ahead": None if not ahead_stripes else _median_int(ahead_stripes),
        "stripe_px_axle": None if not axle_stripes else _median_int(axle_stripes),
        "m": m,
        "m_ahead": None if not ahead_m else _median_float(ahead_m),
        "m_axle": None if not axle_m else _median_float(axle_m),
        "m_per_px_ahead": (
            None if not ahead_stripes else stripe_m_per_px(_median_int(ahead_stripes))
        ),
        "m_per_px_axle": (
            None if not axle_stripes else stripe_m_per_px(_median_int(axle_stripes))
        ),
        "tape_col": _median_int(tape_cols),
        "wheel_col": _median_int(wheel_cols),
    }


def lane_keep_command(
    *_unused,
    cruise: float = DEFAULT_CRUISE_RAD_S,
    k_steer: float = DEFAULT_K_STEER,
    steer_filter: SteerFilter | None = None,
    left_gap_px: int | None = None,
    right_gap_px: int | None = None,
    left_ahead_px: int | None = None,
    right_ahead_px: int | None = None,
    nadir_guard: NadirGuard | None = None,
    dt: float = 0.008,
    **_ignored,
) -> dict:
    """Shoulder nadir only. Extra args are ignored; they do not steer."""
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
            "phase": "nadir_stop",
            "error_source": "nadir",
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
    return {
        "left": round(left, 3),
        "right": round(right, 3),
        "brake": False,
        "error": round(float(steer), 4),
        "steer": round(float(steer), 4),
        "reason": "nadir keep",
        "phase": "seek",
        "error_source": "nadir",
    }


# --- Unprojection for the 20 cm shove probe. Not on the wheel. ---


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


def image_row_elevation_rad(row: float, height: int, fov_rad: float) -> float:
    h = max(1, int(height))
    ny = (float(row) + 0.5) / h
    return (0.5 - ny) * float(fov_rad)


def image_col_azimuth_rad(col: float, width: int, fov_rad: float) -> float:
    w = max(1, int(width))
    nx = (float(col) + 0.5) / w
    return (nx - 0.5) * float(fov_rad)


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
