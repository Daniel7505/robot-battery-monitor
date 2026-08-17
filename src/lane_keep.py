"""Lane-keep + red-line brake — the onboard agent's first driving job.

Yellow edge cameras and a forward finish camera produce 0–1 color scores.
This module is the policy the Webots controller runs every sim step (PMS
ticks are too slow to steer). OnboardAgent uses the same function so the
dashboard log matches what the wheels do.

First rule: flat-floor geometry. Yellow pixels + camera pose → meters
to each wall. Push off if a wall is closer than comfort. Not “keep the
stripe in the middle of the picture.”

Second rule: a slower watch. If the snapshot disagrees with what that
geometry just commanded (implausible lane width, or a shove that makes
the wall closer), stop and re-evaluate. No map, no SLAM, no Gaussians.

Brain outputs a virtual steering wheel (steer in [-1, 1], speed).
The controller maps that onto the two hub speeds (no rack — biped later
can reuse the same steer number as a weight-shift). Peak scores still
decide lost-paint / red brake.
"""

from __future__ import annotations

import math

DEFAULT_CRUISE_RAD_S = 5.5
DEFAULT_K_STEER = 2.4
DEFAULT_STEER_DEADBAND = 0.01
DEFAULT_STEER_SLEW_PER_S = 12.0
DEFAULT_STEER_RELEASE_PER_S = 4.0
PLANNER_SAMPLE_S = 0.08
PLANNER_KP = 1.15
PLANNER_KD = 0.18
PLANNER_KAHEAD = 0.22
PLANNER_AHEAD_REF_S = 2.0
PREVIEW_FALLBACK_T_S = 2.0
PLANNER_CRUISE_ERR = 0.010
PLANNER_CRUISE_RATE = 0.12
PLANNER_D_CAP = 0.40
DEFAULT_PREVIEW_GAIN = 0.30
PREVIEW_SKY_FRAC = 0.80
PREVIEW_RANGE_MIN_M = 0.40
PREVIEW_RANGE_MAX_M = 4.00
DEFAULT_WALL_COMFORT = 0.16
DEFAULT_WALL_PREVIEW = 0.55
DEFAULT_WALL_CONTACT = 0.45
DEFAULT_FILL_COMFORT = 0.08
DEFAULT_FILL_GAIN = 8.0
DEFAULT_WALL_HIT = 0.18
DEFAULT_TURN_SLOW = 0.24
DEFAULT_RED_THRESH = 0.28
LOST_PAINT_YELLOW = 0.12
# Peak yellow on tile and on tape both sit ~0.53 (lit gold, not 0.95/0.95).
# A real stripe fills a few percent of the dirt box; desert speckle is ~1 px.
MIN_STRIPE_FILL = 0.03
MAX_WHEEL_RAD_S = 8.0
# Morning hold-cruise: 0.44 m/s parked ~3 cm after stop. 2.5 m/s² + 6 cm floor
# so we start ABS just before the stripe, not after we are already on it.
DEFAULT_ABS_DECEL_M_S2 = 2.5
MIN_STOP_DISTANCE_M = 0.06
WHEEL_RADIUS_M = 0.08
# Painted corridor: yellows at about ±0.65 m. Comfort is “too close.”
LANE_HALF_M = 0.65
WALL_COMFORT_M = 0.45
LANE_WIDTH_MIN_M = 0.70
LANE_WIDTH_MAX_M = 2.10
WATCH_HOLD_S = 0.20


def yellow_score(rgb: tuple[float, float, float]) -> float:
    """0–1: yellow paint (high R+G, low B) vs gray floor."""
    r, g, b = rgb
    return max(0.0, min(1.0, (r + g) * 0.5 - b))


def yellow_horizon_row(height: int) -> int:
    """First row that can still be dirt. Above this is heaven (~40%)."""
    return max(1, (int(height) * 2) // 5)


def yellow_look_band(height: int) -> tuple[int, int]:
    """Rows the brain scores: [y0, y1). HUD draws this exact box.

    Dirt / paint, just under the horizon — not sky. On the along-stripe
    64×64 eyes the old ¼–⅘ box sat half in the heavens. Change the
    band *here only* — overlays follow.
    """
    h = int(height)
    if h <= 20:
        return 0, h
    # ~50% … ~82% of the frame. Horizon on these cams is ~40%.
    y0 = max(1, h // 2)
    y1 = max(y0 + 1, (h * 13) // 16)
    return y0, y1


def yellow_far_band(height: int) -> tuple[int, int]:
    """Thin dirt sliver just under the horizon. Second measurement.

    Not sky (that is above yellow_horizon_row). Not the contact patch
    underfoot. A bend shows here first — if the eye is actually aimed
    down the stripe. Identity nadir fails the range interlock and
    preview stays off.
    """
    y0, y1 = yellow_look_band(height)
    horizon = yellow_horizon_row(height)
    far0 = max(y0, horizon)
    span = max(1, y1 - y0)
    far1 = min(y1, far0 + max(2, span // 3))
    if far1 <= far0:
        far1 = min(y1, far0 + 1)
    return far0, far1


def yellow_near_band(height: int) -> tuple[int, int]:
    """Close-floor part of the look band. Contact / error-now."""
    y0, y1 = yellow_look_band(height)
    _far0, far1 = yellow_far_band(height)
    span = max(1, y1 - y0)
    n0 = max(far1, y1 - max(2, span // 2))
    return n0, y1


def yellow_look_split(height: int) -> int:
    """Row between the far sliver and the near patch."""
    _far0, far1 = yellow_far_band(height)
    n0, _n1 = yellow_near_band(height)
    return (far1 + n0) // 2


def offset_to_column(offset: float, width: int) -> int:
    """Map [-1, 1] offset back to a pixel column for the HUD tick."""
    mid = 0.5 * float(max(1, int(width) - 1))
    col = mid + float(offset) * mid
    return int(max(0, min(int(width) - 1, round(col))))


def _band_offset(
    image: bytes | bytearray,
    width: int,
    height: int,
    y0: int,
    y1: int,
    *,
    thresh: float = 0.20,
) -> float | None:
    """Peak-yellow column in rows [y0, y1) → [-1, 1]. None if no paint."""
    if width < 2 or height < 1 or not image or y1 <= y0:
        return None
    scale = 1.0 / 255.0
    acc = 0.0
    wsum = 0.0
    buf = memoryview(image)
    nbytes = len(buf)
    y0 = max(0, int(y0))
    y1 = min(int(height), int(y1))
    for y in range(y0, y1):
        row = y * width * 4
        best_x = 0
        best_s = 0.0
        for x in range(width):
            o = row + x * 4
            if o + 2 >= nbytes:
                continue
            rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
            score = yellow_score(rgb)
            if score > best_s:
                best_s = score
                best_x = x
        if best_s >= thresh:
            # Smaller y = farther ahead. Far rows weigh more.
            weight = float((y1 - y) + 1)
            acc += float(best_x) * weight
            wsum += weight
    if wsum < 1e-6:
        return None
    col = acc / wsum
    mid = 0.5 * float(width - 1)
    return max(-1.0, min(1.0, (col - mid) / mid))


def _stripe_width(
    image: bytes | bytearray,
    width: int,
    height: int,
    y0: int,
    y1: int,
    *,
    thresh: float = 0.20,
) -> float:
    """Mean width of the brightest yellow bar in rows [y0, y1)."""
    if width < 1 or height < 1 or not image or y1 <= y0:
        return 0.0
    scale = 1.0 / 255.0
    buf = memoryview(image)
    nbytes = len(buf)
    y0 = max(0, int(y0))
    y1 = min(int(height), int(y1))
    acc = 0.0
    rows = 0
    for y in range(y0, y1):
        row = y * width * 4
        best_x = 0
        best_s = 0.0
        scores = [0.0] * width
        for x in range(width):
            o = row + x * 4
            if o + 2 >= nbytes:
                continue
            rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
            s = yellow_score(rgb)
            scores[x] = s
            if s > best_s:
                best_s = s
                best_x = x
        if best_s < thresh:
            continue
        lo = best_x
        hi = best_x
        while lo > 0 and scores[lo - 1] >= thresh:
            lo -= 1
        while hi + 1 < width and scores[hi + 1] >= thresh:
            hi += 1
        acc += float(hi - lo + 1) / float(width)
        rows += 1
    if rows <= 0:
        return 0.0
    return acc / float(rows)


def yellow_band_fill(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> float:
    """Mean width of the main stripe on the *dirt* look band.

    Not the top of the overlay — that is sky on these eyes. The box
    sits just under the horizon. Closer → fatter, when 64×64 can
    actually see it.
    """
    y0, y1 = yellow_look_band(height)
    return _stripe_width(image, width, height, y0, y1, thresh=thresh)


def yellow_near_fill(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> float:
    """Near-stripe width. Contact only — too late to be the only steer."""
    y0, y1 = yellow_near_band(height)
    return _stripe_width(image, width, height, y0, y1, thresh=thresh)


def yellow_nearest_pixel(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> tuple[float, float] | None:
    """Nearest yellow in the dirt box: (column, row). None if no paint.

    Bottom of the box is closest floor. That pixel is the wall beside us,
    not the vanishing point.
    """
    if width < 1 or height < 1 or not image:
        return None
    y0, y1 = yellow_look_band(height)
    y0 = max(0, int(y0))
    y1 = min(int(height), int(y1))
    scale = 1.0 / 255.0
    buf = memoryview(image)
    nbytes = len(buf)
    for y in range(y1 - 1, y0 - 1, -1):
        row = y * width * 4
        best_x = 0
        best_s = 0.0
        for x in range(width):
            o = row + x * 4
            if o + 2 >= nbytes:
                continue
            rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
            s = yellow_score(rgb)
            if s > best_s:
                best_s = s
                best_x = x
        if best_s >= thresh:
            return (float(best_x), float(y))
    return None


def yellow_ahead_pixel(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> tuple[float, float] | None:
    """Yellow at the *top* of the dirt box — the wall ahead, not underfoot.

    Nearest (bottom) pixel sits on the mount. That range freezes at
    spawn because the camera is bolted over the paint. Ahead is where
    a bend shows up as a new ground hit.
    """
    if width < 1 or height < 1 or not image:
        return None
    y0, y1 = yellow_look_band(height)
    y0 = max(0, int(y0))
    y1 = min(int(height), int(y1))
    scale = 1.0 / 255.0
    buf = memoryview(image)
    nbytes = len(buf)
    for y in range(y0, y1):
        row = y * width * 4
        best_x = 0
        best_s = 0.0
        for x in range(width):
            o = row + x * 4
            if o + 2 >= nbytes:
                continue
            rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
            s = yellow_score(rgb)
            if s > best_s:
                best_s = s
                best_x = x
        if best_s >= thresh:
            return (float(best_x), float(y))
    return None


def yellow_line_offset(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> float | None:
    """Main-stripe column, mapped to [-1, 1] (left … right).

    Scores yellow_look_band() only — the same box the L/R overlays outline.
    Each row keeps only its brightest yellow pixel.
    """
    y0, y1 = yellow_look_band(height)
    return _band_offset(image, width, height, y0, y1, thresh=thresh)


def yellow_far_offset(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> float | None:
    """Stripe column in the far dirt sliver. None if no paint there."""
    y0, y1 = yellow_far_band(height)
    return _band_offset(image, width, height, y0, y1, thresh=thresh)


def yellow_near_offset(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> float | None:
    """Stripe column in the near dirt patch. Error-now."""
    y0, y1 = yellow_near_band(height)
    return _band_offset(image, width, height, y0, y1, thresh=thresh)


def band_mean_rgb(
    image: bytes | bytearray,
    width: int,
    height: int,
    y0: int,
    y1: int,
) -> tuple[float, float, float]:
    """Average RGB of rows [y0, y1). (0,0,0) if the box is empty."""
    if width < 1 or height < 1 or not image or y1 <= y0:
        return (0.0, 0.0, 0.0)
    scale = 1.0 / 255.0
    buf = memoryview(image)
    nbytes = len(buf)
    y0 = max(0, int(y0))
    y1 = min(int(height), int(y1))
    rs = gs = bs = 0
    n = 0
    for y in range(y0, y1):
        row = y * width * 4
        for x in range(width):
            o = row + x * 4
            if o + 2 >= nbytes:
                continue
            bs += buf[o]
            gs += buf[o + 1]
            rs += buf[o + 2]
            n += 1
    if n <= 0:
        return (0.0, 0.0, 0.0)
    inv = scale / float(n)
    return (rs * inv, gs * inv, bs * inv)


def band_sky_frac(
    image: bytes | bytearray,
    width: int,
    height: int,
    y0: int,
    y1: int,
) -> float:
    """Fraction of sampled pixels in the band classified as sky."""
    if width < 1 or height < 1 or not image or y1 <= y0:
        return 1.0
    scale = 1.0 / 255.0
    buf = memoryview(image)
    nbytes = len(buf)
    y0 = max(0, int(y0))
    y1 = min(int(height), int(y1))
    sky = 0
    n = 0
    stride = 2 if width >= 16 else 1
    for y in range(y0, y1):
        row = y * width * 4
        for x in range(0, width, stride):
            o = row + x * 4
            if o + 2 >= nbytes:
                continue
            rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
            n += 1
            if classify_view(rgb) == "sky":
                sky += 1
    if n <= 0:
        return 1.0
    return float(sky) / float(n)


def preview_band_ok(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> bool:
    """Far sliver is dirt/paint, not heaven. False → drop the second measurement.

    A just-under-horizon sliver is *supposed* to mix dirt and sky. Mean
    ground/yellow, or a yellow bar, is enough. Only a mostly-sky box
    is rejected. Does not remount the camera.
    """
    y0, y1 = yellow_far_band(height)
    if y1 <= y0:
        return False
    if y0 < yellow_horizon_row(height):
        return False
    label = classify_view(band_mean_rgb(image, width, height, y0, y1))
    if label in ("ground", "yellow"):
        return True
    fill = _stripe_width(image, width, height, y0, y1, thresh=thresh)
    if fill >= 0.01:
        return True
    return band_sky_frac(image, width, height, y0, y1) < PREVIEW_SKY_FRAC


def dirt_band_ok(
    image: bytes | bytearray,
    width: int,
    height: int,
) -> bool:
    """Whole dirt box is usable (aim check). Sky here means the eye is drunk."""
    y0, y1 = yellow_look_band(height)
    if y1 <= y0:
        return False
    label = classify_view(band_mean_rgb(image, width, height, y0, y1))
    if label in ("ground", "yellow"):
        return True
    return band_sky_frac(image, width, height, y0, y1) < PREVIEW_SKY_FRAC


def preview_look_ahead_m(
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    height: int,
    fov_rad: float,
    floor_z: float = 0.0,
) -> float | None:
    """Robot-frame +X where the far sliver hits the floor.

    None if every far-band row is sky or the hit is not a useful
    look-ahead (nadir under the mount, or a horizon graze).
    """
    y0, y1 = yellow_far_band(height)
    for row in range(int(y0), int(y1)):
        d = image_row_to_ground_ahead_m(
            cam_pos, cam_rot, float(row), height, fov_rad, floor_z
        )
        if d is None:
            continue
        if PREVIEW_RANGE_MIN_M <= float(d) <= PREVIEW_RANGE_MAX_M:
            return float(d)
    return None


def yellow_line_curve(
    image: bytes | bytearray,
    width: int,
    height: int,
    *,
    thresh: float = 0.20,
) -> float:
    """How the stripe bends inside the look band.

    Far-sliver offset minus near-patch offset. Negative = paint moves
    left ahead (road curving left). 0 if either half has no paint. Any
    two yellow edges, not this S.
    """
    fy0, fy1 = yellow_far_band(height)
    ny0, ny1 = yellow_near_band(height)
    far = _band_offset(image, width, height, fy0, fy1, thresh=thresh)
    near = _band_offset(image, width, height, ny0, ny1, thresh=thresh)
    if far is None or near is None:
        return 0.0
    return max(-1.0, min(1.0, float(far) - float(near)))


ONE_EYE_STEER = 0.70


def wall_pressure(
    offset: float | None,
    *,
    side: str,
    fill: float | None = None,
    curve: float | None = None,
    comfort: float = DEFAULT_WALL_COMFORT,
    preview_gain: float = DEFAULT_WALL_PREVIEW,
    contact: float = DEFAULT_WALL_CONTACT,
    fill_comfort: float = DEFAULT_FILL_COMFORT,
    fill_gain: float = DEFAULT_FILL_GAIN,
    hit: float = DEFAULT_WALL_HIT,
) -> float:
    """How hard this yellow wall is shoving us away. 0 = not close.

    Fill is far-stripe width (what's ahead). Past ``hit`` the wall is
    contact. Near-band is too late to steer on. Offset-only is a
    fallback if fill is missing.
    """
    del curve, preview_gain  # kept on the signature so callers stay stable
    close = 0.0
    if fill is not None:
        f = float(fill)
        if f >= float(hit):
            close = 1.0
        else:
            close = max(0.0, f - float(fill_comfort)) * float(fill_gain)
    elif offset is not None:
        inward = float(offset) if side == "left" else -float(offset)
        band = float(comfort)
        close = max(0.0, inward - band)
        if inward > band + 0.20:
            close += (inward - band - 0.20) * float(contact)
    return min(1.5, close)


def steer_from_walls(
    left_offset: float | None,
    right_offset: float | None,
    *,
    deadband: float = DEFAULT_STEER_DEADBAND,
    left_curve: float | None = None,
    right_curve: float | None = None,
    left_fill: float | None = None,
    right_fill: float | None = None,
    comfort: float = DEFAULT_WALL_COMFORT,
    preview_gain: float = DEFAULT_WALL_PREVIEW,
) -> float | None:
    """Virtual wheel from corridor walls. +1 = push off left (yaw right).

    None if neither eye has a stripe or a fill. 0 if both walls are
    equally close — that is “keep rolling,” not “drive to y=0.”
    """
    if (
        left_offset is None
        and right_offset is None
        and left_fill is None
        and right_fill is None
    ):
        return None
    left_p = wall_pressure(
        left_offset,
        side="left",
        fill=left_fill,
        curve=left_curve,
        comfort=comfort,
        preview_gain=preview_gain,
    )
    right_p = wall_pressure(
        right_offset,
        side="right",
        fill=right_fill,
        curve=right_curve,
        comfort=comfort,
        preview_gain=preview_gain,
    )
    raw = left_p - right_p
    if abs(raw) < float(deadband):
        return 0.0
    return max(-1.0, min(1.0, raw))


def steer_from_offsets(
    left_offset: float | None,
    right_offset: float | None,
    *,
    deadband: float = DEFAULT_STEER_DEADBAND,
    left_yellow: float | None = None,
    right_yellow: float | None = None,
    left_curve: float | None = None,
    right_curve: float | None = None,
    preview_gain: float = DEFAULT_PREVIEW_GAIN,
) -> float | None:
    """One virtual wheel from both eyes. −1 full left … +1 full right.

    Opposite L/R ticks are two *measurements*, not two hubs fighting.
    The mean is where we sit in the lane now. Curve is whether the
    paint is sliding left/right farther ahead — any track shape, not
    a pre-mapped S. Preview fades when we are already off-center so
    a lobe we are already correcting does not get double-steered.
    """
    vals = [float(o) for o in (left_offset, right_offset) if o is not None]
    if not vals:
        return None
    pos = sum(vals) / len(vals)
    curves = [float(c) for c in (left_curve, right_curve) if c is not None]
    curve = sum(curves) / len(curves) if curves else 0.0
    fade = max(0.0, 1.0 - min(1.0, abs(pos)))
    raw = pos + float(preview_gain) * curve * fade
    if abs(raw) < float(deadband):
        return 0.0
    return max(-1.0, min(1.0, raw))


def preview_ahead_weight(t_ahead_s: float | None) -> float:
    """Scale the far-sliver error by time-to-that-point.

    Same look-ahead at higher speed → less time → heavier weight.
    None / parked → 0 (do not steer on a guess).
    """
    if t_ahead_s is None:
        return 0.0
    t = float(t_ahead_s)
    if t <= 0.05:
        return 0.0
    return max(0.25, min(1.2, PLANNER_AHEAD_REF_S / t))


class GapPlanner:
    """Slow intent: stay in the yellow gap, go to red. No map.

    Action still runs every ~8 ms. This updates ~10 Hz: error now +
    error ahead (time-to-point) + how fast the error is growing.
    Cruise when the gap is fine. Correct while it is not. Does not
    latch until centerline.
    """

    def __init__(
        self,
        sample_s: float = PLANNER_SAMPLE_S,
        kp: float = PLANNER_KP,
        kd: float = PLANNER_KD,
        kahead: float = PLANNER_KAHEAD,
    ) -> None:
        self.sample_s = float(sample_s)
        self.kp = float(kp)
        self.kd = float(kd)
        self.kahead = float(kahead)
        self.mode = "cruise"
        self.desired_steer = 0.0
        self.error = 0.0
        self.error_ahead = 0.0
        self.t_ahead = None
        self.rate = 0.0
        self._acc = 0.0
        self._last_err = 0.0
        self._have = False

    def reset(self) -> None:
        self.mode = "cruise"
        self.desired_steer = 0.0
        self.error = 0.0
        self.error_ahead = 0.0
        self.t_ahead = None
        self.rate = 0.0
        self._acc = 0.0
        self._last_err = 0.0
        self._have = False

    def step(
        self,
        error: float | None,
        dt: float,
        *,
        lost: bool = False,
        red: bool = False,
        error_ahead: float | None = None,
        t_ahead: float | None = None,
    ) -> dict:
        dt = max(0.0, float(dt))
        self.t_ahead = None if t_ahead is None else float(t_ahead)
        self.error_ahead = 0.0 if error_ahead is None else float(error_ahead)
        if lost:
            self.mode = "recover"
            self.desired_steer = 0.0
            return self._snap()
        if red:
            self.mode = "seek_red"
            return self._snap()
        if error is None:
            self.mode = "cruise"
            self.desired_steer = 0.0
            self.error = 0.0
            return self._snap()
        e = max(-1.0, min(1.0, float(error)))
        self.error = e
        self._acc += dt
        if self._acc >= self.sample_s:
            if self._have:
                self.rate = (e - self._last_err) / max(1e-3, self._acc)
            self._last_err = e
            self._acc = 0.0
            self._have = True
        ahead = 0.0
        if error_ahead is not None:
            ea = max(-1.0, min(1.0, float(error_ahead)))
            self.error_ahead = ea
            ahead = self.kahead * ea * preview_ahead_weight(t_ahead)
            # Now wins when the sides already disagree. Future may
            # start a turn, not yank us off a correction.
            if abs(e) >= 0.15:
                # Loop 2 hard-reversed on far-vs-now and spiked 27 cm
                # on the last straight. Give ahead a little more voice
                # at a crest without treating a noisy sliver as a flip.
                ahead *= 0.45
        p_term = self.kp * e
        d_term = self.kd * self.rate
        # D may soften the pull, never reverse it. Last S the eyes
        # still saw the miss while steer flipped the other way.
        if p_term * d_term < 0.0:
            cap = PLANNER_D_CAP * abs(p_term)
            d_term = max(-cap, min(cap, d_term))
        raw = p_term + d_term + ahead
        # Ahead can start a correct while now still looks centered.
        ahead_busy = abs(ahead) >= 0.04
        if (
            abs(e) < PLANNER_CRUISE_ERR
            and abs(self.rate) < PLANNER_CRUISE_RATE
            and not ahead_busy
        ):
            self.mode = "cruise"
            raw = 0.0
        else:
            self.mode = "correct"
        self.desired_steer = max(-1.0, min(1.0, raw))
        return self._snap()

    def _snap(self) -> dict:
        return {
            "mode": self.mode,
            "desired_steer": self.desired_steer,
            "error": round(self.error, 4),
            "error_ahead": round(self.error_ahead, 4),
            "rate": round(self.rate, 4),
            "t_ahead": None if self.t_ahead is None else round(float(self.t_ahead), 3),
        }


class SteerFilter:
    """Slew the virtual wheel so both hubs ease in together.

    Grab (larger |steer|) is fast. Same-sign release is slow so we keep
    tugging when the planner eases. Sign flip is a new bend — drop the
    old hold at grab speed.
    """

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
        # Same-sign ease = hold (slow release). Sign flip = new bend,
        # drop the old tug at grab speed. Going to 0 is still a release.
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
    """Both hubs stay forward. steer < 0 → left slower, right faster (yaw left).

    Harder turn → lower cruise so yaw can catch a bend (any map).
    """
    s = float(steer)
    slow = max(0.0, min(0.6, float(turn_slow))) * min(1.0, abs(s))
    cruise_eff = float(cruise) * (1.0 - slow)
    left = cruise_eff + k_steer * s
    right = cruise_eff - k_steer * s
    # Keep tandem roll — never flip a hub to reverse for a lane tweak.
    floor = 0.35 * abs(float(cruise))
    left = max(floor, min(MAX_WHEEL_RAD_S, left))
    right = max(floor, min(MAX_WHEEL_RAD_S, right))
    return left, right


def red_score(rgb: tuple[float, float, float]) -> float:
    """0–1: red finish bar vs gray / yellow / green."""
    r, g, b = rgb
    return max(0.0, min(1.0, r - max(g, b)))


def mean_rgb_bgra(image: bytes | bytearray, width: int, height: int) -> tuple[float, float, float]:
    """Average RGB from a Webots Camera BGRA buffer (channels 0–1)."""
    n = max(1, width * height)
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
    scale = 1.0 / (pixels * 255.0)
    return (rs * scale, gs * scale, bs * scale)


def peak_score_bgra(
    image: bytes | bytearray,
    width: int,
    height: int,
    score_fn,
    *,
    percentile: float = 0.98,
) -> float:
    """Score every pixel; return a high percentile so a thin stripe still counts.

    Mean-of-frame washed a 2 cm red finish line to 0.0. Tracks stay as painted —
    the eye has to find the mark, not the other way around.
    """
    buf = memoryview(image)
    pixels = min(width * height, len(buf) // 4)
    if pixels <= 0:
        return 0.0
    scale = 1.0 / 255.0
    stride = 1 if pixels <= 5000 else 2
    scores: list[float] = []
    for i in range(0, pixels, stride):
        o = i * 4
        rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
        scores.append(float(score_fn(rgb)))
    scores.sort()
    idx = min(len(scores) - 1, max(0, int(len(scores) * percentile)))
    return scores[idx]


def stopping_distance_m(
    speed_m_s: float,
    *,
    decel_m_s2: float = DEFAULT_ABS_DECEL_M_S2,
    min_m: float = MIN_STOP_DISTANCE_M,
) -> float:
    """How far the body still travels after ABS is requested."""
    v = abs(float(speed_m_s))
    a = max(0.1, float(decel_m_s2))
    return max(float(min_m), (v * v) / (2.0 * a))


def time_to_mark_s(look_ahead_m: float, speed_m_s: float) -> float | None:
    """Seconds to the ground-hit if we hold current speed. None if parked."""
    v = abs(float(speed_m_s))
    if v < 0.02:
        return None
    return max(0.0, float(look_ahead_m)) / v


def cruise_speed_m_s(wheel_rad_s: float = DEFAULT_CRUISE_RAD_S) -> float:
    return abs(float(wheel_rad_s)) * WHEEL_RADIUS_M


class MarkStopTracker:
    """Range the mark from the red ray. Coast until remaining <= stop distance.

    ``measured_range_m`` is where *this* image row hits the floor. While red
    is in frame that live range wins. After it leaves, count down from the
    last range so we still stop if we just rolled onto the stripe.
    """

    def __init__(self) -> None:
        self.remaining_m: float | None = None
        self.seen: bool = False

    def reset(self) -> None:
        self.remaining_m = None
        self.seen = False

    def step(
        self,
        red_seen: bool,
        look_ahead_m: float,
        speed_m_s: float,
        dt: float,
        measured_range_m: float | None = None,
    ) -> dict:
        v = abs(float(speed_m_s))
        d_look = max(0.0, float(look_ahead_m))
        d_stop = stopping_distance_m(v)
        dt = max(0.0, float(dt))
        live = None
        if measured_range_m is not None:
            live = max(0.0, float(measured_range_m))
        elif red_seen:
            live = d_look
        if live is not None and red_seen:
            if not self.seen or self.remaining_m is None:
                self.seen = True
                self.remaining_m = live
            else:
                # Row can stick. Remaining may only shrink.
                self.remaining_m = min(self.remaining_m - v * dt, live)
        elif self.remaining_m is not None:
            self.remaining_m -= v * dt
        if self.remaining_m is None:
            t = time_to_mark_s(d_look, v)
            return {
                "brake": False,
                "phase": "seek",
                "remaining_m": None,
                "t_to_mark_s": t,
                "d_stop_m": d_stop,
            }
        t_mark = time_to_mark_s(self.remaining_m, v)
        if self.remaining_m <= d_stop:
            return {
                "brake": True,
                "phase": "brake",
                "remaining_m": round(self.remaining_m, 4),
                "t_to_mark_s": t_mark,
                "d_stop_m": d_stop,
            }
        return {
            "brake": False,
            "phase": "coast",
            "remaining_m": round(self.remaining_m, 4),
            "t_to_mark_s": t_mark,
            "d_stop_m": d_stop,
        }


def _eye_has_paint(yellow: float, fill: float | None) -> bool:
    """True if this eye still sees a stripe, not a tile speckle.

    Peak-only when fill was not measured (unit tests / old callers).
    """
    if float(yellow) < LOST_PAINT_YELLOW:
        return False
    if fill is None:
        return True
    return float(fill) >= MIN_STRIPE_FILL


def lane_keep_command(
    left_yellow: float,
    right_yellow: float,
    finish_red: float,
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
    planner: GapPlanner | None = None,
    watch_plan: dict | None = None,
    lookout: ForecastLookout | None = None,
    z_fill: float | None = None,
    w_fill: float | None = None,
    dt: float = 0.008,
) -> dict:
    """Return wheel cmds. Every frame's picture counts.

    Meters win only when they say a wall is *close*. If meters still
    look like spawn (centered) but this frame's yellow has slid in the
    camera, steer on that slide — we do re-read the image; we must not
    throw the slide away.
    """
    lost_paint = not _eye_has_paint(left_yellow, left_fill) and not _eye_has_paint(
        right_yellow, right_fill
    )
    range_steer = steer_from_ranges(left_y_m, right_y_m)
    picture_steer = steer_from_offsets(
        left_offset,
        right_offset,
        left_curve=left_curve,
        right_curve=right_curve,
    )
    if picture_steer is None:
        picture_steer = steer_from_walls(
            left_offset,
            right_offset,
            left_curve=left_curve,
            right_curve=right_curve,
            left_fill=left_fill,
            right_fill=right_fill,
        )
    left_c = None if left_y_m is None else float(left_y_m)
    right_c = None if right_y_m is None else -float(right_y_m)
    band = WALL_COMFORT_M
    range_left_p = (
        0.0
        if left_c is None
        else max(0.0, (band - left_c) / max(0.05, band))
    )
    range_right_p = (
        0.0
        if right_c is None
        else max(0.0, (band - right_c) / max(0.05, band))
    )
    wall_close = range_steer is not None and abs(float(range_steer)) >= DEFAULT_STEER_DEADBAND
    if wall_close:
        raw_steer = range_steer
        left_p = range_left_p
        right_p = range_right_p
    elif picture_steer is not None:
        raw_steer = picture_steer
        left_p = wall_pressure(
            left_offset, side="left", fill=left_fill, curve=left_curve
        )
        right_p = wall_pressure(
            right_offset, side="right", fill=right_fill, curve=right_curve
        )
    else:
        raw_steer = range_steer
        left_p = range_left_p
        right_p = range_right_p
    if raw_steer is None:
        # No geometry this frame — louder yellow is the closer wall.
        raw_steer = float(left_yellow) - float(right_yellow)
        if abs(raw_steer) < DEFAULT_STEER_DEADBAND:
            raw_steer = 0.0
        raw_steer = max(-1.0, min(1.0, raw_steer))
        left_p = max(0.0, float(left_yellow) - float(right_yellow))
        right_p = max(0.0, float(right_yellow) - float(left_yellow))
    elif not lost_paint and allow_one_eye:
        # One eye dark: we are on that wall or through it. Push toward
        # the paint that remains — do not treat a missing wall as "clear."
        # Off until both eyes have been alive (spawn lazy-eye is not contact).
        if (
            float(left_yellow) >= LOST_PAINT_YELLOW
            and float(right_yellow) < LOST_PAINT_YELLOW
        ):
            raw_steer = max(-1.0, min(1.0, float(raw_steer) - ONE_EYE_STEER))
            right_p = max(right_p, ONE_EYE_STEER)
        elif (
            float(right_yellow) >= LOST_PAINT_YELLOW
            and float(left_yellow) < LOST_PAINT_YELLOW
        ):
            raw_steer = max(-1.0, min(1.0, float(raw_steer) + ONE_EYE_STEER))
            left_p = max(left_p, ONE_EYE_STEER)
    plan = None
    err_ahead = None
    t_use = None
    # Z/W meters stay off the wheel. They measure 12 cm of floor under
    # the lens, not upcoming tape. Far L/R picture may still preview.
    if preview_ok:
        err_ahead = steer_from_offsets(left_far_offset, right_far_offset)
        t_use = t_ahead
    if planner is not None:
        plan = planner.step(
            raw_steer,
            dt,
            lost=lost_paint,
            red=bool(mark_plan is not None and mark_plan.get("brake")),
            error_ahead=err_ahead,
            t_ahead=t_use,
        )
        if not lost_paint:
            raw_steer = plan["desired_steer"]
    if steer_filter is not None:
        steer = steer_filter.step(raw_steer, dt)
    else:
        steer = raw_steer
    err = float(steer)
    left, right = wheels_from_steer(err, cruise=cruise, k_steer=k_steer)
    remaining = None if mark_plan is None else mark_plan.get("remaining_m")
    if mark_plan is not None and mark_plan.get("brake"):
        return {
            "left": 0.0,
            "right": 0.0,
            "brake": True,
            "error": 0.0,
            "reason": "red mark — stop on line",
            "remaining_m": remaining,
            "phase": mark_plan.get("phase"),
        }
    if finish_red >= red_thresh and mark_plan is None:
        return {
            "left": 0.0,
            "right": 0.0,
            "brake": True,
            "error": 0.0,
            "reason": "red finish — brake",
            "remaining_m": None,
            "phase": "brake",
        }
    if lookout is not None:
        ahead = lookout.step(z_fill, w_fill, dt)
        if ahead.get("abort"):
            return {
                "left": 0.0,
                "right": 0.0,
                "brake": True,
                "error": 0.0,
                "steer": 0.0,
                "reason": f"lookout — {ahead.get('reason') or 'paint gone ahead'}",
                "remaining_m": remaining,
                "phase": "lookout",
                "z_fill": None if z_fill is None else round(float(z_fill), 3),
                "w_fill": None if w_fill is None else round(float(w_fill), 3),
            }
    if watch_plan is not None and watch_plan.get("abort"):
        return {
            "left": 0.0,
            "right": 0.0,
            "brake": True,
            "error": 0.0,
            "steer": 0.0,
            "reason": f"watch — {watch_plan.get('reason') or 'stop'}",
            "remaining_m": remaining,
            "phase": "watch",
            "left_y_m": None if left_y_m is None else round(float(left_y_m), 3),
            "right_y_m": None if right_y_m is None else round(float(right_y_m), 3),
        }
    if lost_paint:
        # Both eyes dark: stop. Cruising a guessed heading is how we
        # drove into the desert last lap.
        return {
            "left": 0.0,
            "right": 0.0,
            "brake": True,
            "error": 0.0,
            "steer": 0.0,
            "reason": "lost paint — stop",
            "remaining_m": remaining,
            "phase": "lost",
        }
    reason = "corridor keep"
    phase = "seek"
    if mark_plan is not None and mark_plan.get("phase") == "coast":
        reason = "red seen, coast to mark"
        phase = "coast"
    return {
        "left": round(left, 3),
        "right": round(right, 3),
        "brake": False,
        "error": round(err, 4),
        "steer": None if steer is None else round(float(steer), 4),
        "left_pressure": round(float(left_p), 4),
        "right_pressure": round(float(right_p), 4),
        "left_y_m": None if left_y_m is None else round(float(left_y_m), 3),
        "right_y_m": None if right_y_m is None else round(float(right_y_m), 3),
        "reason": reason,
        "remaining_m": remaining,
        "phase": phase,
        "plan_mode": None if plan is None else plan["mode"],
        "plan_rate": None if plan is None else plan["rate"],
        "preview_ok": bool(err_ahead is not None),
        "err_ahead": None if err_ahead is None else round(float(err_ahead), 4),
        "t_ahead": None if t_use is None else round(float(t_use), 3),
        "z_y_m": None if z_y_m is None else round(float(z_y_m), 3),
        "w_y_m": None if w_y_m is None else round(float(w_y_m), 3),
    }


# World colors from butlerbot.wbt — interlock, not paint we change.
SKY_RGB = (0.72, 0.76, 0.82)
FLOOR_RGB = (0.55, 0.56, 0.58)


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


def ground_hit_ahead_m(
    cam_pos: tuple[float, float, float],
    target_pos: tuple[float, float, float],
    floor_z: float = 0.0,
) -> float:
    """Robot-frame +X where the look ray meets the floor. That is D."""
    dx = target_pos[0] - cam_pos[0]
    dz = target_pos[2] - cam_pos[2]
    if abs(dz) < 1e-9:
        return max(0.0, float(target_pos[0]))
    s = (floor_z - cam_pos[2]) / dz
    if s < 0.0:
        return 0.0
    return max(0.0, cam_pos[0] + s * dx)


def image_row_elevation_rad(row: float, height: int, fov_rad: float) -> float:
    """Elevation from the look axis. Row 0 is the top of the image (+up)."""
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
    """Robot-frame +X where this image row hits the floor. None if sky."""
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


def image_col_azimuth_rad(col: float, width: int, fov_rad: float) -> float:
    """Azimuth from the look axis. Col 0 is the left of the image."""
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
    """Parent-frame (ahead_m, lateral_m) where this pixel hits the floor.

    None if the ray misses the floor (sky). Webots FOV is vertical; square
    pixels share that angle horizontally.
    """
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


def yellow_best_pixel(
    image: bytes | bytearray,
    width: int,
    height: int,
    y0: int,
    y1: int,
    *,
    thresh: float = 0.20,
) -> tuple[float, float] | None:
    """Brightest yellow pixel in rows [y0, y1). (col, row) or None."""
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


def forecast_wall_hit(
    image: bytes | bytearray,
    width: int,
    height: int,
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    fov_rad: float,
    *,
    robot_xy: tuple[float, float] | None = None,
    yaw_rad: float | None = None,
    thresh: float = 0.20,
) -> dict | None:
    """Yellow pixel → floor hit. ahead_m and y_m in robot frame.

    The slant *is* the ray. Row is how far ahead; column is which way.
    None if no paint or the ray misses the floor.
    """
    h = int(height)
    y0 = max(1, h // 8)
    y1 = max(y0 + 1, (h * 7) // 8)
    pix = yellow_best_pixel(image, width, height, y0, y1, thresh=thresh)
    if pix is None:
        return None
    col, row = pix
    if robot_xy is not None and yaw_rad is not None:
        hit = pixel_to_ground_robot_m(
            robot_xy,
            float(yaw_rad),
            cam_pos,
            cam_rot,
            col,
            row,
            width,
            height,
            fov_rad,
        )
    else:
        hit = pixel_to_ground_m(
            cam_pos, cam_rot, col, row, width, height, fov_rad
        )
    if hit is None:
        return None
    return {
        "ahead_m": float(hit[0]),
        "y_m": float(hit[1]),
        "col": col,
        "row": row,
    }


def forecast_gap_error(
    left_y_m: float | None,
    right_y_m: float | None,
) -> float | None:
    """Z=W in meters. +1 = left wall closer ahead (steer right).

    Left paint lives at +Y, right at −Y. Clearance is distance from
    body center. Equal clearances → 0. None if neither wall hit.
    """
    left_c = None if left_y_m is None else float(left_y_m)
    right_c = None if right_y_m is None else -float(right_y_m)
    if left_c is None and right_c is None:
        return None
    if left_c is None or right_c is None:
        return 0.0
    width = left_c + right_c
    if abs(width) < 0.08:
        return 0.0
    return max(-1.0, min(1.0, (right_c - left_c) / max(0.15, width)))


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
    """World ray from the moving robot, then hit in robot-frame (ahead, y).

    Camera translation in the .wbt is the *mount*, not world pose. Using
    that alone freezes the tape measure at spawn.
    """
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
    ahead = dx * c + dy * s
    lateral = -dx * s + dy * c
    return (ahead, lateral)


def steer_from_ranges(
    left_y_m: float | None,
    right_y_m: float | None,
    *,
    comfort_m: float = WALL_COMFORT_M,
    deadband: float = DEFAULT_STEER_DEADBAND,
) -> float | None:
    """Virtual wheel from wall ranges in robot-frame Y. +1 = push off left.

    Left paint lives at +Y (~+0.65 when centered). Right at −Y. Clearance
    is how far that paint is from the body center. Below comfort → shove.
    None if neither wall has a ground hit.
    """
    left_c = None if left_y_m is None else float(left_y_m)
    right_c = None if right_y_m is None else -float(right_y_m)
    if left_c is None and right_c is None:
        return None
    band = max(0.05, float(comfort_m))

    def _pressure(clearance: float | None) -> float:
        if clearance is None:
            return 0.0
        return max(0.0, min(1.5, (band - float(clearance)) / band))

    raw = _pressure(left_c) - _pressure(right_c)
    if abs(raw) < float(deadband):
        return 0.0
    return max(-1.0, min(1.0, raw))


class ForecastLookout:
    """Z/W never steer. After they have seen a stripe, both gone → stop.

    Same fill test as the now-eyes. One frame of tile speckle is not
    enough — hold first. Missing cameras do not arm and do not abort.
    """

    def __init__(self, hold_s: float = WATCH_HOLD_S) -> None:
        self.hold_s = float(hold_s)
        self.bad_s = 0.0
        self.seen_z = False
        self.seen_w = False
        self.reason: str | None = None

    def reset(self) -> None:
        self.bad_s = 0.0
        self.seen_z = False
        self.seen_w = False
        self.reason = None

    def step(
        self,
        z_fill: float | None,
        w_fill: float | None,
        dt: float,
    ) -> dict:
        dt = max(0.0, float(dt))
        z_ok = z_fill is not None and float(z_fill) >= MIN_STRIPE_FILL
        w_ok = w_fill is not None and float(w_fill) >= MIN_STRIPE_FILL
        if z_ok:
            self.seen_z = True
        if w_ok:
            self.seen_w = True
        armed = self.seen_z or self.seen_w
        both_measured = z_fill is not None and w_fill is not None
        gone = armed and both_measured and (not z_ok) and (not w_ok)
        if gone:
            self.bad_s += dt
            self.reason = "paint gone ahead"
        else:
            self.bad_s = max(0.0, self.bad_s - 2.0 * dt)
            if self.bad_s <= 1e-6:
                self.reason = None
        abort = gone and self.bad_s >= self.hold_s
        return {
            "abort": abort,
            "reason": self.reason if abort else None,
            "bad_s": round(self.bad_s, 3),
            "armed": armed,
        }


class CorridorWatch:
    """Slow intellect: snapshot vs what geometry said should happen.

    Control still ticks every ~8 ms. This only trips after HOLD seconds
    of disagreement — not a one-frame twitch.
    """

    def __init__(self, hold_s: float = WATCH_HOLD_S) -> None:
        self.hold_s = float(hold_s)
        self.bad_s = 0.0
        self.reason: str | None = None
        self._push_side: str | None = None
        self._push_y0: float | None = None
        self._push_s = 0.0

    def reset(self) -> None:
        self.bad_s = 0.0
        self.reason = None
        self._push_side = None
        self._push_y0 = None
        self._push_s = 0.0

    def step(
        self,
        left_y_m: float | None,
        right_y_m: float | None,
        steer: float | None,
        dt: float,
    ) -> dict:
        dt = max(0.0, float(dt))
        why = self._why(left_y_m, right_y_m, steer, dt)
        if why:
            self.bad_s += dt
            self.reason = why
        else:
            self.bad_s = max(0.0, self.bad_s - 2.0 * dt)
            if self.bad_s <= 1e-6:
                self.reason = None
        abort = self.bad_s >= self.hold_s
        return {
            "abort": abort,
            "reason": self.reason if abort else None,
            "bad_s": round(self.bad_s, 3),
            "watching": why,
        }

    def _why(
        self,
        left_y_m: float | None,
        right_y_m: float | None,
        steer: float | None,
        dt: float,
    ) -> str | None:
        if left_y_m is not None and right_y_m is not None:
            width = abs(float(left_y_m) - float(right_y_m))
            if width < LANE_WIDTH_MIN_M or width > LANE_WIDTH_MAX_M:
                return "lane width implausible"
        s = 0.0 if steer is None else float(steer)
        if s > 0.20 and left_y_m is not None:
            if self._push_side != "left":
                self._push_side = "left"
                self._push_y0 = float(left_y_m)
                self._push_s = 0.0
            self._push_s += dt
            if (
                self._push_s >= 0.25
                and self._push_y0 is not None
                and float(left_y_m) < self._push_y0 - 0.08
            ):
                return "push off left but wall got closer"
        elif s < -0.20 and right_y_m is not None:
            if self._push_side != "right":
                self._push_side = "right"
                self._push_y0 = float(right_y_m)
                self._push_s = 0.0
            self._push_s += dt
            if (
                self._push_s >= 0.25
                and self._push_y0 is not None
                and float(right_y_m) > self._push_y0 + 0.08
            ):
                return "push off right but wall got closer"
        else:
            self._push_side = None
            self._push_y0 = None
            self._push_s = 0.0
        return None


def rotation_matrix_to_sf(columns_xyz: tuple) -> tuple[float, float, float, float]:
    """Orthonormal columns (X, Y, Z) in parent frame → Webots SFRotation."""
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
        # 180°: pick the largest diagonal
        if r00 >= r11 and r00 >= r22:
            axis = _norm((1.0 + r00, r10, r20)) or (1.0, 0.0, 0.0)
        elif r11 >= r22:
            axis = _norm((r01, 1.0 + r11, r21)) or (0.0, 1.0, 0.0)
        else:
            axis = _norm((r02, r12, 1.0 + r22)) or (0.0, 0.0, 1.0)
        return (axis[0], axis[1], axis[2], math.pi)
    return (axis[0], axis[1], axis[2], angle)


def apply_sf_rotation(
    rot: tuple[float, float, float, float],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rodrigues: rotate ``vec`` by Webots SFRotation."""
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
        up_n = (0.0, 1.0, 0.0)
        x = _norm(_cross(up_n, z))
    if x is None:
        return None
    y = _cross(z, x)
    return (x, y, z)


def camera_fov_pyramid(
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    fov_rad: float,
    width: int,
    height: int,
    dist_m: float,
) -> list[tuple[float, float, float]]:
    """Apex + 4 far corners in parent frame. Webots FOV is vertical.

    Overlay wire so Daniel can see orientation and field of view.
    """
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


def look_at_sf_rotation(
    cam_pos: tuple[float, float, float],
    target_pos: tuple[float, float, float],
    up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> tuple[float, float, float, float]:
    """SFRotation so a Webots Camera (−Z look, +Y image-up) faces ``target_pos``.

    Do not hand-tune axis-angle. The aim puck is the spec.
    """
    z_axis = _sub(cam_pos, target_pos)  # camera +Z is opposite the look
    basis = _basis_from_z(z_axis, up)
    if basis is None:
        return (0.0, 1.0, 0.0, 0.0)
    return rotation_matrix_to_sf(basis)


# Do not roll the Camera node — that made Webots look at the feet
# while the FOV wire stayed on the walls. Rotate the *buffer* instead.
FORECAST_IMAGE_ROLL_RAD = 0.0


def rotate_bgra_90_cw(
    image: bytes | bytearray, width: int, height: int
) -> tuple[bytearray, int, int]:
    """90° clockwise. Sky-on-the-left becomes sky-on-top. Size swaps if not square."""
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


def roll_camera_sf(
    rot: tuple[float, float, float, float],
    angle_rad: float,
) -> tuple[float, float, float, float]:
    """Roll around the camera's own +Z. Look (−Z) does not move."""
    local = (0.0, 0.0, 1.0, float(angle_rad))
    x = apply_sf_rotation(rot, apply_sf_rotation(local, (1.0, 0.0, 0.0)))
    y = apply_sf_rotation(rot, apply_sf_rotation(local, (0.0, 1.0, 0.0)))
    z = apply_sf_rotation(rot, apply_sf_rotation(local, (0.0, 0.0, 1.0)))
    return rotation_matrix_to_sf((x, y, z))


def beam_sf_rotation(
    from_pos: tuple[float, float, float],
    to_pos: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """SFRotation so an ENU Cylinder (+Z height) stretches from→to."""
    z_axis = _sub(to_pos, from_pos)
    basis = _basis_from_z(z_axis, (0.0, 1.0, 0.0))
    if basis is None:
        return (0.0, 1.0, 0.0, 0.0)
    return rotation_matrix_to_sf(basis)


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
    """SKY / GROUND interlock from a mean RGB. Paint scores are optional extras."""
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
    return "unknown"
