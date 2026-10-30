"""Downward optical-flow floor sensor, in the spirit of a PMW3901 or a mouse.

A small camera looks at the floor. Motion is the shift of that image
between frames (phase correlation: Kuglin and Hines, 1975), scaled by
the nominal mounting height over the focal length. Nothing here reads
the simulator pose.

Weaknesses, on purpose:

* pixel noise on the shift
* a scale error, because the metres-per-pixel factor is the nominal
  height. Carpet is thicker than laminate, and pitch changes the
  height, and neither is compensated. When the wheels and the flow
  agree, a slow scale update can learn the current height. It does
  not learn during slip, or the slip would be absorbed into the scale.
* low texture. A flat image has no peak and the sample is rejected.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.mapping.body import FLOOR_CAM, FLOOR_NOISE_PX, FLOOR_SCALE_ERROR


@dataclass(frozen=True)
class FloorSample:
    dx_m: float
    dy_m: float
    ok: bool
    quality: float
    reason: str = ""


def _shift_peak(prev: np.ndarray, curr: np.ndarray) -> tuple[float, float, float] | None:
    """Pixel shift of ``curr`` relative to ``prev``, and the peak height.

    A positive row shift means the texture moved toward larger rows
    (down the image). A positive column shift means it moved toward
    larger columns (image right).
    """
    a = np.asarray(prev, dtype=np.float64)
    b = np.asarray(curr, dtype=np.float64)
    if a.shape != b.shape or a.ndim != 2 or a.size < 16:
        return None
    a = a - float(a.mean())
    b = b - float(b.mean())
    # Laminate in this camera is about std 5–8. The carpet pile, 4 cm
    # up, lands around 3. The vertical face of the lip is a flat frame,
    # std about 0.5, and that one stays rejected: there is nothing to
    # track. A cutoff of 4 dropped every carpet frame, so a still pile
    # never became a stuck reading.
    if float(a.std()) < 2.0 or float(b.std()) < 2.0:
        return None
    cross = np.fft.rfft2(a) * np.conj(np.fft.rfft2(b))
    mag = np.abs(cross)
    usable = mag > 1e-8
    cross = np.divide(cross, mag, out=np.zeros_like(cross), where=usable)
    corr = np.fft.irfft2(cross, s=a.shape)
    row, col = np.unravel_index(int(np.argmax(corr)), corr.shape)
    quality = float(corr[row, col])
    if quality < 0.12:
        return None
    height, width = corr.shape
    if row > height // 2:
        row -= height
    if col > width // 2:
        col -= width
    row_f = float(row) + _parab(corr, int(row) % height, int(col) % width, axis=0)
    col_f = float(col) + _parab(corr, int(row) % height, int(col) % width, axis=1)
    return row_f, col_f, quality


def _parab(corr: np.ndarray, row: int, col: int, *, axis: int) -> float:
    height, width = corr.shape
    if axis == 0:
        vm = float(corr[(row - 1) % height, col])
        v0 = float(corr[row, col])
        vp = float(corr[(row + 1) % height, col])
    else:
        vm = float(corr[row, (col - 1) % width])
        v0 = float(corr[row, col])
        vp = float(corr[row, (col + 1) % width])
    den = vm - 2.0 * v0 + vp
    if abs(den) < 1e-9:
        return 0.0
    return max(-0.6, min(0.6, 0.5 * (vm - vp) / den))


def image_shift_px(prev, curr) -> tuple[float, float, float] | None:
    """Pixel shift of ``curr`` relative to ``prev``.

    None means the floor has nothing to track. The row and column are the
    same peak ``FloorFlow`` integrates; callers that only need "did it
    move" use the magnitude. A creep of door speed is many pixels across
    a fifth of a second, and a body that is sitting still stays under a
    couple of pixels.

    The peak is cyclic with the image width. On this 32 px camera one
    period is about 3.9 cm of floor, and a shift of one period comes back
    as a high-quality zero. Callers that decide "stuck" must not compare
    across more than half of that.
    """
    before = np.asarray(prev, dtype=np.float64)
    after = np.asarray(curr, dtype=np.float64)
    if before.ndim == 3:
        before = before.mean(axis=2)
    if after.ndim == 3:
        after = after.mean(axis=2)
    return _shift_peak(before, after)


def _as_gray(image) -> np.ndarray | None:
    if image is None:
        return None
    gray = np.asarray(image, dtype=np.float64)
    if gray.ndim == 3:
        gray = gray.mean(axis=2)
    if gray.ndim != 2 or gray.size < 16:
        return None
    return gray


# Half of the 32 px window is about 1.9 cm of floor. The old stall compared
# a 2 cm wheel window in one shot, which lands on that cliff, and a full
# period correlates as (0, 0). Each step below closes around 1.2 cm
# (about 10 px), where a real slide is still the peak.
_STEP_WHEEL_M = 0.012
_STEP_MAX_S = 0.12
_STILL_PX = 2.0
# Identical frames correlate near 1. A peak that merely fell near zero
# (a shadow fixed in the camera, or a shift past half a period) is well
# under this. Only the sharp match is "the floor did not move".
_STILL_QUALITY = 0.70
_AMBIGUOUS_PX = 12.0


class ShortFloorTrack:
    """Sum of phase-correlation steps that stay inside one image period.

    A still floor is a high-quality match after the wheels have claimed
    about 2 cm. A moving floor shows up in a step of about 1.2 cm, before
    the correlation can wrap around to zero. A flat lip face is not a
    zero: there is nothing to track, so it is not evidence of a stall.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._base: np.ndarray | None = None
        self._wheel = 0.0
        self._age = 0.0
        self.sum_row = 0.0
        self.sum_col = 0.0
        self.sum_wheel_m = 0.0
        self.still_samples = 0
        self.motion_samples = 0
        self.blank_s = 0.0

    @property
    def moved_px(self) -> float:
        return math.hypot(self.sum_row, self.sum_col)

    def confident_still(self) -> bool:
        """Wheels claimed 2 cm and every closed step matched the floor."""
        return (
            self.motion_samples == 0
            and self.still_samples >= 1
            and self.sum_wheel_m >= 0.02
            and self.moved_px <= _STILL_PX
        )

    def push(self, image, dt: float, wheel_m: float) -> str:
        """Fold one frame in. Returns pending, blank, moving, or still.

        ``dt <= 0`` closes on this call once the hubs have claimed any
        travel. The unit tests pass one sample per call that way. The
        simulator steps at 8 ms and passes ``dt``, so a step waits until
        about 1.2 cm of wheel travel or 0.12 s, whichever comes first.
        """
        gray = _as_gray(image)
        if gray is None:
            return "blank"
        if float(gray.std()) < 2.0:
            self.blank_s += max(0.0, float(dt))
            self._base = None
            self._wheel = 0.0
            self._age = 0.0
            return "blank"
        self.blank_s = 0.0
        if self._base is None:
            self._base = np.array(gray, copy=True)
            self._wheel = 0.0
            self._age = 0.0
            return "pending"
        step_wheel = abs(float(wheel_m))
        step_dt = max(0.0, float(dt))
        self._wheel += step_wheel
        self._age += step_dt
        # A caller that does not pass time is handing over one finished sample.
        close = self._wheel >= _STEP_WHEEL_M or self._age >= _STEP_MAX_S
        if step_dt <= 0.0 and step_wheel > 0.0:
            close = True
        if not close:
            return "pending"
        shift = _shift_peak(self._base, gray)
        closed_wheel = self._wheel
        closed_age = self._age
        if shift is None:
            # Nothing to track in this pair. Drop it and start from this frame.
            self._base = np.array(gray, copy=True)
            self._wheel = 0.0
            self._age = 0.0
            return "blank"
        row, col, quality = shift
        mag = math.hypot(float(row), float(col))
        if mag > _AMBIGUOUS_PX:
            # Too close to one period to trust the sign. Not a still floor.
            self._base = np.array(gray, copy=True)
            self._wheel = 0.0
            self._age = 0.0
            return "blank"
        if mag > _STILL_PX:
            self.sum_row += float(row)
            self.sum_col += float(col)
            self.motion_samples += 1
            self._base = np.array(gray, copy=True)
            self._wheel = 0.0
            self._age = 0.0
            return "moving"
        # Closed because the clock ran out, not because the hubs claimed
        # enough ground for a real slide. A few millimetres is under two
        # pixels, so a match is not evidence the shell is stuck.
        if step_dt > 0.0 and closed_wheel < _STEP_WHEEL_M:
            self._base = np.array(gray, copy=True)
            self._wheel = 0.0
            self._age = 0.0
            return "pending"
        if quality < _STILL_QUALITY:
            self._base = np.array(gray, copy=True)
            self._wheel = 0.0
            self._age = 0.0
            return "blank"
        self.still_samples += 1
        self.sum_wheel_m += closed_wheel
        self._base = np.array(gray, copy=True)
        self._wheel = 0.0
        self._age = 0.0
        return "still"


def metres_per_px(camera=FLOOR_CAM, height_m: float | None = None) -> float:
    """Ground metres per pixel at the nominal mounting height."""
    height = float(FLOOR_CAM.translation[2] if height_m is None else height_m)
    focal = camera.focal_px
    if focal <= 1e-6:
        return 0.0
    return height / focal


class FloorFlow:
    def __init__(self, *, scale: float = FLOOR_SCALE_ERROR, noise_px: float = FLOOR_NOISE_PX, rng=None) -> None:
        self.scale = float(scale)
        self.noise_px = float(noise_px)
        # Noise is part of the sensor, not an optional test hook. Callers
        # that want a deterministic shift pass noise_px=0.
        self.rng = rng if rng is not None else np.random.default_rng()
        self._prev: np.ndarray | None = None
        self.samples = 0
        self.rejected = 0

    def reset(self) -> None:
        self._prev = None

    def measure(self, image) -> FloorSample | None:
        """Motion of the robot since the previous frame, in the body frame.

        Image up is body forward and image left is body left, because the
        camera looks down with its local +Z toward the nose. Texture moves
        opposite the robot, so a downward image shift is forward motion.
        The first frame only arms the previous image.
        """
        if image is None:
            return None
        gray = np.asarray(image, dtype=np.float64)
        if gray.ndim == 3:
            gray = gray.mean(axis=2)
        if self._prev is None or self._prev.shape != gray.shape:
            self._prev = gray
            return None
        peak = _shift_peak(self._prev, gray)
        self._prev = gray
        if peak is None:
            self.rejected += 1
            return FloorSample(0.0, 0.0, False, 0.0, "low_texture")
        row, col, quality = peak
        if self.noise_px > 0.0:
            row += float(self.rng.normal(0.0, self.noise_px))
            col += float(self.rng.normal(0.0, self.noise_px))
        metres = metres_per_px() * self.scale
        # Phase correlation's peak is the shift of the previous image onto
        # the new one, which is opposite the texture motion. Negating it
        # makes a texture shift toward larger rows (down the image, body
        # backward) a positive forward step.
        self.samples += 1
        return FloorSample(-row * metres, -col * metres, True, quality, "")

    def learn_scale(self, wheel_forward_m: float, measured_forward_m: float) -> None:
        """Nudge the scale when the wheels and the flow agree. Not during slip."""
        meas = float(measured_forward_m)
        wheel = float(wheel_forward_m)
        if abs(meas) < 0.008 or abs(wheel) < 0.008:
            return
        if meas * wheel <= 0.0:
            return
        if abs(meas - wheel) > 0.35 * abs(wheel):
            return
        target = self.scale * (wheel / meas)
        if not math.isfinite(target):
            return
        target = max(0.55, min(1.8, target))
        self.scale += 0.05 * (target - self.scale)
