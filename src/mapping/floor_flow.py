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
    if float(a.std()) < 4.0 or float(b.std()) < 4.0:
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
