"""Track corners between depth frames and read the robot's motion from them.

Door frames and other depth corners are the features. A short patch is
followed into the next frame, the two pixels are back-projected, and a
2D rigid fit (Kabsch) is the motion. One bad track is dropped. If the
scene did not move and the wheels say it did, the caller treats that as
slip. A fit that disagrees with the wheels without being a full stall
is ignored: a wrong visual step is worse than none.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.mapping.body import DepthCamera
from src.mapping.depth_cloud import axis_angle_matrix


@dataclass(frozen=True)
class VisualSample:
    dx_m: float
    dy_m: float
    dyaw_rad: float
    ok: bool
    quality: float
    landmarks: np.ndarray
    reason: str = ""
    pixel_shift: float = 0.0
    range_m: float = 1.0


def _normalize(gray: np.ndarray, valid: np.ndarray) -> np.ndarray:
    vals = gray[valid] if valid.shape == gray.shape else gray.ravel()
    if vals.size < 8:
        return np.zeros_like(gray)
    lo, hi = np.percentile(vals, [5, 95])
    span = float(hi - lo)
    if span < 1e-6:
        return np.zeros_like(gray)
    out = (gray - float(lo)) / span
    return np.clip(out, 0.0, 1.0)


def _gray_from_depth(depth: np.ndarray, camera: DepthCamera) -> tuple[np.ndarray, np.ndarray]:
    image = np.asarray(depth, dtype=np.float64)
    valid = np.isfinite(image) & (image >= camera.min_range_m) & (image <= camera.max_range_m)
    gray = np.zeros(image.shape, dtype=np.float64)
    gray[valid] = image[valid]
    return gray, valid


def _corners(gray: np.ndarray, valid: np.ndarray, limit: int) -> list[tuple[int, int]]:
    gx = np.zeros_like(gray)
    gy = np.zeros_like(gray)
    gx[:, 1:-1] = gray[:, 2:] - gray[:, :-2]
    gy[1:-1, :] = gray[2:, :] - gray[:-2, :]
    ix2, iy2, ixy = gx * gx, gy * gy, gx * gy

    def box(src: np.ndarray) -> np.ndarray:
        pad = np.pad(src, 1)
        return (
            pad[:-2, :-2] + pad[:-2, 1:-1] + pad[:-2, 2:]
            + pad[1:-1, :-2] + pad[1:-1, 1:-1] + pad[1:-1, 2:]
            + pad[2:, :-2] + pad[2:, 1:-1] + pad[2:, 2:]
        )

    sxx, syy, sxy = box(ix2), box(iy2), box(ixy)
    score = (sxx * syy - sxy * sxy) - 0.04 * (sxx + syy) ** 2
    score[:5, :] = 0.0
    score[-5:, :] = 0.0
    score[:, :5] = 0.0
    score[:, -5:] = 0.0
    score[~valid] = 0.0
    flat = score.ravel()
    if flat.size == 0 or float(flat.max()) <= 0.0:
        return []
    take = min(int(limit), int(np.count_nonzero(flat > 0.0)))
    if take <= 0:
        return []
    picked = np.argpartition(flat, -take)[-take:]
    order = picked[np.argsort(flat[picked])[::-1]]
    found: list[tuple[int, int]] = []
    used: list[tuple[int, int]] = []
    width = score.shape[1]
    for index in order:
        if flat[index] <= 0.0:
            continue
        row, col = divmod(int(index), width)
        if any((row - ur) ** 2 + (col - uc) ** 2 < 36 for ur, uc in used):
            continue
        used.append((row, col))
        found.append((row, col))
        if len(found) >= limit:
            break
    return found


def _body_xy(camera: DepthCamera, col: float, row: float, depth_m: float) -> np.ndarray | None:
    if not math.isfinite(depth_m) or depth_m < camera.min_range_m or depth_m > camera.max_range_m:
        return None
    direction = np.array(
        [camera.focal_px, camera.cx - float(col), camera.cy - float(row)],
        dtype=np.float64,
    )
    point = direction * (float(depth_m) / camera.focal_px)
    rotated = axis_angle_matrix(camera.rotation)
    body = rotated @ point + np.asarray(camera.translation, dtype=np.float64)
    return body[:2]


def _patch(image: np.ndarray, row: int, col: int, radius: int = 2) -> np.ndarray | None:
    if row - radius < 0 or col - radius < 0:
        return None
    if row + radius >= image.shape[0] or col + radius >= image.shape[1]:
        return None
    chip = image[row - radius : row + radius + 1, col - radius : col + radius + 1]
    if not np.isfinite(chip).all():
        return None
    return chip


def _cost(chip: np.ndarray, image: np.ndarray, row: int, col: int) -> float | None:
    other = _patch(image, row, col)
    if other is None:
        return None
    return float(np.mean(np.abs(chip - other)))


def _track(prev: np.ndarray, curr: np.ndarray, row: int, col: int, window: int) -> tuple[int, int] | None:
    """Coarse-to-fine patch search. A full window in Python is too slow at 10 Hz."""
    chip = _patch(prev, row, col)
    if chip is None:
        return None
    step = max(2, int(window) // 3)
    best = (row, col)
    best_cost = _cost(chip, curr, row, col)
    if best_cost is None:
        best_cost = 1e18
    for dr in range(-window, window + 1, step):
        for dc in range(-window, window + 1, step):
            cost = _cost(chip, curr, row + dr, col + dc)
            if cost is not None and cost < best_cost:
                best_cost = cost
                best = (row + dr, col + dc)
    br, bc = best
    for dr in range(br - step, br + step + 1):
        for dc in range(bc - step, bc + step + 1):
            cost = _cost(chip, curr, dr, dc)
            if cost is not None and cost < best_cost:
                best_cost = cost
                best = (dr, dc)
    if best_cost > 0.08:
        return None
    return best


def _images_still(prev: np.ndarray, curr: np.ndarray) -> bool:
    """True when the forward view did not shift by more than a couple of pixels.

    A blank frame is not stillness. There is nothing in it to say the
    body stayed put.
    """
    if prev.shape != curr.shape or prev.size < 30:
        return False
    height, width = prev.shape
    r0, r1 = int(0.25 * height), int(0.75 * height)
    c0, c1 = int(0.20 * width), int(0.80 * width)
    before = prev[r0:r1, c0:c1]
    after = curr[r0:r1, c0:c1]
    if before.size < 30:
        return False
    if float(before.std()) < 0.02 and float(after.std()) < 0.02:
        return False
    best = float(np.mean(np.abs(before - after)))
    for shift in (1, 2):
        best = min(best, float(np.mean(np.abs(before[:, :-shift] - after[:, shift:]))))
        best = min(best, float(np.mean(np.abs(before[:, shift:] - after[:, :-shift]))))
    return best < 0.035


def _still_sample(depth: np.ndarray, camera: DepthCamera) -> VisualSample:
    valid = np.isfinite(depth) & (depth >= camera.min_range_m) & (depth <= camera.max_range_m)
    range_m = float(np.median(depth[valid])) if np.any(valid) else 1.0
    return VisualSample(0.0, 0.0, 0.0, True, 0.8, np.zeros((0, 2), dtype=np.float64), "still", 0.0, range_m)


def rigid_motion(src: np.ndarray, dst: np.ndarray) -> tuple[float, float, float] | None:
    """Robot motion in the previous body frame that takes ``src`` points to ``dst``."""
    if src.shape[0] < 3:
        return None
    center_s = src.mean(axis=0)
    center_d = dst.mean(axis=0)
    matrix = (src - center_s).T @ (dst - center_d)
    _u, _s, vt = np.linalg.svd(matrix)
    rot = vt.T @ _u.T
    if np.linalg.det(rot) < 0.0:
        vt = vt.copy()
        vt[-1, :] *= -1.0
        rot = vt.T @ _u.T
    trans = center_d - rot @ center_s
    # p_curr = R(-dyaw) p_prev + t_pc, and t_body = -R(dyaw) t_pc.
    dyaw = -math.atan2(float(rot[1, 0]), float(rot[0, 0]))
    body = -(rot.T @ trans)
    return float(body[0]), float(body[1]), float(dyaw)


class VisualOdometry:
    def __init__(self, *, limit: int = 16, window_px: int = 14) -> None:
        self.limit = int(limit)
        self.window_px = int(window_px)
        self._prev_gray: np.ndarray | None = None
        self._prev_depth: np.ndarray | None = None
        self._prev_px: list[tuple[int, int]] = []

    def measure(self, depth, camera: DepthCamera, rgb=None) -> VisualSample | None:
        if depth is None:
            return None
        image = np.asarray(depth, dtype=np.float64)
        if image.ndim != 2:
            return None
        gray, valid = _gray_from_depth(image, camera)
        if rgb is not None:
            color = np.asarray(rgb, dtype=np.float64)
            if color.shape[:2] == gray.shape:
                shown = color.mean(axis=2) if color.ndim == 3 else color
                gray = np.where(valid, shown, 0.0)
        gray = _normalize(gray, valid)
        corners = _corners(gray, valid, self.limit)
        prev_gray, prev_depth, prev_px = self._prev_gray, self._prev_depth, self._prev_px
        self._prev_gray = gray
        self._prev_depth = image
        self._prev_px = corners
        if prev_gray is None or prev_depth is None:
            return None
        # A flat wall has no corners to track. An unchanged picture is
        # still a reading: the body did not translate. A picture that
        # moved, with nothing to fit, stays "no sample".
        if len(prev_px) < 4:
            if _images_still(prev_gray, gray):
                return _still_sample(image, camera)
            return None
        src: list[np.ndarray] = []
        dst: list[np.ndarray] = []
        shifts: list[float] = []
        ranges: list[float] = []
        for row, col in prev_px:
            found = _track(prev_gray, gray, row, col, self.window_px)
            if found is None:
                continue
            old = _body_xy(camera, col, row, float(prev_depth[row, col]))
            new = _body_xy(camera, found[1], found[0], float(image[found[0], found[1]]))
            if old is None or new is None:
                continue
            src.append(old)
            dst.append(new)
            shifts.append(math.hypot(found[0] - row, found[1] - col))
            ranges.append(float(image[found[0], found[1]]))
        empty = np.zeros((0, 2), dtype=np.float64)
        if len(src) < 4:
            if _images_still(prev_gray, gray):
                return _still_sample(image, camera)
            return VisualSample(0.0, 0.0, 0.0, False, 0.0, empty, "few_tracks")
        src_a = np.vstack(src)
        dst_a = np.vstack(dst)
        fit = _fit_dropping_one(src_a, dst_a)
        if fit is None:
            return VisualSample(0.0, 0.0, 0.0, False, 0.0, dst_a, "no_fit")
        dx, dy, dyaw, residual, kept = fit
        if abs(dx) > 0.35 or abs(dy) > 0.35 or abs(dyaw) > 0.4:
            return VisualSample(0.0, 0.0, 0.0, False, 0.0, kept, "too_large")
        quality = max(0.0, 1.0 - residual / 0.08)
        return VisualSample(
            dx,
            dy,
            dyaw,
            True,
            quality,
            kept,
            "",
            float(np.median(shifts)) if shifts else 0.0,
            float(np.median(ranges)) if ranges else 1.0,
        )


def _fit_dropping_one(src: np.ndarray, dst: np.ndarray):
    motion = rigid_motion(src, dst)
    if motion is None:
        return None
    residual, pred = _residual(src, dst, motion)
    if residual > 0.06 and src.shape[0] >= 5:
        worst = int(np.argmax(pred))
        keep = np.ones(src.shape[0], dtype=bool)
        keep[worst] = False
        motion = rigid_motion(src[keep], dst[keep])
        if motion is None:
            return None
        residual, _pred = _residual(src[keep], dst[keep], motion)
        kept = dst[keep]
    else:
        kept = dst
    if residual > 0.07:
        return None
    dx, dy, dyaw = motion
    return dx, dy, dyaw, residual, kept


def _residual(src: np.ndarray, dst: np.ndarray, motion: tuple[float, float, float]) -> tuple[float, np.ndarray]:
    dx, dy, dyaw = motion
    # Inverse of the robot motion: where the previous points land now.
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    # p_curr = R(-dyaw) @ (p_prev - t_body)
    shifted = src - np.array([dx, dy])
    landed = np.column_stack((c * shifted[:, 0] - s * shifted[:, 1], s * shifted[:, 0] + c * shifted[:, 1]))
    err = np.hypot(landed[:, 0] - dst[:, 0], landed[:, 1] - dst[:, 1])
    return float(np.median(err)), err
