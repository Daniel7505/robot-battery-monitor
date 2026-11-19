"""Landmark-voting loop closure.

A place is recognized when several landmarks from an older keyframe
line up with the ones in view, and one extra object is allowed to have
moved. The implied pose has to be consistent with how far the robot has
travelled since that keyframe: a huge jump is a wrong room, and a wrong
match is worse than none. An accepted correction is spread back along
the keyframe chain, more on the recent poses than on the old one, and
the grid is replayed from the stored scans.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.mapping.depth_cloud import to_odom
from src.mapping.odometry import wrap_rad


@dataclass
class _Frame:
    x: float
    y: float
    yaw: float
    path_m: float
    landmarks: np.ndarray
    hits: np.ndarray
    floors: np.ndarray


class LoopCloser:
    def __init__(self, *, min_path_m: float = 3.0, keyframe_m: float = 0.45) -> None:
        self.min_path_m = float(min_path_m)
        self.keyframe_m = float(keyframe_m)
        self.frames: list[_Frame] = []
        self.closures = 0
        self._sensor_xy = np.zeros(2, dtype=np.float64)
        self._pending: tuple[int, tuple[float, float, float]] | None = None
        self._closed_at_path = -10.0

    def consider(
        self,
        pose: tuple[float, float, float],
        path_m: float,
        landmarks: np.ndarray | None,
        hits_body: np.ndarray,
        floors_body: np.ndarray,
        sensor_xy: np.ndarray,
    ) -> tuple[float, float, float] | None:
        """Maybe store a keyframe. Returns the corrected pose when a loop closes."""
        self._sensor_xy = np.asarray(sensor_xy, dtype=np.float64).reshape(2)
        marks = _clean(landmarks)
        self._maybe_keyframe(pose, path_m, marks, hits_body, floors_body)
        if marks.shape[0] < 4 or len(self.frames) < 3:
            return None
        best_i = -1
        best_votes = 0
        second = 0
        best_fix: tuple[float, float, float] | None = None
        x, y, yaw = pose
        for index, frame in enumerate(self.frames[:-2]):
            travelled = float(path_m) - frame.path_m
            if travelled < self.min_path_m:
                continue
            match = _vote(frame.landmarks, marks)
            if match is None:
                continue
            votes, dx_body, dy_body, dyaw = match
            pred_x, pred_y, pred_yaw = _compose(frame.x, frame.y, frame.yaw, dx_body, dy_body, dyaw)
            corr_x = pred_x - x
            corr_y = pred_y - y
            corr_w = wrap_rad(pred_yaw - yaw)
            corr = math.hypot(corr_x, corr_y)
            if corr < 0.08 and abs(corr_w) < 0.04:
                continue
            # Drift grows with distance travelled, but a teleport is a wrong room.
            if corr > min(0.70, 0.12 * travelled + 0.12):
                continue
            if abs(corr_w) > min(0.25, 0.03 * travelled + 0.08):
                continue
            if votes > best_votes:
                second = best_votes
                best_votes = votes
                best_i = index
                best_fix = (corr_x, corr_y, corr_w)
            elif votes > second:
                second = votes
        if best_fix is None or best_i < 0:
            self._pending = None
            return None
        if second >= best_votes:
            self._pending = None
            return None
        # One agreeing look is not enough. A repeated doorway voted for the
        # wrong room about thirty times and wiped the occupancy grid.
        if float(path_m) - self._closed_at_path < 2.5:
            return None
        if not self._confirms(best_i, best_fix):
            return None
        self._closed_at_path = float(path_m)
        self._pending = None
        end = self.frames[-1]
        if math.hypot(end.x - x, end.y - y) > 0.02 or abs(wrap_rad(end.yaw - yaw)) > 0.02:
            self.frames.append(
                _Frame(x, y, yaw, float(path_m), marks, _sample(hits_body), _sample(floors_body))
            )
        self._spread(best_i, best_fix)
        self.closures += 1
        last = self.frames[-1]
        return (last.x, last.y, last.yaw)

    def rebuild(self, grid) -> None:
        grid.reset()
        sensor = np.array([self._sensor_xy[0], self._sensor_xy[1], 0.0], dtype=np.float64)
        for frame in self.frames:
            pose = (frame.x, frame.y, frame.yaw)
            origin = to_odom(sensor.reshape(1, 3), pose)[0]
            hits = _as_xy(frame.hits, pose)
            floors = _as_xy(frame.floors, pose)
            grid.integrate(origin, hits, floors)
            grid.stamp_footprint(frame.x, frame.y)

    def _confirms(self, index: int, fix: tuple[float, float, float]) -> bool:
        prev = self._pending
        self._pending = (index, fix)
        if prev is None or prev[0] != index:
            return False
        old = prev[1]
        if math.hypot(fix[0] - old[0], fix[1] - old[1]) > 0.06:
            return False
        if abs(wrap_rad(fix[2] - old[2])) > 0.04:
            return False
        return True

    def _maybe_keyframe(self, pose, path_m, marks, hits_body, floors_body) -> None:
        x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
        if self.frames:
            last = self.frames[-1]
            moved = math.hypot(x - last.x, y - last.y)
            turned = abs(wrap_rad(yaw - last.yaw))
            if moved < self.keyframe_m and turned < 0.35:
                return
        if marks.shape[0] < 3 and hits_body is not None and len(hits_body) == 0:
            return
        self.frames.append(
            _Frame(
                x,
                y,
                yaw,
                float(path_m),
                marks,
                _sample(hits_body),
                _sample(floors_body),
            )
        )
        if len(self.frames) > 140:
            del self.frames[0]

    def _spread(self, index: int, fix: tuple[float, float, float]) -> None:
        """Linear share of the correction. The recognized place stays put."""
        dx, dy, dyaw = fix
        end = len(self.frames) - 1
        span = max(1, end - index)
        for k in range(index, end + 1):
            alpha = (k - index) / span
            frame = self.frames[k]
            frame.x += alpha * dx
            frame.y += alpha * dy
            frame.yaw = wrap_rad(frame.yaw + alpha * dyaw)


def _clean(points: np.ndarray | None) -> np.ndarray:
    if points is None:
        return np.zeros((0, 2), dtype=np.float64)
    arr = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if arr.size == 0:
        return arr
    keep = np.isfinite(arr).all(axis=1)
    return arr[keep][:12]


def _sample(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=np.float64).reshape(-1, 2) if np.size(points) else np.zeros((0, 2))
    if arr.shape[0] > 140:
        step = int(math.ceil(arr.shape[0] / 140))
        arr = arr[::step][:140]
    return np.ascontiguousarray(arr, dtype=np.float32)


def _as_xy(points: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    if points.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    body = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    return to_odom(body, pose)


def _compose(x: float, y: float, yaw: float, dx: float, dy: float, dyaw: float) -> tuple[float, float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, wrap_rad(yaw + dyaw)


def _signature_err(left: np.ndarray, right: np.ndarray) -> float | None:
    """Largest distance error after ignoring one pair.

    One moved object changes a single distance from an otherwise stable
    landmark. The rest still have to agree.
    """
    if left.size < 3 or right.size < 3:
        return None
    used: set[int] = set()
    errs: list[float] = []
    for value in left:
        best = None
        best_i = -1
        for j, other in enumerate(right):
            if j in used:
                continue
            err = abs(float(value) - float(other))
            if best is None or err < best:
                best = err
                best_i = j
        if best is None or best_i < 0:
            return None
        used.add(best_i)
        errs.append(float(best))
    if len(errs) >= 4:
        errs = sorted(errs)[:-1]
    if not errs:
        return None
    return max(errs)


def _signature(points: np.ndarray, index: int) -> np.ndarray:
    delta = points - points[index]
    dist = np.hypot(delta[:, 0], delta[:, 1])
    dist = np.sort(dist[dist > 0.08])
    return dist[:4]


def _vote(old: np.ndarray, new: np.ndarray) -> tuple[int, float, float, float] | None:
    """How many landmarks agree, and the robot motion in the old body frame."""
    if old.shape[0] < 4 or new.shape[0] < 4:
        return None
    pairs: list[tuple[int, int]] = []
    used: set[int] = set()
    for i in range(new.shape[0]):
        sig_n = _signature(new, i)
        if sig_n.size < 2:
            continue
        best_j = -1
        best_err = 0.12
        for j in range(old.shape[0]):
            if j in used:
                continue
            sig_o = _signature(old, j)
            err = _signature_err(sig_n, sig_o)
            if err is not None and err < best_err:
                best_err = err
                best_j = j
        if best_j >= 0:
            used.add(best_j)
            pairs.append((best_j, i))
    if len(pairs) < 3:
        return None
    # One object may have moved. Two unexplained landmarks is a different place.
    unmatched = max(old.shape[0], new.shape[0]) - len(pairs)
    if unmatched > 1:
        return None
    src = old[np.array([a for a, _b in pairs])]
    dst = new[np.array([b for _a, b in pairs])]
    fit = _rigid_drop_one(src, dst)
    if fit is None:
        return None
    dx, dy, dyaw = fit
    return len(pairs), dx, dy, dyaw


def _rigid_drop_one(src: np.ndarray, dst: np.ndarray) -> tuple[float, float, float] | None:
    from src.mapping.visual_odometry import rigid_motion

    motion = rigid_motion(src, dst)
    if motion is None:
        return None
    err = _pair_err(src, dst, motion)
    if float(np.max(err)) > 0.16 and src.shape[0] >= 4:
        keep = np.ones(src.shape[0], dtype=bool)
        keep[int(np.argmax(err))] = False
        if int(keep.sum()) < 3:
            return None
        motion = rigid_motion(src[keep], dst[keep])
        if motion is None:
            return None
        err = _pair_err(src[keep], dst[keep], motion)
    if float(np.median(err)) > 0.10:
        return None
    return motion


def _pair_err(src: np.ndarray, dst: np.ndarray, motion: tuple[float, float, float]) -> np.ndarray:
    dx, dy, dyaw = motion
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    shifted = src - np.array([dx, dy])
    landed = np.column_stack((c * shifted[:, 0] - s * shifted[:, 1], s * shifted[:, 0] + c * shifted[:, 1]))
    return np.hypot(landed[:, 0] - dst[:, 0], landed[:, 1] - dst[:, 1])
