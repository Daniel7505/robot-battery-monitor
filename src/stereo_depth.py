"""Forward stereo pair -> disparity -> depth -> bot-frame points -> "obstacle in my lane corridor".

No SLAM, no learning, no map: every frame stands alone (the gate in
``src.obstacle_gate`` adds the frame-to-frame memory). Pipeline:

1. **Rig** (:class:`StereoRig`): the two ``CameraModel`` s (live pose from the
   Supervisor, like the shoulder cams; Webots axes: a camera looks along its
   local +x, image right = local -y, image up = local +z). Baseline, focal
   length and the rectifying rotation are derived from the two poses.
   Webots cameras are ideal pinholes, so with equal orientations and a
   baseline along the camera's y axis the pair is already rectified and the
   remap is skipped; anything else is rectified with a homography per camera.
2. **Disparity**: OpenCV ``StereoSGBM`` when ``cv2`` imports, else a numpy
   block matcher (x-Sobel + intensity SAD, parabola sub-pixel, uniqueness,
   texture and left-right checks). Same output: float disparity, NaN = none.
3. **Depth / points**: Z = f B / d along the rectified optical axis, then
   into the bot frame (x forward from the axle, y left, z up from the floor).
4. **Ground filter**: the floor is z = 0 in the bot frame (the camera pose
   gives its height and pitch), so floor and tape are points with z below a
   tolerance that grows with range like the stereo height error
   (``ground_tol``). No floor colour or tape model is used.
5. **Corridor** (:func:`detect_obstacle`): points above the floor and below
   the robot's own height + margin, inside the corridor ahead (half width =
   robot half width + margin, bent along the commanded curvature), are
   binned along x; the nearest window with enough points is the obstacle.
   Its lateral extent, height band (low/pallet, waist/table, tall/rack) and
   a confidence go into an :class:`~src.obstacle_gate.ObstacleReport`.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from src.lane_vision import CameraModel
from src.obstacle_gate import ObstacleReport, ObstacleSource, height_band

try:  # optional: pip install opencv-python
    import cv2  # type: ignore

    HAVE_CV2 = hasattr(cv2, "StereoSGBM_create")
except Exception:  # pragma: no cover - depends on the machine
    cv2 = None
    HAVE_CV2 = False

# --- mount (fallback copy of the .wbt; the controller reads the live pose) ----
# Pair on a mast in front of the chest, lenses on the front face of STEREO_BAR.
# Pitch from stereo_pitch_for_top(): the TOP image row reaches TOP_VIEW_H_M
# (above a 0.9 m table / waist) at TOP_VIEW_RANGE_M ahead of the lenses.
STEREO_BASELINE_M = 0.10
STEREO_X_M = 0.14
STEREO_Z_M = 0.78
STEREO_W, STEREO_H, STEREO_FOV = 320, 240, 1.4
TOP_VIEW_H_M = 1.0
TOP_VIEW_RANGE_M = 1.0
ROBOT_FRONT_X_M = 0.15  # front of the casters (0.11 + 0.04 m sphere) = foremost body part at floor level


def vertical_fov(width: int, height: int, hfov: float) -> float:
    return 2.0 * math.atan(math.tan(hfov / 2.0) * height / width)


def stereo_pitch_for_top(z_cam: float, top_h: float = TOP_VIEW_H_M, at_range: float = TOP_VIEW_RANGE_M,
                         width: int = STEREO_W, height: int = STEREO_H, hfov: float = STEREO_FOV) -> float:
    """Down-pitch (rad) that puts the top image row at height ``top_h`` ``at_range`` m ahead."""
    return vertical_fov(width, height, hfov) / 2.0 - math.atan2(top_h - z_cam, at_range)


STEREO_PITCH_RAD = round(stereo_pitch_for_top(STEREO_Z_M), 4)
STEREO_LEFT_POSE = {"translation": (STEREO_X_M, STEREO_BASELINE_M / 2, STEREO_Z_M),
                    "rotation": (0.0, 1.0, 0.0, STEREO_PITCH_RAD), "width": STEREO_W, "height": STEREO_H,
                    "fov": STEREO_FOV}
STEREO_RIGHT_POSE = {**STEREO_LEFT_POSE, "translation": (STEREO_X_M, -STEREO_BASELINE_M / 2, STEREO_Z_M)}
STEREO_NAMES = ("stereo_left", "stereo_right")
STEREO_DEFS = {"stereo_left": "STEREO_CAM_L", "stereo_right": "STEREO_CAM_R"}


def _model(name: str, pose: dict) -> CameraModel:
    return CameraModel(name, pose["translation"], pose["rotation"], pose["width"], pose["height"], pose["fov"])


STEREO_LEFT_CAM = _model("stereo_left", STEREO_LEFT_POSE)
STEREO_RIGHT_CAM = _model("stereo_right", STEREO_RIGHT_POSE)


# ---------------------------------------------------------------------------
# rig + rectification
# ---------------------------------------------------------------------------


class StereoRig:
    """Left/right pinhole cameras in the bot frame, rectified to a common rotation."""

    def __init__(self, left: CameraModel, right: CameraModel) -> None:
        if (left.width, left.height) != (right.width, right.height):
            raise ValueError("stereo cameras must have the same resolution")
        self.left, self.right = left, right
        self.width, self.height = left.width, left.height
        tl, tr = np.array(left.translation), np.array(right.translation)
        b = tl - tr
        self.baseline_m = float(np.linalg.norm(b))
        if self.baseline_m < 1e-3:
            raise ValueError("stereo baseline is zero")
        RL, RR = np.array(left.R), np.array(right.R)
        fwd = RL[:, 0] + RR[:, 0]
        yax = b / self.baseline_m
        xax = fwd - yax * float(fwd @ yax)
        xax /= np.linalg.norm(xax)
        zax = np.cross(xax, yax)
        self.R_rect = np.stack([xax, yax, zax], axis=1)  # columns = rectified camera axes in the bot frame
        self.f = 0.5 * (left.focal_px + right.focal_px)
        self.cx, self.cy = left.cx, left.cy
        self.origin = tl  # rectified left camera centre
        self.needs_rect = {
            "left": not (np.allclose(RL, self.R_rect, atol=1e-6) and abs(left.focal_px - self.f) < 1e-6),
            "right": not (np.allclose(RR, self.R_rect, atol=1e-6) and abs(right.focal_px - self.f) < 1e-6),
        }
        self._maps: dict = {}

    @property
    def fB(self) -> float:
        return self.f * self.baseline_m

    @property
    def cam_z(self) -> float:
        return float(self.origin[2])

    def num_disparities(self, z_min_m: float = 0.30) -> int:
        """Disparity search range for depths down to ``z_min_m`` (multiple of 16, SGBM's rule)."""
        return int(16 * math.ceil(self.fB / max(0.05, z_min_m) / 16.0))

    def ray_rect(self, col, row):
        """Bot-frame direction(s) of rectified pixel(s), forward component along the optical axis = 1."""
        col = np.asarray(col, dtype=np.float64)
        row = np.asarray(row, dtype=np.float64)
        d = np.stack([np.ones_like(col), (self.cx - col) / self.f, (self.cy - row) / self.f], axis=-1)
        return d @ self.R_rect.T

    def view_bottom(self) -> tuple[float, float, float]:
        d = self.ray_rect(self.cx, self.height - 1.0)
        tan_down = -float(d[2]) / max(1e-6, math.hypot(float(d[0]), float(d[1])))
        return float(self.origin[0]), self.cam_z, tan_down

    def top_ray_height(self, x_m: float) -> float:
        d = self.ray_rect(self.cx, 0.0)
        tan_up = float(d[2]) / max(1e-6, math.hypot(float(d[0]), float(d[1])))
        return self.cam_z + (float(x_m) - float(self.origin[0])) * tan_up

    def rectify(self, img: np.ndarray, side: str) -> np.ndarray:
        if not self.needs_rect[side]:
            return img
        if side not in self._maps:
            cam = self.left if side == "left" else self.right
            rows, cols = np.mgrid[0:self.height, 0:self.width].astype(np.float64)
            d = self.ray_rect(cols, rows)  # bot frame
            loc = d @ np.array(cam.R)  # camera-local (R^T d)
            fx = np.maximum(loc[..., 0], 1e-9)
            self._maps[side] = ((cam.cx - cam.focal_px * loc[..., 1] / fx).astype(np.float32),
                                (cam.cy - cam.focal_px * loc[..., 2] / fx).astype(np.float32))
        mx, my = self._maps[side]
        return _remap_bilinear(img, mx, my)

    def describe(self) -> str:
        bx, bz, tan_down = self.view_bottom()
        floor_near = bx + bz / max(1e-6, tan_down)
        rect = "none (already rectified)" if not any(self.needs_rect.values()) else "homography"
        return (f"STEREO rig baseline {self.baseline_m * 100:.1f} cm, f {self.f:.1f} px, fB {self.fB:.2f} px*m, "
                f"{self.width}x{self.height}, lens z {self.cam_z:.3f} m, pitch "
                f"{math.degrees(math.atan2(-self.R_rect[2, 0], math.hypot(self.R_rect[0, 0], self.R_rect[1, 0]))):.1f}deg, "
                f"floor seen from {floor_near:.2f} m, top row {self.top_ray_height(1.0 + bx):.2f} m high at 1 m, "
                f"rectification {rect}")


def _remap_bilinear(img: np.ndarray, mx: np.ndarray, my: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    x0 = np.floor(mx).astype(np.int64)
    y0 = np.floor(my).astype(np.int64)
    ax, ay = mx - x0, my - y0
    x0c, x1c = np.clip(x0, 0, w - 1), np.clip(x0 + 1, 0, w - 1)
    y0c, y1c = np.clip(y0, 0, h - 1), np.clip(y0 + 1, 0, h - 1)
    a = img.astype(np.float32)
    out = ((a[y0c, x0c] * (1 - ax) + a[y0c, x1c] * ax) * (1 - ay) + (a[y1c, x0c] * (1 - ax) + a[y1c, x1c] * ax) * ay)
    out[(mx < 0) | (my < 0) | (mx > w - 1) | (my > h - 1)] = 0.0
    return out.astype(img.dtype) if img.dtype != np.float32 else out


def rig_from_models(models: dict) -> StereoRig:
    return StereoRig(models["stereo_left"], models["stereo_right"])


def bgra_to_gray(buf, width: int, height: int) -> np.ndarray:
    """Webots ``Camera.getImage()`` (BGRA bytes) -> float32 luma 0..255."""
    a = np.frombuffer(bytes(buf), dtype=np.uint8).reshape(height, width, 4).astype(np.float32)
    return 0.114 * a[..., 0] + 0.587 * a[..., 1] + 0.299 * a[..., 2]


# ---------------------------------------------------------------------------
# disparity
# ---------------------------------------------------------------------------


def _box_sum(a: np.ndarray, r: int) -> np.ndarray:
    """Sum over a (2r+1)^2 window on the last two axes (zero padded)."""
    k = 2 * r + 1
    pad = [(0, 0)] * (a.ndim - 2) + [(r + 1, r), (r + 1, r)]
    c = np.pad(a, pad).cumsum(axis=-1).cumsum(axis=-2)
    return c[..., k:, k:] - c[..., :-k, k:] - c[..., k:, :-k] + c[..., :-k, :-k]


def _xsobel(g: np.ndarray, cap: float = 31.0) -> np.ndarray:
    p = np.pad(g, 1, mode="edge")
    gx = (p[:-2, 2:] - p[:-2, :-2]) + 2.0 * (p[1:-1, 2:] - p[1:-1, :-2]) + (p[2:, 2:] - p[2:, :-2])
    return np.clip(gx * 0.25, -cap, cap)


def disparity_numpy(left: np.ndarray, right: np.ndarray, num_disp: int, block: int = 7, *,
                    uniqueness: float = 0.12, texture: float = 2.0, lr_check: float = 1.0,
                    intensity_w: float = 0.3) -> np.ndarray:
    """Block-matching disparity (left reference). Float px, NaN where rejected."""
    L = left.astype(np.float32)
    R = right.astype(np.float32)
    h, w = L.shape
    r = block // 2
    fL = np.stack([_xsobel(L), intensity_w * L])
    fR = np.stack([_xsobel(R), intensity_w * R])
    D = int(num_disp)
    big = np.float32(1e6)
    diff = np.full((D, h, w), 0.0, dtype=np.float32)
    invalid = np.zeros((D, h, w), dtype=bool)
    for d in range(D):
        if d >= w:
            invalid[d] = True
            continue
        diff[d, :, d:] = np.abs(fL[:, :, d:] - fR[:, :, : w - d]).sum(axis=0)
        invalid[d, :, :d] = True
    cost = _box_sum(diff, r)
    cost[invalid] = big
    best = np.argmin(cost, axis=0)
    yy, xx = np.mgrid[0:h, 0:w]
    c1 = cost[best, yy, xx]
    # uniqueness: best clearly better than anything outside +-1 of it
    masked = cost.copy()
    for o in (-1, 0, 1):
        idx = np.clip(best + o, 0, D - 1)
        masked[idx, yy, xx] = big
    c2 = masked.min(axis=0)
    ok = (c2 > c1 * (1.0 + uniqueness)) & (c1 < big)
    # parabola sub-pixel
    bm = np.clip(best - 1, 0, D - 1)
    bp = np.clip(best + 1, 0, D - 1)
    cm, cp = cost[bm, yy, xx], cost[bp, yy, xx]
    den = cm - 2.0 * c1 + cp
    inner = (best > 0) & (best < D - 1) & (cm < big) & (cp < big) & (den > 1e-6)
    sub = np.where(inner, 0.5 * (cm - cp) / np.where(inner, den, 1.0), 0.0)
    disp = best + np.clip(sub, -0.5, 0.5)
    # texture: enough gradient energy in the window
    tex = _box_sum(np.abs(fL[0])[None], r)[0] / float((2 * r + 1) ** 2)
    ok &= tex >= texture
    if lr_check >= 0:
        # right-reference best disparity from the same cost volume: costR(x, d) = cost(x + d, d)
        xr = xx[None] + np.arange(D)[:, None, None]
        valid_r = xr < w
        cR = np.where(valid_r, cost[np.arange(D)[:, None, None], yy[None], np.clip(xr, 0, w - 1)], big)
        bestR = np.argmin(cR, axis=0)
        xm = np.clip(np.round(xx - disp).astype(np.int64), 0, w - 1)
        ok &= np.abs(bestR[yy, xm] - best) <= lr_check
    ok[:, : r + 1] = False
    ok[:, w - r - 1:] = False
    ok[: r, :] = False
    ok[h - r:, :] = False
    out = disp.astype(np.float32)
    out[~ok | (best <= 0)] = np.nan
    return out


_SGBM_CACHE: dict = {}


def disparity_sgbm(left: np.ndarray, right: np.ndarray, num_disp: int, block: int = 5) -> np.ndarray:
    if not HAVE_CV2:
        raise RuntimeError("OpenCV (cv2) not installed: pip install opencv-python")
    key = (int(num_disp), int(block))
    m = _SGBM_CACHE.get(key)
    if m is None:
        m = cv2.StereoSGBM_create(minDisparity=0, numDisparities=int(num_disp), blockSize=int(block),
                                  P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
                                  uniquenessRatio=10, speckleWindowSize=60, speckleRange=2,
                                  preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        _SGBM_CACHE[key] = m
    L = np.clip(left, 0, 255).astype(np.uint8)
    R = np.clip(right, 0, 255).astype(np.uint8)
    d = m.compute(L, R).astype(np.float32) / 16.0
    d[d <= 0.0] = np.nan
    return d


TEXTURE_MIN = 2.0  # mean |x-Sobel|/4 in the matching window (grey levels); flat walls / sky below this


def texture_mask(gray: np.ndarray, block: int = 7, thresh: float = TEXTURE_MIN) -> np.ndarray:
    """True where the left image has enough horizontal texture to trust a match.
    SGBM's smoothness term happily paints disparities across flat walls and the
    floor horizon; without texture there is no measurement there."""
    r = block // 2
    tex = _box_sum(np.abs(_xsobel(gray.astype(np.float32)))[None], r)[0] / float(block * block)
    return tex >= thresh


def compute_disparity(left: np.ndarray, right: np.ndarray, num_disp: int, backend: str = "auto") -> tuple[np.ndarray, str]:
    """(disparity, backend used). backend: auto | sgbm | numpy."""
    if backend == "sgbm" or (backend == "auto" and HAVE_CV2):
        d = disparity_sgbm(left, right, num_disp)
        d[~texture_mask(left)] = np.nan
        return d, "sgbm"
    return disparity_numpy(left, right, num_disp, texture=TEXTURE_MIN), "numpy"


# ---------------------------------------------------------------------------
# points + corridor
# ---------------------------------------------------------------------------


def points_from_disparity(disp: np.ndarray, rig: StereoRig, max_range_m: float = 6.0):
    """(N,3) bot-frame points and their (row, col) for every valid disparity."""
    rows, cols = np.nonzero(np.isfinite(disp) & (disp > rig.fB / max(0.5, max_range_m)))
    d = disp[rows, cols].astype(np.float64)
    Z = rig.fB / d
    pts = rig.origin + rig.ray_rect(cols, rows) * Z[:, None]
    return pts, rows, cols


@dataclass
class CorridorParams:
    half_width_m: float = 0.55  # robot half width (shoulder boxes 0.443 m) + 0.10 m margin
    x_front_m: float = ROBOT_FRONT_X_M
    max_range_m: float = 4.0
    clearance_h_m: float = 0.97  # robot's own height (stereo bar ~0.80 m) + 0.15 m: anything lower blocks
    ground_tol_m: float = 0.05  # floor / tape tolerance at zero range ...
    sigma_disp_px: float = 0.25  # ... plus 3 sigma of the stereo height error (grows with range)
    window_m: float = 0.25  # depth window that must hold min_points
    min_points: int = 20  # at 320x240; scaled with the image area
    min_rows: int = 5  # the window's points must span this many image rows (at 240 rows; scaled):
    #                    a thin band of mismatches along the floor horizon / a flat edge is not an obstacle
    lateral_pad_m: float = 0.5  # how far past the corridor the lateral extent is followed
    min_area_m2: float = 0.01  # the window's points must also cover this much surface (10 x 10 cm; a
    #                            pixel at range x covers (x/f)^2): a small mismatch blob near the horizon
    #                            or an occlusion edge is not an obstacle, a 5 cm table leg still is

    def ground_tol(self, x: np.ndarray, cam_z: float, fB: float) -> np.ndarray:
        return self.ground_tol_m + 3.0 * self.sigma_disp_px * np.maximum(x, 0.0) * cam_z / fB


def detect_obstacle(pts: np.ndarray, rig: StereoRig, prm: CorridorParams, *, kappa: float = 0.0,
                    source: str = "stereo", rows: np.ndarray | None = None) -> ObstacleReport:
    rep = ObstacleReport(source=source, range_m=prm.max_range_m, view_bottom=rig.view_bottom())
    if pts.size == 0:
        rep.notes["n_valid"] = 0
        return rep
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    yc = 0.5 * float(kappa) * x * x  # corridor centre bent along the commanded curvature
    tol = prm.ground_tol(x, rig.cam_z, rig.fB)
    above = z > tol
    ahead = (x > prm.x_front_m) & (x <= prm.max_range_m)
    in_corr = np.abs(y - yc) <= prm.half_width_m
    block = above & ahead & in_corr & (z < prm.clearance_h_m)
    floor = (~above) & ahead & in_corr
    rep.notes.update(n_valid=int(len(x)), n_floor=int(floor.sum()),
                     floor_z_med=float(np.median(z[floor])) if floor.any() else None)
    min_pts = max(6, int(round(prm.min_points * rig.width * rig.height / 76800.0)))
    min_rows = max(3, int(round(prm.min_rows * rig.height / 240.0)))
    order = np.argsort(x[block])
    xb = x[block][order]
    rb = (rows[block][order] if rows is not None else None)
    rep.notes["n_block"] = int(len(xb))
    if len(xb) < min_pts:
        return rep
    hi = np.searchsorted(xb, xb + prm.window_m, side="right")
    cnt = hi - np.arange(len(xb))
    area = np.concatenate([[0.0], np.cumsum((xb / rig.f) ** 2)])  # surface covered (m^2), cumulative
    win_area = area[hi] - area[np.arange(len(xb))]
    good = np.nonzero((cnt >= min_pts) & (win_area >= prm.min_area_m2))[0]
    i0 = None
    for g in good:  # nearest window that is dense AND tall enough in the image
        if rb is None or len(np.unique(rb[g: hi[g]])) >= min_rows:
            i0 = int(g)
            break
    if i0 is None:
        rep.notes["thin_rejected"] = int(len(good))
        return rep
    win = xb[i0: hi[i0]]
    d = float(np.percentile(win, 5))
    slab = above & (x >= d - 0.05) & (x <= d + 0.6) & (np.abs(y - yc) <= prm.half_width_m + prm.lateral_pad_m)
    core = block & (x >= d - 0.05) & (x <= d + prm.window_m + 0.35)
    ys = y[core]
    y_lo, y_hi = float(np.percentile(ys, 2)), float(np.percentile(ys, 98))
    # follow the obstacle sideways past the corridor edge (its real extent)
    side = slab & (y >= y_lo - prm.lateral_pad_m) & (y <= y_hi + prm.lateral_pad_m)
    ysd = y[side]
    if len(ysd) >= min_pts:
        y_lo = min(y_lo, float(np.percentile(ysd, 2)))
        y_hi = max(y_hi, float(np.percentile(ysd, 98)))
    tall = slab & (y >= y_lo) & (y <= y_hi)
    z_top = float(np.percentile(z[tall], 98)) if tall.any() else float(np.percentile(z[core], 98))
    z_bot = float(np.percentile(z[core], 2))
    clipped = z_top >= rig.top_ray_height(d) - 0.06
    n = int(len(win))
    rep.detected = True
    rep.distance_m = round(d, 3)
    rep.clearance_m = round(d - prm.x_front_m, 3)
    rep.y_left_m, rep.y_right_m = round(y_hi, 3), round(y_lo, 3)
    rep.z_bottom_m, rep.z_top_m = round(z_bot, 3), round(z_top, 3)
    rep.top_clipped = bool(clipped)
    rep.band = height_band(z_top)
    rep.n_points = n
    rep.confidence = round(min(1.0, n / (3.0 * min_pts)), 2)
    return rep


class StereoObstacleSensor(ObstacleSource):
    """Images in, :class:`ObstacleReport` out (also an ``ObstacleSource`` for the fusion)."""

    name = "stereo"

    def __init__(self, rig: StereoRig, params: CorridorParams | None = None, backend: str = "auto",
                 z_min_m: float = 0.30) -> None:
        self.rig = rig
        self.params = params or CorridorParams()
        self.backend = backend if backend != "auto" else ("sgbm" if HAVE_CV2 else "numpy")
        self.num_disp = rig.num_disparities(z_min_m)
        self.last: ObstacleReport | None = None
        self._fresh = False
        self.disp: np.ndarray | None = None
        self.ms = 0.0

    def process_gray(self, left: np.ndarray, right: np.ndarray, *, kappa: float = 0.0) -> ObstacleReport:
        t0 = time.perf_counter()
        L = self.rig.rectify(left, "left")
        R = self.rig.rectify(right, "right")
        disp, used = compute_disparity(L, R, self.num_disp, self.backend)
        self.disp = disp
        pts, rows, _c = points_from_disparity(disp, self.rig, self.params.max_range_m + 1.0)
        rep = detect_obstacle(pts, self.rig, self.params, kappa=kappa, source=self.name, rows=rows)
        self.ms = (time.perf_counter() - t0) * 1000.0
        rep.notes.update(backend=used, ms=round(self.ms, 1))
        self.last = rep
        self._fresh = True
        return rep

    def process_bgra(self, left_buf, right_buf, *, kappa: float = 0.0) -> ObstacleReport:
        w, h = self.rig.width, self.rig.height
        return self.process_gray(bgra_to_gray(left_buf, w, h), bgra_to_gray(right_buf, w, h), kappa=kappa)

    def latest(self) -> ObstacleReport | None:
        if not self._fresh:
            return None
        self._fresh = False
        return self.last
