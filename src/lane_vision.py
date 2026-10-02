"""Row-fit lane vision: nadir pixels -> ground points -> fitted lane model.

Stdlib only (the Webots controller runs on the host python and imports
nothing heavier than ``math``). Nothing here knows the track: the lane
is measured from the two shoulder cameras every frame.

Pipeline (one call per new camera frame, see :class:`LaneTracker`):

1. ``CameraModel`` maps a pixel (col, row) to a ray in the robot frame and
   intersects it with the flat paint plane. Webots R2025a camera axes:
   the camera looks along its local +X, +Y is image-left, +Z is image-up
   (doc "camera.md": s=0 <-> +hFov/2, t=0 <-> +vFov/2). Robot frame:
   origin on the floor under the drive axle, +x forward, +y left, +z up.
2. Yellow pixels (same score as ``lane_keep.yellow_score``) are grouped
   into runs along every image ROW and every image COLUMN. Each run's two
   edges are projected to the floor; the midpoint of the two floor points
   lies on the painted stripe's centre line for ANY stripe angle, so
   sideways lines (90 deg turns) work. Runs clipped by the image border
   are dropped (their midpoint is not the stripe centre).
3. Points are labelled left/right line (camera + sign at start-up, then
   the previous frame's model, motion-compensated by odometry/IMU).
4. One lane model is fitted to both lines at once: a centre line that is a
   circle arc (straight when curvature = 0) through (0, a0) with direction
   psi and curvature kappa; left/right paint sit at +/- width/2 from it.
   Robust Gauss-Newton (Huber + gate). Lane width is measured online and
   reused when only one line is visible.
5. The known 6 cm stripe width is only a CHECK: each run's floor length
   times |cos| to the fitted line normal should be ~6 cm.
"""

from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field

STRIPE_W_M = 0.06
# s_track.PAINT_Z (0.008 m box centre) + half the 0.010 m box height.
PAINT_TOP_Z_M = 0.013
YELLOW_THRESH = 0.22  # same as nadir_wheel_to_tape
FALLBACK_LANE_W_M = 1.30  # only until both lines have been seen once
MAX_RANGE_M = 3.5
MIN_SIDE_PTS = 4
NEAR_MAX_X_M = 0.9  # near-field fit (offset / heading at the axle)
FAR_MIN_X_M = 0.25  # far-field fit (curvature ahead / pursuit goal)
SPLIT_X_M = 0.6  # points nearer than this are judged by the near model

_WBT_DEFAULT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "webots",
    "worlds",
    "butlerbot.wbt",
)


# --------------------------------------------------------------------------
# Camera model
# --------------------------------------------------------------------------


def _axis_angle_matrix(rot: tuple[float, float, float, float]) -> tuple[tuple[float, ...], ...]:
    ax, ay, az, ang = (float(v) for v in rot)
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z = ax / n, ay / n, az / n
    c, s = math.cos(ang), math.sin(ang)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


class CameraModel:
    """Pinhole Webots camera fixed on the robot (flat floor)."""

    def __init__(
        self,
        name: str,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
        width: int,
        height: int,
        fov_rad: float,
        plane_z: float = PAINT_TOP_Z_M,
    ) -> None:
        self.name = str(name)
        self.translation = tuple(float(v) for v in translation)
        self.rotation = tuple(float(v) for v in rotation)
        self.width = int(width)
        self.height = int(height)
        self.fov_rad = float(fov_rad)
        self.plane_z = float(plane_z)
        self.R = _axis_angle_matrix(self.rotation)
        # Horizontal FOV; square pixels -> same focal length vertically.
        self.focal_px = (self.width / 2.0) / math.tan(self.fov_rad / 2.0)
        self.cx = (self.width - 1) / 2.0
        self.cy = (self.height - 1) / 2.0

    def ray(self, col: float, row: float) -> tuple[float, float, float]:
        """Robot-frame direction through pixel centre (col, row)."""
        d = (self.focal_px, self.cx - float(col), self.cy - float(row))
        R = self.R
        return (
            R[0][0] * d[0] + R[0][1] * d[1] + R[0][2] * d[2],
            R[1][0] * d[0] + R[1][1] * d[1] + R[1][2] * d[2],
            R[2][0] * d[0] + R[2][1] * d[1] + R[2][2] * d[2],
        )

    def pixel_to_ground(
        self, col: float, row: float, plane_z: float | None = None
    ) -> tuple[float, float] | None:
        """(forward x, left y) on the plane z = plane_z, or None above horizon."""
        z0 = self.plane_z if plane_z is None else float(plane_z)
        dx, dy, dz = self.ray(col, row)
        if dz >= -1e-9:
            return None
        tx, ty, tz = self.translation
        t = (z0 - tz) / dz
        if t <= 0.0:
            return None
        return (tx + t * dx, ty + t * dy)

    def ground_to_pixel(
        self, x: float, y: float, z: float | None = None
    ) -> tuple[float, float] | None:
        """Inverse of :meth:`pixel_to_ground` (continuous col,row)."""
        z0 = self.plane_z if z is None else float(z)
        tx, ty, tz = self.translation
        v = (float(x) - tx, float(y) - ty, z0 - tz)
        R = self.R  # camera = R^T * robot
        cxv = R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2]
        cyv = R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2]
        czv = R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2]
        if cxv <= 1e-9:
            return None
        return (self.cx - self.focal_px * cyv / cxv, self.cy - self.focal_px * czv / cxv)


# Mirrors webots/worlds/butlerbot.wbt (NADIR_CAM_L / NADIR_CAM_R).
# tests/test_lane_vision.py parses the .wbt and fails if these drift.
NADIR_LEFT_POSE = {
    "translation": (-0.386826, 0.504837, 1.306679),
    "rotation": (-0.00125006, 0.999996, 0.00256741, 1.1),
    "width": 128,
    "height": 128,
    "fov": 1.2,
}
NADIR_RIGHT_POSE = {
    "translation": (-0.386826, -0.504837, 1.306679),
    "rotation": (-0.00125006, 0.999996, 0.00256741, 1.1),
    "width": 128,
    "height": 128,
    "fov": 1.2,
}


def _model_from_pose(name: str, pose: dict) -> CameraModel:
    return CameraModel(
        name,
        pose["translation"],
        pose["rotation"],
        pose["width"],
        pose["height"],
        pose["fov"],
    )


NADIR_LEFT_CAM = _model_from_pose("nadir_left", NADIR_LEFT_POSE)
NADIR_RIGHT_CAM = _model_from_pose("nadir_right", NADIR_RIGHT_POSE)
NADIR_CAMS = {"nadir_left": NADIR_LEFT_CAM, "nadir_right": NADIR_RIGHT_CAM}


def parse_wbt_cameras(path: str | None = None) -> dict[str, dict]:
    """Read every ``Camera { ... }`` block from a .wbt (fields at block level).

    Camera nodes in butlerbot.wbt are direct children of the Robot, whose
    origin is the robot frame used here.
    """
    with open(path or _WBT_DEFAULT, encoding="utf-8") as fh:
        text = fh.read()
    out: dict[str, dict] = {}
    for m in re.finditer(r"\bCamera\s*\{", text):
        depth = 0
        i = m.end() - 1
        while i < len(text):
            ch = text[i]
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        body = text[m.end(): i]
        body = re.sub(r"#[^\n]*", "", body)

        def _nums(key: str, n: int, default):
            mm = re.search(rf"(?m)^\s*{key}\s+([-+0-9.eE\s]+)$", body)
            if not mm:
                return default
            vals = [float(v) for v in mm.group(1).split()[:n]]
            return tuple(vals) if n > 1 else vals[0]

        nm = re.search(r'(?m)^\s*name\s+"([^"]+)"', body)
        name = nm.group(1) if nm else "camera"
        out[name] = {
            "translation": _nums("translation", 3, (0.0, 0.0, 0.0)),
            "rotation": _nums("rotation", 4, (0.0, 0.0, 1.0, 0.0)),
            "width": int(_nums("width", 1, 64.0)),
            "height": int(_nums("height", 1, 64.0)),
            "fov": float(_nums("fieldOfView", 1, 0.7854)),
        }
    return out


def camera_coverage(cam: CameraModel) -> dict:
    """Floor footprint numbers for docs/logs (centre column)."""
    c = cam.cx
    h = cam.height

    def _x(row: float) -> float | None:
        p = cam.pixel_to_ground(c, row)
        return None if p is None else p[0]

    def _m_per_px(row: float) -> tuple[float, float] | None:
        p0 = cam.pixel_to_ground(c, row)
        pc = cam.pixel_to_ground(c + 1.0, row)
        r1 = row - 1.0 if row > 0 else row + 1.0
        pr = cam.pixel_to_ground(c, r1)
        if p0 is None or pc is None or pr is None:
            return None
        return (math.hypot(pc[0] - p0[0], pc[1] - p0[1]), math.hypot(pr[0] - p0[0], pr[1] - p0[1]))

    return {
        "bottom_row_x_m": _x(h - 1),
        "top_row_x_m": _x(0),
        "mid_row_x_m": _x(cam.cy),
        "bottom_m_per_px_lat_long": _m_per_px(h - 1),
        "top_m_per_px_lat_long": _m_per_px(0),
        "focal_px": cam.focal_px,
    }


# --------------------------------------------------------------------------
# Detection
# --------------------------------------------------------------------------


def yellow_mask(image, width: int, height: int, thresh: float = YELLOW_THRESH) -> bytearray:
    """1 where ``lane_keep.yellow_score(rgb) >= thresh`` on a BGRA buffer.

    yellow_score = (r+g)/2 - b in 0..1, so >= thresh  <=>  R+G-2B >= 510*thresh.
    """
    n = int(width) * int(height)
    buf = bytes(image[: n * 4]) if not isinstance(image, (bytes, bytearray)) else image
    if len(buf) < n * 4:
        return bytearray(n)
    lim = 510.0 * float(thresh)
    bs = buf[0: n * 4: 4]
    gs = buf[1: n * 4: 4]
    rs = buf[2: n * 4: 4]
    return bytearray(1 if (r + g - 2 * b) >= lim else 0 for b, g, r in zip(bs, gs, rs))


def _runs(seq) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    start = -1
    for i, v in enumerate(seq):
        if v:
            if start < 0:
                start = i
        elif start >= 0:
            out.append((start, i - 1))
            start = -1
    if start >= 0:
        out.append((start, len(seq) - 1))
    return out


@dataclass
class LinePoint:
    x: float
    y: float
    cam: str
    axis: str  # "row" or "col"
    seg_dx: float  # floor vector edge->edge (for the width check)
    seg_dy: float
    run_px: int
    col: float
    row: float
    side: int = 0  # +1 left line, -1 right line, 0 unassigned


def detect_points(
    cam: CameraModel,
    image,
    width: int | None = None,
    height: int | None = None,
    *,
    thresh: float = YELLOW_THRESH,
    max_run_px: int = 60,
) -> list[LinePoint]:
    """Stripe-centre floor points from row and column runs of yellow."""
    w = int(width or cam.width)
    h = int(height or cam.height)
    if not image or w < 4 or h < 4:
        return []
    mask = yellow_mask(image, w, h, thresh)
    pts: list[LinePoint] = []

    def _add(e0, e1, axis, run_px, ccol, crow):
        p0 = cam.pixel_to_ground(*e0)
        p1 = cam.pixel_to_ground(*e1)
        if p0 is None or p1 is None:
            return
        mx, my = 0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])
        if math.hypot(mx, my) > MAX_RANGE_M:
            return
        pts.append(LinePoint(mx, my, cam.name, axis, p1[0] - p0[0], p1[1] - p0[1], run_px, ccol, crow))

    for row in range(h):
        line = mask[row * w: (row + 1) * w]
        for a, b in _runs(line):
            if a == 0 or b == w - 1 or (b - a + 1) > max_run_px:
                continue
            _add((a - 0.5, row), (b + 0.5, row), "row", b - a + 1, 0.5 * (a + b), float(row))
    for col in range(w):
        line = mask[col::w]
        for a, b in _runs(line):
            if a == 0 or b == h - 1 or (b - a + 1) > max_run_px:
                continue
            _add((col, a - 0.5), (col, b + 0.5), "col", b - a + 1, float(col), 0.5 * (a + b))
    return pts


# --------------------------------------------------------------------------
# Lane geometry: centre arc through (0, a0), direction psi, curvature kappa
# --------------------------------------------------------------------------


def signed_lateral(px: float, py: float, a0: float, psi: float, kappa: float) -> float:
    """Signed distance of (px,py) from the centre arc, + = left of it."""
    ux, uy = px, py - a0
    c, s = math.cos(psi), math.sin(psi)
    along = ux * c + uy * s
    lat = -ux * s + uy * c
    A = 2.0 * lat - kappa * (along * along + lat * lat)
    q = 1.0 - kappa * A
    return A / (1.0 + math.sqrt(max(0.0, q)))


def arc_point(a0: float, psi: float, kappa: float, s: float, x0: float = 0.0) -> tuple[float, float]:
    """Point at arc length s from the anchor (x0, a0)."""
    if abs(kappa) < 1e-9:
        fx, fy = s, 0.0
    else:
        fx = math.sin(kappa * s) / kappa
        fy = (1.0 - math.cos(kappa * s)) / kappa
    c, sn = math.cos(psi), math.sin(psi)
    return (x0 + fx * c - fy * sn, a0 + fx * sn + fy * c)


def _solve(A: list[list[float]], b: list[float]) -> list[float] | None:
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-14:
            return None
        M[i], M[p] = M[p], M[i]
        for r in range(n):
            if r != i:
                f = M[r][i] / M[i][i]
                if f:
                    for k in range(i, n + 1):
                        M[r][k] -= f * M[i][k]
    return [M[i][n] / M[i][i] for i in range(n)]


@dataclass
class LaneEstimate:
    valid: bool
    offset_m: float = 0.0  # robot vs lane centre at the axle, + = robot LEFT of centre
    heading_rad: float = 0.0  # robot vs lane direction, + = robot pointed LEFT of lane
    curvature_1pm: float = 0.0  # + = lane bends LEFT
    lookahead_m: float = 0.0  # farthest forward inlier
    confidence: float = 0.0
    n_left: int = 0
    n_right: int = 0
    lane_width_m: float = 0.0
    width_measured: bool = False
    rms_m: float = 0.0
    width_ok_frac: float = 0.0  # share of points whose floor width ~ 6 cm
    a0: float = 0.0
    psi: float = 0.0
    near_curvature_1pm: float = 0.0
    pixels: dict = field(default_factory=dict)  # cam -> [(col,row,side)] for the HUD

    def goal_point(self, lookahead_m: float) -> tuple[float, float]:
        """Centre-line point at straight-line distance ~lookahead from the axle."""
        Ld = max(0.05, float(lookahead_m))
        s0 = -self.a0 * math.sin(self.psi)  # arc length to the point abreast
        lo, hi = s0, s0 + Ld + abs(self.a0) + 0.5
        best = arc_point(self.a0, self.psi, self.curvature_1pm, hi)
        if math.hypot(*best) < Ld:
            # tight curl: take the farthest point reachable on the arc
            far, fs = best, hi
            for i in range(1, 41):
                s = s0 + (hi - s0) * i / 40.0
                p = arc_point(self.a0, self.psi, self.curvature_1pm, s)
                if math.hypot(*p) > math.hypot(*far):
                    far, fs = p, s
            return far
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            p = arc_point(self.a0, self.psi, self.curvature_1pm, mid)
            if math.hypot(*p) < Ld:
                lo = mid
            else:
                hi = mid
        return arc_point(self.a0, self.psi, self.curvature_1pm, 0.5 * (lo + hi))


def fit_lane(
    points: list[LinePoint],
    *,
    init: tuple[float, float, float] = (0.0, 0.0, 0.0),
    lane_w: float = FALLBACK_LANE_W_M,
    fit_width: bool = True,
    iters: int = 15,
    huber_m: float = 0.02,
    gate_m: float = 0.12,
) -> tuple[list[float], list[float], float] | None:
    """Robust GN fit. Points must carry side +/-1. Returns (params, weights, rms)."""
    pts = [p for p in points if p.side in (1, -1)]
    if len(pts) < 3:
        return None
    p = [float(init[0]), float(init[1]), float(init[2]), float(lane_w)]
    n_par = 4 if fit_width else 3
    sig_pt = 0.02
    w_prior = float(lane_w)
    wts = [1.0] * len(pts)
    mu = 1e-3

    def _res(q, pt):
        return signed_lateral(pt.x, pt.y, q[0], q[1], q[2]) - pt.side * 0.5 * q[3]

    for it in range(iters):
        r = [_res(p, pt) for pt in pts]
        for i, ri in enumerate(r):
            a = abs(ri)
            if it >= 2 and a > gate_m:
                wts[i] = 0.0
            else:
                wts[i] = 1.0 if a <= huber_m else huber_m / a
        JTJ = [[0.0] * n_par for _ in range(n_par)]
        JTr = [0.0] * n_par
        eps = 1e-6
        for i, pt in enumerate(pts):
            wi = wts[i]
            if wi <= 0.0:
                continue
            J = []
            for k in range(n_par):
                q = p[:]
                q[k] += eps
                J.append((_res(q, pt) - r[i]) / eps)
            for a_ in range(n_par):
                JTr[a_] += wi * J[a_] * r[i]
                for b_ in range(n_par):
                    JTJ[a_][b_] += wi * J[a_] * J[b_]
        # weak priors: curvature toward 0 (sigma 3 1/m), width toward estimate
        reg = [(2, 0.0, 3.0)]
        if fit_width:
            reg.append((3, w_prior, 0.4))
        for k, target, sig in reg:
            g = (sig_pt / sig) ** 2
            JTJ[k][k] += g
            JTr[k] += g * (p[k] - target)
        for k in range(n_par):
            JTJ[k][k] += mu
        step = _solve(JTJ, [-v for v in JTr])
        if step is None:
            return None
        step[1] = max(-0.5, min(0.5, step[1]))
        step[2] = max(-2.0, min(2.0, step[2]))
        for k in range(n_par):
            p[k] += step[k]
        p[2] = max(-6.0, min(6.0, p[2]))
        if max(abs(v) for v in step) < 1e-6:
            break
    r = [_res(p, pt) for pt in pts]
    for i, ri in enumerate(r):
        wts[i] = 0.0 if abs(ri) > gate_m else (1.0 if abs(ri) <= huber_m else huber_m / abs(ri))
    used = [ri for ri, wi in zip(r, wts) if wi > 0]
    if len(used) < 3:
        return None
    rms = math.sqrt(sum(v * v for v in used) / len(used))
    return p, wts, rms


def _init_from_points(pts: list[LinePoint], lane_w: float) -> tuple[float, float, float]:
    """Shared-slope line fit of (y - side*w/2) vs x as a GN starting point."""
    xs = [p.x for p in pts if p.side]
    ys = [p.y - p.side * 0.5 * lane_w for p in pts if p.side]
    if not xs:
        return (0.0, 0.0, 0.0)
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = 0.0 if sxx < 1e-6 else sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = max(-1.5, min(1.5, b))
    a = my - b * mx
    return (a, math.atan(b), 0.0)


def _side_counts(pts) -> tuple[int, int]:
    return (sum(1 for p in pts if p.side == 1), sum(1 for p in pts if p.side == -1))


def _drop_thin_sides(pts) -> None:
    nl, nr = _side_counts(pts)
    for side, n in ((1, nl), (-1, nr)):
        if 0 < n < MIN_SIDE_PTS:
            for p in pts:
                if p.side == side:
                    p.side = 0


def predict_model(
    model: tuple[float, float, float], dx: float, dyaw: float
) -> tuple[float, float, float] | None:
    """Express a centre arc in the robot frame after moving dx and turning dyaw."""
    a0, psi, k = model
    tx = dx * math.cos(0.5 * dyaw)
    ty = dx * math.sin(0.5 * dyaw)
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    ax, ay = -tx, a0 - ty
    nax, nay = c * ax - s * ay, s * ax + c * ay
    npsi = psi - dyaw
    sarc = 0.0
    for _ in range(25):  # slide the anchor along the arc back to x = 0
        px, _py = arc_point(nay, npsi, k, sarc, x0=nax)
        ca = math.cos(npsi + k * sarc)
        if abs(ca) < 0.2:
            return None
        if abs(px) < 1e-7:
            break
        sarc -= px / ca
    px, py = arc_point(nay, npsi, k, sarc, x0=nax)
    return (py, npsi + k * sarc, k)


class LaneTracker:
    """Stateful per-frame lane fit (keeps prior + online lane width)."""

    def __init__(self, cams: dict[str, CameraModel] | None = None) -> None:
        self.cams = dict(cams or NADIR_CAMS)
        self.prior_near: tuple[float, float, float] | None = None
        self.prior_far: tuple[float, float, float] | None = None
        self.lane_w: float | None = None
        self.frames_lost = 0
        self.last: LaneEstimate | None = None

    def reset(self) -> None:
        self.prior_near = None
        self.prior_far = None
        self.lane_w = None
        self.frames_lost = 0
        self.last = None

    def _predict(self, dx: float, dyaw: float) -> None:
        """Move the priors into the new robot frame (odometry dx, IMU dyaw)."""
        if self.prior_near is None or self.prior_far is None:
            return
        self.prior_near = predict_model(self.prior_near, dx, dyaw)
        self.prior_far = predict_model(self.prior_far, dx, dyaw)
        if self.prior_near is None or self.prior_far is None:
            self.prior_near = self.prior_far = None

    def process(
        self,
        images: dict[str, tuple],
        *,
        dx_m: float = 0.0,
        dyaw_rad: float = 0.0,
    ) -> LaneEstimate:
        """images: cam name -> (bgra_bytes, width, height)."""
        self._predict(float(dx_m), float(dyaw_rad))
        pts: list[LinePoint] = []
        for name, cam in self.cams.items():
            item = images.get(name)
            if not item or not item[0]:
                continue
            pts.extend(detect_points(cam, item[0], item[1], item[2]))
        est = self.fit_points(pts)
        self.last = est
        return est

    def fit_points(self, pts: list[LinePoint]) -> LaneEstimate:
        lane_w = self.lane_w if self.lane_w is not None else FALLBACK_LANE_W_M
        width = lane_w
        if self.prior_near is not None and self.prior_far is not None:
            near_m, far_m = self.prior_near, self.prior_far
            self._assign_piecewise(pts, near_m, far_m, width, gate=0.25)
        else:
            # Start-up: left camera + left of the robot = left line, etc.
            for p in pts:
                if p.cam.endswith("left") and p.y > 0.0:
                    p.side = 1
                elif p.cam.endswith("right") and p.y < 0.0:
                    p.side = -1
                else:
                    p.side = 0
            _drop_thin_sides(pts)
            both = _side_counts(pts) >= (MIN_SIDE_PTS, MIN_SIDE_PTS)
            res = fit_lane(pts, init=_init_from_points(pts, width), lane_w=width, fit_width=both)
            if res is None:
                return self._lost(width)
            near_m = far_m = (res[0][0], res[0][1], res[0][2])
            if both and 0.4 < res[0][3] < 3.0:
                width = res[0][3]
            self._assign_piecewise(pts, near_m, far_m, width, gate=0.15)
        got = False
        for it in range(3):
            _drop_thin_sides(pts)
            nl, nr = _side_counts(pts)
            if nl == 0 and nr == 0:
                break
            near_pts = [p for p in pts if p.side and p.x <= NEAR_MAX_X_M]
            nb = _side_counts(near_pts)
            near_both = nb[0] >= MIN_SIDE_PTS and nb[1] >= MIN_SIDE_PTS
            near = self._refit(near_pts, near_m, width, fit_width=near_both)
            far = self._refit([p for p in pts if p.side and p.x >= FAR_MIN_X_M], far_m, width, False)
            if near is None and far is None:
                res = fit_lane(pts, init=near_m, lane_w=width, fit_width=False)
                if res is None:
                    break
                near = far = ((res[0][0], res[0][1], res[0][2]), res[0][3], res[2])
            near = near or far
            far = far or near
            near_m, far_m = near[0], far[0]
            if near_both and 0.4 < near[1] < 3.0:
                width = near[1]
            got = True
            self._assign_piecewise(pts, near_m, far_m, width, gate=0.15 if it < 2 else 0.10)
        if not got:
            return self._lost(width)
        inl = [p for p in pts if p.side in (1, -1)]
        n_left = sum(1 for p in inl if p.side == 1)
        n_right = sum(1 for p in inl if p.side == -1)
        if n_left + n_right < MIN_SIDE_PTS:
            return self._lost(width)
        both = n_left >= MIN_SIDE_PTS and n_right >= MIN_SIDE_PTS
        if both and 0.4 < width < 3.0:
            self.lane_w = width if self.lane_w is None else 0.8 * self.lane_w + 0.2 * width
        measured = self.lane_w is not None
        a0n, psin, kn = near_m
        a0, psi, kappa = far_m
        res_all = []
        ok = 0
        for p in inl:
            m = near_m if p.x <= SPLIT_X_M else far_m
            res_all.append(signed_lateral(p.x, p.y, *m) - p.side * 0.5 * width)
            # 6 cm sanity check (validation only, not the scale)
            e = 1e-4
            gx = (signed_lateral(p.x + e, p.y, *m) - signed_lateral(p.x - e, p.y, *m)) / (2 * e)
            gy = (signed_lateral(p.x, p.y + e, *m) - signed_lateral(p.x, p.y - e, *m)) / (2 * e)
            gn = math.hypot(gx, gy) or 1.0
            wperp = abs(p.seg_dx * gx + p.seg_dy * gy) / gn
            if 0.5 * STRIPE_W_M <= wperp <= 1.8 * STRIPE_W_M:
                ok += 1
        rms = math.sqrt(sum(v * v for v in res_all) / len(res_all))
        width_ok = ok / len(inl)
        offset = signed_lateral(0.0, 0.0, a0n, psin, kn)
        s_abreast = -a0n * math.sin(psin)
        heading = -(psin + kn * s_abreast)
        look = max(p.x for p in inl)
        n_tot = n_left + n_right
        conf = min(1.0, n_tot / 40.0)
        conf *= 1.0 / (1.0 + (rms / 0.03) ** 2)
        conf *= 1.0 if both else 0.6
        conf *= 0.5 + 0.5 * width_ok
        if not measured:
            conf *= 0.7
        pixels: dict = {}
        for p in inl:
            pixels.setdefault(p.cam, []).append((p.col, p.row, p.side))
        self.frames_lost = 0
        self.prior_near = near_m
        self.prior_far = far_m
        return LaneEstimate(
            valid=True,
            offset_m=offset,
            heading_rad=heading,
            curvature_1pm=kappa,
            lookahead_m=look,
            confidence=max(0.0, min(1.0, conf)),
            n_left=n_left,
            n_right=n_right,
            lane_width_m=width if both else lane_w,
            width_measured=measured,
            rms_m=rms,
            width_ok_frac=width_ok,
            a0=a0,
            psi=psi,
            near_curvature_1pm=kn,
            pixels=pixels,
        )

    def _lost(self, width: float) -> LaneEstimate:
        self.frames_lost += 1
        if self.frames_lost >= 3:
            self.prior_near = self.prior_far = None
        return LaneEstimate(valid=False, lane_width_m=width)

    @staticmethod
    def _refit(pts, init, width, fit_width):
        sub = [p for p in pts if p.side in (1, -1)]
        if len(sub) < 2 * MIN_SIDE_PTS:
            return None
        xs = [p.x for p in sub]
        ys = [p.y for p in sub]
        if max(max(xs) - min(xs), max(ys) - min(ys)) < 0.3:
            return None
        res = fit_lane(sub, init=init, lane_w=width, fit_width=fit_width, iters=12)
        if res is None:
            return None
        params, _w, rms = res
        return (params[0], params[1], params[2]), params[3], rms

    @staticmethod
    def _assign_piecewise(pts, near_m, far_m, lane_w, gate):
        hw = 0.5 * lane_w
        for p in pts:
            m = near_m if p.x <= SPLIT_X_M else far_m
            d = signed_lateral(p.x, p.y, *m)
            side = 1 if d > 0.0 else -1
            g = gate + 0.04 * max(0.0, p.x)
            p.side = side if abs(d - side * hw) < g else 0


# --------------------------------------------------------------------------
# Synthetic renderer (tests / offline tuning). Same camera model.
# --------------------------------------------------------------------------


def lane_polyline_robot(
    centre_world: list[tuple[float, float]],
    robot_xy: tuple[float, float],
    robot_yaw: float,
) -> list[tuple[float, float]]:
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    out = []
    for wx, wy in centre_world:
        dx, dy = wx - robot_xy[0], wy - robot_xy[1]
        out.append((dx * c + dy * s, -dx * s + dy * c))
    return out


def offset_polyline(poly: list[tuple[float, float]], d: float) -> list[tuple[float, float]]:
    """Shift a polyline sideways by d (+ = left of travel direction)."""
    out = []
    n = len(poly)
    for i in range(n):
        a = poly[max(0, i - 1)]
        b = poly[min(n - 1, i + 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        L = math.hypot(tx, ty) or 1.0
        out.append((poly[i][0] - d * ty / L, poly[i][1] + d * tx / L))
    return out


def render_lane_bgra(
    cam: CameraModel,
    lines: list[list[tuple[float, float]]],
    *,
    stripe_w: float = STRIPE_W_M,
    paint=(0.85, 0.85, 0.18),
    floor=(0.55, 0.56, 0.58),
    noise: float = 0.04,
) -> bytes:
    """Render paint polylines (robot frame) into a BGRA buffer, nearest-sample."""
    cell = 0.1
    grid: dict[tuple[int, int], list] = {}
    half = stripe_w / 2.0
    for line in lines:
        for i in range(len(line) - 1):
            (x0, y0), (x1, y1) = line[i], line[i + 1]
            for gx in range(int(math.floor((min(x0, x1) - half) / cell)), int(math.floor((max(x0, x1) + half) / cell)) + 1):
                for gy in range(int(math.floor((min(y0, y1) - half) / cell)), int(math.floor((max(y0, y1) + half) / cell)) + 1):
                    grid.setdefault((gx, gy), []).append((x0, y0, x1, y1))
    w, h = cam.width, cam.height
    out = bytearray(w * h * 4)
    seed = 12345
    for row in range(h):
        for col in range(w):
            seed = (1103515245 * seed + 12345) & 0x7FFFFFFF
            jitter = ((seed / 0x7FFFFFFF) - 0.5) * 2.0 * noise
            color = floor
            p = cam.pixel_to_ground(col, row)
            if p is not None:
                segs = grid.get((int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell))), ())
                for x0, y0, x1, y1 in segs:
                    vx, vy = x1 - x0, y1 - y0
                    L2 = vx * vx + vy * vy
                    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - x0) * vx + (p[1] - y0) * vy) / L2))
                    if math.hypot(p[0] - (x0 + t * vx), p[1] - (y0 + t * vy)) <= half:
                        color = paint
                        break
            else:
                color = (0.3, 0.35, 0.45)
            o = (row * w + col) * 4
            r, g, b = (max(0.0, min(1.0, v + jitter)) for v in color)
            out[o] = int(b * 255)
            out[o + 1] = int(g * 255)
            out[o + 2] = int(r * 255)
            out[o + 3] = 255
    return bytes(out)
