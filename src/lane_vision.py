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
2. Yellow pixels (min(R,G) - B >= 0.22; equals ``lane_keep.yellow_score``
   for r == g but rejects red/green marks) are grouped
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
YELLOW_MODE = "rg"  # yellow_mask: "rg" = min(R,G)-B (rejects red/green bars), "lane_keep" = gap score
FALLBACK_LANE_W_M = 1.30  # only until both lines have been seen once
MAX_RANGE_M = 3.5
MIN_SIDE_PTS = 4
# Fit windows (floor x ahead of the axle). The cameras no longer see behind
# the axle (bottom row ~0.06 m ahead since the box mount), so the near fit
# must stop before a bend that starts ~0.5 m ahead can bend the axle heading:
# 0.9 m was fine only while ~0.55 m of floor behind the axle anchored it.
# fit_windows() widens them if a camera's bottom row sits farther ahead.
NEAR_MAX_X_M = 0.6  # near-field fit (offset / heading at the axle)
NEAR_MIN_SPAN_M = 0.4  # near window is at least this long past the bottom row
FAR_MIN_X_M = 0.25  # far-field fit (curvature ahead / pursuit goal)
SPLIT_X_M = 0.6  # points nearer than this are judged by the near model

# Plausibility gate (generic: robot kinematics + paint physics, no track data).
# Lane heading relative to the robot can only change between frames by the
# robot's own yaw (measured by the IMU and already applied to the prior) plus
# lane curvature x distance driven.
GATE_MAX_HEADING_RAD = math.radians(45.0)  # lane-keep never runs this crossed
GATE_LANE_KAPPA_MAX = 2.0  # 1/m: tightest lane bend (0.5 m radius) for the jump limit
GATE_HEADING_JUMP_RAD = 0.15  # + kappa_max * |dx| per frame
GATE_OFFSET_JUMP_M = 0.06  # + |dx| * GATE_OFFSET_JUMP_PER_M per frame
GATE_OFFSET_JUMP_PER_M = 0.6
GATE_SINGLE_SIDE_MIN_CONF = 0.45  # a side seen last frame now empty AND conf below this -> reject
GATE_MAX_HELD_FRAMES = 3  # matches rowfit_control.LOST_FRAMES_STOP
# Lane visible for less than this ahead of the axle (lines ending, e.g. at a
# finish bar): the fit is too short to trust heading/curvature, so drive the
# held lane. Short-view holds don't count as lost frames until
# GATE_MAX_SHORT_HOLDS in a row (~0.8 m at cruise), then they do.
GATE_MIN_VIEW_M = 0.5
GATE_MAX_SHORT_HOLDS = 6
# Post-pivot settling: after the relaxed re-acquire window (and again when the
# controller returns to LANE) the jump checks stay widened for a few frames,
# because the first fits of the new leg still settle (only part of it in view).
GATE_SETTLE_FRAMES = 5
GATE_SETTLE_HEADING_RAD = math.radians(25.0)  # added to the heading-jump limit
GATE_SETTLE_OFFSET_M = 0.15  # added to the offset-jump limit

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


# Fallback copy of webots/worlds/butlerbot.wbt (NADIR_CAM_L / NADIR_CAM_R).
# The controller reads the live pose from Webots at start-up
# (camera_model_from_device); these are only the last fallback and the
# default for tests/offline tools. tests/test_lane_vision.py parses the .wbt
# and fails if these drift. Lens on the front-bottom edge of NADIR_BOX_L/R
# (box centre 0.01042 +/-0.41808 0.72919, 0.05 m cube), pitch from
# scripts/aim_camera.py (top row -> 1.96 m ahead of the axle).
NADIR_LEFT_POSE = {
    "translation": (0.03542, 0.41808, 0.70419),
    "rotation": (0.0, 1.0, 0.0, 0.9411),
    "width": 128,
    "height": 128,
    "fov": 1.2,
}
NADIR_RIGHT_POSE = {
    "translation": (0.03542, -0.41808, 0.70419),
    "rotation": (0.0, 1.0, 0.0, 0.9411),
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


def _mat_mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _mat_vec(A, v):
    return tuple(A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2] for i in range(3))


def matrix_to_axis_angle(R) -> tuple[float, float, float, float]:
    """Rotation matrix -> Webots axis-angle (x, y, z, angle), angle in [0, pi]."""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    ang = math.acos(c)
    if ang < 1e-9:
        return (0.0, 1.0, 0.0, 0.0)
    x = R[2][1] - R[1][2]
    y = R[0][2] - R[2][0]
    z = R[1][0] - R[0][1]
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:  # angle ~ pi: axis from the diagonal
        x = math.sqrt(max(0.0, (R[0][0] + 1.0) / 2.0))
        y = math.copysign(math.sqrt(max(0.0, (R[1][1] + 1.0) / 2.0)), R[0][1] or 1.0)
        z = math.copysign(math.sqrt(max(0.0, (R[2][2] + 1.0) / 2.0)), R[0][2] or 1.0)
        n = math.sqrt(x * x + y * y + z * z) or 1.0
    return (x / n, y / n, z / n, ang)


def camera_look_angles(cam: CameraModel) -> tuple[float, float]:
    """(pitch_down_rad, yaw_left_rad) of the optical axis (camera +X) in the robot frame."""
    dx, dy, dz = _mat_vec(cam.R, (1.0, 0.0, 0.0))
    return (math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx))


def describe_camera(cam: CameraModel) -> str:
    pitch, yaw = camera_look_angles(cam)
    t = ", ".join(f"{v:.4f}" for v in cam.translation)
    r = " ".join(f"{v:.6g}" for v in cam.rotation)
    return (
        f"t=({t}) rot=({r}) pitch={math.degrees(pitch):.2f}deg yaw={math.degrees(yaw):+.2f}deg "
        f"{cam.width}x{cam.height} fov={cam.fov_rad:.3f}"
    )


def _node_local_pose(node):
    ft, fr = node.getField("translation"), node.getField("rotation")
    if ft is None or fr is None:
        raise LookupError("node has no translation/rotation field")
    t = ft.getSFVec3f()
    r = fr.getSFRotation()
    return tuple(float(v) for v in t[:3]), _axis_angle_matrix(tuple(float(v) for v in r[:4]))


def _node_id(node):
    try:
        return node.getId()
    except Exception:
        return id(node)


def _device_tag(device):
    """Integer device tag of a Webots Device (R2025a Python keeps it in ``_tag``)."""
    for attr in ("getTag", "tag"):
        v = getattr(device, attr, None)
        v = v() if callable(v) else v
        if isinstance(v, int):
            return v
    v = getattr(device, "_tag", None)
    return v if isinstance(v, int) else None


def find_camera_node(robot, device=None, def_name: str | None = None):
    """Supervisor Node of a camera device. Returns (node, how, errors).

    Webots R2025a's Python ``Supervisor.getFromDevice(tag)`` takes the integer
    device TAG and hands it straight to ctypes; passing the Camera object raises
    ``ctypes.ArgumentError`` ("Don't know how to convert parameter 1"). So try
    the tag first, then the object (other wrappers / versions), then
    ``getFromDef(def_name)``. Every failure is recorded in ``errors``.
    """
    errors: list[str] = []
    attempts = []
    if device is not None and hasattr(robot, "getFromDevice"):
        tag = _device_tag(device)
        if tag is not None:
            attempts.append(("getFromDevice(tag)", lambda: robot.getFromDevice(tag)))
        attempts.append(("getFromDevice(device)", lambda: robot.getFromDevice(device)))
    elif device is not None:
        errors.append("robot has no getFromDevice (not a Supervisor?)")
    if def_name:
        if hasattr(robot, "getFromDef"):
            attempts.append((f"getFromDef({def_name!r})", lambda: robot.getFromDef(def_name)))
        else:
            errors.append("robot has no getFromDef (not a Supervisor?)")
    for how, fn in attempts:
        try:
            node = fn()
        except Exception as exc:  # ctypes.ArgumentError, TypeError, ...
            errors.append(f"{how}: {type(exc).__name__}: {exc}")
            continue
        if node is None:
            errors.append(f"{how}: returned None")
            continue
        return node, how, errors
    return None, "", errors


def supervisor_camera_pose(robot, device=None, def_name: str | None = None, max_depth: int = 16) -> dict:
    """Camera pose in the Robot node frame, read live through the Supervisor API.

    Finds the camera node (:func:`find_camera_node`), then composes the local
    translation/rotation fields of the camera and every Transform/Pose/Solid
    between it and the Robot node (``robot.getSelf()``). For a direct Robot
    child that is just the camera's own fields. Raises LookupError with the
    collected reasons if anything is missing (caller falls back).
    """
    node, how, errors = find_camera_node(robot, device, def_name)
    if node is None:
        raise LookupError("camera node not found: " + ("; ".join(errors) or "no lookup possible"))
    if not hasattr(robot, "getSelf"):
        raise LookupError("robot has no getSelf (not a Supervisor?)")
    self_node = robot.getSelf()
    if self_node is None:
        raise LookupError("robot.getSelf() returned None")
    self_id = _node_id(self_node)
    # p_robot = R_acc * p_cam + t_acc, walking up the parent chain
    t_acc = (0.0, 0.0, 0.0)
    R_acc = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    cur = node
    for _ in range(max_depth):
        try:
            t, R = _node_local_pose(cur)
        except Exception as exc:
            raise LookupError(f"translation/rotation fields unreadable via {how}: {type(exc).__name__}: {exc}")
        t_acc = tuple(a + b for a, b in zip(_mat_vec(R, t_acc), t))
        R_acc = _mat_mul(R, R_acc)
        parent = cur.getParentNode()
        if parent is None:
            raise LookupError("camera is not under the robot node")
        if _node_id(parent) == self_id:
            break
        cur = parent
    else:
        raise LookupError("camera nested too deep")
    return {"translation": t_acc, "rotation": matrix_to_axis_angle(R_acc), "lookup": how}


def _device_optics(device, fallback: dict) -> tuple[int, int, float]:
    try:
        return int(device.getWidth()), int(device.getHeight()), float(device.getFov())
    except Exception:
        return int(fallback["width"]), int(fallback["height"]), float(fallback["fov"])


def camera_model_from_device(
    robot,
    device,
    name: str,
    *,
    def_name: str | None = None,
    wbt_path: str | None = None,
    fallback_pose: dict | None = None,
    errors: list | None = None,
) -> tuple[CameraModel, str]:
    """Build the CameraModel for a live camera. Returns (model, source).

    Pose source order: ``supervisor`` (live node fields) -> ``wbt`` (parse the
    loaded world, or the repo world) -> ``constants`` (NADIR_*_POSE).
    Width/height/fov always come from the device when it answers. Why the
    supervisor read failed (exception text) is appended to ``errors``.
    """
    const = fallback_pose or {"nadir_left": NADIR_LEFT_POSE, "nadir_right": NADIR_RIGHT_POSE}.get(name)
    pose = None
    source = ""
    try:
        pose = supervisor_camera_pose(robot, device, def_name)
        source = "supervisor"
    except Exception as exc:
        pose = None
        if errors is not None:
            errors.append(f"supervisor: {type(exc).__name__}: {exc}")
    if pose is None:
        paths = []
        if wbt_path:
            paths.append(wbt_path)
        try:
            wp = robot.getWorldPath()
            if wp:
                paths.append(str(wp))
        except Exception:
            pass
        paths.append(_WBT_DEFAULT)
        for path in paths:
            try:
                cams = parse_wbt_cameras(path)
            except Exception:
                continue
            if name in cams:
                pose = cams[name]
                source = f"wbt {os.path.basename(path)}"
                break
    if pose is None:
        if const is None:
            raise LookupError(f"no pose for camera {name}")
        pose = const
        source = "constants"
    base = const or pose
    if "width" in pose:
        base = {**base, **{k: pose[k] for k in ("width", "height", "fov")}}
    w, h, fov = _device_optics(device, base) if device is not None else (base["width"], base["height"], base["fov"])
    model = CameraModel(name, pose["translation"], pose["rotation"], w, h, fov)
    return model, source


def yaw_pitch_matrix(yaw_rad: float, pitch_rad: float):
    """Camera rotation (robot frame) for "yaw first, then pitch": Rz(yaw) * Ry(pitch).

    pitch > 0 looks down (Webots ``0 1 0 pitch``), yaw > 0 turns the view to
    the robot's left about the vertical, so the horizon stays level (no roll).
    """
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    Rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    Ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    return _mat_mul(Rz, Ry)


def yaw_pitch_rotation(yaw_rad: float, pitch_rad: float) -> tuple[float, float, float, float]:
    """Webots axis-angle ``rotation x y z angle`` for :func:`yaw_pitch_matrix`.

    Mirror a camera across the robot's x-z plane (left <-> right shoulder) by
    negating translation y AND the yaw; for a yawed camera that is axis
    (-x, y, -z) with the same angle. (Pure pitch: the rotation line is unchanged.)
    """
    if abs(yaw_rad) < 1e-12:
        return (0.0, 1.0, 0.0, float(pitch_rad)) if pitch_rad >= 0 else (0.0, -1.0, 0.0, -float(pitch_rad))
    return matrix_to_axis_angle(yaw_pitch_matrix(yaw_rad, pitch_rad))


def aim_pitch_for_reach(
    translation: tuple[float, float, float],
    fov_rad: float,
    width: int,
    height: int,
    far_x_m: float,
    *,
    plane_z: float = PAINT_TOP_Z_M,
    yaw_rad: float = 0.0,
) -> float:
    """Pitch that puts the top row's centre on the floor far_x_m ahead (robot forward x).

    yaw_rad = 0: pure pitch (rotation 0 1 0 theta). Otherwise the camera is
    yawed first, then pitched (:func:`yaw_pitch_rotation`), and far_x_m is the
    forward (robot x) distance of that top-centre floor point.
    Uses CameraModel itself (bisection), so it matches the runtime math
    including the half-pixel centre of row 0.
    """
    if float(translation[2]) <= plane_z:
        raise ValueError("camera must be above the floor plane")
    if float(far_x_m) <= float(translation[0]):
        raise ValueError("far reach must be ahead of the camera")

    def _top_x(p):
        g = CameraModel("aim", translation, yaw_pitch_rotation(yaw_rad, p), width, height, fov_rad,
                        plane_z).pixel_to_ground((width - 1) / 2.0, 0.0)
        return math.inf if g is None else g[0]

    lo, hi = 0.0, math.pi / 2.0  # top_x decreases with pitch
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _top_x(mid) > far_x_m:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# --------------------------------------------------------------------------
# Detection
# --------------------------------------------------------------------------


def yellow_mask(
    image, width: int, height: int, thresh: float = YELLOW_THRESH, mode: str | None = None
) -> bytearray:
    """1 where a BGRA pixel is lane yellow.

    ``mode="rg"`` (default, YELLOW_MODE): min(R, G) - B >= thresh. Yellow needs
    BOTH red and green; red or green marks (finish/start bars) don't pass.
    For r == g (the yellow paint and its blend with a grey floor) it equals
    the gap-mode score.
    ``mode="lane_keep"``: ``lane_keep.yellow_score`` = (r+g)/2 - b >= thresh
    (gap-mode test; scores the red finish bar 0.9/0.15/0.12 as 0.41 = yellow).
    """
    n = int(width) * int(height)
    buf = bytes(image[: n * 4]) if not isinstance(image, (bytes, bytearray)) else image
    if len(buf) < n * 4:
        return bytearray(n)
    bs = buf[0: n * 4: 4]
    gs = buf[1: n * 4: 4]
    rs = buf[2: n * 4: 4]
    if (mode or YELLOW_MODE) == "lane_keep":
        lim = 510.0 * float(thresh)
        return bytearray(1 if (r + g - 2 * b) >= lim else 0 for b, g, r in zip(bs, gs, rs))
    lim = 255.0 * float(thresh)
    return bytearray(1 if (min(r, g) - b) >= lim else 0 for b, g, r in zip(bs, gs, rs))


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
    yellow_mode: str | None = None,
    mask=None,
) -> list[LinePoint]:
    """Stripe-centre floor points from row and column runs of yellow.

    ``mask``: a precomputed :func:`yellow_mask` (then ``image`` may be None)."""
    w = int(width or cam.width)
    h = int(height or cam.height)
    if (not image and mask is None) or w < 4 or h < 4:
        return []
    if mask is None:
        mask = yellow_mask(image, w, h, thresh, yellow_mode)
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
# Sharp-corner cue (Dan's idea: "the line widens, then disappears").
#
# Walking a lane line up the image (near -> far), its row run stays one stripe
# wide. Where the line turns 90 deg the run in one row suddenly spans far more
# than a stripe (the line now runs ACROSS the row), smeared toward the turn,
# and the rows beyond it have no yellow where the line would have continued.
# A rounded corner widens gradually over many rows instead (ramp), so it is
# not flagged. No track knowledge: only the camera model and the mask.
# --------------------------------------------------------------------------

CORNER_WIDE_FACTOR = 3.0  # crossbar run >= this x measured stripe width (ground metres)
CORNER_WIDE_MIN_M = 0.15  # ... and at least this wide (2.5 stripes)
CORNER_NORMAL_FACTOR = 1.6  # run <= this x stripe = still a normal line row
CORNER_RAMP_MAX_M = 0.15  # last normal row -> crossbar row (x). Rounded corners ramp over 0.3+ m
CORNER_MAX_RAMP_ROWS = 1  # rows between normal and crossbar width (a bend has several)
CORNER_MIN_LINE_ROWS = 4  # normal rows tracked before the crossbar
CORNER_MIN_SMEAR_M = 0.12  # smear past the line on the turn side
CORNER_EMPTY_BAND_M = 0.10  # half-width of the "line would continue here" band
CORNER_EMPTY_MAX_PX = 2  # yellow pixels allowed in that band beyond the crossbar
CORNER_CLIP_MARGIN_M = 0.12  # fit points this close to / past the crossbar are dropped
CORNER_PERSIST_FRAMES = 2
CORNER_PERSIST_TOL_M = 0.25  # |d_now - (d_before - dx)|
CORNER_MIN_CONF = 0.6  # ``seen`` needs this (after persistence)


@dataclass
class CornerCue:
    """One camera's view of one line turning sharply."""

    cam: str
    direction: int  # +1 = smear toward robot left (+y) = left turn, -1 = right
    role: str  # "outer": line on the far side of the smear (crosses the lane), "inner"
    x_m: float  # ground x of the crossbar centre at the line (robot frame, ahead of the axle)
    line_y_m: float  # ground y of the line just before the crossbar
    wide_m: float  # crossbar run ground width
    stripe_m: float  # measured stripe width of this line (median of its normal rows)
    ramp_m: float  # x from the last normal row to the crossbar row
    empty_beyond: bool | None  # None = crossbar at the top of the view, can't tell
    row: int
    col: float


@dataclass
class CornerEstimate:
    """Fused corner cue (both cameras, persistence over frames)."""

    seen: bool  # persistent for CORNER_PERSIST_FRAMES and conf >= CORNER_MIN_CONF
    direction: int  # +1 left, -1 right
    distance_m: float  # ahead of the axle to the outer line's crossbar (stripe centre)
    confidence: float
    frames: int  # consecutive consistent frames
    cams: str  # e.g. "nadir_right outer + nadir_left inner"
    clip_x_m: float  # fit points with x >= this were dropped this frame
    cues: list = field(default_factory=list)

    @property
    def side(self) -> str:
        return "left" if self.direction > 0 else "right"


def _row_runs_ground(cam: CameraModel, mask, w: int, row: int):
    """[(a, b, col_mid, ground_width_m, y_a, y_b, x_m)] for one row's yellow runs."""
    out = []
    for a, b in _runs(mask[row * w: (row + 1) * w]):
        p0 = cam.pixel_to_ground(a - 0.5, row)
        p1 = cam.pixel_to_ground(b + 0.5, row)
        if p0 is None or p1 is None:
            continue
        out.append((a, b, 0.5 * (a + b), math.hypot(p1[0] - p0[0], p1[1] - p0[1]), p0[1], p1[1],
                    0.5 * (p0[0] + p1[0])))
    return out


def trace_lines(cam: CameraModel, mask, width: int | None = None, height: int | None = None,
                *, stripe_w: float = STRIPE_W_M):
    """Follow every lane line up the image (near -> far).

    Returns ``(rows, tracks, used)``: ``rows[r]`` = that row's yellow runs with
    ground widths (:func:`_row_runs_ground`); each track is a dict with its
    normal-width rows (``cols``, ``widths``, ground ``pts``), ``first_x`` /
    ``last_x`` / ``last_y`` and how it ended (``end``): ``"wide"`` (crossbar row
    in ``wide``), ``"gap"`` (no paint where it would continue), ``"edge"`` (left
    the picture sideways) or ``"top"`` (still going at the top row). ``used``
    = {row: set(run indexes claimed by a track)}.
    """
    w = int(width or cam.width)
    h = int(height or cam.height)
    rows = {r: _row_runs_ground(cam, mask, w, r) for r in range(h)}
    tracks: list[dict] = []
    used_all: dict[int, set] = {}
    for r in range(h - 1, -1, -1):
        runs = rows[r]
        used = used_all.setdefault(r, set())
        for t in tracks:
            if t["state"] != "on":
                continue
            cols = t["cols"]
            if len(cols) >= 2:
                (r1, c1), (r0, c0) = cols[-1], cols[max(0, len(cols) - 4)]
                slope = (c1 - c0) / (r1 - r0) if r1 != r0 else 0.0
                pred = c1 + slope * (r - r1)
            else:
                pred = cols[-1][1]
            t["pred"] = pred
            tol = 3.0 + 0.6 * t["px"]
            best = None
            for i, run in enumerate(runs):
                a, b = run[0], run[1]
                if a - tol <= pred <= b + tol:
                    d = 0.0 if a <= pred <= b else min(abs(pred - a), abs(pred - b))
                    if best is None or d < best[0]:
                        best = (d, i)
            if best is None:
                t["gap"] += 1
                if t["gap"] > 3:
                    t["state"] = "ended"
                    t["end"] = "edge" if (t["edge_touch"] or pred < 1.0 or pred > w - 2.0) else "gap"
                continue
            i = best[1]
            used.add(i)
            a, b, cm, gw, ya, yb, gx = runs[i]
            ref = _median(t["widths"][:6])  # the line's own stripe width, from its nearest rows
            t["gap"] = 0
            if gw <= CORNER_NORMAL_FACTOR * max(ref, 0.75 * stripe_w):
                t["widths"].append(gw)
                t["cols"].append((r, cm))
                t["px"] = b - a + 1
                t["last_x"] = gx
                t["last_y"] = 0.5 * (ya + yb)
                t["pts"].append((gx, 0.5 * (ya + yb)))
                t["edge_touch"] = a == 0 or b == w - 1
            elif gw >= max(CORNER_WIDE_FACTOR * ref, CORNER_WIDE_MIN_M):
                t["state"] = "wide"
                t["end"] = "wide"
                t["wide"] = (r, a, b, gw, ya, yb, pred)
            else:
                t["ramp_rows"] += 1  # widening, keep following its centre
                t["cols"].append((r, pred))
        for i, run in enumerate(runs):
            if i in used or len(tracks) >= 8:
                continue
            a, b, cm, gw, ya, yb, gx = run
            if gw <= CORNER_NORMAL_FACTOR * stripe_w and a > 0 and b < w - 1:
                used.add(i)
                tracks.append({"state": "on", "cols": [(r, cm)], "widths": [gw], "px": b - a + 1, "gap": 0,
                               "ramp_rows": 0, "last_x": gx, "last_y": 0.5 * (ya + yb), "first_x": gx,
                               "first_row": r, "pts": [(gx, 0.5 * (ya + yb))], "edge_touch": False,
                               "end": None, "cam": cam.name})
    for t in tracks:
        if t["state"] == "on":
            t["end"] = "top"
    return rows, tracks, used_all


BAR_FILTER_MIN_M = 0.20  # free row run this wide (ground) = a line across the view


def _drop_bar_points(pts, traced):
    """Drop fit points that lie on a free wide run (a road line running across).

    At a crossing the far side's lines run across the view; their row runs are
    wide and claimed by no lane line (:func:`trace_lines`), and their column
    runs give points all along them. Only used while a junction is near (the
    controller sets ``LaneTracker.bar_filter_frames``)."""
    rows, _tracks, used = traced
    bad: dict[int, list] = {}
    for r, runs in rows.items():
        for i, (a, b, _cm, gw, *_rest) in enumerate(runs):
            if gw >= BAR_FILTER_MIN_M and i not in used.get(r, ()):
                bad.setdefault(r, []).append((a - 1.0, b + 1.0))
    if not bad:
        return pts
    out = []
    for p in pts:
        spans = bad.get(int(round(p.row)))
        if spans and any(a <= p.col <= b for a, b in spans):
            continue
        out.append(p)
    return out


def corner_cues(cam: CameraModel, mask, width: int | None = None, height: int | None = None,
                *, stripe_w: float = STRIPE_W_M, traced=None) -> list[CornerCue]:
    """Lines in one camera that 'widen, then disappear' (see module notes).

    Each line is followed from the bottom row upward (nearest run to the
    column extrapolated from its last rows). The first row whose run is
    >= CORNER_WIDE_FACTOR x the line's measured stripe width, reached within
    CORNER_RAMP_MAX_M of a normal-width row, is the crossbar. The smear side
    gives the turn direction; the band where the line would continue is
    checked for yellow in the farther rows. ``traced``: a :func:`trace_lines`
    result to reuse.
    """
    w = int(width or cam.width)
    h = int(height or cam.height)
    if not mask or w < 4 or h < 4:
        return []
    rows, tracks, _used = traced if traced is not None else trace_lines(cam, mask, w, h, stripe_w=stripe_w)
    done = [t for t in tracks if t["end"] == "wide"]
    cues: list[CornerCue] = []
    for t in done:
        if len(t["widths"]) < CORNER_MIN_LINE_ROWS:
            continue
        r, a, b, gw, ya, yb, pred = t["wide"]
        if t["ramp_rows"] > CORNER_MAX_RAMP_ROWS:
            continue  # widened gradually over several rows: a rounded bend, not a crossbar
        ref = _median(t["widths"][:6])
        near_edge = cam.pixel_to_ground(pred, r + 0.5)
        line_pt = cam.pixel_to_ground(pred, r)
        if near_edge is None or line_pt is None:
            continue
        ramp = near_edge[0] - t["last_x"]
        if ramp > CORNER_RAMP_MAX_M:
            continue
        yl = t["last_y"]
        y_hi, y_lo = max(ya, yb), min(ya, yb)
        ext_left, ext_right = y_hi - yl, yl - y_lo
        if max(ext_left, ext_right) < CORNER_MIN_SMEAR_M:
            continue
        direction = 1 if ext_left > ext_right else -1
        role = "outer" if (yl < 0.0) == (direction > 0) else "inner"
        # rows beyond: is there yellow where the line would have continued?
        empty: bool | None = None
        rr = r - 1
        while rr >= 0 and any(ra <= pred <= rb and rg >= CORNER_WIDE_FACTOR * ref * 0.6
                              for ra, rb, _c, rg, *_ in rows[rr]):
            rr -= 1  # still inside the crossbar stripe
        if rr >= 1:
            band = 0
            for q in range(rr, -1, -1):
                pl = cam.pixel_to_ground(pred, q)
                if pl is None:
                    continue
                # band in pixels at this row: CORNER_EMPTY_BAND_M on the ground
                p_r = cam.pixel_to_ground(pred + 1.0, q)
                m_per_px = abs(p_r[1] - pl[1]) if p_r is not None else stripe_w
                half_px = CORNER_EMPTY_BAND_M / max(1e-3, m_per_px)
                c0, c1 = int(max(0, pred - half_px)), int(min(w - 1, pred + half_px))
                band += sum(mask[q * w + c0: q * w + c1 + 1])
            empty = band <= CORNER_EMPTY_MAX_PX
        cue = CornerCue(cam=cam.name, direction=direction, role=role,
                        x_m=near_edge[0] + 0.5 * stripe_w, line_y_m=yl, wide_m=gw, stripe_m=ref,
                        ramp_m=ramp, empty_beyond=empty, row=r, col=pred)
        t["cue"] = cue
        cues.append(cue)
    return cues


def _median(v):
    s = sorted(v)
    n = len(s)
    if not n:
        return 0.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def fuse_corner_cues(cues: list[CornerCue]) -> tuple[int, float, float, str, list[CornerCue]] | None:
    """(direction, distance_m, confidence, cams, used cues) from one frame's cues, or None.

    Needs an OUTER cue (the line that crosses the lane: right line smearing
    left = left turn). An inner cue (the inner line bending away) of the same
    direction, nearer by about a lane width, adds confidence; on its own it
    never triggers (a rounded corner still has a mitred inner line).
    """
    outer = [c for c in cues if c.role == "outer"]
    if not outer:
        return None
    dirs = {c.direction for c in outer}
    if len(dirs) != 1:
        return None
    d = dirs.pop()
    best = min(outer, key=lambda c: c.x_m)
    conf = 0.5
    if best.empty_beyond:
        conf += 0.25
    elif best.empty_beyond is False:
        conf -= 0.2
    inner = [c for c in cues if c.role == "inner" and c.direction == d and c.x_m < best.x_m - 0.2]
    used = [best]
    if inner:
        conf += 0.2
        used.append(min(inner, key=lambda c: c.x_m))
    if any(c.role == "outer" and c is not best and abs(c.x_m - best.x_m) < 0.15 for c in outer):
        conf += 0.05  # both cameras see the same crossbar
    cams = " + ".join(f"{c.cam} {c.role}" for c in used)
    return d, best.x_m, max(0.0, min(1.0, conf)), cams, used


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
    corner: "CornerEstimate | None" = None  # sharp-corner cue this frame (LaneTracker)
    junction: "object | None" = None  # src.junction_vision.Junction this frame (LaneTracker)
    view: "dict | None" = None  # src.junction_vision.yellow_view: nearest yellow ahead / left / right / bar
    # Plausibility gate (LaneTracker): a rejected fit comes back valid=False,
    # held=True, carrying the last good lane moved by odometry/IMU, and the
    # reason. The rejected fit's own numbers are kept in ``rejected``.
    held: bool = False
    counts_as_lost: bool = True  # held: does it count toward the lost-lane brake
    reject_reason: str = ""
    rejected: dict = field(default_factory=dict)

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


def fit_windows(cams) -> tuple[float, float, float]:
    """(near_max_x, split_x, far_min_x) for these cameras' floor coverage.

    Defaults NEAR_MAX_X_M / SPLIT_X_M / FAR_MIN_X_M, pushed out so the near
    window keeps >= NEAR_MIN_SPAN_M of floor past the farthest-forward bottom
    row and the far window starts at or after it.
    """
    bots = []
    for cam in cams:
        x = camera_coverage(cam)["bottom_row_x_m"]
        if x is not None:
            bots.append(x)
    bottom = max(bots) if bots else 0.0
    near_max = max(NEAR_MAX_X_M, bottom + NEAR_MIN_SPAN_M)
    split = min(near_max, max(SPLIT_X_M, bottom + NEAR_MIN_SPAN_M))
    far_min = max(FAR_MIN_X_M, bottom)
    return near_max, split, far_min


class LaneTracker:
    """Stateful per-frame lane fit (keeps prior + online lane width)."""

    def __init__(self, cams: dict[str, CameraModel] | None = None, *, yellow_mode: str | None = None) -> None:
        self.cams = dict(cams or NADIR_CAMS)
        self.yellow_mode = yellow_mode
        self.near_max_x, self.split_x, self.far_min_x = fit_windows(self.cams.values())
        self.prior_near: tuple[float, float, float] | None = None
        self.prior_far: tuple[float, float, float] | None = None
        self.lane_w: float | None = None
        self.frames_lost = 0
        self.last: LaneEstimate | None = None
        self._last_good: LaneEstimate | None = None
        self.rejects = 0
        self.short_holds = 0
        self.corner: CornerEstimate | None = None  # this frame's fused corner cue
        self._corner_track: tuple[int, float, int] | None = None  # (dir, distance, frames)
        self.relax_frames = 0  # gate relaxed (jump / vanish / short-view checks off) this many frames
        self.settle_frames = 0  # jump limits widened this many frames (post-pivot settling)
        from src.junction_vision import JunctionTracker

        self.junction = None  # this frame's junction classification (src.junction_vision.Junction)
        self.view: dict | None = None
        self.bar_filter_frames = 0  # drop points on lines running ACROSS the view (junction / crossing)
        self._junction_track = JunctionTracker()

    def reset(self) -> None:
        self.prior_near = None
        self.prior_far = None
        self.lane_w = None
        self.frames_lost = 0
        self.last = None
        self._last_good = None
        self.rejects = 0
        self.short_holds = 0
        self.corner = None
        self._corner_track = None
        self.relax_frames = 0
        self.settle_frames = 0
        self.junction = None
        self._junction_track.reset()

    def begin_settle(self, frames: int = GATE_SETTLE_FRAMES) -> None:
        """Widen the jump checks for ``frames`` frames (the new lane is settling)."""
        self.settle_frames = max(self.settle_frames, int(frames))

    def begin_reacquire(self, relax_frames: int = 6) -> None:
        """After a pivot: forget the old lane (keep the measured width), relax the gate.

        The next fits start like start-up (left camera + left of the robot =
        left line), and for ``relax_frames`` frames only the +/-45 deg check
        of the plausibility gate applies, so the new leg's lane is not
        rejected as a jump from the old one.
        """
        self.prior_near = self.prior_far = None
        self.frames_lost = 0
        self._last_good = None
        self.short_holds = 0
        self.corner = None
        self._corner_track = None
        self.junction = None
        self._junction_track.reset()
        self.relax_frames = int(relax_frames)

    def _update_corner(self, cues: list[CornerCue], dx_m: float) -> CornerEstimate | None:
        fused = fuse_corner_cues(cues)
        if fused is None:
            self._corner_track = None
            return None
        d, dist, conf, cams, used = fused
        frames = 1
        if self._corner_track is not None:
            pd, pdist, pf = self._corner_track
            if pd == d and abs(dist - (pdist - abs(dx_m))) < CORNER_PERSIST_TOL_M:
                frames = pf + 1
        self._corner_track = (d, dist, frames)
        return CornerEstimate(
            seen=frames >= CORNER_PERSIST_FRAMES and conf >= CORNER_MIN_CONF,
            direction=d, distance_m=dist, confidence=conf, frames=frames, cams=cams,
            clip_x_m=dist - 0.5 * STRIPE_W_M - CORNER_CLIP_MARGIN_M, cues=used,
        )

    def _update_junction(self, per_cam, dx_m: float):
        from src.junction_vision import classify_junction, yellow_view

        try:
            self.view = yellow_view(per_cam)
            j = classify_junction(per_cam, self.lane_w or FALLBACK_LANE_W_M)
        except Exception:  # never let the classifier take the lane fit down
            if os.environ.get("RBM_JUNCTION_STRICT"):
                raise
            j = None
        self.junction = self._junction_track.update(j, dx_m)
        return self.junction

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
        cues: list[CornerCue] = []
        per_cam = []
        for name, cam in self.cams.items():
            item = images.get(name)
            if not item or not item[0]:
                continue
            w, h = int(item[1] or cam.width), int(item[2] or cam.height)
            mask = yellow_mask(item[0], w, h, YELLOW_THRESH, self.yellow_mode)
            cam_pts = detect_points(cam, None, w, h, yellow_mode=self.yellow_mode, mask=mask)
            if w >= 4 and h >= 4:
                traced = trace_lines(cam, mask, w, h)
                if self.bar_filter_frames > 0:
                    cam_pts = _drop_bar_points(cam_pts, traced)
            pts.extend(cam_pts)
            if w >= 4 and h >= 4:
                cc = corner_cues(cam, mask, w, h, traced=traced)
                cues.extend(cc)
                per_cam.append((cam, traced, cc))
        corner = self._update_corner(cues, float(dx_m))
        junction = self._update_junction(per_cam, float(dx_m))
        if junction is not None and not junction.kind.startswith(("corner", "opening")):
            lim = junction.near_m - 0.5 * STRIPE_W_M - CORNER_CLIP_MARGIN_M
            if junction.kind in ("side_L", "side_R"):
                left = junction.kind == "side_L"
                hi = (junction.far_m + CORNER_CLIP_MARGIN_M) if junction.far_m is not None else 1e9
                pts = [p for p in pts if not ((p.y > 0.0) == left and lim <= p.x <= hi)]
            else:
                pts = [p for p in pts if p.x < lim]
        if corner is not None:
            # The lane ends at the crossbar: fit only what is before it, so the
            # turning paint neither bends the fit nor trips the jump gate.
            pts = [p for p in pts if p.x < corner.clip_x_m]
            for c in corner.cues:
                if c.role == "inner":  # the inner line bends away here: what is past it is the new leg
                    lim = c.x_m - 0.5 * STRIPE_W_M - CORNER_CLIP_MARGIN_M
                    pts = [p for p in pts if not (p.x >= lim and (p.y > 0.0) == (c.line_y_m > 0.0))]
        self.corner = corner
        est = self.fit_points_gated(pts, dx_m=float(dx_m))
        est.corner = corner
        est.junction = junction
        est.view = self.view
        if self.bar_filter_frames > 0:
            self.bar_filter_frames -= 1
        if self.relax_frames > 0:
            self.relax_frames -= 1
            if self.relax_frames == 0:
                self.begin_settle()  # relaxed window over: settle before the full gate
        elif self.settle_frames > 0:
            self.settle_frames -= 1
        self.last = est
        return est

    def gate_reason(self, est: LaneEstimate, pred_near, dx_m: float) -> str:
        """Why a fitted lane is physically implausible ("" = accept)."""
        if not est.valid:
            return ""
        if abs(est.heading_rad) > GATE_MAX_HEADING_RAD:
            return f"heading {math.degrees(est.heading_rad):+.0f}deg beyond +/-{math.degrees(GATE_MAX_HEADING_RAD):.0f}"
        if self.relax_frames > 0:
            return ""  # re-acquiring after a pivot: the old lane is no reference
        last = self._last_good if pred_near is not None else None
        if last is not None and est.confidence < GATE_SINGLE_SIDE_MIN_CONF:
            for side, n_now, n_before in (("left", est.n_left, last.n_left), ("right", est.n_right, last.n_right)):
                if n_now == 0 and n_before >= MIN_SIDE_PTS:
                    return (
                        f"{side} side vanished (L{est.n_left} R{est.n_right}, was "
                        f"L{last.n_left} R{last.n_right}) at conf {est.confidence:.2f}"
                    )
        if pred_near is not None and est.lookahead_m < GATE_MIN_VIEW_M:
            return f"short view: lane seen only {est.lookahead_m:.2f} m ahead (< {GATE_MIN_VIEW_M:.2f})"
        if pred_near is not None:
            a0, psi, k = pred_near
            p_off = signed_lateral(0.0, 0.0, a0, psi, k)
            p_hd = -(psi + k * (-a0 * math.sin(psi)))
            lim_h = GATE_HEADING_JUMP_RAD + GATE_LANE_KAPPA_MAX * abs(dx_m)
            settling = self.settle_frames > 0
            if settling:
                lim_h += GATE_SETTLE_HEADING_RAD
            if abs(est.heading_rad - p_hd) > lim_h:
                return (
                    f"heading jump {math.degrees(est.heading_rad - p_hd):+.0f}deg > "
                    f"{math.degrees(lim_h):.0f}deg for {abs(dx_m):.2f} m driven (IMU yaw applied"
                    f"{', settling' if settling else ''})"
                )
            lim_o = GATE_OFFSET_JUMP_M + GATE_OFFSET_JUMP_PER_M * abs(dx_m)
            if settling:
                lim_o += GATE_SETTLE_OFFSET_M
            if abs(est.offset_m - p_off) > lim_o:
                return f"offset jump {100 * (est.offset_m - p_off):+.0f}cm > {100 * lim_o:.0f}cm for {abs(dx_m):.2f} m driven"
        return ""

    def held_estimate(self, reason: str, rejected: LaneEstimate | None = None) -> LaneEstimate:
        """Last good lane (priors already moved by odometry/IMU), flagged held."""
        rej = {}
        if rejected is not None and rejected.valid:
            rej = {
                "offset_m": rejected.offset_m, "heading_rad": rejected.heading_rad,
                "curvature_1pm": rejected.curvature_1pm, "confidence": rejected.confidence,
                "n_left": rejected.n_left, "n_right": rejected.n_right,
            }
        width = self.lane_w if self.lane_w is not None else FALLBACK_LANE_W_M
        if self.prior_near is None or self.prior_far is None:
            return LaneEstimate(valid=False, held=False, reject_reason=reason, rejected=rej, lane_width_m=width)
        a0n, psin, kn = self.prior_near
        a0, psi, k = self.prior_far
        last = self._last_good
        return LaneEstimate(
            valid=False,
            offset_m=signed_lateral(0.0, 0.0, a0n, psin, kn),
            heading_rad=-(psin + kn * (-a0n * math.sin(psin))),
            curvature_1pm=k,
            lookahead_m=0.0 if last is None else last.lookahead_m,
            confidence=0.0 if last is None else last.confidence,
            lane_width_m=width,
            width_measured=self.lane_w is not None,
            a0=a0,
            psi=psi,
            near_curvature_1pm=kn,
            held=True,
            reject_reason=reason,
            rejected=rej,
        )

    def fit_points_gated(self, pts: list[LinePoint], *, dx_m: float = 0.0) -> LaneEstimate:
        """:meth:`fit_points` + plausibility gate. A rejected fit does not touch
        the priors / lane width; it counts as a lost frame (priors dropped after
        GATE_MAX_HELD_FRAMES in a row, as for a missing lane)."""
        pred_near, pred_far = self.prior_near, self.prior_far
        lane_w, lost = self.lane_w, self.frames_lost
        est = self.fit_points(pts)
        reason = self.gate_reason(est, pred_near, dx_m)
        if not reason:
            if est.valid:
                self._last_good = est
                self.short_holds = 0
            return est
        self.prior_near, self.prior_far, self.lane_w = pred_near, pred_far, lane_w
        short = reason.startswith("short view")
        self.short_holds = self.short_holds + 1 if short else 0
        if short and self.short_holds <= GATE_MAX_SHORT_HOLDS:
            self.frames_lost = lost  # a short view is not a lost lane (yet)
        else:
            self.frames_lost = lost + 1
        held = self.held_estimate(reason, est)
        held.counts_as_lost = self.frames_lost > lost
        if self.frames_lost >= GATE_MAX_HELD_FRAMES:
            self.prior_near = self.prior_far = None
        self.rejects += 1
        return held

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
            near_pts = [p for p in pts if p.side and p.x <= self.near_max_x]
            nb = _side_counts(near_pts)
            near_both = nb[0] >= MIN_SIDE_PTS and nb[1] >= MIN_SIDE_PTS
            near = self._refit(near_pts, near_m, width, fit_width=near_both)
            far = self._refit([p for p in pts if p.side and p.x >= self.far_min_x], far_m, width, False)
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
        if both and 0.4 < width < 3.0 and self.relax_frames <= 0:
            # (frozen while re-acquiring after a pivot: the corner's paint is still in view)
            self.lane_w = width if self.lane_w is None else 0.8 * self.lane_w + 0.2 * width
        measured = self.lane_w is not None
        a0n, psin, kn = near_m
        a0, psi, kappa = far_m
        res_all = []
        ok = 0
        for p in inl:
            m = near_m if p.x <= self.split_x else far_m
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

    def _assign_piecewise(self, pts, near_m, far_m, lane_w, gate):
        hw = 0.5 * lane_w
        for p in pts:
            m = near_m if p.x <= self.split_x else far_m
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
    marks=(),
) -> bytes:
    """Render paint polylines (robot frame) into a BGRA buffer, nearest-sample.

    ``marks``: extra (polyline, width_m, rgb) strokes in other colours, e.g. a
    red finish bar (drawn first, so lane paint wins where they overlap).
    """
    cell = 0.1
    grid: dict[tuple[int, int], list] = {}
    strokes = [(m[0], 0.5 * float(m[1]), tuple(m[2])) for m in marks]
    strokes += [(line, stripe_w / 2.0, tuple(paint)) for line in lines]
    for line, half, stroke_rgb in strokes:
        for i in range(len(line) - 1):
            (x0, y0), (x1, y1) = line[i], line[i + 1]
            for gx in range(int(math.floor((min(x0, x1) - half) / cell)), int(math.floor((max(x0, x1) + half) / cell)) + 1):
                for gy in range(int(math.floor((min(y0, y1) - half) / cell)), int(math.floor((max(y0, y1) + half) / cell)) + 1):
                    grid.setdefault((gx, gy), []).append((x0, y0, x1, y1, half, stroke_rgb))
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
                for x0, y0, x1, y1, half, stroke_rgb in reversed(segs):
                    vx, vy = x1 - x0, y1 - y0
                    L2 = vx * vx + vy * vy
                    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - x0) * vx + (p[1] - y0) * vy) / L2))
                    if math.hypot(p[0] - (x0 + t * vx), p[1] - (y0 + t * vy)) <= half:
                        color = stroke_rgb
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
