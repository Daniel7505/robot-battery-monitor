"""Pinhole camera, paint constants, and the live Webots pose."""

from __future__ import annotations

import math
import os
import re

STRIPE_W_M = 0.06
# s_track.PAINT_Z (0.008 m box centre) + half the 0.010 m box height.
PAINT_TOP_Z_M = 0.013

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

    Camera nodes live in ``webots/protos/ButlerBot.proto`` and are direct
    children of the Robot. A world that instances ``ButlerBot`` is expanded
    first. The robot origin is the camera frame used here.
    """
    with open(path or _WBT_DEFAULT, encoding="utf-8") as fh:
        text = fh.read()
    from src.butlerbot_proto import expand_world_text

    text = expand_world_text(text)
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
        # DEF names inside ButlerBot.proto are proto-scoped. getFromDef
        # searches the world scope; getFromProtoDef reads the robot PROTO.
        if hasattr(robot, "getSelf"):

            def _from_proto(name=def_name):
                self_node = robot.getSelf()
                if self_node is None or not hasattr(self_node, "getFromProtoDef"):
                    return None
                return self_node.getFromProtoDef(name)

            attempts.append((f"getFromProtoDef({def_name!r})", _from_proto))
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
        # Callers replace src.lane_vision._WBT_DEFAULT; read that binding.
        from src.lane_vision import _WBT_DEFAULT
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
