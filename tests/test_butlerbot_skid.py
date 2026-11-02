"""The skid-steer proto is a separate robot. The baseline proto stays put."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKID = ROOT / "webots" / "protos" / "ButlerBotSkid.proto"
BASE = ROOT / "webots" / "protos" / "ButlerBot.proto"
MAPPER = ROOT / "webots" / "controllers" / "apartment_mapper" / "apartment_mapper.py"

WHEELS = (
    "LEFT_WHEEL_JOINT",
    "RIGHT_WHEEL_JOINT",
    "FRONT_LEFT_WHEEL_JOINT",
    "FRONT_RIGHT_WHEEL_JOINT",
)
ANCHORS = {
    "LEFT_WHEEL_JOINT": "-0.11 0.17 0.08",
    "RIGHT_WHEEL_JOINT": "-0.11 -0.17 0.08",
    "FRONT_LEFT_WHEEL_JOINT": "0.11 0.17 0.08",
    "FRONT_RIGHT_WHEEL_JOINT": "0.11 -0.17 0.08",
}


def _joint(text: str, name: str) -> str:
    start = text.index(f"DEF {name} HingeJoint")
    nxt = text.find("\n    DEF ", start + 1)
    return text[start:] if nxt < 0 else text[start:nxt]


def test_skid_proto_has_four_rolled_tires_and_no_casters():
    text = SKID.read_text(encoding="utf-8")
    assert text.startswith("#VRML_SIM R2025a utf8\n")
    assert "PROTO ButlerBotSkid [" in text
    assert "BallJoint" not in text
    assert 'contactMaterial "caster"' not in text
    assert "boundingObject Sphere" not in text
    assert text.count('contactMaterial "drive_wheel"') == 4
    assert text.count("mass 0.085") == 4
    for name, anchor in ANCHORS.items():
        joint = _joint(text, name)
        assert f"anchor {anchor}" in joint
        assert f"translation {anchor}" in joint
        assert "boundingObject Cylinder" not in joint
        assert joint.count("rotation 1 0 0 1.5708") == 2
        bound = joint.split("boundingObject", 1)[1]
        assert "Pose {" in bound.split("physics", 1)[0]
        assert "rotation 1 0 0 1.5708" in bound
        assert "radius 0.08" in bound
        assert "height 0.05" in bound
    assert 'name "left_wheel"' in text
    assert 'name "right_wheel"' in text
    assert 'name "front_left_wheel"' in text
    assert 'name "front_right_wheel"' in text
    assert 'name "left_wheel_sensor"' in text
    assert 'name "right_wheel_sensor"' in text
    assert 'name "floor_cam"' in text


def test_baseline_proto_and_apartment_world_are_unchanged():
    base = BASE.read_text(encoding="utf-8")
    assert "PROTO ButlerBot [" in base
    assert "ButlerBotSkid" not in base
    assert base.count("BallJoint {") == 4
    assert "anchor 0 0.17 0.08" in base
    assert "anchor 0 -0.17 0.08" in base
    assert "FRONT_LEFT_WHEEL_JOINT" not in base
    world = (ROOT / "webots" / "worlds" / "apartment.wbt").read_text(encoding="utf-8")
    assert "\nButlerBot {\n" in world
    assert "ButlerBotSkid" not in world
    assert 'EXTERNPROTO "../protos/ButlerBot.proto"' in world


def test_mapper_slaves_front_hubs_only_when_the_device_exists():
    text = MAPPER.read_text(encoding="utf-8")
    assert "front_left_wheel" in text
    assert "front_right_wheel" in text
    assert "getNumberOfDevices" in text
    assert "getDeviceByIndex" in text
    # A missing name must not be passed to getDevice: that prints a warning
    # on every baseline run.
    assert "name not in names" in text
    assert "MAPPING front hubs slaved" in text
    assert not re.search(r'getDevice\(\s*"front_left_wheel"\s*\)', text.split("name not in names", 1)[0])


MESHES = (
    "butlerbot_skid_deck.obj",
    "butlerbot_skid_pad.obj",
    "butlerbot_skid_sensors.obj",
    "butlerbot_skid_lenses.obj",
    "butlerbot_skid_wheel.obj",
    "butlerbot_skid_hub.obj",
)

# Device poses the mapper already uses, plus the moved floor-flow pods
# and the corner sonars that now sit outside the tires.
_DEVICE_POSES = (
    "translation 0 0.05 0.055",
    "translation 0 -0.05 0.055",
    "translation 0.14 0.05 0.78",
    "rotation 0 1 0 0.3469",
    "translation 0.14 0 0.78",
    "rotation 0 1 0 0.0873",
    "translation 0.16 0 0.45",
    "translation -0.16 0 0.28",
    "rotation 0 0 1 3.1416",
    "translation 0 0 0.04",
    "rotation 0 1 0 1.5708",
    'name "gps_head"',
    "translation 0.18 0 0.12",
    "translation 0 0.22 0.12",
    "translation -0.18 0 0.12",
    "translation 0.18 0.20 0.20",
    "translation -0.18 0.20 0.20",
    "translation 0.18 -0.20 0.20",
    "translation -0.18 -0.20 0.20",
)


def _brace_block(text: str, start: int) -> str:
    open_at = text.find("{", start)
    depth = 0
    for index in range(open_at, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError("unbalanced brace")


def test_skid_visual_is_mesh_and_collision_matches_blender_boxes():
    text = SKID.read_text(encoding="utf-8")
    dims = json.loads((ROOT / "webots" / "meshes" / "butlerbot_skid_dimensions.json").read_text(encoding="utf-8"))
    for name in MESHES:
        assert f"../meshes/{name}" in text
        obj = ROOT / "webots" / "meshes" / name
        assert obj.stat().st_size > 200
        assert "\nv " in obj.read_text(encoding="utf-8")[:400] or obj.read_text(encoding="utf-8").startswith("v ")
    assert text.count("geometry Mesh {") == 12
    assert "butlerbot_skid_upper.obj" not in text
    assert "0.41808" not in text
    assert "boundingObject Mesh" not in text
    assert "boundingObject IndexedFaceSet" not in text
    blocks = [
        _brace_block(text, match.start())
        for match in re.finditer(r"(?m)^[ \t]*boundingObject\b", text)
    ]
    assert len(blocks) == 5
    for block in blocks:
        assert "Mesh" not in block
        assert "IndexedFaceSet" not in block
    wheel_blocks = [block for block in blocks if "radius 0.08" in block and "height 0.05" in block]
    assert len(wheel_blocks) == 4
    for block in wheel_blocks:
        assert "rotation 1 0 0 1.5708" in block
    body = [block for block in blocks if "size 0.32 0.24 0.04" in block]
    assert len(body) == 1 and "Group" in body[0]
    by_name = {box["name"]: box for box in dims["body_boxes"]}
    assert by_name["deck"]["translation"] == "0 0 0.125"
    assert by_name["deck"]["size"] == "0.32 0.24 0.04"
    assert f"translation {by_name['deck']['translation']}" in text
    assert f"size {by_name['deck']['size']}" in text
    for item in dims["colliders"]:
        assert f"translation {item['translation']}" in text
        if item["shape"] == "box":
            assert f"size {item['size']}" in text
        else:
            assert f"radius {item['radius']}" in text
            assert f"height {item['height']}" in text
    assert dims["tray_pad"]["opening_m"] == "0.14 0.14"
    assert "size 0.16 0.12 0.28" not in text
    assert "size 0.05 0.05 0.26" not in text
    assert dims["wheel"]["radius_m"] == 0.08
    assert dims["wheel"]["height_m"] == 0.05
    for pose in _DEVICE_POSES:
        assert pose in text
    for view in (
        "skid_front.png",
        "skid_side.png",
        "skid_top.png",
        "skid_three_quarter.png",
        "skid_bottom.png",
        "skid_sensors.png",
    ):
        assert (ROOT / "webots" / "meshes" / "previews" / view).stat().st_size > 10000


def _obj_vertices(path: Path) -> list[tuple[float, float, float]]:
    verts = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            x, y, z = (float(part) for part in line.split()[1:4])
            verts.append((x, y, z))
    return verts


def _placed_wheels(verts: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    """Wheel OBJ is a Z-up cylinder. The proto rolls it 90° about X onto each hub."""
    hubs = ((0.11, 0.17, 0.08), (0.11, -0.17, 0.08), (-0.11, 0.17, 0.08), (-0.11, -0.17, 0.08))
    placed = []
    for hx, hy, hz in hubs:
        for x, y, z in verts:
            # R_x(+90°): (x, y, z) -> (x, -z, y)
            placed.append((x + hx, -z + hy, y + hz))
    return placed


def test_visual_footprint_stays_inside_the_shell():
    from src.mapping.bubble import SHELL_HALF_M, SHELL_NOSE_M, SHELL_TAIL_M

    meshes = ROOT / "webots" / "meshes"
    points = []
    for name in (
        "butlerbot_skid_deck.obj",
        "butlerbot_skid_pad.obj",
        "butlerbot_skid_sensors.obj",
        "butlerbot_skid_lenses.obj",
    ):
        points.extend(_obj_vertices(meshes / name))
    points.extend(_placed_wheels(_obj_vertices(meshes / "butlerbot_skid_wheel.obj")))
    points.extend(_placed_wheels(_obj_vertices(meshes / "butlerbot_skid_hub.obj")))
    assert points
    limit_x = max(SHELL_NOSE_M, SHELL_TAIL_M)
    for x, y, _z in points:
        assert abs(x) <= limit_x + 1e-4
        assert abs(y) <= SHELL_HALF_M + 1e-4
    width = max(abs(y) for _x, y, _z in points) * 2.0
    assert width < 0.46


def _axis_angle(rotation: tuple[float, float, float, float]):
    ax, ay, az, ang = rotation
    n = math.sqrt(ax * ax + ay * ay + az * az)
    x, y, z = ax / n, ay / n, az / n
    c, s = math.cos(ang), math.sin(ang)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def _mul(rot, vec):
    return tuple(sum(rot[row][col] * vec[col] for col in range(3)) for row in range(3))


def _sonar_rays(yaw: float, aperture: float = 0.3) -> list[tuple[float, float, float]]:
    """Webots R2025a predefined 3-ray cone, then yaw about Z."""
    quarter = math.pi / 2.0
    third = 2.0 * math.pi / 3.0
    polar = ((0.0, 1.0), (third, 1.0), (-third, 1.0))
    rays = []
    cz, sz = math.cos(yaw), math.sin(yaw)
    for alpha0, theta0 in polar:
        alpha = alpha0 + quarter
        theta = theta0 * aperture / 2.0
        lx = math.cos(theta)
        ly = -math.sin(theta)
        lz = -ly * math.sin(alpha)
        ly *= math.cos(alpha)
        rays.append((cz * lx - sz * ly, sz * lx + cz * ly, lz))
    return rays


def _ray_hits_tire(origin, direction, hub, radius=0.08, half=0.025) -> bool:
    """Tire cylinder: axis along Y, finite half-length `half`."""
    ox, oy, oz = origin
    dx, dy, dz = direction
    cx, cy, cz = hub
    # Infinite cylinder in XZ around the hub.
    a = dx * dx + dz * dz
    bx = ox - cx
    bz = oz - cz
    b = 2.0 * (bx * dx + bz * dz)
    c = bx * bx + bz * bz - radius * radius
    if a < 1e-12:
        if c > 0.0:
            return False
        ts = [0.0]
    else:
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return False
        root = math.sqrt(disc)
        ts = ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))
    for t in ts:
        if t <= 1e-4 or t > 1.2:
            continue
        y = oy + t * dy
        if abs(y - cy) <= half + 1e-6:
            return True
    return False


_HUBS = ((0.11, 0.17, 0.08), (0.11, -0.17, 0.08), (-0.11, 0.17, 0.08), (-0.11, -0.17, 0.08))


def test_corner_sonar_beams_miss_the_tires():
    text = SKID.read_text(encoding="utf-8")
    blocks = re.findall(r"DistanceSensor \{.*?\n    \}", text, flags=re.S)
    assert len(blocks) == 8
    for block in blocks:
        trans = re.search(r"translation ([-0-9.]+) ([-0-9.]+) ([-0-9.]+)", block)
        yaw = re.search(r"rotation 0 0 1 ([-0-9.]+)", block)
        assert trans is not None
        origin = tuple(float(part) for part in trans.groups())
        yaw_rad = float(yaw.group(1)) if yaw else 0.0
        for hub in _HUBS:
            assert not _point_in_tire(origin, hub)
            for direction in _sonar_rays(yaw_rad):
                assert not _ray_hits_tire(origin, direction, hub), (origin, direction, hub)


def _point_in_tire(point, hub, radius=0.08, half=0.025) -> bool:
    x, y, z = point
    cx, cy, cz = hub
    return (x - cx) ** 2 + (z - cz) ** 2 <= radius ** 2 + 1e-8 and abs(y - cy) <= half + 1e-8


def _ray_box(origin, direction, center, size) -> float | None:
    lo = [center[i] - size[i] / 2.0 for i in range(3)]
    hi = [center[i] + size[i] / 2.0 for i in range(3)]
    tmin, tmax = 0.0, 1e9
    for i in range(3):
        if abs(direction[i]) < 1e-12:
            if origin[i] < lo[i] or origin[i] > hi[i]:
                return None
            continue
        inv = 1.0 / direction[i]
        t1 = (lo[i] - origin[i]) * inv
        t2 = (hi[i] - origin[i]) * inv
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return None
    if tmax < 1e-4:
        return None
    return tmin if tmin > 1e-4 else None


def _down_cameras():
    return (
        ("floor_cam", (0.0, 0.0, 0.04), (0.0, 1.0, 0.0, 1.5708), 0.90, 32, 32, 0.01),
        ("nadir_left", (0.0, 0.05, 0.055), (0.0, 1.0, 0.0, 1.5708), 1.2, 128, 128, 0.03),
        ("nadir_right", (0.0, -0.05, 0.055), (0.0, 1.0, 0.0, 1.5708), 1.2, 128, 128, 0.03),
    )


def _solids_above_the_floor():
    boxes = [
        ((0.0, 0.0, 0.123), (0.32, 0.24, 0.036)),
        ((0.115, 0.0, 0.143), (0.09, 0.24, 0.004)),
        ((-0.115, 0.0, 0.143), (0.09, 0.24, 0.004)),
        ((0.0, 0.095, 0.143), (0.14, 0.05, 0.004)),
        ((0.0, -0.095, 0.143), (0.14, 0.05, 0.004)),
        ((0.0, 0.173, 0.12), (0.03, 0.106, 0.03)),
        ((0.0, -0.173, 0.12), (0.03, 0.106, 0.03)),
    ]
    z_cyl = [
        ((0.0, 0.0, 0.0785), 0.028, 0.0335),
        ((0.0, 0.05, 0.082), 0.016, 0.023),
        ((0.0, -0.05, 0.082), 0.016, 0.023),
    ]
    return boxes, z_cyl


def _ray_z_cylinder(origin, direction, center, radius, half_h) -> bool:
    ox, oy, oz = origin
    dx, dy, dz = direction
    cx, cy, cz = center
    a = dx * dx + dy * dy
    bx = ox - cx
    by = oy - cy
    b = 2.0 * (bx * dx + by * dy)
    c = bx * bx + by * by - radius * radius
    if a < 1e-12:
        ts = [0.0] if c <= 0.0 else []
    else:
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return False
        root = math.sqrt(disc)
        ts = ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))
    for t in ts:
        if t <= 1e-3:
            continue
        z = oz + t * dz
        if abs(z - cz) <= half_h:
            return True
    return False


def test_downward_cameras_see_only_floor():
    boxes, pods = _solids_above_the_floor()
    for name, origin, rotation, fov, width, height, near in _down_cameras():
        rot = _axis_angle(rotation)
        focal = (width / 2.0) / math.tan(fov / 2.0)
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        step = 8 if width > 32 else 1
        for row in range(0, height, step):
            for col in range(0, width, step):
                local = (focal, cx - col, cy - row)
                length = math.sqrt(sum(v * v for v in local))
                direction = _mul(rot, tuple(v / length for v in local))
                assert direction[2] < -0.5, name
                for hub in _HUBS:
                    assert not _ray_hits_tire(origin, direction, hub), name
                for center, size in boxes:
                    hit = _ray_box(origin, direction, center, size)
                    assert hit is None or hit < near, (name, center)
                for center, radius, half_h in pods:
                    assert not _ray_z_cylinder(origin, direction, center, radius, half_h), name
