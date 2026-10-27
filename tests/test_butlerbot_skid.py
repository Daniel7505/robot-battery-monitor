"""The skid-steer proto is a separate robot. The baseline proto stays put."""

from __future__ import annotations

import json
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
    "butlerbot_skid_upper.obj",
    "butlerbot_skid_sensors.obj",
    "butlerbot_skid_lenses.obj",
    "butlerbot_skid_wheel.obj",
    "butlerbot_skid_hub.obj",
)

# Device poses the mapper already uses. The Blender mounts sit on these.
_DEVICE_POSES = (
    "translation 0.03542 0.41808 0.70419",
    "rotation 0 1 0 0.9411",
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
    assert "boundingObject Mesh" not in text
    assert "boundingObject IndexedFaceSet" not in text
    blocks = [
        _brace_block(text, match.start())
        for match in re.finditer(r"(?m)^[ \t]*boundingObject\b", text)
    ]
    assert len(blocks) == 8
    for block in blocks:
        assert "Mesh" not in block
        assert "IndexedFaceSet" not in block
    wheel_blocks = [block for block in blocks if "Cylinder" in block]
    assert len(wheel_blocks) == 4
    for block in wheel_blocks:
        assert "radius 0.08" in block
        assert "height 0.05" in block
        assert "rotation 1 0 0 1.5708" in block
    by_name = {box["name"]: box for box in dims["body_boxes"]}
    assert by_name["deck"]["translation"] == "0 0 0.134"
    assert by_name["deck"]["size"] == "0.32 0.24 0.058"
    assert f"translation {by_name['deck']['translation']}" in text
    assert f"size {by_name['deck']['size']}" in text
    assert f"translation {by_name['torso']['translation']}" in text
    assert f"size {by_name['torso']['size']}" in text
    assert f"translation {by_name['left_arm']['translation']}" in text
    assert f"size {by_name['left_arm']['size']}" in text
    assert dims["wheel"]["radius_m"] == 0.08
    assert dims["wheel"]["height_m"] == 0.05
    for pose in _DEVICE_POSES:
        assert pose in text
    for view in ("skid_front.png", "skid_side.png", "skid_top.png", "skid_three_quarter.png"):
        assert (ROOT / "webots" / "meshes" / "previews" / view).stat().st_size > 10000
