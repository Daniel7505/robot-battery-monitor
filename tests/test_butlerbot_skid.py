"""The skid-steer proto is a separate robot. The baseline proto stays put."""

from __future__ import annotations

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
