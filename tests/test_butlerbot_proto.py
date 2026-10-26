"""The shared ButlerBot PROTO expands to the robot the worlds used to paste.

Checked before the split: the 11 committed worlds pasted one Robot node
(md5 f7e514cd3e70cad39251257e4207c006). Pose, name, and controller did not
differ. They are still fields, and every committed instance leaves them at
the PROTO defaults, so expansion is that node byte for byte. Devices, names,
masses, physics, bounding objects, and positions are that text.
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

import pytest

from src.butlerbot_proto import SHIPPED_ROBOT_MD5, expand_world_text

ROOT = Path(__file__).resolve().parents[1]
WORLDS = sorted((ROOT / "webots" / "worlds").glob("butlerbot*.wbt"))

DEVICE_NAMES = (
    "left_wheel",
    "left_wheel_sensor",
    "left_wheel_solid",
    "right_wheel",
    "right_wheel_sensor",
    "right_wheel_solid",
    "front_left_caster",
    "front_right_caster",
    "rear_left_caster",
    "rear_right_caster",
    "torso_joint",
    "torso_sensor",
    "torso_solid",
    "gps_head",
    "left_arm",
    "left_arm_sensor",
    "left_arm_solid",
    "right_arm",
    "right_arm_sensor",
    "right_arm_solid",
    "nadir_left",
    "nadir_right",
    "stereo_left",
    "stereo_right",
    "radar_fwd",
    "hud_nadir",
    "hud_nadir_r",
    "gps",
    "imu",
    "gyro",
    "oak_depth",
    "oak_rgb",
    "oak_gyro",
    "oak_imu",
)


def _robot(name: str) -> str:
    text = (ROOT / "webots" / "worlds" / name).read_text(encoding="utf-8")
    expanded = expand_world_text(text)
    return expanded[expanded.index("Robot {") :]


@pytest.mark.parametrize("world", WORLDS, ids=lambda path: path.name)
def test_every_world_expands_to_the_shipped_robot(world):
    robot = _robot(world.name)
    assert hashlib.md5(robot.encode("utf-8")).hexdigest() == SHIPPED_ROBOT_MD5
    assert not robot.endswith("\n")


def test_warehouse_and_s_share_devices_mass_physics_and_bounds():
    warehouse = _robot("butlerbot_warehouse.wbt")
    course = _robot("butlerbot.wbt")
    assert warehouse == course
    names = re.findall(r'(?m)^\s*name "([^"]+)"', warehouse)
    for device in DEVICE_NAMES:
        assert device in names
    assert warehouse.count('name "ButlerBot"') == 1
    assert "    mass 3.2\n" in warehouse
    assert warehouse.count("mass 0.085") == 2
    assert "0.055 0.062 0.078" in warehouse
    assert "centerOfMass [\n      0 0 0.12\n    ]" in warehouse
    assert "boundingObject Pose" in warehouse
    assert "boundingObject Sphere" in warehouse
    assert "boundingObject Box" in warehouse
    assert "Cylinder {" in warehouse
    assert "translation 0.03542 0.41808 0.70419" in warehouse
    assert "translation 0.03542 -0.41808 0.70419" in warehouse
    assert "translation 0.16 0 0.45" in warehouse
    assert "rotation 0 1 0 1.5708" in warehouse
    assert "DEF RADAR_FWD Radar" in warehouse
    assert "DEF NADIR_CAM_L Camera" in warehouse
    assert "DEF NADIR_CAM_R Camera" in warehouse
    assert "DEF LEFT_WHEEL_JOINT HingeJoint" in warehouse
    assert "DEF RIGHT_WHEEL_JOINT HingeJoint" in warehouse


# Webots R2025a type of the node field on the left of an ``IS`` link.
# Physics.inertiaMatrix is MFVec3f (two vec3f: diagonal, then products).
# An MFFloat PROTO field is dropped and the body inertia comes from the
# collision shapes instead.
_R2025A_IS_FIELD_TYPE = {
    "translation": "SFVec3f",
    "rotation": "SFRotation",
    "name": "SFString",
    "controller": "SFString",
    "controllerArgs": "MFString",
    "mass": "SFFloat",
    "inertiaMatrix": "MFVec3f",
}


def test_is_fields_match_r2025a_node_field_types():
    text = (ROOT / "webots" / "protos" / "ButlerBot.proto").read_text(encoding="utf-8")
    declared = {name: typ for typ, name in re.findall(r"(?m)^  field\s+(\w+)\s+(\w+)\s", text)}
    links = re.findall(r"(?m)^[ \t]*(\w+)\s+IS\s+(\w+)\s*$", text)
    assert links
    by_proto = {proto: node for node, proto in links}
    assert by_proto["bodyMass"] == "mass"
    assert by_proto["bodyInertia"] == "inertiaMatrix"
    assert declared["bodyMass"] == "SFFloat"
    assert declared["bodyInertia"] == "MFVec3f"
    for node_field, proto_field in links:
        assert node_field in _R2025A_IS_FIELD_TYPE
        assert declared[proto_field] == _R2025A_IS_FIELD_TYPE[node_field]


def test_mapping_sensors_are_opt_in_and_do_not_change_the_warehouse_robot():
    from src.butlerbot_proto import expand_robot, expand_world_text

    warehouse = expand_robot()
    assert hashlib.md5(warehouse.encode("utf-8")).hexdigest() == SHIPPED_ROBOT_MD5
    assert "us_front" not in warehouse
    assert "floor_cam" not in warehouse
    apartment = (ROOT / "webots" / "worlds" / "apartment.wbt").read_text(encoding="utf-8")
    expanded = expand_world_text(apartment)
    assert 'name "us_left"' in expanded
    assert 'name "floor_cam"' in expanded
    assert "mappingSensors.value" not in expanded


def test_warehouse_gyro_and_wheel_encoders_stay_clean():
    """Mapping noise lives on oak_* devices. The hubs and the lane gyro do not change."""
    text = (ROOT / "webots" / "protos" / "ButlerBot.proto").read_text(encoding="utf-8")
    assert 'Gyro {\n      name "gyro"\n    }' in text
    assert 'InertialUnit {\n      name "imu"\n    }' in text
    assert 'PositionSensor {\n          name "left_wheel_sensor"\n        }' in text
    assert 'PositionSensor {\n          name "right_wheel_sensor"\n        }' in text
    assert 'name "oak_depth"' in text
    assert "noise 0.002" in text
