from src.twin.webots_power import (
    aggregate_channel_draws,
    build_webots_telemetry,
    estimate_motor_power_w,
    gait_to_task,
    motor_powers_from_joints,
    stress_multiplier,
)
from src.twin.adapters import get_adapter


def test_estimate_motor_power_increases_with_load():
    idle = estimate_motor_power_w(0.0, 0.0)
    loaded = estimate_motor_power_w(2.0, 1.5)
    assert loaded > idle


def test_motor_powers_from_joints():
    joints = [
        {"name": "left_wheel", "velocity": 3.0, "torque": 0.5},
        {"name": "right_wheel", "velocity": 3.0, "torque": 0.5},
    ]
    powers = motor_powers_from_joints(joints)
    assert "left_wheel" in powers
    assert powers["left_wheel"] > 1.0


def test_aggregate_channel_draws_butlerbot_motors():
    motor_power = {
        "left_wheel": 4.0,
        "right_wheel": 4.2,
        "torso_joint": 2.5,
        "left_arm": 1.8,
        "right_arm": 1.6,
    }
    channels = aggregate_channel_draws(motor_power, gait="stand")
    assert channels["Legs"] == 8.2
    assert channels["Torso"] == 2.5
    assert channels["Arms"] == 3.4
    assert channels["Compute"] >= 7.5


def test_gait_to_task_mapping():
    assert gait_to_task("drive") == "moving"
    assert gait_to_task("walk") == "moving"
    assert gait_to_task("patrol") == "balanced"
    assert gait_to_task("manipulate") == "high_load"
    assert gait_to_task("stand") == "idle"


def test_build_webots_telemetry_payload():
    joints = [
        {"name": "left_wheel", "velocity": 4.0, "torque": 0.6, "position": 0.1},
        {"name": "right_wheel", "velocity": 4.0, "torque": 0.6, "position": 0.1},
    ]
    payload = build_webots_telemetry(
        joints=joints,
        gait="drive",
        phase="drive_transit",
        speed_m_s=0.35,
        battery_pct=88.0,
    )
    assert payload["source"] == "webots"
    assert payload["channel_draws"]["Legs"] > 0
    assert payload["mission"]["task"] == "moving"
    assert payload["locomotion"]["gait"] == "drive"
    assert payload["locomotion"]["mode"] == "wheeled"


def test_stress_multiplier_increases_drive_transit_draw():
    stressed = aggregate_channel_draws(
        {"left_wheel": 10.0},
        gait="drive",
        phase="drive_transit",
    )
    assert stress_multiplier(gait="drive", phase="drive_transit") > 2.0
    assert stressed["Legs"] > 20.0


def test_webots_adapter_accepts_controller_payload():
    adapter = get_adapter("webots")
    payload = build_webots_telemetry(
        joints=[{"name": "left_wheel", "velocity": 5.0, "torque": 0.8}],
        gait="patrol",
        phase="patrol",
        speed_m_s=0.2,
        battery_pct=85.0,
    )
    tel = adapter.normalize(payload)
    assert tel.source == "webots"
    assert tel.task == "balanced"
    assert tel.channel_draws.get("Legs", 0) > 0