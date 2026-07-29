from src.hardware_profile import (
    battery_capacity_wh,
    clear_profile_cache,
    get_active_profile,
    load_hardware_profile,
    motor_idle_and_scale,
    motor_spec,
    phase_reference_draw_w,
    wheel_radius_m,
)
from src.twin.webots_power import aggregate_channel_draws, build_webots_telemetry, estimate_motor_power_w


def setup_function():
    clear_profile_cache()


def test_load_butlerbot_wheeled_profile():
    prof = load_hardware_profile("butlerbot_wheeled")
    assert prof["profile_id"] == "butlerbot_wheeled"
    assert prof["battery"]["capacity_wh"] == 480
    assert prof["battery"]["nominal_voltage_v"] == 48
    assert "left_wheel" in prof["motors"]
    assert prof["channels"]["Legs"]["max_draw_w"] == 28
    assert "stabilizers" in prof
    assert "sensors" in prof


def test_active_profile_wheel_has_cruise_and_efficiency():
    prof = get_active_profile()
    spec = motor_spec(prof, "left_wheel")
    assert spec["efficiency"] == 0.75
    assert spec["cruise_w"] == 11.0
    assert spec["rated_power_w"] == 120
    assert battery_capacity_wh(prof) == 480
    assert wheel_radius_m(prof) == 0.08


def test_motor_scale_derived_from_cruise():
    idle, scale, tau = motor_idle_and_scale("left_wheel")
    assert idle == 2.0
    assert scale > 0.1
    assert tau == 0.38


def test_phase_reference_draw():
    assert phase_reference_draw_w("drive_transit") == 48.0
    assert phase_reference_draw_w("standby") == 18.0


def test_idle_draw_low_and_drive_rises():
    idle_joints = [
        {"name": "left_wheel", "velocity": 0.0, "torque": 0.0},
        {"name": "right_wheel", "velocity": 0.0, "torque": 0.0},
        {"name": "torso_joint", "velocity": 0.0, "torque": 0.0},
        {"name": "left_arm", "velocity": 0.0, "torque": 0.0},
        {"name": "right_arm", "velocity": 0.0, "torque": 0.0},
    ]
    # ~0.4 m/s → ω ≈ v/r = 0.4/0.08 = 5 rad/s
    moving_joints = [
        {"name": "left_wheel", "velocity": 5.0, "torque": 0.0},
        {"name": "right_wheel", "velocity": 5.0, "torque": 0.0},
        {"name": "torso_joint", "velocity": 0.0, "torque": 0.0},
        {"name": "left_arm", "velocity": 0.0, "torque": 0.0},
        {"name": "right_arm", "velocity": 0.0, "torque": 0.0},
    ]
    idle = build_webots_telemetry(
        joints=idle_joints, gait="stand", phase="standby", speed_m_s=0.0, battery_pct=98.0
    )
    drive = build_webots_telemetry(
        joints=moving_joints,
        gait="drive",
        phase="teleop",
        speed_m_s=0.40,
        battery_pct=98.0,
    )
    assert idle["channel_draws"]["Legs"] <= 6.0
    assert idle["power"]["total_draw_w"] < 25.0
    assert drive["channel_draws"]["Legs"] >= idle["channel_draws"]["Legs"] + 6.0
    assert drive["robot"]["hardware_profile"] == "butlerbot_wheeled"
    assert drive["robot"]["battery_capacity_wh"] == 480


def test_estimate_uses_profile_idle():
    rest = estimate_motor_power_w(0.0, 0.0, motor_name="left_wheel")
    assert rest == 2.0
    moving = estimate_motor_power_w(5.0, 0.0, motor_name="left_wheel")
    assert moving > rest + 3.0


def test_stabilizer_adds_when_moving():
    motors = {"left_wheel": 2.0, "right_wheel": 2.0, "torso_joint": 1.5}
    idle = aggregate_channel_draws(motors, gait="stand", phase="standby", speed_m_s=0.0)
    move = aggregate_channel_draws(motors, gait="drive", phase="teleop", speed_m_s=0.4)
    assert move["Torso"] >= idle["Torso"]


def test_load_butlerbot_biped_profile():
    prof = load_hardware_profile("butlerbot_biped")
    assert prof["profile_id"] == "butlerbot_biped"
    assert prof["channels"]["Legs"]["dof"] == 12
    assert prof["channels"]["Cooling"]["max_draw_w"] == 14
