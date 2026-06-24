from src.hardware_profile import (
    get_active_profile,
    load_hardware_profile,
    motor_spec,
    phase_reference_draw_w,
)


def test_load_butlerbot_wheeled_profile():
    prof = load_hardware_profile("butlerbot_wheeled")
    assert prof["profile_id"] == "butlerbot_wheeled"
    assert prof["battery"]["capacity_wh"] == 480
    assert "left_wheel" in prof["motors"]
    assert prof["channels"]["Legs"]["max_draw_w"] == 28


def test_active_profile_has_motor_peaks():
    prof = get_active_profile()
    spec = motor_spec(prof, "left_wheel")
    assert spec["peak_w"] == 90
    assert spec["cont_w"] == 35


def test_phase_reference_draw():
    draw = phase_reference_draw_w("drive_transit")
    assert draw == 52.5


def test_load_butlerbot_biped_profile():
    prof = load_hardware_profile("butlerbot_biped")
    assert prof["profile_id"] == "butlerbot_biped"
    assert prof["channels"]["Legs"]["dof"] == 12
    assert prof["channels"]["Cooling"]["max_draw_w"] == 14