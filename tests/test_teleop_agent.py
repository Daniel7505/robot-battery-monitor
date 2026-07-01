from src.teleop_agent import (
    AGENT_FUN_MESSAGE,
    BATTERY_AGENT_FLOOR_PCT,
    THERMAL_WARN_C,
    drive_from_key_set,
    local_agent_throttle,
    merge_throttle,
    normalize_key_code,
    update_thermal_c,
)

KEY_W, KEY_A, KEY_S, KEY_D = ord("W"), ord("A"), ord("S"), ord("D")
KEY_w = ord("w")


def test_normalize_lowercase_key():
    assert normalize_key_code(KEY_w) == KEY_W


def test_drive_forward():
    left, right = drive_from_key_set({KEY_W}, key_w=KEY_W, key_a=KEY_A, key_s=KEY_S, key_d=KEY_D)
    assert left > 0 and right > 0 and left == right


def test_drive_forward_lowercase():
    left, right = drive_from_key_set({KEY_w}, key_w=KEY_W, key_a=KEY_A, key_s=KEY_S, key_d=KEY_D)
    assert left > 0 and right > 0


def test_drive_turn_in_place():
    left, right = drive_from_key_set({KEY_A}, key_w=KEY_W, key_a=KEY_A, key_s=KEY_S, key_d=KEY_D)
    assert left < 0 < right


def test_drive_wasd_combo():
    left, right = drive_from_key_set({KEY_W, KEY_D}, key_w=KEY_W, key_a=KEY_A, key_s=KEY_S, key_d=KEY_D)
    assert left > 0 and right > 0 and left > right


def test_thermal_rises_under_load():
    t0 = 22.0
    t1 = update_thermal_c(t0, draw_w=40.0, dt_s=1.0, motion_factor=1.0)
    assert t1 > t0


def test_thermal_stays_low_during_short_teleop():
    """Short drive sessions should not hit agent throttle thresholds."""
    t = 22.0
    for _ in range(200):
        t = update_thermal_c(t, draw_w=35.0, dt_s=0.032, motion_factor=1.0)
    assert t < THERMAL_WARN_C


def test_thermal_idle_stays_near_ambient():
    t = 22.0
    for _ in range(80):
        t = update_thermal_c(t, draw_w=18.0, dt_s=0.1, motion_factor=0.0)
    assert t < 28.0


def test_thermal_cools_when_idle():
    t1 = update_thermal_c(50.0, draw_w=2.0, dt_s=2.0)
    assert t1 < 50.0


def test_agent_throttle_low_battery():
    factor, msg = local_agent_throttle(BATTERY_AGENT_FLOOR_PCT - 1, 30.0)
    assert factor < 1.0
    assert msg == AGENT_FUN_MESSAGE


def test_agent_throttle_high_heat():
    factor, msg = local_agent_throttle(80.0, THERMAL_WARN_C + 2)
    assert factor < 1.0
    assert msg == AGENT_FUN_MESSAGE


def test_agent_nominal_no_message():
    factor, msg = local_agent_throttle(80.0, 30.0)
    assert factor == 1.0
    assert msg is None


def test_merge_throttle_takes_stricter():
    assert merge_throttle(0.8, 0.4) == 0.4
    assert merge_throttle(0.3, None) == 0.3
    assert merge_throttle(0.8, 1.0) == 0.8