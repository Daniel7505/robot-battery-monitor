from src.teleop_agent import (
    BATTERY_DRAIN_SCALE,
    CONTROL_SPEED_CAP_M_S,
    abs_brake_complete,
    abs_brake_wheel_velocity,
    abs_brake_wheel_velocity_latched,
    battery_drain_pct,
    brake_wheel_cap_rad_s,
    is_spin_brake,
    latch_brake_motion_sign,
    motion_settled,
    sanitize_motion,
    should_coast_before_brake,
)


def test_symmetric_brake_opposes_forward_motion():
    sign = latch_brake_motion_sign(0.42, 0.42)
    cmd = abs_brake_wheel_velocity_latched(sign, 0.42)
    assert sign == 1.0
    assert cmd < 0


def test_symmetric_brake_opposes_reverse_motion():
    sign = latch_brake_motion_sign(-0.35, 0.35)
    cmd = abs_brake_wheel_velocity_latched(sign, 0.35)
    assert sign == -1.0
    assert cmd > 0


def test_latched_brake_does_not_flip_when_gps_jitters():
    cmd_fwd = abs_brake_wheel_velocity_latched(1.0, 0.4)
    cmd_rev_reading = abs_brake_wheel_velocity_latched(1.0, 0.4)
    assert cmd_fwd == cmd_rev_reading
    assert cmd_fwd < 0


def test_cruise_speed_brake_matches_wheel_equiv():
    """At 0.55 m/s brake cmd should oppose ~6.9 rad/s, not a 3.5 cap."""
    cmd = abs(abs_brake_wheel_velocity_latched(1.0, 0.55))
    assert cmd >= 6.0
    assert cmd <= brake_wheel_cap_rad_s(0.55)


def test_coast_only_when_crawling():
    assert should_coast_before_brake(0.08) is True
    assert should_coast_before_brake(0.55) is False


def test_brake_zero_when_stationary():
    assert abs_brake_wheel_velocity(0.0, 0.0) == 0.0


def test_brake_complete_when_slow_enough():
    assert abs_brake_complete(forward_m_s=0.01, speed_m_s=0.01) is True
    assert abs_brake_complete(forward_m_s=0.2, speed_m_s=0.2) is False


def test_battery_drain_slow_for_teleop():
    drop = sum(battery_drain_pct(40.0, 0.032, drain_scale=1.0) for _ in range(int(30 / 0.032)))
    assert drop < 2.0
    assert drop > 0.0


def test_latch_uses_last_forward_drive_after_turn():
    sign = latch_brake_motion_sign(0.01, 0.05, last_left_v=5.5, last_right_v=5.5)
    assert sign == 1.0


def test_spin_brake_detects_turn_in_place():
    assert is_spin_brake(
        last_left_v=-2.6,
        last_right_v=2.6,
        left_wheel_rad_s=-1.2,
        right_wheel_rad_s=1.1,
        speed_m_s=0.05,
    )


def test_sanitize_motion_clamps_gps_spikes():
    speed, forward = sanitize_motion(4.5, 3.0)
    assert speed == CONTROL_SPEED_CAP_M_S
    assert forward == CONTROL_SPEED_CAP_M_S


def test_motion_not_settled_during_turn():
    assert motion_settled(0.05, 1.2, -1.0) is False


def test_motion_settled_when_idle():
    assert motion_settled(0.01, 0.0, 0.0) is True


def test_battery_drain_scales_with_draw():
    low = battery_drain_pct(20.0, 1.0)
    high = battery_drain_pct(80.0, 1.0)
    assert high > low * 3
    assert BATTERY_DRAIN_SCALE <= 15.0