"""Unit tests for physics-based forward speed cap."""

from __future__ import annotations

import math

import pytest

from src.controller_limits import limits_from_mapping, load_controller_limits
from src.physics_speed_limit import (
    DEFAULT_BRAKE_MU,
    combine_brake_decel_m_s2,
    compute_physics_speed_limit,
    effective_cruise_m_s,
    friction_limited_decel_m_s2,
    max_speed_from_stop_distance_m,
    motor_limited_decel_m_s2,
    physics_speed_limit_for_active_robot,
)
from src.twin_energy import GRAVITY_M_S2


def test_friction_decel_hand_value():
    # μ=0.8, f_w=1 → 0.8 * 9.81
    assert friction_limited_decel_m_s2(
        brake_mu=0.8, driven_weight_fraction=1.0
    ) == pytest.approx(0.8 * GRAVITY_M_S2)


def test_motor_decel_light_mass_hand_value():
    # 2 * 0.981 / 0.04 / 6.142
    a = motor_limited_decel_m_s2(
        total_mass_kg=6.142,
        wheel_radius_m=0.04,
        brake_torque_nm_per_motor=0.981,
        motor_qty=2.0,
    )
    assert a == pytest.approx(2 * 0.981 / 0.04 / 6.142)


def test_quadratic_stop_speed_matches_substitution():
    d = 1.0
    a = 7.848
    t = 0.32
    v = max_speed_from_stop_distance_m(
        d_available_m=d, a_brake_m_s2=a, reaction_latency_s=t, safety_margin=1.0
    )
    assert d == pytest.approx(v * t + v * v / (2 * a), rel=1e-9)


def test_safety_margin_scales_v_max():
    base = max_speed_from_stop_distance_m(
        d_available_m=1.0, a_brake_m_s2=2.0, reaction_latency_s=0.32, safety_margin=1.0
    )
    scaled = max_speed_from_stop_distance_m(
        d_available_m=1.0, a_brake_m_s2=2.0, reaction_latency_s=0.32, safety_margin=0.8
    )
    assert scaled == pytest.approx(0.8 * base)


def test_more_mass_lowers_torque_limited_v_max():
    common = dict(
        wheel_radius_m=0.04,
        brake_torque_nm_per_motor=0.981,
        d_available_m=0.35,
        reaction_latency_s=0.32,
        safety_margin=0.8,
        brake_mu=0.8,
        driven_weight_fraction=1.0,
    )
    light = compute_physics_speed_limit(total_mass_kg=6.142, **common)
    heavy = compute_physics_speed_limit(total_mass_kg=56.142, **common)
    assert heavy.v_max_m_s <= light.v_max_m_s + 1e-9
    assert heavy.torque_limited
    assert light.friction_limited


def test_lower_mu_lowers_friction_limited_v_max():
    mass = 6.142
    low_mu = compute_physics_speed_limit(
        total_mass_kg=mass,
        wheel_radius_m=0.04,
        brake_torque_nm_per_motor=0.981,
        brake_mu=0.4,
        d_available_m=1.0,
    )
    high_mu = compute_physics_speed_limit(
        total_mass_kg=mass,
        wheel_radius_m=0.04,
        brake_torque_nm_per_motor=0.981,
        brake_mu=0.8,
        d_available_m=1.0,
    )
    assert low_mu.v_max_m_s <= high_mu.v_max_m_s + 1e-9


def test_effective_cruise_never_exceeds_config_cruise():
    cruise = 0.44
    for v_max in (0.2, 0.44, 0.9, 2.0):
        eff = effective_cruise_m_s(cruise, v_max)
        assert eff <= cruise + 1e-9
        assert eff == pytest.approx(min(cruise, v_max))


def test_limits_from_mapping_effective_cruise():
    data = {
        "bom_build": "light",
        "controller": {
            "cruise_m_s": 0.44,
            "straight_m_s": 0.50,
            "pivot_w_max_rad_s": 0.9,
            "k_steer": 2.0,
            "speed_limit": {
                "stop_available_m": 0.35,
                "reaction_latency_s": 0.32,
                "safety_margin": 0.8,
                "brake_mu": 0.8,
            },
        },
    }
    lim = limits_from_mapping(data)
    assert lim.v_max_m_s is not None
    assert lim.effective_cruise_m_s is not None
    assert lim.effective_cruise_m_s <= lim.cruise_m_s + 1e-9
    assert lim.effective_straight_m_s is not None
    assert lim.effective_straight_m_s <= lim.straight_m_s + 1e-9


HEAVY_MASS_KG = 56.142
HEAVY_A_BRAKE = 2.0 * 0.981 / 0.04 / HEAVY_MASS_KG  # 0.873677… m/s² (torque-limited)
HEAVY_T_S = 0.32
HEAVY_MARGIN = 0.8


def _v_before_margin(a: float, t: float, d: float) -> float:
    return -a * t + math.sqrt(a * a * t * t + 2.0 * a * d)


def test_hand_worked_heavy_payload_exact():
    """Hand-checked values in docs/PHYSICS_SPEED_LIMIT.md (m=56.142 kg)."""
    assert HEAVY_A_BRAKE == pytest.approx(0.874, rel=1e-3)
    for d, v_max_expect in (
        (1.0, 0.857),
        (0.35, 0.441),
        (0.33, 0.424),
    ):
        v_raw = _v_before_margin(HEAVY_A_BRAKE, HEAVY_T_S, d)
        v_max = HEAVY_MARGIN * v_raw
        r = compute_physics_speed_limit(
            total_mass_kg=HEAVY_MASS_KG,
            wheel_radius_m=0.04,
            brake_torque_nm_per_motor=0.981,
            brake_mu=0.8,
            d_available_m=d,
            reaction_latency_s=HEAVY_T_S,
            safety_margin=HEAVY_MARGIN,
        )
        assert r.a_brake_m_s2 == pytest.approx(HEAVY_A_BRAKE, rel=1e-6)
        assert v_max == pytest.approx(v_max_expect, abs=0.002)
        assert r.v_max_m_s == pytest.approx(v_max, rel=1e-9)
    assert effective_cruise_m_s(0.44, HEAVY_MARGIN * _v_before_margin(HEAVY_A_BRAKE, HEAVY_T_S, 1.0)) == pytest.approx(0.44)
    assert effective_cruise_m_s(0.44, HEAVY_MARGIN * _v_before_margin(HEAVY_A_BRAKE, HEAVY_T_S, 0.35)) == pytest.approx(0.44)
    capped = effective_cruise_m_s(0.44, HEAVY_MARGIN * _v_before_margin(HEAVY_A_BRAKE, HEAVY_T_S, 0.33))
    assert capped < 0.44 - 1e-3
    assert capped == pytest.approx(0.424, abs=0.002)


def test_hand_worked_light_body_v_max():
    """Numbers in docs/PHYSICS_SPEED_LIMIT.md (light, d=1 m, margin 0.8)."""
    r = compute_physics_speed_limit(
        total_mass_kg=6.142,
        wheel_radius_m=0.04,
        brake_torque_nm_per_motor=0.981,
        brake_mu=0.8,
        d_available_m=1.0,
        reaction_latency_s=0.32,
        safety_margin=0.8,
    )
    assert r.a_friction_m_s2 == pytest.approx(0.8 * GRAVITY_M_S2, rel=1e-3)
    assert r.a_motor_m_s2 == pytest.approx(49.05 / 6.142, rel=1e-2)
    assert r.a_brake_m_s2 == pytest.approx(r.a_friction_m_s2, rel=1e-3)
    v_raw = -r.a_brake_m_s2 * 0.32 + math.sqrt(
        r.a_brake_m_s2 ** 2 * 0.32 ** 2 + 2 * r.a_brake_m_s2 * 1.0
    )
    assert r.v_max_m_s == pytest.approx(0.8 * v_raw, rel=1e-3)
    assert r.v_max_m_s == pytest.approx(1.742, abs=0.005)


def test_units_sanity_si():
    r = compute_physics_speed_limit(
        total_mass_kg=10.0,
        wheel_radius_m=0.04,
        brake_torque_nm_per_motor=0.981,
        brake_mu=DEFAULT_BRAKE_MU,
        d_available_m=1.0,
    )
    assert r.a_brake_m_s2 > 0
    assert r.v_max_m_s > 0
    assert r.wheel_radius_m == 0.04


def test_physics_for_bom_light_build():
    r = physics_speed_limit_for_active_robot(
        {
            "controller": {
                "speed_limit": {"stop_available_m": 1.0, "safety_margin": 0.8}
            }
        },
        bom_build="light",
    )
    assert r.total_mass_kg == pytest.approx(6.142, rel=1e-3)
    assert r.wheel_radius_m == pytest.approx(0.04)


def test_repo_config_loads_speed_limit():
    lim = load_controller_limits()
    assert lim.v_max_m_s is not None
    assert lim.effective_cruise_m_s is not None
    assert lim.effective_cruise_m_s <= lim.cruise_m_s + 1e-9
