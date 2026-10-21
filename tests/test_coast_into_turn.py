"""Coast-into-turn physics and rowfit approach behaviour."""

from __future__ import annotations

import pytest

from src.physics_speed_limit import (
    coast_decel_m_s2,
    coast_distance_m,
    coast_into_turn_params_for_robot,
)
from src.rowfit_control import RowfitController, STATE_APPROACH

LIGHT_CRR = 0.015
LIGHT_MASS = 6.142
V_CRUISE = 0.44
V_PIVOT = 0.04
LD_MAX = 1.600257176976473


def test_coast_distance_hand_check_light_build():
    """d = (v² − v_pivot²) / (2·a_coast) with a_coast ≈ C_rr·g."""
    a = coast_decel_m_s2(crr=LIGHT_CRR, mass_kg=LIGHT_MASS, v_m_s=V_CRUISE)
    assert a == pytest.approx(LIGHT_CRR * 9.81, rel=1e-3)
    d = coast_distance_m(V_CRUISE, v_target_m_s=V_PIVOT, a_coast_m_s2=a)
    # (0.44² − 0.04²) / (2·0.14715) ≈ 0.653 m
    assert d == pytest.approx(0.653, abs=0.02)


def test_coast_distance_higher_cruise_example():
    """Worked example at cruise 1.0 m/s (physics cap trials)."""
    v = 1.0
    a = coast_decel_m_s2(crr=LIGHT_CRR, mass_kg=LIGHT_MASS, v_m_s=v)
    d = coast_distance_m(v, v_target_m_s=V_PIVOT, a_coast_m_s2=a)
    assert d == pytest.approx(3.37, abs=0.05)


def test_coast_no_op_when_cruise_below_pivot_entry():
    """At cruise 0.44 m/s and pivot entry 0.04 m/s coasting applies; disabled is no-op."""
    params = coast_into_turn_params_for_robot(
        {"controller": {"coast_enabled": False}},
        bom_build="light",
    )
    assert params is not None
    assert params.enabled is False


def test_approach_coast_arms_at_cruise_above_pivot_entry():
    """Cruise 0.44 m/s and pivot entry 0.04 m/s must arm coast before the pivot gate."""
    from src.physics_speed_limit import CoastIntoTurnParams, TurnBrakeParams

    tb = TurnBrakeParams(
        a_brake_m_s2=7.848,
        reaction_latency_s=0.32,
        safety_margin=0.8,
        approach_target_m_s=0.04,
    )
    coast = CoastIntoTurnParams(
        enabled=True,
        margin_m=0.0,
        crr=LIGHT_CRR,
        drag_linear_n_s_per_m=0.0,
        total_mass_kg=LIGHT_MASS,
        v_pivot_m_s=V_PIVOT,
        a_brake_m_s2=7.848,
    )
    ctl = RowfitController(
        v_cruise=V_CRUISE,
        turn_brake=tb,
        coast_into_turn=coast,
        ld_max=LD_MAX,
    )
    ctl.state = STATE_APPROACH
    ctl.remaining_m = 0.50
    ctl.gov.v = V_CRUISE
    ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
    assert ctl._coast_active is True


def test_approach_coast_no_op_when_cruise_leq_pivot_speed_cap():
    """When effective cruise is not above pivot entry, coast never arms."""
    from src.physics_speed_limit import TurnBrakeParams

    tb = TurnBrakeParams(
        a_brake_m_s2=7.848,
        reaction_latency_s=0.32,
        safety_margin=0.8,
        approach_target_m_s=0.50,
    )
    from src.physics_speed_limit import CoastIntoTurnParams

    coast = CoastIntoTurnParams(
        enabled=True,
        margin_m=0.0,
        crr=LIGHT_CRR,
        drag_linear_n_s_per_m=0.0,
        total_mass_kg=LIGHT_MASS,
        v_pivot_m_s=0.50,
        a_brake_m_s2=7.848,
    )
    ctl = RowfitController(
        v_cruise=0.44,
        turn_brake=tb,
        coast_into_turn=coast,
        ld_max=LD_MAX,
    )
    ctl.state = STATE_APPROACH
    ctl.remaining_m = 2.0
    ctl.gov.v = 0.44
    ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
    assert ctl._coast_active is False


def test_gyro_startup_log_line_in_controller_source():
    text = (
        __import__("pathlib").Path("webots/controllers/butlerbot_controller/butlerbot_controller.py")
        .read_text(encoding="utf-8")
    )
    assert "Gyro: device found and enabled" in text
    assert "Gyro: device not found" in text
