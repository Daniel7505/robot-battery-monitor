"""Turn-approach physics braking and perception-limited distance."""

from __future__ import annotations

import pytest

from src.physics_speed_limit import (
    TurnBrakeParams,
    braking_distance_m,
    effective_turn_distance_m,
    v_max_cruise_from_lookahead_m,
)
from src.rowfit_control import RowfitController, STATE_APPROACH

LIGHT_A = 0.8 * 9.81  # 7.848 m/s²
HEAVY_A = 2.0 * 0.981 / 0.04 / 56.142
T_LAT = 0.32
MARGIN = 0.8
V_CRUISE = 0.44
LD_MAX = 1.600257176976473  # nadir constants → lookahead_bounds_from_coverage


def test_hand_braking_start_distance_light_and_heavy():
    """From cruise 0.44 m/s to v_target=0: d >= v·t + v²/(2a) (before margin)."""
    d_light = braking_distance_m(
        V_CRUISE, a_brake_m_s2=LIGHT_A, reaction_latency_s=T_LAT, v_target_m_s=0.0
    )
    d_heavy = braking_distance_m(
        V_CRUISE, a_brake_m_s2=HEAVY_A, reaction_latency_s=T_LAT, v_target_m_s=0.0
    )
    assert d_light == pytest.approx(0.153, abs=0.002)
    assert d_heavy == pytest.approx(0.252, abs=0.002)


def test_v_max_cruise_at_nadir_lookahead():
    """v_max_cruise = margin·(−a·t + sqrt(a²t² + 2ad)) at d_lookahead ≈ 1.60 m."""
    v_light = v_max_cruise_from_lookahead_m(
        LD_MAX,
        a_brake_m_s2=LIGHT_A,
        reaction_latency_s=T_LAT,
        safety_margin=MARGIN,
        v_target_m_s=0.0,
    )
    v_heavy = v_max_cruise_from_lookahead_m(
        LD_MAX,
        a_brake_m_s2=HEAVY_A,
        reaction_latency_s=T_LAT,
        safety_margin=MARGIN,
        v_target_m_s=0.0,
    )
    assert v_light == pytest.approx(2.48, abs=0.02)
    assert v_heavy == pytest.approx(1.13, abs=0.02)


def test_perception_caps_path_distance():
    assert effective_turn_distance_m(5.0, perception_horizon_m=LD_MAX) == pytest.approx(LD_MAX)
    assert effective_turn_distance_m(0.8, perception_horizon_m=LD_MAX) == pytest.approx(0.8)


def _approach_target(rem_m: float, *, a_brake: float) -> float:
    tb = TurnBrakeParams(
        a_brake_m_s2=a_brake,
        reaction_latency_s=T_LAT,
        safety_margin=MARGIN,
        approach_target_m_s=0.04,
    )
    ctl = RowfitController(v_cruise=V_CRUISE, turn_brake=tb, ld_max=LD_MAX)
    ctl.state = STATE_APPROACH
    ctl.remaining_m = rem_m
    ctl.gov.v = V_CRUISE
    ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
    return float(ctl.target_speed)


@pytest.mark.parametrize("a_brake,d_req", [(LIGHT_A, 0.153), (HEAVY_A, 0.252)])
def test_approach_commands_slowdown_before_braking_distance(a_brake, d_req):
    """At remaining ≈ d_req + 2 cm, target speed must drop below cruise."""
    target = _approach_target(d_req + 0.02, a_brake=a_brake)
    assert target < V_CRUISE - 1e-3


def test_track_distance_beyond_lookahead_matches_lookahead_cap():
    """Path remaining 5 m but nadir ld_max 1.6 m → same cap as rem=1.6 m."""
    near = _approach_target(1.6, a_brake=HEAVY_A)
    far = _approach_target(5.0, a_brake=HEAVY_A)
    assert near == pytest.approx(far, abs=1e-6)
