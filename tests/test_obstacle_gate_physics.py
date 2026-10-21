"""Hand-worked stop / slow zone distances (mass-aware a_brake, t=0.32 s)."""

from __future__ import annotations

import pytest

from src.obstacle_gate import SLOW_EXTRA_M, STANDOFF_M, stop_distance_m

LIGHT_A = 0.8 * 9.81  # 7.848 m/s²
HEAVY_A = 2.0 * 0.981 / 0.04 / 56.142  # 0.873677… m/s²
T_LAT = 0.32


def _slow(v: float, a: float) -> float:
    return stop_distance_m(v, a, T_LAT, STANDOFF_M) + SLOW_EXTRA_M


@pytest.mark.parametrize(
    "a_brake,v_m_s,d_stop_expect,slow_expect",
    [
        (LIGHT_A, 0.44, 0.5031, 1.5031),
        (LIGHT_A, 1.13, 0.7930, 1.7930),
        (LIGHT_A, 2.48, 1.5354, 2.5354),
        (HEAVY_A, 0.44, 0.6016, 1.6016),
        (HEAVY_A, 1.13, 1.4421, 2.4421),
        (HEAVY_A, 2.48, 4.6634, 5.6634),
    ],
)
def test_stop_and_slow_zone_hand_worked(a_brake, v_m_s, d_stop_expect, slow_expect):
    d_stop = stop_distance_m(v_m_s, a_brake, T_LAT, STANDOFF_M)
    assert d_stop == pytest.approx(d_stop_expect, abs=0.0005)
    assert _slow(v_m_s, a_brake) == pytest.approx(slow_expect, abs=0.0005)


def test_obstacle_gate_uses_dynamic_slow_distance():
    from src.obstacle_gate import ObstacleGate

    a = LIGHT_A
    g = ObstacleGate(decel_m_s2=a, v_cruise_m_s=2.48, latency_s=T_LAT)
    assert g.slow_distance(0.44) == pytest.approx(_slow(0.44, a), abs=1e-6)
    assert g.slow_distance(2.48) == pytest.approx(_slow(2.48, a), abs=1e-6)
    assert g.slow_distance(2.48) > g.slow_distance(0.44)


def test_v_max_slow_zone_fits_stereo_range_light_and_heavy():
    from src.physics_speed_limit import max_speed_for_obstacle_slow_zone_m

    det = 4.0
    v_light = max_speed_for_obstacle_slow_zone_m(
        det, a_brake_m_s2=LIGHT_A, reaction_latency_s=T_LAT, safety_margin=0.8
    )
    v_heavy = max_speed_for_obstacle_slow_zone_m(
        det, a_brake_m_s2=HEAVY_A, reaction_latency_s=T_LAT, safety_margin=0.8
    )
    assert _slow(v_light / 0.8, LIGHT_A) <= det + 0.01
    assert _slow(v_heavy / 0.8, HEAVY_A) <= det + 0.01
    assert v_light / 0.8 >= 2.48 - 0.02
    assert v_heavy / 0.8 >= 1.13 - 0.02
