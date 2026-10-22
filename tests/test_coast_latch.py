"""Coast arms once per approach and does not stall the wheels."""

from __future__ import annotations

import math

import pytest

from src.physics_speed_limit import CoastIntoTurnParams, TurnBrakeParams
from src.rowfit_control import PIVOT_ARRIVE_M, RowfitController, STATE_APPROACH, STATE_LANE, WHEEL_RADIUS_M

LIGHT_CRR = 0.015
LIGHT_MASS = 6.142
V_CRUISE = 0.44
V_PIVOT = 0.04


def _coast_ctl(*, remaining: float, speed: float) -> RowfitController:
    coast = CoastIntoTurnParams(
        enabled=True,
        margin_m=0.0,
        crr=LIGHT_CRR,
        drag_linear_n_s_per_m=0.0,
        total_mass_kg=LIGHT_MASS,
        v_pivot_m_s=V_PIVOT,
        a_brake_m_s2=7.848,
    )
    tb = TurnBrakeParams(7.848, 0.32, 0.8, V_PIVOT)
    ctl = RowfitController(v_cruise=V_CRUISE, turn_brake=tb, coast_into_turn=coast, ld_max=1.6)
    ctl.state = STATE_APPROACH
    ctl.remaining_m = remaining
    ctl.gov.v = speed
    ctl.corner_dir = 1
    return ctl


def test_coast_latches_once_and_keeps_rolling():
    ctl = _coast_ctl(remaining=0.55, speed=V_CRUISE)
    starts = []
    min_hub = 99.0
    for _ in range(80):
        cmd = ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
        starts.extend(line for line in ctl.drain_events() if line.startswith("COAST start"))
        hub = 0.5 * (abs(cmd["left"]) + abs(cmd["right"]))
        min_hub = min(min_hub, hub)
    assert len(starts) == 1
    assert ctl._coast_latched is True
    assert ctl._coast_starts == 1
    assert ctl.gov.v < V_CRUISE - 0.02
    assert ctl.gov.v >= V_PIVOT - 1e-6
    # 0.04 m/s on the 0.08 m wheel is 0.5 rad/s. Zero would be a stall hold.
    assert min_hub > 0.4
    assert min_hub == pytest.approx(ctl.gov.v / WHEEL_RADIUS_M, rel=0.15)


def test_coast_does_not_rearm_at_pivot_entry_speed():
    ctl = _coast_ctl(remaining=0.20, speed=V_PIVOT)
    for _ in range(30):
        ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
    starts = [line for line in ctl.events if line.startswith("COAST start")]
    assert starts == []
    assert ctl._coast_latched is False


def test_coasted_approach_is_still_true_at_the_pivot():
    ctl = _coast_ctl(remaining=0.55, speed=V_CRUISE)
    ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
    ctl.drain_events()
    ctl.remaining_m = PIVOT_ARRIVE_M
    ctl.gov.v = V_PIVOT
    ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
    pivot = next(line for line in ctl.events if line.startswith("PIVOT start"))
    assert "coast_into_turn=True" in pivot
    assert any(line.startswith("PIVOT_RECORD") for line in ctl.events) or ctl.state == "PIVOT"


def test_zero_command_does_not_pull_the_pivot_forward():
    ctl = RowfitController()
    ctl.remaining_m = 0.30
    ctl._advance_remaining(0.30, 0.0, 0.05, None)
    assert ctl.remaining_m == pytest.approx(0.30)
    ctl._advance_remaining(0.30, 0.20, 0.10, None)
    assert ctl.remaining_m == pytest.approx(0.28)


def test_pivot_and_record_counts_match_including_timeout():
    ctl = RowfitController()
    ctl.corner_dir = 1
    ctl.turn_src = "corner"
    for _ in range(3):
        ctl.state = STATE_APPROACH
        ctl._start_pivot(0.0, entry_v_m_s=0.04)
        ctl._end_pivot(math.pi / 2, "test")
    pivot_rows = [row for row in ctl._turn_records if row.startswith("PIVOT_RECORD")]
    turn_rows = [row for row in ctl._turn_records if row.startswith("TURN_RECORD") and "kind=pivot" in row]
    assert len(pivot_rows) == len(turn_rows) == ctl.pivots == 3
    ctl._start_pivot(0.0, entry_v_m_s=0.04)
    ctl.pivot_t = 9.0
    ctl._pivot_tick(None, False, 0.008, 0.2)
    pivot_rows = [row for row in ctl._turn_records if row.startswith("PIVOT_RECORD")]
    turn_rows = [row for row in ctl._turn_records if row.startswith("TURN_RECORD") and "kind=pivot" in row]
    assert len(pivot_rows) == len(turn_rows) == ctl.pivots == 4


def test_summary_turns_follow_pivot_count_not_heading():
    from src.mission_energy_log import MissionEnergyLog, _stall_hold_step_wh
    from src.mission_summary_file import sample_from_controller_payload
    from src.twin_energy import stall_hold_electrical_wh
    from src.hardware_profile import build_profile_from_bom

    wh = stall_hold_electrical_wh(9.6, build_profile_from_bom("light"))
    # 472b901 logged ~1203 coast ticks (~9.6 s) of a 0 rad/s command.
    # 2 * 24 V * 3 A / 0.92 * 9.6 / 3600 ≈ 0.42 Wh, the gap vs the +1.5% lap.
    assert wh == pytest.approx(0.417, abs=0.03)
    assert _stall_hold_step_wh(
        {"left_wheel_rad_s": 0.0, "right_wheel_rad_s": 0.0}, 9.6, 0.0, "light"
    ) == pytest.approx(wh, rel=1e-6)
    assert _stall_hold_step_wh(
        {"left_wheel_rad_s": 0.5, "right_wheel_rad_s": 0.5}, 9.6, 0.04, "light"
    ) == 0.0

    sample = sample_from_controller_payload(
        {
            "locomotion": {"ctrl_pivot_count": 3, "ctrl_coast_enabled": 1},
            "sensors": {"left_wheel_rad_s": 0.0, "right_wheel_rad_s": 0.0},
            "pose": {},
            "robot": {},
        }
    )
    assert sample["ctrl_pivot_count"] == 3
    assert sample["left_wheel_rad_s"] == 0.0

    log = MissionEnergyLog()
    log._build = "light"
    log._ctrl_bom_build = "light"
    log._mission_dt = 30.0
    log._mission_dist = 10.0
    log._turns = 12
    log._ctrl_pivot_count = 3
    log._mission_wh_hold = wh
    line = log._summary_line()
    assert "turns=3" in line
    assert "heading_turns=12" in line
    assert "wh_model_hold=" in line


def test_lane_yaw_change_logs_a_curve_not_a_pivot():
    ctl = RowfitController()
    ctl.state = STATE_LANE
    ctl.step(None, new_frame=False, dt=0.008, yaw=0.0)
    ctl.step(None, new_frame=False, dt=0.008, yaw=math.radians(90.0))
    curves = [row for row in ctl._turn_records if row.startswith("CURVE_RECORD")]
    pivots = [row for row in ctl._turn_records if row.startswith("PIVOT_RECORD")]
    assert len(curves) == 1
    assert pivots == []
    assert ctl.pivots == 0
    assert "ct_max_m=" in curves[0]
