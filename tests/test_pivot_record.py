"""PIVOT_RECORD lines are emitted for every completed pivot."""

from __future__ import annotations

import math

from src.rowfit_control import RowfitController, STATE_PIVOT


def test_turn_record_emitted_at_pivot_start():
    ctl = RowfitController()
    ctl.pivots = 0
    ctl.corner_dir = 1
    ctl.remaining_m = 0.5
    ctl.turn_src = "corner"
    ctl.gov.v = 0.2
    ctl._start_pivot(0.0, entry_v_m_s=0.2)
    assert any(m.startswith("TURN_RECORD turn=1 kind=pivot") for m in ctl.events)


def test_pivot_record_emitted_on_every_end_pivot():
    ctl = RowfitController()
    ctl.pivots = 0
    ctl.corner_dir = 1
    ctl.yaw_target = math.radians(90.0)
    ctl.pivot_start_yaw = 0.0
    ctl._pivot_entry_v_m_s = 0.04
    ctl._pivot_lateral_max_m = 0.12
    ctl._pivot_start_pos = "(1.0,2.0)"
    ctl.pivot_t = 1.0
    for turn in range(1, 5):
        ctl._end_pivot(math.radians(90.0), "test")
        records = [m for m in ctl.events if m.startswith("PIVOT_RECORD")]
        assert len(records) == turn
        assert f"turn={turn}" in records[-1]
        ctl.state = STATE_PIVOT
        ctl.pivot_start_yaw = 0.0
        ctl.yaw_target = math.radians(90.0)
