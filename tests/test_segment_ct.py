"""Per-straight cross-track, closed at each pivot and at mission end."""

from __future__ import annotations

import math
from pathlib import Path

from src.rowfit_control import STATE_PIVOT, RowfitController

ROOT = Path(__file__).resolve().parents[1]


def test_segment_ct_max_is_the_straight_before_the_pivot():
    ctl = RowfitController()
    ctl.corner_dir = 1
    ctl.turn_src = "corner"
    ctl.note_cross_track(0.04)
    ctl.step(None, new_frame=False, dt=0.008, pose=(0.0, 0.0), yaw=0.0)
    ctl.note_cross_track(0.164)
    ctl.note_cross_track(-0.02)
    ctl.step(None, new_frame=False, dt=0.008, pose=(10.0, 0.0), yaw=0.0)
    ctl._start_pivot(0.0, entry_v_m_s=0.04)
    assert ctl.state == STATE_PIVOT
    ctl.note_cross_track(0.90)  # pivot lateral stays off the straight
    segs = [row for row in ctl._turn_records if row.startswith("SEGMENT_RECORD")]
    assert len(segs) == 1
    assert "ct_max_m=0.164" in segs[0]
    assert "after_turn=0" in segs[0]
    assert "start_pos=(0.000,0.000)" in segs[0]
    assert "end_pos=(10.000,0.000)" in segs[0]
    ctl.drain_events()
    ctl._end_pivot(math.pi / 2, "settled")
    ctl.drain_events()
    ctl.note_cross_track(0.05)
    ctl.step(None, new_frame=False, dt=0.008, pose=(12.0, 1.0), yaw=math.pi / 2)
    lines = ctl.finish_turn_log()
    assert any(
        line.startswith("TURN_LOG") and "segments=2" in line and "pivots=1" in line
        for line in lines
    )
    final = next(line for line in lines if line.startswith("SEGMENT_RECORD"))
    assert "ct_max_m=0.050" in final
    assert "after_turn=1" in final
    assert "0.900" not in final


def test_turn_log_is_printed_on_mission_complete():
    text = (
        ROOT / "webots/controllers/butlerbot_controller/butlerbot_controller.py"
    ).read_text(encoding="utf-8")
    complete = text.index("if mission_complete and not turn_log_sent:")
    stopped = text.index('print("ButlerBot controller stopped')
    assert complete < stopped
    helper = text.index("def _emit_turn_log")
    assert "finish_turn_log" in text[helper:complete]
    assert "flush=True" in text[helper:complete]
