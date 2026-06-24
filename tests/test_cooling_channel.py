from src.cooling_channel import estimate_cooling_draw_w
from src.mission_context import task_phase_alignment


def test_cooling_rises_with_thermal_and_manipulate():
    idle = estimate_cooling_draw_w(24.0, "standby")
    hot = estimate_cooling_draw_w(58.0, "manipulate")
    assert hot > idle
    assert hot >= 7.0


def test_task_phase_alignment_mismatch_note():
    out = task_phase_alignment("drive_transit", "high_load")
    assert out["aligned"] is False
    assert "moving" in out["note"]
    assert "high_load" in out["note"]


def test_task_phase_alignment_match():
    out = task_phase_alignment("manipulate", "high_load")
    assert out["aligned"] is True
    assert out["note"] == ""