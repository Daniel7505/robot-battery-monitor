"""Per-lap folder layout and report card."""

from __future__ import annotations

import json
from pathlib import Path

from optimizer.lap_folder import (
    build_flags,
    build_report_md,
    create_lap_folder,
    finalize_lap_folder,
    lap_env,
    parse_pivot_turns,
    write_summary_from_dashboard_text,
)
from src.mission_summary_file import append_mission_lines, reset_mission_summary_file


def test_lap_env_points_at_folder(tmp_path):
    folder = tmp_path / "lap1"
    folder.mkdir()
    env = lap_env(folder)
    assert env["RBM_CONTROLLER_LOG"] == str(folder / "console.log")
    assert env["RBM_MISSION_SUMMARY_BASENAME"] == "summary.json"
    from optimizer.lap_folder import path_abspath

    assert env["RBM_MISSION_SUMMARY_DIR"] == path_abspath(folder)


def test_mission_summary_basename(tmp_path, monkeypatch):
    reset_mission_summary_file()
    folder = tmp_path / "lap"
    folder.mkdir()
    monkeypatch.setenv("RBM_MISSION_SUMMARY_DIR", str(folder))
    monkeypatch.setenv("RBM_MISSION_SUMMARY_BASENAME", "summary.json")
    path = append_mission_lines(["MISSION_SUMMARY build=light duration_s=10.0 dist_m=1.0 turns=1 err_pct=-1.0"])
    assert path == str(folder / "summary.json")
    payload = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    assert payload["lines"][0].startswith("MISSION_SUMMARY")


def test_parse_pivot_turns_and_report(tmp_path):
    console = """
CONTROLLER LIMITS cruise=0.44 m/s straight=0.50 m/s pivot_w_max=0.90 rad/s k_steer=2.00 v_max=0.55 m/s effective_cruise=0.44 m/s limit=physics
Gyro: device found and enabled
SPEED_TRACE t_sim=1.0 t_wall=1.0 rtf=1.0 v_body=0.430 v_odo=0.430 wL_cmd=0 wR_cmd=0 wL=0 wR=0
PIVOT start left yaw +0.0deg -> target 90.0deg (lane 0.0deg, remaining 0.010 m) entry_v=0.040 m/s pos=(1.000,2.000) coast_into_turn=True
PIVOT done yaw +90.0deg (target 90.0, err +0.0deg, heading_change +90.0deg, entry_v=0.040 m/s ct_max=0.120 m, 2.0 s, settled) — REACQUIRE
SPEED_TRACE t_sim=2.0 t_wall=2.0 rtf=1.0 v_body=0.650 v_odo=0.650 wL_cmd=0 wR_cmd=0 wL=0 wR=0
""".strip()
    turns = parse_pivot_turns(console)
    assert len(turns) == 1
    assert turns[0]["heading_change_deg"] == 90.0
    lap_dir = tmp_path / "20261021-120000_t1"
    lap_dir.mkdir()
    (lap_dir / "console.log").write_text(console, encoding="utf-8")
    summary_line = (
        "MISSION_SUMMARY build=light duration_s=600.0 dist_m=200.0 turns=1 "
        "wh_measured_drive_elec=3.5 err_pct=2.0 "
        "ctrl_cruise_m_s=0.44 ctrl_straight_m_s=0.50 ctrl_pivot_w_max_rad_s=0.90 ctrl_k_steer=2.00 "
        "ctrl_effective_cruise_m_s=0.44 gyro_found=1"
    )
    (lap_dir / "summary.json").write_text(
        json.dumps({"lines": [summary_line, "TELEMETRY_GAPS n=100 mean_s=0.032 max_s=0.500"]}),
        encoding="utf-8",
    )
    report_path = finalize_lap_folder(lap_dir, label="t1")
    body = report_path.read_text(encoding="utf-8")
    assert "## Per-turn pivots" in body
    assert "+90.0" in body
    assert "Peak v_body" in body
    assert "gyro" in body.lower()
    flags = build_flags(
        fields={"err_pct": "2.0", "turns": "1"},
        turns=turns,
        telemetry_gap_s=0.5,
        gyro="found",
    )
    assert flags == []


def test_flags_detect_bad_turn_and_energy():
    turns = [{"turn": 1, "heading_change_deg": 126.0, "entry_v_m_s": 0.27, "max_lateral_m": 0.29, "pivot_start": "(0,0)"}]
    flags = build_flags(
        fields={"err_pct": "11.0", "turns": "13"},
        turns=turns,
        telemetry_gap_s=1.5,
        gyro="missing",
    )
    assert any("126" in f for f in flags)
    assert any("11.0" in f for f in flags)
    assert any("telemetry gap" in f for f in flags)
    assert any("gyro" in f for f in flags)


def test_write_summary_from_dashboard_text(tmp_path):
    lap_dir = tmp_path / "lap"
    lap_dir.mkdir()
    text = "2026-10-21 12:00:00,000 | INFO | robot_monitor | MISSION_SUMMARY build=light duration_s=1.0 dist_m=2.0 turns=1 err_pct=0.0"
    path = write_summary_from_dashboard_text(lap_dir, text)
    assert path is not None
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert "MISSION_SUMMARY" in payload["lines"][0]


def test_create_lap_folder_under_repo(tmp_path):
    folder = create_lap_folder(tmp_path, "search-3")
    assert folder.parent == tmp_path / "logs" / "laps"
    assert folder.name.endswith("_search-3")
