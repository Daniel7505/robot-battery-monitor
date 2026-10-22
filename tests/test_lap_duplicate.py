"""Duplicate MISSION_SUMMARY detection across lap folders."""

from __future__ import annotations

import json

from optimizer.lap_folder import create_lap_folder, finalize_lap_folder, write_lap_meta
from optimizer.lap_launch import LapLaunch


def test_duplicate_summary_flagged_in_report(tmp_path):
    launch_a = LapLaunch.new("run", "lap-a")
    launch_b = LapLaunch.new("run", "lap-b")
    line = (
        "MISSION_SUMMARY build=light duration_s=600.0 dist_m=200.0 turns=13 "
        "wh_measured_drive_elec=3.5 err_pct=-2.0 "
        f"opt_run_id={launch_a.run_id} opt_trial_id={launch_a.trial_id} "
        f"lap_start_wallclock={launch_a.lap_start_wallclock}"
    )
    lap_a = create_lap_folder(tmp_path, "lap-a", when=launch_a.launched_utc)
    write_lap_meta(lap_a, launch_a)
    (lap_a / "summary.json").write_text(json.dumps({"lines": [line]}), encoding="utf-8")
    finalize_lap_folder(lap_a, label="lap-a", repo=tmp_path)

    line_b = line.replace(launch_a.lap_start_wallclock, launch_b.lap_start_wallclock).replace(
        launch_a.trial_id, launch_b.trial_id
    )
    lap_b = create_lap_folder(tmp_path, "lap-b", when=launch_b.launched_utc)
    write_lap_meta(lap_b, launch_b)
    (lap_b / "summary.json").write_text(json.dumps({"lines": [line_b]}), encoding="utf-8")
    report_path = finalize_lap_folder(lap_b, label="lap-b", repo=tmp_path)
    body = report_path.read_text(encoding="utf-8")
    assert "duplicates lap" in body
    assert lap_a.name in body
