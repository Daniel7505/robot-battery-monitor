"""Mission summary stamps and lap-scoped summary pick."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from optimizer.lap_launch import LapLaunch
from optimizer.runner import _mission_summary_pick, classify_summary, read_dashboard_log


def test_lap_folder_summary_requires_matching_stamps(tmp_path, monkeypatch):
    launch = LapLaunch.new("run-win", "search-3")
    lap_dir = tmp_path / "lap"
    lap_dir.mkdir()
    line = (
        "MISSION_SUMMARY build=light duration_s=600.0 dist_m=200.91 turns=13 "
        "err_pct=-2.0 wh_measured_drive_elec=3.5 wh_model_drive=3.4 "
        f"opt_run_id={launch.run_id} opt_trial_id={launch.trial_id} "
        f"lap_start_wallclock={launch.lap_start_wallclock}"
    )
    (lap_dir / "summary.json").write_text(
        json.dumps(
            {
                "source": "controller",
                "opt_run_id": launch.run_id,
                "opt_trial_id": launch.trial_id,
                "lap_start_wallclock": launch.lap_start_wallclock,
                "lines": [line],
            }
        ),
        encoding="utf-8",
    )
    text, _mtime = _mission_summary_pick(None, launch=launch, lap_dir=lap_dir)
    kind, detail = classify_summary(text, launch)
    assert kind == "fresh"
    assert detail == ""


def test_stale_repo_summary_not_merged_during_lap(tmp_path, monkeypatch):
    from optimizer import runner

    launch = LapLaunch.new("run-a", "t1")
    lap_dir = tmp_path / "lap"
    lap_dir.mkdir()
    stale = (
        "MISSION_SUMMARY build=light duration_s=600.0 dist_m=1.0 turns=1 err_pct=-1.0 "
        "opt_run_id=old opt_trial_id=old lap_start_wallclock=2020-01-01T00:00:00Z"
    )
    (tmp_path / "mission_summary_stale.json").write_text(
        json.dumps({"lines": [stale]}),
        encoding="utf-8",
    )
    monkeypatch.setattr(runner.subprocess, "run", lambda *a, **k: type("P", (), {"stdout": stale, "stderr": "", "returncode": 0})())
    text, _note = read_dashboard_log(launch=launch, lap_dir=lap_dir)
    assert "MISSION_SUMMARY" not in text


def test_webots_must_start_or_lap_crashes(tmp_path):
    from unittest.mock import MagicMock, patch

    from optimizer.runner import WebotsLapRunner

    runner = WebotsLapRunner(tmp_path)
    launch = LapLaunch.new("run", "t1")
    with patch.object(runner, "cleanup", return_value=""):
        with patch("optimizer.runner.webots_lap_host", return_value=True):
            with patch("optimizer.runner._wait_for_webots", return_value=False):
                with patch("optimizer.lap_config.apply_trial_params", return_value="ok"):
                    with patch("optimizer.runner.subprocess.Popen") as popen:
                        proc = MagicMock()
                        proc.poll.return_value = 1
                        popen.return_value = proc
                        outcome = runner.run_lap(
                            params={},
                            timeout_s=5.0,
                            launch=launch,
                        )
    assert outcome.status == "crash"
    assert "Webots did not start" in outcome.note
