"""WebotsLapRunner applies trial params before launch (non-Windows stub)."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from optimizer.lap_launch import LapLaunch
from optimizer.runner import WebotsLapRunner


def test_run_lap_applies_params_before_popen(tmp_path):
    cfg_dir = tmp_path / "config"
    cfg_dir.mkdir()
    (cfg_dir / "config.yaml").write_text(
        """
bom_build: light
controller:
  coast_enabled: true
  coast_margin_m: 0.00
  cruise_m_s: 0.44
  straight_m_s: 0.50
  pivot_w_max_rad_s: 0.6
  k_steer: 2.0
""".strip()
        + "\n",
        encoding="utf-8",
    )
    runner = WebotsLapRunner(tmp_path)
    launch = LapLaunch.new("test", "t1")
    log_path = tmp_path / "launch_1.log"
    applied: list[str] = []

    def fake_apply(repo, params):
        applied.append(str(params.get("controller.cruise_m_s")))
        return "optimizer: applied trial config — cruise=0.50 m/s"

    with patch.object(runner, "cleanup", return_value=""):
        with patch("optimizer.runner.webots_lap_host", return_value=True):
            with patch("optimizer.lap_config.apply_trial_params", side_effect=fake_apply):
                with patch("optimizer.runner.subprocess.Popen") as popen:
                    proc = MagicMock()
                    proc.poll.return_value = 0
                    popen.return_value = proc
                    with patch("optimizer.runner.read_dashboard_log", return_value=("", "")):
                        with patch("optimizer.runner.classify_summary", return_value=("absent", "")):
                            outcome = runner.run_lap(
                                params={"controller.cruise_m_s": 0.50},
                                timeout_s=1.0,
                                log_path=log_path,
                                launch=launch,
                            )
    assert applied == ["0.5"]
    assert log_path.is_file()
    body = log_path.read_bytes().decode("utf-8", errors="replace")
    assert "applied trial config" in body
    assert "lap artifacts" in body
    laps = list((tmp_path / "logs" / "laps").glob("*"))
    assert len(laps) == 1
    assert (laps[0] / "report.md").is_file()
    env = popen.call_args.kwargs.get("env") or popen.call_args[1].get("env")
    assert env["RBM_CONTROLLER_LOG"].endswith("console.log")
    assert outcome.status in {"crash", "no_summary", "timeout"}
