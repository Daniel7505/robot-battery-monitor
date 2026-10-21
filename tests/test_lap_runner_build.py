"""Optimizer lap runner resolves bom_build from config when params omit it."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from optimizer.lap_launch import LapLaunch
from optimizer.runner import WebotsLapRunner


def test_run_lap_uses_config_bom_build_when_params_omit_it(tmp_path):
    cfg = tmp_path / "config" / "config.yaml"
    cfg.parent.mkdir(parents=True)
    cfg.write_text(
        "bom_build: heavy\ncontroller:\n  cruise_m_s: 0.44\n",
        encoding="utf-8",
    )
    runner = WebotsLapRunner(tmp_path)
    launch = LapLaunch.new("run", "t1")
    env_captured: dict = {}

    def capture_popen(*args, **kwargs):
        env_captured.update(kwargs.get("env") or {})
        proc = MagicMock()
        proc.poll.return_value = 1
        return proc

    with patch.object(runner, "cleanup", return_value=""):
        with patch("optimizer.runner.webots_lap_host", return_value=True):
            with patch("optimizer.runner._wait_for_webots", return_value=False):
                with patch("optimizer.lap_config.apply_trial_params", return_value="ok"):
                    with patch("optimizer.runner.subprocess.Popen", side_effect=capture_popen):
                        runner.run_lap(params={"controller.cruise_m_s": 0.44}, timeout_s=1.0, launch=launch)
    assert env_captured.get("RBM_OPT_BOM_BUILD") == "heavy"
    laps = list((tmp_path / "logs" / "laps").glob("*"))
    assert len(laps) == 1
    import json

    meta = json.loads((laps[0] / "lap_meta.json").read_text(encoding="utf-8"))
    assert meta["bom_build"] == "heavy"
