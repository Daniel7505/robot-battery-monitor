"""Run the real Webots controller loop against a fake ``controller`` module.

Catches wiring errors in butlerbot_controller.py for both lane modes
(no physics: GPS/IMU are frozen, so only start-up + first frames run).
"""
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
CTRL = ROOT / "webots" / "controllers" / "butlerbot_controller"


def _run(tmp_path, mode):
    env = dict(os.environ)
    env.update(
        PYTHONPATH=str(ROOT / "tests" / "fake_webots"),
        RBM_LANE_MODE=mode,
        RBM_LOG_DIR=str(tmp_path),
        TWIN_DASHBOARD_URL="http://127.0.0.1:9",
        FAKE_WEBOTS_STEPS="200",
    )
    out = subprocess.run(
        [sys.executable, "butlerbot_controller.py"], cwd=CTRL, env=env,
        capture_output=True, text=True, timeout=120,
    )
    return out.stdout + out.stderr


@pytest.mark.parametrize("mode", ["gap", "rowfit"])
def test_controller_loop_runs(tmp_path, mode):
    text = _run(tmp_path, mode)
    assert "ButlerBot controller stopped" in text, text
    assert "Controller step error" not in text, text
    assert "fatal error" not in text, text
    if mode == "rowfit":
        assert "ROWFIT STEER ON" in text
        log = (tmp_path / "lane-vision.csv").read_text().splitlines()
        assert log[0].endswith("new_frame,run_id")
        assert len(log) > 1 and ",rowfit," in log[1]
    else:
        assert "NADIR STEER ON" in text
        assert not (tmp_path / "lane-vision.csv").exists()
