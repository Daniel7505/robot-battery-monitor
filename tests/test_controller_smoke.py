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
    env.pop("RBM_LANE_MODE", None)
    if mode is not None:
        env["RBM_LANE_MODE"] = mode
    env.update(
        PYTHONPATH=str(ROOT / "tests" / "fake_webots"),
        RBM_LOG_DIR=str(tmp_path),
        TWIN_DASHBOARD_URL="http://127.0.0.1:9",
        FAKE_WEBOTS_STEPS="200",
    )
    out = subprocess.run(
        [sys.executable, "butlerbot_controller.py"], cwd=CTRL, env=env,
        capture_output=True, text=True, timeout=120,
    )
    return out.stdout + out.stderr


@pytest.mark.parametrize("mode", ["gap", "rowfit", None])
def test_controller_loop_runs(tmp_path, mode):
    text = _run(tmp_path, mode)
    assert "ButlerBot controller stopped" in text, text
    assert "Controller step error" not in text, text
    assert "fatal error" not in text, text
    if mode in ("rowfit", None):  # unset = rowfit (default)
        assert "LANE MODE rowfit" in text
        assert "ROWFIT STEER ON" in text
        assert "CAM POSE nadir_left from supervisor: t=(0.0354, 0.4181, 0.7042)" in text, text
        assert "CAM POSE nadir_right from supervisor: t=(0.0354, -0.4181, 0.7042)" in text
        assert "pitch=53.9deg" in text
        assert "WARNING rowfit camera model" not in text, text
        log = (tmp_path / "lane-vision.csv").read_text().splitlines()
        assert log[0].endswith("new_frame,run_id")
        assert len(log) > 1 and ",rowfit," in log[1]
    else:
        assert "LANE MODE rowfit" not in text and "CAM POSE" not in text
        assert "NADIR STEER ON" in text
        assert not (tmp_path / "lane-vision.csv").exists()
