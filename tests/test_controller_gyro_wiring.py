"""Gyro must be returned from device init and used in the control loop (no NameError)."""

from __future__ import annotations

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
CTRL_SRC = ROOT / "webots" / "controllers" / "butlerbot_controller" / "butlerbot_controller.py"
CTRL_DIR = CTRL_SRC.parent


def test_gyro_returned_from_init_and_unpacked_in_run_loop():
    text = CTRL_SRC.read_text(encoding="utf-8")
    assert "cam_period, gyro" in text
    assert "cam_period, gyro = _init_devices" in text
    assert "if gyro is not None:" in text
    assert "gyro.getValues()" in text


def test_controller_smoke_enables_gyro_without_step_errors(tmp_path):
    """Fake Webots provides a Gyro device; the loop must not hit undefined ``gyro``."""
    import os
    import subprocess
    import sys

    env = dict(os.environ)
    for key in (
        "RBM_LANE_MODE",
        "RBM_TRACK",
        "RBM_OBSTACLES",
        "RBM_HUD",
        "RBM_SPEED_TRACE",
        "RBM_SPEED_TRACE_CONSOLE",
    ):
        env.pop(key, None)
    env.update(
        PYTHONPATH=str(ROOT / "tests" / "fake_webots"),
        RBM_LOG_DIR=str(tmp_path),
        TWIN_DASHBOARD_URL="http://127.0.0.1:9",
        FAKE_WEBOTS_STEPS="80",
        PYTHONIOENCODING="utf-8",
        PYTHONUTF8="1",
    )
    out = subprocess.run(
        [sys.executable, "butlerbot_controller.py"],
        cwd=CTRL_DIR,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=120,
    )
    text = out.stdout + out.stderr
    assert out.returncode == 0, text
    assert "ButlerBot controller stopped" in text, text
    assert "Controller step error" not in text, text
    assert "name 'gyro' is not defined" not in text, text
    assert "Gyro: device found and enabled" in text, text
