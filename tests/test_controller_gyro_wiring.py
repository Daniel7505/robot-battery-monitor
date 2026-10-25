"""Gyro must be returned from device init and used in the control loop (no NameError)."""

from __future__ import annotations

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
CTRL_SRC = ROOT / "webots" / "controllers" / "butlerbot_controller" / "butlerbot_controller.py"
CTRL_DIR = CTRL_SRC.parent


def test_gyro_returned_from_init_and_unpacked_in_run_loop():
    """Init still returns gyro; the sense step still reads it.

    Device setup lives in controller_devices.py and the per-tick read
    lives in controller_sensing.py. Both have to keep the gyro name.
    """
    devices = (CTRL_DIR / "controller_devices.py").read_text(encoding="utf-8")
    sensing = (CTRL_DIR / "controller_sensing.py").read_text(encoding="utf-8")
    assert "cam_period, gyro" in devices
    assert "cam_period, gyro = _init_devices" in devices
    assert "if gyro is not None:" in sensing
    assert "gyro.getValues()" in sensing


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
