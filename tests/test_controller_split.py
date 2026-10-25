"""Helpers and call order after the butlerbot_controller split.

Webots still launches ``butlerbot_controller.py`` from its own folder.
The step bodies live in sibling modules with absolute imports.
"""

from __future__ import annotations

import importlib
import math
import os
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CTRL = ROOT / "webots" / "controllers" / "butlerbot_controller"
FAKE = ROOT / "tests" / "fake_webots"

_STEP_ORDER = (
    "prepare_run",
    "robot.step",
    "begin_tick",
    "poll_keyboard",
    "sense_motion",
    "handle_manual_and_api",
    "harvest_and_steer",
    "_paint_eye_huds",
    "command_motion",
    "integrate_energy",
    "log_drive_and_overlays",
    "accumulate_wheel_encoders",
    "finish_mission_if_parked",
    "publish_tick",
    "shutdown_run",
)


def _load(name: str):
    """Import a controller sibling the way Webots does: script dir on sys.path."""
    if "RBM_LOG_DIR" not in os.environ:
        os.environ["RBM_LOG_DIR"] = tempfile.mkdtemp(prefix="rbm-ctrl-split-")
    for path in (str(FAKE), str(CTRL)):
        if path in sys.path:
            sys.path.remove(path)
        sys.path.insert(0, path)
    return importlib.import_module(name)


def test_run_loop_calls_steps_in_the_old_order():
    text = (CTRL / "butlerbot_controller.py").read_text(encoding="utf-8")
    body = text[text.index("def _run_loop"): text.index("class _TeeStream")]
    positions = [body.index(name) for name in _STEP_ORDER]
    assert positions == sorted(positions)
    assert body.count("robot.step(") == 1


def test_siblings_use_absolute_imports_after_the_project_root():
    entry = (CTRL / "butlerbot_controller.py").read_text(encoding="utf-8")
    assert entry.index("if _PROJECT_ROOT not in sys.path:") < entry.index(
        "from controller_devices import"
    )
    for name in (
        "butlerbot_controller.py",
        "controller_devices.py",
        "controller_sensing.py",
        "controller_mission.py",
        "controller_energy.py",
    ):
        text = (CTRL / name).read_text(encoding="utf-8")
        assert "\nfrom ." not in text
        assert "\nimport ." not in text


def test_is_top_down_matches_the_warehouse_viewpoint():
    sensing = _load("controller_sensing")
    assert sensing._is_top_down((0.0, 1.0, 0.0, math.pi / 2))
    assert sensing._is_top_down((0.0, -1.0, 0.0, -math.pi / 2))
    assert sensing._is_top_down((1.0, 0.0, 0.0, math.pi / 2)) is False
    assert sensing._is_top_down("not-a-rotation") is False


def test_yaw_rate_wraps_across_pi():
    sensing = _load("controller_sensing")

    class _Imu:
        def __init__(self, yaw: float):
            self.yaw = yaw

        def getRollPitchYaw(self):
            return (0.0, 0.0, self.yaw)

    rate, yaw = sensing._yaw_rate(_Imu(-math.pi + 0.01), math.pi - 0.01, 0.1)
    assert yaw == pytest.approx(-math.pi + 0.01)
    assert rate == pytest.approx(0.2, abs=1e-9)
    assert sensing._yaw_rate(_Imu(0.4), 0.4, 1e-9) == (0.0, 0.4)
    assert sensing._yaw_rate(None, 0.0, 0.1) == (0.0, 0.0)


def test_pitch_balance_delta_holds_inside_the_deadband():
    sensing = _load("controller_sensing")
    cfg = {
        "enabled": True,
        "kp_pitch": 2.0,
        "kd_pitch_rate": 0.0,
        "max_correct_rad_s": 0.8,
        "deadband_rad": 0.025,
        "apply_while_abs": False,
    }

    class _Imu:
        def getRollPitchYaw(self):
            return (0.0, 0.1, 0.0)

    delta, pitch = sensing._pitch_balance_delta(_Imu(), 0.1, 0.032, cfg, abs_active=False)
    assert pitch == pytest.approx(0.1)
    assert delta == pytest.approx(-0.2)
    held, _ = sensing._pitch_balance_delta(_Imu(), 0.1, 0.032, cfg, abs_active=True)
    assert held == 0.0
    off = dict(cfg, enabled=False)
    assert sensing._pitch_balance_delta(_Imu(), 0.1, 0.032, off, abs_active=False)[0] == 0.0


def test_empty_lane_eyes_and_format_pose():
    sensing = _load("controller_sensing")
    eyes = sensing._empty_lane_eyes()
    assert eyes["error_source"] == "nadir"
    assert eyes["steer"] is None
    assert eyes["nadir_lateral_m"] is None
    assert "nadir_r_lateral_m" in eyes

    class _Gps:
        def getValues(self):
            return (1.234, -2.0, 0.1)

    assert sensing._format_pose(_Gps()) == "(1.23, -2.00)"


def test_phase_list_and_builtin_finish_line():
    mission = _load("controller_mission")
    assert len(mission.PHASES) == 5
    assert [row["name"] for row in mission.PHASES] == [
        "standby",
        "drive_transit",
        "patrol",
        "manipulate",
        "return_idle",
    ]
    finish = mission._XFinish()
    assert finish.crossed(16.5, 0.0) is True
    assert finish.crossed(16.49, 9.0) is False
    assert finish.describe() == "finish x>=16.5 m (built-in S)"


def test_throttle_helpers_with_no_teleop_agent():
    energy = _load("controller_energy")
    energy.bind_teleop(None)
    assert energy._local_throttle(80.0, 30.0) == (1.0, None)
    assert energy._update_thermal(30.0, 12.0, 0.008, 1.0) == 30.0
    assert energy._merge_throttle(0.5, None) == 0.5
    assert energy._merge_throttle(0.5, 0.2) == pytest.approx(0.2)
    assert energy._pack_capacity_wh() > 0
    power = energy._estimate_joint_power("left_wheel", 1.0, 0.1, speed_m_s=0.3)
    assert isinstance(power, float)
