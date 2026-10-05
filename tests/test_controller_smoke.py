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


def _run(tmp_path, mode, world=None, extra=None):
    env = dict(os.environ)
    env.pop("RBM_LANE_MODE", None)
    env.pop("RBM_TRACK", None)
    env.pop("RBM_OBSTACLES", None)
    env.update(extra or {})
    if world is not None:
        env["FAKE_WEBOTS_WORLD"] = str(ROOT / "webots" / "worlds" / world)
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
        assert "new_frame,run_id," in log[0] and log[0].endswith(
            "yaw_deg,yaw_target_deg,junction_type,junction_m,openings,route_choice,blind_m,"
            "obstacle_state,obstacle_m,obstacle_band,obstacle_conf,obstacle_moving")
        assert "OBSTACLES off (RBM_OBSTACLES unset" in text, text
        row = dict(zip(log[0].split(","), log[1].split(",")))
        assert row["state"] in ("LANE", "CORNER_APPROACH", "PIVOT", "REACQUIRE")
        assert len(log) > 1 and ",rowfit," in log[1]
    else:
        assert "LANE MODE rowfit" not in text and "CAM POSE" not in text
        assert "NADIR STEER ON" in text
        assert not (tmp_path / "lane-vision.csv").exists()


@pytest.mark.parametrize(
    "world, expect",
    [
        ("butlerbot.wbt", "FINISH REFEREE track s: finish (16.5, 0)"),
        ("butlerbot_corner90.wbt", "FINISH REFEREE track corner90: finish (4, 4) heading +90deg"),
        (None, "FINISH REFEREE finish x>=16.5 m (default S)"),
    ],
)
def test_finish_referee_comes_from_the_loaded_world(tmp_path, world, expect):
    text = _run(tmp_path, None, world)
    assert expect in text, text
    assert "WARNING finish referee" not in text


def test_controller_obstacles_on_sees_box_and_slows(tmp_path):
    """RBM_OBSTACLES=1: stereo cams bound from the live (fake supervisor) pose, a box 1.0 m
    ahead in the fake stereo images -> confirmed -> SLOW_FOR_OBSTACLE, CSV columns filled."""
    text = _run(tmp_path, None, extra={"RBM_OBSTACLES": "1"})
    assert "ButlerBot controller stopped" in text, text
    assert "Controller step error" not in text, text
    assert "OBSTACLES on (RBM_OBSTACLES=1)" in text, text
    assert "CAM POSE stereo_left from supervisor: t=(0.1400, 0.0500, 0.7800)" in text, text
    assert "STEREO rig baseline 10.0 cm" in text, text
    assert "OBSTACLES ON — backend" in text and "corridor +-0.54 m" in text, text
    assert "OBSTACLE CLEAR -> SLOW_FOR_OBSTACLE" in text, text
    assert "WARNING obstacles" not in text, text
    log = (tmp_path / "lane-vision.csv").read_text().splitlines()
    rows = [dict(zip(log[0].split(","), r.split(","))) for r in log[1:]]
    slow = [r for r in rows if r["obstacle_state"] == "SLOW_FOR_OBSTACLE"]
    assert slow and slow[-1]["obstacle_band"] == "low", log[:3]
    assert abs(float(slow[-1]["obstacle_m"]) - 1.0) < 0.08


def test_obstacles_world_tag_turns_sensing_on(tmp_path):
    text = _run(tmp_path, None, "butlerbot_warehouse_obstacles.wbt")
    assert "OBSTACLES on (world tag '# OBSTACLES on' in butlerbot_warehouse_obstacles.wbt" in text, text
    text = _run(tmp_path, None, "butlerbot_warehouse_obstacles.wbt", extra={"RBM_OBSTACLES": "0"})
    assert "OBSTACLES off (RBM_OBSTACLES=0)" in text, text
