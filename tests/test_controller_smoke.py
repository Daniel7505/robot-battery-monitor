"""Run the real Webots controller loop against a fake ``controller`` module.

Catches wiring errors in butlerbot_controller.py (no physics: GPS/IMU
are frozen, so only start-up + first frames run). A leftover gap mode
must still come up as row-fit.
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
    env.pop("RBM_HUD", None)
    env.pop("RBM_SPEED_TRACE", None)
    env.pop("RBM_SPEED_TRACE_CONSOLE", None)
    env.pop("RBM_MISSION_LAPS", None)
    env.pop("RBM_MISSION_MAX_M", None)
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
        PYTHONIOENCODING="utf-8",
        PYTHONUTF8="1",
    )
    out = subprocess.run(
        [sys.executable, "butlerbot_controller.py"], cwd=CTRL, env=env,
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120,
    )
    return out.stdout + out.stderr


@pytest.mark.parametrize("mode", ["gap", "rowfit", None])
def test_controller_loop_runs(tmp_path, mode):
    text = _run(tmp_path, mode)
    assert "ButlerBot controller stopped" in text, text
    assert "Controller step error" not in text, text
    assert "fatal error" not in text, text
    assert "HUD paint is off" in text, text
    assert "SPEED_TRACE off" in text, text
    assert "HUD hud_nadir" not in text
    if mode == "gap":
        assert "RBM_LANE_MODE=gap is retired" in text
    assert "NADIR STEER ON" not in text
    assert "LANE MODE rowfit" in text
    assert "ROWFIT STEER ON" in text
    assert "CAM POSE nadir_left from supervisor: t=(0.0354, 0.4181, 0.7042)" in text, text
    assert "CAM POSE nadir_right from supervisor: t=(0.0354, -0.4181, 0.7042)" in text
    assert "pitch=53.9deg" in text
    assert "WARNING rowfit camera model" not in text, text
    log = (tmp_path / "lane-vision.csv").read_text(encoding="utf-8", errors="replace").splitlines()
    assert "new_frame,run_id," in log[0]
    assert log[0].endswith(
        "obstacle_moving,lane_width_m,width_live,true_width_m,true_width_near_m,width_err_m,center_err_m"
    )
    assert "OBSTACLES off (RBM_OBSTACLES unset" in text, text
    row = dict(zip(log[0].split(","), log[1].split(",")))
    assert row["state"] in ("LANE", "CORNER_APPROACH", "PIVOT", "REACQUIRE")
    assert len(log) > 1 and ",rowfit," in log[1]


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
    log = (tmp_path / "lane-vision.csv").read_text(encoding="utf-8", errors="replace").splitlines()
    rows = [dict(zip(log[0].split(","), r.split(","))) for r in log[1:]]
    slow = [r for r in rows if r["obstacle_state"] == "SLOW_FOR_OBSTACLE"]
    assert slow and slow[-1]["obstacle_band"] == "low", log[:3]
    assert abs(float(slow[-1]["obstacle_m"]) - 1.0) < 0.08


def test_hud_paint_and_speed_trace_follow_the_env_flags(tmp_path):
    text = _run(tmp_path, None, extra={"RBM_HUD": "1", "RBM_SPEED_TRACE": "1"})
    assert "ButlerBot controller stopped" in text, text
    assert "Controller step error" not in text, text
    assert "HUD hud_nadir ← nadir_left" in text, text
    assert "HUD hud_nadir_r ← nadir_right" in text, text
    assert "HUD paint is on" in text
    assert "SPEED_TRACE on" in text
    # Console printing is off. The controller writes the CSV itself.
    assert not any(line.startswith("SPEED_TRACE t_sim=") for line in text.splitlines())
    traces = list(tmp_path.glob("speed_trace_*.csv"))
    assert len(traces) == 1, text
    rows = traces[0].read_text(encoding="utf-8").splitlines()
    assert rows[0].startswith("kind,")
    assert any(row.startswith("trace,") for row in rows), rows[:8]
    assert any(row.startswith("steer,") for row in rows)
    assert any(line.startswith("STEER_SPLIT ") for line in text.splitlines())
    # 200 steps of 8 ms is 1.6 s of sim. The 10 s SIM_RTF sample stays quiet.
    assert not any(line.startswith("SIM_RTF ") for line in text.splitlines())


def test_speed_trace_console_flag_prints_rows(tmp_path):
    text = _run(
        tmp_path,
        None,
        extra={"RBM_SPEED_TRACE": "1", "RBM_SPEED_TRACE_CONSOLE": "1"},
    )
    assert "console on" in text
    assert any(line.startswith("SPEED_TRACE t_sim=") for line in text.splitlines())
    assert "wL_cmd=" in text and "v_body=" in text


def test_obstacles_world_tag_turns_sensing_on(tmp_path):
    text = _run(tmp_path, None, "butlerbot_warehouse_obstacles.wbt")
    assert "OBSTACLES on (world tag '# OBSTACLES on' in butlerbot_warehouse_obstacles.wbt" in text, text
    text = _run(tmp_path, None, "butlerbot_warehouse_obstacles.wbt", extra={"RBM_OBSTACLES": "0"})
    assert "OBSTACLES off (RBM_OBSTACLES=0)" in text, text
