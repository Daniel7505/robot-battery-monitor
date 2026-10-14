"""Speed-trace and HUD-paint flags. No Webots."""

import pytest

from src.mission_park import warehouse_perimeter_m
from src.sim_trace import (
    SpeedTrace,
    overlay_status_lines,
    resolve_hud_overlays,
    resolve_speed_trace,
    resolve_speed_trace_console,
)


def test_hud_paint_defaults_off_and_env_wins():
    assert resolve_hud_overlays() is False
    assert resolve_hud_overlays(config_value=None, env_value=None) is False
    assert resolve_hud_overlays(config_value="", env_value="") is False
    assert resolve_hud_overlays(config_value=False) is False
    assert resolve_hud_overlays(config_value=True) is True
    assert resolve_hud_overlays(config_value=True, env_value="0") is False
    assert resolve_hud_overlays(config_value=False, env_value="1") is True
    assert resolve_hud_overlays(env_value="on") is True
    assert resolve_hud_overlays(env_value="off") is False


def test_speed_trace_defaults_on_only_for_one_lap():
    lap = warehouse_perimeter_m()
    assert lap is not None and lap > 100
    assert resolve_speed_trace() is False
    assert resolve_speed_trace(mission_limit_m=None, lap_m=lap) is False
    assert resolve_speed_trace(mission_limit_m=lap, lap_m=lap) is True
    assert resolve_speed_trace(mission_limit_m=2 * lap, lap_m=lap) is False
    assert resolve_speed_trace(mission_limit_m=0.5 * lap, lap_m=lap) is False
    # A metre cap that is one lap still traces when the laps env is not set.
    assert resolve_speed_trace(mission_limit_m=lap * 1.01, lap_m=lap) is True
    assert resolve_speed_trace(config_value=True, mission_limit_m=None, lap_m=lap) is True
    assert resolve_speed_trace(config_value=False, mission_limit_m=lap, lap_m=lap) is False
    assert resolve_speed_trace(config_value=True, env_value="0", mission_limit_m=lap, lap_m=lap) is False
    assert resolve_speed_trace(env_value="1", mission_limit_m=None) is True
    assert resolve_speed_trace(config_value="yaml", env_value="none", mission_limit_m=lap, lap_m=lap) is True


def test_overlay_lines_name_each_window_and_the_black_stereo():
    off = "\n".join(overlay_status_lines(False))
    assert "nadir_left" in off and "nadir_right" in off
    assert "stereo_left" in off and "black" in off
    assert "hud_nadir" in off and "hud_nadir_r" in off
    assert "HUD paint is off" in off
    assert "controller cannot hide" in off
    on = "\n".join(overlay_status_lines(True))
    assert "HUD paint is on" in on
    assert "lane keeping" in off.lower() or "Lane keeping" in off


def test_speed_trace_samples_sim_wall_and_wheel_speeds_without_a_burst():
    trace = SpeedTrace(False)
    assert trace.lines(
        t_sim=1.0, t_wall=2.0, v_body=0.2, v_odo=0.2,
        w_l_cmd=2.5, w_r_cmd=2.5, w_l=2.5, w_r=2.5,
    ) == []

    trace = SpeedTrace(True, step_s=0.1, rtf_period_s=10.0)
    assert trace.lines(
        t_sim=0.0, t_wall=100.0, v_body=0.0, v_odo=0.0,
        w_l_cmd=0.0, w_r_cmd=0.0, w_l=0.0, w_r=0.0,
    ) == []
    first = trace.lines(
        t_sim=0.1, t_wall=100.2, v_body=0.210, v_odo=0.200,
        w_l_cmd=2.80, w_r_cmd=2.60, w_l=2.70, w_r=2.55,
    )
    assert len(first) == 1
    assert first[0] == (
        "SPEED_TRACE t_sim=0.100 t_wall=100.200 rtf=0.500 "
        "v_body=0.210 v_odo=0.200 "
        "wL_cmd=2.800 wR_cmd=2.600 wL=2.700 wR=2.550"
    )
    # A hitch that skips 0.2 and 0.3 still prints one line.
    jumped = trace.lines(
        t_sim=0.45, t_wall=101.0, v_body=0.22, v_odo=0.21,
        w_l_cmd=2.8, w_r_cmd=2.7, w_l=2.7, w_r=2.6,
    )
    assert len(jumped) == 1
    assert jumped[0].startswith("SPEED_TRACE t_sim=0.450 ")
    assert "rtf=0.438" in jumped[0]


def test_sim_rtf_logs_the_mean_wheel_split():
    trace = SpeedTrace(True, step_s=0.1, rtf_period_s=0.2)
    trace.lines(
        t_sim=0.0, t_wall=0.0, v_body=0.2, v_odo=0.2,
        w_l_cmd=3.0, w_r_cmd=2.0, w_l=2.8, w_r=2.4,
    )
    mid = trace.lines(
        t_sim=0.1, t_wall=0.2, v_body=0.2, v_odo=0.2,
        w_l_cmd=3.0, w_r_cmd=2.0, w_l=2.8, w_r=2.4,
    )
    assert len(mid) == 1 and mid[0].startswith("SPEED_TRACE ")
    both = trace.lines(
        t_sim=0.2, t_wall=0.5, v_body=0.2, v_odo=0.2,
        w_l_cmd=3.0, w_r_cmd=2.0, w_l=2.8, w_r=2.4,
    )
    assert len(both) == 2
    rtf = both[1]
    assert rtf.startswith("SIM_RTF t_sim=0.200 ")
    # Three samples, each |cmd| split 1.0 and |hub| split 0.4.
    assert "rtf=0.400" in rtf
    assert "mean_cmd_split=1.000" in rtf
    assert "mean_hub_split=0.400" in rtf
    assert rtf.endswith("n=3")


def test_speed_trace_console_defaults_off():
    assert resolve_speed_trace_console() is False
    assert resolve_speed_trace_console(config_value=None, env_value="") is False
    assert resolve_speed_trace_console(config_value=True) is True
    assert resolve_speed_trace_console(config_value=True, env_value="0") is False
    assert resolve_speed_trace_console(env_value="1") is True


def test_speed_trace_file_skips_parked_rows_and_stops_at_mission_end(tmp_path):
    path = tmp_path / "speed_trace_test.csv"
    trace = SpeedTrace(True, step_s=0.1, rtf_period_s=10.0, path=str(path))
    for i in range(30):
        assert trace.lines(
            t_sim=i * 0.1,
            t_wall=i * 0.1,
            v_body=0.0,
            v_odo=0.0,
            w_l_cmd=0.0,
            w_r_cmd=0.0,
            w_l=0.0,
            w_r=0.0,
        ) == []
    parked = path.read_text(encoding="utf-8").strip().splitlines()
    assert len(parked) == 1
    assert parked[0].startswith("kind,")

    # Signed command split ±0.2, hub split ±0.1, two moving steps.
    trace.lines(
        t_sim=3.0, t_wall=3.0, v_body=0.20, v_odo=0.20,
        w_l_cmd=2.6, w_r_cmd=2.4, w_l=2.55, w_r=2.45,
    )
    trace.lines(
        t_sim=3.1, t_wall=3.2, v_body=0.20, v_odo=0.20,
        w_l_cmd=2.4, w_r_cmd=2.6, w_l=2.45, w_r=2.55,
    )
    body = path.read_text(encoding="utf-8")
    assert "trace," in body
    line = trace.finish()
    assert line.startswith("STEER_SPLIT ")
    fields = dict(part.split("=", 1) for part in line.split() if "=" in part)
    assert int(fields["n"]) == 2
    # Population variance of +0.2 and -0.2 is 0.04. Mean is 0.
    assert float(fields["mean_cmd_split"]) == pytest.approx(0.0, abs=1e-9)
    assert float(fields["var_cmd_split"]) == pytest.approx(0.04, abs=1e-9)
    assert float(fields["var_hub_split"]) == pytest.approx(0.01, abs=1e-9)
    assert float(fields["mean_abs_cmd_split"]) == pytest.approx(0.2, abs=1e-9)
    # Two written 0.1 s bins: 0.1/0.1 then 0.1/0.2. Time-weighted, 0.2/0.3.
    assert fields["mean_rtf"] == "0.667"
    closed = path.read_text(encoding="utf-8")
    assert "steer," in closed
    trace.lines(
        t_sim=4.0, t_wall=5.0, v_body=0.30, v_odo=0.30,
        w_l_cmd=3.0, w_r_cmd=2.0, w_l=2.8, w_r=2.2,
    )
    assert path.read_text(encoding="utf-8") == closed
    assert trace.finish() == line


def test_mean_rtf_keeps_sim_time_when_a_step_wall_clock_sticks(tmp_path):
    """Run #12 printed mean_rtf 0.419 against a trace median near 0.95.

    The old average dropped any controller step whose wall delta was not
    positive, and did not put that step's sim time into the next step.
    The CSV rtf column is the 0.1 s bin, which still contains the step.
    """
    path = tmp_path / "speed_trace_rtf.csv"
    trace = SpeedTrace(True, step_s=0.1, rtf_period_s=100.0, path=str(path))
    moving = dict(v_body=0.30, v_odo=0.30, w_l_cmd=3.0, w_r_cmd=3.0, w_l=3.0, w_r=3.0)
    trace.lines(t_sim=0.0, t_wall=100.0, **moving)
    trace.lines(t_sim=0.05, t_wall=100.0, **moving)
    trace.lines(t_sim=0.10, t_wall=100.2, **moving)
    trace.lines(t_sim=0.20, t_wall=100.5, **moving)
    line = trace.finish()
    fields = dict(part.split("=", 1) for part in line.split() if "=" in part)
    # Bins 0.10/0.20 and 0.10/0.30. A dropped 0.05 s step would print 0.300.
    assert fields["mean_rtf"] == "0.400"
