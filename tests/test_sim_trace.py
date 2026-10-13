"""Speed-trace and HUD-paint flags. No Webots."""

from src.mission_park import warehouse_perimeter_m
from src.sim_trace import (
    SpeedTrace,
    overlay_status_lines,
    resolve_hud_overlays,
    resolve_speed_trace,
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
