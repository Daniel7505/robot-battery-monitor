"""RBM_LANE_MODE=rowfit controller path (no Webots)."""
import importlib.util
import math
import sys
from pathlib import Path

import pytest

from src.lane_keep import MAX_WHEEL_RAD_S, NadirGuard, WHEEL_RADIUS_M, lane_keep_command
from src.lane_vision import LaneEstimate
from src import rowfit_control as rc

ROOT = Path(__file__).resolve().parent.parent
CTRL_DIR = ROOT / "webots" / "controllers" / "butlerbot_controller"


def est(offset=0.0, heading=0.0, k=0.0, conf=0.95, look=1.9, near_k=None):
    # centre arc anchored at x=0 in the robot frame
    psi = -heading
    a0 = -offset / max(1e-6, math.cos(psi))
    return LaneEstimate(
        valid=True, offset_m=offset, heading_rad=heading, curvature_1pm=k,
        lookahead_m=look, confidence=conf, n_left=60, n_right=60,
        lane_width_m=1.3, width_measured=True, a0=a0, psi=psi,
        near_curvature_1pm=k if near_k is None else near_k,
    )


def run_ticks(ctl, n, dt=0.008):
    out = None
    for _ in range(n):
        out = ctl.step(None, new_frame=False, dt=dt)
    return out


def test_robot_left_of_centre_yaws_right():
    ctl = rc.RowfitController()
    ctl.step(est(offset=0.15), new_frame=True)
    cmd = run_ticks(ctl, 30)
    assert cmd["steer"] > 0.02 and cmd["left"] > cmd["right"]


def test_robot_right_of_centre_yaws_left():
    ctl = rc.RowfitController()
    ctl.step(est(offset=-0.15), new_frame=True)
    cmd = run_ticks(ctl, 30)
    assert cmd["steer"] < -0.02 and cmd["left"] < cmd["right"]


def test_pointed_left_of_lane_yaws_right():
    ctl = rc.RowfitController()
    ctl.step(est(heading=0.15), new_frame=True)
    assert run_ticks(ctl, 30)["steer"] > 0.02


def test_left_curve_feed_forward_yaws_left_on_centre():
    ctl = rc.RowfitController()
    ctl.step(est(k=0.77), new_frame=True)
    cmd = run_ticks(ctl, 40)
    assert cmd["steer"] < -0.05
    # pure pursuit on the arc returns the arc's own curvature
    assert ctl.kappa_cmd == pytest.approx(0.77, abs=0.03)


def test_command_held_between_frames_and_only_new_frames_count():
    ctl = rc.RowfitController()
    ctl.step(est(offset=0.1), new_frame=True)
    k0 = ctl.kappa_cmd
    t0 = ctl.target_speed
    for _ in range(39):  # 39 ticks of a 320 ms frame
        ctl.step(est(offset=-0.3, k=2.0), new_frame=False)  # stale/ignored
    assert ctl.kappa_cmd == k0 and ctl.target_speed == t0
    assert ctl.n_frames == 1
    ctl.step(est(offset=-0.3), new_frame=True)
    assert ctl.n_frames == 2 and ctl.kappa_cmd > 0  # now turns left (negative offset)


def test_derivative_uses_frame_time_not_tick_time():
    a = rc.RowfitController()
    a.step(est(offset=0.00), new_frame=True, frame_dt=0.32)
    a.step(est(offset=0.02), new_frame=True, frame_dt=0.32)
    b = rc.RowfitController()
    b.step(est(offset=0.02), new_frame=True, frame_dt=0.32)
    # 2 cm in 320 ms -> damping 0.6 * 0.0625 = 0.0375 1/m, not a 40x spike
    assert (a.kappa_cmd - b.kappa_cmd) == pytest.approx(-rc.K_D_OFFSET * 0.02 / 0.32, abs=1e-3)


def test_slows_in_on_curvature_and_speeds_up_out():
    ctl = rc.RowfitController()
    ctl.gov.v = rc.V_CRUISE_M_S
    ctl.step(est(k=1.0), new_frame=True)
    assert ctl.target_speed < 0.36
    v_curve = run_ticks(ctl, 125)["speed_cmd"]  # 1 s
    assert v_curve == pytest.approx(ctl.target_speed, abs=0.01)
    ctl.step(est(k=0.0), new_frame=True)
    assert ctl.target_speed == pytest.approx(rc.V_CRUISE_M_S)
    v_half = run_ticks(ctl, 25)["speed_cmd"]  # 0.2 s: accel is gentle
    assert v_curve < v_half < rc.V_CRUISE_M_S
    v_out = run_ticks(ctl, 250)["speed_cmd"]
    assert v_out == pytest.approx(rc.V_CRUISE_M_S, abs=1e-3)


def test_curve_speed_bounds():
    assert rc.curve_speed(0.0) == pytest.approx(0.44)
    assert rc.curve_speed(10.0) == pytest.approx(rc.V_MIN_M_S)
    assert rc.curve_speed(0.0, conf=0.1) == pytest.approx(rc.V_MIN_M_S)
    assert rc.curve_speed(0.2) == pytest.approx(0.44)  # gentle bends: full cruise
    assert rc.curve_speed(1.5) < rc.curve_speed(1.0) < rc.curve_speed(0.77) < 0.44


def test_wheel_limits_match_wheels_from_steer():
    ctl = rc.RowfitController()
    ctl.gov.v = rc.V_CRUISE_M_S
    ctl.step(est(offset=-0.6, heading=-0.5, k=2.0), new_frame=True)
    cmd = run_ticks(ctl, 100)
    cruise = ctl.gov.v / WHEEL_RADIUS_M
    for w in (cmd["left"], cmd["right"]):
        assert 0.35 * cruise - 1e-3 <= w <= MAX_WHEEL_RAD_S + 1e-3
    assert abs(cmd["steer"]) <= rc.STEER_CAP + 1e-9


def test_lane_lost_three_frames_brakes_then_recovers():
    ctl = rc.RowfitController()
    ctl.step(est(), new_frame=True)
    bad = LaneEstimate(valid=False)
    assert not ctl.step(bad, new_frame=True)["brake"]
    assert not ctl.step(bad, new_frame=True)["brake"]
    assert ctl.target_speed == rc.V_MIN_M_S
    assert ctl.step(bad, new_frame=True)["brake"]
    assert not ctl.step(est(), new_frame=True)["brake"]


def test_closed_loop_ninety_degree_turn():
    spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    res = sim.run("turn90", max_s=25.0)
    assert res["finished"]
    assert res["max_ct_m"] < 0.15
    assert res["min_v"] < 0.37  # slowed in the bend (after the start ramp)
    assert res["max_v"] == pytest.approx(0.44, abs=0.01)


# ---------------------------------------------------------- NadirGuard (bug b)


def test_nadir_guard_legacy_per_tick_never_fires():
    # Documents the bug: same frame repeated 40 ticks resets the counter.
    g = NadirGuard()
    l, r = 33, 33
    for _frame in range(10):
        l, r = l - 3, r - 3
        for _ in range(40):
            assert not g.step(l, r)["abort"]


def test_nadir_guard_per_frame_fires_on_unison():
    g = NadirGuard()
    l, r = 33, 33
    fired = False
    for _frame in range(10):
        l, r = l - 3, r - 3
        for t in range(40):
            if g.step(l, r, new_frame=(t == 0))["abort"]:
                fired = True
    assert fired and "unison" in g.reason


def test_nadir_guard_per_frame_ignores_opposite_moves():
    g = NadirGuard()
    l, r = 33, 33
    for _frame in range(10):
        l, r = l - 3, r + 3  # normal steering: one gap shrinks, other grows
        for t in range(40):
            assert not g.step(l, r, new_frame=(t == 0))["abort"]


def test_gap_lane_keep_default_unchanged_by_new_kwarg():
    from src.lane_keep import SteerFilter

    a = lane_keep_command(left_gap_px=20, right_gap_px=40, nadir_guard=NadirGuard(), steer_filter=SteerFilter())
    b = lane_keep_command(left_gap_px=20, right_gap_px=40, nadir_guard=NadirGuard(), steer_filter=SteerFilter(),
                          guard_new_frame=None)
    assert a == b


# ---------------------------------------------------------- controller glue


def _load_glue(monkeypatch, env_file: Path):
    monkeypatch.syspath_prepend(str(CTRL_DIR))
    for m in ("twin_publisher", "controller_rowfit"):
        sys.modules.pop(m, None)
    import twin_publisher

    monkeypatch.setattr(twin_publisher, "_PROJECT_ENV_FILE", str(env_file))
    import controller_rowfit

    return controller_rowfit


def test_lane_mode_default_env_and_dotenv(monkeypatch, tmp_path):
    env = tmp_path / ".env"
    monkeypatch.delenv("RBM_LANE_MODE", raising=False)
    glue = _load_glue(monkeypatch, env)
    assert glue.lane_mode() == "gap"
    env.write_text("# x\nRBM_LANE_MODE=rowfit  # try it\n")
    assert glue.lane_mode() == "rowfit"
    monkeypatch.setenv("RBM_LANE_MODE", "gap")  # process env wins over .env
    assert glue.lane_mode() == "gap"
    monkeypatch.setenv("RBM_LANE_MODE", "bogus")
    assert glue.lane_mode() == "gap"


def test_log_dir_default_and_override(monkeypatch, tmp_path):
    monkeypatch.delenv("RBM_LOG_DIR", raising=False)
    glue = _load_glue(monkeypatch, tmp_path / "none.env")
    assert glue.log_dir().endswith("Grok Workspace")
    monkeypatch.setenv("RBM_LOG_DIR", str(tmp_path / "logs"))
    assert glue.log_dir() == str(tmp_path / "logs")
    assert (tmp_path / "logs").is_dir()


def test_lane_vision_log_header_and_run_id(monkeypatch, tmp_path):
    glue = _load_glue(monkeypatch, tmp_path / "none.env")
    p = tmp_path / "lane-vision.csv"
    log = glue.LaneVisionLog(str(p), run_id="20261001-190000")
    log.write(x_m=1.0, y_m=0.1, mode="rowfit", est=est(offset=0.05), cmd={"steer": 0.1, "target_speed": 0.4}, new_frame=True)
    log.write(x_m=1.1, y_m=0.1, mode="rowfit", est=LaneEstimate(valid=False), cmd={}, new_frame=False)
    lines = p.read_text().splitlines()
    assert lines[0] == (
        "unix_s,x_m,y_m,mode,offset_m,heading_rad,curvature,lookahead_m,confidence,"
        "nL_pts,nR_pts,steer,target_speed,new_frame,run_id"
    )
    assert len(lines) == 3
    row = dict(zip(lines[0].split(","), lines[1].split(",")))
    assert row["mode"] == "rowfit" and row["new_frame"] == "1" and row["run_id"] == "20261001-190000"
    assert float(row["offset_m"]) == pytest.approx(0.05)


def test_runtime_end_to_end_with_fake_cameras(monkeypatch, tmp_path):
    from src.lane_vision import NADIR_CAMS, lane_polyline_robot, offset_polyline, render_lane_bgra

    glue = _load_glue(monkeypatch, tmp_path / "none.env")
    centre = lane_polyline_robot([(-2 + 0.1 * i, 0.0) for i in range(60)], (0.0, 0.1), 0.0)
    lines = [offset_polyline(centre, 0.65), offset_polyline(centre, -0.65)]

    class FakeCam:
        def __init__(self, model):
            self.img = render_lane_bgra(model, lines)

        def getImage(self):
            return self.img

        def getWidth(self):
            return 128

        def getHeight(self):
            return 128

        def getFov(self):
            return 1.2

    cams = {n: FakeCam(m) for n, m in NADIR_CAMS.items()}
    rt = glue.RowfitRuntime()
    assert rt.check_cameras(cams) == []
    rt.harvest(cams, yaw=0.0, sim_t=1.0)
    cmd = rt.command(0.008)
    assert cmd["new_frame"] and cmd["steer"] > 0  # 10 cm left of centre -> yaw right
    assert not rt.command(0.008)["new_frame"]
    eyes = {}
    rt.fill_eyes(eyes)
    assert eyes["error_source"] == "rowfit" and eyes["rowfit_text"]
    assert eyes["rowfit_offset_m"] == pytest.approx(0.1, abs=0.02)
