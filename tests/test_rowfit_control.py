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
    assert res["held_frames"] == 0  # plausibility gate stays quiet in a real bend
    assert res["pivots"] == [] and res["corner_first_seen"] is None  # 1 m radius: no corner cue
    assert res["min_v"] < 0.37  # slowed in the bend (after the start ramp)
    # straight lead-in / exit: straightaway cruise bump; the bend itself still slows (min_v above)
    assert res["max_v"] == pytest.approx(rc.V_STRAIGHT_M_S, abs=0.01)


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
    assert glue.DEFAULT_LANE_MODE == "rowfit"
    assert glue.lane_mode() == "rowfit"  # unset -> rowfit (new default)
    env.write_text("# x\nRBM_LANE_MODE=gap  # old pixel fan\n")
    assert glue.lane_mode() == "gap"
    env.write_text("RBM_LANE_MODE=\n")
    assert glue.lane_mode() == "rowfit"
    monkeypatch.setenv("RBM_LANE_MODE", "rowfit")  # process env wins over .env
    env.write_text("RBM_LANE_MODE=gap\n")
    assert glue.lane_mode() == "rowfit"
    monkeypatch.setenv("RBM_LANE_MODE", " GAP ")
    assert glue.lane_mode() == "gap"
    monkeypatch.setenv("RBM_LANE_MODE", "bogus")
    assert glue.lane_mode() == "rowfit"


def test_lookahead_bounds_follow_camera_coverage():
    from src.lane_vision import NADIR_CAMS, CameraModel, camera_coverage

    covs = [camera_coverage(c) for c in NADIR_CAMS.values()]
    assert rc.lookahead_bounds_from_coverage(covs) == pytest.approx((0.55, 1.6), abs=0.005)
    # the old z 1.31 m mount had the same top reach -> same tuned window
    old = CameraModel("old", (-0.386826, 0.504837, 1.306679), (-0.00125006, 0.999996, 0.00256741, 1.1), 128, 128, 1.2)
    assert rc.lookahead_bounds_from_coverage([camera_coverage(old)]) == pytest.approx((0.55, 1.6), abs=0.005)
    # a shorter-reach camera pulls ld_max in; a far-forward bottom row pushes ld_min out
    short = CameraModel("s", (0.0, 0.4, 0.7), (0, 1, 0, 1.2), 128, 128, 1.2)
    cs = camera_coverage(short)
    lo, hi = rc.lookahead_bounds_from_coverage([cs])
    assert hi == pytest.approx(cs["top_row_x_m"] - rc.LD_FAR_MARGIN_M) and lo <= hi
    lo, hi = rc.lookahead_bounds_from_coverage([{"top_row_x_m": 3.0, "bottom_row_x_m": 0.6}])
    assert lo == pytest.approx(0.85) and hi == pytest.approx(2.5)
    assert rc.lookahead_bounds_from_coverage([]) == (rc.LD_MIN_M, rc.LD_MAX_M)


def test_controller_uses_its_lookahead_window():
    ctl = rc.RowfitController(ld_min=0.7, ld_max=0.8)
    ctl.gov.v = 0.44  # base + gain*v = 0.98 -> clamped to 0.8
    ctl.on_frame(est(look=1.9))
    assert ctl.lookahead_used == pytest.approx(0.8, abs=0.01)
    ctl.reset()
    assert (ctl.ld_min, ctl.ld_max) == (0.7, 0.8)


def test_runtime_bind_cameras_reads_live_pose(monkeypatch, tmp_path):
    glue = _load_glue(monkeypatch, tmp_path / "none.env")

    class F:
        def __init__(self, v):
            self.v = list(v)

        def getSFVec3f(self):
            return self.v

        getSFRotation = getSFVec3f

    class N:
        def __init__(self, nid, parent=None, t=(0, 0, 0), r=(0, 0, 1, 0)):
            self.nid, self.parent, self.f = nid, parent, {"translation": F(t), "rotation": F(r)}

        def getId(self):
            return self.nid

        def getParentNode(self):
            return self.parent

        def getField(self, k):
            return self.f[k]

    class Dev:
        def __init__(self, name):
            self.name = name

        def getWidth(self):
            return 128

        def getHeight(self):
            return 128

        def getFov(self):
            return 1.2

    robot_node = N(1)
    y = {"nadir_left": 0.5, "nadir_right": -0.5}
    nodes = {n: N(10 + i, robot_node, (0.05, y[n], 0.8), (0, 1, 0, 1.0)) for i, n in enumerate(y)}

    class Sup:
        def getSelf(self):
            return robot_node

        def getFromDevice(self, dev):
            return nodes[dev.name]

    cams = {n: Dev(n) for n in y}
    rt = glue.RowfitRuntime()
    lines = []
    msgs = rt.bind_cameras(Sup(), cams, log=lines.append)
    assert msgs == []
    assert lines[0].startswith("CAM POSE nadir_left from supervisor: t=(0.0500, 0.5000, 0.8000)")
    assert "pitch=57.3deg" in lines[0]
    assert lines[1].startswith("CAM POSE nadir_right from supervisor: t=(0.0500, -0.5000, 0.8000)")
    assert rt.pose_source == {"nadir_left": "supervisor", "nadir_right": "supervisor"}
    assert rt.tracker.cams["nadir_left"].translation == (0.05, 0.5, 0.8)
    assert rt.cams_model["nadir_right"].rotation[3] == pytest.approx(1.0)
    # look-ahead window re-derived from THIS camera's (shorter) coverage
    from src.lane_vision import camera_coverage

    top = camera_coverage(rt.cams_model["nadir_left"])["top_row_x_m"]
    assert rt.ctl.ld_max == pytest.approx(max(rc.LD_ABS_MIN_M, top - rc.LD_FAR_MARGIN_M))
    assert rt.check_cameras(cams) == []


def test_runtime_bind_cameras_plain_robot_falls_back_to_wbt(monkeypatch, tmp_path):
    glue = _load_glue(monkeypatch, tmp_path / "none.env")
    from src.lane_vision import NADIR_LEFT_POSE

    class Dev:
        def getWidth(self):
            return 128

        def getHeight(self):
            return 128

        def getFov(self):
            return 1.2

    rt = glue.RowfitRuntime()
    lines = []
    msgs = rt.bind_cameras(object(), {"nadir_left": Dev(), "nadir_right": None}, log=lines.append)
    assert lines[0].startswith("CAM POSE nadir_left from wbt butlerbot.wbt")
    assert any("nadir_right: device missing" in m for m in msgs)
    assert any("using wbt" in m for m in msgs)
    assert rt.cams_model["nadir_left"].translation == NADIR_LEFT_POSE["translation"]


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
        "nL_pts,nR_pts,steer,target_speed,new_frame,run_id,"
        "state,corner_dir,corner_m,corner_conf,yaw_deg,yaw_target_deg,"
        "junction_type,junction_m,openings,route_choice,blind_m,"
        "obstacle_state,obstacle_m,obstacle_band,obstacle_conf,obstacle_moving"
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


def _r2025a_supervisor(nodes_by_tag, robot_node, defs=None, device_lookup_works=True):
    """Shape of Webots R2025a's Python Supervisor (controller/supervisor.py):
    getFromDevice(tag) hands the tag to ctypes, so a Device object raises
    ctypes.ArgumentError; getSelf() is Node(tag=0)."""
    import ctypes

    class Sup:
        def getSelf(self):
            return robot_node

        def getFromDevice(self, tag):
            if not isinstance(tag, int):
                raise ctypes.ArgumentError("argument 1: TypeError: Don't know how to convert parameter 1")
            return nodes_by_tag.get(tag) if device_lookup_works else None

        def getFromDef(self, name):
            return (defs or {}).get(name)

    return Sup()


class _F:
    def __init__(self, v):
        self.v = list(v)

    def getSFVec3f(self):
        return self.v

    getSFRotation = getSFVec3f


class _N:
    def __init__(self, nid, parent=None, t=(0, 0, 0), r=(0, 0, 1, 0)):
        self.nid, self.parent, self.f = nid, parent, {"translation": _F(t), "rotation": _F(r)}

    def getId(self):
        return self.nid

    def getParentNode(self):
        return self.parent

    def getField(self, k):
        return self.f.get(k)


class _TagDev:
    def __init__(self, tag):
        self._tag = tag

    def getWidth(self):
        return 128

    def getHeight(self):
        return 128

    def getFov(self):
        return 1.2


def test_r2025a_get_from_device_needs_the_tag(monkeypatch, tmp_path):
    # Dan's 2026-10-01 run: getFromDevice(camera_object) -> ctypes.ArgumentError,
    # silently swallowed -> "using wbt". Now the tag is passed.
    glue = _load_glue(monkeypatch, tmp_path / "none.env")
    robot = _N(1)
    nodes = {7: _N(2, robot, (0.03542, 0.41808, 0.70419), (0, 1, 0, 0.9411)),
             8: _N(3, robot, (0.03542, -0.41808, 0.70419), (0, 1, 0, 0.9411))}
    cams = {"nadir_left": _TagDev(7), "nadir_right": _TagDev(8)}
    rt = glue.RowfitRuntime()
    lines = []
    assert rt.bind_cameras(_r2025a_supervisor(nodes, robot), cams, log=lines.append) == []
    assert lines[0].startswith("CAM POSE nadir_left from supervisor: t=(0.0354, 0.4181, 0.7042)")
    assert lines[1].startswith("CAM POSE nadir_right from supervisor: t=(0.0354, -0.4181, 0.7042)")


def test_get_from_def_fallback_and_reason_in_warning(monkeypatch, tmp_path):
    glue = _load_glue(monkeypatch, tmp_path / "none.env")
    robot = _N(1)
    defs = {"NADIR_CAM_L": _N(2, robot, (0.1, 0.4, 0.7), (0, 1, 0, 0.9))}
    sup = _r2025a_supervisor({}, robot, defs=defs, device_lookup_works=False)
    rt = glue.RowfitRuntime()
    lines = []
    msgs = rt.bind_cameras(sup, {"nadir_left": _TagDev(7), "nadir_right": _TagDev(8)}, log=lines.append)
    assert lines[0].startswith("CAM POSE nadir_left from supervisor: t=(0.1000, 0.4000, 0.7000)")
    # right: device lookup returned None, no DEF -> wbt, and the warning says why
    assert lines[1].startswith("CAM POSE nadir_right from wbt")
    (m,) = msgs
    assert m.startswith("nadir_right: live pose not read from supervisor, using wbt")
    assert "getFromDevice(tag): returned None" in m
    assert "ArgumentError" in m and "Don't know how to convert parameter 1" in m
    assert "getFromDef('NADIR_CAM_R'): returned None" in m


def test_missing_fields_reason_is_reported():
    from src.lane_vision import supervisor_camera_pose

    robot = _N(1)
    bad = _N(2, robot)
    bad.f = {}
    with pytest.raises(LookupError, match="no translation/rotation field"):
        supervisor_camera_pose(_r2025a_supervisor({7: bad}, robot), _TagDev(7))


# ---------------------------------------------------------- plausibility gate


def _sim():
    spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    return sim


def _tracked_straight():
    """Tracker that has seen a straight, centred lane (prior + last good fit)."""
    from src.lane_vision import LaneTracker, lane_polyline_robot, offset_polyline, render_lane_bgra

    tr = LaneTracker()
    c = lane_polyline_robot([(-2 + 0.1 * i, 0.0) for i in range(60)], (0.0, 0.0), 0.0)
    lines = [offset_polyline(c, 0.65), offset_polyline(c, -0.65)]
    imgs = {n: (render_lane_bgra(m, lines), 128, 128) for n, m in tr.cams.items()}
    est = tr.process(imgs)
    assert est.valid and not est.held
    return tr


def test_dans_finish_frame_is_rejected():
    # Webots 2026-10-01 run 20261001-201555, last frame before parking:
    # off=-0.517 hd=-1.576 k=-0.01 conf=0.38 nL=0 nR=218 (red bar fitted as lane)
    tr = _tracked_straight()
    bogus = LaneEstimate(valid=True, offset_m=-0.517, heading_rad=-1.576, curvature_1pm=-0.01,
                         lookahead_m=1.0, confidence=0.38, n_left=0, n_right=218)
    reason = tr.gate_reason(bogus, tr.prior_near, 0.14)
    assert "heading -90deg beyond" in reason
    # each gate on its own
    assert "heading jump" in tr.gate_reason(
        LaneEstimate(valid=True, heading_rad=-0.6, lookahead_m=1.5, confidence=0.9, n_left=50, n_right=50),
        tr.prior_near, 0.10)
    assert "offset jump" in tr.gate_reason(
        LaneEstimate(valid=True, offset_m=-0.3, lookahead_m=1.5, confidence=0.9, n_left=50, n_right=50),
        tr.prior_near, 0.14)
    assert "left side vanished" in tr.gate_reason(
        LaneEstimate(valid=True, lookahead_m=1.5, confidence=0.38, n_left=0, n_right=218), tr.prior_near, 0.14)
    assert "short view" in tr.gate_reason(
        LaneEstimate(valid=True, lookahead_m=0.3, confidence=0.9, n_left=20, n_right=20), tr.prior_near, 0.14)
    # a normal next frame passes; bigger jumps pass when the robot drove farther
    ok = LaneEstimate(valid=True, offset_m=0.03, heading_rad=0.05, lookahead_m=1.8, confidence=0.9,
                      n_left=60, n_right=60)
    assert tr.gate_reason(ok, tr.prior_near, 0.14) == ""
    assert tr.gate_reason(LaneEstimate(valid=True, heading_rad=0.4, lookahead_m=1.8, confidence=0.9,
                                       n_left=60, n_right=60), tr.prior_near, 0.14) == ""
    # no prior (start-up / after 3 lost frames): only the absolute limits apply
    assert tr.gate_reason(LaneEstimate(valid=True, offset_m=0.4, lookahead_m=0.3, confidence=0.42,
                                       n_left=40, n_right=0), None, 0.0) == ""


def test_rejected_fit_holds_last_lane_and_keeps_lost_brake():
    tr = _tracked_straight()
    ctl = rc.RowfitController()
    run_ticks(ctl, 0)
    ctl.step(tr.last, new_frame=True)
    run_ticks(ctl, 200)
    # three implausible frames in a row (simulate the fits)
    for i in range(3):
        tr._predict(0.14, 0.0)
        pred = tr.prior_near
        lost = tr.frames_lost
        bogus_reason = tr.gate_reason(
            LaneEstimate(valid=True, offset_m=-0.517, heading_rad=-1.576, lookahead_m=1.0,
                         confidence=0.38, n_left=0, n_right=218), pred, 0.14)
        assert bogus_reason
        tr.frames_lost = lost + 1
        held = tr.held_estimate(bogus_reason)
        assert held.held and not held.valid
        assert abs(held.heading_rad) < 1e-6 and abs(held.offset_m) < 0.01  # last good lane, moved
        cmd = ctl.step(held, new_frame=True)
        assert abs(cmd["steer"]) < 0.02, cmd  # no swerve toward the bogus 90 deg lane
        if i < 2:
            assert not cmd["brake"] and cmd["phase"] == "rowfit_hold"
            assert ctl.target_speed <= rc.V_HELD_M_S
    assert cmd["brake"]  # existing brake after LOST_FRAMES_STOP frames
    assert ctl.held_frames == 3


def test_short_view_holds_do_not_brake_at_once():
    from src.lane_vision import GATE_MAX_SHORT_HOLDS

    tr = _tracked_straight()
    short = LaneEstimate(valid=True, lookahead_m=0.3, confidence=0.9, n_left=20, n_right=20)
    tr.fit_points = lambda pts: short  # every frame: lane only 0.3 m long
    seen = []
    for _ in range(GATE_MAX_SHORT_HOLDS + 3):
        tr._predict(0.14, 0.0)
        e = tr.fit_points_gated([], dx_m=0.14)
        seen.append((e.held, e.counts_as_lost, tr.frames_lost))
    assert all(h for h, _c, _f in seen)
    assert [c for _h, c, _f in seen[:GATE_MAX_SHORT_HOLDS]] == [False] * GATE_MAX_SHORT_HOLDS
    assert seen[GATE_MAX_SHORT_HOLDS][1] and seen[-1][2] >= 3


def test_finish_bar_no_turn_closed_loop():
    sim = _sim()
    import src.lane_vision as lv

    kw = dict(start_x=13.5, start_offset=0.03, start_yaw=0.03)
    # Reproduce Dan's bug: gap-style yellow (red bar = paint) and no gate -> big swerve
    # (the corner cue's fit clipping is switched off too: it is newer than the bug)
    gate, cues = lv.LaneTracker.gate_reason, lv.corner_cues
    try:
        lv.LaneTracker.gate_reason = lambda self, *a, **k: ""
        lv.corner_cues = lambda *a, **k: []
        bug = sim.run("s_finish", yellow_mode="lane_keep", **kw)
    finally:
        lv.LaneTracker.gate_reason = gate
        lv.corner_cues = cues
    assert abs(bug["end_yaw_deg"]) > 10 and bug["max_yaw_rate_last_1m"] > 0.3
    # Gate alone (red bar still detected as yellow): no swerve
    gated = sim.run("s_finish", yellow_mode="lane_keep", **kw)
    assert abs(gated["end_yaw_deg"]) < 3 and gated["max_yaw_rate_last_1m"] < 0.05
    assert gated["end_x_m"] > 16.3  # stopped at the line (GPS or lost-lane brake)
    # Default (rg yellow + gate): drives to the GPS finish straight
    dflt = sim.run("s_finish", **kw)
    assert dflt["finished"] and abs(dflt["end_yaw_deg"]) < 2 and dflt["max_yaw_rate_last_1m"] < 0.05
    assert dflt["pivots"] == [] and dflt["corner_first_seen"] is None  # start / finish bars: no corner
    assert gated["pivots"] == []  # red bar scored as yellow: a bar across the lane, still no corner


def test_s_track_does_not_regress_with_gate():
    res = _sim().run("s")
    assert res["finished"] and res["max_ct_m"] < 0.085 and res["held_frames"] == 0
    assert res["pivots"] == [] and res["corner_first_seen"] is None
    assert res["max_v"] == pytest.approx(rc.V_STRAIGHT_M_S, abs=0.01)  # straightaway cruise on the straights
