"""Stereo obstacle sensing: rig geometry, depth accuracy, ground filter, corridor, gate, sim.

Frames are rendered with src.stereo_synth (numpy raycaster: textured floor,
tape, textured boxes) from the SAME camera models, so these check geometry,
matching and logic, not Webots lighting. Both disparity backends run when
OpenCV is installed; CI (no OpenCV) runs the numpy block matcher.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from src.lane_vision import CameraModel, parse_wbt_cameras
from src.obstacle_gate import (
    CLEAR_FRAMES,
    STATE_CLEAR,
    STATE_SLOW,
    STATE_STOPPED,
    ObstacleFusion,
    ObstacleGate,
    ObstacleReport,
    stop_distance_m,
)
from src.rowfit_control import DECEL_M_S2, V_CRUISE_M_S, V_STRAIGHT_M_S, RowfitController
from src.stereo_depth import (
    HAVE_CV2,
    STEREO_BASELINE_M,
    STEREO_LEFT_CAM,
    STEREO_LEFT_POSE,
    STEREO_PITCH_RAD,
    STEREO_RIGHT_CAM,
    STEREO_RIGHT_POSE,
    CorridorParams,
    StereoObstacleSensor,
    StereoRig,
    stereo_pitch_for_top,
)
from src.stereo_synth import SceneBox, render_pair, table

ROOT = Path(__file__).resolve().parents[1]
WORLDS = sorted((ROOT / "webots" / "worlds").glob("*.wbt"))
BACKENDS = ["numpy"] + (["sgbm"] if HAVE_CV2 else [])
RIG = StereoRig(STEREO_LEFT_CAM, STEREO_RIGHT_CAM)
TAPES = [(-1.0, 0.65, 8.0, 0.65, 0.06), (-1.0, -0.65, 8.0, -0.65, 0.06), (3.0, -0.65, 3.0, 0.65, 0.08)]


def sense(boxes, pose=(0.0, 0.0, 0.0), backend="numpy", tapes=(), kappa=0.0, rig=RIG, prm=None):
    L, R = render_pair(rig, pose, boxes, tapes=tapes, supersample=2)
    return StereoObstacleSensor(rig, prm, backend=backend).process_gray(L, R, kappa=kappa)


# ---------------------------------------------------------------- mount / rig


@pytest.mark.parametrize("world", WORLDS, ids=lambda p: p.name)
def test_every_world_has_the_stereo_pair_matching_the_constants(world):
    cams = parse_wbt_cameras(str(world))
    for name, pose in (("stereo_left", STEREO_LEFT_POSE), ("stereo_right", STEREO_RIGHT_POSE)):
        c = cams[name]
        assert c["translation"] == pytest.approx(pose["translation"])
        assert c["rotation"] == pytest.approx(pose["rotation"])
        assert (c["width"], c["height"], c["fov"]) == (pose["width"], pose["height"], pose["fov"])
    assert "nadir_left" in cams and "nadir_right" in cams  # shoulder cams untouched


def test_mount_baseline_pitch_and_coverage_are_derived():
    assert 0.08 <= STEREO_BASELINE_M <= 0.12
    assert RIG.baseline_m == pytest.approx(0.10)
    assert STEREO_PITCH_RAD == pytest.approx(stereo_pitch_for_top(0.78), abs=1e-4)
    assert not any(RIG.needs_rect.values())  # equal rotations, baseline along camera y: already rectified
    assert RIG.top_ray_height(0.14 + 1.0) == pytest.approx(1.0, abs=0.01)  # waist / table top seen 1 m ahead
    lx, lz, tan_down = RIG.view_bottom()
    assert 0.6 < lx + lz / tan_down < 0.9  # floor visible from ~0.75 m ahead of the axle
    assert RIG.num_disparities() == 64  # 0.30 m nearest depth -> fB / 0.3 = 63 px


def test_rectification_corrects_a_toed_in_right_camera():
    """Right lens yawed 0.6 deg: without rectification the depth is off; the rig's homography fixes it."""
    from src.lane_vision import yaw_pitch_rotation

    rot = yaw_pitch_rotation(math.radians(0.6), STEREO_PITCH_RAD)
    right = CameraModel("stereo_right", STEREO_RIGHT_POSE["translation"], rot, 320, 240, 1.4)
    wall = [SceneBox.at(2.5 + 0.05, 0.0, 0.1, 3.0, 2.0)]  # textured wall face at x = 2.5
    rig = StereoRig(STEREO_LEFT_CAM, right)
    assert rig.needs_rect["right"]  # (the common rotation is the average: both get a homography)
    L, R = render_pair(rig, (0, 0, 0), wall, supersample=2)
    good = StereoObstacleSensor(rig, backend="numpy").process_gray(L, R)
    naive = StereoObstacleSensor(StereoRig(STEREO_LEFT_CAM, STEREO_RIGHT_CAM), backend="numpy").process_gray(L, R)
    assert good.detected and abs(good.distance_m - 2.5) < 0.08
    assert not naive.detected or abs(naive.distance_m - 2.5) > 2 * abs(good.distance_m - 2.5)


# ---------------------------------------------------------------- depth accuracy


def wall_depth_error(dist, backend):
    """Median / p90 |x - true| of points on a textured wall facing the robot at x = dist."""
    wall = [SceneBox.at(dist + 0.05, 0.0, 0.1, 4.0, 2.5, seed=3)]
    L, R = render_pair(RIG, (0, 0, 0), wall, supersample=2)
    s = StereoObstacleSensor(RIG, backend=backend)
    s.process_gray(L, R)
    from src.stereo_depth import points_from_disparity

    pts, _r, _c = points_from_disparity(s.disp, RIG, 8.0)
    on = (pts[:, 2] > 0.15) & (np.abs(pts[:, 1]) < 0.8)
    err = pts[on, 0] - dist
    return float(np.median(err)), float(np.percentile(np.abs(err), 90)), int(on.sum())


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("dist", [0.6, 1.0, 2.0, 3.0, 4.0])
def test_depth_accuracy_on_a_textured_wall(dist, backend):
    med, p90, n = wall_depth_error(dist, backend)
    # theory: dZ = Z^2 / (f B) * dd; 0.25 px sub-pixel at fB = 19 px*m -> 1.3 cm at 1 m, 21 cm at 4 m
    tol = 0.01 + 0.25 * dist * dist / RIG.fB
    assert abs(med) < tol, (dist, med, p90)
    assert p90 < 3 * tol + 0.02, (dist, med, p90)
    assert n > 2000


# ---------------------------------------------------------------- ground filter / corridor


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("pose", [(0, 0, 0), (0.5, 0.1, 0.05), (1.2, -0.2, -0.08)])
def test_floor_and_tape_are_not_obstacles(pose, backend):
    rep = sense([], pose, backend, tapes=TAPES)
    assert not rep.detected, rep.describe()
    assert abs(rep.notes["floor_z_med"]) < 0.02  # ground plane from the camera pose agrees with the data
    assert rep.notes["n_floor"] > 5000


@pytest.mark.parametrize("backend", BACKENDS)
def test_box_in_the_lane(backend):
    box = SceneBox.at(2.0 + 0.3, 0.1, 0.6, 0.5, 0.5)  # front face at x = 2.0, y -0.15..0.35
    rep = sense([box], backend=backend, tapes=TAPES)
    assert rep.detected and rep.band == "low"
    assert rep.distance_m == pytest.approx(2.0, abs=0.08)
    assert rep.clearance_m == pytest.approx(2.0 - 0.15, abs=0.08)
    assert rep.y_right_m == pytest.approx(-0.15, abs=0.06) and rep.y_left_m == pytest.approx(0.35, abs=0.06)
    assert rep.z_top_m == pytest.approx(0.5, abs=0.05)
    assert rep.confidence >= 0.9


@pytest.mark.parametrize("backend", BACKENDS)
def test_waist_high_table_edge_intruding(backend):
    """Table 0.9 m high whose edge is 0.30 m into the lane (legs + top): waist band."""
    tb = table(1.6 + 1.2, 0.30 + 0.45, 2.4, 0.9)  # near edge x = 1.6 m, inner edge y = 0.30
    rep = sense(tb, backend=backend, tapes=TAPES)
    assert rep.detected and rep.band == "waist", rep.describe()
    assert rep.distance_m == pytest.approx(1.6, abs=0.1)
    assert rep.y_right_m == pytest.approx(0.30, abs=0.08)
    assert rep.z_top_m == pytest.approx(0.9, abs=0.06)


@pytest.mark.parametrize("backend", BACKENDS)
def test_rack_in_the_lane_is_tall(backend):
    """A 6 m rack: 'tall' from 3.5 m (top ray reaches ~1.5 m there); at 2.5 m the
    top ray only reaches ~1.3 m, so the top is clipped and the gate keeps the tall band."""
    far = sense([SceneBox.at(3.5 + 0.55, 0.0, 1.1, 2.0, 6.0)], backend=backend)
    assert far.detected and far.band == "tall" and far.top_clipped, far.describe()
    assert far.distance_m == pytest.approx(3.5, abs=0.2)
    near = sense([SceneBox.at(2.5 + 0.55, 0.0, 1.1, 2.0, 6.0)], backend=backend)
    assert near.detected and near.top_clipped and near.z_top_m > 1.1, near.describe()
    assert near.distance_m == pytest.approx(2.5, abs=0.12)
    g = new_gate()
    for rep, dx in ((far, 0.0), (far, 0.0), (near, 1.0)):
        g.on_frame(rep, v_m_s=0.44, dx_m=dx)
    assert g.fields()["obstacle_band"] == "tall"


@pytest.mark.parametrize("backend", BACKENDS)
def test_shelf_corner_just_outside_the_corridor_is_ignored(backend):
    """Corner 0.75 m left of the centre (lane edge 0.65 + 0.10), corridor +-0.55: pass by, no detection."""
    shelf = SceneBox.at(1.5 + 0.6, 0.75 + 0.3, 1.2, 0.6, 1.8)
    for x in (0.0, 0.6, 1.0):
        rep = sense([shelf], (x, 0.0, 0.0), backend)
        assert not rep.detected, (x, rep.describe())
    # the same shelf 0.35 m into the corridor is seen
    rep = sense([SceneBox.at(1.5 + 0.6, 0.20 + 0.3, 1.2, 0.6, 1.8)], backend=backend)
    assert rep.detected and rep.top_clipped and rep.distance_m == pytest.approx(1.5, abs=0.1), rep.describe()
    assert rep.y_right_m == pytest.approx(0.20, abs=0.08)


def test_corridor_bends_with_the_commanded_curvature():
    box = SceneBox.at(2.0 + 0.2, 0.85, 0.4, 0.4, 0.5)  # y 0.65..1.05 at 2 m: outside a straight corridor
    assert not sense([box]).detected
    rep = sense([box], kappa=0.425)  # left curve: centre y = k x^2 / 2 = 0.85 m at 2 m
    assert rep.detected and rep.distance_m == pytest.approx(2.0, abs=0.1)


def test_thin_mismatch_band_is_not_an_obstacle():
    """Points that only span 1-2 image rows (a horizon mismatch band) must not trigger."""
    from src.stereo_depth import detect_obstacle

    n = 3000  # dense enough to pass the point-count and surface-area tests
    xs = np.linspace(0.8, 0.95, n)
    pts = np.stack([xs, np.linspace(-0.3, 0.3, n), np.full(n, 0.78)], axis=1)
    rows = np.full(n, 50)
    assert not detect_obstacle(pts, RIG, CorridorParams(), rows=rows).detected
    rows = np.arange(n) % 20 + 40  # same points spread over 20 rows: a real surface
    assert detect_obstacle(pts, RIG, CorridorParams(), rows=rows).detected


def test_small_mismatch_blob_is_not_an_obstacle():
    """30 points spread over 9 rows at 1.1 m cover ~1e-3 m^2 (3 x 3 cm): below min_area_m2."""
    from src.stereo_depth import detect_obstacle

    pts = np.stack([np.full(30, 1.1), np.linspace(0.25, 0.28, 30), np.linspace(0.74, 0.78, 30)], axis=1)
    rows = np.arange(30) % 9 + 50
    assert not detect_obstacle(pts, RIG, CorridorParams(), rows=rows).detected


# ---------------------------------------------------------------- gate


def rep_at(clear, band="low", conf=1.0, z_top=0.5):
    return ObstacleReport(detected=True, distance_m=clear + 0.15, clearance_m=clear, y_left_m=0.2, y_right_m=-0.2,
                          z_bottom_m=0.1, z_top_m=z_top, band=band, confidence=conf, n_points=500, range_m=4.0,
                          view_bottom=RIG.view_bottom())


def clear_rep():
    return ObstacleReport(detected=False, range_m=4.0, view_bottom=RIG.view_bottom())


def new_gate():
    return ObstacleGate(decel_m_s2=DECEL_M_S2, v_cruise_m_s=V_CRUISE_M_S, latency_s=0.32)


def test_stop_distance_is_standoff_plus_reaction_plus_braking():
    assert stop_distance_m(0.44, 0.8, 0.32, 0.35) == pytest.approx(0.35 + 0.44 * 0.32 + 0.44 ** 2 / 1.6)
    g = new_gate()
    assert g.stop_distance(0.0) == pytest.approx(0.35)
    assert g.slow_distance == pytest.approx(g.stop_distance(V_CRUISE_M_S) + 1.0)


def test_gate_needs_two_frames_then_slows_then_stops_then_resumes():
    g = new_gate()
    assert g.on_frame(rep_at(1.4), v_m_s=0.44) == STATE_CLEAR  # one frame is not enough
    assert g.on_frame(rep_at(1.3), v_m_s=0.44, dx_m=0.1) == STATE_SLOW
    assert g.v_cap is not None and g.v_cap <= 0.20
    v = 0.20
    for c in (1.0, 0.8, 0.6):
        g.on_frame(rep_at(c), v_m_s=v, dx_m=0.2)
        assert g.state == STATE_SLOW and g.v_cap < 0.20 + 1e-9
    g.on_frame(rep_at(0.43), v_m_s=0.2, dx_m=0.17)
    assert g.state == STATE_STOPPED and g.v_cap == 0.0 and not g.emergency
    for _ in range(5):  # still there: stays stopped
        assert g.on_frame(rep_at(0.42), v_m_s=0.0) == STATE_STOPPED
    for k in range(CLEAR_FRAMES):  # removed: visible and empty -> CLEAR after CLEAR_FRAMES
        st = g.on_frame(clear_rep(), v_m_s=0.0)
        assert st == (STATE_CLEAR if k == CLEAR_FRAMES - 1 else STATE_STOPPED)
    assert g.v_cap is None and g.stops == 1


def test_gate_stops_at_the_braking_distance_for_the_current_speed():
    """At cruise the stop starts farther out than at creep speed (derived from v and decel)."""
    far = new_gate()
    far.on_frame(rep_at(0.70), v_m_s=0.44)
    far.on_frame(rep_at(0.60), v_m_s=0.44, dx_m=0.1)
    assert far.state == STATE_STOPPED  # 0.60 <= 0.35 + 0.14 + 0.12 = 0.61
    slow = new_gate()
    slow.on_frame(rep_at(0.70), v_m_s=0.10)
    slow.on_frame(rep_at(0.60), v_m_s=0.10, dx_m=0.03)
    assert slow.state == STATE_SLOW  # 0.60 > 0.35 + 0.03 + 0.006


def test_gate_emergency_when_inside_the_ramp_distance():
    g = new_gate()
    g.on_frame(rep_at(0.18), v_m_s=0.44)
    g.on_frame(rep_at(0.15), v_m_s=0.44)
    assert g.state == STATE_STOPPED and g.emergency


def test_gate_holds_a_low_obstacle_in_the_blind_zone():
    """A 0.2 m box under the lowest image row: 'not seen' is not 'gone'."""
    g = new_gate()
    for c in (0.6, 0.5, 0.40):
        g.on_frame(rep_at(c, z_top=0.2), v_m_s=0.2, dx_m=0.1)
    assert g.state == STATE_STOPPED
    for _ in range(10):
        g.on_frame(clear_rep(), v_m_s=0.0)
    assert g.state == STATE_STOPPED and g.blind_hold
    assert "blind" in g.hud_line()


def test_gate_resumes_slow_when_the_obstacle_moves_off():
    g = new_gate()
    g.on_frame(rep_at(0.36), v_m_s=0.0)
    g.on_frame(rep_at(0.36), v_m_s=0.0)
    assert g.state == STATE_STOPPED
    g.on_frame(rep_at(0.55), v_m_s=0.0)  # 0.55 < 0.35 + 0.25: hysteresis, stay
    assert g.state == STATE_STOPPED
    g.on_frame(rep_at(0.75), v_m_s=0.0)
    assert g.state == STATE_SLOW


def test_single_frame_false_positive_is_ignored():
    g = new_gate()
    g.on_frame(rep_at(0.5), v_m_s=0.44)
    g.on_frame(clear_rep(), v_m_s=0.44, dx_m=0.14)
    g.on_frame(clear_rep(), v_m_s=0.44, dx_m=0.14)
    assert g.state == STATE_CLEAR and g.stops == 0 and g.v_cap is None


def test_low_confidence_reports_are_ignored():
    g = new_gate()
    for _ in range(4):
        g.on_frame(rep_at(0.5, conf=0.2), v_m_s=0.44)
    assert g.state == STATE_CLEAR


def test_fusion_takes_the_nearest_detection_any_source():
    f = ObstacleFusion()
    f.add_report(rep_at(1.5))
    radar = rep_at(0.9)
    radar.source, radar.velocity_m_s = "radar", 0.3
    f.add_report(radar)
    assert f.fused().source == "radar"
    f.add_report(ObstacleReport(source="radar", detected=False, range_m=8.0))
    assert f.fused().source == "stereo"


# ---------------------------------------------------------------- controller integration


class _Est:
    valid = True
    held = False
    confidence = 0.9
    offset_m = 0.0
    heading_rad = 0.0
    curvature_1pm = 0.0
    near_curvature_1pm = 0.0
    lookahead_m = 1.9
    n_left = n_right = 100
    lane_width_m = 1.3
    corner = None
    junction = None
    view = None

    def goal_point(self, d):
        return d, 0.0


def test_rowfit_speed_cap_stops_and_releases_without_touching_the_lane_state():
    ctl = RowfitController()
    est = _Est()
    for i in range(600):
        cmd = ctl.step(est, new_frame=i % 40 == 0, dt=0.008)
    # clean, sure straight with no cap: straightaway cruise bump (0.44 -> 0.50)
    assert ctl.gov.v == pytest.approx(V_STRAIGHT_M_S, abs=0.01)
    ctl.obstacle_v_cap = 0.0
    for i in range(100):  # 0.50 m/s at 0.8 m/s^2 -> 0.63 s
        cmd = ctl.step(est, new_frame=i % 40 == 0, dt=0.008)
    assert ctl.gov.v == pytest.approx(0.0, abs=1e-6) and abs(cmd["left"]) < 1e-6 and not cmd["brake"]
    assert ctl.state == "LANE"
    ctl.obstacle_v_cap = None
    for i in range(300):
        cmd = ctl.step(est, new_frame=i % 40 == 0, dt=0.008)
    assert ctl.gov.v > 0.4


def test_corner_approach_keeps_its_remaining_distance_while_stopped():
    from src.rowfit_control import STATE_APPROACH

    ctl = RowfitController()
    ctl.state, ctl.turn_src, ctl.remaining_m, ctl.corner_dir = STATE_APPROACH, "corner", 0.8, 1
    ctl.gov.v = 0.2
    ctl.obstacle_v_cap = 0.0
    for _ in range(200):
        ctl.step(None, dt=0.008, yaw=0.0)
    rem = ctl.remaining_m
    for _ in range(200):
        ctl.step(None, dt=0.008, yaw=0.0)
    assert ctl.remaining_m == pytest.approx(rem) and 0.7 < rem < 0.8 and ctl.state == STATE_APPROACH


# ---------------------------------------------------------------- closed loop (offline sim)


def test_sim_stops_before_a_box_and_resumes_when_it_is_removed():
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    import rowfit_sim as rs
    from s_track import centerline

    box = SceneBox.at(6.3, centerline(6.0)[0], 0.6, 0.5, 0.5)  # on the S, ~6 m in
    r = rs.run("s", obstacles=[box], remove_after_stop_s=3.0, max_s=70)
    o = r["obstacles"]
    assert r["finished"], r
    assert not o["collided"]
    assert len(o["stops"]) == 1, o
    st = o["stops"][0]
    assert st["band"] == "low"
    assert 0.25 < st["true_clear_m"] < 0.60, st  # standoff 0.35 m + creep-speed reaction
    assert abs(st["est_clear_m"] - st["true_clear_m"]) < 0.10
    assert o["removed_at_s"] is not None and o["resumed_at_s"] is not None
    assert 0.0 < o["resumed_at_s"] - o["removed_at_s"] < 2.0  # CLEAR_FRAMES visible-empty frames
    assert r["max_ct_m"] < 0.12  # lane keep unharmed by the stop


def test_sim_without_obstacles_is_unchanged():
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    import rowfit_sim as rs

    a = rs.run("corner90", max_s=60)
    b = rs.run("corner90", max_s=60, obstacles=[])
    assert a["finished"] and b["finished"]
    assert a["max_ct_m"] == b["max_ct_m"] and a["time_s"] == b["time_s"]
    assert b["obstacles"]["stops"] == [] and b["obstacles"]["renders"] == 0
