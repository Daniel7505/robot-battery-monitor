"""Row-fit lane vision: camera model, detection and lane fit on synthetic frames.

Frames are rendered through the SAME camera model (src.lane_vision) from a
known lane, so these tests check geometry + fitting, not Webots lighting.
"""
import math
import random

import pytest

from src.lane_keep import yellow_score
from src.lane_vision import (
    NADIR_CAMS,
    NADIR_LEFT_CAM,
    NADIR_LEFT_POSE,
    NADIR_RIGHT_CAM,
    NADIR_RIGHT_POSE,
    LaneTracker,
    camera_coverage,
    detect_points,
    lane_polyline_robot,
    offset_polyline,
    parse_wbt_cameras,
    predict_model,
    render_lane_bgra,
    signed_lateral,
    yellow_mask,
)

HW = 0.65  # lane half width used to PAINT test frames only
ROOT_WORLD = __import__("pathlib").Path(__file__).resolve().parents[1] / "webots" / "worlds" / "butlerbot.wbt"


def path(segs, start=(-1.5, 0.0), h0=0.0, ds=0.04):
    x, y, h = start[0], start[1], h0
    pts = [(x, y)]
    for L, k in segs:
        n = max(1, int(round(L / ds)))
        s = L / n
        for _ in range(n):
            h += k * s / 2
            x += s * math.cos(h)
            y += s * math.sin(h)
            h += k * s / 2
            pts.append((x, y))
    return pts


def frames(centre_world, xy=(0.0, 0.0), yaw=0.0, *, drop=None, stripe_w=0.06, hw=HW):
    c = lane_polyline_robot(centre_world, xy, yaw)
    lines = []
    if drop != "left":
        lines.append(offset_polyline(c, hw))
    if drop != "right":
        lines.append(offset_polyline(c, -hw))
    return {n: (render_lane_bgra(cam, lines, stripe_w=stripe_w), 128, 128) for n, cam in NADIR_CAMS.items()}


# ---------------------------------------------------------------- camera model


def test_camera_constants_match_wbt():
    cams = parse_wbt_cameras()
    for name, pose in (("nadir_left", NADIR_LEFT_POSE), ("nadir_right", NADIR_RIGHT_POSE)):
        w = cams[name]
        assert w["width"] == pose["width"] and w["height"] == pose["height"]
        assert w["fov"] == pytest.approx(pose["fov"])
        assert w["translation"] == pytest.approx(pose["translation"])
        assert w["rotation"] == pytest.approx(pose["rotation"])


def test_webots_camera_looks_along_local_plus_x_pitched_down():
    cam = NADIR_LEFT_CAM
    dx, dy, dz = cam.ray(cam.cx, cam.cy)
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    # rotation 0 1 0 0.9411 turns camera +X to forward-and-down 53.9 deg
    assert math.degrees(math.asin(-dz / n)) == pytest.approx(53.92, abs=0.05)
    assert dx > 0
    # image col 0 is the camera's +Y (robot left), last col robot right
    assert cam.pixel_to_ground(0, 64)[1] > cam.pixel_to_ground(127, 64)[1]
    # row 0 is far, last row near
    assert cam.pixel_to_ground(64, 0)[0] > cam.pixel_to_ground(64, 127)[0]


def test_right_camera_is_true_mirror_of_left():
    # Pure pitch (0 1 0 theta) + negated translation Y = exact mirror.
    assert NADIR_RIGHT_POSE["rotation"] == NADIR_LEFT_POSE["rotation"]
    lt, rt = NADIR_LEFT_POSE["translation"], NADIR_RIGHT_POSE["translation"]
    assert (rt[0], -rt[1], rt[2]) == lt
    for col, row in ((10, 10), (64, 64), (120, 100)):
        lx, ly = NADIR_LEFT_CAM.pixel_to_ground(col, row)
        rx, ry = NADIR_RIGHT_CAM.pixel_to_ground(127 - col, row)
        assert lx == pytest.approx(rx, abs=1e-9)
        assert ly == pytest.approx(-ry, abs=1e-9)


def test_cameras_sit_in_their_shoulder_boxes():
    # Lens on the front-bottom edge of NADIR_BOX_L/R (0.05 m cube) so the
    # box is entirely behind the near plane (never rendered).
    import re
    from pathlib import Path

    wbt = (Path(__file__).resolve().parent.parent / "webots/worlds/butlerbot.wbt").read_text()
    for box, pose in (("NADIR_BOX_L", NADIR_LEFT_POSE), ("NADIR_BOX_R", NADIR_RIGHT_POSE)):
        blk = wbt[wbt.index(f"DEF {box} Transform"):]
        c = [float(v) for v in re.search(r"translation\s+([-0-9. ]+)\n", blk).group(1).split()]
        size = [float(v) for v in re.search(r"size\s+([-0-9. ]+)\n", blk).group(1).split()]
        t = pose["translation"]
        for i in range(3):
            assert abs(t[i] - c[i]) <= size[i] / 2 + 1e-9  # inside / on the box
        assert t[0] == pytest.approx(c[0] + size[0] / 2)  # front face
        assert t[2] == pytest.approx(c[2] - size[2] / 2)  # bottom face


def test_coverage_of_box_mounted_cameras():
    # Predicted (not yet seen in Webots): top row keeps the old 1.96 m reach,
    # bottom row lands just ahead of the axle instead of 0.55 m behind it.
    cov = camera_coverage(NADIR_LEFT_CAM)
    assert cov["bottom_row_x_m"] == pytest.approx(0.058, abs=0.005)
    assert cov["mid_row_x_m"] == pytest.approx(0.54, abs=0.01)
    assert cov["top_row_x_m"] == pytest.approx(1.96, abs=0.005)
    lat_bottom = cov["bottom_m_per_px_lat_long"][0]
    lat_top = cov["top_m_per_px_lat_long"][0]
    # 6 cm stripe: ~9.8 px wide at the bottom, ~3.3 px at the top
    assert 0.06 / lat_bottom == pytest.approx(9.8, abs=0.3)
    assert 0.06 / lat_top == pytest.approx(3.3, abs=0.2)


def test_ground_pixel_roundtrip():
    for cam in NADIR_CAMS.values():
        for col, row in ((3.0, 5.0), (64.0, 64.0), (110.5, 120.0)):
            g = cam.pixel_to_ground(col, row)
            back = cam.ground_to_pixel(*g)
            assert back[0] == pytest.approx(col, abs=1e-6)
            assert back[1] == pytest.approx(row, abs=1e-6)


def test_yellow_mask_lane_keep_mode_matches_lane_keep_score():
    rnd = random.Random(3)
    px = bytearray()
    want = []
    for _ in range(400):
        b, g, r = rnd.randrange(256), rnd.randrange(256), rnd.randrange(256)
        px += bytes((b, g, r, 255))
        want.append(1 if yellow_score((r / 255, g / 255, b / 255)) >= 0.22 else 0)
    assert list(yellow_mask(bytes(px), 20, 20, 0.22, mode="lane_keep")) == want


def test_yellow_mask_rg_needs_red_and_green():
    def one(rgb, mode=None):
        r, g, b = (int(round(v * 255)) for v in rgb)
        return yellow_mask(bytes((b, g, r, 255)), 1, 1, 0.22, mode=mode)[0]

    paint, floor, red_bar, green_bar = (0.95, 0.95, 0.2), (0.55, 0.56, 0.58), (0.9, 0.15, 0.12), (0.1, 0.85, 0.25)
    assert one(paint) and not one(floor)
    assert not one(red_bar) and not one(green_bar)  # s_track finish / start bars
    assert one(red_bar, "lane_keep")  # the gap score calls the red bar yellow
    rnd = random.Random(5)
    for _ in range(300):  # r == g: identical to the gap score
        v, b = rnd.random(), rnd.random()
        assert one((v, v, b)) == one((v, v, b), "lane_keep")


# ---------------------------------------------------------------- lane fits


@pytest.mark.parametrize("off,yaw_deg", [(0.0, 0.0), (0.12, 0.0), (-0.15, 4.0), (0.05, -8.0)])
def test_straight_lane_offset_and_heading(off, yaw_deg):
    straight = [(-2 + 0.1 * i, 0.0) for i in range(80)]
    est = LaneTracker().process(frames(straight, (0.0, off), math.radians(yaw_deg)))
    assert est.valid
    assert est.offset_m == pytest.approx(off, abs=0.015)
    assert est.heading_rad == pytest.approx(math.radians(yaw_deg), abs=0.015)
    assert abs(est.curvature_1pm) < 0.05
    assert est.lane_width_m == pytest.approx(2 * HW, abs=0.03)
    assert est.n_left > 20 and est.n_right > 20
    assert est.lookahead_m > 1.5
    assert est.confidence > 0.8
    assert est.width_ok_frac > 0.9


@pytest.mark.parametrize("k", [0.5, -0.5, 1 / 1.3, -1 / 1.3])
def test_constant_curve(k):
    # robot at the start of a pure arc (S-track tightest radius is 1.3 m)
    c = path([(1.5, 0.0), (5.0, k)])
    est = LaneTracker().process(frames(c))
    assert est.valid
    assert est.curvature_1pm == pytest.approx(k, abs=0.08)
    assert abs(est.offset_m) < 0.03
    assert abs(est.heading_rad) < 0.09


def test_curve_with_offset():
    c = path([(1.5, 0.0), (5.0, 0.77)])
    est = LaneTracker().process(frames(c, (0.0, -0.10)))
    assert est.offset_m == pytest.approx(-0.10, abs=0.03)
    assert est.curvature_1pm == pytest.approx(0.77, abs=0.08)


def test_ninety_degree_turn_ahead_is_seen_early():
    # straight 0.5 m more, then a 90 deg left turn of radius 0.8 m
    c = path([(2.0, 0.0), (0.8 * math.pi / 2, 1 / 0.8), (2.0, 0.0)])
    est = LaneTracker().process(frames(c))
    assert est.valid
    assert est.curvature_1pm > 0.7  # far field sees the bend
    assert abs(est.offset_m) < 0.03 and abs(est.heading_rad) < 0.05


def test_ninety_degree_turn_inside_goal_point_on_lane():
    c = path([(1.5, 0.0), (1.0 * math.pi / 2, 1.0), (2.0, 0.0)])
    est = LaneTracker().process(frames(c))
    assert est.valid
    gx, gy = est.goal_point(0.7)
    # goal point must lie on the true centre line
    d = min(math.hypot(gx - x, gy - y) for x, y in c)
    assert d < 0.05
    assert 2 * gy / (gx * gx + gy * gy) > 0.8  # strong left turn needed
    assert est.near_curvature_1pm > 0.4


def test_sideways_line_gives_points_without_24px_rejection():
    # a single stripe running ACROSS the view 1.2 m ahead (90 deg to travel)
    line = [(1.2, -3.0 + 0.05 * i) for i in range(121)]
    imgs = {n: (render_lane_bgra(cam, [line]), 128, 128) for n, cam in NADIR_CAMS.items()}
    pts = detect_points(NADIR_LEFT_CAM, *imgs["nadir_left"])
    assert len(pts) > 30
    assert all(abs(p.x - 1.2) < 0.03 for p in pts)
    assert any(p.axis == "col" for p in pts)


def test_one_line_missing_uses_online_width():
    straight = [(-2 + 0.1 * i, 0.0) for i in range(80)]
    tr = LaneTracker()
    first = tr.process(frames(straight, hw=0.55))  # this course is 1.10 m wide
    assert first.lane_width_m == pytest.approx(1.10, abs=0.03)
    # 12 cm sideways needs some driving to be physical (plausibility gate)
    est = tr.process(frames(straight, (0.0, 0.12), 0.0, drop="left", hw=0.55), dx_m=0.2)
    assert est.valid and est.n_left == 0 and est.n_right > 20
    assert est.offset_m == pytest.approx(0.12, abs=0.02)
    assert est.lane_width_m == pytest.approx(1.10, abs=0.03)
    assert est.confidence < first.confidence


def test_one_line_missing_from_start_still_valid():
    straight = [(-2 + 0.1 * i, 0.0) for i in range(80)]
    est = LaneTracker().process(frames(straight, drop="right"))
    assert est.valid and est.n_right == 0 and est.n_left > 20
    assert not est.width_measured
    assert abs(est.heading_rad) < 0.02


def test_no_lane_is_invalid_and_counts_lost():
    blank = {n: (render_lane_bgra(cam, []), 128, 128) for n, cam in NADIR_CAMS.items()}
    tr = LaneTracker()
    est = tr.process(blank)
    assert not est.valid and tr.frames_lost == 1


def test_six_cm_width_is_a_check_not_the_scale():
    straight = [(-2 + 0.1 * i, 0.0) for i in range(80)]
    good = LaneTracker().process(frames(straight))
    fat = LaneTracker().process(frames(straight, stripe_w=0.16))
    assert good.width_ok_frac > 0.9
    assert fat.width_ok_frac < 0.3
    # geometry still right: the projection, not the tape width, sets the scale
    assert fat.offset_m == pytest.approx(0.0, abs=0.02)
    assert fat.lane_width_m == pytest.approx(2 * HW, abs=0.04)
    assert fat.confidence < good.confidence


def test_tracker_follows_with_motion_prediction():
    c = path([(1.5, 0.0), (6.0, 0.6)], start=(-1.5, 0.0))
    tr = LaneTracker()
    tr.process(frames(c))
    # robot moved 0.14 m along the lane and turned with it
    x, y, h = 0.14, 0.6 * 0.14 ** 2 / 2, 0.6 * 0.14
    est = tr.process(frames(c, (x, y), h), dx_m=0.14, dyaw_rad=h)
    assert est.valid
    assert est.curvature_1pm == pytest.approx(0.6, abs=0.08)
    assert abs(est.offset_m) < 0.03


def test_predict_model_straight_lane():
    # straight lane 0.1 m to the left; robot turns +0.1 rad in place
    a0, psi, k = predict_model((0.1, 0.0, 0.0), 0.0, 0.1)
    assert psi == pytest.approx(-0.1, abs=1e-6)
    assert signed_lateral(0.0, 0.0, a0, psi, k) == pytest.approx(-0.1, abs=1e-3)


# ------------------------------------------------ Webots convention (owner drills)


def test_owner_drill_identity_rotation_sees_horizon():
    # 2026-09-16 drill: identity rotation showed the horizon (looks along +X)
    from src.lane_vision import CameraModel

    cam = CameraModel("drill", (0, 0, 1.3), (0, 0, 1, 0), 128, 128, 1.35, plane_z=0.0)
    dx, dy, dz = cam.ray(cam.cx, cam.cy)
    assert dx > 0 and abs(dz) < 1e-9 and abs(dy) < 1e-9
    assert cam.pixel_to_ground(cam.cx, 0) is None  # top half = sky


def test_owner_drill_pitch_90_looks_straight_down():
    from src.lane_vision import CameraModel

    cam = CameraModel("drill", (0, 0, 1.3), (0, 1, 0, 1.5708), 128, 128, 1.35, plane_z=0.0)
    x, y = cam.pixel_to_ground(cam.cx, cam.cy)
    assert abs(x) < 1e-3 and abs(y) < 1e-3


def test_owner_drill_nadir_on_last_row():
    # verified S-line drill: h 1.3 m, rotation 0 1 0 0.8958 (= pi/2 - FOV/2),
    # FOV 1.35 -> bottom row straight down, far ground ~5.8 m
    from src.lane_vision import CameraModel

    cam = CameraModel("drill", (0, 0, 1.3), (0, 1, 0, 0.8958), 128, 128, 1.35, plane_z=0.0)
    assert abs(cam.pixel_to_ground(cam.cx, 127)[0]) < 0.02
    assert cam.pixel_to_ground(cam.cx, 0)[0] == pytest.approx(5.8, abs=0.15)


def test_butlerbot_nadir_rows_under_this_convention():
    # ButlerBot nadirs: Robot node rotation is identity (cameras are its
    # direct children), camera rotation 0 1 0 pitch, FOV 1.2.
    import re
    from pathlib import Path

    wbt = (Path(__file__).resolve().parent.parent / "webots/worlds/butlerbot.wbt").read_text()
    robot = wbt[wbt.index("Robot {"):]
    rot = [float(v) for v in re.search(r"(?m)^\s*rotation\s+([-0-9. ]+)$", robot).group(1).split()]
    assert rot[3] == 0.0  # angle 0 = identity whatever the axis
    cam = NADIR_LEFT_CAM
    cov = camera_coverage(cam)
    pitch = NADIR_LEFT_POSE["rotation"][3]
    half = math.atan(cam.cy / cam.focal_px)  # row 0 / row 127 centre vs optical axis
    tx, _ty, tz = NADIR_LEFT_POSE["translation"]
    h = tz - 0.013
    assert pitch + half < math.pi / 2  # bottom row still looks forward
    assert cov["bottom_row_x_m"] == pytest.approx(tx + h / math.tan(pitch + half), abs=1e-6)
    assert cov["top_row_x_m"] == pytest.approx(tx + h / math.tan(pitch - half), abs=1e-6)


# ------------------------------------------------ live pose (Supervisor mocks)


class _Field:
    def __init__(self, v):
        self.v = list(v)

    def getSFVec3f(self):
        return self.v

    def getSFRotation(self):
        return self.v


class _Node:
    def __init__(self, nid, t=(0, 0, 0), r=(0, 0, 1, 0), parent=None):
        self.nid, self.parent = nid, parent
        self.fields = {"translation": _Field(t), "rotation": _Field(r)}

    def getId(self):
        return self.nid

    def getField(self, name):
        return self.fields[name]

    def getParentNode(self):
        return self.parent


class _Dev:
    def __init__(self, w=128, h=128, fov=1.2):
        self.w, self.h, self.fov = w, h, fov

    def getWidth(self):
        return self.w

    def getHeight(self):
        return self.h

    def getFov(self):
        return self.fov


class _Sup:
    def __init__(self, nodes, robot_node, world=None):
        self.nodes, self.robot_node, self.world = nodes, robot_node, world

    def getFromDevice(self, dev):
        return self.nodes.get(id(dev))

    def getFromDef(self, name):
        return None

    def getSelf(self):
        return self.robot_node

    def getWorldPath(self):
        return self.world


def test_live_pose_direct_child_from_supervisor():
    from src.lane_vision import camera_look_angles, camera_model_from_device

    robot = _Node(1, (5.0, 2.0, 0.0), (0, 0, 1, 0.7))  # world pose must NOT leak in
    cam_node = _Node(2, (0.1, 0.4, 0.8), (0, 1, 0, 0.9), parent=robot)
    dev = _Dev(64, 48, 1.0)
    model, src = camera_model_from_device(_Sup({id(dev): cam_node}, robot), dev, "nadir_left")
    assert src == "supervisor"
    assert model.translation == pytest.approx((0.1, 0.4, 0.8))
    assert (model.width, model.height, model.fov_rad) == (64, 48, 1.0)
    pitch, yaw = camera_look_angles(model)
    assert pitch == pytest.approx(0.9) and yaw == pytest.approx(0.0, abs=1e-9)


def test_live_pose_nested_transform_is_composed():
    from src.lane_vision import CameraModel, camera_model_from_device

    robot = _Node(1)
    # Transform at (0, 0.4, 0.7) yawed 90 deg left; camera 0.1 m along ITS +X, pitched 0.5
    tf = _Node(2, (0.0, 0.4, 0.7), (0, 0, 1, math.pi / 2), parent=robot)
    cam_node = _Node(3, (0.1, 0.0, 0.0), (0, 1, 0, 0.5), parent=tf)
    dev = _Dev()
    model, src = camera_model_from_device(_Sup({id(dev): cam_node}, robot), dev, "nadir_left")
    assert src == "supervisor"
    assert model.translation == pytest.approx((0.0, 0.5, 0.7), abs=1e-9)
    # same rays as an explicit yaw-then-pitch camera
    from src.lane_vision import _axis_angle_matrix, _mat_mul

    ref = CameraModel("ref", (0.0, 0.5, 0.7), (0, 0, 1, 0), 128, 128, 1.2)
    ref.R = _mat_mul(_axis_angle_matrix((0, 0, 1, math.pi / 2)), _axis_angle_matrix((0, 1, 0, 0.5)))
    for col, row in ((0, 0), (64, 64), (127, 100)):
        assert model.ray(col, row) == pytest.approx(ref.ray(col, row), abs=1e-9)


def test_live_pose_falls_back_to_loaded_wbt_then_constants(tmp_path, monkeypatch):
    import src.lane_vision as lv

    class NoSup:  # plain Robot: no supervisor calls
        def __init__(self, world):
            self.world = world

        def getWorldPath(self):
            return self.world

    wbt = tmp_path / "w.wbt"
    wbt.write_text(
        "Robot {\n children [\n Camera {\n translation 0.2 0.3 0.9\n rotation 0 1 0 1.0\n"
        ' name "nadir_left"\n width 128\n height 128\n fieldOfView 1.2\n }\n ]\n}\n'
    )
    model, src = lv.camera_model_from_device(NoSup(str(wbt)), _Dev(64, 64, 1.1), "nadir_left")
    assert src == "wbt w.wbt"
    assert model.translation == pytest.approx((0.2, 0.3, 0.9))
    assert (model.width, model.fov_rad) == (64, 1.1)  # optics from the device
    # loaded world unreadable -> repo world
    model, src = lv.camera_model_from_device(NoSup(str(tmp_path / "missing.wbt")), _Dev(), "nadir_left")
    assert src == "wbt butlerbot.wbt"
    # supervisor that cannot find the node, no world at all -> constants
    monkeypatch.setattr(lv, "_WBT_DEFAULT", str(tmp_path / "nope.wbt"))
    model, src = lv.camera_model_from_device(_Sup({}, _Node(1)), _Dev(), "nadir_left")
    assert src == "constants"
    assert model.translation == pytest.approx(lv.NADIR_LEFT_POSE["translation"])


def test_live_pose_camera_outside_robot_is_rejected():
    from src.lane_vision import supervisor_camera_pose

    stranger = _Node(9)
    cam_node = _Node(2, parent=stranger)
    dev = _Dev()
    with pytest.raises(LookupError):
        supervisor_camera_pose(_Sup({id(dev): cam_node}, _Node(1)), dev)


def test_matrix_axis_angle_roundtrip():
    from src.lane_vision import _axis_angle_matrix, matrix_to_axis_angle

    for rot in ((0, 1, 0, 0.9411), (0, 0, 1, 2.5), (1, 2, 3, 1.0), (0, 1, 0, math.pi - 1e-4), (0, 0, 1, 0.0)):
        R = _axis_angle_matrix(rot)
        R2 = _axis_angle_matrix(matrix_to_axis_angle(R))
        for i in range(3):
            assert R2[i] == pytest.approx(R[i], abs=1e-6)


# ------------------------------------------------ aim_camera.py


def _aim_module():
    import importlib.util
    from pathlib import Path

    p = Path(__file__).resolve().parent.parent / "scripts" / "aim_camera.py"
    spec = importlib.util.spec_from_file_location("aim_camera", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("mount,reach", [((0.03542, 0.41808, 0.70419), 1.96), ((0.0, 0.3, 1.0), 3.0), ((-0.2, 0.0, 0.5), 1.2)])
def test_aim_pitch_roundtrips_to_target_reach(mount, reach):
    from src.lane_vision import CameraModel, aim_pitch_for_reach

    p = aim_pitch_for_reach(mount, 1.2, 128, 128, reach)
    cam = CameraModel("t", mount, (0, 1, 0, p), 128, 128, 1.2)
    assert cam.pixel_to_ground(cam.cx, 0)[0] == pytest.approx(reach, abs=1e-6)


def test_aim_matches_closed_form_and_wbt():
    aim = _aim_module()
    r = aim.aim((0.03542, 0.41808, 0.70419), 1.2, 128, 128, 1.96)
    f = 64 / math.tan(0.6)
    closed = math.atan((0.70419 - 0.013) / (1.96 - 0.03542)) + math.atan(63.5 / f)
    assert r["pitch_rad"] == pytest.approx(closed, abs=1e-6)
    assert r["rotation_line"] == "rotation 0 1 0 0.9411"
    assert tuple(NADIR_LEFT_POSE["rotation"]) == (0.0, 1.0, 0.0, r["pitch_rad_rounded"])
    assert r["own_side_line"]  # left line at 1.30 m lane width is in view
    box = aim.box_inside_near(r["camera"], (0.01042, 0.41808, 0.72919), (0.05, 0.05, 0.05), 0.03)
    assert box["box_invisible"]
    centred = aim.aim((0.01042, 0.41808, 0.72919), 1.2, 128, 128, 1.96)
    assert not aim.box_inside_near(centred["camera"], (0.01042, 0.41808, 0.72919), (0.05,) * 3, 0.03)["box_invisible"]


def test_aim_cli_prints_rotation_line(capsys):
    aim = _aim_module()
    assert aim.main([]) == 0
    out = capsys.readouterr().out
    assert "rotation 0 1 0 0.9411" in out and "53.92 deg" in out
    assert "bottom row +0.058 m" in out and "top row    +1.960 m" in out


def test_fit_windows_follow_camera_coverage():
    from src.lane_vision import CameraModel, LaneTracker, fit_windows

    assert fit_windows(NADIR_CAMS.values()) == pytest.approx((0.6, 0.6, 0.25))
    tr = LaneTracker()
    assert (tr.near_max_x, tr.split_x, tr.far_min_x) == pytest.approx((0.6, 0.6, 0.25))
    # a camera whose bottom row lands 0.35 m ahead still gets a 0.4 m near window
    far = CameraModel("far", (0.0, 0.4, 0.7), (0, 1, 0, 0.5), 128, 128, 1.2)
    b = camera_coverage(far)["bottom_row_x_m"]
    near_max, split, far_min = fit_windows([far])
    assert b > 0.3
    assert near_max == pytest.approx(b + 0.4) and split <= near_max and far_min == pytest.approx(b)


# ---------------------------------------------------------------- toed-out (yawed) cameras


def _toed_out_pair(yaw_deg):
    from src.lane_vision import CameraModel, aim_pitch_for_reach, yaw_pitch_rotation

    t = NADIR_LEFT_POSE["translation"]
    y = math.radians(yaw_deg)
    p = aim_pitch_for_reach(t, 1.2, 128, 128, 1.96, yaw_rad=y)
    left = CameraModel("nadir_left", t, yaw_pitch_rotation(y, p), 128, 128, 1.2)
    right = CameraModel("nadir_right", (t[0], -t[1], t[2]), yaw_pitch_rotation(-y, p), 128, 128, 1.2)
    return {"nadir_left": left, "nadir_right": right}, p


@pytest.mark.parametrize("yaw_deg", [-15.0, 0.0, 10.0, 25.0])
def test_yaw_then_pitch_rotation_round_trips(yaw_deg):
    from src.lane_vision import CameraModel, camera_look_angles, yaw_pitch_rotation

    pitch = math.radians(53.0)
    rot = yaw_pitch_rotation(math.radians(yaw_deg), pitch)
    cam = CameraModel("c", NADIR_LEFT_POSE["translation"], rot, 128, 128, 1.2)
    p, y = camera_look_angles(cam)
    assert p == pytest.approx(pitch, abs=1e-9) and y == pytest.approx(math.radians(yaw_deg), abs=1e-9)
    # no roll: the image's horizontal axis (camera +Y) stays level
    assert cam.R[2][1] == pytest.approx(0.0, abs=1e-12)
    if yaw_deg == 0.0:
        assert rot == (0.0, 1.0, 0.0, pitch)


def test_mirrored_toed_out_pair_sees_mirror_images():
    cams, _ = _toed_out_pair(10.0)
    L, R = cams["nadir_left"], cams["nadir_right"]
    assert (R.rotation[0], R.rotation[1], R.rotation[2]) == pytest.approx((-L.rotation[0], L.rotation[1], -L.rotation[2]))
    assert R.rotation[3] == pytest.approx(L.rotation[3])
    for col, row in ((3.0, 5.0), (64.0, 64.0), (110.5, 127.0)):
        gl = L.pixel_to_ground(col, row)
        gr = R.pixel_to_ground(127.0 - col, row)
        assert gr[0] == pytest.approx(gl[0], abs=1e-9) and gr[1] == pytest.approx(-gl[1], abs=1e-9)
        back = L.ground_to_pixel(*gl)
        assert back == pytest.approx((col, row), abs=1e-6)
    assert L.pixel_to_ground(L.cx, 0)[0] == pytest.approx(1.96, abs=1e-6)  # pitch re-solved for the same reach


@pytest.mark.parametrize("yaw_deg", [10.0, 20.0])
def test_lane_tracker_works_with_toed_out_cameras(yaw_deg):
    """lane_vision is pose-general: the same fit with toed-out cameras, incl. a 1.8 m lane
    that the straight-ahead mount loses near the robot."""
    cams, _ = _toed_out_pair(yaw_deg)
    straight = [(-2 + 0.1 * i, 0.0) for i in range(80)]
    for hw, off, yaw in ((0.65, 0.05, 0.0), (0.65, -0.1, math.radians(5)), (0.9, 0.0, 0.0)):
        c = lane_polyline_robot(straight, (0.0, off), yaw)
        lines = [offset_polyline(c, hw), offset_polyline(c, -hw)]
        imgs = {n: (render_lane_bgra(cam, lines), 128, 128) for n, cam in cams.items()}
        est = LaneTracker(cams).process(imgs)
        assert est.valid
        assert est.offset_m == pytest.approx(off, abs=0.015)
        assert est.heading_rad == pytest.approx(yaw, abs=0.015)
        assert est.lane_width_m == pytest.approx(2 * hw, abs=0.03)


def test_parse_wbt_reads_a_yawed_camera(tmp_path):
    cams, p = _toed_out_pair(10.0)
    text = (ROOT_WORLD).read_text(encoding="utf-8")
    rl = " ".join(f"{v:.6f}" for v in cams["nadir_left"].rotation)
    rr = " ".join(f"{v:.6f}" for v in cams["nadir_right"].rotation)
    a, b = text.split("rotation 0 1 0 0.9411", 2)[0:2], text.split("rotation 0 1 0 0.9411", 2)[2]
    w = tmp_path / "yawed.wbt"
    w.write_text(a[0] + f"rotation {rl}" + a[1] + f"rotation {rr}" + b, encoding="utf-8")
    parsed = parse_wbt_cameras(str(w))
    for n in ("nadir_left", "nadir_right"):
        assert parsed[n]["rotation"] == pytest.approx(cams[n].rotation, abs=1e-5)


def test_aim_camera_yaw_prints_mirrored_rotation():
    import importlib.util
    import io
    from contextlib import redirect_stdout
    from pathlib import Path

    spec = importlib.util.spec_from_file_location("aim_camera", Path(__file__).resolve().parents[1] / "scripts" / "aim_camera.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    buf = io.StringIO()
    with redirect_stdout(buf):
        assert mod.main(["--yaw", "10"]) == 0
    out = buf.getvalue()
    assert "rotation -0.0858892 0.981718 0.169862 0.9512" in out
    assert "rotation 0.0858892 0.981718 -0.169862 0.9512" in out
    assert "53.65 deg down" in out
