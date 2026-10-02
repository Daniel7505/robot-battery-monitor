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
    # rotation ~ +Y by 1.1 rad turns camera +X to forward-and-down 63 deg
    assert math.degrees(math.asin(-dz / n)) == pytest.approx(63.0, abs=0.3)
    assert dx > 0
    # image col 0 is the camera's +Y (robot left), last col robot right
    assert cam.pixel_to_ground(0, 64)[1] > cam.pixel_to_ground(127, 64)[1]
    # row 0 is far, last row near
    assert cam.pixel_to_ground(64, 0)[0] > cam.pixel_to_ground(64, 127)[0]


def test_right_camera_is_mirror_of_left():
    # The wbt copies the left rotation instead of mirroring its tiny x/z axis
    # parts, so the right eye is a ~0.15 deg roll/yaw off a perfect mirror
    # (about 1 cm at the far corner). The model uses the exact wbt values.
    for col, row in ((10, 10), (64, 64), (120, 100)):
        lx, ly = NADIR_LEFT_CAM.pixel_to_ground(col, row)
        rx, ry = NADIR_RIGHT_CAM.pixel_to_ground(127 - col, row)
        assert lx == pytest.approx(rx, abs=0.03)
        assert ly == pytest.approx(-ry, abs=0.03)


def test_coverage_matches_observed_stripe_pixels():
    cov = camera_coverage(NADIR_LEFT_CAM)
    assert cov["bottom_row_x_m"] == pytest.approx(-0.55, abs=0.02)
    assert cov["top_row_x_m"] == pytest.approx(1.96, abs=0.03)
    lat_bottom = cov["bottom_m_per_px_lat_long"][0]
    lat_top = cov["top_m_per_px_lat_long"][0]
    # prior review saw the 6 cm stripe as ~5.2 px (bottom) and ~2.5 px (top)
    assert 0.06 / lat_bottom == pytest.approx(5.2, abs=0.3)
    assert 0.06 / lat_top == pytest.approx(2.5, abs=0.3)


def test_ground_pixel_roundtrip():
    for cam in NADIR_CAMS.values():
        for col, row in ((3.0, 5.0), (64.0, 64.0), (110.5, 120.0)):
            g = cam.pixel_to_ground(col, row)
            back = cam.ground_to_pixel(*g)
            assert back[0] == pytest.approx(col, abs=1e-6)
            assert back[1] == pytest.approx(row, abs=1e-6)


def test_yellow_mask_matches_lane_keep_score():
    rnd = random.Random(3)
    px = bytearray()
    want = []
    for _ in range(400):
        b, g, r = rnd.randrange(256), rnd.randrange(256), rnd.randrange(256)
        px += bytes((b, g, r, 255))
        want.append(1 if yellow_score((r / 255, g / 255, b / 255)) >= 0.22 else 0)
    assert list(yellow_mask(bytes(px), 20, 20, 0.22)) == want


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
    est = tr.process(frames(straight, (0.0, 0.12), 0.0, drop="left", hw=0.55))
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
    # direct children), camera rotation ~ (0 1 0 1.1) -> 63 deg down, FOV 1.2.
    import re
    from pathlib import Path

    wbt = (Path(__file__).resolve().parent.parent / "webots/worlds/butlerbot.wbt").read_text()
    robot = wbt[wbt.index("Robot {"):]
    rot = [float(v) for v in re.search(r"(?m)^\s*rotation\s+([-0-9. ]+)$", robot).group(1).split()]
    assert rot[3] == 0.0  # angle 0 = identity whatever the axis
    cov = camera_coverage(NADIR_LEFT_CAM)
    # bottom row ray is 1.1 + 0.597 rad down = past vertical -> lands behind the camera
    h = NADIR_LEFT_POSE["translation"][2] - 0.013
    assert cov["bottom_row_x_m"] == pytest.approx(-0.3868 + h / math.tan(1.1 + 0.5965), abs=0.01)
    assert cov["top_row_x_m"] == pytest.approx(-0.3868 + h / math.tan(1.1 - 0.5965), abs=0.02)
