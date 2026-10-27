"""Occupancy mapping, frontier exploration, and the apartment scorer.

Synthetic depth frames stand in for Webots. The kinematic room is a box
the test builds itself. Scoring tests are the ones that open the
apartment world and the ground-truth JSON.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from eval.apartment_gt_grid import load_solids, rasterize, spawn_pose
from eval.depth_scene import Box, render_xdepth
from eval.kinematic_runner import run_scene
from eval.score_apartment_map import score_labels, warp_to_ground_truth
from src.mapping.body import (
    ENCODER_RESOLUTION_RAD,
    OAK_DEPTH,
    ROBOT_RADIUS_M,
    WHEEL_RADIUS_M,
    WHEEL_TRACK_M,
    DepthCamera,
)
from src.mapping.depth_cloud import body_points, forward_clearance
from src.mapping.drive import pure_pursuit, wheel_speeds
from src.mapping.frontiers import astar, dilate_disk, frontier_mask, line_up_with_openings, select_goal, traversable
from src.mapping.scan_match import pose_correction, scan_fit
from src.mapping.grid import OccupancyGrid
from src.mapping.noise import noisy_yaw_rate
from src.mapping.odometry import WheelGyroOdometry

ROOT = Path(__file__).resolve().parents[1]


def _small_oak(width: int = 64, height: int = 40) -> DepthCamera:
    return DepthCamera(
        OAK_DEPTH.name,
        OAK_DEPTH.translation,
        OAK_DEPTH.rotation,
        width,
        height,
        OAK_DEPTH.fov_rad,
        OAK_DEPTH.min_range_m,
        OAK_DEPTH.max_range_m,
    )


def test_a_rest_calibration_holds_the_heading_for_ten_minutes():
    """The lookup bias is learned with the hubs still, then left alone.

    Learning it from moving hubs pulled a good rest estimate off, and
    the heading walked. After a few seconds stopped, 600 s of driving
    at +0.001 rad/s stays inside 5 degrees. A gyro spike while the hubs
    turn is not a new bias.
    """
    odom = WheelGyroOdometry()
    odom.update(0.0, 0.0, 0.0, 0.05)
    for _ in range(80):
        odom.update(0.0, 0.0, 0.001, 0.05)
    assert odom.bias_rad_s == pytest_approx(0.001, abs=2e-4)
    assert abs(odom.yaw_unwrapped_rad) < math.radians(0.5)
    left = 0.0
    step = 0.22 / WHEEL_RADIUS_M * 0.05
    held = odom.bias_rad_s
    for _ in range(40):
        left += step
        odom.update(left, left, 0.02, 0.05)
    assert abs(odom.bias_rad_s - held) < 1e-6
    for _ in range(12000):
        left += step
        odom.update(left, left, 0.001, 0.05)
    assert abs(math.degrees(odom.yaw_unwrapped_rad)) < 5.0


def test_wall_line_yaw_follows_the_mapped_wall_not_a_compass():
    """A depth line that is rotated relative to cells the robot already
    occupied is a heading error. The wall itself can sit at any angle.
    """
    from src.mapping.wall_line import wall_yaw_delta

    grid = OccupancyGrid()
    # A diagonal wall the robot mapped while its heading was still good.
    for i in range(40):
        cell = grid.cell_of(1.2 + i * 0.05, 0.4 + i * 0.05)
        assert cell is not None
        grid.log[cell] = grid.l_max
    hits = np.array([[1.2 + i * 0.05, 0.4 + i * 0.05, 0.5] for i in range(40)])
    aligned = wall_yaw_delta(grid, (0.0, 0.0, 0.0), hits[:, :2])
    assert abs(aligned) < 0.004
    # Same points in the body, but the pose yaw is 0.15 rad too far.
    # The mapped line is still the diagonal, so the correction is negative.
    wrong = wall_yaw_delta(grid, (0.0, 0.0, 0.15), hits[:, :2])
    assert wrong < -0.005
    # A short run of cells is a lattice, not a wall. The opening spin
    # used to nudge off that.
    short = OccupancyGrid()
    for i in range(12):
        cell = short.cell_of(1.2 + i * 0.04, 0.4)
        assert cell is not None
        short.log[cell] = short.l_max
    short_hits = np.array([[1.2 + i * 0.04, 0.4, 0.5] for i in range(12)])
    assert wall_yaw_delta(short, (0.0, 0.0, 0.15), short_hits[:, :2]) == 0.0


def test_rest_gyro_bias_does_not_walk_the_heading():
    """oak_gyro reads +0.001 rad/s while the hubs are still.

    Integrating that for the four minutes the robot sat in the doorway
    was about 14 degrees of false yaw. The bias is learned and not integrated.
    """
    odom = WheelGyroOdometry()
    odom.update(0.0, 0.0, 0.0, 0.05)
    for _ in range(400):
        odom.update(0.0, 0.0, 0.001, 0.05)
    assert abs(odom.pose()[2]) < math.radians(0.5)
    assert odom.bias_rad_s == pytest_approx(0.001, abs=3e-4)


def test_a_shove_while_the_hubs_are_still_does_not_poison_the_bias():
    """A press holds the hubs still and the gyro reads the shove.

    That reading is not a new rest bias. Folding it in used to leave
    the heading running away after the robot started moving again.
    """
    odom = WheelGyroOdometry()
    odom.update(0.0, 0.0, 0.0, 0.05)
    for _ in range(120):
        odom.update(0.0, 0.0, 0.001, 0.05)
    held = odom.bias_rad_s
    assert held == pytest_approx(0.001, abs=3e-4)
    for _ in range(40):
        odom.update(0.0, 0.0, 0.01, 0.05)
    assert abs(odom.bias_rad_s - held) < 1e-4
    assert abs(odom.yaw_unwrapped_rad) < math.radians(0.5)


def test_static_depth_with_spinning_wheels_is_slip_not_motion():
    """Encoders advance, the gyro stays at its rest bias, the wall does not get closer.

    That is the wheel-slip half of the Webots run: about a metre of false
    travel and a map warped enough to look finished. The pose must stay put,
    the failed frontier must be dropped, and the run must not end.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(48, 30)
    wall = Box(0.95, -1.2, 0.0, 1.08, 1.2, 2.2)
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), [wall])
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (2.0, 0.0)
    bot.path_xy = [(0.6, 0.0), (2.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    saw_reverse = False
    dt = 0.05
    # Wedged: hubs do not turn, gyro sits on its rest bias. Heading must not walk.
    for _ in range(300):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.001,
            depth=depth,
        )
        if cmd_l < -0.05 and cmd_r < -0.05:
            saw_reverse = True
    assert abs(bot.pose()[2]) < math.radians(1.0)
    # Slip: hubs follow the command, the camera does not move.
    for _ in range(120):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.001,
            depth=depth,
        )
        if cmd_l < -0.05 and cmd_r < -0.05:
            saw_reverse = True
        left += cmd_l * dt
        right += cmd_r * dt
    x, y, yaw = bot.pose()
    assert math.hypot(x, y) < 0.25
    assert abs(yaw) < math.radians(2.0)
    assert saw_reverse
    assert bot.slip_events >= 1
    assert bot._banned
    assert not bot.done


def test_wall_inside_min_range_with_spinning_wheels_is_not_motion():
    """Pressed against a surface the depth camera cannot see.

    Run 2 sat on the bath wall, inside the 0.35 m minimum, so the range
    image was empty. The encoders still counted metres. That travel must
    not move the pose, the drive must stop going forward, and the run
    must not finish.
    """
    from src.mapping.body import CRUISE_M_S
    from src.mapping.explore import Explorer

    cam = _small_oak(48, 30)
    depth = np.full((cam.height, cam.width), np.inf)
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(1.0, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    wheel = CRUISE_M_S / WHEEL_RADIUS_M
    dt = 0.05
    late_forward = 0
    saw_reverse = False
    for i in range(140):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.001,
            depth=depth,
        )
        if cmd_l < -0.05 and cmd_r < -0.05:
            saw_reverse = True
        if i > 10 and (cmd_l + cmd_r) > 1.0:
            late_forward += 1
        # Hubs keep turning forward, which is the slip, whatever was asked.
        left += wheel * dt
        right += wheel * dt
    x, y, yaw = bot.pose()
    assert math.hypot(x, y) < 0.20
    assert abs(yaw) < math.radians(2.0)
    assert not bot.done
    assert bot.slip_events >= 1
    assert bot._slip_debt_m >= 0.35
    assert saw_reverse
    assert late_forward == 0


def test_done_rejects_a_scan_that_fits_somewhere_else():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak())
    _fill(bot.grid, 0.85, -0.6, 1.15, 0.6, bot.grid.l_max)
    ys = np.linspace(-0.45, 0.45, 24)
    on_wall = np.column_stack((np.full(len(ys), 1.0), ys))
    bot._scan_xy = on_wall + np.array([0.80, 0.0])
    bot._scan_at = bot.t_s
    assert scan_fit(bot.grid, bot._scan_xy) == "disagree"
    bot._no_frontier()
    assert not bot.done
    assert bot.phase == "go"
    bot._scan_xy = on_wall
    bot._confident_after = 0.0
    bot._scan_at = bot.t_s
    assert scan_fit(bot.grid, bot._scan_xy) == "support"
    bot._no_frontier()
    assert not bot.done
    assert bot.phase == "spin"


def test_path_aims_through_the_middle_of_a_doorway():
    grid = OccupancyGrid()
    free = grid.l_min
    wall = grid.l_max
    # A closed room with one opening. Unknown space is only beyond that gap,
    # so the frontier is the doorway rather than the whole boundary.
    _fill(grid, 0.0, -0.85, 1.35, 0.85, free)
    _fill(grid, -0.25, -1.10, 1.50, -0.85, wall)
    _fill(grid, -0.25, 0.85, 1.50, 1.10, wall)
    _fill(grid, -0.25, -1.10, 0.0, 1.10, wall)
    _fill(grid, 1.05, -0.85, 1.30, -0.42, wall)
    _fill(grid, 1.05, 0.42, 1.30, 0.85, wall)
    hugging = [(0.3, 0.32), (0.8, 0.32), (1.15, 0.30), (1.32, 0.28)]
    lined = line_up_with_openings(grid, hugging)
    door = [p for p in lined if p[0] > 0.95]
    assert door
    assert max(abs(p[1]) for p in door) < 0.15
    chosen = select_goal(grid, (0.4, 0.30))
    assert chosen is not None
    _goal, path = chosen
    through = [p for p in path if p[0] > 0.9]
    assert through
    assert max(abs(p[1]) for p in through) < 0.20
    # The goal is past the opening, so arrival does not spin in the doorway.
    assert _goal[0] > 1.6


def test_narrow_opening_stays_passable_after_a_thick_jamb():
    """0.81 m between jambs, each smeared 0.15 m into the opening.

    That is a few contact marks on the frame. 0.26 m of inflation leaves
    no path. The tire is 0.195 m from the centre, and 0.20 m still has one.
    """

    def _path(radius: float):
        grid = OccupancyGrid()
        _fill(grid, -1.5, -0.2, 1.5, 1.1, grid.l_min)
        _fill(grid, -0.2, -0.5, 0.3, 0.15, grid.l_max)
        _fill(grid, -0.2, 0.66, 0.3, 1.4, grid.l_max)
        free = traversable(grid.labels(), grid.res, radius)
        start = grid.cell_of(-0.8, 0.40)
        goal = grid.cell_of(0.8, 0.40)
        assert start is not None and goal is not None
        return astar(free, start, goal, step_cost=grid.res)

    assert _path(0.26) is None
    assert _path(ROBOT_RADIUS_M) is not None


def test_a_doorway_ban_expires_and_does_not_finish_the_run():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak())
    bot.grid.stamp_footprint(0.0, 0.0, radius_m=1.2)
    cell = bot.grid.cell_of(1.1, 0.0)
    assert cell is not None
    bot.t_s = 5.0
    bot._blacklist(cell, radius=2, ttl_s=12.0)
    assert cell in bot._ban_set()
    bot.t_s = 17.1
    assert cell not in bot._ban_set()
    # A ban that covers the frontier is cleared and the goal comes back.
    # It must not declare the run finished.
    bot.t_s = 20.0
    bot._blacklist(bot.grid.cell_of(0.0, 0.0), radius=40, ttl_s=30.0)
    bot.phase = "go"
    bot._confident_after = 0.0
    bot._plan()
    assert not bot.done
    assert bot.goal_xy is not None


def test_done_requires_a_stopped_hold_after_the_frontiers_are_gone():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak())
    bot.grid.log[:] = bot.grid.l_min
    bot.phase = "go"
    bot._confident_after = 0.0
    bot._plan()
    assert not bot.done
    assert bot.phase == "spin"
    left = right = 0.0
    dt = 0.05
    for _ in range(500):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.70,
            depth=None,
        )
        left += cmd_l * dt
        right += cmd_r * dt
        if bot.done:
            break
    assert bot.done
    assert bot.phase == "done"


def _fill(grid: OccupancyGrid, x0: float, y0: float, x1: float, y1: float, log: float) -> None:
    corner = grid.cell_of(min(x0, x1), min(y0, y1))
    far = grid.cell_of(max(x0, x1), max(y0, y1))
    assert corner is not None and far is not None
    r0, r1 = sorted((corner[0], far[0]))
    c0, c1 = sorted((corner[1], far[1]))
    grid.log[r0 : r1 + 1, c0 : c1 + 1] = log


def test_straight_odometry_and_gyro_heading():
    odom = WheelGyroOdometry()
    ticks = 100  # exact multiple of the 0.0015 rad encoder
    step = ENCODER_RESOLUTION_RAD * ticks
    odom.update(0.0, 0.0, 0.0, 0.1)
    odom.update(step, step, 0.0, 0.1)
    x, y, yaw = odom.pose()
    assert x == pytest_approx(step * WHEEL_RADIUS_M, abs=1e-9)
    assert y == pytest_approx(0.0, abs=1e-9)
    assert yaw == pytest_approx(0.0, abs=1e-9)
    # A raw 1 rad reading is snapped to the nearest tick before it is integrated.
    snapped = WheelGyroOdometry()
    snapped.update(0.0, 0.0, 0.0, 0.1)
    snapped.update(1.0, 1.0, 0.0, 0.1)
    assert snapped.pose()[0] == pytest_approx(snapped.quantize(1.0) * WHEEL_RADIUS_M, abs=1e-9)
    turned = WheelGyroOdometry()
    turned.update(0.0, 0.0, 0.0, 0.1)
    turned.update(0.0, 0.0, 0.5, 2.0)
    assert turned.pose()[2] == pytest_approx(1.0, abs=1e-6)


def test_encoder_quantization_drops_sub_tick_motion():
    odom = WheelGyroOdometry()
    odom.update(0.0, 0.0, 0.0, 0.05)
    # Smaller than one tick on both hubs: the quantized angle stays 0.
    assert ENCODER_RESOLUTION_RAD == pytest_approx(0.0015)
    odom.update(0.0004, 0.0004, 0.0, 0.05)
    assert odom.pose()[0] == pytest_approx(0.0, abs=1e-9)
    odom.update(ENCODER_RESOLUTION_RAD, ENCODER_RESOLUTION_RAD, 0.0, 0.05)
    assert odom.pose()[0] == pytest_approx(ENCODER_RESOLUTION_RAD * WHEEL_RADIUS_M, abs=1e-9)


def test_center_pixel_lands_on_the_optical_axis():
    cam = DepthCamera("test", (0.0, 0.0, 0.5), (0.0, 1.0, 0.0, 0.0), 11, 11, 1.0, 0.2, 8.0)
    depth = np.full((11, 11), np.inf)
    depth[5, 5] = 2.0
    hits, floors, origin = body_points(cam, depth, stride=1)
    assert floors.shape[0] == 0
    assert hits.shape == (1, 3)
    assert hits[0, 0] == pytest_approx(2.0, abs=1e-6)
    assert abs(hits[0, 1]) < 1e-6
    assert hits[0, 2] == pytest_approx(0.5, abs=1e-6)
    assert origin[2] == pytest_approx(0.5, abs=1e-6)


def test_ray_marks_free_space_and_the_hit():
    grid = OccupancyGrid()
    origin = (0.0, 0.0)
    # Two views: one return is not enough to cross the log-odds threshold.
    for _ in range(2):
        grid.integrate(origin, np.array([[1.0, 0.0]]), np.zeros((0, 2)))
    labels = grid.labels()
    hit = grid.cell_of(1.0, 0.0)
    mid = grid.cell_of(0.4, 0.0)
    assert hit is not None and labels[hit] == 1
    assert mid is not None and labels[mid] == 0


def test_pitch_down_still_sees_a_wall():
    cam = _small_oak(32, 20)
    wall = Box(1.55, -1.0, 0.0, 1.7, 1.0, 2.4)
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), [wall])
    hits, floors, _origin = body_points(cam, depth, stride=1)
    assert hits.shape[0] > 10
    assert floors.shape[0] > 10
    assert float(np.median(hits[:, 0])) == pytest_approx(1.6, abs=0.15)


def test_frontier_rejects_a_speck_and_scores_a_real_edge():
    grid = OccupancyGrid()
    # A free disk with unknown outside produces a frontier ring.
    grid.stamp_footprint(0.0, 0.0, radius_m=1.2)
    chosen = select_goal(grid, (0.0, 0.0))
    assert chosen is not None
    labels = grid.labels()
    assert int(np.count_nonzero(frontier_mask(labels))) > 6
    # Seal the ring with an occupied annulus and the long frontiers go away.
    rows, cols = np.indices(labels.shape)
    x = grid.origin[0] + (cols + 0.5) * grid.res
    y = grid.origin[1] + (rows + 0.5) * grid.res
    ring = (np.hypot(x, y) > 1.15) & (np.hypot(x, y) < 1.45)
    grid.log[ring] = grid.l_max
    assert select_goal(grid, (0.0, 0.0)) is None


def test_dilation_blocks_a_neighbour_cell():
    mask = np.zeros((21, 21), dtype=bool)
    mask[10, 10] = True
    fat = dilate_disk(mask, 0.22, 0.05)
    assert fat[10, 10]
    assert fat[10, 14]  # 4 cells * 0.05 = 0.20 m, inside 0.22
    assert not fat[10, 16]  # 0.30 m


def test_a_door_jamb_is_not_the_nose_clearance():
    # Centred in an 0.81 m opening, 0.6 m before the jambs. The far wall
    # shows through the gap. The jambs are shoulders, not a wall ahead.
    hits = np.array(
        [
            [0.60, 0.40, 0.5],
            [0.60, -0.40, 0.5],
            [2.40, 0.00, 0.5],
        ]
    )
    nose, left, right = forward_clearance(hits)
    assert nose == pytest_approx(2.40)
    assert left == pytest_approx(0.60)
    assert right == pytest_approx(0.60)
    wall = np.array([[0.65, 0.0, 0.5]])
    nose, left, right = forward_clearance(wall)
    assert nose == pytest_approx(0.65)
    assert left == pytest_approx(99.0)
    assert right == pytest_approx(99.0)


def test_depth_scene_motion_ignores_a_pure_yaw():
    """A turn slides the image. The residual after that shift is translation."""
    from src.mapping.depth_cloud import depth_scene_motion_m

    cam = _small_oak(80, 50)
    boxes = [Box(1.8, -2.0, 0.0, 1.9, 2.0, 2.2), Box(0.4, 0.35, 0.0, 1.9, 0.55, 2.2)]
    still = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    yawed = render_xdepth(cam, (0.0, 0.0, 0.10), boxes)
    forward = render_xdepth(cam, (0.08, 0.0, 0.0), boxes)
    yaw_motion = depth_scene_motion_m(still, yawed, cam, 0.10)
    drive_motion = depth_scene_motion_m(still, forward, cam, 0.0)
    assert yaw_motion is not None and drive_motion is not None
    assert yaw_motion < 0.03
    assert drive_motion > 0.05


def test_a_still_scene_with_turning_wheels_freezes_translation():
    """Depth, floor, and visual odometry all still while the hubs are commanded.

    That is the door-G wedge: the tires scrubbed, the forward view still
    showed the room, and the odometry walked off. The pose goes back to
    where the stretch started.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(1.5, -1.5, 0.0, 1.6, 1.5, 2.0), Box(0.5, 0.30, 0.0, 1.6, 0.48, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[8:18, 40:52] = 30.0
    rgb[22:34, 24:36] = 180.0
    rng = np.random.default_rng(7)
    floor = rng.normal(40.0, 25.0, size=(32, 32))
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    bot._confident_after = 0.0
    left = right = 0.0
    wheel = 0.22 / WHEEL_RADIUS_M
    dt = 0.05
    for _ in range(16):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.0,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 0.40, "us_right": 0.40},
        )
        # Hubs follow a forward command whether or not the planner asked.
        if cmd_l + cmd_r > 0.4:
            left += wheel * dt
            right += wheel * dt
        else:
            left += cmd_l * dt
            right += cmd_r * dt
    assert math.hypot(*bot.pose()[:2]) < 0.08
    assert bot._pin_until > 0.0


def test_a_still_scene_freezes_yaw_while_the_wheels_spin():
    """Depth, floor, and vision unchanged. The gyro still reads its rest offset.

    The hubs are turning, so the old still-hubs rule does not apply, and
    integrating that offset is the heading walk while the body sits.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(1.5, -1.5, 0.0, 1.6, 1.5, 2.0), Box(0.5, 0.30, 0.0, 1.6, 0.48, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[8:18, 40:52] = 30.0
    rgb[22:34, 24:36] = 180.0
    rng = np.random.default_rng(7)
    floor = rng.normal(40.0, 25.0, size=(32, 32))
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    wheel = 0.22 / WHEEL_RADIUS_M
    dt = 0.05
    for _ in range(80):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.008,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 0.40, "us_right": 0.40},
        )
        if cmd_l + cmd_r > 0.4:
            left += wheel * dt
            right += wheel * dt
        else:
            left += cmd_l * dt
            right += cmd_r * dt
    assert bot.zupt_events >= 1
    assert abs(bot.pose()[2]) < math.radians(0.5)
    assert math.hypot(*bot.pose()[:2]) < 0.08


def test_a_soft_floor_creep_is_not_a_zero_velocity_update():
    """A low-frequency grain still looks similar after a few pixels of MAD.

    Floor flow against the anchor is the translation check. Depth and the
    forward picture stay fixed, which is a flat wall ahead while the shell
    creeps. That must not freeze the pose.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(2.4, -1.2, 0.0, 2.6, 1.2, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[22:34, 24:36] = 40.0
    yy, xx = np.mgrid[0:32, 0:32]
    base = 80.0 + 28.0 * np.sin(xx / 4.0) + 12.0 * np.cos(yy / 5.0)
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    wheel = 0.12 / WHEEL_RADIUS_M
    dt = 0.05
    for i in range(24):
        floor = np.roll(base, i * 4, axis=1)
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.0,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 1.5, "us_right": 1.5},
        )
        if cmd_l + cmd_r > 0.4:
            left += wheel * dt
            right += wheel * dt
        else:
            left += cmd_l * dt
            right += cmd_r * dt
    assert bot.zupt_events == 0
    assert bot.pose()[0] > 0.05


def test_a_stuck_press_with_an_open_nose_backs_off():
    """The nose looks through the opening. The tires are still scrubbing.

    Side sonar never sees a jamb here. The still pictures are the press,
    and the reverse has to start before the contact has lasted two seconds.
    A carpet-ramp pitch is not a lifted wheel.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(2.4, -1.2, 0.0, 2.6, 1.2, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[8:18, 40:52] = 30.0
    rgb[22:34, 24:36] = 180.0
    rng = np.random.default_rng(4)
    floor = rng.normal(40.0, 25.0, size=(32, 32))
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    wheel = 0.22 / WHEEL_RADIUS_M
    dt = 0.05
    saw_reverse = False
    # Pitch of the carpet ramp. The casters are not unloaded.
    for _ in range(24):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.001,
            pitch_rad=0.05,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 1.5, "us_right": 1.5},
        )
        if cmd_l < -0.05 and cmd_r < -0.05:
            saw_reverse = True
        if cmd_l + cmd_r > 0.4:
            left += wheel * dt
            right += wheel * dt
        else:
            left += cmd_l * dt
            right += cmd_r * dt
    assert bot.zupt_events >= 1
    assert bot.wheels_lifted == 0
    assert saw_reverse
    assert abs(bot.pose()[2]) < math.radians(1.0)
    assert math.hypot(*bot.pose()[:2]) < 0.08


def test_a_still_floor_freezes_translation_while_the_view_turns():
    """The pile is still and the forward picture is not. That is a scrub.

    Depth and vision used to veto the floor, so the wheels kept writing
    travel while the shell sat on the lip. Translation freezes and the
    drive backs off. Yaw does not: the gyro is actually turning.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(2.4, -1.2, 0.0, 2.6, 1.2, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    noise = np.random.default_rng(5).normal(0.0, 1.0, size=(32, 32))
    floor = 40.0 + noise * (3.2 / float(np.std(noise)))
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[22:34, 24:36] = 40.0
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    bot._fusion.floor.noise_px = 0.0
    left = right = 0.0
    wheel = 0.22 / WHEEL_RADIUS_M
    dt = 0.05
    saw_reverse = False
    for i in range(28):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.04,
            depth=depth + 0.2 * (i % 2),
            floor=floor,
            rgb=np.roll(rgb, i, axis=1),
            proximity={"us_left": 1.5, "us_right": 1.5},
        )
        if cmd_l < -0.05 and cmd_r < -0.05:
            saw_reverse = True
        if cmd_l + cmd_r > 0.4:
            left += wheel * dt
            right += wheel * dt
        else:
            left += cmd_l * dt
            right += cmd_r * dt
    assert bot.zupt_events >= 1
    assert saw_reverse
    assert math.hypot(bot.pose()[0], bot.pose()[1]) < 0.08
    assert abs(bot.pose()[2]) > 0.015


def test_a_pivot_about_the_floor_camera_does_not_rewind_yaw():
    """A turn in place is not a stuck floor. The camera is on the axis.

    The door run rewound 0.50 rad in one sample while the shell was
    pivoting, and the heading stayed 44° off after that. The gyro had
    the turn. The still picture must not put the old heading back.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(2.4, -1.2, 0.0, 2.6, 1.2, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    noise = np.random.default_rng(5).normal(0.0, 1.0, size=(32, 32))
    floor = 40.0 + noise * (3.2 / float(np.std(noise)))
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    bot._fusion.floor.noise_px = 0.0
    left = right = 0.0
    yaw = 0.0
    dt = 0.05
    rate = 0.80
    for _ in range(24):
        yaw += rate * dt
        left -= rate * (WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M * dt
        right += rate * (WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M * dt
        bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=rate,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 1.5, "us_right": 1.5},
        )
    assert bot.phase == "go"
    assert bot.zupt_events == 0
    assert abs(bot.pose()[2]) > 0.5


def test_a_pure_pivot_with_a_still_floor_freezes_translation_only():
    """The floor image does not move about the spin axis. The gyro does.

    Depth and the forward view stay still too, which used to rewind yaw
    on every sample. The turn never reached the old 0.12 rad release, so
    the heading stayed put. Translation is the part that freezes.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(2.4, -1.2, 0.0, 2.6, 1.2, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[8:18, 40:52] = 30.0
    rgb[22:34, 24:36] = 180.0
    rng = np.random.default_rng(11)
    floor = rng.normal(40.0, 25.0, size=(32, 32))
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    bot._fusion.floor.noise_px = 0.0
    left = right = 0.0
    dt = 0.05
    # Above the 0.02 rad/s rest gate, and slow enough that 0.4 s of yaw
    # stays under the old 0.12 rad rewind release.
    rate = 0.15
    for _ in range(50):
        left -= rate * (WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M * dt
        right += rate * (WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M * dt
        bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=rate,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 1.5, "us_right": 1.5},
        )
    assert bot.zupt_events >= 1
    assert math.hypot(bot.pose()[0], bot.pose()[1]) < 0.05
    assert abs(bot.pose()[2]) > 0.20


def test_a_real_spin_is_not_stuck_when_the_floor_picture_stays():
    """The survey rotates about the downward camera, so that picture barely translates.

    Treating the match as a stall aborted the spin at 0.8 s on every seed.
    The gyro is turning, so the shell is not stuck.
    """
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(2.4, -1.2, 0.0, 2.6, 1.2, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    noise = np.random.default_rng(5).normal(0.0, 1.0, size=(32, 32))
    floor = 40.0 + noise * (3.2 / float(np.std(noise)))
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "spin"
    bot._spin_start = 0.0
    bot._fusion.floor.noise_px = 0.0
    left = right = 0.0
    yaw = 0.0
    dt = 0.05
    rate = 0.70
    for _ in range(24):
        yaw += rate * dt
        left -= rate * (WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M * dt
        right += rate * (WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M * dt
        bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=rate,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 1.5, "us_right": 1.5},
        )
    assert bot.phase == "spin"
    assert abs(bot.pose()[2]) > 0.4


def test_a_moving_floor_is_not_a_zero_velocity_update():
    """The floor picture shifting is the body moving. Yaw and travel stay."""
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(1.5, -1.5, 0.0, 1.6, 1.5, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[22:34, 24:36] = 40.0
    rng = np.random.default_rng(3)
    floor = rng.normal(40.0, 25.0, size=(32, 32))
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    wheel = 0.22 / WHEEL_RADIUS_M
    dt = 0.05
    for i in range(30):
        shifted = np.roll(floor, i * 3, axis=1)
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.0,
            depth=depth,
            floor=shifted,
            rgb=rgb,
            proximity={"us_left": 0.40, "us_right": 0.40},
        )
        if cmd_l + cmd_r > 0.4:
            left += wheel * dt
            right += wheel * dt
        else:
            left += cmd_l * dt
            right += cmd_r * dt
    assert bot.zupt_events == 0


def test_lifted_wheels_are_a_zero_velocity_update():
    """Casters take the load, the drive wheels spin, the pictures do not move."""
    from src.mapping.explore import Explorer

    cam = _small_oak(64, 40)
    boxes = [Box(1.5, -1.5, 0.0, 1.6, 1.5, 2.0), Box(0.5, 0.30, 0.0, 1.6, 0.48, 2.0)]
    depth = render_xdepth(cam, (0.0, 0.0, 0.0), boxes)
    rgb = np.zeros(depth.shape, dtype=np.float64)
    rgb[8:18, 8:18] = 220.0
    rgb[8:18, 40:52] = 30.0
    rgb[22:34, 24:36] = 180.0
    rng = np.random.default_rng(9)
    floor = rng.normal(40.0, 25.0, size=(32, 32))
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(0.8, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    dt = 0.05
    for _ in range(30):
        bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.001,
            pitch_rad=0.0,
            depth=depth,
            floor=floor,
            rgb=rgb,
        )
    assert bot._pitch_rest == pytest_approx(0.0, abs=1e-6)
    wheel = 0.22 / WHEEL_RADIUS_M
    saw_reverse = False
    for _ in range(40):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.001,
            pitch_rad=0.16,
            depth=depth,
            floor=floor,
            rgb=rgb,
            proximity={"us_left": 0.40, "us_right": 0.40},
        )
        if cmd_l < -0.05 and cmd_r < -0.05:
            saw_reverse = True
        if cmd_l + cmd_r > 0.4:
            left += wheel * dt
            right += wheel * dt
        else:
            left += cmd_l * dt
            right += cmd_r * dt
    assert bot.wheels_lifted >= 1
    assert saw_reverse
    assert abs(bot.pose()[2]) < math.radians(1.0)


def test_a_side_press_backs_off_the_jamb_instead_of_driving_along_it():
    """The left sonar stays on the frame. Reverse has to yaw away from it.

    A straight backup counted wheel radians and the tire never left the
    jamb. The command here is a reverse with the right hub faster, which
    is a yaw to the right, off the left wall.
    """
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.goal_xy = (2.0, 0.0)
    bot.path_xy = [(0.6, 0.0), (2.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 0.0
    left = right = 0.0
    dt = 0.05
    saw_escape = False
    for _ in range(20):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.0,
            proximity={"us_left": 0.04, "us_right": 0.40},
        )
        left += cmd_l * dt
        right += cmd_r * dt
        if cmd_l < -0.05 and cmd_r < cmd_l - 0.3:
            saw_escape = True
    assert saw_escape
    assert bot._side_escape_sign > 0.0


def test_a_door_is_approached_slowly_on_the_centre_line():
    from src.mapping.bubble import BubbleView
    from src.mapping.explore import DOOR_SPEED_M_S, Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.t_s = 5.0
    bot.goal_xy = (2.5, 0.0)
    bot.path_xy = [(0.6, 0.0), (2.5, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 5.0
    bot._last_clear = (1.6, 0.70, 1.00)
    # Left sonar closer: the centre line is to the right, so yaw is negative.
    bot._bubble_view = BubbleView(1.6, 0.22, 0.40, None, 0.22)
    bot._bubble_gap = 1.6
    bot._min_gap = 0.22
    cmd_l, cmd_r = bot._go(False, 0.28, 0.70, 1.00)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    w = (cmd_r - cmd_l) * WHEEL_RADIUS_M / WHEEL_TRACK_M
    assert 0.02 < v <= DOOR_SPEED_M_S + 1e-6
    assert w < -0.05


def test_open_floor_arc_does_not_clip_and_an_inside_wall_does():
    """A straight pass misses a wall that sits only on the inside of a turn."""
    from src.mapping.footprint import fit_motion, footprint_clips

    grid = OccupancyGrid()
    pose = (0.0, 0.0, 0.0)
    assert not footprint_clips(grid, pose, 0.22, 0.0)
    assert not footprint_clips(grid, pose, 0.22, 0.35)
    # Left turn swings the nose to +y. The wall is outside the straight shell.
    _fill(grid, 0.05, 0.34, 0.70, 0.70, grid.l_max)
    assert not footprint_clips(grid, pose, 0.18, 0.0)
    assert footprint_clips(grid, pose, 0.18, 0.9)
    v, w, kind = fit_motion(grid, pose, 0.18, 0.9, 0.0)
    assert kind in {"wider", "straight", "align"}
    assert not footprint_clips(grid, pose, v, w)


def test_a_yawed_door_approach_lines_up_then_drives_straight():
    """Outside a mapped opening, face the centre line before driving through.

    The opening is the robot's own occupied cells. An arc swings the nose
    into a jamb; the straight centred pass does not.
    """
    from src.mapping.explore import DOOR_SPEED_M_S, Explorer
    from src.mapping.footprint import footprint_clips

    grid = OccupancyGrid()
    # 0.81 m opening along +x. Jambs start 0.405 m off the centre line.
    _fill(grid, 1.05, 0.405, 1.25, 1.10, grid.l_max)
    _fill(grid, 1.05, -1.10, 1.25, -0.405, grid.l_max)
    bot = Explorer(camera=_small_oak(), fusion=False, floor=False, grid=grid)
    bot.phase = "go"
    bot.t_s = 5.0
    bot.odometry.x_m = 0.55
    bot.odometry.y_m = 0.0
    bot.odometry.yaw_rad = 0.55
    bot.odometry.yaw_unwrapped_rad = 0.55
    bot.goal_xy = (2.4, 0.0)
    bot.path_xy = [(0.55, 0.0), (1.40, 0.0), (2.4, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 5.0
    bot._last_clear = (1.6, 0.62, 0.70)
    bot._bubble_gap = 1.6
    bot._min_gap = 0.62
    cmd_l, cmd_r = bot._go(False, 0.28, 0.62, 0.70)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    w = (cmd_r - cmd_l) * WHEEL_RADIUS_M / WHEEL_TRACK_M
    assert abs(v) < 0.02
    assert w < -0.40
    assert bot.sweep_aligned >= 1

    bot.odometry.yaw_rad = 0.0
    bot.odometry.yaw_unwrapped_rad = 0.0
    bot.sweep_checks = 0
    cmd_l, cmd_r = bot._go(False, 0.28, 0.62, 0.70)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    w = (cmd_r - cmd_l) * WHEEL_RADIUS_M / WHEEL_TRACK_M
    assert abs(v - DOOR_SPEED_M_S) < 1e-6
    assert abs(w) < 0.05
    # Still yawed a second and a half later: stop spinning and creep.
    bot.odometry.yaw_rad = 0.55
    bot.odometry.yaw_unwrapped_rad = 0.55
    bot._approach_since = bot.t_s - 1.5
    cmd_l, cmd_r = bot._go(False, 0.28, 0.62, 0.70)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.05
    # Close enough that the shorter horizon still reaches the jamb.
    near = (0.90, 0.0, 0.0)
    assert not footprint_clips(grid, near, DOOR_SPEED_M_S, 0.0)
    # A gentle arc reaches the jamb. A spin in place never enters the opening.
    assert footprint_clips(grid, near, DOOR_SPEED_M_S, 0.40)
    # A heading that crosses the align threshold must not restart the spin.
    bot._approach_since = None
    bot.odometry.yaw_rad = 0.55
    bot.odometry.yaw_unwrapped_rad = 0.55
    bot._go(False, 0.28, 0.62, 0.70)
    bot.t_s += 0.4
    bot.odometry.yaw_rad = 0.0
    bot.odometry.yaw_unwrapped_rad = 0.0
    bot._go(False, 0.28, 0.62, 0.70)
    bot.t_s += 1.0
    bot.odometry.yaw_rad = 0.55
    bot.odometry.yaw_unwrapped_rad = 0.55
    cmd_l, cmd_r = bot._go(False, 0.28, 0.62, 0.70)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.05


def test_a_flicker_off_the_doorway_band_does_not_restart_the_align():
    """One depth frame with a shoulder past 1.20 m is not a new approach.

    Resetting the align clock on that frame held the shell spinning on
    the bedroom threshold until the goal was replaced.
    """
    from src.mapping.explore import DOOR_SPEED_M_S, Explorer

    grid = OccupancyGrid()
    _fill(grid, 1.05, 0.405, 1.25, 1.10, grid.l_max)
    _fill(grid, 1.05, -1.10, 1.25, -0.405, grid.l_max)
    bot = Explorer(camera=_small_oak(), fusion=False, floor=False, grid=grid)
    bot.phase = "go"
    bot.t_s = 5.0
    bot.odometry.x_m = 0.55
    bot.odometry.yaw_rad = 0.55
    bot.odometry.yaw_unwrapped_rad = 0.55
    bot.goal_xy = (2.4, 0.0)
    bot.path_xy = [(0.55, 0.0), (1.40, 0.0), (2.4, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 5.0
    bot._last_clear = (1.6, 0.62, 0.70)
    bot._bubble_gap = 1.6
    bot._min_gap = 0.62
    bot._go(False, 0.28, 0.62, 0.70)
    started = bot._approach_since
    assert started is not None
    bot.t_s += 0.2
    bot._last_clear = (1.6, 0.62, 1.40)
    bot._go(False, 0.28, 0.62, 1.40)
    assert bot._approach_since == started
    bot.t_s += 1.1
    bot.odometry.yaw_rad = 0.55
    bot.odometry.yaw_unwrapped_rad = 0.55
    bot._last_clear = (1.6, 0.62, 0.70)
    cmd_l, cmd_r = bot._go(False, 0.28, 0.62, 0.70)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.05
    assert v <= DOOR_SPEED_M_S + 1e-6


def test_an_align_spin_that_loses_the_shoulders_keeps_the_clock():
    """The align yaw swings one shoulder past 1.20 m. That is not a new doorway.

    The hall is narrow enough that the corridor reads as an approach.
    Clearing the clock after 0.40 s of that swing restarted the spin
    for the whole bedroom run.
    """
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.t_s = 5.0
    bot.odometry.x_m = 0.55
    bot.odometry.yaw_rad = 0.55
    bot.odometry.yaw_unwrapped_rad = 0.55
    bot.goal_xy = (2.4, 0.0)
    bot.path_xy = [(0.55, 0.0), (1.40, 0.0), (2.4, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 5.0
    bot._last_clear = (1.6, 0.62, 0.70)
    bot._bubble_gap = 1.6
    bot._min_gap = 0.62
    bot._go(False, 0.28, 0.62, 0.70)
    started = bot._approach_since
    assert started is not None
    bot.t_s += 0.8
    bot._last_clear = (1.6, 0.62, 1.40)
    bot._go(False, 0.28, 0.62, 1.40)
    assert bot._approach_since == started
    bot.t_s += 1.3
    bot._go(False, 0.28, 0.62, 1.40)
    assert bot._approach_since is None


def test_a_smeared_door_still_creeps_after_the_align_window():
    """Occupied cells in the opening do not keep the shell spinning.

    After the align window, both shoulders still show the gap and the
    nose is open. The command is the slow straight creep.
    """
    from src.mapping.explore import DOOR_SPEED_M_S, Explorer

    grid = OccupancyGrid()
    _fill(grid, 0.85, -0.08, 1.05, 0.08, grid.l_max)
    bot = Explorer(camera=_small_oak(), fusion=False, floor=False, grid=grid)
    bot.phase = "go"
    bot.t_s = 8.0
    bot.odometry.x_m = 0.40
    bot.goal_xy = (2.0, 0.0)
    bot.path_xy = [(0.40, 0.0), (1.6, 0.0), (2.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 8.0
    bot._approach_since = 6.0
    bot._last_clear = (0.90, 0.50, 1.05)
    bot._bubble_gap = 0.90
    bot._min_gap = 0.50
    cmd_l, cmd_r = bot._go(False, 0.28, 0.50, 1.05)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    w = (cmd_r - cmd_l) * WHEEL_RADIUS_M / WHEEL_TRACK_M
    assert abs(v - DOOR_SPEED_M_S) < 1e-6
    assert abs(w) < 0.30


def test_a_rear_press_drives_forward_off_the_wall():
    """The tail is on the wall and the nose sees the room. Reverse digs in."""
    from src.mapping.bubble import BubbleView
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.t_s = 5.0
    bot.goal_xy = (2.0, 0.0)
    bot.path_xy = [(1.0, 0.0), (2.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 5.0
    bot._bubble_view = BubbleView(1.4, 0.40, 0.40, 0.02, 0.02)
    bot._bubble_gap = 1.4
    bot._min_gap = 0.02
    cmd_l, cmd_r = bot._go(True, 0.0)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.05


def test_a_tail_gap_recovery_refuses_still_drives_forward():
    """The no-reverse band used to command zero wheels.

    Recovery will not back up when the tail is the tight face inside
    0.12 m. The escape used to require 0.10 m, so 0.11 m did neither.
    The sweep had already stopped the plan. Live depth here is the
    south-seed freeze: an open nose the creep gate does not accept.
    """
    from src.mapping.bubble import BubbleView
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.t_s = 30.0
    bot.odometry.x_m = 1.0
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(1.0, 0.0), (3.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 30.0
    bot._progress_t = 0.0
    bot._last_clear = (0.75, 1.05, 0.52)
    bot._bubble_view = BubbleView(0.32, 0.50, 0.40, 0.11, 0.11)
    bot._bubble_gap = 0.32
    bot._min_gap = 0.11
    assert bot._gap_ahead() == ""
    bot._start_recovery()
    assert bot._reverse_left_m == 0.0
    cmd_l, cmd_r = bot._go(False, 0.0, 1.05, 0.52)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.05
    # Just outside the band, the escape does not take over.
    bot._bubble_view = BubbleView(0.32, 0.50, 0.40, 0.13, 0.13)
    assert bot._leave_rear() is None


def test_an_unchanged_view_with_no_corners_is_still():
    """A flat wall has nothing to track. The picture not moving is the reading."""
    from src.mapping.visual_odometry import VisualOdometry

    cam = _small_oak(48, 32)
    depth = np.full((cam.height, cam.width), 1.6)
    # A smooth gradient has no corner to track. The unchanged frame is the vote.
    rgb = np.tile(np.linspace(20.0, 220.0, cam.width), (cam.height, 1))
    vo = VisualOdometry()
    assert vo.measure(depth, cam, rgb) is None
    still = vo.measure(depth, cam, rgb)
    assert still is not None and still.ok and still.reason == "still"
    assert abs(still.dx_m) < 1e-6 and abs(still.dy_m) < 1e-6
    shifted = np.roll(rgb, 8, axis=1)
    moved = vo.measure(depth, cam, shifted)
    assert moved is None or moved.reason != "still"


def test_an_uncommitted_goal_is_not_held():
    """A young goal can be replaced. Only a lined-up crossing is frozen."""
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.t_s = 10.0
    bot.goal_xy = (3.0, 0.5)
    bot.path_xy = [(1.0, 0.0), (3.0, 0.5)]
    bot._goal_since = 8.0
    bot._commit_until = -1.0
    assert bot._hold_goal() is False
    bot._commit_until = 20.0
    assert bot._hold_goal() is True


def test_a_spin_command_does_not_integrate_translation():
    """Encoder travel during a commanded spin is scrub, not axle motion."""
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.step(dt_s=0.008, left_wheel_rad=0.0, right_wheel_rad=0.0, yaw_rate_rad_s=0.0)
    bot._commanded = (-1.49, 1.49)
    bot.step(dt_s=0.008, left_wheel_rad=0.25, right_wheel_rad=0.25, yaw_rate_rad_s=0.0)
    assert abs(bot.odometry.x_m) < 1e-4
    assert abs(bot.odometry.y_m) < 1e-4


def test_radar_furniture_does_not_spin_an_open_shell():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.t_s = 5.0
    bot.goal_xy = (2.0, 0.0)
    bot.path_xy = [(1.0, 0.0), (2.0, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 5.0
    bot.step(dt_s=0.008, left_wheel_rad=0.0, right_wheel_rad=0.0, yaw_rate_rad_s=0.0, radar_blocked=True)
    cmd_l, cmd_r = bot.step(
        dt_s=0.008,
        left_wheel_rad=0.0,
        right_wheel_rad=0.0,
        yaw_rate_rad_s=0.0,
        radar_blocked=True,
    )
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.05


def test_visual_translation_does_not_write_yaw():
    """Corner tracks may move x and y. Heading stays on the gyro."""
    from src.mapping.fuse import MotionFusion

    fuse = MotionFusion(floor=False, visual=True)
    odom = WheelGyroOdometry()
    interval = fuse._visual
    interval.add(0.02, 0.02, 0.0)
    note = fuse._apply_shift(
        odom,
        interval,
        0.02,
        0.004,
        0.03,
        ok=True,
        quality=0.8,
        kind="visual",
        var_xy=0.008,
        var_yaw=0.01,
        pixel_shift=5.0,
        expect_px=5.0,
    )
    assert note == "visual"
    assert fuse.visual_applied_yaw == 0.0
    assert abs(odom.yaw_unwrapped_rad) < 1e-9
    assert abs(odom.x_m) > 0.0 or abs(odom.y_m) > 0.0


def test_a_still_sample_does_not_pull_yaw():
    from src.mapping.fuse import MotionFusion
    from src.mapping.odometry import WheelGyroOdometry

    cam = _small_oak(48, 32)
    depth = np.full((cam.height, cam.width), 1.6)
    rgb = np.tile(np.linspace(20.0, 220.0, cam.width), (cam.height, 1))
    fuse = MotionFusion(floor=False, visual=True)
    odom = WheelGyroOdometry()
    fuse.observe(odom, ds_raw_m=0.02, ds_applied_m=0.02, dyaw_rad=0.0, depth=depth, camera=cam, rgb=rgb)
    yaw_before = odom.yaw_unwrapped_rad
    fuse.observe(odom, ds_raw_m=0.02, ds_applied_m=0.02, dyaw_rad=0.01, depth=depth, camera=cam, rgb=rgb)
    assert fuse.scene_visual_m == 0.0
    assert abs(odom.yaw_unwrapped_rad - yaw_before) < 1e-9
    assert fuse.visual_applied_yaw == 0.0


def test_a_clear_door_crossing_is_committed():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False)
    bot.phase = "go"
    bot.t_s = 5.0
    bot.goal_xy = (2.4, 0.0)
    bot.path_xy = [(0.4, 0.0), (1.4, 0.0), (2.4, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 5.0
    bot._last_clear = (1.6, 0.70, 0.70)
    bot._bubble_gap = 1.6
    bot._min_gap = 0.40
    bot._go(False, 0.28, 0.70, 0.70)
    assert bot._commit_until > bot.t_s
    bot.t_s = 6.0
    bot._plan()
    assert bot.goal_xy == (2.4, 0.0)


def test_a_smeared_opening_is_crept_not_banned():
    """Occupancy says the shell would clip. The depth nose still looks through.

    That is the bedroom door: a smeared jamb stops the sweep, then the
    stall paints the room beyond the gap as forbidden. The live gap has
    to win, and a flat wall at the same nose range still gets banned.
    """
    from src.mapping.explore import Explorer

    grid = OccupancyGrid()
    _fill(grid, 0.28, -0.08, 0.48, 0.08, grid.l_max)
    bot = Explorer(camera=_small_oak(), fusion=False, floor=False, grid=grid)
    bot.phase = "go"
    bot.t_s = 20.0
    bot.goal_xy = (2.2, 0.0)
    bot.path_xy = [(0.5, 0.0), (2.2, 0.0)]
    bot._replan_at = 1.0e9
    bot._goal_since = 20.0
    bot._last_clear = (0.97, 0.58, 1.23)
    bot._bubble_gap = 0.70
    bot._min_gap = 0.70
    cmd_l, cmd_r = bot._go(False, 0.28, 0.58, 1.23)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.04
    assert bot.goal_xy == (2.2, 0.0)
    assert not bot._banned
    bot._start_recovery()
    assert bot.goal_xy == (2.2, 0.0)
    assert not bot._banned

    # One jamb and an open shoulder: turn toward the opening, do not ban it.
    bot._last_clear = (0.55, 1.95, 0.50)
    bot._bubble_gap = 0.23
    bot._min_gap = 0.23
    bot._sweep_log_at = 1.0e9
    cmd_l, cmd_r = bot._go(False, 0.28, 1.95, 0.50)
    w = (cmd_r - cmd_l) * WHEEL_RADIUS_M / WHEEL_TRACK_M
    assert w > 0.2
    assert bot.goal_xy == (2.2, 0.0)
    # The same view a second later has to drive. A pure spin here is how
    # the bath run turned away from the bedroom.
    bot._opening_since = bot.t_s - 1.5
    cmd_l, cmd_r = bot._go(False, 0.28, 1.95, 0.50)
    v = 0.5 * (cmd_l + cmd_r) * WHEEL_RADIUS_M
    assert v > 0.04
    assert bot.goal_xy == (2.2, 0.0)

    wall = Explorer(camera=_small_oak(), fusion=False, floor=False, grid=grid)
    wall.phase = "go"
    wall.t_s = 20.0
    wall.goal_xy = (2.2, 0.0)
    wall.path_xy = [(0.5, 0.0), (2.2, 0.0)]
    wall._replan_at = 1.0e9
    wall._goal_since = 20.0
    wall._last_clear = (0.95, 0.94, 0.96)
    wall._bubble_gap = 0.70
    wall._min_gap = 0.70
    assert wall._gap_ahead() == ""
    wall._start_recovery()
    assert wall._banned


def test_jamb_steer_only_centres_a_two_sided_gap():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak())
    # A wall on the left and open space on the right is not a doorway.
    assert bot._jamb_yaw(0.40, 99.0) == 0.0
    # Both jambs close, left one nearer: turn right, away from it.
    assert bot._jamb_yaw(0.40, 0.80) < 0.0
    assert bot._jamb_yaw(0.80, 0.40) > 0.0
    # Centred in the opening: no extra yaw.
    assert bot._jamb_yaw(0.55, 0.58) == 0.0


def test_reverse_out_of_a_blind_wall_moves_the_pose():
    """The backup itself is motion. Only the forward skid is frozen."""
    from src.mapping.explore import Explorer

    cam = _small_oak(48, 30)
    depth = np.full((cam.height, cam.width), np.inf)
    bot = Explorer(camera=cam, stride=2)
    bot.phase = "go"
    bot.goal_xy = (3.0, 0.0)
    bot.path_xy = [(3.0, 0.0)]
    bot._replan_at = 1.0e9
    left = right = 0.0
    dt = 0.05
    backed = False
    for _ in range(80):
        cmd_l, cmd_r = bot.step(
            dt_s=dt,
            left_wheel_rad=left,
            right_wheel_rad=right,
            yaw_rate_rad_s=0.0,
            depth=depth,
        )
        if cmd_l < -0.05 and cmd_r < -0.05:
            backed = True
            left += cmd_l * dt
            right += cmd_r * dt
        else:
            # Forward commands stay a skid: the hubs do not follow them.
            pass
    assert backed
    assert bot.pose()[0] < -0.08


def test_contact_mark_survives_a_later_free_ray():
    grid = OccupancyGrid()
    cell = grid.cell_of(0.40, 0.0)
    assert cell is not None
    # Only a cell the camera already called a wall can be reinforced.
    # One free ray does not wipe it. A run of free rays does, so a mark
    # that landed in an opening cannot stay shut.
    grid.integrate((0.0, 0.0), np.array([[0.40, 0.0]]), np.zeros((0, 2)))
    grid.integrate((0.0, 0.0), np.array([[0.40, 0.0]]), np.zeros((0, 2)))
    grid.mark_contact(0.40, 0.0)
    assert grid.labels()[cell] == 1
    grid.integrate((0.0, 0.0), np.zeros((0, 2)), np.array([[2.0, 0.0]]))
    assert grid.labels()[cell] == 1
    for _ in range(8):
        grid.integrate((0.0, 0.0), np.zeros((0, 2)), np.array([[2.0, 0.0]]))
    assert grid.labels()[cell] != 1
    assert not grid.latch[cell]


def test_contact_does_not_invent_a_wall_in_open_space():
    grid = OccupancyGrid()
    grid.mark_contact(0.28, 0.0)
    cell = grid.cell_of(0.28, 0.0)
    assert cell is not None and grid.labels()[cell] == -1
    grid.log[cell] = grid.l_min
    grid.mark_contact(0.28, 0.0)
    assert grid.labels()[cell] == 0
    assert not grid.latch[cell]


def test_stalled_hubs_turn_away_and_mark_the_contact():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(16, 8), stride=2)
    bot.phase = "go"
    bot.goal_xy = (2.0, 0.0)
    bot.path_xy = [(2.0, 0.0)]
    bot._replan_at = 1.0e9
    left, right = bot.step(
        dt_s=0.2,
        left_wheel_rad=0.0,
        right_wheel_rad=0.0,
        yaw_rate_rad_s=0.0,
        depth=None,
    )
    assert left > 0.0 and right > 0.0
    left, right = bot.step(
        dt_s=0.2,
        left_wheel_rad=0.0,
        right_wheel_rad=0.0,
        yaw_rate_rad_s=0.0,
        depth=None,
    )
    assert left < 0.0 and right < 0.0
    # The stall must not drop a wall down the centre of a doorway.
    ahead = bot.grid.cell_of(0.28, 0.0)
    assert ahead is not None and bot.grid.labels()[ahead] != 1


def test_pure_pursuit_turns_left_for_a_left_goal():
    v, w, arrived = pure_pursuit((0.0, 0.0, 0.0), [(1.2, 0.4)], arrive_m=0.2)
    assert not arrived and w > 0.0 and v > 0.0
    left, right = wheel_speeds(v, w)
    assert right > left
    # Track and radius stay the proto numbers.
    assert WHEEL_TRACK_M == pytest_approx(0.34)
    assert WHEEL_RADIUS_M == pytest_approx(0.08)


def test_proto_depth_camera_matches_the_body_constants():
    text = (ROOT / "webots" / "protos" / "ButlerBot.proto").read_text(encoding="utf-8")
    assert 'name "oak_depth"' in text
    assert "fieldOfView 1.396" in text
    assert "noise 0.002" in text
    assert "0.0873" in text
    assert OAK_DEPTH.fov_rad == pytest_approx(1.396)
    assert OAK_DEPTH.rotation[3] == pytest_approx(0.0873)


def test_gyro_noise_is_not_zero():
    rng = np.random.default_rng(0)
    samples = [noisy_yaw_rate(0.0, rng) for _ in range(40)]
    assert abs(float(np.mean(samples))) > 1e-4
    assert float(np.std(samples)) > 1e-3


def _room() -> list[Box]:
    t = 0.12
    z1 = 2.4
    x0, x1, y0, y1 = -1.6, 1.6, -1.2, 1.2
    return [
        Box(x0 - t, y0 - t, 0.0, x1 + t, y0, z1),
        Box(x0 - t, y1, 0.0, x1 + t, y1 + t, z1),
        Box(x0 - t, y0, 0.0, x0, y1, z1),
        Box(x1, y0, 0.0, x1 + t, y1, z1),
    ]


def test_boxed_room_explores_until_the_frontiers_are_gone():
    result = run_scene(
        _room(),
        start=(0.0, 0.0, 0.0),
        steps=280,
        dt_s=0.2,
        noise=True,
        seed=2,
        camera=_small_oak(48, 30),
        stride=2,
    )
    assert result.explorer.done, f"still exploring after {result.steps} steps"
    labels = result.explorer.grid.labels()
    grid = result.explorer.grid
    centre = grid.cell_of(0.0, 0.0)
    wall = grid.cell_of(1.62, 0.0)
    assert centre is not None and labels[centre] == 0
    assert wall is not None and labels[wall] == 1
    # Outside the shell stays unseen: the wall closes the map.
    outside = grid.cell_of(2.4, 0.0)
    assert outside is not None and labels[outside] < 0


def test_scorer_matches_a_perfect_copy_and_a_blank_map():
    gt = rasterize()
    perfect = score_labels(gt, gt.labels.copy())
    assert perfect["occupancy_iou"] == pytest_approx(1.0)
    assert perfect["free_iou"] == pytest_approx(1.0)
    assert perfect["coverage_pct"] == pytest_approx(100.0)
    assert perfect["wall_error_cm"] < 1.0
    blank = np.full(gt.labels.shape, -1, dtype=np.int8)
    empty = score_labels(gt, blank)
    assert empty["coverage_pct"] == pytest_approx(0.0)
    assert empty["free_iou"] == pytest_approx(0.0)
    assert empty["occupancy_iou"] == pytest_approx(0.0)
    assert math.isnan(empty["wall_error_cm"])


def test_spawn_alignment_round_trip_stays_close():
    gt = rasterize()
    sx, sy, yaw = spawn_pose()
    # The spawn cell is open floor.
    col = int(math.floor((sx - gt.origin_xy[0]) / gt.resolution_m))
    row = int(math.floor((sy - gt.origin_xy[1]) / gt.resolution_m))
    assert gt.labels[row, col] == 0
    grid = OccupancyGrid(resolution_m=gt.resolution_m)
    painted = np.full((grid.n, grid.n), -1, dtype=np.int8)
    c, s = math.cos(yaw), math.sin(yaw)
    height, width = gt.labels.shape
    for r in range(height):
        for col_i in range(width):
            if gt.labels[r, col_i] < 0:
                continue
            xw = gt.origin_xy[0] + (col_i + 0.5) * gt.resolution_m
            yw = gt.origin_xy[1] + (r + 0.5) * gt.resolution_m
            dx, dy = xw - sx, yw - sy
            xo = c * dx + s * dy
            yo = -s * dx + c * dy
            cell = grid.cell_of(xo, yo)
            if cell is not None:
                painted[cell] = gt.labels[r, col_i]
    pred = warp_to_ground_truth(painted, grid.meta(), gt, (sx, sy), yaw)
    scores = score_labels(gt, pred)
    assert scores["coverage_pct"] > 98.0
    assert scores["free_iou"] > 0.9
    assert scores["occupancy_iou"] > 0.85
    assert scores["wall_error_cm"] < 12.0


def _wall_grid(hits: np.ndarray) -> OccupancyGrid:
    grid = OccupancyGrid()
    for _ in range(3):
        grid.integrate((0.0, 0.0), hits, np.zeros((0, 2)))
    return grid


def _corner_hits() -> np.ndarray:
    """An L of returns, so a yaw error moves the side wall off the map."""
    front = np.stack((np.full(24, 1.60), np.linspace(-0.55, 0.55, 24)), axis=1)
    side = np.stack((np.linspace(0.85, 1.55, 16), np.full(16, 0.70)), axis=1)
    return np.vstack((front, side))


def test_pose_correction_recovers_translation_and_yaw():
    hits = _corner_hits()
    grid = _wall_grid(hits)
    wrong = (0.06, -0.04, 0.02)
    dx, dy, dyaw = pose_correction(grid, wrong, hits)
    before = math.hypot(wrong[0], wrong[1])
    after = math.hypot(wrong[0] + dx, wrong[1] + dy)
    assert after < before * 0.75
    assert after < 0.05
    assert abs(wrong[2] + dyaw) < 0.02
    # A shift onto a different wall is outside the search and is refused.
    far = pose_correction(grid, (0.35, 0.20, 0.0), hits)
    assert far == (0.0, 0.0, 0.0)


def test_pose_correction_rejects_a_weak_or_empty_match():
    hits = _corner_hits()
    empty = OccupancyGrid()
    assert pose_correction(empty, (0.0, 0.0, 0.0), hits[:8]) == (0.0, 0.0, 0.0)
    assert pose_correction(empty, (0.0, 0.0, 0.0), hits) == (0.0, 0.0, 0.0)
    # Occupied cells the 0.40 m window cannot reach.
    far = np.stack((np.full(40, 4.0), np.linspace(-1.0, 1.0, 40)), axis=1)
    grid = _wall_grid(far)
    dx, dy, dyaw = pose_correction(grid, (0.0, 0.0, 0.0), hits)
    assert (dx, dy, dyaw) == (0.0, 0.0, 0.0)


def test_pose_correction_leaves_a_pose_that_already_explains_the_scan():
    hits = _corner_hits()
    grid = _wall_grid(hits)
    dx, dy, dyaw = pose_correction(grid, (0.0, 0.0, 0.0), hits)
    assert math.hypot(dx, dy) < 0.05
    assert abs(dyaw) < 0.02


def test_scan_correction_moves_the_odometry_pose():
    odom = WheelGyroOdometry()
    odom.apply_correction(0.10, -0.20, math.pi)
    x, y, yaw = odom.pose()
    assert x == pytest_approx(0.10)
    assert y == pytest_approx(-0.20)
    assert yaw == pytest_approx(math.pi) or yaw == pytest_approx(-math.pi)


def test_a_square_script_closes_after_four_legs():
    from src.mapping.scripted import BodyScript

    script = BodyScript("square")
    yaw = 0.0
    spins = 0
    for _leg in range(4):
        saw_forward = False
        for _step in range(100):
            left, right = script.command(ds_m=0.02, yaw_unwrapped=yaw, forward_gap_m=2.0, rear_gap_m=2.0)
            if left * right < 0.0:
                spins += 1
                break
            assert left > 0.0 and right > 0.0
            saw_forward = True
        assert saw_forward
        for _step in range(50):
            yaw += 0.04
            script.command(ds_m=0.0, yaw_unwrapped=yaw, forward_gap_m=2.0, rear_gap_m=2.0)
            if script.done or (script.phase == "drive" and script.leg == _leg + 1):
                break
    assert spins == 4
    assert script.done


def test_a_plunge_stops_when_the_forward_gap_closes():
    from src.mapping.scripted import BodyScript

    script = BodyScript("plunge")
    left, right = script.command(ds_m=0.01, yaw_unwrapped=0.0, forward_gap_m=2.0, rear_gap_m=None)
    assert left > 2.0 and right > 2.0
    assert not script.done
    left, right = script.command(ds_m=0.01, yaw_unwrapped=0.0, forward_gap_m=0.03, rear_gap_m=None)
    assert left == 0.0 and right == 0.0
    assert not script.done
    for _ in range(200):
        left, right = script.command(
            ds_m=0.0, yaw_unwrapped=0.0, forward_gap_m=0.03, rear_gap_m=None, dt_s=0.008
        )
    assert left == 0.0 and right == 0.0
    assert script.done


def test_scripted_drive_starts_the_pattern_instead_of_a_survey_spin():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False, loop=False, drive="square")
    left, right = bot.step(dt_s=0.008, left_wheel_rad=0.0, right_wheel_rad=0.0, yaw_rate_rad_s=0.0)
    assert left > 1.0 and right > 1.0
    assert abs(left - right) < 0.2


def test_scenario_world_changes_only_the_butlerbot_pose():
    from eval.run_mapping_scenarios import WORLD, _patch_world

    text = WORLD.read_text(encoding="utf-8")
    patched = _patch_world(text, 4.4, 1.778, math.pi)
    assert "translation 4.4000 1.7780 0" in patched
    assert "rotation 0 0 1 3.141593" in patched
    assert "translation 6.1 4.7 0" not in patched
    assert 'EXTERNPROTO "../protos/ButlerBot.proto"' in patched
    assert "apartment_pose_logger" in patched


def test_a_short_gap_splits_two_rooms_and_a_long_strip_is_a_hall():
    from src.mapping.rooms import crossing_bonus, segment_map

    labels = np.full((40, 80), -1, dtype=np.int8)
    labels[8:32, 4:28] = 0
    labels[8:32, 36:70] = 0
    labels[8:32, 28:36] = 1
    labels[12:26, 28:36] = 0
    info = segment_map(labels, 0.05)
    assert info["doors"].any()
    assert set(info["names"].values()) == {"room_1", "room_2"}
    # A wide room opening into a walled corridor. The corridor is a metre
    # wide and three metres long, so it is a hall, and the mouth is a door.
    hall = np.full((60, 120), -1, dtype=np.int8)
    hall[4:48, 4:44] = 0
    hall[14:34, 44:104] = 0
    hall[10:14, 44:104] = 1
    hall[34:38, 44:104] = 1
    named = segment_map(hall, 0.05)
    assert "hall" in named["names"].values()
    assert any(name.startswith("room_") for name in named["names"].values())
    assert named["doors"].any()
    # The apartment hall is only 1.02 m by 1.73 m. Length alone must still
    # name it, after the mouth cells are cut out of the region.
    apt = np.full((80, 90), -1, dtype=np.int8)
    apt[10:70, 40:85] = 0
    apt[30:50, 5:40] = 0
    apt[26:30, 5:40] = 1
    apt[50:54, 5:40] = 1
    apt_named = segment_map(apt, 0.05)
    assert "hall" in apt_named["names"].values()
    assert any(name.startswith("room_") for name in apt_named["names"].values())
    doors = np.zeros((4, 4), dtype=bool)
    doors[1, 2] = True
    regions = np.zeros((4, 4), dtype=np.int16)
    regions[1, 0] = 1
    regions[1, 3] = 2
    bonus = crossing_bonus([(1, 0), (1, 2), (1, 3)], {"doors": doors, "labels": regions}, (1, 0))
    assert bonus >= 3.0
    # The same door speck inside one region is not a new place.
    same = np.ones((4, 4), dtype=np.int16)
    assert crossing_bonus([(1, 0), (1, 2), (1, 3)], {"doors": doors, "labels": same}, (1, 0)) == 1.0
    # Standing on the cut-out threshold still counts as leaving that room.
    gap = np.zeros((4, 4), dtype=np.int16)
    gap[0, 0] = 1
    gap[1, 3] = 2
    assert crossing_bonus([(1, 0), (1, 2), (1, 3)], {"doors": doors, "labels": gap}, (1, 0)) >= 3.0
    # An 0.81 m opening is a thin throat. Once the narrow cells run the
    # full width of the gap, the old 0.80 m length cap called it a corridor.
    wide = np.full((60, 90), -1, dtype=np.int8)
    wide[6:54, 4:36] = 0
    wide[6:54, 40:86] = 0
    wide[6:54, 36:40] = 1
    wide[16:37, 36:40] = 0
    split = segment_map(wide, 0.05)
    assert split["doors"].any()
    assert sum(1 for name in split["names"].values() if name.startswith("room_")) >= 2


def test_the_opening_ahead_beats_a_long_frontier_beside_it():
    """A doorway into another region outscores the long unknown edge of this one."""
    from src.mapping.rooms import segment_map

    grid = OccupancyGrid()
    _fill(grid, -0.15, -2.0, 3.2, 2.0, grid.l_min)
    _fill(grid, -0.45, -2.0, -0.15, 2.0, grid.l_max)
    _fill(grid, -0.45, -0.40, -0.15, 0.40, grid.l_min)
    _fill(grid, -2.4, -0.45, -0.45, 0.45, grid.l_min)
    rooms = segment_map(grid.labels(), grid.res)
    assert len(set(rooms["names"])) >= 2
    chosen = select_goal(
        grid,
        (0.6, 0.0),
        heading_xy=(-1.0, 0.0),
        rooms=rooms,
    )
    assert chosen is not None
    goal, _path = chosen
    assert goal[0] < 0.0


def test_a_smeared_threshold_is_still_a_way_into_the_room_beyond():
    """Free space past a thin occupied smear is a goal, not a wall to route around."""
    grid = OccupancyGrid()
    _fill(grid, -0.7, 0.0, 0.7, 1.75, grid.l_min)
    # What a depth ray through the door actually paints: the opening,
    # then the room beyond it. The threshold itself is occupied.
    _fill(grid, -0.40, 2.25, 0.40, 2.90, grid.l_min)
    _fill(grid, -1.8, 2.90, 1.8, 5.2, grid.l_min)
    _fill(grid, -1.8, 1.80, -0.45, 2.90, grid.l_max)
    _fill(grid, 0.45, 1.80, 1.8, 2.90, grid.l_max)
    _fill(grid, -0.45, 1.85, 0.45, 2.20, grid.l_max)
    chosen = select_goal(grid, (0.0, 1.1), heading_xy=(0.0, 1.0))
    assert chosen is not None
    goal, _path = chosen
    assert goal[1] > 2.2


def test_seen_through_a_doorway_is_not_a_visited_room():
    from src.mapping.explore import Explorer

    bot = Explorer(camera=_small_oak(), fusion=False, floor=False, loop=False)
    bot.room_labels = np.zeros((bot.grid.n, bot.grid.n), dtype=np.int16)
    bot.room_labels[80:140, 80:140] = 1
    bot.visited[100, 100] = True
    labels = bot.observed_room_labels()
    assert labels[100, 100] == 1
    assert labels[120, 120] == 0


def test_the_rear_camera_looks_behind_the_tail():
    from src.mapping.body import REAR_DEPTH
    from src.mapping.depth_cloud import body_points, rear_clearance_m

    image = np.full((REAR_DEPTH.height, REAR_DEPTH.width), np.inf)
    image[REAR_DEPTH.height // 2, REAR_DEPTH.width // 2] = 1.0
    hits, _floors, _origin = body_points(REAR_DEPTH, image, stride=1)
    assert hits.shape[0] == 1
    assert hits[0, 0] < -1.0
    gap = rear_clearance_m(hits)
    assert gap is not None and 0.8 < gap < 1.2


def test_solids_include_the_closed_doors():
    names = {solid.name for solid in load_solids()}
    assert "APT_DOOR_FRONT" in names
    assert "APT_DOOR_PATIO" in names
    assert "APT_FURN_ISLAND" in names


def pytest_approx(value, abs=1e-6):
    import pytest

    return pytest.approx(value, abs=abs)
