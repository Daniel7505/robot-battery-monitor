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
from src.mapping.scan_match import scan_fit
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


def test_solids_include_the_closed_doors():
    names = {solid.name for solid in load_solids()}
    assert "APT_DOOR_FRONT" in names
    assert "APT_DOOR_PATIO" in names
    assert "APT_FURN_ISLAND" in names


def pytest_approx(value, abs=1e-6):
    import pytest

    return pytest.approx(value, abs=abs)
