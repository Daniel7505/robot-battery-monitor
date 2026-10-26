"""Safety bubble, floor-flow, fusion, and loop closure."""

from __future__ import annotations

import math

import numpy as np
import pytest

from src.mapping.bubble import (
    assess_bubble,
    max_speed_for_gap,
    reverse_budget_m,
    stop_gap,
)
from src.mapping.floor_flow import FloorFlow, metres_per_px
from src.mapping.fuse import MotionFusion
from src.mapping.loop_close import LoopCloser
from src.mapping.explore import Explorer
from src.mapping.odometry import WheelGyroOdometry


def test_stop_distance_grows_with_speed():
    assert stop_gap(0.28) > stop_gap(0.12) > stop_gap(0.0)
    assert stop_gap(0.0) == pytest.approx(0.04)


def test_the_body_fits_an_81cm_door_with_the_inflation():
    """0.39 m across the tires, 0.81 m doorway, 0.20 m planner inflation.

    The inflated corridor is still wider than nothing, and a robot held
    on the centre line keeps air beside each tire.
    """
    from src.mapping.body import ROBOT_RADIUS_M
    from src.mapping.bubble import SHELL_HALF_M

    door = 0.81
    width = 2.0 * SHELL_HALF_M
    assert width == pytest.approx(0.39)
    free = door - 2.0 * ROBOT_RADIUS_M
    assert free > 0.05
    air = door / 2.0 - SHELL_HALF_M
    assert air > 0.15
    assert SHELL_HALF_M <= ROBOT_RADIUS_M < door / 2.0


def test_three_still_sensors_are_a_pin_and_one_missing_is_not():
    from src.mapping.pin import PinWatch

    pin = PinWatch()
    pin.anchor(1.0, 2.0)
    for _ in range(5):
        pin.add(0.012, 0.0, 0.0, 0.0)
    assert pin.pinned()
    assert pin.x0 == 1.0 and pin.y0 == 2.0
    moving = PinWatch()
    for _ in range(5):
        moving.add(0.012, 0.012, 0.012, 0.012)
    assert moving.ready()
    assert not moving.pinned()
    short_a_sensor = PinWatch()
    for _ in range(5):
        short_a_sensor.add(0.012, 0.0, 0.0, None)
    assert short_a_sensor.ready()
    assert not short_a_sensor.pinned()


def test_a_slow_command_the_hubs_ignore_is_a_stall():
    """Backup speed is about 1.5 rad/s. One 8 ms step is under the old 0.02 rad floor."""
    from src.mapping.explore import Explorer

    bot = Explorer()
    bot._wheels = (0.0, 0.0)
    bot._commanded = (1.5, 1.5)
    stalled = False
    for _ in range(40):
        stalled = bot._wheels_stalled(0.0, 0.0, 0.008)
    assert stalled


def test_floor_agreement_does_not_write_yaw():
    """The downward camera is a translation sensor. An agreeing shift
    must not turn the heading, even though the fusion also carries yaw.
    """
    rng = np.random.default_rng(2)
    image = rng.normal(40.0, 25.0, size=(32, 32))
    shifted = np.roll(image, 4, axis=0)
    odom = WheelGyroOdometry()
    fuse = MotionFusion(floor=True, visual=False)
    odom.update(0.0, 0.0, 0.0, 0.05)
    fuse.observe(odom, ds_raw_m=0.0, ds_applied_m=0.0, dyaw_rad=0.0, floor=image)
    odom.update(0.30, 0.30, 0.0, 0.05)
    fuse.observe(odom, ds_raw_m=0.024, ds_applied_m=0.024, dyaw_rad=0.0, floor=shifted)
    assert fuse.floor_applied_yaw == 0.0
    assert abs(odom.pose()[2]) < 1e-6


def test_a_centred_door_is_passable_and_a_close_side_is_not():
    # 0.81 m opening, tire outer at 0.195 m, about 0.21 m of air each side.
    assert max_speed_for_gap(0.21) > 0.25
    assert max_speed_for_gap(0.05) < 0.08
    open_door = assess_bubble(2.4, blind=False, proximity={"us_left": 0.21, "us_right": 0.22})
    assert open_door.minimum is not None
    assert max_speed_for_gap(open_door.minimum) > 0.25
    clip = assess_bubble(2.0, blind=False, proximity={"us_left": 0.05, "us_right": 0.21})
    assert clip.left is not None and max_speed_for_gap(clip.left) < 0.08


def test_blind_depth_does_not_hide_a_sonar_return():
    blind = assess_bubble(99.0, blind=True, proximity=None)
    assert blind.forward == 0.0
    seen = assess_bubble(99.0, blind=True, proximity={"us_front": 0.25})
    assert seen.forward == pytest.approx(0.25)


def test_reverse_is_only_as_far_as_the_pivot_needs():
    assert reverse_budget_m(0.21) == 0.0
    tight = reverse_budget_m(0.0)
    assert 0.03 < tight <= 0.25
    assert reverse_budget_m(0.0, blind=True) >= 0.12
    assert reverse_budget_m(0.0, blind=True) <= 0.25


def test_floor_shift_sign_and_low_texture():
    rng = np.random.default_rng(2)
    image = rng.normal(128.0, 40.0, size=(32, 32))
    flow = FloorFlow(scale=1.0, noise_px=0.0)
    assert flow.measure(image) is None
    rolled = np.roll(np.roll(image, 2, axis=0), 3, axis=1)
    sample = flow.measure(rolled)
    assert sample is not None and sample.ok
    assert sample.dx_m > 0.0
    assert sample.dy_m > 0.0
    flat = np.full((32, 32), 12.0)
    blank = FloorFlow(scale=1.0, noise_px=0.0)
    assert blank.measure(flat) is None
    rejected = blank.measure(flat)
    assert rejected is not None and not rejected.ok


def test_scale_error_shrinks_when_the_wheels_agree():
    rng = np.random.default_rng(3)
    image = rng.normal(0.0, 30.0, size=(32, 32))
    flow = FloorFlow(scale=1.03, noise_px=0.0)
    assert flow.measure(image) is None
    sample = flow.measure(np.roll(image, 8, axis=0))
    assert sample is not None and sample.ok
    true_m = 8.0 * metres_per_px()
    assert sample.dx_m == pytest.approx(true_m * 1.03, rel=0.2)
    before = flow.scale
    flow.learn_scale(true_m, sample.dx_m)
    assert flow.scale < before


def test_fast_yaw_does_not_apply_a_visual_shove():
    """A spin moves corners by more than the search can follow.

    The boxed-room scan accepted one of those fits and opened a frontier
    in a room the spin had already closed.
    """
    odom = WheelGyroOdometry()
    odom.update(0.0, 0.0, 0.0, 0.2)
    fuse = MotionFusion(floor=False, visual=True)
    fuse._visual.add(0.0, 0.0, 0.14)
    note = fuse._apply_shift(
        odom,
        fuse._visual,
        -0.001,
        0.010,
        0.128,
        ok=True,
        quality=0.8,
        kind="visual",
        var_xy=0.008,
        var_yaw=0.01,
    )
    assert note == ""
    assert fuse.visual_updates == 0
    assert odom.pose()[0] == 0.0
    assert odom.pose()[1] == 0.0


def test_floor_slip_needs_a_few_short_steps():
    """One 8 ms frame is a few millimetres. Slip shows up across the window."""
    rng = np.random.default_rng(5)
    image = rng.normal(40.0, 25.0, size=(32, 32))
    odom = WheelGyroOdometry()
    fuse = MotionFusion(floor=True, visual=False, rng=np.random.default_rng(1))
    fuse.floor.noise_px = 0.0
    odom.update(0.0, 0.0, 0.0, 0.2)
    assert fuse.observe(odom, ds_raw_m=0.0, ds_applied_m=0.0, dyaw_rad=0.0, floor=image) == ""
    notes = []
    for i in range(1, 8):
        odom.update(0.05 * i, 0.05 * i, 0.0, 0.008)
        notes.append(
            fuse.observe(odom, ds_raw_m=0.004, ds_applied_m=0.004, dyaw_rad=0.0, floor=image)
        )
    assert notes.count("slip") == 1
    assert notes.index("slip") < len(notes) - 1
    assert fuse.slip_events == 1
    assert abs(odom.pose()[0]) < 0.015


def test_floor_slip_rejects_wheel_travel():
    rng = np.random.default_rng(4)
    image = rng.normal(40.0, 25.0, size=(32, 32))
    odom = WheelGyroOdometry()
    fuse = MotionFusion(floor=True, visual=False)
    odom.update(0.0, 0.0, 0.0, 0.2)
    fuse.observe(odom, ds_raw_m=0.0, ds_applied_m=0.0, dyaw_rad=0.0, floor=image)
    odom.update(0.5, 0.5, 0.0, 0.2)
    note = fuse.observe(odom, ds_raw_m=0.04, ds_applied_m=0.04, dyaw_rad=0.0, floor=image)
    assert note == "slip"
    assert fuse.slip_events == 1
    assert abs(odom.pose()[0]) < 0.01


def test_loop_closure_fixes_drift_tolerates_one_change_and_rejects_two_points():
    closer = LoopCloser(min_path_m=3.0, keyframe_m=0.5)
    home = np.array(
        [
            [1.0, 0.45],
            [1.15, -0.55],
            [1.7, 0.15],
            [2.05, -0.35],
            [2.4, 0.55],
        ],
        dtype=np.float64,
    )
    hits = np.array([[1.4, 0.0]])
    floors = np.zeros((0, 2))
    sensor = np.array([0.14, 0.0])
    closer.consider((0.0, 0.0, 0.0), 0.0, home, hits, floors, sensor)
    for i in range(1, 10):
        other = np.array(
            [
                [0.6 + 0.05 * i, 0.2],
                [1.4, -0.8 - 0.07 * i],
                [2.2, 0.15 * i],
                [0.9, 0.7 + 0.04 * i],
                [1.8 + 0.03 * i, -0.15 * i],
            ]
        )
        closer.consider((0.5 * i, 0.02 * i, 0.0), 0.5 * i, other, hits, floors, sensor)
    drifted = (0.30, -0.12, 0.0)
    # The first look only arms the match. A second agreement applies it.
    assert closer.consider(drifted, 8.0, home, hits, floors, sensor) is None
    fixed = closer.consider(drifted, 8.0, home, hits, floors, sensor)
    assert fixed is not None
    assert math.hypot(fixed[0], fixed[1]) < 0.08
    # The place we recognized does not move; the far end of the chain does.
    assert math.hypot(closer.frames[0].x, closer.frames[0].y) < 1e-6
    changed = home.copy()
    changed[2] = np.array([3.4, 2.2])
    again = LoopCloser(min_path_m=3.0, keyframe_m=0.5)
    again.consider((0.0, 0.0, 0.0), 0.0, home, hits, floors, sensor)
    for i in range(1, 10):
        other = np.array(
            [
                [0.55 + 0.04 * i, -0.3],
                [1.3, 0.9 + 0.05 * i],
                [2.5, -0.2 * i],
                [0.85, -0.7 - 0.03 * i],
                [1.9, 0.25 * i],
            ]
        )
        again.consider((0.45 * i, 0.0, 0.0), 0.45 * i, other, hits, floors, sensor)
    assert again.consider(drifted, 8.0, changed, hits, floors, sensor) is None
    fixed_one = again.consider(drifted, 8.0, changed, hits, floors, sensor)
    assert fixed_one is not None
    weak = LoopCloser(min_path_m=3.0, keyframe_m=0.5)
    weak.consider((0.0, 0.0, 0.0), 0.0, home, hits, floors, sensor)
    assert weak.consider((0.4, 0.0, 0.0), 6.0, home[:2], hits, floors, sensor) is None
