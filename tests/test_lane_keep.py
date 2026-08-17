import math

from src.lane_keep import (
    SKY_RGB,
    MarkStopTracker,
    apply_sf_rotation,
    beam_sf_rotation,
    classify_view,
    cruise_speed_m_s,
    CorridorWatch,
    ForecastLookout,
    GapPlanner,
    ground_hit_ahead_m,
    lane_keep_command,
    image_row_to_ground_ahead_m,
    pixel_to_ground_m,
    SteerFilter,
    steer_from_offsets,
    steer_from_ranges,
    steer_from_walls,
    wall_pressure,
    wheels_from_steer,
    yellow_line_curve,
    yellow_band_fill,
    yellow_ahead_pixel,
    yellow_nearest_pixel,
    yellow_line_offset,
    yellow_far_band,
    yellow_far_offset,
    yellow_horizon_row,
    yellow_look_band,
    yellow_look_split,
    yellow_near_band,
    yellow_near_fill,
    dirt_band_ok,
    preview_ahead_weight,
    preview_band_ok,
    preview_look_ahead_m,
    offset_to_column,
    look_at_sf_rotation,
    camera_fov_pyramid,
    rotate_bgra_90_cw,
    forecast_gap_error,
    forecast_wall_hit,
    line_wall_hit,
    metric_ct_from_walls,
    metric_walls_plausible,
    yellow_wall_pixel,
    PLANNER_KAHEAD,
    DEFAULT_STEER_RELEASE_PER_S,
    roll_camera_sf,
    FORECAST_IMAGE_ROLL_RAD,
    mean_rgb_bgra,
    peak_score_bgra,
    red_score,
    stopping_distance_m,
    time_to_mark_s,
    yellow_score,
)


def test_yellow_high_on_paint_low_on_gray():
    assert yellow_score((0.95, 0.95, 0.20)) > 0.6
    assert yellow_score((0.50, 0.50, 0.52)) < 0.1


def test_red_high_on_finish_low_on_yellow_and_gray():
    assert red_score((0.90, 0.15, 0.12)) > 0.6
    assert red_score((0.95, 0.95, 0.20)) < 0.1
    assert red_score((0.50, 0.50, 0.52)) < 0.1


def test_score_fallback_pushes_off_louder_wall():
    """No geometry: more yellow on a side is that wall. Push away."""
    left_wall = lane_keep_command(0.7, 0.0, 0.0, cruise=5.5, k_steer=2.0)
    right_wall = lane_keep_command(0.0, 0.7, 0.0, cruise=5.5, k_steer=2.0)
    assert left_wall["brake"] is False
    assert left_wall["left"] > left_wall["right"]
    assert right_wall["right"] > right_wall["left"]


def _bgra_bar(w: int, h: int, x0: int, x1: int) -> bytearray:
    img = bytearray(w * h * 4)
    for y in range(h):
        for x in range(w):
            o = (y * w + x) * 4
            if x0 <= x < x1:
                img[o : o + 4] = bytes((50, 240, 240, 255))
            else:
                img[o : o + 4] = bytes((140, 140, 140, 255))
    return img


def test_yellow_look_band_is_dirt_not_sky():
    """Brain box sits on the paint, not the top-of-frame sky."""
    y0, y1 = yellow_look_band(64)
    # Horizon on these along-stripe eyes is ~40%. Box must start on dirt.
    assert y0 >= 28
    assert y1 > 32
    assert y1 < 64
    assert y0 < y1
    assert offset_to_column(0.0, 64) == 31 or offset_to_column(0.0, 64) == 32


def test_yellow_offset_center_left_right():
    w, h = 32, 32
    mid = yellow_line_offset(_bgra_bar(w, h, 14, 18), w, h)
    left = yellow_line_offset(_bgra_bar(w, h, 2, 6), w, h)
    right = yellow_line_offset(_bgra_bar(w, h, 26, 30), w, h)
    empty = yellow_line_offset(_bgra_bar(w, h, 0, 0), w, h)
    assert mid is not None and abs(mid) < 0.12
    assert left is not None and left < -0.4
    assert right is not None and right > 0.4
    assert empty is None


def test_yellow_offset_ignores_far_stripe_ticks():
    """A few bright ticks on the far edge must not flip the main bar."""
    w, h = 32, 32
    img = _bgra_bar(w, h, 3, 7)
    for y in (8, 12, 16):
        o = (y * w + 28) * 4
        img[o : o + 4] = bytes((50, 240, 240, 255))
    off = yellow_line_offset(img, w, h)
    assert off is not None and off < -0.4


def test_inward_wall_pushes_away():
    """Stripe sliding toward the robot is contact. Push off, do not center it."""
    left = lane_keep_command(0.5, 0.5, 0.0, cruise=5.5, k_steer=2.0, left_offset=0.4)
    right = lane_keep_command(0.5, 0.5, 0.0, cruise=5.5, k_steer=2.0, right_offset=-0.4)
    assert left["left"] > left["right"]
    assert right["right"] > right["left"]
    assert left["steer"] is not None and left["steer"] > 0
    assert right["steer"] is not None and right["steer"] < 0


def test_outboard_walls_keep_rolling():
    """Yellow on the outside of each eye is a far wall. Do not turn toward it."""
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        left_offset=-0.4,
        right_offset=0.4,
    )
    assert cmd["brake"] is False
    assert cmd["steer"] == 0.0
    assert abs(cmd["left"] - cmd["right"]) < 1e-6


def test_equal_inward_walls_are_corridor_not_y0():
    """Both walls equally close → go straight. Not a GPS centerline hunt."""
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        left_offset=0.35,
        right_offset=-0.35,
    )
    assert cmd["steer"] == 0.0
    assert abs(cmd["left"] - cmd["right"]) < 1e-6
    assert cmd["left_pressure"] > 0.1
    assert cmd["right_pressure"] > 0.1


def test_higher_fill_is_the_closer_wall():
    """Along-stripe offset is aim, not range. More yellow = closer wall."""
    now = steer_from_walls(0.05, -0.05, left_fill=0.05, right_fill=0.05)
    coming = steer_from_walls(0.05, -0.05, left_fill=0.20, right_fill=0.05)
    assert now == 0.0 or (now is not None and abs(now) < 0.05)
    assert coming is not None and coming > 0.40


def test_spawn_fill_is_about_five_percent():
    w, h = 32, 32
    img = _bgra_bar(w, h, 14, 16)
    fill = yellow_band_fill(img, w, h)
    assert 0.02 < fill < 0.12


def test_fill_ignores_sky_above_the_dirt_box():
    """Heaven is not a wall. Only paint inside the dirt look box counts."""
    w, h = 32, 32
    y0, y1 = yellow_look_band(h)
    assert y0 >= h // 2
    sky_only = bytearray(w * h * 4)
    dirt = bytearray(w * h * 4)
    for y in range(h):
        for x in range(w):
            o = (y * w + x) * 4
            sky_only[o : o + 4] = bytes((140, 140, 140, 255))
            dirt[o : o + 4] = bytes((140, 140, 140, 255))
            if y < y0 and 8 <= x < 24:
                sky_only[o : o + 4] = bytes((50, 240, 240, 255))
            if y0 <= y < y1 and 8 <= x < 24:
                dirt[o : o + 4] = bytes((50, 240, 240, 255))
    assert yellow_band_fill(sky_only, w, h) < 0.05
    assert yellow_band_fill(dirt, w, h) > 0.30


def test_fill_is_main_stripe_width_not_both_walls():
    """Both eyes see both yellows. Fill must not count the other stripe."""
    w, h = 32, 32
    both = _bgra_bar(w, h, 3, 7)
    for y in range(h):
        for x in range(24, 28):
            o = (y * w + x) * 4
            both[o : o + 4] = bytes((50, 240, 240, 255))
    one = _bgra_bar(w, h, 3, 7)
    two_width = yellow_band_fill(both, w, h)
    one_width = yellow_band_fill(one, w, h)
    assert one_width > 0.08
    assert abs(two_width - one_width) < 0.04


def test_wall_pressure_none_is_zero():
    assert wall_pressure(None, side="left") == 0.0
    assert steer_from_walls(None, None) is None


def test_fat_stripe_is_a_solid_wall():
    """Past the hit width the wall is an object — full shove, not a nudge."""
    tap = wall_pressure(None, side="left", fill=0.12)
    hit = wall_pressure(None, side="left", fill=0.22)
    assert tap < 0.50
    assert hit >= 1.0


def test_fill_pushes_off_the_closer_wall():
    left = lane_keep_command(
        0.5, 0.5, 0.0, cruise=5.5, k_steer=2.0, left_fill=0.22, right_fill=0.05
    )
    right = lane_keep_command(
        0.5, 0.5, 0.0, cruise=5.5, k_steer=2.0, left_fill=0.05, right_fill=0.22
    )
    assert left["left"] > left["right"]
    assert right["right"] > right["left"]


def test_one_eye_dark_is_contact_on_that_wall():
    """Right eye dark while left still sees paint → shove off the right wall."""
    lost_right = lane_keep_command(
        0.50,
        0.03,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        left_offset=0.04,
        allow_one_eye=True,
    )
    lost_left = lane_keep_command(
        0.03,
        0.50,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        right_offset=-0.04,
        allow_one_eye=True,
    )
    assert lost_right["brake"] is False
    assert lost_right["steer"] < -0.30
    assert lost_right["right"] > lost_right["left"]
    assert lost_left["steer"] > 0.30
    assert lost_left["left"] > lost_left["right"]


def test_one_eye_ignored_until_both_eyes_have_been_seen():
    """Spawn lazy-eye must not shove us into the desert."""
    cmd = lane_keep_command(
        0.50, 0.03, 0.0, cruise=5.5, k_steer=2.0, left_offset=0.04
    )
    assert cmd["brake"] is False
    assert abs(float(cmd.get("steer") or 0.0)) < 0.15


def test_virtual_wheel_left_means_left_hub_slows():
    steer = steer_from_offsets(-0.4, -0.4)
    assert steer is not None and steer < 0
    left, right = wheels_from_steer(steer, cruise=5.5, k_steer=2.0)
    assert right > left
    assert left > 0 and right > 0


def test_opposite_eye_ticks_deadband_to_straight():
    """L tick right + R tick left of equal size is not a turn."""
    assert steer_from_offsets(0.08, -0.08) == 0.0


def test_small_curve_leftover_is_not_deadbanded():
    """IN_LANE lap steered at ~−0.05 after the eyes argued. Keep that."""
    s = steer_from_offsets(0.07, -0.17)
    assert s is not None and s < -0.04


def test_preview_adds_turn_when_centered_on_a_bend():
    """Centered now, paint sliding left ahead → turn left. Any map."""
    straight = steer_from_offsets(-0.04, -0.04, left_curve=0.0, right_curve=0.0)
    bent = steer_from_offsets(-0.04, -0.04, left_curve=-0.40, right_curve=-0.40)
    assert straight is not None and bent is not None
    assert bent < straight
    assert bent < -0.10


def test_preview_fades_when_already_off_center():
    """Don't double-steer a lobe we are already correcting."""
    pos_only = steer_from_offsets(-0.70, -0.70, left_curve=-0.40, right_curve=-0.40)
    assert pos_only is not None
    assert abs(pos_only - (-0.70)) < 0.12


def test_yellow_curve_reads_a_bend_not_a_map():
    w, h = 32, 32
    mid = yellow_look_split(h)
    y0, y1 = yellow_look_band(h)
    assert y0 < mid < y1
    vertical = _bgra_bar(w, h, 14, 18)
    assert abs(yellow_line_curve(vertical, w, h)) < 0.08
    bent = bytearray(w * h * 4)
    for y in range(h):
        if y < mid:
            x0, x1 = 3, 7
        else:
            x0, x1 = 20, 24
        for x in range(w):
            o = (y * w + x) * 4
            if x0 <= x < x1:
                bent[o : o + 4] = bytes((50, 240, 240, 255))
            else:
                bent[o : o + 4] = bytes((140, 140, 140, 255))
    curve = yellow_line_curve(bent, w, h)
    assert curve < -0.4


def test_turn_slows_forward_speed():
    l0, r0 = wheels_from_steer(0.0, cruise=5.5, k_steer=2.0)
    l1, r1 = wheels_from_steer(-0.8, cruise=5.5, k_steer=2.0)
    assert abs((l0 + r0) - 11.0) < 1e-6
    assert (l1 + r1) < (l0 + r0) - 1.0
    assert r1 > l1


def test_steer_filter_eases_in():
    filt = SteerFilter(slew_per_s=1.0)
    a = filt.step(-0.5, 0.1)
    b = filt.step(-0.5, 0.1)
    assert a > -0.5 and a < 0
    assert b < a


def test_steer_filter_releases_slower_than_it_grabs():
    """Keep the tug on when the planner eases. Grab stays snappy."""
    grab = SteerFilter(slew_per_s=12.0, release_per_s=4.0)
    up = grab.step(1.0, 0.05)
    assert abs(up - 0.60) < 1e-6
    down = grab.step(0.0, 0.05)
    assert abs(down - 0.40) < 1e-6
    assert (up - down) < up


def test_steer_filter_drops_the_hold_when_the_miss_flips():
    """New bend / opposite yellow: do not keep the old tug."""
    filt = SteerFilter(slew_per_s=12.0, release_per_s=4.0)
    filt.value = 0.60
    out = filt.step(-0.60, 0.05)
    assert abs(out - 0.0) < 1e-6


def test_lost_paint_stops_instead_of_cruising():
    """Both eyes dark: stop. Do not hold a heading into the desert."""
    cmd = lane_keep_command(0.0, 0.0, 0.0, cruise=5.5)
    assert cmd["brake"] is True
    assert cmd["left"] == 0.0
    assert cmd["right"] == 0.0
    assert cmd["phase"] == "lost"
    assert "lost paint" in cmd["reason"]
    assert "stop" in cmd["reason"]


def test_peak_red_finds_thin_stripe_mean_misses():
    """A 2 cm floor line is a few pixels. Averaging the frame hides it."""
    w, h = 16, 8
    img = bytearray(w * h * 4)
    for i in range(w * h):
        img[i * 4 : i * 4 + 4] = bytes((128, 128, 128, 255))
    for x in range(w):
        o = (4 * w + x) * 4
        img[o : o + 4] = bytes((30, 40, 230, 255))
    assert peak_score_bgra(img, w, h, red_score) > 0.5
    assert red_score(mean_rgb_bgra(img, w, h)) < 0.25


def test_red_finish_commands_brake():
    cmd = lane_keep_command(0.0, 0.0, 0.8, red_thresh=0.28)
    assert cmd["brake"] is True
    assert cmd["left"] == 0.0
    assert cmd["right"] == 0.0


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def test_look_at_off_axis_yellow_eye():
    """Side cameras aim at y=+/-0.65, not down the nose."""
    cam = (0.08, -0.55, 0.14)
    puck = (0.40, -0.65, 0.02)
    rot = look_at_sf_rotation(cam, puck)
    look = apply_sf_rotation(rot, (0.0, 0.0, -1.0))
    want = _unit((puck[0] - cam[0], puck[1] - cam[1], puck[2] - cam[2]))
    assert math.sqrt(sum((a - b) ** 2 for a, b in zip(look, want))) < 1e-5
    assert look[1] < 0.0


def test_look_at_points_camera_minus_z_at_puck():
    cam = (0.10, 0.04, 0.32)
    puck = (1.00, 0.04, 0.03)
    rot = look_at_sf_rotation(cam, puck)
    look = apply_sf_rotation(rot, (0.0, 0.0, -1.0))
    want = _unit((puck[0] - cam[0], puck[1] - cam[1], puck[2] - cam[2]))
    assert look[0] > 0.8
    assert look[2] < 0.0
    assert math.sqrt(sum((a - b) ** 2 for a, b in zip(look, want))) < 1e-5


def test_look_at_is_not_the_old_horizon_guess():
    """Identity looks down. A 1 m floor puck must not aim at +Z (sky)."""
    rot = look_at_sf_rotation((0.10, 0.04, 0.32), (1.00, 0.04, 0.03))
    look = apply_sf_rotation(rot, (0.0, 0.0, -1.0))
    assert look[2] < -0.2
    assert abs(look[1]) < 0.05


def test_beam_plus_z_follows_puck():
    a = (0.10, 0.04, 0.32)
    b = (1.00, 0.04, 0.03)
    rot = beam_sf_rotation(a, b)
    along = apply_sf_rotation(rot, (0.0, 0.0, 1.0))
    want = _unit((0.90, 0.0, -0.29))
    assert math.sqrt(sum((x - y) ** 2 for x, y in zip(along, want))) < 1e-5


def test_classify_view_sky_floor_paint():
    assert classify_view(SKY_RGB) == "sky"
    assert classify_view((0.55, 0.56, 0.58)) == "ground"
    assert classify_view((0.95, 0.95, 0.20)) == "yellow"
    assert classify_view((0.90, 0.15, 0.12)) == "red"


def test_ground_hit_is_ahead_of_the_puck_on_the_floor():
    cam = (0.10, 0.04, 0.32)
    puck = (1.00, 0.04, 0.03)
    d = ground_hit_ahead_m(cam, puck)
    assert 1.05 < d < 1.20


def test_time_to_mark_is_distance_over_speed():
    v = cruise_speed_m_s(5.5)
    assert abs(v - 0.44) < 1e-9
    t = time_to_mark_s(1.10, v)
    assert t is not None
    assert abs(t - 1.10 / 0.44) < 1e-9


def test_red_seen_coasts_then_brakes_on_the_mark():
    v = 0.44
    d_look = 1.10
    tracker = MarkStopTracker()
    first = tracker.step(True, d_look, v, 0.0)
    assert first["brake"] is False
    assert first["phase"] == "coast"
    assert first["remaining_m"] == 1.10
    cmd = lane_keep_command(0.5, 0.5, 0.8, mark_plan=first)
    assert cmd["brake"] is False
    assert cmd["reason"] == "red seen, coast to mark"
    # Almost on the stripe
    last = tracker.step(True, d_look, v, 0.0, measured_range_m=stopping_distance_m(v))
    assert last["brake"] is True
    cmd = lane_keep_command(0.5, 0.5, 0.8, mark_plan=last)
    assert cmd["brake"] is True
    assert cmd["reason"] == "red mark — stop on line"


def test_live_red_range_overrides_puck_distance():
    """First-see at 0.19 m (this morning) must not coast another 1.09 m."""
    tracker = MarkStopTracker()
    plan = tracker.step(True, 1.09, 0.44, 0.008, measured_range_m=0.19)
    assert plan["phase"] == "coast"
    assert abs(plan["remaining_m"] - 0.19) < 1e-6
    stuck = tracker.step(True, 1.09, 0.44, 0.1, measured_range_m=0.65)
    assert stuck["remaining_m"] < 0.19
    stop = tracker.step(True, 1.09, 0.44, 0.008, measured_range_m=0.05)
    assert stop["brake"] is True


def test_center_image_row_hits_near_the_puck():
    cam = (0.10, 0.04, 0.32)
    puck = (1.00, 0.04, 0.03)
    rot = look_at_sf_rotation(cam, puck)
    d_puck = ground_hit_ahead_m(cam, puck)
    d_row = image_row_to_ground_ahead_m(cam, rot, 19.5, 40, 0.9)
    assert d_row is not None
    assert abs(d_row - d_puck) < 0.08
    d_near = image_row_to_ground_ahead_m(cam, rot, 39, 40, 0.9)
    assert d_near is not None
    assert d_near < d_puck * 0.6


def test_down_look_center_pixel_hits_under_the_camera():
    cam = (0.30, 0.65, 0.12)
    rot = (0.0, 1.0, 0.0, 0.0)
    hit = pixel_to_ground_m(cam, rot, 31.5, 31.5, 64, 64, 1.2)
    assert hit is not None
    assert abs(hit[0] - 0.30) < 0.05
    assert abs(hit[1] - 0.65) < 0.05


def test_steer_from_ranges_pushes_off_the_close_wall():
    """Left wall at 0.20 m, right still at 0.65 → yaw right."""
    s = steer_from_ranges(0.20, -0.65)
    assert s is not None and s > 0.3
    centered = steer_from_ranges(0.65, -0.65)
    assert centered == 0.0
    none = steer_from_ranges(None, None)
    assert none is None


def test_sliding_picture_is_not_ignored_when_meters_look_fine():
    """Every frame's yellow slide counts. Frozen spawn meters must not mute it."""
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        left_offset=-0.40,
        right_offset=-0.40,
        left_y_m=0.65,
        right_y_m=-0.65,
    )
    assert cmd["brake"] is False
    assert cmd["steer"] is not None and cmd["steer"] < -0.2
    assert cmd["right"] > cmd["left"]


def test_geometry_ranges_win_over_offset_law():
    """A wall that is actually close in meters still wins over the picture."""
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        left_offset=-0.4,
        left_y_m=0.20,
        right_y_m=-0.65,
    )
    assert cmd["brake"] is False
    assert cmd["left"] > cmd["right"]
    assert cmd["steer"] > 0


def test_watch_stops_on_implausible_lane_width():
    watch = CorridorWatch(hold_s=0.15)
    last = None
    for _ in range(40):
        last = watch.step(0.10, -0.12, 0.0, 0.008)
    assert last is not None and last["abort"] is True
    assert "width" in (last["reason"] or "")


def test_watch_stops_when_shove_makes_the_wall_closer():
    watch = CorridorWatch(hold_s=0.15)
    last = None
    y = 0.50
    for _ in range(50):
        y -= 0.004
        last = watch.step(y, -0.65, 0.40, 0.008)
    assert last is not None and last["abort"] is True
    assert "closer" in (last["reason"] or "")


def test_watch_does_not_trip_on_one_bad_frame():
    watch = CorridorWatch(hold_s=0.20)
    one = watch.step(0.10, -0.10, 0.0, 0.008)
    assert one["abort"] is False


def test_lane_keep_watch_abort_brakes():
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        watch_plan={"abort": True, "reason": "lane width implausible"},
    )
    assert cmd["brake"] is True
    assert cmd["phase"] == "watch"
    assert "watch" in cmd["reason"]


def test_planner_harder_when_error_grows():
    p = GapPlanner(sample_s=0.10, kp=1.0, kd=0.22)
    p.step(0.10, 0.10)
    grew = p.step(0.30, 0.10)
    p2 = GapPlanner(sample_s=0.10, kp=1.0, kd=0.22)
    p2.step(0.30, 0.10)
    shrank = p2.step(0.10, 0.10)
    assert grew["mode"] == "correct"
    assert grew["desired_steer"] > 0.30
    assert shrank["desired_steer"] < grew["desired_steer"]


def test_planner_cruises_when_gap_is_fine():
    p = GapPlanner()
    out = p.step(0.004, 0.10)
    assert out["mode"] == "cruise"
    assert out["desired_steer"] == 0.0


def test_planner_d_does_not_reverse_the_now_pull():
    """Shrinking error may ease the wheel. It must not steer the other way."""
    p = GapPlanner(sample_s=0.10, kp=1.0, kd=2.5)
    p.step(0.50, 0.10)
    out = p.step(0.10, 0.10)
    assert out["mode"] == "correct"
    assert out["desired_steer"] > 0.0
    assert out["desired_steer"] < 0.10 * 1.0


def test_planner_recover_on_lost_paint():
    p = GapPlanner()
    out = p.step(0.40, 0.10, lost=True)
    assert out["mode"] == "recover"
    assert out["desired_steer"] == 0.0


def test_nearest_yellow_is_the_bottom_of_the_box():
    w, h = 32, 32
    img = _bgra_bar(w, h, 10, 14)
    pix = yellow_nearest_pixel(img, w, h)
    assert pix is not None
    _col, row = pix
    y0, y1 = yellow_look_band(h)
    assert row == y1 - 1
    ahead = yellow_ahead_pixel(img, w, h)
    assert ahead is not None
    assert ahead[1] == y0


def _bgra_fill(w: int, h: int, rgb, y0: int = 0, y1: int | None = None) -> bytearray:
    """Solid BGRA rectangle. rgb is 0–1."""
    img = bytearray(w * h * 4)
    y1 = h if y1 is None else y1
    b = int(rgb[2] * 255)
    g = int(rgb[1] * 255)
    r = int(rgb[0] * 255)
    for y in range(h):
        for x in range(w):
            o = (y * w + x) * 4
            if y0 <= y < y1:
                img[o : o + 4] = bytes((b, g, r, 255))
            else:
                img[o : o + 4] = bytes((140, 140, 140, 255))
    return img


def test_far_band_is_dirt_just_under_horizon():
    """Second measurement sits under heaven, above the contact patch."""
    h = 64
    horizon = yellow_horizon_row(h)
    y0, y1 = yellow_look_band(h)
    fy0, fy1 = yellow_far_band(h)
    ny0, ny1 = yellow_near_band(h)
    split = yellow_look_split(h)
    assert horizon <= 26
    assert fy0 >= horizon
    assert fy0 >= y0
    assert fy1 <= ny0
    assert ny1 == y1
    assert fy0 < fy1
    assert ny0 < ny1
    assert y0 < split < y1
    assert (fy1 - fy0) <= (ny1 - ny0)


def test_far_offset_reads_only_the_far_sliver():
    w, h = 32, 32
    fy0, fy1 = yellow_far_band(h)
    ny0, ny1 = yellow_near_band(h)
    img = bytearray(w * h * 4)
    for y in range(h):
        for x in range(w):
            o = (y * w + x) * 4
            img[o : o + 4] = bytes((140, 140, 140, 255))
            if fy0 <= y < fy1 and 2 <= x < 6:
                img[o : o + 4] = bytes((50, 240, 240, 255))
            if ny0 <= y < ny1 and 24 <= x < 28:
                img[o : o + 4] = bytes((50, 240, 240, 255))
    far = yellow_far_offset(img, w, h)
    assert far is not None and far < -0.4


def test_preview_rejects_a_sky_sliver():
    """Heaven is not a wall. Drop the second measurement."""
    w, h = 32, 32
    fy0, fy1 = yellow_far_band(h)
    y0, y1 = yellow_look_band(h)
    sky = _bgra_fill(w, h, SKY_RGB, 0, h)
    assert preview_band_ok(sky, w, h) is False
    dirt = _bgra_fill(w, h, (0.55, 0.56, 0.58), y0, y1)
    # Floor in the dirt box, sky everywhere else. Far sliver is floor.
    for y in range(h):
        for x in range(w):
            if y < y0 or y >= y1:
                o = (y * w + x) * 4
                dirt[o : o + 4] = bytes((209, 194, 184, 255))
    for y in range(fy0, fy1):
        for x in range(14, 18):
            o = (y * w + x) * 4
            dirt[o : o + 4] = bytes((50, 240, 240, 255))
    assert preview_band_ok(dirt, w, h) is True
    assert dirt_band_ok(dirt, w, h) is True


def test_nadir_far_band_is_not_a_preview_range():
    """Identity look-down hits under the mount. That is not look-ahead."""
    cam = (0.30, 0.65, 0.12)
    rot = (0.0, 1.0, 0.0, 0.0)
    assert preview_look_ahead_m(cam, rot, 64, 1.2) is None


def test_forecast_z_look_is_forward_not_rolled():
    """Nose cam at the 2 m left wall puck: look +X. Roll keeps aim."""
    cam = (0.18, 0.12, 0.22)
    puck = (2.00, 0.65, 0.02)
    rot = look_at_sf_rotation(cam, puck)
    look = apply_sf_rotation(rot, (0.0, 0.0, -1.0))
    assert look[0] > 0.85
    assert look[1] > 0.0
    assert look[2] < 0.0
    rolled = roll_camera_sf(rot, FORECAST_IMAGE_ROLL_RAD)
    look2 = apply_sf_rotation(rolled, (0.0, 0.0, -1.0))
    assert abs(look2[0] - look[0]) < 1e-5
    assert abs(look2[1] - look[1]) < 1e-5
    assert abs(look2[2] - look[2]) < 1e-5


def test_forecast_gap_error_equal_walls_are_zero():
    eq = forecast_gap_error(0.65, -0.65)
    assert eq is not None and abs(eq) < 0.02
    assert forecast_gap_error(None, None) is None


def test_forecast_gap_error_close_left_steers_right():
    e = forecast_gap_error(0.30, -0.65)
    assert e is not None and e > 0.20


def test_forecast_wall_hit_identity_finds_the_floor():
    w = h = 32
    img = _bgra_bar(w, h, 14, 18)
    hit = forecast_wall_hit(
        img, w, h, (0.16, 0.06, 0.20), (0.0, 1.0, 0.0, 0.0), 1.2
    )
    assert hit is not None
    assert abs(hit["ahead_m"] - 0.16) < 0.20
    assert hit["y_m"] is not None


def test_forecast_meters_do_not_steer():
    """Z/W y_m is 12 cm of floor. Picture stays in charge."""
    p = GapPlanner(sample_s=0.01, kp=1.0, kd=0.0, kahead=0.45)
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        left_offset=0.0,
        right_offset=0.0,
        z_y_m=0.30,
        w_y_m=-0.65,
        t_ahead=1.5,
        planner=p,
        dt=0.02,
    )
    assert cmd["err_ahead"] is None
    assert abs(float(cmd["steer"] or 0.0)) < 0.05


def test_lookout_does_not_abort_before_it_has_seen_paint():
    look = ForecastLookout(hold_s=0.05)
    cmd = lane_keep_command(
        0.53,
        0.53,
        0.0,
        left_fill=0.08,
        right_fill=0.08,
        lookout=look,
        z_fill=0.01,
        w_fill=0.01,
        dt=0.08,
    )
    assert cmd["brake"] is False
    assert cmd["phase"] != "lookout"


def test_lookout_stops_when_both_ahead_stripes_vanish():
    """Now-eyes still happy. Z/W both lost the stripe. Stop. No steer vote."""
    look = ForecastLookout(hold_s=0.05)
    lane_keep_command(
        0.53,
        0.53,
        0.0,
        left_fill=0.08,
        right_fill=0.08,
        left_offset=0.0,
        right_offset=0.0,
        lookout=look,
        z_fill=0.08,
        w_fill=0.08,
        dt=0.02,
    )
    still = lane_keep_command(
        0.53,
        0.53,
        0.0,
        left_fill=0.08,
        right_fill=0.08,
        left_offset=0.0,
        right_offset=0.0,
        lookout=look,
        z_fill=0.01,
        w_fill=0.08,
        dt=0.08,
    )
    assert still["brake"] is False
    gone = lane_keep_command(
        0.53,
        0.53,
        0.0,
        left_fill=0.08,
        right_fill=0.08,
        left_offset=0.11,
        right_offset=-0.81,
        lookout=look,
        z_fill=0.01,
        w_fill=0.01,
        dt=0.08,
    )
    assert gone["brake"] is True
    assert gone["phase"] == "lookout"
    assert gone["left"] == 0.0 and gone["right"] == 0.0
    assert "ahead" in gone["reason"]


def test_tile_speckle_is_lost_paint():
    """Peak gold on chequered tile is not a stripe. Stop."""
    cmd = lane_keep_command(
        0.53,
        0.53,
        0.0,
        left_fill=0.016,
        right_fill=0.016,
        left_offset=0.11,
        right_offset=-0.81,
        cruise=5.5,
    )
    assert cmd["brake"] is True
    assert cmd["phase"] == "lost"
    keep = lane_keep_command(
        0.53,
        0.53,
        0.0,
        left_fill=0.08,
        right_fill=0.08,
        left_offset=0.08,
        right_offset=-0.22,
        cruise=5.5,
    )
    assert keep["brake"] is False


def test_rotate_bgra_90_cw_moves_top_pixel_to_the_right():
    w = h = 2
    img = bytearray(w * h * 4)
    # top-left pixel red
    img[0:4] = bytes((0, 0, 255, 255))
    out, nw, nh = rotate_bgra_90_cw(img, w, h)
    assert (nw, nh) == (2, 2)
    # CW: top-left -> top-right
    o = (0 * 2 + 1) * 4
    assert out[o + 2] == 255


def test_fov_pyramid_has_apex_and_four_corners():
    cam = (0.18, 0.12, 0.22)
    rot = look_at_sf_rotation(cam, (2.00, 0.65, 0.02))
    pts = camera_fov_pyramid(cam, rot, 0.85, 64, 64, 2.0)
    assert len(pts) == 5
    assert abs(pts[0][0] - 0.18) < 1e-9
    for p in pts[1:]:
        assert p[0] > cam[0]


def test_yellow_aim_look_is_forward_not_rolled():
    """2 m down the stripe: look +X, image-up is sky. Not the 90° trap."""
    cam = (0.30, 0.65, 0.12)
    puck = (2.00, 0.65, 0.02)
    rot = look_at_sf_rotation(cam, puck)
    look = apply_sf_rotation(rot, (0.0, 0.0, -1.0))
    up = apply_sf_rotation(rot, (0.0, 1.0, 0.0))
    assert look[0] > 0.90
    assert abs(look[1]) < 0.05
    assert look[2] < 0.0
    assert up[2] > 0.90
    d = preview_look_ahead_m(cam, rot, 64, 1.2)
    assert d is not None
    assert 0.40 <= d <= 4.00


def test_preview_ahead_weight_grows_when_time_is_short():
    assert preview_ahead_weight(None) == 0.0
    assert preview_ahead_weight(0.0) == 0.0
    assert preview_ahead_weight(4.0) < preview_ahead_weight(1.0)


def test_planner_turns_early_when_ahead_disagrees():
    """Centered now, far sliver already sliding left → start the turn."""
    now = GapPlanner(sample_s=0.10, kp=1.0, kd=0.0, kahead=0.45)
    only_now = now.step(0.004, 0.10)
    ahead = GapPlanner(sample_s=0.10, kp=1.0, kd=0.0, kahead=0.45)
    with_ahead = ahead.step(0.004, 0.10, error_ahead=-0.50, t_ahead=1.5)
    assert only_now["mode"] == "cruise"
    assert with_ahead["mode"] == "correct"
    assert with_ahead["desired_steer"] < -0.08


def test_planner_ignores_ahead_without_a_time():
    p = GapPlanner(sample_s=0.10, kp=1.0, kd=0.0, kahead=0.45)
    out = p.step(0.004, 0.10, error_ahead=-0.80, t_ahead=None)
    assert out["mode"] == "cruise"
    assert out["desired_steer"] == 0.0


def test_lane_keep_drops_ahead_when_preview_is_rejected():
    """Sky / no-range interlock: far offsets must not yank the wheel."""
    off = lane_keep_command(
        0.5,
        0.5,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        left_offset=0.0,
        right_offset=0.0,
        left_far_offset=-0.70,
        right_far_offset=-0.70,
        t_ahead=1.0,
        preview_ok=False,
    )
    on = lane_keep_command(
        0.5,
        0.5,
        0.0,
        cruise=5.5,
        k_steer=2.0,
        left_offset=0.0,
        right_offset=0.0,
        left_far_offset=-0.70,
        right_far_offset=-0.70,
        t_ahead=1.0,
        preview_ok=True,
        planner=GapPlanner(sample_s=0.01, kp=1.0, kd=0.0, kahead=0.45),
        dt=0.02,
    )
    assert abs(float(off.get("steer") or 0.0)) < 0.05
    assert on["preview_ok"] is True
    assert on["steer"] is not None and on["steer"] < -0.08


def test_frozen_knobs_are_the_week_freeze():
    assert PLANNER_KAHEAD == 0.22
    assert DEFAULT_STEER_RELEASE_PER_S == 4.0


def test_metric_ct_positive_when_left_wall_is_closer():
    """Robot left of center → +ct → steer right."""
    assert metric_ct_from_walls(0.65, 0.65) == 0.0
    ct = metric_ct_from_walls(0.40, 0.80)
    assert ct is not None and ct > 0.15
    assert metric_ct_from_walls(None, 0.65) is None


def test_metric_walls_plausible_rejects_wreck_and_sky():
    assert metric_walls_plausible(0.65, 0.65) is True
    assert metric_walls_plausible(0.05, 0.65) is False
    assert metric_walls_plausible(0.65, 2.0) is False
    assert metric_walls_plausible(None, 0.65) is False


def test_yellow_wall_pixel_prefers_the_near_band():
    w, h = 32, 32
    img = bytearray(w * h * 4)
    for y in range(h):
        for x in range(w):
            o = (y * w + x) * 4
            img[o : o + 4] = bytes((140, 140, 140, 255))
    ny0, ny1 = yellow_near_band(h)
    y0, _y1 = yellow_look_band(h)
    # Far-ish look-band paint on the left, near-band paint on the right.
    for y in range(y0, ny0):
        for x in range(4, 8):
            o = (y * w + x) * 4
            img[o : o + 4] = bytes((50, 240, 240, 255))
    for y in range(ny0, ny1):
        for x in range(22, 26):
            o = (y * w + x) * 4
            img[o : o + 4] = bytes((50, 240, 240, 255))
    pix = yellow_wall_pixel(img, w, h)
    assert pix is not None
    col, row = pix
    assert ny0 <= row < ny1
    assert col >= 20


def test_line_wall_hit_identity_is_about_a_half_lane():
    """Nadir math on a cam bolted over the stripe → ~0.65 m."""
    w = h = 32
    img = _bgra_bar(w, h, 14, 18)
    hit = line_wall_hit(
        img,
        w,
        h,
        side="left",
        cam_pos=(0.30, 0.65, 0.12),
        cam_rot=(0.0, 1.0, 0.0, 0.0),
        fov_rad=1.2,
    )
    assert hit is not None
    assert 0.45 <= hit["dist_m"] <= 0.85


def test_metric_path_steers_right_when_left_wall_is_close():
    p = GapPlanner(sample_s=0.01, kp=1.0, kd=0.0, kahead=0.22)
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        left_offset=0.0,
        right_offset=0.0,
        left_fill=0.08,
        right_fill=0.08,
        left_wall_dist_m=0.35,
        right_wall_dist_m=0.85,
        planner=p,
        dt=0.08,
    )
    assert cmd["metric_active"] is True
    assert cmd["error_source"] == "metric"
    assert cmd["steer"] > 0.10
    assert cmd["left"] > cmd["right"]


def test_metric_falls_back_when_meters_are_implausible():
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        left_offset=-0.40,
        right_offset=-0.40,
        left_fill=0.08,
        right_fill=0.08,
        left_wall_dist_m=0.05,
        right_wall_dist_m=0.65,
    )
    assert cmd["metric_active"] is False
    assert cmd["error_source"] == "picture"
    assert cmd["steer"] is not None and cmd["steer"] < -0.2


def test_metric_falls_back_when_frozen_meters_disagree_with_picture():
    """Spawn-like 0.65/0.65 must not mute a sliding picture."""
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        left_offset=-0.40,
        right_offset=-0.40,
        left_fill=0.08,
        right_fill=0.08,
        left_wall_dist_m=0.65,
        right_wall_dist_m=0.65,
    )
    assert cmd["metric_active"] is False
    assert cmd["error_source"] == "picture"
    assert cmd["steer"] is not None and cmd["steer"] < -0.2


def test_zw_meters_still_do_not_steer():
    p = GapPlanner(sample_s=0.01, kp=1.0, kd=0.0, kahead=0.22)
    cmd = lane_keep_command(
        0.5,
        0.5,
        0.0,
        left_offset=0.0,
        right_offset=0.0,
        left_fill=0.08,
        right_fill=0.08,
        z_y_m=0.30,
        w_y_m=-0.65,
        t_ahead=1.5,
        planner=p,
        dt=0.02,
    )
    assert cmd["err_ahead"] is None
    assert abs(float(cmd["steer"] or 0.0)) < 0.05
