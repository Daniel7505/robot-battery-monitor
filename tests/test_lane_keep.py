"""Nadir-only lane keep. Choir suite is at archives/test_lane_keep_choir_2026-09-02.py."""

from src.lane_keep import (
    NADIR_AHEAD_L_PX,
    NADIR_AHEAD_R_PX,
    NADIR_BASE_L_PX,
    NADIR_BASE_R_PX,
    NADIR_V_SCALE_MAX,
    NadirGuard,
    SteerFilter,
    lane_keep_command,
    nadir_wheel_to_tape,
    steer_from_nadir_gaps,
    stripe_m_per_px,
    track_cross_track_m,
    wheels_from_steer,
    yellow_score,
)


def test_yellow_high_on_paint_low_on_gray():
    assert yellow_score((0.95, 0.95, 0.20)) > 0.6
    assert yellow_score((0.50, 0.50, 0.52)) < 0.1


def test_cross_track_is_vs_s_centerline_not_world_y0():
    # First-lobe crest (x = start + lobe/2 = 5.5) sits at y = -1.
    ct = track_cross_track_m(5.5, -1.0)
    assert abs(ct) < 0.05
    ct_world_zero = track_cross_track_m(5.5, 0.0)
    assert ct_world_zero > 0.8


def test_nadir_deadband_holds_straight_at_bases():
    steer, err = steer_from_nadir_gaps(NADIR_BASE_L_PX, NADIR_BASE_R_PX, cruise=5.5)
    assert err == 0.0
    assert steer == 0.0


def test_nadir_left_gap_drop_yaws_right():
    # Closer to left tape → positive steer (yaw right).
    steer, _err = steer_from_nadir_gaps(19, 37, cruise=2.5)
    assert steer is not None and steer > 0.05


def test_nadir_right_gap_drop_yaws_left():
    steer, _err = steer_from_nadir_gaps(45, 18, cruise=2.5)
    assert steer is not None and steer < -0.05


def test_ahead_hold_yaws_before_axle_moves():
    straight, _ = steer_from_nadir_gaps(
        NADIR_BASE_L_PX,
        NADIR_BASE_R_PX,
        left_ahead_px=NADIR_AHEAD_L_PX,
        right_ahead_px=NADIR_AHEAD_R_PX,
        cruise=2.5,
    )
    walking, _ = steer_from_nadir_gaps(
        NADIR_BASE_L_PX,
        NADIR_BASE_R_PX,
        left_ahead_px=NADIR_AHEAD_L_PX,
        right_ahead_px=NADIR_AHEAD_R_PX + 5,
        cruise=2.5,
    )
    assert straight == 0.0
    assert walking is not None and walking > 0.0


def test_v_scale_raises_steer_at_champ_cruise():
    slow, _ = steer_from_nadir_gaps(10, NADIR_BASE_R_PX, cruise=2.5)
    fast, _ = steer_from_nadir_gaps(10, NADIR_BASE_R_PX, cruise=5.5)
    assert slow is not None and fast is not None
    assert fast > slow
    assert fast / slow <= NADIR_V_SCALE_MAX + 0.05


def test_nadir_guard_stops_when_both_eyes_gone():
    g = NadirGuard()
    halt = g.step(None, None)
    assert halt["abort"] is True
    cmd = lane_keep_command(left_gap_px=None, right_gap_px=None, nadir_guard=g)
    assert cmd["brake"] is True
    assert cmd["error_source"] == "nadir"


def test_lane_keep_command_fights_bases():
    cmd = lane_keep_command(
        left_gap_px=NADIR_BASE_L_PX,
        right_gap_px=NADIR_BASE_R_PX,
        left_ahead_px=NADIR_AHEAD_L_PX,
        right_ahead_px=NADIR_AHEAD_R_PX,
        cruise=5.5,
        k_steer=2.0,
    )
    assert cmd["brake"] is False
    assert cmd["steer"] == 0.0
    assert cmd["error_source"] == "nadir"
    assert abs(cmd["left"] - cmd["right"]) < 1e-6


def test_steer_filter_releases_slower_than_it_grabs():
    f = SteerFilter(slew_per_s=12.0, release_per_s=4.0)
    grabbed = f.step(0.4, 0.05)
    f.value = 0.4
    released = abs(f.step(0.0, 0.05) - 0.4)
    assert grabbed > 0.2
    assert released < grabbed


def test_wheels_stay_forward_on_a_turn():
    left, right = wheels_from_steer(0.3, cruise=5.5, k_steer=2.0)
    assert left > 0.0 and right > 0.0
    assert left > right


def _paint_cols(w: int, h: int, yellow: range, dark: range) -> bytearray:
    img = bytearray(w * h * 4)
    for i in range(0, len(img), 4):
        img[i : i + 4] = bytes((128, 128, 128, 255))
    for row in range(h):
        for c in yellow:
            o = (row * w + c) * 4
            img[o : o + 4] = bytes((56, 150, 232, 255))
        for c in dark:
            o = (row * w + c) * 4
            img[o : o + 4] = bytes((20, 20, 20, 255))
    return img


def test_stripe_stretch_grows_m_per_px_when_tape_shrinks():
    near = stripe_m_per_px(4)
    far = stripe_m_per_px(2)
    assert near is not None and far is not None
    assert abs(near - 0.06 / 4) < 1e-12
    assert abs(far - 0.06 / 2) < 1e-12
    assert far == 2.0 * near


def test_ahead_band_uses_its_own_stripe_not_the_axle():
    """Same gap px, thinner far stripe → more meters up-track. Steer px unchanged."""
    w = h = 64
    img = bytearray(w * h * 4)
    for i in range(0, len(img), 4):
        img[i : i + 4] = bytes((128, 128, 128, 255))
    for row in range(h):
        if row < h // 2 - 6:
            yellow, dark = range(20, 22), range(41, 46)
        else:
            yellow, dark = range(18, 22), range(41, 46)
        for c in yellow:
            o = (row * w + c) * 4
            img[o : o + 4] = bytes((56, 150, 232, 255))
        for c in dark:
            o = (row * w + c) * 4
            img[o : o + 4] = bytes((20, 20, 20, 255))
    hit = nadir_wheel_to_tape(img, w, h)
    assert hit is not None
    assert hit["gap_ahead_px"] == hit["gap_axle_px"]
    assert hit["stripe_px_ahead"] == 2
    assert hit["stripe_px_axle"] == 4
    assert hit["m_ahead"] is not None and hit["m_axle"] is not None
    assert abs(hit["m_ahead"] - 2.0 * hit["m_axle"]) < 1e-9


def test_right_yellow_ruler_counts_wheel_left_of_tape():
    img = _paint_cols(64, 64, range(48, 52), range(12, 17))
    hit = nadir_wheel_to_tape(img, 64, 64, side="right")
    assert hit is not None
    assert hit["gap_px"] == 48 - 16
