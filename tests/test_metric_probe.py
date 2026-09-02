"""20 cm shove probe: old LINE_CAM proxy is dead; drawing-2 nadir is not."""

from src.lane_keep import (
    NADIR_FOV_RAD,
    NADIR_MOUNT_POS,
    NADIR_MOUNT_ROT,
    PAINT_Y_LEFT_M,
    NadirGuard,
    image_row_to_ground_ahead_m,
    nadir_axle_row,
    nadir_wheel_to_tape,
    pixel_to_ground_m,
    steer_from_nadir_gaps,
)
from src.metric_probe import (
    SHOVE_M,
    frozen_spawn_dist_m,
    line_cam_identity_dist_m,
    low_nadir_paint_y_m,
    nadir_gap_m,
    nadir_paint_y_m,
    picture_offset_as_meters,
    probe_lateral_metric,
    render_floor_bgra,
)


def test_frozen_spawn_meters_fail_the_shove_probe():
    result = probe_lateral_metric(frozen_spawn_dist_m)
    assert result.validated is False
    assert result.sensitive is False
    assert "flat" in result.reason or "unvalidated" in result.reason


def test_line_cam_identity_fails_the_shove_probe():
    """12 cm identity over the stripe cannot measure a 20 cm body offset."""
    result = probe_lateral_metric(line_cam_identity_dist_m)
    assert result.validated is False


def test_picture_offset_is_not_a_meter_metric():
    result = probe_lateral_metric(picture_offset_as_meters)
    assert result.validated is False


def test_low_nadir_at_line_cam_height_fails():
    """Height is data: 12 cm at the gap cannot hold wheel and tape."""
    result = probe_lateral_metric(low_nadir_paint_y_m)
    assert result.validated is False


def test_yellow_ruler_is_not_a_frozen_31px():
    """31 px is a centered spawn reading. A 20 cm shove must shrink it."""
    img0 = render_floor_bgra(
        robot_xy=(0.0, 0.0),
        cam_pos=NADIR_MOUNT_POS,
        cam_rot=NADIR_MOUNT_ROT,
        width=64,
        height=64,
        fov_rad=NADIR_FOV_RAD,
    )
    img1 = render_floor_bgra(
        robot_xy=(0.0, 0.20),
        cam_pos=NADIR_MOUNT_POS,
        cam_rot=NADIR_MOUNT_ROT,
        width=64,
        height=64,
        fov_rad=NADIR_FOV_RAD,
    )
    a = nadir_wheel_to_tape(img0, 64, 64)
    b = nadir_wheel_to_tape(img1, 64, 64)
    assert a is not None and b is not None
    assert a["gap_px"] >= 20
    assert b["gap_px"] < a["gap_px"] - 5
    assert abs((a["m"] - b["m"]) - 0.20) < 0.08


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


def test_nadir_ruler_splits_ahead_and_axle_bands():
    """Top half of the crop is ahead (~0.5 m). Median of all rows hid the walk."""
    w = h = 64
    img = bytearray(w * h * 4)
    for i in range(0, len(img), 4):
        img[i : i + 4] = bytes((128, 128, 128, 255))
    for row in range(h):
        if row < h // 2:
            yellow, dark = range(20, 24), range(44, 49)
        else:
            yellow, dark = range(12, 16), range(44, 49)
        for c in yellow:
            o = (row * w + c) * 4
            img[o : o + 4] = bytes((56, 150, 232, 255))
        for c in dark:
            o = (row * w + c) * 4
            img[o : o + 4] = bytes((20, 20, 20, 255))
    hit = nadir_wheel_to_tape(img, w, h)
    assert hit is not None
    assert hit["gap_ahead_px"] == 44 - 23
    assert hit["gap_axle_px"] == 44 - 15


def test_right_yellow_ruler_counts_wheel_left_of_tape():
    """Right eye: dark wheel then yellow. Scale from 6 cm stripe, not spawn Y."""
    img = _paint_cols(64, 64, range(48, 52), range(12, 17))
    hit = nadir_wheel_to_tape(img, 64, 64, side="right")
    assert hit is not None
    assert hit["stripe_px"] == 4
    assert hit["gap_px"] == 48 - 16
    assert abs(hit["m"] - (32 * 0.06 / 4)) < 1e-9


def test_nadir_fan_fights_32_and_29_not_the_average():
    s0, err0 = steer_from_nadir_gaps(32, 29)
    assert s0 == 0.0 and err0 == 0.0
    s_right, err_r = steer_from_nadir_gaps(19, 42)
    assert s_right is not None and s_right > 0.1
    s_left, err_l = steer_from_nadir_gaps(45, 16)
    assert s_left is not None and s_left < -0.1
    assert err_r > 0 and err_l < 0


def test_nadir_hold_yaws_when_axle_is_still_centered():
    """Forward-row walk must command before the axle gap leaves 32/29."""
    s0, err0 = steer_from_nadir_gaps(32, 29, left_ahead_px=32, right_ahead_px=26)
    assert s0 == 0.0 and err0 == 0.0
    s_hold, err = steer_from_nadir_gaps(
        32, 29, left_ahead_px=24, right_ahead_px=34
    )
    assert err == 0.0
    assert s_hold is not None and s_hold > 0.1


def test_nadir_fan_scales_p_with_cruise_not_d():
    """Same pixel error: faster cruise → stronger P, still under the cap."""
    slow, _ = steer_from_nadir_gaps(24, 37, cruise=2.5)
    fast, _ = steer_from_nadir_gaps(24, 37, cruise=5.5)
    assert slow is not None and fast is not None
    assert fast > slow
    assert fast <= 0.55


def test_nadir_unison_lie_stops():
    g = NadirGuard(unison_px=2, unison_frames=3)
    g.step(32, 29)
    g.step(28, 25)
    g.step(24, 21)
    out = g.step(20, 17)
    assert out["abort"] is True
    assert "unison" in out["reason"]


def test_yellow_ruler_meters_pass_the_20cm_shove():
    result = probe_lateral_metric(nadir_gap_m)
    assert result.validated is True
    assert result.delta_m is not None
    assert abs(abs(result.delta_m) - SHOVE_M) < 0.08


def test_nadir_paint_y_passes_the_20cm_shove():
    result = probe_lateral_metric(nadir_paint_y_m)
    assert result.center is not None
    assert abs(result.center - PAINT_Y_LEFT_M) < 0.04
    assert result.validated is True
    assert result.delta_m is not None
    assert abs(abs(result.delta_m) - SHOVE_M) < 0.05


def test_nadir_axle_row_hits_near_x_zero():
    row = nadir_axle_row(
        NADIR_MOUNT_POS, NADIR_MOUNT_ROT, 64, NADIR_FOV_RAD
    )
    ahead = image_row_to_ground_ahead_m(
        NADIR_MOUNT_POS, NADIR_MOUNT_ROT, row, 64, NADIR_FOV_RAD
    )
    assert ahead is not None
    assert abs(ahead) < 0.06


def test_nadir_center_pixel_is_under_the_gap_cam():
    hit = pixel_to_ground_m(
        NADIR_MOUNT_POS,
        NADIR_MOUNT_ROT,
        31.5,
        31.5,
        64,
        64,
        NADIR_FOV_RAD,
    )
    assert hit is not None
    assert abs(hit[0] - NADIR_MOUNT_POS[0]) < 0.05
    assert abs(hit[1] - NADIR_MOUNT_POS[1]) < 0.05


def test_nadir_render_sees_paint_and_wheel_at_spawn():
    img = render_floor_bgra(
        cam_pos=NADIR_MOUNT_POS,
        cam_rot=NADIR_MOUNT_ROT,
        width=64,
        height=64,
        fov_rad=NADIR_FOV_RAD,
    )
    # Paint gold BGRA ≈ (56, 150, 232). Wheel dark. Floor gray.
    paint = wheel = floor = 0
    for i in range(0, len(img), 4):
        b, g, r = img[i], img[i + 1], img[i + 2]
        if r > 180 and g > 100:
            paint += 1
        elif r < 50 and g < 50 and b < 50:
            wheel += 1
        elif abs(r - g) < 20 and abs(g - b) < 20:
            floor += 1
    assert paint >= 20
    assert wheel >= 8
    assert floor >= 100
