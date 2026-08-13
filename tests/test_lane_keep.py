from src.lane_keep import (
    lane_keep_command,
    mean_rgb_bgra,
    peak_score_bgra,
    red_score,
    yellow_score,
)


def test_yellow_high_on_paint_low_on_gray():
    assert yellow_score((0.95, 0.95, 0.20)) > 0.6
    assert yellow_score((0.50, 0.50, 0.52)) < 0.1


def test_red_high_on_finish_low_on_yellow_and_gray():
    assert red_score((0.90, 0.15, 0.12)) > 0.6
    assert red_score((0.95, 0.95, 0.20)) < 0.1
    assert red_score((0.50, 0.50, 0.52)) < 0.1


def test_steer_right_when_left_sees_more_yellow():
    cmd = lane_keep_command(0.7, 0.0, 0.0, cruise=5.5, k_steer=2.0)
    assert cmd["brake"] is False
    assert cmd["left"] > cmd["right"]


def test_steer_left_when_right_sees_more_yellow():
    cmd = lane_keep_command(0.0, 0.7, 0.0, cruise=5.5, k_steer=2.0)
    assert cmd["right"] > cmd["left"]


def test_straight_when_no_yellow():
    cmd = lane_keep_command(0.0, 0.0, 0.0, cruise=5.5)
    assert cmd["left"] == cmd["right"] == 5.5
    assert cmd["brake"] is False


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
