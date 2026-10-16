"""Shared wheel math and the painted-S centre line.

The pixel-gap keeper tests left with that keeper.
"""

from src.lane_keep import (
    SteerFilter,
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
