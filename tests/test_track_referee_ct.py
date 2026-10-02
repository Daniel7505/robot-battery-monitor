"""FinishReferee.cross_track: the controller's `Driving ... ct=` uses the loaded track."""
import pytest

from src.track_geometry import FinishReferee, load_track


def test_cross_track_on_corner90_legs():
    ref = FinishReferee(load_track("corner90"), source="test")
    assert ref.cross_track(2.0, 0.05) == pytest.approx(0.05, abs=1e-3)  # first leg, left of centre
    assert ref.cross_track(3.0, -0.10) == pytest.approx(-0.10, abs=1e-3)
    # second leg runs north: x 3.97 is 3 cm LEFT (west) of its centre x=4 (was +3.2 m vs the S)
    assert ref.cross_track(3.97, 3.0) == pytest.approx(0.03, abs=1e-3)
    assert ref.cross_track(4.05, 3.5) == pytest.approx(-0.05, abs=1e-3)


def test_cross_track_none_for_s_default():
    assert FinishReferee(None).cross_track(5.0, 0.3) is None
