"""Straightaway cruise bump: higher v* on clean, sure, clear straights only."""
import math
from pathlib import Path

import pytest

from src import rowfit_control as rc
from src.lane_keep import MAX_WHEEL_RAD_S, WHEEL_RADIUS_M
from src.lane_vision import LaneEstimate
from src.obstacle_gate import ObstacleGate

ROOT = Path(__file__).resolve().parent.parent
FRAME_TICKS = 40  # 320 ms frames at 8 ms ticks


def est(offset=0.0, heading=0.0, k=0.0, conf=0.95, look=1.9, corner=None, junction=None):
    psi = -heading
    a0 = -offset / max(1e-6, math.cos(psi))
    e = LaneEstimate(
        valid=True, offset_m=offset, heading_rad=heading, curvature_1pm=k,
        lookahead_m=look, confidence=conf, n_left=60, n_right=60,
        lane_width_m=1.3, width_measured=True, a0=a0, psi=psi, near_curvature_1pm=k,
    )
    e.corner = corner
    e.junction = junction
    return e


def drive(ctl, e, frames, **kw):
    cmd = None
    for i in range(frames * FRAME_TICKS):
        cmd = ctl.step(e, new_frame=i % FRAME_TICKS == 0, dt=0.008, **kw)
    return cmd


def test_caps_old_vs_new():
    # straight bump is small; every other cap is unchanged
    assert rc.V_CRUISE_M_S == pytest.approx(0.44)
    assert rc.V_STRAIGHT_M_S == pytest.approx(0.50)
    assert rc.V_STRAIGHT_M_S - rc.V_CRUISE_M_S <= 0.08
    assert rc.V_MAX_M_S == pytest.approx(rc.V_STRAIGHT_M_S)
    assert rc.V_JUNCTION_M_S == pytest.approx(0.25)
    assert rc.V_APPROACH_M_S == pytest.approx(0.25)
    assert rc.V_REACQUIRE_M_S == pytest.approx(0.20)
    assert rc.V_GAP_M_S == pytest.approx(0.15)
    assert rc.V_HELD_M_S == pytest.approx(0.30)
    assert rc.curve_speed(0.0) == pytest.approx(rc.V_CRUISE_M_S)  # the curve governor still tops at 0.44


def test_clean_straight_bumps_after_persistence():
    ctl = rc.RowfitController()
    e = est()
    cmd = drive(ctl, e, rc.STRAIGHT_FRAMES - 1)
    assert ctl.target_speed == pytest.approx(rc.V_CRUISE_M_S) and not cmd["straight_boost"]
    cmd = drive(ctl, e, 1)
    assert ctl.target_speed == pytest.approx(rc.V_STRAIGHT_M_S) and cmd["straight_boost"]
    cmd = drive(ctl, e, 12)
    assert cmd["speed_cmd"] == pytest.approx(rc.V_STRAIGHT_M_S, abs=1e-3)
    assert not cmd["brake"] and cmd["state"] == rc.STATE_LANE


@pytest.mark.parametrize("kw", [
    {"conf": 0.70},  # fit not sure enough
    {"k": 0.3},  # a gentle bend: curve governor only
    {"offset": 0.15},  # off centre, correcting
    {"heading": math.radians(8.0)},  # pointed off the lane
])
def test_no_bump_unless_straight_sure_and_centred(kw):
    ctl = rc.RowfitController()
    drive(ctl, est(**kw), 10)
    assert ctl.target_speed <= rc.V_CRUISE_M_S + 1e-9
    assert ctl.gov.v <= rc.V_CRUISE_M_S + 1e-6
    assert not ctl.straight_boost


def test_any_obstacle_or_pick_cap_cancels_the_bump():
    ctl = rc.RowfitController()
    drive(ctl, est(), 10)
    assert ctl.straight_boost
    ctl.obstacle_v_cap = 0.20  # SLOW_FOR_OBSTACLE
    drive(ctl, est(), 3)
    assert not ctl.straight_boost and ctl.gov.v == pytest.approx(0.20, abs=1e-3)
    ctl.obstacle_v_cap = 1.0  # even a cap above the bump means 'not CLEAR'
    drive(ctl, est(), 3)
    assert not ctl.straight_boost and ctl.target_speed <= rc.V_CRUISE_M_S + 1e-9
    ctl.obstacle_v_cap = 0.0  # STOPPED / WAITING / pick hold
    drive(ctl, est(), 3)
    assert ctl.gov.v == pytest.approx(0.0, abs=1e-6)
    ctl.obstacle_v_cap = None  # CLEAR: has to re-earn the persistence
    drive(ctl, est(), rc.STRAIGHT_FRAMES - 1)
    assert not ctl.straight_boost
    drive(ctl, est(), 1)
    assert ctl.straight_boost


def test_cap_set_between_frames_applies_immediately():
    ctl = rc.RowfitController()
    drive(ctl, est(), 10)
    ctl.obstacle_v_cap = 0.20
    cmd = None
    for _ in range(FRAME_TICKS - 1):  # no new frame yet
        cmd = ctl.step(None, new_frame=False, dt=0.008)
    assert ctl.target_speed == pytest.approx(0.20)
    assert cmd["speed_cmd"] < rc.V_STRAIGHT_M_S


def test_corner_or_junction_cue_cancels_the_bump():
    class _Cue:  # unconfirmed / low-conf cue: not enough for a state change, enough to drop the bump
        seen = False
        confidence = 0.2
        direction = 1
        distance_m = 1.8
        kind = "opening_L"
        near_m = 1.8
        events = {"R": ("", None)}

    for field in ("corner", "junction"):
        ctl = rc.RowfitController()
        drive(ctl, est(), 10)
        assert ctl.straight_boost
        drive(ctl, est(**{field: _Cue()}), 1)
        assert not ctl.straight_boost, field
        assert ctl.target_speed <= rc.V_CRUISE_M_S + 1e-9


def test_junction_slow_and_cooldowns_keep_their_caps():
    for attr, cap in (("jn_slow_m", rc.V_JUNCTION_M_S), ("jn_cooldown_m", rc.V_JUNCTION_M_S),
                      ("cooldown_m", rc.V_CRUISE_M_S)):
        ctl = rc.RowfitController()
        setattr(ctl, attr, 50.0)
        drive(ctl, est(), 10)
        assert not ctl.straight_boost, attr
        assert ctl.gov.v <= cap + 1e-6, attr


def test_reacquire_stays_slow_and_bump_resets_outside_lane():
    ctl = rc.RowfitController()
    drive(ctl, est(), 10)
    assert ctl.straight_boost
    ctl.state = rc.STATE_REACQUIRE
    cmd = drive(ctl, est(), 2)
    assert not ctl.straight_boost and ctl.straight_frames == 0
    assert cmd["speed_cmd"] <= rc.V_REACQUIRE_M_S + 1e-6


def test_approach_and_junction_caps_ignore_the_bump():
    ctl = rc.RowfitController()
    drive(ctl, est(), 10)
    assert ctl.gov.v == pytest.approx(rc.V_STRAIGHT_M_S, abs=1e-3)
    ctl.state, ctl.turn_src, ctl.remaining_m, ctl.corner_dir = rc.STATE_APPROACH, "corner", 3.0, 1
    cmd = drive(ctl, est(), 2, yaw=0.0)
    assert not ctl.straight_boost
    assert cmd["speed_cmd"] <= rc.V_APPROACH_M_S + 1e-6


def test_held_or_lost_frame_resets_the_bump():
    ctl = rc.RowfitController()
    drive(ctl, est(), 10)
    held = est()
    held.valid, held.held, held.counts_as_lost = False, True, False
    drive(ctl, held, 1)
    assert not ctl.straight_boost and ctl.target_speed <= rc.V_HELD_M_S + 1e-9
    ctl2 = rc.RowfitController()
    drive(ctl2, est(), 10)
    ctl2.step(LaneEstimate(valid=False), new_frame=True)
    assert not ctl2.straight_boost and ctl2.target_speed == rc.V_MIN_M_S


def test_wheels_stay_in_limits_at_straight_speed():
    ctl = rc.RowfitController()
    drive(ctl, est(), 10)
    assert ctl.gov.v == pytest.approx(rc.V_STRAIGHT_M_S, abs=1e-3)
    cmd = drive(ctl, est(offset=0.07, heading=math.radians(3.5)), 1)  # still qualifies, max correction
    for w in (cmd["left"], cmd["right"]):
        assert 0.0 < w <= MAX_WHEEL_RAD_S + 1e-6
    assert rc.V_STRAIGHT_M_S / WHEEL_RADIUS_M < MAX_WHEEL_RAD_S - 1.0  # steer headroom


def test_obstacle_slow_zone_sized_for_top_speed():
    g = ObstacleGate(decel_m_s2=rc.DECEL_M_S2, v_cruise_m_s=rc.V_MAX_M_S, latency_s=0.32)
    assert g.slow_distance >= g.stop_distance(rc.V_STRAIGHT_M_S) + 1.0 - 1e-9
    for p in ("webots/controllers/butlerbot_controller/controller_obstacles.py", "scripts/rowfit_sim.py"):
        src = (ROOT / p).read_text(encoding="utf-8")
        assert "v_cruise_m_s=V_MAX_M_S" in src, p
