"""Radar moving-object sense: corridor filter, ego compensation, WAITING_FOR_MOVING gate."""
import math
from pathlib import Path

import pytest

from src.obstacle_gate import (
    STATE_CLEAR,
    STATE_SLOW,
    STATE_STOPPED,
    STATE_WAITING,
    ObstacleFusion,
    ObstacleGate,
    ObstacleReport,
)
from src.radar_sense import (
    MOVING_SPEED_M_S,
    RADAR_NAME,
    RadarObstacleSensor,
    RadarParams,
    RadarTarget,
    classify_motion,
    report_from_targets,
    target_in_corridor,
)
from src.rowfit_control import DECEL_M_S2, V_CRUISE_M_S

ROOT = Path(__file__).resolve().parents[1]
WORLDS = sorted((ROOT / "webots" / "worlds").glob("butlerbot*.wbt"))


def new_gate():
    return ObstacleGate(decel_m_s2=DECEL_M_S2, v_cruise_m_s=V_CRUISE_M_S, latency_s=0.32)


# ---------------------------------------------------------------- mount in worlds


@pytest.mark.parametrize("world", WORLDS, ids=lambda p: p.name)
def test_every_butlerbot_world_has_forward_radar(world):
    text = world.read_text(encoding="utf-8")
    assert f'name "{RADAR_NAME}"' in text
    assert "DEF RADAR_FWD Radar" in text
    # Aimed along +x: rotation 0 1 0 ~pi/2
    assert "rotation 0 1 0 1.5708" in text


def test_radar_motion_world_has_rolling_ball_and_obstacles_on():
    """Ball must cross the S *start straight* (y=0 for x<=3), not sit on the lobe."""
    import re
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    from s_track import START_STRAIGHT_M, centerline

    p = ROOT / "webots" / "worlds" / "butlerbot_radar_motion.wbt"
    assert p.is_file()
    text = p.read_text(encoding="utf-8")
    assert "# OBSTACLES on" in text
    assert "DEF WH_MOVING_BALL Solid" in text
    assert "radarCrossSection" in text
    m = re.search(
        r"DEF WH_MOVING_BALL Solid \{\s*translation ([\d.+-]+) ([\d.+-]+) ([\d.+-]+).*?"
        r"linearVelocity ([\d.+-]+) ([\d.+-]+) ([\d.+-]+)",
        text,
        re.S,
    )
    assert m, "ball translation/velocity missing"
    x, y, z = map(float, m.group(1, 2, 3))
    vx, vy, vz = map(float, m.group(4, 5, 6))
    assert 0.5 < x < START_STRAIGHT_M, f"ball x={x} must be on the start straight (<{START_STRAIGHT_M})"
    assert centerline(x)[0] == 0.0
    # Starts outside the corridor half-width and rolls toward lane centre (vy toward 0)
    assert abs(y) > 0.7, f"ball should start clearly outside the lane (y={y})"
    assert vy * y < 0, f"velocity must aim toward y=0 (y={y}, vy={vy})"
    assert abs(vy) >= 0.5, f"need a decisive cross-lane speed, got vy={vy}"


def test_warehouse_obstacle_props_have_radar_cross_section():
    text = (ROOT / "webots" / "worlds" / "butlerbot_warehouse_obstacles.wbt").read_text(encoding="utf-8")
    for name in ("obs_box", "obs_table", "obs_shelf"):
        assert f'name "{name}"' in text
        idx = text.index(f'name "{name}"')
        assert "radarCrossSection" in text[idx:idx + 80]


# ---------------------------------------------------------------- pure geometry / motion


def test_corridor_rejects_targets_outside_half_width():
    prm = RadarParams(half_width_m=0.5)
    in_lane = RadarTarget(2.0, 0.0, 0.0)
    out = RadarTarget(2.0, 0.0, math.asin(0.7 / 2.0))  # |y|=0.7 > 0.5
    assert target_in_corridor(in_lane, prm)
    assert not target_in_corridor(out, prm)


def test_static_wall_while_driving_is_not_moving():
    # Robot approaches a static wall at 0.44 m/s: Webots relative speed ≈ -0.44
    t = RadarTarget(distance_m=2.0, speed_m_s=-0.44, azimuth_rad=0.0)
    closing, radial, moving = classify_motion(t, v_ego_m_s=0.44)
    assert closing == pytest.approx(0.44, abs=1e-6)
    assert radial == pytest.approx(0.0, abs=1e-6)
    assert not moving


def test_ball_rolling_toward_robot_is_moving():
    # Ball closes at 0.55 m/s in world while robot is stopped → Webots speed -0.55
    t = RadarTarget(distance_m=3.0, speed_m_s=-0.55, azimuth_rad=0.0)
    closing, radial, moving = classify_motion(t, v_ego_m_s=0.0)
    assert closing == pytest.approx(0.55)
    assert radial == pytest.approx(0.55)
    assert moving and radial >= MOVING_SPEED_M_S


def test_report_marks_moving_and_sets_velocity():
    t = RadarTarget(2.5, -0.4, 0.05)
    rep = report_from_targets([t], v_ego_m_s=0.0)
    assert rep.detected and rep.source == "radar"
    assert rep.velocity_m_s == pytest.approx(0.4)
    assert rep.notes["moving"] is True
    assert rep.clearance_m is not None and rep.clearance_m > 0


def test_clear_corridor_report():
    rep = report_from_targets([], v_ego_m_s=0.0)
    assert not rep.detected and rep.range_m >= 8.0


def test_sensor_latest_consumes_once():
    s = RadarObstacleSensor()
    s.process([RadarTarget(1.5, 0.0, 0.0)], v_ego_m_s=0.0)
    assert s.latest() is not None and s.latest() is None


# ---------------------------------------------------------------- gate behaviour


def _radar_moving(clear_m=1.2, radial=0.4):
    # distance from axle ≈ clear + front (0.15)
    return ObstacleReport(
        source="radar",
        detected=True,
        distance_m=clear_m + 0.15,
        clearance_m=clear_m,
        y_left_m=0.2,
        y_right_m=-0.2,
        z_bottom_m=0.0,
        z_top_m=0.4,
        band="low",
        confidence=0.9,
        n_points=1,
        range_m=8.0,
        velocity_m_s=radial,
        notes={"moving": True, "target_radial_m_s": radial},
    )


def _radar_still(clear_m=1.2):
    r = _radar_moving(clear_m=clear_m, radial=0.0)
    r.velocity_m_s = 0.0
    r.notes = {"moving": False, "target_radial_m_s": 0.0}
    return r


def test_gate_waits_for_moving_then_stops_when_still():
    g = new_gate()
    g.on_frame(_radar_moving(1.0), v_m_s=0.44)
    g.on_frame(_radar_moving(0.95), v_m_s=0.40)
    assert g.state == STATE_WAITING and g.v_cap == 0.0
    assert "WAIT" in g.hud_line() and "mov" in g.hud_line()
    # Object stops still inside the stop zone (standoff 0.35 + 0.02)
    g.on_frame(_radar_still(0.36), v_m_s=0.0)
    g.on_frame(_radar_still(0.36), v_m_s=0.0)  # still frames clear the moving flag
    assert g.state == STATE_STOPPED and g.v_cap == 0.0


def test_gate_moving_far_away_still_waits_not_cruises():
    g = new_gate()
    for c in (2.5, 2.4):
        g.on_frame(_radar_moving(c), v_m_s=0.44)
    assert g.state == STATE_WAITING  # even beyond the slow zone
    assert g.fields()["obstacle_moving"] is True


def test_fusion_prefers_moving_radar_over_farther_stereo():
    f = ObstacleFusion()
    stereo = ObstacleReport(
        source="stereo", detected=True, distance_m=1.6, clearance_m=1.45,
        y_left_m=0.2, y_right_m=-0.2, z_bottom_m=0.0, z_top_m=0.5,
        band="low", confidence=1.0, n_points=40, range_m=4.0,
    )
    f.add_report(stereo)
    f.add_report(_radar_moving(1.5))
    assert f.fused().source == "radar" and f.fused().notes["moving"]


def test_static_radar_still_feeds_slow_stop_like_stereo():
    g = new_gate()
    for c in (1.4, 1.3):
        g.on_frame(_radar_still(c), v_m_s=0.44)
    assert g.state == STATE_SLOW
    for c in (0.5, 0.45, 0.40):
        g.on_frame(_radar_still(c), v_m_s=0.2)
    assert g.state == STATE_STOPPED
