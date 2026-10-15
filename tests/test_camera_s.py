"""Camera lane keeping on a course the memorized S cannot stay inside.

tracks/s.json bends south first (amplitude 1 m, lobes 5 m). tracks/camera_s.json
bends north first (amplitude 1.5 m, lobes 6 m). At the old trough the two
centerlines are more than a lane apart, so a robot that drove the s.json
waypoints would leave the paint. The world tag ``# STEER_TRACK off`` also
withholds that file from the wheel command. GPS still scores the finish and
the cross-track peak.
"""

from __future__ import annotations

import importlib.util
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from src.lane_vision import LaneEstimate
from src.mission_energy_log import MissionEnergyLog
from src.rowfit_control import RowfitController
from src.track_geometry import distance_to_next_corner, load_track, steer_track_enabled

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "webots" / "worlds" / "butlerbot_camera_s.wbt"
S_WORLD = ROOT / "webots" / "worlds" / "butlerbot.wbt"
HALF_LANE_M = 0.65
# x just before the camera crest. The scored S is already at its south trough.
PROBE_X_M = 5.8


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _est(k: float) -> LaneEstimate:
    """On the centerline, aligned with it, curvature taken from the paint."""
    return LaneEstimate(
        valid=True,
        offset_m=0.0,
        heading_rad=0.0,
        curvature_1pm=k,
        lookahead_m=1.2,
        confidence=0.9,
        n_left=60,
        n_right=60,
        lane_width_m=1.3,
        a0=0.0,
        psi=0.0,
        near_curvature_1pm=k,
    )


def _kappa(centerline, x: float, dx: float = 0.05) -> float:
    y0, h0 = centerline(x - dx)
    y1, h1 = centerline(x + dx)
    ds = math.hypot(2.0 * dx, y1 - y0)
    dh = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
    return dh / ds


def test_camera_world_refuses_the_track_for_steering():
    lines = WORLD.read_text(encoding="utf-8").splitlines()
    assert "# STEER_TRACK off" in lines
    assert "# TRACK_FILE tracks/camera_s.json" in lines
    assert steer_track_enabled(str(WORLD), env={}) is False
    assert steer_track_enabled(str(S_WORLD), env={}) is True
    for raw in ("1", "true", "yes", "on"):
        assert steer_track_enabled(str(S_WORLD), env={"RBM_NO_TRACK": raw}) is False
    assert steer_track_enabled(None, env={}) is True
    ctrl = (ROOT / "webots" / "controllers" / "butlerbot_controller" / "butlerbot_controller.py").read_text(
        encoding="utf-8"
    )
    gate = ctrl[ctrl.index("if rowfit_rt is not None and _steer_track"): ctrl.index("rowfit_rt.ctl.set_track")]
    assert "_steer_track" in gate
    assert "ROWFIT steer track off" in ctrl


def test_memorized_s_leaves_the_camera_lane():
    """A robot on the s.json waypoints is outside the camera_s paint."""
    camera = load_track("camera_s")
    old = load_track("s")
    assert camera.spec.waypoints[-1][0] == pytest.approx(18.5, abs=1e-6)
    assert old.spec.waypoints[-1][0] == pytest.approx(16.5, abs=1e-6)
    assert all(abs(w - 1.3) < 1e-9 for w in old.spec.widths)
    assert max(camera.spec.widths) - min(camera.spec.widths) == pytest.approx(0.30, abs=0.01)
    assert min(camera.spec.widths) == pytest.approx(1.15, abs=0.01)
    assert max(camera.spec.widths) == pytest.approx(1.45, abs=0.01)
    worst = 0.0
    worst_xy = (0.0, 0.0)
    for x, y in old.spec.waypoints:
        if x > camera.spec.waypoints[-1][0]:
            continue
        dist = float(camera.project(x, y)["dist_m"])
        if dist > worst:
            worst = dist
            worst_xy = (x, y)
    assert worst > 2.0, f"worst map point {worst_xy} is only {worst:.3f} m off the paint"
    assert worst > HALF_LANE_M
    cam = _load("camera_s_track")
    y_cam, _ = cam.centerline(PROBE_X_M)
    y_old, _ = _load("s_track").centerline(PROBE_X_M)
    assert y_cam > 1.0 and y_old < -0.5
    assert abs(y_cam - y_old) > 2.0


def test_camera_s_has_no_corner_the_map_can_own():
    """Smooth polyline: distance_to_next_corner stays None, so the map cannot set remaining distance."""
    camera = load_track("camera_s")
    assert max(abs(phi) for phi in camera._phi) < math.radians(20.0)
    cam = _load("camera_s_track")
    for i in range(0, int(cam.FINISH_X_M * 10) + 1):
        x = i / 10.0
        y, hd = cam.centerline(x)
        assert distance_to_next_corner(camera, x, y, hd) is None


def test_wheels_follow_the_camera_curve_with_the_old_map_loaded():
    """GPS pose plus tracks/s.json must not change the curvature command.

    At the probe the scored S is bending the other way. Pure pursuit on the
    camera fit returns that fit's curvature. The same command comes back with
    the camera file loaded and with no track at all.
    """
    cam = _load("camera_s_track")
    old = _load("s_track")
    y, hd = cam.centerline(PROBE_X_M)
    k = cam.curvature_1pm(PROBE_X_M)
    k_map = _kappa(old.centerline, PROBE_X_M)
    assert k < -0.5
    assert k_map > 0.3
    assert abs(k - k_map) > 1.0

    def command(track):
        ctl = RowfitController()
        if track is not None:
            ctl.set_track(track)
        return ctl.step(_est(k), new_frame=True, dt=0.32, yaw=hd, pose=(PROBE_X_M, y))

    for track in (load_track("s"), load_track("camera_s"), None):
        cmd = command(track)
        assert cmd["steer_source"] == "camera"
        assert cmd["steer_lateral"] == "camera"
        assert cmd["phase"] == "rowfit"
        assert cmd["brake"] is False
        assert cmd["kappa_cmd"] == pytest.approx(k, abs=0.03)
        assert abs(cmd["kappa_cmd"] - k_map) > 0.5
    # The probe is off the scored road, so that file cannot own a pivot distance either.
    assert distance_to_next_corner(load_track("s"), PROBE_X_M, y, hd) is None


def test_live_width_tracks_the_paint_and_one_line_falls_back():
    """Both lines in view: the fit follows the sine, not the 1.30 m constant.

    A frame that then loses one line keeps the last measured width. 1.30 m
    is only the stand-in before both lines have been seen.
    """
    from src.lane_vision import (
        NADIR_CAMS,
        WIDTH_FIT_AHEAD_M,
        LaneTracker,
        lane_polyline_robot,
        render_lane_bgra,
    )
    from src.rowfit_control import RowfitController

    cam = _load("camera_s_track")
    geom = load_track("camera_s")
    left = geom.lane_line(1.0)
    right = geom.lane_line(-1.0)

    def frames(x, drop=None):
        y, hd = cam.centerline(x)
        lines = []
        if drop != "left":
            lines.append(lane_polyline_robot(left, (x, y), hd))
        if drop != "right":
            lines.append(lane_polyline_robot(right, (x, y), hd))
        imgs = {n: (render_lane_bgra(c, lines), 128, 128) for n, c in NADIR_CAMS.items()}
        return imgs, y, hd

    tr = LaneTracker()
    prev = None
    width_err = []
    center_err = []
    x = 1.0
    while x <= 17.5 + 1e-9:
        imgs, y, hd = frames(x)
        ds = dyaw = 0.0
        if prev is not None:
            ds = math.hypot(x - prev[0], y - prev[1])
            dyaw = math.atan2(math.sin(hd - prev[2]), math.cos(hd - prev[2]))
        est = tr.process(imgs, dx_m=ds, dyaw_rad=dyaw)
        assert est.valid and est.width_live
        assert est.n_left >= 4 and est.n_right >= 4
        pr = geom.project(x, y)
        true_near = geom.width_at_s(pr["s_m"] + WIDTH_FIT_AHEAD_M)
        width_err.append(est.lane_width_m - true_near)
        center_err.append(est.offset_m - pr["lateral_m"])
        prev = (x, y, hd)
        x += 0.5
    width_rms = math.sqrt(sum(v * v for v in width_err) / len(width_err))
    assert width_rms < 0.03
    assert max(abs(v) for v in width_err) < 0.05
    assert max(abs(v) for v in center_err) < 0.04
    # A fitter locked at 1.30 m would miss the paint by the full sine.
    assert max(abs(geom.width_at_s(s) - 1.30) for s in (0.0, geom.length_m / 4, geom.length_m / 2)) > 0.10

    wide = LaneTracker()
    seen = wide.process(frames(4.62)[0])
    assert seen.width_live and seen.lane_width_m == pytest.approx(1.45, abs=0.04)
    one = wide.process(frames(4.62, drop="left")[0], dx_m=0.2)
    assert one.valid and one.n_left == 0 and not one.width_live
    assert one.lane_width_m == pytest.approx(wide.lane_w, abs=1e-6)
    assert abs(one.lane_width_m - 1.30) > 0.05

    ctl = RowfitController()
    ctl.tracker = wide
    assert ctl._lane_width(seen) == pytest.approx(seen.lane_width_m, abs=1e-6)
    assert ctl._lane_width(one) == pytest.approx(wide.lane_w, abs=1e-6)


def test_lane_frame_fields_and_lap_summary_name_both_errors():
    import importlib.util

    from src.lane_vision import LaneEstimate

    spec = importlib.util.spec_from_file_location(
        "controller_rowfit_width",
        ROOT / "webots" / "controllers" / "butlerbot_controller" / "controller_rowfit.py",
    )
    glue = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(glue)

    geom = load_track("camera_s")
    est = LaneEstimate(valid=True, offset_m=0.04, lane_width_m=1.40, width_live=True, n_left=20, n_right=20)
    wp = min(geom.spec.waypoints, key=lambda p: abs(p[0] - 4.62))
    fields = glue.lane_frame_fields(est, geom, wp[0], wp[1], 0.01)
    assert fields["width_live"] == 1
    assert fields["width_err_m"] == pytest.approx(1.40 - fields["true_width_near_m"], abs=1e-6)
    assert fields["center_err_m"] == pytest.approx(0.03, abs=1e-6)
    assert abs(fields["true_width_m"] - 1.30) > 0.10
    held = LaneEstimate(valid=True, offset_m=0.0, lane_width_m=1.40, width_live=False, n_left=0, n_right=20)
    fallback = glue.lane_frame_fields(held, geom, wp[0], wp[1], 0.0)
    assert fallback["width_err_m"] is None
    assert fallback["lane_width_m"] == pytest.approx(1.40)

    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0) | {"steer_source": "camera", "width_err_m": 0.02, "center_err_m": -0.01})
    done = log.observe(
        _sample(5.0, 2.0, event="complete")
        | {"steer_source": "camera", "width_err_m": -0.04, "center_err_m": 0.03, "ct_max_m": 0.05}
    )
    summary = _fields(next(line for line in done if line.startswith("MISSION_SUMMARY")))
    assert float(summary["width_err_rms_m"]) == pytest.approx(math.sqrt((0.02 ** 2 + 0.04 ** 2) / 2), abs=0.002)
    assert float(summary["width_err_max_m"]) == pytest.approx(0.04, abs=1e-9)
    assert float(summary["width_err_n"]) == 2
    assert float(summary["center_err_rms_m"]) == pytest.approx(math.sqrt((0.01 ** 2 + 0.03 ** 2) / 2), abs=0.002)
    assert float(summary["center_err_max_m"]) == pytest.approx(0.03, abs=1e-9)
    assert float(summary["ct_max_m"]) == pytest.approx(0.05, abs=1e-9)


def test_lap_summary_names_the_camera_fraction_and_the_cross_track_peak():
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0) | {"steer_source": "camera", "cross_track_m": 0.04})
    log.observe(
        _sample(5.0, 2.0) | {"steer_source": "camera", "cross_track_m": -0.08, "ct_max_m": 0.11}
    )
    done = log.observe(
        _sample(10.0, 4.0, event="complete")
        | {"steer_source": "camera", "cross_track_m": 0.02, "ct_max_m": 0.11}
    )
    summary = next(line for line in done if line.startswith("MISSION_SUMMARY"))
    steer = next(line for line in done if line.startswith("STEER_SOURCE"))
    fields = _fields(summary)
    split = _fields(steer)
    assert float(fields["steer_frac_camera"]) == pytest.approx(1.0, abs=0.02)
    assert float(fields["steer_frac_track"]) == pytest.approx(0.0, abs=0.02)
    assert float(fields["ct_max_m"]) == pytest.approx(0.11, abs=1e-6)
    assert float(split["ct_max_m"]) == pytest.approx(0.11, abs=1e-6)
    assert float(split["frac_camera"]) == pytest.approx(1.0, abs=0.02)
    # A map follower on this course would print a peak above a lane half-width.
    assert float(fields["ct_max_m"]) < HALF_LANE_M


_T0 = datetime(2026, 10, 15, 12, 0, tzinfo=timezone.utc)


def _sample(t_s: float, x_m: float, *, event: str | None = None) -> dict:
    sample = {
        "timestamp": _T0 + timedelta(seconds=t_s),
        "pose": {"x_m": x_m, "y_m": 0.0, "heading_rad": 0.0},
        "speed_m_s": 0.40,
        "battery_pct": 100.0,
        "capacity_wh": 240.0,
        "payload_kg": 0.0,
        "build": "light",
        "subsystem_draws": {
            "compute": 7.20,
            "sensors": 2.66,
            "cooling": 0.0,
            "locomotion": 3.43,
            "arms": 0.0,
            "torso": 0.0,
        },
        "channel_draws": {"Compute": 9.86, "Legs": 3.43, "Arms": 0.0, "Torso": 0.0, "Cooling": 0.0},
    }
    if event:
        sample["mission_event"] = event
    return sample


def _fields(line: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in line.split() if "=" in part)
