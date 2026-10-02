"""Waypoint tracks (src/track_geometry.py + scripts/track_builder.py)."""
from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import s_track  # noqa: E402
import track_builder  # noqa: E402
from src.track_geometry import (  # noqa: E402
    TrackGeometry,
    load_track,
    referee_for,
    spec_from_dict,
    track_file_from_world,
)

WORLDS = ROOT / "webots" / "worlds"
NEW_WORLDS = {"corner90": "butlerbot_corner90.wbt", "corner_mix": "butlerbot_corner_mix.wbt",
              "widen": "butlerbot_widen.wbt"}


def _geom(wps, **kw):
    return TrackGeometry(spec_from_dict({"waypoints": wps, **kw}))


def _area(poly):
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                     for i in range(len(poly)))


def _top_faces(pts, faces):
    top = max(p[2] for p in pts)
    return [[pts[i] for i in f] for f in faces if all(abs(pts[i][2] - top) < 1e-12 for i in f)]


def test_sharp_corner_lines_are_true_mitres():
    g = _geom([[0, 0], [4, 0], [4, 4]], paint_before_start_m=0.0, paint_after_finish_m=0.0)
    assert g.lane_line(+1) == [(0.0, 0.65), (3.35, 0.65), (3.35, 4.0)]  # inner mitre
    assert g.lane_line(-1) == [(0.0, -0.65), (4.65, -0.65), (4.65, 4.0)]  # outer mitre
    assert g.length_m == pytest.approx(8.0)


@pytest.mark.parametrize("side, length", [(+1, 3.35 + 3.35), (-1, 4.65 + 4.65)])
def test_mitred_stripe_has_no_overlap(side, length):
    """Top area of a mitred L-stripe = stripe width x centre length exactly, every
    top face winds counter-clockwise (+Z) and no two top faces overlap."""
    g = _geom([[0, 0], [4, 0], [4, 4]], paint_before_start_m=0.0, paint_after_finish_m=0.0)
    pts, faces = [], []
    track_builder.strip_prism(*g.stripe_edges(side), 0.013, 0.01, pts, faces)
    tops = _top_faces(pts, faces)
    areas = [_area([p[:2] for p in t]) for t in tops]
    assert all(a > 0 for a in areas)
    assert sum(areas) == pytest.approx(0.06 * length, rel=1e-9)


def test_rounded_corner_tighter_than_half_width_gives_inner_mitre():
    """Centre radius 0.5 m < half width 0.65 m: inner line folds to a sharp mitre,
    outer line is an arc of radius 1.15 m about the centre-arc centre."""
    g = _geom([[0, 0], {"x": 4, "y": 0, "radius_m": 0.5}, [4, 4]], paint_before_start_m=0, paint_after_finish_m=0)
    inner = g.lane_line(+1)
    assert (3.35, 0.65) in [(round(x, 9), round(y, 9)) for x, y in inner]
    outer = g.lane_line(-1)
    arc = [p for p in outer if p[0] > 3.5 + 1e-9 and p[1] < 0.5 - 1e-9]
    assert len(arc) > 10
    for x, y in arc:
        assert math.hypot(x - 3.5, y - 0.5) == pytest.approx(1.15, abs=1e-9)
    assert g.length_m == pytest.approx(8 - 2 * 0.5 + 0.5 * math.pi / 2, abs=1e-4)  # sampled arc
    pts, faces = [], []
    track_builder.strip_prism(*g.stripe_edges(+1), 0.013, 0.01, pts, faces)
    assert all(_area([p[:2] for p in t]) > 0 for t in _top_faces(pts, faces))


def test_leg_too_short_for_the_corners_is_rejected():
    with pytest.raises(ValueError, match="too short"):
        _geom([[0, 0], [4, 0], [4, 0.5], [0, 0.5]])  # U-turn narrower than the lane


def test_variable_width_is_linear_between_waypoints():
    g = load_track("widen")
    assert g.width_at_s(2.0) == pytest.approx(1.30)
    assert g.width_at_s(4.0) == pytest.approx(1.45)
    assert g.width_at_s(6.0) == pytest.approx(1.60)
    assert g.width_at_s(8.0) == pytest.approx(1.45)
    left = g.lane_line(+1, extend=False)
    assert (5.0, 0.8) in [(round(x, 9), round(y, 9)) for x, y in left]


def test_project_gives_progress_and_signed_lateral():
    g = load_track("corner90")
    p = g.project(2.0, 0.3)
    assert p["s_m"] == pytest.approx(2.0) and p["lateral_m"] == pytest.approx(0.3)
    p = g.project(4.2, 2.0)  # on the north leg, right of centre
    assert p["s_m"] == pytest.approx(6.0) and p["lateral_m"] == pytest.approx(-0.2)
    assert g.project(-1.0, 0.1)["lateral_m"] == pytest.approx(0.1)  # run-in: still sideways


def test_finish_referee_corner90():
    g = load_track("corner90")
    assert g.crossed_finish(4.1, 4.01)
    assert not g.crossed_finish(4.1, 3.9)
    assert not g.crossed_finish(4.0 + 1.5, 4.2)  # beside the line, not across it
    # A hook whose first leg runs just past its own finish line: the referee must
    # not fire there (progress check), only at the real finish.
    hook = _geom([[0, 0], [4, 0], [4, 2.5], [2, 2.5], [2, 1]])
    assert hook.past_finish_m(2.0, 0.0)[0] > 0  # first leg IS past the finish line
    assert not hook.crossed_finish(2.0, 0.0)
    assert hook.crossed_finish(2.0, 0.95)


def test_referee_from_world_tag_and_env(tmp_path):
    w = WORLDS / "butlerbot_corner90.wbt"
    assert track_file_from_world(str(w)) == "tracks/corner90.json"
    ref = referee_for(str(w), env={})
    assert ref.name == "corner90" and ref.crossed(4.0, 4.05) and not ref.crossed(16.6, 0.0)
    ref = referee_for(None, env={"RBM_TRACK": "widen"})
    assert ref.name == "widen" and ref.crossed(11.01, 0.0)
    ref = referee_for(str(tmp_path / "missing.wbt"), env={})
    assert ref.track is None and ref.crossed(16.5, 0.0) and not ref.crossed(16.4, 0.0)
    s = referee_for(str(WORLDS / "butlerbot.wbt"), env={})
    assert s.name == "s" and s.crossed(16.5, 0.02) and not s.crossed(16.45, 0.0)


def test_s_track_file_is_generator_output():
    assert (ROOT / "tracks" / "s.json").read_text(encoding="utf-8") == s_track.track_json()
    d = json.loads((ROOT / "tracks" / "s.json").read_text(encoding="utf-8"))
    assert d["waypoints"][0] == [0, 0] and d["waypoints"][-1] == [16.5, 0]


def test_s_waypoint_build_matches_exact_offset_curve_within_1mm():
    """The S built from waypoints vs the exact normal-offset curve (old generator)."""
    g = s_track.geometry()

    def seg(p, a, b):
        vx, vy = b[0] - a[0], b[1] - a[1]
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
        return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)

    for side in (1.0, -1.0):
        ref = s_track.lane_line_samples(side)
        le, re_ = g.stripe_edges(side)
        for edge, k in ((le, 1), (re_, 2)):
            exact = [q[k] for q in ref]
            worst = 0.0
            for i, p in enumerate(edge[::3]):
                j = min(range(max(0, 3 * i - 30), min(len(exact) - 1, 3 * i + 30)),
                        key=lambda j: math.dist(p, exact[j]))
                worst = max(worst, min(seg(p, exact[max(0, j - 1)], exact[j]), seg(p, exact[j], exact[min(len(exact) - 1, j + 1)])))
            assert worst < 1e-3


@pytest.mark.parametrize("name", sorted(NEW_WORLDS))
def test_test_worlds_are_generated_and_keep_the_robot(name):
    text = (WORLDS / NEW_WORLDS[name]).read_text(encoding="utf-8")
    base = (WORLDS / "butlerbot.wbt").read_text(encoding="utf-8")
    assert track_builder.emit_track_vrml(load_track(name)).strip() in text
    assert text.split("\nRobot {", 1)[1] == base.split("\nRobot {", 1)[1]  # robot, cameras, HUDs identical
    assert text.count("{") == text.count("}") and text.count("[") == text.count("]")
    assert re.findall(r"^DEF TRACK Pose \{", text, re.M) == ["DEF TRACK Pose {"]
    assert not re.search(r"^(Transform|Pose) \{", text, re.M)
    assert "solid" not in text.split("DEF TRACK")[1].split("Robot {")[0]
    assert f"# TRACK_FILE tracks/{name}.json" in text
    assert f'"TRACK (ENU): {name} ' in text


@pytest.mark.parametrize("name", ["s", "corner90", "corner_mix", "widen"])
def test_every_top_face_points_up(name):
    g = s_track.geometry() if name == "s" else load_track(name)
    for part, (_, pts, faces, _) in track_builder.track_meshes(g).items():
        for t in _top_faces(pts, faces):
            assert _area([p[:2] for p in t]) > 0, f"{name} {part} top face winds downward"


def test_drift_report_scores_any_track(tmp_path, capsys):
    import importlib.util

    spec = importlib.util.spec_from_file_location("drift_report", ROOT / "scripts" / "drift_report.py")
    dr = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dr)
    g = load_track("corner90")
    rows = ["unix_s,x_m,y_m,steer"]
    t = 0.0
    for p in g.dense_centerline(0.2):
        rows.append(f"{t},{p[0] + (0.05 if p[1] > 0.5 else 0.0)},{p[1]},0")  # 5 cm right... on the north leg
        t += 0.3
    rows.append(f"{t},4.0,4.05,0")
    log = tmp_path / "log.csv"
    log.write_text("\n".join(rows) + "\n")
    assert dr.main([str(log), "--track", "corner90", "--out", str(tmp_path / "o.csv")]) == 0
    out = capsys.readouterr().out
    assert "corner90 track centre line" in out
    import csv

    row = next(csv.DictReader((tmp_path / "o.csv").open()))
    assert row["finished"] == "True"
    assert float(row["max_drift_m"]) == pytest.approx(0.05, abs=1e-3)
    assert float(row["distance_m"]) == pytest.approx(8.0, abs=0.06)
    assert dr.main([str(log), "--track", "no_such_track"]) == 1
