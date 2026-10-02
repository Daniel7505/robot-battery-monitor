"""Track length single source of truth: the red finish line in butlerbot.wbt.

The S corridor was tightened on 2026-09-04 (1ff49a5): 5 m lobes, finish x=16.5 m.
Before that it was 9 m lobes / finish x=24.5 m. Every constant and doc must agree
with the world file so drift / finish checks do not silently use the old track.
"""

from __future__ import annotations

import csv
import importlib.util
import io
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "webots" / "worlds" / "butlerbot.wbt"
CAL = ROOT / "webots" / "worlds" / "track_calibration.json"
PATH_CSV = ROOT / "webots" / "worlds" / "s_track_path.csv"
CONTROLLER = ROOT / "webots" / "controllers" / "butlerbot_controller" / "butlerbot_controller.py"

sys.path.insert(0, str(ROOT / "scripts"))
import s_track  # noqa: E402


def _load_script(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _world_text() -> str:
    return WORLD.read_text(encoding="utf-8")


def _world_meshes() -> dict:
    """DEF TRACK_* Shape -> (baseColor, points, faces) parsed from the world file."""
    out = {}
    for m in re.finditer(
        r"DEF (TRACK_\w+) Shape \{(.*?)coordIndex \[(.*?)\]", _world_text(), re.S
    ):
        body = m.group(2)
        col = tuple(float(v) for v in re.search(r"baseColor (\S+) (\S+) (\S+)", body).groups())
        nums = [float(v) for v in re.search(r"point \[(.*?)\]", body, re.S).group(1).replace(",", " ").split()]
        pts = [tuple(nums[i : i + 3]) for i in range(0, len(nums), 3)]
        faces, cur = [], []
        for v in (int(t) for t in m.group(3).split()):
            if v < 0:
                faces.append(cur)
                cur = []
            else:
                cur.append(v)
        out[m.group(1)] = (col, pts, faces)
    return out


def _world_finish_x() -> float:
    meshes = _world_meshes()
    assert "TRACK_FINISH" in meshes, "red TRACK_FINISH mesh not found in butlerbot.wbt"
    col, pts, _ = meshes["TRACK_FINISH"]
    assert col == pytest.approx((0.9, 0.15, 0.12))
    xs = [p[0] for p in pts]
    return (min(xs) + max(xs)) / 2.0


def test_world_red_finish_line_is_16_5():
    assert _world_finish_x() == pytest.approx(16.5)


def test_world_paint_ends_at_finish():
    meshes = _world_meshes()
    for name in ("TRACK_LINE_L", "TRACK_LINE_R"):
        xs = [p[0] for p in meshes[name][1]]
        assert xs, "no lane paint found"
        # Paint starts half a 0.38 m piece behind the start and ends just past the red line.
        assert min(xs) == pytest.approx(-0.19, abs=1e-3)
        assert 16.5 <= max(xs) <= 16.5 + 0.19


def test_track_is_one_pose_no_loose_segments():
    """Scene tree: one DEF TRACK Pose with 5 Shapes, no top-level paint Transforms."""
    text = _world_text()
    assert text.count("{") == text.count("}")
    assert text.count("[") == text.count("]")
    assert re.findall(r"^DEF TRACK Pose \{", text, re.M) == ["DEF TRACK Pose {"]
    assert not re.search(r"^(Transform|Pose) \{", text, re.M)
    assert set(_world_meshes()) == {"TRACK_LINE_L", "TRACK_LINE_R", "TRACK_START", "TRACK_FINISH", "TRACK_TICKS"}


def _legacy_box_corners():
    """Corners of the 105 archived Transform+Box paint nodes (parsed, not regenerated)."""
    import math

    arc = (ROOT / "archives" / "butlerbot_track_boxes_2026-10-01.wbt").read_text(encoding="utf-8")
    pat = (
        r"Transform \{\n  translation (\S+) (\S+) (\S+)\n  rotation 0 0 1 (\S+)\n"
        r"(.*?)size (\S+) (\S+) (\S+)"
    )
    for m in re.finditer(pat, arc, re.S):
        x, y, z, yaw = map(float, m.group(1, 2, 3, 4))
        sx, sy, sz = map(float, m.group(6, 7, 8))
        color = tuple(float(v) for v in re.search(r"baseColor (\S+) (\S+) (\S+)", m.group(5)).groups())
        c, s = math.cos(yaw), math.sin(yaw)
        for dx in (-sx / 2, sx / 2):
            for dy in (-sy / 2, sy / 2):
                for dz in (-sz / 2, sz / 2):
                    yield color, (x + dx * c - dy * s, y + dx * s + dy * c, z + dz)


def test_mesh_reproduces_old_box_paint():
    """Bars and ticks: every archived box corner is a mesh vertex (<0.2 mm).

    The yellow lane lines are now continuous ribbons (Dan 2026-10-01), so they
    are checked separately in test_lane_lines_are_continuous_ribbons.
    """
    import math

    yellow = (0.95, 0.95, 0.2)
    meshes = _world_meshes()
    by_col = {}
    for col, pts, _ in meshes.values():
        by_col.setdefault(col, []).extend(pts)
    n = 0
    worst = 0.0
    for color, corner in _legacy_box_corners():
        if color == yellow:
            continue
        vs = by_col[color]
        worst = max(worst, min(math.dist(corner, v) for v in vs if abs(v[0] - corner[0]) < 0.01))
        n += 1
    assert n > 0
    assert worst < 2e-4


def test_lane_lines_are_continuous_ribbons():
    """Each lane line is one strip: top-face vertices march along x with no gaps,
    and every top vertex sits half a stripe width off the exact offset curve."""
    import math

    meshes = _world_meshes()
    for name, side in (("TRACK_LINE_L", 1.0), ("TRACK_LINE_R", -1.0)):
        _, pts, _ = meshes[name]
        top = max(p[2] for p in pts)
        tops = [p for p in pts if abs(p[2] - top) < 1e-9]
        xs = sorted({round(p[0], 4) for p in tops})
        assert max(b - a for a, b in zip(xs, xs[1:])) < 0.06, f"{name} has a gap"
        sm = s_track.lane_line_samples(side)
        for p in tops[:: max(1, len(tops) // 200)]:
            d = min(math.dist(p[:2], q[3]) for q in sm)
            assert abs(d - s_track.LINE_W_M / 2) < 0.004, f"{name} off curve by {d}"


def test_mesh_top_faces_point_up():
    meshes = _world_meshes()
    for name, (_, pts, faces) in meshes.items():
        top = max(p[2] for p in pts)
        n_top = 0
        for f in faces:
            P = [pts[i] for i in f]
            if all(abs(p[2] - top) < 1e-9 for p in P):
                u = [P[1][j] - P[0][j] for j in range(3)]
                v = [P[2][j] - P[0][j] for j in range(3)]
                assert u[0] * v[1] - u[1] * v[0] > 0, f"{name} top face winds downward"
                n_top += 1
        assert n_top >= 1
    assert "ccw TRUE" in _world_text()
    assert "solid" not in _world_text().split("DEF TRACK")[1].split("Robot")[0]  # not a field in R2025a IndexedFaceSet


def test_old_boxes_archived_out_of_world():
    arc = (ROOT / "archives" / "butlerbot_track_boxes_2026-10-01.wbt").read_text(encoding="utf-8")
    assert s_track.emit_vrml_boxes().strip() in arc
    assert s_track.emit_vrml_boxes().strip() not in _world_text()


def test_generator_matches_world_exactly():
    """s_track.emit_vrml() is what is painted in the world (no hand edits)."""
    assert s_track.emit_vrml().strip() in _world_text()
    assert s_track.FINISH_X_M == pytest.approx(_world_finish_x())


def test_world_info_string_matches_finish():
    info = re.search(r'"TRACK \(ENU\):[^"]*"', _world_text()).group(0)
    assert "finish (16.5,0)" in info
    assert "24.5" not in _world_text()


def test_code_constants_agree_with_world():
    finish = _world_finish_x()
    from src import lane_keep

    assert lane_keep.FINISH_X_M == pytest.approx(finish)
    assert _load_script("drift_report").FINISH_X_M == pytest.approx(finish)
    assert _load_script("measure_track_run").FINISH_X == pytest.approx(finish)
    m = re.search(r"^_FINISH_X_M\s*=\s*([\d.]+)", CONTROLLER.read_text(encoding="utf-8"), re.M)
    assert m and float(m.group(1)) == pytest.approx(finish)


def test_lane_keep_centerline_matches_generator():
    from src import lane_keep

    for i in range(0, 166):
        x = i * 0.1
        y_ref, th_ref = s_track.centerline(x)
        y, th = lane_keep.track_centerline(x)
        assert y == pytest.approx(y_ref, abs=1e-9)
        assert th == pytest.approx(th_ref, abs=1e-9)


def test_calibration_json_matches_world():
    cal = json.loads(CAL.read_text(encoding="utf-8"))
    assert cal["finish"]["x_m"] == pytest.approx(_world_finish_x())
    assert cal["lane"]["lobe_m"] == pytest.approx(s_track.LOBE_M)
    assert cal["lane"]["path_length_m"] == pytest.approx(s_track.path_length_m(), abs=0.01)
    assert cal["lane"]["min_radius_m"] == pytest.approx(s_track.min_radius_m(), abs=0.01)
    assert _load_script("measure_track_run").TRACK_LENGTH_M == pytest.approx(
        cal["lane"]["path_length_m"]
    )


def test_path_csv_is_current_generator_output():
    assert PATH_CSV.read_text(encoding="utf-8") == s_track.emit_path_csv()
    rows = list(csv.DictReader(io.StringIO(PATH_CSV.read_text(encoding="utf-8"))))
    assert rows[-1]["kind"] == "finish"
    assert float(rows[-1]["x_m"]) == pytest.approx(16.5)


def test_readme_and_dashboard_hint_say_16_5():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "x=16.5" in readme
    assert "finish ``x=24.5``" not in readme
    dash = (ROOT / "src" / "dashboard.py").read_text(encoding="utf-8")
    assert "GPS stop at 16.5 m" in dash


def test_design_agent_default_is_current_track_path():
    from src.design_agent import Mission

    assert Mission().track_length_m == pytest.approx(s_track.path_length_m(), abs=0.05)
