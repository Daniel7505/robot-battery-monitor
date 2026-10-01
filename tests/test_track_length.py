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


def _world_finish_x() -> float:
    m = re.search(
        r"# FINISH line \(red\)[^\n]*\nTransform \{\n\s*translation\s+(-?[\d.]+)\s+(-?[\d.]+)",
        _world_text(),
    )
    assert m, "red FINISH Transform not found in butlerbot.wbt"
    return float(m.group(1))


def test_world_red_finish_line_is_16_5():
    assert _world_finish_x() == pytest.approx(16.5)


def test_world_paint_ends_at_finish():
    xs = [
        float(m.group(1))
        for m in re.finditer(r"translation\s+(-?[\d.]+)\s+-?[\d.]+\s+0\.008\b", _world_text())
    ]
    assert xs, "no lane paint found"
    # Last lane-edge box (0.38 m long) is centered within one paint step of the line.
    assert 16.5 - 0.38 <= max(xs) <= 16.5
    assert min(xs) == pytest.approx(0.0)


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
