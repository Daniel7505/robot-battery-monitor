"""Optimizer agent reads lap FLAGS from report.md / summary.json."""

from __future__ import annotations

import json
from pathlib import Path

from optimizer.lap_agent import (
    compose_keep_flag_explanation,
    flag_citation,
    keep_blocked_without_flag_explanation,
    read_lap_artifacts,
)
from optimizer.postmortem import build_postmortem


def test_read_lap_artifacts_from_summary_flags(tmp_path):
    lap = tmp_path / "20261021-120000_search-1"
    lap.mkdir()
    (lap / "summary.json").write_text(
        json.dumps(
            {
                "lines": ["MISSION_SUMMARY build=light err_pct=11.0 turns=13"],
                "flags": ["energy model error +11.0% (|err| > 3%)"],
                "speed_limiter": "cruise / straight config (effective_cruise≈0.44 m/s)",
                "top_speed_m_s": 0.43,
            }
        ),
        encoding="utf-8",
    )
    (lap / "report.md").write_text(
        "# Lap report\n\n## FLAGS\n\n- energy model error +11.0% (|err| > 3%)\n",
        encoding="utf-8",
    )
    artifacts = read_lap_artifacts(lap)
    assert artifacts["flags"]
    assert "energy" in artifacts["flags"][0]
    assert "cruise" in artifacts["speed_limiter"]


def test_keep_blocked_without_flag_explanation():
    artifacts = {"flags": ["turn 1: heading change +126.0° outside 80–100°"]}
    assert keep_blocked_without_flag_explanation(artifacts, "") is not None
    expl = compose_keep_flag_explanation(
        artifacts,
        score_reason="combined score 0.95 beat threshold",
        param="controller.cruise_m_s",
        new=0.5,
        effects={"err_gain_pp": -2.0},
    )
    assert keep_blocked_without_flag_explanation(artifacts, expl) is None
    assert "Lap FLAGS" in expl
    assert "126" in expl


def test_postmortem_cites_lap_flags():
    artifacts = {
        "flags": ["energy model error +11.0% (|err| > 3%)"],
        "report_path": "/logs/laps/x/report.md",
        "speed_limiter": "physics-limited effective cruise (0.44 m/s)",
        "top_speed_m_s": 0.65,
    }
    post = build_postmortem(
        param="controller.cruise_m_s",
        old=0.44,
        new=0.65,
        action="rollback",
        before=None,
        after=None,
        lap_artifacts=artifacts,
    )
    assert "Lap FLAGS" in post["mechanism"]
    assert "report.md" in post["lesson"]
    assert post["lap_flags"] == artifacts["flags"]


def test_flag_citation_includes_speed_limiter():
    cite = flag_citation(
        {
            "flags": ["energy model error +4.0% (|err| > 3%)"],
            "report_path": "report.md",
            "top_speed_m_s": 0.65,
            "speed_limiter": "wheel cap (~0.64 m/s from MAX_WHEEL_RAD_S × contact radius)",
        }
    )
    assert "wheel cap" in cite
    assert "0.650" in cite
