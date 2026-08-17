"""Design agent first cut — catalog in, ranked BOMs out. No Webots."""

from pathlib import Path

from src.design_agent import Mission, format_report, propose
from src.parts_db import connect


def test_propose_returns_two_to_four(tmp_path: Path):
    db = tmp_path / "parts.db"
    connect(db).close()
    result = propose(
        Mission(track_length_m=25, max_cost_usd=800, max_mass_g=6000, max_minutes=5),
        db_path=db,
        max_candidates=4,
    )
    assert 2 <= len(result["candidates"]) <= 4
    assert result["considered"] >= 2
    top = result["candidates"][0]
    assert "bom" in top and len(top["bom"]) >= 4
    assert "rationale" in top
    assert "totals" in top
    assert top["totals"]["cost_usd"] > 0
    assert top["totals"]["mass_g"] > 0


def test_feasible_twin_class_ranks_first_on_s(tmp_path: Path):
    db = tmp_path / "parts.db"
    connect(db).close()
    result = propose(
        Mission(name="mirrored S", track_length_m=25, max_cost_usd=900, max_minutes=5),
        db_path=db,
    )
    feasible = [c for c in result["candidates"] if c["feasible"]]
    assert feasible
    assert any("POLOLU-4753" in c["name"] for c in feasible)
    assert all(c["totals"]["energy_ok"] for c in feasible)


def test_tight_budget_marks_heavy_pack_infeasible(tmp_path: Path):
    db = tmp_path / "parts.db"
    connect(db).close()
    result = propose(
        Mission(track_length_m=25, max_cost_usd=200, max_mass_g=1500, max_minutes=5),
        db_path=db,
        max_candidates=4,
    )
    names = " ".join(c["name"] for c in result["candidates"])
    if "AEGIS-48V-10AH" in names:
        heavy = next(c for c in result["candidates"] if "AEGIS-48V-10AH" in c["name"])
        assert heavy["feasible"] is False


def test_prefer_low_compute_favors_csi(tmp_path: Path):
    db = tmp_path / "parts.db"
    connect(db).close()
    cheap = propose(
        Mission(track_length_m=25, max_cost_usd=800, prefer_low_compute=True),
        db_path=db,
        max_candidates=4,
    )
    top_cams = [
        row["sku"]
        for c in cheap["candidates"]
        if c["feasible"]
        for row in c["bom"]
        if row["role"] == "camera"
    ]
    assert top_cams
    assert top_cams[0] == "RPI-CAM-V2"


def test_format_report_mentions_mission(tmp_path: Path):
    db = tmp_path / "parts.db"
    connect(db).close()
    text = format_report(propose(Mission(name="mirrored S"), db_path=db))
    assert "mirrored S" in text
    assert "BOM:" in text
