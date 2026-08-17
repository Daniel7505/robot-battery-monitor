"""Parts catalog is SQLite and must not touch Postgres telemetry."""

from pathlib import Path

import pytest

from src.parts_db import (
    add_part,
    connect,
    get_part,
    list_motors,
    list_parts,
)


def test_init_and_seed_creates_examples(tmp_path: Path):
    db = tmp_path / "parts.db"
    conn = connect(db)
    rows = list_parts(conn)
    assert len(rows) >= 8
    cats = {r["category"] for r in rows}
    assert {"motor", "battery", "camera"}.issubset(cats)
    conn.close()


def test_seed_does_not_duplicate(tmp_path: Path):
    db = tmp_path / "parts.db"
    connect(db).close()
    conn = connect(db)
    n1 = len(list_parts(conn))
    conn.close()
    conn = connect(db)
    n2 = len(list_parts(conn))
    assert n1 == n2
    conn.close()


def test_list_motors_under_cost_and_mass(tmp_path: Path):
    db = tmp_path / "parts.db"
    conn = connect(db)
    cheap = list_motors(conn, max_cost_usd=50, max_mass_g=250)
    assert cheap
    assert all(r["cost_usd"] <= 50 for r in cheap)
    assert all(r["mass_g"] <= 250 for r in cheap)
    assert all(r["category"] == "motor" for r in cheap)
    conn.close()


def test_get_part_joins_spec(tmp_path: Path):
    db = tmp_path / "parts.db"
    conn = connect(db)
    motors = list_parts(conn, category="motor")
    full = get_part(conn, motors[0]["id"])
    assert full is not None
    assert full["motor"]["voltage_v"] == 12
    pack = next(r for r in list_parts(conn, category="battery") if r["sku"] == "AEGIS-48V-10AH")
    batt = get_part(conn, pack["id"])
    assert batt["battery"]["capacity_wh"] == 480
    conn.close()


def test_insert_then_query(tmp_path: Path):
    db = tmp_path / "parts.db"
    conn = connect(db)
    pid = add_part(
        conn,
        sku="TEST-WIDGET",
        name="Test widget",
        category="other",
        mass_g=10,
        cost_usd=1.0,
    )
    got = get_part(conn, pid)
    assert got["sku"] == "TEST-WIDGET"
    conn.close()


def test_unknown_category_rejected(tmp_path: Path):
    db = tmp_path / "parts.db"
    conn = connect(db)
    with pytest.raises(ValueError):
        add_part(conn, name="nope", category="thruster")
    conn.close()
