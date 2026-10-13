"""BoM file, catalog loader, and the profile built from one build-out."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from src.bom_profile import (
    BOM_JSON_PATH,
    electronics_draw_w,
    load_bom,
    parts_by_id,
    subsystem_part_watts,
)
from src.hardware_profile import (
    battery_capacity_wh,
    build_profile_from_bom,
    clear_profile_cache,
    format_bom_log_line,
    wheel_radius_m,
)
from src.parts_db import connect, get_part, list_parts, seed_from_bom_json
from src.twin.webots_power import build_webots_telemetry

_NUMBER_KEYS_SKIP = set()


def _walk_numbers(obj, path=""):
    if isinstance(obj, dict):
        for key, value in obj.items():
            yield from _walk_numbers(value, f"{path}.{key}")
    elif isinstance(obj, list):
        for i, value in enumerate(obj):
            yield from _walk_numbers(value, f"{path}[{i}]")
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        yield path, obj


def test_bom_json_schema():
    data = json.loads(BOM_JSON_PATH.read_text(encoding="utf-8"))
    parts = data["parts"]
    assert len(parts) == 32
    assert set(data["builds"]) == {"light", "heavy", "budget"}
    ids = set()
    for part in parts:
        assert part.get("source_url"), part.get("id")
        assert part.get("price_date"), part.get("id")
        assert part["id"] not in ids
        ids.add(part["id"])
    for build_id, build in data["builds"].items():
        for line in build["lines"]:
            assert line["part_id"] in ids, f"{build_id} line {line}"
            assert line["qty"] > 0
    for path, number in _walk_numbers(data):
        assert number >= 0, path


def test_seed_from_bom_json_upserts_motor_and_battery(tmp_path: Path):
    db = tmp_path / "parts.db"
    conn = connect(db)
    n = seed_from_bom_json(conn)
    assert n == 32
    assert seed_from_bom_json(conn) == 32
    assert len(list_parts(conn)) >= 32
    motor = next(r for r in list_parts(conn) if r["sku"] == "4693")
    full = get_part(conn, motor["id"])
    assert full["category"] == "motor"
    assert full["motor"]["no_load_current_a"] == pytest.approx(0.1)
    assert full["motor"]["peak_torque_nm"] == pytest.approx(2.256)
    assert "price_date=2026-10-13" in (full["notes"] or "")
    assert full["source_url"].startswith("https://")
    pack = next(r for r in list_parts(conn) if r["sku"] == "BLF-2410A")
    batt = get_part(conn, pack["id"])
    assert batt["battery"]["capacity_wh"] == 240
    assert batt["battery"]["max_continuous_a"] == 10
    conn.close()


def test_build_profile_from_bom_light():
    prof = build_profile_from_bom("light")
    assert battery_capacity_wh(prof) == 240
    assert wheel_radius_m(prof) == 0.04
    assert prof["battery"]["nominal_voltage_v"] == 24
    assert prof["battery"]["continuous_discharge_a"] == 10
    assert prof["motors"]["left_wheel"]["stall_torque_nm"] == pytest.approx(2.256)
    assert prof["motors"]["left_wheel"]["no_load_current_a"] == pytest.approx(0.1)
    assert prof["geometry"]["mass_kg_known"] == pytest.approx(6.142)
    assert "drv_pololu_2516" in prof["unknown_parts"]


def test_bom_profile_drive_power_and_zero_upper_body():
    prof = build_profile_from_bom("light")
    idle = build_webots_telemetry(
        joints=[
            {"name": "left_wheel", "velocity": 0.0, "torque": 0.0},
            {"name": "right_wheel", "velocity": 0.0, "torque": 0.0},
            {"name": "left_arm", "velocity": 2.0, "torque": 1.0},
            {"name": "torso_joint", "velocity": 1.0, "torque": 1.0},
        ],
        gait="stand",
        phase="standby",
        speed_m_s=0.0,
        profile=prof,
    )
    drive = build_webots_telemetry(
        joints=[
            {"name": "left_wheel", "velocity": 11.0, "torque": 0.0},
            {"name": "right_wheel", "velocity": 11.0, "torque": 0.0},
            {"name": "left_arm", "velocity": 2.0, "torque": 1.0},
        ],
        gait="drive",
        phase="teleop",
        speed_m_s=0.44,
        profile=prof,
    )
    assert idle["channel_draws"]["Arms"] == 0.0
    assert idle["channel_draws"]["Torso"] == 0.0
    assert idle["channel_draws"]["Cooling"] == 0.0
    assert drive["channel_draws"]["Arms"] == 0.0
    assert drive["channel_draws"]["Torso"] == 0.0
    assert drive["channel_draws"]["Cooling"] == 0.0
    # Published light-build drive power at 0.44 m/s is 3.43 W.
    assert drive["channel_draws"]["Legs"] == pytest.approx(3.43, abs=0.15)
    assert drive["hardware"]["bom_build"] == "light"
    assert drive["locomotion"]["wheel_radius_m"] == 0.04
    # Compute is the Jetson idle plus the other published sensor watts, not a flat guess.
    # Moving does not switch the Jetson to its 25 W mode.
    assert idle["channel_draws"]["Compute"] == pytest.approx(9.86, abs=0.05)
    assert drive["channel_draws"]["Compute"] == pytest.approx(9.86, abs=0.05)
    assert electronics_draw_w(prof, moving=True) == pytest.approx(9.86, abs=0.05)
    parts = subsystem_part_watts(prof)
    assert parts["compute"] + parts["sensors"] == pytest.approx(9.86, abs=0.05)
    for payload in (idle, drive):
        channels = payload["channel_draws"]
        subs = payload["subsystem_draws"]
        assert subs["sensors"] is not None
        assert sum(subs.values()) == pytest.approx(sum(channels.values()), abs=0.05)
    # Stopped light robot sits on the published idle (~9.96 W). Cruise adds the
    # 3.43 W drive figure, not a second copy of the encoder idle.
    assert sum(idle["channel_draws"].values()) == pytest.approx(9.96, abs=0.2)
    assert sum(drive["channel_draws"].values()) == pytest.approx(13.39, abs=0.3)


def test_bom_log_line_names_the_light_build(monkeypatch):
    from src.config import config

    old = config._config.get("bom_build", None)
    config._config["bom_build"] = "light"
    clear_profile_cache()
    try:
        line = format_bom_log_line()
    finally:
        config._config["bom_build"] = old
        clear_profile_cache()
    assert line == (
        "BOM build=light schema=1.0 mass_kg=6.142 pack_wh=240 "
        "pack_cont_a=10 idle_w=9.96 unknown_parts=drv_pololu_2516,rad_dfrobot_c4001"
    )


def test_default_profile_when_bom_build_blank():
    from src.config import config
    from src.hardware_profile import active_bom_build, get_active_profile

    assert config.get("bom_build") in (None, "")
    assert active_bom_build() is None
    prof = get_active_profile()
    assert prof["profile_id"] == "butlerbot_wheeled"
    assert battery_capacity_wh(prof) == 480


def test_every_build_line_part_is_in_the_index():
    data = load_bom()
    index = parts_by_id(data)
    for build in data["builds"].values():
        for line in build["lines"]:
            assert line["part_id"] in index
