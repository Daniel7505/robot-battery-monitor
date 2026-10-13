"""Segment, breakdown, and end-of-mission lines from measured samples only."""

from __future__ import annotations

import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from src.hardware_profile import clear_profile_cache
from src.mission_energy_log import MissionEnergyLog
from src.twin.webots_power import reported_wheel_torque_nm
from src.twin_energy import TEN_FT_M

_T0 = datetime(2026, 10, 13, 12, 0, tzinfo=timezone.utc)
_SUBS = {
    "compute": 7.20,
    "sensors": 2.66,
    "cooling": 0.0,
    "locomotion": 3.43,
    "arms": 0.0,
    "torso": 0.0,
}


def test_shell_scripts_checkout_as_lf():
    text = Path(".gitattributes").read_text(encoding="utf-8")
    assert "*.sh text eol=lf" in text


def test_unmeasured_wheel_torque_is_not_invented_for_a_bom_build():
    assert reported_wheel_torque_nm(5.5, None, bom=True) == 0.0
    assert reported_wheel_torque_nm(5.5, 1e-6, bom=True) == 0.0
    assert reported_wheel_torque_nm(5.5, None, bom=False) == pytest.approx(5.5 * 0.45)
    assert reported_wheel_torque_nm(5.5, 1.0, bom=True) == 1.0
    assert reported_wheel_torque_nm(5.5, -1.0, bom=False) == 1.0


def _sample(
    t_s: float,
    x_m: float,
    *,
    speed: float = 0.44,
    heading: float = 0.0,
    battery: float = 100.0,
    event: str | None = None,
    subs: dict | None = None,
    channels: dict | None = None,
    include_subs: bool = True,
) -> dict:
    sample = {
        "timestamp": _T0 + timedelta(seconds=t_s),
        "pose": {"x_m": x_m, "y_m": 0.0, "heading_rad": heading},
        "speed_m_s": speed,
        "battery_pct": battery,
        "capacity_wh": 240.0,
        "payload_kg": 0.0,
        "build": "light",
        "channel_draws": channels
        or {"Compute": 9.86, "Legs": 3.43, "Arms": 0.0, "Torso": 0.0, "Cooling": 0.0},
    }
    if include_subs:
        sample["subsystem_draws"] = _SUBS if subs is None else subs
    if event:
        sample["mission_event"] = event
    return sample


def _fields(line: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in line.split() if "=" in part)


def test_segment_closes_every_ten_feet_from_measured_draws():
    log = MissionEnergyLog()
    assert log.observe(_sample(0.0, 0.0)) == []
    dt = TEN_FT_M / 0.44
    lines = log.observe(_sample(dt, TEN_FT_M, heading=math.radians(90.0)))
    segments = [line for line in lines if line.startswith("SEGMENT_ENERGY")]
    assert len(segments) == 1
    fields = _fields(segments[0])
    assert fields["build"] == "light"
    assert fields["seg_idx"] == "0"
    assert float(fields["dist_m"]) == pytest.approx(TEN_FT_M, abs=1e-6)
    assert float(fields["dt_s"]) == pytest.approx(dt, abs=0.02)
    assert float(fields["avg_speed"]) == pytest.approx(0.44, abs=0.01)
    assert float(fields["heading_change_deg"]) == pytest.approx(90.0, abs=0.2)
    assert fields["accel_events"] == "0"
    assert fields["payload_kg"] == "0"
    wh_drive = 3.43 * dt / 3600.0
    wh_idle = (7.20 + 2.66) * dt / 3600.0
    assert float(fields["wh_drive"]) == pytest.approx(wh_drive, abs=1e-4)
    assert float(fields["wh_idle"]) == pytest.approx(wh_idle, abs=1e-4)
    assert float(fields["wh_drive"]) + float(fields["wh_idle"]) == pytest.approx(
        float(fields["wh_total"]), abs=1e-4
    )
    # A short step stays open. No second segment line.
    more = log.observe(_sample(dt + 1.0, TEN_FT_M + 1.0))
    assert not any(line.startswith("SEGMENT_ENERGY") for line in more)


def test_one_long_step_splits_into_whole_segments_and_counts_accel_once():
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, speed=0.0))
    # 1 m/s gained in 1 s is an accel event, and the step covers two segments.
    lines = log.observe(_sample(1.0, 2.0 * TEN_FT_M, speed=1.0))
    segments = [line for line in lines if line.startswith("SEGMENT_ENERGY")]
    assert len(segments) == 2
    first, second = (_fields(line) for line in segments)
    assert first["seg_idx"] == "0"
    assert second["seg_idx"] == "1"
    assert first["accel_events"] == "1"
    assert second["accel_events"] == "0"
    assert float(first["dist_m"]) == pytest.approx(TEN_FT_M, abs=1e-6)
    assert float(second["dist_m"]) == pytest.approx(TEN_FT_M, abs=1e-6)


def test_power_breakdown_every_ten_seconds_sums_to_system_draw():
    log = MissionEnergyLog()
    lines: list[str] = []
    log.observe(_sample(0.0, 0.0))
    for i in range(1, 11):
        lines.extend(log.observe(_sample(float(i), 0.0)))
    breakdowns = [line for line in lines if line.startswith("POWER_BREAKDOWN")]
    assert len(breakdowns) == 1
    fields = _fields(breakdowns[0])
    assert fields["build"] == "light"
    assert float(fields["t_s"]) == pytest.approx(10.0, abs=0.1)
    names = ("compute", "sensors", "cooling", "locomotion", "arms", "torso")
    total = sum(float(fields[name]) for name in names)
    assert total == pytest.approx(float(fields["total"]), abs=0.02)
    assert float(fields["total"]) == pytest.approx(7.20 + 2.66 + 3.43, abs=0.02)
    # The next window has not closed yet.
    assert not any(
        line.startswith("POWER_BREAKDOWN") for line in log.observe(_sample(15.0, 0.0))
    )


def test_unmetered_sensors_are_logged_as_na():
    log = MissionEnergyLog()
    log.observe(
        _sample(
            0.0,
            0.0,
            include_subs=False,
            channels={"Compute": 9.86, "Legs": 3.43},
        )
    )
    lines = log.observe(
        _sample(
            10.0,
            0.0,
            include_subs=False,
            channels={"Compute": 9.86, "Legs": 3.43},
        )
    )
    breakdown = next(line for line in lines if line.startswith("POWER_BREAKDOWN"))
    fields = _fields(breakdown)
    assert fields["sensors"] == "na"
    assert fields["cooling"] == "na"
    assert fields["arms"] == "na"
    assert fields["torso"] == "na"
    assert float(fields["compute"]) == pytest.approx(9.86, abs=0.01)
    assert float(fields["locomotion"]) == pytest.approx(3.43, abs=0.01)
    assert float(fields["total"]) == pytest.approx(9.86 + 3.43, abs=0.02)
    assert "sensors=0" not in breakdown


def _use_bom(build):
    from src.config import config

    old = config._config.get("bom_build", None)
    config._config["bom_build"] = build
    clear_profile_cache()
    return old


def _restore_bom(old):
    from src.config import config

    if old is None:
        config._config.pop("bom_build", None)
    else:
        config._config["bom_build"] = old
    clear_profile_cache()


def test_mission_summary_fires_once_when_the_track_parks():
    old = _use_bom("light")
    try:
        log = MissionEnergyLog()
        log.observe(_sample(0.0, 0.0, heading=0.0, battery=100.0))
        # Two 90° turns, 10 m, 20 s. No end event yet.
        mid = log.observe(_sample(10.0, 5.0, heading=math.radians(90.0), battery=99.5))
        assert not any(line.startswith("MISSION_SUMMARY") for line in mid)
        done = log.observe(
            _sample(20.0, 10.0, heading=math.radians(180.0), battery=99.0, event="complete")
        )
        summary = next(line for line in done if line.startswith("MISSION_SUMMARY"))
        energy = next(line for line in done if line.startswith("MISSION_ENERGY"))
        fields = _fields(summary)
        assert float(fields["duration_s"]) == pytest.approx(20.0, abs=0.1)
        assert float(fields["dist_m"]) == pytest.approx(10.0, abs=0.05)
        assert fields["turns"] == "2"
        assert float(fields["soc_start"]) == pytest.approx(100.0)
        assert float(fields["soc_end"]) == pytest.approx(99.0)
        assert float(fields["wh_model_idle"]) == pytest.approx(9.96 * 20.0 / 3600.0, abs=0.002)
        assert float(fields["wh_model"]) > float(fields["wh_model_drive"]) > 0
        measured = float(fields["wh_measured"])
        assert measured == pytest.approx((7.20 + 2.66 + 3.43) * 20.0 / 3600.0, abs=0.002)
        assert fields["err_pct"] != ""
        assert float(fields["wh_measured_soc"]) == pytest.approx(240.0 * 1.0 / 100.0, abs=0.01)
        assert "wh_measured=" in energy and "wh_measured= " not in energy
        assert "wh_model=" in energy
        # Idle over the whole run is in the model, so it is larger than drive-only.
        assert "build=light" in energy
        again = log.observe(
            _sample(21.0, 10.0, heading=math.radians(180.0), battery=99.0, event="complete")
        )
        assert again == []
    finally:
        _restore_bom(old)


def test_summary_is_not_emitted_without_the_park_complete_event():
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0))
    lines = log.observe(_sample(5.0, 2.0))
    assert not any(line.startswith("MISSION_SUMMARY") for line in lines)


def test_bridge_logs_a_breakdown_from_a_webots_payload(caplog):
    import logging

    from src.logger import logger
    from src.twin.bridge import DigitalTwinBridge

    bridge = DigitalTwinBridge()
    subs = dict(_SUBS)
    base = {
        "source": "webots",
        "adapter": "webots",
        "robot": {
            "name": "ButlerBot",
            "main_battery_pct": 100.0,
            "battery_capacity_wh": 240.0,
        },
        "mission": {"task": "moving"},
        "channel_draws": {
            "Compute": 9.86,
            "Legs": 3.43,
            "Arms": 0.0,
            "Torso": 0.0,
            "Cooling": 0.0,
        },
        "subsystem_draws": subs,
        "locomotion": {
            "gait": "drive",
            "speed_m_s": 0.0,
            "bom_build": "light",
            "payload_kg": 0.0,
        },
        "pose": {"x_m": 0.0, "y_m": 0.0, "heading_rad": 0.0},
        "sensors": {},
    }
    with caplog.at_level(logging.INFO, logger=logger.name):
        first = dict(base)
        first["timestamp"] = _T0.isoformat()
        assert bridge.ingest_telemetry(first, adapter="webots")["ok"] is True
        second = dict(base)
        second["timestamp"] = (_T0 + timedelta(seconds=10)).isoformat()
        second["robot"] = dict(base["robot"])
        second["robot"]["main_battery_pct"] = 99.9
        assert bridge.ingest_telemetry(second, adapter="webots")["ok"] is True
    breakdowns = [rec.message for rec in caplog.records if rec.message.startswith("POWER_BREAKDOWN")]
    assert len(breakdowns) == 1
    fields = _fields(breakdowns[0])
    assert fields["sensors"] != "na"
    assert float(fields["total"]) == pytest.approx(
        sum(float(fields[name]) for name in ("compute", "sensors", "cooling", "locomotion", "arms", "torso")),
        abs=0.02,
    )
