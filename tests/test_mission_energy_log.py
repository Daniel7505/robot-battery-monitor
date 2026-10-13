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
    # Hinge torque feedback is motor effort, not a load cell. A BoM build
    # does not pass it into the rolling line.
    assert reported_wheel_torque_nm(5.5, 1.0, bom=True) == 0.0
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
    measured_drive_w: float | None = None,
    report_measured: bool = False,
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
    if report_measured:
        sample["measured_drive_w"] = measured_drive_w
    return sample


def _fields(line: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in line.split() if "=" in part)


def test_segment_closes_every_ten_feet_from_measured_draws():
    log = MissionEnergyLog()
    # 12 W is a twin shaft-power reading. It is not the 3.43 W Legs bill.
    assert log.observe(_sample(0.0, 0.0, report_measured=True, measured_drive_w=12.0)) == []
    dt = TEN_FT_M / 0.44
    lines = log.observe(
        _sample(dt, TEN_FT_M, heading=math.radians(90.0), report_measured=True, measured_drive_w=12.0)
    )
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
    wh_measured = 12.0 * dt / 3600.0
    wh_idle = (7.20 + 2.66) * dt / 3600.0
    assert float(fields["wh_measured_drive"]) == pytest.approx(wh_measured, abs=1e-4)
    assert float(fields["wh_measured_drive"]) != pytest.approx(3.43 * dt / 3600.0, abs=1e-4)
    assert "wh_model_drive" in fields
    assert "wh_model_roll" in fields
    assert "err_pct" in fields
    assert float(fields["wh_idle"]) == pytest.approx(wh_idle, abs=1e-4)
    assert float(fields["wh_measured_drive"]) + float(fields["wh_idle"]) == pytest.approx(
        float(fields["wh_total"]), abs=1e-4
    )
    # A short step stays open. No second segment line.
    more = log.observe(_sample(dt + 1.0, TEN_FT_M + 1.0))
    assert not any(line.startswith("SEGMENT_ENERGY") for line in more)


def test_pose_jump_is_not_split_into_segments():
    # 6 m in 1 s is ~6 m/s. The live log turned that into several SEGMENT
    # lines with the same timestamp and avg_speed=1.842.
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, speed=0.0))
    lines = log.observe(_sample(1.0, 2.0 * TEN_FT_M, speed=1.84))
    assert not any(line.startswith("SEGMENT_ENERGY") for line in lines)
    # The jump is not travel, so the next honest step still starts at seg 0.
    dt = TEN_FT_M / 0.44
    later = log.observe(_sample(1.0 + dt, 2.0 * TEN_FT_M + TEN_FT_M, speed=0.44))
    segments = [line for line in later if line.startswith("SEGMENT_ENERGY")]
    assert len(segments) == 1
    assert _fields(segments[0])["seg_idx"] == "0"
    assert float(_fields(segments[0])["avg_speed"]) == pytest.approx(0.44, abs=0.02)


def test_one_sample_closes_at_most_one_segment():
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, speed=0.44))
    dt = (2.0 * TEN_FT_M) / 0.44
    lines = log.observe(_sample(dt, 2.0 * TEN_FT_M, speed=0.44))
    segments = [line for line in lines if line.startswith("SEGMENT_ENERGY")]
    assert len(segments) == 1
    assert _fields(segments[0])["seg_idx"] == "0"
    more = log.observe(_sample(dt + 0.5, 2.0 * TEN_FT_M + 0.05, speed=0.44))
    more_segs = [line for line in more if line.startswith("SEGMENT_ENERGY")]
    assert len(more_segs) == 1
    assert _fields(more_segs[0])["seg_idx"] == "1"


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
        log.observe(
            _sample(0.0, 0.0, heading=0.0, battery=100.0, report_measured=True, measured_drive_w=3.43)
        )
        # Two 90° turns, 10 m, 20 s. No end event yet.
        mid = log.observe(
            _sample(
                10.0, 5.0, heading=math.radians(90.0), battery=99.5,
                report_measured=True, measured_drive_w=3.43,
            )
        )
        assert not any(line.startswith("MISSION_SUMMARY") for line in mid)
        done = log.observe(
            _sample(
                20.0, 10.0, heading=math.radians(180.0), battery=99.0, event="complete",
                report_measured=True, measured_drive_w=3.43,
            )
        )
        summary = next(line for line in done if line.startswith("MISSION_SUMMARY"))
        energy = next(line for line in done if line.startswith("MISSION_ENERGY"))
        fields = _fields(summary)
        assert float(fields["duration_s"]) == pytest.approx(20.0, abs=0.1)
        assert float(fields["dist_m"]) == pytest.approx(10.0, abs=0.05)
        # 0° → 90° → 180° is 180° of yaw. One turn is 30° of that sum.
        assert fields["turns"] == "6"
        assert float(fields["soc_start"]) == pytest.approx(100.0)
        assert float(fields["soc_end"]) == pytest.approx(99.0)
        assert float(fields["wh_model_idle"]) == pytest.approx(9.96 * 20.0 / 3600.0, abs=0.002)
        assert float(fields["wh_model"]) > float(fields["wh_model_drive"]) > 0
        assert float(fields["wh_model_turn"]) == 0.0
        assert float(fields["wh_model_visc"]) > 0.0
        assert float(fields["wh_model_drive"]) == pytest.approx(
            float(fields["wh_model_roll"])
            + float(fields["wh_model_visc"])
            + float(fields["wh_model_accel"]),
            abs=0.003,
        )
        measured = float(fields["wh_measured"])
        assert measured == pytest.approx((7.20 + 2.66 + 3.43) * 20.0 / 3600.0, abs=0.002)
        assert float(fields["wh_measured_drive"]) == pytest.approx(3.43 * 20.0 / 3600.0, abs=0.002)
        assert fields["err_pct"] != "na"
        assert float(fields["wh_measured_drive"]) != pytest.approx(
            float(fields["wh_model_drive"]), abs=0.01
        )
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


def test_log_build_follows_the_config_when_the_sample_says_default():
    old = _use_bom("light")
    try:
        log = MissionEnergyLog()
        first = _sample(0.0, 0.0)
        first["build"] = "default"
        log.observe(first)
        lines: list[str] = []
        for i in range(1, 11):
            sample = _sample(float(i), 0.0)
            sample["build"] = "default"
            lines.extend(log.observe(sample))
        breakdown = next(line for line in lines if line.startswith("POWER_BREAKDOWN"))
        assert "build=light" in breakdown
    finally:
        _restore_bom(old)


def test_unstamped_parked_sample_uses_the_light_build_watts():
    from src.twin.adapters.webots import WebotsAdapter

    old = _use_bom("light")
    try:
        tel = WebotsAdapter().normalize({
            "source": "webots",
            "robot": {"name": "ButlerBot", "main_battery_pct": 100},
            "channel_draws": {
                "Compute": 9.0,
                "Legs": 2.73,
                "Arms": 2.8,
                "Torso": 2.3,
                "Cooling": 0.0,
            },
            "joints": [
                {"name": "left_wheel", "velocity": 0.0, "torque": 2.0},
                {"name": "right_wheel", "velocity": 0.0, "torque": 2.0},
                {"name": "left_arm", "velocity": 1.0, "torque": 1.0},
                {"name": "torso_joint", "velocity": 1.0, "torque": 1.0},
            ],
            "locomotion": {"gait": "stand", "speed_m_s": 0.0, "phase": "standby"},
            "mission": {"task": "idle"},
        })
        assert tel.locomotion.get("bom_build") == "light"
        assert tel.channel_draws["Arms"] == 0.0
        assert tel.channel_draws["Torso"] == 0.0
        assert tel.channel_draws["Compute"] == pytest.approx(9.86, abs=0.05)
        assert tel.raw["subsystem_draws"]["sensors"] is not None
        assert sum(tel.channel_draws.values()) == pytest.approx(9.96, abs=0.25)
    finally:
        _restore_bom(old)


def test_reported_speed_above_the_control_cap_is_clipped():
    from src.twin.adapters.webots import WebotsAdapter

    tel = WebotsAdapter().normalize({
        "source": "webots",
        "robot": {"name": "ButlerBot", "main_battery_pct": 90},
        "channel_draws": {"Compute": 9.0, "Legs": 3.0, "Arms": 0.0, "Torso": 0.0, "Cooling": 0.0},
        "locomotion": {"gait": "drive", "speed_m_s": 1.84, "phase": "teleop"},
        "mission": {"task": "moving"},
    })
    assert tel.locomotion["speed_m_s"] == pytest.approx(0.85)


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


def test_mission_complete_does_not_wait_forever_on_gps_noise():
    from src.mission_park import mission_should_complete

    common = dict(track_done=True, already_complete=False, abs_done=False, no_drive_command=True)
    assert mission_should_complete(
        **common, hubs_quiet=False, finish_age_s=0.2
    ) is False
    assert mission_should_complete(
        **common, hubs_quiet=True, finish_age_s=1.0
    ) is True
    # Hub twitch after the finish must not block the summary. The hold
    # is only after track_done, and only once the drive command is zero.
    assert mission_should_complete(
        **common, hubs_quiet=False, finish_age_s=1.0
    ) is False
    assert mission_should_complete(
        **common, hubs_quiet=False, finish_age_s=3.0
    ) is True
    assert mission_should_complete(
        **common, hubs_quiet=False, finish_age_s=12.0
    ) is True
    assert mission_should_complete(
        track_done=True, already_complete=False, abs_done=False,
        hubs_quiet=False, no_drive_command=False, finish_age_s=90.0,
    ) is False
    assert mission_should_complete(
        track_done=False, already_complete=False, abs_done=False,
        hubs_quiet=True, no_drive_command=True, finish_age_s=90.0,
    ) is False
    assert mission_should_complete(
        track_done=True, already_complete=False, abs_done=True,
        hubs_quiet=False, no_drive_command=False, finish_age_s=0.1,
    ) is True
    assert mission_should_complete(
        track_done=False, already_complete=False, abs_done=True,
        hubs_quiet=True, no_drive_command=True, finish_age_s=20,
    ) is False


def test_distance_cap_is_off_unless_asked_and_never_finishes_the_s_early():
    from src.mission_park import (
        mission_distance_reached,
        mission_should_complete,
        resolve_mission_limit_m,
        warehouse_perimeter_m,
    )

    assert mission_distance_reached(500.0, None) is False
    assert mission_distance_reached(500.0, 0) is False
    assert mission_distance_reached(500.0, "") is False
    assert mission_distance_reached(10.0, 50) is False
    assert mission_distance_reached(50.0, 50) is True
    assert resolve_mission_limit_m() is None
    assert resolve_mission_limit_m(config_m="", env_m=None, env_laps=None) is None
    perim = warehouse_perimeter_m()
    assert perim == pytest.approx(201.0, abs=0.1)
    assert resolve_mission_limit_m(env_laps="2", lap_m=perim) == pytest.approx(402.0, abs=0.2)
    # An explicit metre cap wins over laps. Env metres win over YAML.
    assert resolve_mission_limit_m(config_m=100, env_laps="2", lap_m=perim) == 100
    assert resolve_mission_limit_m(
        config_m=100, env_m="40", env_laps="2", lap_m=perim
    ) == 40
    # Env 0 turns the cap off. It does not fall through to laps.
    assert resolve_mission_limit_m(
        config_m=100, env_m="0", env_laps="2", lap_m=perim
    ) is None
    # No cap, and the track is not finished: a long pause still does not complete.
    assert mission_should_complete(
        track_done=False,
        already_complete=False,
        abs_done=False,
        hubs_quiet=True,
        no_drive_command=True,
        finish_age_s=90.0,
    ) is False


def test_live_twin_ignores_ros2_mission_override():
    from datetime import datetime, timezone

    from src.hardware import reset_hardware_source
    from src.hardware_ros2 import ROS2BatterySource
    from src.twin.bridge import reset_twin_bridge

    reset_twin_bridge()
    reset_hardware_source()
    source = ROS2BatterySource()
    source._ros2.inject_command(mission="high_load")
    source._apply_ros2_commands()
    assert source._mission.task_id == "high_load"

    now = datetime.now(timezone.utc).isoformat()
    assert source._twin.ingest_telemetry({
        "source": "webots",
        "adapter": "webots",
        "timestamp": now,
        "robot": {"name": "ButlerBot", "main_battery_pct": 100, "battery_capacity_wh": 240},
        "mission": {"task": "moving"},
        "channel_draws": {"Compute": 9.86, "Legs": 3.43, "Arms": 0, "Torso": 0, "Cooling": 0},
        "locomotion": {"gait": "drive", "speed_m_s": 0.44, "phase": "teleop"},
        "pose": {"x_m": 1.0, "y_m": 0.0, "heading_rad": 0.0},
    }, adapter="webots")["ok"] is True
    assert source._twin._is_external_active()
    source._ros2.inject_command(mission="idle")
    source._apply_ros2_commands()
    assert source._mission.task_id == "high_load"


def test_turn_counter_counts_yaw_that_straightens_out():
    # Peaks stay under 30° from the start, so the old net-anchor counter
    # stayed at 0. The segment sums are the live 61.6° then 32.3°.
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, heading=0.0, speed=0.2))
    headings = (20.0, 0.0, 21.6, 0.0, 10.7)
    # 20+20+21.6 = 61.6, then 21.6+10.7 = 32.3. Total 93.9° → 3 turns.
    lines: list[str] = []
    x = 0.0
    for i, deg in enumerate(headings, start=1):
        x += 0.2
        lines.extend(log.observe(_sample(float(i), x, heading=math.radians(deg), speed=0.2)))
    lines.extend(
        log.observe(_sample(6.0, x + 0.2, heading=math.radians(10.7), speed=0.2, event="complete"))
    )
    summary = next(line for line in lines if line.startswith("MISSION_SUMMARY"))
    assert _fields(summary)["turns"] == "3"


def test_pose_jump_does_not_count_as_a_turn():
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, heading=0.0, speed=0.0))
    log.observe(_sample(1.0, 10.0, heading=math.radians(90.0), speed=2.0))
    done = log.observe(
        _sample(2.0, 10.2, heading=math.radians(90.0), speed=0.2, event="complete")
    )
    summary = next(line for line in done if line.startswith("MISSION_SUMMARY"))
    assert _fields(summary)["turns"] == "0"


def test_segment_logs_roll_turn_and_accel_from_the_model():
    old = _use_bom("light")
    try:
        log = MissionEnergyLog()
        log.observe(_sample(0.0, 0.0, heading=0.0, speed=0.44))
        dt = TEN_FT_M / 0.44
        straight = log.observe(_sample(dt, TEN_FT_M, heading=0.0, speed=0.44))
        seg = _fields(next(line for line in straight if line.startswith("SEGMENT_ENERGY")))
        assert float(seg["wh_model_turn"]) == pytest.approx(0.0, abs=1e-6)
        assert float(seg["wh_model_accel"]) == pytest.approx(0.0, abs=1e-6)
        assert float(seg["wh_model_roll"]) == pytest.approx(0.0066, abs=0.0003)
        assert float(seg["wh_model_visc"]) > float(seg["wh_model_roll"])
        assert float(seg["wh_model_drive"]) == pytest.approx(
            float(seg["wh_model_roll"]) + float(seg["wh_model_visc"]) + float(seg["wh_model_accel"]),
            abs=1e-4,
        )
        # No torque feedback on the sample: measured stays na, not wh_model_roll.
        assert seg["wh_measured_drive"] == "na"
        assert seg["err_pct"] == "na"

        curve = MissionEnergyLog()
        curve.observe(_sample(0.0, 0.0, heading=0.0, speed=0.0))
        curve.observe(_sample(1.0, 0.22, heading=0.0, speed=0.44))
        remain = TEN_FT_M - 0.22
        closed = curve.observe(
            _sample(
                1.0 + remain / 0.44,
                TEN_FT_M,
                heading=math.radians(61.6),
                speed=0.44,
            )
        )
        fields = _fields(next(line for line in closed if line.startswith("SEGMENT_ENERGY")))
        assert float(fields["heading_change_deg"]) == pytest.approx(61.6, abs=0.2)
        assert float(fields["wh_model_turn"]) == 0.0
        assert float(fields["wh_model_accel"]) > 0
        assert float(fields["wh_model_roll"]) == pytest.approx(0.0066, abs=0.0004)
        assert float(fields["wh_model_visc"]) > float(fields["wh_model_roll"])
        assert fields["wh_measured_drive"] == "na"
    finally:
        _restore_bom(old)


def test_a_killed_process_does_not_log_an_empty_summary():
    """0.6 s and no travel is the Webots process dying, not a mission."""
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, speed=0.0))
    killed = log.observe(_sample(0.6, 0.02, speed=0.0, event="complete"))
    assert not any(line.startswith("MISSION_SUMMARY") for line in killed)
    # The empty complete does not consume the one summary. Steps stay
    # under the 15 s stall guard so the distance is actually billed.
    log.observe(_sample(10.0, 8.0, speed=0.4))
    done = log.observe(_sample(12.0, 10.0, speed=0.4, event="complete"))
    assert any(line.startswith("MISSION_SUMMARY") for line in done)
    again = log.observe(_sample(13.0, 10.0, speed=0.0, event="complete"))
    assert not any(line.startswith("MISSION_SUMMARY") for line in again)


def test_hinge_torque_does_not_inflate_bom_locomotion():
    """Sim getTorqueFeedback must not become the drive bill.

    dampingConstant 0.15 on the 0.08 m wheel, plus a few tenths of a
    newton-metre of contact, is the order of torque that made live
    locomotion ~16× rolling. The published Legs watts stay on rolling.
    """
    import math

    from src.hardware_profile import get_active_profile
    from src.twin.webots_power import build_webots_telemetry
    from src.twin_energy import bom_drive_terms, sim_hinge_torque_nm

    old = _use_bom("light")
    try:
        prof = get_active_profile()
        v = 0.12
        hub = v / 0.08
        # Published damping only. The +0.25 N·m is contact, not a bill.
        assert sim_hinge_torque_nm(v) == pytest.approx(0.15 * hub, abs=1e-9)
        tau = 0.15 * hub + 0.25
        spec = prof["motors"]["left_wheel"]
        omega = v / 0.04
        omega0 = float(spec["no_load_rpm"]) * 2.0 * math.pi / 60.0
        tau_s = float(spec["stall_torque_nm"])
        duty = omega / omega0 + tau / tau_s
        current = float(spec["no_load_current_a"]) + (
            float(spec["stall_current_a"]) - float(spec["no_load_current_a"])
        ) * (tau / tau_s)
        old_line_w = 2.0 * duty * float(spec["voltage_v"]) * current / 0.92
        roll = bom_drive_terms(prof, v)
        assert old_line_w > 10.0 * roll["drive_w"]

        joints = [
            {"name": "left_wheel", "velocity": hub, "torque": tau},
            {"name": "right_wheel", "velocity": hub, "torque": tau},
        ]
        billed = build_webots_telemetry(
            joints=joints, gait="drive", phase="teleop", speed_m_s=v, profile=prof
        )
        assert billed["channel_draws"]["Legs"] == pytest.approx(roll["drive_w"], abs=0.15)
    finally:
        _restore_bom(old)


def test_measured_drive_is_hinge_power_not_the_bom_line():
    """Shaft power |τ|·|ω| on the 0.08 m wheel, kept off the Legs bill.

    A missing ``torque_feedback_nm`` is na. The model ``torque`` field
    is not a substitute, and body speed is not divided by the 0.04 m
    BoM radius.
    """
    from src.hardware_profile import get_active_profile
    from src.twin.webots_power import (
        build_webots_telemetry,
        measured_drive_w,
        restamp_active_bom,
    )
    from src.twin_energy import bom_drive_terms

    v = 0.16
    omega = v / 0.08
    tau = 0.4
    quiet = [
        {"name": "left_wheel", "velocity": 0.0, "torque": 0.0, "torque_feedback_nm": tau},
        {"name": "right_wheel", "velocity": 0.0, "torque": 0.0, "torque_feedback_nm": tau},
    ]
    assert measured_drive_w(quiet, v) == pytest.approx(2 * tau * omega, abs=1e-9)
    assert measured_drive_w(quiet, v) != pytest.approx(2 * tau * (v / 0.04), abs=1e-3)

    sensed = [
        {"name": "left_wheel", "velocity": omega, "torque": 0.0, "torque_feedback_nm": tau},
        {"name": "right_wheel", "velocity": omega, "torque": 0.0, "torque_feedback_nm": -tau},
    ]
    assert measured_drive_w(sensed, v) == pytest.approx(2 * abs(tau) * omega, abs=1e-9)
    invented = [
        {"name": "left_wheel", "velocity": omega, "torque": 0.5},
        {"name": "right_wheel", "velocity": omega, "torque": 0.5},
    ]
    assert measured_drive_w(invented, v) is None
    half = [
        sensed[0],
        {"name": "right_wheel", "velocity": omega, "torque": 0.0, "torque_feedback_nm": None},
    ]
    assert measured_drive_w(half, v) is None

    old = _use_bom("light")
    try:
        prof = get_active_profile()
        roll = bom_drive_terms(prof, v)
        payload = build_webots_telemetry(
            joints=sensed, gait="drive", phase="teleop", speed_m_s=v, profile=prof
        )
        stamped = restamp_active_bom(payload)
        assert stamped["channel_draws"]["Legs"] == pytest.approx(roll["drive_w"], abs=0.15)
        assert stamped["locomotion"]["measured_drive_w"] == pytest.approx(2 * tau * omega, abs=1e-6)
        assert stamped["measured_drive_w"] == pytest.approx(2 * tau * omega, abs=1e-6)

        blank = build_webots_telemetry(
            joints=invented, gait="drive", phase="teleop", speed_m_s=v, profile=prof
        )
        assert restamp_active_bom(blank)["locomotion"]["measured_drive_w"] is None

        log = MissionEnergyLog()
        shaft_w = 2 * tau * omega
        # Two steps stay under the 15 s stall guard. 0.16 m/s × 20 s clears 10 ft.
        log.observe(_sample(0.0, 0.0, speed=v, report_measured=True, measured_drive_w=shaft_w))
        log.observe(_sample(10.0, 1.6, speed=v, report_measured=True, measured_drive_w=shaft_w))
        lines = log.observe(
            _sample(20.0, 3.2, speed=v, report_measured=True, measured_drive_w=shaft_w)
        )
        seg = _fields(next(line for line in lines if line.startswith("SEGMENT_ENERGY")))
        assert float(seg["wh_measured_drive"]) == pytest.approx(shaft_w * 20.0 / 3600.0 * (TEN_FT_M / 3.2), abs=1e-4)
        assert float(seg["wh_model_roll"]) == pytest.approx(roll["drive_wh_per_10ft"], abs=0.0004)
        assert float(seg["wh_measured_drive"]) != pytest.approx(float(seg["wh_model_roll"]), abs=0.001)
        assert float(seg["wh_model_drive"]) == pytest.approx(
            float(seg["wh_model_roll"]) + float(seg["wh_model_visc"]) + float(seg["wh_model_accel"]),
            abs=1e-4,
        )
        assert seg["err_pct"] != "na"

        missing = MissionEnergyLog()
        missing.observe(_sample(0.0, 0.0, speed=v, report_measured=True, measured_drive_w=None))
        missing.observe(_sample(10.0, 1.6, speed=v, report_measured=True, measured_drive_w=None))
        gone = missing.observe(
            _sample(20.0, 3.2, speed=v, report_measured=True, measured_drive_w=None)
        )
        na_seg = _fields(next(line for line in gone if line.startswith("SEGMENT_ENERGY")))
        assert na_seg["wh_measured_drive"] == "na"
        assert na_seg["err_pct"] == "na"
        assert float(na_seg["wh_model_roll"]) > 0
    finally:
        _restore_bom(old)
