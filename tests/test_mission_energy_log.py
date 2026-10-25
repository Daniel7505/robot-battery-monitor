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


def test_steer_source_fraction_on_the_lap_summary():
    """Camera, held, and track metres are a partition of the lap."""
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, speed=0.44))
    log.observe(_sample(10.0, 4.4, speed=0.44) | {"steer_source": "camera"})
    log.observe(_sample(20.0, 7.4, speed=0.30) | {"steer_source": "held"})
    done = log.observe(
        _sample(30.0, 9.4, speed=0.25, event="complete") | {"steer_source": "track"}
    )
    summary = next(line for line in done if line.startswith("MISSION_SUMMARY"))
    steer = next(line for line in done if line.startswith("STEER_SOURCE"))
    fields = _fields(summary)
    split = _fields(steer)
    assert float(fields["steer_frac_camera"]) == pytest.approx(4.4 / 9.4, abs=0.02)
    assert float(fields["steer_frac_held"]) == pytest.approx(3.0 / 9.4, abs=0.02)
    assert float(fields["steer_frac_track"]) == pytest.approx(2.0 / 9.4, abs=0.02)
    assert float(split["camera_m"]) == pytest.approx(4.4, abs=0.05)
    assert float(split["held_m"]) == pytest.approx(3.0, abs=0.05)
    assert float(split["track_m"]) == pytest.approx(2.0, abs=0.05)
    assert float(split["frac_camera"]) + float(split["frac_held"]) + float(split["frac_track"]) == pytest.approx(1.0, abs=0.02)


def test_proof_status_is_available_before_the_lap_ends():
    """The dashboard reads this while the robot is still moving."""
    log = MissionEnergyLog()
    log.observe(_sample(0.0, 0.0, speed=0.44) | {"cross_track_m": 0.02})
    log.observe(
        _sample(10.0, 4.4, speed=0.44)
        | {
            "steer_source": "camera",
            "cross_track_m": 0.041,
            "width_err_m": 0.012,
            "center_err_m": -0.008,
            "lane_frame_n": 1,
        }
    )
    proof = log.proof_status()
    assert proof["ok"] is True
    assert proof["steer_frac_camera"] == pytest.approx(1.0)
    assert proof["ct_max_m"] == pytest.approx(0.041)
    assert proof["cross_track_m"] == pytest.approx(0.041)
    assert proof["width_err_rms_m"] == pytest.approx(0.012)
    assert proof["center_err_rms_m"] == pytest.approx(0.008)
    assert proof["err_pct"] is None


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
    measured_drive_elec_w: float | None = None,
    report_measured: bool = False,
    report_elec: bool = False,
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
    if report_elec:
        sample["measured_drive_elec_w"] = measured_drive_elec_w
    return sample


def _fields(line: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in line.split() if "=" in part)


def test_segment_closes_every_ten_feet_from_measured_draws():
    log = MissionEnergyLog()
    # 12 W is shaft power. 20 W is the electrical conversion of that point.
    # Neither number is the 3.43 W Legs bill.
    assert log.observe(
        _sample(
            0.0, 0.0,
            report_measured=True, measured_drive_w=12.0,
            report_elec=True, measured_drive_elec_w=20.0,
        )
    ) == []
    dt = TEN_FT_M / 0.44
    lines = log.observe(
        _sample(
            dt, TEN_FT_M, heading=math.radians(90.0),
            report_measured=True, measured_drive_w=12.0,
            report_elec=True, measured_drive_elec_w=20.0,
        )
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
    wh_shaft = 12.0 * dt / 3600.0
    wh_elec = 20.0 * dt / 3600.0
    wh_idle = (7.20 + 2.66) * dt / 3600.0
    assert float(fields["wh_measured_drive_shaft"]) == pytest.approx(wh_shaft, abs=1e-4)
    assert float(fields["wh_measured_drive_elec"]) == pytest.approx(wh_elec, abs=1e-4)
    assert float(fields["eta_avg"]) == pytest.approx(12.0 / 20.0, abs=1e-3)
    assert float(fields["wh_measured_drive_shaft"]) != pytest.approx(3.43 * dt / 3600.0, abs=1e-4)
    assert "wh_model_drive" in fields
    assert "wh_model_roll" in fields
    assert "err_pct" in fields
    assert float(fields["wh_idle"]) == pytest.approx(wh_idle, abs=1e-4)
    assert float(fields["wh_measured_drive_elec"]) + float(fields["wh_idle"]) == pytest.approx(
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
            _sample(
                0.0, 0.0, heading=0.0, battery=100.0,
                report_measured=True, measured_drive_w=1.2,
                report_elec=True, measured_drive_elec_w=8.0,
            )
        )
        # Two 90° turns, 10 m, 20 s. No end event yet.
        mid = log.observe(
            _sample(
                10.0, 5.0, heading=math.radians(90.0), battery=99.5,
                report_measured=True, measured_drive_w=1.2,
                report_elec=True, measured_drive_elec_w=8.0,
            )
        )
        assert not any(line.startswith("MISSION_SUMMARY") for line in mid)
        done = log.observe(
            _sample(
                20.0, 10.0, heading=math.radians(180.0), battery=99.0, event="complete",
                report_measured=True, measured_drive_w=1.2,
                report_elec=True, measured_drive_elec_w=8.0,
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
        assert float(fields["wh_model_turn"]) > 0.0
        assert float(fields["wh_model_visc"]) > 0.0
        assert float(fields["wh_model_drive"]) == pytest.approx(
            float(fields["wh_model_roll"])
            + float(fields["wh_model_visc"])
            + float(fields["wh_model_turn"])
            + float(fields["wh_model_accel"]),
            abs=0.003,
        )
        measured = float(fields["wh_measured"])
        assert measured == pytest.approx((7.20 + 2.66 + 8.0) * 20.0 / 3600.0, abs=0.002)
        assert float(fields["wh_measured_drive_shaft"]) == pytest.approx(1.2 * 20.0 / 3600.0, abs=0.002)
        assert float(fields["wh_measured_drive_elec"]) == pytest.approx(8.0 * 20.0 / 3600.0, abs=0.002)
        assert float(fields["eta_avg"]) == pytest.approx(1.2 / 8.0, abs=0.001)
        assert fields["err_pct"] != "na"
        # Error is electrical measured versus electrical model, not shaft.
        elec_wh = 8.0 * 20.0 / 3600.0
        shaft_wh = 1.2 * 20.0 / 3600.0
        model_drive = float(fields["wh_model_drive"])
        assert float(fields["err_pct"]) == pytest.approx(
            (elec_wh - model_drive) / model_drive * 100.0, abs=0.15
        )
        assert float(fields["err_pct"]) != pytest.approx(
            (shaft_wh - model_drive) / model_drive * 100.0, abs=0.5
        )
        assert float(fields["wh_measured"]) == pytest.approx(
            float(fields["wh_measured_drive_elec"]) + (7.20 + 2.66) * 20.0 / 3600.0,
            abs=0.002,
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


def test_dashboard_summary_bills_the_same_stall_hold_as_the_controller(caplog):
    """Wheel rates are on sensors. Both summaries must bill that stall hold.

    The controller JSON copied the rates and reported wh_model_hold. The
    dashboard log used to ignore them, so hold stayed 0 and err_pct moved.
    """
    import logging

    from src.hardware_profile import build_profile_from_bom
    from src.logger import logger
    from src.mission_summary_file import sample_from_controller_payload
    from src.twin.bridge import DigitalTwinBridge
    from src.twin_energy import stall_hold_electrical_wh

    dt_s = 3.0
    expected = stall_hold_electrical_wh(dt_s, build_profile_from_bom("light"))
    assert expected > 0.01

    def payload(ts, event=None):
        sensors = {"left_wheel_rad_s": 0.0, "right_wheel_rad_s": 0.0}
        if event:
            sensors["mission_event"] = event
        return {
            "source": "webots",
            "adapter": "webots",
            "timestamp": ts.isoformat(),
            "robot": {
                "name": "ButlerBot",
                "main_battery_pct": 90.0,
                "battery_capacity_wh": 240.0,
            },
            "mission": {"task": "idle"},
            "channel_draws": {
                "Legs": 0.0, "Arms": 0.0, "Torso": 0.0, "Compute": 8.0, "Cooling": 0.0,
            },
            "locomotion": {
                "gait": "stand",
                "speed_m_s": 0.0,
                "phase": "standby",
                "bom_build": "light",
                "ctrl_bom_build": "light",
                "payload_kg": 0.0,
                "measured_drive_elec_w": 12.0,
            },
            "pose": {"x_m": 0.0, "y_m": 0.0, "heading_rad": 0.0},
            "sensors": sensors,
        }

    first = payload(_T0)
    second = payload(_T0 + timedelta(seconds=dt_s), event="complete")
    assert "left_wheel_rad_s" not in first["locomotion"]

    clog = MissionEnergyLog()
    clog.observe(sample_from_controller_payload(first))
    clines = clog.observe(sample_from_controller_payload(second))
    cfields = _fields(next(line for line in clines if line.startswith("MISSION_SUMMARY")))

    bridge = DigitalTwinBridge()
    with caplog.at_level(logging.INFO, logger=logger.name):
        assert bridge.ingest_telemetry(first, adapter="webots")["ok"] is True
        assert bridge.ingest_telemetry(second, adapter="webots")["ok"] is True
    dlines = [rec.message for rec in caplog.records if rec.message.startswith("MISSION_SUMMARY")]
    assert dlines
    dfields = _fields(dlines[-1])

    # The summary prints hold to 3 decimals, so compare at that precision.
    assert float(cfields["wh_model_hold"]) == pytest.approx(expected, abs=5e-4)
    assert float(dfields["wh_model_hold"]) == pytest.approx(float(cfields["wh_model_hold"]), abs=5e-4)
    assert float(dfields["wh_model_hold"]) > 0.01
    assert dfields["err_pct"] == cfields["err_pct"]
    assert dfields["err_pct"] != "na"


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
        assert seg["wh_measured_drive_shaft"] == "na"
        assert seg["wh_measured_drive_elec"] == "na"
        assert seg["eta_avg"] == "na"
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
        assert float(fields["wh_model_turn"]) > 0.0
        assert float(fields["wh_model_accel"]) > 0
        assert float(fields["wh_model_roll"]) == pytest.approx(0.0066, abs=0.0004)
        assert float(fields["wh_model_visc"]) > float(fields["wh_model_roll"])
        assert float(fields["wh_model_drive"]) == pytest.approx(
            float(fields["wh_model_roll"])
            + float(fields["wh_model_visc"])
            + float(fields["wh_model_turn"])
            + float(fields["wh_model_accel"]),
            abs=1e-4,
        )
        assert fields["wh_measured_drive_shaft"] == "na"
        assert fields["wh_measured_drive_elec"] == "na"
        assert fields["eta_avg"] == "na"
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
        measured_drive_split,
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
        split = measured_drive_split(sensed, v, prof)
        assert split is not None
        shaft_w = 2 * tau * omega
        elec_w = float(split["elec_w"])
        assert split["shaft_w"] == pytest.approx(shaft_w, abs=1e-9)
        # Hinge rate, not body speed / 0.04 m. Doubling ω changes the electrical point.
        doubled = measured_drive_split(
            [
                {"name": "left_wheel", "velocity": v / 0.04, "torque_feedback_nm": tau},
                {"name": "right_wheel", "velocity": v / 0.04, "torque_feedback_nm": tau},
            ],
            v,
            prof,
        )
        assert doubled is not None
        assert elec_w != pytest.approx(float(doubled["elec_w"]), rel=0.05)
        assert stamped["channel_draws"]["Legs"] == pytest.approx(roll["drive_w"], abs=0.15)
        assert stamped["locomotion"]["measured_drive_w"] == pytest.approx(shaft_w, abs=1e-6)
        assert stamped["locomotion"]["measured_drive_elec_w"] == pytest.approx(elec_w, abs=1e-6)
        assert stamped["measured_drive_elec_w"] == pytest.approx(elec_w, abs=1e-6)
        # Legs stay the rolling prediction. Measured electrical is the hinge point.
        assert elec_w != pytest.approx(roll["drive_w"], rel=0.05)

        blank = build_webots_telemetry(
            joints=invented, gait="drive", phase="teleop", speed_m_s=v, profile=prof
        )
        blank_stamped = restamp_active_bom(blank)
        assert blank_stamped["locomotion"]["measured_drive_w"] is None
        assert blank_stamped["locomotion"]["measured_drive_elec_w"] is None

        log = MissionEnergyLog()
        # Two steps stay under the 15 s stall guard. 0.16 m/s × 20 s clears 10 ft.
        def _at(t_s: float, x_m: float) -> dict:
            return _sample(
                t_s, x_m, speed=v,
                report_measured=True, measured_drive_w=shaft_w,
                report_elec=True, measured_drive_elec_w=elec_w,
            )

        log.observe(_at(0.0, 0.0))
        log.observe(_at(10.0, 1.6))
        lines = log.observe(_at(20.0, 3.2))
        seg = _fields(next(line for line in lines if line.startswith("SEGMENT_ENERGY")))
        closed_s = 20.0 * (TEN_FT_M / 3.2)
        assert float(seg["wh_measured_drive_shaft"]) == pytest.approx(shaft_w * closed_s / 3600.0, abs=1e-4)
        assert float(seg["wh_measured_drive_elec"]) == pytest.approx(elec_w * closed_s / 3600.0, abs=1e-4)
        assert float(seg["eta_avg"]) == pytest.approx(shaft_w / elec_w, abs=1e-3)
        assert float(seg["wh_model_roll"]) == pytest.approx(roll["drive_wh_per_10ft"], abs=0.0004)
        assert float(seg["wh_model_drive"]) == pytest.approx(
            float(seg["wh_model_roll"]) + float(seg["wh_model_visc"]) + float(seg["wh_model_accel"]),
            abs=1e-4,
        )
        model_drive = float(seg["wh_model_drive"])
        elec_wh = elec_w * closed_s / 3600.0
        shaft_wh = shaft_w * closed_s / 3600.0
        assert float(seg["err_pct"]) == pytest.approx((elec_wh - model_drive) / model_drive * 100.0, abs=0.15)
        assert float(seg["err_pct"]) != pytest.approx((shaft_wh - model_drive) / model_drive * 100.0, abs=0.5)

        missing = MissionEnergyLog()
        missing.observe(_sample(
            0.0, 0.0, speed=v,
            report_measured=True, measured_drive_w=None,
            report_elec=True, measured_drive_elec_w=None,
        ))
        missing.observe(_sample(
            10.0, 1.6, speed=v,
            report_measured=True, measured_drive_w=None,
            report_elec=True, measured_drive_elec_w=None,
        ))
        gone = missing.observe(_sample(
            20.0, 3.2, speed=v,
            report_measured=True, measured_drive_w=None,
            report_elec=True, measured_drive_elec_w=None,
        ))
        na_seg = _fields(next(line for line in gone if line.startswith("SEGMENT_ENERGY")))
        assert na_seg["wh_measured_drive_shaft"] == "na"
        assert na_seg["wh_measured_drive_elec"] == "na"
        assert na_seg["eta_avg"] == "na"
        assert na_seg["err_pct"] == "na"
        assert float(na_seg["wh_model_roll"]) > 0
    finally:
        _restore_bom(old)


def test_light_motor_electrical_point_cites_published_specs():
    """Shaft → pack watts uses the 4693 no-load/stall line, and names the gaps.

    Kt and R are derived. They do not also reproduce 200 rpm, so the
    operating point stays on the published interpolation. Peak efficiency
    0.5 and gear ratio 50 are not the conversion.
    """
    import math

    from src.hardware_profile import get_active_profile
    from src.teleop_agent import battery_drain_pct
    from src.twin.webots_power import measured_drive_split
    from src.twin_energy import (
        brushed_drive_point,
        electrical_from_shaft_w,
        SimPackMeter,
        electrical_pack_draw_w,
        pack_drain_dt_s,
        published_motor_circuit,
    )

    old = _use_bom("light")
    try:
        spec = get_active_profile()["motors"]["left_wheel"]
        driver = float(get_active_profile()["motor_driver"]["efficiency"])
        circuit = published_motor_circuit(spec, eta_driver=driver)
        assert circuit["voltage_v"] == 24.0
        assert circuit["no_load_rpm"] == 200.0
        assert circuit["no_load_current_a"] == 0.1
        assert circuit["stall_current_a"] == 3.0
        assert circuit["stall_torque_nm"] == pytest.approx(2.256)
        assert circuit["kt_nm_per_a"] == pytest.approx(2.256 / 3.0)
        assert circuit["kt_line_nm_per_a"] == pytest.approx(2.256 / (3.0 - 0.1))
        assert circuit["r_ohm"] == pytest.approx(24.0 / 3.0)
        assert circuit["eta_driver"] == pytest.approx(0.92)
        assert circuit["max_efficiency_datasheet"] == pytest.approx(0.5)
        assert circuit["gear_ratio"] == pytest.approx(50.0)
        rpm_from_kt_r = circuit["omega_from_kt_r_rad_s"] * 60.0 / (2.0 * math.pi)
        assert rpm_from_kt_r == pytest.approx((24.0 - 0.1 * 8.0) / 0.752 * 60.0 / (2.0 * math.pi))
        assert rpm_from_kt_r != pytest.approx(200.0, abs=1.0)
        assert any("Kt" in gap and "R" in gap for gap in circuit["gaps"])
        assert any("0.04" in gap for gap in circuit["gaps"])
        assert any("0.92" in gap for gap in circuit["gaps"])

        tau = 0.4
        omega = 0.16 / 0.08
        point = electrical_from_shaft_w(
            tau,
            omega,
            voltage_v=24.0,
            no_load_rpm=200.0,
            no_load_current_a=0.1,
            stall_current_a=3.0,
            stall_torque_nm=2.256,
            eta_driver=0.92,
        )
        omega0 = 200.0 * 2.0 * math.pi / 60.0
        duty = omega / omega0 + tau / 2.256
        current = 0.1 + (3.0 - 0.1) * (tau / 2.256)
        expect = duty * 24.0 * current / 0.92
        assert point["elec_w"] == pytest.approx(expect, rel=1e-9)
        assert point["shaft_w"] == pytest.approx(tau * omega, rel=1e-9)
        assert point["eta"] == pytest.approx((tau * omega) / expect, rel=1e-9)
        assert point["eta"] != pytest.approx(0.5, abs=0.05)
        assert point["clamped"] is False
        # Same line as the model, at the hinge rate. Mass 0 so the only
        # torque is the measured shaft torque. Radius 0.04 makes the
        # model's ω equal the hinge ω we passed in.
        same_line = brushed_drive_point(
            mass_kg=0.0,
            wheel_radius_m=0.04,
            speed_m_s=omega * 0.04,
            voltage_v=24.0,
            no_load_rpm=200.0,
            no_load_current_a=0.1,
            stall_current_a=3.0,
            stall_torque_nm=2.256,
            eta_driver=0.92,
            n_motors=1,
            extra_torque_nm=tau,
        )
        assert point["elec_w"] == pytest.approx(same_line["drive_w"], rel=1e-9)

        stalled = electrical_from_shaft_w(
            2.256 * 2.0,
            1.0,
            voltage_v=24.0,
            no_load_rpm=200.0,
            no_load_current_a=0.1,
            stall_current_a=3.0,
            stall_torque_nm=2.256,
            eta_driver=0.92,
        )
        assert stalled["clamped"] is True
        assert stalled["current_a"] == pytest.approx(3.0)
        assert stalled["current_a"] <= 3.0

        left_right = [
            {"name": "left_wheel", "velocity": omega, "torque_feedback_nm": 0.2},
            {"name": "right_wheel", "velocity": omega, "torque_feedback_nm": 0.5},
        ]
        split = measured_drive_split(left_right, 0.16, get_active_profile())
        assert split is not None
        parts = [
            electrical_from_shaft_w(
                tau_i, omega,
                voltage_v=24.0, no_load_rpm=200.0, no_load_current_a=0.1,
                stall_current_a=3.0, stall_torque_nm=2.256, eta_driver=0.92,
            )
            for tau_i in (0.2, 0.5)
        ]
        assert split["shaft_w"] == pytest.approx(sum(p["shaft_w"] for p in parts), rel=1e-9)
        assert split["elec_w"] == pytest.approx(sum(p["elec_w"] for p in parts), rel=1e-9)
        assert split["eta"] == pytest.approx(split["shaft_w"] / split["elec_w"], rel=1e-9)
    finally:
        _restore_bom(old)

    # 16646d5 warehouse lap. wh_measured mixed shaft with electrical idle.
    # wh_measured_soc is only the pack percent on the 240 Wh pack.
    soc_wh = 240.0 * (99.60 - 98.66) / 100.0
    assert soc_wh == pytest.approx(2.256, abs=1e-9)
    assert 3.724 - 1.206 == pytest.approx(2.518, abs=1e-9)
    assert 2.518 == pytest.approx(9.96 * 919.1 / 3600.0, abs=0.03)
    assert (2.543 + 0.441) != pytest.approx(soc_wh, abs=0.2)
    assert 1.206 != pytest.approx(3.909, abs=0.5)

    channels = {"Compute": 9.96, "Legs": 3.43, "Arms": 0.0, "Torso": 0.0, "Cooling": 0.0}
    pack = electrical_pack_draw_w(channels, 15.0)
    assert pack == pytest.approx(9.96 + 15.0)
    assert pack != pytest.approx(9.96 + 3.43)
    assert electrical_pack_draw_w(channels, None) is None
    dt = 10.0
    drop = battery_drain_pct(pack, dt, capacity_wh=240.0)
    assert drop / 100.0 * 240.0 == pytest.approx(pack * dt / 3600.0, rel=1e-9)
    assert pack_drain_dt_s(None, sim_dt_s=0.032) == pytest.approx(0.032)
    assert pack_drain_dt_s(2.0, sim_dt_s=0.032) == pytest.approx(2.0)
    assert pack_drain_dt_s(20.0, sim_dt_s=0.032) == 0.0
    # Live drain is the simulation step, not that wall gap. One step of
    # 20 W drive and 9.96 W idle matches the percent on a 240 Wh pack.
    meter = SimPackMeter()
    step_wh = meter.step(0.008, shaft_w=4.0, elec_w=20.0, idle_w=9.96)
    assert step_wh == pytest.approx((20.0 + 9.96) * 0.008 / 3600.0, rel=1e-12)
    assert meter.snapshot()["energy_wh_pack"] == pytest.approx(step_wh, rel=1e-12)
    assert meter.step(20.0, shaft_w=4.0, elec_w=20.0, idle_w=9.96) == 0.0
    missed = SimPackMeter()
    missed.step(0.008, shaft_w=None, elec_w=None, idle_w=9.96)
    assert missed.elec_gaps == 1
    assert missed.wh_elec == 0.0
    assert missed.wh_idle == pytest.approx(9.96 * 0.008 / 3600.0, rel=1e-12)

    controller = Path(
        "webots/controllers/butlerbot_controller/butlerbot_controller.py"
    ).read_text(encoding="utf-8")
    assert "SimPackMeter" in controller
    assert "pack_drain_dt_s" not in controller
    assert "measured_drive_split" in controller
    assert "pack_meter.snapshot()" in controller


def _with_counter(sample: dict, **fields) -> dict:
    out = dict(sample)
    out.update(fields)
    return out


def test_restamp_keeps_the_controller_energy_counters():
    from src.twin.adapters.webots import WebotsAdapter

    old = _use_bom("light")
    try:
        payload = {
            "robot": {
                "name": "ButlerBot",
                "main_battery_pct": 99.0,
                "battery_capacity_wh": 240,
            },
            "joints": [
                {"name": "left_wheel", "velocity": 1.0, "torque": 0.0, "torque_feedback_nm": 0.2},
                {"name": "right_wheel", "velocity": 1.0, "torque": 0.0, "torque_feedback_nm": 0.2},
            ],
            "locomotion": {
                "gait": "drive",
                "speed_m_s": 0.2,
                "controller_run_id": "run-a",
                "energy_sim_s": 1.5,
                "energy_wh_pack": 0.02,
                "energy_wh_elec": 0.01,
                "energy_wh_idle": 0.01,
                "energy_wh_shaft": 0.004,
                "steer_split": "STEER_SPLIT n=3",
            },
            "channel_draws": {"Compute": 9.96, "Legs": 3.0},
        }
        tel = WebotsAdapter().normalize(payload)
        assert tel.locomotion["controller_run_id"] == "run-a"
        assert tel.locomotion["energy_wh_pack"] == pytest.approx(0.02)
        assert tel.locomotion["energy_sim_s"] == pytest.approx(1.5)
        assert str(tel.locomotion["steer_split"]).startswith("STEER_SPLIT")
    finally:
        _restore_bom(old)


def test_controller_counter_keeps_energy_the_15s_rule_used_to_drop():
    """A 20 s post gap used to drop the watt-hours. The counter does not.

    Run #11: wh_measured 6.381 vs wh_measured_soc 7.224. Posts were a
    few seconds apart, and a hole over 15 s moved the percent without
    entering the log. The controller integral is what both totals use.
    """
    idle_w = 7.20 + 2.66
    elec_w = 20.0
    shaft_w = 5.0
    sim_s = 10.0
    pack = (elec_w + idle_w) * sim_s / 3600.0
    log = MissionEnergyLog()
    arm = _with_counter(
        _sample(
            0.0, 0.0, battery=100.0,
            report_measured=True, measured_drive_w=shaft_w,
            report_elec=True, measured_drive_elec_w=elec_w,
        ),
        controller_run_id="run-a",
        energy_sim_s=0.0,
        energy_wh_shaft=0.0,
        energy_wh_elec=0.0,
        energy_wh_idle=0.0,
        energy_wh_pack=0.0,
        energy_elec_gaps=0,
    )
    assert log.observe(arm) == []
    done = log.observe(
        _with_counter(
            _sample(
                20.0, 4.0, speed=0.4, battery=100.0 - pack / 240.0 * 100.0,
                event="complete",
                report_measured=True, measured_drive_w=shaft_w,
                report_elec=True, measured_drive_elec_w=elec_w,
            ),
            controller_run_id="run-a",
            energy_sim_s=sim_s,
            energy_wh_shaft=shaft_w * sim_s / 3600.0,
            energy_wh_elec=elec_w * sim_s / 3600.0,
            energy_wh_idle=idle_w * sim_s / 3600.0,
            energy_wh_pack=pack,
            energy_elec_gaps=0,
            steer_split="STEER_SPLIT n=4 var_cmd_split=0.0400",
        )
    )
    summary = _fields(next(line for line in done if line.startswith("MISSION_SUMMARY")))
    gaps = _fields(next(line for line in done if line.startswith("TELEMETRY_GAPS")))
    assert float(summary["duration_s"]) == pytest.approx(sim_s, abs=0.05)
    assert float(summary["wh_measured"]) == pytest.approx(pack, abs=0.001)
    assert float(summary["wh_measured_soc"]) == pytest.approx(pack, abs=0.001)
    assert float(summary["wh_measured_drive_elec"]) == pytest.approx(
        elec_w * sim_s / 3600.0, abs=0.001
    )
    assert gaps["cumulative"] == "1"
    assert float(gaps["skipped_s"]) == pytest.approx(20.0, abs=0.01)
    assert float(gaps["skipped_wh"]) == pytest.approx(pack, abs=0.001)
    assert int(gaps["n"]) == 1
    assert float(gaps["max_s"]) == pytest.approx(20.0, abs=0.01)
    assert gaps["sim_s"] == "10.000"
    assert any(line.startswith("STEER_SPLIT ") for line in done)


def test_post_hole_keeps_the_counter_and_elec_gaps_are_steps():
    """camera_s on 875eb13 reported 106/246 samples and max 0.8 s.

    ``n`` is the number of post intervals. ``elec_gaps`` is physics steps
    whose electrical watts were missing, not a fraction of those posts.
    106 steps at 8 ms is 0.848 s, about 1.4% of a 60.5 s lap. A 0.8 s
    hole between posts does not drop watt-hours already in the controller
    counter. Gap steps add 0 drive Wh, so they undercount only while the
    hinge was turning without feedback.
    """
    from src.twin_energy import SimPackMeter

    meter = SimPackMeter()
    for _ in range(106):
        meter.step(0.008, shaft_w=None, elec_w=None, idle_w=19.0)
    assert meter.elec_gaps == 106
    assert meter.wh_elec == 0.0
    assert meter.sim_s == pytest.approx(106 * 0.008, abs=1e-12)
    assert meter.sim_s / 60.5 == pytest.approx(0.014016, abs=1e-6)
    meter.step(0.008, shaft_w=4.0, elec_w=20.0, idle_w=19.0)
    assert meter.wh_elec == pytest.approx(20.0 * 0.008 / 3600.0, rel=1e-9)

    idle_w = 7.20 + 2.66
    elec_w = 20.0
    shaft_w = 4.0
    sim_s = 60.5
    pack = (elec_w + idle_w) * sim_s / 3600.0
    log = MissionEnergyLog()
    assert log.observe(
        _with_counter(
            _sample(
                0.0, 0.0,
                report_measured=True, measured_drive_w=shaft_w,
                report_elec=True, measured_drive_elec_w=elec_w,
            ),
            controller_run_id="camera-s",
            energy_sim_s=0.0,
            energy_wh_shaft=0.0,
            energy_wh_elec=0.0,
            energy_wh_idle=0.0,
            energy_wh_pack=0.0,
            energy_elec_gaps=0,
            steer_source="camera",
        )
    ) == []
    done = log.observe(
        _with_counter(
            _sample(
                0.8, 0.26, event="complete",
                report_measured=True, measured_drive_w=shaft_w,
                report_elec=True, measured_drive_elec_w=elec_w,
            ),
            controller_run_id="camera-s",
            energy_sim_s=sim_s,
            energy_wh_shaft=shaft_w * sim_s / 3600.0,
            energy_wh_elec=elec_w * sim_s / 3600.0,
            energy_wh_idle=idle_w * sim_s / 3600.0,
            energy_wh_pack=pack,
            energy_elec_gaps=106,
            steer_source="camera",
        )
    )
    summary = _fields(next(line for line in done if line.startswith("MISSION_SUMMARY")))
    gaps = _fields(next(line for line in done if line.startswith("TELEMETRY_GAPS")))
    assert float(summary["duration_s"]) == pytest.approx(sim_s, abs=0.05)
    assert float(summary["wh_measured_drive_elec"]) == pytest.approx(
        elec_w * sim_s / 3600.0, abs=0.001
    )
    assert float(summary["wh_measured"]) == pytest.approx(pack, abs=0.001)
    assert gaps["n"] == "1"
    assert float(gaps["max_s"]) == pytest.approx(0.8, abs=0.01)
    assert gaps["elec_gaps"] == "106"
    assert gaps["cumulative"] == "1"
    assert float(gaps["skipped_s"]) == pytest.approx(0.0, abs=1e-9)
    assert float(summary["duration_s"]) != pytest.approx(0.8, abs=0.05)


def test_legacy_integrator_records_a_skipped_gap_and_does_not_bill_it():
    idle_w = 7.20 + 2.66
    elec_w = 20.0
    log = MissionEnergyLog()
    log.observe(
        _sample(
            0.0, 0.0,
            report_measured=True, measured_drive_w=5.0,
            report_elec=True, measured_drive_elec_w=elec_w,
        )
    )
    log.observe(
        _sample(
            10.0, 3.0, speed=0.3,
            report_measured=True, measured_drive_w=5.0,
            report_elec=True, measured_drive_elec_w=elec_w,
        )
    )
    done = log.observe(
        _sample(
            40.0, 3.1, speed=0.0, event="complete",
            report_measured=True, measured_drive_w=5.0,
            report_elec=True, measured_drive_elec_w=elec_w,
        )
    )
    summary = _fields(next(line for line in done if line.startswith("MISSION_SUMMARY")))
    gaps = _fields(next(line for line in done if line.startswith("TELEMETRY_GAPS")))
    billed = (elec_w + idle_w) * 10.0 / 3600.0
    skipped = (elec_w + idle_w) * 30.0 / 3600.0
    assert float(summary["duration_s"]) == pytest.approx(10.0, abs=0.05)
    assert float(summary["wh_measured"]) == pytest.approx(billed, abs=0.001)
    assert float(summary["wh_measured"]) < billed + skipped * 0.5
    assert gaps["cumulative"] == "0"
    assert gaps["sim_s"] == "na"
    assert float(gaps["skipped_s"]) == pytest.approx(30.0, abs=0.01)
    assert float(gaps["skipped_wh"]) == pytest.approx(skipped, abs=0.001)
    assert int(gaps["n"]) == 2
    assert float(gaps["mean_s"]) == pytest.approx(20.0, abs=0.01)
    assert float(gaps["max_s"]) == pytest.approx(30.0, abs=0.01)


def test_new_controller_run_summarizes_the_previous_mission_and_resets():
    log = MissionEnergyLog()
    common = dict(
        report_measured=True, measured_drive_w=5.0,
        report_elec=True, measured_drive_elec_w=20.0,
    )
    log.observe(_with_counter(_sample(0.0, 0.0, **common), controller_run_id="webots-1"))
    log.observe(_with_counter(_sample(10.0, 8.0, speed=0.8, **common), controller_run_id="webots-1"))
    restarted = log.observe(
        _with_counter(_sample(0.0, 0.0, speed=0.0, **common), controller_run_id="webots-2")
    )
    summary = _fields(next(line for line in restarted if line.startswith("MISSION_SUMMARY")))
    assert float(summary["dist_m"]) == pytest.approx(8.0, abs=0.05)
    assert float(summary["duration_s"]) == pytest.approx(10.0, abs=0.05)
    # The new process only armed. Its first metres are a new segment 0.
    moved = log.observe(
        _with_counter(
            _sample(10.0, TEN_FT_M, speed=TEN_FT_M / 10.0, **common),
            controller_run_id="webots-2",
        )
    )
    seg = _fields(next(line for line in moved if line.startswith("SEGMENT_ENERGY")))
    assert seg["seg_idx"] == "0"
    assert float(seg["dist_m"]) == pytest.approx(TEN_FT_M, abs=1e-3)


def test_a_trivial_restart_does_not_consume_the_summary():
    log = MissionEnergyLog()
    log.observe(_with_counter(_sample(0.0, 0.0, speed=0.0), controller_run_id="webots-1"))
    restarted = log.observe(
        _with_counter(_sample(0.2, 0.01, speed=0.0), controller_run_id="webots-2")
    )
    assert not any(line.startswith("MISSION_SUMMARY") for line in restarted)
    log.observe(_with_counter(_sample(5.0, 4.0, speed=0.8), controller_run_id="webots-2"))
    done = log.observe(
        _with_counter(
            _sample(8.0, 6.0, speed=0.6, event="complete"),
            controller_run_id="webots-2",
        )
    )
    assert any(line.startswith("MISSION_SUMMARY") for line in done)


def test_same_run_id_does_not_reset_the_segment_index():
    log = MissionEnergyLog()
    common = dict(
        report_measured=True, measured_drive_w=5.0,
        report_elec=True, measured_drive_elec_w=20.0,
    )
    log.observe(_with_counter(_sample(0.0, 0.0, **common), controller_run_id="webots-1"))
    first = log.observe(
        _with_counter(_sample(8.0, 4.0, speed=0.5, **common), controller_run_id="webots-1")
    )
    assert any(line.startswith("SEGMENT_ENERGY") for line in first)
    second = log.observe(
        _with_counter(_sample(16.0, 8.0, speed=0.5, **common), controller_run_id="webots-1")
    )
    seg = _fields(next(line for line in second if line.startswith("SEGMENT_ENERGY")))
    assert seg["seg_idx"] == "1"


def test_blank_bom_stamps_measured_from_the_yaml_motor():
    """Shaft |τ|·|ω| does not need a catalog build. Legs are left alone."""
    from src.hardware_profile import active_bom_build, get_active_profile
    from src.twin.webots_power import (
        build_webots_telemetry,
        measured_drive_split,
        restamp_active_bom,
    )

    old = _use_bom(None)
    try:
        assert active_bom_build() is None
        v = 0.16
        omega = v / 0.08
        tau = 0.2
        joints = [
            {"name": "left_wheel", "velocity": omega, "torque": 0.4, "torque_feedback_nm": tau},
            {"name": "right_wheel", "velocity": omega, "torque": 0.4, "torque_feedback_nm": tau},
        ]
        prof = get_active_profile()
        payload = build_webots_telemetry(
            joints=joints, gait="drive", phase="teleop", speed_m_s=v, profile=prof
        )
        legs = payload["channel_draws"]["Legs"]
        stamped = restamp_active_bom(payload)
        split = measured_drive_split(joints, v, prof)
        assert split is not None and split["elec_w"] is not None
        assert split["shaft_w"] == pytest.approx(2 * tau * omega)
        assert stamped["channel_draws"]["Legs"] == legs
        assert stamped["locomotion"]["measured_drive_w"] == pytest.approx(split["shaft_w"])
        assert stamped["locomotion"]["measured_drive_elec_w"] == pytest.approx(split["elec_w"])
        assert split["elec_w"] > split["shaft_w"]
        quiet = build_webots_telemetry(
            joints=[
                {"name": "left_wheel", "velocity": omega, "torque": 0.4},
                {"name": "right_wheel", "velocity": omega, "torque": 0.4},
            ],
            gait="drive",
            phase="teleop",
            speed_m_s=v,
            profile=prof,
        )
        missing = restamp_active_bom(quiet)
        assert missing["locomotion"]["measured_drive_w"] is None
        assert missing["locomotion"]["measured_drive_elec_w"] is None
        assert missing["channel_draws"]["Legs"] == quiet["channel_draws"]["Legs"]
    finally:
        _restore_bom(old)


def test_summary_file_keeps_body_summary_and_measured_without_a_bom_build(tmp_path):
    """The three lines land in a file. Measured Wh is the hinge integral."""
    from src.mission_summary_file import (
        append_mission_lines,
        observe_controller_payload,
        reset_mission_summary_file,
    )
    from src.webots_body import format_bom_body_log_line

    old = _use_bom(None)
    reset_mission_summary_file()
    try:
        body = format_bom_body_log_line()
        assert body is not None and body.startswith("BOM BODY build=default ")
        assert "contact_radius_m=0.08" in body
        append_mission_lines(
            [body],
            source="controller",
            directory=str(tmp_path),
            mirror_dashboard_log=True,
        )
        log = MissionEnergyLog()
        loco = {
            "gait": "drive",
            "speed_m_s": 0.4,
            "energy_sim_s": 0.0,
            "energy_wh_shaft": 0.0,
            "energy_wh_elec": 0.0,
            "energy_wh_idle": 0.0,
            "energy_wh_pack": 0.0,
            "steer_source": "camera",
        }
        base = {
            "source": "webots",
            "robot": {"name": "ButlerBot", "main_battery_pct": 99.0, "battery_capacity_wh": 480},
            "channel_draws": {"Compute": 10.0, "Legs": 3.0, "Arms": 0.0, "Torso": 0.0},
            "pose": {"x_m": 0.0, "y_m": 0.0, "heading_rad": 0.0},
            "locomotion": loco,
            "sensors": {},
            "timestamp": _T0.isoformat(),
        }
        observe_controller_payload(log, base, directory=str(tmp_path))
        done_payload = {
            **base,
            "timestamp": (_T0 + timedelta(seconds=20)).isoformat(),
            "pose": {"x_m": 8.0, "y_m": 0.2, "heading_rad": 0.1},
            "sensors": {"mission_event": "complete"},
            "locomotion": {
                **loco,
                "energy_sim_s": 20.0,
                "energy_wh_shaft": 0.05,
                "energy_wh_elec": 0.20,
                "energy_wh_idle": 0.06,
                "energy_wh_pack": 0.26,
                "steer_source": "camera",
                "width_err_m": 0.011,
                "center_err_m": -0.006,
                "ct_max_m": 0.02,
            },
        }
        lines, written = observe_controller_payload(log, done_payload, directory=str(tmp_path))
        summary = next(line for line in lines if line.startswith("MISSION_SUMMARY"))
        energy = next(line for line in lines if line.startswith("MISSION_ENERGY"))
        assert "steer_frac_camera=" in summary
        assert "ct_max_m=" in summary
        assert "width_err_rms_m=" in summary
        assert "center_err_rms_m=" in summary
        fields = _fields(summary)
        assert fields["wh_measured"] != "na"
        assert float(fields["wh_measured"]) == pytest.approx(0.26, abs=0.001)
        # No BoM build: the summary model column is blank. Measured is the hinge integral.
        assert fields["wh_model"] == ""
        assert fields["build"] == "default"
        energy_fields = _fields(energy)
        assert energy_fields["build"] == "default"
        assert energy_fields["wh_measured"] not in ("", "na")
        assert float(energy_fields["wh_measured"]) == pytest.approx(0.26, abs=0.02)
        assert energy_fields["wh_model"] not in ("", "na")
        # V1 cruise lump vs measured drive + idle is not a like-for-like error.
        assert energy_fields["err_pct"] == "na"
        text = Path(written).read_text(encoding="utf-8")
        assert "BOM BODY build=default" in text
        assert "MISSION_SUMMARY" in text
        assert "MISSION_ENERGY" in text
        assert "steer_frac_camera=" in text
        assert "width_err_rms_m=" in text
        logged = next(tmp_path.glob("robot_monitor_*.log")).read_text(encoding="utf-8")
        assert "BOM BODY build=default" in logged
        assert "MISSION_SUMMARY" in logged
        assert "wh_measured=" in logged
    finally:
        reset_mission_summary_file()
        _restore_bom(old)


def test_controller_writes_the_summary_file_and_meters_a_blank_build():
    controller = Path(
        "webots/controllers/butlerbot_controller/butlerbot_controller.py"
    ).read_text(encoding="utf-8")
    assert "observe_controller_payload" in controller
    assert "MISSION_SUMMARY written" in controller
    assert "append_mission_lines" in controller
    assert "if pack_meter is not None and _uses_bom_pack():" not in controller
    assert "loco.update(pack_meter.snapshot())" in controller

