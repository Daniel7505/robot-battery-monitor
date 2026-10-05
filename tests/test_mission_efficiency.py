"""Pure-Python mission efficiency tracker tests (no Webots / dashboard)."""

from datetime import datetime, timedelta, timezone

import pytest

from src.mission_efficiency import (
    MissionEfficiencyTracker,
    compute_efficiency_delta,
    format_elapsed,
)
from src.twin.bridge import DigitalTwinBridge
from src.twin.control import build_twin_control_status


def _ts(seconds: float = 0.0) -> datetime:
    return datetime(2026, 10, 5, 18, 0, 0, tzinfo=timezone.utc) + timedelta(seconds=seconds)


def test_format_elapsed():
    assert format_elapsed(None) == "—"
    assert format_elapsed(65) == "1:05"
    assert format_elapsed(3661) == "1:01:01"


def test_tracker_arms_and_elapsed():
    t = MissionEfficiencyTracker(capacity_wh=480.0)
    snap = t.update(
        timestamp=_ts(0),
        battery_pct=90.0,
        channel_draws={"Legs": 20.0, "Compute": 10.0},
        pose={"x_m": 0.0, "y_m": 0.0},
    )
    assert snap["active"] is True
    assert snap["elapsed_s"] == 0.0
    assert snap["battery_start_pct"] == 90.0
    assert snap["distance_m"] == 0.0

    snap2 = t.update(
        timestamp=_ts(10),
        battery_pct=89.5,
        channel_draws={"Legs": 20.0, "Compute": 10.0},
        pose={"x_m": 4.0, "y_m": 0.0},
    )
    assert snap2["elapsed_s"] == pytest.approx(10.0)
    assert snap2["battery_delta_pct"] == pytest.approx(0.5)
    # SoC: 0.5% of 480 Wh = 2.4 Wh
    assert snap2["energy_wh_soc"] == pytest.approx(2.4)
    assert snap2["energy_wh_used"] == pytest.approx(2.4)
    assert snap2["energy_source"] == "soc_delta"
    assert snap2["distance_m"] == pytest.approx(4.0)
    assert snap2["avg_speed_m_s"] == pytest.approx(0.4)


def test_integrated_draw_when_soc_flat():
    t = MissionEfficiencyTracker(capacity_wh=480.0)
    t.update(timestamp=_ts(0), battery_pct=88.0, channel_draws={"Legs": 36.0}, pose=None)
    snap = t.update(
        timestamp=_ts(10),
        battery_pct=88.0,
        channel_draws={"Legs": 36.0},
        pose=None,
    )
    # 36 W * 10 s / 3600 = 0.1 Wh
    assert snap["energy_wh_integrated"] == pytest.approx(0.1)
    assert snap["energy_source"] == "integrated_draw"
    assert snap["energy_wh_used"] == pytest.approx(0.1)
    assert snap["distance_m"] is None
    assert snap["avg_speed_m_s"] is None


def test_reset_clears_totals():
    t = MissionEfficiencyTracker(capacity_wh=480.0)
    t.update(timestamp=_ts(0), battery_pct=90.0, channel_draws={"Legs": 10}, pose={"x_m": 0, "y_m": 0})
    t.update(timestamp=_ts(5), battery_pct=89.0, channel_draws={"Legs": 10}, pose={"x_m": 2, "y_m": 0})
    t.reset()
    snap = t.snapshot()
    assert snap["active"] is False
    assert snap["samples"] == 0
    snap2 = t.update(timestamp=_ts(20), battery_pct=95.0, channel_draws={}, pose={"x_m": 10, "y_m": 0})
    assert snap2["battery_start_pct"] == 95.0
    assert snap2["distance_m"] == 0.0


def test_dt_spike_skips_energy_keeps_distance():
    t = MissionEfficiencyTracker(capacity_wh=480.0, max_dt_s=5.0)
    t.update(timestamp=_ts(0), battery_pct=90.0, channel_draws={"Legs": 40}, pose={"x_m": 0, "y_m": 0})
    snap = t.update(
        timestamp=_ts(30),  # > max_dt_s → no ∫P, but GPS chord still counts
        battery_pct=90.0,
        channel_draws={"Legs": 40},
        pose={"x_m": 10, "y_m": 0},
    )
    assert snap["energy_wh_integrated"] == pytest.approx(0.0)
    assert snap["distance_m"] == pytest.approx(10.0)
    assert snap["elapsed_s"] == pytest.approx(30.0)


def test_compute_efficiency_delta_helper():
    out = compute_efficiency_delta(
        elapsed_s=60.0,
        battery_start_pct=90.0,
        battery_end_pct=89.0,
        capacity_wh=480.0,
        distance_m=20.0,
    )
    assert out["elapsed_s"] == pytest.approx(60.0)
    assert out["energy_wh_soc"] == pytest.approx(4.8)
    assert out["distance_m"] == pytest.approx(20.0)
    assert out["avg_speed_m_s"] == pytest.approx(20.0 / 60.0, rel=1e-3)


def test_bridge_ingest_updates_efficiency():
    bridge = DigitalTwinBridge()
    bridge.reset_efficiency(capacity_wh=480.0)
    r1 = bridge.ingest_telemetry(
        {
            "source": "webots",
            "timestamp": _ts(0).isoformat(),
            "robot": {"main_battery_pct": 90.0, "battery_capacity_wh": 480},
            "channel_draws": {"Legs": 18, "Arms": 4, "Torso": 3, "Compute": 9},
            "locomotion": {"gait": "drive", "phase": "drive_transit", "speed_m_s": 0.4},
            "pose": {"x_m": 0.0, "y_m": 0.0},
            "mission": {"task": "moving"},
        },
        adapter="webots",
    )
    assert r1["ok"] is True
    assert r1["efficiency"]["active"] is True

    r2 = bridge.ingest_telemetry(
        {
            "source": "webots",
            "timestamp": _ts(20).isoformat(),
            "robot": {"main_battery_pct": 89.8, "battery_capacity_wh": 480},
            "channel_draws": {"Legs": 18, "Arms": 4, "Torso": 3, "Compute": 9},
            "locomotion": {"gait": "drive", "phase": "drive_transit", "speed_m_s": 0.4},
            "pose": {"x_m": 8.0, "y_m": 0.0},
            "mission": {"task": "moving"},
        },
        adapter="webots",
    )
    eff = r2["efficiency"]
    assert eff["elapsed_s"] == pytest.approx(20.0)
    assert eff["distance_m"] == pytest.approx(8.0)
    assert bridge.status()["efficiency"]["distance_m"] == pytest.approx(8.0)

    class _HW:
        allocation_status = {"task": "moving", "status": "ok", "utilization_pct": 40, "throttled_channels": []}
        mission_info = {"task": "moving", "task_label": "Moving"}
        agent_status = {"posture": "normal", "controlling": False, "recommendations": [], "applied_actions": []}

    status = build_twin_control_status(bridge, _HW())
    assert status["efficiency"]["elapsed_s"] == pytest.approx(20.0)
    assert status["efficiency"]["distance_m"] == pytest.approx(8.0)


def test_battery_reset_resets_efficiency():
    bridge = DigitalTwinBridge()
    bridge.ingest_telemetry(
        {
            "source": "webots",
            "timestamp": _ts(0).isoformat(),
            "robot": {"main_battery_pct": 80.0},
            "channel_draws": {"Legs": 10, "Arms": 4, "Torso": 3, "Compute": 8},
            "locomotion": {"gait": "drive", "phase": "teleop", "speed_m_s": 0.2},
            "pose": {"x_m": 0.0, "y_m": 0.0},
            "mission": {"task": "moving"},
        },
        adapter="webots",
    )
    bridge.ingest_telemetry(
        {
            "source": "webots",
            "timestamp": _ts(15).isoformat(),
            "robot": {"main_battery_pct": 79.0},
            "channel_draws": {"Legs": 10, "Arms": 4, "Torso": 3, "Compute": 8},
            "locomotion": {"gait": "drive", "phase": "teleop", "speed_m_s": 0.2},
            "pose": {"x_m": 3.0, "y_m": 0.0},
            "mission": {"task": "moving"},
        },
        adapter="webots",
    )
    assert bridge.status()["efficiency"]["samples"] >= 2

    class _HW:
        _main_battery = 79.0

    out = bridge.apply_command(_HW(), {"battery_reset": True, "battery_pct": 100})
    assert out["ok"] is True
    assert bridge.status()["efficiency"]["active"] is False
