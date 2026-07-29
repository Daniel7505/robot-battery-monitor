"""Twin motion should move PMS mission off Idle while driving."""

from types import SimpleNamespace

from src.hardware_ros2 import ROS2BatterySource
from src.twin.control import PHASE_LABELS, build_twin_control_status, _phase_index, webots_phase_flow


def _tel(*, gait="stand", phase="standby", speed=0.0, task="idle"):
    return SimpleNamespace(
        task=task,
        source="webots",
        locomotion={"gait": gait, "phase": phase, "speed_m_s": speed, "mode": "wheeled"},
        pose={},
        raw={},
    )


def test_task_from_twin_drive_gait():
    assert ROS2BatterySource._task_from_twin_telemetry(
        _tel(gait="drive", phase="teleop", speed=0.4, task="idle")
    ) == "moving"


def test_task_from_twin_speed_alone():
    assert ROS2BatterySource._task_from_twin_telemetry(
        _tel(gait="stand", phase="standby", speed=0.25, task="idle")
    ) == "moving"


def test_task_from_twin_idle_when_stopped():
    assert ROS2BatterySource._task_from_twin_telemetry(
        _tel(gait="stand", phase="standby", speed=0.0, task="idle")
    ) == "idle"


def test_teleop_phase_label_and_timeline_index():
    assert PHASE_LABELS["teleop"] == "Teleop Drive"
    flow = webots_phase_flow()
    idx = _phase_index("teleop", flow)
    assert idx >= 0
    assert flow[idx]["phase"] == "drive_transit"


def test_control_status_overrides_idle_pms_when_driving():
    class Bridge:
        def status(self):
            return {
                "external_active": True,
                "active_source": "webots",
                "telemetry_count": 3,
                "last_telemetry_at": None,
            }

        _last_telemetry = SimpleNamespace(
            locomotion={"gait": "drive", "phase": "teleop", "speed_m_s": 0.42, "mode": "wheeled"},
            pose={"x_m": 0.1, "y_m": 0.0},
            raw={"sensors": {}},
            task="idle",
        )

    class Hardware:
        allocation_status = {"task": "idle", "status": "ok", "utilization_pct": 40, "throttled_channels": []}
        mission_info = {"task": "idle", "task_label": "Idle / Standby"}
        agent_status = {"posture": "normal", "intervening": False, "applied_actions": [], "summary": ""}

    status = build_twin_control_status(Bridge(), Hardware())
    assert status["pms_task"] == "moving"
    assert "Idle" not in (status["pms_task_label"] or "")
    assert status["sim_phase_label"] == "Teleop Drive"
