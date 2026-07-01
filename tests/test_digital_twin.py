import time

from src.twin import DigitalTwinBridge, get_twin_bridge, reset_twin_bridge
from src.twin.butlerbot import BUTLERBOT_WALKING_FLOW, butlerbot_telemetry_step
from src.twin.adapters import get_adapter
from src.digital_twin import DigitalTwinInterface, get_twin_interface, reset_twin_interface
from src.hardware import reset_hardware_source
from src.hardware_ros2 import ROS2BatterySource


def test_backward_compat_aliases():
    reset_twin_bridge()
    assert DigitalTwinInterface is DigitalTwinBridge
    assert get_twin_interface() is get_twin_bridge()
    reset_twin_interface()


def test_bridge_schema_documents_contract():
    bridge = DigitalTwinBridge()
    schema = bridge.schema()
    assert schema["schema_version"] == "1.1"
    assert schema["bridge_class"] == "DigitalTwinBridge"
    assert "webots" in schema["adapters"]
    assert "butlerbot_example" in schema


def test_butlerbot_walking_flow():
    assert len(BUTLERBOT_WALKING_FLOW) == 5
    tasks = [s["task"] for s in BUTLERBOT_WALKING_FLOW]
    assert "moving" in tasks
    step = butlerbot_telemetry_step(1)
    assert step["mission"]["task"] == "moving"
    assert step["channel_draws"]["Legs"] == 22.0


def test_ingest_butlerbot_telemetry():
    reset_twin_bridge()
    bridge = DigitalTwinBridge()
    payload = butlerbot_telemetry_step(0, source="custom", adapter="butlerbot")
    result = bridge.ingest_telemetry(payload)
    assert result["ok"] is True
    assert result["source"] == "custom"
    assert bridge.active_source == "custom"
    assert bridge.status()["telemetry_count"] == 1


def test_webots_adapter_normalizes_motors():
    adapter = get_adapter("webots")
    tel = adapter.normalize({
        "motor_power_w": {"leg_left": 8.0, "leg_right": 9.0, "gripper": 4.0},
        "gait": "walk",
        "phase": "walk_transit",
        "robot": {"name": "ButlerBot"},
    })
    # walk_transit stress is damped when speed/joints show no motion (~0.15 scale)
    assert tel.channel_draws["Legs"] >= 19.0
    assert tel.channel_draws["Arms"] >= 4.0
    assert tel.task == "moving"


def test_pybullet_adapter_maps_joints():
    adapter = get_adapter("pybullet")
    tel = adapter.normalize({
        "joints": [
            {"name": "hip_left", "torque": 2.5, "velocity": 1.2},
            {"name": "elbow_right", "torque": 1.0, "velocity": 0.8},
        ],
        "locomotion": {"mode": "walk"},
    })
    assert tel.channel_draws.get("Legs", 0) > 0
    assert tel.task == "moving"


def test_bridge_export_state_from_hardware():
    reset_twin_bridge()
    reset_hardware_source()
    source = ROS2BatterySource()
    source.start()
    time.sleep(4)
    bridge = DigitalTwinBridge()
    state = bridge.export_state(source)
    assert state["schema_version"] == "1.1"
    assert "bridge" in state
    assert state["robot"]["main_battery_pct"] is not None
    assert len(state["channels"]) >= 1
    source.stop()
    reset_hardware_source()


def test_bridge_sync_external_to_hardware():
    reset_twin_bridge()
    reset_hardware_source()
    source = ROS2BatterySource()
    source.start()
    time.sleep(2)
    bridge = DigitalTwinBridge()
    payload = butlerbot_telemetry_step(1, source="custom")
    bridge.ingest_telemetry(payload)
    assert bridge.sync_to_hardware(source) is True
    draws = source._ros2.get_sensor_draws()
    assert draws.get("Legs") == 22.0
    source.stop()
    reset_hardware_source()


def test_bridge_apply_mission_command():
    reset_twin_bridge()
    reset_hardware_source()
    source = ROS2BatterySource()
    source.start()
    time.sleep(2)
    bridge = DigitalTwinBridge()
    result = bridge.apply_command(source, {"task": "idle"})
    assert result["ok"] is True
    assert "mission=idle" in result["applied"]
    source.stop()
    reset_hardware_source()


def test_bridge_rejects_invalid_task():
    bridge = DigitalTwinBridge()
    reset_hardware_source()
    source = ROS2BatterySource()
    result = bridge.apply_command(source, {"task": "fly"})
    assert result["ok"] is False
    assert result["errors"]


def test_get_bridge_singleton():
    reset_twin_bridge()
    assert get_twin_bridge() is get_twin_bridge()
    reset_twin_bridge()