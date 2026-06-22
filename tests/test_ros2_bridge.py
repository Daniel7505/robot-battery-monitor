import time

from src.ros2_bridge import ROS2Bridge
from src.hardware import reset_hardware_source, get_hardware_source, get_hardware_mode, SimulatorSource
from src.hardware_ros2 import ROS2BatterySource
from src.mission_tasks import MissionTaskManager


def test_ros2_bridge_mock_publish():
    bridge = ROS2Bridge()
    bridge.start()
    bridge.publish(
        90.0,
        {"Legs": {"draw": 20}, "Arms": {"draw": 10}},
        allocation={"task": "moving", "status": "ok", "utilization_pct": 55},
        mission_info={"task": "moving", "task_label": "Moving"},
    )
    assert bridge.status["active"]
    assert bridge.status["mode"] == "mock"
    assert bridge._publish_count == 1
    last = bridge.last_published()
    assert last["main_battery"] == 90.0
    bridge.stop()


def test_ros2_bridge_mission_command():
    bridge = ROS2Bridge()
    bridge.inject_command(mission="high_load")
    assert bridge.consume_commanded_task() == "high_load"
    assert bridge.consume_commanded_task() is None


def test_ros2_bridge_rejects_invalid_mission():
    bridge = ROS2Bridge()
    bridge.inject_command(mission="fly_to_moon")
    assert bridge.consume_commanded_task() is None
    assert bridge.status["rejected_commands"] == 1


def test_ros2_bridge_rich_publish_payload():
    bridge = ROS2Bridge()
    bridge.start()
    bridge.publish(
        88.5,
        {"Legs": {"draw": 22}, "Arms": {"draw": 11}, "Torso": {"draw": 9}, "Compute": {"draw": 7}},
        allocation={"task": "moving", "status": "ok", "utilization_pct": 65, "budget_w": 72},
        mission_info={"task": "moving", "task_label": "Moving"},
    )
    last = bridge.last_published()
    assert last["total_draw_w"] == 49.0
    assert last["bridge_mode"] == "mock"
    assert "channel_order" in last
    assert bridge.status["last_payload_task"] == "moving"
    bridge.stop()


def test_ros2_bridge_sensor_blend():
    bridge = ROS2Bridge()
    bridge.inject_command(sensor_draws={"Legs": 30.0, "Arms": 15.0})
    draws = bridge.get_sensor_draws()
    assert draws["Legs"] == 30.0
    assert draws["Arms"] == 15.0


def test_mission_force_task():
    mgr = MissionTaskManager(start_task="idle")
    assert mgr.force_task("moving", duration_s=30)
    assert mgr.task_id == "moving"
    assert mgr.seconds_remaining == 30


def test_ros2_source_publishes_and_accepts_commands():
    reset_hardware_source()
    source = ROS2BatterySource()
    source.start()
    time.sleep(0.5)
    source._ros2.inject_command(mission="balanced")
    time.sleep(4)
    assert source.ros2_status.get("publish_count", 0) >= 1
    assert source.last_readings
    source.stop()
    reset_hardware_source()


def test_hardware_mode_switching():
    reset_hardware_source()
    from src.config import config

    original = dict(config._config.get("hardware", {}))
    config._config["hardware"] = {"mode": "simulator", "type": "generic"}
    src1 = get_hardware_source(force_reload=True)
    assert isinstance(src1, SimulatorSource)

    config._config["hardware"] = {"mode": "real", "type": "ros2", "ros2": {"mock": True}}
    src2 = get_hardware_source()
    assert isinstance(src2, ROS2BatterySource)
    assert src2 is not src1

    config._config["hardware"] = original
    reset_hardware_source()


def test_get_hardware_mode():
    mode = get_hardware_mode()
    assert "mode" in mode
    assert "type" in mode