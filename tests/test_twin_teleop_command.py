from src.hardware import reset_hardware_source
from src.hardware_ros2 import ROS2BatterySource
from src.twin import DigitalTwinBridge, reset_twin_bridge


def test_battery_reset_command():
    reset_twin_bridge()
    reset_hardware_source()
    bridge = DigitalTwinBridge()
    hw = ROS2BatterySource()
    hw._main_battery = 12.0
    result = bridge.apply_command(hw, {"battery_reset": True, "battery_pct": 100})
    assert result["ok"] is True
    assert "battery_reset=100%" in result["applied"][0]
    assert hw._main_battery == 100.0
    state = bridge.export_state(hw)
    assert state["teleop"]["battery_pct"] == 100.0


def test_drive_command_exported_to_teleop():
    reset_twin_bridge()
    reset_hardware_source()
    bridge = DigitalTwinBridge()
    hw = ROS2BatterySource()
    result = bridge.apply_command(
        hw,
        {"drive": {"left": 5.5, "right": 5.5, "duration_s": 2}, "source": "test"},
    )
    assert result["ok"] is True
    teleop = bridge.export_state(hw)["teleop"]
    assert teleop["active"] is True
    assert teleop["left_v"] == 5.5
    assert teleop["right_v"] == 5.5


def test_drive_stop_clears_teleop():
    reset_twin_bridge()
    bridge = DigitalTwinBridge()
    hw = ROS2BatterySource()
    bridge.apply_command(hw, {"drive": {"left": 4, "right": 4, "duration_s": 5}})
    bridge.apply_command(hw, {"drive_stop": True})
    teleop = bridge.export_state(hw)["teleop"]
    assert teleop["active"] is False
    assert teleop["left_v"] == 0.0