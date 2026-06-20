import pytest
from src.hardware import get_hardware_source, SimulatorSource, RealHardwareSource, HardwareSource
from src.config import config


def test_get_hardware_source_returns_simulator_by_default():
    from src.hardware import reset_hardware_source
    original_hw = dict(config._config.get("hardware", {}))
    config._config["hardware"] = {"mode": "simulator", "type": "generic"}
    reset_hardware_source()
    source = get_hardware_source(force_reload=True)
    assert isinstance(source, SimulatorSource)
    config._config["hardware"] = original_hw
    reset_hardware_source()


def test_validate_reading_works():
    """Should accept/reject readings correctly"""
    # Use SimulatorSource which inherits the validation method
    source = SimulatorSource()
    
    assert source.validate_reading("Legs", 85, 12) is True
    assert source.validate_reading("Legs", -5, 10) is False
    assert source.validate_reading("Legs", 80, -10) is False
    assert source.validate_reading("Legs", 80, 999) is False


def test_simulator_can_start():
    source = SimulatorSource()
    source.start()
    assert source.running is True
    source.stop()


def test_ros2_source_can_be_instantiated():
    from src.hardware_ros2 import ROS2BatterySource
    source = ROS2BatterySource()
    assert isinstance(source, HardwareSource)


print("✅ Basic hardware tests loaded successfully")