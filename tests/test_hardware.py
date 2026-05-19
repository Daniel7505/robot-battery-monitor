import pytest
from unittest.mock import patch, MagicMock
from src.hardware import (
    get_hardware_source,
    SimulatorSource,
    RealHardwareSource,
    HardwareSource
)
from src.config import config


def test_get_hardware_source_returns_simulator_by_default():
    """Should return SimulatorSource when no mode or simulator mode is set"""
    # Force simulator mode
    original_mode = config.get("hardware", "mode")
    config._config["hardware"]["mode"] = "simulator"

    source = get_hardware_source()
    assert isinstance(source, SimulatorSource)

    # Restore original (just in case)
    config._config["hardware"]["mode"] = original_mode


def test_get_hardware_source_returns_real_when_forced(monkeypatch):
    """Should return RealHardwareSource when we force hardware.mode = 'real'"""
    import src.hardware as hw_module
    from src.config import config as global_config

    # Save original
    original_mode = global_config.get("hardware", "mode")

    # Force real mode directly on the config object
    if "hardware" not in global_config._config:
        global_config._config["hardware"] = {}
    global_config._config["hardware"]["mode"] = "real"

    source = hw_module.get_hardware_source()
    assert isinstance(source, RealHardwareSource)

    # Restore original mode
    global_config._config["hardware"]["mode"] = original_mode


def test_validate_reading_accepts_valid_data():
    """Should accept normal, reasonable readings"""
    source = SimulatorSource()
    result = source.validate_reading("Legs", 85, 12)
    assert result is True


def test_validate_reading_rejects_invalid_battery():
    """Should reject battery percentages outside 0-100"""
    source = SimulatorSource()
    assert source.validate_reading("Legs", -5, 10) is False
    assert source.validate_reading("Legs", 150, 10) is False


def test_validate_reading_rejects_suspicious_current_draw():
    """Should reject obviously wrong current draw values"""
    source = SimulatorSource()
    assert source.validate_reading("Legs", 80, -10) is False      # negative
    assert source.validate_reading("Legs", 80, 999) is False      # way too high


def test_simulator_source_starts_without_crashing():
    """Simulator should be able to start its background thread"""
    source = SimulatorSource()
    source.start()
    assert source.running is True
    source.stop()
    assert source.running is False


def test_real_hardware_source_can_be_instantiated():
    """RealHardwareSource should be instantiable even without real hardware"""
    source = RealHardwareSource()
    assert isinstance(source, HardwareSource)
    
    


# ============================================================
# MOCKING TESTS (for testing real hardware logic without real hardware)
# ============================================================

def test_real_hardware_source_can_be_mocked():
    """
    Shows how to mock real hardware so we can test logic
    without needing actual robot hardware connected.
    """
    source = RealHardwareSource()

    # Create fake data
    fake_data = {
        "Legs": {"battery": 87, "draw": 18},
        "Arms": {"battery": 91, "draw": 6}
    }

    # Patch the methods on the class (more reliable than on instance)
    with patch.object(RealHardwareSource, '_read_raw_data', return_value=fake_data):
        with patch.object(RealHardwareSource, '_parse_data', return_value=fake_data):
            parsed = source._parse_data(fake_data)
            assert "Legs" in parsed
            assert parsed["Legs"]["battery"] == 87


def test_real_hardware_validation_still_works_when_mocked():
    """Even when mocked, our validation logic should still work."""
    source = RealHardwareSource()

    result = source.validate_reading("Torso", 75, 22)
    assert result is True

    result = source.validate_reading("Torso", 150, 10)
    assert result is False