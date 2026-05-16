import pytest
import os
from src.database import init_db, log_channel_reading, get_all_readings, get_channel_history
from src.simulator import channels  # for testing channel names

def test_database_creates_table():
    """Test that database initializes without errors"""
    init_db()
    assert os.path.exists("logs/robot_battery.db")

def test_log_channel_reading():
    """Test we can log data"""
    init_db()  # fresh start
    log_channel_reading("Legs", 87, 22)
    entries = get_all_readings(limit=5)
    assert len(entries) >= 1
    assert any(e["channel"] == "Legs" for e in entries)

def test_get_channel_history():
    """Test history retrieval works"""
    init_db()
    log_channel_reading("Arms", 65, 18)
    history = get_channel_history("Arms", limit=10)
    assert len(history) >= 1

def test_config_file_exists():
    """Test config file is present"""
    assert os.path.exists("config/config.yaml")

def test_channels_defined():
    """Test we have expected power channels"""
    assert "Legs" in channels
    assert "Arms" in channels
    assert len(channels) >= 3

def test_battery_values_in_range():
    """Test battery values stay realistic"""
    entries = get_all_readings(limit=20)
    for e in entries:
        assert 0 <= e["battery"] <= 100