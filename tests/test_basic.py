import pytest
import os
from src.database import init_db, log_channel_reading, get_all_readings, get_channel_history
from src.config import config


def test_database_creates_table(clean_database):
    """Test that database initializes without errors"""
    init_db()
    assert True


def test_log_channel_reading(clean_database):
    """Test we can log data"""
    log_channel_reading("Legs", 87, 22)
    entries = get_all_readings(limit=5)
    assert len(entries) >= 1
    assert any(e["channel"] == "Legs" for e in entries)


def test_get_channel_history(clean_database):
    """Test history retrieval works"""
    log_channel_reading("Arms", 65, 18)
    history = get_channel_history("Arms", limit=10)
    assert len(history) >= 1


def test_config_file_exists():
    """Test config file is present"""
    assert os.path.exists("config/config.yaml")


def test_config_loads_power_channels():
    """Test that config properly loads our power channels"""
    channels = config.get('power_channels') or []
    assert len(channels) >= 3

    channel_ids = [ch.get('id') for ch in channels]
    assert "Legs" in channel_ids
    assert "Arms" in channel_ids


def test_battery_values_in_range(clean_database):
    """Test battery values stay realistic"""
    log_channel_reading("Legs", 85, 10)
    entries = get_all_readings(limit=20)
    for e in entries:
        assert 0 <= e["battery"] <= 100