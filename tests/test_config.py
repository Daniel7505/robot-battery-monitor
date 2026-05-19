import os
import pytest
from src.config import Config


def test_config_loads_yaml():
    """Config should load from config.yaml without crashing"""
    cfg = Config()
    assert cfg.get("robot", "name") is not None
    assert cfg.get("dashboard", "port") is not None


def test_config_get_whole_section():
    """Should return whole section when only section is passed"""
    cfg = Config()
    channels = cfg.get("power_channels")
    assert isinstance(channels, list)
    assert len(channels) >= 3


def test_config_get_with_default():
    """Should return default when key doesn't exist"""
    cfg = Config()
    value = cfg.get("monitoring", "nonexistent_key", default=42)
    assert value == 42


def test_config_env_override(monkeypatch):
    """Should allow overriding via environment variables"""
    monkeypatch.setenv("DASHBOARD_PORT", "8080")
    monkeypatch.setenv("HARDWARE_MODE", "real")

    cfg = Config()
    # Note: load() is called in __init__, so we re-load manually for test
    cfg.load()

    assert cfg.get("dashboard", "port") == 8080
    assert cfg.get("hardware", "mode") == "real"


def test_config_power_channels_structure():
    """Power channels should have id and name"""
    cfg = Config()
    channels = cfg.get("power_channels") or []
    for ch in channels:
        assert "id" in ch
        assert "name" in ch