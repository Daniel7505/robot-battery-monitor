# src/config.py
"""
Centralized configuration with env var support.
"""

import os
import yaml
import logging

logger = logging.getLogger(__name__)


class Config:
    def __init__(self):
        self._config = {}
        self.load()

    def load(self):
        try:
            config_path = 'config/config.yaml'
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    self._config = yaml.safe_load(f) or {}
                logger.info("✅ Loaded config.yaml")
            else:
                logger.warning("config.yaml not found — using defaults")
                self._config = self._get_defaults()

            self._override_with_env()
            logger.info("✅ Configuration loaded and validated")

        except Exception as e:
            logger.error(f"Config loading failed: {e}")
            self._config = self._get_defaults()

    def _get_defaults(self):
        return {
            "dashboard": {"host": "127.0.0.1", "port": 5000},
            "monitoring": {"low_battery_threshold": 20, "log_level": "INFO"},
            "robot": {"name": "Robot"},
            "power_channels": [],
            "hardware": {"mode": "simulator"}
        }

    def _override_with_env(self):
        env_map = {
            "HARDWARE_MODE": ("hardware", "mode"),
            "DASHBOARD_PORT": ("dashboard", "port"),
            "LOG_LEVEL": ("monitoring", "log_level"),
        }
        for env_key, (section, key) in env_map.items():
            value = os.getenv(env_key)
            if value is not None:
                if section not in self._config:
                    self._config[section] = {}
                if key == "port":
                    value = int(value)
                self._config[section][key] = value
                logger.info(f"Overrode with env var: {env_key}={value}")

    def get(self, section: str, key: str = None, default=None):
        """
        Flexible getter.
        - config.get('robot', 'name')           → returns value or default
        - config.get('power_channels')          → returns the whole list
        - config.get('power_channels', default=[]) → returns list or default
        """
        if key is None:
            # Return whole section (can be dict or list)
            return self._config.get(section, default)

        section_data = self._config.get(section, {})
        if isinstance(section_data, dict):
            return section_data.get(key, default)
        return default

    def get_all(self):
        return self._config


# Global instance
config = Config()