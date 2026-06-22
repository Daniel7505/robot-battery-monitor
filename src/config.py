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
                with open(config_path, 'r', encoding='utf-8') as f:
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
            "HARDWARE_TYPE": ("hardware", "type"),
            "DASHBOARD_PORT": ("dashboard", "port"),
            "LOG_LEVEL": ("monitoring", "log_level"),
        }
        database_url = os.getenv("DATABASE_URL")
        if database_url:
            if "database" not in self._config:
                self._config["database"] = {}
            self._config["database"]["url"] = database_url
            logger.info("Overrode database with env var: DATABASE_URL")
        for env_key, (section, key) in env_map.items():
            value = os.getenv(env_key)
            if value is not None:
                if section not in self._config:
                    self._config[section] = {}
                if key == "port":
                    value = int(value)
                self._config[section][key] = value
                logger.info(f"Overrode with env var: {env_key}={value}")

        ros2_mock = os.getenv("ROS2_MOCK")
        if ros2_mock is not None:
            if "hardware" not in self._config:
                self._config["hardware"] = {}
            if "ros2" not in self._config["hardware"]:
                self._config["hardware"]["ros2"] = {}
            self._config["hardware"]["ros2"]["mock"] = ros2_mock.lower() in (
                "1", "true", "yes", "on"
            )
            logger.info(f"Overrode with env var: ROS2_MOCK={ros2_mock}")

        sim_enabled = os.getenv("SIMULATION_ENABLED")
        if sim_enabled is not None:
            if "simulation" not in self._config:
                self._config["simulation"] = {}
            self._config["simulation"]["enabled"] = sim_enabled.lower() in (
                "1", "true", "yes", "on"
            )
        sim_loop = os.getenv("SIMULATION_LOOP")
        if sim_loop is not None:
            if "simulation" not in self._config:
                self._config["simulation"] = {}
            self._config["simulation"]["loop"] = sim_loop.lower() in (
                "1", "true", "yes", "on"
            )
        sim_auto = os.getenv("SIMULATION_AUTO_START")
        if sim_auto is not None:
            if "simulation" not in self._config:
                self._config["simulation"] = {}
            self._config["simulation"]["auto_start"] = sim_auto.lower() in (
                "1", "true", "yes", "on"
            )

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