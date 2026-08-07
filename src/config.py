# src/config.py
"""
Centralized configuration loader for the Robot Battery Monitor.

Role in the system
------------------
Single source of runtime settings. Modules import the module-level ``config``
singleton rather than opening YAML themselves.

Load order (later wins)
-----------------------
1. ``config/config.yaml`` if present, else hard-coded defaults
2. Selected environment variables (Docker / CI / demo scripts)

Key sections (typical)
----------------------
dashboard, monitoring, robot, power / power_channels, hardware (mode/type/ros2),
simulation, safety, requirements, lru, agent, digital_twin, database

Invariants
----------
- ``get(section)`` returns the whole section (dict or list) when ``key`` is None.
- ``get(section, key, default)`` digs into a dict section; non-dict sections
  yield ``default``.
- Env overrides are applied once at construction; call ``config.load()`` again
  only if you intentionally hot-reload (tests).
"""

import os
import yaml
import logging

logger = logging.getLogger(__name__)


class Config:
    """YAML + env configuration with flexible section/key access."""

    def __init__(self):
        self._config = {}
        self.load()

    def load(self):
        """Load YAML (or defaults), then apply env overrides."""
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
            # Never leave the process without a usable config dict.
            self._config = self._get_defaults()

    def _get_defaults(self):
        """Minimal bootstrap when YAML is missing or unreadable."""
        return {
            "dashboard": {"host": "127.0.0.1", "port": 5000},
            "monitoring": {"low_battery_threshold": 20, "log_level": "INFO"},
            "robot": {"name": "Robot"},
            "power_channels": [],
            "hardware": {"mode": "simulator"}
        }

    def _override_with_env(self):
        """
        Apply deployment-oriented env vars.

        Boolean flags accept 1/true/yes/on (case-insensitive). DATABASE_URL is
        the standard 12-factor connection string for Docker Compose Postgres.
        """
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

        # Simulation flags let scripts toggle the ButlerBot mission loop without
        # editing YAML (used heavily by demo / Docker entrypoints).
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

        agent_enabled = os.getenv("AGENT_ENABLED")
        if agent_enabled is not None:
            if "agent" not in self._config:
                self._config["agent"] = {}
            self._config["agent"]["enabled"] = agent_enabled.lower() in (
                "1", "true", "yes", "on"
            )
            logger.info(f"Overrode with env var: AGENT_ENABLED={agent_enabled}")

    def get(self, section: str, key: str = None, default=None):
        """
        Flexible getter.
        - config.get('robot', 'name')           → value or default
        - config.get('power_channels')          → whole list/dict section
        - config.get('power_channels', default=[]) → section or default
        """
        if key is None:
            return self._config.get(section, default)

        section_data = self._config.get(section, {})
        if isinstance(section_data, dict):
            return section_data.get(key, default)
        return default

    def get_all(self):
        """Return the full config dict (mutable — treat as read-only)."""
        return self._config


# Process-wide singleton; imported by nearly every module at import time.
config = Config()
