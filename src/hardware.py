# src/hardware.py
"""
Hardware Abstraction Layer (HAL) — factory + base sources for battery telemetry.

Role in the system
------------------
Decouples the dashboard from *how* power data is produced:

  mode=simulator  → SimulatorSource (random channel draws, simple DB logs)
  mode=real
    type=ros2     → ROS2BatterySource (full PMS: mission, allocator, safety, twin)
    type=generic  → RealHardwareSource (stub base for future sensor drivers)

Public API
----------
get_hardware_source()   Process-wide singleton; reloads if config mode/type changes
reset_hardware_source() Stop and clear singleton (tests / mode switch)
get_hardware_mode()     Current resolved {mode, type}

Invariants
----------
- Only one active HardwareSource per process.
- ``is_healthy`` requires a successful read within the last 30 seconds.
- ``validate_reading`` rejects battery outside [0,100] and draw outside [0,500] W.
"""

from abc import ABC, abstractmethod
import threading
import time
import random
from datetime import datetime, timedelta
import functools
import logging
from src.database import log_channel_reading
from src.config import config
from src.logger import logger

# Process-wide singleton; None until first get_hardware_source() call.
_hardware_instance = None
_resolved_mode: str | None = None

# ============================================================
# BASE CLASS
# ============================================================
class HardwareSource(ABC):
    """
    Abstract telemetry source.

    Subclasses own a background update path and populate ``last_readings``:
      {channel_id: {battery, draw, …, timestamp}}
    """

    def __init__(self):
        self.running = False
        self.last_readings = {}
        self.health_status = "STARTING"
        self.last_successful_read = None

    @abstractmethod
    def start(self):
        """Begin producing readings (usually starts a daemon thread)."""
        pass

    def stop(self):
        self.running = False
        logger.info(f"{self.__class__.__name__} stopping...")

    def is_healthy(self) -> bool:
        """True if a valid read occurred within the last 30s."""
        if not self.last_successful_read:
            return False
        return (datetime.now() - self.last_successful_read) < timedelta(seconds=30)

    def validate_reading(self, channel: str, battery: float, draw: float) -> bool:
        """Sanity-check before accepting hardware samples into the DB / UI."""
        if battery < 0 or battery > 100:
            return False
        # 500 W is far above ButlerBot peak; rejects unit/scale bugs.
        if draw < 0 or draw > 500:
            return False
        return True

# ============================================================
# SIMULATOR
# ============================================================
_DEFAULT_POWER_CHANNELS = [
    {"id": "Legs", "name": "Leg Drive Motors", "max_draw_w": 35, "nominal_voltage": 48},
    {"id": "Arms", "name": "Arm + Gripper Systems", "max_draw_w": 25, "nominal_voltage": 48},
    {"id": "Torso", "name": "Torso & Balance Systems", "max_draw_w": 20, "nominal_voltage": 48},
    {"id": "Compute", "name": "Main Computer & Sensors", "max_draw_w": 15, "nominal_voltage": 24},
]


class SimulatorSource(HardwareSource):
    """
    Lightweight random telemetry for demos without ROS2 or a twin.

    Not physics-accurate — for the scripted ButlerBot mission loop use
    ROS2BatterySource + SimulationDriver instead.
    """

    def __init__(self):
        super().__init__()
        self.power_channels = config.get("power_channels") or _DEFAULT_POWER_CHANNELS

    def _generate_readings(self) -> dict:
        readings = {}
        for ch in self.power_channels:
            ch_id = ch.get("id")
            max_w = ch.get("max_draw_w", 30)
            voltage = ch.get("nominal_voltage", 48)

            # Draw in mid-high band of channel capability for a "busy robot" feel.
            draw_w = round(random.uniform(max_w * 0.4, max_w * 0.95), 1)
            # Floor at 22% so the simple sim never shows a fully dead pack.
            battery_pct = max(22, 88 + random.randint(-9, 5))
            amps = round(draw_w / voltage, 2) if voltage > 0 else 0.0
            status = (
                "critical" if battery_pct < 25
                else "warning" if draw_w > max_w * 0.9
                else "normal"
            )

            readings[ch_id] = {
                "battery": battery_pct,
                "draw": draw_w,
                "amps": amps,
                "status": status,
                "timestamp": datetime.now(),
            }
            log_channel_reading(ch_id, int(battery_pct), int(draw_w))
        return readings

    def start(self):
        if self.running:
            return
        self.running = True
        self.health_status = "RUNNING"
        logger.info("SimulatorSource started")

        self.last_readings = self._generate_readings()
        self.last_successful_read = datetime.now()

        def _run():
            while self.running:
                self.last_readings = self._generate_readings()
                self.last_successful_read = datetime.now()
                time.sleep(4)

        threading.Thread(target=_run, daemon=True).start()

# ============================================================
# REAL HARDWARE BASE
# ============================================================
class RealHardwareSource(HardwareSource):
    """
    Base for live hardware integrations.

    Subclasses parse sensor/ROS data and call ``_process_parsed_data`` to
    validate, log, and update ``last_readings``.
    """

    def __init__(self):
        super().__init__()
        self.hardware_name = "Generic Real Hardware"

    def start(self):
        if self.running:
            return
        self.running = True
        self.health_status = "RUNNING"
        logger.info(f"🔌 {self.hardware_name} started")
        # Child classes start their own acquisition threads after super().start().

    def _process_parsed_data(self, parsed_data: dict):
        """Validate + persist + store each channel from a parsed sensor frame."""
        for channel, values in parsed_data.items():
            battery = values.get("battery", 0)
            draw = values.get("draw", 0)
            if self.validate_reading(channel, battery, draw):
                log_channel_reading(channel, int(battery), draw)
                self.last_readings[channel] = {
                    "battery": battery,
                    "draw": draw,
                    "timestamp": datetime.now()
                }
                self.last_successful_read = datetime.now()

# ============================================================
# FACTORY (Singleton)
# ============================================================
def _resolve_mode() -> tuple[str, str]:
    """Read hardware.mode and hardware.type from config (normalized lowercase)."""
    mode = config.get("hardware", "mode", "simulator")
    if isinstance(mode, str):
        mode = mode.lower()
    else:
        mode = "simulator"
    hw_type = config.get("hardware", "type", "generic")
    if isinstance(hw_type, str):
        hw_type = hw_type.lower()
    else:
        hw_type = "generic"
    return mode, hw_type


def _create_hardware_source(mode: str, hw_type: str) -> HardwareSource:
    if mode == "real":
        if hw_type == "ros2":
            # Lazy import: ROS2BatterySource pulls allocator, twin, agent, etc.
            from src.hardware_ros2 import ROS2BatterySource
            logger.info("Using ROS2 hardware mode")
            return ROS2BatterySource()
        logger.info("Using REAL hardware mode (generic)")
        return RealHardwareSource()
    logger.info("Using SIMULATOR mode")
    return SimulatorSource()


def reset_hardware_source() -> None:
    """Stop and clear the singleton — enables mode switching and clean tests."""
    global _hardware_instance, _resolved_mode
    if _hardware_instance is not None:
        try:
            _hardware_instance.stop()
        except Exception as e:
            logger.warning(f"Error stopping hardware source: {e}")
    _hardware_instance = None
    _resolved_mode = None


def get_hardware_mode() -> dict:
    """Return the currently configured {mode, type} without creating a source."""
    mode, hw_type = _resolve_mode()
    return {"mode": mode, "type": hw_type}


def get_hardware_source(force_reload: bool = False):
    """
    Return the process singleton hardware source.

    Recreates the instance when ``force_reload`` is set or when config mode:type
    differs from the instance that is already running.
    """
    global _hardware_instance, _resolved_mode
    mode, hw_type = _resolve_mode()
    mode_key = f"{mode}:{hw_type}"

    if force_reload or (_hardware_instance and _resolved_mode != mode_key):
        logger.info(f"Hardware mode change detected ({_resolved_mode} → {mode_key})")
        reset_hardware_source()

    if _hardware_instance is None:
        _hardware_instance = _create_hardware_source(mode, hw_type)
        _resolved_mode = mode_key
    return _hardware_instance
