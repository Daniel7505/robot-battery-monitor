# src/hardware.py
"""
Hardware Abstraction Layer - Production Ready
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


# ============================================================
# BASE CLASS
# ============================================================
class HardwareSource(ABC):
    """Base class for any data source (simulator or real hardware)."""

    def __init__(self):
        self.running = False
        self.last_readings = {}
        self.health_status = "STARTING"
        self.last_successful_read = None

    @abstractmethod
    def start(self):
        pass

    def stop(self):
        self.running = False
        logger.info(f"{self.__class__.__name__} stopping...")

    def is_healthy(self) -> bool:
        if not self.last_successful_read:
            return False
        return (datetime.now() - self.last_successful_read) < timedelta(seconds=30)

    def validate_reading(self, channel: str, battery_pct: float, current_draw: int) -> bool:
        if not (0 <= battery_pct <= 100):
            logger.warning(f"Invalid battery percentage: {battery_pct}%")
            return False
        if current_draw < 0 or current_draw > 200:
            logger.warning(f"Suspicious current draw on {channel}: {current_draw}W")
            return False

        if channel in self.last_readings:
            last_bat = self.last_readings[channel].get('battery', battery_pct)
            if abs(last_bat - battery_pct) > 15:
                logger.warning(f"Large battery jump on {channel}: {last_bat}% → {battery_pct}%")
                return False
        return True


# ============================================================
# SIMULATOR
# ============================================================
class SimulatorSource(HardwareSource):
    """Realistic simulator with health reporting."""

    def __init__(self):
        super().__init__()
        self.main_battery = 98.0
        self.channels = {
            "Legs": {"draw": 0, "name": "Leg Drive Motors", "max_draw": 35},
            "Arms": {"draw": 0, "name": "Arm + Gripper Systems", "max_draw": 25},
            "Torso": {"draw": 0, "name": "Torso & Balance Systems", "max_draw": 20},
            "Compute": {"draw": 0, "name": "Main Computer & Sensors", "max_draw": 15},
        }

    def start(self):
        if self.running:
            return
        self.running = True
        self.health_status = "RUNNING"
        logger.info("🤖 SimulatorSource started")

        def _run():
            while self.running:
                total_draw = 0
                for ch_id, ch in self.channels.items():
                    base = random.uniform(3, ch["max_draw"])
                    spike = random.uniform(0, 15) if random.random() < 0.3 else 0
                    current_draw = max(0, round(base + spike))

                    drain = total_draw / 25.0
                    self.main_battery = max(5.0, self.main_battery - drain * 0.08)

                    if self.validate_reading(ch_id, self.main_battery, current_draw):
                        log_channel_reading(ch_id, int(self.main_battery), current_draw)
                        self.last_readings[ch_id] = {
                            'battery': self.main_battery,
                            'draw': current_draw,
                            'timestamp': datetime.now()
                        }
                        self.last_successful_read = datetime.now()

                    total_draw += current_draw

                logger.debug(f"Main Battery: {int(self.main_battery)}% | Total Draw: {total_draw}W")
                time.sleep(random.uniform(3, 6))

        threading.Thread(target=_run, daemon=True, name="SimulatorThread").start()


# ============================================================
# REAL HARDWARE BASE CLASS
# ============================================================
class RealHardwareSource(HardwareSource):
    """
    Base class for connecting real robot hardware.

    HOW TO USE:
    -----------
    1. Create a new class that inherits from RealHardwareSource
    2. Override these two methods:
       - _read_raw_data()   → Get data from your hardware (ROS2, serial, CAN, etc.)
       - _parse_data()      → Convert that data into our standard format

    EXAMPLES:
    ---------
    - ROS2BatterySource     (for ROS2 topics)
    - SerialBMSHardware     (for JK-BMS, Daly, etc via serial/UART)
    - CANBatteryHardware    (for CAN bus battery data)

    This design makes it easy to support many different hardware types
    without changing the core monitoring system.
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

        def _read_loop():
            while self.running:
                try:
                    raw_data = self._read_raw_data()
                    if raw_data:
                        parsed = self._parse_data(raw_data)
                        self._process_parsed_data(parsed)
                    time.sleep(1.0)
                except Exception as e:
                    logger.error(f"[{self.hardware_name}] Read error: {e}")
                    self.health_status = "DEGRADED"
                    time.sleep(3.0)

        threading.Thread(target=_read_loop, daemon=True, name=f"{self.hardware_name}Thread").start()

    def _read_raw_data(self):
        """Override this. Return raw data from your hardware."""
        return None

    def _parse_data(self, raw_data):
        """Override this. Convert raw data into standard dict format."""
        return {}

    def _process_parsed_data(self, parsed_data: dict):
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

    def stop(self):
        super().stop()
        logger.info(f"🔌 {self.hardware_name} stopped")


# ============================================================
# FACTORY
# ============================================================
def get_hardware_source():
    """Returns the correct hardware source based on config."""
    mode = config.get("hardware", "mode", "simulator").lower()

    if mode == "real":
        # You can expand this later to support different real hardware types
        hardware_type = config.get("hardware", "type", "generic")

        if hardware_type == "ros2":
            from src.hardware_ros2 import ROS2BatterySource
            logger.info("Using ROS2 hardware mode")
            return ROS2BatterySource()
        else:
            logger.info("Using REAL hardware mode (generic)")
            return RealHardwareSource()
    else:
        logger.info("Using SIMULATOR mode")
        return SimulatorSource()