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

logger = logging.getLogger(__name__)


def retry_on_failure(max_attempts=5, delay=1.0, backoff=2.0):
    """Decorator for hardware operations that can fail temporarily."""
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            attempts = 0
            while attempts < max_attempts:
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    attempts += 1
                    wait = delay * (backoff ** (attempts - 1))
                    logger.warning(f"{func.__name__} failed (attempt {attempts}/{max_attempts}): {e}. Retrying in {wait:.1f}s...")
                    time.sleep(wait)
            logger.error(f"{func.__name__} failed after {max_attempts} attempts.")
            raise
        return wrapper
    return decorator


class HardwareSource(ABC):
    """Base class for any data source."""

    def __init__(self):
        self.running = False
        self.last_readings = {}          # For sanity checks and fallback
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
        """Sanity check on incoming data."""
        if not (0 <= battery_pct <= 100):
            logger.warning(f"Invalid battery percentage: {battery_pct}%")
            return False
        if current_draw < 0 or current_draw > 200:   # adjust per your robot
            logger.warning(f"Suspicious current draw on {channel}: {current_draw}W")
            return False

        # Check for unrealistic jumps
        if channel in self.last_readings:
            last_bat = self.last_readings[channel].get('battery', battery_pct)
            if abs(last_bat - battery_pct) > 15:   # max 15% jump per reading
                logger.warning(f"Large battery jump on {channel}: {last_bat}% → {battery_pct}%")
                return False
        return True


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


class RealHardwareSource(HardwareSource):
    """
    Real hardware implementation.
    Add your actual sensors / BMS / ROS2 topics here.
    """

    def start(self):
        if self.running:
            return
        self.running = True
        self.health_status = "RUNNING"
        logger.info("🔌 RealHardwareSource started")

        # TODO: Put your real hardware reading code here
        # Example placeholders are commented below

        def _real_read_loop():
            while self.running:
                try:
                    # === YOUR REAL HARDWARE CODE GOES HERE ===
                    # Example: read from INA219, serial BMS, ROS2, etc.
                    # For now we simulate a small healthy signal
                    data = {
                        "Legs": (self.last_readings.get("Legs", {}).get("battery", 85.0), 12),
                        "Arms": (self.last_readings.get("Arms", {}).get("battery", 85.0), 8),
                        "Torso": (self.last_readings.get("Torso", {}).get("battery", 85.0), 15),
                        "Compute": (self.last_readings.get("Compute", {}).get("battery", 85.0), 22),
                    }

                    for ch_id, (bat, draw) in data.items():
                        if self.validate_reading(ch_id, bat, draw):
                            log_channel_reading(ch_id, int(bat), draw)
                            self.last_readings[ch_id] = {
                                'battery': bat, 'draw': draw, 'timestamp': datetime.now()
                            }
                            self.last_successful_read = datetime.now()

                    time.sleep(2.0)

                except Exception as e:
                    logger.error(f"Real hardware read error: {e}")
                    self.health_status = "DEGRADED"
                    time.sleep(5.0)

        threading.Thread(target=_real_read_loop, daemon=True, name="RealHardwareThread").start()

    def stop(self):
        super().stop()
        # Add any serial port closing, ROS2 shutdown, etc. here


def get_hardware_source():
    """Returns the correct hardware source using centralized config."""
    from src.config import config
    from src.logger import logger

    mode = config.get("hardware", "mode", "simulator").lower()

    if mode == "real":
        logger.info("Using REAL hardware mode")
        return RealHardwareSource()
    else:
        logger.info("Using SIMULATOR mode")
        return SimulatorSource()