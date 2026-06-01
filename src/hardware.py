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

# Global singleton instance
_hardware_instance = None

# ============================================================
# BASE CLASS
# ============================================================
class HardwareSource(ABC):
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

# ============================================================
# SIMULATOR
# ============================================================
class SimulatorSource(HardwareSource):
    # ... (keep your existing SimulatorSource code) ...
    def start(self):
        if self.running:
            return
        self.running = True
        self.health_status = "RUNNING"
        logger.info("SimulatorSource started")
        def _run():
            while self.running:
                total_draw = 0
                for ch_id, ch in getattr(self, 'channels', {}).items():
                    base = random.uniform(3, ch.get("max_draw", 20))
                    current_draw = max(0, round(base))
                    self.last_readings[ch_id] = {
                        'battery': 85,
                        'draw': current_draw,
                        'timestamp': datetime.now()
                    }
                    total_draw += current_draw
                time.sleep(4)
        threading.Thread(target=_run, daemon=True).start()

# ============================================================
# REAL HARDWARE BASE
# ============================================================
class RealHardwareSource(HardwareSource):
    def __init__(self):
        super().__init__()
        self.hardware_name = "Generic Real Hardware"

    def start(self):
        if self.running:
            return
        self.running = True
        self.health_status = "RUNNING"
        logger.info(f"🔌 {self.hardware_name} started")
        # The child class will start its own thread

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

# ============================================================
# FACTORY (Singleton)
# ============================================================
def get_hardware_source():
    """Always return the same instance"""
    global _hardware_instance
    if _hardware_instance is None:
        mode = config.get("hardware", "mode", "simulator").lower()
        if mode == "real":
            hardware_type = config.get("hardware", "type", "generic")
            if hardware_type == "ros2":
                from src.hardware_ros2 import ROS2BatterySource
                logger.info("Using ROS2 hardware mode")
                _hardware_instance = ROS2BatterySource()
            else:
                _hardware_instance = RealHardwareSource()
        else:
            logger.info("Using SIMULATOR mode")
            _hardware_instance = SimulatorSource()
    return _hardware_instance