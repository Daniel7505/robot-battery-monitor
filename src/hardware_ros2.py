# src/hardware_ros2.py
"""
Enhanced ROS2 Battery Source with Power Management
"""

import threading
import time
import random
from src.hardware import RealHardwareSource
from src.config import config

class ROS2BatterySource(RealHardwareSource):
    def __init__(self):
        super().__init__()
        self.hardware_name = "ROS2 Battery Source"
        self._latest_data = {}
        self._peak_power = {}

    def start(self):
        super().start()
        print(f"[{self.hardware_name}] Enhanced simulation with power metrics started")
        threading.Thread(target=self._simulate_test_data, daemon=True).start()

    def _simulate_test_data(self):
        """Dynamic simulation with engineering metrics"""
        print(f"[{self.hardware_name}] Simulation running - monitoring power limits")
        
        while True:
            power_channels = config.get('power_channels') or []
            if not power_channels:
                power_channels = [
                    {"id": "Legs", "name": "Leg Drive Motors", "max_draw_w": 35, "nominal_voltage": 48},
                    {"id": "Arms", "name": "Arm + Gripper Systems", "max_draw_w": 25, "nominal_voltage": 48},
                    {"id": "Torso", "name": "Torso & Balance Systems", "max_draw_w": 20, "nominal_voltage": 48},
                    {"id": "Compute", "name": "Main Computer & Sensors", "max_draw_w": 15, "nominal_voltage": 24}
                ]

            self._latest_data = {}

            for ch in power_channels:
                ch_id = ch.get('id')
                max_w = ch.get('max_draw_w', 30)
                voltage = ch.get('nominal_voltage', 48)
                
                draw_w = round(random.uniform(max_w * 0.4, max_w * 0.95), 1)
                battery_pct = max(22, 88 + random.randint(-9, 5))
                
                amps = round(draw_w / voltage, 2) if voltage > 0 else 0.0
                
                status = "critical" if battery_pct < 25 else "warning" if draw_w > max_w * 0.9 else "normal"
                
                self._latest_data[ch_id] = {
                    "battery": battery_pct,
                    "draw": draw_w,
                    "amps": amps,
                    "max_draw_w": max_w,
                    "voltage": voltage,
                    "status": status
                }
                
                # Track peak power
                if ch_id not in self._peak_power or draw_w > self._peak_power[ch_id]:
                    self._peak_power[ch_id] = draw_w

            self.last_readings = self._latest_data.copy()
            print(f"[{self.hardware_name}] Updated {len(self._latest_data)} channels")
            time.sleep(3)

    def stop(self):
        super().stop()