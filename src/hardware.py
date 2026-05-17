# src/hardware.py
"""
Hardware Abstraction Layer
Easy to switch between Simulator and Real hardware.
"""

from abc import ABC, abstractmethod
import threading
import time
import random
from datetime import datetime

from src.database import log_channel_reading


class HardwareSource(ABC):
    """Base class for any data source (simulator or real hardware)."""

    @abstractmethod
    def start(self):
        """Start generating or reading data."""
        pass


class SimulatorSource(HardwareSource):
    """Built-in realistic simulator."""

    def __init__(self):
        self.main_battery = 98.0
        self.channels = {
            "Legs": {"draw": 0, "name": "Leg Drive Motors", "max_draw": 35},
            "Arms": {"draw": 0, "name": "Arm + Gripper Systems", "max_draw": 25},
            "Torso": {"draw": 0, "name": "Torso & Balance Systems", "max_draw": 20},
            "Compute": {"draw": 0, "name": "Main Computer & Sensors", "max_draw": 15},
        }
        self.running = False

    def start(self):
        if self.running:
            return
        self.running = True
        print("🤖 SimulatorSource started")

        def _run():
            while self.running:
                total_draw = 0
                for ch_id, ch in self.channels.items():
                    base = random.uniform(3, ch["max_draw"])
                    spike = random.uniform(0, 15) if random.random() < 0.3 else 0
                    current_draw = round(base + spike)
                    ch["draw"] = current_draw
                    total_draw += current_draw

                    drain = total_draw / 25.0
                    self.main_battery = max(5.0, self.main_battery - drain * 0.08)

                    log_channel_reading(ch_id, int(self.main_battery), current_draw)

                print(f"   🔋 Main Battery: {int(self.main_battery)}% | Total Draw: {int(total_draw)}W")
                time.sleep(random.uniform(3, 6))

        threading.Thread(target=_run, daemon=True).start()


class RealHardwareSource(HardwareSource):
    """
    Placeholder for real robot hardware.

    When you set `hardware.mode: "real"` in config.yaml,
    this class will be used instead of the simulator.
    """

    def start(self):
        print("🔌 RealHardwareSource started (waiting for real hardware code)")

        # ============================================================
        # EXAMPLE 1: Serial / UART (very common for battery BMS)
        # ============================================================
        # Uncomment and adapt this when you have a real serial device
        """
        import serial

        def read_serial_data():
            try:
                ser = serial.Serial('COM3', 115200, timeout=1)   # Change port as needed
                while True:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        # Example: parse something like "Legs:23,Arms:15,Torso:9,Compute:12"
                        print(f"[Serial] {line}")
                        # TODO: Parse the line and call log_channel_reading(...)
                    time.sleep(0.2)
            except Exception as e:
                print(f"Serial error: {e}")

        threading.Thread(target=read_serial_data, daemon=True).start()
        """

        # ============================================================
        # EXAMPLE 2: ROS2 Subscription
        # ============================================================
        # Requires: pip install rclpy
        """
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray   # or your custom message

        class BatteryListener(Node):
            def __init__(self):
                super().__init__('battery_listener')
                self.subscription = self.create_subscription(
                    Float32MultiArray,
                    '/robot/battery_data',      # Change topic name as needed
                    self.listener_callback,
                    10)

            def listener_callback(self, msg):
                # Example: msg.data = [main_battery, legs_draw, arms_draw, torso_draw, compute_draw]
                print(f"[ROS2] Received: {msg.data}")
                # TODO: Parse and call log_channel_reading(...) for each channel

        def ros_spin():
            rclpy.init()
            node = BatteryListener()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()

        threading.Thread(target=ros_spin, daemon=True).start()
        """

        print("   → Add your real hardware code in the examples above (Serial, ROS2, CAN, etc.)")


def get_hardware_source():
    """Returns the correct hardware source based on config.yaml"""
    import yaml
    with open('config/config.yaml', 'r') as f:
        cfg = yaml.safe_load(f)

    mode = cfg.get('hardware', {}).get('mode', 'simulator')

    if mode == "simulator":
        return SimulatorSource()
    elif mode == "real":
        return RealHardwareSource()
    else:
        print(f"Unknown hardware mode '{mode}'. Falling back to simulator.")
        return SimulatorSource()