# src/hardware_ros2.py
"""
ROS2 Hardware Source Example
This file shows how to connect real ROS2 data into the battery monitor.

This is meant to be a STARTING POINT — not a complete working solution.
You will likely need to adjust it based on your actual ROS2 messages.
"""

from src.hardware import RealHardwareSource
import threading
import time

# Uncomment these when you're ready to use real ROS2:
# import rclpy
# from rclpy.node import Node


class ROS2BatterySource(RealHardwareSource):
    """
    Real hardware implementation that reads battery data from ROS2.

    HOW TO USE:
    -----------
    1. Set hardware.mode = "real" in config/config.yaml
    2. Update get_hardware_source() in src/hardware.py to return this class
    3. Implement the ROS2 subscriber logic below

    This class inherits from RealHardwareSource so it reuses
    validation, logging, and health checking automatically.
    """

    def __init__(self):
        super().__init__()
        self.hardware_name = "ROS2 Battery Source"
        self._latest_data = {}           # Stores the latest battery data from ROS2

    def _read_raw_data(self):
        """Return the latest data we received from ROS2."""
        return self._latest_data if self._latest_data else None

    def _parse_data(self, raw_data):
        """
        Convert the raw ROS2 data into the format our system expects.
        You will need to change this to match your actual ROS2 message.
        """
        # Example format we expect:
        # {
        #     "Legs":   {"battery": 85, "draw": 12},
        #     "Arms":   {"battery": 82, "draw": 7},
        #     ...
        # }
        return raw_data

    def start(self):
        super().start()
        # Start ROS2 in a background thread (basic example)
        threading.Thread(target=self._ros2_spin, daemon=True).start()
        print(f"[{self.hardware_name}] ROS2 thread started")

    def _ros2_spin(self):
        """
        This is where you would normally initialize ROS2 and create subscribers.
        This is a simplified placeholder.
        """
        print(f"[{self.hardware_name}] ROS2 spinning would happen here...")
        # Example of what real code might look like:
        # rclpy.init()
        # node = rclpy.create_node('battery_monitor')
        # node.create_subscription(...)
        # rclpy.spin(node)