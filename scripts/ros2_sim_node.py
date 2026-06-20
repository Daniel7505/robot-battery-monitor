#!/usr/bin/env python3
"""
Standalone ROS2 simulation node for Docker deployment.

Publishes mission commands and sensor power draws; subscribes to battery
telemetry topics for integration testing on the shared ROS2 DDS network.
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import config
from src.logger import logger

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Float32MultiArray, String
except ImportError:
    logger.error("rclpy not available — run this node inside the ros2-sim container")
    sys.exit(1)

_CHANNEL_ORDER = ["Legs", "Arms", "Torso", "Compute"]
_MISSION_CYCLE = ["idle", "moving", "high_load", "idle"]


class ROSSimNode(Node):
    def __init__(self):
        super().__init__("robot_battery_ros2_sim")
        ros_cfg = (config.get("hardware") or {}).get("ros2") or {}
        topics = {
            "main_battery": "/robot/battery/main_level",
            "power_draw": "/robot/battery/power_draw",
            "power_status": "/robot/battery/status",
            "mission_command": "/robot/battery/command/mission",
            "throttle_command": "/robot/battery/command/throttle",
            "sensor_power": "/robot/sensors/power_draw",
            **(ros_cfg.get("topics") or {}),
        }

        self._mission_pub = self.create_publisher(String, topics["mission_command"], 10)
        self._sensor_pub = self.create_publisher(Float32MultiArray, topics["sensor_power"], 10)
        self._throttle_pub = self.create_publisher(Float32, topics["throttle_command"], 10)

        self.create_subscription(Float32, topics["main_battery"], self._on_battery, 10)
        self.create_subscription(Float32MultiArray, topics["power_draw"], self._on_draw, 10)
        self.create_subscription(String, topics["power_status"], self._on_status, 10)

        self._mission_idx = 0
        self._tick = 0
        interval = float(os.getenv("ROS2_SIM_INTERVAL", "5"))
        self.create_timer(interval, self._simulate_traffic)
        self.get_logger().info(
            f"ROS2 sim active — publishing {topics['mission_command']}, "
            f"listening on {topics['main_battery']}"
        )

    def _on_battery(self, msg: Float32) -> None:
        self.get_logger().info(f"Telemetry: main battery {msg.data:.1f}%")

    def _on_draw(self, msg: Float32MultiArray) -> None:
        self.get_logger().info(f"Telemetry: channel draws {list(msg.data)}")

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            task = payload.get("task") or payload.get("ros2_status", {}).get("commanded_task")
            self.get_logger().info(f"Telemetry: status task={task}")
        except json.JSONDecodeError:
            self.get_logger().info(f"Telemetry: status {msg.data[:80]}")

    def _simulate_traffic(self) -> None:
        self._tick += 1
        if self._tick % 3 == 0:
            mission = _MISSION_CYCLE[self._mission_idx % len(_MISSION_CYCLE)]
            self._mission_idx += 1
            cmd = String()
            cmd.data = mission
            self._mission_pub.publish(cmd)
            self.get_logger().info(f"Sim command: mission → {mission}")

        sensor = Float32MultiArray()
        base = [12.0, 8.0, 10.0, 6.0]
        sensor.data = [round(v + (self._tick % 5), 1) for v in base]
        self._sensor_pub.publish(sensor)

        if self._tick % 6 == 0:
            throttle = Float32()
            throttle.data = 0.85
            self._throttle_pub.publish(throttle)
            self.get_logger().info("Sim command: throttle → 0.85")


def main() -> None:
    rclpy.init()
    node = ROSSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()