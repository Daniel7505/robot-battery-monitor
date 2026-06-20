#!/usr/bin/env python3
"""
Standalone ROS2 publisher — reads live hardware telemetry and publishes to topics.
Run separately: python -m src.ros2_node
"""

import json
import sys
import time

from src.logger import logger

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Float32MultiArray, String
    _RCLPY_AVAILABLE = True
except ImportError:
    _RCLPY_AVAILABLE = False

_CHANNEL_ORDER = ["Legs", "Arms", "Torso", "Compute"]


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__("robot_battery_publisher")
        from src.config import config

        topics = (config.get("hardware", "ros2") or {}).get("topics") or {}
        self._main_topic = topics.get("main_battery", "/robot/battery/main_level")
        self._draw_topic = topics.get("power_draw", "/robot/battery/power_draw")
        self._status_topic = topics.get("power_status", "/robot/battery/status")

        self._main_pub = self.create_publisher(Float32, self._main_topic, 10)
        self._draw_pub = self.create_publisher(Float32MultiArray, self._draw_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self.create_timer(2.0, self.publish_battery_data)

        self.get_logger().info(
            f"ROS2 publisher started — {self._main_topic}, {self._draw_topic}, {self._status_topic}"
        )

    def publish_battery_data(self):
        from src.hardware import get_hardware_source

        try:
            hardware = get_hardware_source()
            readings = getattr(hardware, "last_readings", {}) or {}
            if not readings:
                self.get_logger().warn("No live readings yet — is the dashboard running?")
                return

            batteries = [d.get("battery", 0) for d in readings.values()]
            main_battery = sum(batteries) / len(batteries) if batteries else 0

            bat_msg = Float32()
            bat_msg.data = float(main_battery)
            self._main_pub.publish(bat_msg)

            draw_msg = Float32MultiArray()
            draw_msg.data = [float(readings.get(ch, {}).get("draw", 0)) for ch in _CHANNEL_ORDER]
            self._draw_pub.publish(draw_msg)

            status = {
                "main_battery": main_battery,
                "channel_draws": {ch: readings.get(ch, {}).get("draw", 0) for ch in _CHANNEL_ORDER},
                "ros2_status": getattr(hardware, "ros2_status", {}),
            }
            status_msg = String()
            status_msg.data = json.dumps(status, default=str)
            self._status_pub.publish(status_msg)

            self.get_logger().info(f"Published main={main_battery:.1f}% draws={draw_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")


def main(args=None):
    if not _RCLPY_AVAILABLE:
        logger.error("rclpy not installed — cannot run ROS2 publisher node")
        sys.exit(1)

    rclpy.init(args=args)
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()