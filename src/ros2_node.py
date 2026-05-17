#!/usr/bin/env python3
"""
ROS2 Integration Hooks for Robot Battery Monitor
------------------------------------------------
This node publishes battery status to ROS2 topics.

Run this alongside the main dashboard when you want ROS2 integration.

Requirements:
    pip install rclpy
    source /opt/ros/<distro>/setup.bash   (or your ROS2 installation)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from datetime import datetime
import time
import threading

from src.database import get_all_readings


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('robot_battery_publisher')
        
        # Publishers
        self.main_battery_pub = self.create_publisher(Float32, '/robot/battery/main_level', 10)
        self.power_draw_pub = self.create_publisher(Float32MultiArray, '/robot/battery/power_draw', 10)
        
        self.get_logger().info("🚀 Robot Battery ROS2 Publisher started")
        self.get_logger().info("   Publishing on: /robot/battery/main_level and /robot/battery/power_draw")

        # Timer to publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_battery_data)

    def publish_battery_data(self):
        entries = get_all_readings(limit=50)
        if not entries:
            return

        main_battery = float(entries[0]["battery"])

        # Publish main battery level
        msg = Float32()
        msg.data = main_battery
        self.main_battery_pub.publish(msg)

        # Publish power draw per channel (example order: Legs, Arms, Torso, Compute)
        draw_msg = Float32MultiArray()
        channels = ["Legs", "Arms", "Torso", "Compute"]
        draws = []
        for ch in channels:
            latest = next((e for e in entries if e["channel"] == ch), None)
            draws.append(float(latest["draw"]) if latest else 0.0)
        draw_msg.data = draws
        self.power_draw_pub.publish(draw_msg)

        self.get_logger().info(
            f"Published → Main: {main_battery:.1f}% | Draws: {draws}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()