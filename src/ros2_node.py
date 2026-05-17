#!/usr/bin/env python3
"""
ROS2 Battery Publisher Node (Improved)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import sys
import os
import time

# Allow importing from src/ when running inside Docker
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.database import get_all_readings


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('robot_battery_publisher')

        self.main_battery_pub = self.create_publisher(Float32, '/robot/battery/main_level', 10)
        self.power_draw_pub = self.create_publisher(Float32MultiArray, '/robot/battery/power_draw', 10)

        self.get_logger().info("🚀 Robot Battery ROS2 Publisher started")
        self.get_logger().info("   Publishing on: /robot/battery/main_level and /robot/battery/power_draw")

        # Publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_battery_data)
        self.get_logger().info("Node is running. Press Ctrl+C to stop.")

    def publish_battery_data(self):
        try:
            entries = get_all_readings(limit=50)

            if not entries:
                self.get_logger().warn("No data in database yet. Waiting...")
                return

            main_battery = float(entries[0]["battery"])

            # Publish main battery
            msg = Float32()
            msg.data = main_battery
            self.main_battery_pub.publish(msg)

            # Publish power draw per channel
            draw_msg = Float32MultiArray()
            channels = ["Legs", "Arms", "Torso", "Compute"]
            draws = []
            for ch in channels:
                latest = next((e for e in entries if e["channel"] == ch), None)
                draws.append(float(latest["draw"]) if latest else 0.0)

            draw_msg.data = draws
            self.power_draw_pub.publish(draw_msg)

            self.get_logger().info(f"Published → Main: {main_battery:.1f}% | Draws: {draws}")

        except Exception as e:
            self.get_logger().error(f"Error publishing battery data: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down ROS2 Battery Publisher...")
        super().destroy_node()


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