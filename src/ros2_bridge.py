"""
ROS2 bridge — publishers, subscribers, and mock fallback when rclpy is unavailable.
"""

from __future__ import annotations

import json
import threading
import time
from datetime import datetime

from src.config import config
from src.logger import logger

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float32, Float32MultiArray, String

    _RCLPY_AVAILABLE = True
except ImportError:
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    SingleThreadedExecutor = None  # type: ignore
    Float32 = Float32MultiArray = String = None  # type: ignore
    _RCLPY_AVAILABLE = False

_DEFAULT_TOPICS = {
    "main_battery": "/robot/battery/main_level",
    "power_draw": "/robot/battery/power_draw",
    "power_status": "/robot/battery/status",
    "mission_command": "/robot/battery/command/mission",
    "throttle_command": "/robot/battery/command/throttle",
    "sensor_power": "/robot/sensors/power_draw",
}

_CHANNEL_ORDER = ["Legs", "Arms", "Torso", "Compute"]


def ros2_available() -> bool:
    return _RCLPY_AVAILABLE


if _RCLPY_AVAILABLE:

    class _BatteryMonitorNode(Node):
        """ROS2 node with publishers and command/sensor subscribers."""

        def __init__(self, bridge: "ROS2Bridge"):
            ros_cfg = bridge._ros_cfg
            super().__init__(ros_cfg.get("node_name", "robot_battery_monitor"))
            self._bridge = bridge
            topics = bridge._topics

            self._main_pub = self.create_publisher(Float32, topics["main_battery"], 10)
            self._draw_pub = self.create_publisher(Float32MultiArray, topics["power_draw"], 10)
            self._status_pub = self.create_publisher(String, topics["power_status"], 10)

            self.create_subscription(String, topics["mission_command"], self._on_mission, 10)
            self.create_subscription(Float32, topics["throttle_command"], self._on_throttle, 10)
            self.create_subscription(Float32MultiArray, topics["sensor_power"], self._on_sensor, 10)

            self.get_logger().info(
                f"ROS2 bridge active — publishing {topics['main_battery']}, "
                f"{topics['power_draw']}, {topics['power_status']}"
            )

        def _on_mission(self, msg: String) -> None:
            self._bridge._handle_mission_command(msg.data)

        def _on_throttle(self, msg: Float32) -> None:
            self._bridge._handle_throttle_command(float(msg.data))

        def _on_sensor(self, msg: Float32MultiArray) -> None:
            self._bridge._handle_sensor_draw(list(msg.data))

        def publish_state(self, payload: dict) -> None:
            battery = float(payload.get("main_battery", 0))
            draws = payload.get("channel_draws", {})

            bat_msg = Float32()
            bat_msg.data = battery
            self._main_pub.publish(bat_msg)

            draw_msg = Float32MultiArray()
            draw_msg.data = [float(draws.get(ch, 0.0)) for ch in _CHANNEL_ORDER]
            self._draw_pub.publish(draw_msg)

            status_msg = String()
            status_msg.data = json.dumps(payload, default=str)
            self._status_pub.publish(status_msg)

else:
    _BatteryMonitorNode = None  # type: ignore


class ROS2Bridge:
    """Publishes power telemetry and receives commands / sensor data."""

    def __init__(self, channel_ids: list[str] | None = None):
        hw = config.get("hardware") or {}
        self._ros_cfg = hw.get("ros2") or {}
        self._topics = {**_DEFAULT_TOPICS, **(self._ros_cfg.get("topics") or {})}
        self._channel_ids = channel_ids or _CHANNEL_ORDER
        self._use_mock = not _RCLPY_AVAILABLE or self._ros_cfg.get("mock", False)
        self._running = False
        self._node = None
        self._executor = None
        self._spin_thread: threading.Thread | None = None

        self._commanded_task: str | None = None
        self._commanded_throttle: float | None = None
        self._sensor_draws: dict[str, float] = {}
        self._last_publish: datetime | None = None
        self._publish_count = 0
        self._mock_outbox: list[dict] = []

    @property
    def status(self) -> dict:
        return {
            "available": _RCLPY_AVAILABLE,
            "active": self._running,
            "mode": "mock" if self._use_mock else "live",
            "node_name": self._ros_cfg.get("node_name", "robot_battery_monitor"),
            "topics": self._topics,
            "publish_count": self._publish_count,
            "last_publish": self._last_publish.isoformat() if self._last_publish else None,
            "commanded_task": self._commanded_task,
            "commanded_throttle": self._commanded_throttle,
            "sensor_channels": list(self._sensor_draws.keys()),
        }

    def start(self) -> None:
        if self._running:
            return
        self._running = True

        if self._use_mock:
            logger.info("ROS2 bridge started in mock mode (rclpy unavailable or mock=true)")
            return

        if not _BatteryMonitorNode:
            logger.warning("rclpy not available — falling back to mock mode")
            self._use_mock = True
            return

        if not rclpy.ok():
            rclpy.init()
        self._node = _BatteryMonitorNode(self)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin():
            while self._running and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)

        self._spin_thread = threading.Thread(target=_spin, daemon=True, name="ROS2Spin")
        self._spin_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._executor and self._node:
            self._executor.remove_node(self._node)
            self._node.destroy_node()
        if _RCLPY_AVAILABLE and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        self._node = None
        self._executor = None

    def _handle_mission_command(self, task: str) -> None:
        task = (task or "").strip().lower()
        if task:
            self._commanded_task = task
            logger.info(f"ROS2 mission command received: {task}")

    def _handle_throttle_command(self, factor: float) -> None:
        self._commanded_throttle = max(0.0, min(1.0, factor))
        logger.info(f"ROS2 throttle command received: {self._commanded_throttle:.2f}")

    def _handle_sensor_draw(self, values: list[float]) -> None:
        for i, ch_id in enumerate(_CHANNEL_ORDER):
            if i < len(values) and values[i] > 0:
                self._sensor_draws[ch_id] = round(float(values[i]), 1)
        if self._sensor_draws:
            logger.debug(f"ROS2 sensor draw update: {self._sensor_draws}")

    def consume_commanded_task(self) -> str | None:
        task = self._commanded_task
        self._commanded_task = None
        return task

    def get_throttle_override(self) -> float | None:
        return self._commanded_throttle

    def get_sensor_draws(self) -> dict[str, float]:
        return dict(self._sensor_draws)

    def inject_command(self, mission: str | None = None, throttle: float | None = None,
                       sensor_draws: dict[str, float] | None = None) -> None:
        """Test helper — inject commands without ROS2."""
        if mission:
            self._handle_mission_command(mission)
        if throttle is not None:
            self._handle_throttle_command(throttle)
        if sensor_draws:
            ordered = [sensor_draws.get(ch, 0.0) for ch in _CHANNEL_ORDER]
            self._handle_sensor_draw(ordered)

    def publish(
        self,
        main_battery: float,
        readings: dict,
        allocation: dict | None = None,
        mission_info: dict | None = None,
    ) -> None:
        channel_draws = {ch_id: data.get("draw", 0.0) for ch_id, data in readings.items()}
        payload = {
            "main_battery": main_battery,
            "channel_draws": channel_draws,
            "total_draw_w": sum(channel_draws.values()),
            "task": (mission_info or {}).get("task") or (allocation or {}).get("task"),
            "task_label": (mission_info or {}).get("task_label"),
            "allocation_status": (allocation or {}).get("status"),
            "utilization_pct": (allocation or {}).get("utilization_pct"),
            "timestamp": datetime.now().isoformat(),
        }

        if self._use_mock:
            self._mock_outbox.append(payload)
            if len(self._mock_outbox) > 50:
                self._mock_outbox.pop(0)
        elif self._node:
            self._node.publish_state(payload)

        self._last_publish = datetime.now()
        self._publish_count += 1

    def last_published(self) -> dict | None:
        return self._mock_outbox[-1] if self._mock_outbox else None