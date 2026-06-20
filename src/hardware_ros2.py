# src/hardware_ros2.py
"""
ROS2 Battery Source — physics simulation with ROS2 pub/sub integration.
"""

import threading
import time
import random
from datetime import datetime

from src.hardware import RealHardwareSource, _DEFAULT_POWER_CHANNELS
from src.config import config
from src.logger import logger
from src.power_allocator import PowerAllocator
from src.mission_tasks import MissionTaskManager, TICK_SECONDS
from src.energy_predictor import EnergyPredictor
from src.safety_monitor import SafetyMonitor
from src.power_requirements import PowerRequirements
from src.ros2_bridge import ROS2Bridge
from src.database import log_power_snapshot

_START_BATTERY_PCT = 92.0
_SENSOR_BLEND = 0.30


class ROS2BatterySource(RealHardwareSource):
    def __init__(self):
        super().__init__()
        self.hardware_name = "ROS2 Battery Source"
        self._channel_draw: dict[str, float] = {}
        self._peak_power: dict[str, float] = {}
        self._main_battery = _START_BATTERY_PCT
        self._power_channels = config.get("power_channels") or _DEFAULT_POWER_CHANNELS
        budget = config.get("power", "system_budget_w")
        self._allocator = PowerAllocator(self._power_channels, system_budget_w=budget)
        self._safety = SafetyMonitor(self._power_channels, system_budget_w=budget)
        self._mission = MissionTaskManager()
        self._predictor = EnergyPredictor()
        ch_ids = [ch.get("id") for ch in self._power_channels if ch.get("id")]
        self._ros2 = ROS2Bridge(channel_ids=ch_ids)
        self.allocation_status: dict = {}
        self.mission_info: dict = {}
        self.prediction_status: dict = {}
        self.safety_status: dict = {}
        self.requirements_status: dict = {}
        self.ros2_status: dict = {}
        self._requirements = PowerRequirements(self._power_channels, budget)

    def _apply_ros2_commands(self) -> None:
        task = self._ros2.consume_commanded_task()
        if task and self._mission.force_task(task):
            for ch_id, draw in self._channel_draw.items():
                self._mission._blend[ch_id] = draw
            logger.info(f"{self.hardware_name} ROS2 mission override → {task}")

    def _blend_sensor_draw(self, ch_id: str, requested: float) -> float:
        sensor = self._ros2.get_sensor_draws().get(ch_id)
        if sensor is None:
            return requested
        return round(requested * (1 - _SENSOR_BLEND) + sensor * _SENSOR_BLEND, 1)

    def _request_draw(self, ch_id: str, target: float) -> float:
        profile = self._mission.profile
        max_delta = profile.max_draw_delta
        band = profile.variation_band
        smooth = profile.smooth_factor

        if ch_id not in self._channel_draw:
            self._channel_draw[ch_id] = round(target, 1)
            self._mission._blend[ch_id] = self._channel_draw[ch_id]

        current = self._channel_draw[ch_id]
        noise = random.uniform(-max_delta * 0.5, max_delta * 0.5)
        pull = (target - current) * smooth
        requested = current + pull + noise

        lower = target * (1 - band)
        upper = target * (1 + band)
        requested = max(lower, min(upper, requested))
        requested = self._blend_sensor_draw(ch_id, round(requested, 1))
        return requested

    def _apply_allocated_draw(self, ch_id: str, allocated_w: float) -> float:
        throttle = self._ros2.get_throttle_override()
        if throttle is not None:
            allocated_w = round(allocated_w * throttle, 1)

        if ch_id not in self._channel_draw:
            self._channel_draw[ch_id] = allocated_w
            return allocated_w

        current = self._channel_draw[ch_id]
        smoothed = round(current + (allocated_w - current) * 0.32, 1)
        self._channel_draw[ch_id] = smoothed
        return smoothed

    def _drain_battery(self, total_draw_w: float) -> None:
        capacity_wh = config.get("robot", "main_battery_capacity_wh", 1000) or 1000
        energy_wh = total_draw_w * (TICK_SECONDS / 3600)
        drain_pct = (energy_wh / capacity_wh) * 100
        self._main_battery = max(5.0, round(self._main_battery - drain_pct, 3))

    def _channel_status(
        self,
        draw_w: float,
        max_w: float,
        battery_pct: float,
        throttled: bool,
        ch_id: str,
        safety: dict,
    ) -> str:
        low_warn = (config.get("safety") or {}).get("low_battery_warning_pct", 20)
        if battery_pct <= (config.get("safety") or {}).get("low_battery_critical_pct", 10):
            return "critical"
        if battery_pct < low_warn:
            return "critical"
        if ch_id in safety.get("over_draw_channels", []):
            return "critical"
        if ch_id in safety.get("spike_channels", []):
            return "warning"
        if throttled:
            return "throttled"
        task = self._mission.task_id
        if task in ("moving", "high_load") and draw_w > max_w * 0.80:
            return "warning"
        if draw_w > max_w * 0.88:
            return "warning"
        return "normal"

    def _build_readings(self) -> dict:
        try:
            return self._build_readings_inner()
        except Exception as e:
            logger.error(f"{self.hardware_name} telemetry error: {e}", exc_info=True)
            self.health_status = "DEGRADED"
            return self.last_readings if self.last_readings else {}

    def _build_readings_inner(self) -> dict:
        self._apply_ros2_commands()
        task_changed = self._mission.advance()
        if task_changed:
            for ch_id, draw in self._channel_draw.items():
                self._mission._blend[ch_id] = draw

        capacity_wh = config.get("robot", "main_battery_capacity_wh", 1000) or 1000

        recent_draw = sum(self._channel_draw.values()) if self._channel_draw else 0
        pre_prediction = self._predictor.forecast(
            battery_pct=self._main_battery,
            capacity_wh=capacity_wh,
            task_id=self._mission.task_id,
            task_remaining_s=self._mission.seconds_remaining,
            blend_progress=self._mission.blend_progress,
            current_draw_w=recent_draw,
        )
        pre_prediction["task"] = self._mission.task_id

        requested: dict[str, float] = {}
        channel_meta: dict[str, dict] = {}

        for ch in self._power_channels:
            ch_id = ch.get("id")
            max_w = ch.get("max_draw_w", 30)
            current = self._channel_draw.get(ch_id)
            target = self._mission.target_draw(ch_id, max_w, current)
            requested[ch_id] = self._request_draw(ch_id, target)
            channel_meta[ch_id] = {"max_w": max_w, "voltage": ch.get("nominal_voltage", 48)}

        allocation = self._allocator.allocate(
            self._mission.task_id, requested, prediction=pre_prediction
        )
        total_draw = allocation["total_allocated_w"]

        self._predictor.update(total_draw)
        self.prediction_status = self._predictor.forecast(
            battery_pct=self._main_battery,
            capacity_wh=capacity_wh,
            task_id=self._mission.task_id,
            task_remaining_s=self._mission.seconds_remaining,
            blend_progress=self._mission.blend_progress,
            current_draw_w=total_draw,
        )
        self.prediction_status["task"] = self._mission.task_id

        if task_changed:
            logger.info(
                f"{self.hardware_name} mission → {allocation.get('task_label')} "
                f"({self._mission.seconds_remaining}s, "
                f"forecast {self.prediction_status.get('mission_forecast_min')}min, "
                f"conf {self.prediction_status.get('confidence_pct')}%)"
            )

        self.mission_info = self._mission.mission_info(
            battery_pct=self._main_battery,
            capacity_wh=capacity_wh,
            current_draw_w=total_draw,
        )
        self.mission_info.update(self.prediction_status)
        allocation.update(self.mission_info)

        battery_pct = round(self._main_battery, 1)
        safety = self._safety.evaluate(
            battery_pct=battery_pct,
            requested=allocation["requested"],
            allocated=allocation["allocated"],
            allocation=allocation,
            tick_seconds=TICK_SECONDS,
            channel_meta=channel_meta,
            task_id=self._mission.task_id,
            task_budget_w=allocation.get("budget_w"),
        )
        self.requirements_status = safety.get("requirements", {})
        if safety.get("throttle_required"):
            allocation["allocated"] = self._safety.apply_throttle(
                allocation["allocated"], safety
            )
            allocation["total_allocated_w"] = round(sum(allocation["allocated"].values()), 1)
            for w in safety.get("warnings", []):
                if w not in allocation["warnings"]:
                    allocation["warnings"].append(w)
            allocation["warnings"].append(
                f"Safety throttle ({safety.get('throttle_reason', 'limit')}): "
                f"factor {safety.get('throttle_factor', 1):.0%}"
            )
            allocation["status"] = "throttled"
            for ch_id in allocation["allocated"]:
                if ch_id not in allocation["throttled_channels"]:
                    allocation["throttled_channels"].append(ch_id)

        if safety.get("faults"):
            allocation["status"] = "fault"
        elif safety.get("status") == "warning" and allocation["status"] == "ok":
            allocation["status"] = "warning"

        allocation["safety"] = {
            "status": safety.get("status"),
            "thermal_c": safety.get("thermal_c"),
            "thermal_status": safety.get("thermal_status"),
            "alerts": safety.get("alerts", []),
            "faults": safety.get("faults", []),
            "warnings": safety.get("warnings", []),
            "degradation_level": safety.get("degradation_level"),
            "lru": safety.get("lru"),
            "requirements": safety.get("requirements"),
        }
        self.safety_status = safety
        self.allocation_status = allocation

        readings = {}
        throttled_set = set(allocation["throttled_channels"])

        for ch_id, meta in channel_meta.items():
            max_w = meta["max_w"]
            voltage = meta["voltage"]
            allocated_w = allocation["allocated"].get(ch_id, 0.0)
            draw_w = self._apply_allocated_draw(ch_id, allocated_w)
            amps = round(draw_w / voltage, 2) if voltage > 0 else 0.0
            req_w = allocation["requested"].get(ch_id, draw_w)
            throttled = ch_id in throttled_set or allocated_w < req_w - 0.05

            readings[ch_id] = {
                "battery": battery_pct,
                "draw": draw_w,
                "amps": amps,
                "max_draw_w": max_w,
                "voltage": voltage,
                "requested_w": req_w,
                "allocated_w": allocated_w,
                "allocation_pct": round((draw_w / max_w) * 100, 1) if max_w else 0,
                "throttled": throttled,
                "status": self._channel_status(
                    draw_w, max_w, battery_pct, throttled, ch_id, safety
                ),
                "task": self._mission.task_id,
                "timestamp": datetime.now(),
            }

            if ch_id not in self._peak_power or draw_w > self._peak_power[ch_id]:
                self._peak_power[ch_id] = draw_w

        self._drain_battery(total_draw)
        try:
            log_power_snapshot(allocation, readings, battery_pct, self.prediction_status)
        except Exception as e:
            logger.warning(f"DB snapshot failed (non-fatal): {e}")

        try:
            self._ros2.publish(battery_pct, readings, allocation, self.mission_info)
            self.ros2_status = self._ros2.status
        except Exception as e:
            logger.warning(f"ROS2 publish failed (non-fatal): {e}")

        self.health_status = "RUNNING"
        return readings

    def start(self):
        if self.running:
            return
        super().start()
        self._ros2.start()
        capacity_wh = config.get("robot", "main_battery_capacity_wh", 1000) or 1000
        info = self._mission.mission_info(_START_BATTERY_PCT, capacity_wh, 0)
        ros_mode = self._ros2.status.get("mode", "mock")
        logger.info(
            f"{self.hardware_name} started — battery {_START_BATTERY_PCT}%, "
            f"budget {self._allocator.system_budget_w}W, mission {info['task_label']}, "
            f"ROS2 {ros_mode}"
        )

        self._main_battery, startup = self._requirements.apply_startup(
            self._main_battery, capacity_wh
        )
        self.requirements_status = {"startup": startup}

        self.last_readings = self._build_readings()
        self.last_successful_read = datetime.now()
        self.ros2_status = self._ros2.status

        threading.Thread(target=self._telemetry_loop, daemon=True, name="ROS2Telemetry").start()

    def _telemetry_loop(self):
        while self.running:
            try:
                self.last_readings = self._build_readings()
                self.last_successful_read = datetime.now()
                self.ros2_status = self._ros2.status
            except Exception as e:
                logger.error(f"Telemetry loop error: {e}", exc_info=True)
                self.health_status = "DEGRADED"
            time.sleep(TICK_SECONDS)

    def stop(self):
        self._ros2.stop()
        super().stop()