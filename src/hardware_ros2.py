# src/hardware_ros2.py
"""
ROS2BatterySource — primary production-like hardware path for ButlerBot PMS.

Role in the system
------------------
This is the orchestration hub for one telemetry tick (default TICK_SECONDS = 3s):

  twin feed / ROS2 commands
       → mission task advance (scripted sim OR Webots phase)
       → per-channel power request (mission targets + cooling + sensor blend)
       → PowerAllocator (task budget + predictive tightening)
       → EnergyPredictor update / forecast
       → SafetyMonitor (thermal, LRU, requirements) + optional throttle
       → OnboardAgent recommendations (optional auto-apply)
       → battery drain (unless twin owns SOC)
       → DB snapshot + ROS2 publish
       → last_readings for dashboard broadcast

When a Webots (or other) digital twin is active, channel draws and optionally
battery SOC come from the twin PowerFeed; the internal SimulationDriver is
paused for task advancement. When the twin is idle, SimulationDriver is the
**PMS script** (Idle → Transit → Patrol → High Load) for the dashboard —
not the Webots robot. Eval is the S in Webots.

Thread safety
-------------
``_readings_lock`` is an RLock because ``_build_readings`` → twin sync →
``apply_power_feed`` may re-enter on the same thread. The dashboard may also
call ``apply_power_feed`` between ticks for low-latency UI updates.

Key public attributes (consumed by dashboard / twin control)
------------------------------------------------------------
last_readings, allocation_status, mission_info, prediction_status,
safety_status, simulation_status, agent_status, twin_status,
twin_control_status, ros2_status, power_source
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
from src.simulation_driver import SimulationDriver, status_for_external_twin
from src.onboard_agent import OnboardAgent
from src.twin import get_twin_bridge
from src.cooling_channel import estimate_cooling_draw_w
from src.mission_forecast import forecast_twin_loop
from src.twin.control import build_twin_control_status, is_twin_stress_phase
from src.twin.power_feed import PowerFeed
from src.database import log_power_snapshot

_START_BATTERY_PCT = 100.0
# Weight for external sensor power samples blended into mission-requested draw.
_SENSOR_BLEND = 0.30


class ROS2BatterySource(RealHardwareSource):
    """
    Full Power Management System loop with ROS2 I/O and digital-twin hooks.

    Extends RealHardwareSource but owns a much richer tick than generic hardware:
    mission, allocation, safety, prediction, agent, twin, and persistence.
    """

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
        # Dashboard-facing status blobs updated every tick.
        self.allocation_status: dict = {}
        self.mission_info: dict = {}
        self.prediction_status: dict = {}
        self.safety_status: dict = {}
        self.requirements_status: dict = {}
        self.ros2_status: dict = {}
        self._requirements = PowerRequirements(self._power_channels, budget)
        self._ros2_throttle_pulse: float | None = None
        self._simulator = SimulationDriver()
        self._mission.attach_simulation_driver(self._simulator)
        self.simulation_status: dict = {}
        self._agent = OnboardAgent()
        self.agent_status: dict = {}
        self._twin = get_twin_bridge()
        self.twin_status: dict = {}
        self.twin_control_status: dict = {}
        # After agent force_task, hold PMS task for N seconds unless twin is driving.
        self._agent_task_hold_remaining: float = 0.0
        # "internal" | "webots" | "custom" — labels power numbers for the UI.
        self.power_source: str = "internal"
        # RLock: _build_readings → sync_to_hardware → apply_power_feed may re-enter.
        self._readings_lock = threading.RLock()

    def _apply_ros2_commands(self) -> None:
        """Consume one-shot ROS2 mission/throttle commands for this tick."""
        twin_active = self._twin._is_external_active()
        # When the internal script owns the mission, discard external task cmds
        # so mock ROS2 feed does not fight the SimulationDriver timeline.
        if self._simulator.enabled and self._simulator.running and not twin_active:
            self._ros2.consume_commanded_task()
            return

        task = self._ros2.consume_commanded_task()
        if task:
            if self._mission.force_task(task):
                # Seed blend from current draws so the transition is continuous.
                for ch_id, draw in self._channel_draw.items():
                    self._mission._blend[ch_id] = draw
                logger.info(f"{self.hardware_name} ROS2 mission override → {task}")
            else:
                logger.warning(f"{self.hardware_name} ignored invalid ROS2 mission: {task}")

        throttle = self._ros2.consume_throttle_override()
        if throttle is not None:
            # Applied once in _apply_allocated_draw, then cleared at end of tick.
            self._ros2_throttle_pulse = throttle
            logger.info(f"{self.hardware_name} ROS2 throttle pulse → {throttle:.0%}")

    def _blend_sensor_draw(self, ch_id: str, requested: float) -> float:
        """Mix mission request with optional ROS2 sensor power samples."""
        sensor = self._ros2.get_sensor_draws().get(ch_id)
        if sensor is None:
            return requested
        return round(requested * (1 - _SENSOR_BLEND) + sensor * _SENSOR_BLEND, 1)

    def _request_draw(self, ch_id: str, target: float) -> float:
        """
        Produce a smoothed, noise-bounded request toward the mission target.

        Uses task profile knobs: smooth_factor (pull), max_draw_delta (noise),
        variation_band (hard envelope around target).
        """
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

    def _apply_allocated_draw(self, ch_id: str, allocated_w: float, *, twin_active: bool = False) -> float:
        """
        Commit allocator output into ``_channel_draw`` (actual reported draw).

        Twin path tracks allocated watts immediately (physics already smooth).
        Internal path eases with alpha=0.32 to avoid UI flicker on throttle steps.
        """
        if self._ros2_throttle_pulse is not None and self._ros2_throttle_pulse < 1.0:
            allocated_w = round(allocated_w * self._ros2_throttle_pulse, 1)

        if twin_active:
            self._channel_draw[ch_id] = round(allocated_w, 1)
            return self._channel_draw[ch_id]

        if ch_id not in self._channel_draw:
            self._channel_draw[ch_id] = allocated_w
            return allocated_w

        current = self._channel_draw[ch_id]
        smoothed = round(current + (allocated_w - current) * 0.32, 1)
        self._channel_draw[ch_id] = smoothed
        return smoothed

    def _battery_capacity_wh(self) -> float:
        """Pack capacity: hardware profile first, then simulation/robot config."""
        try:
            from src.hardware_profile import battery_capacity_wh

            return float(battery_capacity_wh())
        except Exception:
            sim_cfg = config.get("simulation") or {}
            return float(
                sim_cfg.get("battery_capacity_wh")
                or config.get("robot", "main_battery_capacity_wh", 480)
                or 480
            )

    def _drain_battery(self, total_draw_w: float) -> None:
        """
        Integrate pack energy: E = P · Δt; SOC% = E / capacity_wh · 100.

        Floor at 5% so the sim never reports a fully flat pack (avoids div-by-zero
        pathologies downstream and keeps the UI recoverable).
        """
        capacity_wh = self._battery_capacity_wh()
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
        """Map draw / battery / safety flags to a UI channel status string."""
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
        """Public tick wrapper: never raise into the loop; degrade gracefully."""
        try:
            return self._build_readings_inner()
        except Exception as e:
            logger.error(f"{self.hardware_name} telemetry error: {e}", exc_info=True)
            self.health_status = "DEGRADED"
            return self.last_readings if self.last_readings else {}

    def apply_power_feed(self, feed: PowerFeed) -> bool:
        """
        Write external PowerFeed into last_readings immediately.

        Used by DigitalTwinBridge on each Webots POST so the dashboard does not
        wait for the 3s telemetry tick. Does not advance mission/sim clocks.
        """
        if feed is None or not feed.usable():
            return False

        if feed.battery_pct is not None and self._twin._apply_battery:
            self._main_battery = round(float(feed.battery_pct), 2)

        battery_pct = round(self._main_battery, 1)
        self.power_source = feed.source or "external"

        with self._readings_lock:
            if not self.last_readings:
                # Seed minimal channel entries so the broadcaster has something to show.
                seeded: dict = {}
                for ch in self._power_channels:
                    ch_id = ch.get("id")
                    if not ch_id:
                        continue
                    max_w = ch.get("max_draw_w", 30)
                    voltage = ch.get("nominal_voltage", 48)
                    draw = float(feed.channel_draws.get(ch_id, 0.0))
                    seeded[ch_id] = {
                        "battery": battery_pct,
                        "draw": round(draw, 1),
                        "amps": round(draw / voltage, 2) if voltage else 0.0,
                        "max_draw_w": max_w,
                        "voltage": voltage,
                        "requested_w": round(draw, 1),
                        "allocated_w": round(draw, 1),
                        "allocation_pct": round((draw / max_w) * 100, 1) if max_w else 0,
                        "throttled": False,
                        "status": "normal",
                        "task": feed.task or self._mission.task_id,
                        "power_source": self.power_source,
                        "timestamp": datetime.now(),
                    }
                    if ch_id in feed.channel_draws:
                        self._channel_draw[ch_id] = round(draw, 1)
                self.last_readings = seeded
            else:
                for ch_id, data in self.last_readings.items():
                    if ch_id in feed.channel_draws:
                        draw = round(float(feed.channel_draws[ch_id]), 1)
                        voltage = data.get("voltage") or 48
                        max_w = data.get("max_draw_w") or 30
                        data["draw"] = draw
                        data["requested_w"] = draw
                        data["allocated_w"] = draw
                        data["amps"] = round(draw / voltage, 2) if voltage else 0.0
                        data["allocation_pct"] = (
                            round((draw / max_w) * 100, 1) if max_w else 0
                        )
                        self._channel_draw[ch_id] = draw
                    data["battery"] = battery_pct
                    data["power_source"] = self.power_source
                    data["timestamp"] = datetime.now()

            self.last_successful_read = datetime.now()
            self.health_status = "RUNNING"
        return True

    @staticmethod
    def _task_from_twin_telemetry(tel) -> str | None:
        """
        Map live Webots gait/phase/speed to a PMS mission task.

        Priority: explicit motion (drive/teleop/speed) > patrol > manipulate >
        declared feed task > standby. Keeps the dashboard out of Idle while the
        robot is clearly moving even if feed.task is stale.
        """
        if tel is None:
            return None
        loc = tel.locomotion or {}
        try:
            speed = float(loc.get("speed_m_s") or 0.0)
        except (TypeError, ValueError):
            speed = 0.0
        gait = str(loc.get("gait") or "").lower()
        phase = str(loc.get("phase") or "").lower()
        # Motion / teleop always wins over a stale idle task from the feed.
        # 0.08 m/s filters sensor noise / stand sway from true transit.
        if (
            gait in ("drive", "transit", "walk", "turn")
            or phase in ("teleop", "teleop_turn", "drive_transit", "walk_transit")
            or speed >= 0.08
        ):
            return "moving"
        if gait == "patrol" or phase == "patrol":
            return "balanced"
        if gait in ("manipulate", "high_load", "grasp") or phase == "manipulate":
            return "high_load"
        if tel.task in ("idle", "moving", "balanced", "high_load"):
            return tel.task
        if gait in ("stand", "idle", "standby") or phase in ("standby", "return_idle"):
            return "idle"
        return tel.task

    def _apply_twin_mission_task(self, tel) -> bool:
        """Force PMS task from twin motion so dashboard leaves Idle while driving."""
        desired = self._task_from_twin_telemetry(tel)
        if not desired or desired == self._mission.task_id:
            return False
        # Real teleop/transit motion clears agent task hold so we don't stay locked
        # on Idle/balanced for ~14s while the robot is clearly driving.
        if desired == "moving" and self._agent_task_hold_remaining > 0:
            self._agent_task_hold_remaining = 0.0
        elif self._agent_task_hold_remaining > 0 and desired != "moving":
            return False
        if self._mission.force_task(desired):
            for ch_id, draw in self._channel_draw.items():
                self._mission._blend[ch_id] = draw
            logger.info(f"{self.hardware_name} twin mission → {desired}")
            return True
        return False

    def _sync_twin_feed(self, twin_active: bool) -> None:
        """
        Pull twin telemetry into bridge/hardware state without double-writing draws.

        apply_readings=False: we are already inside _build_readings which owns
        last_readings; only inject mission/sensors/battery, do not re-enter
        apply_power_feed (would nest the RLock and risk inconsistent mid-tick data).
        """
        if not twin_active:
            self.power_source = "internal"
            self._twin.sync_to_hardware(self, apply_readings=False)
            return
        tel = self._twin._last_telemetry
        motion_task = self._task_from_twin_telemetry(tel)
        # Agent task hold: keep twin power numbers; still allow moving override.
        if self._agent_task_hold_remaining > 0 and motion_task != "moving":
            self._agent_task_hold_remaining = max(
                0.0, self._agent_task_hold_remaining - TICK_SECONDS
            )
            self._twin.sync_to_hardware(
                self,
                include_mission=False,
                include_sensors=True,
                include_battery=True,
                apply_readings=False,
            )
            return
        if self._agent_task_hold_remaining > 0 and motion_task == "moving":
            self._agent_task_hold_remaining = 0.0
        self._twin.sync_to_hardware(self, apply_readings=False)

    def _build_readings_inner(self) -> dict:
        """
        One full PMS tick. See module docstring for pipeline order.

        Returns channel_id → reading dict for dashboard / DB / ROS2.
        """
        self.twin_status = self._twin.status()
        twin_active = self._twin._is_external_active()
        self._sync_twin_feed(twin_active)
        tel = self._twin._last_telemetry if twin_active else None
        twin_ctx: dict = {}
        if tel:
            loc = tel.locomotion or {}
            phase = loc.get("phase")
            gait = loc.get("gait")
            if phase:
                self._agent.record_phase_change(phase, gait, tel.task)
            raw = tel.raw if isinstance(getattr(tel, "raw", None), dict) else {}
            sensors = raw.get("sensors") if isinstance(raw.get("sensors"), dict) else {}
            pose = tel.pose if isinstance(getattr(tel, "pose", None), dict) else {}
            twin_ctx = {
                "phase": phase,
                "gait": gait,
                "source": tel.source,
                "lane_sensors": {
                    "nadir_gap_px": sensors.get("nadir_gap_px"),
                    "nadir_r_gap_px": sensors.get("nadir_r_gap_px"),
                    "nadir_ahead_px": sensors.get("nadir_ahead_px"),
                    "nadir_r_ahead_px": sensors.get("nadir_r_ahead_px"),
                    "nadir_lateral_m": sensors.get("nadir_lateral_m"),
                    "nadir_r_lateral_m": sensors.get("nadir_r_lateral_m"),
                    "steer": sensors.get("steer"),
                    "error_source": sensors.get("error_source"),
                    "x_m": pose.get("x_m"),
                    "y_m": pose.get("y_m"),
                },
                "lane_keep": bool(
                    sensors.get("lane_keep")
                    or (getattr(self._twin, "_webots_teleop", None) or {}).get("lane_keep")
                ),
            }
        self._apply_ros2_commands()
        # Prefer explicit twin motion → PMS task (not only ROS2 inject path).
        twin_task_changed = False
        if twin_active and tel:
            twin_task_changed = self._apply_twin_mission_task(tel)
        if twin_active:
            # Twin owns the timeline; do not advance the internal script.
            task_changed = twin_task_changed
        else:
            task_changed = self._simulator.advance(self._mission)
        if task_changed:
            for ch_id, draw in self._channel_draw.items():
                self._mission._blend[ch_id] = draw

        capacity_wh = self._battery_capacity_wh()

        # Pre-allocation forecast lets PowerAllocator tighten budget before cuts.
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
        prev_thermal = (self.safety_status or {}).get("thermal_c", 22.0)
        cooling_target = estimate_cooling_draw_w(
            prev_thermal,
            twin_ctx.get("phase") if twin_active else None,
        )

        twin_draws: dict[str, float] = {}
        if twin_active and tel and tel.channel_draws:
            twin_draws = {str(k): float(v) for k, v in tel.channel_draws.items()}

        for ch in self._power_channels:
            ch_id = ch.get("id")
            max_w = ch.get("max_draw_w", 30)
            current = self._channel_draw.get(ch_id)
            if twin_active and twin_draws:
                # Twin is ground truth for channels it reports; Cooling may be local.
                if ch_id in twin_draws:
                    requested[ch_id] = round(twin_draws[ch_id], 1)
                elif ch_id == "Cooling":
                    requested[ch_id] = round(cooling_target, 1)
                else:
                    requested[ch_id] = round(float(current or 0.0), 1)
            else:
                if ch_id == "Cooling":
                    # Take max of mission profile vs thermal estimate (never under-cool).
                    mission_target = self._mission.target_draw(ch_id, max_w, current)
                    target = max(mission_target, cooling_target)
                else:
                    target = self._mission.target_draw(ch_id, max_w, current)
                requested[ch_id] = self._request_draw(ch_id, target)
            channel_meta[ch_id] = {"max_w": max_w, "voltage": ch.get("nominal_voltage", 48)}

        allocation = self._allocator.allocate(
            self._mission.task_id, requested, prediction=pre_prediction
        )
        if twin_active and twin_draws:
            # Override allocator cuts: twin physics draws must match UI / DB truth.
            allocation["requested"] = dict(requested)
            for ch_id, watts in twin_draws.items():
                if ch_id in allocation.get("allocated", {}):
                    allocation["allocated"][ch_id] = round(watts, 1)
            if "Cooling" in requested and "Cooling" in allocation.get("allocated", {}):
                allocation["allocated"]["Cooling"] = requested["Cooling"]
            allocation["total_allocated_w"] = round(
                sum(allocation.get("allocated", {}).values()), 1
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
        loop_forecast: dict = {}
        if twin_active and twin_ctx.get("phase"):
            # Can ButlerBot finish the remaining mission-loop phases on this pack?
            loop_forecast = forecast_twin_loop(
                battery_pct=self._main_battery,
                capacity_wh=capacity_wh,
                current_phase=twin_ctx.get("phase"),
                current_draw_w=total_draw,
            )
            self.prediction_status["loop_forecast"] = loop_forecast
            self.mission_info["loop_forecast"] = loop_forecast
        if twin_active and twin_ctx.get("phase"):
            live_draw = (
                round(sum(twin_draws.values()), 1)
                if twin_draws
                else None
            )
            self.simulation_status = status_for_external_twin(
                self._mission,
                twin_ctx.get("phase"),
                twin_ctx.get("gait"),
                source=twin_ctx.get("source", "webots"),
                live_draw_w=live_draw,
            )
        else:
            self.simulation_status = self._simulator.status(self._mission)
        self.mission_info["simulation"] = self.simulation_status
        allocation.update(self.mission_info)

        if twin_active and twin_ctx.get("phase"):
            from src.twin.control import PHASE_LABELS

            phase = twin_ctx.get("phase", "")
            phase_label = PHASE_LABELS.get(phase, phase.replace("_", " ").title())
            gait = twin_ctx.get("gait") or "—"
            speed = (tel.locomotion or {}).get("speed_m_s") if tel else None
            speed_txt = f"{speed:.2f} m/s" if speed is not None else "—"
            # Live twin: outlook is descriptive only (no synthetic transition timer).
            self.prediction_status["locomotion_outlook"] = {
                "current_phase": phase,
                "current_phase_label": phase_label,
                "outlook": (
                    f"Webots live — {phase_label} (gait {gait}, speed {speed_txt})"
                ),
                "transition_in_s": None,
                "likely_next_phase": None,
                "likely_next_label": None,
            }
            self.mission_info["locomotion_outlook"] = self.prediction_status["locomotion_outlook"]

        battery_pct = round(self._main_battery, 1)
        thermal_stress = 1.0
        if twin_active and is_twin_stress_phase(twin_ctx.get("phase")):
            # Manipulate / high-load twin phases heat faster than idle transit.
            safety_cfg = config.get("safety") or {}
            thermal_stress = float(safety_cfg.get("twin_thermal_stress_mult", 1.9))
        safety = self._safety.evaluate(
            battery_pct=battery_pct,
            requested=allocation["requested"],
            allocated=allocation["allocated"],
            allocation=allocation,
            tick_seconds=TICK_SECONDS,
            channel_meta=channel_meta,
            task_id=self._mission.task_id,
            task_budget_w=allocation.get("budget_w"),
            thermal_stress=thermal_stress,
            twin_phase=twin_ctx.get("phase") if twin_active else None,
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
            throttle_msg = (
                f"Safety throttle ({safety.get('throttle_reason', 'limit')}): "
                f"factor {safety.get('throttle_factor', 1):.0%}"
            )
            allocation["warnings"].append(throttle_msg)
            allocation["status"] = "throttled"
            for ch_id in allocation["allocated"]:
                if ch_id not in allocation["throttled_channels"]:
                    allocation["throttled_channels"].append(ch_id)
            if twin_active:
                self._agent.record_pms_influence(
                    throttle_msg,
                    sim_phase=twin_ctx.get("phase"),
                    gait=twin_ctx.get("gait"),
                    pms_task=self._mission.task_id,
                    priority="high" if safety.get("status") == "fault" else "medium",
                )

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

        agent_result = self._agent.evaluate(
            battery_pct=battery_pct,
            task_id=self._mission.task_id,
            allocation=allocation,
            safety=safety,
            prediction=self.prediction_status,
            mission=self.mission_info,
            readings={},
            twin_context=twin_ctx,
        )
        safety_already_throttled = bool(safety.get("throttle_required"))
        if self._agent.should_auto_apply(twin_active) and agent_result.recommendations:
            # Agent throttle stacks only when safety did not already cut power.
            allocation["allocated"], throttle_applied = self._agent.apply_throttle(
                allocation["allocated"],
                agent_result,
                safety_already_throttled=safety_already_throttled,
            )
            if throttle_applied:
                allocation["total_allocated_w"] = round(
                    sum(allocation["allocated"].values()), 1
                )
                msg = f"Agent throttle applied: {', '.join(throttle_applied)}"
                allocation["warnings"].append(msg)
                allocation["status"] = "throttled"
                self._agent.record_pms_influence(
                    msg,
                    sim_phase=twin_ctx.get("phase"),
                    gait=twin_ctx.get("gait"),
                    pms_task=self._mission.task_id,
                    priority="high",
                )
            if self._agent.auto_apply_task_suggestions or (
                twin_active and self._agent.twin_auto_apply
            ):
                task_applied = self._agent.apply_task_suggestions(
                    self._mission, agent_result
                )
                for item in task_applied:
                    hold_s = float(self._agent._cfg.get("task_override_hold_s", 12))
                    self._agent_task_hold_remaining = hold_s
                    self._agent.record_event(
                        "task_change",
                        f"Agent set PMS task {item} (hold {hold_s:.0f}s)",
                        priority="high",
                        influence="agent",
                        sim_phase=twin_ctx.get("phase"),
                        gait=twin_ctx.get("gait"),
                        pms_task=self._mission.task_id,
                        applied=True,
                    )
        self.agent_status = self._agent.status_dict(agent_result)
        allocation["agent"] = self.agent_status
        self.allocation_status = allocation
        self.twin_control_status = build_twin_control_status(self._twin, self)

        readings = {}
        throttled_set = set(allocation["throttled_channels"])
        if twin_active and tel:
            self.power_source = tel.source or "external"
        else:
            self.power_source = "internal"

        for ch_id, meta in channel_meta.items():
            max_w = meta["max_w"]
            voltage = meta["voltage"]
            allocated_w = allocation["allocated"].get(ch_id, 0.0)
            draw_w = self._apply_allocated_draw(ch_id, allocated_w, twin_active=twin_active)
            amps = round(draw_w / voltage, 2) if voltage > 0 else 0.0
            req_w = allocation["requested"].get(ch_id, draw_w)
            # 0.05 W hysteresis avoids flapping throttled flag on rounding noise.
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
                "power_source": self.power_source,
                "timestamp": datetime.now(),
            }

            if ch_id not in self._peak_power or draw_w > self._peak_power[ch_id]:
                self._peak_power[ch_id] = draw_w

        # When twin owns SOC, skip local drain so pack energy is not double-counted.
        if not (twin_active and self._twin._apply_battery):
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

        # One-shot pulse — must not stick across ticks.
        self._ros2_throttle_pulse = None
        self.health_status = "RUNNING"
        return readings

    def _seed_channel_draws(self) -> None:
        """Initialize channel state from sim draw targets before first tick."""
        targets = self._simulator.draw_targets_for(self._mission.task_id)
        for ch in self._power_channels:
            ch_id = ch.get("id")
            if not ch_id:
                continue
            seed = targets.get(ch_id, ch.get("max_draw_w", 20) * 0.35)
            self._channel_draw[ch_id] = round(seed, 1)
            self._mission._blend[ch_id] = self._channel_draw[ch_id]

    def start(self):
        """Start ROS2 bridge, optional sim loop, apply startup energy cost, begin ticks."""
        if self.running:
            return
        super().start()
        self._seed_channel_draws()
        self._ros2.start()
        if self._simulator.auto_start:
            self._simulator.start(self._mission)
        capacity_wh = self._battery_capacity_wh()
        info = self._mission.mission_info(_START_BATTERY_PCT, capacity_wh, 0)
        ros_mode = self._ros2.status.get("mode", "mock")
        logger.info(
            f"{self.hardware_name} started — battery {_START_BATTERY_PCT}%, "
            f"budget {self._allocator.system_budget_w}W, mission {info['task_label']}, "
            f"ROS2 {ros_mode}"
        )

        # One-time boot energy (inverters/bring-up) so SOC does not start at 100% forever.
        self._main_battery, startup = self._requirements.apply_startup(
            self._main_battery, capacity_wh
        )
        self.requirements_status = {"startup": startup}

        self.last_readings = self._build_readings()
        self.last_successful_read = datetime.now()
        self.ros2_status = self._ros2.status

        threading.Thread(target=self._telemetry_loop, daemon=True, name="ROS2Telemetry").start()

    def _telemetry_loop(self):
        """Daemon loop: rebuild readings every TICK_SECONDS under the readings lock."""
        while self.running:
            try:
                with self._readings_lock:
                    self.last_readings = self._build_readings()
                    self.last_successful_read = datetime.now()
                    self.ros2_status = self._ros2.status
            except Exception as e:
                logger.error(f"Telemetry loop error: {e}", exc_info=True)
                self.health_status = "DEGRADED"
            time.sleep(TICK_SECONDS)

    def start_simulation(self) -> dict:
        """API hook: start/restart the internal ButlerBot mission script."""
        self._simulator.start(self._mission)
        self.simulation_status = self._simulator.status(self._mission)
        return self.simulation_status

    def stop_simulation(self) -> dict:
        """API hook: pause the internal mission script (twin can still drive)."""
        self._simulator.stop()
        self.simulation_status = self._simulator.status(self._mission)
        return self.simulation_status

    def stop(self):
        self._simulator.stop()
        self._ros2.stop()
        super().stop()