"""
DigitalTwinBridge — PMS interface layer for external simulators and hardware.

Architecture
------------
The bridge is the single arbitration point between:

  1. **External twin** (Webots, PyBullet, custom HTTP feed, hardware)
  2. **Internal simulation** (built-in ButlerBot / SimulationDriver)

Inbound path::

    POST /api/twin/telemetry
        → get_adapter(name).normalize(payload)   # source-specific → TwinTelemetry
        → validate
        → store as _last_telemetry
        → (later) get_power_feed() / sync_to_hardware()

Outbound path::

    POST /api/twin/command
        → mission / throttle / channel_draws into PMS
        → drive left/right + duration for Webots teleop
        → drive_stop bumps stop_epoch (see below)
        → battery_reset parks a one-shot override for the controller poll

Webots polls GET /api/twin/state (export_state → teleop block) each sim step
to pick up drive commands, stop_epoch, and pending battery overrides.

External vs internal
--------------------
``_is_external_active()`` is true only when:
  - bridge enabled
  - last telemetry exists
  - source is not "internal"
  - telemetry is fresher than ``stale_after_s``

When external is active *and* ``prefer_external`` is true, ``get_power_feed()``
supplies live watts to the hardware layer. Otherwise the PMS continues on its
internal loop — external twin never hard-fails the stack when Webots is down.

Battery override
----------------
``apply_battery_override`` (config) gates whether twin battery_pct overwrites
``hardware._main_battery``. Separately, API ``battery_reset`` always parks
``battery_pct`` + ``reset_thermal`` on the teleop export so the *Webots*
controller can snap its local pack/thermal model; that one-shot is cleared
after a single export so it is not re-applied forever.

stop_epoch (drive halt handshake)
---------------------------------
A simple left_v=right_v=0 is easy to miss if the controller is mid-command or
the poll races a new drive. ``drive_stop`` increments ``_webots_stop_seq`` and
stamps ``stop_epoch`` into the teleop export. Controllers treat a *new* epoch
as a hard halt signal (zero wheels, clear residual spin logic) even if velocity
fields were already zero — critical for residual spin / ABS stop reliability.

Telemetry POST contract
-----------------------
Adapters produce ``TwinTelemetry``; this class does not re-implement source
math. Integration notes and schema live in ``schema()`` for live discovery.
"""

from __future__ import annotations

import time
from datetime import datetime, timezone

from src.config import config
from src.logger import logger
from src.mission_tasks import TASK_PROFILES
from src.twin.adapters import get_adapter, registered_adapters
from src.twin.butlerbot import butlerbot_flow_description
from src.twin.models import TWIN_SCHEMA_VERSION, TwinTelemetry
from src.twin.power_feed import PowerFeed, power_feed_from_telemetry

_VALID_TASKS = frozenset(TASK_PROFILES.keys())


class DigitalTwinBridge:
    """Central bridge between external data sources and the PMS hardware layer.

    Instantiated once via ``get_twin_bridge()`` so API routes, hardware sync,
    and Webots teleop share the same telemetry and command state.
    """

    def __init__(self):
        twin_cfg = config.get("digital_twin") or {}
        self._enabled = bool(twin_cfg.get("enabled", True))
        self._adapter_name = twin_cfg.get("adapter", "generic")
        self._accept_commands = bool(twin_cfg.get("accept_external_commands", True))
        self._accept_telemetry = bool(twin_cfg.get("accept_external_telemetry", True))
        self._sync_interval_s = float(twin_cfg.get("sync_interval_s", 3))
        # Telemetry older than this is treated as "Webots disconnected".
        self._stale_after_s = float(twin_cfg.get("stale_after_s", 12))
        self._prefer_external = bool(twin_cfg.get("prefer_external", True))
        # When True, twin battery_pct overwrites hardware._main_battery on sync.
        self._apply_battery = bool(twin_cfg.get("apply_battery_override", False))
        self._robot_name = config.get("robot", "name", "ButlerBot")
        try:
            from src.hardware_profile import battery_capacity_wh

            self._battery_wh = battery_capacity_wh()
        except Exception:
            self._battery_wh = config.get("robot", "main_battery_capacity_wh", 480)

        self._last_telemetry: TwinTelemetry | None = None
        self._telemetry_count = 0
        self._last_export_at: datetime | None = None
        self._last_command_at: datetime | None = None
        self._command_count = 0
        self._active_source = "internal"
        # Shared scratchpad polled by Webots each step (drive + one-shot battery).
        self._webots_teleop: dict = {
            "left_v": 0.0,
            "right_v": 0.0,
            "drive_until": 0.0,
            "source": "",
            "battery_pct": None,
            "reset_thermal": False,
        }
        # Monotonic stop handshake; bumped on every drive_stop command.
        self._webots_stop_seq = 0

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def active_source(self) -> str:
        """Report external source name while feed is live; else \"internal\"."""
        if self._is_external_active():
            return self._last_telemetry.source if self._last_telemetry else "external"
        return "internal"

    def _is_stale(self) -> bool:
        if not self._last_telemetry:
            return True
        age = (datetime.now(timezone.utc) - self._last_telemetry.timestamp).total_seconds()
        return age > self._stale_after_s

    def _is_external_active(self) -> bool:
        """True only for a fresh, non-internal telemetry sample."""
        if not self._enabled or not self._last_telemetry:
            return False
        if self._last_telemetry.source == "internal":
            return False
        return not self._is_stale()

    def ingest_telemetry(self, payload: dict, adapter: str | None = None) -> dict:
        """Receive telemetry from an external simulator or custom script.

        Adapter resolution order: explicit arg → payload.adapter → config default.
        On success, replaces ``_last_telemetry`` (last writer wins); does not
        immediately push to hardware — callers/sync path do that on the tick.
        """
        if not self._enabled:
            return {"ok": False, "error": "Digital twin bridge disabled"}
        if not self._accept_telemetry:
            return {"ok": False, "error": "External telemetry disabled in config"}

        adapter_name = adapter or payload.get("adapter") or self._adapter_name
        adapter_impl = get_adapter(str(adapter_name))
        telemetry = adapter_impl.normalize(payload)
        errors = telemetry.validate()
        if errors:
            return {"ok": False, "errors": errors, "adapter": adapter_impl.name}

        self._last_telemetry = telemetry
        self._telemetry_count += 1
        self._active_source = telemetry.source
        logger.info(
            f"DigitalTwinBridge ingest [{telemetry.source}/{adapter_impl.name}] "
            f"task={telemetry.task} draws={len(telemetry.channel_draws)}"
        )
        return {
            "ok": True,
            "adapter": adapter_impl.name,
            "source": telemetry.source,
            "task": telemetry.task,
            "channel_count": len(telemetry.channel_draws),
            "timestamp": telemetry.timestamp.isoformat(),
        }

    def get_power_feed(self) -> PowerFeed | None:
        """Return the live external power contract, or None if twin feed is inactive.

        ``prefer_external`` must be true — otherwise the hardware layer is told
        to stay on internal simulation even if telemetry is arriving (useful for
        dry-run dual feeds).
        """
        if not self._is_external_active() or not self._prefer_external:
            return None
        return power_feed_from_telemetry(self._last_telemetry)

    def sync_to_hardware(
        self,
        hardware,
        *,
        include_mission: bool = True,
        include_sensors: bool = True,
        include_battery: bool = True,
        apply_readings: bool = True,
    ) -> bool:
        """Apply fresh external telemetry into the live PMS layer (non-blocking).

        Returns False when external is inactive so the caller can keep its
        internal tick without special casing. Battery write is gated by
        ``apply_battery_override``; mission/draws go through ROS2 inject so
        the allocator sees the same path as mock/hardware commands.

        ``apply_readings=False`` skips ``apply_power_feed`` when the caller is
        already inside ``_build_readings`` — re-entering that path can deadlock
        or double-apply under the hardware lock.
        """
        if not self._is_external_active() or not self._prefer_external:
            return False

        tel = self._last_telemetry
        if not tel:
            return False

        feed = power_feed_from_telemetry(tel)
        if feed is None:
            return False

        if hasattr(hardware, "_ros2") and (include_mission or include_sensors):
            hardware._ros2.inject_command(
                mission=tel.task if include_mission else None,
                throttle=tel.throttle if include_mission else None,
                sensor_draws=(tel.channel_draws or None) if include_sensors else None,
            )

        if (
            include_battery
            and self._apply_battery
            and feed.battery_pct is not None
            and hasattr(hardware, "_main_battery")
        ):
            hardware._main_battery = round(float(feed.battery_pct), 2)

        # Immediate last_readings patch so dashboard does not wait for the 3s tick.
        # Skip when called from inside _build_readings (apply_readings=False) — that
        # path already rebuilds last_readings and would only risk lock re-entry.
        if apply_readings and hasattr(hardware, "apply_power_feed"):
            hardware.apply_power_feed(feed)
        elif hasattr(hardware, "power_source"):
            hardware.power_source = feed.source

        return True

    def export_state(self, hardware) -> dict:
        """Export PMS state plus twin bridge metadata for external consumers.

        Webots and dashboards use this as a single snapshot: allocation, safety,
        agent posture, optional external_feed, and the teleop poll block.
        """
        self._last_export_at = datetime.now(timezone.utc)
        readings = getattr(hardware, "last_readings", {}) or {}
        allocation = getattr(hardware, "allocation_status", {}) or {}
        mission = getattr(hardware, "mission_info", {}) or {}
        prediction = getattr(hardware, "prediction_status", {}) or {}
        safety = getattr(hardware, "safety_status", {}) or {}
        agent = getattr(hardware, "agent_status", {}) or {}
        simulation = getattr(hardware, "simulation_status", {}) or {}

        batteries = [d.get("battery", 0) for d in readings.values() if d]
        main_battery = round(sum(batteries) / len(batteries), 1) if batteries else None

        channels = [
            {
                "id": ch_id,
                "draw_w": data.get("draw"),
                "requested_w": data.get("requested_w"),
                "allocated_w": data.get("allocated_w"),
                "amps": data.get("amps"),
                "voltage": data.get("voltage"),
                "max_draw_w": data.get("max_draw_w"),
                "allocation_pct": data.get("allocation_pct"),
                "throttled": data.get("throttled", False),
                "status": data.get("status", "normal"),
            }
            for ch_id, data in readings.items()
        ]

        tel = self._last_telemetry
        return {
            "schema_version": TWIN_SCHEMA_VERSION,
            "bridge": self.status(),
            "timestamp": self._last_export_at.isoformat(),
            "robot": {
                "name": self._robot_name,
                "battery_capacity_wh": self._battery_wh,
                "main_battery_pct": main_battery,
                "energy_wh_remaining": mission.get("energy_wh_remaining"),
            },
            "mission": {
                "task": mission.get("task") or allocation.get("task"),
                "task_label": mission.get("task_label"),
                "task_remaining_s": mission.get("task_remaining_s"),
                "runtime_min_at_current_draw": mission.get("runtime_min_at_current_draw"),
            },
            "power": {
                "system_budget_w": allocation.get("system_budget_w"),
                "budget_w": allocation.get("budget_w"),
                "total_draw_w": allocation.get("total_allocated_w"),
                "total_requested_w": allocation.get("total_requested_w"),
                "utilization_pct": allocation.get("utilization_pct"),
                "status": allocation.get("status"),
                "throttled_channels": allocation.get("throttled_channels", []),
            },
            "channels": channels,
            "prediction": {
                "predicted_draw_w": prediction.get("predicted_draw_w"),
                "risk_level": prediction.get("risk_level"),
                "confidence_pct": prediction.get("confidence_pct"),
                "mission_energy_ok": prediction.get("mission_energy_ok"),
            },
            "safety": {
                "status": safety.get("status"),
                "thermal_c": safety.get("thermal_c"),
                "thermal_status": safety.get("thermal_status"),
                "degradation_level": safety.get("degradation_level"),
                "throttle_required": safety.get("throttle_required"),
                "throttle_factor": safety.get("throttle_factor"),
            },
            "agent": {
                "posture": agent.get("posture"),
                "summary": agent.get("summary"),
                "recommendation_count": agent.get("recommendation_count", 0),
                "intervening": bool(agent.get("intervening") or agent.get("applied_actions")),
                "throttle_factor": safety.get("throttle_factor"),
            },
            "simulation": {
                "running": simulation.get("running", False),
                "segment_label": simulation.get("segment_label"),
                "expected_draw_w": simulation.get("expected_draw_w"),
            },
            "external_feed": tel.to_dict() if tel and self._is_external_active() else None,
            "teleop": self._export_teleop(),
            # Top-level control snapshot for suites / agents (from Webots sensors)
            "control_diag": self._export_control_diag(tel),
        }

    def _export_control_diag(self, tel) -> dict | None:
        """Pull ``sensors.control_diag`` from last external telemetry if live."""
        if tel is None or not self._is_external_active():
            return None
        raw = tel.raw if isinstance(getattr(tel, "raw", None), dict) else {}
        sensors = raw.get("sensors") if isinstance(raw.get("sensors"), dict) else {}
        diag = sensors.get("control_diag")
        if isinstance(diag, dict) and diag:
            return diag
        # Fallback: promote flat hub fields if present
        if sensors.get("left_wheel_rad_s") is not None:
            return {
                "schema": "butlerbot_control_diag_v1_flat",
                "hub_left_rad_s": sensors.get("left_wheel_rad_s"),
                "hub_right_rad_s": sensors.get("right_wheel_rad_s"),
                "yaw_rate_rad_s": sensors.get("yaw_rate"),
                "abs_active": sensors.get("braking"),
                "residual_spin": sensors.get("residual_spin"),
            }
        return None

    def _export_teleop(self) -> dict:
        """Active external drive / battery commands for Webots to poll.

        Auto-expires timed drive commands when ``drive_until`` elapses so a
        stuck API client cannot leave wheels commanded forever.

        ``stop_epoch`` is included when set so the controller can detect a new
        hard-stop event even if left/right are already zero.

        Battery override is one-shot: after export, ``battery_pct`` /
        ``reset_thermal`` are cleared so a single reset does not re-fire every
        poll.
        """
        now = time.time()
        left = float(self._webots_teleop.get("left_v", 0.0))
        right = float(self._webots_teleop.get("right_v", 0.0))
        until = float(self._webots_teleop.get("drive_until", 0.0))
        if until and now > until:
            left = right = 0.0
            self._webots_teleop["left_v"] = 0.0
            self._webots_teleop["right_v"] = 0.0
            self._webots_teleop["drive_until"] = 0.0
        out = {
            "left_v": left,
            "right_v": right,
            "active": abs(left) > 0.01 or abs(right) > 0.01,
            "source": self._webots_teleop.get("source") or "",
            "drive_until": until if until > now else None,
        }
        stop_epoch = self._webots_teleop.get("stop_epoch")
        if stop_epoch:
            out["stop_epoch"] = float(stop_epoch)
        pending_batt = self._webots_teleop.get("battery_pct")
        if pending_batt is not None:
            # One-shot delivery to Webots local battery / thermal model.
            out["battery_pct"] = float(pending_batt)
            out["reset_thermal"] = bool(self._webots_teleop.get("reset_thermal"))
            self._webots_teleop["battery_pct"] = None
            self._webots_teleop["reset_thermal"] = False
        return out

    def apply_command(self, hardware, command: dict) -> dict:
        """Apply outbound commands from twin consumers into the PMS.

        Supported keys (any subset):
          mission/task, throttle, sensor_draws/channel_draws,
          simulation start|stop, battery_reset, drive, drive_stop

        Drive commands only update the teleop scratchpad — Webots must poll
        export to actuate. ``drive_stop`` zeros velocities *and* advances
        ``stop_epoch`` for the residual-spin hard-halt handshake.
        """
        if not self._enabled:
            return {"ok": False, "error": "Digital twin bridge disabled"}
        if not self._accept_commands:
            return {"ok": False, "error": "External commands disabled in config"}

        applied: list[str] = []
        errors: list[str] = []

        task = command.get("mission") or command.get("task")
        if task:
            task = str(task).strip().lower()
            if task not in _VALID_TASKS:
                errors.append(f"Invalid task: {task}")
            elif hasattr(hardware, "_mission"):
                if hardware._mission.force_task(task):
                    applied.append(f"mission={task}")
                else:
                    errors.append(f"Mission override rejected: {task}")
            elif hasattr(hardware, "_ros2"):
                hardware._ros2.inject_command(mission=task)
                applied.append(f"mission={task}")

        throttle = command.get("throttle")
        if throttle is not None:
            try:
                factor = float(throttle)
                if not 0.0 < factor <= 1.0:
                    errors.append("Throttle must be in (0, 1]")
                elif hasattr(hardware, "_ros2"):
                    hardware._ros2.inject_command(throttle=factor)
                    applied.append(f"throttle={factor:.0%}")
            except (TypeError, ValueError):
                errors.append("Invalid throttle value")

        sensor_draws = command.get("sensor_draws") or command.get("channel_draws")
        if sensor_draws and isinstance(sensor_draws, dict):
            cleaned = {str(k): float(v) for k, v in sensor_draws.items() if v is not None}
            if hasattr(hardware, "_ros2"):
                hardware._ros2.inject_command(sensor_draws=cleaned)
                applied.append(f"sensor_draws={len(cleaned)} channels")

        sim_action = command.get("simulation")
        if sim_action == "start" and hasattr(hardware, "start_simulation"):
            hardware.start_simulation()
            applied.append("simulation=start")
        elif sim_action == "stop" and hasattr(hardware, "stop_simulation"):
            hardware.stop_simulation()
            applied.append("simulation=stop")

        if command.get("battery_reset"):
            try:
                pct = float(command.get("battery_pct", 100))
            except (TypeError, ValueError):
                errors.append("Invalid battery_pct")
            else:
                pct = max(5.0, min(100.0, pct))
                if hasattr(hardware, "_main_battery"):
                    hardware._main_battery = pct
                # Park one-shot for Webots poll (cleared in _export_teleop).
                self._webots_teleop["battery_pct"] = pct
                self._webots_teleop["reset_thermal"] = True
                applied.append(f"battery_reset={pct:.0f}%")

        drive = command.get("drive")
        if isinstance(drive, dict):
            try:
                left = float(drive.get("left", 0.0))
                right = float(drive.get("right", 0.0))
                duration = float(drive.get("duration_s", 2.0))
            except (TypeError, ValueError):
                errors.append("Invalid drive command")
            else:
                duration = max(0.2, min(30.0, duration))
                self._webots_teleop.update({
                    "left_v": left,
                    "right_v": right,
                    "drive_until": time.time() + duration,
                    "source": str(command.get("source") or "api"),
                })
                applied.append(f"drive L={left:.1f} R={right:.1f} ({duration:.1f}s)")

        if command.get("drive_stop"):
            # Hard stop handshake: zero cmd + new stop_epoch so the controller
            # cannot treat "already zero" as a no-op while hubs still coast/spin.
            self._webots_stop_seq += 1
            self._webots_teleop.update({
                "left_v": 0.0,
                "right_v": 0.0,
                "drive_until": 0.0,
                "source": "stop",
                "stop_epoch": float(self._webots_stop_seq),
            })
            applied.append("drive_stop")

        self._last_command_at = datetime.now(timezone.utc)
        if applied:
            self._command_count += 1
            logger.info(f"DigitalTwinBridge command: {', '.join(applied)}")

        return {
            "ok": not errors,
            "applied": applied,
            "errors": errors,
            "timestamp": self._last_command_at.isoformat(),
        }

    def status(self) -> dict:
        """Compact bridge health for dashboard twin panel and export_state."""
        tel = self._last_telemetry
        feed = self.get_power_feed()
        return {
            "enabled": self._enabled,
            "adapter": self._adapter_name,
            "schema_version": TWIN_SCHEMA_VERSION,
            "active_source": self.active_source,
            "external_active": self._is_external_active(),
            "accept_commands": self._accept_commands,
            "accept_telemetry": self._accept_telemetry,
            "prefer_external": self._prefer_external,
            "apply_battery_override": self._apply_battery,
            "sync_interval_s": self._sync_interval_s,
            "stale_after_s": self._stale_after_s,
            "robot_name": self._robot_name,
            "telemetry_count": self._telemetry_count,
            "power_feed": feed.to_dict() if feed else None,
            "command_count": self._command_count,
            "last_telemetry_at": tel.timestamp.isoformat() if tel else None,
            "last_export_at": (
                self._last_export_at.isoformat() if self._last_export_at else None
            ),
            "last_command_at": (
                self._last_command_at.isoformat() if self._last_command_at else None
            ),
            "registered_adapters": registered_adapters(),
        }

    def schema(self) -> dict:
        """Live integration contract for external controllers and docs UIs."""
        return {
            "schema_version": TWIN_SCHEMA_VERSION,
            "description": "DigitalTwinBridge — PMS interface for external simulators",
            "bridge_class": "DigitalTwinBridge",
            "adapters": registered_adapters(),
            "endpoints": {
                "state": "GET /api/twin/state",
                "telemetry": "POST /api/twin/telemetry",
                "command": "POST /api/twin/command",
                "schema": "GET /api/twin/schema",
                "example": "GET /api/twin/example",
            },
            "telemetry": {
                "source": "internal | external | webots | pybullet | custom | hardware",
                "adapter": "generic | webots | pybullet | butlerbot",
                "robot": {"name": "string", "main_battery_pct": "number"},
                "mission": {"task": "idle | moving | balanced | high_load"},
                "channel_draws": {
                    "Legs": "watts",
                    "Arms": "watts",
                    "Torso": "watts",
                    "Compute": "watts",
                    "Cooling": "watts (optional)",
                },
                "locomotion": "optional gait/speed metadata",
            },
            "power_feed": {
                "schema_version": "1.0",
                "description": (
                    "Minimal contract that becomes hardware.last_readings: "
                    "main battery % + per-channel watts. See src/twin/power_feed.py."
                ),
                "fields": {
                    "source": "webots | custom | pybullet | hardware | external",
                    "battery_pct": "0–100 (from robot.main_battery_pct)",
                    "channel_draws": "dict channel_id → watts",
                    "task": "optional mission task",
                },
            },
            "command": {
                "task": "optional mission task",
                "throttle": "optional 0–1",
                "channel_draws": "optional per-channel watts",
                "simulation": "optional start | stop",
            },
            "butlerbot_example": butlerbot_flow_description(),
            "integration_notes": {
                "webots": (
                    "Controller POSTs build_webots_telemetry() to /api/twin/telemetry?adapter=webots. "
                    "Bridge get_power_feed() → ROS2BatterySource.apply_power_feed → last_readings."
                ),
                "pybullet": "POST joint torques via PyBulletAdapter; map to channel draws.",
                "custom": "Use examples/butlerbot_twin_feed.py as a reference loop.",
                "backward_compat": "With no external feed, internal ButlerBot simulation runs unchanged.",
                "enable": (
                    "config digital_twin.enabled=true, prefer_external=true, "
                    "apply_battery_override=true; hardware.mode=real type=ros2."
                ),
                "verify": (
                    "POST sample telemetry → GET /api/battery (or WebSocket) shows matching "
                    "main_battery + channel draws; twin.external_active true; power_source=webots."
                ),
            },
        }


_bridge_instance: DigitalTwinBridge | None = None


def get_twin_bridge() -> DigitalTwinBridge:
    """Process-wide singleton — API, hardware, and tests share one bridge."""
    global _bridge_instance
    if _bridge_instance is None:
        _bridge_instance = DigitalTwinBridge()
    return _bridge_instance


def reset_twin_bridge() -> None:
    """Drop the singleton (tests / config reload). Next get rebuilds from config."""
    global _bridge_instance
    _bridge_instance = None
