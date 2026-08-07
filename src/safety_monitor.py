"""
Fault detection, safety limits, thermal model, and LRU/requirements aggregation.

Role in the system
------------------
Runs every PMS tick after allocation. Produces:

  - faults / warnings / alerts
  - estimated thermal_c and thermal_status
  - degradation_level (normal → caution → degraded → critical)
  - throttle_required / throttle_factor / throttle_reason
  - nested lru + requirements payloads (optionally filtered by twin phase)

Hard vs soft (by design)
------------------------
Hard faults: pack over system_budget_w, channel over max_draw_w, critical
battery/thermal, requirement hard-max faults.

Soft warnings: over task budget while twin drive watts disagree with PMS task,
expected teleop power steps, soft requirement envelope.

Twin-aware behavior
-------------------
When ``twin_phase`` is set, mission_context filters suppress false alarms on
intentionally idle LRUs and exempt them from throttle. Drive-like phases skip
"caution" throttle for high Legs utilization. Twin power spikes warn only
(no throttle) — teleop 5→20W steps are expected.

Thermal model
-------------
First-order lag toward a load-dependent target with continuous cooling toward
ambient. ``thermal_stress`` multiplies heating during twin high-load phases.
"""

from __future__ import annotations

from src.config import config
from src.logger import logger
from src.lru_monitor import LRUMonitor
from src.mission_context import (
    context_summary,
    filter_lru_result,
    filter_requirements,
    is_drive_like_phase,
    throttle_exempt_channels,
)
from src.power_requirements import PowerRequirements

_DEFAULT_SAFETY = {
    "low_battery_warning_pct": 20,
    "low_battery_critical_pct": 10,
    "spike_threshold_pct": 35,
    "over_draw_warning_pct": 92,
    "system_over_budget_pct": 98,
    "thermal_ambient_c": 22.0,
    "thermal_warning_c": 55.0,
    "thermal_critical_c": 68.0,
    "thermal_max_c": 75.0,
    "thermal_heating_factor": 0.42,
    "thermal_cooling_rate": 0.08,
    "twin_thermal_stress_mult": 1.75,
    "safety_throttle_factor": 0.85,
    "critical_throttle_factor": 0.70,
}


class SafetyMonitor:
    """Aggregate safety evaluation and graceful-degradation throttling."""

    def __init__(self, power_channels: list, system_budget_w: float):
        self._channels = {ch["id"]: ch for ch in power_channels}
        self.system_budget_w = system_budget_w
        self._cfg = self._load_config()
        self._lru = LRUMonitor(power_channels, system_budget_w=system_budget_w)
        self._requirements = PowerRequirements(power_channels, system_budget_w)
        self._prev_draw: dict[str, float] = {}
        self._prev_total_draw = 0.0
        self._thermal_c = self._cfg["thermal_ambient_c"]

    def _load_config(self) -> dict:
        cfg = dict(_DEFAULT_SAFETY)
        safety = config.get("safety") or {}
        monitoring = config.get("monitoring") or {}
        # Bridge legacy monitoring.low_battery_threshold into safety keys.
        if monitoring.get("low_battery_threshold") is not None:
            cfg["low_battery_warning_pct"] = monitoring["low_battery_threshold"]
        for key, default in _DEFAULT_SAFETY.items():
            if key in safety:
                cfg[key] = safety[key]
        return cfg

    def _thermal_step(
        self, total_draw_w: float, tick_seconds: float, *, thermal_stress: float = 1.0
    ) -> float:
        """
        Advance estimated chassis temperature for this tick.

        target rises with load_ratio × heating; alpha is a lag factor so heat
        does not jump instantly to the steady-state curve.
        """
        ambient = self._cfg["thermal_ambient_c"]
        max_c = self._cfg["thermal_max_c"]
        stress = max(1.0, float(thermal_stress))
        load_ratio = min(1.5, total_draw_w / self.system_budget_w) if self.system_budget_w else 0
        heating = self._cfg["thermal_heating_factor"] * stress
        target = ambient + (max_c - ambient) * load_ratio * heating
        alpha = min(1.0, heating * (tick_seconds / 2.0))
        heated = self._thermal_c + (target - self._thermal_c) * alpha
        cooled = heated - self._cfg["thermal_cooling_rate"] * (heated - ambient) * (tick_seconds / 3.0)
        return round(max(ambient, min(max_c, cooled)), 1)

    def _thermal_status(self, temp_c: float) -> str:
        if temp_c >= self._cfg["thermal_critical_c"]:
            return "critical"
        if temp_c >= self._cfg["thermal_warning_c"]:
            return "warning"
        return "normal"

    def evaluate(
        self,
        battery_pct: float,
        requested: dict[str, float],
        allocated: dict[str, float],
        allocation: dict,
        tick_seconds: float = 3.0,
        channel_meta: dict[str, dict] | None = None,
        task_id: str = "idle",
        task_budget_w: float | None = None,
        thermal_stress: float = 1.0,
        twin_phase: str | None = None,
    ) -> dict:
        """
        Full safety pass for one tick.

        Side effect: updates internal previous-draw and thermal state used for
        spike detection and the next thermal step.
        """
        faults: list[str] = []
        warnings: list[str] = []
        alerts: list[str] = []
        spike_channels: list[str] = []
        over_draw_channels: list[str] = []

        total_draw = round(sum(allocated.values()), 1)
        budget = allocation.get("budget_w", self.system_budget_w)
        utilization = allocation.get("utilization_pct", 0)

        meta = channel_meta or {
            ch_id: {
                "max_w": self._channels.get(ch_id, {}).get("max_draw_w", draw),
                "voltage": self._channels.get(ch_id, {}).get("nominal_voltage", 48),
            }
            for ch_id, draw in allocated.items()
        }

        # LRU-level checks
        lru_result = self._lru.evaluate(allocated, meta)
        if twin_phase:
            lru_result = filter_lru_result(lru_result, twin_phase)
        faults.extend(lru_result["faults"])
        warnings.extend(lru_result["warnings"])

        req_result = self._requirements.evaluate(
            task_id=task_id,
            lru_states=lru_result["lrus"],
            total_draw_w=total_draw,
            task_budget_w=task_budget_w,
        )
        if twin_phase:
            req_result = filter_requirements(req_result, twin_phase)
        if req_result["violations"]:
            warnings.extend(req_result["violations"][:3])
        # Only hard requirement faults escalate (channel max). Soft task-budget
        # overruns stay as warnings — otherwise idle profiles permanently fault.
        if not req_result["overall_compliant"]:
            for entry in req_result.get("lru_requirements") or []:
                if entry.get("status") != "fault":
                    continue
                label = entry.get("label") or entry.get("id") or "LRU"
                draw = entry.get("draw_w", 0)
                max_w = entry.get("max_draw_w", 0)
                msg = f"{label}: {draw:.1f}W exceeds hard max ({max_w:.1f}W)"
                if msg not in faults:
                    faults.append(msg)
            eps = req_result.get("eps") or {}
            if eps.get("status") == "fault":
                msg = (
                    f"EPS exceeds max: {eps.get('draw_w', 0):.1f}W > "
                    f"{eps.get('max_draw_w', 0):.1f}W"
                )
                if msg not in faults:
                    faults.append(msg)

        # Low battery
        if battery_pct <= self._cfg["low_battery_critical_pct"]:
            faults.append(f"Critical battery: {battery_pct:.1f}%")
            alerts.append("CRITICAL — Battery critically low")
        elif battery_pct <= self._cfg["low_battery_warning_pct"]:
            warnings.append(f"Low battery: {battery_pct:.1f}%")
            alerts.append("WARNING — Battery below safe threshold")

        warn_pct = self._cfg["over_draw_warning_pct"] / 100.0
        spike_pct = self._cfg["spike_threshold_pct"] / 100.0

        for ch_id, draw_w in allocated.items():
            ch = self._channels.get(ch_id, {})
            max_w = ch.get("max_draw_w", draw_w)
            req_w = requested.get(ch_id, draw_w)

            if draw_w > max_w + 0.05:
                msg = f"{ch_id} over-draw: {draw_w:.1f}W > {max_w}W cap"
                if msg not in faults:
                    faults.append(msg)
                over_draw_channels.append(ch_id)
            elif draw_w >= max_w * warn_pct:
                warnings.append(f"{ch_id} near channel limit ({draw_w:.1f}/{max_w}W)")

            if req_w > max_w + 0.05:
                warnings.append(f"{ch_id} requested {req_w:.1f}W exceeds cap ({max_w}W)")

            prev = self._prev_draw.get(ch_id)
            if prev and prev > 1.0:
                delta_pct = (draw_w - prev) / prev
                if delta_pct >= spike_pct:
                    spike_channels.append(ch_id)
                    warnings.append(
                        f"{ch_id} power spike: {prev:.1f}W → {draw_w:.1f}W (+{delta_pct * 100:.0f}%)"
                    )

        # Hard system fault only vs absolute pack budget. Task budget overshoots
        # (e.g. twin drive watts while PMS task still idle) are warnings.
        if total_draw > self.system_budget_w + 0.1:
            faults.append(
                f"System over-draw: {total_draw:.1f}W > {self.system_budget_w:.1f}W system budget"
            )
        elif total_draw > budget + 0.1:
            warnings.append(
                f"Over task budget: {total_draw:.1f}W > {budget:.1f}W (task envelope)"
            )
        elif utilization >= self._cfg["system_over_budget_pct"]:
            warnings.append(f"System budget nearly saturated ({utilization}%)")

        if self._prev_total_draw > 5.0:
            total_delta = (total_draw - self._prev_total_draw) / self._prev_total_draw
            if total_delta >= spike_pct:
                warnings.append(
                    f"System power spike: {self._prev_total_draw:.1f}W → {total_draw:.1f}W "
                    f"(+{total_delta * 100:.0f}%)"
                )

        thermal_c = self._thermal_step(total_draw, tick_seconds, thermal_stress=thermal_stress)
        thermal_status = self._thermal_status(thermal_c)
        if thermal_status == "critical":
            faults.append(f"Thermal critical: {thermal_c}°C")
            alerts.append(f"CRITICAL — Estimated temperature {thermal_c}°C")
        elif thermal_status == "warning":
            warnings.append(f"Thermal warning: {thermal_c}°C")

        degradation = lru_result["degradation_level"]
        exempt = throttle_exempt_channels(twin_phase) if twin_phase else frozenset()
        if degradation == "degraded" and thermal_status == "critical":
            degradation = "critical"
        elif degradation == "caution" and (faults or thermal_status == "warning"):
            degradation = "degraded"

        throttle_required = False
        throttle_factor = 1.0
        throttle_reason = ""
        lru_factors = self._lru.throttle_factors(
            degradation, lru_result["spike_lrus"] + lru_result["over_draw_lrus"]
        )

        if faults:
            throttle_required = True
            throttle_factor = self._cfg["critical_throttle_factor"]
            throttle_reason = "safety fault"
        elif degradation in ("degraded", "critical"):
            throttle_required = True
            throttle_factor = self._cfg["critical_throttle_factor"] if degradation == "critical" else self._cfg["safety_throttle_factor"]
            throttle_reason = f"LRU degradation ({degradation})"
        elif thermal_status == "critical":
            throttle_required = True
            throttle_factor = self._cfg["safety_throttle_factor"]
            throttle_reason = "thermal critical"
        elif thermal_status == "warning":
            throttle_required = True
            throttle_factor = self._cfg["safety_throttle_factor"]
            throttle_reason = "thermal warning"
        elif spike_channels and not twin_phase:
            # Internal sim spikes can throttle; Webots teleop 5→20W+ steps are
            # expected when drive starts — warn only while twin is live.
            throttle_required = True
            throttle_factor = self._cfg["safety_throttle_factor"]
            throttle_reason = "power spike"
        elif spike_channels and twin_phase:
            warnings.append(
                f"Twin power step on {', '.join(spike_channels)} (no throttle — expected teleop)"
            )
        elif degradation == "caution":
            # Soft LRU warnings alone should not force continuous throttle in
            # internal/no-twin idle — only when an active-role LRU is warning/fault.
            # Drive-like twin phases: high Legs util is intentional, skip caution throttle.
            if is_drive_like_phase(twin_phase):
                pass
            else:
                active_lrus = [
                    l for l in lru_result.get("lrus", [])
                    if l.get("mission_role") == "active"
                    and l.get("status") in ("warning", "fault")
                ]
                fault_lrus = [
                    l for l in lru_result.get("lrus", [])
                    if l.get("status") == "fault"
                ]
                if active_lrus or fault_lrus:
                    throttle_required = True
                    throttle_factor = self._lru._cfg.get("degrade_throttle_caution", 0.92)
                    throttle_reason = "LRU caution"

        if battery_pct <= self._cfg["low_battery_critical_pct"]:
            throttle_required = True
            throttle_factor = min(throttle_factor, self._cfg["critical_throttle_factor"])
            throttle_reason = "critical battery"

        status = "ok"
        if faults:
            status = "fault"
        elif throttle_required or warnings:
            status = "warning"

        result = {
            "status": status,
            "battery_pct": battery_pct,
            "thermal_c": thermal_c,
            "thermal_status": thermal_status,
            "thermal_ambient_c": self._cfg["thermal_ambient_c"],
            "thermal_warning_c": self._cfg["thermal_warning_c"],
            "thermal_critical_c": self._cfg["thermal_critical_c"],
            "faults": faults,
            "warnings": warnings,
            "alerts": alerts,
            "spike_channels": spike_channels,
            "over_draw_channels": over_draw_channels,
            "throttle_required": throttle_required,
            "throttle_factor": throttle_factor,
            "throttle_reason": throttle_reason,
            "total_draw_w": total_draw,
            "degradation_level": degradation,
            "lru": lru_result,
            "lru_factors": lru_factors,
            "requirements": req_result,
            "mission_context": context_summary(twin_phase, task_id) if twin_phase else {},
            "throttle_exempt_channels": sorted(exempt),
        }

        self._prev_draw = dict(allocated)
        self._prev_total_draw = total_draw
        self._thermal_c = thermal_c

        if faults:
            logger.warning(f"Safety fault(s): {'; '.join(faults[:4])}")
        elif warnings:
            logger.info(f"Safety warning(s): {'; '.join(warnings[:3])}")
        if degradation != "normal":
            logger.info(f"Graceful degradation: {degradation} — throttle {throttle_reason or 'none'}")

        return result

    def apply_throttle(self, allocated: dict[str, float], safety: dict) -> dict[str, float]:
        """
        Scale allocated watts by global/per-LRU factors from evaluate().

        Exempt channels (twin standby LRUs) keep full draw. Spike channels get
        at least safety_throttle_factor even if LRU factor is milder.
        """
        if not safety.get("throttle_required"):
            return dict(allocated)

        factor = safety.get("throttle_factor", 1.0)
        reason = safety.get("throttle_reason", "safety")
        lru_factors = safety.get("lru_factors") or {}
        exempt = set(safety.get("throttle_exempt_channels") or [])
        spike_set = set(safety.get("spike_channels", []))
        throttled: dict[str, float] = {}

        for ch_id, draw_w in allocated.items():
            if ch_id in exempt:
                throttled[ch_id] = draw_w
                continue
            ch_factor = lru_factors.get(ch_id, factor)
            if ch_id in spike_set:
                ch_factor = min(ch_factor, self._cfg["safety_throttle_factor"])
            throttled[ch_id] = round(draw_w * ch_factor, 1)

        logger.info(
            f"Safety throttle ({reason}): "
            f"{sum(allocated.values()):.1f}W → {sum(throttled.values()):.1f}W"
        )
        return throttled