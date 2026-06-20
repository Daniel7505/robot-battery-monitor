# src/power_allocator.py
"""
Power allocation with task-aware limits and predictive dynamic budgeting.
"""

from src.logger import logger
from src.mission_tasks import TASK_PROFILES

_MIN_CHANNEL_DRAW_W = {
    "Legs": 4.0,
    "Arms": 5.0,
    "Torso": 4.0,
    "Compute": 6.0,
}


class PowerAllocator:
    def __init__(self, power_channels: list, system_budget_w: float | None = None):
        self._channels = {ch["id"]: ch for ch in power_channels}
        if system_budget_w is None:
            system_budget_w = sum(ch.get("max_draw_w", 0) for ch in power_channels) * 0.76
        self.system_budget_w = system_budget_w

    def _base_budget(self, task: str) -> float:
        profile = TASK_PROFILES.get(task)
        factor = profile.budget_factor if profile else 1.0
        return round(self.system_budget_w * factor, 1)

    def _dynamic_budget(self, task: str, base_budget: float, prediction: dict | None) -> float:
        if not prediction:
            return base_budget

        budget = base_budget
        confidence = prediction.get("confidence_pct", 0) / 100.0
        predicted_draw = prediction.get("predicted_draw_w", 0)

        if confidence >= 0.60 and predicted_draw > budget:
            tighten = 0.08 + 0.10 * confidence
            budget = round(budget * (1.0 - tighten), 1)

        if not prediction.get("mission_energy_ok", True):
            budget = round(budget * 0.90, 1)

        if prediction.get("mission_battery_pct_at_end", 100) < 15 and confidence >= 0.55:
            budget = round(budget * 0.88, 1)

        return max(round(base_budget * 0.72, 1), budget)

    def _throttle_order(self, task: str) -> list[str]:
        profile = TASK_PROFILES.get(task)
        if profile:
            return profile.throttle_order
        return ["Compute", "Arms", "Torso", "Legs"]

    def allocate(
        self,
        task: str,
        requested: dict[str, float],
        prediction: dict | None = None,
    ) -> dict:
        decisions: list[str] = []
        warnings: list[str] = []
        throttled_channels: list[str] = []

        capped: dict[str, float] = {}
        for ch_id, req_w in requested.items():
            ch = self._channels.get(ch_id, {})
            max_w = ch.get("max_draw_w", req_w)
            if req_w > max_w:
                capped[ch_id] = round(max_w, 1)
                decisions.append(f"{ch_id}: channel cap {req_w:.1f}W → {max_w:.1f}W")
                warnings.append(f"{ch_id} exceeded max_draw_w ({max_w}W)")
            else:
                capped[ch_id] = round(req_w, 1)

        total_requested = round(sum(capped.values()), 1)
        base_budget = self._base_budget(task)
        budget = self._dynamic_budget(task, base_budget, prediction)

        if prediction and budget < base_budget:
            decisions.append(
                f"predictive budget: {base_budget:.1f}W → {budget:.1f}W "
                f"(conf {prediction.get('confidence_pct', 0)}%, "
                f"pred {prediction.get('predicted_draw_w', 0)}W)"
            )
            if not prediction.get("mission_energy_ok", True):
                warnings.append("Predicted insufficient energy for mission — budget reduced")

        allocated = dict(capped)

        if total_requested > budget:
            over = total_requested - budget
            profile = TASK_PROFILES.get(task)
            task_label = profile.label if profile else task
            decisions.append(
                f"[{task_label}] over budget by {over:.1f}W "
                f"({total_requested:.1f}W requested / {budget:.1f}W effective budget)"
            )

            for ch_id in self._throttle_order(task):
                if over <= 0 or ch_id not in allocated:
                    continue
                floor = _MIN_CHANNEL_DRAW_W.get(ch_id, 2.0)
                reducible = allocated[ch_id] - floor
                if reducible <= 0:
                    continue
                cut = min(reducible, over)
                new_val = round(allocated[ch_id] - cut, 1)
                decisions.append(f"{ch_id}: task throttle {allocated[ch_id]:.1f}W → {new_val:.1f}W")
                allocated[ch_id] = new_val
                over = round(over - cut, 2)
                if ch_id not in throttled_channels:
                    throttled_channels.append(ch_id)

            if over > 0:
                scale = budget / total_requested
                for ch_id in allocated:
                    old = allocated[ch_id]
                    allocated[ch_id] = round(old * scale, 1)
                    decisions.append(f"{ch_id}: proportional scale {old:.1f}W → {allocated[ch_id]:.1f}W")
                    if ch_id not in throttled_channels:
                        throttled_channels.append(ch_id)
                warnings.append(f"Task '{task}' exceeded budget — channels scaled down")

        total_allocated = round(sum(allocated.values()), 1)
        utilization = round((total_allocated / budget) * 100, 1) if budget else 0.0

        if utilization >= 95:
            warnings.append(f"Budget nearly saturated ({utilization}%)")
        if throttled_channels:
            warnings.append(f"Throttled: {', '.join(throttled_channels)}")

        if throttled_channels or total_requested > budget:
            status = "throttled"
        elif warnings:
            status = "warning"
        else:
            status = "ok"

        profile = TASK_PROFILES.get(task)
        result = {
            "task": task,
            "task_label": profile.label if profile else task,
            "task_description": profile.description if profile else "",
            "budget_w": budget,
            "base_budget_w": base_budget,
            "system_budget_w": self.system_budget_w,
            "total_requested_w": total_requested,
            "total_allocated_w": total_allocated,
            "utilization_pct": utilization,
            "allocated": allocated,
            "requested": capped,
            "throttled_channels": throttled_channels,
            "warnings": warnings,
            "decisions": decisions,
            "status": status,
            "dynamic_budget_applied": budget < base_budget,
        }

        if decisions:
            logger.info(
                f"Power allocation [{task}]: {total_allocated:.1f}/{budget:.1f}W "
                f"({utilization}%) — {len(decisions)} decision(s)"
            )
            for line in decisions:
                logger.info(f"  ↳ {line}")

        return result