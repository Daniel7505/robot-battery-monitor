"""
High-level power requirements — startup cost, task budgets, min/max per LRU.
"""

from __future__ import annotations

from src.config import config
from src.logger import logger
from src.mission_tasks import TASK_PROFILES

_LRU_CHANNEL_MAP = {
    "locomotion": ["Legs"],
    "arms": ["Arms"],
    "torso": ["Torso"],
    "compute": ["Compute"],
    "cooling": ["Cooling"],
}

_DEFAULT_REQUIREMENTS = {
    "startup_cost_wh": 6.0,
    "eps_reserve_pct": 12,
    "system_min_draw_w": 18,
    "system_max_draw_w": 72,
}


class PowerRequirements:
    def __init__(self, power_channels: list, system_budget_w: float):
        self._channels = {ch["id"]: ch for ch in power_channels}
        self.system_budget_w = system_budget_w
        self._cfg = self._load_config()
        self._startup_applied = False
        self._lru_map = self._load_lru_map()

    def _load_config(self) -> dict:
        cfg = dict(_DEFAULT_REQUIREMENTS)
        req = config.get("requirements") or {}
        cfg.update({k: v for k, v in req.items() if k != "lru_budgets" and k != "tasks"})
        if config.get("power", "system_budget_w"):
            cfg["system_max_draw_w"] = config.get("power", "system_budget_w")
        return cfg

    def _load_lru_map(self) -> dict[str, list[str]]:
        groups = (config.get("lru") or {}).get("groups") or []
        if groups:
            return {g["id"]: g.get("channels", []) for g in groups if g["id"] != "eps"}
        return dict(_LRU_CHANNEL_MAP)

    def _task_lru_limits(self, task_id: str, lru_id: str) -> dict:
        custom = (config.get("requirements") or {}).get("lru_budgets") or {}
        task_cfg = custom.get(task_id, {}).get(lru_id)
        if task_cfg:
            return {
                "min_draw_w": task_cfg.get("min_draw_w", 0),
                "max_draw_w": task_cfg.get("max_draw_w", 99),
                "budget_w": task_cfg.get("budget_w", 99),
            }

        profile = TASK_PROFILES.get(task_id)
        channels = self._lru_map.get(lru_id, [])
        if not profile or not channels:
            ch = self._channels.get(channels[0] if channels else "", {})
            max_w = ch.get("max_draw_w", 30)
            return {"min_draw_w": 2.0, "max_draw_w": max_w, "budget_w": max_w * 0.85}

        targets = [profile.draw_targets.get(ch, 0) for ch in channels]
        budget = round(sum(targets), 1)
        max_w = sum(self._channels.get(ch, {}).get("max_draw_w", 0) for ch in channels)
        return {
            "min_draw_w": round(budget * 0.55, 1),
            "max_draw_w": max_w,
            "budget_w": round(budget * 1.12, 1),
        }

    def apply_startup(self, battery_pct: float, capacity_wh: float) -> tuple[float, dict]:
        if self._startup_applied:
            return battery_pct, {"applied": False, "cost_wh": 0}

        cost = self._cfg["startup_cost_wh"]
        drain_pct = (cost / capacity_wh) * 100 if capacity_wh else 0
        new_pct = max(0.0, round(battery_pct - drain_pct, 2))
        self._startup_applied = True
        info = {"applied": True, "cost_wh": cost, "battery_delta_pct": round(drain_pct, 2)}
        logger.info(f"Startup power cost applied: {cost}Wh ({drain_pct:.2f}% battery)")
        return new_pct, info

    def evaluate(
        self,
        task_id: str,
        lru_states: list[dict],
        total_draw_w: float,
        task_budget_w: float | None = None,
    ) -> dict:
        violations: list[str] = []
        lru_reqs: list[dict] = []
        task_budget = task_budget_w or self.system_budget_w

        reserve = self._cfg["eps_reserve_pct"] / 100.0
        eps_budget = round(task_budget * (1.0 - reserve), 1)
        eps_max = min(self._cfg["system_max_draw_w"], self.system_budget_w)
        eps_min = self._cfg["system_min_draw_w"]

        eps_ok = eps_min <= total_draw_w <= eps_max
        eps_within_budget = total_draw_w <= eps_budget + 0.1
        if not eps_within_budget:
            violations.append(f"EPS over task budget: {total_draw_w:.1f}W > {eps_budget:.1f}W")
        if total_draw_w > eps_max:
            violations.append(f"EPS exceeds max: {total_draw_w:.1f}W > {eps_max:.1f}W")

        eps_entry = {
            "id": "eps",
            "label": "EPS (Power System)",
            "draw_w": round(total_draw_w, 1),
            "min_draw_w": eps_min,
            "max_draw_w": eps_max,
            "budget_w": eps_budget,
            "task_budget_w": round(task_budget, 1),
            "reserve_pct": self._cfg["eps_reserve_pct"],
            "compliant": eps_ok and eps_within_budget,
            "status": "ok" if eps_ok and eps_within_budget else "warning",
        }
        if total_draw_w > eps_max:
            eps_entry["status"] = "fault"

        for lru in lru_states:
            if lru["id"] == "eps":
                continue
            limits = self._task_lru_limits(task_id, lru["id"])
            draw = lru.get("draw_w", 0)
            min_w = limits["min_draw_w"]
            max_w = limits["max_draw_w"]
            budget_w = limits["budget_w"]

            compliant = min_w <= draw <= max_w and draw <= budget_w + 0.1
            status = "ok"
            if draw > max_w or draw > budget_w + 0.1:
                status = "fault"
                violations.append(f"{lru['label']}: {draw:.1f}W exceeds limit ({budget_w:.1f}W budget)")
            elif draw < min_w * 0.85 and task_id not in ("idle",):
                status = "warning"
                violations.append(f"{lru['label']}: {draw:.1f}W below min ({min_w:.1f}W)")
            elif draw > budget_w * 0.92:
                status = "warning"

            lru_reqs.append({
                "id": lru["id"],
                "label": lru["label"],
                "draw_w": draw,
                "min_draw_w": min_w,
                "max_draw_w": max_w,
                "budget_w": budget_w,
                "compliant": compliant,
                "status": status,
                "utilization_pct": lru.get("utilization_pct"),
                "monitor_status": lru.get("status"),
            })

        overall = not violations and eps_entry["compliant"]
        return {
            "task": task_id,
            "startup_cost_wh": self._cfg["startup_cost_wh"],
            "startup_applied": self._startup_applied,
            "overall_compliant": overall,
            "violations": violations,
            "eps": eps_entry,
            "lru_requirements": lru_reqs,
            "task_budget_w": round(task_budget, 1),
        }