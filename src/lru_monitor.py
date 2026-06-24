"""
LRU-level monitoring — hierarchical power groups with fault detection and degradation.
"""

from __future__ import annotations

from src.config import config
from src.logger import logger

_DEFAULT_LRUS = [
    {
        "id": "locomotion",
        "label": "Locomotion (LRUA)",
        "channels": ["Legs"],
        "degrade_priority": 3,
        "tier": 2,
    },
    {
        "id": "arms",
        "label": "Arms",
        "channels": ["Arms"],
        "degrade_priority": 1,
        "tier": 2,
    },
    {
        "id": "torso",
        "label": "Torso",
        "channels": ["Torso"],
        "degrade_priority": 2,
        "tier": 2,
    },
    {
        "id": "compute",
        "label": "Compute",
        "channels": ["Compute"],
        "degrade_priority": 4,
        "tier": 2,
    },
    {
        "id": "eps",
        "label": "EPS (Power System)",
        "channels": ["Legs", "Arms", "Torso", "Compute"],
        "degrade_priority": 5,
        "tier": 1,
    },
]

_DEFAULT_LRU_CFG = {
    "over_draw_warning_pct": 90,
    "spike_threshold_pct": 30,
    "voltage_sag_factor": 0.18,
    "low_voltage_warning_pct": 0.92,
    "low_voltage_critical_pct": 0.85,
    "degrade_throttle_caution": 0.92,
    "degrade_throttle_degraded": 0.80,
    "degrade_throttle_critical": 0.65,
}


class LRUMonitor:
    def __init__(self, power_channels: list, system_budget_w: float = 72):
        self._channels = {ch["id"]: ch for ch in power_channels}
        self.system_budget_w = system_budget_w
        self._lrus = self._load_lrus()
        self._cfg = self._load_config()
        self._prev_draw: dict[str, float] = {}

    def _load_lrus(self) -> list[dict]:
        custom = config.get("lru") or {}
        groups = custom.get("groups") or _DEFAULT_LRUS
        result = []
        for g in groups:
            channels = g.get("channels", [])
            if g["id"] == "eps":
                max_draw = self.system_budget_w
                nominal_v = 48
            else:
                max_draw = sum(
                    self._channels.get(ch, {}).get("max_draw_w", 0) for ch in channels
                )
                nominal_v = max(
                    (self._channels.get(ch, {}).get("nominal_voltage", 48) for ch in channels),
                    default=48,
                )
            result.append({
                "id": g["id"],
                "label": g.get("label", g["id"]),
                "channels": channels,
                "degrade_priority": g.get("degrade_priority", 2),
                "tier": g.get("tier", 2),
                "max_draw_w": max_draw,
                "nominal_voltage": nominal_v,
            })
        return sorted(result, key=lambda x: (x["tier"], x["id"]))

    def _load_config(self) -> dict:
        cfg = dict(_DEFAULT_LRU_CFG)
        lru_cfg = (config.get("lru") or {}).get("thresholds") or {}
        safety = config.get("safety") or {}
        if safety.get("spike_threshold_pct") is not None:
            cfg["spike_threshold_pct"] = safety["spike_threshold_pct"]
        cfg.update(lru_cfg)
        return cfg

    def hierarchy_summary(self) -> list[dict]:
        return [
            {"id": l["id"], "label": l["label"], "tier": l["tier"], "channels": l["channels"]}
            for l in self._lrus
        ]

    def _estimate_voltage(self, draw_w: float, max_w: float, nominal_v: float) -> float:
        if max_w <= 0:
            return nominal_v
        load_ratio = min(1.2, draw_w / max_w)
        sag = self._cfg["voltage_sag_factor"] * load_ratio
        return round(nominal_v * (1.0 - sag), 2)

    def _lru_status(self, over: bool, spike: bool, low_v: bool, critical_v: bool, util: float) -> str:
        if over or critical_v:
            return "fault"
        if spike or low_v or util >= self._cfg["over_draw_warning_pct"]:
            return "warning"
        return "ok"

    def evaluate(
        self,
        allocated: dict[str, float],
        channel_meta: dict[str, dict] | None = None,
    ) -> dict:
        faults: list[str] = []
        warnings: list[str] = []
        lru_states: list[dict] = []
        spike_lrus: list[str] = []
        over_draw_lrus: list[str] = []
        low_voltage_lrus: list[str] = []

        spike_pct = self._cfg["spike_threshold_pct"] / 100.0
        v_warn = self._cfg["low_voltage_warning_pct"]
        v_crit = self._cfg["low_voltage_critical_pct"]

        for lru in self._lrus:
            lru_id = lru["id"]
            ch_list = lru["channels"]
            draw_w = round(sum(allocated.get(ch, 0.0) for ch in ch_list), 1)
            max_w = lru["max_draw_w"]
            nominal_v = lru["nominal_voltage"]
            prev = self._prev_draw.get(lru_id, 0.0)

            estimated_v = nominal_v if lru_id == "eps" else self._estimate_voltage(draw_w, max_w, nominal_v)
            voltage_pct = 100.0 if lru_id == "eps" else round((estimated_v / nominal_v) * 100, 1) if nominal_v else 100.0
            util_pct = round((draw_w / max_w) * 100, 1) if max_w else 0.0

            over = draw_w > max_w + 0.05
            spike = prev > 1.0 and (draw_w - prev) / prev >= spike_pct
            util_floor = 0.70 if lru_id == "compute" else 0.5
            low_v = lru_id != "eps" and voltage_pct <= v_warn * 100 and draw_w > max_w * util_floor
            crit_v = lru_id != "eps" and voltage_pct <= v_crit * 100

            if over:
                faults.append(f"LRU {lru['label']} over-draw: {draw_w:.1f}W > {max_w}W")
                over_draw_lrus.append(lru_id)
            elif util_pct >= self._cfg["over_draw_warning_pct"]:
                warnings.append(f"LRU {lru['label']} near limit ({util_pct}%)")

            if spike:
                spike_lrus.append(lru_id)
                warnings.append(f"LRU {lru['label']} spike: {prev:.1f}W → {draw_w:.1f}W")

            if crit_v:
                faults.append(
                    f"LRU {lru['label']} low voltage: {estimated_v:.1f}V ({voltage_pct:.0f}% nominal)"
                )
                low_voltage_lrus.append(lru_id)
            elif low_v:
                warnings.append(
                    f"LRU {lru['label']} voltage sag: {estimated_v:.1f}V ({voltage_pct:.0f}% nominal)"
                )
                low_voltage_lrus.append(lru_id)

            status = self._lru_status(over, spike, low_v or crit_v, crit_v, util_pct)
            lru_states.append({
                "id": lru_id,
                "label": lru["label"],
                "tier": lru["tier"],
                "channels": ch_list,
                "draw_w": draw_w,
                "max_draw_w": max_w,
                "utilization_pct": util_pct,
                "nominal_voltage": nominal_v,
                "estimated_voltage": estimated_v,
                "voltage_pct": voltage_pct,
                "status": status,
                "degrade_priority": lru["degrade_priority"],
            })
            self._prev_draw[lru_id] = draw_w

        degradation = "normal"
        if faults:
            degradation = "critical"
        elif over_draw_lrus or spike_lrus:
            degradation = "degraded"
        elif warnings:
            degradation = "caution"

        return {
            "hierarchy": self.hierarchy_summary(),
            "lrus": lru_states,
            "faults": faults,
            "warnings": warnings,
            "spike_lrus": spike_lrus,
            "over_draw_lrus": over_draw_lrus,
            "low_voltage_lrus": low_voltage_lrus,
            "degradation_level": degradation,
        }

    def throttle_factors(self, degradation: str, spike_lrus: list[str]) -> dict[str, float]:
        level_factors = {
            "normal": 1.0,
            "caution": self._cfg["degrade_throttle_caution"],
            "degraded": self._cfg["degrade_throttle_degraded"],
            "critical": self._cfg["degrade_throttle_critical"],
        }
        base = level_factors.get(degradation, 1.0)
        if degradation == "normal":
            return {}

        sorted_lrus = sorted(
            (l for l in self._lrus if l["id"] != "eps"),
            key=lambda x: x["degrade_priority"],
        )
        factors: dict[str, float] = {}
        spike_set = set(spike_lrus)

        for i, lru in enumerate(sorted_lrus):
            if degradation == "caution":
                lru_factor = base if lru["id"] in spike_set else 1.0
            elif degradation == "degraded":
                lru_factor = base if i < 2 else max(base, 0.88)
            else:
                lru_factor = base if i < 3 else max(base, 0.75)

            for ch in lru["channels"]:
                factors[ch] = min(factors.get(ch, 1.0), lru_factor)

        return factors