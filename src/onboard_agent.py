"""
Onboard AI agent — reactive rule-based decision engine.

Evaluates live telemetry each tick and produces throttle suggestions, task-change
recommendations, and safety alerts. Designed for modular upgrade to richer logic.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable

from src.config import config
from src.logger import logger
from src.mission_context import context_summary, is_standby_lru, throttle_exempt_channels
from src.mission_tasks import TASK_PROFILES

_DEFAULT_AGENT = {
    "enabled": True,
    "auto_apply_throttle": False,
    "auto_apply_task_suggestions": False,
    "twin_auto_apply": True,
    "history_limit": 24,
    "low_battery_suggest_pct": 22,
    "critical_battery_pct": 12,
    "high_utilization_pct": 88,
    "twin_stress_utilization_pct": 65,
    "task_override_hold_s": 12,
    "prediction_risk_tasks": ("high", "critical"),
    "loop_margin_warn_pct": 18,
    "loop_margin_critical_pct": 8,
}

_TWIN_STRESS_PHASES = frozenset({"drive_transit", "walk_transit", "patrol", "manipulate"})

_TASK_DOWNGRADE = {
    "high_load": "balanced",
    "moving": "balanced",
    "balanced": "idle",
    "idle": "idle",
}

_PRIORITY_ORDER = {"info": 0, "low": 1, "medium": 2, "high": 3, "critical": 4}


@dataclass
class AgentContext:
    battery_pct: float
    task_id: str
    total_draw_w: float
    utilization_pct: float
    allocation_status: str
    throttled_channels: list[str]
    safety: dict
    prediction: dict
    mission: dict
    readings: dict[str, dict]
    twin_phase: str | None = None
    twin_gait: str | None = None
    twin_source: str | None = None
    loop_forecast: dict = field(default_factory=dict)


@dataclass
class AgentRecommendation:
    action: str
    priority: str
    reason: str
    channel: str | None = None
    factor: float | None = None
    task: str | None = None
    message: str | None = None
    rule_id: str = ""
    applied: bool = False

    def to_dict(self) -> dict:
        return {
            "action": self.action,
            "priority": self.priority,
            "reason": self.reason,
            "channel": self.channel,
            "factor": self.factor,
            "task": self.task,
            "message": self.message,
            "rule_id": self.rule_id,
            "applied": self.applied,
        }

    def signature(self) -> str:
        parts = [self.action, self.rule_id, self.channel or "", self.task or "", self.message or ""]
        if self.factor is not None:
            parts.append(f"{self.factor:.2f}")
        return "|".join(parts)


@dataclass
class AgentResult:
    posture: str = "normal"
    summary: str = "All systems nominal"
    recommendations: list[AgentRecommendation] = field(default_factory=list)
    rules_fired: list[str] = field(default_factory=list)
    applied_actions: list[str] = field(default_factory=list)

    def to_status(self, enabled: bool, recent_log: list[dict]) -> dict:
        recs = sorted(
            self.recommendations,
            key=lambda r: _PRIORITY_ORDER.get(r.priority, 0),
            reverse=True,
        )
        controlling = enabled and (bool(recs) or bool(self.applied_actions))
        intervening = enabled and (
            bool(self.applied_actions)
            or any(r.get("applied") for r in (rec.to_dict() for rec in recs))
            or any(r.priority in ("high", "critical") for r in recs)
        )
        return {
            "enabled": enabled,
            "active": enabled and bool(recs),
            "controlling": controlling,
            "intervening": intervening,
            "intervention_count": len(self.applied_actions),
            "posture": self.posture,
            "summary": self.summary,
            "recommendations": [r.to_dict() for r in recs],
            "recommendation_count": len(recs),
            "rules_fired": self.rules_fired,
            "applied_actions": self.applied_actions,
            "recent_log": recent_log,
            "action_log": recent_log,
        }


RuleFn = Callable[[AgentContext, dict], list[AgentRecommendation]]


class OnboardAgent:
    """Rule-based onboard agent with pluggable decision rules."""

    def __init__(self):
        self._cfg = self._load_config()
        self._rules: list[tuple[str, RuleFn]] = []
        self._recent_log: list[dict] = []
        self._last_signatures: set[str] = set()
        self._last_twin_phase: str | None = None
        self._register_default_rules()

    def _load_config(self) -> dict:
        cfg = dict(_DEFAULT_AGENT)
        agent = config.get("agent") or {}
        for key, default in _DEFAULT_AGENT.items():
            if key in agent:
                cfg[key] = agent[key]
        return cfg

    @property
    def enabled(self) -> bool:
        return bool(self._cfg.get("enabled", True))

    @property
    def auto_apply_throttle(self) -> bool:
        return bool(self._cfg.get("auto_apply_throttle", False))

    @property
    def auto_apply_task_suggestions(self) -> bool:
        return bool(self._cfg.get("auto_apply_task_suggestions", False))

    @property
    def twin_auto_apply(self) -> bool:
        return bool(self._cfg.get("twin_auto_apply", True))

    def should_auto_apply(self, twin_active: bool) -> bool:
        if not self.enabled:
            return False
        if twin_active and self.twin_auto_apply:
            return True
        return self.auto_apply_throttle

    def register_rule(self, rule_id: str, fn: RuleFn) -> None:
        self._rules.append((rule_id, fn))

    def _register_default_rules(self) -> None:
        self._rules = [
            ("critical_battery", self._rule_critical_battery),
            ("low_battery", self._rule_low_battery),
            ("prediction_risk", self._rule_prediction_risk),
            ("mission_energy", self._rule_mission_energy),
            ("thermal", self._rule_thermal),
            ("lru_degraded", self._rule_lru_degraded),
            ("high_utilization", self._rule_high_utilization),
            ("power_spike", self._rule_power_spike),
            ("safety_mirror", self._rule_safety_mirror),
            ("twin_stress", self._rule_twin_stress),
            ("loop_forecast", self._rule_loop_forecast),
            ("negotiator", self._rule_negotiator),
        ]

    def evaluate(
        self,
        battery_pct: float,
        task_id: str,
        allocation: dict,
        safety: dict,
        prediction: dict,
        mission: dict,
        readings: dict[str, dict],
        *,
        twin_context: dict | None = None,
    ) -> AgentResult:
        if not self.enabled:
            return AgentResult(posture="disabled", summary="Agent disabled")

        twin = twin_context or {}
        ctx = AgentContext(
            battery_pct=battery_pct,
            task_id=task_id,
            total_draw_w=allocation.get("total_allocated_w", 0),
            utilization_pct=allocation.get("utilization_pct", 0),
            allocation_status=allocation.get("status", "ok"),
            throttled_channels=list(allocation.get("throttled_channels") or []),
            safety=safety,
            prediction=prediction,
            mission=mission,
            readings=readings,
            twin_phase=twin.get("phase"),
            twin_gait=twin.get("gait"),
            twin_source=twin.get("source"),
            loop_forecast=dict(mission.get("loop_forecast") or {}),
        )

        recommendations: list[AgentRecommendation] = []
        rules_fired: list[str] = []

        for rule_id, rule_fn in self._rules:
            try:
                recs = rule_fn(ctx, self._cfg)
            except Exception as exc:
                logger.warning(f"OnboardAgent rule {rule_id} failed: {exc}")
                continue
            if recs:
                rules_fired.append(rule_id)
                for rec in recs:
                    rec.rule_id = rule_id
                recommendations.extend(recs)

        recommendations = self._phase_filter_recommendations(recommendations, ctx)
        recommendations = self._dedupe_recommendations(recommendations)
        posture, summary = self._derive_posture(ctx, recommendations)
        result = AgentResult(
            posture=posture,
            summary=summary,
            recommendations=recommendations,
            rules_fired=rules_fired,
        )
        self._log_decisions(result, ctx)
        return result

    def record_event(
        self,
        action: str,
        detail: str,
        *,
        priority: str = "info",
        influence: str = "system",
        sim_phase: str | None = None,
        gait: str | None = None,
        pms_task: str | None = None,
        rule_id: str = "",
        applied: bool = False,
    ) -> None:
        """Append a twin-linked control event to the action log."""
        ts = datetime.now().strftime("%H:%M:%S")
        entry = {
            "time": ts,
            "action": action,
            "detail": detail,
            "priority": priority,
            "influence": influence,
            "sim_phase": sim_phase,
            "gait": gait,
            "pms_task": pms_task,
            "rule_id": rule_id,
            "applied": applied,
        }
        self._append_log(entry)
        logger.info(f"OnboardAgent [{influence}] {action}: {detail}")

    def record_phase_change(
        self,
        phase: str,
        gait: str | None,
        pms_task: str | None,
    ) -> None:
        if not phase or phase == self._last_twin_phase:
            return
        self._last_twin_phase = phase
        label = phase.replace("_", " ").title()
        self.record_event(
            "phase_change",
            f"Webots phase → {label}"
            + (f" (gait {gait})" if gait else "")
            + (f" · PMS task {pms_task}" if pms_task else ""),
            priority="info",
            influence="twin",
            sim_phase=phase,
            gait=gait,
            pms_task=pms_task,
        )

    def record_pms_influence(
        self,
        detail: str,
        *,
        sim_phase: str | None = None,
        gait: str | None = None,
        pms_task: str | None = None,
        priority: str = "medium",
    ) -> None:
        self.record_event(
            "pms_throttle",
            detail,
            priority=priority,
            influence="pms",
            sim_phase=sim_phase,
            gait=gait,
            pms_task=pms_task,
            applied=True,
        )

    def _phase_exempt_channels(self, ctx: AgentContext) -> frozenset[str]:
        if not ctx.twin_phase:
            return frozenset()
        return throttle_exempt_channels(ctx.twin_phase)

    def _lru_is_standby(self, lru: dict, phase: str | None) -> bool:
        if lru.get("mission_role") == "standby":
            return True
        return is_standby_lru(lru.get("id", ""), phase)

    def _phase_filter_recommendations(
        self, recs: list[AgentRecommendation], ctx: AgentContext
    ) -> list[AgentRecommendation]:
        exempt = self._phase_exempt_channels(ctx)
        if not exempt:
            return recs
        return [
            rec for rec in recs
            if not (rec.action == "throttle_channel" and rec.channel in exempt)
        ]

    def _dedupe_recommendations(self, recs: list[AgentRecommendation]) -> list[AgentRecommendation]:
        seen: set[str] = set()
        unique: list[AgentRecommendation] = []
        for rec in recs:
            key = rec.signature()
            if key in seen:
                continue
            seen.add(key)
            unique.append(rec)
        return unique

    def _derive_posture(
        self, ctx: AgentContext, recs: list[AgentRecommendation]
    ) -> tuple[str, str]:
        priorities = {r.priority for r in recs}
        if "critical" in priorities or ctx.battery_pct <= self._cfg["critical_battery_pct"]:
            summary = next(
                (r.reason for r in recs if r.priority == "critical"),
                f"Critical battery at {ctx.battery_pct:.1f}%",
            )
            return "critical", summary
        if "high" in priorities or ctx.safety.get("status") == "fault":
            summary = next(
                (r.reason for r in recs if r.priority == "high"),
                "Elevated risk — review recommendations",
            )
            return "cautious", summary
        if recs:
            return "advisory", recs[0].reason
        if ctx.twin_phase:
            mctx = context_summary(ctx.twin_phase, ctx.task_id)
            return "normal", mctx.get("summary", "Twin mission in progress")
        return "normal", "All systems nominal — no agent actions required"

    def _log_decisions(self, result: AgentResult, ctx: AgentContext) -> None:
        new_sigs = {r.signature() for r in result.recommendations}
        if new_sigs == self._last_signatures and result.posture == "normal":
            return
        self._last_signatures = new_sigs

        ts = datetime.now().strftime("%H:%M:%S")
        twin_tag = ""
        if ctx.twin_phase:
            twin_tag = f" @ {ctx.twin_phase.replace('_', ' ')}"

        if not result.recommendations:
            if result.posture == "normal":
                return
            entry = {
                "time": ts,
                "action": "status",
                "detail": result.summary + twin_tag,
                "priority": result.posture,
                "influence": "agent",
                "sim_phase": ctx.twin_phase,
                "gait": ctx.twin_gait,
                "pms_task": ctx.task_id,
            }
            self._append_log(entry)
            logger.info(f"OnboardAgent [{result.posture}] {result.summary}")
            return

        for rec in result.recommendations:
            detail = rec.reason
            if rec.channel and rec.factor is not None:
                detail = f"{rec.channel} throttle {rec.factor:.0%} — {rec.reason}"
            elif rec.task:
                detail = f"Suggest task → {rec.task} — {rec.reason}"
            elif rec.message:
                detail = rec.message
            if twin_tag:
                detail += twin_tag

            entry = {
                "time": ts,
                "action": rec.action,
                "detail": detail,
                "priority": rec.priority,
                "rule_id": rec.rule_id,
                "influence": "agent",
                "sim_phase": ctx.twin_phase,
                "gait": ctx.twin_gait,
                "pms_task": ctx.task_id,
                "applied": rec.applied,
            }
            self._append_log(entry)
            logger.info(f"OnboardAgent [{rec.priority}] {rec.action}: {detail}")

    def _append_log(self, entry: dict) -> None:
        self._recent_log.insert(0, entry)
        limit = int(self._cfg.get("history_limit", 24))
        if len(self._recent_log) > limit:
            self._recent_log = self._recent_log[:limit]

    def status_dict(self, result: AgentResult) -> dict:
        return result.to_status(self.enabled, list(self._recent_log))

    def apply_throttle(
        self, allocated: dict[str, float], result: AgentResult
    ) -> tuple[dict[str, float], list[str]]:
        if not self.auto_apply_throttle:
            return allocated, []

        applied: list[str] = []
        out = dict(allocated)
        system_factor = 1.0

        for rec in result.recommendations:
            if rec.action == "throttle_system" and rec.factor is not None:
                system_factor = min(system_factor, rec.factor)
            elif rec.action == "throttle_channel" and rec.channel and rec.factor is not None:
                ch = rec.channel
                if ch in out:
                    out[ch] = round(out[ch] * rec.factor, 1)
                    rec.applied = True
                    applied.append(f"{ch} ×{rec.factor:.0%}")

        if system_factor < 1.0:
            for ch_id in out:
                out[ch_id] = round(out[ch_id] * system_factor, 1)
            applied.append(f"system ×{system_factor:.0%}")
            for rec in result.recommendations:
                if rec.action == "throttle_system":
                    rec.applied = True

        result.applied_actions = applied
        return out, applied

    def apply_task_suggestions(self, mission, result: AgentResult) -> list[str]:
        """Apply high-priority task-downgrade recommendations to the PMS mission."""
        if not self.auto_apply_task_suggestions and not self.twin_auto_apply:
            return []

        applied: list[str] = []
        for rec in result.recommendations:
            if rec.action != "suggest_task" or not rec.task:
                continue
            if rec.priority not in ("critical", "high", "medium"):
                continue
            if hasattr(mission, "force_task") and mission.force_task(rec.task):
                rec.applied = True
                applied.append(f"task→{rec.task}")
        if applied:
            result.applied_actions = list(result.applied_actions) + applied
        return applied

    # --- Default rules (modular — register custom rules via register_rule) ---

    def _rule_twin_stress(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        phase = (ctx.twin_phase or "").lower()
        if phase not in _TWIN_STRESS_PHASES:
            return []
        threshold = cfg.get("twin_stress_utilization_pct", 65)
        if ctx.utilization_pct < threshold:
            return []

        label = phase.replace("_", " ").title()
        recs: list[AgentRecommendation] = [
            AgentRecommendation(
                action="safety_alert",
                priority="high",
                reason=f"Webots {label} phase stressing power budget ({ctx.utilization_pct:.0f}%)",
                message=f"TWIN STRESS — {label} draw elevated, agent intervening",
            ),
            AgentRecommendation(
                action="throttle_system",
                priority="high",
                reason=f"Twin phase {label} exceeds comfortable utilization",
                factor=0.80,
            ),
        ]
        if phase in ("drive_transit", "walk_transit"):
            recs.append(
                AgentRecommendation(
                    action="throttle_channel",
                    priority="high",
                    reason="Locomotion surge during wheeled transit",
                    channel="Legs",
                    factor=0.75,
                )
            )
        elif phase == "manipulate":
            recs.append(
                AgentRecommendation(
                    action="throttle_channel",
                    priority="high",
                    reason="Manipulation peak load on arms/torso",
                    channel="Arms",
                    factor=0.72,
                )
            )
            recs.append(
                AgentRecommendation(
                    action="throttle_channel",
                    priority="medium",
                    reason="Torso load during manipulation",
                    channel="Torso",
                    factor=0.78,
                )
            )
        elif phase == "patrol" and ctx.task_id == "moving":
            recs.append(
                AgentRecommendation(
                    action="suggest_task",
                    priority="medium",
                    reason="Patrol weave + high utilization",
                    task="balanced",
                )
            )
        return recs

    def _rule_loop_forecast(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        fc = ctx.loop_forecast
        if not fc or not fc.get("ok"):
            return []
        if ctx.twin_source != "webots" and not ctx.twin_phase:
            return []

        margin_pct = float(fc.get("margin_pct", 100))
        finish_pct = float(fc.get("finish_battery_pct", 100))
        warn_pct = cfg.get("loop_margin_warn_pct", 18)
        crit_pct = cfg.get("loop_margin_critical_pct", 8)

        if margin_pct >= warn_pct and fc.get("can_complete_loop", True):
            return []

        loop_wh = fc.get("loop_wh_remaining", 0)
        energy_wh = fc.get("energy_wh_remaining", 0)
        status = fc.get("feasibility_status", "unknown")

        if not fc.get("can_complete_loop", True) or margin_pct < crit_pct:
            priority = "critical"
            msg = (
                f"LOOP RISK — need {loop_wh:.2f} Wh, have {energy_wh:.2f} Wh "
                f"(finish ~{finish_pct:.0f}%)"
            )
        else:
            priority = "high"
            msg = (
                f"LOOP TIGHT — {loop_wh:.2f} Wh to finish cycle, "
                f"~{finish_pct:.0f}% battery expected ({status})"
            )

        recs = [
            AgentRecommendation(
                action="safety_alert",
                priority=priority,
                reason=f"Mission loop energy {status} (margin {margin_pct:.0f}%)",
                message=msg,
            ),
        ]
        if not fc.get("can_complete_loop", True):
            recs.append(
                AgentRecommendation(
                    action="suggest_task",
                    priority="critical",
                    reason="Insufficient energy to complete twin mission loop",
                    task="idle",
                )
            )
            recs.append(
                AgentRecommendation(
                    action="throttle_system",
                    priority="critical",
                    reason="Protect battery for safe return",
                    factor=0.68,
                )
            )
        return recs

    def _rule_negotiator(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        """Trade time vs energy when loop margin is tight but completion is possible."""
        fc = ctx.loop_forecast
        if not fc or not fc.get("ok") or not fc.get("can_complete_loop", True):
            return []

        margin_pct = float(fc.get("margin_pct", 100))
        warn_pct = cfg.get("loop_margin_warn_pct", 18)
        if margin_pct >= warn_pct:
            return []

        phase = (ctx.twin_phase or "").lower()
        recs: list[AgentRecommendation] = []

        if phase in ("drive_transit", "walk_transit") and ctx.task_id == "moving":
            recs.append(
                AgentRecommendation(
                    action="suggest_task",
                    priority="medium",
                    reason=(
                        f"Loop margin {margin_pct:.0f}% — slower patrol saves ~25% locomotion Wh"
                    ),
                    task="balanced",
                    message="NEGOTIATE — switch to balanced patrol to preserve loop headroom",
                )
            )
            recs.append(
                AgentRecommendation(
                    action="throttle_channel",
                    priority="medium",
                    reason="Ease transit draw while keeping mission on track",
                    channel="Legs",
                    factor=0.88,
                )
            )
        elif phase == "manipulate" and margin_pct < warn_pct * 0.7:
            recs.append(
                AgentRecommendation(
                    action="suggest_task",
                    priority="high",
                    reason=f"Only {margin_pct:.0f}% loop margin — skip heavy manipulation",
                    task="balanced",
                    message="NEGOTIATE — defer manipulate, proceed to return_idle",
                )
            )
            recs.append(
                AgentRecommendation(
                    action="throttle_channel",
                    priority="high",
                    reason="Reduce arm/torso peaks to protect finish margin",
                    channel="Arms",
                    factor=0.78,
                )
            )
        elif phase == "patrol" and margin_pct < warn_pct:
            recs.append(
                AgentRecommendation(
                    action="throttle_system",
                    priority="medium",
                    reason=f"Patrol weave costly with {margin_pct:.0f}% margin",
                    factor=0.9,
                    message="NEGOTIATE — gentle system throttle during patrol",
                )
            )

        if margin_pct < cfg.get("loop_margin_critical_pct", 8) * 1.5:
            recs.append(
                AgentRecommendation(
                    action="safety_alert",
                    priority="high",
                    reason="Agent negotiating time vs energy tradeoff",
                    message=(
                        f"Finish forecast {fc.get('finish_battery_pct', '?')}% — "
                        "agent favoring energy over speed"
                    ),
                )
            )
        return recs

    def _rule_critical_battery(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        if ctx.battery_pct > cfg["critical_battery_pct"]:
            return []
        return [
            AgentRecommendation(
                action="safety_alert",
                priority="critical",
                reason=f"Battery critically low ({ctx.battery_pct:.1f}%)",
                message="CRITICAL — Return to charger or switch to idle immediately",
            ),
            AgentRecommendation(
                action="suggest_task",
                priority="critical",
                reason="Battery cannot sustain current mission load",
                task="idle",
            ),
            AgentRecommendation(
                action="throttle_system",
                priority="critical",
                reason="Protect remaining capacity",
                factor=0.70,
            ),
        ]

    def _rule_low_battery(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        warn = cfg["low_battery_suggest_pct"]
        if ctx.battery_pct > warn or ctx.battery_pct <= cfg["critical_battery_pct"]:
            return []
        suggested = _TASK_DOWNGRADE.get(ctx.task_id, "idle")
        recs = [
            AgentRecommendation(
                action="safety_alert",
                priority="high",
                reason=f"Battery low ({ctx.battery_pct:.1f}%)",
                message=f"LOW BATTERY — Consider lighter task ({suggested})",
            ),
            AgentRecommendation(
                action="throttle_system",
                priority="high",
                reason="Reduce draw to extend runtime",
                factor=0.88,
            ),
        ]
        if ctx.task_id != suggested:
            recs.append(
                AgentRecommendation(
                    action="suggest_task",
                    priority="high",
                    reason="Conserve energy for safe return",
                    task=suggested,
                )
            )
        return recs

    def _rule_prediction_risk(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        risk = (ctx.prediction.get("risk_level") or "low").lower()
        if risk not in cfg.get("prediction_risk_tasks", ("high", "critical")):
            return []
        suggested = _TASK_DOWNGRADE.get(ctx.task_id, "idle")
        recs = [
            AgentRecommendation(
                action="safety_alert",
                priority="high" if risk == "high" else "critical",
                reason=f"Energy forecast risk: {risk.upper()}",
                message=f"Forecast risk {risk.upper()} — draw trend unfavorable",
            ),
        ]
        if ctx.task_id != suggested:
            recs.append(
                AgentRecommendation(
                    action="suggest_task",
                    priority="medium",
                    reason="Prediction indicates marginal mission energy",
                    task=suggested,
                )
            )
        if ctx.twin_source == "webots":
            recs.append(
                AgentRecommendation(
                    action="throttle_system",
                    priority="high",
                    reason="Webots twin under forecast risk — proactive throttle",
                    factor=0.84,
                )
            )
        return recs

    def _rule_mission_energy(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        if ctx.prediction.get("mission_energy_ok") is not False:
            return []
        suggested = _TASK_DOWNGRADE.get(ctx.task_id, "idle")
        recs = [
            AgentRecommendation(
                action="safety_alert",
                priority="medium",
                reason="Mission may not complete on remaining energy",
                message="Mission energy marginal — allocator reduced budget",
            ),
        ]
        if ctx.task_id != suggested:
            recs.append(
                AgentRecommendation(
                    action="suggest_task",
                    priority="medium",
                    reason="Insufficient energy headroom for current task",
                    task=suggested,
                )
            )
        return recs

    def _rule_thermal(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        thermal = ctx.safety.get("thermal_status", "normal")
        temp = ctx.safety.get("thermal_c", 0)
        if thermal == "normal":
            return []
        factor = 0.82 if thermal == "warning" else 0.72
        priority = "high" if thermal == "warning" else "critical"
        return [
            AgentRecommendation(
                action="safety_alert",
                priority=priority,
                reason=f"Thermal {thermal} ({temp}°C)",
                message=f"THERMAL {thermal.upper()} — {temp}°C estimated",
            ),
            AgentRecommendation(
                action="throttle_system",
                priority=priority,
                reason="Cooling headroom limited",
                factor=factor,
            ),
        ]

    def _rule_lru_degraded(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        lrus = (ctx.safety.get("lru") or {}).get("lrus") or []
        recs: list[AgentRecommendation] = []
        for lru in lrus:
            if self._lru_is_standby(lru, ctx.twin_phase):
                continue
            status = lru.get("status", "ok")
            if status in ("standby", "ok"):
                continue
            if status not in ("warning", "fault"):
                continue
            channels = lru.get("channels") or []
            factor = 0.80 if status == "warning" else 0.68
            priority = "medium" if status == "warning" else "high"
            for ch in channels:
                if ch == "Compute" and status == "warning":
                    continue
                recs.append(
                    AgentRecommendation(
                        action="throttle_channel",
                        priority=priority,
                        reason=f"{lru.get('label', lru.get('id'))} {status}",
                        channel=ch,
                        factor=factor,
                    )
                )
        return recs

    def _rule_high_utilization(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        if ctx.utilization_pct < cfg["high_utilization_pct"]:
            return []
        profile = TASK_PROFILES.get(ctx.task_id)
        order = list(profile.throttle_order) if profile else []
        exempt = self._phase_exempt_channels(ctx)
        order = [ch for ch in order if ch not in exempt]
        recs: list[AgentRecommendation] = []
        factor = 0.82 if ctx.twin_source == "webots" else 0.90
        priority = "high" if ctx.utilization_pct >= 80 else "medium"
        for ch in order[:2]:
            if ch in ctx.throttled_channels:
                continue
            recs.append(
                AgentRecommendation(
                    action="throttle_channel",
                    priority=priority,
                    reason=f"Budget utilization {ctx.utilization_pct:.0f}%",
                    channel=ch,
                    factor=factor,
                )
            )
        if not recs:
            recs.append(
                AgentRecommendation(
                    action="throttle_system",
                    priority=priority,
                    reason=f"System utilization {ctx.utilization_pct:.0f}%",
                    factor=0.86 if ctx.twin_source == "webots" else 0.92,
                )
            )
        return recs

    def _rule_power_spike(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        spikes = ctx.safety.get("spike_channels") or []
        exempt = self._phase_exempt_channels(ctx)
        return [
            AgentRecommendation(
                action="throttle_channel",
                priority="medium",
                reason="Sudden draw spike detected",
                channel=ch,
                factor=0.85,
            )
            for ch in spikes
            if ch not in ctx.throttled_channels and ch not in exempt
        ]

    def _rule_safety_mirror(self, ctx: AgentContext, cfg: dict) -> list[AgentRecommendation]:
        recs: list[AgentRecommendation] = []
        for alert in ctx.safety.get("alerts") or []:
            if "CRITICAL" in alert.upper() or "THERMAL" in alert.upper():
                continue
            recs.append(
                AgentRecommendation(
                    action="safety_alert",
                    priority="high",
                    reason="Safety system alert",
                    message=alert,
                )
            )
        if ctx.safety.get("throttle_required") and ctx.allocation_status != "throttled":
            recs.append(
                AgentRecommendation(
                    action="throttle_system",
                    priority="high",
                    reason=ctx.safety.get("throttle_reason", "Safety limit"),
                    factor=ctx.safety.get("throttle_factor", 0.85),
                )
            )
        return recs