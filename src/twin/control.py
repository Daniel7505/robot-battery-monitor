"""
Twin control view — PMS + agent status tied to Webots simulation phases.
"""

from __future__ import annotations

from src.mission_tasks import TASK_PROFILES
from src.twin.butlerbot import BUTLERBOT_WALKING_FLOW

TWIN_STRESS_PHASES = frozenset({"walk_transit", "patrol", "manipulate"})


def is_twin_stress_phase(phase: str | None) -> bool:
    return bool(phase and str(phase).lower() in TWIN_STRESS_PHASES)


PHASE_LABELS: dict[str, str] = {
    "standby": "Standby",
    "walk_transit": "Walk / Transit",
    "patrol": "Patrol",
    "manipulate": "Manipulate",
    "return_idle": "Return to Idle",
}


def webots_phase_flow() -> list[dict]:
    """ButlerBot Webots phase timeline for dashboard UI."""
    flow = []
    for step in BUTLERBOT_WALKING_FLOW:
        phase = step["phase"]
        flow.append({
            "phase": phase,
            "label": PHASE_LABELS.get(phase, phase.replace("_", " ").title()),
            "task": step["task"],
            "gait": (step.get("locomotion") or {}).get("gait", "stand"),
            "duration_s": step["duration_s"],
        })
    return flow


def _phase_index(phase: str | None, flow: list[dict]) -> int:
    if not phase:
        return -1
    for i, step in enumerate(flow):
        if step["phase"] == phase:
            return i
    return -1


def _power_influence_summary(allocation: dict, agent: dict) -> str:
    parts: list[str] = []
    status = allocation.get("status", "ok")
    throttled = allocation.get("throttled_channels") or []
    if status == "throttled" and throttled:
        parts.append(f"PMS throttling {', '.join(throttled)}")
    elif status == "warning":
        parts.append("PMS allocation warning")
    applied = agent.get("applied_actions") or []
    if applied:
        parts.append(f"Agent applied: {', '.join(applied)}")
    recs = agent.get("recommendations") or []
    top = recs[0] if recs else None
    if top and not applied:
        action = top.get("action", "")
        if action == "throttle_system":
            factor = top.get("factor")
            parts.append(
                f"Agent recommends system throttle ×{int((factor or 1) * 100)}%"
                if factor
                else "Agent recommends system throttle"
            )
        elif action == "throttle_channel" and top.get("channel"):
            factor = top.get("factor")
            parts.append(
                f"Agent → throttle {top['channel']}"
                + (f" ×{int((factor or 1) * 100)}%" if factor else "")
            )
        elif action == "suggest_task" and top.get("task"):
            parts.append(f"Agent → suggest task {top['task']}")
        elif action == "safety_alert":
            parts.append("Agent safety alert active")
    if not parts:
        if agent.get("controlling"):
            return "Agent monitoring — no throttle applied"
        return "Nominal — PMS tracking twin telemetry"
    return " · ".join(parts)


def build_twin_control_status(bridge, hardware) -> dict:
    """Snapshot for dashboard: Webots phase, PMS task, agent influence."""
    flow = webots_phase_flow()
    twin_status = bridge.status() if bridge else {}
    external = bool(twin_status.get("external_active"))
    tel = getattr(bridge, "_last_telemetry", None) if bridge else None

    locomotion = (tel.locomotion if tel else {}) or {}
    phase = locomotion.get("phase") or locomotion.get("phase_name")
    gait = locomotion.get("gait", "stand")
    speed = locomotion.get("speed_m_s")
    pose = (tel.pose if tel else {}) or {}

    allocation = getattr(hardware, "allocation_status", {}) or {}
    mission = getattr(hardware, "mission_info", {}) or {}
    agent = getattr(hardware, "agent_status", {}) or {}

    pms_task = mission.get("task") or allocation.get("task") or "idle"
    profile = TASK_PROFILES.get(pms_task)
    phase_label = PHASE_LABELS.get(str(phase or ""), (phase or "—").replace("_", " ").title())

    intervening = bool(agent.get("intervening") or agent.get("applied_actions"))
    stress = is_twin_stress_phase(phase)

    return {
        "active": external,
        "source": twin_status.get("active_source", "internal"),
        "sim_phase": phase,
        "sim_phase_label": phase_label if phase else "—",
        "stress_phase": stress,
        "gait": gait,
        "speed_m_s": speed,
        "pose": pose,
        "pms_task": pms_task,
        "pms_task_label": mission.get("task_label") or (profile.label if profile else pms_task),
        "allocation_status": allocation.get("status", "ok"),
        "utilization_pct": allocation.get("utilization_pct"),
        "throttled_channels": allocation.get("throttled_channels") or [],
        "agent_posture": agent.get("posture", "normal"),
        "agent_controlling": bool(agent.get("controlling") or agent.get("active")),
        "agent_intervening": intervening,
        "intervention_count": len(agent.get("applied_actions") or []),
        "applied_actions": agent.get("applied_actions") or [],
        "agent_summary": agent.get("summary", ""),
        "power_influence": _power_influence_summary(allocation, agent),
        "phase_flow": flow,
        "active_phase_index": _phase_index(phase, flow),
        "telemetry_count": twin_status.get("telemetry_count", 0),
        "last_telemetry_at": twin_status.get("last_telemetry_at"),
    }