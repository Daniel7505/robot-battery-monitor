"""
Twin phase context — align LRU, requirements, and throttles with ButlerBot mission.

When Webots reports drive_transit, arms/torso are intentionally idle; safety
should not flag them or throttle them like a walking biped.
"""

from __future__ import annotations

from src.hardware_profile import normalize_phase_name
from src.twin.butlerbot import BUTLERBOT_MISSION_FLOW

# PMS task the twin phase normally runs (intentional mismatch is OK)
_PHASE_EXPECTED_TASK: dict[str, str] = {
    step["phase"]: step["task"]
    for step in BUTLERBOT_MISSION_FLOW
}
# walk_transit alias
_PHASE_EXPECTED_TASK["walk_transit"] = _PHASE_EXPECTED_TASK.get("drive_transit", "moving")

# LRU ids in standby for each twin phase (low draw is expected)
_PHASE_STANDBY_LRUS: dict[str, frozenset[str]] = {
    "standby": frozenset({"locomotion", "arms", "torso", "cooling"}),
    "drive_transit": frozenset({"arms", "torso"}),
    "walk_transit": frozenset({"arms", "torso"}),
    "patrol": frozenset({"arms"}),
    "manipulate": frozenset({"locomotion"}),
    "return_idle": frozenset({"arms", "torso", "cooling"}),
}

_PHASE_PRIMARY_LRUS: dict[str, frozenset[str]] = {
    "standby": frozenset({"compute", "eps"}),
    "drive_transit": frozenset({"locomotion", "compute", "cooling"}),
    "walk_transit": frozenset({"locomotion", "compute", "cooling"}),
    "patrol": frozenset({"locomotion", "compute", "cooling"}),
    "manipulate": frozenset({"arms", "torso", "compute", "cooling"}),
    "return_idle": frozenset({"locomotion", "compute"}),
}

_PHASE_LABELS: dict[str, str] = {
    "standby": "Standby — compute + sensors active",
    "drive_transit": "Wheeled transit — locomotion primary, arms/torso tucked",
    "walk_transit": "Wheeled transit — locomotion primary, arms/torso tucked",
    "patrol": "Patrol — locomotion primary, arms low",
    "manipulate": "Manipulation — arms/torso + cooling primary, base idle",
    "return_idle": "Return — locomotion only, arms/torso idle",
}


def expected_task_for_phase(phase: str | None) -> str | None:
    key = phase_key(phase)
    return _PHASE_EXPECTED_TASK.get(key)


def task_phase_alignment(phase: str | None, pms_task: str | None) -> dict:
    """Whether PMS task matches twin phase expectation (mismatch can be normal)."""
    expected = expected_task_for_phase(phase)
    if not phase or not expected or not pms_task:
        return {"aligned": True, "expected_task": expected, "note": ""}
    aligned = pms_task == expected
    note = ""
    if not aligned:
        note = (
            f"PMS task is '{pms_task}' while Webots phase '{phase_key(phase)}' "
            f"usually runs '{expected}' — allocation follows PMS; LRU follows twin phase."
        )
    return {
        "aligned": aligned,
        "expected_task": expected,
        "pms_task": pms_task,
        "twin_phase": phase,
        "note": note,
    }


def phase_key(phase: str | None) -> str:
    if not phase:
        return ""
    return normalize_phase_name(phase)


def standby_lrus(phase: str | None) -> frozenset[str]:
    return _PHASE_STANDBY_LRUS.get(phase_key(phase), frozenset())


def primary_lrus(phase: str | None) -> frozenset[str]:
    return _PHASE_PRIMARY_LRUS.get(phase_key(phase), frozenset())


def context_summary(phase: str | None, task_id: str | None = None) -> dict:
    key = phase_key(phase)
    return {
        "twin_phase": phase,
        "phase_key": key,
        "task_id": task_id,
        "summary": _PHASE_LABELS.get(key, "Mission context unknown"),
        "standby_lrus": sorted(standby_lrus(phase)),
        "primary_lrus": sorted(primary_lrus(phase)),
    }


def is_standby_lru(lru_id: str, phase: str | None) -> bool:
    return lru_id in standby_lrus(phase)


def filter_lru_result(lru_result: dict, phase: str | None) -> dict:
    """Mark standby LRUs and suppress misleading warnings."""
    if not phase:
        return lru_result

    standby = standby_lrus(phase)
    if not standby:
        return lru_result

    out = dict(lru_result)
    faults = list(out.get("faults") or [])
    warnings = list(out.get("warnings") or [])
    lrus = []

    for lru in out.get("lrus") or []:
        entry = dict(lru)
        lid = entry.get("id", "")
        if lid in standby:
            entry["mission_role"] = "standby"
            entry["status"] = "standby" if entry.get("status") in ("ok", "warning") else entry.get("status")
            # Drop voltage-sag noise when LRU is intentionally idle
            if entry.get("utilization_pct", 0) < 35:
                warnings = [
                    w for w in warnings
                    if lid not in w and entry.get("label", "") not in w
                ]
        else:
            entry["mission_role"] = "active"
        lrus.append(entry)

    out["lrus"] = lrus
    out["faults"] = faults
    out["warnings"] = warnings
    out["mission_context"] = context_summary(phase)
    out["standby_lrus"] = sorted(standby)

    # Recompute degradation without standby LRU warnings
    if faults:
        out["degradation_level"] = "critical"
    elif any(l.get("status") == "fault" for l in lrus):
        out["degradation_level"] = "degraded"
    elif any(l.get("status") == "warning" and l.get("mission_role") == "active" for l in lrus):
        out["degradation_level"] = "caution"
    else:
        out["degradation_level"] = "normal"

    return out


def filter_requirements(req_result: dict, phase: str | None) -> dict:
    """Don't penalize idle LRUs for being below min draw during transit."""
    if not phase:
        return req_result

    standby = standby_lrus(phase)
    out = dict(req_result)
    violations = list(out.get("violations") or [])
    lru_reqs = []

    for req in out.get("lru_requirements") or []:
        entry = dict(req)
        lid = entry.get("id", "")
        draw = float(entry.get("draw_w", 0))
        min_w = float(entry.get("min_draw_w", 0))

        if lid in standby and draw < min_w * 0.85:
            entry["mission_role"] = "standby"
            entry["compliant"] = True
            entry["status"] = "standby"
            violations = [
                v for v in violations
                if entry.get("label", "") not in v and "below min" not in v.lower()
            ]
        else:
            entry["mission_role"] = "active"
        lru_reqs.append(entry)

    out["lru_requirements"] = lru_reqs
    out["violations"] = violations
    out["overall_compliant"] = not violations and out.get("eps", {}).get("compliant", True)
    out["mission_context"] = context_summary(phase, out.get("task"))
    return out


def throttle_exempt_channels(phase: str | None) -> frozenset[str]:
    """Channels that should not be throttled during this phase (intentionally idle)."""
    standby = standby_lrus(phase)
    mapping = {
        "locomotion": "Legs",
        "arms": "Arms",
        "torso": "Torso",
        "compute": "Compute",
        "cooling": "Cooling",
    }
    return frozenset(mapping[lid] for lid in standby if lid in mapping)