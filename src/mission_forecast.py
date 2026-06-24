"""
Twin mission loop feasibility — can ButlerBot finish the current cycle?
"""

from __future__ import annotations

from src.hardware_profile import (
    get_active_profile,
    normalize_phase_name,
    phase_reference_draw_w,
    phase_reference_duration_s,
)
from src.twin.butlerbot import BUTLERBOT_MISSION_FLOW


def _flow_steps(flow: list[dict] | None = None) -> list[dict]:
    return list(flow or BUTLERBOT_MISSION_FLOW)


def _phase_index(phase: str | None, flow: list[dict]) -> int:
    if not phase:
        return 0
    norm = normalize_phase_name(phase)
    for i, step in enumerate(flow):
        if normalize_phase_name(step.get("phase", "")) == norm:
            return i
        if step.get("phase") == phase:
            return i
    return 0


def _step_wh(step: dict, profile: dict) -> float:
    duration_s = float(
        step.get("duration_s")
        or phase_reference_duration_s(step.get("phase", ""), profile)
    )
    if step.get("channel_draws"):
        draw_w = sum(float(v) for v in step["channel_draws"].values())
    else:
        draw_w = phase_reference_draw_w(step.get("phase", ""), profile)
    return round(draw_w * (duration_s / 3600.0), 3)


def forecast_twin_loop(
    battery_pct: float,
    capacity_wh: float,
    current_phase: str | None = None,
    *,
    phase_elapsed_s: float = 0.0,
    current_draw_w: float = 0.0,
    flow: list[dict] | None = None,
    profile: dict | None = None,
) -> dict:
    """
    Estimate Wh to complete the current mission loop from the active phase onward.

    Returns feasibility score, margin, and per-phase energy breakdown.
    """
    prof = profile or get_active_profile()
    steps = _flow_steps(flow)
    if not steps:
        return {"ok": False, "reason": "no_mission_flow"}

    energy_wh = round((battery_pct / 100.0) * capacity_wh, 2)
    start_idx = _phase_index(current_phase, steps)

    breakdown: list[dict] = []
    loop_wh = 0.0

    for i in range(start_idx, len(steps)):
        step = steps[i]
        phase = step.get("phase", "")
        duration_s = float(
            step.get("duration_s")
            or phase_reference_duration_s(phase, prof)
        )
        if step.get("channel_draws"):
            draw_w = round(sum(float(v) for v in step["channel_draws"].values()), 1)
        else:
            draw_w = round(phase_reference_draw_w(phase, prof), 1)

        if i == start_idx and phase_elapsed_s > 0:
            remaining_s = max(0.0, duration_s - phase_elapsed_s)
            if current_draw_w > 0:
                draw_w = round(current_draw_w, 1)
        else:
            remaining_s = duration_s

        wh = round(draw_w * (remaining_s / 3600.0), 3)
        loop_wh += wh
        breakdown.append({
            "phase": phase,
            "label": step.get("label") or phase.replace("_", " ").title(),
            "duration_s": round(remaining_s, 1),
            "expected_draw_w": draw_w,
            "energy_wh": wh,
        })

    # Full loop (one more pass) for repeat missions
    full_loop_wh = round(sum(_step_wh(s, prof) for s in steps), 3)

    margin_wh = round(energy_wh - loop_wh, 2)
    margin_pct = round((margin_wh / capacity_wh) * 100, 1) if capacity_wh else 0.0
    finish_battery_pct = round(battery_pct - (loop_wh / capacity_wh) * 100, 1) if capacity_wh else 0.0

    can_complete = energy_wh >= loop_wh * 1.08 if loop_wh > 0 else True
    can_repeat = energy_wh >= full_loop_wh * 1.1

    if margin_pct >= 25:
        status = "comfortable"
    elif margin_pct >= 12:
        status = "tight"
    elif can_complete:
        status = "marginal"
    else:
        status = "insufficient"

    return {
        "ok": True,
        "current_phase": current_phase,
        "start_phase_index": start_idx,
        "energy_wh_remaining": energy_wh,
        "loop_wh_remaining": round(loop_wh, 3),
        "full_loop_wh": full_loop_wh,
        "margin_wh": margin_wh,
        "margin_pct": margin_pct,
        "finish_battery_pct": finish_battery_pct,
        "can_complete_loop": can_complete,
        "can_repeat_loop": can_repeat,
        "feasibility_status": status,
        "phase_breakdown": breakdown,
        "hardware_profile": prof.get("profile_id", "unknown"),
    }