"""Numeric trial predictions from the physics / energy model (optimizer post-mortem)."""

from __future__ import annotations

from optimizer.diagnose import Diagnosis

# Hinge rate matches twin_energy straight viscous bill at the commanded wheel speed.
_WHEEL_RADIUS_M = 0.08


def _ref_speed_m_s(before: Diagnosis | None, param: str | None, direction: str) -> float | None:
    summary = None if before is None else before.summary
    if summary is None:
        return None
    cruise = summary.ctrl_cruise_m_s
    straight = summary.ctrl_straight_m_s
    base = cruise
    if param == "controller.straight_m_s" and straight is not None:
        base = straight
    if base is None:
        return None
    try:
        speed = float(base)
    except (TypeError, ValueError):
        return None
    if param in {"controller.cruise_m_s", "controller.straight_m_s"}:
        step = 0.06 if param == "controller.cruise_m_s" else 0.02
        if direction == "up":
            speed += step
        elif direction == "down":
            speed -= step
    return max(0.0, speed)


def numeric_predictions(
    param: str | None,
    direction: str,
    before: Diagnosis | None,
) -> dict[str, str] | None:
    """Return metric → predicted numeric string, or None to fall back to prose."""
    if before is None or before.summary is None or not param:
        return None
    summary = before.summary
    err = summary.err_pct
    if err is None:
        return None
    ref_err = abs(float(err))
    ref_wpm = summary.wh_per_m()
    speed = _ref_speed_m_s(before, param, direction)
    out: dict[str, str] = {}
    if param in {"controller.cruise_m_s", "controller.straight_m_s"} and speed is not None:
        # Viscous shaft power scales ~linearly with hinge rate v/r.
        ratio = speed / max(0.12, float(summary.ctrl_cruise_m_s or 0.44))
        visc = summary.wh_model_visc
        if visc is not None and ref_wpm is not None:
            pred_visc = float(visc) * ratio
            pred_wpm = ref_wpm * (0.7 + 0.3 * ratio)  # blend with efficiency term
            pred_err = ref_err * (0.85 + 0.15 / max(ratio, 0.5)) if float(err) > 0 else ref_err * ratio
            out["err_pct"] = f"{pred_err:+.1f}% (from {ref_err:.1f}% at {speed:.2f} m/s)"
            out["wh_per_m"] = f"{pred_wpm:.4f} Wh/m (was {ref_wpm:.4f})"
            out["visc"] = f"{pred_visc:.3f} Wh model visc (was {visc:.3f})"
            out["duration_s"] = f"{(summary.duration_s or 0) / max(ratio, 0.5):.1f} s (shorter if faster)"
    if param == "controller.pivot_w_max_rad_s" and direction == "down":
        turn = before.turn_err
        if turn is not None:
            pred_turn = abs(float(turn)) * 0.85
            out["turn_err"] = f"{pred_turn:.1f}% segment |err| (was {abs(float(turn)):.1f}%)"
    if param == "controller.k_steer" and direction == "up":
        straight = before.straight_err
        if straight is not None:
            pred_s = abs(float(straight)) * 0.9
            out["straight_err"] = f"{pred_s:.1f}% segment |err| (was {abs(float(straight)):.1f}%)"
    return out or None
