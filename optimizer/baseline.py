"""Kept baseline vs starting reference for scoring and repeat detection."""

from __future__ import annotations

from optimizer.diagnose import MissionSummary
from optimizer.score import lap_metrics_match
from optimizer.trial_validity import LIMIT_TOLERANCE

_CTRL_KEYS = (
    "controller.cruise_m_s",
    "controller.straight_m_s",
    "controller.pivot_w_max_rad_s",
    "controller.k_steer",
)


def _ctrl_from_summary(summary: MissionSummary | None) -> dict[str, float | None]:
    if summary is None:
        return {}
    return {
        "controller.cruise_m_s": summary.ctrl_cruise_m_s,
        "controller.straight_m_s": summary.ctrl_straight_m_s,
        "controller.pivot_w_max_rad_s": summary.ctrl_pivot_w_max_rad_s,
        "controller.k_steer": summary.ctrl_k_steer,
    }


def ctrl_settings_match(left: MissionSummary | None, right: MissionSummary | None) -> bool:
    """True when controller limits reported on both summaries agree."""
    a = _ctrl_from_summary(left)
    b = _ctrl_from_summary(right)
    for key in _CTRL_KEYS:
        av, bv = a.get(key), b.get(key)
        if av is None or bv is None:
            return False
        if abs(float(av) - float(bv)) > LIMIT_TOLERANCE:
            return False
    return True


def format_kept_baseline_label(params: dict | None, *, prefix: str = "kept baseline") -> str:
    if not params:
        return prefix
    parts = []
    for key in _CTRL_KEYS:
        if key in params and params[key] is not None:
            short = key.split(".")[-1]
            parts.append(f"{short}={float(params[key]):g}")
    if not parts:
        return prefix
    return f"{prefix} ({', '.join(parts)})"


def format_baseline_from_summary(summary: MissionSummary | None, *, prefix: str = "kept baseline") -> str:
    ctrl = _ctrl_from_summary(summary)
    if not ctrl:
        return prefix
    params = {key: ctrl[key] for key in _CTRL_KEYS if ctrl.get(key) is not None}
    return format_kept_baseline_label(params, prefix=prefix)


def repeat_of_prior_lap(
    summary: MissionSummary | None,
    *,
    starting_reference: MissionSummary | None,
    prior_laps: list[tuple[str, MissionSummary]],
) -> str | None:
    """When metrics and ctrl_* match an earlier lap, name that lap (deterministic twin)."""
    if summary is None:
        return None
    if (
        starting_reference is not None
        and lap_metrics_match(summary, starting_reference)
        and ctrl_settings_match(summary, starting_reference)
    ):
        return "starting settings (reference lap)"
    for label, prior in reversed(prior_laps):
        if lap_metrics_match(summary, prior) and ctrl_settings_match(summary, prior):
            return label
    return None
