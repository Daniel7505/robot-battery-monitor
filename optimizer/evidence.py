"""Minimum effect sizes before a lap counts as evidence or a keep."""

from __future__ import annotations

from optimizer.diagnose import Diagnosis, MissionSummary

# |err| must move more than this vs the reference to count as real (percent points).
ERR_IMPROVE_MIN = 1.5
# Electrical Wh/m must fall by more than this fraction vs reference to count alone.
WPM_IMPROVE_MIN_REL = 0.015
# Turn-segment aggregate error must beat this, or segment spread if larger.
TURN_ERR_IMPROVE_MIN = 2.5


def turn_segment_noise(diag: Diagnosis | None) -> float:
    """Typical lap-to-lap swing on turn segments (percent points on |err|)."""
    if diag is None:
        return TURN_ERR_IMPROVE_MIN
    values = [
        float(seg.err_pct)
        for seg in diag.segments
        if seg.is_turn and seg.err_pct is not None
    ]
    if len(values) < 2:
        return TURN_ERR_IMPROVE_MIN
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return max(TURN_ERR_IMPROVE_MIN, variance**0.5 * 1.5)


def measure_effects(
    before: Diagnosis | None,
    after: Diagnosis | None,
    reference: MissionSummary | None,
) -> dict:
    """Numeric deltas vs the reference and turn-segment noise band."""
    out = {
        "err_gain_pp": None,
        "wpm_gain_rel": None,
        "turn_delta_pp": None,
        "turn_noise_band_pp": turn_segment_noise(before),
        "meaningful": False,
        "detail": "",
    }
    if after is None or after.summary is None or reference is None:
        out["detail"] = "missing lap or reference summary"
        return out
    if after.summary.err_pct is None or reference.err_pct is None:
        out["detail"] = "err_pct is na"
        return out
    ref_err = abs(reference.err_pct)
    abs_err = abs(after.summary.err_pct)
    err_gain = ref_err - abs_err
    out["err_gain_pp"] = err_gain
    ref_wpm = reference.wh_per_m()
    cand_wpm = after.summary.wh_per_m()
    if ref_wpm is None or cand_wpm is None or ref_wpm <= 0:
        wpm_gain = 0.0
    else:
        wpm_gain = (ref_wpm - cand_wpm) / ref_wpm
    out["wpm_gain_rel"] = wpm_gain
    turn_delta = None
    if before is not None and before.turn_err is not None and after.turn_err is not None:
        turn_delta = abs(before.turn_err) - abs(after.turn_err)
        out["turn_delta_pp"] = turn_delta
    band = float(out["turn_noise_band_pp"])
    err_ok = err_gain > ERR_IMPROVE_MIN
    wpm_ok = wpm_gain > WPM_IMPROVE_MIN_REL
    turn_ok = turn_delta is not None and turn_delta >= band
    out["meaningful"] = bool(err_ok or wpm_ok or turn_ok)
    if out["meaningful"]:
        out["detail"] = ""
    else:
        out["detail"] = (
            f"|err| improved {err_gain:.2f} points (need >{ERR_IMPROVE_MIN}), "
            f"Wh/m improved {wpm_gain:.1%} (need >{WPM_IMPROVE_MIN_REL:.1%}), "
            f"turn error did not beat the segment noise band ({band:.1f} points)"
        )
    return out


def meaningful_improvement(
    before: Diagnosis | None,
    after: Diagnosis | None,
    reference: MissionSummary | None,
) -> tuple[bool, str]:
    """True when err, Wh/m, or turn error moved beyond lap noise."""
    effects = measure_effects(before, after, reference)
    return bool(effects["meaningful"]), str(effects["detail"])


def evidence_status(
    *,
    action: str,
    post_verdict: str,
    meaningful: bool,
    flat: bool,
) -> str:
    """Plain status for results.jsonl and the morning report."""
    if action in {"reference", "baseline"}:
        return "reference"
    if action == "invalid":
        return "invalid"
    if action == "keep":
        return "tentative"
    if action == "confirm-pass":
        return "supported"
    if action == "confirm-fail":
        return "refuted"
    if flat:
        return "inconclusive"
    if post_verdict == "tentative":
        return "tentative"
    if post_verdict == "supported":
        return "supported"
    if post_verdict == "refuted":
        return "refuted"
    if not meaningful and action in {"inconclusive", "rollback"}:
        return "inconclusive"
    if post_verdict == "inconclusive":
        return "inconclusive"
    return post_verdict or "inconclusive"
