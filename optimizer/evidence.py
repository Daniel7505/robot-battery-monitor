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


def meaningful_improvement(
    before: Diagnosis | None,
    after: Diagnosis | None,
    reference: MissionSummary | None,
) -> tuple[bool, str]:
    """True when err, Wh/m, or turn error moved beyond lap noise."""
    if after is None or after.summary is None or reference is None:
        return False, "missing lap or reference summary"
    if after.summary.err_pct is None or reference.err_pct is None:
        return False, "err_pct is na"
    ref_err = abs(reference.err_pct)
    abs_err = abs(after.summary.err_pct)
    err_gain = ref_err - abs_err
    ref_wpm = reference.wh_per_m()
    cand_wpm = after.summary.wh_per_m()
    if ref_wpm is None or cand_wpm is None or ref_wpm <= 0:
        wpm_gain = 0.0
    else:
        wpm_gain = (ref_wpm - cand_wpm) / ref_wpm
    err_ok = err_gain > ERR_IMPROVE_MIN
    wpm_ok = wpm_gain > WPM_IMPROVE_MIN_REL
    turn_ok = False
    if before is not None and before.turn_err is not None and after.turn_err is not None:
        band = turn_segment_noise(before)
        turn_delta = abs(before.turn_err) - abs(after.turn_err)
        turn_ok = turn_delta >= band
    if err_ok or wpm_ok or turn_ok:
        return True, ""
    return (
        False,
        f"|err| improved {err_gain:.2f} points (need >{ERR_IMPROVE_MIN}), "
        f"Wh/m improved {wpm_gain:.1%} (need >{WPM_IMPROVE_MIN_REL:.1%}), "
        f"turn error did not beat the segment noise band ({turn_segment_noise(before):.1f} points)",
    )
