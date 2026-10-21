"""Minimum effect sizes before a lap counts as evidence or a keep."""

from __future__ import annotations

from optimizer.diagnose import Diagnosis, MissionSummary

# |err| must move more than this vs the reference to count as real (percent points).
ERR_IMPROVE_MIN = 1.5
# Electrical Wh/m must fall by more than this fraction vs reference to count alone.
WPM_IMPROVE_MIN_REL = 0.015
# Minimum turn-segment |err| move to count as evidence (percent points).
TURN_ERR_IMPROVE_MIN = 2.5
# Warehouse Webots twin is deterministic: identical ctrl_* gives identical laps.
# Segment spread is geometry along the route, not lap-to-lap stochastic noise.
DETERMINISTIC_TWIN = True

# Tie-break when a lap both clears the 0.98 score line and sits inside the noise band:
# * |err| or turn |err| worsening past TURN_ERR_IMPROVE_MIN → refuted (wins over keep).
# * Otherwise → inconclusive (neither keep nor refuted).
# Documented so morning reports do not read a noise-band lap as refuted.
KEEP_VS_REFUTED_TIEBREAK = "worsening_beyond_band_refutes_else_inconclusive"


def turn_segment_noise(diag: Diagnosis | None) -> float:
    """Minimum effect size for turn aggregate error (not segment stdev)."""
    del diag
    return TURN_ERR_IMPROVE_MIN


def measure_effects(
    after: Diagnosis | None,
    reference: MissionSummary | None,
    *,
    before: Diagnosis | None = None,
    baseline_diag: Diagnosis | None = None,
    baseline_label: str = "kept baseline",
) -> dict:
    """Numeric deltas vs the scoring baseline summary and its diagnosis."""
    turn_ref = baseline_diag if baseline_diag is not None else before
    out = {
        "err_gain_pp": None,
        "wpm_gain_rel": None,
        "turn_delta_pp": None,
        "turn_noise_band_pp": turn_segment_noise(turn_ref),
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
    if turn_ref is not None and turn_ref.turn_err is not None and after.turn_err is not None:
        turn_delta = abs(turn_ref.turn_err) - abs(after.turn_err)
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
            f"vs {baseline_label}: |err| improved {err_gain:.2f} points (need >{ERR_IMPROVE_MIN}), "
            f"Wh/m improved {wpm_gain:.1%} (need >{WPM_IMPROVE_MIN_REL:.1%}), "
            f"turn error did not beat the minimum band ({band:.1f} points; "
            f"{'fixed effect size' if DETERMINISTIC_TWIN else 'segment spread'})"
        )
    return out


def meaningful_improvement(
    before: Diagnosis | None,
    after: Diagnosis | None,
    reference: MissionSummary | None,
    *,
    baseline_label: str = "kept baseline",
) -> tuple[bool, str]:
    """True when err, Wh/m, or turn error moved beyond lap noise."""
    effects = measure_effects(
        after,
        reference,
        before=before,
        baseline_label=baseline_label,
    )
    return bool(effects["meaningful"]), str(effects["detail"])


def worsened_beyond_band(effects: dict | None) -> tuple[bool, str]:
    """True when |err| or turn |err| moved worse past the configured minimum band."""
    if not effects:
        return False, ""
    band = float(effects.get("turn_noise_band_pp") or TURN_ERR_IMPROVE_MIN)
    turn_delta = effects.get("turn_delta_pp")
    if turn_delta is not None and float(turn_delta) < -band:
        delta = abs(float(turn_delta))
        return True, f"turn |err| worsened {delta:.1f} pp vs the {band:.1f} pp band"
    err_gain = effects.get("err_gain_pp")
    if err_gain is not None and float(err_gain) < -ERR_IMPROVE_MIN:
        return True, f"|err| worsened {abs(float(err_gain)):.1f} pp beyond the noise band"
    return False, ""


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
    if action == "refuted":
        return "refuted"
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
