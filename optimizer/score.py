"""Keep or roll back one lap. Lower score is better. The model source is not a knob."""

from __future__ import annotations

from dataclasses import dataclass

from optimizer.diagnose import Diagnosis, MissionSummary
from optimizer.evidence import meaningful_improvement

# A warehouse lap is about 201 m. Under this, the robot did not finish.
MIN_DIST_M = 150.0
# Combined score must beat this fraction of the reference. 1.0 is a tie.
KEEP_MARGIN = 0.98
# Accuracy is not allowed to give up more than this many |err| points for a Wh/m win.
ERR_REGRESS_POINTS = 2.0
DIST_DROP = 0.15
# Held-out repeat on the same build must land near the provisional |err|.
CONFIRM_ERR_BAND = 3.0
# A different build only has to finish and stay out of a wild miss.
CONFIRM_ABS_CAP = 40.0
# A lap this close to the reference did not really change. Score lands on 1.000,
# which does not beat KEEP_MARGIN, and it is not evidence about the mechanism.
ERR_MATCH = 0.05
WPM_MATCH_REL = 0.005
DIST_MATCH_M = 0.5
TIME_MATCH_S = 1.0
VISC_MATCH_REL = 0.01
# Accuracy denominator floor so tiny reference |err| does not explode the ratio.
MIN_REF_ERR = 5.0


@dataclass(frozen=True)
class Decision:
    action: str  # keep, rollback, baseline, invalid
    reason: str
    score: float | None
    abs_err: float | None
    wh_per_m: float | None


def _blank_decision(action: str, reason: str) -> Decision:
    return Decision(action, reason, None, None, None)


def _close(left: float | None, right: float | None, tol: float) -> bool:
    if left is None or right is None:
        return True
    return abs(float(left) - float(right)) <= tol


def lap_metrics_match(candidate: MissionSummary | None, reference: MissionSummary | None) -> bool:
    """True when the scored lap did not move relative to the reference.

    Identical err, Wh/m, distance, lap time, and viscous watt-hours produce a
    score of 1.000. That misses the 0.98 keep line, and it usually means the
    config edit never reached the controller.
    """
    if candidate is None or reference is None:
        return False
    if candidate.err_pct is None or reference.err_pct is None:
        return False
    if abs(abs(candidate.err_pct) - abs(reference.err_pct)) > ERR_MATCH:
        return False
    cand_wpm = candidate.wh_per_m()
    ref_wpm = reference.wh_per_m()
    if cand_wpm is None or ref_wpm is None:
        return False
    if abs(cand_wpm - ref_wpm) > max(1e-6, WPM_MATCH_REL * abs(ref_wpm)):
        return False
    if not _close(candidate.dist_m, reference.dist_m, DIST_MATCH_M):
        return False
    if not _close(candidate.duration_s, reference.duration_s, TIME_MATCH_S):
        return False
    if candidate.wh_model_visc is not None and reference.wh_model_visc is not None:
        scale = max(abs(reference.wh_model_visc), 1e-6)
        if abs(candidate.wh_model_visc - reference.wh_model_visc) > max(1e-4, VISC_MATCH_REL * scale):
            return False
    return True


def decide(
    candidate: MissionSummary | None,
    reference: MissionSummary | None,
    *,
    baseline_label: str = "kept baseline",
) -> Decision:
    """Score ``candidate`` against the last kept lap on the same build.

    The first valid lap on a build is a baseline, not a win. A later lap
    is kept only when the combined score improves and |err_pct| does not
    regress by more than ``ERR_REGRESS_POINTS``.
    """
    if candidate is None:
        return _blank_decision("invalid", "no MISSION_SUMMARY")
    abs_err = None if candidate.err_pct is None else abs(candidate.err_pct)
    wh_per_m = candidate.wh_per_m()
    if candidate.err_pct is None:
        return Decision("invalid", "err_pct is na", None, None, wh_per_m)
    if candidate.dist_m is None or candidate.dist_m < MIN_DIST_M:
        return Decision(
            "invalid",
            f"distance {candidate.dist_m} m is under {MIN_DIST_M:.0f} m",
            None,
            abs_err,
            wh_per_m,
        )
    if candidate.wh_measured_drive_elec is None:
        return Decision("invalid", "measured electrical drive is na", None, abs_err, None)
    if (
        reference is None
        or reference.err_pct is None
        or reference.dist_m is None
        or reference.dist_m <= 0
        or reference.wh_measured_drive_elec is None
    ):
        return Decision(
            "baseline",
            "first valid lap on this build",
            1.0,
            abs_err,
            wh_per_m,
        )
    if reference.build and candidate.build and reference.build != candidate.build:
        return Decision(
            "invalid",
            f"build {candidate.build} is not the reference build {reference.build}",
            None,
            abs_err,
            wh_per_m,
        )
    if candidate.dist_m < reference.dist_m * (1.0 - DIST_DROP):
        return Decision(
            "rollback",
            f"distance fell from {reference.dist_m:.1f} m to {candidate.dist_m:.1f} m",
            None,
            abs_err,
            wh_per_m,
        )
    ref_err = abs(reference.err_pct)
    ref_wpm = reference.wh_measured_drive_elec / reference.dist_m
    accuracy = (abs_err or 0.0) / max(ref_err, MIN_REF_ERR)
    if ref_wpm <= 1e-12 or wh_per_m is None:
        efficiency = 1.0
    else:
        efficiency = wh_per_m / ref_wpm
    value = 0.7 * accuracy + 0.3 * efficiency
    err_regress = (abs_err or 0.0) - ref_err
    if err_regress > ERR_REGRESS_POINTS:
        return Decision(
            "rollback",
            f"|err| rose by {err_regress:.1f} points (score {value:.3f})",
            value,
            abs_err,
            wh_per_m,
        )
    if lap_metrics_match(candidate, reference):
        return Decision(
            "rollback",
            f"score {value:.3f} did not beat {KEEP_MARGIN} vs {baseline_label}; "
            "the lap metrics matched the kept baseline",
            value,
            abs_err,
            wh_per_m,
        )
    if value < KEEP_MARGIN:
        return Decision(
            "keep",
            f"combined score {value:.3f} is below keep threshold {KEEP_MARGIN} vs "
            f"{baseline_label} (lower scores are better)",
            value,
            abs_err,
            wh_per_m,
        )
    return Decision(
        "rollback",
        f"combined score {value:.3f} did not clear keep threshold {KEEP_MARGIN} vs "
        f"{baseline_label} (lower scores are better)",
        value,
        abs_err,
        wh_per_m,
    )


def confirmation_ok(
    provisional: MissionSummary | None,
    check: MissionSummary | None,
    *,
    same_build: bool,
) -> tuple[bool, str]:
    """Held-out check. Same build: a repeat lap. Other build: a sanity cap."""
    if (
        check is None
        or check.err_pct is None
        or check.dist_m is None
        or check.dist_m < MIN_DIST_M
    ):
        return False, "confirmation lap did not finish with a numeric err_pct"
    if same_build:
        if provisional is None or provisional.err_pct is None:
            return False, "no provisional lap to compare"
        gap = abs(abs(check.err_pct) - abs(provisional.err_pct))
        if gap > CONFIRM_ERR_BAND:
            return False, f"repeat |err| moved by {gap:.1f} points"
        if provisional.dist_m and check.dist_m < provisional.dist_m * 0.9:
            return False, "repeat lap was short"
        return True, "repeat lap stayed within 3 points of the provisional |err|"
    if abs(check.err_pct) > CONFIRM_ABS_CAP:
        return False, (
            f"held-out build |err| {abs(check.err_pct):.1f}% exceeds {CONFIRM_ABS_CAP:.0f}%"
        )
    return True, "held-out build finished inside the sanity cap"


ConfirmVerdict = str  # pass | fail | missing | missing_turn


def knob_confirm_comparison_ok(
    pre_diag: Diagnosis | None,
    kept_diag: Diagnosis | None,
) -> tuple[ConfirmVerdict, str]:
    """Pre-keep knob value must lose to kept value on the same run body."""
    if (
        pre_diag is None
        or kept_diag is None
        or pre_diag.summary is None
        or kept_diag.summary is None
    ):
        return "missing", "confirm missing MISSION_SUMMARY for pre-keep or kept lap"
    pre_keep = pre_diag.summary
    kept = kept_diag.summary
    if pre_keep.err_pct is None or kept.err_pct is None:
        return "missing", "confirm lap missing err_pct"
    for label, row in ("pre-keep", pre_keep), ("kept", kept):
        if row.dist_m is None or row.dist_m < MIN_DIST_M:
            return "missing", f"{label} confirm lap did not finish (dist_m too short)"
        if abs(row.err_pct) > CONFIRM_ABS_CAP:
            return "missing", (
                f"{label} confirm |err| {abs(row.err_pct):.1f}% exceeds {CONFIRM_ABS_CAP:.0f}%"
            )
    if pre_diag.turn_err is None or kept_diag.turn_err is None:
        return (
            "missing_turn",
            "confirm lap log missing SEGMENT_ENERGY turn segments (turn_err n/a); "
            "cannot score the confirm pair yet",
        )
    ok, detail = meaningful_improvement(
        pre_diag,
        kept_diag,
        pre_keep,
        baseline_label="pre-keep knob value",
    )
    if not ok:
        return "fail", f"kept value did not beat pre-keep on the held-out pair: {detail}"
    return "pass", "kept knob value beat pre-keep by the evidence threshold"
