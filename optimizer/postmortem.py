"""Why a trial came out the way it did, in language the morning report can quote.

The picker reads the verdict tag. A flat lap (score stuck at 1.000) is not
treated as evidence about the mechanism.
"""

from __future__ import annotations

from optimizer.diagnose import Diagnosis
from optimizer.score import lap_metrics_match
from optimizer.trial_validity import limits_match_intended

_SPEED = {"controller.straight_m_s", "controller.cruise_m_s"}
_SHARE_NAMES = ("roll", "visc", "turn", "accel")


def step_direction(old, new) -> str:
    """``down`` / ``up`` for a numeric knob, ``switch`` for a build change."""
    try:
        before = float(old)
        after = float(new)
    except (TypeError, ValueError):
        return "switch"
    if after < before - 1e-9:
        return "down"
    if after > before + 1e-9:
        return "up"
    return "switch"


def _num(summary, name: str) -> float | None:
    if summary is None:
        return None
    value = getattr(summary, name, None)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _observed(before: float | None, after: float | None, tol: float, places: int = 2) -> str:
    if before is None and after is None:
        return "not in this log"
    def fmt(value: float | None) -> str:
        return "n/a" if value is None else f"{value:.{places}f}"
    if before is None or after is None:
        return f"{fmt(before)} → {fmt(after)}"
    delta = after - before
    if abs(delta) <= tol:
        return f"{fmt(before)} → {fmt(after)} (no change)"
    word = "up" if delta > 0 else "down"
    return f"{fmt(before)} → {fmt(after)} ({word} {abs(delta):.{places}f})"


def _shares_match(before: Diagnosis | None, after: Diagnosis | None) -> bool:
    b = None if before is None else before.summary
    a = None if after is None else after.summary
    if b is None or a is None:
        return True
    left, right = b.shares(), a.shares()
    if not left and not right:
        return True
    for name in _SHARE_NAMES:
        if name not in left or name not in right:
            continue
        if abs(left[name] - right[name]) > 0.01:
            return False
    return True


def _segments_match(before: Diagnosis | None, after: Diagnosis | None) -> bool:
    if before is None or after is None:
        return True
    for attr in ("straight_err", "turn_err"):
        left = getattr(before, attr, None)
        right = getattr(after, attr, None)
        if left is None or right is None:
            continue
        if abs(float(left) - float(right)) > 0.05:
            return False
    return True


def lap_was_flat(before: Diagnosis | None, after: Diagnosis | None) -> bool:
    """Summary, shares, and segment errors all sat still."""
    b = None if before is None else before.summary
    a = None if after is None else after.summary
    if not lap_metrics_match(a, b):
        return False
    return _shares_match(before, after) and _segments_match(before, after)


def _predictions(param: str | None, direction: str, before: Diagnosis | None) -> dict[str, str]:
    summary = None if before is None else before.summary
    err = None if summary is None else summary.err_pct
    high = err is not None and err < 0
    if param in _SPEED and direction == "down":
        err_pred = (
            "a smaller miss, if viscous watt-hours fall with speed"
            if high
            else "little change unless the model error was coming from speed"
        )
        return {
            "err_pct": err_pred,
            "wh_per_m": "fewer electrical watt-hours per metre",
            "visc": "less viscous watt-hours",
            "roll": "almost no change",
            "turn": "almost no change",
            "accel": "almost no change",
            "straight_err": "straights closer to the model",
            "turn_err": "about the same, because the pivot rate was not changed",
            "duration_s": "a longer lap",
        }
    if param in _SPEED and direction == "up":
        return {
            "err_pct": "a larger miss if the model is already high on viscous loss",
            "wh_per_m": "more electrical watt-hours per metre",
            "visc": "more viscous watt-hours",
            "roll": "almost no change",
            "turn": "almost no change",
            "accel": "almost no change",
            "straight_err": "straights no closer, and possibly worse",
            "turn_err": "about the same",
            "duration_s": "a shorter lap",
        }
    if param == "controller.pivot_w_max_rad_s" and direction == "down":
        return {
            "err_pct": "a smaller miss if the error is in the turns",
            "wh_per_m": "slightly fewer watt-hours per metre if turn scrub drops",
            "visc": "almost no change",
            "roll": "almost no change",
            "turn": "less turn watt-hours",
            "accel": "almost no change",
            "straight_err": "about the same",
            "turn_err": "turn segments closer to the model",
            "duration_s": "a slightly longer lap",
        }
    if param == "controller.k_steer" and direction == "up":
        return {
            "err_pct": "a smaller miss if the path was weaving",
            "wh_per_m": "almost no change",
            "visc": "almost no change",
            "roll": "almost no change",
            "turn": "almost no change",
            "accel": "almost no change",
            "straight_err": "straighter tracking, so less straight-segment error",
            "turn_err": "about the same",
            "duration_s": "about the same",
        }
    if param == "bom_build":
        return {key: "a different robot, not a better version of the same lap" for key in (
            "err_pct", "wh_per_m", "visc", "roll", "turn", "accel", "straight_err", "turn_err", "duration_s",
        )}
    return {key: "a visible change if this knob is doing what the hypothesis says" for key in (
        "err_pct", "wh_per_m", "visc", "roll", "turn", "accel", "straight_err", "turn_err", "duration_s",
    )}


def _metric_rows(before: Diagnosis | None, after: Diagnosis | None, predicted: dict[str, str]) -> list[dict]:
    b = None if before is None else before.summary
    a = None if after is None else after.summary
    b_shares = b.shares() if b is not None else {}
    a_shares = a.shares() if a is not None else {}

    def share(store: dict, name: str) -> float | None:
        if name not in store:
            return None
        return 100.0 * float(store[name])

    specs = [
        ("err_pct", "err_pct", _num(b, "err_pct"), _num(a, "err_pct"), 0.05, 1),
        ("Wh/m", "wh_per_m", None if b is None else b.wh_per_m(), None if a is None else a.wh_per_m(), 1e-4, 4),
        ("viscous share", "visc", share(b_shares, "visc"), share(a_shares, "visc"), 1.0, 0),
        ("roll share", "roll", share(b_shares, "roll"), share(a_shares, "roll"), 1.0, 0),
        ("turn share", "turn", share(b_shares, "turn"), share(a_shares, "turn"), 1.0, 0),
        ("accel share", "accel", share(b_shares, "accel"), share(a_shares, "accel"), 1.0, 0),
        ("straight segment error", "straight_err",
         None if before is None else before.straight_err,
         None if after is None else after.straight_err, 0.05, 1),
        ("turn segment error", "turn_err",
         None if before is None else before.turn_err,
         None if after is None else after.turn_err, 0.05, 1),
        ("lap time", "duration_s", _num(b, "duration_s"), _num(a, "duration_s"), 1.0, 1),
    ]
    rows = []
    for label, key, left, right, tol, places in specs:
        rows.append({
            "label": label,
            "predicted": predicted.get(key, "a change if the knob took effect"),
            "observed": _observed(left, right, tol, places),
        })
    return rows


def _visc_fell(before: Diagnosis | None, after: Diagnosis | None) -> bool:
    b = None if before is None or before.summary is None else before.summary.wh_model_visc
    a = None if after is None or after.summary is None else after.summary.wh_model_visc
    if b is None or a is None:
        return False
    return float(a) < float(b) * 0.98


def _turn_improved(before: Diagnosis | None, after: Diagnosis | None) -> bool:
    if before is None or after is None or before.turn_err is None or after.turn_err is None:
        return False
    return abs(after.turn_err) + 0.5 < abs(before.turn_err)


def _straight_improved(before: Diagnosis | None, after: Diagnosis | None) -> bool:
    if before is None or after is None or before.straight_err is None or after.straight_err is None:
        return False
    return abs(after.straight_err) + 0.5 < abs(before.straight_err)


def _judge(
    *,
    param: str | None,
    direction: str,
    action: str,
    before: Diagnosis | None,
    after: Diagnosis | None,
    flat: bool,
    within_noise: bool = False,
    noise_detail: str = "",
) -> tuple[str, str, str]:
    """Return verdict, mechanism paragraph, and the plain lesson."""
    if action == "invalid":
        return (
            "invalid",
            "This lap is not valid evidence: the mission summary, body, or controller limits "
            "did not match what the trial asked for.",
            "Do not score or learn from this lap. Fix limits visibility or the body pin before retrying.",
        )
    if param is None:
        return (
            "inconclusive",
            "No knob was changed, so this lap does not support or refute a mechanism. It is the reference.",
            "Use this lap as the reference. Do not read a mechanism into it.",
        )
    if before is None or after is None or before.summary is None or after.summary is None:
        return (
            "inconclusive",
            "The lap did not finish with a mission summary, so the hypothesis was not tested.",
            f"Do not treat a lap with no summary as evidence about {param}.",
        )
    if action in {"baseline", "reference"}:
        return (
            "inconclusive",
            "This lap ran at the starting settings before any search trial, so it is the reference rather than a test of a knob.",
            "Use this lap as the reference for every trial in this run.",
        )
    knob = param
    if action == "keep":
        return (
            "tentative",
            "The combined score beat 0.98 on this lap, but the move may still be within lap noise until the held-out repeat agrees.",
            f"Treat {knob} {direction} as tentative until the confirm lap agrees; only then mark it supported in KNOWLEDGE.md.",
        )
    if flat:
        return (
            "inconclusive",
            "err_pct, Wh/m, the energy shares, straight and turn error, and lap time matched the "
            "previous lap. A real change of this knob should have moved at least the lap time. "
            "The new value may not have reached the controller (check the CONTROLLER LIMITS line). "
            "The score stays at 1.000, which does not beat 0.98, so the step is not kept. "
            "This does not support or refute the mechanism.",
            f"A flat lap after moving {knob} {direction} is not evidence. "
            "Check CONTROLLER LIMITS before repeating that step.",
        )
    if action == "confirm-pass":
        return (
            "supported",
            "The held-out lap agreed with the provisional keep, so the change is not a one-lap fluke.",
            f"The held-out lap agreed with {knob} {direction}. Prefer that direction while it keeps clearing the score.",
        )
    if action == "confirm-fail":
        return (
            "refuted",
            "The held-out lap did not back the provisional keep, so the change does not travel.",
            f"Do not keep pushing {knob} {direction}. The held-out lap disagreed.",
        )
    if action == "inconclusive" or (
        within_noise and action in {"rollback", "inconclusive"}
    ):
        if (
            param == "controller.pivot_w_max_rad_s"
            and direction == "down"
            and _turn_improved(before, after)
        ):
            return (
                "inconclusive",
                "Turn-segment error moved the way a slower pivot predicts, but the combined "
                "score or overall |err| stayed inside the lap noise band, so this lap is not "
                "enough to keep or refute the knob.",
                f"Do not treat pivot-down as refuted. {noise_detail}".strip(),
            )
        detail = noise_detail or "The move was within the configured noise band."
        return (
            "inconclusive",
            "The lap did not clear the keep threshold and the measured move was within lap "
            "noise, so this direction is not refuted.",
            f"Do not block {knob} {direction} in the picker from this lap alone. {detail}",
        )
    kept = action == "keep"
    if param == "bom_build":
        return (
            "inconclusive",
            "Switching BoM build measures a different robot. Mass and inertia change. "
            "The contact cylinder stays 0.08 m. This lap is that build's baseline, not a test of the previous speed idea.",
            "Treat each BoM build as its own baseline. Do not compare its Wh/m with another build as if they were the same robot.",
        )
    if param in _SPEED and direction == "down" and not kept:
        if not _visc_fell(before, after):
            which = (
                "straight"
                if param == "controller.straight_m_s"
                else "cruise"
                if param == "controller.cruise_m_s"
                else "speed"
            )
            return (
                "refuted",
                f"Lowering {which} speed did not cut viscous Wh enough to beat the score threshold, "
                "so viscous loss is less speed-sensitive than assumed.",
                f"Do not slow {knob} again to chase viscous error. The last slower step did not move viscous Wh enough.",
            )
        return (
            "refuted",
            "Viscous watt-hours fell, but not enough for the combined score to beat 0.98. The step is not worth keeping.",
            f"Do not repeat {knob} down until the score can clear 0.98. The last drop in viscous Wh was too small.",
        )
    if param in _SPEED and direction == "down" and kept:
        return (
            "tentative",
            "Slowing down cut the miss enough for the score to beat 0.98 on this lap; viscous loss may track speed if the confirm lap agrees.",
            f"Tentatively prefer slowing {knob} when viscous watt-hours dominate; wait for the held-out lap.",
        )
    if param in _SPEED and direction == "up" and not kept:
        return (
            "refuted",
            "Speeding up did not improve the score. Extra speed is not buying back accuracy.",
            f"Do not raise {knob} to chase the score. The last faster step did not help.",
        )
    if param == "controller.pivot_w_max_rad_s" and direction == "down" and not kept:
        if not _turn_improved(before, after):
            return (
                "refuted",
                "A slower pivot did not pull turn-segment error down, so gentle-turn scrub is not "
                "tracking the pivot cap the way the notes assume.",
                "Do not lower controller.pivot_w_max_rad_s again until turn-segment error actually moves.",
            )
        return (
            "refuted",
            "Turn error moved, but the combined score still missed 0.98.",
            "Do not take another pivot-rate step until a lap clears 0.98.",
        )
    if param == "controller.pivot_w_max_rad_s" and direction == "down" and kept:
        return (
            "tentative",
            "A slower pivot improved the score on this lap; turn scrub may follow the yaw rate if the confirm lap agrees.",
            "Tentatively prefer a slower controller.pivot_w_max_rad_s while turn segments are the miss.",
        )
    if param == "controller.k_steer" and not kept:
        if direction == "up" and not _straight_improved(before, after):
            return (
                "refuted",
                "A calmer steer did not tighten the straight-segment error, so weave is not the residual.",
                "Do not raise controller.k_steer again until straight-segment error moves.",
            )
        return (
            "refuted",
            "The steer change did not clear the score threshold.",
            f"Do not repeat controller.k_steer {direction}. The last step did not clear 0.98.",
        )
    if kept and param == "controller.k_steer":
        return (
            "tentative",
            "The steer change cleared the score on this lap; calmer steering may help if the confirm lap agrees.",
            "Tentatively prefer controller.k_steer up while the path is weaving.",
        )
    return (
        "refuted",
        "The score did not beat 0.98, so this direction did not earn another step.",
        f"Do not repeat {knob} {direction}. The last step missed the score threshold.",
    )


def build_postmortem(
    *,
    param: str | None,
    old,
    new,
    action: str,
    before: Diagnosis | None,
    after: Diagnosis | None,
    intended: dict | None = None,
    within_noise: bool = False,
    noise_detail: str = "",
    summary_trusted: bool = True,
    evidence: dict | None = None,
) -> dict:
    """Structured post-mortem for one trial. ``next_hypothesis`` is filled by the loop."""
    direction = "switch" if param is None else step_direction(old, new)
    flat = bool(param) and lap_was_flat(before, after)
    verdict, mechanism, lesson = _judge(
        param=param,
        direction=direction,
        action=action,
        before=before,
        after=after,
        flat=flat,
        within_noise=within_noise,
        noise_detail=noise_detail,
    )
    if action in {"confirm-pass", "confirm-fail"}:
        predicted = {
            key: "the held-out lap should agree with the provisional keep"
            for key in (
                "err_pct", "wh_per_m", "visc", "roll", "turn", "accel",
                "straight_err", "turn_err", "duration_s",
            )
        }
    else:
        predicted = _predictions(param, direction, before)
    tag = ""
    if param and verdict in {"supported", "refuted", "inconclusive", "tentative"} and direction in {
        "up",
        "down",
        "switch",
    }:
        if param is not None and action not in {"baseline", "reference"}:
            tag = f"[{verdict} {param} {direction}]"
    if action in {"baseline", "reference"}:
        tag = ""
        verdict = "inconclusive"
    if verdict == "invalid":
        tag = ""
    limit_note = None
    if summary_trusted:
        limit_note = limits_match_intended(None if after is None else after.summary, intended)
    if limit_note and action != "invalid":
        verdict = "invalid"
        mechanism = limit_note
        lesson = "Do not learn from this lap until MISSION_SUMMARY carries the controller limits."
        tag = ""
    elif limit_note and action == "invalid":
        mechanism = limit_note
    payload = {
        "verdict": verdict,
        "param": param,
        "direction": direction if param else "",
        "flat": flat,
        "mechanism": mechanism,
        "lesson": lesson,
        "tag": tag,
        "next_hypothesis": "",
        "metrics": _metric_rows(before, after, predicted),
    }
    if evidence:
        payload["evidence"] = evidence
    return payload
