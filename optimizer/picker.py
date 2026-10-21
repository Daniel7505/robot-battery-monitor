"""Pick one allowlisted change. Rules first; an external command can replace them.

The LLM hook shells out to ``OPTIMIZER_LLM_CMD``. The command reads a JSON
object on stdin and prints a JSON proposal on stdout. No API key is stored
in this repo, and a proposal that names a file edit or a non-allowlisted
parameter is discarded.
"""

from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass, field

from src.controller_limits import KNOBS, AllowlistError, check_param, check_set

BUILDS = ("light", "heavy", "budget")

_VISC_ORDER = (
    "controller.pivot_w_max_rad_s",
    "controller.k_steer",
    "controller.cruise_m_s",
)
_TURN_ORDER = (
    "controller.pivot_w_max_rad_s",
    "controller.k_steer",
    "controller.cruise_m_s",
)


@dataclass
class Proposal:
    param: str
    value: float | str | None
    hypothesis: str
    expected_effect: str
    code_suggestions: list[str] = field(default_factory=list)
    note: str = ""


def _same(left, right) -> bool:
    if left in (None, "") and right in (None, ""):
        return True
    try:
        return abs(float(left) - float(right)) < 1e-6
    except (TypeError, ValueError):
        return str(left) == str(right)


def _kept_knob_state(history: list[dict]) -> dict | None:
    """Most recent provisional or confirmed keep (param, new, old)."""
    for row in reversed(history or []):
        if row.get("phase") == "confirm" and row.get("decision") == "confirm-pass":
            return {
                "param": row.get("param"),
                "new": row.get("new"),
                "old": row.get("old"),
            }
        if row.get("decision") == "keep":
            return {
                "param": row.get("param"),
                "new": row.get("new"),
                "old": row.get("old"),
            }
    return None


def _undoes_keep(proposal: Proposal, kept: dict | None) -> bool:
    if kept is None or kept.get("param") is None:
        return False
    if proposal.param != kept.get("param"):
        return False
    return _same(proposal.value, kept.get("old"))


def _confirmed_keeps(history: list[dict]) -> list[dict]:
    """Latest confirmed value per param (confirm-pass wins over older confirms)."""
    out: dict[str, dict] = {}
    for row in history or []:
        if row.get("phase") != "confirm" or row.get("decision") != "confirm-pass":
            continue
        param = row.get("param")
        if param:
            out[str(param)] = {
                "param": param,
                "new": row.get("new"),
                "old": row.get("old"),
            }
    return list(out.values())


def _toward_pre_keep(proposal: Proposal, keep: dict) -> bool:
    """True when the proposal moves a confirmed knob back toward its pre-keep value."""
    if proposal.param != keep.get("param"):
        return False
    try:
        kept = float(keep["new"])
        pre = float(keep["old"])
        val = float(proposal.value)
    except (TypeError, ValueError, KeyError):
        return _same(proposal.value, keep.get("old"))
    if kept < pre - 1e-9:
        return val > kept + 1e-6
    if kept > pre + 1e-9:
        return val < kept - 1e-6
    return _same(val, pre)


def _reverts_confirmed_keep(proposal: Proposal, history: list[dict]) -> bool:
    for keep in _confirmed_keeps(history):
        if _toward_pre_keep(proposal, keep):
            return True
    return False


def _rolled_back(history: list[dict], param: str, value) -> bool:
    for row in history:
        if row.get("param") != param:
            continue
        if row.get("decision") not in {"rollback", "invalid", "confirm-fail"}:
            continue
        if _same(row.get("new"), value):
            return True
    return False


def _consecutive_rollbacks(history: list[dict]) -> int:
    count = 0
    for row in reversed(history):
        if row.get("phase") == "confirm":
            continue
        if row.get("decision") in {"rollback", "invalid"}:
            count += 1
            continue
        break
    return count


def _suggestions(diagnosis) -> list[str]:
    summary = getattr(diagnosis, "summary", None)
    err = None if summary is None else summary.err_pct
    if err is None or abs(err) <= 10:
        return []
    return [
        "If |err_pct| stays above 10 after the allowlisted speed steps, the residual "
        "is in the energy model (straight-line contact is not billed; visc is hinge "
        "damping at v/0.08). That needs a reviewed patch. This loop will not edit it."
    ]


def _speed_direction(diagnosis) -> int:
    """−1 slows the robot. +1 speeds it up."""
    summary = diagnosis.summary
    err = None if summary is None else summary.err_pct
    corr = diagnosis.speed_err_corr
    if corr is not None and corr > 0.3 and err is not None and err > 0:
        return -1
    if err is not None and err < 0:
        return -1
    if err is not None and abs(err) < 3:
        return -1
    return 1


def _pivot_direction(diagnosis) -> int:
    # Negative turn error: the model is high. Gentle-turn force is v_slip/FDS,
    # and v_slip follows the controller's yaw rate, so a slower pivot lowers it.
    if diagnosis.turn_err is None:
        return -1
    return -1 if diagnosis.turn_err < 0 else 1


def _direction_for(key: str, diagnosis) -> int:
    if key == "controller.pivot_w_max_rad_s":
        return _pivot_direction(diagnosis)
    if key == "controller.k_steer":
        # A slightly calmer steer (higher k) is the first try when the path is weaving.
        return 1
    return _speed_direction(diagnosis)


_LESSON_TAG = re.compile(r"\[(supported|refuted|inconclusive) (\S+) (up|down|switch)\]")


def lesson_book(knowledge: str, history: list[dict]) -> dict:
    """Directions earlier trials supported, refuted, or left untested.

    A flat lap is ``inconclusive`` and is tried only after other knobs.
    A refuted direction is not repeated.
    """
    blocked: set[tuple[str, str]] = set()
    deferred: set[tuple[str, str]] = set()
    preferred: list[tuple[str, str]] = []

    def add(verdict, param, direction) -> None:
        if not param or direction not in {"up", "down", "switch"}:
            return
        if verdict in {"invalid"}:
            return
        if verdict not in {"supported", "refuted", "inconclusive"}:
            return
        key = (str(param), str(direction))
        if verdict == "refuted":
            blocked.add(key)
        elif verdict == "inconclusive":
            deferred.add(key)
        elif key not in preferred:
            preferred.append(key)

    for row in history or []:
        post = row.get("postmortem") if isinstance(row, dict) else None
        if isinstance(post, dict):
            add(post.get("verdict"), post.get("param"), post.get("direction"))
    for verdict, param, direction in _LESSON_TAG.findall(knowledge or ""):
        add(verdict, param, direction)
    return {"blocked": blocked, "deferred": deferred, "preferred": preferred}


def _direction_word(sign: int) -> str:
    return "down" if sign < 0 else "up"


def _ordered_keys(order: tuple[str, ...], diagnosis, book: dict) -> list[str]:
    def primary_blocked(key: str) -> bool:
        return (key, _direction_word(_direction_for(key, diagnosis))) in book["blocked"]

    def rank(key: str) -> int:
        if any(param == key for param, _direction in book["preferred"]):
            return 0
        if any(param == key for param, _direction in book["deferred"]):
            return 2
        return 1

    open_keys = [key for key in order if not primary_blocked(key)]
    if not open_keys:
        open_keys = list(order)
    return sorted(open_keys, key=rank)


def _try_float(params: dict, key: str, direction: int, history: list[dict], book: dict | None = None):
    knob = KNOBS[key]
    current = float(params[key])
    signs = [direction, -direction]
    if book is not None:
        signs = [sign for sign in signs if (key, _direction_word(sign)) not in book["blocked"]]
    for sign in signs:
        nxt = round(current + sign * float(knob.step), 4)
        try:
            nxt = check_param(key, nxt)
        except AllowlistError:
            continue
        trial = dict(params)
        trial[key] = nxt
        try:
            check_set(trial)
        except AllowlistError:
            continue
        if _rolled_back(history, key, nxt) or _same(current, nxt):
            continue
        return float(nxt)
    return None


def _next_build(params: dict, history: list[dict]) -> str | None:
    current = params.get("bom_build") or ""
    seen = {
        row.get("build")
        for row in history
        if row.get("decision") in {"baseline", "reference", "keep", "confirm-pass"}
    }
    for name in BUILDS:
        if name != current and name not in seen and not _rolled_back(history, "bom_build", name):
            return name
    start = BUILDS.index(current) if current in BUILDS else -1
    for offset in range(1, len(BUILDS) + 1):
        name = BUILDS[(start + offset) % len(BUILDS)]
        if name != current and not _rolled_back(history, "bom_build", name):
            return name
    return None


def _turn_dominates(diagnosis) -> bool:
    if diagnosis.turn_err is None or diagnosis.straight_err is None:
        return False
    return abs(diagnosis.turn_err) > abs(diagnosis.straight_err) + 10.0


def held_out_build(current: str | None) -> str:
    """A different BoM build for the confirmation lap."""
    if current == "light":
        return "heavy"
    if current == "heavy":
        return "budget"
    return "light"


class RulePicker:
    """One change, with a hypothesis the morning report can quote."""

    def pick(self, diagnosis, knowledge: str, history: list[dict], params: dict) -> Proposal | None:
        suggestions = _suggestions(diagnosis)
        book = lesson_book(knowledge, history)
        order = _TURN_ORDER if _turn_dominates(diagnosis) else _VISC_ORDER
        kept = _kept_knob_state(history)
        for key in _ordered_keys(order, diagnosis, book):
            value = _try_float(params, key, _direction_for(key, diagnosis), history, book)
            if value is None:
                continue
            proposal = self._float_proposal(params, key, value, diagnosis, suggestions)
            if _undoes_keep(proposal, kept):
                continue
            if _reverts_confirmed_keep(proposal, history):
                continue
            return proposal
        return None

    def _float_proposal(self, params, key, value, diagnosis, suggestions) -> Proposal:
        summary = diagnosis.summary
        err = None if summary is None else summary.err_pct
        err_txt = "unknown" if err is None else f"{err:.1f}%"
        visc = diagnosis.share("visc")
        old = params.get(key)
        if key == "controller.pivot_w_max_rad_s":
            hypothesis = (
                f"Turn segments are further off than straights (turn err {diagnosis.turn_err}, "
                f"straight err {diagnosis.straight_err}). Turn scrub force is "
                f"min(mu*m*g/2, v_slip/FDS), so on a gentle turn mass barely matters and "
                f"v_slip follows the controller yaw rate. Stepping {key} from {old} to {value}."
            )
            expected = (
                "The turn share of the model should move toward the measured electrical "
                "drive. A miss is rolled back. Mass is not the knob, and the model source is not edited."
            )
        else:
            hypothesis = (
                f"Model drive is {visc:.0%} viscous damping and err_pct is {err_txt}. "
                f"Negative means the model is high. Viscous watt-hours grow with speed, "
                f"while turn scrub on a gentle turn is min(mu*m*g/2, v_slip/FDS) so mass "
                f"barely matters and turn rate is the controller's. Stepping {key} from {old} to {value}."
            )
            expected = (
                "If the residual tracks speed, |err_pct| falls and drive Wh/m does not rise. "
                "Otherwise the change is rolled back. The energy model is not edited."
            )
        return Proposal(key, value, hypothesis, expected, suggestions)

    def _build_proposal(self, params, build: str, diagnosis, suggestions) -> Proposal:
        current = params.get("bom_build") or "(unset)"
        return Proposal(
            "bom_build",
            build,
            f"Speed steps on {current} are not improving the score. The last heavy lap "
            f"in the notes was -5.0% with shaft 0.866 Wh, before BoM mass was on the body. "
            f"The contact cylinder stays 0.08 m. Switching the body to {build} measures "
            f"that build instead of fitting the model.",
            f"A finished {build} lap becomes that build's baseline. It is not compared "
            f"with {current} as if they were the same robot.",
            suggestions,
        )


class LlmPicker:
    """Optional picker. Invalid output falls back to :class:`RulePicker`."""

    def __init__(self, command: str, fallback: RulePicker | None = None):
        self.command = (command or "").strip()
        self.fallback = fallback or RulePicker()

    def pick(self, diagnosis, knowledge: str, history: list[dict], params: dict) -> Proposal | None:
        if not self.command:
            proposal = self.fallback.pick(diagnosis, knowledge, history, params)
            if proposal is not None:
                proposal.note = "LLM picker has no OPTIMIZER_LLM_CMD; used the rule picker"
            return proposal
        payload = {
            "allowlist": {
                key: {
                    "kind": knob.kind,
                    "lo": knob.lo,
                    "hi": knob.hi,
                    "step": knob.step,
                    "choices": list(knob.choices),
                }
                for key, knob in KNOBS.items()
            },
            "current": params,
            "knowledge": knowledge,
            "history": history,
            "diagnosis_notes": list(getattr(diagnosis, "notes", []) or []),
            "instruction": (
                "Return one JSON object with param, value, hypothesis, expected_effect, "
                "and optional code_suggestions (strings). Do not include a patch or file edit. "
                "param must be on the allowlist."
            ),
        }
        try:
            proc = subprocess.run(
                self.command,
                input=json.dumps(payload),
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=120,
                shell=True,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return self._fallback(diagnosis, knowledge, history, params, f"LLM command failed: {exc}")
        if proc.returncode != 0:
            return self._fallback(
                diagnosis,
                knowledge,
                history,
                params,
                f"LLM command exit {proc.returncode}",
            )
        try:
            data = json.loads(proc.stdout)
        except json.JSONDecodeError:
            return self._fallback(diagnosis, knowledge, history, params, "LLM output was not JSON")
        if not isinstance(data, dict):
            return self._fallback(diagnosis, knowledge, history, params, "LLM output was not an object")
        if any(key in data for key in ("patch", "file_edit", "diff", "files")):
            return self._fallback(
                diagnosis,
                knowledge,
                history,
                params,
                "LLM proposal included a file edit and was ignored",
            )
        try:
            param = str(data["param"])
            value = check_param(param, data.get("value"))
            trial = dict(params)
            trial[param] = "" if value is None else value
            check_set(trial)
        except (AllowlistError, KeyError) as exc:
            return self._fallback(diagnosis, knowledge, history, params, f"LLM proposal rejected: {exc}")
        if _same(params.get(param), value):
            return self._fallback(
                diagnosis, knowledge, history, params, "LLM proposal repeated the current value"
            )
        hypothesis = str(data.get("hypothesis") or "").strip()
        expected = str(data.get("expected_effect") or "").strip()
        if not hypothesis or not expected:
            return self._fallback(
                diagnosis,
                knowledge,
                history,
                params,
                "LLM proposal was missing hypothesis or expected_effect",
            )
        suggestions = data.get("code_suggestions") or []
        if isinstance(suggestions, str):
            suggestions = [suggestions]
        return Proposal(
            param,
            value,
            hypothesis,
            expected,
            [str(item) for item in suggestions],
            note="picked by OPTIMIZER_LLM_CMD",
        )

    def _fallback(self, diagnosis, knowledge, history, params, note: str) -> Proposal | None:
        proposal = self.fallback.pick(diagnosis, knowledge, history, params)
        if proposal is None:
            return None
        proposal.note = note
        return proposal
