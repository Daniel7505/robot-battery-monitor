"""The unattended loop: diagnose, pick, run, keep or roll back, then restore config."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

from optimizer.config_store import ConfigStore
from optimizer.diagnose import Diagnosis, diagnose
from optimizer.knowledge import append_learning
from optimizer.logbook import Logbook, redact
from optimizer.notify import notify_run_finished
from optimizer.picker import Proposal, RulePicker
from optimizer.postmortem import build_postmortem
from optimizer.report import write_report
from optimizer.runner import RunOutcome
from optimizer.evidence import meaningful_improvement
from optimizer.score import KEEP_MARGIN, confirmation_ok, decide
from optimizer.trial_validity import body_matches_run, run_body_from_params, trial_invalid_reason

SESSION_NAME = "session.json"


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _learning(
    iteration: int,
    phase: str,
    proposal: Proposal | None,
    action: str,
    reason: str,
    summary,
    postmortem: dict | None = None,
) -> str:
    stamp = _utc_now().strftime("%Y-%m-%dT%H:%M:%SZ")
    lines = [f"### {stamp} iteration {iteration} ({phase})"]
    if proposal is not None:
        lines.append(f"- Change: `{proposal.param}` → `{proposal.value}`")
        lines.append(f"- Hypothesis: {proposal.hypothesis}")
        lines.append(f"- Expected: {proposal.expected_effect}")
        if proposal.note:
            lines.append(f"- Picker note: {proposal.note}")
    lines.append(f"- Result: {action}. {reason}")
    if summary is not None and summary.err_pct is not None:
        lines.append(
            f"- Measured: build={summary.build} err_pct={summary.err_pct} "
            f"shaft={summary.wh_measured_drive_shaft} Wh "
            f"elec={summary.wh_measured_drive_elec} Wh dist={summary.dist_m} m"
        )
    if postmortem:
        lesson = str(postmortem.get("lesson") or "").strip()
        tag = str(postmortem.get("tag") or "").strip()
        if lesson or tag:
            lines.append(f"- Lesson: {lesson} {tag}".rstrip())
        nxt = str(postmortem.get("next_hypothesis") or "").strip()
        if nxt:
            lines.append(f"- Next hypothesis: {nxt}")
    if proposal is not None and proposal.code_suggestions:
        lines.append("- Code suggestions (not applied):")
        for item in proposal.code_suggestions:
            lines.append(f"  - {item}")
    return "\n".join(lines)


def _knowledge_text(path: Path) -> str:
    if path.is_file():
        try:
            return path.read_text(encoding="utf-8")
        except OSError:
            return ""
    return ""


def _next_hypothesis(picker, diag, knowledge_path: Path, history: list[dict], rec: dict, store) -> str:
    """What the picker would try next, after this trial's lesson is in the book."""
    post = rec.get("postmortem") or {}
    if rec.get("decision") == "keep":
        return (
            f"From that lesson, hold `{rec.get('param')}` at {rec.get('new')} and run the "
            "held-out lap before trying another knob. The lesson stands only if that lap agrees."
        )
    extra = ""
    tag = str(post.get("tag") or "").strip()
    lesson = str(post.get("lesson") or "").strip()
    if tag or lesson:
        extra = f"\n- Lesson: {lesson} {tag}\n"
    try:
        proposal = picker.pick(
            diag,
            _knowledge_text(knowledge_path) + extra,
            list(history) + [rec],
            store.read(),
        )
    except Exception as exc:
        return f"Could not derive the next hypothesis ({type(exc).__name__}: {exc})."
    if proposal is None:
        return "From that lesson, no allowlisted step is left."
    return (
        f"From that lesson, try `{proposal.param}` at {proposal.value} "
        f"rather than repeating the step that just finished. {proposal.hypothesis}"
    )


def _attach_postmortem(
    rec: dict,
    *,
    before,
    after,
    knowledge_path: Path,
    history: list[dict],
    picker,
    store,
    next_diag,
) -> dict:
    post = build_postmortem(
        param=rec.get("param"),
        old=rec.get("old"),
        new=rec.get("new"),
        action=str(rec.get("decision") or ""),
        before=before,
        after=after,
        intended=rec.get("intended_params"),
    )
    rec["postmortem"] = post
    if rec.get("decision") == "invalid" and rec.get("reason"):
        post["mechanism"] = str(rec["reason"])
        post["verdict"] = "invalid"
        post["tag"] = ""
    rec["lesson"] = post["lesson"]
    rec["mechanism"] = post["mechanism"]
    post["next_hypothesis"] = _next_hypothesis(picker, next_diag, knowledge_path, history, rec, store)
    rec["next_hypothesis"] = post["next_hypothesis"]
    return rec


def _record(
    *,
    iteration: int,
    phase: str,
    proposal: Proposal | None,
    old,
    action: str,
    reason: str,
    summary,
    score,
    note: str,
) -> dict:
    wh_per_m = None if summary is None else summary.wh_per_m()
    return {
        "iteration": iteration,
        "utc": _utc_now().strftime("%Y-%m-%dT%H:%M:%SZ"),
        "phase": phase,
        "param": None if proposal is None else proposal.param,
        "old": old,
        "new": None if proposal is None else proposal.value,
        "hypothesis": "" if proposal is None else proposal.hypothesis,
        "expected_effect": "" if proposal is None else proposal.expected_effect,
        "decision": action,
        "reason": reason,
        "build": None if summary is None else summary.build,
        "err_pct": None if summary is None else summary.err_pct,
        "wh_model_drive": None if summary is None else summary.wh_model_drive,
        "wh_measured_drive_elec": None if summary is None else summary.wh_measured_drive_elec,
        "wh_measured_drive_shaft": None if summary is None else summary.wh_measured_drive_shaft,
        "wh_per_m": wh_per_m,
        "dist_m": None if summary is None else summary.dist_m,
        "duration_s": None if summary is None else summary.duration_s,
        "score": score,
        "note": note,
        "code_suggestions": [] if proposal is None else list(proposal.code_suggestions),
    }


def recover_session(store: ConfigStore, session_path: Path) -> dict | None:
    """If a previous run died mid-lap, put config back before doing anything else."""
    if not session_path.is_file():
        return None
    try:
        data = json.loads(session_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if data.get("in_progress") and isinstance(data.get("snapshot"), dict):
        store.write_params(data["snapshot"])
        return data["snapshot"]
    return None


def _run_with_retry(runner, params: dict, timeout_s: float, world: str, log_path: Path | None) -> RunOutcome:
    def once() -> RunOutcome:
        try:
            return runner.run_lap(params=params, timeout_s=timeout_s, world=world, log_path=log_path)
        except Exception as exc:
            return RunOutcome("crash", "", note=f"{type(exc).__name__}: {exc}")

    first = once()
    if first.status == "ok" and "MISSION_SUMMARY" in first.log_text:
        return first
    second = once()
    second.note = f"retried after {first.status}. {first.note} {second.note}".strip()
    if second.status == "ok" and "MISSION_SUMMARY" in second.log_text:
        return second
    if second.status == "ok":
        second.status = "no_summary"
    return second


def run_loop(
    *,
    config_path: Path,
    knowledge_path: Path,
    run_dir: Path,
    runner,
    picker=None,
    max_iterations: int = 10,
    max_hours: float = 8.0,
    lap_timeout_s: float = 40 * 60,
    world: str = "warehouse",
    initial_log: str | None = None,
    fetch_existing=None,
    monotonic=None,
    dry_run: bool = False,
    notify_url: str | None = None,
) -> Path:
    """Run until the iteration cap, the wall clock, or the picker has nothing left.

    ``config_path`` is restored to its starting allowlisted values before return,
    including when a lap runner raises.
    """
    clock = monotonic or time.monotonic
    picker = picker or RulePicker()
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    store = ConfigStore(config_path)
    session_path = run_dir / SESSION_NAME
    recover_session(store, session_path)
    snapshot = store.read()
    run_body = run_body_from_params(snapshot)
    if (snapshot.get("bom_build") or "") != run_body:
        store.apply_one("bom_build", run_body)
        try:
            runner.restart_dashboard()
        except Exception:
            pass
    session_path.write_text(
        json.dumps({"in_progress": True, "snapshot": snapshot}, sort_keys=True),
        encoding="utf-8",
    )
    book = Logbook(run_dir)
    started = _utc_now()
    t0 = clock()
    records: list[dict] = []
    history: list[dict] = []
    diagnoses: dict = {}
    references: dict = {}
    pending: dict | None = None
    stop_reason = "iteration limit"
    iteration = 0
    run_failed = False

    def over_time() -> bool:
        return (clock() - t0) >= float(max_hours) * 3600.0

    def remember(diag: Diagnosis) -> None:
        summary = diag.summary
        if summary is None or summary.err_pct is None:
            return
        if not body_matches_run(summary, run_body):
            return
        if trial_invalid_reason(summary, store.read(), run_body):
            return
        diagnoses[run_body] = diag
        references[run_body] = summary

    def run_reference_lap() -> bool:
        """One lap at starting settings; scores all later trials against it."""
        nonlocal diag, stop_reason
        if over_time():
            stop_reason = "wall-clock limit before the reference lap"
            return False
        outcome = _run_with_retry(
            runner, store.read(), lap_timeout_s, world, run_dir / "launch_reference.log"
        )
        ref_diag = diagnose(outcome.log_text)
        intended = store.read()
        invalid = trial_invalid_reason(ref_diag.summary, intended, run_body)
        if invalid:
            action = "invalid"
            reason = invalid
        elif ref_diag.summary is None:
            action = "invalid"
            reason = outcome.note or "reference lap produced no MISSION_SUMMARY"
        else:
            action = "reference"
            reason = "starting settings reference lap"
        rec = _record(
            iteration=0,
            phase="reference",
            proposal=None,
            old=None,
            action=action,
            reason=reason,
            summary=ref_diag.summary,
            score=1.0 if ref_diag.summary is not None else None,
            note=outcome.note,
        )
        rec["intended_params"] = intended
        _attach_postmortem(
            rec,
            before=None,
            after=ref_diag,
            knowledge_path=knowledge_path,
            history=history,
            picker=picker,
            store=store,
            next_diag=ref_diag if ref_diag.summary is not None else diag,
        )
        records.append(book.append(rec))
        history.append(rec)
        append_learning(
            knowledge_path,
            redact(_learning(
                0, "reference", None, action, reason, ref_diag.summary, rec["postmortem"],
            )),
        )
        if action == "reference":
            remember(ref_diag)
            diag = ref_diag
            return True
        stop_reason = reason
        return False

    try:
        log_text = initial_log
        if log_text is None and fetch_existing is not None:
            try:
                log_text = fetch_existing() or ""
            except Exception as exc:
                log_text = ""
                records.append(
                    _record(
                        iteration=0,
                        phase="diagnose",
                        proposal=None,
                        old=None,
                        action="note",
                        reason=f"could not read the existing dashboard log: {exc}",
                        summary=None,
                        score=None,
                        note="",
                    )
                )
        diag = diagnose(log_text or "")
        # Prior dashboard laps inform the picker only; the run's reference is a fresh lap.
        if max_iterations > 0 and run_body not in references:
            if not run_reference_lap():
                diag = diag if diag.summary is not None else diagnose("")

        while iteration < max_iterations and run_body in references:
            if over_time():
                stop_reason = "wall-clock limit"
                break
            knowledge = ""
            if knowledge_path.is_file():
                knowledge = knowledge_path.read_text(encoding="utf-8")
            params = store.read()

            if pending is not None:
                iteration += 1
                restart_note = ""
                outcome = _run_with_retry(
                    runner,
                    store.read(),
                    lap_timeout_s,
                    world,
                    run_dir / f"launch_{iteration}_confirm.log",
                )
                check = diagnose(outcome.log_text)
                invalid = trial_invalid_reason(check.summary, store.read(), run_body)
                if invalid:
                    ok, why = False, invalid
                else:
                    ok, why = confirmation_ok(
                        pending["provisional"], check.summary, same_build=True
                    )
                if ok:
                    action = "confirm-pass"
                    if pending["provisional"] is not None:
                        references[run_body] = pending["provisional"]
                    diag = pending["prev_diag"]
                else:
                    action = "confirm-fail" if not invalid else "invalid"
                    store.apply_one(pending["param"], pending["old"])
                    diag = pending["prev_diag"]
                    if not invalid:
                        why = why + " The change was rolled back."
                proposal = Proposal(
                    pending["param"],
                    pending["new"],
                    pending["hypothesis"],
                    pending["expected"],
                    list(pending["suggestions"]),
                    note=restart_note,
                )
                rec = _record(
                    iteration=iteration,
                    phase="confirm",
                    proposal=proposal,
                    old=pending["old"],
                    action=action,
                    reason=why,
                    summary=check.summary,
                    score=None,
                    note=(outcome.note + " " + restart_note).strip(),
                )
                rec["intended_params"] = store.read()
                _attach_postmortem(
                    rec,
                    before=pending["prev_diag"],
                    after=check,
                    knowledge_path=knowledge_path,
                    history=history,
                    picker=picker,
                    store=store,
                    next_diag=diag,
                )
                records.append(book.append(rec))
                history.append(rec)
                append_learning(
                    knowledge_path,
                    redact(_learning(
                        iteration, "confirm", proposal, action, why, check.summary, rec["postmortem"],
                    )),
                )
                if check.summary is not None and check.summary.build:
                    diagnoses[check.summary.build] = check
                pending = None
                continue

            proposal = picker.pick(diag, knowledge, history, params)
            if proposal is None:
                stop_reason = "picker had no further allowlisted change"
                break
            iteration += 1
            old = params.get(proposal.param)
            try:
                store.apply_one(proposal.param, proposal.value)
            except Exception as exc:
                rec = _record(
                    iteration=iteration,
                    phase="search",
                    proposal=proposal,
                    old=old,
                    action="invalid",
                    reason=f"rejected before launch: {exc}",
                    summary=None,
                    score=None,
                    note="",
                )
                _attach_postmortem(
                    rec,
                    before=diag,
                    after=None,
                    knowledge_path=knowledge_path,
                    history=history,
                    picker=picker,
                    store=store,
                    next_diag=diag,
                )
                records.append(book.append(rec))
                history.append(rec)
                append_learning(
                    knowledge_path,
                    redact(_learning(
                        iteration, "search", proposal, "invalid", rec["reason"], None, rec["postmortem"],
                    )),
                )
                continue
            restart_note = ""
            outcome = _run_with_retry(
                runner, store.read(), lap_timeout_s, world, run_dir / f"launch_{iteration}.log"
            )
            new_diag = diagnose(outcome.log_text)
            summary = new_diag.summary
            intended_params = store.read()
            reference = references.get(run_body)
            before_diag = diag
            if outcome.status != "ok":
                store.apply_one(proposal.param, old)
                action = "rollback"
                reason = f"{outcome.status}: {outcome.note or 'lap failed'}"
                score = None
                diag = before_diag
            elif (invalid := trial_invalid_reason(summary, intended_params, run_body)):
                store.apply_one(proposal.param, old)
                action = "invalid"
                reason = invalid
                score = None
                diag = before_diag
            else:
                decision = decide(summary, reference)
                score = decision.score
                reason = decision.reason
                if decision.action == "keep" and summary is not None:
                    meaningful, noise = meaningful_improvement(
                        before_diag, new_diag, reference
                    )
                    if not meaningful:
                        store.apply_one(proposal.param, old)
                        action = "rollback"
                        reason = (
                            f"score {decision.score:.3f} beat {KEEP_MARGIN} but the move was "
                            f"within lap noise ({noise})"
                        )
                        diag = before_diag
                    else:
                        action = "keep"
                        pending = {
                            "param": proposal.param,
                            "old": old,
                            "new": proposal.value,
                            "hypothesis": proposal.hypothesis,
                            "expected": proposal.expected_effect,
                            "suggestions": list(proposal.code_suggestions),
                            "provisional": summary,
                            "search_build": run_body,
                            "prev_diag": diag,
                        }
                        remember(new_diag)
                        diag = new_diag
                elif decision.action == "baseline":
                    store.apply_one(proposal.param, old)
                    action = "rollback"
                    reason = (
                        "no reference lap was recorded for this build; "
                        "the trial was not scored"
                    )
                    diag = before_diag
                else:
                    store.apply_one(proposal.param, old)
                    action = "rollback"
            rec = _record(
                iteration=iteration,
                phase="search",
                proposal=proposal,
                old=old,
                action=action,
                reason=reason,
                summary=summary,
                score=score,
                note=(outcome.note + " " + restart_note).strip(),
            )
            rec["intended_params"] = intended_params
            _attach_postmortem(
                rec,
                before=before_diag,
                after=new_diag,
                knowledge_path=knowledge_path,
                history=history,
                picker=picker,
                store=store,
                next_diag=diag,
            )
            records.append(book.append(rec))
            history.append(rec)
            append_learning(
                knowledge_path,
                redact(_learning(
                    iteration, "search", proposal, action, reason, summary, rec["postmortem"],
                )),
            )
    except Exception as exc:
        run_failed = True
        stop_reason = f"{stop_reason}; loop error: {type(exc).__name__}: {exc}"
        raise
    finally:
        if pending is not None:
            stop_reason = f"{stop_reason}; last keep was not confirmed before the stop"
        try:
            store.write_params(snapshot)
        except Exception as exc:
            run_failed = True
            stop_reason = f"{stop_reason}; config restore failed: {exc}"
        session_path.write_text(
            json.dumps({"in_progress": False, "snapshot": snapshot}, sort_keys=True),
            encoding="utf-8",
        )
        report_path = write_report(
            run_dir / "morning.md",
            started=started,
            finished=_utc_now(),
            records=records,
            snapshot=snapshot,
            diagnoses=diagnoses,
            dry_run=dry_run,
            stop_reason=stop_reason,
        )
        status = "error" if run_failed or any(
            word in stop_reason.lower() for word in ("crash", "timeout", "failed", "error")
        ) else "ok"
        summary = stop_reason
        if records:
            last = records[-1]
            summary = f"{last.get('decision')}: {last.get('reason') or stop_reason}"
        notify_run_finished(
            notify_url,
            status=status,
            iterations=len(records),
            report_path=report_path,
            summary=summary,
        )
    return report_path
