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
from optimizer.picker import Proposal, RulePicker, held_out_build
from optimizer.report import write_report
from optimizer.runner import RunOutcome
from optimizer.score import confirmation_ok, decide

SESSION_NAME = "session.json"


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _learning(iteration: int, phase: str, proposal: Proposal | None, action: str, reason: str, summary) -> str:
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
    if proposal is not None and proposal.code_suggestions:
        lines.append("- Code suggestions (not applied):")
        for item in proposal.code_suggestions:
            lines.append(f"  - {item}")
    return "\n".join(lines)


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

    def over_time() -> bool:
        return (clock() - t0) >= float(max_hours) * 3600.0

    def remember(diag: Diagnosis) -> None:
        if diag.summary is not None and diag.summary.build:
            diagnoses[diag.summary.build] = diag
            if diag.summary.build not in references and diag.summary.err_pct is not None:
                references[diag.summary.build] = diag.summary

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
        for build, summary in diag.per_build.items():
            references[build] = summary
            diagnoses[build] = diag if diag.summary and diag.summary.build == build else diagnose(
                summary.raw
            )
        if diag.summary is None:
            if over_time():
                stop_reason = "wall-clock limit before the baseline lap"
            else:
                outcome = _run_with_retry(
                    runner, store.read(), lap_timeout_s, world, run_dir / "launch_baseline.log"
                )
                diag = diagnose(outcome.log_text)
                decision = decide(diag.summary, None)
                action = "baseline" if decision.action == "baseline" else decision.action
                rec = _record(
                    iteration=0,
                    phase="baseline",
                    proposal=None,
                    old=None,
                    action=action,
                    reason=decision.reason if diag.summary else outcome.note or decision.reason,
                    summary=diag.summary,
                    score=decision.score,
                    note=outcome.note,
                )
                records.append(book.append(rec))
                history.append(rec)
                append_learning(
                    knowledge_path,
                    redact(_learning(0, "baseline", None, action, rec["reason"], diag.summary)),
                )
                remember(diag)
                if diag.summary is None:
                    stop_reason = "baseline lap produced no MISSION_SUMMARY"

        while iteration < max_iterations and diag.summary is not None:
            if over_time():
                stop_reason = "wall-clock limit"
                break
            knowledge = ""
            if knowledge_path.is_file():
                knowledge = knowledge_path.read_text(encoding="utf-8")
            params = store.read()

            if pending is not None:
                iteration += 1
                search_build = pending["search_build"] or ""
                same_build = pending["param"] == "bom_build"
                saved_build = params.get("bom_build") or ""
                held = search_build if same_build else held_out_build(search_build)
                if not same_build and held != saved_build:
                    store.apply_one("bom_build", held)
                    restart_note = runner.restart_dashboard()
                else:
                    restart_note = ""
                outcome = _run_with_retry(
                    runner,
                    store.read(),
                    lap_timeout_s,
                    world,
                    run_dir / f"launch_{iteration}_confirm.log",
                )
                check = diagnose(outcome.log_text)
                ok, why = confirmation_ok(pending["provisional"], check.summary, same_build=same_build)
                if not same_build:
                    store.apply_one("bom_build", saved_build)
                    if held != saved_build:
                        runner.restart_dashboard()
                if ok:
                    action = "confirm-pass"
                    if search_build and pending["provisional"] is not None:
                        references[search_build] = pending["provisional"]
                    diag = pending["prev_diag"]
                else:
                    action = "confirm-fail"
                    store.apply_one(pending["param"], pending["old"])
                    if pending["param"] == "bom_build":
                        runner.restart_dashboard()
                    diag = pending["prev_diag"]
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
                records.append(book.append(rec))
                history.append(rec)
                append_learning(
                    knowledge_path,
                    redact(_learning(iteration, "confirm", proposal, action, why, check.summary)),
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
                records.append(book.append(rec))
                history.append(rec)
                continue
            restart_note = ""
            if proposal.param == "bom_build":
                restart_note = runner.restart_dashboard()
                if restart_note.startswith("failed"):
                    store.apply_one(proposal.param, old)
                    rec = _record(
                        iteration=iteration,
                        phase="search",
                        proposal=proposal,
                        old=old,
                        action="invalid",
                        reason=restart_note,
                        summary=None,
                        score=None,
                        note=restart_note,
                    )
                    records.append(book.append(rec))
                    history.append(rec)
                    append_learning(
                        knowledge_path,
                        redact(_learning(iteration, "search", proposal, "invalid", restart_note, None)),
                    )
                    continue
            outcome = _run_with_retry(
                runner, store.read(), lap_timeout_s, world, run_dir / f"launch_{iteration}.log"
            )
            new_diag = diagnose(outcome.log_text)
            summary = new_diag.summary
            build = None if summary is None else summary.build
            if not build and proposal.param == "bom_build":
                build = proposal.value or None
            reference = None if not build else references.get(build)
            decision = decide(summary, reference)
            if decision.action == "baseline" and summary is not None and build:
                references[build] = summary
                diagnoses[build] = new_diag
                action = "baseline"
                diag = new_diag
            elif decision.action == "keep" and summary is not None:
                action = "keep"
                pending = {
                    "param": proposal.param,
                    "old": old,
                    "new": proposal.value,
                    "hypothesis": proposal.hypothesis,
                    "expected": proposal.expected_effect,
                    "suggestions": list(proposal.code_suggestions),
                    "provisional": summary,
                    "search_build": build or (params.get("bom_build") or ""),
                    "prev_diag": diag,
                }
                if build:
                    diagnoses[build] = new_diag
                diag = new_diag
            else:
                store.apply_one(proposal.param, old)
                if proposal.param == "bom_build":
                    runner.restart_dashboard()
                action = "rollback"
            reason = decision.reason
            if outcome.status != "ok":
                reason = f"{outcome.status}: {outcome.note or decision.reason}"
            rec = _record(
                iteration=iteration,
                phase="search",
                proposal=proposal,
                old=old,
                action=action,
                reason=reason,
                summary=summary,
                score=decision.score,
                note=(outcome.note + " " + restart_note).strip(),
            )
            records.append(book.append(rec))
            history.append(rec)
            append_learning(
                knowledge_path,
                redact(_learning(iteration, "search", proposal, action, reason, summary)),
            )
    finally:
        if pending is not None:
            stop_reason = f"{stop_reason}; last keep was not confirmed before the stop"
        try:
            store.write_params(snapshot)
        except Exception as exc:
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
    return report_path
