"""The unattended loop: diagnose, pick, run, keep or roll back, then restore config."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

from optimizer.baseline import (
    format_baseline_from_summary,
    format_kept_baseline_label,
    repeat_of_prior_lap,
)
from optimizer.config_store import ConfigStore
from optimizer.diagnose import Diagnosis, diagnose, diagnose_lap
from optimizer.knowledge import append_learning
from optimizer.logbook import Logbook, redact
from optimizer.notify import notify_run_finished
from optimizer.confirm import MAX_CONFIRM_PAIR_RETRIES, confirm_invalid_retryable
from optimizer.held_out import cross_build_confirm_disabled_reason, confirm_strategy_note
from optimizer.lap_launch import LapLaunch, assert_light_search_config, trial_id_for, wait_config
from optimizer.picker import Proposal, RulePicker
from optimizer.postmortem import build_postmortem
from optimizer.agent_journal import append_agent_journal
from optimizer.report import write_report
from optimizer.runner import REPO_ROOT
from optimizer.runner import RunOutcome
from optimizer.evidence import evidence_status, measure_effects, meaningful_improvement
from optimizer.score import (
    ERR_REGRESS_POINTS,
    KEEP_MARGIN,
    decide,
    knob_confirm_comparison_ok,
    lap_metrics_match,
)
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
    if rec.get("decision") == "invalid":
        param = rec.get("param")
        new = rec.get("new")
        if param:
            return (
                f"Retry `{param}` at {new} once more before trying another knob or direction. "
                "The last lap was invalid (missing summary, limits, or body mismatch)."
            )
    if rec.get("decision") == "keep":
        return (
            f"From that lesson, hold `{rec.get('param')}` at {rec.get('new')} and run the "
            "pre-keep vs kept confirm pair on the run body before trying another knob. "
            "The lesson stands only if that pair passes."
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


def _apply_evidence(rec: dict, effects: dict | None, *, flat: bool = False) -> None:
    if not effects:
        return
    post = rec.get("postmortem") if isinstance(rec.get("postmortem"), dict) else {}
    verdict = str(post.get("verdict") or "")
    rec["err_gain_pp"] = effects.get("err_gain_pp")
    rec["wpm_gain_rel"] = effects.get("wpm_gain_rel")
    rec["turn_delta_pp"] = effects.get("turn_delta_pp")
    rec["turn_noise_band_pp"] = effects.get("turn_noise_band_pp")
    if rec.get("score_baseline_label"):
        effects = dict(effects)
        effects.setdefault("baseline_label", rec["score_baseline_label"])
    rec["evidence_status"] = evidence_status(
        action=str(rec.get("decision") or ""),
        post_verdict=verdict,
        meaningful=bool(effects.get("meaningful")),
        flat=flat,
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
    within_noise: bool = False,
    noise_detail: str = "",
    summary_trusted: bool = True,
    effects: dict | None = None,
    repeat_label: str | None = None,
    baseline_diag: Diagnosis | None = None,
    baseline_label: str = "kept baseline",
) -> dict:
    post = build_postmortem(
        param=rec.get("param"),
        old=rec.get("old"),
        new=rec.get("new"),
        action=str(rec.get("decision") or ""),
        before=before,
        after=after,
        intended=rec.get("intended_params"),
        within_noise=within_noise,
        noise_detail=noise_detail,
        summary_trusted=summary_trusted,
        evidence=effects,
        repeat_label=repeat_label,
        baseline_diag=baseline_diag,
        baseline_label=baseline_label,
    )
    rec["postmortem"] = post
    if rec.get("decision") == "invalid" and rec.get("reason"):
        from optimizer.postmortem import invalid_lesson

        post["mechanism"] = str(rec["reason"])
        post["lesson"] = invalid_lesson(str(rec["reason"]))
        post["verdict"] = "invalid"
        post["tag"] = ""
    rec["lesson"] = post["lesson"]
    rec["mechanism"] = post["mechanism"]
    post["next_hypothesis"] = _next_hypothesis(picker, next_diag, knowledge_path, history, rec, store)
    rec["next_hypothesis"] = post["next_hypothesis"]
    _apply_evidence(rec, effects, flat=bool(post.get("flat")))
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


def _run_with_retry(
    runner,
    params: dict,
    timeout_s: float,
    world: str,
    log_path: Path | None,
    *,
    launch: LapLaunch,
) -> RunOutcome:
    def once() -> RunOutcome:
        try:
            return runner.run_lap(
                params=params,
                timeout_s=timeout_s,
                world=world,
                log_path=log_path,
                launch=launch,
            )
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
    agent_journal_path: Path | None = None,
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
    # Leading letter so MISSION_SUMMARY kv parsing does not truncate at ISO ``T``.
    run_id = f"r{started.strftime('%Y%m%d%H%M%S')}"
    book.begin_run(run_id)
    t0 = clock()
    records: list[dict] = []
    history: list[dict] = []
    diagnoses: dict = {}
    starting_references: dict = {}
    score_baselines: dict = {}
    baseline_labels: dict = {}
    score_baseline_diags: dict = {}
    starting_reference_diags: dict = {}
    prior_laps: list[tuple[str, object]] = []
    pending: dict | None = None
    retry_search: Proposal | None = None
    retry_search_old = None
    stop_reason = "iteration limit"
    iteration = 0
    run_failed = False

    def over_time() -> bool:
        return (clock() - t0) >= float(max_hours) * 3600.0

    def remember_diag(diag: Diagnosis) -> None:
        summary = diag.summary
        if summary is None or summary.err_pct is None:
            return
        if not body_matches_run(summary, run_body):
            return
        if trial_invalid_reason(summary, store.read(), run_body):
            return
        diagnoses[run_body] = diag

    def set_score_baseline(summary, label: str, diag: Diagnosis | None = None) -> None:
        score_baselines[run_body] = summary
        baseline_labels[run_body] = label
        if diag is not None:
            score_baseline_diags[run_body] = diag
        elif summary is not None:
            score_baseline_diags[run_body] = Diagnosis(summary=summary)

    def catalog_lap(label: str, summary) -> None:
        if summary is not None:
            prior_laps.append((label, summary))

    def run_reference_lap() -> bool:
        """One lap at starting settings; scores all later trials against it."""
        nonlocal diag, stop_reason
        if over_time():
            stop_reason = "wall-clock limit before the reference lap"
            return False
        ref_launch = LapLaunch.new(run_id, trial_id_for("reference", 0))
        outcome = _run_with_retry(
            runner,
            store.read(),
            lap_timeout_s,
            world,
            run_dir / "launch_reference.log",
            launch=ref_launch,
        )
        ref_diag = diagnose(outcome.log_text)
        intended = store.read()
        invalid = trial_invalid_reason(
            ref_diag.summary,
            intended,
            run_body,
            launch=ref_launch,
        )
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
        rec["run_id"] = run_id
        rec["score_baseline_label"] = baseline_labels.get(run_body, "starting settings")
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
            remember_diag(ref_diag)
            starting_references[run_body] = ref_diag.summary
            starting_reference_diags[run_body] = ref_diag
            set_score_baseline(
                ref_diag.summary,
                format_baseline_from_summary(ref_diag.summary, prefix="starting settings"),
                ref_diag,
            )
            catalog_lap("reference lap", ref_diag.summary)
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
        if max_iterations > 0 and run_body not in score_baselines:
            if not run_reference_lap():
                diag = diag if diag.summary is not None else diagnose("")

        while iteration < max_iterations and run_body in score_baselines:
            if over_time():
                stop_reason = "wall-clock limit"
                break
            knowledge = ""
            if knowledge_path.is_file():
                knowledge = knowledge_path.read_text(encoding="utf-8")
            params = store.read()
            assert_light_search_config(params, run_body)

            if pending is not None:
                iteration += 1
                restart_note = ""
                light_kept = dict(pending.get("light_kept_params") or store.read())
                param = pending["param"]
                pending_old = pending["old"]
                pending_new = pending["new"]
                pending_hypothesis = pending["hypothesis"]
                pending_expected = pending["expected"]
                pending_suggestions = list(pending["suggestions"])
                pending_prev_diag = pending["prev_diag"]
                pre_diag: Diagnosis | None = None
                kept_diag: Diagnosis | None = None
                confirm_notes: list[str] = []
                invalid_reason: str | None = None
                verdict = "missing"

                def _confirm_params(knob_value) -> dict:
                    merged = dict(light_kept)
                    merged[param] = knob_value
                    merged["bom_build"] = run_body
                    return merged

                try:
                    settle_note = getattr(runner, "settle_between_laps", None)
                    if int(pending.get("confirm_retries") or 0) > 0 and settle_note is not None:
                        confirm_notes.append(settle_note())
                    for idx, (part, knob_value) in enumerate(
                        (
                            ("pre-keep", pending["old"]),
                            ("kept", pending["new"]),
                        )
                    ):
                        if idx > 0 and settle_note is not None:
                            confirm_notes.append(settle_note())
                        intended = _confirm_params(knob_value)
                        store.write_params(intended)
                        cfg_wait = wait_config(store, intended)
                        if cfg_wait:
                            invalid_reason = cfg_wait
                            break
                        launch = LapLaunch.new(
                            run_id,
                            trial_id_for("confirm", iteration, part=part),
                        )
                        outcome = _run_with_retry(
                            runner,
                            intended,
                            lap_timeout_s,
                            world,
                            run_dir / f"launch_{iteration}_confirm_{part}.log",
                            launch=launch,
                        )
                        confirm_notes.append(outcome.note)
                        if outcome.status != "ok" or "MISSION_SUMMARY" not in (outcome.log_text or ""):
                            invalid_reason = (
                                f"{outcome.status}: {outcome.note or 'confirm lap did not finish'}"
                            )
                            break
                        lap_diag = diagnose(outcome.log_text)
                        invalid_reason = trial_invalid_reason(
                            lap_diag.summary,
                            intended,
                            run_body,
                            launch=launch,
                        )
                        if invalid_reason:
                            break
                        if part == "pre-keep":
                            pre_diag = lap_diag
                        else:
                            kept_diag = lap_diag
                finally:
                    store.write_params(light_kept)
                    assert_light_search_config(store.read(), run_body)

                why = ""
                if invalid_reason:
                    verdict = "missing"
                    why = invalid_reason
                else:
                    verdict, why = knob_confirm_comparison_ok(pre_diag, kept_diag)
                check_diag = kept_diag or pre_diag
                check_summary = None if check_diag is None else check_diag.summary
                if verdict == "pass":
                    action = "confirm-pass"
                    if pending["provisional"] is not None:
                        set_score_baseline(
                            pending["provisional"],
                            format_kept_baseline_label(
                                light_kept,
                                prefix="kept baseline (confirmed)",
                            ),
                            pending.get("provisional_diag"),
                        )
                    diag = pending["prev_diag"]
                    pending = None
                elif verdict in {"missing", "missing_turn"} or invalid_reason:
                    action = "invalid"
                    retries = int(pending.get("confirm_retries") or 0)
                    if confirm_invalid_retryable(why) and retries < MAX_CONFIRM_PAIR_RETRIES:
                        pending["confirm_retries"] = retries + 1
                        diag = pending["prev_diag"]
                    else:
                        store.apply_one(pending["param"], pending["old"])
                        set_score_baseline(
                            starting_references.get(run_body),
                            format_baseline_from_summary(
                                starting_references.get(run_body),
                                prefix="starting settings",
                            ),
                            starting_reference_diags.get(run_body),
                        )
                        diag = pending["prev_diag"]
                        why = why + " Confirm retries exhausted; the provisional keep was rolled back."
                        pending = None
                else:
                    action = "confirm-fail"
                    store.apply_one(pending["param"], pending["old"])
                    set_score_baseline(
                        starting_references.get(run_body),
                        format_baseline_from_summary(
                            starting_references.get(run_body),
                            prefix="starting settings",
                        ),
                        starting_reference_diags.get(run_body),
                    )
                    diag = pending["prev_diag"]
                    why = why + " The change was rolled back."
                    pending = None
                if action == "invalid":
                    check_summary = None
                    check_diag = None
                proposal = Proposal(
                    param,
                    pending_new,
                    pending_hypothesis,
                    pending_expected,
                    pending_suggestions,
                    note=restart_note,
                )
                rec = _record(
                    iteration=iteration,
                    phase="confirm",
                    proposal=proposal,
                    old=pending_old,
                    action=action,
                    reason=why,
                    summary=check_summary,
                    score=None,
                    note=(" ".join(confirm_notes) + " " + restart_note).strip(),
                )
                rec["intended_params"] = light_kept
                if pre_diag is not None and pre_diag.summary is not None:
                    rec["confirm_pre_keep_summary"] = {
                        "err_pct": pre_diag.summary.err_pct,
                        "wh_measured_drive_elec": pre_diag.summary.wh_measured_drive_elec,
                        "turn_err": pre_diag.turn_err,
                    }
                _attach_postmortem(
                    rec,
                    before=pending_prev_diag,
                    after=check_diag,
                    knowledge_path=knowledge_path,
                    history=history,
                    picker=picker,
                    store=store,
                    next_diag=diag,
                    summary_trusted=check_summary is not None,
                )
                records.append(book.append(rec))
                history.append(rec)
                append_learning(
                    knowledge_path,
                    redact(_learning(
                        iteration, "confirm", proposal, action, why, check_summary, rec["postmortem"],
                    )),
                )
                continue

            search_is_retry = False
            old = None
            if retry_search is not None:
                proposal = retry_search
                old = retry_search_old
                search_is_retry = True
                retry_search = None
                retry_search_old = None
            else:
                proposal = picker.pick(diag, knowledge, history, params)
            if proposal is None:
                stop_reason = "picker had no further allowlisted change"
                break
            iteration += 1
            if old is None:
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
            search_launch = LapLaunch.new(run_id, trial_id_for("search", iteration))
            outcome = _run_with_retry(
                runner,
                store.read(),
                lap_timeout_s,
                world,
                run_dir / f"launch_{iteration}.log",
                launch=search_launch,
            )
            summary_trusted = "MISSION_SUMMARY" in (outcome.log_text or "")
            new_diag = diagnose_lap(outcome.log_text) if summary_trusted else diagnose_lap("")
            summary = new_diag.summary
            intended_params = store.read()
            baseline = score_baselines.get(run_body)
            baseline_label = baseline_labels.get(run_body, "kept baseline")
            before_diag = diag
            effects: dict | None = None
            within_noise = False
            noise_detail = ""
            repeat_label: str | None = None
            decision = None
            if outcome.status == "no_summary" or (
                outcome.status == "ok" and not summary_trusted
            ):
                store.apply_one(proposal.param, old)
                action = "invalid"
                reason = (
                    "lap had no MISSION_SUMMARY / did not finish; "
                    "no metrics from this lap were scored"
                )
                score = None
                summary = None
                new_diag = diagnose_lap("")
                diag = before_diag
                if not search_is_retry:
                    retry_search = proposal
                    retry_search_old = old
            elif outcome.status != "ok":
                store.apply_one(proposal.param, old)
                action = "invalid"
                reason = f"{outcome.status}: {outcome.note or 'lap failed'}"
                score = None
                summary = None
                new_diag = diagnose_lap("")
                diag = before_diag
                if not search_is_retry:
                    retry_search = proposal
                    retry_search_old = old
            elif (
                invalid := trial_invalid_reason(
                    summary,
                    intended_params,
                    run_body,
                    launch=search_launch,
                )
            ):
                store.apply_one(proposal.param, old)
                action = "invalid"
                reason = invalid
                score = None
                summary = None
                new_diag = diagnose_lap("")
                diag = before_diag
                if not search_is_retry:
                    retry_search = proposal
                    retry_search_old = old
            else:
                repeat_label = repeat_of_prior_lap(
                    summary,
                    starting_reference=starting_references.get(run_body),
                    prior_laps=prior_laps,
                )
                if repeat_label:
                    store.apply_one(proposal.param, old)
                    action = "inconclusive"
                    reason = (
                        f"lap repeats {repeat_label} with identical metrics; "
                        "not scored as improvement vs the kept baseline"
                    )
                    score = None
                    effects = measure_effects(
                        new_diag,
                        baseline,
                        baseline_diag=score_baseline_diags.get(run_body),
                        baseline_label=baseline_label,
                    )
                    within_noise = True
                    noise_detail = reason
                    diag = before_diag
                if repeat_label is None:
                    effects = measure_effects(
                        new_diag,
                        baseline,
                        baseline_diag=score_baseline_diags.get(run_body),
                        baseline_label=baseline_label,
                    )
                    within_noise = not effects.get("meaningful")
                    noise_detail = str(effects.get("detail") or "")
                    clearly_worse = False
                    if (
                        summary is not None
                        and baseline is not None
                        and summary.err_pct is not None
                        and baseline.err_pct is not None
                    ):
                        err_regress = abs(summary.err_pct) - abs(baseline.err_pct)
                        clearly_worse = err_regress > ERR_REGRESS_POINTS
                    decision = decide(
                        summary, baseline, baseline_label=baseline_label
                    )
                    score = decision.score
                    reason = decision.reason
                if repeat_label is None and decision.action == "keep" and summary is not None:
                    if within_noise:
                        store.apply_one(proposal.param, old)
                        action = "inconclusive"
                        reason = (
                            f"score {decision.score:.3f} beat {KEEP_MARGIN} but the move was "
                            f"within lap noise ({noise_detail})"
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
                            "provisional_diag": new_diag,
                            "search_build": run_body,
                            "prev_diag": diag,
                            "light_kept_params": dict(intended_params),
                        }
                        set_score_baseline(
                            summary,
                            format_kept_baseline_label(
                                intended_params,
                                prefix="kept baseline (provisional)",
                            ),
                            new_diag,
                        )
                        remember_diag(new_diag)
                        diag = new_diag
                elif repeat_label is None and decision.action == "baseline":
                    store.apply_one(proposal.param, old)
                    action = "inconclusive"
                    reason = (
                        "no kept baseline was recorded for this build; "
                        "the trial was not scored"
                    )
                    diag = before_diag
                elif (
                    repeat_label is None
                    and lap_metrics_match(summary, baseline)
                    if baseline
                    else False
                ):
                    store.apply_one(proposal.param, old)
                    action = "inconclusive"
                    reason = decision.reason
                    diag = before_diag
                elif repeat_label is None and within_noise and not clearly_worse:
                    store.apply_one(proposal.param, old)
                    action = "inconclusive"
                    reason = f"{decision.reason}; {noise_detail}".strip("; ")
                    diag = before_diag
                elif repeat_label is None:
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
            rec["run_id"] = run_id
            rec["score_baseline_label"] = baseline_label
            if summary is not None and action not in {"invalid"}:
                catalog_lap(f"search {iteration}", summary)
            _attach_postmortem(
                rec,
                before=before_diag,
                after=new_diag,
                knowledge_path=knowledge_path,
                history=history,
                picker=picker,
                store=store,
                next_diag=diag,
                within_noise=within_noise and action == "inconclusive",
                noise_detail=noise_detail,
                summary_trusted=summary_trusted,
                effects=effects,
                repeat_label=repeat_label,
                baseline_diag=score_baseline_diags.get(run_body),
                baseline_label=baseline_label,
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
        finished = _utc_now()
        report_path = write_report(
            run_dir / "morning.md",
            started=started,
            finished=finished,
            records=records,
            snapshot=snapshot,
            diagnoses=diagnoses,
            dry_run=dry_run,
            stop_reason=stop_reason,
            held_out_note=confirm_strategy_note(),
            cross_build_note=cross_build_confirm_disabled_reason(),
        )
        try:
            append_agent_journal(
                agent_journal_path or (REPO_ROOT / "docs" / "AGENT_JOURNAL.md"),
                run_id=run_id,
                started=started,
                finished=finished,
                records=records,
                stop_reason=stop_reason,
            )
        except OSError:
            pass
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
