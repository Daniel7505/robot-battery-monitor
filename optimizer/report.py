"""Morning markdown, written from this session's trials."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from src.controller_limits import OPTIMIZER_TUNING_KEYS, PROTECTED_PATHS
from optimizer.trial_validity import RUN_BODY_DEFAULT


def _fmt(value) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def _trial_story(row: dict) -> list[str]:
    """Plain-language post-mortem for one trial, placed under that trial."""
    lines = [f"### Trial {row.get('iteration')} ({row.get('phase')})", ""]
    if row.get("param"):
        lines.append(
            f"Change: `{row.get('param')}` {_fmt(row.get('old'))} → {_fmt(row.get('new'))}."
        )
    hypothesis = (row.get("hypothesis") or "").strip()
    if hypothesis:
        lines.append(hypothesis)
    post = row.get("postmortem") if isinstance(row.get("postmortem"), dict) else None
    if post:
        lines.append("")
        lines.append("Predicted versus observed:")
        lines.append("")
        for metric in post.get("metrics") or []:
            label = metric.get("label") or "metric"
            predicted = metric.get("predicted") or ""
            observed = metric.get("observed") or ""
            lines.append(f"- {label}: predicted {predicted}; observed {observed}.")
        lines.append("")
        mechanism = (post.get("mechanism") or "").strip()
        if mechanism:
            lines.append(mechanism)
            lines.append("")
        lesson = (post.get("lesson") or row.get("lesson") or "").strip()
        if lesson:
            lines.append(f"Lesson: {lesson}")
        miss = (post.get("prediction_miss") or "").strip()
        if miss:
            lines.append(f"Prediction miss: {miss}")
        evidence = post.get("evidence") if isinstance(post.get("evidence"), dict) else None
        status = row.get("evidence_status") or (evidence and evidence.get("evidence_status"))
        if status or evidence or row.get("turn_noise_band_pp") is not None:
            lines.append("")
            lines.append("Evidence:")
            if status:
                lines.append(f"- Status: {status}")
            bl = row.get("score_baseline_label") or "kept baseline"
            lines.append(f"- Scored vs: {bl}")
            if row.get("err_gain_pp") is not None:
                lines.append(f"- |err| gain vs {bl}: {row.get('err_gain_pp'):.2f} pp")
            if row.get("wpm_gain_rel") is not None:
                lines.append(f"- Wh/m gain vs {bl}: {float(row.get('wpm_gain_rel')):.1%}")
            if row.get("turn_delta_pp") is not None:
                lines.append(
                    f"- Turn |err| delta: {row.get('turn_delta_pp'):.2f} pp "
                    f"(noise band {row.get('turn_noise_band_pp')} pp)"
                )
            elif row.get("turn_noise_band_pp") is not None:
                lines.append(f"- Turn segment noise band: {row.get('turn_noise_band_pp')} pp")
            if evidence and evidence.get("detail"):
                lines.append(f"- Detail: {evidence.get('detail')}")
        nxt = (post.get("next_hypothesis") or row.get("next_hypothesis") or "").strip()
        if nxt:
            lines.append(f"Next hypothesis: {nxt}")
    else:
        reason = (row.get("reason") or "").strip()
        if reason:
            lines.append(f"Result: {reason}")
    lines.append("")
    return lines


def render_report(
    *,
    started: datetime,
    finished: datetime,
    records: list[dict],
    snapshot: dict,
    diagnoses: dict,
    dry_run: bool,
    stop_reason: str,
    held_out_note: str = "",
    cross_build_note: str = "",
) -> str:
    lines = ["# Overnight optimizer", ""]
    if dry_run:
        lines.append(
            "This report is from `--dry-run`. No Webots lap was launched. "
            "The numbers are synthetic, so they are not a measurement of the robot."
        )
        lines.append("")
    lines.append(f"Started: {started.strftime('%Y-%m-%d %H:%M:%S UTC')}")
    lines.append(f"Finished: {finished.strftime('%Y-%m-%d %H:%M:%S UTC')}")
    lines.append(f"Trials logged: {len(records)}")
    lines.append(f"Stopped because: {stop_reason}")
    lines.append("")
    lines.append("The allowlisted keys were put back to the values from the start of the run.")
    lines.append("A winner is listed here so it can be copied into `config/config.yaml` by hand.")
    lines.append("")
    lines.append("## Original config (restored)")
    lines.append("")
    for key in OPTIMIZER_TUNING_KEYS:
        lines.append(f"- `{key}`: `{_fmt(snapshot.get(key))}`")
    lines.append(f"- `bom_build` (fixed for the run): `{_fmt(snapshot.get('bom_build') or RUN_BODY_DEFAULT)}`")
    lines.append("")
    lines.append("## Trials")
    lines.append("")
    lines.append(
        "| # | phase | change | decision | evidence | build | err_pct | Wh/m | score | dist_m |"
    )
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    narrative: list[str] = []
    for row in records:
        change = f"`{row.get('param')}` {_fmt(row.get('old'))} → {_fmt(row.get('new'))}"
        lines.append(
            "| {iteration} | {phase} | {change} | {decision} | {evidence} | {build} | {err} | {wpm} | {score} | {dist} |".format(
                iteration=row.get("iteration", ""),
                phase=row.get("phase", ""),
                change=change,
                decision=row.get("decision", ""),
                evidence=_fmt(row.get("evidence_status")),
                build=row.get("build", ""),
                err=_fmt(row.get("err_pct")),
                wpm=_fmt(row.get("wh_per_m")),
                score=_fmt(row.get("score")),
                dist=_fmt(row.get("dist_m")),
            )
        )
        narrative.extend(_trial_story(row))
    lines.append("")
    lines.extend(narrative)
    lines.append("## Where the energy went")
    lines.append("")
    if not diagnoses:
        lines.append("No finished mission was diagnosed.")
    for build, diag in diagnoses.items():
        summary = diag.summary
        lines.append(f"### {build}")
        lines.append("")
        if summary is None:
            lines.append("No summary.")
            continue
        shares = summary.shares()
        lines.append(
            f"err_pct {summary.err_pct}, shaft {summary.wh_measured_drive_shaft} Wh, "
            f"electrical drive {summary.wh_measured_drive_elec} Wh, "
            f"model drive {summary.wh_model_drive} Wh, distance {summary.dist_m} m."
        )
        if shares:
            lines.append(
                "Model drive shares: "
                + ", ".join(f"{name} {share:.0%}" for name, share in shares.items())
                + "."
            )
        for note in diag.notes:
            lines.append(f"- {note}")
        lines.append("")
    confirmed = [row for row in records if row.get("decision") == "confirm-pass"]
    lines.append("## Confirmed changes")
    lines.append("")
    if not confirmed:
        lines.append("None confirmed. Unconfirmed keeps were rolled back when the check failed,")
        lines.append("or the run stopped before the check.")
    else:
        lines.append("These passed the pre-keep vs kept confirm pair on the run body.")
        lines.append("They are not written back into config.")
        lines.append("")
        for row in confirmed:
            lines.append(
                f"- `{row.get('param')}` = `{_fmt(row.get('new'))}` "
                f"on {row.get('build')} ({row.get('reason')})"
            )
    lines.append("")
    lines.append("## Suggestions (not applied, not committed)")
    lines.append("")
    seen: set[str] = set()
    for row in records:
        raw = row.get("code_suggestions") or ""
        parts = raw if isinstance(raw, list) else [part.strip() for part in str(raw).split("|")]
        for part in parts:
            text = str(part).strip()
            if text and text not in seen:
                seen.add(text)
                lines.append(f"- {text}")
    if not seen:
        lines.append("None.")
    lines.append("")
    lines.append("## Held-out confirm")
    lines.append("")
    if held_out_note:
        lines.append(held_out_note)
    else:
        lines.append("(not recorded)")
    lines.append("")
    if cross_build_note:
        lines.append("Cross-build check:")
        lines.append("")
        lines.append(cross_build_note)
        lines.append("")
    lines.append("## Heavy vs light measured Wh/m")
    lines.append("")
    lines.append(
        "If a heavy-build lap shows much lower drive Wh/m than light with similar "
        "wh_measured_drive_shaft, treat it as a model/physics mismatch, not evidence "
        "that heavy is more efficient. Measured shaft still uses the 0.08 m contact "
        "cylinder; only the electrical model and body mass follow bom_build until "
        "catalog wheel radius is applied in Webots (see src/webots_body.py)."
    )
    lines.append("")
    lines.append("## Guardrails")
    lines.append("")
    lines.append(
        "Tuned keys: "
        + ", ".join(f"`{key}`" for key in OPTIMIZER_TUNING_KEYS)
        + f". `bom_build` stays `{RUN_BODY_DEFAULT}` for the whole run."
    )
    lines.append("Not edited: " + ", ".join(f"`{path}`" for path in PROTECTED_PATHS) + ".")
    lines.append("Credential-shaped text in the trial log is replaced with `[REDACTED]`.")
    lines.append("The loop does not commit.")
    lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def write_report(path: Path, **kwargs) -> Path:
    text = render_report(**kwargs)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path
