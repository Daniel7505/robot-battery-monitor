"""Morning markdown, written from this session's trials."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from src.controller_limits import ALLOWLIST, PROTECTED_PATHS


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
    for key in ALLOWLIST:
        lines.append(f"- `{key}`: `{_fmt(snapshot.get(key))}`")
    lines.append("")
    lines.append("## Trials")
    lines.append("")
    lines.append("| # | phase | change | decision | build | err_pct | Wh/m | dist_m |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    narrative: list[str] = []
    for row in records:
        change = f"`{row.get('param')}` {_fmt(row.get('old'))} → {_fmt(row.get('new'))}"
        lines.append(
            "| {iteration} | {phase} | {change} | {decision} | {build} | {err} | {wpm} | {dist} |".format(
                iteration=row.get("iteration", ""),
                phase=row.get("phase", ""),
                change=change,
                decision=row.get("decision", ""),
                build=row.get("build", ""),
                err=_fmt(row.get("err_pct")),
                wpm=_fmt(row.get("wh_per_m")),
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
        lines.append("These passed a held-out lap (another BoM build, or a repeat lap when the")
        lines.append("change itself was the build). They are not written back into config.")
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
    lines.append("## Guardrails")
    lines.append("")
    lines.append("Tuned keys: " + ", ".join(f"`{key}`" for key in ALLOWLIST) + ".")
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
