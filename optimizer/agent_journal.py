"""Plain-language run notes for humans (separate from KNOWLEDGE.md)."""

from __future__ import annotations

import re
from datetime import datetime, timezone
from pathlib import Path

_SECRET = re.compile(r"(?i)(token|password|api[_-]?key|secret)\s*=")


def _redact(text: str) -> str:
    if not text:
        return ""
    if _SECRET.search(text):
        return "[redacted detail]"
    return text.strip()


def _confirmed(records: list[dict]) -> list[str]:
    lines = []
    for row in records:
        if row.get("decision") != "confirm-pass":
            continue
        param = row.get("param") or "knob"
        lines.append(f"Confirmed {param} at {row.get('new')} (was {row.get('old')}).")
    return lines


def render_journal_entry(
    *,
    run_id: str,
    started: datetime,
    finished: datetime,
    records: list[dict],
    stop_reason: str,
) -> str:
    """~5–10 lines: what was hard, what worked, mistakes, next time."""
    day = started.astimezone(timezone.utc).strftime("%Y-%m-%d")
    stamp = started.astimezone(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lines = [f"## {day} run_id={run_id} ({stamp})", ""]

    invalid = [r for r in records if r.get("decision") == "invalid"]
    misses = [
        r
        for r in records
        if (r.get("postmortem") or {}).get("prediction_miss")
    ]
    confirms = _confirmed(records)

    hard_parts: list[str] = []
    if invalid:
        hard_parts.append(
            f"{len(invalid)} invalid lap(s) ({_redact(str(invalid[-1].get('reason') or ''))})"
        )
    if any(r.get("decision") == "confirm-fail" for r in records):
        hard_parts.append("at least one confirm pair failed the evidence bar")
    if not hard_parts:
        hard_parts.append("launches stayed clean; stamps and summaries matched each trial")

    lines.append(
        "What was hard: " + "; ".join(hard_parts) + "."
    )

    figured: list[str] = []
    for row in records:
        if row.get("decision") not in {"keep", "confirm-pass"}:
            continue
        param = row.get("param") or "setting"
        err = row.get("err_gain_pp")
        wpm = row.get("wpm_gain_rel")
        turn = row.get("turn_delta_pp")
        bits = []
        if err is not None:
            bits.append(f"|err| {err:+.2f} pp vs baseline")
        if wpm is not None:
            bits.append(f"Wh/m {wpm:+.1%}")
        if turn is not None:
            bits.append(f"turn |err| {turn:+.2f} pp vs baseline")
        if bits:
            figured.append(f"{param} → {row.get('new')}: {', '.join(bits)}")
    if confirms:
        figured.extend(confirms)
    if not figured:
        figured.append("No new keeps; the run mostly held starting settings.")
    lines.append("What we learned: " + "; ".join(figured[:3]) + ".")

    wrong: list[str] = []
    for row in misses:
        miss = (row.get("postmortem") or {}).get("prediction_miss") or ""
        if miss:
            wrong.append(_redact(miss))
    for row in invalid:
        reason = _redact(str(row.get("reason") or ""))
        if reason and reason not in wrong:
            wrong.append(f"Invalid: {reason}")
    if wrong:
        lines.append("Where we were wrong: " + " ".join(wrong[:2]))
    else:
        lines.append("Where we were wrong: nothing major flagged in prediction_miss or invalid rows.")

    next_steps: list[str] = []
    last = records[-1] if records else None
    if last and last.get("next_hypothesis"):
        next_steps.append(_redact(str(last["next_hypothesis"])))
    if stop_reason and "iteration limit" not in stop_reason.lower():
        next_steps.append(_redact(stop_reason))
    if not next_steps:
        next_steps.append("Continue from the last kept baseline on the next run.")
    lines.append("Next time: " + next_steps[0][:280] + ".")

    lines.append("")
    return "\n".join(lines)


def append_agent_journal(
    path: Path,
    *,
    run_id: str,
    started: datetime,
    finished: datetime,
    records: list[dict],
    stop_reason: str,
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    entry = render_journal_entry(
        run_id=run_id,
        started=started,
        finished=finished,
        records=records,
        stop_reason=stop_reason,
    )
    existing = path.read_text(encoding="utf-8") if path.is_file() else ""
    if existing and not existing.endswith("\n"):
        existing += "\n"
    path.write_text(existing + entry, encoding="utf-8")
