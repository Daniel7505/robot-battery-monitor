"""Read per-lap report cards and fold FLAGS into optimizer verdicts."""

from __future__ import annotations

import json
import re
from pathlib import Path

from optimizer.lap_folder import (
    _REPORT_NAME,
    _SUMMARY_NAME,
    build_flags,
    classify_speed_limiter,
    gyro_status,
    load_summary_payload,
    mission_summary_fields,
    parse_limits_line,
    parse_pivot_turns,
    parse_telemetry_gap_s,
    parse_top_speed_m_s,
    summary_lines,
)

_FLAGS_SECTION = re.compile(
    r"^## FLAGS\s*$([\s\S]*?)(?=^## |\Z)",
    re.MULTILINE,
)
_FLAG_BULLET = re.compile(r"^\s*-\s+(?!\(none\))(.*)$", re.MULTILINE)


def parse_flags_from_report(report_md: str) -> list[str]:
    match = _FLAGS_SECTION.search(report_md or "")
    if not match:
        return []
    block = match.group(1)
    return [line.strip() for line in _FLAG_BULLET.findall(block) if line.strip()]


def _analysis_from_lap_dir(lap_dir: Path) -> dict:
    payload = load_summary_payload(lap_dir)
    lines = summary_lines(payload)
    fields = mission_summary_fields(lines)
    console_path = lap_dir / "console.log"
    console_text = (
        console_path.read_text(encoding="utf-8", errors="replace") if console_path.is_file() else ""
    )
    limits = parse_limits_line(console_text)
    turns = parse_pivot_turns(console_text)
    top_v = parse_top_speed_m_s(console_text)
    limiter = classify_speed_limiter(top_v, limits, fields)
    gyro = gyro_status(console_text, fields, payload)
    gap = parse_telemetry_gap_s(lines, console_text)
    flags = build_flags(fields=fields, turns=turns, telemetry_gap_s=gap, gyro=gyro)
    return {
        "flags": flags,
        "top_speed_m_s": top_v,
        "speed_limiter": limiter,
        "gyro_status": gyro,
        "fields": fields,
        "limits": limits,
    }


def read_lap_artifacts(lap_dir: Path | str | None) -> dict:
    """Load ``report.md``, ``summary.json`` FLAGS, and derived lap metrics."""
    if not lap_dir:
        return {}
    folder = Path(lap_dir)
    if not folder.is_dir():
        return {}
    report_path = folder / _REPORT_NAME
    report_md = report_path.read_text(encoding="utf-8", errors="replace") if report_path.is_file() else ""
    payload = load_summary_payload(folder) or {}
    flags = payload.get("flags")
    if isinstance(flags, list) and flags:
        flag_list = [str(item) for item in flags if str(item).strip()]
    else:
        flag_list = parse_flags_from_report(report_md)
    analysis = _analysis_from_lap_dir(folder)
    if not flag_list:
        flag_list = analysis.get("flags") or []
    top_v = payload.get("top_speed_m_s", analysis.get("top_speed_m_s"))
    limiter = payload.get("speed_limiter") or analysis.get("speed_limiter") or ""
    return {
        "lap_dir": str(folder.resolve()),
        "report_path": str(report_path) if report_path.is_file() else "",
        "summary_path": str(folder / _SUMMARY_NAME),
        "flags": flag_list,
        "top_speed_m_s": top_v,
        "speed_limiter": str(limiter or ""),
        "gyro_status": payload.get("gyro_status") or analysis.get("gyro_status"),
        "report_excerpt": _report_excerpt(report_md),
    }


def _report_excerpt(report_md: str, max_lines: int = 12) -> str:
    lines = [line for line in (report_md or "").splitlines() if line.strip()]
    if not lines:
        return ""
    return "\n".join(lines[:max_lines])


def flag_citation(artifacts: dict | None) -> str:
    if not artifacts:
        return ""
    flags = artifacts.get("flags") or []
    if not flags:
        return ""
    report = artifacts.get("report_path") or "report.md"
    bits = [f"Lap FLAGS ({report}): " + "; ".join(flags)]
    limiter = artifacts.get("speed_limiter")
    top = artifacts.get("top_speed_m_s")
    if top is not None and limiter:
        bits.append(f"Top speed {float(top):.3f} m/s — {limiter}.")
    return " ".join(bits)


def compose_keep_flag_explanation(
    artifacts: dict | None,
    *,
    score_reason: str,
    param: str | None = None,
    new=None,
    effects: dict | None = None,
) -> str:
    """Human-readable keep rationale that cites every lap flag."""
    if not artifacts:
        return ""
    flags = artifacts.get("flags") or []
    if not flags:
        return ""
    parts = [flag_citation(artifacts)]
    if param and new is not None:
        parts.append(f"Provisional keep for `{param}` → {new}.")
    parts.append(f"Score: {score_reason.strip()}")
    if effects:
        err = effects.get("err_gain_pp")
        wpm = effects.get("wpm_gain_rel")
        turn = effects.get("turn_delta_pp")
        if err is not None:
            parts.append(f"|err| Δ {float(err):+.2f} pp vs baseline.")
        if wpm is not None:
            parts.append(f"Wh/m Δ {float(wpm):+.1%} vs baseline.")
        if turn is not None:
            parts.append(f"Turn |err| Δ {float(turn):+.2f} pp vs baseline.")
    parts.append(
        "FLAGS noted above; keep stands only if the confirm lap repeats without worsening those issues."
    )
    return " ".join(part for part in parts if part)


def keep_blocked_without_flag_explanation(
    artifacts: dict | None,
    explanation: str,
) -> str | None:
    """Return a rollback reason when a flagged lap would be kept without explanation."""
    flags = (artifacts or {}).get("flags") or []
    if not flags:
        return None
    text = (explanation or "").strip()
    if len(text) < 40:
        return (
            "lap FLAGS require an explicit keep explanation before marking keep: "
            + "; ".join(flags)
        )
    lower = text.lower()
    if "lap flags" not in lower and "flags (" not in lower:
        return (
            "keep explanation must cite lap FLAGS (see report.md): " + "; ".join(flags)
        )
    return None


def merge_lap_flags_into_postmortem(
    post: dict,
    artifacts: dict | None,
    *,
    action: str,
    keep_explanation: str = "",
) -> None:
    cite = flag_citation(artifacts)
    if not cite:
        return
    post["lap_flags"] = list((artifacts or {}).get("flags") or [])
    post["lap_report"] = (artifacts or {}).get("report_path") or ""
    if keep_explanation:
        post["flag_keep_explanation"] = keep_explanation
    mechanism = str(post.get("mechanism") or "").strip()
    lesson = str(post.get("lesson") or "").strip()
    post["mechanism"] = f"{mechanism} {cite}".strip() if mechanism else cite
    flag_lesson = f"Read {post.get('lap_report') or 'report.md'}: {cite}"
    post["lesson"] = f"{lesson} {flag_lesson}".strip() if lesson else flag_lesson
    if action == "keep" and keep_explanation:
        post["lesson"] = f"{post['lesson']} Keep rationale: {keep_explanation}".strip()


def persist_lap_analysis_meta(lap_dir: Path, meta: dict) -> None:
    """Write FLAGS and speed metadata into ``summary.json``."""
    path = lap_dir / _SUMMARY_NAME
    payload: dict
    if path.is_file():
        try:
            with path.open(encoding="utf-8") as fh:
                payload = json.load(fh)
        except (OSError, json.JSONDecodeError):
            payload = {}
    else:
        payload = {}
    if not isinstance(payload, dict):
        payload = {}
    for key in ("flags", "top_speed_m_s", "speed_limiter", "gyro_status"):
        if key in meta and meta[key] is not None:
            payload[key] = meta[key]
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)
        fh.write("\n")
