"""Per-lap artifact folder under ``logs/laps/<timestamp>_<label>/``."""

from __future__ import annotations

import json
import os
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path, PosixPath, WindowsPath

from src.mission_summary_file import _kv_tokens

_LAP_ROOT = "logs/laps"
_SUMMARY_NAME = "summary.json"
_CONSOLE_NAME = "console.log"
_REPORT_NAME = "report.md"

_PIVOT_START = re.compile(
    r"PIVOT start (?P<dir>left|right) yaw .*?"
    r"entry_v=(?P<entry>[-+0-9.]+) m/s pos=\((?P<x>[-+0-9.]+),(?P<y>[-+0-9.]+)\)"
    r"(?: coast_into_turn=(?P<coast>true|false))?",
    re.IGNORECASE,
)
_PIVOT_DONE = re.compile(
    r"PIVOT done yaw .*?heading_change (?P<hchange>[-+0-9.]+)deg, "
    r"entry_v=(?P<entry>[-+0-9.]+) m/s ct_max=(?P<ct>[-+0-9.]+) m",
    re.IGNORECASE,
)
_LIMITS = re.compile(
    r"CONTROLLER LIMITS cruise=(?P<cruise>[-+0-9.]+) m/s straight=(?P<straight>[-+0-9.]+) m/s "
    r"pivot_w_max=(?P<pivot>[-+0-9.]+) rad/s k_steer=(?P<ksteer>[-+0-9.]+)"
    r"(?: v_max=(?P<vmax>[-+0-9.]+) m/s effective_cruise=(?P<eff>[-+0-9.]+) m/s limit=(?P<tag>\w+))?",
    re.IGNORECASE,
)
_GYRO = re.compile(r"Gyro: device found and enabled", re.IGNORECASE)
_GYRO_MISSING = re.compile(r"Gyro: (?:not found|unavailable)", re.IGNORECASE)
_SPEED_TRACE = re.compile(r"SPEED_TRACE .* v_body=(?P<v>[-+0-9.]+)")
_TELEM_GAPS = re.compile(r"TELEMETRY_GAPS .* max_s=(?P<max>[-+0-9.]+)")
_COAST_LINE = re.compile(r"coast_enabled[=:]\s*(?P<on>true|false|1|0)", re.IGNORECASE)


def _host_path(text: str) -> Path:
    """Native path type for this OS (tests may patch ``os.name`` to ``nt``)."""
    if sys.platform == "win32":
        return WindowsPath(text)
    return PosixPath(text.replace("\\", "/"))


def path_abspath(path: os.PathLike[str] | str) -> str:
    """Absolute path for comparisons without ``Path.resolve()`` (safe when ``os.name`` is patched)."""
    return os.path.normcase(os.path.abspath(os.fspath(path)))


def laps_root(repo: Path) -> Path:
    return _host_path(os.path.abspath(os.fspath(repo))) / _LAP_ROOT


def sanitize_label(label: str) -> str:
    text = re.sub(r"[^\w.\-]+", "_", str(label or "lap").strip())
    return text[:80] or "lap"


def create_lap_folder(repo: Path, label: str, *, when: datetime | None = None) -> Path:
    ts = (when or datetime.now(timezone.utc)).strftime("%Y%m%d-%H%M%S")
    folder = laps_root(repo) / f"{ts}_{sanitize_label(label)}"
    folder.mkdir(parents=True, exist_ok=False)
    return folder


def lap_env(folder: Path) -> dict[str, str]:
    """Environment variables for one lap's controller + mission summary."""
    resolved = path_abspath(folder)
    return {
        "RBM_LAP_DIR": resolved,
        "RBM_MISSION_SUMMARY_DIR": resolved,
        "RBM_MISSION_SUMMARY_BASENAME": _SUMMARY_NAME,
        "RBM_CONTROLLER_LOG": str(folder / _CONSOLE_NAME),
        "RBM_LOG_DIR": resolved,
    }


def write_legacy_log_pointer(log_path: Path, lap_dir: Path) -> None:
    """Keep optimizer ``launch_*.log`` paths useful."""
    lap_dir = _host_path(path_abspath(lap_dir))
    text = (
        f"optimizer: lap artifacts -> {lap_dir}\n"
        f"optimizer: controller console -> {lap_dir / _CONSOLE_NAME}\n"
        f"optimizer: mission summary -> {lap_dir / _SUMMARY_NAME}\n"
        f"optimizer: report card -> {lap_dir / _REPORT_NAME}\n"
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists() and log_path.stat().st_size > 0:
        existing = log_path.read_text(encoding="utf-8", errors="replace")
        if str(lap_dir) in existing:
            return
        text = existing.rstrip() + "\n" + text
    log_path.write_text(text, encoding="utf-8")


def copy_legacy_controller_log(log_path: Path | None, lap_dir: Path) -> None:
    """If an older layout wrote ``*_controller.log`` beside the launcher log, mirror it."""
    if log_path is None:
        return
    legacy = log_path.with_name(f"{log_path.stem}_controller{log_path.suffix}")
    target = lap_dir / _CONSOLE_NAME
    if legacy.is_file() and not target.is_file():
        shutil.copyfile(legacy, target)


def load_summary_payload(lap_dir: Path) -> dict | None:
    path = lap_dir / _SUMMARY_NAME
    if not path.is_file():
        return None
    try:
        with path.open(encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def summary_lines(payload: dict | None) -> list[str]:
    if not payload:
        return []
    lines = payload.get("lines")
    if not isinstance(lines, list):
        return []
    return [str(line) for line in lines]


def mission_summary_fields(lines: list[str]) -> dict[str, str]:
    summary = next((line for line in lines if "MISSION_SUMMARY" in line), "")
    return _kv_tokens(summary) if summary else {}


def parse_pivot_turns(console_text: str) -> list[dict]:
    starts = list(_PIVOT_START.finditer(console_text))
    dones = list(_PIVOT_DONE.finditer(console_text))
    rows: list[dict] = []
    for idx, done in enumerate(dones):
        start = starts[idx] if idx < len(starts) else None
        row = {
            "turn": idx + 1,
            "heading_change_deg": float(done.group("hchange")),
            "entry_v_m_s": float(done.group("entry")),
            "max_lateral_m": float(done.group("ct")),
            "pivot_start": "—",
        }
        if start is not None:
            row["pivot_start"] = f"({start.group('x')},{start.group('y')})"
            if start.group("entry"):
                row["entry_v_m_s"] = float(start.group("entry"))
        rows.append(row)
    return rows


def parse_limits_line(console_text: str) -> dict:
    for line in console_text.splitlines():
        match = _LIMITS.search(line)
        if match:
            out = {
                "cruise_m_s": float(match.group("cruise")),
                "straight_m_s": float(match.group("straight")),
                "pivot_w_max_rad_s": float(match.group("pivot")),
                "k_steer": float(match.group("ksteer")),
            }
            if match.group("vmax"):
                out["v_max_m_s"] = float(match.group("vmax"))
                out["effective_cruise_m_s"] = float(match.group("eff"))
                out["limit_tag"] = match.group("tag")
            return out
    return {}


def parse_coast_enabled(console_text: str, fields: dict[str, str]) -> bool | None:
    for key in ("coast_enabled", "ctrl_coast_enabled"):
        if key in fields:
            return fields[key].lower() in {"1", "true", "yes"}
    for line in console_text.splitlines():
        match = _COAST_LINE.search(line)
        if match:
            return match.group("on").lower() in {"1", "true"}
    return None


def parse_top_speed_m_s(console_text: str) -> float | None:
    peak = None
    for match in _SPEED_TRACE.finditer(console_text):
        v = float(match.group("v"))
        peak = v if peak is None else max(peak, v)
    return peak


def wheel_speed_cap_m_s(wheel_radius_m: float = 0.08) -> float:
    from src.lane_keep import MAX_WHEEL_RAD_S

    return float(MAX_WHEEL_RAD_S) * float(wheel_radius_m)


def classify_speed_limiter(
    top_v: float | None,
    limits: dict,
    fields: dict[str, str],
) -> str:
    if top_v is None:
        return "unknown (no SPEED_TRACE in console)"
    eff = fields.get("ctrl_effective_cruise_m_s") or limits.get("effective_cruise_m_s")
    cruise = fields.get("ctrl_cruise_m_s") or limits.get("cruise_m_s")
    straight = fields.get("ctrl_straight_m_s") or limits.get("straight_m_s")
    v_max = fields.get("ctrl_v_max_m_s") or limits.get("v_max_m_s")
    tag = limits.get("limit_tag") or ""
    try:
        eff_f = float(eff) if eff is not None else float(cruise) if cruise else None
        straight_f = float(straight) if straight else None
        v_max_f = float(v_max) if v_max is not None else None
    except (TypeError, ValueError):
        eff_f = straight_f = v_max_f = None
    wcap = wheel_speed_cap_m_s()
    tol = 0.025
    if top_v >= wcap - tol:
        return f"wheel cap (~{wcap:.2f} m/s from MAX_WHEEL_RAD_S × contact radius)"
    if tag == "physics" and v_max_f is not None and top_v >= v_max_f - tol:
        return f"physics cap (v_max={v_max_f:.2f} m/s)"
    if eff_f is not None and top_v >= eff_f - tol:
        if tag == "physics":
            return f"physics-limited effective cruise ({eff_f:.2f} m/s)"
        return f"cruise / straight config (effective_cruise≈{eff_f:.2f} m/s)"
    if straight_f is not None and top_v >= straight_f - tol:
        return f"straight_m_s cap ({straight_f:.2f} m/s)"
    if eff_f is not None and top_v < eff_f - 0.05:
        return "lane-confidence / curvature (curve_speed below cruise)"
    return "mixed or approach braking"


def parse_telemetry_gap_s(lines: list[str], console_text: str) -> float | None:
    for line in lines:
        match = _TELEM_GAPS.search(line)
        if match:
            return float(match.group("max"))
    for line in console_text.splitlines():
        match = _TELEM_GAPS.search(line)
        if match:
            return float(match.group("max"))
    return None


def gyro_status(console_text: str, fields: dict[str, str], payload: dict | None) -> str:
    if payload and "gyro_found" in payload:
        return "found" if payload["gyro_found"] else "missing"
    if fields.get("gyro_found") in {"1", "true", "True"}:
        return "found"
    if fields.get("gyro_found") in {"0", "false", "False"}:
        return "missing"
    if _GYRO.search(console_text):
        return "found"
    if _GYRO_MISSING.search(console_text):
        return "missing"
    return "unknown"


def build_flags(
    *,
    fields: dict[str, str],
    turns: list[dict],
    telemetry_gap_s: float | None,
    gyro: str,
) -> list[str]:
    flags: list[str] = []
    err = fields.get("err_pct")
    try:
        if err is not None and abs(float(err)) > 3.0:
            flags.append(f"energy model error {float(err):+.1f}% (|err| > 3%)")
    except ValueError:
        pass
    for row in turns:
        h = abs(row.get("heading_change_deg", 0.0))
        if h < 80.0 or h > 100.0:
            flags.append(
                f"turn {row['turn']}: heading change {row['heading_change_deg']:+.1f}° outside 80–100°"
            )
    if telemetry_gap_s is not None and telemetry_gap_s > 1.0:
        flags.append(f"telemetry gap max {telemetry_gap_s:.2f} s (> 1 s)")
    if gyro == "missing":
        flags.append("gyro missing on mission summary")
    turns_n = fields.get("turns")
    if turns and turns_n is not None:
        try:
            if int(turns_n) != len(turns):
                flags.append(f"turn count mismatch: summary turns={turns_n}, pivots logged={len(turns)}")
        except ValueError:
            pass
    return flags


def build_report_md(
    *,
    lap_dir: Path,
    label: str,
    fields: dict[str, str],
    limits: dict,
    console_text: str,
    payload: dict | None,
    outcome_status: str = "",
    outcome_note: str = "",
) -> str:
    lines = summary_lines(payload)
    turns = parse_pivot_turns(console_text)
    coast = parse_coast_enabled(console_text, fields)
    top_v = parse_top_speed_m_s(console_text)
    limiter = classify_speed_limiter(top_v, limits, fields)
    gyro = gyro_status(console_text, fields, payload)
    gap = parse_telemetry_gap_s(lines, console_text)
    flags = build_flags(fields=fields, turns=turns, telemetry_gap_s=gap, gyro=gyro)

    def fnum(key: str, fmt: str = ".2f") -> str:
        raw = fields.get(key)
        if raw is None:
            return "—"
        try:
            return format(float(raw), fmt)
        except ValueError:
            return str(raw)

    duration = fnum("duration_s", ".1f")
    dist = fnum("dist_m", ".2f")
    err = fnum("err_pct", ".1f")
    wh_m = "—"
    try:
        elec = float(fields["wh_measured_drive_elec"])
        dist_m = float(fields["dist_m"])
        if dist_m > 0:
            wh_m = f"{elec / dist_m * 1000:.2f}"
    except (KeyError, ValueError, ZeroDivisionError):
        pass

    coast_margin = fields.get("coast_margin_m", limits.get("coast_margin_m", "—"))
    coast_txt = "—" if coast is None else ("on" if coast else "off")

    md = [
        f"# Lap report — `{lap_dir.name}`",
        "",
        f"- **Label:** {label}",
        f"- **Folder:** `{lap_dir}`",
    ]
    if outcome_status:
        md.append(f"- **Optimizer outcome:** {outcome_status}")
    if outcome_note:
        md.append(f"- **Note:** {outcome_note.strip()}")
    md.extend(
        [
            "",
            "## Effective settings",
            "",
            "| Setting | Value |",
            "| --- | --- |",
            f"| cruise_m_s | {fnum('ctrl_cruise_m_s') or limits.get('cruise_m_s', '—')} |",
            f"| straight_m_s | {fnum('ctrl_straight_m_s') or limits.get('straight_m_s', '—')} |",
            f"| pivot_w_max_rad_s | {fnum('ctrl_pivot_w_max_rad_s') or limits.get('pivot_w_max_rad_s', '—')} |",
            f"| k_steer | {fnum('ctrl_k_steer', '.2f') or limits.get('k_steer', '—')} |",
            f"| coast | {coast_txt} (margin_m={coast_margin}) |",
            f"| effective_cruise_m_s | {fnum('ctrl_effective_cruise_m_s') or limits.get('effective_cruise_m_s', '—')} |",
            f"| v_max_m_s | {fnum('ctrl_v_max_m_s') or limits.get('v_max_m_s', '—')} |",
            "",
            "## Gyro",
            "",
            f"Status: **{gyro}**",
            "",
            "## Top speed",
            "",
            f"- **Peak v_body (SPEED_TRACE):** {top_v if top_v is not None else '—'} m/s",
            f"- **Limiter (heuristic):** {limiter}",
            "",
            "## Per-turn pivots",
            "",
            "| Turn | Heading Δ (deg) | Entry v (m/s) | Pivot start | Max lateral (m) |",
            "| --- | --- | --- | --- | --- |",
        ]
    )
    if turns:
        for row in turns:
            md.append(
                f"| {row['turn']} | {row['heading_change_deg']:+.1f} | "
                f"{row['entry_v_m_s']:.3f} | {row['pivot_start']} | {row['max_lateral_m']:.3f} |"
            )
    else:
        md.append("| — | (no PIVOT lines in console.log) | — | — | — |")
    md.extend(
        [
            "",
            "## Energy & lap",
            "",
            f"| Metric | Value |",
            f"| --- | --- |",
            f"| err_pct | {err}% |",
            f"| Wh/m (measured drive elec) | {wh_m} |",
            f"| Lap time | {duration} s |",
            f"| Distance | {dist} m |",
            f"| Turns (summary) | {fields.get('turns', '—')} |",
            "",
            "## FLAGS",
            "",
        ]
    )
    if flags:
        md.extend(f"- {flag}" for flag in flags)
    else:
        md.append("- (none)")
    md.append("")
    return "\n".join(md)


def finalize_lap_folder(
    lap_dir: Path,
    *,
    label: str,
    log_path: Path | None = None,
    outcome_status: str = "",
    outcome_note: str = "",
    extra_console: str = "",
) -> Path:
    """Write ``report.md`` and refresh legacy pointers."""
    copy_legacy_controller_log(log_path, lap_dir)
    console_path = lap_dir / _CONSOLE_NAME
    console_text = ""
    if console_path.is_file():
        console_text = console_path.read_text(encoding="utf-8", errors="replace")
    if extra_console:
        console_text = console_text + ("\n" if console_text else "") + extra_console
    payload = load_summary_payload(lap_dir)
    lines = summary_lines(payload)
    fields = mission_summary_fields(lines)
    if not fields and outcome_note:
        for line in (extra_console or "").splitlines():
            if "MISSION_SUMMARY" in line:
                fields = _kv_tokens(line)
    limits = parse_limits_line(console_text)
    report = build_report_md(
        lap_dir=lap_dir,
        label=label,
        fields=fields,
        limits=limits,
        console_text=console_text,
        payload=payload,
        outcome_status=outcome_status,
        outcome_note=outcome_note,
    )
    (lap_dir / _REPORT_NAME).write_text(report, encoding="utf-8")
    turns = parse_pivot_turns(console_text)
    top_v = parse_top_speed_m_s(console_text)
    limiter = classify_speed_limiter(top_v, limits, fields)
    gyro = gyro_status(console_text, fields, payload)
    gap = parse_telemetry_gap_s(lines, console_text)
    flags = build_flags(fields=fields, turns=turns, telemetry_gap_s=gap, gyro=gyro)
    try:
        from optimizer.lap_agent import persist_lap_analysis_meta

        persist_lap_analysis_meta(
            lap_dir,
            {
                "flags": flags,
                "top_speed_m_s": top_v,
                "speed_limiter": limiter,
                "gyro_status": gyro,
            },
        )
    except Exception:
        pass
    if log_path is not None:
        write_legacy_log_pointer(log_path, lap_dir)
    return lap_dir / _REPORT_NAME


def write_summary_from_dashboard_text(
    lap_dir: Path,
    text: str,
    *,
    launch: object | None = None,
) -> Path | None:
    """Persist ``summary.json`` when the dashboard log carried the mission lines."""
    target = lap_dir / _SUMMARY_NAME
    if target.is_file():
        return target
    kept: list[str] = []
    for raw in (text or "").splitlines():
        line = raw.strip()
        if not line:
            continue
        if "|" in line and "MISSION_SUMMARY" in line:
            parts = line.split("|")
            line = parts[-1].strip()
        if any(
            token in line
            for token in (
                "MISSION_SUMMARY",
                "TELEMETRY_GAPS",
                "MISSION_ENERGY",
                "STEER_SOURCE",
                "STEER_SPLIT",
            )
        ):
            kept.append(line)
    if not any("MISSION_SUMMARY" in line for line in kept):
        return None
    payload: dict = {
        "source": "optimizer",
        "path": str(target),
        "lines": kept,
    }
    run_id = getattr(launch, "run_id", None) if launch is not None else None
    trial_id = getattr(launch, "trial_id", None) if launch is not None else None
    if run_id:
        payload["opt_run_id"] = run_id
    if trial_id:
        payload["opt_trial_id"] = trial_id
    fields = mission_summary_fields(kept)
    if fields.get("gyro_found") in {"1", "true", "True"}:
        payload["gyro_found"] = True
    elif fields.get("gyro_found") in {"0", "false", "False"}:
        payload["gyro_found"] = False
    with target.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)
        fh.write("\n")
    return target


def maybe_copy_summary_to_legacy_logs(repo: Path, lap_dir: Path) -> None:
    """Optional mirror so ``logs/mission_summary_*.json`` pickers still see the lap."""
    payload = load_summary_payload(lap_dir)
    if payload is None:
        return
    from src.mission_summary_file import summary_directory

    legacy = _host_path(path_abspath(summary_directory()))
    if path_abspath(legacy) == path_abspath(lap_dir):
        return
    legacy.mkdir(parents=True, exist_ok=True)
    stamp = lap_dir.name.split("_", 1)[0]
    dest = legacy / f"mission_summary_{stamp}.json"
    try:
        shutil.copyfile(lap_dir / _SUMMARY_NAME, dest)
    except OSError:
        pass
