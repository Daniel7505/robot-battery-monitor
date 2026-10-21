"""Write mission lines to a file the Webots console does not own.

Webots on Windows draws controller ``print`` into the webots-bin console.
Redirecting that process leaves a 0-byte file (``--stdout`` with
Start-Process). The dashboard logger does write
``logs/robot_monitor_<date>.log`` when telemetry arrives, but that file
is the Docker process. A blank ``bom_build`` used to skip ``BOM BODY``
there entirely.

This module appends the same lines to ``logs/mission_summary_<ts>.json``
under the repo (or ``RBM_LOG_DIR``). The controller calls it on the host,
so the lap summary is on disk even when the console cannot be captured.
"""

from __future__ import annotations

import glob
import json
import os
from datetime import datetime, timezone

_STATE: dict[str, dict] = {}


def _enabled(directory: str | None) -> bool:
    if directory is not None or os.environ.get("RBM_MISSION_SUMMARY_DIR"):
        return True
    # The suite ingests telemetry. Do not drop a file into the repo logs.
    return not os.environ.get("PYTEST_CURRENT_TEST")


def summary_directory(directory: str | None = None) -> str:
    raw = directory if directory is not None else os.environ.get("RBM_MISSION_SUMMARY_DIR")
    if raw:
        return os.path.expanduser(os.path.expandvars(raw))
    from src.sim_trace import default_speed_trace_dir

    return default_speed_trace_dir()


def reset_mission_summary_file() -> None:
    """Drop the open file handle. Tests use this between cases."""
    _STATE.clear()


def append_mission_lines(
    lines: list[str],
    *,
    source: str = "dashboard",
    directory: str | None = None,
    mirror_dashboard_log: bool = False,
) -> str | None:
    """Append lines to this process's ``mission_summary_<ts>.json``.

    Returns the file path, or None when there is nothing to write.
    ``mirror_dashboard_log`` also appends the lines to
    ``robot_monitor_<date>.log`` in that same directory. The controller
    uses that because its own logger follows the Webots working directory,
    not the repo ``logs/`` the dashboard volume mounts.
    """
    kept = [str(line) for line in lines if line]
    if not kept or not _enabled(directory):
        return None
    folder = summary_directory(directory)
    key = os.path.abspath(folder) + "|" + source
    state = _STATE.get(key)
    if state is None:
        os.makedirs(folder, exist_ok=True)
        basename = (os.environ.get("RBM_MISSION_SUMMARY_BASENAME") or "").strip()
        if basename:
            path = os.path.join(folder, basename)
        else:
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            path = os.path.join(folder, f"mission_summary_{stamp}.json")
            if os.path.exists(path):
                path = os.path.join(folder, f"mission_summary_{stamp}_{os.getpid()}.json")
        state = {"path": path, "lines": [], "source": source}
        _STATE[key] = state
    state["lines"].extend(kept)
    payload = {
        "source": state["source"],
        "path": state["path"],
        "lines": list(state["lines"]),
    }
    run_id = os.environ.get("RBM_OPT_RUN_ID")
    trial_id = os.environ.get("RBM_OPT_TRIAL_ID")
    if run_id:
        payload["opt_run_id"] = run_id
    if trial_id:
        payload["opt_trial_id"] = trial_id
    summary_line = next((line for line in state["lines"] if "MISSION_SUMMARY" in line), None)
    if summary_line:
        fields = _kv_tokens(summary_line)
        if "gyro_found" in fields:
            payload["gyro_found"] = fields["gyro_found"] in {"1", "true", "True"}
        if "coast_wh_saved" in fields:
            payload["coast_wh_saved"] = _proof_number(fields.get("coast_wh_saved"))
    with open(state["path"], "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)
        fh.write("\n")
    if mirror_dashboard_log:
        _mirror_robot_monitor(kept, folder)
    return state["path"]


def _mirror_robot_monitor(lines: list[str], folder: str) -> None:
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"robot_monitor_{datetime.now().strftime('%Y-%m-%d')}.log")
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S,%f")[:-3]
    with open(path, "a", encoding="utf-8") as fh:
        for line in lines:
            fh.write(f"{stamp} | INFO     | robot_monitor | {line}\n")


_PROOF_NUMBER_KEYS = (
    "err_pct",
    "wh_measured_drive_elec",
    "wh_model_drive",
    "steer_frac_camera",
    "steer_frac_held",
    "steer_frac_track",
    "ct_max_m",
    "width_err_rms_m",
    "width_err_max_m",
    "center_err_rms_m",
    "center_err_max_m",
)


def _kv_tokens(line: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for token in str(line).split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        out[key] = value
    return out


def _proof_number(raw: str | None) -> float | None:
    if raw is None:
        return None
    text = str(raw).strip().lower()
    if text in {"", "na", "none", "null"}:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def proof_from_summary_lines(lines: list[str]) -> dict | None:
    """Pull the lap-proof fields off a finished ``MISSION_SUMMARY`` line."""
    summary = next((line for line in lines if "MISSION_SUMMARY" in line), None)
    if not summary:
        return None
    fields = _kv_tokens(summary)
    proof = {key: _proof_number(fields.get(key)) for key in _PROOF_NUMBER_KEYS}
    if not any(value is not None for value in proof.values()):
        return None
    proof["ok"] = True
    proof["cross_track_m"] = proof.get("ct_max_m")
    proof["width_err_n"] = None
    proof["center_err_n"] = None
    return proof


def latest_proof_from_disk(directory: str | None = None) -> dict | None:
    """Newest ``logs/mission_summary_*.json``, or None when the folder is empty.

    Used when the dashboard process was not the one that drove the lap, so
    the in-memory energy log has nothing to show.
    """
    folder = summary_directory(directory)
    if not os.path.isdir(folder):
        return None
    paths = glob.glob(os.path.join(folder, "mission_summary_*.json"))
    if not paths:
        return None
    path = max(paths, key=os.path.getmtime)
    try:
        with open(path, encoding="utf-8") as fh:
            payload = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    lines = payload.get("lines") if isinstance(payload, dict) else None
    if not isinstance(lines, list):
        return None
    proof = proof_from_summary_lines([str(line) for line in lines])
    if proof is None:
        return None
    proof["path"] = path
    return proof


def _timestamp(value):
    if isinstance(value, datetime):
        ts = value
    elif isinstance(value, str) and value.strip():
        try:
            ts = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            return None
    else:
        return None
    if ts.tzinfo is None:
        ts = ts.replace(tzinfo=timezone.utc)
    return ts


def sample_from_controller_payload(payload: dict) -> dict:
    """The energy-log sample for one controller POST, before the dashboard sees it."""
    from src.twin_energy import LOCO_PASSTHROUGH_KEYS

    loc = payload.get("locomotion") if isinstance(payload.get("locomotion"), dict) else {}
    sensors = payload.get("sensors") if isinstance(payload.get("sensors"), dict) else {}
    robot = payload.get("robot") if isinstance(payload.get("robot"), dict) else {}
    hw = payload.get("hardware") if isinstance(payload.get("hardware"), dict) else {}
    draws = payload.get("channel_draws")
    if not isinstance(draws, dict):
        power = payload.get("power") if isinstance(payload.get("power"), dict) else {}
        draws = power.get("channel_draws") if isinstance(power.get("channel_draws"), dict) else {}
    sample = {
        "timestamp": _timestamp(payload.get("timestamp")),
        "pose": payload.get("pose") if isinstance(payload.get("pose"), dict) else {},
        "channel_draws": draws,
        "subsystem_draws": payload.get("subsystem_draws") or {},
        "battery_pct": robot.get("main_battery_pct", robot.get("battery_pct")),
        "capacity_wh": robot.get("battery_capacity_wh"),
        "payload_kg": loc.get("payload_kg"),
        "build": loc.get("bom_build") or loc.get("ctrl_bom_build") or hw.get("bom_build"),
        "speed_m_s": loc.get("speed_m_s", payload.get("speed_m_s")),
        "mission_event": sensors.get("mission_event"),
    }
    if sample["timestamp"] is None:
        sample.pop("timestamp")
    for key in ("measured_drive_w", "measured_drive_elec_w"):
        if key in loc:
            sample[key] = loc.get(key)
        elif key in payload:
            sample[key] = payload.get(key)
    run_id = loc.get("controller_run_id") or sensors.get("controller_run_id")
    if run_id:
        sample["controller_run_id"] = run_id
    for key in LOCO_PASSTHROUGH_KEYS:
        if key in loc:
            sample[key] = loc.get(key)
        elif key in sensors and key not in sample:
            sample[key] = sensors.get(key)
    return sample


def observe_controller_payload(log, payload: dict, *, directory: str | None = None) -> tuple[list[str], str | None]:
    """Observe one controller payload and append any new lines to the summary file."""
    lines = log.observe(sample_from_controller_payload(payload))
    path = None
    if lines:
        path = append_mission_lines(
            lines,
            source="controller",
            directory=directory,
            mirror_dashboard_log=True,
        )
    return lines, path
