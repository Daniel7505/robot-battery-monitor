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
        "build": loc.get("bom_build") or hw.get("bom_build"),
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
