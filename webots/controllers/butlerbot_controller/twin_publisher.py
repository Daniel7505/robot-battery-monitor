"""
POST Webots telemetry to the Robot Battery Monitor DigitalTwinBridge.

Uses only stdlib (urllib) so it runs inside Webots without extra packages.
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import urllib.error
import urllib.request

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def _load_build_webots_telemetry():
    """Load webots_power directly — avoids src.twin __init__ pulling in config."""
    module_path = os.path.join(_PROJECT_ROOT, "src", "twin", "webots_power.py")
    if not os.path.isfile(module_path):
        return None
    spec = importlib.util.spec_from_file_location("webots_power", module_path)
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, "build_webots_telemetry", None)


build_webots_telemetry = _load_build_webots_telemetry()


DEFAULT_DASHBOARD_URL = "http://127.0.0.1:5000"


def dashboard_url() -> str:
    return (
        os.environ.get("TWIN_DASHBOARD_URL")
        or os.environ.get("DASHBOARD_URL")
        or DEFAULT_DASHBOARD_URL
    ).rstrip("/")


def parse_controller_args() -> dict:
    """Read --dashboard-url= and --telemetry-interval= from controllerArgs."""
    opts = {"dashboard_url": dashboard_url(), "interval_s": 0.5}
    for arg in sys.argv[1:]:
        if arg.startswith("--dashboard-url="):
            opts["dashboard_url"] = arg.split("=", 1)[1].rstrip("/")
        elif arg.startswith("--telemetry-interval="):
            try:
                opts["interval_s"] = float(arg.split("=", 1)[1])
            except ValueError:
                pass
    return opts


def publish_telemetry(payload: dict, base_url: str | None = None) -> dict:
    """POST telemetry JSON to /api/twin/telemetry?adapter=webots."""
    base = (base_url or dashboard_url()).rstrip("/")
    url = f"{base}/api/twin/telemetry?adapter=webots"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.URLError as exc:
        return {"ok": False, "error": str(exc)}


def build_payload(
    joints: list[dict],
    *,
    gait: str,
    phase: str,
    speed_m_s: float,
    battery_pct: float,
    pose: dict | None = None,
    sensors: dict | None = None,
) -> dict:
    if build_webots_telemetry is not None:
        return build_webots_telemetry(
            joints=joints,
            gait=gait,
            phase=phase,
            speed_m_s=speed_m_s,
            battery_pct=battery_pct,
            pose=pose,
            sensors=sensors,
        )
    # Minimal fallback if project src not on path
    return {
        "source": "webots",
        "adapter": "webots",
        "joints": joints,
        "locomotion": {"gait": gait, "speed_m_s": speed_m_s, "phase": phase},
        "robot": {"name": "ButlerBot", "main_battery_pct": battery_pct},
        "motor_power_w": {},
        "channel_draws": {"Legs": 5, "Arms": 5, "Torso": 4, "Compute": 7.5},
    }