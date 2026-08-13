"""
HTTP client from the Webots controller to the PMS DigitalTwinBridge.

Uses only the Python standard library (``urllib``) so it runs inside Webots
without pip-installing Flask or project extras into the controller env.

Direction of traffic
--------------------
* **Outbound** ``publish_telemetry`` → ``POST /api/twin/telemetry?adapter=webots``
  Sends joint states, locomotion, channel_draws, and battery %.
* **Inbound** ``fetch_twin_state`` → ``GET /api/twin/state``
  Pulls agent/safety throttle, dashboard teleop wheel cmds, ``stop_epoch``,
  and one-shot battery replenish for the controller loop.

Payload shape (see ``build_payload`` / ``src.twin.webots_power``)::

    {
      "source": "webots", "adapter": "webots",
      "joints": [{name, velocity, torque, power_w}, ...],
      "locomotion": {gait, speed_m_s, phase, mode},
      "robot": {name, main_battery_pct},
      "channel_draws": {Legs, Arms, Torso, Compute, ...},
      "mission": {task, phase}, ...
    }

Throttle pull-back
------------------
``remote_throttle_factor`` only returns a factor when the dashboard agent or
safety layer is *intervening* (``throttle_required`` / ``intervening``).
Allocation "warning" status alone must not cap teleop (that previously forced
a ~70% drive limit while still "healthy").
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import urllib.error
import urllib.request

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_LOAD_ERROR: str | None = None


def _load_build_webots_telemetry():
    """Load webots_power directly — avoids src.twin __init__ pulling in config.

    Webots controllers do not put the repo root on sys.path, so we inject it
    before exec (otherwise import of src.hardware_profile fails and we fall
    back to static Legs=5W forever).
    """
    global _LOAD_ERROR
    module_path = os.path.join(_PROJECT_ROOT, "src", "twin", "webots_power.py")
    if not os.path.isfile(module_path):
        _LOAD_ERROR = f"missing {module_path}"
        return None
    if _PROJECT_ROOT not in sys.path:
        sys.path.insert(0, _PROJECT_ROOT)
    spec = importlib.util.spec_from_file_location("webots_power", module_path)
    if spec is None or spec.loader is None:
        _LOAD_ERROR = "spec_from_file_location failed"
        return None
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        _LOAD_ERROR = f"{type(exc).__name__}: {exc}"
        return None
    _LOAD_ERROR = None
    return module


_webots_power_module = _load_build_webots_telemetry()
build_webots_telemetry = (
    getattr(_webots_power_module, "build_webots_telemetry", None)
    if _webots_power_module is not None
    else None
)
estimate_motor_power_w = (
    getattr(_webots_power_module, "estimate_motor_power_w", None)
    if _webots_power_module is not None
    else None
)
if build_webots_telemetry is None:
    print(
        f"twin_publisher: webots_power unavailable ({_LOAD_ERROR}); "
        "using joint/speed power fallback"
    )
else:
    print("twin_publisher: webots_power loaded — motion-aware channel draws enabled")


DEFAULT_DASHBOARD_URL = "http://127.0.0.1:5000"


def dashboard_url() -> str:
    """Resolve dashboard base URL: env TWIN_DASHBOARD_URL / DASHBOARD_URL, else localhost."""
    return (
        os.environ.get("TWIN_DASHBOARD_URL")
        or os.environ.get("DASHBOARD_URL")
        or DEFAULT_DASHBOARD_URL
    ).rstrip("/")


def parse_controller_args() -> dict:
    """Read ``--dashboard-url=`` and ``--telemetry-interval=`` from Webots controllerArgs."""
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


def fetch_twin_state(base_url: str | None = None) -> dict:
    """GET PMS/agent state — teleop cmds, stop_epoch, remote throttle, battery.

    Returns ``{}`` on network failure so the controller can keep running offline.
    """
    base = (base_url or dashboard_url()).rstrip("/")
    url = f"{base}/api/twin/state"
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=2) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, json.JSONDecodeError, TimeoutError):
        return {}


def remote_throttle_factor(state: dict) -> float | None:
    """Dashboard intervention throttle only (not soft allocation warnings).

    Returns a factor in (0, 1) when safety/agent is intervening; else None so
    teleop runs at full local capability.
    """
    safety = state.get("safety") or {}
    agent = state.get("agent") or {}
    intervening = bool(safety.get("throttle_required") or agent.get("intervening"))
    if not intervening:
        return None
    for source in (safety, agent):
        raw = source.get("throttle_factor")
        if raw is None:
            continue
        try:
            factor = float(raw)
        except (TypeError, ValueError):
            continue
        if 0.0 < factor < 1.0:
            return factor
    return None


def teleop_from_twin_state(state: dict) -> dict:
    """Unpack polled dashboard drive + battery replenish fields for Webots.

    Keys: left_v, right_v, active, source, stop_epoch, battery_pct, reset_thermal.
    """
    teleop = state.get("teleop") or {}
    return {
        "left_v": float(teleop.get("left_v") or 0.0),
        "right_v": float(teleop.get("right_v") or 0.0),
        "active": bool(teleop.get("active")),
        "source": teleop.get("source") or "",
        "stop_epoch": float(teleop.get("stop_epoch") or 0.0),
        "battery_pct": teleop.get("battery_pct"),
        "reset_thermal": bool(teleop.get("reset_thermal")),
        "lane_keep": bool(teleop.get("lane_keep")),
    }


def battery_from_twin_state(state: dict, default: float = 100.0) -> float:
    """Startup / sync battery % from bridge robot state (clamped 5–100)."""
    robot = state.get("robot") or {}
    raw = robot.get("main_battery_pct")
    if raw is None:
        return default
    try:
        return max(5.0, min(100.0, float(raw)))
    except (TypeError, ValueError):
        return default


def publish_telemetry(payload: dict, base_url: str | None = None) -> dict:
    """POST telemetry JSON to ``/api/twin/telemetry?adapter=webots``.

    Returns parsed JSON, or ``{"ok": False, "error": ...}`` on network failure.
    """
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


def _fallback_channel_draws(joints: list[dict], *, speed_m_s: float, gait: str) -> dict:
    """Motion-aware channel watts when ``webots_power`` cannot be imported in Webots."""
    by_name = {str(j.get("name", "")).lower(): j for j in (joints or [])}
    def _pw(name: str, idle: float) -> float:
        j = by_name.get(name) or {}
        if j.get("power_w") is not None:
            return float(j["power_w"])
        v = abs(float(j.get("velocity", 0.0) or 0.0))
        t = abs(float(j.get("torque", 0.0) or 0.0))
        if t < 1e-6 and v > 0.05:
            t = v * 0.45
        return idle + abs(t * v) * (3.6 if "wheel" in name else 4.5)

    legs = _pw("left_wheel", 2.5) + _pw("right_wheel", 2.5)
    # GPS can show speed while encoder vel lags one step — boost drive load from speed
    if speed_m_s > 0.05 or str(gait).lower() in ("drive", "transit", "walk"):
        motion = min(1.0, max(speed_m_s / 0.35, 0.0))
        legs = max(legs, 5.0 + 18.0 * motion)
    arms = _pw("left_arm", 1.6) + _pw("right_arm", 1.6)
    torso = _pw("torso_joint", 1.8)
    compute = 7.5 if speed_m_s < 0.06 else 9.5
    return {
        "Legs": round(min(legs, 28.0), 1),
        "Arms": round(min(max(arms, 3.0), 18.0), 1),
        "Torso": round(min(max(torso, 2.0), 14.0), 1),
        "Compute": round(compute, 1),
    }


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
    """Build the twin telemetry dict (prefer project ``webots_power``, else fallback)."""
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
    # Motion-aware fallback if project src not importable in Webots
    draws = _fallback_channel_draws(joints, speed_m_s=speed_m_s, gait=gait)
    return {
        "source": "webots",
        "adapter": "webots",
        "joints": joints,
        "locomotion": {
            "gait": gait,
            "speed_m_s": speed_m_s,
            "phase": phase,
            "mode": "wheeled",
        },
        "robot": {"name": "ButlerBot", "main_battery_pct": battery_pct},
        "motor_power_w": {
            str(j.get("name")): float(j.get("power_w") or 0.0) for j in (joints or [])
        },
        "channel_draws": draws,
        "power": {
            "total_draw_w": round(sum(draws.values()), 1),
            "channel_draws": draws,
        },
        "mission": {
            "task": (
                "moving"
                if str(gait).lower() in ("drive", "transit", "walk")
                else "idle"
            ),
            "phase": phase,
        },
    }