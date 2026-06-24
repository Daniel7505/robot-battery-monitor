"""
One-click demo mode — UI highlight state + optional Webots launch on host.
"""

from __future__ import annotations

import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from src.config import config
from src.logger import logger

_demo: dict = {
    "active": False,
    "started_at": None,
    "launch_attempted": False,
    "launch_message": "",
}


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _in_container() -> bool:
    return os.path.exists("/.dockerenv") or os.environ.get("RUNNING_IN_DOCKER") == "1"


def activate_demo() -> dict:
    _demo["active"] = True
    _demo["started_at"] = datetime.now(timezone.utc).isoformat()
    _demo["launch_attempted"] = False
    _demo["launch_message"] = ""
    logger.info("Demo mode activated")
    return status()


def deactivate_demo() -> dict:
    _demo["active"] = False
    return status()


def status() -> dict:
    twin_cfg = config.get("digital_twin") or {}
    dash_port = config.get("dashboard", "port", 5000)
    dash_host = config.get("dashboard", "host", "127.0.0.1")
    return {
        "active": _demo["active"],
        "started_at": _demo["started_at"],
        "launch_attempted": _demo["launch_attempted"],
        "launch_message": _demo["launch_message"],
        "in_container": _in_container(),
        "dashboard_url": twin_cfg.get("webots", {}).get("dashboard_url")
        or f"http://{dash_host}:{dash_port}",
        "launch_script": "scripts/launch_webots_twin.ps1",
        "world": "webots/worlds/butlerbot.wbt",
    }


def launch_webots_on_host() -> dict:
    """Start Webots twin on the host OS (no-op with instructions when in Docker)."""
    st = status()
    if _in_container():
        _demo["launch_attempted"] = True
        _demo["launch_message"] = (
            "Dashboard is in Docker — run .\\scripts\\launch_webots_twin.ps1 on the host "
            "or .\\scripts\\launch_demo.ps1 from PowerShell."
        )
        return {**st, "ok": False, "launched": False}

    script = _project_root() / "scripts" / "launch_webots_twin.ps1"
    if not script.is_file():
        _demo["launch_message"] = f"Launch script not found: {script}"
        return {**st, "ok": False, "launched": False}

    if sys.platform != "win32":
        _demo["launch_message"] = "Auto-launch is Windows-only; open Webots with webots/worlds/butlerbot.wbt"
        return {**st, "ok": False, "launched": False}

    try:
        subprocess.Popen(
            [
                "powershell",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(script),
                "-DashboardUrl",
                st["dashboard_url"],
            ],
            cwd=str(_project_root()),
            creationflags=subprocess.CREATE_NEW_CONSOLE,
        )
        _demo["launch_attempted"] = True
        _demo["launch_message"] = "Webots launch started in new console"
        logger.info("Demo: launched Webots via launch_webots_twin.ps1")
        return {**status(), "ok": True, "launched": True}
    except Exception as exc:
        _demo["launch_message"] = str(exc)
        logger.warning(f"Demo Webots launch failed: {exc}")
        return {**status(), "ok": False, "launched": False, "error": str(exc)}