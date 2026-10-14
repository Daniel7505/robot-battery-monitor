"""Launch one warehouse lap. The real launcher is Windows-only; tests pass a fake."""

from __future__ import annotations

import os
import subprocess
import time
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DASHBOARD_CONTAINER = "robot-battery-dashboard"
DASHBOARD_URL = "http://127.0.0.1:5000/"


@dataclass
class RunOutcome:
    status: str  # ok, timeout, crash, no_summary
    log_text: str
    note: str = ""


def _summary_in(text: str) -> bool:
    return "MISSION_SUMMARY" in text


def launch_command(repo: Path, world: str) -> list[str]:
    """The existing launcher, so BoM mass is patched in before Webots opens."""
    script = repo / "scripts" / "launch_webots_twin.ps1"
    return [
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(script),
        "-World",
        world,
    ]


def _kill_webots() -> None:
    subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-Command",
            "Get-Process -Name webots,webots-bin,webotsw -ErrorAction SilentlyContinue | "
            "Stop-Process -Force",
        ],
        timeout=60,
        check=False,
    )


def _line_time(line: str) -> datetime | None:
    # 2026-10-14 23:00:00,123 | INFO ...
    if len(line) < 23 or line[4] != "-" or line[10] != " ":
        return None
    try:
        return datetime.strptime(line[:23], "%Y-%m-%d %H:%M:%S,%f")
    except ValueError:
        return None


def _file_log(since_local: datetime | None) -> str:
    folder = REPO_ROOT / "logs"
    files = sorted(folder.glob("robot_monitor_*.log"))
    if not files:
        return ""
    try:
        lines = files[-1].read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return ""
    if since_local is None:
        return "\n".join(lines[-4000:])
    kept = []
    for line in lines:
        stamp = _line_time(line)
        if stamp is None or stamp >= since_local:
            kept.append(line)
    return "\n".join(kept)


def read_dashboard_log(
    *,
    since_utc: datetime | None = None,
    since_local: datetime | None = None,
    tail: int | None = None,
) -> tuple[str, str]:
    """``docker logs robot-battery-dashboard``, with the day's file as a fallback."""
    cmd = ["docker", "logs", DASHBOARD_CONTAINER]
    if tail is not None:
        cmd[2:2] = ["--tail", str(tail)]
    elif since_utc is not None:
        cmd[2:2] = ["--since", since_utc.strftime("%Y-%m-%dT%H:%M:%SZ")]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=90, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return _file_log(since_local), f"docker logs failed ({exc}); used the dashboard file log"
    text = proc.stdout or ""
    if proc.stderr:
        text = text + ("\n" if text else "") + proc.stderr
    if proc.returncode != 0:
        file_text = _file_log(since_local)
        if file_text:
            return file_text, f"docker logs exit {proc.returncode}; used the dashboard file log"
    return text, ""


def _wait_dashboard(timeout_s: float = 90.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(DASHBOARD_URL, timeout=3) as resp:
                if getattr(resp, "status", 200) == 200:
                    return True
        except Exception:
            time.sleep(2)
    return False


class WebotsLapRunner:
    """Kill Webots, set a one-lap mission, launch the warehouse through the ps1."""

    def __init__(self, repo: Path | None = None, world: str = "warehouse", poll_s: float = 15.0):
        self.repo = Path(repo) if repo else REPO_ROOT
        self.world = world
        self.poll_s = poll_s

    def launch_argv(self) -> list[str]:
        return launch_command(self.repo, self.world)

    def recent_log(self) -> str:
        text, _note = read_dashboard_log(tail=4000)
        return text

    def restart_dashboard(self) -> str:
        try:
            proc = subprocess.run(
                ["docker", "restart", DASHBOARD_CONTAINER],
                capture_output=True,
                text=True,
                timeout=180,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return f"failed: docker restart ({exc})"
        if proc.returncode != 0:
            detail = (proc.stderr or proc.stdout or "").strip()
            return f"failed: docker restart exit {proc.returncode} {detail}"
        if not _wait_dashboard():
            return "failed: dashboard did not answer on port 5000 after restart"
        return f"restarted {DASHBOARD_CONTAINER}"

    def run_lap(
        self,
        *,
        params: dict,
        timeout_s: float,
        world: str = "warehouse",
        log_path: Path | None = None,
    ) -> RunOutcome:
        del params  # the launcher reads config.yaml, which the loop already wrote
        if os.name != "nt":
            raise RuntimeError(
                "Webots laps run on Windows only. Start scripts\\overnight_optimize.ps1 "
                "on the PC that has Webots, or pass --dry-run."
            )
        world_name = world or self.world
        since_utc = datetime.now(timezone.utc) - timedelta(seconds=2)
        since_local = datetime.now() - timedelta(seconds=2)
        _kill_webots()
        env = os.environ.copy()
        env["RBM_MISSION_LAPS"] = "1"
        sink = None
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            sink = open(log_path, "ab")
        proc = subprocess.Popen(
            launch_command(self.repo, world_name),
            cwd=str(self.repo),
            env=env,
            stdout=sink or subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        deadline = time.time() + float(timeout_s)
        note = ""
        try:
            while True:
                text, note = read_dashboard_log(since_utc=since_utc, since_local=since_local)
                if _summary_in(text):
                    _kill_webots()
                    try:
                        proc.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                    return RunOutcome("ok", text, note)
                if proc.poll() is not None:
                    text, note = read_dashboard_log(since_utc=since_utc, since_local=since_local)
                    if _summary_in(text):
                        return RunOutcome("ok", text, note)
                    return RunOutcome(
                        "crash",
                        text,
                        f"launcher exited {proc.returncode} before MISSION_SUMMARY. {note}".strip(),
                    )
                if time.time() >= deadline:
                    _kill_webots()
                    try:
                        proc.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                    text, note = read_dashboard_log(since_utc=since_utc, since_local=since_local)
                    status = "ok" if _summary_in(text) else "timeout"
                    return RunOutcome(status, text, note)
                time.sleep(self.poll_s)
        finally:
            if proc.poll() is None:
                _kill_webots()
                proc.kill()
            if sink is not None:
                sink.close()


class DryRunRunner:
    """Synthetic laps so the loop can be exercised without Webots.

    Slower than the default straight/cruise caps moves a negative err_pct
    toward zero and trims electrical drive. This is not a measurement.
    """

    def restart_dashboard(self) -> str:
        return "dry-run: dashboard restart skipped"

    def run_lap(
        self,
        *,
        params: dict,
        timeout_s: float,
        world: str = "warehouse",
        log_path: Path | None = None,
    ) -> RunOutcome:
        del timeout_s, world, log_path
        build = params.get("bom_build") or "light"
        base_err = {"light": -28.3, "heavy": -5.0, "budget": -12.0}.get(build, -20.0)
        base_elec = {"light": 3.742, "heavy": 4.10, "budget": 4.40}.get(build, 4.0)
        straight = float(params.get("controller.straight_m_s") or 0.50)
        cruise = float(params.get("controller.cruise_m_s") or 0.44)
        slow_steps = (0.50 - straight) / 0.02 + (0.44 - cruise) / 0.02
        err = base_err + 3.0 * slow_steps
        elec = max(0.5, base_elec * (1.0 - 0.02 * slow_steps))
        text = _summary_line(build, err, elec, shaft=0.866)
        return RunOutcome("ok", text, "dry-run synthetic lap")


def _summary_line(build: str, err: float, elec: float, shaft: float) -> str:
    drive = elec / (1.0 + err / 100.0) if abs(err) < 500 else elec
    visc = drive * 0.85
    roll = drive * 0.10
    turn = drive * 0.04
    accel = drive * 0.01
    idle = 1.8
    return (
        f"MISSION_SUMMARY build={build} duration_s=662.0 dist_m=200.91 turns=13 "
        f"payload_kg=0 soc_start=100.00 soc_end=97.00 "
        f"wh_model={idle + drive:.3f} wh_model_idle={idle:.3f} wh_model_drive={drive:.3f} "
        f"wh_model_roll={roll:.3f} wh_model_visc={visc:.3f} wh_model_turn={turn:.3f} "
        f"wh_model_accel={accel:.3f} "
        f"wh_measured_drive_shaft={shaft:.3f} wh_measured_drive_elec={elec:.3f} "
        f"eta_avg=0.230 wh_measured={elec + idle:.3f} wh_measured_soc={elec + idle:.3f} "
        f"err_pct={err:.1f}"
    )
