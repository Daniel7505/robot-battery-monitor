"""Launch one warehouse lap. The real launcher is Windows-only; tests pass a fake."""

from __future__ import annotations

import json
import os
import subprocess
import time
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

from optimizer.diagnose import diagnose_lap, real_summary_line
from optimizer.lap_launch import (
    BETWEEN_LAP_SETTLE_S,
    MIN_LAUNCH_WALL_S,
    LapLaunch,
    merge_launch_env,
)
from optimizer.trial_validity import summary_fresh_reason

REPO_ROOT = Path(__file__).resolve().parent.parent
DASHBOARD_CONTAINER = "robot-battery-dashboard"
DASHBOARD_URL = "http://127.0.0.1:5000/"


@dataclass
class RunOutcome:
    status: str  # ok, timeout, crash, no_summary
    log_text: str
    note: str = ""


def _summary_in(text: str) -> bool:
    """True only for a scored summary line (duration or distance present)."""
    return any(real_summary_line(line) for line in (text or "").splitlines())


def classify_summary(
    text: str,
    launch: LapLaunch | None,
    file_mtime: float | None = None,
) -> tuple[str, str]:
    """Classify the last real summary in ``text``.

    Returns ``("fresh", "")`` when the line belongs to this trial,
    ``("reject", reason)`` when a summary is present but must not end the lap,
    or ``("absent", "")`` when no scored summary line is present.
    """
    line = ""
    for candidate in (text or "").splitlines():
        if real_summary_line(candidate):
            line = candidate
    if not line:
        return "absent", ""
    summary = diagnose_lap(line).summary
    reason = summary_fresh_reason(summary, launch=launch, file_mtime=file_mtime)
    if reason:
        return "reject", reason
    return "fresh", ""


def _launch_log_excerpt(path: Path | None, limit: int = 500) -> str:
    """ERROR / WARNING / launch lines from this trial's launcher log."""
    if path is None or not path.is_file():
        return ""
    try:
        raw = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    lines = [line.strip() for line in raw.splitlines() if line.strip()]
    picked = [
        line
        for line in lines
        if line.startswith("ERROR")
        or line.startswith("WARNING")
        or line.startswith("optimizer:")
        or line.startswith("Launching")
        or line.startswith("--- retry")
    ]
    chosen = picked[-4:] or lines[-4:]
    return " | ".join(chosen)[:limit]


def stamp_mission_log(text: str, launch: LapLaunch | None) -> str:
    """Attach optimizer trial stamps to MISSION_SUMMARY lines (tests and dry-run)."""
    if launch is None or "MISSION_SUMMARY" not in text or "opt_run_id=" in text:
        return text
    stamped: list[str] = []
    for line in text.splitlines():
        if "MISSION_SUMMARY" in line:
            line = (
                f"{line.rstrip()} opt_run_id={launch.run_id} "
                f"opt_trial_id={launch.trial_id}"
            )
        stamped.append(line)
    return "\n".join(stamped)


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


def _kill_stray_controller_python() -> None:
    """Best-effort cleanup of a stuck butlerbot controller after a short Webots exit."""
    if os.name != "nt":
        return
    subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-Command",
            "Get-CimInstance Win32_Process -Filter \"name='python.exe'\" -ErrorAction SilentlyContinue | "
            "Where-Object { $_.CommandLine -match 'butlerbot_controller' } | "
            "ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }",
        ],
        timeout=45,
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


def _summary_folder() -> Path:
    try:
        from src.mission_summary_file import summary_directory

        return Path(summary_directory())
    except Exception:
        return REPO_ROOT / "logs"


def _mission_summary_pick(
    since_local: datetime | None,
    *,
    launch: LapLaunch | None = None,
) -> tuple[str, float | None]:
    """Newest host ``mission_summary_*.json`` that matches this trial, if any."""
    folder = _summary_folder()
    if not folder.is_dir():
        return "", None
    files = [path for path in folder.glob("mission_summary_*.json") if path.is_file()]
    if not files:
        return "", None
    candidates: list[tuple[float, str, float]] = []
    for path in files:
        try:
            mtime = path.stat().st_mtime
        except OSError:
            continue
        if since_local is not None and datetime.fromtimestamp(mtime) < since_local:
            continue
        if launch is not None:
            launched_local = launch.launched_utc.astimezone().replace(tzinfo=None)
            if datetime.fromtimestamp(mtime) < launched_local - timedelta(seconds=1):
                continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if launch is not None and isinstance(payload, dict):
            if payload.get("opt_run_id") not in (None, launch.run_id):
                continue
            if payload.get("opt_trial_id") not in (None, launch.trial_id):
                continue
        lines = payload.get("lines") if isinstance(payload, dict) else None
        if not isinstance(lines, list):
            continue
        text = "\n".join(str(line) for line in lines if line)
        if "MISSION_SUMMARY" not in text:
            continue
        summary = diagnose_lap(text).summary
        stale = summary_fresh_reason(summary, launch=launch, file_mtime=mtime)
        if stale:
            continue
        candidates.append((mtime, text, mtime))
    if not candidates:
        return "", None
    _mtime, text, raw_mtime = max(candidates, key=lambda item: item[0])
    return text, raw_mtime


def _mission_summary_text(
    since_local: datetime | None,
    *,
    launch: LapLaunch | None = None,
) -> str:
    text, _mtime = _mission_summary_pick(since_local, launch=launch)
    return text


def _append_mission_summary(
    text: str,
    since_local: datetime | None,
    note: str,
    *,
    launch: LapLaunch | None = None,
) -> tuple[str, str]:
    summary_text, _mtime = _mission_summary_pick(since_local, launch=launch)
    if not summary_text:
        return text, note
    combined = summary_text if not text else text.rstrip() + "\n" + summary_text
    extra = "included logs/mission_summary_*.json"
    note = f"{note}; {extra}" if note else extra
    return combined, note


def read_dashboard_log(
    *,
    since_utc: datetime | None = None,
    since_local: datetime | None = None,
    tail: int | None = None,
    launch: LapLaunch | None = None,
) -> tuple[str, str]:
    """Docker logs, the dashboard file, and main's host mission-summary JSON.

    The controller prints into the Webots console, which cannot be redirected
    on Windows. The lap lines that matter are in ``logs/mission_summary_*.json``
    (and mirrored into ``robot_monitor_*.log``). Docker is still read first.
    """
    cmd = ["docker", "logs", DASHBOARD_CONTAINER]
    if tail is not None:
        cmd[2:2] = ["--tail", str(tail)]
    elif since_utc is not None:
        cmd[2:2] = ["--since", since_utc.strftime("%Y-%m-%dT%H:%M:%SZ")]
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=90,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        text = _file_log(since_local)
        note = f"docker logs failed ({exc}); used the dashboard file log"
        return _append_mission_summary(text, since_local, note, launch=launch)
    text = proc.stdout or ""
    if proc.stderr:
        text = text + ("\n" if text else "") + proc.stderr
    note = ""
    if proc.returncode != 0:
        file_text = _file_log(since_local)
        if file_text:
            text = file_text
            note = f"docker logs exit {proc.returncode}; used the dashboard file log"
    return _append_mission_summary(text, since_local, note, launch=launch)


def _finish_lap_outcome(
    *,
    launch: LapLaunch,
    proc: subprocess.Popen,
    text: str,
    note: str,
    wall_started: float,
    status_if_summary: str = "ok",
    ignored: str = "",
    log_path: Path | None = None,
) -> RunOutcome:
    """Build the outcome after the launcher has stopped or a fresh summary arrived.

    A summary that fails the stamp / mtime / duration checks does not count.
    ``ignored`` is that rejection, kept so the trial log says why it was invalid.
    """
    elapsed = time.monotonic() - wall_started
    kind, detail = classify_summary(text, launch)
    if kind == "fresh":
        if elapsed < MIN_LAUNCH_WALL_S:
            return RunOutcome(
                "crash",
                text,
                (
                    f"lap returned MISSION_SUMMARY after {elapsed:.1f}s "
                    f"(<{MIN_LAUNCH_WALL_S:.0f}s); likely cached summary, "
                    f"existing Webots, or launcher early-exit. {note}"
                ).strip(),
            )
        return RunOutcome(status_if_summary, text, note)
    extra = detail or ignored
    excerpt = _launch_log_excerpt(log_path)
    if excerpt:
        extra = f"{extra} launch log: {excerpt}".strip()
    if proc.poll() is not None and kind != "fresh":
        return RunOutcome(
            "crash",
            text,
            (
                f"launcher exited {proc.returncode} before a fresh MISSION_SUMMARY "
                f"for this trial. {extra} {note}"
            ).strip(),
        )
    if extra:
        return RunOutcome(
            "no_summary",
            text,
            f"{extra}. {note}".strip(),
        )
    return RunOutcome(
        "crash",
        text,
        f"launcher exited {proc.returncode} before a fresh MISSION_SUMMARY. {note}".strip(),
    )


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

    def settle_between_laps(self) -> str:
        """Gap before back-to-back launches (confirm pair, retries)."""
        self.cleanup()
        time.sleep(float(BETWEEN_LAP_SETTLE_S))
        return f"settled {BETWEEN_LAP_SETTLE_S:.0f}s between laps"

    def cleanup(self) -> str:
        """Stop Webots and a leftover butlerbot controller. No settle sleep."""
        _kill_webots()
        _kill_stray_controller_python()
        return "stopped webots and butlerbot_controller"

    def restart_dashboard(self) -> str:
        try:
            proc = subprocess.run(
                ["docker", "restart", DASHBOARD_CONTAINER],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
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
        launch: LapLaunch | None = None,
    ) -> RunOutcome:
        if os.name != "nt":
            raise RuntimeError(
                "Webots laps run on Windows only. Start scripts\\overnight_optimize.ps1 "
                "on the PC that has Webots, or pass --dry-run."
            )
        launch = launch or LapLaunch.new("local", "lap")
        world_name = world or self.world
        since_utc = launch.launched_utc - timedelta(seconds=2)
        since_local = launch.launched_utc.astimezone().replace(tzinfo=None) - timedelta(seconds=2)
        wall_started = launch.launched_monotonic
        self.cleanup()
        config_note = ""
        try:
            from optimizer.lap_config import apply_trial_params

            config_note = apply_trial_params(self.repo, params)
        except Exception as exc:
            config_note = f"optimizer: config apply failed ({type(exc).__name__}: {exc})"
        env = merge_launch_env(os.environ.copy(), launch)
        env["RBM_MISSION_LAPS"] = "1"
        build = str((params or {}).get("bom_build") or "").strip()
        if build:
            # launch_webots_twin.ps1 refuses the shipped world when this is set.
            env["RBM_OPT_BOM_BUILD"] = build
        # The controller prints arrows. A cp1252 console turns those into a crash.
        env.setdefault("PYTHONIOENCODING", "utf-8")
        env.setdefault("PYTHONUTF8", "1")
        sink = None
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            sink = open(log_path, "ab")
            ctrl_log = log_path.with_name(f"{log_path.stem}_controller{log_path.suffix}")
            env["RBM_CONTROLLER_LOG"] = str(ctrl_log)
            sink.write(
                (
                    f"optimizer: launching trial={launch.trial_id} world={world_name} "
                    f"bom_build={build or 'unset'}\n{config_note}\n"
                    f"optimizer: controller console -> {ctrl_log}\n"
                ).encode()
            )
            sink.flush()
        proc = subprocess.Popen(
            launch_command(self.repo, world_name),
            cwd=str(self.repo),
            env=env,
            stdout=sink or subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        deadline = time.time() + float(timeout_s)
        note = ""
        ignored = ""

        def _read() -> str:
            nonlocal note
            text, note = read_dashboard_log(
                since_utc=since_utc,
                since_local=since_local,
                launch=launch,
            )
            return text

        try:
            while True:
                text = _read()
                kind, detail = classify_summary(text, launch)
                if kind == "reject" and detail and detail != ignored:
                    ignored = detail
                    if sink is not None:
                        sink.write(f"optimizer: ignoring summary ({detail})\n".encode())
                        sink.flush()
                if kind == "fresh":
                    self.cleanup()
                    try:
                        proc.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                    return _finish_lap_outcome(
                        launch=launch,
                        proc=proc,
                        text=text,
                        note=note,
                        wall_started=wall_started,
                        ignored=ignored,
                        log_path=log_path,
                    )
                if proc.poll() is not None:
                    text = _read()
                    kind, detail = classify_summary(text, launch)
                    if kind == "reject" and detail:
                        ignored = detail
                    # The launcher can return while webots-bin and the controller
                    # keep running. Always reap them before the next trial.
                    self.cleanup()
                    if kind == "fresh":
                        return _finish_lap_outcome(
                            launch=launch,
                            proc=proc,
                            text=text,
                            note=note,
                            wall_started=wall_started,
                            ignored=ignored,
                            log_path=log_path,
                        )
                    return _finish_lap_outcome(
                        launch=launch,
                        proc=proc,
                        text=text,
                        note=note,
                        wall_started=wall_started,
                        ignored=ignored or detail,
                        log_path=log_path,
                    )
                if time.time() >= deadline:
                    self.cleanup()
                    try:
                        proc.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                    text = _read()
                    kind, detail = classify_summary(text, launch)
                    if kind == "fresh":
                        return _finish_lap_outcome(
                            launch=launch,
                            proc=proc,
                            text=text,
                            note=note,
                            wall_started=wall_started,
                            status_if_summary="timeout",
                            ignored=ignored,
                            log_path=log_path,
                        )
                    why = ignored or detail or "no fresh MISSION_SUMMARY for this trial"
                    excerpt = _launch_log_excerpt(log_path)
                    if excerpt:
                        why = f"{why} launch log: {excerpt}"
                    return RunOutcome("timeout", text, f"{why}. {note}".strip())
                time.sleep(self.poll_s)
        finally:
            if proc.poll() is None:
                self.cleanup()
                proc.kill()
            else:
                self.cleanup()
            if sink is not None:
                sink.close()


class DryRunRunner:
    """Synthetic laps so the loop can be exercised without Webots.

    Slower than the default straight/cruise caps moves a negative err_pct
    toward zero and trims electrical drive. This is not a measurement.
    """

    def restart_dashboard(self) -> str:
        return "dry-run: dashboard restart skipped"

    def settle_between_laps(self) -> str:
        return "dry-run: settle skipped"

    def cleanup(self) -> str:
        return "dry-run: cleanup skipped"

    def run_lap(
        self,
        *,
        params: dict,
        timeout_s: float,
        world: str = "warehouse",
        log_path: Path | None = None,
        launch: LapLaunch | None = None,
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
        text = _summary_line(build, err, elec, shaft=0.866, params=params, launch=launch)
        return RunOutcome("ok", text, "dry-run synthetic lap")


def _summary_line(
    build: str,
    err: float,
    elec: float,
    shaft: float,
    params: dict | None = None,
    launch: LapLaunch | None = None,
) -> str:
    drive = elec / (1.0 + err / 100.0) if abs(err) < 500 else elec
    visc = drive * 0.85
    roll = drive * 0.10
    turn = drive * 0.04
    accel = drive * 0.01
    idle = 1.8
    ctrl = ""
    if params:
        cruise = float(params.get("controller.cruise_m_s") or 0.44)
        straight = float(params.get("controller.straight_m_s") or 0.50)
        pivot = float(params.get("controller.pivot_w_max_rad_s") or 0.9)
        k_steer = float(params.get("controller.k_steer") or 2.0)
        body = params.get("bom_build") or build or "light"
        mass_kg = 3.2
        try:
            from optimizer.body_mass import expected_body_mass_kg

            expected = expected_body_mass_kg(str(body))
            if expected is not None:
                mass_kg = expected
        except ImportError:
            pass
        ctrl = (
            f" ctrl_cruise_m_s={cruise:.2f} ctrl_straight_m_s={straight:.2f}"
            f" ctrl_pivot_w_max_rad_s={pivot:.2f} ctrl_k_steer={k_steer:.2f}"
            f" ctrl_bom_build={body} body_mass_kg={mass_kg:g}"
        )
    stamp = ""
    if launch is not None:
        stamp = f" opt_run_id={launch.run_id} opt_trial_id={launch.trial_id}"
    return (
        f"MISSION_SUMMARY build={build} duration_s=662.0 dist_m=200.91 turns=13 "
        f"payload_kg=0 soc_start=100.00 soc_end=97.00 "
        f"wh_model={idle + drive:.3f} wh_model_idle={idle:.3f} wh_model_drive={drive:.3f} "
        f"wh_model_roll={roll:.3f} wh_model_visc={visc:.3f} wh_model_turn={turn:.3f} "
        f"wh_model_accel={accel:.3f} "
        f"wh_measured_drive_shaft={shaft:.3f} wh_measured_drive_elec={elec:.3f} "
        f"eta_avg=0.230 wh_measured={elec + idle:.3f} wh_measured_soc={elec + idle:.3f} "
        f"err_pct={err:.1f}{ctrl}{stamp}"
    )
