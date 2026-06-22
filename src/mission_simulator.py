"""
Scripted mission timeline for mock robot simulation.

Drives task transitions through a repeatable script (e.g. Idle → Transit → Idle → High Load)
so power allocation, LRU monitoring, predictions, and safety all follow realistic phases.
"""

from __future__ import annotations

from src.config import config
from src.logger import logger
from src.mission_tasks import TASK_PROFILES, TICK_SECONDS, MissionTaskManager

_DEFAULT_SCRIPT = [
    {"task": "idle", "duration_s": 18, "label": "Idle / Standby"},
    {"task": "moving", "duration_s": 24, "label": "Walking / Transit"},
    {"task": "idle", "duration_s": 15, "label": "Idle / Rest"},
    {"task": "high_load", "duration_s": 20, "label": "High Load / Manipulation"},
]


def _load_script(cfg: dict) -> list[dict]:
    raw = cfg.get("script") or _DEFAULT_SCRIPT
    script = []
    for entry in raw:
        task = (entry.get("task") or "").strip().lower()
        if task not in TASK_PROFILES:
            logger.warning(f"Skipping unknown simulation task: {task!r}")
            continue
        duration = int(entry.get("duration_s", 15))
        script.append({
            "task": task,
            "duration_s": max(TICK_SECONDS, duration),
            "label": entry.get("label") or TASK_PROFILES[task].label,
        })
    return script or list(_DEFAULT_SCRIPT)


class MissionSimulator:
    """Runs a scripted mission timeline with optional looping."""

    def __init__(self):
        sim_cfg = config.get("simulation") or {}
        self._enabled = bool(sim_cfg.get("enabled", True))
        self._auto_start = bool(sim_cfg.get("auto_start", True))
        self._loop = bool(sim_cfg.get("loop", True))
        self._script = _load_script(sim_cfg)
        self._running = False
        self._segment_idx = 0
        self._ticks_remaining = 0
        self._loops_completed = 0

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def auto_start(self) -> bool:
        return self._auto_start

    @property
    def running(self) -> bool:
        return self._running

    def start(self, mission: MissionTaskManager) -> None:
        if not self._enabled:
            logger.info("Mission simulator disabled in config")
            return
        self._running = True
        self._segment_idx = 0
        self._loops_completed = 0
        self._apply_current_segment(mission)
        logger.info(
            f"Mission simulator started — {len(self._script)} segments, loop={self._loop}"
        )

    def stop(self) -> None:
        if self._running:
            logger.info("Mission simulator stopped")
        self._running = False

    def reset(self, mission: MissionTaskManager) -> None:
        self._segment_idx = 0
        self._loops_completed = 0
        if self._running:
            self._apply_current_segment(mission)

    def advance(self, mission: MissionTaskManager) -> bool:
        """Advance one telemetry tick. Returns True when the active task changes."""
        if not self._enabled or not self._running:
            return mission.advance()

        mission._ticks_remaining = self._ticks_remaining
        self._ticks_remaining -= 1
        mission._blend_progress = min(1.0, mission._blend_progress + 0.10)

        if self._ticks_remaining > 0:
            return False

        return self._advance_segment(mission)

    def _apply_current_segment(self, mission: MissionTaskManager) -> bool:
        seg = self._script[self._segment_idx]
        prev = mission.task_id
        mission.force_task(seg["task"], duration_s=seg["duration_s"])
        self._ticks_remaining = mission._ticks_remaining
        changed = prev != seg["task"]
        if changed:
            logger.info(
                f"Simulation segment {self._segment_idx + 1}/{len(self._script)}: "
                f"{seg['label']} ({seg['duration_s']}s)"
            )
        return changed

    def _advance_segment(self, mission: MissionTaskManager) -> bool:
        self._segment_idx += 1
        if self._segment_idx >= len(self._script):
            self._loops_completed += 1
            if self._loop:
                self._segment_idx = 0
                logger.info(f"Mission simulator loop #{self._loops_completed} restart")
            else:
                self.stop()
                logger.info("Mission simulator finished script")
                return False

        return self._apply_current_segment(mission)

    def status(self, mission: MissionTaskManager | None = None) -> dict:
        seg = self._script[self._segment_idx] if self._script else {}
        return {
            "enabled": self._enabled,
            "running": self._running,
            "loop": self._loop,
            "loops_completed": self._loops_completed,
            "segment_index": self._segment_idx + 1 if self._script else 0,
            "segment_total": len(self._script),
            "segment_label": seg.get("label", ""),
            "segment_task": seg.get("task", mission.task_id if mission else ""),
            "segment_remaining_s": max(0, self._ticks_remaining * TICK_SECONDS),
            "current_task": mission.task_id if mission else None,
            "script": [
                {
                    "task": s["task"],
                    "label": s["label"],
                    "duration_s": s["duration_s"],
                }
                for s in self._script
            ],
        }