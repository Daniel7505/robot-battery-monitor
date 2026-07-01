"""
ButlerBot simulation driver — realistic scripted mission loop.

Cycles Idle → Walking/Transit → Balanced/Patrol → High Load → Idle, driving
mission status, LRU draws, allocation, prediction, safety, and thermal together.
Power levels are tuned for small/medium servo motors on a compact mobile robot.
"""

from __future__ import annotations

from src.config import config
from src.logger import logger
from src.mission_tasks import TASK_PROFILES, TICK_SECONDS, MissionTaskManager

# ButlerBot default timeline (full realistic loop)
BUTLERBOT_SCRIPT = [
    {"task": "idle", "duration_s": 20, "label": "Idle / Standby"},
    {"task": "moving", "duration_s": 28, "label": "Walking / Transit"},
    {"task": "balanced", "duration_s": 22, "label": "Balanced / Patrol"},
    {"task": "high_load", "duration_s": 18, "label": "High Load / Manipulation"},
    {"task": "idle", "duration_s": 14, "label": "Return to Idle"},
]

# Small/medium servo motor draw targets (watts) per channel
BUTLERBOT_DRAW_PROFILES = {
    "idle": {"Legs": 3.5, "Arms": 5.0, "Torso": 3.5, "Compute": 7.5},
    "moving": {"Legs": 20.0, "Arms": 8.0, "Torso": 10.5, "Compute": 9.0},
    "balanced": {"Legs": 12.0, "Arms": 9.0, "Torso": 7.5, "Compute": 8.5},
    "high_load": {"Legs": 24.0, "Arms": 16.0, "Torso": 12.0, "Compute": 10.0},
}


def _load_script(cfg: dict) -> list[dict]:
    raw = cfg.get("script") or BUTLERBOT_SCRIPT
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
    return script or list(BUTLERBOT_SCRIPT)


class SimulationDriver:
    """Runs the ButlerBot (or configured) mission simulation loop."""

    def __init__(self):
        sim_cfg = config.get("simulation") or {}
        self._enabled = bool(sim_cfg.get("enabled", True))
        self._auto_start = bool(sim_cfg.get("auto_start", True))
        self._loop = bool(sim_cfg.get("loop", True))
        self._driver_name = sim_cfg.get("driver", "butlerbot")
        self._robot_name = sim_cfg.get("robot_name") or config.get("robot", "name", "ButlerBot")
        self._battery_wh = sim_cfg.get("battery_capacity_wh") or config.get(
            "robot", "main_battery_capacity_wh", 480
        )
        self._script = _load_script(sim_cfg)
        self._draw_profiles = {
            **BUTLERBOT_DRAW_PROFILES,
            **(sim_cfg.get("draw_profiles") or {}),
        }
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

    def draw_targets_for(self, task_id: str) -> dict[str, float]:
        custom = self._draw_profiles.get(task_id)
        if custom:
            return dict(custom)
        return dict(TASK_PROFILES[task_id].draw_targets)

    def start(self, mission: MissionTaskManager) -> None:
        if not self._enabled:
            logger.info("Simulation driver disabled in config")
            return
        self._running = True
        self._segment_idx = 0
        self._loops_completed = 0
        self._apply_current_segment(mission)
        logger.info(
            f"{self._robot_name} simulation started — {len(self._script)} phases, "
            f"loop={self._loop}, battery={self._battery_wh}Wh"
        )

    def stop(self) -> None:
        if self._running:
            logger.info(f"{self._robot_name} simulation stopped")
        self._running = False

    def reset(self, mission: MissionTaskManager) -> None:
        self._segment_idx = 0
        self._loops_completed = 0
        if self._running:
            self._apply_current_segment(mission)

    def advance(self, mission: MissionTaskManager) -> bool:
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
        targets = self.draw_targets_for(seg["task"])
        for ch_id, draw in targets.items():
            mission._blend[ch_id] = draw
        changed = prev != seg["task"]
        if changed:
            total = round(sum(targets.values()), 1)
            logger.info(
                f"{self._robot_name} phase {self._segment_idx + 1}/{len(self._script)}: "
                f"{seg['label']} (~{total}W, {seg['duration_s']}s)"
            )
        return changed

    def _advance_segment(self, mission: MissionTaskManager) -> bool:
        self._segment_idx += 1
        if self._segment_idx >= len(self._script):
            self._loops_completed += 1
            if self._loop:
                self._segment_idx = 0
                logger.info(
                    f"{self._robot_name} simulation loop #{self._loops_completed} — "
                    "restarting Idle → Transit → Patrol → High Load"
                )
            else:
                self.stop()
                logger.info(f"{self._robot_name} simulation script complete")
                return False

        return self._apply_current_segment(mission)

    def status(self, mission: MissionTaskManager | None = None) -> dict:
        seg = self._script[self._segment_idx] if self._script else {}
        task = seg.get("task", mission.task_id if mission else "")
        targets = self.draw_targets_for(task) if task in TASK_PROFILES else {}
        return {
            "driver": self._driver_name,
            "robot_name": self._robot_name,
            "enabled": self._enabled,
            "running": self._running,
            "loop": self._loop,
            "loops_completed": self._loops_completed,
            "battery_capacity_wh": self._battery_wh,
            "segment_index": self._segment_idx + 1 if self._script else 0,
            "segment_total": len(self._script),
            "segment_label": seg.get("label", ""),
            "segment_task": task,
            "segment_remaining_s": max(0, self._ticks_remaining * TICK_SECONDS),
            "expected_draw_w": round(sum(targets.values()), 1) if targets else None,
            "current_task": mission.task_id if mission else None,
            "script": [
                {
                    "task": s["task"],
                    "label": s["label"],
                    "duration_s": s["duration_s"],
                    "expected_draw_w": round(
                        sum(self.draw_targets_for(s["task"]).values()), 1
                    ),
                }
                for s in self._script
            ],
        }


def status_for_external_twin(
    mission: MissionTaskManager | None,
    phase: str | None,
    gait: str | None = None,
    *,
    source: str = "webots",
    live_draw_w: float | None = None,
) -> dict:
    """Dashboard simulation panel when Webots (not internal script) drives the loop."""
    from src.twin.butlerbot import BUTLERBOT_MISSION_FLOW
    from src.twin.control import PHASE_LABELS, webots_phase_flow

    flow = webots_phase_flow()
    norm = (phase or "").lower()
    idx = 0
    for i, step in enumerate(flow):
        sp = step.get("phase", "").lower()
        if sp == norm or (norm == "walk_transit" and sp == "drive_transit"):
            idx = i
            break

    step = flow[idx] if flow else {}
    script_step = BUTLERBOT_MISSION_FLOW[idx] if idx < len(BUTLERBOT_MISSION_FLOW) else {}
    draws = script_step.get("channel_draws") or {}
    return {
        "driver": source,
        "robot_name": config.get("robot", "name", "ButlerBot"),
        "enabled": True,
        "running": True,
        "external_twin": True,
        "twin_source": source,
        "loop": True,
        "loops_completed": None,
        "segment_index": idx + 1,
        "segment_total": len(flow),
        "segment_label": step.get("label") or PHASE_LABELS.get(norm, phase or "—"),
        "segment_task": script_step.get("task") or step.get("task", ""),
        "segment_remaining_s": None,
        "webots_phase": phase,
        "webots_gait": gait,
        "expected_draw_w": live_draw_w if live_draw_w is not None else (
            round(sum(float(v) for v in draws.values()), 1) if draws else None
        ),
        "current_task": mission.task_id if mission else None,
        "note": "Internal script paused — Webots twin drives the mission loop",
        "script": [
            {
                "task": s.get("task", ""),
                "label": s.get("label", ""),
                "duration_s": s.get("duration_s"),
                "phase": s.get("phase"),
                "expected_draw_w": round(
                    sum(
                        float(v)
                        for v in (
                            BUTLERBOT_MISSION_FLOW[i].get("channel_draws") or {}
                        ).values()
                    ),
                    1,
                )
                if i < len(BUTLERBOT_MISSION_FLOW)
                else None,
            }
            for i, s in enumerate(flow)
        ],
    }