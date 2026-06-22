# src/mission_tasks.py
"""
Mission / task awareness with draw profiles and simple energy prediction.
"""

import random
from dataclasses import dataclass

TICK_SECONDS = 3


@dataclass(frozen=True)
class TaskProfile:
    id: str
    label: str
    description: str
    draw_targets: dict[str, float]
    budget_factor: float
    throttle_order: list[str]
    max_draw_delta: float
    variation_band: float
    smooth_factor: float

    @property
    def avg_draw_w(self) -> float:
        return round(sum(self.draw_targets.values()), 1)


TASK_PROFILES: dict[str, TaskProfile] = {
    "idle": TaskProfile(
        id="idle",
        label="Idle / Standby",
        description="Stationary — compute and sensors draw most power; locomotion minimal.",
        draw_targets={"Legs": 5.0, "Arms": 8.0, "Torso": 5.5, "Compute": 10.5},
        budget_factor=0.70,
        throttle_order=["Arms", "Torso", "Legs", "Compute"],
        max_draw_delta=0.22,
        variation_band=0.05,
        smooth_factor=0.14,
    ),
    "moving": TaskProfile(
        id="moving",
        label="Moving / Transit",
        description="Locomotion active — legs and torso drive increase; arms stabilized for transit.",
        draw_targets={"Legs": 26.0, "Arms": 11.0, "Torso": 13.5, "Compute": 12.0},
        budget_factor=0.92,
        throttle_order=["Compute", "Arms", "Torso", "Legs"],
        max_draw_delta=0.40,
        variation_band=0.07,
        smooth_factor=0.18,
    ),
    "balanced": TaskProfile(
        id="balanced",
        label="Balanced / Patrol",
        description="Moderate patrol pace — load spread evenly across locomotion, arms, and compute.",
        draw_targets={"Legs": 16.0, "Arms": 12.0, "Torso": 10.0, "Compute": 11.5},
        budget_factor=0.82,
        throttle_order=["Arms", "Compute", "Torso", "Legs"],
        max_draw_delta=0.30,
        variation_band=0.06,
        smooth_factor=0.16,
    ),
    "high_load": TaskProfile(
        id="high_load",
        label="High Load / Manipulation",
        description="Peak manipulation and locomotion — allocator may throttle lower-priority channels.",
        draw_targets={"Legs": 31.0, "Arms": 22.0, "Torso": 17.0, "Compute": 13.5},
        budget_factor=1.0,
        throttle_order=["Compute", "Torso", "Arms", "Legs"],
        max_draw_delta=0.50,
        variation_band=0.08,
        smooth_factor=0.20,
    ),
}

LOCOMOTION_PHASES: dict[str, str] = {
    "idle": "standby",
    "moving": "active_locomotion",
    "balanced": "patrol",
    "high_load": "manipulation_peak",
}

TASK_TRANSITIONS: dict[str, list[tuple[str, tuple[int, int], float]]] = {
    "idle": [("balanced", (6, 12), 0.40), ("moving", (8, 16), 0.60)],
    "moving": [
        ("high_load", (3, 5), 0.22),
        ("balanced", (5, 10), 0.33),
        ("idle", (10, 18), 0.45),
    ],
    "balanced": [("moving", (6, 12), 0.45), ("idle", (8, 14), 0.55)],
    "high_load": [("moving", (3, 6), 0.55), ("balanced", (4, 7), 0.45)],
}


def anticipated_phases(task_id: str) -> list[dict]:
    """Likely next mission phases with probability and expected draw."""
    options = TASK_TRANSITIONS.get(task_id, [])
    phases = []
    for next_task, _duration, prob in options:
        profile = TASK_PROFILES.get(next_task)
        if not profile:
            continue
        phases.append(
            {
                "task": next_task,
                "task_label": profile.label,
                "probability_pct": round(prob * 100, 1),
                "expected_draw_w": profile.avg_draw_w,
                "locomotion_phase": LOCOMOTION_PHASES.get(next_task, "unknown"),
            }
        )
    return phases


def predict_runtime(
    battery_pct: float,
    capacity_wh: float,
    current_draw_w: float,
    task_avg_draw_w: float | None = None,
) -> dict:
    """Estimate remaining energy and runtime at current and task-average draw."""
    energy_wh = round((battery_pct / 100.0) * capacity_wh, 1)

    def _minutes(draw_w: float) -> float | None:
        if draw_w <= 0:
            return None
        return round((energy_wh / draw_w) * 60, 1)

    runtime_current = _minutes(current_draw_w)
    runtime_task = _minutes(task_avg_draw_w) if task_avg_draw_w else None

    return {
        "energy_wh_remaining": energy_wh,
        "runtime_min_at_current_draw": runtime_current,
        "runtime_min_at_task_avg": runtime_task,
        "runtime_hours_at_current_draw": round(runtime_current / 60, 2) if runtime_current else None,
    }


class MissionTaskManager:
    def __init__(self, start_task: str = "idle"):
        self._task = start_task
        self._ticks_remaining = random.randint(12, 22)
        self._blend: dict[str, float] = {}
        self._blend_progress = 1.0
        self._sim_driver = None

    @property
    def task_id(self) -> str:
        return self._task

    @property
    def profile(self) -> TaskProfile:
        return TASK_PROFILES[self._task]

    @property
    def ticks_remaining(self) -> int:
        return self._ticks_remaining

    @property
    def seconds_remaining(self) -> int:
        return self._ticks_remaining * TICK_SECONDS

    @property
    def blend_progress(self) -> float:
        return self._blend_progress

    def _ease_blend(self, t: float) -> float:
        """Smooth ease-in-out for task transitions."""
        return t * t * (3 - 2 * t)

    def _resolve_draw_targets(self) -> dict[str, float]:
        try:
            from src.simulation_driver import SimulationDriver

            sim = getattr(self, "_sim_driver", None)
            if sim and sim.enabled and sim.running:
                return sim.draw_targets_for(self._task)
        except ImportError:
            pass
        return self.profile.draw_targets

    def attach_simulation_driver(self, driver) -> None:
        self._sim_driver = driver

    def target_draw(self, channel_id: str, max_draw_w: float, current_draw: float | None = None) -> float:
        new_target = self._resolve_draw_targets().get(channel_id, max_draw_w * 0.35)
        new_target = min(new_target, max_draw_w * 0.92)

        if channel_id not in self._blend:
            self._blend[channel_id] = current_draw if current_draw is not None else new_target

        start = self._blend[channel_id]
        eased = self._ease_blend(self._blend_progress)
        blended = start + (new_target - start) * eased
        return round(blended, 1)

    def force_task(self, task_id: str, duration_s: int | None = None) -> bool:
        """Apply an external mission command (e.g. from ROS2)."""
        if task_id not in TASK_PROFILES:
            return False
        self._task = task_id
        if duration_s is not None:
            self._ticks_remaining = max(1, duration_s // TICK_SECONDS)
        else:
            self._ticks_remaining = random.randint(12, 22)
        self._blend_progress = 0.0
        return True

    def advance(self) -> bool:
        self._ticks_remaining -= 1
        if self._ticks_remaining > 0:
            self._blend_progress = min(1.0, self._blend_progress + 0.10)
            return False

        options = TASK_TRANSITIONS.get(self._task, [("idle", (10, 20), 1.0)])
        roll = random.random()
        cumulative = 0.0
        chosen = options[-1]
        for option in options:
            cumulative += option[2]
            if roll <= cumulative:
                chosen = option
                break

        next_task, duration_range, _ = chosen
        self._task = next_task
        self._ticks_remaining = random.randint(*duration_range)
        self._blend_progress = 0.0
        return True

    def mission_info(
        self,
        battery_pct: float = 92.0,
        capacity_wh: float = 1000.0,
        current_draw_w: float = 0.0,
    ) -> dict:
        p = self.profile
        prediction = predict_runtime(battery_pct, capacity_wh, current_draw_w, p.avg_draw_w)
        return {
            "task": p.id,
            "task_label": p.label,
            "task_description": p.description,
            "task_remaining_s": self.seconds_remaining,
            "task_avg_draw_w": p.avg_draw_w,
            "budget_factor": p.budget_factor,
            **prediction,
        }