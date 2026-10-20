"""Per-lap identity for mission summaries and config restore checks."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone

# Warehouse laps are ~11 min. A lap that returns in seconds reused a cached summary.
MIN_LAUNCH_WALL_S = 45.0
MIN_MISSION_DURATION_S = 120.0

ENV_RUN_ID = "RBM_OPT_RUN_ID"
ENV_TRIAL_ID = "RBM_OPT_TRIAL_ID"


@dataclass(frozen=True)
class LapLaunch:
    run_id: str
    trial_id: str
    launched_utc: datetime
    launched_monotonic: float

    @classmethod
    def new(cls, run_id: str, trial_id: str) -> LapLaunch:
        return cls(
            run_id=run_id,
            trial_id=trial_id,
            launched_utc=datetime.now(timezone.utc),
            launched_monotonic=time.monotonic(),
        )


def trial_id_for(phase: str, iteration: int, *, part: str | None = None) -> str:
    base = f"{phase}-{iteration}"
    return f"{base}-{part}" if part else base


def launch_env(launch: LapLaunch) -> dict[str, str]:
    return {ENV_RUN_ID: launch.run_id, ENV_TRIAL_ID: launch.trial_id}


def merge_launch_env(base: dict[str, str], launch: LapLaunch | None) -> dict[str, str]:
    merged = dict(base)
    if launch is not None:
        merged.update(launch_env(launch))
    return merged


def assert_light_search_config(params: dict, run_body: str) -> None:
    """Fail loudly when the next search lap would not run on the fixed body."""
    from optimizer.trial_validity import run_body_from_params

    got = run_body_from_params(params)
    if got != run_body:
        raise RuntimeError(
            f"config bom_build={got!r} before search lap; expected fixed run body {run_body!r}"
        )
