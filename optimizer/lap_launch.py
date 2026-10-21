"""Per-lap identity for mission summaries and config restore checks."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone

# Warehouse laps are ~11 min. A lap that returns in seconds reused a cached summary.
MIN_LAUNCH_WALL_S = 45.0
MIN_MISSION_DURATION_S = 120.0
# Let config.yaml flush and Webots/controller processes exit before the next launch.
CONFIG_SETTLE_S = 2.0
BETWEEN_LAP_SETTLE_S = 12.0

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


def params_match_intended(got: dict, intended: dict) -> str | None:
    """Return a reason when disk config does not match what we just wrote."""
    from optimizer.trial_validity import LIMIT_TOLERANCE, _LIMIT_KEYS

    for key in (*_LIMIT_KEYS, "bom_build"):
        if key not in intended:
            continue
        want = intended.get(key)
        got_val = got.get(key)
        if want is None:
            continue
        if got_val is None:
            return f"config on disk missing {key} after write"
        if key == "bom_build":
            if str(got_val).strip() != str(want).strip():
                return f"config bom_build={got_val!r} but expected {want!r}"
            continue
        if abs(float(want) - float(got_val)) > LIMIT_TOLERANCE:
            return f"config {key}={float(got_val):.3g} but expected {float(want):.3g}"
    return None


def wait_config(store, intended: dict, *, settle_s: float = CONFIG_SETTLE_S) -> str | None:
    """Sleep briefly and verify allowlisted keys on disk match ``intended``."""
    deadline = time.monotonic() + float(settle_s) + 4.0
    while time.monotonic() < deadline:
        time.sleep(min(0.4, max(0.0, deadline - time.monotonic())))
        try:
            mismatch = params_match_intended(store.read(), intended)
        except Exception as exc:
            mismatch = f"config read failed after write: {exc}"
        if mismatch is None:
            return None
    return mismatch or "config did not match intended values before launch"


def assert_light_search_config(params: dict, run_body: str) -> None:
    """Fail loudly when the next search lap would not run on the fixed body."""
    from optimizer.trial_validity import run_body_from_params

    got = run_body_from_params(params)
    if got != run_body:
        raise RuntimeError(
            f"config bom_build={got!r} before search lap; expected fixed run body {run_body!r}"
        )
