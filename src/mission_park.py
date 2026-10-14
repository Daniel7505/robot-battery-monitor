"""When a finished track counts as parked for the mission summary.

The Webots ABS hold can stay open forever if GPS speed keeps twitching,
and the hub-rate check (0.03 rad/s) stays noisy after the wheels are
told to stop. Either one used to leave FULL S DONE with no summary.

Completion is the track-finished signal, then a park:

* ``track_done`` is the controller's FULL S DONE / route-end crossing,
  or an optional distance cap (off unless configured). A no-drive pause
  before that, including a slow first segment, is not the finish. There
  is no pre-finish timeout.
* After that signal, the ABS park reporting done completes immediately.
* Otherwise nobody is commanding the wheels. Quiet hubs complete after
  ``HUB_QUIET_HOLD_S``. Hubs that never go quiet still complete after
  ``FINISH_PARK_HOLD_S``, so a finished route logs once.

A world with no finish (the warehouse perimeter loops) can stop on
that distance cap. The cap defaults to off, so the S track is unchanged.

GPS speed is not part of this decision.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

# Hubs already quiet, and nobody is commanding the wheels, after the finish.
HUB_QUIET_HOLD_S = 1.0
# Route is finished and the drive command is zero, but a hub is still
# twitching above the quiet threshold. Do not wait for ABS.
FINISH_PARK_HOLD_S = 3.0


def mission_should_complete(
    *,
    track_done: bool,
    already_complete: bool,
    abs_done: bool,
    hubs_quiet: bool,
    no_drive_command: bool,
    finish_age_s: float,
) -> bool:
    """True once the route has finished and the robot is no longer being driven.

    ``finish_age_s`` only counts time after the finish signal. It cannot
    complete a mission on its own, and it does not run before the track
    is finished.
    """
    if already_complete:
        return True
    if not track_done:
        return False
    if abs_done:
        return True
    if not no_drive_command:
        return False
    age = max(0.0, float(finish_age_s))
    if hubs_quiet and age >= HUB_QUIET_HOLD_S:
        return True
    if age >= FINISH_PARK_HOLD_S:
        return True
    return False


def mission_distance_reached(traveled_m: float, limit_m: float | None) -> bool:
    """True when a configured distance cap has been met.

    ``limit_m`` None, blank, or <= 0 means no cap. The S track does not
    stop early unless a cap is set.
    """
    limit = _positive_m(limit_m)
    if limit is None:
        return False
    try:
        traveled = float(traveled_m)
    except (TypeError, ValueError):
        return False
    return traveled >= limit


def resolve_mission_limit_m(
    *,
    config_m: float | None = None,
    env_m: str | None = None,
    env_laps: str | None = None,
    lap_m: float | None = None,
) -> float | None:
    """Distance cap in metres. Unset means the run goes to the track finish.

    ``RBM_MISSION_MAX_M`` wins over the YAML ``mission_max_m``. A set
    value of 0 (or blank-equivalent) turns the cap off and does not
    fall through to laps. ``RBM_MISSION_LAPS`` applies only when no
    metre cap is configured: laps times ``lap_m`` (the warehouse
    perimeter). Laps are not a second way to invent a drive coefficient.
    """
    if env_m is not None and str(env_m).strip() != "":
        return _positive_m(env_m)
    metres = _positive_m(config_m)
    if metres is not None:
        return metres
    laps = _positive_m(env_laps)
    lap = _positive_m(lap_m)
    if laps is None or lap is None:
        return None
    return laps * lap


def warehouse_perimeter_m(track_path: Path | None = None) -> float | None:
    """Length of the ``perimeter`` road in ``tracks/warehouse.json``.

    One lap of the taped loop. The file is generated; this only reads it.
    """
    path = (
        Path(track_path)
        if track_path is not None
        else Path(__file__).resolve().parents[1] / "tracks" / "warehouse.json"
    )
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return None
    points = None
    for road in data.get("roads") or []:
        if isinstance(road, dict) and road.get("name") == "perimeter":
            points = road.get("waypoints")
            break
    if not isinstance(points, list) or len(points) < 2:
        return None
    total = 0.0
    for start, end in zip(points, points[1:]):
        try:
            total += math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
        except (TypeError, ValueError, IndexError):
            return None
    return total if total > 0.0 else None


def _positive_m(value) -> float | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"", "none", "null", "off", "~", "default", "yaml"}:
        return None
    try:
        number = float(text)
    except (TypeError, ValueError):
        return None
    if number <= 0.0 or not math.isfinite(number):
        return None
    return number
