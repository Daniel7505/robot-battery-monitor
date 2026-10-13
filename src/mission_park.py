"""When a finished track counts as parked for the mission summary.

The Webots ABS hold can stay open forever if GPS speed keeps twitching.
The energy log still needs one reliable "the route is done and the robot
is sitting there" signal.

Completion is the track-finished signal, then a park:

* ``track_done`` is the controller's FULL S DONE / route-end crossing.
  A no-drive pause before that, including a slow first segment, is not
  the finish. The old 12 s no-drive timeout is not a completion path.
* After that signal, either the ABS park reports done, or both hubs are
  quiet and nobody is commanding the wheels, held for ``HUB_QUIET_HOLD_S``.

GPS speed is not part of this decision.
"""

from __future__ import annotations

# Hubs already quiet, and nobody is commanding the wheels, after the finish.
HUB_QUIET_HOLD_S = 1.0


def mission_should_complete(
    *,
    track_done: bool,
    already_complete: bool,
    abs_done: bool,
    hubs_quiet: bool,
    no_drive_command: bool,
    finish_age_s: float,
) -> bool:
    """True once the route has finished and the robot is parked or quiet.

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
    age = max(0.0, float(finish_age_s))
    if hubs_quiet and no_drive_command and age >= HUB_QUIET_HOLD_S:
        return True
    return False
