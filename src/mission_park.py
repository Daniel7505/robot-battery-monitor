"""When a finished track counts as parked for the mission summary.

The Webots ABS hold can stay open forever if GPS speed keeps twitching.
The energy log still needs one reliable "the S is done and the robot is
sitting there" signal. Hubs and the finish age are the backup. GPS speed
is not.
"""

from __future__ import annotations

# Same window as the controller's ABS timeout. After this long on a
# finished track with no drive command, the summary is logged anyway.
PARK_TIMEOUT_S = 12.0
# Hubs already quiet, and nobody is commanding the wheels.
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
    """True once, and on every later sample, after the track is parked."""
    if already_complete:
        return True
    if not track_done:
        return False
    if abs_done:
        return True
    age = max(0.0, float(finish_age_s))
    if hubs_quiet and no_drive_command and age >= HUB_QUIET_HOLD_S:
        return True
    if no_drive_command and age >= PARK_TIMEOUT_S:
        return True
    return False
