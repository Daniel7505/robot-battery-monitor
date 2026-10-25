"""Twin-backed mission energy — calibrated, not a live physics search.

The design agent's first-cut paper math (~3 Wh on a 25 m S) assumed the
robot crawled for ``max_minutes``. Live twin data says otherwise:

* ``docs/V1_WHEELED_ENERGY_BASELINE.md`` — idle ~18 W total, cruise
  Legs ~21 W, system ~40 W at 0.44 m/s
* Hold-cruise 15 m @ ω=5.5 finished in ~37 s (session ids 21–27)

That is about **0.025 Wh/m** on a straight. An S with turns costs more;
``s_turn_factor`` (1.35) is a documented fudge, not a new lap.

Optional overlay: ``data/twin_energy.json`` with ``wh_per_m`` and
``source`` wins if present. Live Webots is **not** required. A later
cut can write that JSON from ``measure_track_run``.

The pieces live in sibling modules. This module re-exports what callers
already import. Shared constants stay with the cruise baseline. The
brushed circuit, pack meter, and stall-hold point are in
``twin_energy_motor``. The BoM and sim bill, hinge damping, and turn
scrub are in ``twin_energy_drive``. The segment rollup and the
``MISSION_ENERGY`` line are in ``twin_energy_mission``.
"""

from __future__ import annotations

from src.twin_energy_cruise import *  # noqa: F403
from src.twin_energy_motor import *  # noqa: F403
from src.twin_energy_drive import *  # noqa: F403
from src.twin_energy_drive import _hub_electrical_w, _profile_motor_args
from src.twin_energy_mission import *  # noqa: F403
