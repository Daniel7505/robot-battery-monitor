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
"""

from __future__ import annotations

import json
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
OVERRIDE_PATH = _REPO / "data" / "twin_energy.json"

# Grounded in V1_WHEELED_ENERGY_BASELINE + 15 m hold-cruise logs.
TWIN_CRUISE = {
    "source": "v1_wheeled_energy_baseline",
    "ref": "docs/V1_WHEELED_ENERGY_BASELINE.md",
    "idle_w": 18.0,
    "cruise_total_w": 40.0,
    "cruise_legs_w": 21.0,
    "cruise_v_m_s": 0.44,
    "baseline_drive_w": 16.0,  # 2 × Pololu 37D cruise_w 8 W
    "s_turn_factor": 1.35,
}


def load_override(path: Path | None = None) -> dict | None:
    p = path if path is not None else OVERRIDE_PATH
    if not p.is_file():
        return None
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if data.get("wh_per_m") is None:
        return None
    return data


def straight_wh_per_m() -> float:
    v = float(TWIN_CRUISE["cruise_v_m_s"])
    w = float(TWIN_CRUISE["cruise_total_w"])
    if v <= 0:
        return 0.0
    return w / 3600.0 / v


def estimate_mission_energy(
    length_m: float,
    *,
    track: str = "s",
    motor_cont_w: float | None = None,
    motor_qty: int = 2,
    override_path: Path | None = None,
) -> dict:
    """Return twin-calibrated energy for a path of ``length_m``.

    Always succeeds (no Webots). ``source`` is ``twin_override`` or
    ``twin_calibrated``.
    """
    length = max(0.0, float(length_m))
    ov = load_override(override_path)
    if ov is not None:
        wh_m = float(ov["wh_per_m"])
        source = str(ov.get("source") or "twin_override")
        factor = 1.0
        draw_w = float(ov.get("cruise_total_w") or TWIN_CRUISE["cruise_total_w"])
        v = float(ov.get("cruise_v_m_s") or TWIN_CRUISE["cruise_v_m_s"])
    else:
        wh_m = straight_wh_per_m()
        factor = float(TWIN_CRUISE["s_turn_factor"]) if track == "s" else 1.0
        source = TWIN_CRUISE["source"]
        draw_w = float(TWIN_CRUISE["cruise_total_w"])
        v = float(TWIN_CRUISE["cruise_v_m_s"])
        if motor_cont_w is not None:
            drive = float(motor_cont_w) * max(1, int(motor_qty))
            rest = draw_w - float(TWIN_CRUISE["baseline_drive_w"])
            draw_w = max(1.0, drive + rest)
            if TWIN_CRUISE["cruise_total_w"]:
                wh_m *= draw_w / float(TWIN_CRUISE["cruise_total_w"])
    energy = length * wh_m * factor
    time_s = length / v if v > 0 else 0.0
    return {
        "energy_wh": round(energy, 3),
        "wh_per_m": round(wh_m * factor, 5),
        "draw_w": round(draw_w, 1),
        "v_m_s": v,
        "time_s": round(time_s * factor, 1) if track == "s" and ov is None else round(time_s, 1),
        "source": source,
        "track": track,
    }


def paper_energy(
    length_m: float,
    *,
    draw_w: float,
    time_s: float,
) -> dict:
    """Old first-cut math — kept so we can print the comparison."""
    e = max(0.0, float(draw_w)) * max(0.0, float(time_s)) / 3600.0
    return {
        "energy_wh": round(e, 3),
        "source": "paper",
        "draw_w": round(float(draw_w), 1),
        "time_s": round(float(time_s), 1),
    }
