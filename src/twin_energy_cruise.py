"""V1 cruise baseline, JSON override, and the shared energy constants."""

from __future__ import annotations

import json
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
OVERRIDE_PATH = _REPO / "data" / "twin_energy.json"

# Flat-floor motor model from bom.json (Crr and driver η are assumptions there).
TEN_FT_M = 3.048
GRAVITY_M_S2 = 9.81
DEFAULT_CRR = 0.015
# Copper I²R above the rolling-only current at this speed (mission model).
MOTOR_COPPER_REF_SPEED_M_S = 0.44
DEFAULT_DRIVER_EFF = 0.92
# Drawn mesh and contact cylinder in every shipped butlerbot*.wbt, and the
# radius the controller's v = ω·r uses. A selected bom_build changes body
# mass only (src/webots_body.py). The catalog wheel diameter is not written
# onto the contact cylinder: hinge-damper shaft energy scales as 1/r², and
# a 0.04 m / 0.10 m tire made light and heavy disagree with this bill.
# This constant is that bill. It is not refit per build.
SIM_WHEEL_RADIUS_M = 0.08
SIM_HINGE_DAMPING_NM_S_RAD = 0.15
# butlerbot_warehouse.wbt WorldInfo contactProperties, drive_wheel vs
# default. The same block is in the other butlerbot*.wbt files.
# coulombFriction 8 is the no-slip cap: the world comment says μ=80 made
# yaw impossible. forceDependentSlip 0.01 is ODE's k in ``v = k · F``
# (Webots R2025a ContactProperties, quoting the ODE manual).
# frictionRotation is 0 0, so the world applies no bore torque and this
# model does not invent one. The tire cylinder height is 0.05 m; it is
# not a second slip length.
WORLD_DRIVE_COULOMB_FRICTION = 8.0
WORLD_DRIVE_FDS_M_S_PER_N = 0.01
TURN_SOURCE = "world_fds_track_scrub"

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


def straight_wh_per_m(speed_m_s: float | None = None, payload_kg: float = 0.0) -> float:
    """Wh per metre at cruise.

    With no ``bom_build`` this stays the V1 baseline (~0.025 Wh/m) and the
    extra arguments are ignored, so existing callers do not change.

    With a BoM build selected, this is ``total_wh_per_10ft / 3.048`` at the
    requested speed, plus ``payload_kg * extra_wh_per_10ft_per_kg / 3.048``.
    """
    from src.hardware_profile import active_bom_build, get_active_profile

    if active_bom_build():
        prof = get_active_profile()
        row = _energy_row(prof, speed_m_s)
        if row is None:
            return 0.0
        wh = float(row["total_wh_per_10ft"]) / TEN_FT_M
        extra = float(row.get("extra_wh_per_10ft_per_kg_payload") or 0.0) / TEN_FT_M
        return wh + float(payload_kg) * extra
    v = float(TWIN_CRUISE["cruise_v_m_s"])
    w = float(TWIN_CRUISE["cruise_total_w"])
    if v <= 0:
        return 0.0
    return w / 3600.0 / v


def _energy_row(profile: dict, speed_m_s: float | None) -> dict | None:
    rows = list(profile.get("energy_rows") or [])
    if not rows:
        return None
    if speed_m_s is None:
        speed_m_s = 0.44
    return min(rows, key=lambda row: abs(float(row["speed_m_s"]) - float(speed_m_s)))


def estimate_mission_energy(
    length_m: float,
    *,
    track: str = "s",
    motor_cont_w: float | None = None,
    motor_qty: int = 2,
    override_path: Path | None = None,
    payload_kg: float = 0.0,
    speed_m_s: float | None = None,
    use_override: bool = True,
) -> dict:
    """Return twin-calibrated energy for a path of ``length_m``.

    Always succeeds (no Webots). A ``data/twin_energy.json`` override wins
    over both the V1 baseline and a selected BoM build, because a measured
    Webots run should beat the paper model.

    With ``bom_build`` set and no override, Wh/m comes from that build's
    per-10-ft table (including the payload term). This shift table stays
    rolling-only. A logged run adds published hinge damping, the turn
    term, and the acceleration bound in ``bom_run_energy_wh``, not here.
    """
    length = max(0.0, float(length_m))
    ov = load_override(override_path) if use_override else None
    from src.hardware_profile import active_bom_build, get_active_profile

    bom_id = active_bom_build()
    if ov is not None:
        wh_m = float(ov["wh_per_m"])
        source = str(ov.get("source") or "twin_override")
        factor = 1.0
        draw_w = float(ov.get("cruise_total_w") or TWIN_CRUISE["cruise_total_w"])
        v = float(ov.get("cruise_v_m_s") or TWIN_CRUISE["cruise_v_m_s"])
    elif bom_id:
        prof = get_active_profile()
        speed = float(speed_m_s or 0.44)
        row = _energy_row(prof, speed) or {}
        wh_m = straight_wh_per_m(speed, payload_kg)
        factor = 1.0
        source = f"bom_{bom_id}"
        draw_w = float((prof.get("bom_totals") or {}).get("cruise_044_total_w") or 0.0)
        if row:
            # Scale the tabulated cruise draw when the caller asked for another speed.
            drive = float(row.get("drive_w") or 0.0)
            idle = float((prof.get("bom_totals") or {}).get("idle_w_known") or 0.0)
            draw_w = drive + idle
        v = speed
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
        "payload_kg": float(payload_kg),
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
