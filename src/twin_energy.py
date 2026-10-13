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
import math
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
OVERRIDE_PATH = _REPO / "data" / "twin_energy.json"

# Flat-floor motor model from bom.json (Crr and driver η are assumptions there).
TEN_FT_M = 3.048
GRAVITY_M_S2 = 9.81
DEFAULT_CRR = 0.015
DEFAULT_DRIVER_EFF = 0.92

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


def brushed_drive_point(
    *,
    mass_kg: float,
    wheel_radius_m: float,
    speed_m_s: float,
    voltage_v: float,
    no_load_rpm: float,
    no_load_current_a: float,
    stall_current_a: float,
    stall_torque_nm: float,
    crr: float = DEFAULT_CRR,
    eta_driver: float = DEFAULT_DRIVER_EFF,
    n_motors: int = 2,
) -> dict:
    """Linear brushed-DC point from bom.json ``formulas``.

    tau = Crr * m * g * r / 2
    duty = omega/omega0 + tau/tau_stall
    I = I0 + (I_stall - I0) * tau/tau_stall
    P_drive = 2 * duty * V * I / eta_driver
    """
    radius = max(float(wheel_radius_m), 1e-6)
    speed = float(speed_m_s)
    tau = float(crr) * float(mass_kg) * GRAVITY_M_S2 * radius / float(n_motors)
    omega = speed / radius
    omega0 = float(no_load_rpm) * 2.0 * math.pi / 60.0
    tau_stall = float(stall_torque_nm)
    if omega0 <= 0 or tau_stall <= 0 or speed <= 0:
        return {
            "duty": 0.0,
            "current_a": float(no_load_current_a),
            "torque_nm": tau,
            "drive_w": 0.0,
            "drive_wh_per_10ft": 0.0,
            "extra_wh_per_10ft_per_kg_payload": 0.0,
        }
    duty = omega / omega0 + tau / tau_stall
    current = float(no_load_current_a) + (
        float(stall_current_a) - float(no_load_current_a)
    ) * (tau / tau_stall)
    drive_w = float(n_motors) * duty * float(voltage_v) * current / float(eta_driver)
    seconds = TEN_FT_M / speed
    drive_wh = drive_w * seconds / 3600.0

    def _drive_at(mass: float) -> float:
        tau_m = float(crr) * mass * GRAVITY_M_S2 * radius / float(n_motors)
        duty_m = omega / omega0 + tau_m / tau_stall
        current_m = float(no_load_current_a) + (
            float(stall_current_a) - float(no_load_current_a)
        ) * (tau_m / tau_stall)
        return float(n_motors) * duty_m * float(voltage_v) * current_m / float(eta_driver)

    extra = (_drive_at(float(mass_kg) + 1.0) - drive_w) * seconds / 3600.0
    return {
        "duty": duty,
        "current_a": current,
        "torque_nm": tau,
        "drive_w": drive_w,
        "drive_wh_per_10ft": drive_wh,
        "extra_wh_per_10ft_per_kg_payload": extra,
    }


def shift_energy_wh(
    *,
    idle_w: float,
    shift_h: float,
    distance_m: float,
    drive_wh_per_10ft: float,
    extra_wh_per_10ft_per_kg: float,
    payload_kg: float,
) -> float:
    """Shift energy from the BoM rule, rounded to 0.01 Wh before any compare.

    Idle is charged for the whole shift. Drive and payload are charged only
    for the distance. Unknown idle watts are not added here — the caller
    fails the build instead of guessing them.
    """
    per_10ft = float(drive_wh_per_10ft) + float(payload_kg) * float(extra_wh_per_10ft_per_kg)
    energy = float(idle_w) * float(shift_h) + (float(distance_m) / TEN_FT_M) * per_10ft
    return round(energy, 2)


def bom_drive_terms(profile: dict, speed_m_s: float, mass_kg: float | None = None) -> dict:
    """Drive Wh at ``speed_m_s`` for a BoM profile, from the motor formula."""
    assumptions = profile.get("assumptions") or {}
    spec = (profile.get("motors") or {}).get("left_wheel") or {}
    geom = profile.get("geometry") or {}
    mass = float(geom.get("mass_kg_known") if mass_kg is None else mass_kg)
    return brushed_drive_point(
        mass_kg=mass,
        wheel_radius_m=float(geom.get("wheel_radius_m") or 0.04),
        speed_m_s=speed_m_s,
        voltage_v=float(spec.get("voltage_v") or 0.0),
        no_load_rpm=float(spec.get("no_load_rpm") or 0.0),
        no_load_current_a=float(spec.get("no_load_current_a") or 0.0),
        stall_current_a=float(spec.get("stall_current_a") or 0.0),
        stall_torque_nm=float(spec.get("stall_torque_nm") or 0.0),
        crr=float(assumptions.get("crr") or DEFAULT_CRR),
        eta_driver=float(assumptions.get("driver_eff") or DEFAULT_DRIVER_EFF),
        n_motors=2,
    )


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
    per-10-ft table (including the payload term). Turning losses are not
    in that table, so the S-track factor is not applied on top.
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


def format_mission_energy_line(
    dist_m: float,
    payload_kg: float | None = None,
    wh_measured: float | None = None,
) -> str:
    """``MISSION_ENERGY build=...`` line. Model and measured stay separate.

    The model ignores ``data/twin_energy.json`` so a measured Webots Wh can
    sit next to it as ``wh_measured`` / ``err_pct``. When that file is
    present and the caller did not pass ``wh_measured``, it is the measured
    value.
    """
    from src.config import config
    from src.hardware_profile import active_bom_build

    if payload_kg is None:
        payload_kg = float(config.get("robot", "payload_kg") or 0.0)
    model = estimate_mission_energy(
        dist_m,
        track="straight",
        payload_kg=float(payload_kg),
        use_override=False,
    )
    wh_model = model["energy_wh"]
    if wh_measured is None:
        ov = load_override()
        if ov is not None and ov.get("wh_per_m") is not None:
            wh_measured = float(ov["wh_per_m"]) * float(dist_m)
    if wh_measured is None:
        measured_txt = ""
        err_txt = ""
    else:
        measured_txt = f"{float(wh_measured):.2f}"
        if wh_model:
            err_txt = f"{(float(wh_measured) - wh_model) / wh_model * 100.0:.1f}"
        else:
            err_txt = ""
    build = active_bom_build() or "default"
    return (
        f"MISSION_ENERGY build={build} dist_m={dist_m} payload_kg={payload_kg} "
        f"wh_model={wh_model} wh_measured={measured_txt} err_pct={err_txt}"
    )


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
