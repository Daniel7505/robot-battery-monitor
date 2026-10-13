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
    extra_force_n: float = 0.0,
) -> dict:
    """Linear brushed-DC point from bom.json ``formulas``.

    tau = (Crr * m * g + extra_force) * r / 2
    duty = omega/omega0 + tau/tau_stall
    I = I0 + (I_stall - I0) * tau/tau_stall
    P_drive = 2 * duty * V * I / eta_driver

    ``extra_force_n`` is an added longitudinal force (Newtons, both wheels
    together), used for the acceleration term. Zero keeps the rolling-only
    point. Torque is clamped to stall so a hard accel does not invent a
    current past the datasheet.
    """
    radius = max(float(wheel_radius_m), 1e-6)
    speed = float(speed_m_s)
    motors = max(1, int(n_motors))
    force_n = float(crr) * float(mass_kg) * GRAVITY_M_S2 + max(0.0, float(extra_force_n))
    tau = force_n * radius / float(motors)
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
    tau_elec = min(tau, tau_stall)
    duty = omega / omega0 + tau_elec / tau_stall
    current = float(no_load_current_a) + (
        float(stall_current_a) - float(no_load_current_a)
    ) * (tau_elec / tau_stall)
    drive_w = float(motors) * duty * float(voltage_v) * current / float(eta_driver)
    seconds = TEN_FT_M / speed
    drive_wh = drive_w * seconds / 3600.0

    def _drive_at(mass: float) -> float:
        tau_m = min(
            (float(crr) * mass * GRAVITY_M_S2 + max(0.0, float(extra_force_n)))
            * radius
            / float(motors),
            tau_stall,
        )
        duty_m = omega / omega0 + tau_m / tau_stall
        current_m = float(no_load_current_a) + (
            float(stall_current_a) - float(no_load_current_a)
        ) * (tau_m / tau_stall)
        return float(motors) * duty_m * float(voltage_v) * current_m / float(eta_driver)

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


def bom_drive_terms(
    profile: dict,
    speed_m_s: float,
    mass_kg: float | None = None,
    extra_force_n: float = 0.0,
) -> dict:
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
        extra_force_n=extra_force_n,
    )


# One-point fit from the live light-build S (Windows, commit 3b59d70).
# A curve SEGMENT_ENERGY logged about 0.12 Wh of locomotion per 10 ft.
# The published rolling term for that distance is 0.0066 Wh (light, ~0.44 m/s).
# The larger heading change on that run was 61.6 deg. These three numbers are
# the fit. They are not a friction coefficient from a datasheet.
FITTED_CURVE_WH = 0.12
FITTED_CURVE_ROLL_WH = 0.0066
FITTED_CURVE_HEADING_DEG = 61.6
TURN_SOURCE = "fitted-from-twin"


def chassis_track_width_m() -> float:
    """Axle track from the wheeled profile (metres). A chassis dimension, not a fit."""
    from src.hardware_profile import load_hardware_profile

    geom = load_hardware_profile("butlerbot_wheeled").get("geometry") or {}
    width = geom.get("track_width_m")
    return float(width) if width else 0.0


def geometry_scrub_wh(
    mass_kg: float,
    heading_change_deg: float,
    *,
    crr: float = DEFAULT_CRR,
    track_width_m: float | None = None,
    eta_driver: float = DEFAULT_DRIVER_EFF,
) -> float:
    """Lower bound: rolling resistance along the yaw arc, and nothing else.

    During a yaw ``Δθ``, each drive wheel's contact moves ``(track/2)·|Δθ|``
    relative to the chassis centre. Both wheels together travel
    ``track·|Δθ|``. Work is ``Crr · m · g · track · |Δθ|``. Electrical Wh is
    that work divided by driver efficiency.

    This is not the term the model bills. On the logged 61.6° curve it is
    about 0.0001 Wh, roughly a thousand times under the 0.12 Wh the twin
    logged. Skid-steer scrub is sliding friction, and the BoM has no μ.
    """
    theta = math.radians(abs(float(heading_change_deg)))
    track = chassis_track_width_m() if track_width_m is None else float(track_width_m)
    if theta <= 0 or track <= 0 or float(eta_driver) <= 0 or float(mass_kg) <= 0:
        return 0.0
    work_j = float(crr) * float(mass_kg) * GRAVITY_M_S2 * track * theta
    return (work_j / float(eta_driver)) / 3600.0


def fitted_turn_wh(
    profile: dict,
    heading_change_deg: float,
    speed_m_s: float,
    payload_kg: float = 0.0,
) -> float:
    """Skid-steer electrical Wh for a heading change. Source: fitted-from-twin.

    ``(0.12 − 0.0066) / radians(61.6)`` watt-hours per radian on the light
    build at payload 0. Another build, or a payload, is scaled by that
    profile's rolling Wh per 10 ft divided by 0.0066, so a heavier motor
    bill tracks the same motor model. Straight travel (no yaw) is zero,
    which leaves the 3.43 W cruise point unchanged.

    Assumption: the extra electrical energy of a yaw scales like the
    rolling electrical energy. That is a scale factor on one measured
    point, not a derived μ.
    """
    heading = abs(float(heading_change_deg))
    if heading <= 0 or FITTED_CURVE_HEADING_DEG <= 0 or FITTED_CURVE_ROLL_WH <= 0:
        return 0.0
    speed = float(speed_m_s) if float(speed_m_s) > 0.05 else 0.44
    terms = bom_drive_terms(profile, speed)
    per = float(terms["drive_wh_per_10ft"]) + float(payload_kg) * float(
        terms["extra_wh_per_10ft_per_kg_payload"]
    )
    per_rad = (FITTED_CURVE_WH - FITTED_CURVE_ROLL_WH) / math.radians(FITTED_CURVE_HEADING_DEG)
    return per_rad * math.radians(heading) * (per / FITTED_CURVE_ROLL_WH)


def accel_electrical_wh(
    profile: dict,
    v0_m_s: float,
    v1_m_s: float,
    dt_s: float,
    payload_kg: float = 0.0,
) -> float:
    """Extra electrical Wh while speeding up, from the same motor line.

    Added force is ``m · a`` (known BoM mass plus this run's payload; the
    trolley tare is unknown and stays out). The point is evaluated at the
    higher speed of the step, with and without that force, and the
    difference is billed for ``dt_s``. Slowing down bills nothing: the
    brushed model has no regen.

    This is a bound from the motor equation, not a fit. Wheel spin inertia
    is not in the BoM (mass is published, moment of inertia is not), so
    rotational KE is not added. On the light build a 0 → 0.44 m/s step is
    a few thousandths of a watt-hour, far under the curve gap the turn
    term is there to cover.
    """
    v0 = max(0.0, float(v0_m_s))
    v1 = max(0.0, float(v1_m_s))
    dt = float(dt_s)
    if dt <= 0 or v1 <= v0 + 1e-9:
        return 0.0
    geom = profile.get("geometry") or {}
    mass = float(geom.get("mass_kg_known") or 0.0) + max(0.0, float(payload_kg))
    if mass <= 0:
        return 0.0
    accel = (v1 - v0) / dt
    speed = max(v1, 1e-3)
    base = bom_drive_terms(profile, speed, mass_kg=mass)
    boosted = bom_drive_terms(profile, speed, mass_kg=mass, extra_force_n=mass * accel)
    return max(0.0, float(boosted["drive_w"]) - float(base["drive_w"])) * dt / 3600.0


def segment_model_terms(
    dist_m: float,
    avg_speed_m_s: float,
    heading_change_deg: float,
    wh_accel: float,
    payload_kg: float = 0.0,
    profile: dict | None = None,
) -> dict:
    """Model split for one segment: rolling, fitted yaw, acceleration bound."""
    from src.hardware_profile import get_active_profile

    prof = profile if profile is not None else get_active_profile()
    dist = max(0.0, float(dist_m))
    speed = float(avg_speed_m_s)
    if dist <= 0 or speed <= 0:
        roll = 0.0
    else:
        terms = bom_drive_terms(prof, speed)
        per = float(terms["drive_wh_per_10ft"]) + float(payload_kg) * float(
            terms["extra_wh_per_10ft_per_kg_payload"]
        )
        roll = (dist / TEN_FT_M) * per
    turn = fitted_turn_wh(prof, heading_change_deg, speed, payload_kg)
    return {
        "wh_roll": roll,
        "wh_turn": turn,
        "wh_accel": max(0.0, float(wh_accel)),
        "turn_source": TURN_SOURCE,
    }


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
    rolling-only. A logged run's yaw and accel are added in
    ``bom_run_energy_wh``, not here, so the S-track factor is not applied
    on top.
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


def bom_run_energy_wh(
    dist_m: float,
    duration_s: float,
    payload_kg: float = 0.0,
    profile: dict | None = None,
    heading_change_deg: float = 0.0,
    wh_accel: float = 0.0,
) -> dict | None:
    """Idle for the whole run, plus roll + yaw + accel for the distance.

    ``idle_w_known × hours`` is the compute/sensor idle from the BoM (9.96 W
    on the light build). Rolling drive is the motor formula at the run's
    average speed. Yaw is ``fitted_turn_wh`` (fitted-from-twin; zero when
    the path does not turn). Acceleration is the caller's summed motor-line
    bound (zero when omitted). Unknown idle watts return None.

    The design-agent shift table stays rolling-only. Pass a heading only
    for a logged run that actually yawed.
    """
    from src.hardware_profile import get_active_profile

    prof = profile if profile is not None else get_active_profile()
    totals = prof.get("bom_totals") or {}
    if totals.get("idle_w_known") is None:
        return None
    idle_w = float(totals["idle_w_known"])
    duration = max(0.0, float(duration_s))
    distance = max(0.0, float(dist_m))
    idle_wh = idle_w * duration / 3600.0
    speed = distance / duration if duration > 0.05 and distance > 0 else 0.44
    if distance <= 0 or speed <= 0:
        roll_wh = 0.0
    else:
        terms = bom_drive_terms(prof, speed)
        per_10ft = float(terms["drive_wh_per_10ft"]) + float(payload_kg) * float(
            terms["extra_wh_per_10ft_per_kg_payload"]
        )
        roll_wh = (distance / TEN_FT_M) * per_10ft
    turn_wh = fitted_turn_wh(prof, heading_change_deg, speed, payload_kg)
    accel_wh = max(0.0, float(wh_accel))
    drive_wh = roll_wh + turn_wh + accel_wh
    return {
        "wh_model": round(idle_wh + drive_wh, 3),
        "wh_model_idle": round(idle_wh, 3),
        "wh_model_drive": round(drive_wh, 3),
        "wh_model_roll": round(roll_wh, 3),
        "wh_model_turn": round(turn_wh, 3),
        "wh_model_accel": round(accel_wh, 3),
        "idle_w": idle_w,
        "turn_source": TURN_SOURCE,
    }


def format_mission_energy_line(
    dist_m: float,
    payload_kg: float | None = None,
    wh_measured: float | None = None,
    duration_s: float | None = None,
    heading_change_deg: float = 0.0,
    wh_accel: float = 0.0,
) -> str:
    """``MISSION_ENERGY build=...`` line. Model and measured stay separate.

    The model ignores ``data/twin_energy.json`` so a measured Webots Wh can
    sit next to it as ``wh_measured`` / ``err_pct``. When that file is
    present and the caller did not pass ``wh_measured``, it is the measured
    value.

    With ``duration_s`` and a BoM build, the model is idle over the whole
    run plus drive for ``dist_m``, not drive-only and not idle only while
    the wheels are turning.
    """
    from src.config import config
    from src.hardware_profile import active_bom_build

    if payload_kg is None:
        payload_kg = float(config.get("robot", "payload_kg") or 0.0)
    run = None
    if duration_s is not None and active_bom_build():
        run = bom_run_energy_wh(
            dist_m,
            duration_s,
            float(payload_kg),
            heading_change_deg=heading_change_deg,
            wh_accel=wh_accel,
        )
    if run is not None:
        wh_model = run["wh_model"]
    else:
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
