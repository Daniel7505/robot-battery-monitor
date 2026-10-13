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
# Drive-wheel hinge in every butlerbot*.wbt. The mesh stays 0.08 m and
# lane_keep.WHEEL_RADIUS_M stays 0.08. These are world-file constants,
# not a fit to a logged run.
SIM_WHEEL_RADIUS_M = 0.08
SIM_HINGE_DAMPING_NM_S_RAD = 0.15

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
    extra_torque_nm: float = 0.0,
) -> dict:
    """Linear brushed-DC point from bom.json ``formulas``.

    tau = (Crr * m * g + extra_force) * r / 2 + extra_torque
    duty = omega/omega0 + tau/tau_stall
    I = I0 + (I_stall - I0) * tau/tau_stall
    P_drive = 2 * duty * V * I / eta_driver

    ``extra_force_n`` is an added longitudinal force (Newtons, both wheels
    together), used for the acceleration term. ``extra_torque_nm`` is an
    added shaft torque on each motor. Both default to zero, which is the
    rolling-only point (light build ~3.43 W and ~0.0066 Wh per 10 ft at
    0.44 m/s). Torque is clamped to stall so a hard accel does not invent
    a current past the datasheet.
    """
    radius = max(float(wheel_radius_m), 1e-6)
    speed = float(speed_m_s)
    motors = max(1, int(n_motors))
    force_n = float(crr) * float(mass_kg) * GRAVITY_M_S2 + max(0.0, float(extra_force_n))
    added = max(0.0, float(extra_torque_nm))
    tau = force_n * radius / float(motors) + added
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
            / float(motors)
            + added,
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


# Pololu #4693, the light-build drive motor. bom.json copies these from
# https://www.pololu.com/product/4693/specs and the 37D gearmotor datasheet
# https://www.pololu.com/file/0J1736/pololu-37d-metal-gearmotors-rev-1-2.pdf
# Kt and R are not fields on that page. They are derived below.
LIGHT_MOTOR_PART_ID = "mot_pololu_4693"
# Same stall guard as the mission energy log: a paused Webots gap is not a drain.
PACK_DRAW_MAX_DT_S = 15.0


def published_motor_circuit(
    spec: dict,
    *,
    eta_driver: float = DEFAULT_DRIVER_EFF,
) -> dict:
    """Constants of one BoM gearmotor, and the gaps in that circuit.

    Published, from the part (light build is Pololu 4693):

    * ``voltage_v``, ``no_load_rpm``, ``no_load_current_a``
    * ``stall_current_a``, ``stall_torque_nm``
    * ``max_efficiency`` — datasheet peak, not an operating-point η
    * ``gear_ratio`` — already inside the output-shaft no-load and stall
      figures. It is not applied a second time.

    Derived, because bom.json does not publish Kt or R as their own fields:

    * ``Kt = stall_torque_nm / stall_current_a`` (N·m/A)
    * ``Kt_line = stall_torque_nm / (stall_current_a − no_load_current_a)``
      which is the slope of the published current line
      ``I = I0 + (Istall − I0) · τ/τstall``
    * ``R = voltage_v / stall_current_a`` (ohm), the stall approximation

    Gap: those derived Kt and R are not one circuit with the published
    no-load speed. ``(V − I0·R) / Kt`` is about 295 rpm on the 4693.
    The datasheet no-load speed is 200 rpm. The operating-point conversion
    therefore uses the published interpolation (the same duty and current
    line as ``brushed_drive_point``), which hits both published endpoints,
    and does not use the inconsistent Kt/R circuit to invent a current.

    ``eta_driver`` is ``assumptions.driver_eff`` (0.92). bom.json marks
    that as an assumption: G2/VNH5019 efficiency is not published.
    """
    voltage = float(spec.get("voltage_v") or 0.0)
    i0 = float(spec.get("no_load_current_a") or 0.0)
    i_stall = float(spec.get("stall_current_a") or 0.0)
    tau_stall = float(spec.get("stall_torque_nm") or 0.0)
    rpm = float(spec.get("no_load_rpm") or 0.0)
    omega0 = rpm * 2.0 * math.pi / 60.0
    kt = (tau_stall / i_stall) if i_stall > 0 else None
    kt_line = (tau_stall / (i_stall - i0)) if (i_stall - i0) > 0 else None
    resistance = (voltage / i_stall) if i_stall > 0 else None
    omega_from_kt_r = None
    if kt and resistance is not None and kt > 0:
        omega_from_kt_r = (voltage - i0 * resistance) / kt
    peak = spec.get("max_efficiency")
    if peak is None:
        # The profile loader stores the datasheet peak on ``efficiency``.
        peak = spec.get("efficiency")
    return {
        "part_id": spec.get("part_number") or LIGHT_MOTOR_PART_ID,
        "voltage_v": voltage,
        "no_load_rpm": rpm,
        "omega0_rad_s": omega0,
        "no_load_current_a": i0,
        "stall_current_a": i_stall,
        "stall_torque_nm": tau_stall,
        "kt_nm_per_a": kt,
        "kt_line_nm_per_a": kt_line,
        "r_ohm": resistance,
        "omega_from_kt_r_rad_s": omega_from_kt_r,
        "eta_driver": float(eta_driver),
        "max_efficiency_datasheet": None if peak is None else float(peak),
        "gear_ratio": spec.get("gear_ratio"),
        "gaps": (
            "Kt and R are derived from stall torque, stall current, and "
            "nominal voltage. bom.json does not publish them as fields.",
            "Kt = τstall/Istall and R = V/Istall predict a no-load speed "
            "higher than no_load_rpm, because no-load current drop and the "
            "published no-load speed are not the same circuit. The "
            "operating point uses the published duty/current line instead.",
            "max_efficiency is the datasheet peak. It is not the efficiency "
            "at the measured torque and speed.",
            "gear_ratio is already inside the output-shaft no-load and stall "
            "numbers. It is not applied again. The Webots hinge is treated "
            "as that output shaft.",
            "The sim wheel radius is 0.08 m and the BoM wheel is 0.04 m. "
            "Measured electrical speed is the hinge rad/s. The model line "
            "uses body speed / 0.04 m, which is twice the hinge rate at the "
            "same body speed. Measured watts are not doubled to hide that.",
            "driver efficiency is the 0.92 assumption in bom.json, applied "
            "once, the same way brushed_drive_point applies it. G2/VNH5019 "
            "efficiency is not published.",
            "There is no regen. Torque past stall is clamped to stall so "
            "current is not invented past the datasheet. Contact torque is "
            "already inside the measured hinge torque and is not added again.",
        ),
    }


def electrical_from_shaft_w(
    tau_nm: float,
    omega_rad_s: float,
    *,
    voltage_v: float,
    no_load_rpm: float,
    no_load_current_a: float,
    stall_current_a: float,
    stall_torque_nm: float,
    eta_driver: float = DEFAULT_DRIVER_EFF,
) -> dict:
    """Pack-side watts of one motor at a measured shaft point.

    ``tau_nm`` and ``omega_rad_s`` are the hinge reading. Rolling
    resistance is not added on top: the hinge torque already includes
    whatever load the sim applied. ``omega`` is not converted through the
    0.04 m BoM radius.

    The electrical line is the published interpolation, same as
    ``brushed_drive_point``:

    duty = |ω| / ω0 + τ_elec / τstall
    I = I0 + (Istall − I0) · τ_elec / τstall
    P = duty · V · I / eta_driver

    ``eta_driver`` is applied once. Point efficiency is shaft watts
    (|τ|·|ω|) divided by that pack-side P. The datasheet peak efficiency
    is not substituted. Torque above stall is clamped; the returned
    ``clamped`` flag marks that gap. A stopped, unloaded motor is 0 W.
    """
    tau = abs(float(tau_nm))
    omega = abs(float(omega_rad_s))
    omega0 = float(no_load_rpm) * 2.0 * math.pi / 60.0
    tau_stall = float(stall_torque_nm)
    driver = float(eta_driver)
    shaft_w = tau * omega
    if omega0 <= 0 or tau_stall <= 0 or driver <= 0:
        return {
            "shaft_w": shaft_w,
            "elec_w": None,
            "eta": None,
            "duty": None,
            "current_a": None,
            "clamped": False,
        }
    clamped = tau > tau_stall
    tau_elec = min(tau, tau_stall)
    if omega <= 0.0 and tau_elec <= 0.0:
        return {
            "shaft_w": 0.0,
            "elec_w": 0.0,
            "eta": None,
            "duty": 0.0,
            "current_a": float(no_load_current_a),
            "clamped": False,
        }
    duty = omega / omega0 + tau_elec / tau_stall
    current = float(no_load_current_a) + (
        float(stall_current_a) - float(no_load_current_a)
    ) * (tau_elec / tau_stall)
    elec_w = duty * float(voltage_v) * current / driver
    eta = (shaft_w / elec_w) if elec_w > 1e-12 else None
    return {
        "shaft_w": shaft_w,
        "elec_w": elec_w,
        "eta": eta,
        "duty": duty,
        "current_a": current,
        "clamped": clamped,
    }


def electrical_pack_draw_w(
    channel_draws: dict | None,
    measured_drive_elec_w: float | None,
) -> float | None:
    """Pack watts that SoC and ``wh_measured`` both integrate.

    Non-locomotion channels stay (published electronics on a BoM build).
    The Legs channel is the rolling prediction, so it is replaced by the
    measured electrical drive. ``None`` when that measurement is missing:
    the rolling line is not written in its place.
    """
    if measured_drive_elec_w is None:
        return None
    idle = 0.0
    for name, watts in (channel_draws or {}).items():
        if name == "Legs" or watts is None:
            continue
        idle += float(watts)
    return idle + float(measured_drive_elec_w)


def pack_drain_dt_s(
    wall_gap_s: float | None,
    *,
    sim_dt_s: float,
    max_dt_s: float = PACK_DRAW_MAX_DT_S,
) -> float:
    """Seconds to bill so SoC matches the energy log's clock.

    The log integrates the gap between telemetry timestamps and drops a
    gap longer than 15 s. The controller used to multiply watts by the
    simulation timestep instead, so a warehouse step slower than realtime
    moved the percent less than the logged watt-hours. The first step has
    no wall gap yet and uses the simulation step. A non-positive gap or a
    gap past ``max_dt_s`` bills nothing.
    """
    if wall_gap_s is None:
        dt = float(sim_dt_s)
    else:
        dt = float(wall_gap_s)
    if dt <= 0.0 or dt > float(max_dt_s):
        return 0.0
    return dt


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
    extra_torque_nm: float = 0.0,
) -> dict:
    """Drive Wh at ``speed_m_s`` for a BoM profile, from the motor formula.

    ``extra_torque_nm`` defaults to 0. The shift table and the 3.43 W
    light-build figure use that default. The sim hinge term passes the
    published damping torque instead.
    """
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
        extra_torque_nm=extra_torque_nm,
    )


def sim_hinge_torque_nm(speed_m_s: float) -> float:
    """One drive hinge's viscous torque at body speed, from the world file.

    ``dampingConstant`` is 0.15 N·m·s/rad on the 0.08 m wheel, so
    τ = 0.15 · v / 0.08. Tire contact (friction, force-dependent slip)
    is not in this number: the world file does not publish one torque
    for it.
    """
    speed = max(0.0, float(speed_m_s))
    if speed <= 0.0 or SIM_WHEEL_RADIUS_M <= 0.0:
        return 0.0
    return SIM_HINGE_DAMPING_NM_S_RAD * speed / SIM_WHEEL_RADIUS_M


def sim_drive_terms(
    profile: dict,
    speed_m_s: float,
    mass_kg: float | None = None,
    extra_force_n: float = 0.0,
) -> dict:
    """Rolling resistance plus published hinge damping, same motor line.

    This is the sim drive bill. ``bom_drive_terms`` with no extra torque
    stays the design bill. The difference is ``wh_visc``.
    """
    return bom_drive_terms(
        profile,
        speed_m_s,
        mass_kg=mass_kg,
        extra_force_n=extra_force_n,
        extra_torque_nm=sim_hinge_torque_nm(speed_m_s),
    )


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

    This is not billed. A 61.6° yaw on the light chassis is about 0.0001 Wh.
    Live segments did not correlate locomotion watt-hours with heading
    (warehouse correlation −0.06 across 52 segments). The old fitted yaw
    term was that gap misread as scrub.
    """
    theta = math.radians(abs(float(heading_change_deg)))
    track = chassis_track_width_m() if track_width_m is None else float(track_width_m)
    if theta <= 0 or track <= 0 or float(eta_driver) <= 0 or float(mass_kg) <= 0:
        return 0.0
    work_j = float(crr) * float(mass_kg) * GRAVITY_M_S2 * track * theta
    return (work_j / float(eta_driver)) / 3600.0


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
    a few thousandths of a watt-hour.
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


def _per_10ft(terms: dict, payload_kg: float) -> float:
    return float(terms["drive_wh_per_10ft"]) + float(payload_kg) * float(
        terms["extra_wh_per_10ft_per_kg_payload"]
    )


def segment_model_terms(
    dist_m: float,
    avg_speed_m_s: float,
    heading_change_deg: float,
    wh_accel: float,
    payload_kg: float = 0.0,
    profile: dict | None = None,
) -> dict:
    """Model split for one segment.

    ``wh_roll`` is the rolling-only motor line. ``wh_visc`` is the extra
    electrical energy from the published hinge damping on that same line
    (the cross term included, so roll + visc is the combined point).
    ``heading_change_deg`` is accepted so callers can keep passing it.
    It is not billed. Warehouse locomotion did not track heading.
    """
    del heading_change_deg
    from src.hardware_profile import get_active_profile

    prof = profile if profile is not None else get_active_profile()
    dist = max(0.0, float(dist_m))
    speed = float(avg_speed_m_s)
    roll = 0.0
    visc = 0.0
    if dist > 0 and speed > 0:
        roll = (dist / TEN_FT_M) * _per_10ft(bom_drive_terms(prof, speed), payload_kg)
        sim = (dist / TEN_FT_M) * _per_10ft(sim_drive_terms(prof, speed), payload_kg)
        visc = max(0.0, sim - roll)
    return {
        "wh_roll": roll,
        "wh_visc": visc,
        "wh_turn": 0.0,
        "wh_accel": max(0.0, float(wh_accel)),
        "turn_source": "none",
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
    rolling-only. A logged run adds published hinge damping and the
    acceleration bound in ``bom_run_energy_wh``, not here.
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
    """Idle for the whole run, plus rolling, published hinge damping, and accel.

    ``idle_w_known × hours`` is the compute/sensor idle from the BoM (9.96 W
    on the light build). Drive is the brushed-DC line at the run's average
    speed. ``wh_model_roll`` is rolling resistance only. ``wh_model_visc``
    is the extra from ``dampingConstant 0.15`` on the 0.08 m sim wheel.
    Yaw is not a term. ``heading_change_deg`` is ignored. Acceleration is
    the caller's summed motor-line bound (zero when omitted). Unknown idle
    watts return None.

    The design-agent shift table stays rolling-only. A fitted
    watt-hour-versus-speed line from a warehouse log is not a coefficient
    here.
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
    roll_wh = 0.0
    visc_wh = 0.0
    if distance > 0 and speed > 0:
        roll_wh = (distance / TEN_FT_M) * _per_10ft(
            bom_drive_terms(prof, speed), payload_kg
        )
        sim_wh = (distance / TEN_FT_M) * _per_10ft(
            sim_drive_terms(prof, speed), payload_kg
        )
        visc_wh = max(0.0, sim_wh - roll_wh)
    del heading_change_deg
    accel_wh = max(0.0, float(wh_accel))
    drive_wh = roll_wh + visc_wh + accel_wh
    return {
        "wh_model": round(idle_wh + drive_wh, 3),
        "wh_model_idle": round(idle_wh, 3),
        "wh_model_drive": round(drive_wh, 3),
        "wh_model_roll": round(roll_wh, 3),
        "wh_model_visc": round(visc_wh, 3),
        "wh_model_turn": 0.0,
        "wh_model_accel": round(accel_wh, 3),
        "idle_w": idle_w,
        "turn_source": "none",
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
