"""BoM and sim drive bill, hinge damping, and turn scrub."""

from __future__ import annotations

import math

from src.twin_energy_cruise import (
    DEFAULT_CRR,
    DEFAULT_DRIVER_EFF,
    GRAVITY_M_S2,
    SIM_HINGE_DAMPING_NM_S_RAD,
    SIM_WHEEL_RADIUS_M,
    TEN_FT_M,
    TURN_SOURCE,
    WORLD_DRIVE_COULOMB_FRICTION,
    WORLD_DRIVE_FDS_M_S_PER_N,
)
from src.twin_energy_motor import brushed_drive_point, electrical_from_shaft_w

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
    electrical_omega_rad_s: float | None = None,
) -> dict:
    """Drive Wh at ``speed_m_s`` for a BoM profile, from the motor formula.

    ``extra_torque_nm`` defaults to 0. The shift table and the 3.43 W
    light-build figure use that default, with electrical speed
    ``v / 0.04``. The sim hinge term passes the published damping torque
    and the 0.08 m hinge rate.
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
        electrical_omega_rad_s=electrical_omega_rad_s,
    )


def sim_hinge_torque_nm(speed_m_s: float) -> float:
    """One drive hinge's viscous torque at body speed, from the world file.

    Webots ``JointParameters.dampingConstant`` for a rotational motor is
    in N·m·s/rad. The joint torque is ``τ = −c · ω``, applied in addition
    to the motor torque. ``c`` is 0.15 and the drawn wheel is 0.08 m, so
    the magnitude is ``0.15 · v / 0.08``. Tire contact is not in this
    number: the world file does not publish one torque for it.

    ``getTorqueFeedback`` is the motor's own effort. The motor page says
    it does not add spring or damping force on top of that effort. The
    velocity controller still has to produce torque against the damper,
    so the reading contains that load once. The measured column is not
    increased by this torque a second time.
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
    """Rolling resistance plus published hinge damping.

    Rolling torque still uses the BoM radius (0.04 m). Damping torque is
    ``c · ω`` on the drawn hinge, ``ω = v / 0.08``. Electrical speed for
    this combined point is that same hinge rate. ``bom_drive_terms`` with
    no extra torque stays the design bill at ``v / 0.04``, so the 3.43 W
    figure does not move. ``wh_visc`` is this point minus that bill.

    Run #12 billed the damper at ``v / 0.08`` and then ran the motor line
    at ``v / 0.04``. That is two different shaft speeds for one hinge.
    The hinge and the torque feedback turn at ``v / 0.08``.
    """
    speed = max(0.0, float(speed_m_s))
    omega = speed / SIM_WHEEL_RADIUS_M if SIM_WHEEL_RADIUS_M > 0.0 else 0.0
    return bom_drive_terms(
        profile,
        speed_m_s,
        mass_kg=mass_kg,
        extra_force_n=extra_force_n,
        extra_torque_nm=sim_hinge_torque_nm(speed),
        electrical_omega_rad_s=omega,
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
    The billed turn term is ``turning_scrub_wh``: track geometry and the
    world's force-dependent slip, through the motor curve. Not this arc,
    and not a coefficient fitted to a log.
    """
    theta = math.radians(abs(float(heading_change_deg)))
    track = chassis_track_width_m() if track_width_m is None else float(track_width_m)
    if theta <= 0 or track <= 0 or float(eta_driver) <= 0 or float(mass_kg) <= 0:
        return 0.0
    work_j = float(crr) * float(mass_kg) * GRAVITY_M_S2 * track * theta
    return (work_j / float(eta_driver)) / 3600.0


def _profile_motor_args(profile: dict) -> dict:
    spec = (profile.get("motors") or {}).get("left_wheel") or {}
    assumptions = profile.get("assumptions") or {}
    geom = profile.get("geometry") or {}
    return {
        "voltage_v": float(spec.get("voltage_v") or 0.0),
        "no_load_rpm": float(spec.get("no_load_rpm") or 0.0),
        "no_load_current_a": float(spec.get("no_load_current_a") or 0.0),
        "stall_current_a": float(spec.get("stall_current_a") or 0.0),
        "stall_torque_nm": float(spec.get("stall_torque_nm") or 0.0),
        "eta_driver": float(assumptions.get("driver_eff") or DEFAULT_DRIVER_EFF),
        "crr": float(assumptions.get("crr") or DEFAULT_CRR),
        "wheel_radius_m": float(geom.get("wheel_radius_m") or 0.04),
        "mass_kg": float(geom.get("mass_kg_known") or 0.0),
    }


def _hub_electrical_w(args: dict, mass_kg: float, speed_m_s: float, tau_scrub_nm: float) -> tuple[float, bool]:
    """One hub's pack watts at body speed, plus an optional scrub torque.

    Electrical speed is body speed / 0.08 m. That is the contact hinge
    the measured pack watts use, and the same rate as ``sim_drive_terms``.
    Rolling torque still uses the catalog BoM radius, so the 3.43 W
    design bill does not move. ``turning_scrub_point`` bills only the
    difference between a yawed pair and this straight pair.

    Billing that difference at ``v / catalog radius`` doubled the light
    shaft speed (0.04 m vs 0.08 m) and over-billed light corners. Heavy's
    catalog radius is 0.10 m, close to the hinge, so the same mistake
    stayed small. A hub at 0 m/s bills 0 W. Scrub torque is clamped at stall.
    """
    speed = float(speed_m_s)
    if speed <= 1e-9:
        return 0.0, False
    radius = max(float(args["wheel_radius_m"]), 1e-6)
    hinge = SIM_WHEEL_RADIUS_M if SIM_WHEEL_RADIUS_M > 0.0 else radius
    tau = (
        float(args["crr"]) * float(mass_kg) * GRAVITY_M_S2 * radius / 2.0
        + SIM_HINGE_DAMPING_NM_S_RAD * speed / hinge
        + max(0.0, float(tau_scrub_nm))
    )
    point = electrical_from_shaft_w(
        tau,
        speed / hinge,
        voltage_v=args["voltage_v"],
        no_load_rpm=args["no_load_rpm"],
        no_load_current_a=args["no_load_current_a"],
        stall_current_a=args["stall_current_a"],
        stall_torque_nm=args["stall_torque_nm"],
        eta_driver=args["eta_driver"],
    )
    if point["elec_w"] is None:
        return 0.0, False
    return float(point["elec_w"]), bool(point["clamped"])


def turning_scrub_point(
    profile: dict,
    speed_m_s: float,
    yaw_rate_rad_s: float,
    payload_kg: float = 0.0,
) -> dict:
    """Extra pack watts of a yaw, from track geometry and the world contact.

    Each drive hub is ``track/2`` from the chassis centre (0.34 m in
    ``butlerbot_wheeled.yaml``, anchors ±0.17 m in the world). At yaw
    rate ω the hub's ground-speed offset from the centre is
    ``(track/2)·|ω|``. The world's ODE force-dependent slip turns a
    tangential slip speed into a force, ``v = k · F`` with
    ``k = 0.01``, capped by ``μ · N`` with ``μ = 8``.

    ``N`` is half the known BoM mass plus this run's payload.
    ASSUMPTION: the two drive wheels carry the robot. Caster bottoms
    sit about 4 mm above the tire (world comment), so they are not in
    the normal load. ``μ = 8`` is only the cap. At a warehouse corner
    the force is ``v/k``, well below ``μ·N``. Fully mobilizing μ = 8
    as a skid would clamp the motor at stall and is not billed.
    ``frictionRotation`` is 0, so there is no bore torque.

    The force is an axle torque ``F · R`` on the drawn 0.08 m wheel.
    Both hubs get it. Each hub is also evaluated at its own speed
    ``v ± (track/2)·|ω|`` so the viscous hinge torque follows the
    faster outer wheel. Pack watts are ``electrical_from_shaft_w``.
    Driver efficiency 0.92 is inside that call, once. The straight
    bill (both hubs at ``v``, scrub torque 0) is subtracted, so rolling
    and damping are not billed twice.

    A differential drive that matches each hub to that offset is
    rolling, and the residual slip is only the tire width plus tracking
    error. Using the full offset is the contact law's force when that
    offset is slip. It is an upper bound when the controller cancels
    it. The number is not adjusted to a log.
    """
    args = _profile_motor_args(profile)
    mass = args["mass_kg"] + max(0.0, float(payload_kg))
    speed = max(0.0, float(speed_m_s))
    yaw = abs(float(yaw_rate_rad_s))
    empty = {
        "delta_w": 0.0,
        "straight_w": 0.0,
        "turned_w": 0.0,
        "scrub_force_n": 0.0,
        "scrub_torque_nm": 0.0,
        "clamped": False,
        "turn_source": "none",
    }
    if mass <= 0 or speed <= 0 or yaw <= 0:
        if mass > 0 and speed > 0:
            straight = 2.0 * _hub_electrical_w(args, mass, speed, 0.0)[0]
            empty["straight_w"] = straight
            empty["turned_w"] = straight
        return empty
    track = chassis_track_width_m()
    if track <= 0 or WORLD_DRIVE_FDS_M_S_PER_N <= 0:
        return empty
    v_slip = (track / 2.0) * yaw
    normal = mass * GRAVITY_M_S2 / 2.0
    force = min(
        WORLD_DRIVE_COULOMB_FRICTION * normal,
        v_slip / WORLD_DRIVE_FDS_M_S_PER_N,
    )
    tau_scrub = force * SIM_WHEEL_RADIUS_M
    half = v_slip
    inner_v = max(0.0, speed - half)
    outer_v = speed + half
    straight = 2.0 * _hub_electrical_w(args, mass, speed, 0.0)[0]
    inner_w, inner_clamp = _hub_electrical_w(args, mass, inner_v, tau_scrub)
    outer_w, outer_clamp = _hub_electrical_w(args, mass, outer_v, tau_scrub)
    turned = inner_w + outer_w
    return {
        "delta_w": max(0.0, turned - straight),
        "straight_w": straight,
        "turned_w": turned,
        "scrub_force_n": force,
        "scrub_torque_nm": tau_scrub,
        "clamped": inner_clamp or outer_clamp,
        "turn_source": TURN_SOURCE,
    }


def turning_scrub_wh(
    profile: dict,
    speed_m_s: float,
    heading_change_deg: float,
    dt_s: float,
    payload_kg: float = 0.0,
) -> float:
    """Electrical watt-hours of one yaw step. Zero when the step does not yaw."""
    dt = float(dt_s)
    if dt <= 0:
        return 0.0
    yaw = math.radians(abs(float(heading_change_deg)))
    if yaw <= 0:
        return 0.0
    point = turning_scrub_point(profile, speed_m_s, yaw / dt, payload_kg)
    return float(point["delta_w"]) * dt / 3600.0


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
