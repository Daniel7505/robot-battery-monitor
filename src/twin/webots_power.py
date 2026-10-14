"""
Webots power estimation — map motor/joint telemetry to PMS channel draws.

Architecture
------------
The Webots controller observes joint velocity (and optional torque) each step.
This module turns those samples into the five PMS channels the dashboard knows:

  joint ω, τ  →  estimate_motor_power_w  →  per-motor watts
              →  aggregate_channel_draws →  Legs / Arms / Torso / Compute
              →  build_webots_telemetry  →  POST /api/twin/telemetry payload

Ground truth for motor idle/cruise/peak, wheel radius, compute, and stabilizers
is the active hardware profile (``config/hardware_profiles/<id>.yaml``), not
hardcoded constants — constants below are only fallbacks if a block is missing.

Motion-aware design goals
-------------------------
  1. Idle teleop must not report full drive stress (motion_scale → 0 near rest).
  2. Legs should land mid-band at cruise, not peg channel max every frame.
  3. Prefer body speed when available (smoother than noisy hub encoders).
  4. Missing torque (common in Webots) synthesizes load from |ω| via τ_proxy.

Stress multipliers (gait/phase) are applied *lightly* and only when motion is
actually present, so a labeled "drive" phase with stationary wheels stays calm.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone

from src.config import config
from src.hardware_profile import (
    battery_capacity_wh,
    battery_c_rate_limits,
    battery_draw_c_rate,
    clamp_motor_power_w,
    get_active_profile,
    motor_driver_spec,
    motor_idle_and_scale,
    motor_part_meta,
    motor_spec,
    normalize_phase_name,
    wheel_mass_kg,
    wheel_radius_m,
)

# ButlerBot Webots motor name → PMS channel.
# "Legs" is historical channel id for locomotion (wheels on the wheeled profile).
WEBOTS_MOTOR_CHANNELS: dict[str, str] = {
    "left_wheel": "Legs",
    "right_wheel": "Legs",
    "leg_left": "Legs",
    "leg_right": "Legs",
    "leg_motors": "Legs",
    "caster": "Legs",
    "torso_joint": "Torso",
    "torso_motor": "Torso",
    "left_arm": "Arms",
    "right_arm": "Arms",
    "arm_motors": "Arms",
    "gripper": "Arms",
}

# Fallbacks only if profile missing compute block.
COMPUTE_IDLE_W = 6.5
COMPUTE_ACTIVE_W = 10.0

# Gait-level stress when phase is unknown; phase table wins when present.
_GAIT_STRESS: dict[str, float] = {
    "stand": 1.0,
    "idle": 1.0,
    "standby": 1.0,
    "drive": 1.35,
    "turn": 1.4,
    "transit": 1.35,
    "walk": 1.35,
    "patrol": 1.2,
    "manipulate": 1.5,
    "high_load": 1.5,
}

_PHASE_STRESS: dict[str, float] = {
    "standby": 1.0,
    "drive_transit": 1.4,
    "walk_transit": 1.4,
    "teleop": 1.35,
    "teleop_turn": 1.4,
    "patrol": 1.2,
    "manipulate": 1.55,
    "return_idle": 1.05,
}

_GAIT_TO_TASK = {
    "stand": "idle",
    "idle": "idle",
    "standby": "idle",
    "drive": "moving",
    "turn": "moving",
    "transit": "moving",
    "walk": "moving",
    "patrol": "balanced",
    "balanced": "balanced",
    "manipulate": "high_load",
    "grasp": "high_load",
    "high_load": "high_load",
}


def _resolve_profile(profile: dict | None) -> dict:
    return profile if profile is not None else get_active_profile()


def estimate_motor_power_w(
    velocity: float,
    torque: float,
    *,
    motor_name: str = "",
    motor_idle_w: float | None = None,
    scale: float | None = None,
    profile: dict | None = None,
    speed_m_s: float | None = None,
) -> float:
    """Estimate electrical draw from joint velocity (rad/s) and torque (Nm).

    Wheel motors use a profile-grounded cruise curve:
    - Partial load (frac < 1): slightly superlinear rise toward cruise_w
    - Over-cruise: soft headroom (~25% of cruise), not a hard jump toward peak
    - Real torque (when present): blend in P ≈ idle + (τ·ω)/η
    Goal: mid-band at cruise, avoid always pegging the Legs channel max.

    Non-wheel joints fall back to mechanical power × scale or efficiency.
    Final value is clamped by profile motor peak via ``clamp_motor_power_w``.
    """
    prof = _resolve_profile(profile)
    name = str(motor_name or "")
    idle_w, motor_scale, tau_proxy = motor_idle_and_scale(name or "joint", prof)
    if motor_idle_w is not None:
        idle_w = float(motor_idle_w)
    if scale is not None:
        motor_scale = float(scale)

    vel = abs(float(velocity))
    tau = abs(float(torque))
    # Missing torque feedback (common in Webots) → synthesize load from |ω|
    if tau < 1e-4 and vel > 0.05:
        tau = vel * tau_proxy

    spec = motor_spec(prof, name) if name else {}
    is_wheel = "wheel" in name.lower()
    eff = float(spec.get("efficiency") or 0.0)
    # Real-part electrical limits from datasheet (optional fields)
    free_run_w = float(spec.get("free_run_w") or 0.0)
    stall_elec_w = 0.0
    if spec.get("stall_current_a") is not None and spec.get("voltage_v") is not None:
        stall_elec_w = float(spec["stall_current_a"]) * float(spec["voltage_v"])
    elif spec.get("peak_power_w") is not None:
        stall_elec_w = float(spec["peak_power_w"])

    # Near-zero hub rate (and no body-speed override) → pure idle. Avoids
    # counting position-hold / tiny encoder noise as cruise power.
    if is_wheel and vel <= 0.05 and (speed_m_s is None or float(speed_m_s) <= 0.04):
        return clamp_motor_power_w(name, idle_w, prof) if name else round(idle_w, 2)

    # BoM builds use the published no-load / stall line (duty·V·I).
    # The YAML profile keeps cruise_w, so it stays on the curve below.
    if is_wheel and _use_bom_linear(prof, spec):
        raw = _linear_wheel_electrical_w(
            spec, prof, velocity=vel, torque=float(torque), speed_m_s=speed_m_s
        )
        if name:
            return clamp_motor_power_w(name, raw, prof)
        return round(raw, 2)

    # Profile cruise curve for wheels: smooth |ω| (or body speed) → watts
    cruise_w = spec.get("cruise_w")
    if is_wheel and cruise_w is not None and vel > 0.02:
        radius = wheel_radius_m(prof)
        v_cruise = float(spec.get("cruise_speed_m_s") or 0.40)
        omega_cruise = max(v_cruise / max(radius, 1e-4), 0.5)
        # Prefer body speed when provided (smoother than noisy encoder)
        if speed_m_s is not None and float(speed_m_s) > 0.02:
            omega_equiv = float(speed_m_s) / max(radius, 1e-4)
            frac = max(0.0, omega_equiv / omega_cruise)
        else:
            frac = max(0.0, vel / omega_cruise)
        # Cap load fraction so full teleop does not explode past soft headroom
        frac = min(1.25, frac)
        cruise = float(cruise_w)
        # Floor partial-spin draw near free-run when datasheet provides it
        base = max(idle_w, free_run_w * min(1.0, frac) if free_run_w > 0 else idle_w)
        if frac <= 1.0:
            # Partial load: slightly superlinear (iron/copper grow with speed)
            shape = frac ** 1.25
            raw = base + (cruise - base) * shape
        else:
            # Over-cruise: soft approach to ~1.25× cruise (not channel peak)
            over = min(1.0, (frac - 1.0) / 0.25)
            headroom = cruise * 0.25
            raw = cruise + headroom * (over ** 0.85)
        # Efficiency-aware blend when torque path is meaningful
        if abs(float(torque)) > 1e-3 and eff > 0.05:
            mech = abs(float(torque)) * vel
            elec_from_mech = idle_w + mech / eff
            # Weight real-torque path more as mechanical load rises
            blend = min(0.4, mech / max(cruise, 1.0) * 0.25)
            raw = raw * (1.0 - blend) + elec_from_mech * blend
        # Soft per-motor ceiling below peak clamp (prefers cont/curve_top)
        curve_top = float(
            spec.get("curve_top_w")
            or min(float(spec.get("cont_w") or cruise * 1.6), cruise * 1.45)
        )
        if stall_elec_w > 0:
            curve_top = min(curve_top, stall_elec_w * 0.45)  # stay off stall burn
        raw = min(raw, curve_top)
    else:
        mechanical = tau * vel
        if eff > 0.05 and mechanical > 0:
            raw = idle_w + mechanical / eff
        else:
            raw = idle_w + mechanical * motor_scale

    if name:
        return clamp_motor_power_w(name, raw, prof)
    return round(raw, 2)


def _use_bom_linear(profile: dict, spec: dict) -> bool:
    """True when this wheel should use the BoM brushed-DC line, not cruise_w^1.25."""
    if profile.get("power_model") == "bom_linear":
        needed = (
            "no_load_current_a",
            "stall_current_a",
            "stall_torque_nm",
            "no_load_rpm",
            "voltage_v",
        )
        return all(spec.get(key) is not None for key in needed)
    # A profile that published stall numbers and has no cruise curve.
    if spec.get("cruise_w") is not None:
        return False
    needed = (
        "no_load_current_a",
        "stall_current_a",
        "stall_torque_nm",
        "no_load_rpm",
        "voltage_v",
    )
    return all(spec.get(key) is not None for key in needed)


def reported_wheel_torque_nm(
    velocity: float,
    feedback_nm: float | None,
    *,
    bom: bool,
) -> float:
    """Torque the electrical BoM line is allowed to use as a load.

    A BoM build returns 0 so Legs stay on rolling resistance. Live
    ``getTorqueFeedback`` is not that load. The measured column does
    not use this helper: it reads ``torque_feedback_nm`` and integrates
    |τ|·|ω|. The YAML profile still keeps a real feedback reading, and
    with no reading it keeps the old ``|ω| × 0.45`` stand-in.
    """
    if bom:
        return 0.0
    if feedback_nm is not None and abs(float(feedback_nm)) > 1e-4:
        return abs(float(feedback_nm))
    return abs(float(velocity)) * 0.45


def wheel_torque_feedback_nm(feedback_nm) -> float | None:
    """Signed Webots ``getTorqueFeedback``, or None when it did not report.

    A numeric 0 is a real reading (parked hinge). None, blank, and
    non-finite values are unusable. Callers log ``na`` and do not
    substitute rolling resistance or ``|ω| × 0.45``.
    """
    if feedback_nm is None or feedback_nm == "":
        return None
    try:
        value = float(feedback_nm)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


def measured_drive_w(
    joints: list[dict] | None,
    speed_m_s: float | None = None,
) -> float | None:
    """Mechanical drive watts from the sim hinges: sum of |τ| · |ω|.

    τ is ``torque_feedback_nm`` only. The ``torque`` field is the load
    the electrical model used, and a BoM build stores 0 there, so it is
    not a measurement. ω is the wheel position-sensor rate. When that
    rate is missing and the body is moving, ω = speed / R. R is the
    drawn 0.08 m wheel until a BoM body loads, then the contact radius.

    Returns None when any drive wheel has no torque feedback. The energy
    log writes ``na`` in that case.
    """
    wheels = [
        joint
        for joint in (joints or [])
        if isinstance(joint, dict) and "wheel" in str(joint.get("name") or "").lower()
    ]
    if not wheels:
        return None
    total = 0.0
    for joint in wheels:
        if "torque_feedback_nm" not in joint:
            return None
        tau = wheel_torque_feedback_nm(joint.get("torque_feedback_nm"))
        if tau is None:
            return None
        omega = _hinge_rad_s(joint, speed_m_s)
        if omega is None:
            return None
        total += abs(tau) * abs(omega)
    return total


def measured_drive_split(
    joints: list[dict] | None,
    speed_m_s: float | None = None,
    profile: dict | None = None,
) -> dict | None:
    """Shaft watts and pack-side electrical watts for the drive hinges.

    Shaft is |τ|·|ω| on the 0.08 m wheel, same as ``measured_drive_w``.
    Electrical watts run each hinge's (τ, ω) through the BoM motor's
    published duty/current line (see ``electrical_from_shaft_w``). The
    hinge rad/s is not doubled to the 0.04 m BoM wheel.

    Returns None when any drive wheel has no torque feedback. When the
    profile has no published no-load/stall numbers, ``elec_w`` is None
    and the shaft total is still returned. Rolling resistance is not
    substituted for either number.
    """
    wheels = [
        joint
        for joint in (joints or [])
        if isinstance(joint, dict) and "wheel" in str(joint.get("name") or "").lower()
    ]
    if not wheels:
        return None
    points: list[tuple[float, float]] = []
    for joint in wheels:
        if "torque_feedback_nm" not in joint:
            return None
        tau = wheel_torque_feedback_nm(joint.get("torque_feedback_nm"))
        if tau is None:
            return None
        omega = _hinge_rad_s(joint, speed_m_s)
        if omega is None:
            return None
        points.append((tau, omega))
    shaft_w = sum(abs(tau) * abs(omega) for tau, omega in points)
    spec, eta_driver = _drive_motor_spec(profile)
    if spec is None:
        return {"shaft_w": shaft_w, "elec_w": None, "eta": None}
    from src.twin_energy import electrical_from_shaft_w

    elec_w = 0.0
    for tau, omega in points:
        point = electrical_from_shaft_w(
            tau,
            omega,
            voltage_v=float(spec["voltage_v"]),
            no_load_rpm=float(spec["no_load_rpm"]),
            no_load_current_a=float(spec["no_load_current_a"]),
            stall_current_a=float(spec["stall_current_a"]),
            stall_torque_nm=float(spec["stall_torque_nm"]),
            eta_driver=eta_driver,
        )
        if point["elec_w"] is None:
            return {"shaft_w": shaft_w, "elec_w": None, "eta": None}
        elec_w += float(point["elec_w"])
    eta = (shaft_w / elec_w) if elec_w > 1e-12 else None
    return {"shaft_w": shaft_w, "elec_w": elec_w, "eta": eta}


def _drive_motor_spec(profile: dict | None) -> tuple[dict | None, float]:
    """Light/heavy/budget wheel spec, or None when the line is not published."""
    from src.twin_energy import DEFAULT_DRIVER_EFF

    prof = profile
    if prof is None:
        try:
            prof = get_active_profile()
        except Exception:
            return None, DEFAULT_DRIVER_EFF
    spec = {}
    motors = (prof or {}).get("motors") or {}
    if isinstance(motors, dict):
        spec = dict(motors.get("left_wheel") or motors.get("right_wheel") or {})
    needed = (
        "voltage_v",
        "no_load_rpm",
        "no_load_current_a",
        "stall_current_a",
        "stall_torque_nm",
    )
    if not spec or any(spec.get(key) is None for key in needed):
        return None, DEFAULT_DRIVER_EFF
    driver = (prof or {}).get("motor_driver") or {}
    assumptions = (prof or {}).get("assumptions") or {}
    eta = driver.get("efficiency")
    if eta is None:
        eta = assumptions.get("driver_eff")
    if eta is None:
        eta = DEFAULT_DRIVER_EFF
    return spec, float(eta)


def _hinge_rad_s(joint: dict, speed_m_s: float | None) -> float | None:
    """Hinge rate in rad/s. A quiet encoder uses the command wheel radius."""
    omega = None
    if joint.get("velocity") is not None and joint.get("velocity") != "":
        try:
            omega = abs(float(joint.get("velocity")))
        except (TypeError, ValueError):
            omega = None
        if omega is not None and not math.isfinite(omega):
            omega = None
    speed = None
    if speed_m_s is not None and speed_m_s != "":
        try:
            speed = abs(float(speed_m_s))
        except (TypeError, ValueError):
            speed = None
        if speed is not None and not math.isfinite(speed):
            speed = None
    # Encoder quiet while the body is moving. Unset bom_build uses the drawn
    # 0.08 m wheel. After a BoM body is loaded, the command radius is the
    # contact radius (src.webots_body.install_command_radius).
    if speed is not None and speed > 0.02 and (omega is None or omega < 1e-3):
        return speed / _command_wheel_radius_m()
    return omega


def _command_wheel_radius_m() -> float:
    """Hinge radius for a body-speed fallback. 0.08 m until a BoM build loads."""
    try:
        from src.lane_keep import WHEEL_RADIUS_M as command_r

        radius = float(command_r)
        if radius > 0:
            return radius
    except Exception:
        pass
    return SIM_WHEEL_RADIUS_M


def _rolling_torque_nm(profile: dict, radius_m: float) -> float:
    """tau = Crr * m * g * r / 2. Mass is the known BoM mass plus payload."""
    from src.bom_profile import rolling_mass_kg

    assumptions = profile.get("assumptions") or {}
    crr = float(assumptions.get("crr") or 0.015)
    mass = rolling_mass_kg(profile)
    return crr * mass * 9.81 * float(radius_m) / 2.0


# Drawn wheel mesh in the shipped butlerbot*.wbt cylinders (Shape radius).
# The contact cylinder matches it until a BoM build is loaded. Do not resize
# the Shape. src.webots_body sets the contact cylinder and lane_keep's
# command radius from the BoM; this constant stays the drawn mesh.
SIM_WHEEL_RADIUS_M = 0.08


def _linear_wheel_electrical_w(
    spec: dict,
    profile: dict,
    *,
    velocity: float,
    torque: float,
    speed_m_s: float | None,
) -> float:
    """One motor's terminal power: duty * V * I, from the BoM motor line.

    Driver efficiency is applied later, once, in ``aggregate_channel_draws``,
    so this is not divided by eta here. Load torque is rolling resistance
    (Crr · m · g · r / 2 on the BoM radius). ``torque`` is ignored on this
    predicted line. The measured column is ``measured_drive_w`` (|τ|·|ω|
    from ``torque_feedback_nm``), not this function.

    Why the logged drive was ~16× this line
    ---------------------------------------
    Live locomotion was this same duty·V·I line with ω = body speed /
    0.04 m, and the torque was Webots ``getTorqueFeedback``: hinge motor
    effort, not the ~0.018 N·m rolling torque. Heading did not matter.
    A full warehouse run (121 segments, 369 m, 2145 s, stopped by hand
    because that world loops) measured mean 0.120 Wh per 10 ft
    (sd 0.023), correlation with heading −0.11 and with speed +0.82,
    14.5 Wh drive and 5.9 Wh idle. Idle matches 9.96 W. Rolling-only
    does not: it is ~0.8 Wh over that distance and it falls slightly as
    speed rises.

    The worlds set ``dampingConstant 0.15`` on a 0.08 m wheel, so
    viscous torque is ``0.15 · v / 0.08``. Put through the motor line
    at that hinge rate it rises with speed and is about 4.2 Wh together
    with rolling at that run's average speed, still short of 14.5 Wh.
    The remainder is tire contact (friction and
    force-dependent slip). That is not one published torque, and a
    straight-line fit of the log is not a bill. The mission energy
    model adds only the published damping, as ``wh_visc``. This
    function stays on rolling resistance so Legs at 0.44 m/s stay
    ~3.43 W. The 0.08 m mesh versus the 0.04 m BoM wheel doubles ω if
    a hub rate is used as the electrical rate, but it is not the 16×.
    The fitted yaw term treated the gap as scrub. It is removed.

    With no body speed, hub rad/s is the sim hinge. Electrical ω is that
    rate times 0.08 / 0.04, so a spinning sim wheel matches body speed
    on the real wheel.
    """
    del torque  # sim feedback is not the BoM load; see the docstring
    radius = wheel_radius_m(profile)
    radius = max(radius, 1e-4)
    if speed_m_s is not None and float(speed_m_s) > 0.02:
        omega = float(speed_m_s) / radius
    else:
        omega = abs(float(velocity)) * SIM_WHEEL_RADIUS_M / radius
    tau = _rolling_torque_nm(profile, radius)
    omega0 = float(spec["no_load_rpm"]) * 2.0 * math.pi / 60.0
    tau_stall = float(spec["stall_torque_nm"])
    if omega0 <= 0 or tau_stall <= 0:
        return float(spec.get("idle_w") or 0.0)
    duty = max(0.0, omega / omega0 + tau / tau_stall)
    i0 = float(spec["no_load_current_a"])
    i_stall = float(spec["stall_current_a"])
    current = i0 + (i_stall - i0) * (tau / tau_stall)
    return duty * float(spec["voltage_v"]) * current


def motor_powers_from_joints(
    joints: list[dict],
    profile: dict | None = None,
    *,
    speed_m_s: float = 0.0,
) -> dict[str, float]:
    """Build per-motor watt map from joint state samples.

    Always recomputes from v/τ so profile idle/cruise applies even if the
    controller stamped an older ``power_w`` — profile remains source of truth.
    """
    prof = _resolve_profile(profile)
    powers: dict[str, float] = {}
    for joint in joints:
        name = str(joint.get("name", "")).lower()
        if not name:
            continue
        # Recompute from v/τ so profile idle/cruise applies even if controller
        # stamped an older power_w (keeps profile as source of truth).
        powers[name] = estimate_motor_power_w(
            joint.get("velocity", 0.0),
            joint.get("torque", 0.0),
            motor_name=name,
            profile=prof,
            speed_m_s=speed_m_s,
        )
    return powers


def motion_scale(*, speed_m_s: float = 0.0, joints: list[dict] | None = None) -> float:
    """0–1 factor from measured motion — idle teleop should not use full drive stress.

    Normalized near profile cruise (~0.4 m/s, ~5 rad/s hub) so partial crawl
    does not already saturate the stress multiplier.
    """
    wheel_peak = 0.0
    if joints:
        for joint in joints:
            name = str(joint.get("name", "")).lower()
            if "wheel" not in name:
                continue
            wheel_peak = max(wheel_peak, abs(float(joint.get("velocity", 0.0))))
    wheel_factor = min(1.0, wheel_peak / 5.0)
    speed_factor = min(1.0, max(0.0, speed_m_s) / 0.40)
    # Near-idle floor stays low so stress barely applies when barely rolling
    if wheel_peak < 0.2 and speed_m_s < 0.04:
        return 0.0
    return max(0.08, max(wheel_factor, speed_factor))


def stress_multiplier(*, gait: str = "stand", phase: str = "") -> float:
    """Combined gait/phase stress factor for Webots twin telemetry.

    Phase table takes precedence when the phase name is known (scripted mission
    or teleop labels); otherwise gait-only stress is used.
    """
    g = str(gait).lower()
    p = normalize_phase_name(phase) if phase else ""
    if p and p in _PHASE_STRESS:
        return _PHASE_STRESS[p]
    if phase and str(phase).lower() in _PHASE_STRESS:
        return _PHASE_STRESS[str(phase).lower()]
    return _GAIT_STRESS.get(g, 1.0)


def _compute_draw_w(
    gait: str,
    profile: dict,
    *,
    speed_m_s: float = 0.0,
    vision: bool = False,
) -> float:
    """SBC + sensors (+ optional vision/agent modes) from profile compute blocks."""
    if profile.get("bom_build") or profile.get("power_model") == "bom_linear":
        from src.bom_profile import electronics_draw_w

        # Published idle (9.96 W on light), not the Jetson/OAK active mode.
        return electronics_draw_w(profile, moving=False)
    compute = profile.get("compute") or {}
    sensors = profile.get("sensors") or {}
    imu = profile.get("imu") or {}
    modes = profile.get("modes") or {}
    idle_w = float(compute.get("idle_w", COMPUTE_IDLE_W))
    active_w = float(compute.get("active_w", COMPUTE_ACTIVE_W))
    sens_idle = float(sensors.get("idle_w", 0.0)) + float(imu.get("idle_w", 0.0))
    sens_active = float(sensors.get("active_w", sensors.get("idle_w", 0.0))) + float(
        imu.get("active_w", imu.get("idle_w", 0.0))
    )
    moving = not (gait in ("stand", "idle", "standby") and speed_m_s < 0.06)
    if not moving:
        board = idle_w
        sens = sens_idle
    else:
        motion = min(1.0, max(0.0, speed_m_s) / 0.35)
        board = idle_w + (active_w - idle_w) * max(motion, 0.25)
        sens = sens_idle + (sens_active - sens_idle) * max(motion, 0.25)
    vision_w = float(modes.get("vision_active_w", 0.0)) if (vision or moving) else 0.0
    agent_w = float(modes.get("agent_active_w", 0.0))
    return round(board + sens + vision_w + agent_w, 2)


def _stabilizer_draw_w(profile: dict, *, speed_m_s: float = 0.0) -> tuple[str, float]:
    """Profile stabilizer idle→active ramp; channel is usually Torso."""
    stab = profile.get("stabilizers") or {}
    channel = str(stab.get("channel") or "Torso")
    idle_w = float(stab.get("idle_w", 0.0))
    active_w = float(stab.get("active_w", idle_w))
    v_full = float(stab.get("active_speed_m_s", 0.15) or 0.15)
    if speed_m_s <= 0.02:
        watts = idle_w
    else:
        frac = min(1.0, speed_m_s / max(v_full, 0.05))
        watts = idle_w + (active_w - idle_w) * frac
    return channel, round(watts, 2)


def aggregate_channel_draws(
    motor_power_w: dict[str, float],
    *,
    compute_w: float | None = None,
    gait: str = "stand",
    phase: str = "",
    speed_m_s: float = 0.0,
    joints: list[dict] | None = None,
    profile: dict | None = None,
) -> dict[str, float]:
    """Sum motor draws into Legs / Arms / Torso / Compute (+ stabilizers).

    Mild stress from gait/phase is motion-gated: without real wheel/body
    motion the stress multiplier does not inflate channels. Caps come from
    profile channel max_draw_w so twin posts never exceed the allocator map.
    """
    prof = _resolve_profile(profile)
    channels: dict[str, float] = {}
    channel_caps = prof.get("channels") or {}

    for motor, watts in motor_power_w.items():
        ch = WEBOTS_MOTOR_CHANNELS.get(motor.lower())
        if ch:
            channels[ch] = round(channels.get(ch, 0.0) + float(watts), 2)

    bom = bool(prof.get("bom_build") or prof.get("power_model") == "bom_linear")
    if not bom:
        stab_ch, stab_w = _stabilizer_draw_w(prof, speed_m_s=speed_m_s)
        if stab_w > 0:
            channels[stab_ch] = round(channels.get(stab_ch, 0.0) + stab_w, 2)

    if compute_w is None:
        compute_w = _compute_draw_w(gait, prof, speed_m_s=speed_m_s)
    channels["Compute"] = round(compute_w, 2)

    # Mild stress from gait/phase (profile cruise already sets base drive level).
    # Keep stress light so Legs land mid-band instead of hard-pegging channel max.
    mult = 1.0 if bom else stress_multiplier(gait=gait, phase=phase)
    if mult > 1.0 and (speed_m_s > 0.05 or motion_scale(speed_m_s=speed_m_s, joints=joints) > 0.2):
        motion = motion_scale(speed_m_s=speed_m_s, joints=joints)
        if motion > 0.0:
            # Only a fraction of (mult-1) is applied — full mult would peg Legs.
            effective = 1.0 + (mult - 1.0) * motion * 0.32
            for ch_id in list(channels.keys()):
                if ch_id == "Compute":
                    channels[ch_id] = round(channels[ch_id] * min(effective, 1.08), 2)
                elif ch_id == "Legs":
                    channels[ch_id] = round(channels[ch_id] * min(effective, 1.12), 2)
                else:
                    channels[ch_id] = round(channels[ch_id] * min(effective, 1.10), 2)

    # H-bridge losses: motor electrical → 12 V bus (driver η)
    driver = motor_driver_spec(prof)
    if driver and "Legs" in channels:
        eff_d = float(driver.get("efficiency") or 1.0)
        eff_d = max(0.5, min(1.0, eff_d))
        idle_d = float(driver.get("idle_w") or 0.0)
        legs_motors = float(channels["Legs"])
        legs_bus = legs_motors / eff_d + idle_d
        cont_bus = driver.get("continuous_bus_w")
        if cont_bus is not None:
            legs_bus = min(legs_bus, float(cont_bus))
        channels["Legs"] = round(legs_bus, 2)

    # DC-DC 48→12: pack-side drive power = 12 V bus / converter η + idle
    dcdc = prof.get("dc_dc_48_12") or {}
    if dcdc and "Legs" in channels:
        eff_c = float(dcdc.get("efficiency") or 1.0)
        eff_c = max(0.5, min(1.0, eff_c))
        idle_c = float(dcdc.get("idle_w") or 0.0)
        channels["Legs"] = round(float(channels["Legs"]) / eff_c + idle_c, 2)

    for ch_id, total in list(channels.items()):
        cap = channel_caps.get(ch_id, {})
        max_w = cap.get("max_draw_w")
        if max_w is not None:
            channels[ch_id] = round(min(total, float(max_w)), 2)

    if bom:
        # No arm, torso, or cooling hardware is sourced on a wheeled BoM build.
        channels["Arms"] = 0.0
        channels["Torso"] = 0.0
        channels["Cooling"] = 0.0

    return channels


def _locomotion_block(prof: dict, *, gait: str, phase: str, speed_m_s: float) -> dict:
    block = {
        "gait": gait,
        "speed_m_s": round(speed_m_s, 3),
        "phase": phase,
        "mode": "wheeled",
        "wheel_radius_m": wheel_radius_m(prof),
    }
    if prof.get("bom_build"):
        block["bom_build"] = prof.get("bom_build")
        block["bom_schema_version"] = prof.get("bom_schema_version")
        block["payload_kg"] = float(config.get("robot", "payload_kg") or 0.0)
    return block


def _subsystem_draws(profile: dict, channel_draws: dict[str, float]) -> dict:
    """Split channel watts into the log names. The numbers are the channel draws.

    On a BoM build, compute and sensors are the published idle watts that
    already make up the Compute channel. Anything left over after rounding
    stays on compute so the six names still add up to the system draw.
    Without a BoM, sensors are not metered separately and the value is None.
    """
    bom = bool(profile.get("bom_build") or profile.get("power_model") == "bom_linear")
    out: dict = {
        "locomotion": round(float(channel_draws.get("Legs") or 0.0), 2),
        "arms": round(float(channel_draws.get("Arms") or 0.0), 2),
        "torso": round(float(channel_draws.get("Torso") or 0.0), 2),
        "cooling": round(float(channel_draws.get("Cooling") or 0.0), 2),
    }
    compute_ch = round(float(channel_draws.get("Compute") or 0.0), 2)
    if not bom:
        out["compute"] = compute_ch
        out["sensors"] = None
        return out
    from src.bom_profile import subsystem_part_watts

    parts = subsystem_part_watts(profile)
    gap = round(compute_ch - float(parts["compute"]) - float(parts["sensors"]), 2)
    out["compute"] = round(float(parts["compute"]) + gap, 2)
    out["sensors"] = float(parts["sensors"])
    return out


def gait_to_task(gait: str) -> str:
    """Map locomotion gait string onto a PMS mission task id."""
    return _GAIT_TO_TASK.get(str(gait).lower(), "balanced")


def build_webots_telemetry(
    *,
    joints: list[dict],
    gait: str = "stand",
    phase: str = "",
    speed_m_s: float = 0.0,
    battery_pct: float = 90.0,
    pose: dict | None = None,
    sensors: dict | None = None,
    robot_name: str = "ButlerBot",
    motor_power_w: dict[str, float] | None = None,
    profile: dict | None = None,
) -> dict:
    """Build a DigitalTwinBridge telemetry payload from Webots controller data.

    Controllers should pass ``sensors`` with hub rates, yaw, braking flags, etc.
    so residual-spin diagnostics survive the twin models → dashboard path.
    Pre-built channel_draws from this function are preferred by WebotsAdapter
    over re-aggregation, so HUD and PMS stay numerically aligned.
    """
    prof = _resolve_profile(profile)
    capacity_wh = battery_capacity_wh(prof)
    batt = prof.get("battery") or {}

    motor_power_w = motor_power_w or motor_powers_from_joints(
        joints, prof, speed_m_s=speed_m_s
    )
    channel_draws = aggregate_channel_draws(
        motor_power_w,
        gait=gait,
        phase=phase,
        speed_m_s=speed_m_s,
        joints=joints,
        profile=prof,
    )
    task = gait_to_task(gait)
    norm_phase = normalize_phase_name(phase) if phase else phase
    subsystem_draws = _subsystem_draws(prof, channel_draws)
    locomotion = _locomotion_block(
        prof, gait=gait, phase=norm_phase or phase, speed_m_s=speed_m_s
    )
    hardware = {}
    if locomotion.get("bom_build"):
        hardware = {
            "bom_build": locomotion.get("bom_build"),
            "bom_schema_version": locomotion.get("bom_schema_version"),
            "payload_kg": locomotion.get("payload_kg"),
        }

    return {
        "schema_version": "1.1",
        "source": "webots",
        "adapter": "webots",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "robot": {
            "name": robot_name,
            "main_battery_pct": round(battery_pct, 2),
            "battery_capacity_wh": capacity_wh,
            "battery_voltage_v": batt.get("nominal_voltage_v"),
            "hardware_profile": prof.get("profile_id"),
            "hardware_profile_version": prof.get("version"),
        },
        "mission": {"task": task, "phase": norm_phase or phase},
        "motor_power_w": motor_power_w,
        "channel_draws": channel_draws,
        "subsystem_draws": subsystem_draws,
        "joints": joints,
        "locomotion": locomotion,
        "hardware": hardware,
        # Real-part BOM trace (motors, driver, pack, wheels)
        "hardware_parts": {
            "left_wheel": motor_part_meta("left_wheel", prof),
            "right_wheel": motor_part_meta("right_wheel", prof),
            "motor_driver": {
                k: v
                for k, v in (motor_driver_spec(prof) or {}).items()
                if k
                in (
                    "part_number",
                    "vendor",
                    "chip",
                    "product_url",
                    "datasheet_url",
                    "continuous_current_a_per_ch",
                    "peak_current_a_per_ch",
                    "efficiency",
                    "continuous_bus_w",
                )
            },
            "dc_dc_48_12": {
                k: v
                for k, v in (prof.get("dc_dc_48_12") or {}).items()
                if k
                in (
                    "label",
                    "part_class",
                    "vendor",
                    "product_url",
                    "efficiency",
                    "continuous_power_w",
                    "idle_w",
                    "mass_kg",
                )
            },
            "battery": {
                k: v
                for k, v in (prof.get("battery") or {}).items()
                if k
                in (
                    "label",
                    "chemistry",
                    "capacity_wh",
                    "capacity_ah",
                    "nominal_voltage_v",
                    "continuous_c_rate",
                    "peak_c_rate",
                    "continuous_discharge_a",
                    "peak_discharge_a",
                    "continuous_power_w",
                    "mass_kg",
                    "product_url",
                )
            },
            "compute": {
                k: v
                for k, v in (prof.get("compute") or {}).items()
                if k
                in (
                    "label",
                    "model",
                    "vendor",
                    "product_url",
                    "idle_w",
                    "active_w",
                    "peak_w",
                    "mass_kg",
                )
            },
            "wheel_tire": (prof.get("geometry") or {}).get("tire") or {
                "mass_kg": wheel_mass_kg(prof),
                "diameter_mm": 1000 * wheel_radius_m(prof) * 2,
            },
            "chassis": (prof.get("geometry") or {}).get("chassis")
            or {
                "mass_kg": (prof.get("geometry") or {}).get("chassis_mass_kg"),
            },
        },
        "pose": pose or {},
        "sensors": sensors or {},
        "power": {
            "total_draw_w": round(sum(channel_draws.values()), 2),
            "pack_c_rate": round(
                battery_draw_c_rate(sum(channel_draws.values()), prof), 3
            ),
            "pack_limits": battery_c_rate_limits(prof),
            "channel_draws": channel_draws,
            "hardware_profile": prof.get("profile_id"),
        },
    }


def restamp_active_bom(payload: dict) -> dict:
    """Make a live Webots sample use the config BoM build.

    The dashboard and the controller do not share a working directory.
    Webots used to miss ``config/config.yaml``, post the YAML profile
    (arms and torso watts, ``sensors=na``, ``build=default``), while the
    dashboard log said ``BOM build=light``.     With a build selected, joint samples are recomputed on the BoM line
    even if the controller already stamped that build. That rebuild is
    the predicted Legs bill. ``measured_drive_w`` is shaft |τ|·|ω| on
    the 0.08 m hinge. ``measured_drive_elec_w`` is that same (τ, ω)
    through the published motor line. Missing feedback is None on both,
    and the rolling line is not written in their place.
    """
    from src.hardware_profile import active_bom_build

    if not isinstance(payload, dict):
        return payload
    build = active_bom_build()
    if not build:
        return payload
    loc = payload.get("locomotion") if isinstance(payload.get("locomotion"), dict) else {}
    hw = payload.get("hardware") if isinstance(payload.get("hardware"), dict) else {}
    # Always recompute when joints are present. Channel Legs stay on the
    # rolling line. Measured shaft power is captured first, from
    # torque_feedback_nm, and is not replaced by that line.
    joints = [j for j in (payload.get("joints") or []) if isinstance(j, dict)]
    if not joints:
        out = dict(payload)
        loc_out = dict(loc)
        loc_out["bom_build"] = build
        hw_out = dict(hw)
        hw_out["bom_build"] = build
        out["locomotion"] = loc_out
        out["hardware"] = hw_out
        return out
    clean = []
    for joint in joints:
        copied = dict(joint)
        if "wheel" in str(copied.get("name") or ""):
            copied["torque"] = 0.0
        clean.append(copied)
    robot = payload.get("robot") if isinstance(payload.get("robot"), dict) else {}
    battery = robot.get("main_battery_pct")
    if battery is None:
        battery = robot.get("battery_pct")
    try:
        battery_f = float(battery) if battery is not None else 90.0
    except (TypeError, ValueError):
        battery_f = 90.0
    speed = loc.get("speed_m_s")
    if speed is None:
        speed = payload.get("speed_m_s") or 0.0
    try:
        speed_f = float(speed)
    except (TypeError, ValueError):
        speed_f = 0.0
    split = measured_drive_split(joints, speed_f, get_active_profile())
    measured = None if split is None else split["shaft_w"]
    measured_elec = None if split is None else split["elec_w"]
    rebuilt = build_webots_telemetry(
        joints=clean,
        gait=str(payload.get("gait") or loc.get("gait") or "stand"),
        phase=str(payload.get("phase") or loc.get("phase") or ""),
        speed_m_s=speed_f,
        battery_pct=battery_f,
        pose=payload.get("pose") if isinstance(payload.get("pose"), dict) else {},
        sensors=payload.get("sensors") if isinstance(payload.get("sensors"), dict) else {},
        robot_name=str(robot.get("name") or "ButlerBot"),
        profile=get_active_profile(),
    )
    if payload.get("timestamp"):
        rebuilt["timestamp"] = payload["timestamp"]
    loco = rebuilt.get("locomotion") if isinstance(rebuilt.get("locomotion"), dict) else {}
    loco = dict(loco)
    loco["measured_drive_w"] = measured
    loco["measured_drive_elec_w"] = measured_elec
    # The rebuild replaces locomotion. Keep the controller's sim-time
    # energy counters, run id, and steer summary, or a dashboard restart
    # cannot tell one Webots process from the next.
    from src.twin_energy import LOCO_PASSTHROUGH_KEYS

    for key in LOCO_PASSTHROUGH_KEYS:
        if key in loc:
            loco[key] = loc[key]
    rebuilt["locomotion"] = loco
    rebuilt["measured_drive_w"] = measured
    rebuilt["measured_drive_elec_w"] = measured_elec
    # Keep the feedback on the joints the model rebuild stored, so a later
    # reader can see the raw hinge torque next to the zero model load.
    feedback_by_name = {
        str(joint.get("name") or ""): joint.get("torque_feedback_nm")
        for joint in joints
        if "torque_feedback_nm" in joint
    }
    stamped_joints = []
    for joint in rebuilt.get("joints") or []:
        if not isinstance(joint, dict):
            stamped_joints.append(joint)
            continue
        copied = dict(joint)
        name = str(copied.get("name") or "")
        if name in feedback_by_name:
            copied["torque_feedback_nm"] = feedback_by_name[name]
        stamped_joints.append(copied)
    rebuilt["joints"] = stamped_joints
    return rebuilt
