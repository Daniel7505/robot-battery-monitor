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

from datetime import datetime, timezone

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
    compute = profile.get("compute") or {}
    sensors = profile.get("sensors") or {}
    modes = profile.get("modes") or {}
    idle_w = float(compute.get("idle_w", COMPUTE_IDLE_W))
    active_w = float(compute.get("active_w", COMPUTE_ACTIVE_W))
    sens_idle = float(sensors.get("idle_w", 0.0))
    sens_active = float(sensors.get("active_w", sens_idle))
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

    stab_ch, stab_w = _stabilizer_draw_w(prof, speed_m_s=speed_m_s)
    if stab_w > 0:
        channels[stab_ch] = round(channels.get(stab_ch, 0.0) + stab_w, 2)

    if compute_w is None:
        compute_w = _compute_draw_w(gait, prof, speed_m_s=speed_m_s)
    channels["Compute"] = round(compute_w, 2)

    # Mild stress from gait/phase (profile cruise already sets base drive level).
    # Keep stress light so Legs land mid-band instead of hard-pegging channel max.
    mult = stress_multiplier(gait=gait, phase=phase)
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

    return channels


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
        "joints": joints,
        "locomotion": {
            "gait": gait,
            "speed_m_s": round(speed_m_s, 3),
            "phase": norm_phase or phase,
            "mode": "wheeled",
            "wheel_radius_m": wheel_radius_m(prof),
        },
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
            "total_draw_w": round(sum(channel_draws.values()), 1),
            "pack_c_rate": round(
                battery_draw_c_rate(sum(channel_draws.values()), prof), 3
            ),
            "pack_limits": battery_c_rate_limits(prof),
            "channel_draws": channel_draws,
            "hardware_profile": prof.get("profile_id"),
        },
    }
