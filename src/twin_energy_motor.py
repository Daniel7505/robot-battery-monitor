"""Brushed-DC circuit, pack meter, and the stall-hold electrical point."""

from __future__ import annotations

import math

from src.twin_energy_cruise import (
    DEFAULT_CRR,
    DEFAULT_DRIVER_EFF,
    GRAVITY_M_S2,
    MOTOR_COPPER_REF_SPEED_M_S,
    TEN_FT_M,
)

def _pack_watts_from_shaft(
    tau_nm: float,
    omega_rad_s: float,
    *,
    voltage_v: float,
    no_load_rpm: float,
    no_load_current_a: float,
    stall_current_a: float,
    stall_torque_nm: float,
    eta_driver: float,
) -> tuple[float, float, float]:
    """Duty, current, and pack watts of one motor. One curve for every bill.

    This is ``electrical_from_shaft_w``. A missing line (bad stall or
    driver efficiency) is 0 W, matching the old early return.
    """
    point = electrical_from_shaft_w(
        tau_nm,
        omega_rad_s,
        voltage_v=voltage_v,
        no_load_rpm=no_load_rpm,
        no_load_current_a=no_load_current_a,
        stall_current_a=stall_current_a,
        stall_torque_nm=stall_torque_nm,
        eta_driver=eta_driver,
    )
    if point["elec_w"] is None:
        return 0.0, float(no_load_current_a), 0.0
    return float(point["duty"]), float(point["current_a"]), float(point["elec_w"])


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
    electrical_omega_rad_s: float | None = None,
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

    ``electrical_omega_rad_s``, when set, replaces ``speed / wheel_radius``
    in the duty term only. The torque lever stays ``wheel_radius_m``.
    The design bill leaves it unset, so that rate is body speed / 0.04 m.
    The sim viscous bill passes the drawn hinge rate, body speed / 0.08 m.

    Pack watts are ``electrical_from_shaft_w`` on each motor: the same
    speed-and-torque line as the measured hinge conversion. Driver
    efficiency is that function's ``eta_driver``, applied once. The
    datasheet peak efficiency is not used.
    """
    radius = max(float(wheel_radius_m), 1e-6)
    speed = float(speed_m_s)
    motors = max(1, int(n_motors))
    force_n = float(crr) * float(mass_kg) * GRAVITY_M_S2 + max(0.0, float(extra_force_n))
    added = max(0.0, float(extra_torque_nm))
    tau = force_n * radius / float(motors) + added
    # Design bill: ω = v / BoM radius. A caller can pass the hinge rate
    # without moving the rolling lever arm.
    if electrical_omega_rad_s is None:
        omega = speed / radius
    else:
        omega = max(0.0, float(electrical_omega_rad_s))
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
    duty, current, drive_one = _pack_watts_from_shaft(
        tau_elec,
        omega,
        voltage_v=float(voltage_v),
        no_load_rpm=float(no_load_rpm),
        no_load_current_a=float(no_load_current_a),
        stall_current_a=float(stall_current_a),
        stall_torque_nm=tau_stall,
        eta_driver=float(eta_driver),
    )
    drive_w = float(motors) * drive_one
    if (
        speed > MOTOR_COPPER_REF_SPEED_M_S + 1e-6
        and float(stall_current_a) > 1e-6
        and float(voltage_v) > 0.0
    ):
        ref_omega = MOTOR_COPPER_REF_SPEED_M_S / radius
        ref_tau = min(
            (float(crr) * float(mass_kg) * GRAVITY_M_S2 + max(0.0, float(extra_force_n)))
            * radius
            / float(motors)
            + added,
            tau_stall,
        )
        _, i_ref, _ = _pack_watts_from_shaft(
            ref_tau,
            ref_omega,
            voltage_v=float(voltage_v),
            no_load_rpm=float(no_load_rpm),
            no_load_current_a=float(no_load_current_a),
            stall_current_a=float(stall_current_a),
            stall_torque_nm=tau_stall,
            eta_driver=float(eta_driver),
        )
        r_arm = float(voltage_v) / float(stall_current_a)
        copper = r_arm * max(0.0, float(current) ** 2 - float(i_ref) ** 2)
        drive_w += float(motors) * copper
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
        _duty_m, _current_m, watts = _pack_watts_from_shaft(
            tau_m,
            omega,
            voltage_v=float(voltage_v),
            no_load_rpm=float(no_load_rpm),
            no_load_current_a=float(no_load_current_a),
            stall_current_a=float(stall_current_a),
            stall_torque_nm=tau_stall,
            eta_driver=float(eta_driver),
        )
        return float(motors) * watts

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
# Posted on locomotion. The dashboard bills the positive delta of these
# running totals. They are sim-time integrals, not wall-clock gaps.
ENERGY_COUNTER_KEYS = (
    "energy_sim_s",
    "energy_wh_shaft",
    "energy_wh_elec",
    "energy_wh_idle",
    "energy_wh_pack",
)
# Copied through the Webots adapter even though restamp rebuilds locomotion.
LOCO_PASSTHROUGH_KEYS = ENERGY_COUNTER_KEYS + (
    "controller_run_id",
    "opt_run_id",
    "opt_trial_id",
    "ctrl_cruise_m_s",
    "ctrl_straight_m_s",
    "ctrl_pivot_w_max_rad_s",
    "ctrl_k_steer",
    "ctrl_bom_build",
    "body_mass_kg",
    "steer_split",
    "steer_source",
    "steer_lateral",
    "fit_reject",
    "cross_track_m",
    "ct_max_m",
    "lane_width_m",
    "width_live",
    "true_width_m",
    "true_width_near_m",
    "width_err_m",
    "center_err_m",
    "lane_frame_n",
    "energy_elec_gaps",
    "gyro_found",
    "coast_wh_saved",
    "coast_starts",
    "ctrl_pivot_count",
    "ctrl_coast_enabled",
    "left_wheel_rad_s",
    "right_wheel_rad_s",
)


# Body slower than creep and both hinges quieter than a slow pivot:
# the kinematic roll term bills 0 W, but a velocity hold at 0 rad/s does not.
STALL_HOLD_BODY_M_S = 0.08
STALL_HOLD_WHEEL_RAD_S = 0.15


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
            "The mission turn term uses this same duty/current line. "
            "It does not divide scrub work by a fixed efficiency.",
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
    """Old wall-gap drain clock. The live BoM path does not call this.

    The log used to integrate the gap between telemetry timestamps and
    drop a gap longer than 15 s, and the controller used this same rule
    on ``time.monotonic``. A POST that blocked for a few seconds billed
    those seconds of sim watts. A hole longer than 15 s billed on the
    controller (each step was under 15 s) and was dropped by the log.
    Run #11's 0.84 Wh split was that hole. ``SimPackMeter`` integrates
    the simulation step instead and posts the running total.
    """
    if wall_gap_s is None:
        dt = float(sim_dt_s)
    else:
        dt = float(wall_gap_s)
    if dt <= 0.0 or dt > float(max_dt_s):
        return 0.0
    return dt


class SimPackMeter:
    """Pack watt-hours on the simulation clock, one physics step at a time.

    Shaft, electrical drive, idle, and the pack total (drive + idle) share
    this step. The controller drains the percent from ``last_step_wh`` and
    posts ``snapshot`` on telemetry. The dashboard reads the delta, so a
    late or dropped POST cannot move SoC without moving ``wh_measured``.

    A missing electrical sample adds no drive watts and increments
    ``elec_gaps``. That counter is physics steps, not telemetry posts.
    Idle is still billed. The rolling prediction is not written in
    place of the missing drive. The dashboard bills the snapshot
    delta, so a short hole between posts (the camera_s lap's 0.8 s
    max) does not drop watt-hours already in the counter. A gap step
    undercounts drive only while the hinge was turning without
    feedback. A parked default-build wheel omits feedback and counts
    as a gap at about 0 W. A single step longer than
    ``PACK_DRAW_MAX_DT_S`` adds nothing: a physics step is 8 ms, so that
    guard is only a bad clock.
    """

    def __init__(self) -> None:
        self.sim_s = 0.0
        self.wh_shaft = 0.0
        self.wh_elec = 0.0
        self.wh_idle = 0.0
        self.wh_pack = 0.0
        self.elec_gaps = 0
        self.shaft_gaps = 0
        self.last_pack_w = 0.0
        self.last_step_wh = 0.0

    def step(
        self,
        sim_dt_s: float,
        *,
        shaft_w: float | None,
        elec_w: float | None,
        idle_w: float | None,
    ) -> float:
        """Add one simulation step. Return the pack watt-hours just added."""
        try:
            dt = float(sim_dt_s)
        except (TypeError, ValueError):
            dt = 0.0
        if dt <= 0.0 or dt > PACK_DRAW_MAX_DT_S:
            self.last_pack_w = 0.0
            self.last_step_wh = 0.0
            return 0.0
        idle = 0.0 if idle_w is None else float(idle_w)
        if shaft_w is None:
            self.shaft_gaps += 1
        else:
            self.wh_shaft += float(shaft_w) * dt / 3600.0
        if elec_w is None:
            self.elec_gaps += 1
            drive = 0.0
        else:
            drive = float(elec_w)
            self.wh_elec += drive * dt / 3600.0
        self.sim_s += dt
        self.wh_idle += idle * dt / 3600.0
        step_wh = (drive + idle) * dt / 3600.0
        self.wh_pack += step_wh
        self.last_pack_w = drive + idle
        self.last_step_wh = step_wh
        return step_wh

    def snapshot(self) -> dict:
        return {
            "energy_sim_s": self.sim_s,
            "energy_wh_shaft": self.wh_shaft,
            "energy_wh_elec": self.wh_elec,
            "energy_wh_idle": self.wh_idle,
            "energy_wh_pack": self.wh_pack,
            "energy_elec_gaps": self.elec_gaps,
        }


def stall_hold_electrical_wh(dt_s: float, profile: dict | None) -> float:
    """Pack watt-hours for ``dt_s`` with both drive motors at published stall.

    ``ω = 0`` and ``τ = τstall`` is the point a velocity command of 0 rad/s
    hits while the axle still has load. The distance-based roll term is 0 W
    there (``brushed_drive_point`` returns early when speed is 0). Two motors,
    ``P = duty·V·I/η`` with duty = 1 and I = Istall.
    """
    if profile is None or dt_s <= 0.0:
        return 0.0
    motors = profile.get("motors") or {}
    spec = {}
    if isinstance(motors, dict):
        spec = dict(motors.get("left_wheel") or motors.get("right_wheel") or {})
    try:
        voltage = float(spec.get("voltage_v"))
        rpm = float(spec.get("no_load_rpm"))
        i0 = float(spec.get("no_load_current_a"))
        i_stall = float(spec.get("stall_current_a"))
        tau = float(spec.get("stall_torque_nm"))
    except (TypeError, ValueError):
        return 0.0
    assumptions = profile.get("assumptions") or {}
    driver = profile.get("motor_driver") or {}
    eta = driver.get("efficiency")
    if eta is None:
        eta = assumptions.get("driver_eff", DEFAULT_DRIVER_EFF)
    point = electrical_from_shaft_w(
        tau,
        0.0,
        voltage_v=voltage,
        no_load_rpm=rpm,
        no_load_current_a=i0,
        stall_current_a=i_stall,
        stall_torque_nm=tau,
        eta_driver=float(eta),
    )
    if point.get("elec_w") is None:
        return 0.0
    return 2.0 * float(point["elec_w"]) * float(dt_s) / 3600.0
