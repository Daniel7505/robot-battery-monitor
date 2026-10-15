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
    "energy_elec_gaps",
)


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
    ``elec_gaps``. Idle is still billed. The rolling prediction is not
    written in place of the missing drive. A single step longer than
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

    ``wh_roll`` is the rolling-only motor line at ``v / 0.04``. ``wh_visc``
    is the hinge-rate combined point (rolling torque on 0.04 m, damping
    torque ``c·v/0.08``, electrical speed ``v/0.08``) minus that roll
    bill, so roll + visc is the sim point and the printed roll does not
    move.
    ``wh_turn`` is ``turning_scrub_wh`` for this piece's heading, at the
    piece's average speed. Zero heading bills zero. Rolling per 10 ft
    does not change.
    """
    from src.hardware_profile import get_active_profile

    prof = profile if profile is not None else get_active_profile()
    dist = max(0.0, float(dist_m))
    speed = float(avg_speed_m_s)
    roll = 0.0
    visc = 0.0
    turn = 0.0
    source = "none"
    if dist > 0 and speed > 0:
        roll = (dist / TEN_FT_M) * _per_10ft(bom_drive_terms(prof, speed), payload_kg)
        sim = (dist / TEN_FT_M) * _per_10ft(sim_drive_terms(prof, speed), payload_kg)
        visc = max(0.0, sim - roll)
        dt = dist / speed
        turn = turning_scrub_wh(prof, speed, heading_change_deg, dt, payload_kg)
        if turn > 0:
            source = TURN_SOURCE
    return {
        "wh_roll": roll,
        "wh_visc": visc,
        "wh_turn": turn,
        "wh_accel": max(0.0, float(wh_accel)),
        "turn_source": source,
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


def bom_run_energy_wh(
    dist_m: float,
    duration_s: float,
    payload_kg: float = 0.0,
    profile: dict | None = None,
    heading_change_deg: float = 0.0,
    wh_accel: float = 0.0,
    wh_turn: float | None = None,
) -> dict | None:
    """Idle for the whole run, plus rolling, hinge damping, turn, and accel.

    ``idle_w_known × hours`` is the compute/sensor idle from the BoM (9.96 W
    on the light build). Drive is the brushed-DC line at the run's average
    speed. ``wh_model_roll`` is rolling resistance only, at ``v / 0.04``.
    ``wh_model_visc`` is the extra from ``dampingConstant 0.15`` evaluated
    at the 0.08 m hinge rate (``τ = c·ω``, electrical ``ω = v/0.08``).
    ``wh_model_turn`` is the track/FDS scrub term. Passed ``wh_turn`` wins:
    the live log integrates each sample, because one average yaw rate over
    a whole lap under-bills a short corner. Otherwise the term uses
    ``heading_change_deg`` at the average speed. Acceleration is the
    caller's summed motor-line bound (zero when omitted). Unknown idle
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
    if wh_turn is None:
        turn_wh = 0.0
        if distance > 0 and speed > 0 and duration > 0:
            turn_wh = turning_scrub_wh(
                prof, speed, heading_change_deg, duration, payload_kg
            )
    else:
        turn_wh = max(0.0, float(wh_turn))
    accel_wh = max(0.0, float(wh_accel))
    drive_wh = roll_wh + visc_wh + turn_wh + accel_wh
    return {
        "wh_model": round(idle_wh + drive_wh, 3),
        "wh_model_idle": round(idle_wh, 3),
        "wh_model_drive": round(drive_wh, 3),
        "wh_model_roll": round(roll_wh, 3),
        "wh_model_visc": round(visc_wh, 3),
        "wh_model_turn": round(turn_wh, 3),
        "wh_model_accel": round(accel_wh, 3),
        "idle_w": idle_w,
        "turn_source": TURN_SOURCE if turn_wh > 0 else "none",
    }


def format_mission_energy_line(
    dist_m: float,
    payload_kg: float | None = None,
    wh_measured: float | None = None,
    duration_s: float | None = None,
    heading_change_deg: float = 0.0,
    wh_accel: float = 0.0,
    wh_turn: float | None = None,
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
            wh_turn=wh_turn,
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
