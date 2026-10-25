"""Segment rollup and the MISSION_ENERGY line."""

from __future__ import annotations

from src.twin_energy_cruise import (
    TEN_FT_M,
    TURN_SOURCE,
    estimate_mission_energy,
    load_override,
)
from src.twin_energy_drive import bom_drive_terms, sim_drive_terms, turning_scrub_wh

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
    wh_hold: float = 0.0,
) -> str:
    """``MISSION_ENERGY build=...`` line. Model and measured stay separate.

    The model ignores ``data/twin_energy.json`` so a measured Webots Wh can
    sit next to it as ``wh_measured`` / ``err_pct``. When that file is
    present and the caller did not pass ``wh_measured``, it is the measured
    value.

    With ``duration_s`` and a BoM build, the model is idle over the whole
    run plus drive for ``dist_m``, not drive-only and not idle only while
    the wheels are turning. ``err_pct`` is that total against measured
    drive plus idle.

    With no BoM build the printed model is the V1 cruise lump (idle and
    drive in one number, blank drive terms). Measured is hinge drive
    plus channel idle. Those are not the same split, so ``err_pct`` is
    ``na`` instead of a percent.

    ``wh_hold`` is the stall-hold watt-hours the summary already adds to
    its model. It is reporting only: the drive model in
    ``bom_run_energy_wh`` is unchanged, and the printed ``wh_model`` /
    ``err_pct`` include the hold so this line matches that summary.
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
    hold = round(max(0.0, float(wh_hold or 0.0)), 3)
    if run is not None:
        wh_model = run["wh_model"]
        if hold:
            wh_model = round(float(wh_model) + hold, 3)
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
        # No BoM: V1 lump vs measured drive+idle is not a like-for-like error.
        if not active_bom_build():
            err_txt = "na"
        elif wh_model:
            err_txt = f"{(float(wh_measured) - wh_model) / wh_model * 100.0:.1f}"
        else:
            err_txt = ""
    build = active_bom_build() or "default"
    hold_field = f" wh_model_hold={hold:.3f}" if run is not None else ""
    return (
        f"MISSION_ENERGY build={build} dist_m={dist_m} payload_kg={payload_kg} "
        f"wh_model={wh_model} wh_measured={measured_txt} err_pct={err_txt}"
        f"{hold_field}"
    )
