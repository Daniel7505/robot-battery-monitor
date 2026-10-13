"""Piecewise energy log for a live twin run.

The dashboard logger writes three lines, in the usual
``time | LEVEL | robot_monitor | message`` format:

* ``SEGMENT_ENERGY`` — one line each time the robot travels about 10 ft
  (3.048 m). Watt-hours are the integral of the channel draws over that
  piece, split into locomotion (``wh_drive``) and everything else
  (``wh_idle``).
* ``POWER_BREAKDOWN`` — about every 10 s, the average watts of compute,
  sensors, cooling, locomotion, arms, and torso. Those fields add up to
  ``total``, which is the system draw.
* ``MISSION_SUMMARY`` — once, when the controller reports
  ``mission_event=complete`` (ABS park finished after the track finish
  line). Model vs measured, SoC start/end, turns.

Measured values come only from telemetry samples. A subsystem that was
not metered is logged as ``na``, not as zero. A BoM build does meter
compute and sensors separately because those published idle watts are
what the Compute channel is already charging.

Why the old panel, SoC, and BoM idle disagreed
----------------------------------------------
On the light build the published idle is 9.96 W. A live Windows run
showed system draw 20.3 W, SoC energy 0.864 Wh over 8:47, and 232.4 Wh
left on a 240 Wh pack (96.8%). Those three numbers were not the same
load:

* System draw was the allocator's channel sum. Moving used the Jetson's
  25 W mode instead of the 7 W idle the energy table uses, and a
  made-up wheel torque (``|ω| × 0.45``) is past the light motor's stall.
* SoC was integrated in the Webots controller from joint watts only
  (no compute) against a hardcoded 480 Wh pack. The dashboard then
  turned that percent into watt-hours with the 240 Wh BoM pack, so
  20.3 W × 8:47 (~2.97 Wh) could not equal 0.864 Wh.

With a BoM selected, channel draws, the pack drain, and these log lines
all use the published idle plus the motor model, and the controller
drains that same total against the BoM pack. ``wh_model`` is idle over
the whole run plus drive. Drive on each ``SEGMENT_ENERGY`` line is:

* ``wh_roll`` — brushed-DC electrical energy at rolling resistance.
  Watt-hours per metre depend on speed: no-load current is paid per
  revolution, rolling torque is paid per second. On the light build
  that is about 0.0066 Wh per 10 ft near 0.44 m/s, a little more when
  the same 10 ft takes longer. The design shift table uses only this.
* ``wh_visc`` — extra electrical energy from the published Webots hinge
  (``dampingConstant 0.15`` on the 0.08 m wheel) through that same
  motor line. It rises with speed. It is not a coefficient fitted to
  a log.
* ``wh_turn`` — always 0. A fitted yaw term used to bill ~0.11 Wh per
  10 ft. Live locomotion tracks distance and speed, not heading.
* ``wh_accel`` — extra motor-line watts while speeding up (``m·a`` on
  top of rolling torque). Slowing down is zero (no regen). Wheel spin
  inertia is not in the BoM.

``wh_drive`` is the measured locomotion integral. Dashboard Legs stay
on the rolling line (about 3.43 W at 0.44 m/s), so a new run's
``wh_drive`` sits near ``wh_roll``. Webots ``getTorqueFeedback`` is
hinge effort, not a load cell, and is not billed.

A full warehouse run (no finish; stopped by hand) is the check on this
split: 121 segments, 369 m, 2145 s. Mean locomotion 0.120 Wh per 10 ft
(sd 0.023). Correlation with heading −0.11, with average speed +0.82.
Speed bins, Wh per segment: under 0.15 m/s → 0.096; 0.15–0.25 → 0.103;
0.25–0.35 → 0.126; over 0.35 → 0.153. Totals: drive 14.5 Wh, idle
5.9 Wh. Idle matches 9.96 W × 2145 s. Rolling-only does not: it is
about 0.8 Wh and it falls slightly as speed rises. Published damping
rises with speed and is about 6 Wh at the run's average speed, short
of 14.5 Wh. The remainder is sim tire contact. That torque is not one
published number, and a straight-line fit of the log is not pasted in
as a bill.

``err_pct`` is model versus integrated channel watts
(``wh_measured``). ``wh_measured_soc`` is
``capacity × (start% − end%) / 100``.

A ``mission_event=complete`` with essentially no distance and no
duration is not a mission (a Webots process killed on startup). It
does not log a summary, and it does not consume the one summary.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone

from src.teleop_agent import plausible_travel_m
from src.twin_energy import TEN_FT_M

POWER_PERIOD_S = 10.0
# A speed change steeper than this, from one sample to the next, counts
# as one acceleration event. It is not a planned accel profile.
ACCEL_EVENT_M_S2 = 0.4
# One turn per this many degrees of accumulated |heading change|.
# A bend that later straightens still counts: the old net-from-anchor
# counter stayed at 0 on the S while segments logged 61.6° and 32.3°.
TURN_DEG = 30.0
# Same stall guard as the efficiency tracker: do not bill a paused Webots.
MAX_DT_S = 15.0
# A killed Webots process can post complete after a fraction of a second
# and a few centimetres. That is not a mission summary.
MIN_SUMMARY_DIST_M = 0.5
MIN_SUMMARY_DURATION_S = 2.0


def _as_float(value) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _wrap_rad(delta: float) -> float:
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def _fmt(value: float | None, digits: int) -> str:
    if value is None:
        return ""
    return f"{value:.{digits}f}"


class MissionEnergyLog:
    """Fold live samples into segment, breakdown, and end-of-mission lines."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._armed = False
        self._summarized = False
        self._last_ts: datetime | None = None
        self._last_xy: tuple[float, float] | None = None
        self._last_heading: float | None = None
        self._last_speed: float | None = None
        self._heading_accum_deg = 0.0
        self._turns = 0
        self._soc_start: float | None = None
        self._soc_end: float | None = None
        self._capacity_wh: float | None = None
        self._payload_kg = 0.0
        self._build = "default"
        self._mission_dt = 0.0
        self._mission_dist = 0.0
        self._mission_wh_drive = 0.0
        self._mission_wh_idle = 0.0
        self._mission_wh_accel = 0.0
        self._clear_segment()
        self._clear_power()

    def _clear_segment(self) -> None:
        self._seg_idx = 0
        self._seg_dist = 0.0
        self._seg_dt = 0.0
        self._seg_wh_drive = 0.0
        self._seg_wh_idle = 0.0
        self._seg_wh_accel = 0.0
        self._seg_heading_deg = 0.0
        self._seg_accel = 0

    def _clear_power(self) -> None:
        self._power_dt = 0.0
        self._power_acc: dict[str, float] = {}
        self._power_seen: dict[str, bool] = {}

    def observe(self, sample: dict) -> list[str]:
        """Ingest one telemetry sample. Return log lines to emit (may be empty)."""
        ts = sample.get("timestamp")
        if not isinstance(ts, datetime):
            ts = datetime.now(timezone.utc)
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)

        lines: list[str] = []
        if not self._armed:
            self._arm(sample, ts)
            return lines

        dt = (ts - self._last_ts).total_seconds() if self._last_ts else 0.0
        self._last_ts = ts
        if dt <= 0 or dt > MAX_DT_S:
            self._note_pose(sample)
            lines.extend(self._maybe_summary(sample))
            return lines

        step_m, speed, heading_deg, trusted = self._step_motion(sample, dt)
        subs = _subsystems(sample)
        drive_w, idle_w = _drive_idle_w(sample, subs)
        wh_drive = drive_w * dt / 3600.0
        wh_idle = idle_w * dt / 3600.0
        accel = 0
        wh_accel_step = 0.0
        if self._last_speed is not None and abs(speed - self._last_speed) / dt >= ACCEL_EVENT_M_S2:
            accel = 1
        if trusted and self._last_speed is not None and speed > self._last_speed:
            wh_accel_step = _accel_step_wh(self._last_speed, speed, dt, self._payload_kg)
        self._last_speed = speed

        self._mission_dt += dt
        self._mission_dist += step_m
        self._mission_wh_drive += wh_drive
        self._mission_wh_idle += wh_idle
        self._mission_wh_accel += wh_accel_step
        self._note_soc(sample)
        if trusted:
            self._note_turn(heading_deg)

        self._power_dt += dt
        for key, watts in subs.items():
            if watts is None:
                self._power_seen[key] = False
                continue
            self._power_seen[key] = True
            self._power_acc[key] = self._power_acc.get(key, 0.0) + float(watts) * dt
        if self._power_dt >= POWER_PERIOD_S:
            lines.append(self._power_line())
            self._clear_power()

        if trusted:
            self._seg_dist += step_m
            self._seg_dt += dt
            self._seg_wh_drive += wh_drive
            self._seg_wh_idle += wh_idle
            self._seg_wh_accel += wh_accel_step
            self._seg_heading_deg += abs(heading_deg)
            if accel:
                self._seg_accel += accel
        # One line per sample, and only when this sample actually moved.
        # A pose jump used to close several segments in the same millisecond.
        if trusted and step_m > 0 and self._seg_dist >= TEN_FT_M - 1e-9 and self._seg_dist > 0:
            frac = min(1.0, TEN_FT_M / self._seg_dist)
            closed_dist = self._seg_dist
            closed_dt = self._seg_dt
            closed_drive = self._seg_wh_drive
            closed_idle = self._seg_wh_idle
            closed_accel = self._seg_wh_accel
            closed_head = self._seg_heading_deg
            self._seg_dist = closed_dist * frac
            self._seg_dt = closed_dt * frac
            self._seg_wh_drive = closed_drive * frac
            self._seg_wh_idle = closed_idle * frac
            self._seg_wh_accel = closed_accel * frac
            self._seg_heading_deg = closed_head * frac
            lines.append(self._segment_line())
            self._seg_idx += 1
            keep = 1.0 - frac
            self._seg_dist = closed_dist * keep
            self._seg_dt = closed_dt * keep
            self._seg_wh_drive = closed_drive * keep
            self._seg_wh_idle = closed_idle * keep
            self._seg_wh_accel = closed_accel * keep
            self._seg_heading_deg = closed_head * keep
            self._seg_accel = 0

        lines.extend(self._maybe_summary(sample))
        return lines

    def _arm(self, sample: dict, ts: datetime) -> None:
        self._armed = True
        self._last_ts = ts
        self._build = resolve_log_build(sample.get("build"))
        payload = _as_float(sample.get("payload_kg"))
        self._payload_kg = 0.0 if payload is None else payload
        cap = _as_float(sample.get("capacity_wh"))
        if cap is not None and cap > 0:
            self._capacity_wh = cap
        soc = _as_float(sample.get("battery_pct"))
        if soc is not None:
            self._soc_start = soc
            self._soc_end = soc
        self._note_pose(sample)
        heading = _heading_rad(sample)
        if heading is not None:
            self._last_heading = heading
        speed = _as_float(sample.get("speed_m_s"))
        if speed is not None:
            self._last_speed = speed

    def _note_soc(self, sample: dict) -> None:
        soc = _as_float(sample.get("battery_pct"))
        if soc is not None:
            if self._soc_start is None:
                self._soc_start = soc
            self._soc_end = soc
        cap = _as_float(sample.get("capacity_wh"))
        if cap is not None and cap > 0:
            self._capacity_wh = cap
        self._build = resolve_log_build(sample.get("build") or self._build)
        payload = _as_float(sample.get("payload_kg"))
        if payload is not None:
            self._payload_kg = payload

    def _note_pose(self, sample: dict) -> None:
        xy = _xy(sample.get("pose") or {})
        if xy is not None:
            self._last_xy = xy
        heading = _heading_rad(sample)
        if heading is not None:
            self._last_heading = heading

    def _note_turn(self, heading_deg: float) -> None:
        """Count every ``TURN_DEG`` of accumulated yaw, including a bend that unwinds."""
        self._heading_accum_deg += abs(float(heading_deg))
        self._turns = int((self._heading_accum_deg + 1e-6) // TURN_DEG)

    def _step_motion(self, sample: dict, dt: float) -> tuple[float, float, float, bool]:
        xy = _xy(sample.get("pose") or {})
        step_m = 0.0
        if xy is not None and self._last_xy is not None:
            step_m = math.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1])
        if xy is not None:
            self._last_xy = xy
        reported = _as_float(sample.get("speed_m_s"))
        speed = reported if reported is not None else (step_m / dt if dt > 0 else 0.0)
        heading = _heading_rad(sample)
        heading_deg = 0.0
        if heading is not None and self._last_heading is not None:
            heading_deg = math.degrees(_wrap_rad(heading - self._last_heading))
        trusted = True
        credited = plausible_travel_m(step_m, dt)
        if credited + 1e-9 < step_m:
            # Pose teleport. Re-anchor so the next sample is not measured
            # from the old point, and do not bill the jump as a turn.
            heading_deg = 0.0
            step_m = 0.0
            trusted = False
        else:
            step_m = credited
        if heading is not None:
            self._last_heading = heading
        return step_m, speed, heading_deg, trusted

    def _segment_line(self) -> str:
        avg = (self._seg_dist / self._seg_dt) if self._seg_dt > 0 else 0.0
        total = self._seg_wh_drive + self._seg_wh_idle
        terms = _drive_breakdown(
            self._seg_dist,
            avg,
            self._seg_heading_deg,
            self._seg_wh_accel,
            self._payload_kg,
        )
        return (
            f"SEGMENT_ENERGY build={self._build} seg_idx={self._seg_idx} "
            f"dist_m={self._seg_dist:.3f} dt_s={self._seg_dt:.2f} "
            f"avg_speed={avg:.3f} heading_change_deg={self._seg_heading_deg:.1f} "
            f"accel_events={self._seg_accel} payload_kg={self._payload_kg:g} "
            f"wh_drive={self._seg_wh_drive:.4f} "
            f"wh_roll={terms['wh_roll']:.4f} wh_visc={terms['wh_visc']:.4f} "
            f"wh_turn={terms['wh_turn']:.4f} "
            f"wh_accel={terms['wh_accel']:.4f} "
            f"wh_idle={self._seg_wh_idle:.4f} "
            f"wh_total={total:.4f}"
        )

    def _power_line(self) -> str:
        order = ("compute", "sensors", "cooling", "locomotion", "arms", "torso")
        values: dict[str, float | None] = {}
        numeric = 0.0
        for key in order:
            if not self._power_seen.get(key, False) or self._power_dt <= 0:
                values[key] = None
                continue
            avg = self._power_acc.get(key, 0.0) / self._power_dt
            values[key] = round(avg, 2)
            numeric += values[key]
        bits = []
        for key in order:
            if values[key] is None:
                bits.append(f"{key}=na")
            else:
                bits.append(f"{key}={values[key]:.2f}")
        return (
            f"POWER_BREAKDOWN build={self._build} t_s={self._power_dt:.1f} "
            + " ".join(bits)
            + f" total={numeric:.2f}"
        )

    def _maybe_summary(self, sample: dict) -> list[str]:
        if self._summarized:
            return []
        if str(sample.get("mission_event") or "") != "complete":
            return []
        if (
            self._mission_dist < MIN_SUMMARY_DIST_M
            and self._mission_dt < MIN_SUMMARY_DURATION_S
        ):
            return []
        self._summarized = True
        self._note_soc(sample)
        return [self._summary_line(), self._mission_energy_line()]

    def _summary_line(self) -> str:
        measured = self._mission_wh_drive + self._mission_wh_idle
        model = _model(
            self._mission_dist,
            self._mission_dt,
            self._payload_kg,
            self._heading_accum_deg,
            self._mission_wh_accel,
        )
        wh_model = None if model is None else model["wh_model"]
        idle = None if model is None else model["wh_model_idle"]
        drive = None if model is None else model["wh_model_drive"]
        roll = None if model is None else model.get("wh_model_roll")
        visc = None if model is None else model.get("wh_model_visc")
        turn = None if model is None else model.get("wh_model_turn")
        accel_wh = None if model is None else model.get("wh_model_accel")
        soc_wh = None
        if (
            self._soc_start is not None
            and self._soc_end is not None
            and self._capacity_wh
        ):
            soc_wh = max(0.0, self._soc_start - self._soc_end) * self._capacity_wh / 100.0
        err = ""
        if wh_model:
            err = f"{(measured - wh_model) / wh_model * 100.0:.1f}"
        return (
            f"MISSION_SUMMARY build={self._build} "
            f"duration_s={self._mission_dt:.1f} dist_m={self._mission_dist:.2f} "
            f"turns={self._turns} payload_kg={self._payload_kg:g} "
            f"soc_start={_fmt(self._soc_start, 2)} soc_end={_fmt(self._soc_end, 2)} "
            f"wh_model={_fmt(wh_model, 3)} wh_model_idle={_fmt(idle, 3)} "
            f"wh_model_drive={_fmt(drive, 3)} "
            f"wh_model_roll={_fmt(roll, 3)} wh_model_visc={_fmt(visc, 3)} "
            f"wh_model_turn={_fmt(turn, 3)} "
            f"wh_model_accel={_fmt(accel_wh, 3)} "
            f"wh_measured={measured:.3f} "
            f"wh_measured_soc={_fmt(None if soc_wh is None else round(soc_wh, 3), 3)} "
            f"err_pct={err}"
        )

    def _mission_energy_line(self) -> str:
        from src.twin_energy import format_mission_energy_line

        measured = self._mission_wh_drive + self._mission_wh_idle
        return format_mission_energy_line(
            round(self._mission_dist, 2),
            payload_kg=self._payload_kg,
            wh_measured=measured,
            duration_s=self._mission_dt,
            heading_change_deg=self._heading_accum_deg,
            wh_accel=self._mission_wh_accel,
        )


def _xy(pose: dict) -> tuple[float, float] | None:
    if not isinstance(pose, dict):
        return None
    x = _as_float(pose.get("x_m"))
    y = _as_float(pose.get("y_m"))
    if x is None or y is None:
        return None
    return (x, y)


def _heading_rad(sample: dict) -> float | None:
    pose = sample.get("pose") or {}
    if not isinstance(pose, dict):
        return None
    return _as_float(pose.get("heading_rad"))


def _subsystems(sample: dict) -> dict[str, float | None]:
    raw = sample.get("subsystem_draws") or {}
    keys = ("compute", "sensors", "cooling", "locomotion", "arms", "torso")
    if isinstance(raw, dict) and raw:
        out: dict[str, float | None] = {}
        for key in keys:
            if key not in raw or raw.get(key) is None:
                out[key] = None
            else:
                out[key] = float(raw[key])
        return out
    draws = sample.get("channel_draws") or {}
    return {
        "compute": _as_float(draws.get("Compute")),
        "sensors": None,
        "cooling": _as_float(draws.get("Cooling")),
        "locomotion": _as_float(draws.get("Legs")),
        "arms": _as_float(draws.get("Arms")),
        "torso": _as_float(draws.get("Torso")),
    }


def _drive_idle_w(sample: dict, subs: dict[str, float | None]) -> tuple[float, float]:
    loco = subs.get("locomotion")
    if loco is None:
        draws = sample.get("channel_draws") or {}
        loco = float(draws.get("Legs") or 0.0)
    else:
        loco = float(loco)
    idle = 0.0
    for key, watts in subs.items():
        if key == "locomotion" or watts is None:
            continue
        idle += float(watts)
    if idle == 0.0 and not any(k != "locomotion" and subs.get(k) is not None for k in subs):
        draws = sample.get("channel_draws") or {}
        idle = sum(float(v) for name, v in draws.items() if name != "Legs")
    return loco, idle


def resolve_log_build(stamped) -> str:
    """Build name for a log line.

    A controller that never saw ``bom_build`` stamps nothing, and the log
    used to say ``default`` next to a dashboard ``BOM build=light`` line.
    The config selection wins over a blank or default stamp.
    """
    active = None
    try:
        from src.hardware_profile import active_bom_build

        active = active_bom_build()
    except Exception:
        active = None
    text = str(stamped).strip() if stamped else ""
    if text.lower() in {"", "default", "none", "null", "yaml", "~"}:
        return active or "default"
    return text


def _accel_step_wh(v0: float, v1: float, dt: float, payload_kg: float) -> float:
    try:
        from src.hardware_profile import active_bom_build, get_active_profile
        from src.twin_energy import accel_electrical_wh

        if not active_bom_build():
            return 0.0
        return accel_electrical_wh(get_active_profile(), v0, v1, dt, payload_kg)
    except Exception:
        return 0.0


def _drive_breakdown(
    dist_m: float,
    avg_speed: float,
    heading_deg: float,
    wh_accel: float,
    payload_kg: float,
) -> dict:
    zeros = {
        "wh_roll": 0.0,
        "wh_visc": 0.0,
        "wh_turn": 0.0,
        "wh_accel": max(0.0, float(wh_accel)),
    }
    try:
        from src.hardware_profile import active_bom_build
        from src.twin_energy import segment_model_terms

        if not active_bom_build():
            return zeros
        return segment_model_terms(dist_m, avg_speed, heading_deg, wh_accel, payload_kg)
    except Exception:
        return zeros


def _model(
    dist_m: float,
    duration_s: float,
    payload_kg: float,
    heading_deg: float,
    wh_accel: float,
) -> dict | None:
    try:
        from src.hardware_profile import active_bom_build
        from src.twin_energy import bom_run_energy_wh

        if not active_bom_build():
            return None
        return bom_run_energy_wh(
            dist_m,
            duration_s,
            payload_kg,
            heading_change_deg=heading_deg,
            wh_accel=wh_accel,
        )
    except Exception:
        return None
