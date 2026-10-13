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
all use the published idle plus the rolling-resistance motor model, and
the controller drains that same total against the BoM pack. ``wh_model``
is idle over the whole run plus drive for the distance. It still has no
turn or acceleration term; ``err_pct`` is the difference from the
integrated channel watts (``wh_measured``). ``wh_measured_soc`` is
``capacity × (start% − end%) / 100`` so a remaining pack mismatch stays
visible.
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
# Net heading change that counts as one turn.
TURN_DEG = 30.0
# Same stall guard as the efficiency tracker: do not bill a paused Webots.
MAX_DT_S = 15.0


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
        self._turn_anchor: float | None = None
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
        self._clear_segment()
        self._clear_power()

    def _clear_segment(self) -> None:
        self._seg_idx = 0
        self._seg_dist = 0.0
        self._seg_dt = 0.0
        self._seg_wh_drive = 0.0
        self._seg_wh_idle = 0.0
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
        if self._last_speed is not None and abs(speed - self._last_speed) / dt >= ACCEL_EVENT_M_S2:
            accel = 1
        self._last_speed = speed

        self._mission_dt += dt
        self._mission_dist += step_m
        self._mission_wh_drive += wh_drive
        self._mission_wh_idle += wh_idle
        self._note_soc(sample)
        if trusted:
            self._note_turn(sample)

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
            closed_head = self._seg_heading_deg
            self._seg_dist = closed_dist * frac
            self._seg_dt = closed_dt * frac
            self._seg_wh_drive = closed_drive * frac
            self._seg_wh_idle = closed_idle * frac
            self._seg_heading_deg = closed_head * frac
            lines.append(self._segment_line())
            self._seg_idx += 1
            keep = 1.0 - frac
            self._seg_dist = closed_dist * keep
            self._seg_dt = closed_dt * keep
            self._seg_wh_drive = closed_drive * keep
            self._seg_wh_idle = closed_idle * keep
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
            self._turn_anchor = heading
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

    def _note_turn(self, sample: dict) -> None:
        heading = _heading_rad(sample)
        if heading is None or self._turn_anchor is None:
            if heading is not None:
                self._turn_anchor = heading
            return
        delta_deg = abs(math.degrees(_wrap_rad(heading - self._turn_anchor)))
        if delta_deg >= TURN_DEG:
            self._turns += 1
            self._turn_anchor = heading

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
        return (
            f"SEGMENT_ENERGY build={self._build} seg_idx={self._seg_idx} "
            f"dist_m={self._seg_dist:.3f} dt_s={self._seg_dt:.2f} "
            f"avg_speed={avg:.3f} heading_change_deg={self._seg_heading_deg:.1f} "
            f"accel_events={self._seg_accel} payload_kg={self._payload_kg:g} "
            f"wh_drive={self._seg_wh_drive:.4f} wh_idle={self._seg_wh_idle:.4f} "
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
        self._summarized = True
        self._note_soc(sample)
        return [self._summary_line(), self._mission_energy_line()]

    def _summary_line(self) -> str:
        measured = self._mission_wh_drive + self._mission_wh_idle
        model = _model(self._mission_dist, self._mission_dt, self._payload_kg)
        wh_model = None if model is None else model["wh_model"]
        idle = None if model is None else model["wh_model_idle"]
        drive = None if model is None else model["wh_model_drive"]
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
            f"wh_model_drive={_fmt(drive, 3)} wh_measured={measured:.3f} "
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


def _model(dist_m: float, duration_s: float, payload_kg: float) -> dict | None:
    try:
        from src.hardware_profile import active_bom_build
        from src.twin_energy import bom_run_energy_wh

        if not active_bom_build():
            return None
        return bom_run_energy_wh(dist_m, duration_s, payload_kg)
    except Exception:
        return None
