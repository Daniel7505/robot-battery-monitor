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
  line). Model vs measured, SoC start/end, turns. When samples carry
  ``steer_source``, the same close also prints ``STEER_SOURCE``:
  metres and time on camera, held (odometry of a rejected fit), and
  track (map distance during a committed turn). The controller stamps
  that field on every lane-keep step, so the buckets cover the lap
  and not only the finish ``stopped`` interval. Width and center
  errors (``width_err_rms_m``, ``width_err_max_m``, ``center_err_rms_m``,
  ``center_err_max_m``) are one sample per camera frame. The controller also
  writes these lines, plus ``BOM BODY`` and ``MISSION_ENERGY``, to
  ``logs/mission_summary_<ts>.json`` (repo or ``RBM_LOG_DIR``). The
  webots-bin console cannot be redirected on Windows.

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

Measured and modeled drive are different columns. Both measured
columns are electrical or mechanical on purpose. ``err_pct`` compares
the electrical ones.

* ``wh_measured_drive_shaft`` — mechanical watt-hours from the twin:
  sum of |torque feedback| × |hinge rad/s| on the drive wheels.
  The hinge rate is the position sensor. When that rate is missing
  and the body is moving, it is body speed / 0.08 m (the drawn
  wheel). The BoM radius 0.04 m is not used. If ``getTorqueFeedback``
  did not report, this field is ``na``. Rolling resistance is not
  written in its place, and neither is ``|ω| × 0.45``.
* ``wh_measured_drive_elec`` — those same hinge points converted to
  pack-side electrical watt-hours. Each motor uses the published
  brushed line (duty = |ω|/ω0 + τ/τstall, I = I0 + (Istall−I0)·τ/τstall,
  P = duty·V·I / η_driver). On the light build the motor is Pololu
  4693: 24 V, 200 rpm and 0.1 A no-load, 3.0 A and 2.256 N·m stall
  (https://www.pololu.com/product/4693/specs). Kt = 2.256/3.0 =
  0.752 N·m/A and R = 24/3 = 8 Ω are derived; bom.json does not
  publish them, and they are not a circuit that also hits 200 rpm
  (that circuit predicts about 295 rpm). The conversion therefore
  uses the interpolation above, not Kt and R as a winding model.
  Datasheet peak efficiency 0.5 is not the operating-point η.
  Gear ratio 50 is already in the output-shaft figures and is not
  applied again. Driver η is the 0.92 assumption, applied once.
  Hinge rad/s is not rewritten as body speed / 0.04 m. Missing
  feedback is ``na``, not the rolling prediction.
* ``eta_avg`` — ``wh_measured_drive_shaft / wh_measured_drive_elec``
  over the piece (energy-weighted). ``na`` when electrical energy
  is missing or zero.
* ``wh_model_roll`` — predicted brushed-DC electrical energy at
  rolling resistance. On the light build that is about 0.0066 Wh
  per 10 ft near 0.44 m/s. The design shift table uses only this.
* ``wh_model_visc`` — extra electrical energy from the published
  hinge damper. ``dampingConstant`` 0.15 is N·m·s/rad (Webots
  JointParameters): each drive hinge gets ``τ = −c·ω`` with
  ``ω = v / 0.08``, in addition to the motor torque. That torque is
  converted at the same hinge rate. The rolling design bill stays at
  ``ω = v / 0.04``, and this column is the difference, so the printed
  roll number does not move. ``getTorqueFeedback`` is motor effort and
  does not add the damper on top of that effort, so the measured shaft
  already contains the load once. This column is not subtracted from
  measured torque. It rises with speed. It is not a coefficient fitted
  to a log.
* ``wh_model_turn`` — extra electrical watt-hours while yawing.
  Each hub sits half the axle track off the chassis centre. Its
  ground-speed offset is ``(track/2)·|ω|``. The world contact
  (``butlerbot_warehouse.wbt``, drive_wheel vs default) uses ODE
  force-dependent slip ``v = k·F`` with ``k = 0.01``, capped by
  ``μ = 8``. That force times the 0.08 m wheel is shaft torque on
  the same Pololu 4693 line as ``wh_measured_drive_elec``. Driver
  efficiency 0.92 is inside that line, once. ``μ = 8`` is the cap,
  not a skid coefficient: the world comment says μ=80 made yaw
  impossible, and ``frictionRotation`` is 0 so bore torque is not
  billed. ASSUMPTION: both drive wheels carry the weight, because
  the caster bottoms are about 4 mm above the tire. Straight
  segments stay at the rolling-plus-damping line. The 11–39%
  straight residual is not given a fitted coefficient.
* ``wh_model_accel`` — predicted extra motor-line watts while
  speeding up (``m·a`` on top of rolling torque). Slowing down is
  zero (no regen).
* ``wh_model_drive`` — ``wh_model_roll + wh_model_visc + wh_model_turn + wh_model_accel``.
* ``err_pct`` — ``(wh_measured_drive_elec − wh_model_drive) / wh_model_drive × 100``.
  ``na`` when the electrical measurement is missing or the model
  drive is 0. Shaft watts are not subtracted from the electrical
  model. A live light warehouse lap (commit 16646d5) printed
  err_pct=−69.1 because it compared 1.206 Wh of shaft to 3.909 Wh
  of electrical model.

Dashboard Legs stay the rolling electrical prediction (about 3.43 W
at 0.44 m/s) so the power allocator keeps the BoM bill. That channel
is not ``wh_measured_drive_elec``.

Why ``wh_measured`` and ``wh_measured_soc`` disagreed on that lap
----------------------------------------------------------------
The summary was duration 919.1 s, 200.91 m, SoC 99.60% → 98.66%,
``wh_measured_drive`` 1.206 (shaft), ``wh_measured`` 3.724,
``wh_measured_soc`` 2.256, model drive 3.909 (roll 0.441 + visc
3.461 + accel 0.007), idle 2.543.

``wh_measured`` was shaft plus the non-drive channels:
1.206 + 2.518 = 3.724. The 2.518 Wh is about 9.96 W × 919.1 s, the
published electronics idle. Shaft watts are mechanical. Adding them
to electrical idle mixes two units.

``wh_measured_soc`` is only the pack percent times the 240 Wh pack:
240 × (99.60 − 98.66) / 100 = 2.256 exactly. The controller did not
integrate shaft watts. On a BoM build it integrated ``total_draw_w``
from ``build_payload``: published electronics plus the rolling-only
motor line (feedback torque forced to 0, so viscous hinge torque was
not in the drain), multiplied by the simulation timestep
(``timestep/1000``), against the BoM pack. Idle plus rolling on the
log's wall clock is 2.543 + 0.441 = 2.984 Wh, which would be a 1.24
point drop, not the 0.94 points observed. The log's clock is the gap
between telemetry timestamps. Those two integrals are not the same
load and not the same clock.

``wh_measured`` is electrical drive plus the same non-drive channels.
The controller integrates that pack draw on the simulation step
(``SimPackMeter``) and posts the running totals
(``energy_wh_shaft``, ``energy_wh_elec``, ``energy_wh_idle``,
``energy_wh_pack``, ``energy_sim_s``). This log bills the positive
delta. A late, coalesced, or dropped POST cannot lose watt-hours,
and the percent is drained from the same step, so
``wh_measured_soc`` matches ``wh_measured``. A sample with no
feedback adds no drive watts for that step (``energy_elec_gaps``)
and does not substitute the rolling line.

Run #11 (fc55ab7, HUD paint off) split the two totals again:
``wh_measured`` 6.381 and ``wh_measured_soc`` 7.224, which is
``240 × (100 − 96.99) / 100``. The 0.843 Wh is about 101 s at the
log's average pack power (~30 W). Telemetry was arriving about
every 3 s. This log integrated the timestamp gap and dropped any
gap over 15 s. The controller drained every physics step's wall
gap, including each gap under 15 s while a POST was failing or the
dashboard had just restarted. Those seconds moved the percent and
never entered ``wh_measured``. Runs #8 and #9 matched within 0.1%
because no such hole landed between arm and summary.

The same wall clock made HUD paint look like an energy change.
Run #9 (HUD on, 0.70× realtime, 896 s) measured 5.075 Wh of drive.
Run #11 (HUD off, 765 s) measured 4.287 Wh. Instantaneous drive was
about 20 W either way; the watt-hours scaled with wall duration.
Both clocks are now simulation time, so a faster realtime factor
does not shrink the bill. ``TELEMETRY_GAPS`` still counts intervals,
the mean and max gap, seconds the old 15 s rule would skip, and the
watt-hours in that skipped time. ``cumulative=1`` means those
watt-hours were kept via the controller counter.

A new ``controller_run_id`` closes the previous mission. Webots
restarted without a dashboard restart used to keep the old log, so
a parked stretch (run #10, segment 65, about 1500 s) never got a
``MISSION_SUMMARY`` and the next lap inherited it. A real previous
mission is summarized, then the log arms on the new id. A trivial
previous run (under 0.5 m and under 2 s) is dropped without a
summary, same as a killed process.

A full warehouse run (no finish; stopped by hand) checked the
prediction: 121 segments, 369 m, 2145 s. The old locomotion integral
(hinge torque run through the electrical line) averaged 0.120 Wh per
10 ft (sd 0.023), correlation with heading −0.11 and with speed
+0.82. Speed bins, Wh per segment: under 0.15 m/s → 0.096;
0.15–0.25 → 0.103; 0.25–0.35 → 0.126; over 0.35 → 0.153. Totals
from that integral: drive 14.5 Wh, idle 5.9 Wh. Idle matches
9.96 W × 2145 s. Rolling-only does not: it is about 0.8 Wh and it
falls slightly as speed rises. Published damping, converted at the
hinge rate ``v / 0.08``, is about 4.2 Wh together with rolling at the
run's average speed, short of 14.5 Wh.
The remainder is sim tire contact. That torque is not one published
number, and a straight-line fit of the log is not pasted in as a
bill. That 0.120 Wh figure is not copied into
``wh_measured_drive_shaft`` or ``wh_measured_drive_elec``.
The shaft column is the hinge's own |τ|·|ω|. The electrical column
converts that hinge point; it does not reuse the old 0.120 fit.

``wh_measured_soc`` is ``capacity × (start% − end%) / 100``.

A ``mission_event=complete`` with essentially no distance and no
duration is not a mission (a Webots process killed on startup). It
does not log a summary, and it does not consume the one summary.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone

from src.teleop_agent import plausible_travel_m
from src.twin_energy import ENERGY_COUNTER_KEYS, TEN_FT_M

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
# Wheel-command source for one sample. ``held`` is the odometry copy of
# a rejected camera fit (0.30 m/s). ``track`` is map distance during a
# turn that is already committed. A straight with a track loaded is not
# ``track``.
_STEER_KEYS = (
    "camera",
    "held",
    "track",
    "pivot",
    "recover",
    "blind",
    "stopped",
    "other",
)


def _has_energy_counter(sample: dict) -> bool:
    return any(key in sample for key in ENERGY_COUNTER_KEYS)


def _counter_went_backwards(deltas: dict) -> bool:
    """A new controller whose counters were not tagged with a new run id."""
    sim = deltas.get("energy_sim_s")
    pack = deltas.get("energy_wh_pack")
    if sim is not None and float(sim) < -1e-4:
        return True
    if pack is not None and float(pack) < -1e-4:
        return True
    return False


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


def _na(value: float | None, digits: int) -> str:
    """A missing measurement is the word na, not a model number."""
    if value is None:
        return "na"
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
        self._mission_wh_shaft = 0.0
        self._mission_shaft_known = True
        self._mission_wh_elec = 0.0
        self._mission_elec_known = True
        self._mission_wh_idle = 0.0
        self._mission_wh_accel = 0.0
        self._mission_wh_turn = 0.0
        self._clear_segment()
        self._clear_power()
        self._run_id = ""
        self._cum: dict | None = None
        self._used_cumulative = False
        self._n_intervals = 0
        self._interval_sum = 0.0
        self._interval_max = 0.0
        self._skipped_s = 0.0
        self._skipped_wh = 0.0
        self._steer_split = ""
        self._elec_gaps: int | None = None
        self._steer_m = {key: 0.0 for key in _STEER_KEYS}
        self._steer_s = {key: 0.0 for key in _STEER_KEYS}
        self._ct_max: float | None = None
        self._width_err: list[float] = []
        self._center_err: list[float] = []
        self._true_width: list[float] = []
        self._fit_seen: set = set()

    def _clear_segment(self) -> None:
        self._seg_idx = 0
        self._seg_dist = 0.0
        self._seg_dt = 0.0
        self._seg_wh_shaft = 0.0
        self._seg_shaft_known = True
        self._seg_wh_elec = 0.0
        self._seg_elec_known = True
        self._seg_wh_idle = 0.0
        self._seg_wh_accel = 0.0
        self._seg_wh_turn = 0.0
        self._seg_heading_deg = 0.0
        self._seg_accel = 0

    def _clear_power(self) -> None:
        self._power_dt = 0.0
        self._power_acc: dict[str, float] = {}
        self._power_seen: dict[str, bool] = {}
        self._power_loco_gap = False

    def observe(self, sample: dict) -> list[str]:
        """Ingest one telemetry sample. Return log lines to emit (may be empty)."""
        ts = sample.get("timestamp")
        if not isinstance(ts, datetime):
            ts = datetime.now(timezone.utc)
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)

        lines: list[str] = []
        run_id = str(sample.get("controller_run_id") or "").strip()
        if self._armed and run_id and self._run_id and run_id != self._run_id:
            # Webots restarted. Summarize the open mission, then arm the new id.
            lines.extend(self._summarize_open_mission())
            self.reset()
        if not self._armed:
            self._arm(sample, ts)
            self._run_id = run_id
            self._remember_cumulative(sample)
            self._remember_steer(sample)
            self._note_cross_track(sample)
            self._note_lane_fit(sample)
            return lines

        wall_dt = (ts - self._last_ts).total_seconds() if self._last_ts else 0.0
        self._last_ts = ts
        deltas = self._take_deltas(sample)
        if deltas is not None and _counter_went_backwards(deltas):
            lines.extend(self._summarize_open_mission())
            self.reset()
            self._arm(sample, ts)
            self._run_id = run_id
            self._remember_cumulative(sample)
            self._remember_steer(sample)
            self._note_cross_track(sample)
            self._note_lane_fit(sample)
            return lines
        self._note_cross_track(sample)
        self._note_lane_fit(sample)
        self._note_interval(wall_dt, sample, deltas)
        self._remember_steer(sample)

        if deltas is not None:
            self._used_cumulative = True
            sim_dt = deltas.get("energy_sim_s")
            dt = 0.0 if sim_dt is None else max(0.0, float(sim_dt))
        elif self._used_cumulative:
            # Counters disappeared. Do not also bill timestamp watts.
            self._note_pose(sample)
            lines.extend(self._maybe_summary(sample))
            return lines
        else:
            dt = wall_dt
            if dt <= 0 or dt > MAX_DT_S:
                self._note_pose(sample)
                lines.extend(self._maybe_summary(sample))
                return lines

        if dt <= 0:
            # Duplicate post, or sim time did not advance. Keep the pose
            # anchor and do not bill a wall gap the counter did not.
            self._note_pose(sample)
            self._note_soc(sample)
            lines.extend(self._maybe_summary(sample))
            return lines

        step_m, speed, heading_deg, trusted = self._step_motion(sample, dt)
        subs = _subsystems(sample)
        shaft_w, shaft_known = _metered_w(sample, "measured_drive_w")
        elec_w, elec_known = _metered_w(sample, "measured_drive_elec_w")
        if "measured_drive_elec_w" in sample:
            # Breakdown locomotion is pack-side electrical drive, or na.
            subs["locomotion"] = elec_w
            if not elec_known:
                self._power_loco_gap = True
        elif "measured_drive_w" in sample:
            subs["locomotion"] = shaft_w
            if not shaft_known:
                self._power_loco_gap = True
        idle_w = _idle_w(sample, subs)
        if deltas is not None:
            d_shaft = deltas.get("energy_wh_shaft")
            d_elec = deltas.get("energy_wh_elec")
            d_idle = deltas.get("energy_wh_idle")
            shaft_known = d_shaft is not None
            elec_known = d_elec is not None
            wh_shaft = 0.0 if d_shaft is None else max(0.0, float(d_shaft))
            wh_elec = 0.0 if d_elec is None else max(0.0, float(d_elec))
            wh_idle = 0.0 if d_idle is None else max(0.0, float(d_idle))
            if not elec_known:
                self._power_loco_gap = True
        else:
            wh_shaft = 0.0 if shaft_w is None else shaft_w * dt / 3600.0
            wh_elec = 0.0 if elec_w is None else elec_w * dt / 3600.0
            wh_idle = idle_w * dt / 3600.0
        accel = 0
        wh_accel_step = 0.0
        if (
            dt > 0
            and self._last_speed is not None
            and abs(speed - self._last_speed) / dt >= ACCEL_EVENT_M_S2
        ):
            accel = 1
        if trusted and self._last_speed is not None and speed > self._last_speed:
            wh_accel_step = _accel_step_wh(self._last_speed, speed, dt, self._payload_kg)
        wh_turn_step = 0.0
        if trusted:
            wh_turn_step = _turn_step_wh(speed, heading_deg, dt, self._payload_kg)
        self._last_speed = speed

        self._mission_dt += dt
        self._mission_dist += step_m
        if shaft_known:
            self._mission_wh_shaft += wh_shaft
        else:
            self._mission_shaft_known = False
        if elec_known:
            self._mission_wh_elec += wh_elec
        else:
            self._mission_elec_known = False
        self._mission_wh_idle += wh_idle
        self._mission_wh_accel += wh_accel_step
        self._mission_wh_turn += wh_turn_step
        self._note_steer_motion(sample, step_m if trusted else 0.0, dt)
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
            if shaft_known:
                self._seg_wh_shaft += wh_shaft
            else:
                self._seg_shaft_known = False
            if elec_known:
                self._seg_wh_elec += wh_elec
            else:
                self._seg_elec_known = False
            self._seg_wh_idle += wh_idle
            self._seg_wh_accel += wh_accel_step
            self._seg_wh_turn += wh_turn_step
            self._seg_heading_deg += abs(heading_deg)
            if accel:
                self._seg_accel += accel
        # One line per sample, and only when this sample actually moved.
        # A pose jump used to close several segments in the same millisecond.
        if trusted and step_m > 0 and self._seg_dist >= TEN_FT_M - 1e-9 and self._seg_dist > 0:
            frac = min(1.0, TEN_FT_M / self._seg_dist)
            closed_dist = self._seg_dist
            closed_dt = self._seg_dt
            closed_shaft = self._seg_wh_shaft
            closed_elec = self._seg_wh_elec
            closed_idle = self._seg_wh_idle
            closed_accel = self._seg_wh_accel
            closed_turn = self._seg_wh_turn
            closed_head = self._seg_heading_deg
            self._seg_dist = closed_dist * frac
            self._seg_dt = closed_dt * frac
            self._seg_wh_shaft = closed_shaft * frac
            self._seg_wh_elec = closed_elec * frac
            closed_shaft_known = self._seg_shaft_known
            closed_elec_known = self._seg_elec_known
            self._seg_wh_idle = closed_idle * frac
            self._seg_wh_accel = closed_accel * frac
            self._seg_wh_turn = closed_turn * frac
            self._seg_heading_deg = closed_head * frac
            lines.append(
                self._segment_line(
                    shaft_known=closed_shaft_known,
                    elec_known=closed_elec_known,
                )
            )
            self._seg_idx += 1
            keep = 1.0 - frac
            self._seg_dist = closed_dist * keep
            self._seg_dt = closed_dt * keep
            self._seg_wh_shaft = closed_shaft * keep
            self._seg_wh_elec = closed_elec * keep
            # A sample with no torque feedback poisons the piece it
            # straddles, including the leftover that opens the next one.
            self._seg_shaft_known = closed_shaft_known
            self._seg_elec_known = closed_elec_known
            self._seg_wh_idle = closed_idle * keep
            self._seg_wh_accel = closed_accel * keep
            self._seg_wh_turn = closed_turn * keep
            self._seg_heading_deg = closed_head * keep
            self._seg_accel = 0

        lines.extend(self._maybe_summary(sample))
        return lines

    def _remember_cumulative(self, sample: dict) -> None:
        if not _has_energy_counter(sample):
            return
        self._cum = {key: _as_float(sample.get(key)) for key in ENERGY_COUNTER_KEYS}
        self._used_cumulative = True
        gaps = _as_float(sample.get("energy_elec_gaps"))
        if gaps is not None:
            self._elec_gaps = int(gaps)

    def _take_deltas(self, sample: dict) -> dict | None:
        """Positive change in the controller counters since the last sample.

        The first time the counters show up, that sample is the baseline
        and the delta is zero. Billing it would count energy from the
        start of the process, which the percent has already absorbed.
        """
        if not _has_energy_counter(sample):
            return None
        cur = {key: _as_float(sample.get(key)) for key in ENERGY_COUNTER_KEYS}
        gaps = _as_float(sample.get("energy_elec_gaps"))
        if gaps is not None:
            self._elec_gaps = int(gaps)
        if self._cum is None:
            self._cum = cur
            self._used_cumulative = True
            return {key: (0.0 if cur[key] is not None else None) for key in ENERGY_COUNTER_KEYS}
        deltas = {}
        for key in ENERGY_COUNTER_KEYS:
            prev = self._cum.get(key)
            value = cur.get(key)
            if prev is None or value is None:
                deltas[key] = None
            else:
                deltas[key] = float(value) - float(prev)
        self._cum = cur
        return deltas

    def _remember_steer(self, sample: dict) -> None:
        text = sample.get("steer_split")
        if text:
            self._steer_split = str(text).strip()

    def _note_steer_motion(self, sample: dict, step_m: float, dt: float) -> None:
        raw = sample.get("steer_source")
        if raw is None or str(raw).strip() == "":
            return
        name = str(raw).strip().lower()
        if name not in self._steer_m:
            name = "other"
        self._steer_m[name] += max(0.0, float(step_m))
        self._steer_s[name] += max(0.0, float(dt))

    def _note_cross_track(self, sample: dict) -> None:
        """Largest |sideways error| this lap, from the referee centerline."""
        vals = []
        peak = _as_float(sample.get("ct_max_m"))
        if peak is not None:
            vals.append(abs(peak))
        inst = _as_float(sample.get("cross_track_m"))
        if inst is not None:
            vals.append(abs(inst))
        if not vals:
            return
        got = max(vals)
        self._ct_max = got if self._ct_max is None else max(self._ct_max, got)

    def note_lane_frame(self, fields: dict) -> None:
        """Record one new camera frame. A later post of the same frame is ignored."""
        self._note_lane_fit(fields)

    def _note_lane_fit(self, sample: dict) -> None:
        """One camera frame's width error and center error, when the controller sent them.

        ``lane_frame_n`` dedupes. Telemetry is slower than the camera, and
        the same frame used to be either missed (not on a post) or counted
        again on the next post.
        """
        frame = sample.get("lane_frame_n")
        if frame is not None and frame != "":
            try:
                key: object = int(frame)
            except (TypeError, ValueError):
                key = str(frame)
            if key in self._fit_seen:
                return
            self._fit_seen.add(key)
        width = _as_float(sample.get("width_err_m"))
        if width is not None:
            self._width_err.append(width)
        center = _as_float(sample.get("center_err_m"))
        if center is not None:
            self._center_err.append(center)
        true_w = _as_float(sample.get("true_width_m"))
        if true_w is not None:
            self._true_width.append(true_w)

    def _fit_err_suffix(self) -> str:
        def pack(name: str, vals: list[float]) -> str:
            if not vals:
                return ""
            rms = math.sqrt(sum(v * v for v in vals) / len(vals))
            peak = max(abs(v) for v in vals)
            return f" {name}_rms_m={rms:.4f} {name}_max_m={peak:.4f} {name}_n={len(vals)}"

        span = ""
        if self._true_width:
            span = (
                f" true_width_min_m={min(self._true_width):.4f}"
                f" true_width_max_m={max(self._true_width):.4f}"
                f" true_width_n={len(self._true_width)}"
            )
        return pack("width_err", self._width_err) + pack("center_err", self._center_err) + span

    def _steer_source_line(self) -> str | None:
        total_m = sum(self._steer_m.values())
        total_s = sum(self._steer_s.values())
        if total_m <= 1e-6 and total_s <= 1e-6:
            return None

        def frac(part: float, total: float) -> str:
            if total <= 1e-6:
                return "na"
            return f"{part / total:.3f}"

        metres = " ".join(f"{key}_m={self._steer_m[key]:.2f}" for key in _STEER_KEYS)
        seconds = " ".join(f"{key}_s={self._steer_s[key]:.1f}" for key in _STEER_KEYS)
        return (
            f"STEER_SOURCE dist_m={total_m:.2f} time_s={total_s:.1f} "
            f"{metres} {seconds} "
            f"frac_camera={frac(self._steer_m['camera'], total_m)} "
            f"frac_held={frac(self._steer_m['held'], total_m)} "
            f"frac_track={frac(self._steer_m['track'], total_m)} "
            f"frac_camera_t={frac(self._steer_s['camera'], total_s)} "
            f"frac_held_t={frac(self._steer_s['held'], total_s)} "
            f"frac_track_t={frac(self._steer_s['track'], total_s)}"
            + ("" if self._ct_max is None else f" ct_max_m={self._ct_max:.3f}")
        )

    def _note_interval(self, wall_dt: float, sample: dict, deltas: dict | None) -> None:
        """Count one telemetry gap, including one the old 15 s rule skips."""
        if wall_dt <= 0.0:
            return
        self._n_intervals += 1
        self._interval_sum += wall_dt
        if wall_dt > self._interval_max:
            self._interval_max = wall_dt
        if wall_dt <= MAX_DT_S:
            return
        self._skipped_s += wall_dt
        pack = None if deltas is None else deltas.get("energy_wh_pack")
        if pack is None:
            elec_w, _known = _metered_w(sample, "measured_drive_elec_w")
            idle = _idle_w(sample, _subsystems(sample))
            drive = 0.0 if elec_w is None else float(elec_w)
            pack = (drive + idle) * wall_dt / 3600.0
        self._skipped_wh += max(0.0, float(pack))

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

    def _segment_line(
        self,
        shaft_known: bool | None = None,
        elec_known: bool | None = None,
    ) -> str:
        avg = (self._seg_dist / self._seg_dt) if self._seg_dt > 0 else 0.0
        if shaft_known is None:
            shaft_known = self._seg_shaft_known
        if elec_known is None:
            elec_known = self._seg_elec_known
        shaft = self._seg_wh_shaft if shaft_known else None
        elec = self._seg_wh_elec if elec_known else None
        terms = _drive_breakdown(
            self._seg_dist,
            avg,
            self._seg_heading_deg,
            self._seg_wh_accel,
            self._payload_kg,
        )
        # Per-sample integral. Averaging the segment's yaw under-bills a
        # corner that occupies only part of the 10 ft.
        roll = round(float(terms["wh_roll"]), 4)
        visc = round(float(terms["wh_visc"]), 4)
        turn = round(self._seg_wh_turn, 4)
        accel = round(float(terms["wh_accel"]), 4)
        model_drive = round(roll + visc + turn + accel, 4)
        err = _err_pct(elec, model_drive)
        eta = _eta(shaft if shaft_known else None, elec)
        total = None if elec is None else elec + self._seg_wh_idle
        return (
            f"SEGMENT_ENERGY build={self._build} seg_idx={self._seg_idx} "
            f"dist_m={self._seg_dist:.3f} dt_s={self._seg_dt:.2f} "
            f"avg_speed={avg:.3f} heading_change_deg={self._seg_heading_deg:.1f} "
            f"accel_events={self._seg_accel} payload_kg={self._payload_kg:g} "
            f"wh_measured_drive_shaft={_na(shaft, 4)} "
            f"wh_measured_drive_elec={_na(elec, 4)} "
            f"eta_avg={eta} "
            f"wh_model_drive={model_drive:.4f} "
            f"wh_model_roll={roll:.4f} "
            f"wh_model_visc={visc:.4f} "
            f"wh_model_turn={turn:.4f} "
            f"wh_model_accel={accel:.4f} "
            f"err_pct={err} "
            f"wh_idle={self._seg_wh_idle:.4f} "
            f"wh_total={_na(total, 4)}"
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
        if self._power_loco_gap:
            values["locomotion"] = None
            numeric = sum(v for v in values.values() if v is not None)
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
        return self._summary_bundle()

    def _summarize_open_mission(self) -> list[str]:
        """Close a real mission that never posted ``complete``.

        A Webots restart used to leave the dashboard log armed, so the
        next lap inherited a parked stretch and never printed a summary.
        """
        if self._summarized:
            return []
        if (
            self._mission_dist < MIN_SUMMARY_DIST_M
            and self._mission_dt < MIN_SUMMARY_DURATION_S
        ):
            return []
        self._summarized = True
        return self._summary_bundle()

    def _summary_bundle(self) -> list[str]:
        lines = [self._summary_line(), self._mission_energy_line(), self._gaps_line()]
        steer = self._steer_source_line()
        if steer:
            lines.append(steer)
        if self._steer_split:
            lines.append(self._steer_split)
        return lines

    def _gaps_line(self) -> str:
        n = self._n_intervals
        mean = (self._interval_sum / n) if n else 0.0
        sim = self._mission_dt if self._used_cumulative else None
        gaps = "" if self._elec_gaps is None else f" elec_gaps={self._elec_gaps}"
        return (
            f"TELEMETRY_GAPS n={n} mean_s={mean:.3f} max_s={self._interval_max:.3f} "
            f"skipped_s={self._skipped_s:.3f} skipped_wh={self._skipped_wh:.3f} "
            f"wall_s={self._interval_sum:.3f} "
            f"sim_s={_na(None if sim is None else round(sim, 3), 3)} "
            f"cumulative={'1' if self._used_cumulative else '0'}"
            f"{gaps}"
        )

    def _summary_line(self) -> str:
        shaft = self._mission_wh_shaft if self._mission_shaft_known else None
        elec = self._mission_wh_elec if self._mission_elec_known else None
        # Electrical total: measured drive at the pack plus non-drive channels.
        # Shaft watts are logged beside it and are not added into this sum.
        measured = None if elec is None else elec + self._mission_wh_idle
        model = _model(
            self._mission_dist,
            self._mission_dt,
            self._payload_kg,
            self._heading_accum_deg,
            self._mission_wh_accel,
        )
        idle = None if model is None else model["wh_model_idle"]
        roll = None if model is None else model.get("wh_model_roll")
        visc = None if model is None else model.get("wh_model_visc")
        accel_wh = None if model is None else model.get("wh_model_accel")
        # Sample integral, not yaw averaged over the whole lap.
        turn = round(self._mission_wh_turn, 3)
        drive = None
        wh_model = None
        if model is not None:
            drive = round(
                float(roll or 0.0) + float(visc or 0.0) + turn + float(accel_wh or 0.0),
                3,
            )
            wh_model = round(float(idle or 0.0) + drive, 3)
        soc_wh = None
        if (
            self._soc_start is not None
            and self._soc_end is not None
            and self._capacity_wh
        ):
            soc_wh = max(0.0, self._soc_start - self._soc_end) * self._capacity_wh / 100.0
        err = _err_pct(elec, drive)
        eta = _eta(shaft, elec)
        steer_m = sum(self._steer_m.values())
        steer = ""
        if steer_m > 1e-6:
            steer = (
                f" steer_frac_camera={self._steer_m['camera'] / steer_m:.3f}"
                f" steer_frac_held={self._steer_m['held'] / steer_m:.3f}"
                f" steer_frac_track={self._steer_m['track'] / steer_m:.3f}"
            )
        if self._ct_max is not None:
            steer += f" ct_max_m={self._ct_max:.3f}"
        steer += self._fit_err_suffix()
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
            f"wh_measured_drive_shaft={_na(None if shaft is None else round(shaft, 3), 3)} "
            f"wh_measured_drive_elec={_na(None if elec is None else round(elec, 3), 3)} "
            f"eta_avg={eta} "
            f"wh_measured={_na(None if measured is None else round(measured, 3), 3)} "
            f"wh_measured_soc={_fmt(None if soc_wh is None else round(soc_wh, 3), 3)} "
            f"err_pct={err}"
            f"{steer}"
        )

    def _mission_energy_line(self) -> str:
        """``MISSION_ENERGY`` err_pct uses total measured over total model.

        ``MISSION_SUMMARY`` err_pct (``_err_pct``) is drive electrical over
        model drive only, so the two percentages on one lap are different
        on purpose: idle sits in the energy line and not in the summary
        denominator.
        """
        from src.twin_energy import format_mission_energy_line

        measured = None if not self._mission_elec_known else (
            self._mission_wh_elec + self._mission_wh_idle
        )
        return format_mission_energy_line(
            round(self._mission_dist, 2),
            payload_kg=self._payload_kg,
            wh_measured=measured,
            duration_s=self._mission_dt,
            heading_change_deg=self._heading_accum_deg,
            wh_accel=self._mission_wh_accel,
            wh_turn=self._mission_wh_turn,
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


def _metered_w(sample: dict, key: str) -> tuple[float | None, bool]:
    """A measured watt field, and whether this sample actually reported it.

    The key has to be present. A present ``None`` is a gap (torque
    feedback missing) and is not a cue to use channel Legs. Samples
    from before the field existed also have no measurement.
    """
    if key not in sample:
        return None, False
    value = _as_float(sample.get(key))
    return value, value is not None


def _err_pct(elec_wh: float | None, model_drive_wh: float | None) -> str:
    """Electrical measured versus electrical model. Shaft is not the error."""
    if elec_wh is None or model_drive_wh is None or abs(float(model_drive_wh)) <= 1e-12:
        return "na"
    return f"{(float(elec_wh) - float(model_drive_wh)) / float(model_drive_wh) * 100.0:.1f}"


def _eta(shaft_wh: float | None, elec_wh: float | None) -> str:
    """Energy-weighted efficiency. Zero electrical energy is na, not 0."""
    if shaft_wh is None or elec_wh is None or abs(float(elec_wh)) <= 1e-12:
        return "na"
    return f"{float(shaft_wh) / float(elec_wh):.3f}"


def _idle_w(sample: dict, subs: dict[str, float | None]) -> float:
    """Non-drive channels. Locomotion is not added here."""
    idle = 0.0
    for key, watts in subs.items():
        if key == "locomotion" or watts is None:
            continue
        idle += float(watts)
    if idle == 0.0 and not any(k != "locomotion" and subs.get(k) is not None for k in subs):
        draws = sample.get("channel_draws") or {}
        idle = sum(float(v) for name, v in draws.items() if name != "Legs")
    return idle


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


def _turn_step_wh(speed: float, heading_deg: float, dt: float, payload_kg: float) -> float:
    try:
        from src.hardware_profile import active_bom_build, get_active_profile
        from src.twin_energy import turning_scrub_wh

        if not active_bom_build():
            return 0.0
        return turning_scrub_wh(
            get_active_profile(), speed, heading_deg, dt, payload_kg
        )
    except Exception:
        return 0.0


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
