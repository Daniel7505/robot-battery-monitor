# Optimizer knowledge

Notes for the overnight loop. The seeded facts below are the standing
picture of the energy model. The loop appends a short learning after
each try under **Learnings**. It does not rewrite the seeded facts.

The loop may change only the tuning keys in `config/config.yaml`
(`controller.cruise_m_s`, `controller.pivot_w_max_rad_s`,
`controller.k_steer`). `bom_build` stays **light** for the whole run.

**Step sizes (warehouse noise):** lap evidence needs roughly **>1.5 pp**
on |err|, **>1.5%** on Wh/m, or a turn-segment move above the segment
noise band (~2.5 pp). The allowlisted steps are therefore coarse:
`controller.pivot_w_max_rad_s` **0.30** rad/s (e.g. 0.9→0.6),
`controller.cruise_m_s` **±0.06** m/s, `controller.k_steer` **±0.5**.
Smaller grid steps were scoring as 1.000 and could not clear the band.
Repeating the same knob is reserved for invalid laps (one retry), not for
noise-band inconclusive trials.

**Scoring baseline:** each search trial is scored against the **kept
baseline** (starting settings until a keep, then the provisional kept lap).
The starting reference lap is not reused as the score denominator after a
keep.

**Held-out check:** the Webots warehouse twin is deterministic (identical
`ctrl_*` gives identical laps). A same-build repeat lap is not used to
confirm a keep. After a tentative keep, the loop runs one lap on the
**heavy** BoM build with the same controller knobs, then restores
`bom_build: light`. That checks the settings are sane on another body, not
a duplicate light lap.

**Evidence bands:** |err| and Wh/m thresholds are minimum effect sizes, not
stochastic noise. Turn evidence uses a fixed **2.5 pp** floor, not the
spread of turn-segment errors around the lap (that spread is route geometry).
`controller.straight_m_s` is still in config for the row-fit controller,
but the overnight loop does **not** tune it: on the warehouse track the
straightaway bump rarely qualifies. The picker tries **pivot** and
**k_steer** before **cruise** because junctions cap at **0.25 m/s**
(`V_JUNCTION_M_S`), corners use `curve_speed` below cruise, and measured
top speed often sits at `cruise_m_s` (~0.44 m/s) without small cruise
steps moving lap time or Wh/m much. It does not edit
the energy model or the physics source to make `err_pct` look better.

## Known facts

- Turn scrub force is `min(μ · m · g / 2, v_slip / FDS)`. On a gentle
  turn the slip term binds before the friction cap, so mass barely
  matters. `v_slip` is `(track / 2) · |yaw rate|` with track 0.34 m.
  FDS (`k`) is 0.01 and `μ` is 8, from the warehouse drive-wheel contact.
- Turn speed is set by the controller (pivot rate and cruise), not by mass.
- The SoC carry-over bug is fixed. A new `controller_run_id` closes the
  previous mission instead of inheriting it, and `wh_measured_soc` is the
  pack integral on the simulation step.
- The last heavy lap was **−5.0%** model error, shaft **0.866 Wh**. That
  shaft matched the light lap: the Webots body had not yet taken the BoM
  mass (see `JOURNEY.md`, 2026-10-13). Main applies that mass and scales
  body inertia, and leaves the contact cylinder at 0.08 m. The catalog
  radius is motor math only. Writing it onto the tire made shaft energy
  scale as 1/r² and left the ~5% band. A new heavy lap is a measurement.
  It is not a reason to edit the model.
- `err_pct` is `(wh_measured_drive_elec − wh_model_drive) / wh_model_drive × 100`.
  It compares electrical drive only. Shaft watt-hours are a different column.
  Negative means the model is high.
- `wh_model_drive = roll + visc + turn + accel`. Visc is hinge damping at
  `ω = v / 0.08`. Roll stays on the design bill `v / 0.04`. A warehouse
  lap is about 201 m.
- Idle watt-hours scale with time. Drive viscous watt-hours scale up with
  speed where the robot is not junction- or curve-limited. On the warehouse
  loop, long stretches run at `controller.cruise_m_s`, but junction
  approach and corner states often dominate; nudging cruise by 0.02 m/s may
  barely change lap time or Wh/m. Prefer pivot and steer knobs when turn
  error leads. Speeding up is not a free accuracy win.
- Code ideas (a missing straight-line contact term, a different motor
  line) are written in the morning report. They are not applied and not
  committed by this loop.

## Learnings

The overnight run appends one block per try under this heading.
