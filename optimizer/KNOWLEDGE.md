# Optimizer knowledge

Notes for the overnight loop. The seeded facts below are the standing
picture of the energy model. The loop appends a short learning after
each try under **Learnings**. It does not rewrite the seeded facts.

The loop may change only the allowlisted keys in `config/config.yaml`
(`bom_build`, `controller.cruise_m_s`, `controller.straight_m_s`,
`controller.pivot_w_max_rad_s`, `controller.k_steer`). It does not edit
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
  mass (see `JOURNEY.md`, 2026-10-13). This branch loads BoM mass and the
  contact-wheel radius. A new heavy lap is a measurement. It is not a
  reason to edit the model.
- `err_pct` is `(wh_measured_drive_elec − wh_model_drive) / wh_model_drive × 100`.
  It compares electrical drive only. Shaft watt-hours are a different column.
  Negative means the model is high.
- `wh_model_drive = roll + visc + turn + accel`. Visc is hinge damping at
  `ω = v / 0.08`. Roll stays on the design bill `v / 0.04`. A warehouse
  lap is about 201 m.
- Idle watt-hours scale with time. Drive viscous watt-hours scale up with
  speed. Slowing the straight cap is the first knob when visc dominates
  and the model is high. Speeding up is not a free accuracy win.
- Code ideas (a missing straight-line contact term, a different motor
  line) are written in the morning report. They are not applied and not
  committed by this loop.

## Learnings

The overnight run appends one block per try under this heading.
