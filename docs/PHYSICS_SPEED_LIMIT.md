# Physics speed limit (Webots ButlerBot)

The rowfit controller’s **effective cruise** is:

```text
effective_cruise_m_s = min(controller.cruise_m_s, v_max_m_s)
```

`controller.cruise_m_s` stays the overnight optimizer’s preference. **`v_max_m_s`** is computed from stopping distance, tyre friction, and motor braking torque using **SI units** throughout.

**Webots contact radius:** the shipped worlds keep a **0.08 m** contact cylinder (`src.webots_body.DRAWN_WHEEL_RADIUS_M`). Braking torque and friction use the **BoM catalog radius** (`geometry.wheel_radius_m`, **0.04 m** for 80 mm tires). Odometry and hinge energy still use 0.08 m in sim.

**Hardware validation:** default **`brake_mu = 0.8`** (longitudinal peak on dry sealed concrete) and using Pololu **recommended continuous** torque (**0.981 N·m**) as the braking torque limit are **planning assumptions**. They are not measured on this robot yet. Validate both with coast-down / brake tests on your floor and driver before trusting the cap for safety-critical deployments.

## Braking deceleration

Total mass:

```text
m = body mass (BoM mass_kg_known, e.g. light 6.142 kg) + payload_kg + trolley tare (when configured)
```

Friction-limited deceleration (driven wheels):

```text
a_friction = μ · f_w · g     (m/s²)
```

- `μ` → `controller.speed_limit.brake_mu` (default **0.8**, ASSUMPTION dry rubber on sealed concrete; **not** Webots `coulombFriction` **8** in `butlerbot_warehouse.wbt`)
- `f_w` → `controller.speed_limit.driven_weight_fraction` (default **1.0**, from `bom.json` `assumptions.notes.turning_scrub`: both drive wheels carry weight)
- `g` = **9.81 m/s²** (`src.twin_energy.GRAVITY_M_S2`)

Motor/torque-limited deceleration:

```text
F_brake = n_motors · τ_brake / r_wheel     (N)
a_motor = F_brake / m                      (m/s²)
```

- `r_wheel` → BoM `geometry.wheel_radius_m` (**0.04 m** for light/budget 80 mm wheels)
- `τ_brake` → Pololu **recommended continuous** **0.981 N·m** (10 kg·cm) from `bom.json` / `torque_headroom.limit_nm_per_motor`, unless `controller.speed_limit.brake_torque_nm_per_motor` is set
- `n_motors` = **2**

Combined:

```text
a_brake = min(a_friction, a_motor)     (m/s²)
```

## Stopping distance and v_max

Available path length `d` (`controller.speed_limit.stop_available_m`, default **1.0 m**) must cover reaction travel plus braking:

```text
d = v · t_latency + v² / (2 · a_brake)
```

with `t_latency` = `controller.speed_limit.reaction_latency_s` (default **0.32 s**, one lane camera frame).

**Consistent closed form** (positive root):

```text
v = −a·t + sqrt(a²·t² + 2·a·d)
v_max_m_s = safety_margin · v
```

`safety_margin` = `controller.speed_limit.safety_margin` (default **0.8**).

Logging: `CONTROLLER LIMITS … v_max=… effective_cruise=… limit=physics|cruise_pref`, plus `ctrl_v_max_m_s` and `ctrl_effective_cruise_m_s` on `MISSION_SUMMARY`.

## Worked examples (t = 0.32 s, margin = 0.8, μ = 0.8, τ = 0.981 N·m, r = 0.04 m, n = 2)

Use the same formula the code implements (`src/physics_speed_limit.py`).

### Light body, no payload (m = 6.142 kg), d = 1.0 m

| Quantity | Value |
|----------|------:|
| a_friction | 0.8 × 9.81 = **7.848 m/s²** |
| F_brake | 2 × 0.981 / 0.04 = **49.05 N** |
| a_motor | 49.05 / 6.142 = **7.986 m/s²** |
| a_brake | min(7.848, 7.986) = **7.848 m/s²** (friction-limited) |
| v before margin | −7.848×0.32 + √(7.848²×0.32² + 2×7.848×1.0) = **2.177 m/s** |
| **v_max** | 0.8 × 2.177 = **1.742 m/s** |
| effective_cruise (cruise pref 0.44) | min(0.44, 1.742) = **0.44 m/s** (cruise preference active) |

### Heavy payload (m = 6.142 + 50 = **56.142 kg**), torque-limited

| Quantity | Value |
|----------|------:|
| a_friction | **7.848 m/s²** (unchanged) |
| a_motor | 49.05 / 56.142 = **0.874 m/s²** |
| a_brake | **0.874 m/s²** |

**d = 1.0 m** (default stop budget — physics does *not* bind cruise 0.44):

| v before margin | −0.874×0.32 + √(0.874²×0.32² + 2×0.874×1.0) = **1.072 m/s** |
| **v_max** | 0.8 × 1.072 = **0.857 m/s** |
| effective_cruise | min(0.44, 0.857) = **0.44 m/s** (cruise preference active) |

**d = 0.35 m** (shorter aisle — still barely above cruise pref):

| v before margin | **0.551 m/s** |
| **v_max** | 0.8 × 0.551 = **0.441 m/s** |
| effective_cruise | min(0.44, 0.441) = **0.44 m/s** (cruise preference still active; v_max ≈ cruise) |

**d = 0.33 m** (heavy case with a **clearly active** physics cap vs cruise 0.44):

| v before margin | −0.874×0.32 + √(0.874²×0.32² + 2×0.874×0.33) = **0.530 m/s** |
| **v_max** | 0.8 × 0.530 = **0.424 m/s** |
| effective_cruise | min(0.44, 0.424) = **0.424 m/s** (**physics limit active**) |

Implementation: `src/physics_speed_limit.py`, wired in `src/controller_limits.py` and `webots/controllers/butlerbot_controller/controller_rowfit.py`.
