# Physics speed limit (Webots ButlerBot)

The rowfit controller’s **effective cruise** is:

```text
effective_cruise_m_s = min(controller.cruise_m_s, v_max_m_s)
```

`controller.cruise_m_s` stays the overnight optimizer’s preference. **`v_max_m_s`** is computed from stopping distance, tyre friction, and motor braking torque using **SI units** throughout.

**Webots contact radius:** the shipped worlds keep a **0.08 m** contact cylinder (`src.webots_body.DRAWN_WHEEL_RADIUS_M`). Braking torque and friction use the **BoM catalog radius** (`geometry.wheel_radius_m`, **0.04 m** for 80 mm tires). Odometry and hinge energy still use 0.08 m in sim.

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

## Worked examples (repo defaults: d = 1.0 m, t = 0.32 s, margin = 0.8, μ = 0.8, τ = 0.981 N·m, r = 0.04 m)

### Light body, no payload (m = 6.142 kg)

| Quantity | Value |
|----------|------:|
| a_friction | 0.8 × 9.81 = **7.85 m/s²** |
| F_brake | 2 × 0.981 / 0.04 = **49.1 N** |
| a_motor | 49.1 / 6.142 = **7.99 m/s²** |
| a_brake | min(7.85, 7.99) = **7.85 m/s²** (friction-limited) |
| v before margin | −7.85×0.32 + √(7.85²×0.32² + 2×7.85×1.0) ≈ **2.18 m/s** |
| **v_max** | 2.18 × 0.8 ≈ **1.74 m/s** |
| effective_cruise (cruise pref 0.44) | min(0.44, 1.74) = **0.44 m/s** (cruise preference active) |

### Heavy payload (m = 6.142 + 50 = **56.142 kg**)

| Quantity | Value |
|----------|------:|
| a_friction | **7.85 m/s²** (unchanged) |
| a_motor | 49.1 / 56.142 = **0.87 m/s²** (torque-limited) |
| a_brake | **0.87 m/s²** |
| v before margin | −0.28 + √(0.08 + 1.74) ≈ **1.04 m/s** |
| **v_max** | 1.04 × 0.8 ≈ **0.83 m/s** |
| effective_cruise | min(0.44, 0.83) = **0.44 m/s** (still cruise pref at d = 1 m) |

With a **shorter** aisle budget **d = 0.35 m** (same heavy payload):

| v before margin | ≈ **0.52 m/s** |
| **v_max** | 0.52 × 0.8 ≈ **0.42 m/s** |
| effective_cruise | min(0.44, 0.42) = **0.42 m/s** (**physics limit active**) |

Implementation: `src/physics_speed_limit.py`, wired in `src/controller_limits.py` and `webots/controllers/butlerbot_controller/controller_rowfit.py`.
