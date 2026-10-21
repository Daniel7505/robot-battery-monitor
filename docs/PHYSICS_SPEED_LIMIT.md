# Physics speed limit (Webots ButlerBot)

The rowfit controller’s **effective cruise** is:

```text
effective_cruise_m_s = min(controller.cruise_m_s, v_max_m_s)
```

`controller.cruise_m_s` stays the overnight optimizer’s preference. **`v_max_m_s`** is computed from stopping distance, tyre friction, and motor braking torque using **SI units** throughout.

**Webots contact radius:** the shipped worlds keep a **0.08 m** contact cylinder (`src.webots_body.DRAWN_WHEEL_RADIUS_M`). Braking torque and friction use the **BoM catalog radius** (`geometry.wheel_radius_m`, **0.04 m** for 80 mm tires). Odometry and hinge energy still use 0.08 m in sim.

**Hardware validation:** default **`brake_mu = 0.8`** and using Pololu **recommended continuous** torque as the braking torque limit are **planning assumptions** — validate with coast-down / brake tests on your floor and driver.

## Operating assumption

The sim models a **dark factory with no humans present**. Forward speed is capped only by **physics and automation constraints**: stopping distance (friction and motor braking), perception range (nadir look-ahead, stereo/radar when obstacles are on), and robot/shelf avoidance via the obstacle gate — **not** by a separate human-safety speed limit. If people may enter the workspace, add a **configurable hard cap** (for example **~1.5 m/s**) on top of the computed `v_max`; that cap is **not** implemented in this repo today.

## What the robot can see ahead

### (a) Upcoming turn / pivot (nadir lane cameras)

| Source | Parameter | Value |
|--------|-----------|------:|
| `webots/worlds/butlerbot*.wbt` `NADIR_CAM_L/R` | resolution | **128 × 128** px |
| same | `fieldOfView` | **1.2 rad** |
| same | `near` | **0.03 m** |
| `src/lane_vision.py` | `MAX_RANGE_M` (floor projection discard) | **3.5 m** |
| `src/lane_vision.py` `camera_coverage()` on repo poses | `top_row_x_m` (centre column) | **≈ 1.96 m** ahead of axle |
| `src/rowfit_control.py` `lookahead_bounds_from_coverage()` | `ld_max` = min(top row) − **`LD_FAR_MARGIN_M` 0.36 m** | **≈ 1.60 m** |
| `src/rowfit_control.py` | pure-pursuit clamp | **`ld_min` … `ld_max`** (typ. **0.55–1.60 m**) |
| `src/lane_vision.py` corner cues | pivot distance when outer line “widens then disappears” | vision-limited (often **~1–2 m** before the crossbar; see `tests/test_corner.py`) |

**Turn-approach braking** uses path distance `remaining_m` from rowfit (vision corner / junction centre, optionally GPS track), capped by **`ld_max`** (what nadir geometry is trusted for):

```text
d_eff = min(remaining_m, ld_max)
v_allowed = max_speed_for_distance(d_eff, …, v_target = approach_target_m_s)
v_cmd = min(effective_cruise, v_allowed, …)
```

Config: `controller.speed_limit.approach_target_m_s` (default **0.04 m/s**, `V_CREEP_M_S`).

### (b) Obstacles (stereo + radar, when `RBM_OBSTACLES=1`)

| Source | Parameter | Value |
|--------|-----------|------:|
| `webots/worlds/butlerbot*.wbt` `STEREO_CAM_*` | size / FOV | **320×240**, **1.4 rad** |
| `webots/worlds/butlerbot*.wbt` `RADAR_FWD` | range / HFOV | **0.15–8 m**, **1.0 rad** |
| `src/stereo_depth.py` `CorridorParams` | corridor depth | **4.0 m** (`max_range_m`) |
| `docs/OBSTACLES.md` | floor seen from axle | **≈ 0.75 m** |
| `src/obstacle_gate.py` | `STANDOFF_M` | **0.35 m** |
| same | stop distance at speed **v**, decel **`a_brake`** | **`STANDOFF_M + v·t + v²/(2a)`** |
| same | slow zone at **v** | **`d_stop(v) + SLOW_EXTRA_M` 1.0 m** |

**Global cruise cap** stop budget (default, no YAML override):

```text
d_available = min(ld_max ≈ 1.60 m, obstacle detection range 4.0 m when obstacles_in_d_available)
```

Optional **`controller.speed_limit.stop_available_m`** overrides that computed distance. Turn approach still uses **`ld_max`** via path `remaining_m`.

Hand-check at **t = 0.32 s**, light **`a = 7.848`**, heavy **`a = 0.874`**: slow-zone clearance (m) at **v = 0.44 / 1.13 / 2.48** — light **1.503 / 1.793 / 2.535**; heavy **1.602 / 2.442 / 5.663**. At heavy **v_max ≈ 1.13** the slow zone **≈ 2.44 m** stays inside **4.0 m** stereo range; light **v_max ≈ 2.48** → slow **≈ 2.54 m** likewise.

## Braking deceleration

Total mass:

```text
m = body mass (BoM mass_kg_known, e.g. light 6.142 kg) + payload_kg + trolley tare (when configured)
```

Friction-limited deceleration (driven wheels):

```text
a_friction = μ · f_w · g     (m/s²)
```

- `μ` → `controller.speed_limit.brake_mu` (default **0.8**)
- `f_w` → `controller.speed_limit.driven_weight_fraction` (default **1.0**)
- `g` = **9.81 m/s²**

Motor/torque-limited deceleration:

```text
F_brake = n_motors · τ_brake / r_wheel     (N)
a_motor = F_brake / m                      (m/s²)
```

- `r_wheel` = BoM **0.04 m**; `τ_brake` = **0.981 N·m** continuous unless overridden; `n_motors` = **2**
- `a_brake = min(a_friction, a_motor)`

## Stopping / approach speed formulas

With reaction time `t` and safety margin `m`:

```text
d = v·t + (v² − v_target²) / (2·a)
v = −a·t + sqrt(a²·t² + 2·a·d + v_target²)
v_cmd = m · v
```

Braking must begin by ( **`v_target = 0`**, from **`v = 0.44 m/s`**):

| case | a (m/s²) | d_start = v·t + v²/(2a) |
|------|---------:|------------------------:|
| light | **7.848** | **≈ 0.153 m** |
| heavy (+50 kg) | **0.874** | **≈ 0.252 m** |

**`v_max_cruise`** if the only horizon is nadir look-ahead **`d_lookahead ≈ 1.60 m`** (`t = 0.32 s`, `m = 0.8`, `v_target = 0`):

| case | a (m/s²) | v_max_cruise (m/s) |
|------|---------:|-------------------:|
| light | **7.848** | **≈ 2.48** |
| heavy | **0.874** | **≈ 1.13** |

(Cruise pref **0.44 m/s** still binds unless payload + short `d_eff` cap lower.)

## Gyro-assisted pivot

Worlds add **`Gyro { name "gyro" }`** beside `InertialUnit`. Controller reads **`gyro.getValues()[2]`** (rad/s, + = left). Pivot rate tracks `pivot_w_max_rad_s` with gyro feedback; completion can use **integrated heading** when gyro is live. Telemetry: `ctrl_yaw_rate_rad_s`, `ctrl_pivot_heading_err_rad`, `ctrl_pivot_integrated_deg`.

## Worked examples (t = 0.32 s, margin = 0.8, μ = 0.8, τ = 0.981 N·m, r = 0.04 m)

### Light body (m = 6.142 kg), d = 1.60 m (default `d_available`)

| a_brake | **7.848 m/s²** (friction-limited) |
| v_max | **≈ 2.48 m/s** → cruise pref **0.44 m/s** still active |

At **d = 1.0 m**: v_max **≈ 1.742 m/s** (same physics, shorter budget).

### Heavy payload (m = 56.142 kg)

| a_brake | **0.874 m/s²** (torque-limited) |
| d = 1.0 m | v_max **≈ 0.857 m/s** → cruise pref active |
| d = 0.33 m | v_max **≈ 0.424 m/s** → **physics cap active** |

Implementation: `src/physics_speed_limit.py`, `src/rowfit_control.py` (`_approach_tick`, pivot + gyro), `webots/controllers/butlerbot_controller/`.
