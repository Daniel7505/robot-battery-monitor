# Hardware profile (wheeled ButlerBot v1)

## Where it lives

```
config/hardware_profiles/butlerbot_wheeled.yaml   # active profile
config/hardware_profiles/butlerbot_biped.yaml     # alternate (not default)
src/hardware_profile.py                           # load + helpers
src/twin/webots_power.py                          # uses profile for channel watts
```

Config selection (`config/config.yaml`):

```yaml
hardware_profile: butlerbot_wheeled
robot:
  hardware_profile: butlerbot_wheeled
  main_battery_capacity_wh: 480   # kept in sync with profile battery.capacity_wh
```

## Parts in the v1 profile

| Block | What |
|-------|------|
| **battery** | 48 V Li-ion, **480 Wh** (10 Ah class) |
| **motors.left/right_wheel** | 48 V BLDC hub class (~120 W cont), idle/cruise/peak + efficiency |
| **stabilizers** | Small caster/balance load on **Torso** (idle + speed-linked) |
| **IMU** | **Adafruit BNO085** (#4754) 9-DOF fusion — catalog SKU, I2C 0x4A |
| **balance_control** | Pitch-hold PD on both hubs (`kp` 2.0 / `max_correct` 0.8 after 2026-08-12 seesaw; sensor is real) |
| **compute + sensors** | Jetson Orin Nano class + cameras/encoders (IMU is its own block). Twin also has two down-look line cameras + a finish camera for agent lane-keep. |
| **modes** | Optional vision/agent adders when moving |
| **channels** | PMS caps (Legs max 28 W, etc.) |

## How numbers are used

1. **Webots controller** builds joint velocities → `estimate_motor_power_w` / `build_webots_telemetry`.
2. **Motor model** (per wheel):  
   - rest → `idle_w`  
   - moving → `idle_w` + load from τ·ω (or τ≈coeff·ω), scaled so **cruise_speed ≈ cruise_w**  
   - clamp to per-motor `peak_w` and channel `max_draw_w`
3. **Stabilizers** add a few watts to Torso when speed > 0.
4. **Compute** = SBC + sensors (+ vision/agent when moving).
5. **Battery % drain** (sim + Webots HUD scale) uses **profile `capacity_wh`** via `battery_capacity_wh()`.

## What changed vs old generic constants

| Before | After |
|--------|--------|
| Opaque `idle_w` / `scale: 3.6` only | Public-spec style rated W, efficiency, cruise_w @ speed |
| No explicit stabilizers | Stabilizer idle/active on Torso |
| Compute single idle/active | Compute + sensors + optional vision/agent |
| Capacity often hard-coded 480 in multiple places | Single helper prefers profile YAML |
| Stress multipliers could push to channel peak always | Milder stress; cruise_w sets mid-band drive |

Idle should stay near **~5 W Legs / ~18 W total**. Drive should still show a **clear Legs rise** (then fall when stopped).

## Verify

```powershell
# Dashboard + Webots twin
.\scripts\start.ps1
.\scripts\launch_webots_twin.ps1

# Same baseline: idle → short forward → stop
# Expect: idle calm; drive Legs up; mission moving; stop returns toward idle
# Longer mission: python scripts/agent_extended_drive.py
```

Also: twin telemetry / export includes `robot.hardware_profile` and `battery_capacity_wh`.
