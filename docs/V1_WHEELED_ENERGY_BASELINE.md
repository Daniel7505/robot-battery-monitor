# V1 wheeled energy baseline — ButlerBot

Snapshot of the **robot-battery-monitor** power stack after PowerFeed, throttle-loop fixes, mission-from-motion, and the grounded `butlerbot_wheeled` hardware profile.

*Last verified: 2026-07-29 (live Webots + dashboard).*

---

## Architecture path

```
Webots butlerbot_controller
  → joint / GPS / teleop state
  → build_webots_telemetry()  [src/twin/webots_power.py]
       uses config/hardware_profiles/butlerbot_wheeled.yaml
  → POST /api/twin/telemetry?adapter=webots
  → DigitalTwinBridge.ingest + sync_to_hardware
  → ROS2BatterySource.last_readings  (PowerFeed)
  → dashboard WebSocket / GET /api/twin/state
```

**Fallback:** no fresh twin feed → internal `SimulationDriver` + mission loop (no Webots).  
**Battery % in sim:** Webots controller drains with gameplay scale; dashboard can override from twin when `apply_battery_override: true`.

---

## Hardware profile (active)

| Field | Value |
|-------|--------|
| **ID** | `butlerbot_wheeled` |
| **File** | `config/hardware_profiles/butlerbot_wheeled.yaml` |
| **Battery** | 48 V Li-ion class, **480 Wh** (10 Ah) |
| **Drive** | 2× 48 V BLDC hub class (~120 W cont / 250 W peak, η≈0.75) |
| **Stabilizers** | Small caster/balance load on Torso |
| **Compute** | Jetson-class SBC + sensor baseload (+ optional vision when moving) |
| **Legs channel cap** | 28 W |

Selected via `hardware_profile: butlerbot_wheeled` in `config/config.yaml`.  
Loader: `src/hardware_profile.py` (`battery_capacity_wh()`, `motor_idle_and_scale()`, …).

---

## Energy signature (v1 baseline)

Observed on live twin (profile-grounded model; short API / agent-script drive):

| Mode | Legs | Total (approx) | Mission / phase | Agent / safety |
|------|------|----------------|-----------------|----------------|
| **Idle** | **~4 W** | **~18 W** | idle / standby | normal; thr usually false |
| **Forward roll** | **~8–26 W** (crawl low teens, cruise ~20–22, full teleop high-20s without hard peg) | **~26–46 W** | moving / teleop | may go cautious; no permanent spiral |
| **In-place turn** | **above idle** (from wheel \|ω\| / cmd; GPS speed may be ~0) | **elevated** | moving / **teleop_turn** | same as drive |
| **After stop** | **back toward ~4–5 W** when speed ~0 | **~18 W** | idle / standby | settles (incl. after turn) |

**Manual / command teleop baseline (earlier same day):** idle Legs 5→4 W (after profile), drive peak 28 W, mission idle→moving→idle.

**Agent/command script:** `python scripts/agent_short_drive.py`  
Same twin command API as dashboard Drive buttons (not Webots keyboard). Expected: Legs rise, mission → moving, stop settles.

---

## What “good” means for this baseline

1. Twin feed active; profile id + 480 Wh visible on telemetry when controller is current.
2. Idle: low stable Legs, Idle/Standby, no throttle spam.
3. Short forward (~2 ft): Legs clearly above idle; mission leaves Idle.
4. Stop: speed ~0, Legs and mission return toward idle.
5. No permanent “safety fault → system ×70% every tick” loop.

---

## Known limitations

| Topic | Note |
|-------|------|
| **Turn cost** | Pure rotation uses wheel rates (not GPS speed). Mission phase `teleop_turn` / gait `turn`. Residual spin after Forward→Turn→Stop fixed (spin-halt + wheel settle). |
| **Mid-band** | P2: partial-load ^1.25 + soft over-cruise headroom + lighter gait stress; cruise ~21 W Legs, full ~26 W (under 28 W cap). |
| **Stop settle** | ABS requires body *and* wheels settled; spin path always halts (no symmetric reverse on yaw). |
| **Agent during drive** | Can go cautious and briefly throttle; clears when idle — not the old latch loop. |
| **Not a vendor BOM** | Specs are public-class averages for interpretability, not a purchase list. |

---

## Quick re-verify

```powershell
.\scripts\start.ps1
.\scripts\launch_webots_twin.ps1
python scripts/agent_short_drive.py
```

Idle → drive → stop should match the table above.  
See also: `docs/HARDWARE_PROFILE.md`, `docs/TWIN_POWER_BRIDGE.md`.

---

## Internship / talk track (one paragraph)

We built a **power-aware wheeled robot digital twin**: Webots posts battery and per-channel watts through a stable **PowerFeed** into the same dashboard path used for live hardware. Numbers are grounded in a small **hardware profile** (hub motors, stabilizers, compute, 480 Wh pack). The v1 energy signature is clear—about **4 W** drive channel at idle and a strong rise under forward motion with mission state tracking drive vs standby—without a runaway safety/agent throttle loop. Next steps are turn-cost characterization and tighter mid-band motor curves.
