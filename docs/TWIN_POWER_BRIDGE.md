# Twin power bridge — Webots → dashboard `last_readings`

## Goal

When Webots (or any external twin) is live, **battery % and per-channel watts** on the dashboard come from that feed — not from the pure Python simulator / mock ROS2 numbers.

When the twin is **not** live, behavior is unchanged: internal simulation + ROS2 mock/live path.

## Contract (`PowerFeed`)

Defined in `src/twin/power_feed.py` (schema **1.0**).

Minimal payload accepted by `POST /api/twin/telemetry`:

```json
{
  "source": "webots",
  "adapter": "webots",
  "robot": { "name": "ButlerBot", "main_battery_pct": 87.5 },
  "mission": { "task": "moving" },
  "channel_draws": {
    "Legs": 18.2,
    "Arms": 6.0,
    "Torso": 7.5,
    "Compute": 9.0,
    "Cooling": 3.0
  },
  "locomotion": { "gait": "drive", "speed_m_s": 0.35, "phase": "drive_transit" }
}
```

| Field | Required | Notes |
|-------|----------|--------|
| `source` | yes | `webots`, `custom`, `pybullet`, `hardware`, `external` |
| `robot.main_battery_pct` | recommended | 0–100; applied when `apply_battery_override: true` |
| `channel_draws` | recommended | Watts by channel id (wheeled: Legs/Arms/Torso/Compute/Cooling) |
| `mission.task` | optional | `idle` \| `moving` \| `balanced` \| `high_load` |

Webots controller builds this via `build_webots_telemetry()` / `twin_publisher.publish_telemetry()`.

## Data path

```
Webots controller
  → POST /api/twin/telemetry?adapter=webots
  → DigitalTwinBridge.ingest_telemetry()
  → DigitalTwinBridge.sync_to_hardware()
       ├─ ROS2Bridge.inject_command(sensor draws / mission)
       ├─ hardware._main_battery  (if apply_battery_override)
       └─ ROS2BatterySource.apply_power_feed()
            → hardware.last_readings  (immediate)
  → dashboard _build_battery_payload() reads last_readings
  → SocketIO battery_update / GET APIs
```

Also every **3s** telemetry tick, `ROS2BatterySource._build_readings()` rebuilds full PMS state (allocator, agent, safety) using twin draws when `external_active`.

## Config (on by default for this project)

```yaml
hardware:
  mode: "real"
  type: "ros2"

digital_twin:
  enabled: true
  prefer_external: true
  accept_external_telemetry: true
  apply_battery_override: true   # Webots battery becomes dashboard battery
  stale_after_s: 12              # feed must refresh within this window
```

| Flag | Effect |
|------|--------|
| `prefer_external: false` | Ignore twin power; keep internal sim numbers |
| `apply_battery_override: false` | Twin channel draws still apply; battery drains via PMS model |
| `accept_external_telemetry: false` | POST telemetry rejected |
| No fresh POSTs for `stale_after_s` | Falls back to internal sim |

## How to turn it on

1. Start dashboard (Docker recommended):

   ```powershell
   cd ...\robot_battery_monitor_project
   .\scripts\start.ps1
   # http://127.0.0.1:5000
   ```

2. Launch Webots twin (host):

   ```powershell
   .\scripts\launch_webots_twin.ps1
   ```

3. Or inject without Webots:

   ```powershell
   curl -X POST http://127.0.0.1:5000/api/twin/telemetry?adapter=webots `
     -H "Content-Type: application/json" `
     -d "{\"source\":\"webots\",\"robot\":{\"main_battery_pct\":55},\"channel_draws\":{\"Legs\":20,\"Arms\":6,\"Torso\":8,\"Compute\":10,\"Cooling\":3},\"mission\":{\"task\":\"moving\"}}"
   ```

## How to verify

| Check | Expected |
|-------|----------|
| Twin panel badge | `FEED: WEBOTS` / external feed active |
| WebSocket / API payload | `"power_source": "webots"` |
| `main_battery` | Matches Webots HUD (when override on) |
| Channel `draw` | Moves with drive / teleop, not fixed mock cycle |
| POST response | `"applied_to_hardware": true` |
| Schema | `GET /api/twin/schema` → `power_feed` section |

Without Webots and without POSTs: `power_source` is `"internal"` and the mission script simulator keeps driving numbers.

## Key files

| Role | Path |
|------|------|
| Contract | `src/twin/power_feed.py` |
| Bridge | `src/twin/bridge.py` |
| Hardware consumer | `src/hardware_ros2.py` (`apply_power_feed`) |
| Webots adapter | `src/twin/adapters/webots.py` |
| Webots publisher | `webots/controllers/butlerbot_controller/twin_publisher.py` |
| Power model | `src/twin/webots_power.py` |
| Dashboard API | `src/dashboard.py` (`/api/twin/telemetry`) |

## Fallback summary

- **No twin feed** → `SimulationDriver` + mission loop (unchanged).
- **Twin feed stale** → treated as inactive; internal resumes.
- **`hardware.mode: simulator`** → pure `SimulatorSource` (no twin power path); use `real` + `ros2` for the bridge.
