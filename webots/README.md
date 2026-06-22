# ButlerBot Webots Digital Twin

Wheeled mobile manipulator simulation that feeds live power telemetry into the Robot Battery Monitor dashboard via **DigitalTwinBridge**.

## What’s included

| Path | Description |
|------|-------------|
| `worlds/butlerbot.wbt` | World + ButlerBot robot (wheeled base, torso, dual arms) |
| `controllers/butlerbot_controller/` | Python controller + twin HTTP publisher |
| `../scripts/launch_webots_twin.ps1` | Windows launcher (dashboard + Webots) |
| `../scripts/launch_webots_twin.sh` | Linux/macOS launcher |

## Prerequisites

1. **Webots** R2023b or newer (tested with **R2025a**) — [cyberbotics.com/download](https://cyberbotics.com/download)
2. **Dashboard running** at `http://127.0.0.1:5000` (Docker or local)
3. Python 3 (bundled with Webots is fine for the controller)

## Quick start (Windows)

```powershell
# Terminal 1 — start dashboard + database
.\scripts\start.ps1

# Terminal 2 — launch Webots twin (opens simulation)
.\scripts\launch_webots_twin.ps1
```

Open **http://127.0.0.1:5000** → **Digital Twin Bridge** panel should show `FEED: WEBOTS`.

## Quick start (Linux / macOS)

```bash
./scripts/start.sh
./scripts/launch_webots_twin.sh
```

## Manual launch

```bash
# 1. Start dashboard first
docker compose up -d
# or: python src/dashboard.py

# 2. Open world in Webots
cd webots
webots --mode=realtime worlds/butlerbot.wbt
```

Or open Webots GUI → **File → Open World** → `webots/worlds/butlerbot.wbt` → Play.

## Telemetry flow

```
Webots ButlerBot controller
  → reads motors / GPS / IMU
  → estimates motor_power_w + channel_draws
  → POST /api/twin/telemetry?adapter=webots
  → DigitalTwinBridge (WebotsAdapter)
  → PMS dashboard (live power, LRU, agent, safety)
```

### Example payload (abbreviated)

```json
{
  "source": "webots",
  "adapter": "webots",
  "motor_power_w": {"left_wheel": 4.2, "right_wheel": 4.1, "torso_joint": 2.0},
  "channel_draws": {"Legs": 8.3, "Arms": 3.1, "Torso": 2.0, "Compute": 9.0},
  "joints": [{"name": "left_wheel", "velocity": 4.5, "torque": 0.3}],
  "locomotion": {"gait": "walk", "speed_m_s": 0.32, "phase": "walk_transit"}
}
```

## Controller options

Set via `controllerArgs` in the world file or environment:

| Option | Default | Description |
|--------|---------|-------------|
| `--dashboard-url=http://HOST:PORT` | `http://127.0.0.1:5000` | Twin bridge base URL |
| `--telemetry-interval=0.5` | `0.5` | Seconds between POSTs |
| `TWIN_DASHBOARD_URL` | (same) | Env override for dashboard URL |

## Mission phases (controller)

The robot cycles automatically:

1. **standby** — idle, motors off  
2. **walk_transit** — drive forward (moving)  
3. **patrol** — slow weave (balanced)  
4. **manipulate** — arm motion, wheels stopped (high_load)  
5. **return_idle** — idle  

## Motor → PMS channel map

| Webots motor | PMS channel |
|--------------|-------------|
| `left_wheel`, `right_wheel` | Legs |
| `torso_joint` | Torso |
| `left_arm`, `right_arm` | Arms |
| (estimated) | Compute |

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Twin panel shows `SOURCE: INTERNAL` | Dashboard not reachable; check URL and firewall |
| `Twin publish failed` in Webots console | Start dashboard first; verify `curl http://127.0.0.1:5000/api/twin/schema` |
| Robot falls through floor | Reset simulation (Ctrl+Shift+T) |
| Floor / sky not visible | World uses R2025a `Floor` + `TexturedBackground` PROTOs — reload world after update |
| Controller not found | Open world from `webots/` folder so controllers path resolves |

## Expanding the model

The current ButlerBot is a **foundation** wheeled base. To grow later:

- Replace wheels with leg joints (biped) — keep the same motor names or update `src/twin/webots_power.py`
- Add PROTO in `protos/ButlerBot.proto`
- Enable torque feedback on motors for more accurate power estimates

## API reference

- `GET /api/twin/state` — poll PMS state from Webots supervisor scripts  
- `POST /api/twin/command` — send task/throttle back to PMS  
- `GET /api/twin/schema` — full integration contract