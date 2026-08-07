# Stability notes — ButlerBot wheeled twin

Short record of control/stop reliability as of **2026-07-29**, git **`9350d44`** on `main`.

## Residual spin (layers)

### Layer A — freewheel / “spaz” (resolved `9350d44`)

| | |
|--|--|
| **Symptom** | Left-wheel / in-place spin on camera while dashboard showed speed≈0, idle/standby, Legs~4 W |
| **Root cause** | (1) GPS-only settle (yaw/freewheel invisible to translation speed); (2) `setPosition(NaN)` from invalid early encoder reads → motor lock rejected |
| **Fix** | Finite encoder check; hard-zero **both** hubs with full torque; ABS complete on hub rates + IMU yaw quiet; residual re-ABS / yaw oppose |
| **Human confirm** | No more chaotic freewheel spin |

### Layer B — ultra-slow creep (addressed 2026-08-06)

| | |
|--|--|
| **Symptom** | After Stop, looks still; look away ~3 s and pose/heading has changed. Forward may still coast; turn leaves sub-visual yaw |
| **Root cause** | ABS completed when hubs/yaw were only *mostly* quiet (`STOP_YAW≈0.12`, short calm hold); GPS zeroed for telemetry when hubs+yaw quiet (masked coast); residual kill ignored pose window + GPS coast |
| **Fix** | Tighter stop gates; ABS requires linear GPS calm + longer hold; earlier hub position-lock; residual re-ABS on hub / yaw / coast / **pose-window drift**; firmer hold torque |
| **Verify** | Reload Webots world (controller is live from repo files). Forward→Stop, Turn→Stop — hold 5 s; pose should not creep |

## Live stop matrix (current)

| Sequence | Status |
|----------|--------|
| Forward → Stop | Pass (API + visual) |
| Forward → Turn → Stop | Pass (no continued circle) |
| In-place turn → Stop | Pass |

## Do not regress

- Do not lock hubs with non-finite positions.
- Do not declare idle from GPS alone while either hub or yaw is active.
- Prefer one Webots instance; kill orphans before relaunch (`taskkill` / close window).
- On Webots exit: **do not save world** (avoids polluting `butlerbot.wbt`).

## Related

- Energy baseline: `docs/V1_WHEELED_ENERGY_BASELINE.md`
- PowerFeed path: `docs/TWIN_POWER_BRIDGE.md`
- Controller: `webots/controllers/butlerbot_controller/butlerbot_controller.py`
