# Stability notes — ButlerBot wheeled twin

Short record of control/stop reliability as of **2026-07-29**, git **`9350d44`** on `main`.

## Residual spin (resolved)

| | |
|--|--|
| **Symptom** | Left-wheel / in-place spin on camera while dashboard showed speed≈0, idle/standby, Legs~4 W |
| **Root cause** | (1) GPS-only settle (yaw/freewheel invisible to translation speed); (2) `setPosition(NaN)` from invalid early encoder reads → motor lock rejected |
| **Fix** | Finite encoder check; hard-zero **both** hubs with full torque; ABS complete on hub rates + IMU yaw quiet; residual re-ABS / yaw oppose |
| **Human confirm** | Robot stationary after fix (operator visual check) |

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
