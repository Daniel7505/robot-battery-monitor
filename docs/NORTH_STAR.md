# North star — what this project is building

**Standing orientation.** Updated 2026-09-02 by Daniel + Grok Build.  
Lane-keep is the **evaluation harness**, not the product. This particular S is **closed**.

## The product

An onboard agent that can:

1. **Keep a local catalog of real, purchasable hardware** — motors, batteries, wheels, cameras, structure, ESCs, sensors — with mass, cost, power curves, compute / vision cost, and other specs that matter.
2. **Accept a mission / track** — eventually user-drawn or easily specified.
3. **Design or assemble** the most cost-efficient physical robot that can complete that mission.
4. **Evaluate candidates** in the Webots twin (later on real hardware).
5. **Transfer** the chosen design + software to real hardware.

The current wheeled ButlerBot + PMS dashboard + lane-keep stack is **scaffolding**: a harness that answers “can this config drive A to B without walking off, and what did it cost in energy?” It is not the end product. The repo name (“battery monitor”) is leftover.

## What “good enough” means for the harness

Warehouse-style **travel** often tolerates **5–20 cm** lateral error. **Docking / picking** wants **1–3 cm**.  
The 2026-09-02 nadir lap (full S, IN_LANE on the red, max **3.2 cm** at 0.44 m/s) is in the **travel** ballpark **in sim**, with margin toward docking-grade *in this world*. A clean 3 cm band in ideal Webots will open up on real hardware (lighting, calibration, latency, play). Design corridors and safety margins with that gap. The old 128 picture-wins card (max 15.7 cm) is historical.

## Hard constraints we already paid for

| Lesson | Do not forget |
|:-------|:--------------|
| LINE_CAM looks **along** the corridor | Metric wall distance from these views is unreliable. Closed. |
| 64×64 vs 128×128 | 64 is fast and coarse. 128 cut first-lobe peak to 15.7 cm and dropped whole-lap realtime to ~0.09–0.13. |
| Real cameras will be far higher res | **Cannot** scan every pixel every 8 ms in the final system. |
| Compute is a design cost | A high-res camera that forces the loop to 5 Hz is a bad part for many missions. |
| Drive policy (this phase) | **Shoulder nadir on the wheel.** Fight 32/29 px + ahead-row HOLD, cruise 5.5 (0.44 m/s), v-scale 2.10, steer cap 0.55. LINE_CAM / forecast / finish cameras exist in the world and are **not enabled**. This S is closed — no more knobs on it. |

Details: [`LANE_KEEP_BASELINE_2026-08-17.md`](LANE_KEEP_BASELINE_2026-08-17.md), [`LANE_KEEP_CLOSED_2026-08-17.md`](LANE_KEEP_CLOSED_2026-08-17.md).

## How we build around those limits

- Perception must be **resolution-optional**: ROI / multi-rate / downsample so higher-res cameras help instead of only taxing the clock.
- The design agent scores **compute / vision cost**, not just mass and dollars.
- Keep a **fast low-res** sim mode for iteration and a **higher-fidelity** mode for final checks.
- The controller should eventually consume a **clean error + confidence** signal, not a raw full-frame scan, so the same law works across camera resolutions.

## Build order (do not skip)

1. **Parts / hardware database** — **first cut lives here:** [`PARTS_CATALOG.md`](PARTS_CATALOG.md) / `src/parts_db.py` / `data/parts.db`. Grow columns when the design agent needs them.
2. **Design / assembly agent** — **first cut lives here:** [`DESIGN_AGENT.md`](DESIGN_AGENT.md) / `src/design_agent.py`. Heuristic BOMs from the catalog. Twin eval is the next plug-in.
3. **Only then** return to tightening the drive stack or raising cruise.

When in doubt, ask: **which numbered north-star item does this work serve?** If the answer is “none,” stop.

## Working style

- One coherent experiment or feature at a time.
- Easy revert.
- Mechanical extracts and clear interfaces over god-file growth.
- Visual still beats “idle” telemetry when they disagree.
- Do not save the Webots world on exit.
