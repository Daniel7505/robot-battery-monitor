# Lane-keep baselines (locked 2026-08-17)

**Historical.** Picture-wins / LINE_CAM 128 scorecard. **Not** the live operator.

Live law (2026-09-02): two shoulder nadirs, fight 32/29 + ahead HOLD 32/26, cruise 5.5 (0.44 m/s), v-scale 2.10. Full S champ **3.2 cm**, parked on the red. World cameras besides those nadirs were removed 2026-09-02. See [`NORTH_STAR.md`](NORTH_STAR.md).

Daniel + Web (then): freeze 128×128 as the working baseline.  
This file is the **old drive scorecard** book. Product direction lives in [`NORTH_STAR.md`](NORTH_STAR.md).

## Working baseline — 128×128 (then)

What we run. First-lobe reliability over pure 5 cm share.

| Knob | Value |
|:-----|:------|
| `LINE_CAM_L` / `LINE_CAM_R` | **128×128**, identity, mount `(0.30, ±0.65, 0.12)`, FOV 1.2. Never remount / never look-at |
| `FORECAST_CAM_Z/W` | 64×64, identity `0 0 1 0`, lookout only. Not on the wheel |
| Finish cams | 64×64, look-at magenta pucks (unchanged) |
| `PLANNER_KAHEAD` | **0.22** |
| `DEFAULT_STEER_RELEASE_PER_S` | **4.0** |
| Cruise | **5.5 rad/s ≈ 0.44 m/s** |
| Drive law | picture-wins + far-preview on (`t_ahead` fallback 2.0 s) |
| Metric wall path | **off the wheel** (closed) |
| SIDELOOK (`SIDE_CAM_*`) | **off** (`SIDELOOK_ON` deleted) |
| H / H2 | HUD every 4 steps; Z/W scan every 8 |
| World save | **never** |

Scored S (2026-08-17, 900 s wall budget, `--mode=realtime`):

| item | value |
|:-----|:------|
| Result | **IN_LANE ON_LINE** |
| End | **24.171, 0.124** \|x−24.5\|=0.329 |
| max \|ct\| | **15.7 cm** |
| First-lobe peak | **15.7 cm** @ x=3.60 |
| Crest @ x≈15.3 | **5.9 cm** |
| % @ 5 cm / 10 cm | **23% / 81%** |
| Coast + ABS | yes, parked on the mark |
| Whole-lap rt | **0.094** (15 s straight clock was 0.132) |

Report: `Desktop/Grok Workspace/grok-web-butlerbot-128-full-s-2026-08-17.md`  
Revert eyes only: set the two LINE_CAM `width`/`height` fields back to 64.

HUD `hud_left`/`hud_right` Displays are still 64×64. Overlay looks sky-heavy; the **brain** reads the 128 buffer. That mismatch is cosmetic.

Cruise bump still gated: first-lobe ≤ 14 cm **and** ≥ 60 % @ 5 cm on a finished S.

## Historical champion — 64×64 (keep this)

Tightest centerline we ever recorded on this track. **Not** the current checkout. Written down so it cannot go missing.

| Knob (as recorded) | Value |
|:-----|:------|
| Era | 2026-08-16 night, **before** A/D/H/H2/I. Picture-wins + Z/W mounted **HUD only** |
| LINE_CAM | **64×64**, identity, same mounts / FOV |
| Planner | ahead fade **0.45**, sign-flip let-go (this is **not** today's `kahead=0.22`) |
| Release | 4.0 (D later tried 6 and wrecked) |
| Cruise | 5.5 |
| Z/W | mounted, **not** steering |
| Result | **IN_LANE ON_LINE** |
| Score | **71% @ 5 cm, 96% @ 10 cm, max 11 cm** |

Exact git hash of that lap was **never tagged**. Later freeze HEAD `d2c8642` is A-winner `kahead=0.22` + H+H2, **not** this 71% card. Do not claim `d2c8642` is the 71% robot.

To A/B against it later: LINE_CAM back to 64, and remember the planner was still on the **0.45 ahead-fade** family, not 0.22.

## Closed this session (do not retry as “new”)

| ID | What | Result |
|:---|:-----|:-------|
| Metric walls on LINE_CAM | `metric_ct` into GapPlanner | LEFT_LANE, max ct 1.49 m. Meters frozen. Off the wheel |
| SIDELOOK Option A | extra cams, yaw ±0.30, observation | Pictures changed; meters moved 4–7 mm on a 20 cm pin. Not promoted |
| LINE_CAM 128 | this baseline | First lobe 15.7 cm, finished ON_LINE, 23/81 @ 5/10. Clock cost real |

Do not retry A / D / preview-off / Z/W meters / H / H2 / I / metric-on-wheel / SIDELOOK-as-steer.

## Measure

```text
python -u scripts/measure_clocks.py
python -u scripts/measure_s_lane_keep.py
```

S timeout is **900 s** wall (128 molasses). Integrate rt as Δsim/Δwall.
