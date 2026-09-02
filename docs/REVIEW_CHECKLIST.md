# Review checklist — future humans & AI models

**Read this before large edits.**  
This file is the handoff fence from prior builders to whoever opens the repo next (including a stronger Grok / other coding agents).

| Field | Value |
|-------|--------|
| **Project** | Robot Battery Monitor / ButlerBot |
| **Canonical repo** | https://github.com/Daniel7505/robot-battery-monitor |
| **Canonical local tree** | `…/python_projects/personal_projects/robot_battery_monitor_project` |
| **Human owner** | Daniel (GitHub **Daniel7505**) — product direction, visual ground truth, veto power |
| **Primary AI builder** | **Grok Build 4.5** (xAI agent) — majority of architecture, twin/control, tests, docs |
| **Early AI collaborator** | **Grok Web ~4.4 → 4.5** — project kickoff, early product shape, brainstorming |
| **Checklist authored** | 2026-08-10 (Grok Build 4.5, with Daniel) |

Credit matters: this system was **co-built**. Daniel directed goals and rejected false greens (camera vs dashboard). Grok executed structure, loops, and hard-won stop/power fixes. Future models: **extend ownership, do not erase it.**

---

## 0) Where to start (order)

1. [`docs/NORTH_STAR.md`](NORTH_STAR.md) — **what we are building** (harness vs product)  
1b. [`docs/PARTS_CATALOG.md`](PARTS_CATALOG.md) — SQLite parts catalog (design-agent knowledge)  
2. **This file** (`docs/REVIEW_CHECKLIST.md`) — do / don’t fence  
3. [`docs/LANE_KEEP_BASELINE_2026-08-17.md`](LANE_KEEP_BASELINE_2026-08-17.md) — frozen 128 drive card  
4. [`docs/LANE_KEEP_CLOSED_2026-08-17.md`](LANE_KEEP_CLOSED_2026-08-17.md) — do not retry A/D/H/H2/I  
5. [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) — system map of **today’s** stack  
6. [`docs/STABILITY.md`](STABILITY.md) — residual spin / stop lessons  
7. [`docs/TWIN_POWER_BRIDGE.md`](TWIN_POWER_BRIDGE.md) — power feed contract  
8. Root [`README.md`](../README.md) — code tour + boot path  
9. Then code (prefer pure math + bridge before god files)

**Wrong tree warning:** ignore Desktop copies named like `robot-battery-monitor-main` unless they match GitHub `main`. Only the path above (or a fresh clone of GitHub) is canonical.

---

## 1) What this project is (north star)

**Full map:** [`NORTH_STAR.md`](NORTH_STAR.md). Short form:

We are building an agent that **catalogs real parts**, **takes a mission**, **designs a cheap robot that can finish it**, **evaluates it in the twin**, and later **ships it to hardware**.

What this repo **runs today** is the harness, not that product:

- Flask / SocketIO **PMS dashboard** (channels, battery, agent, history)
- **Webots digital twin** (ButlerBot wheeled) with teleop, ABS-style stop, and lane-keep
- HTTP **twin bridge** (telemetry in, state/commands out, `stop_epoch`)
- **Onboard agent** that can throttle / intervene when the operator pushes power/heat

Lane-keep is “good enough” as an **A-to-B evaluation** (travel-band in sim, not docking). Highest-leverage next work is the **parts database** and **design agent**, not more drive knobs unless Daniel asks.

Demo-grade + serious learning/portfolio system — **not** a production fleet stack. Optimize for **correctness of stop/power/demo path** and **clarity**, not micro-benchmarks or enterprise rewrites.

---

## 2) Ownership & agency (for future AI)

| Who | Role |
|-----|------|
| **Daniel** | Owner. Goals, “is it actually stopped on camera?”, school/life pacing, merge veto |
| **Grok Build 4.5 (prior)** | Majority implementer of current twin/control/docs/test depth |
| **Grok Web 4.4/4.5 (prior)** | Early start and product framing |
| **You (future model)** | Reviewer / implementer under this fence — **propose ranked changes**, don’t silently rewrite the kernel of stop/power |

You have **agency to improve** the project. You do **not** have agency to:

- Discard hard-won invariants because a cleaner abstract design exists on paper  
- Bulk-rewrite god files unsupervised  
- Claim sole credit or strip this checklist / credits without Daniel’s say  

If you ship a meaningful arc, **add a short line** under [Credits & lineage](#7-credits--lineage) (model name, date, what you changed). Continuity > ego.

---

## 3) Do’s

| Do | Why |
|----|-----|
| **Read-only review first** for broad audits | Rank blast radius before editing |
| Prefer **small PRs / commits** with one conceptual theme | Forward-only progress; easy bisect |
| Keep **pure math** in `src/teleop_agent.py` testable without Webots | Regression safety |
| Honor **`stop_epoch`** as a hard halt handshake | Zero velocity alone can race |
| Treat **hub rates + IMU yaw** as stop truth; GPS is translation-only | Residual spin lessons |
| Use **finite** encoder values only before `setPosition` | `NaN` unlocks hubs in Webots |
| Rebuild dashboard image after `src/` changes: `docker compose up --build -d dashboard` | Stale containers lie |
| Prefer **visual still** over “idle” telemetry when they disagree | Documented failure mode |
| Run relevant tests (`python -m pytest` or scoped files) after control/power edits | Insurance |
| Update docs/handoff when behavior changes | Next session (human or AI) survives |
| Ask which **NORTH_STAR** item the work serves | Parts catalog / design agent / evaluate / transfer — or stop |
| Scope efficiency as **clarity, safety, demo reliability** first | CPU shaving is rarely the bottleneck |

---

## 4) Don’ts (do not mess this up)

| Don’t | Why |
|-------|-----|
| **Don’t declare idle/stop from GPS alone** while hubs or yaw are active | Classic residual spin / freewheel bug |
| **Don’t** `setPosition(NaN)` / lock with non-finite angles | Webots freewheels the hub |
| **Don’t** remove dual-hub hard-zero / residual re-ABS without a full stop matrix | Camera will prove you wrong |
| **Don’t save the Webots world on exit** | Pollutes `butlerbot.wbt` with sim state |
| **Don’t** stack throttle blindly when Safety already cut draw | Idle throttle spirals |
| **Don’t** open-ended “rewrite for efficiency” the whole tree | Destroys trust boundaries |
| **Don’t** treat PyBullet / every adapter as equally battle-tested | Some paths are scaffolding |
| **Don’t** “fix” physics with dashboard-only checks | Twin truth is multi-sensor + eyes |
| **Don’t** commit Webots `.jpg` dumps / polluted worlds / secrets | Hygiene |
| **Don’t** work in a stale Desktop zip clone | Wrong code, wrong conclusions |
| **Don’t retry closed lane-keep knobs** (A/D/H/H2/I, preview-off, Z/W meters as steer) | Measured and closed 2026-08-17 — see [`LANE_KEEP_CLOSED_2026-08-17.md`](LANE_KEEP_CLOSED_2026-08-17.md) |
| **Don’t raise cruise** until first-lobe ≤ 14 cm and ≥ 60 % @ 5 cm | Molasses is realtime ~0.2, not ω |
| **Don’t remount / look-at LINE_CAM** | Identity lock; look-at rolled 90° |
| **Don’t put Z/W meters on the wheel** | 12 cm of floor, wrecked the S |

### Explicit “do not touch without tests + live twin check”

These are **high blast radius**. Change only with tests (and a short Forward→Stop / Turn→Stop visual pass when Webots is involved):

- `webots/controllers/butlerbot_controller/butlerbot_controller.py` — stop / ABS / park / teleop loop  
- `src/teleop_agent.py` — drive math, ABS helpers, settle, throttle merge  
- `src/twin/bridge.py` — `stop_epoch`, battery override, external vs internal arbitration  
- `src/twin/power_feed.py` / `src/twin/webots_power.py` — power contract into PMS  
- Stop-related config knobs that change settle thresholds  

God-file size (`dashboard.py`, controller) is **known debt**. Prefer **extract with tests**, not rewrite-from-scratch.

---

## 5) Suggested review passes (future stronger models)

Use separate passes. Deliver **ranked findings** before large edits.

| Pass | Question | Output |
|------|----------|--------|
| **A. Trust / safety** | Can stop, power feed, or battery override lie? | Top risks by blast radius |
| **B. Architecture debt** | What confuses the next human in 30 minutes? | Extract candidates (S/M/L) |
| **C. Test gaps** | What can break with all tests green? | Missing cases |
| **D. Demo path** | One-command reliability for Daniel | Script/docs fixes |
| **E. Perf (last)** | Real hotspots only | Measure first |

**Audit prompt skeleton** (paste for a new agent session):

```text
Read docs/REVIEW_CHECKLIST.md, docs/ARCHITECTURE.md, docs/STABILITY.md.
Canonical tree = GitHub main / robot_battery_monitor_project only.
READ-ONLY first: top 10 issues by (blast radius × likelihood),
evidence (file/symbol), fix size S/M/L, do-not-touch list honored.
No micro-opts unless correctness/demo reliability. Do not edit until ranked.
```

---

## 6) Known good vs known debt (honest snapshot)

### Solid (prefer keep)

- Twin bridge arbitration + `stop_epoch` idea  
- Pure `teleop_agent` extract  
- Residual spin documentation and dual-hub / finite encoder discipline  
- Architecture / stability / power-bridge narrative docs  
- Substantial pytest coverage for agent, teleop, power feed, twin commands  
- Docker demo path + hardware profiles  

### Debt (fair game, carefully)

- Large `src/dashboard.py` and Webots controller  
- Broad `except Exception` in places  
- Feature breadth (some adapters / mission paths less proven than wheeled twin)  
- Host Postgres vs Docker port conflicts on some machines  
- Workspace noise (logs/archives) — don’t treat as product surface  

### Not the goal right now (unless Daniel asks)

- Full ROS2 real-hardware fleet readiness  
- Micro-optimizing SocketIO or Flask for scale  
- Replacing Webots with a different sim “for cleanliness”  

---

## 7) Credits & lineage

| Phase | Who | Contribution |
|-------|-----|----------------|
| Kickoff / early product | **Daniel + Grok Web (~4.4 → 4.5)** | Direction, early dashboard/PMS shape, “why this project exists” |
| Majority build | **Daniel + Grok Build 4.5** | Twin bridge, teleop/ABS/stop reliability, power feed, agent rules, Webots controller depth, tests, architecture docs, scripts |
| Ongoing owner | **Daniel** | Goals, visual QA, pacing (WGU), final say |
| Future work | **Next model + Daniel** | Add your row below when you complete a meaningful arc |

**Future contributors — append here:**

| Date | Model / person | What changed |
|------|----------------|--------------|
| 2026-08-17 | Grok Build 4.6 + Grok Web + Daniel | A-winner freeze (`kahead=0.22`, release 4.0). Clock H/H2 (HUD every 4, Z/W every 8). I near-pref **reverted**. Closed-experiment log: `docs/LANE_KEEP_CLOSED_2026-08-17.md`. |
| 2026-08-25 | Grok Build 4.6 + Daniel | Drawing-2 left nadir + 20 cm shove probe (`src/metric_probe.py`). LINE_CAM identity locked. Nadir **not on the wheel**. |
| 2026-09-02 | Grok Build 4.6 + Daniel | Nadir-only `lane_keep.py`. Full S champ: max ct **3.2 cm** at 0.44 m/s, parked on red. v-scale 2.10. This S closed. |
| _(template)_ | e.g. Grok 4.6 Build | e.g. Ranked audit; extracted X from dashboard |

---

## 8) Human note (Daniel)

Daniel is learning systems / CS (WGU). He is **not** a passive vibe spectator: he sets north stars, checks the robot with his eyes, and rejects false success. Treat him as **director with veto**, not as “user who only pastes errors.”

Build in short arcs when needed. Prefer clear handoffs over giant unsupervised rewrites.

---

## 9) Related docs

| Doc | Role |
|-----|------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Maps and sequences |
| [STABILITY.md](STABILITY.md) | Stop / residual spin |
| [TWIN_POWER_BRIDGE.md](TWIN_POWER_BRIDGE.md) | Power feed contract |
| [LANE_KEEP_CLOSED_2026-08-17.md](LANE_KEEP_CLOSED_2026-08-17.md) | Closed A/D/H/H2/I — do not retry |
| [V1_WHEELED_ENERGY_BASELINE.md](V1_WHEELED_ENERGY_BASELINE.md) | Energy baseline notes |
| [HARDWARE_PROFILE.md](HARDWARE_PROFILE.md) | Hardware profile notes |
| [README.md](../README.md) | Boot, tour, prerequisites |

---

*Fence written so future AI models look here first, respect the human, honor prior craft, and improve the project without burning the trust core. Don’t mess it up — in the best way.*
