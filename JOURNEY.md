# Project journey: building ButlerBot with AI tools

A dated, honest log of how this project was built: which AI tools I used at each stage, what went wrong, and what actually worked. It is written for anyone (including future employers) who wants to see *how* I work with AI, not just the result.

**Owner:** Daniel ([Daniel7505](https://github.com/Daniel7505)). I set the goals, judged the results with my own eyes in the simulator, and had the final say on every merge. The AI tools wrote most of the code.

**How the dates were sourced:** everything from 2026-05-15 on comes from `git log`, pull request titles and `save-*` tags in this repo. Code sizes are Python line counts at those commits. The earliest stage happened before the git history starts, so its dates are approximate and come from memory. Git does not record which AI tool wrote a given commit, so the tool boundaries below are approximate too.

---

## At a glance

| Period | Main tool | Python lines (git) | Where it got to |
|--------|-----------|--------------------|-----------------|
| ~2025, over a year before Oct 2026 (before git) | Gemini, copy-paste in chat | (not in git) | Barely past `main.py` |
| up to mid-2026 (git starts 2026-05-15) | Grok web app | 342 on 2026-05-15 | A good-looking dashboard where nothing really worked |
| ~2026-06 → 2026-09 | Grok Build (CLI coding agent) | 5,378 (06-20) → 23,987 (09-04) | Docker + Postgres stack, Webots twin, robot talks to the dashboard, long fight with the S track |
| 2026-10-01 → 2026-10-25 | Grok Bot (agent with GitHub, CI, its own machine) | 24,031 (10-01) → 41,497 (10-05) | Tests fixed + CI, security, rowfit lane vision, corners, junctions, warehouse, obstacles, picking, PRs #1-#17 |

Test files grew from 2 (2026-05-15) to 42 (2026-09-04) to 65 (2026-10-05). As of 2026-10-25, pytest collects 867 tests: 27 need Postgres (the `db` marker), and one skips without the docker CLI. The suite runs in GitHub Actions on every pull request.

---

## Stage 1: Gemini, copy and paste (~2025, over a year before Oct 2026; before git)

- **Challenge:** The assistant couldn't see my code. Every change meant pasting files into the chat and pasting answers back.
- **What happened:** We barely got past `main.py`. Keeping the AI's picture of the project in sync with the real files was the bottleneck, not writing code.
- **What I learned:** Context is everything. An AI that can't see the whole project can't build one.

## Stage 2: Grok web app (until mid-2026; git starts 2026-05-15)

- **Challenge:** Get a real dashboard running.
- **What changed:** The web app could read the GitHub repo, so it understood the project much better. Still lots of copy-paste.
- **What happened:** A polished dashboard UI (`v4.0 - Polished Live Dashboard with Web UI`, 2026-05-15; Docker support 2026-05-17), but it looked far better than it worked. Early commits are fixes like "websocket updates still not showing on dashboard" (2026-05-19).
- **What I learned:** "It looks done" and "it works" are different things. I started checking behaviour, not screenshots.

## Stage 3: Grok Build, a coding agent in the terminal (~June to early September 2026)

- **Challenge:** Turn the dashboard into a real power-management system and connect it to a physics simulator.
- **Milestones (from git):**
  - 2026-06-20: LRU groups, safety rules, predictions and Docker deploy in one push (5.4k lines).
  - 2026-06-22: **Webots ButlerBot digital twin** added, with the PMS and onboard-agent control.
  - 2026-06-24 → 06-30: teleop, twin API drive, ABS brake.
  - 2026-07-29 → 08-06: a long run of `fix(control)` commits against **residual wheel spin after Stop**. The dashboard said speed ≈ 0 while the wheel was visibly still spinning in Webots. Root causes were GPS-only "is it stopped?" checks and NaN motor locks (`docs/STABILITY.md`).
  - 2026-08-06: architecture diagrams and first-reader comments. 2026-08-09: `docs/REVIEW_CHECKLIST.md`, a "fence" so future AI models don't rewrite the hard-won stop/power core.
  - 2026-08-12 → 09-04: lane keeping on a painted S track. Many approaches were tried, measured and **closed** (`docs/LANE_KEEP_CLOSED_2026-08-17.md`). A nadir-camera lap landed on 2026-09-02 and the S was declared closed. The track was tightened on 2026-09-04.
  - 2026-08-17: first **parts catalog** (SQLite) and a thin **design agent**: the start of the real goal (`docs/NORTH_STAR.md`).
- **What went wrong:** The robot talked to the dashboard, but driving the S reliably was a struggle, and the code grew fast (≈5k → ≈24k lines). That made it hard for me to verify what each change really did.
- **What worked:** My visual check in Webots beat telemetry every time they disagreed. Writing down closed experiments stopped us from re-trying the same ideas.
- **What I learned:** A human has to own "ground truth". Write down what failed, not just what worked.

## Stage 4: Grok Bot, an agent with GitHub, CI and its own machine (2026-10-01 →)

- **Challenge:** The project was big and the tests were stale. I didn't have the experience to review 25,000 lines line by line.
- **Milestones (from PRs and tags):**
  - 2026-10-01: **PR #1** fixed the stale tests and **added CI** (GitHub Actions + Postgres). **PR #2** added security hardening (loopback-only by default, CORS allowlist, optional API token, no hard-coded DB password). **#3** added a drift report tool, **#4** stopped DB startup from hanging on an orphaned lock, and **#5** made every number agree with the 16.5 m finish. Tag `save-2026-10-01-lanekeep-v1`.
  - 2026-10-01/02: **#6** rowfit lane vision (camera model + lane fit + pure pursuit). **#7** cameras moved into the shoulder boxes (`save-2026-10-01-rowfit-boxcams`). **#8** waypoint tracks and sharp 90° corners, built on my "the line widens, then disappears" idea for spotting a corner (`save-2026-10-01-corner90-v1`).
  - 2026-10-02: **#9** intersections and route policy, **#10** a prototype warehouse course, **#11** stereo obstacle sensing. These PRs were titled "do not merge before Webots test": I tested each one in Webots before merging.
  - 2026-10-05: **#12** junction-turn lineup fix (`save-2026-10-05-junction-turn-v1`), **#13** radar for moving objects, **#14** inventory cubes + pull-behind trolley, **#15** multi-trip shopping list, **#16** straightaway cruise 0.44 → 0.50 m/s, **#17** mission efficiency metrics (time, Wh, distance) on the dashboard. Each has a `save-…-pre-…` / `save-…-v1` tag pair as an easy rollback point.
  - 2026-10-11: **#18** README rewrite for newcomers, corrected architecture diagrams, tests that run without a database, and this journal; Grok Bot added to the README credits (`save-2026-10-11-readme-refresh-v1`).
- **What worked:** Small, single-purpose branches and PRs. CI as the first line of verification. Tags before and after every feature. An offline simulator (`scripts/rowfit_sim.py`) to catch mistakes before Webots.
- **What I learned:** Trust comes from proof. Once the work was checkable (tests, CI, a rollback tag, a Webots run), I could move fast without reading every line. The open gap is still verification on my side. The next step is a fixed set of Webots scenarios that report pass/fail automatically.

---

## The north star

The goal is an onboard agent that picks real parts, designs the cheapest robot that can finish a mission, tests it in simulation, then builds it (`docs/NORTH_STAR.md`). The robot and dashboard here are the test bench. **The north star may change as the project learns;** when it does, record the change here with a date and the reason.

---

## Template for new entries

Copy this block, newest entry at the bottom of the current stage (or start a new stage).

```markdown
### YYYY-MM-DD: <short title>

- **Challenge:** What problem or goal, in one or two sentences.
- **Tools:** Which AI tool(s) and how I used them (chat, agent, CI, simulator).
- **What we tried:** Approaches, including the ones that failed (link PRs / commits / docs).
- **What worked:** The fix or result, and how I verified it (test, CI, Webots run, measurement).
- **What I learned:** One or two lessons I'd reuse.
- **Links:** PR #…, tag `save-…`, docs/…
```

### 2026-10-14: Overnight optimizer

- **Challenge:** Leave the Windows PC running warehouse laps and tune controller speeds and BoM build selection for energy-model accuracy, without anyone babysitting it.
- **Tools:** Cursor Grok agent. Webots was not run here; the loop's diagnose / score / log path is tested with a fake lap runner.
- **What we tried:** One allowlisted config change per lap, keep or roll back on a score, then a held-out lap so a one-build fluke does not stick. The energy-model source is not a knob.
- **What worked:** The non-Webots logic is covered by `tests/test_optimizer.py`. A real night still has to be started on the PC that has Webots and Docker.
- **What I learned:** Mass barely moves gentle-turn scrub, so the loop steps turn rate and speed, not the physics file.
- **Links:** `optimizer/KNOWLEDGE.md`, `scripts/overnight_optimize.ps1`

## 2026-10-13: Real parts and an energy model that matches (PR #19)

**What we built.** A bill of materials (`config/bom/bom.json`) with real, datasheet-backed parts in three builds (light, heavy, budget). Specs that aren't published are flagged as unknown rather than guessed.

**Challenge: the meters disagreed.** The predicted and measured energy didn't line up, and we chased the gap one cause at a time:
- The model was comparing motor-shaft energy against battery-side (electrical) energy. Converting both to the same units on the Pololu motor curve fixed the mismatch.
- Turns were off by about 110% because turning wasn't modeled at all. We added a turn scrub term.
- Dan suspected the gap came from timing between telemetry updates. He was right: the dashboard was integrating sparse posts and dropping about 101 seconds of data. The controller now keeps cumulative energy totals on simulation time.
- Damping was billed at a 0.04 m wheel radius instead of 0.08 m. On runs #8 and #9, SoC-derived and integrated measured Wh agreed to about 0.1% (two meters on the same drain, not model versus measured). Lap #12 (`docs/baselines/2026-10-13_light_warehouse.md`) model error is −28.3% drive-only / −21.2% drive+idle. The 0.08 m wheel-radius damping fix (`ee98c61`) predicts model drive about 3.7 Wh versus 3.742 Wh measured, but that is unverified until a rerun lap is logged.

**Debugging calls.** The stutter Dan saw was rendering only (real-time factor swinging 0.6–1.5x while actual speed barely changed). Turning off the HUD did not cut energy use. It only shortened wall-clock time, and Grok Bot corrected its own earlier claim about that.

**Honest result.** The first lap on the heavy build was 5% off. But measured shaft energy was identical to the light build, which showed the 3D robot in Webots never actually got heavier. Only the battery and motor math changed. Next step: make the simulated robot load the BoM's mass and wheel size.

**Dan's design notes for later.**
- The bot nearly stops for 90° turns and takes them fine, but struggles to re-enter the lane afterward.
- Build choices need common-sense rules written as formulas, such as wheel and motor load ratings checked against total mass, so "a LEGO wheel can't support a forklift."
