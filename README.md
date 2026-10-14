# ButlerBot (repo: robot-battery-monitor)

**A simulated delivery robot plus a live "car dashboard" for its battery and power use.**
Owner: [Daniel7505](https://github.com/Daniel7505) · Co-built with Grok Build · Standing goals: [`docs/NORTH_STAR.md`](docs/NORTH_STAR.md) · Review rules: [`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md) · Project story: [`JOURNEY.md`](JOURNEY.md)

> **About the name.** The repo is still called "robot-battery-monitor" because that is how it started. It has grown into a full robot simulation project. The name is leftover.

---

## Contents

1. [What is this, in plain English?](#1-what-is-this-in-plain-english)
2. [What you'll see](#2-what-youll-see)
3. [Where the project stands (honest status)](#3-where-the-project-stands-honest-status)
4. [How it fits together (diagrams)](#4-how-it-fits-together-diagrams)
5. [Pick your setup](#5-pick-your-setup)
   - [A. Just look around (no install)](#a-just-look-around-no-install)
   - [B. Run the lightweight simulator and tests (no Webots)](#b-run-the-lightweight-simulator-and-tests-no-webots)
   - [C. Full Windows setup: dashboard + Webots robot](#c-full-windows-setup-dashboard--webots-robot)
   - [D. Linux or macOS (untested)](#d-linux-or-macos-untested)
   - [E. Developer / contributor workflow](#e-developer--contributor-workflow)
   - [F. Overnight optimizer (Windows)](#f-overnight-optimizer-windows)
6. [Reading the dashboard](#6-reading-the-dashboard)
7. [Settings you might change](#7-settings-you-might-change)
8. [Common problems](#8-common-problems)
9. [Where things live (folder map)](#9-where-things-live-folder-map)
10. [Credits and AI collaboration](#10-credits-and-ai-collaboration)
11. [Project journey](#11-project-journey)

---

## 1. What is this, in plain English?

Imagine a small wheeled robot that carries things around a warehouse. Before anyone buys parts and builds it, this project lets you:

- **Drive it in a 3D simulator** ([Webots](https://cyberbotics.com/), free). The robot follows yellow painted lanes using two downward-facing cameras, turns at corners and intersections, slows for obstacles, and stops cleanly.
- **Watch its "fuel gauge" live** in a web page: battery %, how many watts each body part uses (wheels, arms, torso, computer, cooling), safety warnings, and how long the battery should last.
- **Save the history** in a database so you can look back at past runs.
- **Test ideas without the simulator**: a fast "offline" lane-following simulator and about 630 automated tests.
- **Start designing real robots**: a small catalog of real, buyable parts and a first-draft "design agent" that suggests the cheapest parts list for a mission.

The long-term goal ([`docs/NORTH_STAR.md`](docs/NORTH_STAR.md)) is an agent that picks real parts, designs the cheapest robot that can finish a mission, tests it in the simulator, and then builds it for real. Today's robot and dashboard are the **test bench** for that goal. The north star may evolve as the project learns; `NORTH_STAR.md` is the current version.

You **do not need a real robot**. Everything runs on a normal PC.

---

## 2. What you'll see

| Where | What it looks like |
|-------|--------------------|
| **Browser** at `http://127.0.0.1:5000` | A page titled **"ButlerBot Live Monitor"** with a big battery %, power per channel, green/yellow/red safety status, forecast, mission info and history charts. Numbers update by themselves every second or two. |
| **Webots window** | ButlerBot (wheeled base, torso, two arms) driving along a yellow lane: by default a double S-curve (about 17.4 m along the lane) that ends at a red finish line at GPS x=16.5 m, or corners, intersections, or a warehouse with boxes and a pull-behind trolley. Small camera overlays show what the robot sees. |
| **Dashboard while Webots runs** | The **Digital Twin Bridge** panel changes from `SOURCE: INTERNAL` to **`FEED: webots`**. Battery and watts now come from the simulated robot. A strip shows mission time, energy used (Wh) and distance. |
| **Terminal (no Webots)** | `rowfit_sim.py` prints a small table: did the robot finish the course, how far off the lane it drifted (cm), and how long it took. |

---

## 3. Where the project stands (honest status)

**Works today**

- Live power dashboard with safety rules, forecasts and an "onboard agent" that throttles the robot if you push it too hard.
- Webots robot that lane-keeps on its own, handles 90° corners, T-junctions and crossroads, detects obstacles (stereo cameras + radar) in obstacle worlds, and runs multi-trip "shopping list" pick runs in the warehouse world.
- A reliable stop (ABS, no wheel spin after stopping). See [`docs/STABILITY.md`](docs/STABILITY.md).
- Docker one-command startup, Postgres history, automated tests on every push (GitHub Actions).

**Limitations (please read)**

- **Simulation only.** There is no real robot or battery hardware connected. "Real" in settings like `HARDWARE_MODE=real` means "the full, realistic software path," **not** a physical robot.
- **Lane accuracy is a simulator number** (about 3 cm in ideal Webots). Real cameras, lighting and floors will be worse.
- **"Legs" means wheels.** The power channels are named Legs / Arms / Torso / Compute / Cooling because a walking robot was planned. On the wheeled ButlerBot, "Legs" is the drive wheels. A biped profile exists but the wheeled one is the trusted baseline.
- **ROS 2 is mocked by default.** No ROS install is needed; real ROS 2 is optional and less tested.
- **Design agent is a first draft.** The parts catalog has about 10 parts, and the suggested designs are **not** yet test-driven in Webots automatically.
- **Developed and tested on Windows 10/11 only.** Linux/macOS scripts exist but are **untested**.

---

## 4. How it fits together (diagrams)

### The simple picture

Three things run on your computer: the **Webots simulator** (optional), the **dashboard app**, and a **database**. Your browser talks to the dashboard. The robot in Webots reports its battery and power to the dashboard, and gets "drive", "stop" or "slow down" back.

```mermaid
flowchart LR
    YOU["👤 You<br/>web browser<br/>http://127.0.0.1:5000"]

    subgraph PC["Your computer"]
        WEBOTS["🤖 Webots robot simulator<br/>ButlerBot drives a painted lane<br/>(optional)"]
        subgraph DOCKER["Docker (or plain Python)"]
            DASH["📊 Dashboard app<br/>run_dashboard.py<br/>power math · safety · agent"]
            DB[("🗄️ Postgres<br/>history")]
        end
    end

    OFFLINE["🧪 Offline tools - no Webots needed<br/>tests · rowfit_sim.py · parts catalog"]

    YOU -- "watch live numbers,<br/>press Drive / Stop" --> DASH
    WEBOTS -- "battery, power, position" --> DASH
    DASH -- "drive, stop, slow down" --> WEBOTS
    DASH -- "saves readings" --> DB
    OFFLINE -. "same robot code,<br/>runs by itself" .- WEBOTS
```

### What actually flows where (for the curious)

Each box names the file that does the job. Items marked * only switch on in the obstacle or warehouse worlds.

```mermaid
flowchart TB
    subgraph HOST["① Webots process on your PC - webots/controllers/butlerbot_controller/"]
        direction LR
        SENS["Sensors<br/>GPS · IMU · encoders<br/>nadir cameras L/R<br/>stereo + radar*"]
        VIS["Lane vision<br/>src/lane_vision.py<br/>src/junction_vision.py"]
        OBS["Obstacles*<br/>controller_obstacles.py<br/>src/obstacle_gate.py"]
        INV["Pick list*<br/>controller_inventory.py<br/>src/shopping_list.py"]
        DRIVE["Steer + speed<br/>controller_rowfit.py<br/>src/rowfit_control.py"]
        WHEELS["Wheels · ABS · stop<br/>controller_wheels.py"]
        POWER["Power estimate<br/>src/twin/webots_power.py"]
        PUB["twin_publisher.py"]
        SENS --> VIS --> DRIVE
        SENS --> OBS -- "speed cap" --> DRIVE
        INV -- "stop / slow" --> DRIVE
        DRIVE --> WHEELS --> POWER --> PUB
    end

    subgraph APP["② Dashboard process - run_dashboard.py (Docker 'dashboard' container, or local Python)"]
        direction LR
        API["Flask + SocketIO<br/>src/dashboard.py"]
        BRIDGE["DigitalTwinBridge<br/>src/twin/bridge.py<br/>latest feed · stop_epoch"]
        TICK["PMS tick<br/>src/hardware_ros2.py"]
        PMS["Allocator · safety · LRU<br/>forecast · mission<br/>src/onboard_agent.py"]
        API <--> BRIDGE
        BRIDGE -- "Webots feed if fresh,<br/>else internal sim" --> TICK
        TICK <--> PMS
        PMS -- "agent throttle" --> BRIDGE
        TICK -- "live state" --> API
    end

    subgraph OUT["③ Storage and screens"]
        direction LR
        DB[("Postgres 16<br/>Docker 'postgres'<br/>src/database.py")]
        BROWSER["Browser<br/>ButlerBot Live Monitor"]
        CLI["robot_battery_monitor.py<br/>terminal reports"]
    end

    PUB -- "POST /api/twin/telemetry" --> API
    API -- "GET /api/twin/state:<br/>drive · stop_epoch · throttle" --> PUB
    TICK -- "write readings" --> DB
    DB -- "history / analytics" --> API
    DB --> CLI
    API -- "SocketIO battery_update" --> BROWSER
    BROWSER -- "POST /api/twin/command:<br/>drive · drive_stop · battery_reset" --> API
```

How to read it:

1. **Inside Webots** (top): the cameras see the yellow lane, `lane_vision` turns pixels into "where is the lane", `rowfit_control` turns that into steering and speed, and `controller_wheels` drives the motors and handles stopping. The power estimate is computed from what the motors are doing.
2. **Over HTTP** (middle arrows): about twice a second the robot **POSTs** its readings to the dashboard and **GETs** back any commands (drive, stop, throttle). This is plain web traffic. ROS 2 is not needed for it.
3. **Inside the dashboard app**: the Twin Bridge keeps the latest robot reading. The PMS tick (Power Management System) uses that reading when it is fresh, or its own built-in simulation when Webots is not running, then runs the budget, safety and agent rules and saves a snapshot to Postgres.
4. **Out to you**: the browser gets a live push (`battery_update`) and its Drive/Stop buttons send commands back the same way.

More diagrams (stop handshake, per-tick sequence, deployment): [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

---

## 5. Pick your setup

| I want to... | Go to | Install | Time |
|--------------|-------|---------|------|
| Understand the project without installing anything | [A](#a-just-look-around-no-install) | Nothing | 10 min |
| Run the offline robot simulator and tests | [B](#b-run-the-lightweight-simulator-and-tests-no-webots) | Git, Python 3.12 (Docker optional, for the database tests) | 20-30 min |
| See the robot drive and the dashboard live (Windows) | [C](#c-full-windows-setup-dashboard--webots-robot) | Git, Docker Desktop, Webots, Python | 45-60 min (mostly downloads) |
| Same on Linux or Mac (**untested**) | [D](#d-linux-or-macos-untested) | Git, Docker, Webots | 45-60 min |
| Change code and send it back | [E](#e-developer--contributor-workflow) | All of B | - |

Ports used: **5000** (dashboard) and **5432** (database), both only reachable from your own computer.

---

### A. Just look around (no install)

1. Read [Section 1](#1-what-is-this-in-plain-english) and the [diagrams](#4-how-it-fits-together-diagrams) above.
2. Open [`docs/NORTH_STAR.md`](docs/NORTH_STAR.md): the "why" and the build order.
3. Open [`docs/CHEAT_SHEET.md`](docs/CHEAT_SHEET.md): the latest state of the robot's driving brain.
4. Browse [`webots/README.md`](webots/README.md) for the simulator side and [`docs/TRACKS.md`](docs/TRACKS.md) for the test tracks.
5. Want to read code? Start with these, in order:

| File | What it does |
|------|--------------|
| `run_dashboard.py` | Starts everything: database → power loop → web dashboard |
| `src/dashboard.py` | The web page, the `/api/twin/*` endpoints and the live push |
| `src/twin/bridge.py` | The "post office" between Webots and the dashboard |
| `src/hardware_ros2.py` | The main power loop: budget, safety, agent, database |
| `webots/controllers/butlerbot_controller/butlerbot_controller.py` | The robot's main loop inside Webots |
| `src/lane_vision.py`, `src/rowfit_control.py` | How the robot sees the lane and steers |
| `config/config.yaml` | Most settings |

---

### B. Run the lightweight simulator and tests (no Webots)

Good for checking the project works on your machine, or trying a driving change quickly.

**You need:** [Git](https://git-scm.com/downloads) and [Python 3.12](https://www.python.org/downloads/) (tested with 3.12.10 on Windows; CI uses 3.12; on Windows tick **"Add python.exe to PATH"** during install). Optional: a Postgres database for the ~30 database tests; the easiest way is [Docker Desktop](https://www.docker.com/products/docker-desktop/).

**Step 1. Get the code**

```
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
```

**Step 2. Make a private Python environment and install the packages**

Windows (PowerShell):
```
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt pytest
```

Mac / Linux:
```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt pytest
```

Your prompt now starts with `(.venv)`. Run the activate line again whenever you open a new terminal.

**Step 3. Run the offline lane-following simulator** (no database, no Webots)

```
python scripts/rowfit_sim.py --table
```

It takes about 15 seconds. ✅ **It worked if** the last lines look like this (numbers may differ slightly):

```
course          finish time_s worst_cm mean_cm min_v pivots  pivot_err_deg(imu/true) pivot_s  stop
s               yes      40.8      7.8     2.9  0.20      0                        -       -  finish
turn90          yes      15.7      7.4     3.0  0.20      0                        -       -  finish
```

`finish yes` means the robot completed the course; `worst_cm` is the furthest it drifted from the lane centre.

More to try:

```
python scripts/rowfit_sim.py corner90 --table --quiet       # a sharp 90° corner
python scripts/rowfit_sim.py --help                          # every option
python -m src.parts_db                                       # list the parts catalog
python -m src.design_agent --length 17                       # suggest cheapest parts list for a 17 m track
```

Any file in `tracks/` (for example `corner_mix`, `plus`, `t_end`, `widen`) can be used as a course name.

**Step 4. Run the automated tests**

```
python -m pytest
```

That works with or without a database:

- **No Postgres running:** the ~30 tests marked `db` are **skipped** with a note at the end, and everything else runs (about 600 tests). ✅ **It worked if** the last line shows `passed` and `skipped`, with no `failed` or `error`.
- **With Postgres:** every test runs. ✅ **It worked if** the last line says roughly `629 passed, 1 skipped` (the count grows over time; the one skip is a Docker check that only runs when the `docker` command is available).

The full run takes about 5 minutes. To run just one file: `python -m pytest tests/test_lane_keep.py`.

**Optional: run the database tests too**

1. Copy the settings file once (Windows: `copy .env.example .env`, Mac/Linux: `cp .env.example .env`).
2. Start just the database with Docker and create the tables (this also creates the test database `robot_battery_test`):

   ```
   docker compose up -d postgres
   ```
   Wait about 10 seconds, then:

   Windows (PowerShell):
   ```
   $env:PGPASSWORD="robot"
   python scripts/setup_postgres.py
   ```
   Mac / Linux:
   ```
   export PGPASSWORD=robot
   python scripts/setup_postgres.py
   ```
3. Run `python -m pytest` again.

The tests empty the tables of `robot_battery_test` only. Your dashboard history in `robot_battery` is not touched.

Already have Postgres installed without Docker? Create a `robot` user (password `robot`) with permission to create databases and run `scripts/setup_postgres.py` as above, or point the tests elsewhere with `TEST_DATABASE_URL=postgresql://user:pass@host:5432/robot_battery_test`.

---

### C. Full Windows setup: dashboard + Webots robot

This is the main demo: the robot drives in Webots and the dashboard shows its battery live.

**You need**

| Tool | Why | Link |
|------|-----|------|
| Git | Download the code | https://git-scm.com/downloads |
| Docker Desktop | Runs the dashboard and database for you (no Python needed for that part) | https://www.docker.com/products/docker-desktop/ |
| Webots R2025a (R2023b or newer) | The 3D robot simulator. Runs on Windows, **not** in Docker | https://cyberbotics.com/ |
| Python 3.12 (tested with 3.12.10 on Windows) | Webots runs the robot's controller code with the Python on your PC. After installing, check **Webots → Tools → Preferences → Python command** points at it | https://www.python.org/downloads/ (tick "Add python.exe to PATH") |
| A browser | To view the dashboard | Chrome, Edge, Firefox |

About 5-10 GB of downloads in total.

**Step 1. Install and start Docker Desktop.** Reboot if asked. Start it and wait until it says **Engine running** (the first start can take a few minutes; WSL2 is normal).

**Step 2. Get the code** (PowerShell):
```
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
```

**Step 3. Start the dashboard and database**
```
.\scripts\start.ps1
```
The first run builds the app and takes a few minutes. It copies `.env.example` to `.env` for you if you don't have one.

✅ **It worked if** it prints `Dashboard ready: http://127.0.0.1:5000`. Open that address: you should see **ButlerBot Live Monitor** with numbers changing on their own.

**Step 4. Start the robot in Webots** (with the dashboard still running)
```
.\scripts\launch_webots_twin.ps1
```
Other worlds:
```
.\scripts\launch_webots_twin.ps1 -World corner90      # sharp corner
.\scripts\launch_webots_twin.ps1 -World plus          # crossroads
.\scripts\launch_webots_twin.ps1 -World warehouse     # warehouse + trolley
```
Full list: `butlerbot` (the S, default), `corner90`, `corner_mix`, `widen`, `plus`, `t_end`, `gap45`, `warehouse`, `warehouse_obstacles`, `radar_motion`. You can also open Webots yourself and use **File → Open World → `webots/worlds/butlerbot.wbt`**, then press Play.

✅ **It worked if** the robot starts driving along the yellow lane and the dashboard's **Digital Twin Bridge** panel shows **`FEED: webots`**.

**Step 5. Stop everything**

1. Close Webots. When it asks to save the world, choose **No / Don't save** (saving writes the robot's position into the world file).
2. Stop the dashboard and database:
   ```
   .\scripts\stop.ps1
   ```

**Optional extras**
- One-click demo (starts the dashboard if needed, turns on demo highlighting, opens Webots): `.\scripts\launch_demo.ps1`
- Extra ROS 2 test container (not needed for the demo): `.\scripts\start.ps1 -Profile full`
- Run the dashboard without Docker: follow [B](#b-run-the-lightweight-simulator-and-tests-no-webots) steps 1-2 and "Optional: run the database tests too" steps 1-2, then:
  ```
  $env:DATABASE_URL="postgresql://robot:robot@localhost:5432/robot_battery"
  python run_dashboard.py
  ```
  Press Ctrl+C to stop it.
- Terminal summary of the latest readings (needs the database running and `DATABASE_URL` set as above): `python robot_battery_monitor.py --summary`

---

### D. Linux or macOS (untested)

> **Untested.** The project has only been run on Windows. These scripts exist and should work, but nobody has run this path end to end yet. If you try it, please open an issue with what happened.

Same as C, with these differences:

1. Install [Docker](https://docs.docker.com/get-docker/) (Docker Desktop on Mac), [Webots](https://cyberbotics.com/) and Python 3.12. On Linux the launcher looks for `webots` on your PATH or in `/usr/local/webots` (`WEBOTS_HOME` overrides it).
2. Use the `.sh` scripts:
   ```
   ./scripts/start.sh                    # dashboard + database
   ./scripts/launch_webots_twin.sh       # robot (default S world)
   ./scripts/launch_webots_twin.sh http://127.0.0.1:5000 corner90   # another world
   ./scripts/stop.sh                     # stop dashboard + database
   ```
3. `./scripts/start.sh full` adds the optional ROS 2 container.
4. Note: `launch_webots_twin.sh` closes any Webots window that is already open before starting a new one.

---

### E. Developer / contributor workflow

**Before you change anything** read [`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md). It lists the parts that must not be rewritten casually (stop/ABS and power core) and how credits are recorded.

1. **Set up** as in [B](#b-run-the-lightweight-simulator-and-tests-no-webots) (venv, `pip install -r requirements.txt pytest`, Postgres up).
2. **Branch** from `main` with a descriptive name, following the existing pattern:
   ```
   git checkout main && git pull
   git checkout -b feature/my-change
   ```
3. **Make one coherent change at a time.** Driving changes should be checked offline first:
   ```
   python scripts/rowfit_sim.py corner90 corner_mix --table --quiet
   ```
   then in Webots on the matching world.
4. **Run the tests** before pushing: `python -m pytest` (or one file: `python -m pytest tests/test_lane_keep.py`). New tests that need Postgres should use the `clean_database` fixture or `@pytest.mark.db` so they skip cleanly on machines without a database.
5. **Open a pull request into `main`.** GitHub Actions ([`.github/workflows/tests.yml`](.github/workflows/tests.yml)) runs the full test suite on Python 3.12 against a Postgres 16 service for every push and pull request to `main`. In CI a missing database is an error, never a skip (`RBM_REQUIRE_DB=1`). Merge only when it is green.
6. **Restore points:** before and after a feature, the project tags `main` as `save-YYYY-MM-DD-<name>` (for example `save-2026-10-05-pre-cruise-v1` and `save-2026-10-05-cruise-v1`). To look at an old state: `git checkout save-2026-10-05-cruise-v1`. To list them: `git tag -l "save-*"`.
7. **Webots worlds are generated.** Track worlds come from `tracks/*.json` via `scripts/track_builder.py`, the S from `scripts/s_track.py --write-world`, and the warehouse from `scripts/warehouse_builder.py` (see [`docs/TRACKS.md`](docs/TRACKS.md), [`docs/WAREHOUSE.md`](docs/WAREHOUSE.md)). Regenerate instead of hand-editing, and never save a world from the Webots exit prompt.
8. **Never commit secrets.** Settings go in `.env` (ignored by git); `.env.example` is the template.
9. **Log the story.** When you finish a meaningful arc (or get stuck on one), add an entry to [`JOURNEY.md`](JOURNEY.md).

---

### F. Overnight optimizer (Windows)

A script you can start before bed. It runs warehouse laps one after another, changes **one** allowlisted setting at a time, and leaves `optimizer/runs/morning.md` for the morning. Webots has to be on this PC. The cloud and Linux cannot launch it.

What it is allowed to change, in `config/config.yaml`:

| Key | What it does |
| --- | --- |
| `bom_build` | `light`, `heavy`, or `budget` (the launcher applies that mass) |
| `controller.cruise_m_s` | Rowfit curve / lane cruise |
| `controller.straight_m_s` | Straightaway cap |
| `controller.pivot_w_max_rad_s` | In-place turn rate |
| `controller.k_steer` | How hard the steer command is |

It does **not** edit the energy model or `config/bom/bom.json` to chase a better score. A change that looks good is checked on another BoM build (or a second lap, when the change was the build itself). Ideas for code changes are written in the report and are not committed. When the loop stops, those five keys are put back to what they were at the start.

```
powershell -ExecutionPolicy Bypass -File scripts\start.ps1
powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1
```

`overnight_optimize.ps1` starts the dashboard if it is down, then runs laps with `RBM_MISSION_LAPS=1` through `scripts\launch_webots_twin.ps1` (warehouse world). Defaults are 10 iterations, 8 hours, 40 minutes per lap. Shorter night:

```
powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1 -MaxIterations 4 -MaxHours 6
```

In the morning open `optimizer/runs/morning.md`. The same tries are in `results.jsonl` and `results.csv`. Learnings are appended to `optimizer/KNOWLEDGE.md`. If you want a confirmed setting kept, copy it into `config.yaml` yourself.

A no-Webots smoke test (synthetic numbers, not a measurement):

```
python -m optimizer --dry-run --max-iterations 2
```

An optional later picker can shell out to a command you supply. Nothing in the repo holds an API key.

```
set OPTIMIZER_LLM_CMD=python my_picker.py
powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1 -Picker llm
```

The command reads a JSON request on stdin and prints one JSON proposal (`param`, `value`, `hypothesis`, `expected_effect`). A proposal outside the allowlist is ignored and the rule picker is used instead.

---

## 6. Reading the dashboard

| Panel | Meaning |
|-------|---------|
| **Main Battery** | The big battery % for the whole robot |
| **Power Channels** | Watts, amps and status for Legs (drive wheels), Arms, Torso, Compute, Cooling |
| **Safety & Thermal Status** | Green = OK. Yellow/red = high draw, low battery or heat |
| **Power Allocation** | How the power budget is split across channels |
| **LRU / Requirements** | Parts grouped into units ("Line Replaceable Units") and whether each is within its limit for the current task |
| **Mission / Energy Forecast** | What the robot is doing and how long the battery should last at this rate |
| **Digital Twin Bridge** | Where the numbers come from: `SOURCE: INTERNAL` (built-in simulation) or `FEED: webots` (the Webots robot) |
| **Efficiency strip** | Mission time, energy used (Wh) and distance; starts with the first Webots reading, resets with a battery reset |
| **ROS2** | Usually says MOCK. That is normal |
| **History charts** | Trends saved in Postgres (power, battery, missions, anomalies) |

---

## 7. Settings you might change

All in `.env` (copied from [`.env.example`](.env.example)). After editing, restart: `.\scripts\stop.ps1` then `.\scripts\start.ps1` (or the `.sh` versions).

| Setting | Default | What it does |
|---------|---------|--------------|
| `HARDWARE_MODE` / `HARDWARE_TYPE` / `ROS2_MOCK` | `real` / `ros2` / `true` | The full realistic power loop with mocked ROS 2. **Use this with Webots.** Set `HARDWARE_MODE=simulator` for simple random numbers (dashboard check only; Webots readings are not used) |
| `DASHBOARD_PORT` | `5000` | Change if port 5000 is busy |
| `POSTGRES_PASSWORD` | `robot` | Dev-only password. Change it (and `DATABASE_URL`) for anything beyond your own laptop |
| `RBM_API_TOKEN` | empty | Optional lock on the buttons and API. If set, open `http://127.0.0.1:5000/?token=<value>` once in the browser. Webots and the scripts pick it up automatically |
| `RBM_ROUTE` | unset | Which way the robot goes at each junction, e.g. `L,S,R` |
| `RBM_LANE_MODE` | `rowfit` | Lane-following method (leave as is) |

Deeper knobs live in [`config/config.yaml`](config/config.yaml) and the hardware profiles in `config/hardware_profiles/`.

**Security defaults:** the dashboard and database listen on `127.0.0.1` only (not reachable from other computers). `/api/demo/launch-webots` only answers local requests.

---

## 8. Common problems

| Problem | Fix |
|---------|-----|
| `start.ps1 cannot be loaded because running scripts is disabled` | Run it as `powershell -ExecutionPolicy Bypass -File scripts\start.ps1` (same for the other `.ps1` scripts), or once: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` |
| `./scripts/start.sh: Permission denied` (Mac/Linux) | Run `chmod +x scripts/*.sh` once (or use `sh scripts/start.sh` / `bash scripts/launch_webots_twin.sh`) |
| `docker: command not found` / "Cannot connect to the Docker daemon" | Start Docker Desktop and wait for **Engine running** |
| `Port 5000 already in use` | Close the other program, or set `DASHBOARD_PORT=5001` in `.env` and open `http://127.0.0.1:5001` |
| `set POSTGRES_PASSWORD in .env` error from Docker | You have no `.env`. Run `copy .env.example .env` (Windows) or `cp .env.example .env` |
| Dashboard page doesn't load | Wait 30 s and refresh. Check `docker compose logs dashboard` |
| Many tests `skipped` with "Postgres test database not reachable" | Normal without a database. To run them, see "Optional: run the database tests too" in [B](#b-run-the-lightweight-simulator-and-tests-no-webots) |
| Twin panel stays `SOURCE: INTERNAL` while Webots runs | Start the dashboard **before** Webots. Check `http://127.0.0.1:5000/api/twin/schema` opens in the browser. Make sure `HARDWARE_MODE` is not `simulator` |
| Webots console: `Twin publish failed` | Same as above: the dashboard isn't reachable |
| Webots: controller fails to start / Python not found | Install Python 3.12 (64-bit) and make sure `python` works in a new terminal; in Webots check **Tools → Preferences → Python command** |
| Obstacle or warehouse worlds error on `numpy` | `pip install numpy` (and optionally `opencv-python`) for the Python that Webots uses |
| Robot falls through the floor | Reset the simulation (Ctrl+Shift+T) |
| `git status` shows a changed `.wbt` file | You saved the world on exit. Undo with `git checkout -- webots/worlds/<file>.wbt` |

---

## 9. Where things live (folder map)

| Folder / file | Contents |
|---------------|----------|
| `run_dashboard.py` | Starts the dashboard app (database → power loop → web page) |
| `robot_battery_monitor.py` | Terminal reports from the database (`--summary`, `--history Legs`, `--report`) |
| `src/` | All the Python logic: dashboard, power loop, safety, agent, lane vision, steering, obstacles, parts catalog |
| `src/twin/` | The Webots ↔ dashboard bridge and power model |
| `webots/` | Simulator worlds and the robot controller ([`webots/README.md`](webots/README.md)) |
| `tracks/` | Test track definitions (JSON) |
| `scripts/` | Start/stop scripts, Webots launchers, offline simulator, measurement tools, world builders |
| `optimizer/` | Overnight lap loop, knowledge notes, morning report (`runs/` is local, not committed) |
| `config/` | `config.yaml` settings and hardware profiles |
| `data/parts.db` | The parts catalog (SQLite) |
| `tests/` | Automated tests (`tests/fake_webots/` stands in for Webots) |
| `docs/` | Design notes, baselines, run history and architecture |
| `JOURNEY.md` | Dated project log: how the project was built with AI tools, what went wrong, what worked |
| `docker-compose.yml`, `Dockerfile` | The Docker setup (dashboard + Postgres, optional ROS 2) |

---

## 10. Credits and AI collaboration

This repo is a **human + AI co-build** (credit where credit is due):

| Who | Role |
|-----|------|
| **Daniel** ([Daniel7505](https://github.com/Daniel7505)) | Owner — goals, visual ground truth, pacing, final say |
| **Grok Build 4.5** (xAI) | Majority of the deep build — twin bridge, teleop/ABS/stop reliability, power feed, agent, Webots controller work, tests, architecture docs |
| **Grok Bot** | Repo review, tests and CI, security hardening, navigation features (junctions, radar, warehouse), and this documentation refresh (Oct 2026 onward) |
| **Grok Web ~4.4 → 4.5** (xAI) | Project kickoff and early product framing |

For future models (and humans doing a serious review), the standing fence is:

**[`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md)** — look here first; do not silently rewrite the stop/power core; append your lineage row when you ship a meaningful arc.

---

## 11. Project journey

How this project got here (copy-paste chat sessions → repo-aware chat → a coding agent in the terminal → a full agent with GitHub and CI), with dated milestones, the problems along the way, and what each step taught: **[`JOURNEY.md`](JOURNEY.md)**.
