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
10. [Credits and Licenses](#10-credits-and-licenses)
11. [Project journey](#11-project-journey)

---

## 1. What is this, in plain English?

Imagine a small wheeled robot that carries things around a warehouse. Before anyone buys parts and builds it, this project lets you:

- **Drive it in a 3D simulator** ([Webots](https://cyberbotics.com/), free). The robot follows yellow painted lanes using two downward-facing cameras, turns at corners and intersections, slows for obstacles, and stops cleanly.
- **Watch its "fuel gauge" live** in a web page: battery %, how many watts each body part uses (wheels, arms, torso, computer, cooling), safety warnings, and how long the battery should last.
- **Save the history** in a database so you can look back at past runs.
- **Test ideas without the simulator**: a fast "offline" lane-following simulator and 880 automated tests.
- **Start designing real robots**: a small catalog of real, buyable parts and a first-draft "design agent" that suggests the cheapest parts list for a mission.

The long-term goal ([`docs/NORTH_STAR.md`](docs/NORTH_STAR.md)) is an agent that picks real parts, designs the cheapest robot that can finish a mission, tests it in the simulator, and then builds it for real. Today's robot and dashboard are the **test bench** for that goal. The north star may evolve as the project learns; `NORTH_STAR.md` is the current version.

You **do not need a real robot**. Everything runs on a normal PC.

---

## 2. What you'll see

| Where | What it looks like |
|-------|--------------------|
| **Browser** at `http://127.0.0.1:5000` | The page that shipped after PR #23. Cards for **battery**, **system draw**, **runtime left**, and **energy left**. A **This lap** panel (phase, speed, position, mission time, energy used, distance) with teleop buttons (Forward, Back, Left, Right, Stop, Lane keep) and a **Proof** panel (measured vs model, steer source, cross-track, width and center error). The badge reads **TWIN LIVE** while Webots is posting and **AWAITING TWIN** otherwise. The page polls `GET /api/live` every 500 ms. |
| **Webots window** | ButlerBot (wheeled base, torso, two arms) driving along a yellow lane: by default a double S-curve (about 17.4 m along the lane) that ends at a red finish line at GPS x=16.5 m, or corners, intersections, or a warehouse with boxes. Small camera overlays show what the robot sees. |
| **Dashboard while Webots runs** | The badge changes from **AWAITING TWIN** to **TWIN LIVE**. Battery, system draw, and the This lap numbers come from the simulated robot. |
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

### Mapping the apartment

Stage 1 explores `webots/worlds/apartment.wbt` with no map and no true pose. `apartment_mapper` reads the depth camera (`oak_depth`), the gyro yaw rate (`oak_gyro`), roll and pitch from `oak_imu`, and the two wheel encoders. It builds a 2D occupancy grid and drives to frontiers until none longer than 0.3 m is reachable. A second controller, `apartment_pose_logger`, is the only process allowed to read the simulator pose. The mapper does not open `eval/apartment_ground_truth.json` or the world file. `tests/test_apartment_twin.py` fails the build if it does.

Run artifacts (`map.npy`, `map_meta.json`, `pose_log.csv`, and the logger's `gt_pose_log.csv`) go under `logs/mapping_runs/<timestamp>/`. That directory is gitignored. `logs/mapping_runs/CURRENT` points at the latest folder.

**Windows (Webots).** From the repo root:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World apartment
```

Optional caps, set in the same terminal before launch: `RBM_MAP_MAX_S` (stop after this many seconds), `RBM_MAP_STRIDE` (depth decimation, default 4), `RBM_MAP_DIR` (run folder).

**Score a run.** The scorer is the only tool that reads the tape and the world. It prints occupancy IoU, free-space IoU, coverage %, and mean wall error in cm, and writes `map_vs_ground_truth.png` next to the map.

```powershell
python eval\score_apartment_map.py (Get-Content logs\mapping_runs\CURRENT).Trim()
```

**No Webots.** The same mapper can be driven by a NumPy ray caster (not a Webots render). This is what CI runs:

```powershell
python eval\kinematic_apartment_map.py --steps 1600 --noise
python eval\score_apartment_map.py (Get-Content logs\mapping_runs\CURRENT).Trim()
```

One kinematic run (1600 steps, depth noise on, 64×40 rays) had not finished the frontier search. Honest scores against the tape: occupancy IoU 0.168, free-space IoU 0.673, coverage 91.8%, wall error 17.9 cm. Occupancy IoU stays low because walls are thin and furniture is only partly seen. The VM this was measured on does not have Webots installed, so there is no headless simulator number yet.

The first Webots run (R2025a, fast, no rendering) scored occupancy IoU 0.232, free-space IoU 0.641, coverage 90.5%, wall error 38.9 cm, and printed `MAPPING done` at 355 s. The robot was physically stuck by the bedroom doorway for most of that time. `oak_gyro` reads +0.001 rad/s at rest, and integrating that bias walked the heading about 14° while the hubs were still. Wheel slip after that added false travel, and the warped grid looked closed. The mapper now learns that rest bias and does not integrate it, drops encoder travel when a wall in view does not move, backs up and blacklists the frontier when the body is not progressing, and will not declare done in the few seconds after a slip. A small yaw match of the depth scan against occupied cells (Olson, *Real-Time Correlative Scan Matching*, ICRA 2009) trims whatever is left. The inertial unit's yaw is not fused: in Webots it is a world heading.

The second Webots run, with that bias fix, scored occupancy IoU 0.180, free-space IoU 0.657, coverage 93.8%, wall error 28.4 cm, and printed `MAPPING done` at 229 s. Heading error was about 9° at the end. From 140–200 s the body was pressed into the wall beside the bath opening, inside the depth camera's 0.35 m minimum, so the range image was empty and the old slip check never fired. The encoders counted about 6.5 m. An empty forward view is now contact: that wheel travel is dropped, the drive does not keep going forward, and `done` stays off while the pose is blind or the latest scan fits the map better somewhere else. Doorway goals are the centre of the opening. A bumper was not added to the shared robot; a new collision shape would change warehouse contact. Wheel-motor torque at a speed the hubs are already reaching is rolling friction, not a contact flag, so it is not used as one.

The third Webots run kept heading to about +3°, then printed `MAPPING done` at 88 s after the living room and a corner of the kitchen. Scores: occupancy IoU 0.162, free-space IoU 0.625, coverage 86.7%, wall error 24.9 cm. It stalled on the bedroom doorway, backed up, banned that opening, and finished while it was still driving. Bans now expire, a contact does not black out the whole doorway, and `done` waits for a final scan plus a few seconds stopped with no reachable frontier. Planning inflation is 0.20 m, the tire radius, because 0.26 m plus a smeared jamb leaves no path through the 0.81 m opening. A stall no longer paints a permanent cell 0.28 m ahead of the nose: that point is the opening, while the tire is the part that meets the jamb, and a latched cell there is what closed the door. Contact only reinforces a wall the depth camera has already seen, and a free ray can wear that mark back down. The mapper prints contact, backup, blacklist, goal, and the done reason. Encoder travel is dropped while a forward command is skidding or the view is blind. The reverse that leaves that wall is kept, and it lasts long enough to clear the camera's minimum range. A door jamb is a shoulder to steer past, not a wall to stop for: stopping on the jamb is what turned the robot away from the bedroom door and the hallway. The goal is placed in the room beyond the opening, so the robot drives through instead of spinning in the doorway. The pose logger, which is already a supervisor, quits the world when the mapper writes `mapping_done`.

Borrowed methods, implemented here (not copied from another repo): frontier cells are free cells next to unknown space (Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, IEEE CIRA 1997); the grid is log-odds (Moravec and Elfes, *High Resolution Maps from Wide Angle Sonar*, ICRA 1985; Thrun, Burgard, and Fox, *Probabilistic Robotics*, ch. 9); the drive is pure pursuit (Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01, 1992); the yaw tweak is a reduced correlative scan match (Olson, ICRA 2009). Depth noise matches a Webots `RangeFinder` fraction of max range: 0.002 × 8 m = 1.6 cm. Wheel angles are quantized to 0.0015 rad in the mapper only. The warehouse sensors and the warehouse worlds are unchanged.

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
        API["Flask HTTP<br/>src/dashboard.py"]
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
    API -- "GET /api/live every 500 ms" --> BROWSER
    BROWSER -- "POST /api/twin/command:<br/>drive · drive_stop · battery_reset" --> API
```

How to read it:

1. **Inside Webots** (top): the cameras see the yellow lane, `lane_vision` turns pixels into "where is the lane", `rowfit_control` turns that into steering and speed, and `controller_wheels` drives the motors and handles stopping. The power estimate is computed from what the motors are doing.
2. **Over HTTP** (middle arrows): about twice a second the robot **POSTs** its readings to the dashboard and **GETs** back any commands (drive, stop, throttle). This is plain web traffic. ROS 2 is not needed for it.
3. **Inside the dashboard app**: the Twin Bridge keeps the latest robot reading. The PMS tick (Power Management System) uses that reading when it is fresh, or its own built-in simulation when Webots is not running, then runs the budget, safety and agent rules and saves a snapshot to Postgres.
4. **Out to you**: the browser polls `GET /api/live` every 500 ms, and its Drive/Stop buttons send commands back the same way.

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
| `webots/controllers/butlerbot_controller/butlerbot_controller.py` | Webots entry point. `_run_loop` calls the step modules below |
| `webots/controllers/butlerbot_controller/controller_devices.py` | Device setup, BoM reload, wheel commands |
| `webots/controllers/butlerbot_controller/controller_sensing.py` | IMU, follow camera, GPS / encoder reads |
| `webots/controllers/butlerbot_controller/controller_mission.py` | Route, R-key phases, nadir harvest, mission end |
| `webots/controllers/butlerbot_controller/controller_energy.py` | Pack watt-hours and telemetry POST |
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

- **No Postgres running:** the 27 tests marked `db` are **skipped**, and everything else runs. One more test skips when the `docker` command is not on PATH. ✅ **It worked if** the last line shows `passed` and `skipped`, with no `failed` or `error`.
- **With Postgres:** those 27 run. The only skip is `test_docker_compose_config_valid` when the docker CLI is absent. ✅ **It worked if** the last line shows `passed` (and `skipped` only for that docker check), with no `failed` or `error`. pytest collects 880 tests.

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
.\scripts\launch_webots_twin.ps1 -World warehouse     # warehouse
```
Full list: `butlerbot` (the S, default), `corner90`, `corner_mix`, `widen`, `plus`, `t_end`, `gap45`, `warehouse`, `warehouse_obstacles`, `radar_motion`. You can also open Webots yourself and use **File → Open World → `webots/worlds/butlerbot.wbt`**, then press Play. Every world instances the same robot from [`webots/protos/ButlerBot.proto`](webots/protos/ButlerBot.proto).

✅ **It worked if** the robot starts driving along the yellow lane and the dashboard badge says **TWIN LIVE**.

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

What the overnight loop tunes, in `config/config.yaml`. These are exactly `OPTIMIZER_TUNING_KEYS` in `src/controller_limits.py`:

| Key | What it does |
| --- | --- |
| `controller.cruise_m_s` | Rowfit curve / lane cruise |
| `controller.pivot_w_max_rad_s` | In-place turn rate |
| `controller.k_steer` | How hard the steer command is |
| `controller.coast_enabled` | Coast into a turn instead of braking to a stop |
| `controller.coast_margin_m` | How early that coast starts |

`bom_build` stays `light` for the whole run. `controller.straight_m_s` is still read by the controller and is not tuned. Heavy-build confirm is off: a heavy lap is not a held-out physics check (see [`optimizer/KNOWLEDGE.md`](optimizer/KNOWLEDGE.md)). The loop does **not** edit the energy model or `config/bom/bom.json` to chase a better score. A change that looks good is checked with two fresh light laps (the value before the keep, and the kept value). Ideas for code changes are written in the report and are not committed. When the loop stops, those five tuning keys are put back to what they were at the start.

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

The page polls `GET /api/live` every 500 ms.

| On the page | Meaning |
|-------------|---------|
| **Battery** | Pack percent |
| **System draw** | Sum of channel watts, and percent of the power budget |
| **Runtime left** | How long the pack lasts at the current draw |
| **Energy left** | Watt-hours still in the pack |
| **This lap** | Badge **TWIN LIVE** or **AWAITING TWIN**. Phase, speed, position, mission time, energy used, battery change, distance, average speed. Teleop: Forward, Back, Left, Right, Stop, Lane keep, and a steer bar |
| **Proof** | Measured vs model drive watt-hours, steer source (camera / held / track), cross-track, width error, center error |

The previous engineering view is on branch `archive/engineering-dashboard`. See [`docs/ENGINEERING_DASHBOARD_ARCHIVE.md`](docs/ENGINEERING_DASHBOARD_ARCHIVE.md). That archive stays in the repo.

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
| `RBM_LANE_MODE` | unset | Lane following is row-fit. A leftover `gap` is ignored. |

Deeper knobs live in [`config/config.yaml`](config/config.yaml) and the hardware profiles in `config/hardware_profiles/`.

### Which battery pack?

`battery_capacity_wh()` in `src/hardware_profile.py` is the pack the teleop drain, the simulation driver, and the Webots fallback use.

| `bom_build` | Pack |
| --- | --- |
| blank (the default) | Wheeled YAML pack, **480 Wh** |
| `light` | BoM pack, **240 Wh** (Bioenno 24V 10Ah LiFePO4) |

The overnight optimizer and reference laps #8–#12 use `light`. A blank `bom_build` still means the wheeled 480 Wh YAML pack.

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
| Badge stays **AWAITING TWIN** while Webots runs | Start the dashboard **before** Webots. The page polls `GET /api/live` every 500 ms. Check `http://127.0.0.1:5000/api/twin/schema` opens in the browser. Make sure `HARDWARE_MODE` is not `simulator` |
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
| `src/mapping/` | Apartment occupancy grid, odometry, frontiers, and the exploration loop. No ground truth |
| `eval/` | Apartment scorer and the kinematic depth stand-in. This is the code allowed to read the tape |
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

## 10. Credits and Licenses

This section lists third-party software and assets used by the project, what license applies, and what you must keep when you redistribute or ship binaries. It also records an **attribution audit** (Oct 2026): searched `requirements.txt`, Docker/CI manifests, Webots worlds, bundled images, and source comments/URLs for copied or adapted material.

There is **no** `package.json`, `pyproject.toml`, or ROS 2 `package.xml` in this repository.

### This repository

| Item | Link | License | Notes |
|------|------|---------|--------|
| **robot-battery-monitor** (this repo) | [GitHub](https://github.com/Daniel7505/robot-battery-monitor) | [MIT](LICENSE) | [`LICENSE`](LICENSE) is present (Copyright (c) 2026 Daniel). Keep the MIT notice in copies or substantial portions. |

### Python — pinned in `requirements.txt` (dashboard / Docker)

Installed by `pip install -r requirements.txt` and the [`Dockerfile`](Dockerfile). Matplotlib pulls in additional transitive packages (NumPy, Pillow, fonttools, etc.); run `pip install pip-licenses && pip-licenses --from=mixed` in your environment for the full tree.

| Package | Version (pinned) | Link | License | Redistribution notes |
|---------|------------------|------|---------|----------------------|
| [Flask](https://pypi.org/project/Flask/) | 3.1.0 | https://pypi.org/project/Flask/ | BSD-3-Clause | Retain copyright and license notices. |
| [PyYAML](https://pypi.org/project/PyYAML/) | 6.0.2 | https://pyyaml.org/ | MIT | Retain copyright and permission notice. |
| [matplotlib](https://pypi.org/project/matplotlib/) | 3.10.0 | https://matplotlib.org/ | [PSF / Matplotlib license](https://matplotlib.org/stable/project/license.html) (BSD-style) | Retain the Matplotlib Development Team copyright notice; brief change summary if you distribute a derivative of matplotlib itself. |
| [psycopg2-binary](https://pypi.org/project/psycopg2-binary/) | 2.9.10 | https://www.psycopg.org/ | **LGPL-3.0-or-later** (library) | **Copyleft (weak).** Shipping the prebuilt wheel (e.g. in a Docker image) triggers LGPL obligations: provide license text, allow relinking/replacement of the library, and document how users can substitute their own build. See the [psycopg2 license](https://www.psycopg.org/license/). |

### Python — CI and tests only

| Package | Where | Link | License | Notes |
|---------|-------|------|---------|--------|
| [pytest](https://pypi.org/project/pytest/) | [`.github/workflows/tests.yml`](.github/workflows/tests.yml) (`pip install pytest`) | https://docs.pytest.org/ | MIT | Not in `requirements.txt`; used only in CI/local test runs. |

### Python — optional on the host (Webots / scripts)

Not pinned in `requirements.txt`; install manually when using obstacle worlds, `scripts/warehouse_builder.py`, or host-side Webots controllers (see [`webots/README.md`](webots/README.md)).

| Package | Link | License | Notes |
|---------|------|---------|--------|
| [NumPy](https://pypi.org/project/numpy/) | https://numpy.org/ | BSD-3-Clause | Used by `src/stereo_depth.py`, `src/stereo_synth.py`, tests, and warehouse tooling. Also installed transitively via matplotlib in Docker. |
| [opencv-python](https://pypi.org/project/opencv-python/) | https://opencv.org/ | Apache-2.0 | Optional faster stereo backend (`StereoSGBM` in `src/stereo_depth.py`). Not required for CI (numpy fallback). |
| [Pillow](https://pypi.org/project/pillow/) | https://python-pillow.org/ | HPND (permissive) | Used only when regenerating textures/previews via `scripts/warehouse_builder.py` (`write_prop_texture`, `draw_preview`). |

### Docker images and base OS packages

| Component | Where | Link | License | Notes |
|-----------|-------|------|---------|--------|
| [python:3.12-slim](https://hub.docker.com/_/python) | [`Dockerfile`](Dockerfile) | https://www.python.org/psf/license/ | Python Software Foundation License | Base image for the dashboard container. |
| Debian packages (`libglib2.0-0`, `libsm6`, `libxext6`, `libxrender-dev`) | [`Dockerfile`](Dockerfile) `apt-get` | https://www.debian.org/legal/licenses/ | Mixed (mostly GPL/LGPL for libs) | Runtime deps for matplotlib; standard container practice is to ship with the image and honor Debian copyright files in `/usr/share/doc/`. |
| [postgres:16-alpine](https://hub.docker.com/_/postgres) | [`docker-compose.yml`](docker-compose.yml) | https://www.postgresql.org/about/licence/ | PostgreSQL License (permissive, BSD-like) | Database service; not linked into application Python code. |
| [postgres:16](https://hub.docker.com/_/postgres) | [`.github/workflows/tests.yml`](.github/workflows/tests.yml) CI service | same | same | CI-only Postgres instance. |
| [osrf/ros:rolling-ros-base](https://hub.docker.com/r/osrf/ros) | [`docker-compose.yml`](docker-compose.yml) (`profile: full`) | https://www.ros.org/ | Mostly Apache-2.0 and BSD-3-Clause per package | Optional ROS 2 peer; not used in default `docker compose up`. |

### GitHub Actions (CI)

| Action / image | Where | Link | License |
|----------------|-------|------|---------|
| [actions/checkout@v4](https://github.com/actions/checkout) | [`.github/workflows/tests.yml`](.github/workflows/tests.yml) | https://github.com/actions/checkout | MIT |
| [actions/setup-python@v5](https://github.com/actions/setup-python) | same | https://github.com/actions/setup-python | MIT |

### Webots simulator and PROTO assets

| Item | Link | License | Notes |
|------|------|---------|--------|
| [Webots](https://cyberbotics.com/) (host install, not in repo) | https://cyberbotics.com/ | [Apache-2.0](https://github.com/cyberbotics/webots/blob/master/LICENSE) (open-source distribution) | Worlds and controllers target Webots **R2023b+** (tested with **R2025a**). You install Webots separately; it is not vendored in git. |
| **Parquetry.proto** | `EXTERNPROTO` in all `webots/worlds/butlerbot*.wbt` → [Cyberbotics Webots R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/appearances/protos/Parquetry.proto) | Apache-2.0 (Webots project) | Loaded from GitHub at world load time; not copied into this repo. |
| **Floor.proto** | same pattern → [Floor.proto](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto) | Apache-2.0 (Webots project) | same |

All ButlerBot `.wbt` worlds under [`webots/worlds/`](webots/worlds/) reference only these two external PROTOs, plus the local [`webots/protos/ButlerBot.proto`](webots/protos/ButlerBot.proto). That robot PROTO, the track paint, the warehouse layout, and the controllers in this repo are **original to this project** (generated or authored here).

### Bundled images and models (in git)

| Asset | Path | Origin | License |
|-------|------|--------|---------|
| Obstacle prop speckle texture | [`webots/worlds/textures/obstacle_speckle.png`](webots/worlds/textures/obstacle_speckle.png) | Generated by `scripts/warehouse_builder.py` (`write_prop_texture`, deterministic NumPy noise) | Same as this repo (**MIT**) |
| Warehouse layout preview | [`docs/warehouse_preview.png`](docs/warehouse_preview.png) | Generated by `scripts/warehouse_builder.py` (`draw_preview`) | Same as this repo (**MIT**) |
| Track calibration / paths | `webots/worlds/s_track_path.csv`, `webots/worlds/track_calibration.json`, [`tracks/*.json`](tracks/) | Project data | Same as this repo (**MIT**) |
| Parts catalog DB | [`data/parts.db`](data/parts.db) | SQLite BOM seeded from project config (`config/bom/`, `src/parts_db.py`); vendor **URLs** point to public product pages, not copied datasheets | Same as this repo (**MIT**); respect third-party site terms when visiting links |

No `.proto` files, font files, 3D mesh assets (`.obj`/`.dae`/`.stl`), or external texture packs are committed besides the PNGs above.

### Attribution audit — code, algorithms, and comments

Searched the tree for `EXTERNPROTO`, `Copyright`, `SPDX`, `adapted`, `copied`, `based on`, and `http(s)://` in source headers. **Findings:**

| Topic | Verdict |
|-------|---------|
| **Copied third-party source files** | None found. No vendored upstream modules besides Python packages installed via pip/Docker. |
| **`tests/fake_webots/controller.py`** | Original minimal stub for CI; not the Cyberbotics Webots `controller` module. |
| **`src/lane_vision.py`** | Original; stdlib-only by design. |
| **`src/rowfit_control.py` / pure pursuit** | Standard geometric path-following algorithm described in docs ([`docs/ROWFIT_LANE_VISION.md`](docs/ROWFIT_LANE_VISION.md)); **implemented in-repo**, not pasted from an external repo. |
| **`src/mapping/`** | Frontier exploration (Yamauchi, IEEE CIRA 1997), log-odds occupancy (Moravec and Elfes, ICRA 1985; Thrun, Burgard, Fox, *Probabilistic Robotics*, ch. 9), pure pursuit (Coulter, CMU-RI-TR-92-01, 1992), and a reduced correlative yaw match (Olson, ICRA 2009). Implemented in-repo from those descriptions. |
| **`src/stereo_depth.py`** | Original pipeline; calls OpenCV `StereoSGBM` when available, else an **original** NumPy block matcher (documented in file header). |
| **Motor / battery constants** | Comments and YAML cite [Pololu](https://www.pololu.com/), [Adafruit](https://www.adafruit.com/), and similar **datasheet URLs** for engineering values — reference only, not copied code. |
| **Hardware BOM JSON** | `config/bom/bom.json` and profiles link to vendor product pages; no scraped proprietary text committed as code. |
| **Archived world snippet** | [`archives/butlerbot_track_boxes_2026-10-01.wbt`](archives/butlerbot_track_boxes_2026-10-01.wbt) is an in-repo archive of an older track representation, not third-party. |

**GPL / strong copyleft in application dependencies:** only **psycopg2-binary (LGPL-3.0)** among pinned Python wheels. No GPL-licensed application code was identified in the repo. **Unlicensed third-party material:** none identified; all bundled assets above are project-authored or generated by project scripts.

**ROS 2:** the live hardware path is [`src/hardware_ros2.py`](src/hardware_ros2.py) through [`src/ros2_bridge.py`](src/ros2_bridge.py). Default runs use `ROS2_MOCK=true` without installing ROS. Optional `osrf/ros` image noted above. The old standalone publisher `src/ros2_node.py` is gone; it duplicated the bridge and had no Cooling channel.

### People and AI tools

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
