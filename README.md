# Robot Battery Monitor

**Co-built with Grok Build** · Owner: [Daniel7505](https://github.com/Daniel7505) · Kickoff: Grok Web  
**North star:** [`docs/NORTH_STAR.md`](docs/NORTH_STAR.md) · **Review fence:** [`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md)

The repo name is leftover. **What we are building:** an onboard agent that catalogs real purchasable parts, takes a mission, designs the cheapest robot that can finish it, evaluates that design in a Webots twin, and later transfers it to hardware.

What you can run *today* is the **scaffolding** — a power-aware dashboard plus a wheeled ButlerBot twin that can follow a painted S and park on a red mark. That drive stack is the evaluation harness (“can this config go A to B?”), not the product.

Standing map: [`docs/NORTH_STAR.md`](docs/NORTH_STAR.md). Drive baseline: [`docs/LANE_KEEP_BASELINE_2026-08-17.md`](docs/LANE_KEEP_BASELINE_2026-08-17.md).

You do not need a real robot to try the harness. The app runs on simulated data, a Webots twin, or a ROS2-mock path.

---

## What does this project do today?

The live demo is still a robot “car dashboard”:

- Main battery level (percentage) and watts per body part (legs, arms, torso, compute, cooling)
- Warnings when draw, pack, or heat look unsafe
- How long the pack might last; history in Postgres
- Webots twin: teleop, ABS stop, and **lane-keep** on a painted corridor (travel-band, not docking-grade)

**Next harness work:** parts catalog (`python -m src.parts_db`) and a thin design agent (`python -m src.design_agent`). Twin eval of those BOMs is not wired yet. No more lane-keep knobs unless asked.

---

## Architecture (high level — start here for reviews)

**Future AI models / cold reviewers:** read **[`docs/NORTH_STAR.md`](docs/NORTH_STAR.md)** then **[`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md)** — what we are actually building, ownership, do/don’t fence, blast-radius list, and credits. Then architecture.

If you want to understand **how the system fits together** before reading code, open:

**[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)** — pyramid breakdown, layered stack, PMS hub interactions, live-twin sequence, stop handshake, and deploy swimlanes (Mermaid, renders on GitHub).

**Pyramid idea (one glance):** ButlerBot as a whole → operator UI / PMS / twin / hardware / data → channels, safety, agent, motion control → concrete files in this repo.

```mermaid
flowchart TB
    ROBOT["ButlerBot system"]
    ROBOT --> OPS["Operator dashboard"]
    ROBOT --> PMS["Power Management System"]
    ROBOT --> TWIN["Webots digital twin"]
    ROBOT --> HW["Hardware abstraction"]
    ROBOT --> DATA["Postgres history"]
    PMS --> CH["Legs · Arms · Torso · Compute · Cooling"]
    TWIN --> BR["HTTP Twin Bridge"]
    BR <-->|"telemetry / state / commands"| PMS
```

**Data path when Webots is live:**

```
Webots controller  --POST /api/twin/telemetry-->  DigitalTwinBridge  -->  PMS (channels, agent, DB)
Webots controller  <--GET  /api/twin/state-------  (drive, stop_epoch, throttle, battery reset)
Browser            <--SocketIO battery_update----  dashboard broadcast loop
```

**Do not save the Webots world on exit** — it can pollute `butlerbot.wbt` with sim state.

---

## Code tour (for first-time readers)

If you are opening this repo cold (code review, portfolio, or family showcase), start here after the architecture page. Modules have long docstrings that explain **role, data flow, and non-obvious design choices** — especially stop/spin handling and the Webots twin bridge.

| Start here | Why |
|------------|-----|
| [`docs/NORTH_STAR.md`](docs/NORTH_STAR.md) | **What we are building** — harness vs product, next work |
| [`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md) | **AI + human review fence** — do/don’t, stop/power sacred list, credits |
| [`docs/LANE_KEEP_BASELINE_2026-08-17.md`](docs/LANE_KEEP_BASELINE_2026-08-17.md) | Frozen drive card + historical 64 champion |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | High-level diagrams of **today’s** stack (pyramid, layers, sequences) |
| [`docs/PARTS_CATALOG.md`](docs/PARTS_CATALOG.md) | Design-agent SQLite catalog (`src/parts_db.py`) |
| [`docs/DESIGN_AGENT.md`](docs/DESIGN_AGENT.md) | Thin BOM proposer (`src/design_agent.py`) |
| `src/__init__.py` | Package map of the whole PMS |
| `run_dashboard.py` | Process boot: DB → hardware → Flask dashboard |
| `src/dashboard.py` | Operator UI + REST twin APIs + SocketIO broadcast |
| `src/twin/bridge.py` | Arbitration: external Webots vs internal sim; `stop_epoch`; battery override |
| `src/teleop_agent.py` | Pure drive/ABS/throttle math (testable without Webots) |
| `src/onboard_agent.py` | Rules that intervene when the operator pushes power/heat too hard |
| `src/hardware_ros2.py` | Production-like tick: allocate, safety, twin, agent, log |
| `webots/.../butlerbot_controller.py` | Sim step loop, keyboard teleop, residual-spin fixes |
| `config/config.yaml` | Knobs for twin, agent, mission, hardware profile |
| `docs/STABILITY.md` | Why the robot used to spin after stop, and what fixed it |

---

## What you need (prerequisites)

Visitors often assume “clone and go.” In practice, what you install depends on **how deep** you want to go. Use the table first, then the notes.

### Quick matrix

| Tool | Dashboard only (Docker) | Full digital twin (recommended demo) | Dev / no Docker | Notes |
|------|-------------------------|--------------------------------------|-----------------|-------|
| **Git** | Required | Required | Required | Clone this repo |
| **Web browser** | Required | Required | Required | Chrome, Firefox, Edge, etc. |
| **Docker Desktop** | Required | Required | Optional | Runs dashboard + Postgres for you |
| **Webots** | Not needed | **Required** | Optional | Physics twin; runs **on the host**, not in Docker |
| **Python 3.12+** | Not needed* | Not needed* | Required | \*Bundled inside the dashboard image |
| **pip** | Not needed* | Not needed* | Required | Installs `requirements.txt` on host |
| **PostgreSQL** | Not needed* | Not needed* | Required (or Docker just for DB) | \*Compose starts `postgres:16` for you |
| **ROS 2** | Not needed | Not needed | Optional | Default path uses **ROS2_MOCK=true** — no `rclpy` install |
| **Real robot / BMS** | Not needed | Not needed | Not needed | Hardware is simulated or mocked |

\* “Not needed on your machine” because Docker already provides it inside containers.

### Minimum path — live dashboard in a browser

Good for “does this project run?” at someone else’s house.

1. **Git** — [https://git-scm.com/downloads](https://git-scm.com/downloads)  
2. **Docker Desktop** — [https://www.docker.com/products/docker-desktop/](https://www.docker.com/products/docker-desktop/)  
   - Install, reboot if asked, **start Docker Desktop** and wait until it says running  
   - Windows: WSL2 backend is normal; first engine start can take a few minutes  
3. **A browser**  
4. Clone + start (see [How to run with Docker](#how-to-run-with-docker-recommended))  
5. Open `http://127.0.0.1:5000`

You do **not** need a separate Postgres installer, a host Python install, or ROS 2 for this path.

### Full path — dashboard + ButlerBot in Webots

Everything in the minimum path, **plus**:

5. **Webots** (Cyberbotics) — [https://cyberbotics.com/](https://cyberbotics.com/)  
   - Install on Windows/macOS/Linux  
   - Open the world file: `webots/worlds/butlerbot.wbt`  
   - Or use `.\scripts\launch_webots_twin.ps1` / `./scripts/launch_webots_twin.sh` after the dashboard is up  
6. **Dashboard must already be running** on port **5000** so the controller can POST/GET twin APIs  
7. On Webots exit: **do not save the world** (avoids polluting `butlerbot.wbt` with sim pose)

Webots is the piece people forget when they only install Docker.

### Optional / advanced

| Want… | Also install / enable |
|-------|------------------------|
| Edit Python on the host, run tests | **Python 3.12+**, `pip install -r requirements.txt`, often **pytest** |
| Host Postgres without Docker | **PostgreSQL 16** (or close) and set `DATABASE_URL` / `.env` |
| Real ROS 2 topics (not mock) | A **ROS 2** distro on Linux (or the compose `full` profile with `ros2-sim`); set `ROS2_MOCK=false` only if you know what you are doing |
| Compose “full” profile | Still Docker; adds the optional ROS2 sim container — **not** a substitute for Webots |

### Disk / ports / OS reality check

- **Ports used:** `5000` (dashboard), `5432` (Postgres). Free them or change values in `.env`.  
- **Disk:** Docker images + Webots are the large downloads (multi‑GB combined is normal).  
- **OS:** Developed and demoed on **Windows 10/11** with Docker Desktop; Linux/macOS work for Docker scripts; Webots is cross‑platform.  
- **GitHub account:** only needed if you fork/push — **cloning a public repo does not require login**.

### What this project is *not* asking you to buy

- No real humanoid / wheeled hardware  
- No paid cloud GPU  
- No separate “ROS for Windows” install for the default mock demo  

If you only remember three words for a laptop bag visit: **Git, Docker, Webots** (Webots only if you want the moving robot).

---

## How to run with Docker (recommended)

This is the best option for beginners. Docker starts the database and the app for you.  
Prerequisites: see **[What you need](#what-you-need-prerequisites)** (at least **Git + Docker Desktop + browser**).

### Step 0 — Get the code

```
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
```

(If you already have a local copy, just `cd` into that folder.)

### Step 1 — Open a terminal in the project folder

On Windows, open PowerShell in the project folder.

On Mac or Linux, open Terminal in the project folder.

Confirm **Docker Desktop is running** before the next step (whale icon / engine ready).

### Step 2 — Run the start script

Windows:
```
.\scripts\start.ps1
```

Mac or Linux:
```
./scripts/start.sh
```

The first time may take a few minutes while Docker downloads and builds things. That is normal.

### Step 3 — Open the dashboard

When the script finishes, open this address in your browser:

```
http://127.0.0.1:5000
```

You should see a page titled something like "Optimus Unit 1 Live Monitor" with numbers updating on their own.

### Step 4 (optional) — Webots digital twin

Install Webots if you have not already, start it after the dashboard is healthy, and open:

```
webots/worlds/butlerbot.wbt
```

Or from the project folder:

Windows: `.\scripts\launch_webots_twin.ps1`  
Mac/Linux: `./scripts/launch_webots_twin.sh`

### Step 5 — Stop everything when you are done

Windows:
```
.\scripts\stop.ps1
```

Mac or Linux:
```
./scripts/stop.sh
```

Close the Webots window separately (do **not** save the world on exit).

### Optional: run with a ROS2 simulator container too

If you want the extra ROS2 test container (not required for beginners):

Windows:
```
.\scripts\start.ps1 -Profile full
```

Mac or Linux:
```
./scripts/start.sh full
```

### Manual Docker commands (if you prefer)

```
copy .env.example .env
docker compose up --build -d
```

Then open http://127.0.0.1:5000

To stop:
```
docker compose down
```

---

## How to run without Docker

Use this if you want to run Python directly on your computer.

### Step 1 — Install Python packages

```
pip install -r requirements.txt
```

### Step 2 — Start the database

The easiest way is to use Docker for just the database:

```
docker compose up -d postgres
```

Wait about 10 seconds, then set up the database tables:

```
python scripts/setup_postgres.py
```

### Step 3 — Tell the app how to connect to the database

Windows (PowerShell):
```
$env:DATABASE_URL="postgresql://robot:robot@localhost:5432/robot_battery"
```

Mac or Linux:
```
export DATABASE_URL=postgresql://robot:robot@localhost:5432/robot_battery
```

### Step 4 — Start the app

```
python run_dashboard.py
```

### Step 5 — Open the dashboard

```
http://127.0.0.1:5000
```

Press Ctrl+C in the terminal to stop the app.

---

## Hardware profile (wheeled)

Power estimates for the Webots twin are grounded in:

```
config/hardware_profiles/butlerbot_wheeled.yaml
```

Drive motors, stabilizers, compute/sensors, and battery Wh are defined there. See [docs/HARDWARE_PROFILE.md](docs/HARDWARE_PROFILE.md).

---

## Webots twin power bridge

When the Webots ButlerBot twin is running, battery % and channel watts on the dashboard can come **from Webots** instead of the internal simulator.

- **Contract + enable + verify:** [docs/TWIN_POWER_BRIDGE.md](docs/TWIN_POWER_BRIDGE.md)
- **Config:** `digital_twin.prefer_external` + `apply_battery_override` (see `config/config.yaml`)
- **API:** `POST /api/twin/telemetry` → updates `hardware.last_readings` immediately
- **Fallback:** no live twin feed → internal mission simulation continues as before

```powershell
.\scripts\start.ps1
.\scripts\launch_webots_twin.ps1
# Dashboard: power_source should show "webots" while the twin is posting
```

---

## Basic usage — what am I looking at?

Once the dashboard is open, here is what each section means.

**ROS2 Integration**
Shows whether the app is talking to ROS2 (a common robot software system). In most beginner setups this will say "MOCK" — that is fine. It means the app is using built-in test data.

**Safety and Thermal**
Shows if the robot is in a safe power range. Green means OK. Yellow or red means something needs attention (high power draw, low battery, heat, etc.).

**LRU Hierarchy and Requirements**
Groups robot parts into categories and shows if power use fits the expected limits for the current task. LRU just means "a group of related parts treated as one unit."

**Mission**
Shows what the robot is doing right now (idle, moving, high load, etc.) and how much time or battery might be left for that task.

**Energy Forecast**
A short-term guess of future power use and battery level. Helpful for planning.

**Main Battery**
The big battery percentage number.

**Power Allocation**
How the total power budget is split across parts of the robot.

**Historical Analytics**
Summary of saved data from the database (snapshots, averages, etc.).

**Power Channels**
The raw numbers for each channel: Legs, Arms, Torso, and Compute. Shows watts, amps, and status for each.

The page updates automatically. You do not need to refresh it.

---

## How to switch modes (simulator vs ROS2)

The app has two main hardware modes. You pick one depending on what you are testing.

### Simulator mode (simplest — good for first day)

Uses completely fake random-ish data. No ROS2 needed. Easiest to understand.

**Without Docker (Windows PowerShell):**
```
$env:HARDWARE_MODE="simulator"
python run_dashboard.py
```

**Without Docker (Mac/Linux):**
```
export HARDWARE_MODE=simulator
python run_dashboard.py
```

**With Docker:** edit the `.env` file in the project folder:
```
HARDWARE_MODE=simulator
```
Then restart:
```
docker compose down
docker compose up --build -d
```

### ROS2 mode (more realistic — still works without a real robot)

Uses a built-in physics simulation that acts more like a real robot. Can connect to ROS2 topics when available. On Windows and in Docker, it usually runs in "mock" ROS2 mode, which still works well for learning.

**Without Docker (Windows PowerShell):**
```
$env:HARDWARE_MODE="real"
$env:HARDWARE_TYPE="ros2"
$env:ROS2_MOCK="true"
python run_dashboard.py
```

**Without Docker (Mac/Linux):**
```
export HARDWARE_MODE=real
export HARDWARE_TYPE=ros2
export ROS2_MOCK=true
python run_dashboard.py
```

**With Docker:** the `.env` file already defaults to ROS2 mode with mock enabled:
```
HARDWARE_MODE=real
HARDWARE_TYPE=ros2
ROS2_MOCK=true
```

### Quick comparison

Simulator mode:
- Easiest
- Fake data
- Good for "does the dashboard work?"

ROS2 mode (with mock):
- More realistic behavior
- Mission tasks, predictions, safety rules all active
- Good for "how would this work on a real robot?"

You can also change the default in `config/config.yaml` under the `hardware:` section, but using environment variables (shown above) is usually easier.

---

## Other useful commands

**Run tests (for developers):**
```
python -m pytest tests/ -q --ignore=tests/test_websocket.py
```

**Command-line summary (without opening the browser):**
```
python robot_battery_monitor.py --summary
```

**View history for one channel:**
```
python robot_battery_monitor.py --history Legs
```

---

## Project folders (short guide)

- `config/config.yaml` — main settings file
- `run_dashboard.py` — starts the app
- `src/dashboard.py` — the web page and live updates
- `src/hardware.py` — simulator and hardware switching
- `scripts/` — helper scripts for Docker startup
- `tests/` — automated tests
- `docker-compose.yml` — defines the Docker setup

---

## Common problems

**"Port 5000 already in use"**
Something else is using that port. Stop the other program, or change `DASHBOARD_PORT` in `.env` to something like `5001`.

**Dashboard page does not load**
Wait 30 seconds after starting Docker, then try again. Check logs:
```
docker compose logs dashboard
```

**Database connection error (running without Docker)**
Make sure Postgres is running and you set `DATABASE_URL` correctly (see steps above).

---

## Summary

1. Install Docker Desktop
2. Run `.\scripts\start.ps1` (Windows) or `./scripts/start.sh` (Mac/Linux)
3. Open http://127.0.0.1:5000
4. Watch the live power and battery data
5. Try simulator mode first, then ROS2 mode when you are ready

That is it. You do not need to understand every file in the project on day one. Start the app, open the dashboard, and explore.

---

## Credits & AI collaboration

This repo is a **human + AI co-build** (credit where credit is due):

| Who | Role |
|-----|------|
| **Daniel** ([Daniel7505](https://github.com/Daniel7505)) | Owner — goals, visual ground truth, pacing, final say |
| **Grok Build 4.5** (xAI) | Majority of the deep build — twin bridge, teleop/ABS/stop reliability, power feed, agent, Webots controller work, tests, architecture docs |
| **Grok Web ~4.4 → 4.5** (xAI) | Project kickoff and early product framing |

For future models (and humans doing a serious review), the standing fence is:

**[`docs/REVIEW_CHECKLIST.md`](docs/REVIEW_CHECKLIST.md)** — look here first; do not silently rewrite the stop/power core; append your lineage row when you ship a meaningful arc.