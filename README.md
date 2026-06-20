# Robot Battery Monitor

A simple web app that watches how much power a robot uses and how much battery is left.

Think of it like a car dashboard, but for a robot. It tracks power for different body parts (legs, arms, torso, computer) and shows everything on a live web page that updates every few seconds.

You do not need a real robot to try it. The app can run with fake (simulated) data, or in a more advanced ROS2 mode that mimics how a real robot would send data.

---

## What does this project do?

In plain terms:

- Shows the main battery level (as a percentage)
- Shows how many watts each part of the robot is using right now
- Warns you if power draw is too high, battery is low, or something looks unsafe
- Groups parts into bigger categories (locomotion, arms, torso, compute, and the power system)
- Estimates how long the battery might last
- Saves history to a database so you can look back later

Good for learning, testing, and eventually connecting to real robot hardware.

---

## What you need

**To run with Docker (easiest way):**
- Docker Desktop installed and running
- A web browser (Chrome, Firefox, Edge, etc.)

**To run without Docker:**
- Python 3.12 (or close to that)
- pip (comes with Python)
- PostgreSQL (the app can start just the database in Docker for you)

---

## How to run with Docker (recommended)

This is the best option for beginners. Docker starts the database and the app for you.

### Step 1 — Open a terminal in the project folder

On Windows, open PowerShell in the project folder.

On Mac or Linux, open Terminal in the project folder.

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

### Step 4 — Stop everything when you are done

Windows:
```
.\scripts\stop.ps1
```

Mac or Linux:
```
./scripts/stop.sh
```

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