# Architecture diagrams — Robot Battery Monitor / ButlerBot

High-level maps for **first-time professional visitors** (and future maintainers).  
These diagrams describe **what the repo actually does today**. The long-term product (parts catalog → design agent → twin eval → real hardware) lives in [`NORTH_STAR.md`](NORTH_STAR.md). Lane-keep is the evaluation harness, not the end product.

| Diagram | Type | Best question it answers |
|---------|------|---------------------------|
| [0. Overview](#0-overview--the-simple-picture) | Context | “What runs where, in plain words?” (same as the README) |
| [1. Pyramid hierarchy](#1-pyramid-hierarchy--how-the-unit-breaks-down) | Hierarchy / decomposition | “What are the major pieces of the robot system?” |
| [2. Data flow](#2-data-flow--what-actually-flows-where) | Data flow by process | “Which file sends what to whom?” (same as the README) |
| [3. PMS hub interactions](#3-pms-hub-interactions--who-talks-to-whom) | Systems interaction | “Where does power data and command authority live?” |
| [4. Live twin tick](#4-live-twin-tick-sequence) | Sequence | “What happens every simulation step when Webots is live?” |
| [5. Stop handshake](#5-stop-handshake-sequence) | Sequence | “How does ABS / dashboard Stop actually halt the robot?” |
| [6. Deployed processes](#6-deployed-processes-swimlanes) | Swimlanes | “What runs on my machine when I boot the demo?” |

Related deep dives:

- **North star (what we are building):** [`NORTH_STAR.md`](NORTH_STAR.md)
- **AI / cold review fence (start here if you will edit):** [`REVIEW_CHECKLIST.md`](REVIEW_CHECKLIST.md)
- Twin power contract: [`TWIN_POWER_BRIDGE.md`](TWIN_POWER_BRIDGE.md)
- Residual spin / stop reliability: [`STABILITY.md`](STABILITY.md)
- Wheeled energy baseline: [`V1_WHEELED_ENERGY_BASELINE.md`](V1_WHEELED_ENERGY_BASELINE.md)
- Code entry points: root [`README.md`](../README.md) setup A (“Just look around”)

---

## How to read these (student / visitor cheat sheet)

| Name | What it emphasizes |
|------|--------------------|
| **Pyramid / hierarchy** | Top = whole unit; each level **breaks down** into parts (not message order) |
| **Layered architecture** | Vertical stack of responsibility (hardware → edge → app → UI) |
| **Systems interaction** | **Who talks to whom** (hub-and-spoke or mesh) |
| **Sequence diagram** | **Time order** of messages for one scenario |
| **Swimlanes** | Same flow, lanes = process / person / machine |

Mermaid is the text format GitHub renders automatically in Markdown.

**Arrow rule used on this page:** in the pyramid (§1) an arrow means *“is made of”*. In every other diagram an arrow means *“sends data or commands to”*. Do not mix the two in one diagram.

---

## 0. Overview — the simple picture

The same diagram as the root README. Three things run on your computer; the browser talks to the dashboard; Webots and the dashboard exchange readings and commands over HTTP.

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

---

## 1. Pyramid hierarchy — how the unit breaks down

This is the “pyramid scheme” style: **top is the total unit**, each level is a coarser-to-finer partition of the same system.  
It is **not** a call order diagram — it is a **decomposition** so a reviewer can zoom from ButlerBot down to channels and control helpers.

```mermaid
flowchart TB
    subgraph L0["Level 0 — Total unit"]
        ROBOT["ButlerBot system<br/>power-aware wheeled service robot + digital twin"]
    end

    subgraph L1["Level 1 — Primary partitions"]
        OPS["Operator interface<br/>Flask dashboard + SocketIO"]
        PMS["Power Management System<br/>allocate · safety · agent · log"]
        TWIN["Digital twin path<br/>Webots + Twin Bridge HTTP"]
        HW["Hardware abstraction<br/>simulator · ROS2 mock/real"]
        DATA["Persistence<br/>Postgres history + analytics"]
    end

    subgraph L2["Level 2 — Major subsystems"]
        CH["Power channels<br/>Legs (= drive wheels) · Arms · Torso · Compute · Cooling"]
        SAFE["Safety + thermal<br/>throttle · degrade · alerts"]
        AGENT["Onboard agent<br/>rules · mission stress · intervention"]
        CTRL["Motion control<br/>lane keep · teleop · ABS · stop_epoch"]
        SENS["Sensing model<br/>battery · IMU/GPS/encoders · draws"]
        BR["Twin bridge<br/>telemetry in · state/cmd out"]
    end

    subgraph L3["Level 3 — Implementation anchors in this repo"]
        DASH["src/dashboard.py"]
        HAL["src/hardware.py (factory)<br/>src/hardware_ros2.py (PMS tick)"]
        PWR["src/power_allocator.py<br/>src/cooling_channel.py"]
        SAF["src/safety_monitor.py<br/>src/lru_monitor.py"]
        DBF["src/database.py<br/>src/analytics.py"]
        VISF["src/lane_vision.py<br/>src/rowfit_control.py"]
        TWINPKG["src/twin/*"]
        TELE["src/teleop_agent.py"]
        OBA["src/onboard_agent.py"]
        WCTRL["webots/.../butlerbot_controller.py"]
        CFG["config/config.yaml + hardware profiles"]
    end

    ROBOT --> OPS & PMS & TWIN & HW & DATA
    OPS --> DASH
    PMS --> CH & SAFE & AGENT
    TWIN --> BR & CTRL & SENS
    HW --> HAL
    DATA --> DBF
    CH --> PWR
    SAFE --> SAF
    AGENT --> OBA
    CTRL --> TELE & WCTRL & VISF
    SENS --> WCTRL & TWINPKG
    BR --> TWINPKG
    CFG -.-> PMS & TWIN & HW
```

**Plain English:** every arrow here means “is made of”, not “sends data to” (for data, see §2). Level 0 is “the robot project as a whole.” Level 1 is the five big boxes a defense/tech visitor should remember. Level 2 is what those boxes contain. Level 3 is “where do I open a file?”

---

## 2. Data flow — what actually flows where

Replaces the older “layered stack” sketch, which showed hardware sensors that do not exist yet, drew the telemetry POST twice, and left out lane vision. Each box names its real file; each arrow is a real call. Items marked * switch on only in the obstacle / warehouse worlds.

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

Notes:

- The live twin path is **HTTP** (`/api/twin/*`), not ROS 2. ROS 2 is a **hardware mode** of the dashboard process (`ROS2_MOCK=true` by default); the optional `ros2-sim` container (`--profile full`) is a topic-level test peer, not a kinematic twin.
- With `HARDWARE_MODE=simulator` the dashboard uses `SimulatorSource` (random draws) and does not use the Webots power feed. Use the default `real` / `ros2` / mock mode with Webots.
- Postgres stores readings and analytics. The design-agent parts catalog is a separate SQLite file (`data/parts.db`).

---

## 3. PMS hub interactions — who talks to whom

Pyramid-style **relationship** view: PMS near the center of authority for power state; twin and agent sit above/beside for planning and physics; body systems are channels the PMS budgets.

```mermaid
flowchart TB
    AGENT["Onboard agent<br/>throttle / intervene / mission stress"]
    WEB["Webots digital twin<br/>physics + local teleop/ABS"]
    PMS["Power Management System<br/>PMS core"]

    BAT["Battery pack model"]
    ARMS["Arms channel"]
    TORSO["Torso channel"]
    COMP["Compute channel"]
    LEGS["Legs / locomotion channel"]
    COOL["Cooling channel"]

    DASH["Flask dashboard<br/>operator + twin API surface"]
    ROS["ROS2 nodes path<br/>mock or live"]
    MOTORS["Wheel / joint command path<br/>in Webots controller"]

    AGENT <-->|"recommendations · twin_auto_apply"| PMS
    WEB <-->|"telemetry · state · commands"| PMS
    PMS --> BAT & ARMS & TORSO & COMP & LEGS & COOL
    PMS <--> DASH
    PMS <--> ROS
    WEB --> MOTORS
    WEB -.->|"draw estimates"| LEGS
    DASH -->|"drive / drive_stop / battery_reset"| WEB
```

**Key relationships (plain English):**

- **PMS** is the hub for *budget, safety, logging, and operator-visible state*.
- **Webots** owns *physics and motor commands*; it does not replace the PMS.
- **Onboard agent** influences throttle / mission behavior through the PMS path — it does not talk to wheel motors by secret side channel.
- **Dashboard** is both a *view* (SocketIO) and a *command surface* (REST twin commands).

---

## 4. Live twin tick sequence

One normal simulation step while the twin is linked.

```mermaid
sequenceDiagram
    autonumber
    participant Ag as Agent / dashboard API
    participant W as Webots controller
    participant B as DigitalTwinBridge
    participant H as Hardware tick / PMS
    participant UI as Browser dashboard

    W->>W: Read GPS, IMU, encoders
    W->>W: Apply teleop / ABS / throttle
    W->>W: Estimate channel draws + battery drain
    W->>B: POST /api/twin/telemetry
    B->>B: Normalize + store PowerFeed
    B-->>W: 200 OK
    W->>B: GET /api/twin/state
    B-->>W: teleop, stop_epoch, throttle, battery override
    H->>B: Pull external power feed if fresh
    H->>H: Allocate · safety · agent · DB write
    H->>UI: SocketIO battery_update payload
    UI->>UI: Refresh channels, twin panel, agent banner
```

---

## 5. Stop handshake sequence

Why a plain `left=0, right=0` is not enough — `stop_epoch` and dual-hub hard-zero (see `STABILITY.md`).

```mermaid
sequenceDiagram
    autonumber
    participant Ag as Agent / dashboard API
    participant B as Twin Bridge
    participant W as Webots controller
    participant M as Left/right wheel motors

    alt Controller stop (finish line / lane lost)
        W->>W: ABS sequence
        W->>W: Coast / brake / hard-zero both hubs
        W->>W: Require finite encoders + quiet yaw/hub rates
    else Stop command
        Ag->>B: POST /api/twin/command {drive_stop: true}
        B->>B: Increment stop_epoch
        W->>B: GET /api/twin/state
        B-->>W: stop_epoch N (new)
        W->>W: Treat new epoch as hard halt
        W->>M: Finite setPosition lock both hubs
    end
    W->>B: POST telemetry speed≈0, Legs idle
    Note over W,M: GPS alone cannot prove stop<br/>pure yaw can read 0 m/s while a hub spins
```

---

## 6. Deployed processes (swimlanes)

What is running when you follow the standard Windows demo boot.

```mermaid
flowchart LR
    subgraph Host["Host Windows"]
        DD[Docker Desktop engine]
        WB[Webots process<br/>butlerbot.wbt]
        BRW[Browser]
    end

    subgraph Compose["docker compose project"]
        PG[(Postgres :5432)]
        DASH[Dashboard container :5000]
    end

    DD --> Compose
    DASH --> PG
    WB -->|"HTTP twin API"| DASH
    BRW -->|"HTTP + SocketIO"| DASH
```

**Boot order (ops):**

1. Docker Desktop running  
2. `.\scripts\start.ps1` → Postgres + dashboard healthy  
3. Open Webots on `webots/worlds/butlerbot.wbt` (prefer a stable user-owned window)  
4. Browser → `http://127.0.0.1:5000`  
5. On Webots exit: **do not save world**

---

## Control diagnostics (for agents / Build Grok)

Live Webots controller publishes ``sensors.control_diag`` each telemetry tick
(faster while ABS park is active). Also mirrored at top-level on twin state:

```
GET http://127.0.0.1:5000/api/twin/state  →  control_diag { ... }
```

Useful fields: ``hub_left_rad_s``, ``hub_right_rad_s``, ``yaw_rate_rad_s``,
``gps_speed_m_s``, ``cmd_left`` / ``cmd_right``, ``abs_active``, ``locks_engaged``,
``stop_epoch_seen``, ``park_holdoff_s``.

Stop quality suite (longer drives + hard park + diag print):

```
.\scripts\stop_suite.ps1
.\scripts\stop_suite.ps1 -DriveSeconds 4 -LongDriveSeconds 6
```

## North star vs implemented (honest scope)

| Concept | Status in this repo |
|---------|---------------------|
| Live multi-channel power monitor + dashboard | **Implemented** |
| Webots wheeled twin + HTTP twin bridge | **Implemented** |
| Onboard agent throttle / intervention | **Implemented** |
| ABS / residual-spin hardened stop path | **Implemented** |
| ROS2 mock/real hardware mode | **Implemented** (depth varies) |
| Real physical BMS / motor controllers | **Interface intent only** |
| Parts catalog + design agent | **First cut:** `src/parts_db.py` + `data/parts.db` (~10 parts), heuristic BOMs in `src/design_agent.py`; twin evaluation of BOMs **not wired yet** |
| Rowfit lane vision + pure-pursuit drive | **Implemented** (sim; `src/lane_vision.py`, `src/rowfit_control.py`) |
| Sharp corners, junctions, routes | **Implemented** (sim; `src/junction_vision.py`, `src/route_policy.py`) |
| Stereo + radar obstacle gate | **Implemented, first pass** (obstacle worlds) |
| Warehouse, trolley, multi-trip shopping list | **Implemented, prototype** (warehouse worlds) |
| Bipedal multi-DOF primary model | **Profile exists; wheeled is the trusted baseline** |
| Regenerative braking energy recovery stack | **Not a first-class subsystem today** |

Keeping diagrams honest helps reviewers trust the rest of the engineering narrative.

---

## Suggested reading order for a 10-minute review

1. This page — §0 overview + §2 data flow  
2. Sequence §4 (live tick)  
3. Sequence §5 + [`STABILITY.md`](STABILITY.md) if control reliability matters  
4. README setup A file list → open `src/twin/bridge.py` and `src/dashboard.py`
