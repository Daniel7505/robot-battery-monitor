# Architecture diagrams — Robot Battery Monitor / ButlerBot

High-level maps for **first-time professional visitors** (and future maintainers).  
These diagrams describe **what the repo actually does today**, not every long-term north-star idea.

| Diagram | Type | Best question it answers |
|---------|------|---------------------------|
| [1. Pyramid hierarchy](#1-pyramid-hierarchy--how-the-unit-breaks-down) | Hierarchy / decomposition | “What are the major pieces of the robot system?” |
| [2. Layered stack](#2-layered-stack--where-software-sits) | Layered architecture | “How do sensors, twin, PMS, and UI stack?” |
| [3. PMS hub interactions](#3-pms-hub-interactions--who-talks-to-whom) | Systems interaction | “Where does power data and command authority live?” |
| [4. Live twin tick](#4-live-twin-tick-sequence) | Sequence | “What happens every simulation step when Webots is live?” |
| [5. Stop handshake](#5-stop-handshake-sequence) | Sequence | “How does ABS / dashboard Stop actually halt the robot?” |
| [6. Deployed processes](#6-deployed-processes-swimlanes) | Swimlanes | “What runs on my machine when I boot the demo?” |

Related deep dives:

- **AI / cold review fence (start here if you will edit):** [`REVIEW_CHECKLIST.md`](REVIEW_CHECKLIST.md)
- Twin power contract: [`TWIN_POWER_BRIDGE.md`](TWIN_POWER_BRIDGE.md)
- Residual spin / stop reliability: [`STABILITY.md`](STABILITY.md)
- Wheeled energy baseline: [`V1_WHEELED_ENERGY_BASELINE.md`](V1_WHEELED_ENERGY_BASELINE.md)
- Code entry points: root [`README.md`](../README.md) “Code tour”

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
        CH["Power channels<br/>Legs · Arms · Torso · Compute · Cooling"]
        SAFE["Safety + thermal<br/>throttle · degrade · alerts"]
        AGENT["Onboard agent<br/>rules · mission stress · intervention"]
        CTRL["Motion control<br/>teleop · ABS · stop_epoch"]
        SENS["Sensing model<br/>battery · IMU/GPS/encoders · draws"]
        BR["Twin bridge<br/>telemetry in · state/cmd out"]
    end

    subgraph L3["Level 3 — Implementation anchors in this repo"]
        DASH["src/dashboard.py"]
        HAL["src/hardware*.py"]
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
    DATA --> DASH
    CH --> HAL
    SAFE --> HAL
    AGENT --> OBA
    CTRL --> TELE & WCTRL
    SENS --> WCTRL & TWINPKG
    BR --> TWINPKG
    CFG -.-> PMS & TWIN & HW
```

**Plain English:** Level 0 is “the robot project as a whole.” Level 1 is the five big boxes a defense/tech visitor should remember. Level 2 is what those boxes contain. Level 3 is “where do I open a file?”

---

## 2. Layered stack — where software sits

Inspired by layered architecture sketches, **aligned to this codebase** (Docker PMS + host Webots is the common demo).

```mermaid
flowchart TB
    subgraph HWLAYER["Hardware intent layer (real robot later · sim today)"]
        S1["Channel current/voltage model<br/>Legs · Arms · Torso · Compute"]
        S2["Motion sensors<br/>GPS · IMU · wheel encoders"]
        S3["Energy store<br/>main pack model · capacity Wh"]
    end

    subgraph EDGE["Edge / on-robot software path"]
        ROS["ROS2 bridge path<br/>live or MOCK topics"]
        CTRL2["Webots controller<br/>teleop · ABS · power estimate"]
        BR2["ROS2 ↔ Webots / twin feed<br/>optional; HTTP twin is primary today"]
    end

    subgraph TWINL["Digital twin / simulation"]
        WEB["Webots physics<br/>butlerbot.wbt wheeled model"]
        POW["Motion-aware power model<br/>src/twin/webots_power.py"]
    end

    subgraph PMSL["PMS core — Python process in Docker"]
        HAL2["Hardware tick<br/>src/hardware_ros2.py"]
        ALLOC["Power allocator + requirements"]
        SAFE2["Safety / LRU / thermal"]
        AGENT2["Onboard agent"]
        BRIDGE["DigitalTwinBridge<br/>src/twin/bridge.py"]
    end

    subgraph UIL["Dashboard and data"]
        FLASK["Flask + SocketIO<br/>src/dashboard.py"]
        PG["Postgres<br/>readings · allocations · analytics"]
        BROW["Browser operator UI"]
    end

    S1 & S2 & S3 --> ROS & CTRL2
    WEB --> CTRL2
    CTRL2 --> POW
    POW --> BRIDGE
    CTRL2 -->|"POST /api/twin/telemetry"| BRIDGE
    BRIDGE -->|"GET /api/twin/state<br/>drive · stop_epoch · throttle"| CTRL2
    ROS --> HAL2
    BRIDGE --> HAL2
    HAL2 --> ALLOC --> SAFE2 --> AGENT2
    HAL2 --> PG
    FLASK --> BRIDGE
    FLASK --> BROW
    HAL2 --> FLASK
    PG --> FLASK
```

**Compared to older concept sketches:** this project’s live twin path is primarily **HTTP** (`/api/twin/*`), not a mandatory ROS2 bus between Webots and the dashboard. ROS2 is a **hardware mode** for more realistic ticks; Webots is the **physics twin**. Postgres is the Docker-backed store (not a parts catalog SQLite optimizer — that remains future work).

---

## 3. PMS hub interactions — who talks to whom

Pyramid-style **relationship** view: PMS near the center of authority for power state; twin and agent sit above/beside for planning and physics; body systems are channels the PMS budgets.

```mermaid
flowchart TB
    AGENT["Onboard agent<br/>throttle / intervene / mission stress"]
    WEB["Webots digital twin<br/>physics + local teleop/ABS"]
    PMS["Power Monitoring System<br/>PMS core"]

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
    LEGS -.->|"draw estimates"| WEB
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
    participant Op as Operator / keyboard
    participant W as Webots controller
    participant B as DigitalTwinBridge
    participant H as Hardware tick / PMS
    participant UI as Browser dashboard

    Op->>W: I/J/K/L or Space (focus 3D view)
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
    participant Op as Operator
    participant UI as Dashboard
    participant B as Twin Bridge
    participant W as Webots controller
    participant M as Left/right wheel motors

    alt Space in Webots
        Op->>W: Space → ABS sequence
        W->>W: Coast / brake / hard-zero both hubs
        W->>W: Require finite encoders + quiet yaw/hub rates
    else Stop from UI
        Op->>UI: Drive Stop
        UI->>B: POST /api/twin/command {drive_stop: true}
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
| Parts catalog + optimizer from DB | **Not implemented** (older concept art) |
| Bipedal multi-DOF primary model | **Profile exists; wheeled is the trusted baseline** |
| Regenerative braking energy recovery stack | **Not a first-class subsystem today** |

Keeping diagrams honest helps reviewers trust the rest of the engineering narrative.

---

## Suggested reading order for a 10-minute review

1. This page — pyramid + PMS hub  
2. Sequence §4 (live tick)  
3. Sequence §5 + [`STABILITY.md`](STABILITY.md) if control reliability matters  
4. README code tour → open `src/twin/bridge.py` and `src/dashboard.py`
