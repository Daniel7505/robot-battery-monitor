# 🤖 Robot Fleet Energy Monitor

**Real-time multi-channel battery and power consumption monitoring system for humanoid and mobile robots.**

Tracks the main battery and individual power draws from subsystems (Legs, Arms, Torso, Compute, etc.).

---

## ✨ Features

- Live multi-channel monitoring with dark theme dashboard
- Realistic simulator with continuous drain, usage spikes, and charging
- Hardware Abstraction Layer (easy to switch between Simulator ↔ Real hardware)
- SQLite database with automatic archiving
- **Full Docker + docker-compose support** (one-command deployment)
- Fully configurable via `config/config.yaml`
- Clean logging and basic test coverage

---

## 🚀 Quick Start (Local Python)

```bash
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
pip install -r requirements.txt
python run_dashboard.py
Then open: http://127.0.0.1:5000

🐳 Docker (Recommended for Teams)
Bash# Build and start
docker-compose up --build -d

# View live logs
docker-compose logs -f

# Stop everything
docker-compose down
Open dashboard: http://127.0.0.1:5000

🔧 Switching Between Simulator and Real Hardware
Edit config/config.yaml:
YAMLhardware:
  mode: "simulator"   # Change to "real" when you have actual hardware

📁 Project Structure
textrobot-battery-monitor/
├── config/
│   └── config.yaml
├── src/
│   ├── dashboard.py
│   ├── database.py
│   ├── hardware.py          # ← Hardware Abstraction Layer
│   ├── simulator.py
│   └── ros2_node.py
├── logs/                    # Live logs + SQLite DB
├── archives/                # Timestamped backups
├── Dockerfile
├── docker-compose.yml
├── run_dashboard.py
├── requirements.txt
└── README.md

🛣️ Roadmap

 Docker + docker-compose support
 ROS2 integration (publisher node)
 WebSocket real-time updates (instead of polling)
 Per-channel historical graphs
 Alert system (Discord / Telegram / Email)
 Expanded test coverage


Built with Grok • May 2026
Ready for real robotics teams.