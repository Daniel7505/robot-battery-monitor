# 🤖 Robot Fleet Energy Monitor

**Real-time multi-channel battery and power consumption monitoring system for humanoid and mobile robots.**

Tracks the main battery and individual power draws from subsystems (Legs, Arms, Torso, Compute, etc.).

---

## ✨ Features

- Live multi-channel monitoring (Main Battery + per-channel power draw)
- Clean auto-refreshing web dashboard with toggleable history per channel
- Realistic simulator with continuous drain, usage spikes, and charging
- SQLite database with automatic archiving
- One-click Clear Data and Archive & Reset functions
- Fully configurable via `config/config.yaml`
- Proper logging and basic test coverage

---

## 🚀 Quick Start

```powershell
# 1. Clone the repo
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run the system
python run_dashboard.py
Open your browser → http://127.0.0.1:5000

📁 Project Structure
textrobot-battery-monitor/
├── config/config.yaml          # Settings, power channels, thresholds
├── logs/                       # Live logs + SQLite database
├── archives/                   # Timestamped historical backups
├── src/
│   ├── database.py
│   ├── simulator.py
│   └── dashboard.py
├── run_dashboard.py            # Main entry point
├── requirements.txt
├── README.md
└── LICENSE

Safety Disclaimer
This is an educational and development tool.
It is not certified for safety-critical use. Real robot power systems require proper hardware safety mechanisms, redundancy, and professional engineering review.
Use at your own risk.

License
This project is licensed under the MIT License.
See LICENSE for full details.

Roadmap

WebSocket real-time updates (no page refresh)
Per-channel historical graphs
Docker support for easy deployment
ROS2 / MQTT integration for real robots
Alert system (Discord/Telegram)
Expanded test coverage


Built with Grok
Made by Daniel — May 2026