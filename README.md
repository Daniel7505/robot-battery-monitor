# 🤖 Robot Fleet Energy Monitor

**Real-time multi-channel battery and power consumption monitoring system for humanoid and mobile robots.**

---

## ✨ Features

- Live multi-channel monitoring with dark theme dashboard
- Realistic simulator with spikes and battery drain
- Hardware Abstraction Layer (Simulator ↔ Real hardware / ROS2)
- SQLite database with automatic archiving
- **Full Docker support** (one-command deploy)
- Easy to extend for real robotics teams

---

## 🚀 Quick Start (Local)

```powershell
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
pip install -r requirements.txt
python run_dashboard.py
Open: http://127.0.0.1:5000

🐳 Docker (Recommended)
PowerShell# 1. Build and run
docker-compose up --build -d

# 2. Open dashboard
http://127.0.0.1:5000
Useful commands:
PowerShelldocker-compose logs -f          # Live logs
docker-compose down             # Stop
docker-compose up --build -d    # Restart with changes

🔧 Switching Simulator ↔ Real Hardware
Edit config/config.yaml:
YAMLhardware:
  mode: "simulator"     # or "real"

📁 Project Structure
text├── config/config.yaml
├── src/
│   ├── dashboard.py
│   ├── hardware.py          # Abstraction layer
│   ├── database.py
│   └── ros2_node.py         # ROS2 publisher (coming soon)
├── run_dashboard.py
├── Dockerfile
├── docker-compose.yml
├── requirements.txt
└── README.md

Built with Grok — May 2026
Ready for real robotics teams!