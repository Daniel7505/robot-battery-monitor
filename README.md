# 🤖 Robot Fleet Energy Monitor

**A real-time multi-channel battery and power consumption monitoring system for humanoid and mobile robots.**

Built to track the main battery and individual power draws from different subsystems (Legs, Arms, Torso, Compute, etc.).

---

## ✨ Features

- Live multi-channel monitoring (Main Battery + individual power channels)
- Clean, auto-refreshing web dashboard with toggleable history per channel
- Realistic simulator with continuous drain, usage spikes, and charging events
- SQLite database with persistent historical data
- One-click actions: Clear data, Archive old readings, Toggle history
- Fully configurable via `config.yaml`
- Modular, clean code structure

---

## 🚀 Quick Start

### 1. Clone or download the repo
```powershell
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
2. Install dependencies
PowerShellpip install -r requirements.txt
3. Run the system
PowerShellpython run_dashboard.py
Open your browser and go to: http://127.0.0.1:5000

📁 Project Structure
textrobot-battery-monitor/
├── config/config.yaml          # All settings (thresholds, channels, refresh rate, etc.)
├── logs/                       # Live logs + SQLite database
├── archives/                   # Timestamped historical backups
├── src/
│   ├── database.py             # SQLite handling
│   ├── simulator.py            # Realistic multi-channel simulator
│   └── dashboard.py            # Flask web UI
├── run_dashboard.py            # Main entry point
├── requirements.txt
├── README.md
└── LICENSE

Configuration
Edit config/config.yaml to:

Change robot name
Add/remove power channels
Adjust low battery thresholds
Change dashboard refresh rate


Safety Disclaimer
This is an educational and development tool.
It is not intended for use in safety-critical systems without proper hardware-level safety mechanisms, redundancy, and professional validation.
Use at your own risk.

License
This project is licensed under the MIT License — you are free to use, modify, and distribute it.
See the LICENSE file for details.

Contributing
Pull requests, suggestions, and real robot integration stories are very welcome!
Especially interested in:

ROS2 / MQTT integration
Real hardware testing (Optimus, Digit, Unitree, etc.)
Better visualization ideas


Built with curiosity + Grok
Made by Daniel