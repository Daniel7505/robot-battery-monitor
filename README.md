# 🤖 Robot Fleet Energy Monitor

**Real-time multi-channel battery and power consumption monitoring system for humanoid and mobile robots.**

Tracks the main battery and individual power draws from subsystems (Legs, Arms, Torso, Compute, etc.).

---

## ✨ Features

- Live multi-channel monitoring with dark theme web dashboard
- Realistic simulator with continuous drain, usage spikes, and charging
- Hardware Abstraction Layer (easy to switch between Simulator and Real hardware)
- SQLite database with automatic archiving
- Full Docker + docker-compose support (recommended)
- ROS2 integration (publishes battery data to topics)
- Fully configurable via `config/config.yaml`

---

## 🚀 Quick Start (Recommended - Docker)

This is the easiest and cleanest way to run everything.

### 1. Clone the repository

```bash
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor
2. Start everything with Docker
Bashdocker-compose up --build -d
3. Open the Dashboard
Open your browser and go to:
http://127.0.0.1:5000
You should see live battery data updating every 2 seconds.
4. (Optional) Watch the ROS2 node
Bashdocker-compose logs -f ros2
You should see messages like:
textPublished → Main: 78.0% | Draws: [18.0, 21.0, 12.0, 6.0]
5. Stop everything
Bashdocker-compose down

🛠 Local Python Setup (Without Docker)
If you prefer running it directly with Python:
Bashgit clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor

pip install -r requirements.txt
python run_dashboard.py
Then open: http://127.0.0.1:5000

🤖 ROS2 Integration
The system includes a ROS2 node that publishes battery data.
Topics Published:

/robot/battery/main_level → Main battery percentage (Float32)
/robot/battery/power_draw → Power draw per channel (Float32MultiArray)

How to use ROS2:

Make sure the system is running:Bashdocker-compose up --build -d
Enter the ROS2 container:Bashdocker exec -it ros2-node bash
Source ROS2 and check topics:Bashsource /opt/ros/rolling/setup.bash
ros2 topic list
ros2 topic echo /robot/battery/main_level


🔧 Switching Between Simulator and Real Hardware
Edit the file config/config.yaml and change the mode:
YAMLhardware:
  mode: "simulator"     # Change to "real" when using actual hardware

📁 Project Structure
textrobot-battery-monitor/
├── config/
│   └── config.yaml              # Main configuration
├── src/
│   ├── dashboard.py             # Web dashboard (Flask)
│   ├── database.py              # SQLite handling
│   ├── hardware.py              # Hardware Abstraction Layer
│   ├── simulator.py             # Battery simulator
│   └── ros2_node.py             # ROS2 publisher node
├── logs/                        # Live logs + database
├── archives/                    # Timestamped backups
├── Dockerfile
├── docker-compose.yml
├── run_dashboard.py
├── requirements.txt
└── README.md

🛠 Common Commands





























CommandDescriptiondocker-compose up --build -dStart everything in backgrounddocker-compose logs -fWatch all logsdocker-compose logs -f ros2Watch only ROS2 nodedocker-compose downStop and remove containersdocker-compose psCheck running containers

📌 Notes

This project currently uses polling for the dashboard (updates every 2 seconds).
The ROS2 node runs alongside the dashboard and reads from the same database.
Everything is designed to be easy to extend for real robotics use.


🛣️ Future Improvements

WebSocket real-time updates (instead of polling)
Historical graphs in the dashboard
Alert system (low battery, high draw, etc.)
Better real hardware integration examples
More ROS2 message types


Built with Grok • May 2026
If you find this useful or build something cool with it, feel free to open an issue or share what you made!