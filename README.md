# 🤖 Robot Battery Monitor

A clean, production-oriented battery and power monitoring system for robots.  
It supports both **simulator mode** (for development) and **real hardware** (ROS2, Serial, CAN, etc.).

The goal is to give you clear visibility into how different parts of your robot are consuming power over time, with good structure, testing, and extensibility.

---

## What This Project Does

- Monitors battery percentage and power draw across multiple channels (Legs, Arms, Torso, Compute, etc.)
- Works in **simulator mode** out of the box
- Designed to be extended with real hardware (ROS2, Serial BMS, CAN, etc.)
- Includes a live web dashboard with WebSocket updates
- Has solid test coverage (31+ tests)
- Docker ready

---

## Quick Start (Simulator Mode)

```bash
# 1. Clone the repo
git clone https://github.com/Daniel7505/robot-battery-monitor.git
cd robot-battery-monitor

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run the dashboard
python run_dashboard.py
Then open your browser and go to:
http://127.0.0.1:5000
You should see live updating battery and power data.

Project Structure
textrobot-battery-monitor/
├── src/
│   ├── hardware.py          # Core hardware abstraction (Simulator + RealHardwareSource)
│   ├── hardware_ros2.py     # Example structure for ROS2 hardware
│   ├── database.py          # SQLite logging + archiving
│   ├── config.py            # Centralized config with env var support
│   ├── dashboard.py         # Web dashboard + WebSocket
│   └── logger.py
├── tests/                   # Test suite (31+ tests)
├── config/
│   └── config.yaml          # Main configuration
├── run_dashboard.py         # Main entry point
├── robot_battery_monitor.py # CLI tool
├── Dockerfile
└── README.md

Running Tests
Bash# Run all tests
python -m pytest tests/ -v

# Run specific test files
python -m pytest tests/test_hardware.py -v
python -m pytest tests/test_websocket.py -v

Hardware Modes
Simulator Mode (Default)

No hardware needed
Generates realistic fake data
Great for development and testing

Real Hardware Mode
Set in config/config.yaml:
YAMLhardware:
  mode: "real"
  type: "ros2"        # or "generic", "serial", etc.
The system is built around RealHardwareSource.
You can create new hardware types by subclassing it.
See the section below: 🔌 Adding Support for New Hardware

🔌 Adding Support for New Hardware (ROS2, Serial, CAN, etc.)
This project was designed to make adding new hardware types reasonably straightforward.
Core Idea
All real hardware should inherit from RealHardwareSource.
This gives you validation, logging, health checks, and structure for free.
Steps to Add New Hardware

Create a new file in src/ (example: src/hardware_ros2.py)
Create a class that inherits from RealHardwareSource
Override these two methods:
_read_raw_data() — Get data from your hardware
_parse_data() — Convert it into the expected format

Update config/config.yaml:YAMLhardware:
  mode: "real"
  type: "ros2"

Current ROS2 Support
We have included a basic example structure in src/hardware_ros2.py.
Important:
At this stage, the ROS2 integration is a starting template, not a fully working ROS2 node.
It shows the correct architecture and where to put your ROS2 subscriber logic.
Full ROS2 integration (including proper node spinning, Docker networking, and message definitions) is planned for future work.

Docker
A Dockerfile is included for containerized runs.
Bashdocker build -t robot-battery-monitor .
docker run -p 5000:5000 robot-battery-monitor
Note: The current Dockerfile works well for simulator mode.
ROS2 + Docker has additional complexity and is not fully set up yet.

CLI Tool
You can also use the command-line tool:
Bash# Show current status
python robot_battery_monitor.py --summary

# Show history for a channel
python robot_battery_monitor.py --history Legs

# Manually archive old data
python robot_battery_monitor.py --archive

Current Status

✅ Simulator mode works well
✅ WebSocket live dashboard
✅ Good test coverage (31+ tests)
✅ Clean structure for adding real hardware
⚠️ ROS2 support is currently at the example/structure level
⚠️ Full ROS2 + Docker integration is not complete yet


Future Work

Improve ROS2 integration (subscriber + Docker support)
Add more hardware examples (Serial BMS, CAN)
Enhance dashboard with historical charts
Add end-to-end browser testing (Selenium/Playwright)


Contributing
If you're adding support for a new robot or sensor, feel free to open an issue or start a discussion.
We're happy to help make the integration clean.

Built with the goal of being understandable, testable, and extensible.