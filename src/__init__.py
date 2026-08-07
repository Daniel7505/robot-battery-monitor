"""
Robot Battery Monitor / ButlerBot — application package root.

This package implements a Power Management System (PMS) for a small mobile
service robot (ButlerBot). Layers, roughly outside-in:

  Entry points
    run_dashboard.py          Live dashboard + hardware telemetry process
    robot_battery_monitor.py  Offline CLI over the same database

  Hardware abstraction (src/hardware*.py)
    Factory selects simulator | ROS2 physics loop | generic real hardware.
    ROS2BatterySource is the production-like path: mission, allocation,
    safety, twin feed, agent, and DB logging all tick together.

  Power & safety
    power_allocator, power_requirements, safety_monitor, lru_monitor,
    cooling_channel, energy_predictor

  Mission & simulation
    mission_tasks, mission_context, mission_forecast, simulation_driver,
    demo_mode

  Integration
    ros2_bridge / ros2_node, twin/* (Webots digital twin), dashboard, database

Import submodules directly (e.g. ``from src.config import config``); this
package does not re-export a public API surface.
"""
