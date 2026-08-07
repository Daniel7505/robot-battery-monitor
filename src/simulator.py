# src/simulator.py
"""
Legacy multi-channel battery simulator (standalone / early prototype).

Role
----
Older real-time loop that randomly varies per-channel draws, drains a global
``main_battery``, logs to the DB, and optionally emits SocketIO
``battery_update`` events.

When to use
-----------
Kept for historical / SocketIO demos. Prefer:
  - ``SimulatorSource`` in ``hardware.py`` for HAL-integrated simple sim
  - ``ROS2BatterySource`` + ``SimulationDriver`` for the ButlerBot mission loop

Not started by ``run_dashboard.py`` unless something imports and runs
``simulate_robot_data()`` explicitly.
"""

import random
import time
import threading
from datetime import datetime
from src.database import log_channel_reading

# SocketIO may not exist when imported outside the dashboard process.
try:
    from src.dashboard import socketio
except Exception:
    socketio = None

main_battery = 98.0

channels = {
    "Legs": {"draw": 0, "name": "Leg Drive Motors"},
    "Arms": {"draw": 0, "name": "Arm + Gripper"},
    "Torso": {"draw": 0, "name": "Torso & Balance"},
    "Compute": {"draw": 0, "name": "Compute & Sensors"}
}


def simulate_robot_data():
    """
    Infinite loop: random draws → shared SOC drain → DB + optional SocketIO.

    Drain heuristic is intentionally simple (not Wh-based) for a lively demo.
    """
    global main_battery
    print("🤖 Multi-Channel Robot Simulator Started (Real-time mode)")

    while True:
        total_draw = 0

        for channel_id, data in channels.items():
            base_draw = random.uniform(3, data.get("max_draw", 25))
            # Occasional spikes (~30%) mimic motor inrush / compute bursts.
            spike = random.uniform(0, 15) if random.random() < 0.3 else 0
            current_draw = round(base_draw + spike)

            data["draw"] = current_draw
            total_draw += current_draw

            # Non-physical drain factor chosen for demo pacing, not Wh accuracy.
            drain = total_draw / 25.0
            main_battery = max(5.0, main_battery - drain * 0.08)

            log_channel_reading(channel_id, int(main_battery), current_draw)

        # Push live update to all connected browsers when dashboard is loaded.
        if socketio:
            try:
                socketio.emit('battery_update', {
                    'main_battery': round(main_battery, 1),
                    'timestamp': datetime.now().strftime("%H:%M:%S"),
                    'channels': [
                        {"id": cid, "draw": data["draw"], "battery": round(main_battery, 1)}
                        for cid, data in channels.items()
                    ]
                })
            except Exception:
                pass

        print(f"   🔋 Main Battery: {int(main_battery)}% | Total Draw: {int(total_draw)}W")

        time.sleep(random.uniform(3, 6))   # slightly faster updates feel more "live"
