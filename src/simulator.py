# src/simulator.py - Now emits live updates via SocketIO
import random
import time
import threading
from datetime import datetime
from src.database import log_channel_reading

# We import socketio from dashboard (will be set up in run_dashboard)
try:
    from src.dashboard import socketio
except:
    socketio = None

main_battery = 98.0

channels = {
    "Legs": {"draw": 0, "name": "Leg Drive Motors"},
    "Arms": {"draw": 0, "name": "Arm + Gripper"},
    "Torso": {"draw": 0, "name": "Torso & Balance"},
    "Compute": {"draw": 0, "name": "Compute & Sensors"}
}

def simulate_robot_data():
    global main_battery
    print("🤖 Multi-Channel Robot Simulator Started (Real-time mode)")

    while True:
        total_draw = 0

        for channel_id, data in channels.items():
            base_draw = random.uniform(3, data.get("max_draw", 25))
            spike = random.uniform(0, 15) if random.random() < 0.3 else 0
            current_draw = round(base_draw + spike)
            
            data["draw"] = current_draw
            total_draw += current_draw

            drain = total_draw / 25.0
            main_battery = max(5.0, main_battery - drain * 0.08)

            log_channel_reading(channel_id, int(main_battery), current_draw)

        # Emit live update to all connected browsers
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
            except:
                pass

        print(f"   🔋 Main Battery: {int(main_battery)}% | Total Draw: {int(total_draw)}W")

        time.sleep(random.uniform(3, 6))   # slightly faster updates feel more "live"