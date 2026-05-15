import random
import time
import threading
from datetime import datetime
from src.database import log_channel_reading

# Persistent state for one robot
main_battery = 98.0

channels = {
    "Legs": {"draw": 0, "name": "Leg Drive Motors"},
    "Arms": {"draw": 0, "name": "Arm + Gripper"},
    "Torso": {"draw": 0, "name": "Torso & Balance"},
    "Compute": {"draw": 0, "name": "Compute & Sensors"}
}

def simulate_robot_data():
    global main_battery
    print("🤖 Multi-Channel Robot Simulator Started (Single Robot - Multiple Systems)")

    while True:
        total_draw = 0

        for channel_id, data in channels.items():
            # Random power draw for each system
            base_draw = random.uniform(3, data.get("max_draw", 25))
            spike = random.uniform(0, 15) if random.random() < 0.3 else 0
            current_draw = round(base_draw + spike)
            
            data["draw"] = current_draw
            total_draw += current_draw

            # Main battery drain based on total usage
            drain = total_draw / 25.0  # Scale factor
            main_battery = max(5.0, main_battery - drain * 0.08)

            # Log this channel
            log_channel_reading(channel_id, int(main_battery), current_draw)

        # Overall log line for compatibility
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open("logs/robot_battery_log.log", "a") as f:
            f.write(f"[{timestamp}] - MAIN - {int(main_battery)}%\n")

        print(f"   🔋 Main Battery: {int(main_battery)}% | Total Draw: {int(total_draw)}W")

        time.sleep(random.uniform(4, 7))