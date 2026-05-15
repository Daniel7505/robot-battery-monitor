import argparse
import os
import re
import time
from datetime import datetime
from typing import List

class BatteryLogEntry:
    def __init__(self, timestamp: str, battery_level: int, robot_id: str):
        self.timestamp = timestamp
        self.battery_level = battery_level
        self.robot_id = robot_id

    def __str__(self):
        return f"[{self.timestamp}] {self.robot_id} - {self.battery_level}%"

    def get_status_description(self) -> str:
        if self.battery_level <= 20:
            return "CRITICALLY LOW - Charge immediately!"
        elif self.battery_level >= 95:
            return "Fully Charged"
        else:
            return "Normal Operation"

def parse_log_file(log_path: str) -> List[BatteryLogEntry]:
    """Force fresh read every time"""
    if not os.path.exists(log_path):
        print(f"❌ Log file not found!")
        return []
    
    entries = []
    with open(log_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            match = re.search(r'\[(.+?)\]\s*-\s*(\S+)\s*-\s*(\d+)%', line)
            if match:
                timestamp = match.group(1)
                robot_id = match.group(2)
                try:
                    battery_level = int(match.group(3))
                    entries.append(BatteryLogEntry(timestamp, battery_level, robot_id))
                except:
                    continue
    return entries

def print_live_summary(entries: List[BatteryLogEntry]):
    if not entries:
        return
    levels = [e.battery_level for e in entries]
    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 🔴 LIVE UPDATE — {len(entries)} total readings")
    print(f"Avg: {sum(levels)/len(levels):.1f}% | Lowest: {min(levels)}% | Critical (<20%): {sum(1 for l in levels if l <= 20)}")
    print("-" * 90)
    for e in entries[-8:]:   # Show the most recent 8
        print(f"  {e} → {e.get_status_description()}")

def main():
    print("🤖 Robot Battery Monitor v3.1 — Improved Live Mode\n")
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()

    log_path = 'robot_battery_log.log'

    if args.live:
        print("🔴 LIVE MODE ACTIVE — Updates every 5 seconds")
        print("   (Edit and save robot_battery_log.log to see changes)\n")
        try:
            while True:
                entries = parse_log_file(log_path)
                print_live_summary(entries)
                time.sleep(5)
        except KeyboardInterrupt:
            print("\n👋 Live monitoring stopped.")
        return

if __name__ == "__main__":
    main()