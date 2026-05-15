import sqlite3
from datetime import datetime
import os

DB_PATH = "logs/robot_battery.db"

def init_db():
    os.makedirs("logs", exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.execute('''
        CREATE TABLE IF NOT EXISTS battery_readings (
            id INTEGER PRIMARY KEY,
            timestamp TEXT,
            channel TEXT,
            battery_level INTEGER,
            power_draw INTEGER,
            inserted_at TEXT
        )
    ''')
    conn.commit()
    conn.close()

def log_channel_reading(channel: str, battery_level: int, power_draw: int = 0):
    conn = sqlite3.connect(DB_PATH)
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn.execute("""
        INSERT INTO battery_readings 
        (timestamp, channel, battery_level, power_draw, inserted_at) 
        VALUES (?, ?, ?, ?, ?)
    """, (now, channel, battery_level, power_draw, now))
    conn.commit()
    conn.close()

def get_all_readings(limit=300):
    conn = sqlite3.connect(DB_PATH)
    rows = conn.execute("""
        SELECT timestamp, channel, battery_level, power_draw 
        FROM battery_readings 
        ORDER BY id DESC LIMIT ?
    """, (limit,)).fetchall()
    conn.close()
    return [{"time": r[0], "channel": r[1], "battery": r[2], "draw": r[3]} for r in rows]

def get_channel_history(channel: str, limit=200):
    conn = sqlite3.connect(DB_PATH)
    rows = conn.execute("""
        SELECT timestamp, battery_level, power_draw 
        FROM battery_readings 
        WHERE channel = ? 
        ORDER BY id DESC LIMIT ?
    """, (channel, limit)).fetchall()
    conn.close()
    return [{"time": r[0], "battery": r[1], "draw": r[2]} for r in rows]

# Initialize
init_db()