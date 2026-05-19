# src/database.py
"""
Database layer with reliability, performance, and archiving features.
"""

import sqlite3
import os
from datetime import datetime, timedelta
import logging
import shutil

logger = logging.getLogger(__name__)

DB_PATH = "logs/robot_battery.db"
ARCHIVE_DIR = "logs/archives"

def get_db_connection():
    """Context-friendly connection with better settings."""
    conn = sqlite3.connect(DB_PATH, timeout=10)
    conn.execute("PRAGMA journal_mode=WAL")      # Better concurrency
    conn.execute("PRAGMA synchronous=NORMAL")
    conn.execute("PRAGMA cache_size=-64000")     # 64MB cache
    return conn


def init_db():
    os.makedirs("logs", exist_ok=True)
    os.makedirs(ARCHIVE_DIR, exist_ok=True)

    conn = get_db_connection()
    conn.execute('''
        CREATE TABLE IF NOT EXISTS battery_readings (
            id INTEGER PRIMARY KEY,
            timestamp TEXT,
            channel TEXT,
            battery_level INTEGER,
            power_draw INTEGER,
            inserted_at TEXT DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    
    # Production indexes
    conn.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON battery_readings(timestamp)")
    conn.execute("CREATE INDEX IF NOT EXISTS idx_channel ON battery_readings(channel)")
    conn.execute("CREATE INDEX IF NOT EXISTS idx_channel_time ON battery_readings(channel, timestamp)")
    
    conn.commit()
    conn.close()
    logger.info("✅ Database initialized with performance indexes")


def log_channel_reading(channel: str, battery_level: int, power_draw: int = 0):
    """Thread-safe logging with retry."""
    for attempt in range(3):
        try:
            conn = get_db_connection()
            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            conn.execute("""
                INSERT INTO battery_readings 
                (timestamp, channel, battery_level, power_draw)
                VALUES (?, ?, ?, ?)
            """, (now, channel, battery_level, power_draw))
            
            conn.commit()
            conn.close()
            return
        except Exception as e:
            logger.warning(f"DB write failed (attempt {attempt+1}/3): {e}")
            if attempt == 2:
                logger.error("Failed to log reading after retries")
            else:
                time.sleep(0.2)


def archive_old_data(days=30):
    """Move old records to archive to keep main DB fast."""
    try:
        cutoff = (datetime.now() - timedelta(days=days)).strftime("%Y-%m-%d")
        conn = get_db_connection()
        
        # Count records to archive
        count = conn.execute("SELECT COUNT(*) FROM battery_readings WHERE timestamp < ?", 
                           (cutoff,)).fetchone()[0]
        
        if count > 0:
            # Create archive table/file if needed
            archive_path = f"{ARCHIVE_DIR}/battery_archive_{datetime.now().strftime('%Y%m%d')}.db"
            shutil.copy(DB_PATH, archive_path)
            
            # Delete old data
            conn.execute("DELETE FROM battery_readings WHERE timestamp < ?", (cutoff,))
            conn.commit()
            conn.execute("VACUUM")   # Reclaim space
            logger.info(f"✅ Archived {count} old records and vacuumed DB")
        
        conn.close()
    except Exception as e:
        logger.error(f"Archiving failed: {e}")


def get_all_readings(limit=500):
    conn = get_db_connection()
    rows = conn.execute("""
        SELECT timestamp, channel, battery_level, power_draw 
        FROM battery_readings 
        ORDER BY id DESC LIMIT ?
    """, (limit,)).fetchall()
    conn.close()
    return [{"time": r[0], "channel": r[1], "battery": r[2], "draw": r[3]} for r in rows]


def get_channel_history(channel: str, limit=300):
    conn = get_db_connection()
   