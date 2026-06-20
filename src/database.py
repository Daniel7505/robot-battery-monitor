# src/database.py
"""
PostgreSQL database layer for power channel and allocation telemetry.
"""

import json
import os
import time
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone

import psycopg2
import psycopg2.extras

from src.config import config
from src.logger import logger

ARCHIVE_DIR = "logs/archives"

_SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS allocation_snapshots (
    id SERIAL PRIMARY KEY,
    recorded_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    task VARCHAR(32) NOT NULL DEFAULT 'unknown',
    budget_w NUMERIC(8,2),
    total_requested_w NUMERIC(8,2),
    total_allocated_w NUMERIC(8,2),
    utilization_pct NUMERIC(5,2),
    status VARCHAR(16) NOT NULL DEFAULT 'ok',
    main_battery_pct NUMERIC(5,2),
    warnings JSONB NOT NULL DEFAULT '[]'::jsonb
);

CREATE TABLE IF NOT EXISTS channel_readings (
    id SERIAL PRIMARY KEY,
    snapshot_id INTEGER REFERENCES allocation_snapshots(id) ON DELETE SET NULL,
    channel_id VARCHAR(32) NOT NULL,
    recorded_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    battery_level NUMERIC(5,2) NOT NULL,
    power_draw_w NUMERIC(8,2) NOT NULL,
    requested_w NUMERIC(8,2),
    allocated_w NUMERIC(8,2),
    amps NUMERIC(8,2),
    nominal_voltage NUMERIC(6,2),
    max_draw_w NUMERIC(8,2),
    allocation_pct NUMERIC(5,2),
    throttled BOOLEAN NOT NULL DEFAULT FALSE,
    status VARCHAR(16) NOT NULL DEFAULT 'normal'
);

CREATE INDEX IF NOT EXISTS idx_channel_readings_recorded_at
    ON channel_readings (recorded_at DESC);
CREATE INDEX IF NOT EXISTS idx_channel_readings_channel_time
    ON channel_readings (channel_id, recorded_at DESC);
CREATE INDEX IF NOT EXISTS idx_allocation_snapshots_recorded_at
    ON allocation_snapshots (recorded_at DESC);

CREATE TABLE IF NOT EXISTS energy_predictions (
    id SERIAL PRIMARY KEY,
    recorded_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    snapshot_id INTEGER REFERENCES allocation_snapshots(id) ON DELETE SET NULL,
    task VARCHAR(32) NOT NULL,
    battery_pct NUMERIC(5,2) NOT NULL,
    predicted_draw_w NUMERIC(8,2),
    predicted_runtime_min NUMERIC(8,2),
    mission_forecast_min NUMERIC(8,2),
    confidence_pct NUMERIC(5,2),
    mission_energy_ok BOOLEAN,
    mission_battery_pct_at_end NUMERIC(5,2),
    details JSONB NOT NULL DEFAULT '{}'::jsonb
);

CREATE INDEX IF NOT EXISTS idx_energy_predictions_recorded_at
    ON energy_predictions (recorded_at DESC);
"""


def _connection_params() -> dict:
    url = os.getenv("DATABASE_URL") or (config.get("database") or {}).get("url")
    if url:
        return {"dsn": url}

    db_cfg = config.get("database") or {}
    return {
        "host": db_cfg.get("host", "localhost"),
        "port": db_cfg.get("port", 5432),
        "dbname": db_cfg.get("name", "robot_battery"),
        "user": db_cfg.get("user", "robot"),
        "password": db_cfg.get("password", "robot"),
    }


def get_db_connection():
    """Return a new PostgreSQL connection."""
    params = _connection_params()
    if "dsn" in params:
        return psycopg2.connect(params["dsn"])
    return psycopg2.connect(**params)


@contextmanager
def db_cursor():
    conn = get_db_connection()
    cur = conn.cursor()
    try:
        yield conn, cur
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()


def init_db():
    os.makedirs("logs", exist_ok=True)
    os.makedirs(ARCHIVE_DIR, exist_ok=True)

    with db_cursor() as (conn, cur):
        cur.execute(_SCHEMA_SQL)

    from src.analytics import init_analytics_views
    init_analytics_views()
    logger.info("✅ PostgreSQL schema initialized")


def truncate_tables():
    """Clear telemetry tables — used in tests."""
    with db_cursor() as (conn, cur):
        cur.execute(
            "TRUNCATE energy_predictions, channel_readings, "
            "allocation_snapshots RESTART IDENTITY CASCADE"
        )


def _format_time(value) -> str:
    if isinstance(value, datetime):
        if value.tzinfo is None:
            value = value.replace(tzinfo=timezone.utc)
        return value.astimezone().strftime("%Y-%m-%d %H:%M:%S")
    return str(value)


def log_channel_reading(channel: str, battery_level: int, power_draw: int = 0):
    """Log a simple channel reading (simulator / legacy path)."""
    for attempt in range(3):
        try:
            with db_cursor() as (conn, cur):
                cur.execute(
                    """
                    INSERT INTO channel_readings
                        (channel_id, battery_level, power_draw_w, status)
                    VALUES (%s, %s, %s, 'normal')
                    """,
                    (channel, battery_level, power_draw),
                )
            return
        except Exception as e:
            logger.warning(f"DB write failed (attempt {attempt + 1}/3): {e}")
            if attempt == 2:
                logger.error("Failed to log reading after retries")
            else:
                time.sleep(0.2)


def log_power_snapshot(
    allocation: dict,
    readings: dict,
    main_battery: float,
    prediction: dict | None = None,
) -> None:
    """Log allocation snapshot, channel readings, and optional energy prediction."""
    if not allocation or not readings:
        return

    for attempt in range(3):
        try:
            with db_cursor() as (conn, cur):
                cur.execute(
                    """
                    INSERT INTO allocation_snapshots (
                        task, budget_w, total_requested_w, total_allocated_w,
                        utilization_pct, status, main_battery_pct, warnings
                    ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                    RETURNING id
                    """,
                    (
                        allocation.get("task", "unknown"),
                        allocation.get("budget_w"),
                        allocation.get("total_requested_w"),
                        allocation.get("total_allocated_w"),
                        allocation.get("utilization_pct"),
                        allocation.get("status", "ok"),
                        main_battery,
                        json.dumps(allocation.get("warnings", [])),
                    ),
                )
                snapshot_id = cur.fetchone()[0]

                for ch_id, data in readings.items():
                    cur.execute(
                        """
                        INSERT INTO channel_readings (
                            snapshot_id, channel_id, battery_level, power_draw_w,
                            requested_w, allocated_w, amps, nominal_voltage,
                            max_draw_w, allocation_pct, throttled, status
                        ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                        """,
                        (
                            snapshot_id,
                            ch_id,
                            data.get("battery", main_battery),
                            data.get("draw", 0),
                            data.get("requested_w"),
                            data.get("allocated_w"),
                            data.get("amps"),
                            data.get("voltage"),
                            data.get("max_draw_w"),
                            data.get("allocation_pct"),
                            data.get("throttled", False),
                            data.get("status", "normal"),
                        ),
                    )

                if prediction:
                    cur.execute(
                        """
                        INSERT INTO energy_predictions (
                            snapshot_id, task, battery_pct, predicted_draw_w,
                            predicted_runtime_min, mission_forecast_min,
                            confidence_pct, mission_energy_ok,
                            mission_battery_pct_at_end, details
                        ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                        """,
                        (
                            snapshot_id,
                            prediction.get("task", allocation.get("task", "unknown")),
                            main_battery,
                            prediction.get("predicted_draw_w"),
                            prediction.get("predicted_runtime_min"),
                            prediction.get("mission_forecast_min"),
                            prediction.get("confidence_pct"),
                            prediction.get("mission_energy_ok"),
                            prediction.get("mission_battery_pct_at_end"),
                            json.dumps(prediction),
                        ),
                    )
            return
        except Exception as e:
            logger.warning(f"Snapshot write failed (attempt {attempt + 1}/3): {e}")
            if attempt == 2:
                logger.error("Failed to log power snapshot after retries")
            else:
                time.sleep(0.2)


def archive_old_data(days: int = 30):
    """Delete readings and snapshots older than X days."""
    try:
        cutoff = datetime.now(timezone.utc) - timedelta(days=days)

        with db_cursor() as (conn, cur):
            cur.execute(
                "SELECT COUNT(*) FROM channel_readings WHERE recorded_at < %s",
                (cutoff,),
            )
            count = cur.fetchone()[0]

            if count == 0:
                logger.info(f"No data older than {days} days to archive.")
                return

            cur.execute(
                "DELETE FROM channel_readings WHERE recorded_at < %s",
                (cutoff,),
            )
            cur.execute(
                "DELETE FROM energy_predictions WHERE recorded_at < %s",
                (cutoff,),
            )
            cur.execute(
                "DELETE FROM allocation_snapshots WHERE recorded_at < %s",
                (cutoff,),
            )

        logger.info(f"✅ Archived {count} old channel readings (> {days} days)")
    except Exception as e:
        logger.error(f"Archiving failed: {e}")


def get_all_readings(limit=500):
    with db_cursor() as (conn, cur):
        cur.execute(
            """
            SELECT recorded_at, channel_id, battery_level, power_draw_w
            FROM channel_readings
            ORDER BY id DESC
            LIMIT %s
            """,
            (limit,),
        )
        rows = cur.fetchall()

    return [
        {
            "time": _format_time(r[0]),
            "channel": r[1],
            "battery": float(r[2]),
            "draw": float(r[3]),
        }
        for r in rows
    ]


def get_channel_history(channel: str, limit=300):
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT recorded_at, battery_level, power_draw_w
                FROM channel_readings
                WHERE channel_id = %s
                ORDER BY id DESC
                LIMIT %s
                """,
                (channel, limit),
            )
            rows = cur.fetchall()

        return [
            {"time": _format_time(r[0]), "battery": float(r[1]), "draw": float(r[2])}
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_channel_history failed for channel '{channel}': {e}")
        return []


def get_latest_allocation():
    """Return the most recent allocation snapshot with channel details."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT id, recorded_at, task, budget_w, total_requested_w,
                       total_allocated_w, utilization_pct, status,
                       main_battery_pct, warnings
                FROM allocation_snapshots
                ORDER BY id DESC
                LIMIT 1
                """
            )
            snap = cur.fetchone()
            if not snap:
                return None

            cur.execute(
                """
                SELECT channel_id, battery_level, power_draw_w, requested_w,
                       allocated_w, amps, allocation_pct, throttled, status
                FROM channel_readings
                WHERE snapshot_id = %s
                ORDER BY channel_id
                """,
                (snap[0],),
            )
            channels = cur.fetchall()

        return {
            "recorded_at": _format_time(snap[1]),
            "task": snap[2],
            "budget_w": float(snap[3]) if snap[3] is not None else 0,
            "total_requested_w": float(snap[4]) if snap[4] is not None else 0,
            "total_allocated_w": float(snap[5]) if snap[5] is not None else 0,
            "utilization_pct": float(snap[6]) if snap[6] is not None else 0,
            "status": snap[7],
            "main_battery_pct": float(snap[8]) if snap[8] is not None else 0,
            "warnings": snap[9] or [],
            "channels": [
                {
                    "channel": r[0],
                    "battery": float(r[1]),
                    "draw": float(r[2]),
                    "requested_w": float(r[3]) if r[3] is not None else None,
                    "allocated_w": float(r[4]) if r[4] is not None else None,
                    "amps": float(r[5]) if r[5] is not None else None,
                    "allocation_pct": float(r[6]) if r[6] is not None else None,
                    "throttled": r[7],
                    "status": r[8],
                }
                for r in channels
            ],
        }
    except Exception as e:
        logger.error(f"get_latest_allocation failed: {e}")
        return None