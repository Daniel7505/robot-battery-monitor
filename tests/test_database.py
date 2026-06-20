import pytest
from datetime import datetime, timedelta, timezone

from src.database import (
    archive_old_data,
    get_all_readings,
    log_channel_reading,
    log_power_snapshot,
    init_db,
    db_cursor,
)


def test_archive_old_data_does_nothing_when_no_old_data(clean_database):
    """Should not crash and should report nothing to archive when data is recent."""
    log_channel_reading("Legs", 85, 12)
    archive_old_data(days=30)
    entries = get_all_readings(limit=10)
    assert len(entries) >= 1


def test_archive_old_data_removes_old_records(clean_database):
    """Should delete records older than the specified days."""
    log_channel_reading("Arms", 70, 8)

    old_time = datetime.now(timezone.utc) - timedelta(days=40)
    with db_cursor() as (conn, cur):
        cur.execute(
            """
            INSERT INTO channel_readings
                (channel_id, recorded_at, battery_level, power_draw_w, status)
            VALUES (%s, %s, %s, %s, 'normal')
            """,
            ("Torso", old_time, 50, 5),
        )

    archive_old_data(days=30)

    entries = get_all_readings(limit=20)
    assert len(entries) == 1
    assert entries[0]["channel"] == "Arms"


def test_log_power_snapshot_stores_allocation(clean_database):
    allocation = {
        "task": "moving",
        "budget_w": 72,
        "total_requested_w": 65,
        "total_allocated_w": 65,
        "utilization_pct": 90.3,
        "status": "ok",
        "warnings": [],
    }
    readings = {
        "Legs": {
            "battery": 92.0,
            "draw": 26.5,
            "requested_w": 26.5,
            "allocated_w": 26.5,
            "amps": 0.55,
            "voltage": 48,
            "max_draw_w": 35,
            "allocation_pct": 75.7,
            "throttled": False,
            "status": "normal",
        }
    }
    log_power_snapshot(allocation, readings, 92.0)

    entries = get_all_readings(limit=5)
    assert len(entries) == 1
    assert entries[0]["channel"] == "Legs"
    assert entries[0]["draw"] == 26.5