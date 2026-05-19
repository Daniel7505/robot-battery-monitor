import pytest
from src.database import (
    archive_old_data,
    get_all_readings,
    log_channel_reading,
    init_db
)
from datetime import datetime, timedelta


def test_archive_old_data_does_nothing_when_no_old_data(clean_database):
    """Should not crash and should report nothing to archive when data is recent."""
    init_db()
    log_channel_reading("Legs", 85, 12)

    # This should run without error
    archive_old_data(days=30)

    # Data should still be there
    entries = get_all_readings(limit=10)
    assert len(entries) >= 1


def test_archive_old_data_removes_old_records(clean_database, monkeypatch):
    """Should archive and delete records older than the specified days."""
    init_db()

    # Insert a recent reading
    log_channel_reading("Arms", 70, 8)

    # Manually insert an old reading by manipulating the timestamp
    from src.database import get_db_connection
    conn = get_db_connection()
    old_time = (datetime.now() - timedelta(days=40)).strftime("%Y-%m-%d %H:%M:%S")
    conn.execute("""
        INSERT INTO battery_readings (timestamp, channel, battery_level, power_draw)
        VALUES (?, ?, ?, ?)
    """, (old_time, "Torso", 50, 5))
    conn.commit()
    conn.close()

    # Run archive for 30 days
    archive_old_data(days=30)

    # Only the recent reading should remain
    entries = get_all_readings(limit=20)
    assert len(entries) == 1
    assert entries[0]["channel"] == "Arms"