from datetime import datetime, timedelta, timezone

from src.analytics import (
    build_report,
    format_report_text,
    get_channel_summary,
    get_mission_summaries,
    get_power_trends,
    get_system_summary,
    init_analytics_views,
)
from src.database import db_cursor, init_db, log_power_snapshot, truncate_tables


def _seed_snapshots():
    tasks = [
        ("idle", 28, 92, "ok"),
        ("moving", 58, 91, "ok"),
        ("high_load", 72, 90, "throttled"),
    ]
    for task, draw, battery, status in tasks:
        allocation = {
            "task": task,
            "budget_w": 72,
            "total_requested_w": draw,
            "total_allocated_w": draw,
            "utilization_pct": round(draw / 72 * 100, 1),
            "status": status,
            "warnings": [],
        }
        readings = {
            "Legs": {
                "battery": battery,
                "draw": draw * 0.4,
                "requested_w": draw * 0.4,
                "allocated_w": draw * 0.4,
                "amps": 0.5,
                "voltage": 48,
                "max_draw_w": 35,
                "allocation_pct": 50,
                "throttled": status == "throttled",
                "status": "normal",
            }
        }
        log_power_snapshot(allocation, readings, battery)


def test_analytics_views_initialize(clean_database):
    init_db()
    init_analytics_views()
    with db_cursor() as (conn, cur):
        cur.execute(
            "SELECT COUNT(*) FROM information_schema.views WHERE table_name = 'v_power_trends'"
        )
        assert cur.fetchone()[0] == 1


def test_system_summary_with_data(clean_database):
    init_db()
    _seed_snapshots()
    summary = get_system_summary(hours=24)
    assert summary["snapshot_count"] == 3
    assert summary["avg_draw_w"] > 0
    assert summary["throttle_events"] == 1


def test_channel_and_mission_summaries(clean_database):
    init_db()
    _seed_snapshots()
    channels = get_channel_summary(hours=24)
    assert len(channels) == 1
    assert channels[0]["channel"] == "Legs"
    assert channels[0]["reading_count"] == 3

    missions = get_mission_summaries(hours=24)
    assert len(missions) == 3
    tasks = {m["task"] for m in missions}
    assert "idle" in tasks
    assert "moving" in tasks


def test_power_trends_returns_buckets(clean_database):
    init_db()
    _seed_snapshots()
    trends = get_power_trends(hours=1, limit=10)
    assert len(trends) >= 1
    assert "avg_draw_w" in trends[0]


def test_build_report_and_format(clean_database):
    init_db()
    _seed_snapshots()
    report = build_report(hours=24)
    text = format_report_text(report)
    assert "Analytics Report" in text
    assert "idle" in text
    assert report["summary"]["snapshot_count"] == 3


def test_report_empty_window(clean_database):
    init_db()
    report = build_report(hours=1)
    text = format_report_text(report)
    assert "No telemetry" in text