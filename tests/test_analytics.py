from datetime import datetime, timedelta, timezone

from src.analytics import (
    _ANALYTICS_VIEW_NAMES,
    build_report,
    format_report_text,
    get_anomaly_events,
    get_anomaly_summary,
    get_channel_summary,
    get_forecast_history,
    get_lru_history,
    get_lru_summaries,
    get_mission_history,
    get_mission_summaries,
    get_mission_transitions,
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
            """
            SELECT COUNT(*) FROM information_schema.views
            WHERE table_name = ANY(%s)
            """,
            (list(_ANALYTICS_VIEW_NAMES),),
        )
        assert cur.fetchone()[0] == len(_ANALYTICS_VIEW_NAMES)


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


def test_lru_and_mission_history(clean_database):
    init_db()
    _seed_snapshots()
    lru = get_lru_summaries(hours=24)
    assert len(lru) == 1
    assert lru[0]["lru_id"] == "locomotion"

    history = get_mission_history(hours=24, limit=5)
    assert len(history) == 3
    assert history[0]["task"] in ("idle", "moving", "high_load")


def test_build_report_and_format(clean_database):
    init_db()
    _seed_snapshots()
    report = build_report(hours=24)
    text = format_report_text(report)
    assert "Analytics Report" in text
    assert "LRU Group Summary" in text
    assert "idle" in text
    assert report["summary"]["snapshot_count"] == 3
    assert len(report["lru_groups"]) >= 1
    assert len(report["mission_history"]) == 3


def test_report_empty_window(clean_database):
    init_db()
    report = build_report(hours=1)
    text = format_report_text(report)
    assert "No telemetry" in text


def test_anomaly_and_transition_views(clean_database):
    init_db()
    _seed_snapshots()
    anomalies = get_anomaly_events(hours=24, limit=10)
    assert isinstance(anomalies, list)

    summary = get_anomaly_summary(hours=24)
    assert "by_type" in summary

    transitions = get_mission_transitions(hours=24, limit=10)
    assert len(transitions) >= 2
    assert transitions[0]["from_task"] in ("idle", "moving", "balanced", "high_load")

    lru_hist = get_lru_history(hours=24, limit=20)
    assert len(lru_hist) >= 1
    assert lru_hist[0]["lru_id"] == "locomotion"


def test_build_report_includes_extended_sections(clean_database):
    init_db()
    _seed_snapshots()
    report = build_report(hours=24)
    assert "anomaly_summary" in report
    assert "mission_transitions" in report
    assert "anomalies" in report
    assert "forecast_history" in report
    assert "lru_history" in report
    text = format_report_text(report)
    assert "Mission Transitions" in text or report["mission_transitions"]