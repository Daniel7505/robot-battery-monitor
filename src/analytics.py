"""
PostgreSQL analytics — curated views, queries, and report generation.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

from src.database import db_cursor, _format_time
from src.logger import logger

_ANALYTICS_VIEWS_SQL = """
CREATE OR REPLACE VIEW v_channel_history_summary AS
SELECT
    channel_id,
    COUNT(*) AS reading_count,
    ROUND(AVG(power_draw_w)::numeric, 2) AS avg_draw_w,
    ROUND(MAX(power_draw_w)::numeric, 2) AS max_draw_w,
    ROUND(MIN(power_draw_w)::numeric, 2) AS min_draw_w,
    ROUND(AVG(battery_level)::numeric, 2) AS avg_battery_pct,
    SUM(CASE WHEN throttled THEN 1 ELSE 0 END) AS throttle_count,
    MIN(recorded_at) AS first_seen,
    MAX(recorded_at) AS last_seen
FROM channel_readings
GROUP BY channel_id;

CREATE OR REPLACE VIEW v_mission_summaries AS
SELECT
    task,
    COUNT(*) AS snapshot_count,
    ROUND(AVG(total_allocated_w)::numeric, 2) AS avg_allocated_w,
    ROUND(MAX(total_allocated_w)::numeric, 2) AS max_allocated_w,
    ROUND(MIN(total_allocated_w)::numeric, 2) AS min_allocated_w,
    ROUND(AVG(main_battery_pct)::numeric, 2) AS avg_battery_pct,
    ROUND(AVG(utilization_pct)::numeric, 2) AS avg_utilization_pct,
    SUM(CASE WHEN status = 'throttled' THEN 1 ELSE 0 END) AS throttle_events,
    SUM(CASE WHEN status IN ('fault', 'warning') THEN 1 ELSE 0 END) AS alert_events,
    MIN(recorded_at) AS first_seen,
    MAX(recorded_at) AS last_seen
FROM allocation_snapshots
GROUP BY task;

CREATE OR REPLACE VIEW v_power_trends AS
SELECT
    date_trunc('minute', recorded_at) AS bucket,
    ROUND(AVG(total_allocated_w)::numeric, 2) AS avg_draw_w,
    ROUND(MAX(total_allocated_w)::numeric, 2) AS max_draw_w,
    ROUND(MIN(total_allocated_w)::numeric, 2) AS min_draw_w,
    ROUND(AVG(main_battery_pct)::numeric, 2) AS avg_battery_pct,
    COUNT(*) AS sample_count
FROM allocation_snapshots
GROUP BY date_trunc('minute', recorded_at);

CREATE OR REPLACE VIEW v_lru_summaries AS
SELECT
    CASE channel_id
        WHEN 'Legs' THEN 'locomotion'
        WHEN 'Arms' THEN 'arms'
        WHEN 'Torso' THEN 'torso'
        WHEN 'Compute' THEN 'compute'
        ELSE 'other'
    END AS lru_id,
    CASE channel_id
        WHEN 'Legs' THEN 'Locomotion (LRUA)'
        WHEN 'Arms' THEN 'Arms'
        WHEN 'Torso' THEN 'Torso'
        WHEN 'Compute' THEN 'Compute'
        ELSE channel_id
    END AS lru_label,
    channel_id,
    COUNT(*) AS reading_count,
    ROUND(AVG(power_draw_w)::numeric, 2) AS avg_draw_w,
    ROUND(MAX(power_draw_w)::numeric, 2) AS max_draw_w,
    ROUND(MIN(power_draw_w)::numeric, 2) AS min_draw_w,
    ROUND(AVG(battery_level)::numeric, 2) AS avg_battery_pct,
    SUM(CASE WHEN throttled THEN 1 ELSE 0 END) AS throttle_count,
    SUM(CASE WHEN status IN ('warning', 'critical', 'throttled') THEN 1 ELSE 0 END) AS alert_count,
    MIN(recorded_at) AS first_seen,
    MAX(recorded_at) AS last_seen
FROM channel_readings
WHERE channel_id IN ('Legs', 'Arms', 'Torso', 'Compute')
GROUP BY channel_id;

CREATE OR REPLACE VIEW v_mission_history AS
SELECT
    id,
    recorded_at,
    task,
    budget_w,
    total_allocated_w,
    total_requested_w,
    utilization_pct,
    main_battery_pct,
    status,
    warnings
FROM allocation_snapshots
ORDER BY recorded_at DESC;

CREATE OR REPLACE VIEW v_power_trends_by_task AS
SELECT
    date_trunc('minute', recorded_at) AS bucket,
    task,
    ROUND(AVG(total_allocated_w)::numeric, 2) AS avg_draw_w,
    ROUND(AVG(main_battery_pct)::numeric, 2) AS avg_battery_pct,
    COUNT(*) AS sample_count
FROM allocation_snapshots
GROUP BY date_trunc('minute', recorded_at), task;

CREATE OR REPLACE VIEW v_battery_trends AS
SELECT
    date_trunc('minute', recorded_at) AS bucket,
    ROUND(AVG(main_battery_pct)::numeric, 2) AS avg_battery_pct,
    ROUND(MIN(main_battery_pct)::numeric, 2) AS min_battery_pct,
    ROUND(MAX(main_battery_pct)::numeric, 2) AS max_battery_pct,
    COUNT(*) AS sample_count
FROM allocation_snapshots
GROUP BY date_trunc('minute', recorded_at);

CREATE OR REPLACE VIEW v_lru_history AS
SELECT
    date_trunc('minute', recorded_at) AS bucket,
    channel_id,
    CASE channel_id
        WHEN 'Legs' THEN 'locomotion'
        WHEN 'Arms' THEN 'arms'
        WHEN 'Torso' THEN 'torso'
        WHEN 'Compute' THEN 'compute'
        ELSE 'other'
    END AS lru_id,
    CASE channel_id
        WHEN 'Legs' THEN 'Locomotion (LRUA)'
        WHEN 'Arms' THEN 'Arms'
        WHEN 'Torso' THEN 'Torso'
        WHEN 'Compute' THEN 'Compute'
        ELSE channel_id
    END AS lru_label,
    ROUND(AVG(power_draw_w)::numeric, 2) AS avg_draw_w,
    ROUND(MAX(power_draw_w)::numeric, 2) AS max_draw_w,
    SUM(CASE WHEN throttled THEN 1 ELSE 0 END) AS throttle_samples,
    SUM(CASE WHEN status IN ('warning', 'critical', 'throttled') THEN 1 ELSE 0 END) AS alert_samples,
    COUNT(*) AS sample_count
FROM channel_readings
WHERE channel_id IN ('Legs', 'Arms', 'Torso', 'Compute')
GROUP BY date_trunc('minute', recorded_at), channel_id;

CREATE OR REPLACE VIEW v_anomaly_events AS
SELECT
    recorded_at,
    CASE
        WHEN status = 'throttled' THEN 'throttle'
        WHEN status IN ('fault', 'warning') THEN 'allocation_alert'
        WHEN main_battery_pct <= 10 THEN 'critical_battery'
        WHEN main_battery_pct <= 20 THEN 'low_battery'
        WHEN utilization_pct >= 95 THEN 'high_utilization'
        ELSE 'other'
    END AS anomaly_type,
    task AS context,
    total_allocated_w AS metric_value,
    status AS severity,
    main_battery_pct AS battery_pct,
    utilization_pct,
    warnings
FROM allocation_snapshots
WHERE status IN ('throttled', 'fault', 'warning')
   OR main_battery_pct <= 20
   OR utilization_pct >= 95
UNION ALL
SELECT
    cr.recorded_at,
    CASE
        WHEN cr.throttled THEN 'channel_throttle'
        WHEN cr.status IN ('warning', 'critical') THEN 'channel_alert'
        WHEN cr.allocation_pct >= 90 THEN 'channel_overdraw'
        ELSE 'channel_other'
    END AS anomaly_type,
    cr.channel_id AS context,
    cr.power_draw_w AS metric_value,
    cr.status AS severity,
    cr.battery_level AS battery_pct,
    cr.allocation_pct AS utilization_pct,
    '[]'::jsonb AS warnings
FROM channel_readings cr
WHERE cr.throttled = TRUE
   OR cr.status IN ('warning', 'critical')
   OR cr.allocation_pct >= 90;

CREATE OR REPLACE VIEW v_mission_transitions AS
WITH ordered AS (
    SELECT
        id,
        recorded_at,
        task,
        total_allocated_w,
        main_battery_pct,
        LAG(task) OVER (ORDER BY recorded_at, id) AS prev_task,
        LAG(recorded_at) OVER (ORDER BY recorded_at, id) AS prev_at
    FROM allocation_snapshots
)
SELECT
    recorded_at,
    prev_task,
    task AS new_task,
    ROUND(EXTRACT(EPOCH FROM (recorded_at - prev_at))::numeric, 1) AS seconds_since_prev,
    total_allocated_w,
    main_battery_pct
FROM ordered
WHERE prev_task IS NOT NULL AND task IS DISTINCT FROM prev_task;

CREATE OR REPLACE VIEW v_forecast_history AS
SELECT
    date_trunc('minute', recorded_at) AS bucket,
    task,
    ROUND(AVG(predicted_draw_w)::numeric, 2) AS avg_predicted_draw_w,
    ROUND(AVG(confidence_pct)::numeric, 2) AS avg_confidence_pct,
    ROUND(AVG(mission_forecast_min)::numeric, 2) AS avg_mission_forecast_min,
    ROUND(AVG(mission_battery_pct_at_end)::numeric, 2) AS avg_mission_battery_end_pct,
    SUM(CASE WHEN mission_energy_ok = FALSE THEN 1 ELSE 0 END) AS marginal_count,
    COUNT(*) AS sample_count
FROM energy_predictions
GROUP BY date_trunc('minute', recorded_at), task;

CREATE OR REPLACE VIEW v_hourly_rollups AS
SELECT
    date_trunc('hour', recorded_at) AS bucket,
    COUNT(*) AS snapshot_count,
    ROUND(AVG(total_allocated_w)::numeric, 2) AS avg_draw_w,
    ROUND(MAX(total_allocated_w)::numeric, 2) AS peak_draw_w,
    ROUND(AVG(main_battery_pct)::numeric, 2) AS avg_battery_pct,
    ROUND(MIN(main_battery_pct)::numeric, 2) AS min_battery_pct,
    SUM(CASE WHEN status = 'throttled' THEN 1 ELSE 0 END) AS throttle_events,
    SUM(CASE WHEN status IN ('fault', 'warning') THEN 1 ELSE 0 END) AS alert_events,
    COUNT(DISTINCT task) AS distinct_tasks
FROM allocation_snapshots
GROUP BY date_trunc('hour', recorded_at);
"""


def _num(value):
    if value is None:
        return None
    return float(value)


_DROP_VIEWS_SQL = """
DROP VIEW IF EXISTS v_hourly_rollups CASCADE;
DROP VIEW IF EXISTS v_forecast_history CASCADE;
DROP VIEW IF EXISTS v_mission_transitions CASCADE;
DROP VIEW IF EXISTS v_anomaly_events CASCADE;
DROP VIEW IF EXISTS v_lru_history CASCADE;
DROP VIEW IF EXISTS v_battery_trends CASCADE;
DROP VIEW IF EXISTS v_power_trends_by_task CASCADE;
DROP VIEW IF EXISTS v_mission_history CASCADE;
DROP VIEW IF EXISTS v_lru_summaries CASCADE;
DROP VIEW IF EXISTS v_power_trends CASCADE;
DROP VIEW IF EXISTS v_mission_summaries CASCADE;
DROP VIEW IF EXISTS v_channel_history_summary CASCADE;
"""

_ANALYTICS_VIEW_NAMES = (
    "v_channel_history_summary",
    "v_mission_summaries",
    "v_power_trends",
    "v_lru_summaries",
    "v_mission_history",
    "v_power_trends_by_task",
    "v_battery_trends",
    "v_lru_history",
    "v_anomaly_events",
    "v_mission_transitions",
    "v_forecast_history",
    "v_hourly_rollups",
)


def init_analytics_views() -> None:
    with db_cursor() as (conn, cur):
        cur.execute(_DROP_VIEWS_SQL)
        cur.execute(_ANALYTICS_VIEWS_SQL)
    logger.info("✅ Analytics views initialized")


def _since(hours: float) -> datetime:
    return datetime.now(timezone.utc) - timedelta(hours=hours)


def get_power_trends(hours: float = 1.0, limit: int = 60) -> list[dict]:
    """Minute-bucketed system power draw over the recent window."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT bucket, avg_draw_w, max_draw_w, min_draw_w,
                       avg_battery_pct, sample_count
                FROM v_power_trends
                WHERE bucket >= %s
                ORDER BY bucket ASC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "bucket": _format_time(r[0]),
                "avg_draw_w": _num(r[1]),
                "max_draw_w": _num(r[2]),
                "min_draw_w": _num(r[3]),
                "avg_battery_pct": _num(r[4]),
                "sample_count": int(r[5]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_power_trends failed: {e}")
        return []


def get_channel_summary(hours: float = 24.0) -> list[dict]:
    """Per-channel draw statistics for the recent window."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT
                    channel_id,
                    COUNT(*) AS reading_count,
                    ROUND(AVG(power_draw_w)::numeric, 2) AS avg_draw_w,
                    ROUND(MAX(power_draw_w)::numeric, 2) AS max_draw_w,
                    ROUND(MIN(power_draw_w)::numeric, 2) AS min_draw_w,
                    ROUND(AVG(battery_level)::numeric, 2) AS avg_battery_pct,
                    SUM(CASE WHEN throttled THEN 1 ELSE 0 END) AS throttle_count
                FROM channel_readings
                WHERE recorded_at >= %s
                GROUP BY channel_id
                ORDER BY channel_id
                """,
                (_since(hours),),
            )
            rows = cur.fetchall()
        return [
            {
                "channel": r[0],
                "reading_count": int(r[1]),
                "avg_draw_w": _num(r[2]),
                "max_draw_w": _num(r[3]),
                "min_draw_w": _num(r[4]),
                "avg_battery_pct": _num(r[5]),
                "throttle_count": int(r[6]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_channel_summary failed: {e}")
        return []


def get_lru_summaries(hours: float = 24.0) -> list[dict]:
    """Per-LRU group statistics from curated view."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT lru_id, lru_label, channel_id, reading_count,
                       avg_draw_w, max_draw_w, min_draw_w, avg_battery_pct,
                       throttle_count, alert_count
                FROM v_lru_summaries
                WHERE last_seen >= %s
                ORDER BY lru_id
                """,
                (_since(hours),),
            )
            rows = cur.fetchall()
        return [
            {
                "lru_id": r[0],
                "lru_label": r[1],
                "channel": r[2],
                "reading_count": int(r[3]),
                "avg_draw_w": _num(r[4]),
                "max_draw_w": _num(r[5]),
                "min_draw_w": _num(r[6]),
                "avg_battery_pct": _num(r[7]),
                "throttle_count": int(r[8]),
                "alert_count": int(r[9]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_lru_summaries failed: {e}")
        return []


def get_mission_summaries(hours: float = 24.0) -> list[dict]:
    """Per-task allocation snapshot statistics."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT
                    task,
                    COUNT(*) AS snapshot_count,
                    ROUND(AVG(total_allocated_w)::numeric, 2) AS avg_allocated_w,
                    ROUND(MAX(total_allocated_w)::numeric, 2) AS max_allocated_w,
                    ROUND(AVG(main_battery_pct)::numeric, 2) AS avg_battery_pct,
                    ROUND(AVG(utilization_pct)::numeric, 2) AS avg_utilization_pct,
                    SUM(CASE WHEN status = 'throttled' THEN 1 ELSE 0 END) AS throttle_events,
                    SUM(CASE WHEN status IN ('fault', 'warning') THEN 1 ELSE 0 END) AS alert_events
                FROM allocation_snapshots
                WHERE recorded_at >= %s
                GROUP BY task
                ORDER BY snapshot_count DESC
                """,
                (_since(hours),),
            )
            rows = cur.fetchall()
        return [
            {
                "task": r[0],
                "snapshot_count": int(r[1]),
                "avg_allocated_w": _num(r[2]),
                "max_allocated_w": _num(r[3]),
                "avg_battery_pct": _num(r[4]),
                "avg_utilization_pct": _num(r[5]),
                "throttle_events": int(r[6]),
                "alert_events": int(r[7]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_mission_summaries failed: {e}")
        return []


def get_battery_trends(hours: float = 6.0, limit: int = 60) -> list[dict]:
    """Minute-bucketed battery SOC trends."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT bucket, avg_battery_pct, min_battery_pct, max_battery_pct, sample_count
                FROM v_battery_trends
                WHERE bucket >= %s
                ORDER BY bucket ASC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "bucket": _format_time(r[0]),
                "avg_battery_pct": _num(r[1]),
                "min_battery_pct": _num(r[2]),
                "max_battery_pct": _num(r[3]),
                "sample_count": int(r[4]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_battery_trends failed: {e}")
        return []


def get_lru_history(hours: float = 6.0, limit: int = 120) -> list[dict]:
    """Minute-bucketed per-LRU draw history."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT bucket, lru_id, lru_label, channel_id,
                       avg_draw_w, max_draw_w, throttle_samples, alert_samples, sample_count
                FROM v_lru_history
                WHERE bucket >= %s
                ORDER BY bucket ASC, lru_id ASC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "bucket": _format_time(r[0]),
                "lru_id": r[1],
                "lru_label": r[2],
                "channel": r[3],
                "avg_draw_w": _num(r[4]),
                "max_draw_w": _num(r[5]),
                "throttle_samples": int(r[6]),
                "alert_samples": int(r[7]),
                "sample_count": int(r[8]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_lru_history failed: {e}")
        return []


def get_anomaly_events(hours: float = 24.0, limit: int = 30) -> list[dict]:
    """Detected anomalies: throttles, alerts, low battery, over-draw."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT recorded_at, anomaly_type, context, metric_value,
                       severity, battery_pct, utilization_pct
                FROM v_anomaly_events
                WHERE recorded_at >= %s
                ORDER BY recorded_at DESC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "time": _format_time(r[0]),
                "type": r[1],
                "context": r[2],
                "metric_value": _num(r[3]),
                "severity": r[4],
                "battery_pct": _num(r[5]),
                "utilization_pct": _num(r[6]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_anomaly_events failed: {e}")
        return []


def get_mission_transitions(hours: float = 24.0, limit: int = 20) -> list[dict]:
    """Task change events with duration since previous task."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT recorded_at, prev_task, new_task, seconds_since_prev,
                       total_allocated_w, main_battery_pct
                FROM v_mission_transitions
                WHERE recorded_at >= %s
                ORDER BY recorded_at DESC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "time": _format_time(r[0]),
                "from_task": r[1],
                "to_task": r[2],
                "seconds_since_prev": _num(r[3]),
                "allocated_w": _num(r[4]),
                "battery_pct": _num(r[5]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_mission_transitions failed: {e}")
        return []


def get_forecast_history(hours: float = 6.0, limit: int = 60) -> list[dict]:
    """Minute-bucketed energy prediction history."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT bucket, task, avg_predicted_draw_w, avg_confidence_pct,
                       avg_mission_forecast_min, avg_mission_battery_end_pct,
                       marginal_count, sample_count
                FROM v_forecast_history
                WHERE bucket >= %s
                ORDER BY bucket ASC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "bucket": _format_time(r[0]),
                "task": r[1],
                "avg_predicted_draw_w": _num(r[2]),
                "avg_confidence_pct": _num(r[3]),
                "avg_mission_forecast_min": _num(r[4]),
                "avg_mission_battery_end_pct": _num(r[5]),
                "marginal_count": int(r[6]),
                "sample_count": int(r[7]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_forecast_history failed: {e}")
        return []


def get_hourly_rollups(hours: float = 24.0, limit: int = 48) -> list[dict]:
    """Hourly system rollups for longer trend windows."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT bucket, snapshot_count, avg_draw_w, peak_draw_w,
                       avg_battery_pct, min_battery_pct,
                       throttle_events, alert_events, distinct_tasks
                FROM v_hourly_rollups
                WHERE bucket >= %s
                ORDER BY bucket ASC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "bucket": _format_time(r[0]),
                "snapshot_count": int(r[1]),
                "avg_draw_w": _num(r[2]),
                "peak_draw_w": _num(r[3]),
                "avg_battery_pct": _num(r[4]),
                "min_battery_pct": _num(r[5]),
                "throttle_events": int(r[6]),
                "alert_events": int(r[7]),
                "distinct_tasks": int(r[8]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_hourly_rollups failed: {e}")
        return []


def get_anomaly_summary(hours: float = 24.0) -> dict:
    """Aggregate anomaly counts by type for the recent window."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT anomaly_type, COUNT(*) AS event_count
                FROM v_anomaly_events
                WHERE recorded_at >= %s
                GROUP BY anomaly_type
                ORDER BY event_count DESC
                """,
                (_since(hours),),
            )
            rows = cur.fetchall()
        by_type = {r[0]: int(r[1]) for r in rows}
        return {
            "hours": hours,
            "total_events": sum(by_type.values()),
            "by_type": by_type,
        }
    except Exception as e:
        logger.error(f"get_anomaly_summary failed: {e}")
        return {"hours": hours, "total_events": 0, "by_type": {}}


def get_mission_history(hours: float = 24.0, limit: int = 20) -> list[dict]:
    """Recent mission snapshots in chronological order (newest first)."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT recorded_at, task, total_allocated_w, main_battery_pct,
                       utilization_pct, status, budget_w
                FROM v_mission_history
                WHERE recorded_at >= %s
                ORDER BY recorded_at DESC
                LIMIT %s
                """,
                (_since(hours), limit),
            )
            rows = cur.fetchall()
        return [
            {
                "time": _format_time(r[0]),
                "task": r[1],
                "allocated_w": _num(r[2]),
                "battery_pct": _num(r[3]),
                "utilization_pct": _num(r[4]),
                "status": r[5],
                "budget_w": _num(r[6]),
            }
            for r in rows
        ]
    except Exception as e:
        logger.error(f"get_mission_history failed: {e}")
        return []


def get_system_summary(hours: float = 24.0) -> dict:
    """High-level telemetry summary for the recent window."""
    try:
        with db_cursor() as (conn, cur):
            cur.execute(
                """
                SELECT
                    COUNT(*) AS snapshot_count,
                    ROUND(AVG(total_allocated_w)::numeric, 2) AS avg_draw_w,
                    ROUND(MAX(total_allocated_w)::numeric, 2) AS peak_draw_w,
                    ROUND(MIN(main_battery_pct)::numeric, 2) AS min_battery_pct,
                    ROUND(MAX(main_battery_pct)::numeric, 2) AS max_battery_pct,
                    ROUND(AVG(main_battery_pct)::numeric, 2) AS avg_battery_pct,
                    SUM(CASE WHEN status = 'throttled' THEN 1 ELSE 0 END) AS throttle_events,
                    COUNT(DISTINCT task) AS distinct_tasks
                FROM allocation_snapshots
                WHERE recorded_at >= %s
                """,
                (_since(hours),),
            )
            snap = cur.fetchone()

            cur.execute(
                "SELECT COUNT(*) FROM channel_readings WHERE recorded_at >= %s",
                (_since(hours),),
            )
            channel_count = cur.fetchone()[0]
    except Exception as e:
        logger.error(f"get_system_summary failed: {e}")
        return {"hours": hours, "snapshot_count": 0, "channel_readings": 0}

    if not snap or snap[0] == 0:
        return {
            "hours": hours,
            "snapshot_count": 0,
            "channel_readings": channel_count,
            "avg_draw_w": None,
            "peak_draw_w": None,
            "min_battery_pct": None,
            "max_battery_pct": None,
            "avg_battery_pct": None,
            "throttle_events": 0,
            "distinct_tasks": 0,
        }

    return {
        "hours": hours,
        "snapshot_count": int(snap[0]),
        "channel_readings": int(channel_count),
        "avg_draw_w": _num(snap[1]),
        "peak_draw_w": _num(snap[2]),
        "min_battery_pct": _num(snap[3]),
        "max_battery_pct": _num(snap[4]),
        "avg_battery_pct": _num(snap[5]),
        "throttle_events": int(snap[6]),
        "distinct_tasks": int(snap[7]),
    }


def build_report(hours: float = 24.0) -> dict:
    """Assemble a full analytics report."""
    trend_hours = min(hours, 6.0) if hours > 2 else hours
    return {
        "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "window_hours": hours,
        "summary": get_system_summary(hours),
        "anomaly_summary": get_anomaly_summary(hours),
        "power_trends": get_power_trends(hours=trend_hours, limit=90),
        "battery_trends": get_battery_trends(hours=trend_hours, limit=90),
        "hourly_rollups": get_hourly_rollups(hours=hours, limit=48),
        "channels": get_channel_summary(hours),
        "lru_groups": get_lru_summaries(hours),
        "lru_history": get_lru_history(hours=trend_hours, limit=120),
        "missions": get_mission_summaries(hours),
        "mission_history": get_mission_history(hours, limit=15),
        "mission_transitions": get_mission_transitions(hours, limit=15),
        "anomalies": get_anomaly_events(hours, limit=20),
        "forecast_history": get_forecast_history(hours=trend_hours, limit=60),
    }


def format_report_text(report: dict) -> str:
    """Render report as plain text for CLI output."""
    s = report["summary"]
    lines = [
        "",
        "Robot Battery Monitor — Analytics Report",
        "=" * 62,
        f"Generated : {report['generated_at']}",
        f"Window    : last {report['window_hours']}h",
        "-" * 62,
        f"Snapshots       : {s.get('snapshot_count', 0)}",
        f"Channel readings: {s.get('channel_readings', 0)}",
        f"Distinct tasks  : {s.get('distinct_tasks', 0)}",
    ]

    if s.get("snapshot_count", 0) == 0:
        lines.append("No telemetry in window. Start the dashboard to collect data.")
        return "\n".join(lines)

    lines.extend([
        f"Avg system draw : {s.get('avg_draw_w', '—')} W",
        f"Peak system draw: {s.get('peak_draw_w', '—')} W",
        f"Battery range   : {s.get('min_battery_pct', '—')}% – {s.get('max_battery_pct', '—')}%",
        f"Throttle events : {s.get('throttle_events', 0)}",
    ])

    anomaly = report.get("anomaly_summary", {})
    if anomaly.get("total_events"):
        lines.extend([
            "",
            "Anomaly Summary",
            "-" * 62,
            f"Total events    : {anomaly['total_events']}",
        ])
        for atype, count in anomaly.get("by_type", {}).items():
            lines.append(f"  {atype:22}  {count}")

    lines.extend(["", "LRU Group Summary", "-" * 62])

    for lru in report.get("lru_groups", []):
        lines.append(
            f"  {lru['lru_label']:22}  avg {lru['avg_draw_w']:5}W  "
            f"peak {lru['max_draw_w']:5}W  alerts {lru['alert_count']:3}  "
            f"throttles {lru['throttle_count']}"
        )

    lines.extend(["", "Per-Channel Summary", "-" * 62])
    for ch in report.get("channels", []):
        lines.append(
            f"  {ch['channel']:10}  avg {ch['avg_draw_w']:5}W  "
            f"peak {ch['max_draw_w']:5}W  readings {ch['reading_count']:4}  "
            f"throttles {ch['throttle_count']}"
        )

    lines.extend(["", "Mission Summaries", "-" * 62])
    for m in report.get("missions", []):
        lines.append(
            f"  {m['task']:12}  samples {m['snapshot_count']:4}  "
            f"avg {m['avg_allocated_w']:5}W  util {m['avg_utilization_pct']:5}%  "
            f"alerts {m['alert_events']}"
        )

    history = report.get("mission_history", [])
    if history:
        lines.extend(["", "Recent Mission History", "-" * 62])
        for h in history[:10]:
            lines.append(
                f"  {h['time']}  {h['task']:12}  {h['allocated_w']:5}W  "
                f"bat {h['battery_pct']}%  {h['status']}"
            )

    transitions = report.get("mission_transitions", [])
    if transitions:
        lines.extend(["", "Mission Transitions", "-" * 62])
        for t in transitions[:8]:
            lines.append(
                f"  {t['time']}  {t['from_task']} → {t['to_task']}  "
                f"({t['seconds_since_prev']}s)  {t['allocated_w']}W"
            )

    anomalies = report.get("anomalies", [])
    if anomalies:
        lines.extend(["", "Recent Anomalies", "-" * 62])
        for a in anomalies[:8]:
            lines.append(
                f"  {a['time']}  {a['type']:18}  {a['context']}  "
                f"{a['metric_value']}  {a['severity']}"
            )

    forecasts = report.get("forecast_history", [])
    if forecasts:
        lines.extend(["", f"Forecast History ({len(forecasts)} buckets)", "-" * 62])
        for f in forecasts[-6:]:
            lines.append(
                f"  {f['bucket']}  {f['task']:10}  pred {f['avg_predicted_draw_w']}W  "
                f"conf {f['avg_confidence_pct']}%  marginal {f['marginal_count']}"
            )

    trends = report.get("power_trends", [])
    if trends:
        lines.extend(["", f"Power Trend ({len(trends)} minute buckets)", "-" * 62])
        for t in trends[-8:]:
            lines.append(
                f"  {t['bucket']}  avg {t['avg_draw_w']}W  "
                f"battery {t['avg_battery_pct']}%"
            )

    return "\n".join(lines)