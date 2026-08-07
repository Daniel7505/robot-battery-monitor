# robot_battery_monitor.py
"""
CLI tool for offline analysis of battery / power telemetry.

Reads from the same PostgreSQL store the dashboard writes to. Does not start
hardware or the web UI — use this after a run, or alongside a live system, to:
  - Print a quick latest-status summary
  - Dump per-channel history
  - Generate a multi-section analytics report
  - Manually prune (archive) old readings past a retention window

Typical usage:
  python robot_battery_monitor.py --summary
  python robot_battery_monitor.py --history Legs --limit 50
  python robot_battery_monitor.py --report --hours 12
  python robot_battery_monitor.py --archive --archive-days 30
"""

import argparse
import sys
from datetime import datetime
from src.logger import logger
from src.database import get_all_readings, get_channel_history, archive_old_data
from src.analytics import build_report, format_report_text


def print_summary():
    """Print latest main battery and per-channel draw from recent DB rows."""
    entries = get_all_readings(limit=100)
    if not entries:
        logger.info("No readings yet. Start the dashboard first!")
        return

    # Rows are newest-first; first row is the freshest main-battery sample.
    main_battery = entries[0]["battery"]
    total_readings = len(entries)

    print("\n🤖 Robot Battery Monitor — CLI Summary")
    print("=" * 60)
    print(f"📊 Latest Main Battery : {main_battery}%")
    print(f"📈 Total Readings      : {total_readings}")
    print(f"🕒 Last Update         : {entries[0]['time']}")
    print("-" * 60)

    # First occurrence of each channel in newest-first order = latest sample.
    latest = {}
    for e in entries:
        if e["channel"] not in latest:
            latest[e["channel"]] = e

    for ch_id, data in latest.items():
        status = "🟢" if data["battery"] > 30 else "🔴"
        print(f"{status} {ch_id:10} → {data['battery']:3}% | Draw: {data['draw']:3}W")


def show_channel_history(channel: str, limit: int = 20):
    """Print a chronological (newest-first) slice of one channel's history."""
    history = get_channel_history(channel, limit=limit)
    if not history:
        print(f"No history for channel '{channel}'")
        return

    print(f"\n📜 History for {channel} (last {len(history)} readings)")
    print("-" * 70)
    for entry in history[:limit]:
        print(f"  {entry['time']}  |  {entry['battery']:3}%  |  {entry['draw']:3}W")


def main():
    parser = argparse.ArgumentParser(description="Robot Battery Monitor CLI")
    parser.add_argument('--summary', action='store_true', help="Show latest status")
    parser.add_argument('--history', type=str, help="Show history for a channel (e.g. Legs)")
    parser.add_argument('--limit', type=int, default=30, help="Number of records for history")
    parser.add_argument('--archive', action='store_true', help="Archive old data manually")
    parser.add_argument('--archive-days', type=int, default=30, help="Days to keep (default 30)")
    parser.add_argument('--report', action='store_true', help="Generate analytics report")
    parser.add_argument('--hours', type=float, default=24.0, help="Hours of data for report (default 24)")

    args = parser.parse_args()
    logger.info("CLI started")

    # Mutating ops short-circuit so they never mix with read-only display paths.
    if args.archive:
        archive_old_data(days=args.archive_days)
        return

    if args.report:
        report = build_report(hours=args.hours)
        print(format_report_text(report))
        return

    if args.history:
        show_channel_history(args.history, args.limit)
    else:
        # Default action when no flags are given.
        print_summary()

    logger.info("CLI finished")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"CLI error: {e}", exc_info=True)
        sys.exit(1)
