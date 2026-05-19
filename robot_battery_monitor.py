# robot_battery_monitor.py
"""
CLI Tool - Analyze battery history from the database.
Production-ready version using our new logger + DB layer.
"""

import argparse
import sys
from datetime import datetime
from src.logger import logger
from src.database import get_all_readings, get_channel_history, archive_old_data


def print_summary():
    """Show overall system status"""
    entries = get_all_readings(limit=100)
    if not entries:
        logger.info("No readings yet. Start the dashboard first!")
        return

    main_battery = entries[0]["battery"]
    total_readings = len(entries)

    print("\n🤖 Robot Battery Monitor — CLI Summary")
    print("=" * 60)
    print(f"📊 Latest Main Battery : {main_battery}%")
    print(f"📈 Total Readings      : {total_readings}")
    print(f"🕒 Last Update         : {entries[0]['time']}")
    print("-" * 60)

    # Per-channel latest
    latest = {}
    for e in entries:
        if e["channel"] not in latest:
            latest[e["channel"]] = e

    for ch_id, data in latest.items():
        status = "🟢" if data["battery"] > 30 else "🔴"
        print(f"{status} {ch_id:10} → {data['battery']:3}% | Draw: {data['draw']:3}W")


def show_channel_history(channel: str, limit: int = 20):
    """Show detailed history for one channel"""
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
    parser.add_argument('--history', type=str, help="Show history for specific channel (e.g. Legs)")
    parser.add_argument('--limit', type=int, default=30, help="Number of records to show")
    parser.add_argument('--archive', action='store_true', help="Run manual archive of old data")

    args = parser.parse_args()

    logger.info("CLI Tool started")

    if args.archive:
        archive_old_data(days=30)
        return

    if args.history:
        show_channel_history(args.history, args.limit)
    else:
        print_summary()

    logger.info("CLI Tool finished")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"CLI error: {e}", exc_info=True)
        sys.exit(1)