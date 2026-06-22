#!/usr/bin/env python3
"""
ButlerBot digital twin feed — example external simulator loop.

Posts walking-robot telemetry to the PMS DigitalTwinBridge each step.
Run while the dashboard is up: python examples/butlerbot_twin_feed.py

Usage:
  python examples/butlerbot_twin_feed.py
  python examples/butlerbot_twin_feed.py --url http://127.0.0.1:5000 --cycles 3
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import requests
except ImportError:
    print("Install requests: pip install requests")
    sys.exit(1)

from src.twin.butlerbot import BUTLERBOT_WALKING_FLOW, butlerbot_telemetry_step


def main():
    parser = argparse.ArgumentParser(description="ButlerBot twin telemetry feed example")
    parser.add_argument("--url", default="http://127.0.0.1:5000", help="Dashboard base URL")
    parser.add_argument("--cycles", type=int, default=2, help="Full walking cycles to send")
    parser.add_argument("--interval", type=float, default=3.0, help="Seconds between steps")
    args = parser.parse_args()

    base = args.url.rstrip("/")
    print(f"ButlerBot twin feed → {base}/api/twin/telemetry")
    print(f"Cycle: {len(BUTLERBOT_WALKING_FLOW)} phases × {args.cycles} cycles\n")

    battery = 92.0
    step = 0
    for cycle in range(args.cycles):
        for phase in BUTLERBOT_WALKING_FLOW:
            payload = butlerbot_telemetry_step(
                step,
                source="custom",
                adapter="butlerbot",
                battery_pct=battery,
            )
            step += 1
            battery = max(5.0, battery - 0.08)

            try:
                resp = requests.post(
                    f"{base}/api/twin/telemetry",
                    json=payload,
                    timeout=5,
                )
                data = resp.json()
                ok = data.get("ok", False)
                mark = "✓" if ok else "✗"
                print(
                    f"{mark} [{phase['phase']:14}] task={phase['task']:10} "
                    f"draw≈{sum(phase['channel_draws'].values()):.0f}W  "
                    f"bat={battery:.1f}%  → {data.get('source', '?')}"
                )
                if not ok:
                    print(f"   errors: {data.get('errors') or data.get('error')}")
            except requests.RequestException as exc:
                print(f"✗ Request failed: {exc}")
                print("   Is the dashboard running? docker compose up -d")
                sys.exit(1)

            time.sleep(args.interval)

    print("\nDone. Open dashboard → Digital Twin panel to see external feed.")


if __name__ == "__main__":
    main()