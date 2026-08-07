#!/usr/bin/env python3
"""
Agent/command-path baseline: drive forward ~2 feet, then stop.

Integration smoke test for the twin command path (same API as the dashboard
Drive buttons) — not Webots keyboard teleop. Requires:

* Dashboard up at ``--url`` (default http://127.0.0.1:5000)
* Webots twin publishing external feed (``bridge.external_active``)

Distance estimate: ~0.4 m/s * 1.6 s ≈ 0.64 m ≈ 2.1 ft.

Pass criteria (approx.): Legs draw rises ≥4 W over preflight and mission
shows ``moving`` during the drive window; then stop settles to idle.

Usage:
  python scripts/agent_short_drive.py
  python scripts/agent_short_drive.py --url http://127.0.0.1:5000
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def _req(url: str, method: str = "GET", body: dict | None = None, timeout: float = 5.0) -> dict:
    """Minimal JSON HTTP helper (stdlib only — no requests dependency)."""
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def snap(base: str) -> dict:
    """Flatten twin state into a small row for logging / pass-fail checks."""
    st = _req(f"{base}/api/twin/state")
    feed = st.get("external_feed") or {}
    loc = feed.get("locomotion") or {}
    draws = feed.get("channel_draws") or {}
    robot = feed.get("robot") or st.get("robot") or {}
    mission = st.get("mission") or {}
    agent = st.get("agent") or {}
    safety = st.get("safety") or {}
    power = st.get("power") or {}
    return {
        "mission": mission.get("task"),
        "mission_label": mission.get("task_label"),
        "phase": loc.get("phase"),
        "gait": loc.get("gait"),
        "speed": loc.get("speed_m_s"),
        "Legs": draws.get("Legs"),
        "total": sum(float(v) for v in draws.values()) if draws else None,
        "batt": (st.get("robot") or {}).get("main_battery_pct"),
        "hw": robot.get("hardware_profile"),
        "cap": robot.get("battery_capacity_wh"),
        "thr": safety.get("throttle_required"),
        "power": power.get("status"),
        "agent": agent.get("posture"),
        "intervening": agent.get("intervening"),
        "ext": (st.get("bridge") or {}).get("external_active"),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:5000")
    ap.add_argument("--duration", type=float, default=1.6, help="Drive duration seconds (~2 ft)")
    ap.add_argument("--wheel-v", type=float, default=5.0, help="Wheel cmd rad/s")
    args = ap.parse_args()
    base = args.url.rstrip("/")

    # --- Preflight: dashboard + live Webots feed ---
    print("=== agent_short_drive: preflight ===")
    try:
        s0 = snap(base)
    except urllib.error.URLError as exc:
        print(f"FAIL: dashboard not reachable at {base}: {exc}")
        return 1
    if not s0.get("ext"):
        print("FAIL: no external twin feed — start Webots first")
        return 1
    print(
        f"pre: mission={s0['mission']} Legs={s0['Legs']} speed={s0['speed']} "
        f"hw={s0['hw']} cap={s0['cap']} thr={s0['thr']}"
    )

    # Ensure stopped before the measured leg
    _req(f"{base}/api/twin/command", "POST", {"drive_stop": True})
    time.sleep(1.5)

    print("=== command: drive forward ===")
    cmd = {
        "drive": {
            "left": args.wheel_v,
            "right": args.wheel_v,
            "duration_s": args.duration,
        },
        "source": "agent_short_drive",
    }
    result = _req(f"{base}/api/twin/command", "POST", cmd)
    print("command result:", json.dumps(result, default=str))

    # Sample while the timed drive is active (bridge zeros wheels after duration_s)
    drive_rows = []
    t_end = time.time() + args.duration + 0.8
    while time.time() < t_end:
        time.sleep(0.45)
        s = snap(base)
        drive_rows.append(s)
        print(
            f"  drive Legs={s['Legs']} speed={s['speed']} mission={s['mission']}/"
            f"{s['phase']} thr={s['thr']} agent={s['agent']}"
        )

    print("=== command: stop ===")
    _req(f"{base}/api/twin/command", "POST", {"drive_stop": True, "source": "agent_short_drive"})
    stop_rows = []
    for _ in range(8):
        time.sleep(1.0)
        s = snap(base)
        stop_rows.append(s)
        print(
            f"  stop Legs={s['Legs']} speed={s['speed']} mission={s['mission']}/"
            f"{s['phase']} thr={s['thr']}"
        )
        if (
            (s.get("speed") or 0) < 0.06
            and (s.get("Legs") or 99) < 9
            and s.get("mission") == "idle"
        ):
            print("  SETTLED")
            break

    pre_legs = s0.get("Legs") or 0
    peak = max((r.get("Legs") or 0) for r in drive_rows) if drive_rows else 0
    peak_speed = max((r.get("speed") or 0) for r in drive_rows) if drive_rows else 0
    saw_moving = any(r.get("mission") == "moving" for r in drive_rows)
    final = stop_rows[-1] if stop_rows else {}
    print("=== SUMMARY ===")
    print(f"pre Legs={pre_legs} peak Legs={peak} peak speed={peak_speed}")
    print(f"saw_moving_mission={saw_moving} final mission={final.get('mission')} Legs={final.get('Legs')}")
    ok = peak >= (pre_legs or 0) + 4 and saw_moving
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
