#!/usr/bin/env python3
"""
Longer agent/command-path mission (beyond ~2 ft short-drive baseline).

Sequence (default):
  1. Forward leg (~3.2 s ≈ 4+ ft at cruise)
  2. Brief pause / settle
  3. In-place turn (optional; uses P1 spin-halt stop path)
  4. Return leg (reverse)
  5. Final stop and settle

Uses the twin command API (same path as dashboard Drive buttons).

Usage:
  python scripts/agent_extended_drive.py
  python scripts/agent_extended_drive.py --url http://127.0.0.1:5000 --no-turn
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def _req(url: str, method: str = "GET", body: dict | None = None, timeout: float = 5.0) -> dict:
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def snap(base: str) -> dict:
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
        "thr": safety.get("throttle_required"),
        "power": power.get("status"),
        "agent": agent.get("posture"),
        "ext": (st.get("bridge") or {}).get("external_active"),
    }


def sample_window(base: str, duration_s: float, label: str) -> list[dict]:
    rows: list[dict] = []
    t_end = time.time() + duration_s
    while time.time() < t_end:
        time.sleep(0.4)
        s = snap(base)
        rows.append(s)
        print(
            f"  [{label}] Legs={s['Legs']} speed={s['speed']} mission={s['mission']}/"
            f"{s['phase']} gait={s['gait']} thr={s['thr']}"
        )
    return rows


def drive_cmd(base: str, left: float, right: float, duration_s: float, source: str) -> dict:
    return _req(
        f"{base}/api/twin/command",
        "POST",
        {
            "drive": {"left": left, "right": right, "duration_s": duration_s},
            "source": source,
        },
    )


def stop_and_settle(base: str, source: str, wait_s: float = 8.0) -> list[dict]:
    _req(f"{base}/api/twin/command", "POST", {"drive_stop": True, "source": source})
    rows: list[dict] = []
    t_end = time.time() + wait_s
    while time.time() < t_end:
        time.sleep(0.8)
        s = snap(base)
        rows.append(s)
        print(
            f"  [stop] Legs={s['Legs']} speed={s['speed']} mission={s['mission']}/"
            f"{s['phase']}"
        )
        if (
            (s.get("speed") or 0) < 0.06
            and (s.get("Legs") or 99) < 10
            and s.get("mission") in ("idle", None)
        ):
            print("  SETTLED")
            break
    return rows


def _peak_legs(rows: list[dict]) -> float:
    vals = [float(r["Legs"]) for r in rows if r.get("Legs") is not None]
    return max(vals) if vals else 0.0


def _any_moving(rows: list[dict]) -> bool:
    return any(r.get("mission") == "moving" for r in rows)


def _any_turn_phase(rows: list[dict]) -> bool:
    return any(
        (r.get("phase") or "").lower() in ("teleop_turn",)
        or (r.get("gait") or "").lower() == "turn"
        for r in rows
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:5000")
    ap.add_argument("--forward-s", type=float, default=3.2, help="Forward leg duration")
    ap.add_argument("--return-s", type=float, default=3.0, help="Return (reverse) duration")
    ap.add_argument("--turn-s", type=float, default=1.2, help="In-place turn duration")
    ap.add_argument("--wheel-v", type=float, default=5.0, help="Straight wheel cmd rad/s")
    ap.add_argument("--turn-v", type=float, default=4.0, help="Turn wheel cmd rad/s")
    ap.add_argument("--no-turn", action="store_true", help="Skip in-place turn segment")
    ap.add_argument("--pause-s", type=float, default=1.5, help="Pause between legs")
    args = ap.parse_args()
    base = args.url.rstrip("/")
    source = "agent_extended_drive"

    print("=== agent_extended_drive: preflight ===")
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
        f"hw={s0['hw']} thr={s0['thr']}"
    )

    print("=== ensure stopped ===")
    stop_and_settle(base, source, wait_s=4.0)

    # --- Leg 1: forward ---
    print(f"=== leg 1: forward {args.forward_s:.1f}s @ {args.wheel_v} ===")
    drive_cmd(base, args.wheel_v, args.wheel_v, args.forward_s, source)
    forward_rows = sample_window(base, args.forward_s + 0.6, "fwd")
    print("=== pause after forward ===")
    stop_rows_1 = stop_and_settle(base, source, wait_s=max(args.pause_s + 2.0, 4.0))

    turn_rows: list[dict] = []
    if not args.no_turn:
        print(f"=== turn left in-place {args.turn_s:.1f}s ===")
        drive_cmd(base, -args.turn_v, args.turn_v, args.turn_s, source)
        turn_rows = sample_window(base, args.turn_s + 0.5, "turn")
        print("=== stop after turn ===")
        stop_rows_turn = stop_and_settle(base, source, wait_s=5.0)
    else:
        stop_rows_turn = []
        time.sleep(args.pause_s)

    # --- Leg 2: return (reverse) ---
    print(f"=== leg 2: reverse {args.return_s:.1f}s ===")
    drive_cmd(base, -args.wheel_v, -args.wheel_v, args.return_s, source)
    return_rows = sample_window(base, args.return_s + 0.6, "rev")
    print("=== final stop ===")
    final_stop = stop_and_settle(base, source, wait_s=8.0)

    pre_legs = float(s0.get("Legs") or 0)
    peak_fwd = _peak_legs(forward_rows)
    peak_rev = _peak_legs(return_rows)
    peak_turn = _peak_legs(turn_rows) if turn_rows else 0.0
    peak_all = max(peak_fwd, peak_rev, peak_turn)
    saw_moving = _any_moving(forward_rows) or _any_moving(return_rows)
    saw_turn = _any_turn_phase(turn_rows) if turn_rows else False
    final = final_stop[-1] if final_stop else {}
    settled = (
        (final.get("speed") or 0) < 0.08
        and (final.get("Legs") or 99) < 12
    )

    print("=== SUMMARY (vs short-drive ~2 ft baseline) ===")
    print(f"pre Legs={pre_legs}")
    print(f"peak forward Legs={peak_fwd}  reverse Legs={peak_rev}  turn Legs={peak_turn}")
    print(f"peak overall Legs={peak_all}")
    print(f"saw_moving={saw_moving} saw_turn_phase={saw_turn} final_settled={settled}")
    print(f"final mission={final.get('mission')} Legs={final.get('Legs')} speed={final.get('speed')}")
    print(
        "Expect vs short drive: longer energy exposure (two legs), similar peak Legs "
        "band, mission moving on both legs, clean final stop."
    )

    ok = (
        peak_fwd >= pre_legs + 3
        and peak_rev >= pre_legs + 3
        and saw_moving
        and settled
    )
    if turn_rows and not args.no_turn:
        # Turn should show non-idle power OR explicit turn phase when feed is current
        turn_ok = peak_turn >= pre_legs + 1.5 or saw_turn
        if not turn_ok:
            print("WARN: turn segment weak (Legs/phase) — still pass if straight legs OK")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
