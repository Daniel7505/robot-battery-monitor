#!/usr/bin/env python3
"""
First energy + wheel-lock measurement: short forward drive, then park.

Uses wheel rotation sensors (encoders) from control_diag.wheels to verify
hubs actually lock after stop, integrates power for Wh, and POSTs a row to
Postgres via ``POST /api/measurements``.

Requires:
  * Dashboard + Postgres (docker compose)
  * Webots twin linked (control_diag present)

Usage:
  python scripts/measure_short_drive.py
  python scripts/measure_short_drive.py --duration 2.5 --wheel-v 5.5
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request


def _req(url: str, method: str = "GET", body: dict | None = None, timeout: float = 8.0) -> dict:
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def twin(base: str) -> dict:
    return _req(f"{base}/api/twin/state")


def cmd(base: str, body: dict) -> dict:
    return _req(f"{base}/api/twin/command", method="POST", body=body)


def legs_w(st: dict) -> float:
    for ch in st.get("channels") or []:
        if ch.get("id") == "Legs":
            return float(ch.get("draw_w") or ch.get("draw") or 0.0)
    feed = st.get("external_feed") or {}
    draws = feed.get("channel_draws") or {}
    return float(draws.get("Legs") or 0.0)


def total_w(st: dict) -> float:
    p = st.get("power") or {}
    if p.get("total_draw_w") is not None:
        return float(p["total_draw_w"])
    feed = st.get("external_feed") or {}
    draws = feed.get("channel_draws") or {}
    return sum(float(v) for v in draws.values()) if draws else 0.0


def pose_xy(st: dict) -> tuple[float, float]:
    feed = st.get("external_feed") or {}
    pose = feed.get("pose") or {}
    return float(pose.get("x_m") or 0.0), float(pose.get("y_m") or 0.0)


def diag(st: dict) -> dict:
    return st.get("control_diag") or {}


def wheels(st: dict) -> dict:
    d = diag(st)
    w = d.get("wheels") or {}
    if not w:
        sensors = (st.get("external_feed") or {}).get("sensors") or {}
        w = sensors.get("wheels") or {}
    return w


def wait_park(base: str, timeout_s: float = 12.0) -> dict:
    """Stop and wait until pose is still and hubs report locked if available."""
    for _ in range(3):
        cmd(base, {"drive_stop": True})
        time.sleep(0.35)
    deadline = time.time() + timeout_s
    last = twin(base)
    while time.time() < deadline:
        time.sleep(0.6)
        now = twin(base)
        x0, y0 = pose_xy(last)
        x1, y1 = pose_xy(now)
        dist = math.hypot(x1 - x0, y1 - y0)
        d = diag(now)
        hubs = abs(float(d.get("hub_left_rad_s") or 0)) + abs(
            float(d.get("hub_right_rad_s") or 0)
        )
        w = wheels(now)
        both = w.get("both_locked")
        if dist < 0.04 and hubs < 0.2 and (both is True or both is None):
            return now
        cmd(base, {"drive_stop": True})
        last = now
    return twin(base)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:5000")
    ap.add_argument("--duration", type=float, default=2.5, help="Drive seconds")
    ap.add_argument("--wheel-v", type=float, default=5.5, help="Wheel cmd rad/s")
    ap.add_argument("--label", default="short_drive_v1")
    ap.add_argument("--sample-hz", type=float, default=5.0)
    args = ap.parse_args()
    base = args.url.rstrip("/")

    print("=== short drive measurement ===")
    st0 = twin(base)
    if not (st0.get("bridge") or {}).get("external_active"):
        print("FAIL: Webots twin not linked")
        return 1
    if not diag(st0):
        print("WARN: control_diag missing — wheel sensors may be empty")

    print("Parking before drive...")
    st0 = wait_park(base)
    w0 = wheels(st0)
    print(
        f"  pre-lock both={w0.get('both_locked')} "
        f"L_omega={diag(st0).get('hub_left_rad_s')} "
        f"R_omega={diag(st0).get('hub_right_rad_s')} "
        f"legs={legs_w(st0):.1f}W"
    )
    # Controller ignores API drive for park_holdoff_s (~3s) after each Stop.
    print("  waiting 3.5s for park holdoff to expire...")
    time.sleep(3.5)

    st0 = twin(base)
    batt_start = float((st0.get("robot") or {}).get("main_battery_pct") or 0)
    x0, y0 = pose_xy(st0)
    dt_sample = 1.0 / max(args.sample_hz, 1.0)

    print(f"Driving {args.duration}s at wheel_v={args.wheel_v}...")
    cmd(
        base,
        {
            "drive": {
                "left": args.wheel_v,
                "right": args.wheel_v,
                "duration_s": args.duration + 0.5,
            }
        },
    )
    # Confirm teleop armed
    time.sleep(0.35)
    st_arm = twin(base)
    tel = st_arm.get("teleop") or {}
    print(
        f"  teleop active={tel.get('active')} L={tel.get('left_v')} R={tel.get('right_v')} "
        f"diag_cmd={diag(st_arm).get('cmd_source')}"
    )

    t0 = time.time()
    energy_j = 0.0
    samples: list[dict] = []
    peak_legs = 0.0
    hub_l_max = 0.0
    hub_r_max = 0.0
    while time.time() - t0 < args.duration:
        st = twin(base)
        lw = legs_w(st)
        tw = total_w(st)
        peak_legs = max(peak_legs, lw)
        d = diag(st)
        hub_l_max = max(hub_l_max, abs(float(d.get("hub_left_rad_s") or 0)))
        hub_r_max = max(hub_r_max, abs(float(d.get("hub_right_rad_s") or 0)))
        energy_j += tw * dt_sample
        samples.append(
            {
                "t": round(time.time() - t0, 3),
                "legs_w": round(lw, 2),
                "total_w": round(tw, 2),
                "hub_l": d.get("hub_left_rad_s"),
                "hub_r": d.get("hub_right_rad_s"),
                "gps": d.get("gps_speed_m_s"),
            }
        )
        print(
            f"  t={time.time()-t0:4.1f}s legs={lw:5.1f}W total={tw:5.1f}W "
            f"hubs={d.get('hub_left_rad_s')}/{d.get('hub_right_rad_s')} "
            f"gps={d.get('gps_speed_m_s')}"
        )
        time.sleep(dt_sample)

    print("Stopping...")
    st1 = wait_park(base)
    w1 = wheels(st1)
    batt_end = float((st1.get("robot") or {}).get("main_battery_pct") or 0)
    x1, y1 = pose_xy(st1)
    distance = math.hypot(x1 - x0, y1 - y0)
    energy_wh = energy_j / 3600.0
    avg_legs = (
        sum(s["legs_w"] for s in samples) / len(samples) if samples else 0.0
    )
    avg_total = (
        sum(s["total_w"] for s in samples) / len(samples) if samples else 0.0
    )
    locked = bool(w1.get("both_locked"))
    rot_l = (w1.get("left") or {}).get("rot_since_stop_deg")
    rot_r = (w1.get("right") or {}).get("rot_since_stop_deg")

    print("=== results ===")
    print(f"  distance_m      = {distance:.4f}")
    print(f"  duration_s      = {args.duration:.2f}")
    print(f"  energy_Wh       = {energy_wh:.6f}")
    print(f"  avg_legs_W      = {avg_legs:.2f}")
    print(f"  peak_legs_W     = {peak_legs:.2f}")
    print(f"  avg_total_W     = {avg_total:.2f}")
    print(f"  battery %       = {batt_start:.2f} -> {batt_end:.2f}")
    print(f"  wheels_locked   = {locked}")
    print(f"  hub max |ω|     = L {hub_l_max:.3f}  R {hub_r_max:.3f}")
    print(f"  rot since stop  = L {rot_l} deg  R {rot_r} deg")
    if locked and (rot_l is None or rot_l < 5.0) and (rot_r is None or rot_r < 5.0):
        print("  wheel sensors: LOCKED (little/no rotation after stop) — good")
    elif not locked:
        print("  wheel sensors: NOT fully locked after stop — investigate")
    else:
        print("  wheel sensors: locked flag set but rotation residual after stop")

    row = {
        "label": args.label,
        "duration_s": args.duration,
        "distance_m": round(distance, 4),
        "energy_wh": round(energy_wh, 6),
        "avg_legs_w": round(avg_legs, 2),
        "peak_legs_w": round(peak_legs, 2),
        "avg_total_w": round(avg_total, 2),
        "battery_start_pct": batt_start,
        "battery_end_pct": batt_end,
        "wheels_locked_after": locked,
        "hub_left_max_abs": round(hub_l_max, 4),
        "hub_right_max_abs": round(hub_r_max, 4),
        "rot_left_since_stop_deg": rot_l,
        "rot_right_since_stop_deg": rot_r,
        "details": {
            "wheel_v": args.wheel_v,
            "samples": samples[:40],
            "pre_wheels": w0,
            "post_wheels": w1,
        },
    }
    try:
        res = _req(f"{base}/api/measurements", method="POST", body=row)
        print(f"  DB row id       = {res.get('id')} ok={res.get('ok')}")
    except urllib.error.URLError as e:
        print(f"  DB write failed: {e}")
        print("  (dashboard may need rebuild for /api/measurements)")
        return 2

    return 0 if locked else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
