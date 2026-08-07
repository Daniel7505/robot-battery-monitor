#!/usr/bin/env python3
"""
Track run: start line → finish line with known GPS / pose coordinates.

World markers (butlerbot.wbt, ENU):
  START  (green): x = 0.0 m, y = 0.0 m
  FINISH (red):   x = 5.0 m, y = 0.0 m
  Length along lane: 5.0 m
  Robot GPS (Webots GPS device) reports world translation — same frame.

Drive continuously (re-assert cmd) until robot GPS x >= finish - tol,
or timeout. Compare:
  * start pose vs start line GPS
  * end pose vs finish line GPS
  * path length vs 5.0 m
  * time vs min 5 s free roll
  * wheel odometry ∫ωr vs ground distance

Usage:
  python scripts/measure_track_run.py
  python scripts/measure_track_run.py --wheel-v 5.5 --timeout 40
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

# Defaults match webots/worlds/track_calibration.json and butlerbot.wbt paint
START_X = 0.0
START_Y = 0.0
FINISH_X = 5.0
FINISH_Y = 0.0
TRACK_LENGTH_M = 5.0
WHEEL_RADIUS_M = 0.08


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


def diag(st: dict) -> dict:
    return st.get("control_diag") or {}


def pose_xy(st: dict) -> tuple[float, float]:
    feed = st.get("external_feed") or {}
    p = feed.get("pose") or {}
    return float(p.get("x_m") or 0.0), float(p.get("y_m") or 0.0)


def wait_park(base: str, timeout_s: float = 12.0) -> None:
    for _ in range(3):
        cmd(base, {"drive_stop": True})
        time.sleep(0.3)
    deadline = time.time() + timeout_s
    last = twin(base)
    while time.time() < deadline:
        time.sleep(0.5)
        now = twin(base)
        x0, y0 = pose_xy(last)
        x1, y1 = pose_xy(now)
        d = math.hypot(x1 - x0, y1 - y0)
        hubs = abs(float(diag(now).get("hub_left_rad_s") or 0)) + abs(
            float(diag(now).get("hub_right_rad_s") or 0)
        )
        if d < 0.04 and hubs < 0.25:
            return
        cmd(base, {"drive_stop": True})
        last = now


def load_calibration() -> dict:
    path = (
        Path(__file__).resolve().parent.parent
        / "webots"
        / "worlds"
        / "track_calibration.json"
    )
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return {}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:5000")
    ap.add_argument("--wheel-v", type=float, default=5.5, help="Commanded hub rad/s")
    ap.add_argument("--timeout", type=float, default=45.0, help="Max seconds to reach finish")
    ap.add_argument("--finish-tol", type=float, default=0.25, help="x within finish (m)")
    ap.add_argument("--label", default="track_run_5m_v1")
    ap.add_argument("--sample-hz", type=float, default=8.0)
    args = ap.parse_args()
    base = args.url.rstrip("/")
    cal = load_calibration()
    start_x = float((cal.get("start") or {}).get("x_m", START_X))
    finish_x = float((cal.get("finish") or {}).get("x_m", FINISH_X))
    track_len = float((cal.get("lane") or {}).get("length_m", TRACK_LENGTH_M))
    r = float(cal.get("wheel_radius_m", WHEEL_RADIUS_M))
    v_cmd = args.wheel_v * r
    dt = 1.0 / max(args.sample_hz, 2.0)

    print("=" * 62)
    print(" TRACK RUN — start line to finish line")
    print("=" * 62)
    print(f"  START GPS/pose  x={start_x:.3f} m  y={START_Y:.3f} m  (green)")
    print(f"  FINISH GPS/pose x={finish_x:.3f} m  y={FINISH_Y:.3f} m  (red)")
    print(f"  Track length    {track_len:.3f} m")
    print(f"  Command ω={args.wheel_v:.2f} rad/s  →  v=ωr={v_cmd:.3f} m/s")
    print(f"  ETA if no-slip  ~{track_len / max(v_cmd, 0.01):.1f} s")
    print()

    st = twin(base)
    if not (st.get("bridge") or {}).get("external_active"):
        print("FAIL: twin not linked — open butlerbot.wbt with dashboard up")
        return 1

    print("Park + holdoff (so drive is accepted)...")
    wait_park(base)
    time.sleep(3.5)

    st0 = twin(base)
    x0, y0 = pose_xy(st0)
    err_start = math.hypot(x0 - start_x, y0 - START_Y)
    print(f"  robot at start: x={x0:.4f} y={y0:.4f}  |err vs start line|={err_start:.4f} m")
    if err_start > 0.5:
        print("  WARN: robot not near start line — reload world (spawn is x=0)")

    print("CONTINUOUS free roll toward finish (re-assert drive)...")
    print("  Watch Webots: green → red, one continuous push, then stop.")
    t0 = time.time()
    last_assert = 0.0
    samples: list[dict] = []
    path_gps = 0.0
    path_odo = 0.0
    finished = False
    t_finish = None

    while time.time() - t0 < args.timeout:
        t = time.time() - t0
        if t - last_assert >= 0.4:
            # Keep drive alive until past finish; long duration each reassert
            cmd(
                base,
                {
                    "drive": {
                        "left": args.wheel_v,
                        "right": args.wheel_v,
                        "duration_s": 3.0,
                    }
                },
            )
            last_assert = t

        st = twin(base)
        x, y = pose_xy(st)
        d = diag(st)
        wl = abs(float(d.get("hub_left_rad_s") or 0))
        wr = abs(float(d.get("hub_right_rad_s") or 0))
        v_odo = 0.5 * (wl + wr) * r
        if samples:
            px, py = samples[-1]["x"], samples[-1]["y"]
            path_gps += math.hypot(x - px, y - py)
            path_odo += v_odo * dt
        samples.append(
            {
                "t": round(t, 3),
                "x": x,
                "y": y,
                "v_odo": v_odo,
                "omega": 0.5 * (wl + wr),
                "abs": d.get("abs_active"),
                "src": d.get("cmd_source"),
            }
        )
        if int(t) != int(samples[-2]["t"] if len(samples) > 1 else -1):
            print(
                f"  t={t:5.2f}s  GPS=({x:.3f},{y:.3f})  "
                f"dist_to_finish={finish_x - x:.3f}m  "
                f"ω={0.5*(wl+wr):.2f}  src={d.get('cmd_source')} abs={d.get('abs_active')}"
            )
            if d.get("abs_active") and t > 0.8:
                print("  !! ABS active mid-track (free-roll interrupted)")

        if x >= finish_x - args.finish_tol:
            finished = True
            t_finish = t
            print(f"  FINISH CROSSED at t={t:.2f}s  x={x:.4f}")
            break
        time.sleep(dt)

    print("Stop on/after finish...")
    wait_park(base)
    st1 = twin(base)
    x1, y1 = pose_xy(st1)
    err_finish = abs(x1 - finish_x)
    err_lane = abs(y1 - FINISH_Y)
    run_t = t_finish if t_finish is not None else (time.time() - t0)
    chord = math.hypot(x1 - x0, y1 - y0)

    print()
    print("=" * 62)
    print(" TRACK RESULTS")
    print("=" * 62)
    print(f"  start line GPS     ({start_x:.3f}, {START_Y:.3f})")
    print(f"  robot at start     ({x0:.4f}, {y0:.4f})  err={err_start:.4f} m")
    print(f"  finish line GPS    ({finish_x:.3f}, {FINISH_Y:.3f})")
    print(f"  robot at end       ({x1:.4f}, {y1:.4f})  |x-finish|={err_finish:.4f} m")
    print(f"  lane y error       {err_lane:.4f} m")
    print(f"  track length       {track_len:.3f} m")
    print(f"  GPS chord start→end {chord:.4f} m")
    print(f"  GPS path ∫ds       {path_gps:.4f} m")
    print(f"  wheel odo ∫ωr      {path_odo:.4f} m")
    print(f"  run time           {run_t:.2f} s  (min free-roll target 5.0 s)")
    print(f"  finished           {finished}")

    ok_start = err_start < 0.5
    ok_finish = finished and err_finish < args.finish_tol + 0.15
    ok_time = run_t >= 5.0
    ok_dist = abs(chord - track_len) < 1.0 or abs(path_gps - track_len) < 1.0
    # Wheel odo vs track: if slip, will fail — report honestly
    ok_odo = abs(path_odo - track_len) < 1.5

    print()
    print("  CHECKS:")
    print(f"    start coords match spawn     : {'PASS' if ok_start else 'FAIL'}")
    print(f"    reached finish line          : {'PASS' if finished else 'FAIL'}")
    print(f"    end x ≈ finish GPS           : {'PASS' if ok_finish else 'FAIL'}")
    print(f"    run time ≥ 5 s               : {'PASS' if ok_time else 'FAIL'}")
    print(f"    body distance ≈ 5 m          : {'PASS' if ok_dist else 'FAIL'}")
    print(f"    wheel odo ≈ 5 m              : {'PASS' if ok_odo else 'FAIL'} (slip-sensitive)")

    row = {
        "label": args.label,
        "duration_s": round(run_t, 3),
        "distance_m": round(chord, 4),
        "energy_wh": None,
        "avg_legs_w": None,
        "peak_legs_w": None,
        "avg_total_w": None,
        "battery_start_pct": (st0.get("robot") or {}).get("main_battery_pct"),
        "battery_end_pct": (st1.get("robot") or {}).get("main_battery_pct"),
        "wheels_locked_after": (diag(st1).get("wheels") or {}).get("both_locked"),
        "hub_left_max_abs": max((s["omega"] for s in samples), default=0),
        "hub_right_max_abs": max((s["omega"] for s in samples), default=0),
        "details": {
            "test": "track_run",
            "start_line": {"x": start_x, "y": START_Y},
            "finish_line": {"x": finish_x, "y": FINISH_Y},
            "track_length_m": track_len,
            "robot_start": {"x": x0, "y": y0},
            "robot_end": {"x": x1, "y": y1},
            "err_start_m": err_start,
            "err_finish_x_m": err_finish,
            "path_gps_m": path_gps,
            "path_odo_m": path_odo,
            "chord_m": chord,
            "v_cmd": args.wheel_v * r,
            "finished": finished,
            "samples_n": len(samples),
            "samples_head": samples[:6],
            "samples_tail": samples[-4:],
        },
    }
    try:
        res = _req(f"{base}/api/measurements", method="POST", body=row)
        print(f"  DB id = {res.get('id')}")
    except urllib.error.URLError as e:
        print(f"  DB write failed: {e}")

    print("=" * 62)
    return 0 if (ok_start and finished and ok_time) else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)
