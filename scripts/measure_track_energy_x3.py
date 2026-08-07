#!/usr/bin/env python3
"""
Three independent 15 m track simulations with energy readings each run.

Each run:
  1. Reloads Webots world (fresh spawn at start line)
  2. Waits for twin link
  3. Runs measure_track_run.py (energy + free-roll metrics)
  4. Collects results and prints a side-by-side comparison

Usage:
  python scripts/measure_track_energy_x3.py
  python scripts/measure_track_energy_x3.py --runs 3 --timeout 60
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LAUNCH = ROOT / "scripts" / "launch_webots_twin.ps1"
MEASURE = ROOT / "scripts" / "measure_track_run.py"


def twin_ok(url: str, timeout: float = 4.0) -> bool:
    try:
        req = urllib.request.Request(f"{url.rstrip('/')}/api/twin/state")
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            st = json.loads(resp.read().decode("utf-8"))
        return bool((st.get("bridge") or {}).get("external_active"))
    except Exception:
        return False


def pose_x(url: str) -> float | None:
    try:
        req = urllib.request.Request(f"{url.rstrip('/')}/api/twin/state")
        with urllib.request.urlopen(req, timeout=4.0) as resp:
            st = json.loads(resp.read().decode("utf-8"))
        p = (st.get("external_feed") or {}).get("pose") or {}
        return float(p.get("x_m") or 0.0)
    except Exception:
        return None


def reload_webots() -> None:
    # launch script kills old Webots and opens butlerbot.wbt
    subprocess.Popen(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(LAUNCH),
        ],
        cwd=str(ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def wait_ready(url: str, max_s: float = 45.0) -> bool:
    deadline = time.time() + max_s
    while time.time() < deadline:
        if twin_ok(url):
            x = pose_x(url)
            # Fresh spawn should be near start; allow brief settle
            if x is not None and x < 1.0:
                time.sleep(2.0)
                return True
            if x is not None and x >= 1.0:
                # Still linked but mid-track — keep waiting for reload spawn
                pass
        time.sleep(1.0)
    return twin_ok(url)


def run_one(url: str, label: str, wheel_v: float, timeout: float) -> dict:
    cmd = [
        sys.executable,
        str(MEASURE),
        "--url",
        url,
        "--label",
        label,
        "--wheel-v",
        str(wheel_v),
        "--timeout",
        str(timeout),
    ]
    print()
    print("#" * 62)
    print(f"  RUN {label}")
    print("#" * 62)
    proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
    out = (proc.stdout or "") + (proc.stderr or "")
    print(out)
    # Pull latest measurement from API for structured compare
    row: dict = {"label": label, "exit": proc.returncode, "raw_tail": out[-800:]}
    try:
        req = urllib.request.Request(
            f"{url.rstrip('/')}/api/measurements?limit=1"
        )
        with urllib.request.urlopen(req, timeout=8) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        meas = (data.get("measurements") or [None])[0]
        if meas:
            row.update(
                {
                    "id": meas.get("id"),
                    "duration_s": meas.get("duration_s"),
                    "distance_m": meas.get("distance_m"),
                    "energy_wh": meas.get("energy_wh"),
                    "avg_legs_w": meas.get("avg_legs_w"),
                    "peak_legs_w": meas.get("peak_legs_w"),
                    "avg_total_w": meas.get("avg_total_w"),
                    "battery_start_pct": meas.get("battery_start_pct"),
                    "battery_end_pct": meas.get("battery_end_pct"),
                    "details": meas.get("details") or {},
                }
            )
    except Exception as e:
        row["fetch_error"] = str(e)
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:5000")
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--wheel-v", type=float, default=5.5)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--label-prefix", default="energy_15m")
    args = ap.parse_args()

    results: list[dict] = []
    for i in range(1, args.runs + 1):
        print(f"\n>>> Reloading Webots for independent sim {i}/{args.runs} ...")
        reload_webots()
        if not wait_ready(args.url, max_s=50.0):
            print(f"FAIL: twin not ready for run {i}")
            results.append({"label": f"{args.label_prefix}_r{i}", "error": "twin not ready"})
            continue
        label = f"{args.label_prefix}_r{i}"
        results.append(run_one(args.url, label, args.wheel_v, args.timeout))
        time.sleep(1.5)

    print()
    print("=" * 62)
    print(" ENERGY REPEATABILITY — 3× independent 15 m runs")
    print("=" * 62)
    print(
        f"{'run':<16} {'t_s':>7} {'dist':>7} {'E_Wh':>9} "
        f"{'Legs_avg':>9} {'Legs_pk':>8} {'tot_avg':>8} {'fin':>5}"
    )
    energies = []
    legs = []
    totals = []
    times = []
    for r in results:
        det = r.get("details") or {}
        fin = det.get("finished")
        e = r.get("energy_wh")
        if e is not None:
            energies.append(float(e))
        if r.get("avg_legs_w") is not None:
            legs.append(float(r["avg_legs_w"]))
        if r.get("avg_total_w") is not None:
            totals.append(float(r["avg_total_w"]))
        if r.get("duration_s") is not None:
            times.append(float(r["duration_s"]))
        print(
            f"{str(r.get('label')):<16} "
            f"{_fmt(r.get('duration_s'), 7, 2)} "
            f"{_fmt(r.get('distance_m'), 7, 2)} "
            f"{_fmt(r.get('energy_wh'), 9, 4)} "
            f"{_fmt(r.get('avg_legs_w'), 9, 2)} "
            f"{_fmt(r.get('peak_legs_w'), 8, 2)} "
            f"{_fmt(r.get('avg_total_w'), 8, 2)} "
            f"{'Y' if fin else 'N':>5}"
        )

    def spread(vals: list[float]) -> str:
        if not vals:
            return "n/a"
        if len(vals) == 1:
            return f"{vals[0]:.4f} (n=1)"
        mean = sum(vals) / len(vals)
        mn, mx = min(vals), max(vals)
        pct = 100.0 * (mx - mn) / mean if mean else 0.0
        return f"mean={mean:.4f}  min={mn:.4f}  max={mx:.4f}  spread={pct:.1f}%"

    print()
    print("  COMPARISON:")
    print(f"    energy Wh (∫P dt) : {spread(energies)}")
    print(f"    avg Legs W        : {spread(legs)}")
    print(f"    avg total W       : {spread(totals)}")
    print(f"    duration s        : {spread(times)}")
    if energies and max(energies) > 0:
        rel = (max(energies) - min(energies)) / (sum(energies) / len(energies))
        print(
            f"    energy consistency: "
            f"{'SIMILAR' if rel < 0.10 else 'VARIES'} "
            f"(max-min)/mean = {100*rel:.1f}%"
        )
    print("=" * 62)
    return 0 if all(r.get("exit", 1) == 0 for r in results if "exit" in r) else 2


def _fmt(v, width: int, decimals: int) -> str:
    if v is None:
        return f"{'—':>{width}}"
    return f"{float(v):>{width}.{decimals}f}"


if __name__ == "__main__":
    sys.exit(main())
