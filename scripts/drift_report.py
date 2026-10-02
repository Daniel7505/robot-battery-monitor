#!/usr/bin/env python3
"""Lane-keep drift report for the Webots steer log.

Reads the CSV the ButlerBot controller appends to (default:
~/OneDrive/Desktop/Grok Workspace/steer-actions.csv), splits it into runs,
and prints how far the bot drifted off the centre line (y_m) in each run.

A new run starts when x_m jumps back toward the start (a reset / new sim)
or when there is a long gap between rows. Logs that carry a ``run_id``
column (``lane-vision.csv`` from rowfit, the default lane mode) are split by
run_id instead (one run per controller start); a ``mode`` column, when
present, is shown per run.

Usage:
    python scripts/drift_report.py                 # default log, all runs
    python scripts/drift_report.py path\\to\\log.csv
    python scripts/drift_report.py --last 5        # only the 5 newest runs
    python scripts/drift_report.py --out runs.csv  # also save a per-run CSV
    python scripts/drift_report.py --track s       # drift vs the S centre line, not y=0
    python scripts/drift_report.py "%USERPROFILE%\\OneDrive\\Desktop\\Grok Workspace\\lane-vision.csv"

The default folder can be moved with the RBM_LOG_DIR environment variable
(same as the Webots controller).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from datetime import datetime
from pathlib import Path

DEFAULT_LOG = (
    Path(os.path.expanduser(os.path.expandvars(os.environ["RBM_LOG_DIR"])))
    if os.environ.get("RBM_LOG_DIR")
    else Path.home() / "OneDrive" / "Desktop" / "Grok Workspace"
) / "steer-actions.csv"
try:  # single source of truth for the track: scripts/s_track.py (generates the .wbt paint)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from s_track import FINISH_X_M  # noqa: E402  (16.5 m red finish line)
except ImportError:  # pragma: no cover
    FINISH_X_M = 16.5  # end of the S corridor (matches butlerbot_controller)
RESET_DROP_M = 1.0     # x going backwards by this much = new run
GAP_S = 30.0           # silence this long between rows = new run
# Log rows land ~0.3 s apart (one camera frame), so the last row can be up to
# ~0.15 m short of the finish line the controller actually stopped on.
FINISH_TOL_M = 0.15


def _f(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def load_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8", errors="replace") as fh:
        reader = csv.DictReader(fh)
        rows = []
        for r in reader:
            t, x, y = _f(r.get("unix_s")), _f(r.get("x_m")), _f(r.get("y_m"))
            if t is None or x is None or y is None:
                continue  # skips repeated header lines / junk
            row = {"t": t, "x": x, "y": y, "steer": _f(r.get("steer"))}
            if (r.get("run_id") or "").strip():
                row["run_id"] = r["run_id"].strip()
            if (r.get("mode") or "").strip():
                row["mode"] = r["mode"].strip()
            rows.append(row)
    return rows


def split_runs(rows: list[dict]) -> list[list[dict]]:
    if rows and all("run_id" in r for r in rows):
        runs, cur = [], []
        for r in rows:
            if cur and r["run_id"] != cur[-1]["run_id"]:
                runs.append(cur)
                cur = []
            cur.append(r)
        if cur:
            runs.append(cur)
        return runs
    runs, cur = [], []
    for r in rows:
        if cur:
            prev = cur[-1]
            if r["x"] < prev["x"] - RESET_DROP_M or r["t"] - prev["t"] > GAP_S:
                runs.append(cur)
                cur = []
        cur.append(r)
    if cur:
        runs.append(cur)
    return runs


def summarize(run: list[dict], n: int) -> dict:
    ys = [r["y"] for r in run]
    absy = [abs(y) for y in ys]
    worst = max(run, key=lambda r: abs(r["y"]))
    steers = [abs(r["steer"]) for r in run if r["steer"] is not None]
    max_x = max(r["x"] for r in run)
    extra = {}
    if "run_id" in run[0]:
        extra["run_id"] = run[0]["run_id"]
    if "mode" in run[0]:
        extra["mode"] = run[0]["mode"]
    return {
        "run": n,
        **extra,
        "start": datetime.fromtimestamp(run[0]["t"]).strftime("%Y-%m-%d %H:%M:%S"),
        "duration_s": round(run[-1]["t"] - run[0]["t"], 1),
        "rows": len(run),
        "distance_m": round(max_x - min(r["x"] for r in run), 2),
        "finished": max_x >= FINISH_X_M - FINISH_TOL_M,
        "max_drift_m": round(max(absy), 3),
        "worst_at_x_m": round(worst["x"], 2),
        "worst_side": "left" if worst["y"] > 0 else "right" if worst["y"] < 0 else "-",
        "mean_drift_m": round(sum(absy) / len(absy), 3),
        "rms_drift_m": round(math.sqrt(sum(y * y for y in ys) / len(ys)), 3),
        "max_steer": round(max(steers), 3) if steers else None,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", nargs="?", type=Path, default=DEFAULT_LOG)
    ap.add_argument("--last", type=int, default=0, help="only show the N newest runs")
    ap.add_argument("--min-rows", type=int, default=5, help="ignore runs shorter than this")
    ap.add_argument("--out", type=Path, help="write the per-run summary to this CSV")
    ap.add_argument("--track", choices=["s"], help="score drift vs the S-track centre line "
                    "(scripts/s_track.py) instead of world y=0")
    a = ap.parse_args(argv)

    if not a.log.is_file():
        print(f"Log not found: {a.log}", file=sys.stderr)
        return 1
    rows = load_rows(a.log)
    if a.track == "s":
        from s_track import centerline

        for r in rows:
            cy, th = centerline(r["x"])
            r["y"] = (r["y"] - cy) * math.cos(th)
    runs = [r for r in split_runs(rows) if len(r) >= a.min_rows]
    stats = [summarize(r, i + 1) for i, r in enumerate(runs)]
    if a.last:
        stats = stats[-a.last:]
    if not stats:
        print("No runs found.")
        return 0

    print(f"{a.log}  ({len(rows)} rows, {len(runs)} runs)\n")
    labelled = any("run_id" in s for s in stats)
    hdr = f"{'run':>3}  {'start':19}  {'dur s':>6}  {'dist m':>6}  {'done':4}  " \
          f"{'max m':>6}  {'@x m':>5}  {'side':5}  {'mean m':>6}  {'rms m':>6}"
    if labelled:
        hdr += f"  {'run_id':15}  {'mode':6}"
    print(hdr)
    print("-" * len(hdr))
    for s in stats:
        line = (f"{s['run']:>3}  {s['start']:19}  {s['duration_s']:>6}  {s['distance_m']:>6}  "
                f"{'yes' if s['finished'] else 'no':4}  {s['max_drift_m']:>6}  "
                f"{s['worst_at_x_m']:>5}  {s['worst_side']:5}  {s['mean_drift_m']:>6}  "
                f"{s['rms_drift_m']:>6}")
        if labelled:
            line += f"  {s.get('run_id', '-'):15}  {s.get('mode', '-'):6}"
        print(line)
    if a.track:
        print("\nmax/mean/rms = sideways distance off the S-track centre line, metres.")
    else:
        print("\nmax/mean/rms = sideways distance off the centre line (|y_m|), metres.")

    if a.out:
        with a.out.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(stats[0]))
            w.writeheader()
            w.writerows(stats)
        print(f"Saved {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
