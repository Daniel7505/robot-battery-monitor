#!/usr/bin/env python3
"""Per-run summary of lane-vision.csv, junction columns included.

For each run (one ``run_id`` per controller start) it prints: the track,
states visited, junction classifications / openings seen, route choices,
blind distance, max / mean cross-track error, time and the outcome (exit
crossed or where it stopped). ``drift_report.py`` scores drift only; this
one is for the intersection runs.

Which track a run was on:
  1. ``--track NAME`` (forces it for every run), else
  2. ``lane-vision-runs.csv`` next to the log (written by the controller
     since 2026-10-02: world, track, RBM_ROUTE, warnings), else
  3. a guess from where the run ended and what the classifier saw
     (marked ``guess``).
Runs whose rows are identical to an earlier run (same world + same route
replay frame for frame in Webots) are flagged.

Usage (Windows cmd, from the repo folder):
    python scripts\\run_report.py "%USERPROFILE%\\OneDrive\\Desktop\\Grok Workspace\\lane-vision.csv"
    python scripts\\run_report.py --last 3
    python scripts\\run_report.py path\\to\\lane-vision.csv --track t_end
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

LOG_DIR = (Path(os.path.expanduser(os.path.expandvars(os.environ["RBM_LOG_DIR"])))
           if os.environ.get("RBM_LOG_DIR") else Path.home() / "OneDrive" / "Desktop" / "Grok Workspace")
DEFAULT_LOG = LOG_DIR / "lane-vision.csv"
META_NAME = "lane-vision-runs.csv"
EXIT_TOL_M = 0.15  # rows are ~0.3 s apart: the last one can stop short of the red line
# What the junction classifier should report on each intersection track
# (first junction); used only to guess the track of an unlabelled run.
TRACK_JUNCTION = {"plus": "plus", "t_end": "t_end", "t_left": "side_L", "t_right": "side_R",
                  "gap15": "plus", "gap25": "plus", "gap35": "plus", "gap45": "plus"}
_SKIP_SAME = {"unix_s", "run_id"}


def _f(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def load_runs(path: Path) -> list[list[dict]]:
    runs: list[list[dict]] = []
    with path.open(newline="", encoding="utf-8", errors="replace") as fh:
        for r in csv.DictReader(fh):
            if _f(r.get("unix_s")) is None or _f(r.get("x_m")) is None:
                continue  # repeated header / junk
            r = {k: (v or "").strip() for k, v in r.items() if k is not None}
            if runs and runs[-1][-1].get("run_id", "") == r.get("run_id", ""):
                runs[-1].append(r)
            else:
                runs.append([r])
    return runs


def load_meta(path: Path) -> dict[str, dict]:
    if not path.is_file():
        return {}
    with path.open(newline="", encoding="utf-8", errors="replace") as fh:
        return {r["run_id"]: r for r in csv.DictReader(fh) if r.get("run_id")}


def _compress(seq):
    out = []
    for v in seq:
        if out and out[-1][0] == v:
            out[-1][1] += 1
        else:
            out.append([v, 1])
    return out


def junction_views(run) -> list[tuple[str, str, int]]:
    """Consecutive (junction_type, openings) the classifier reported, with frame counts."""
    seq = [(r.get("junction_type", ""), r.get("openings", "")) for r in run]
    return [(k, o, n) for (k, o), n in _compress(seq) if k or o]


def route_choices(run) -> list[str]:
    return [c for c, _n in _compress(r.get("route_choice", "") for r in run) if c]


def _load_track(name):
    from src.track_geometry import load_track

    return load_track(name)


def exit_of(geom, run):
    if geom is None:
        return None
    for r in run:
        x, y = _f(r["x_m"]), _f(r["y_m"])
        if hasattr(geom, "exit_crossed"):
            e = geom.exit_crossed(x, y, EXIT_TOL_M)
            if e:
                return e
        elif geom.crossed_finish(x, y, tol_m=EXIT_TOL_M):
            return "finish"
    return None


def guess_track(run, tracks: dict) -> str | None:
    """Intersection track whose exit the run ended on and whose junction kind
    matches the classifier's first junction."""
    views = junction_views(run)
    first = views[0][0] if views else None
    best, best_score = None, 0
    for name, g in tracks.items():
        score = (2 if exit_of(g, run[-3:]) else 0) + (1 if first and TRACK_JUNCTION.get(name) == first else 0)
        if score > best_score:
            best, best_score = name, score
    return best if best_score >= 2 else None


def summarize(n, run, meta, track_arg, tracks, earlier):
    t0, t1 = _f(run[0]["unix_s"]), _f(run[-1]["unix_s"])
    rid = run[0].get("run_id", "")
    m = meta.get(rid, {})
    track, how = None, ""
    if track_arg:
        track, how = track_arg, "--track"
    elif m.get("track"):
        track, how = m["track"], f"logged ({m.get('world', '?')})"
    else:
        track = guess_track(run, tracks)
        how = "guess" if track else ""
    geom = None
    if track:
        try:
            geom = tracks.get(track) or _load_track(track)
        except (OSError, ValueError, KeyError):
            geom = None
    ct = []
    if geom is not None:
        hint = None
        for r in run:
            pr = geom.project(_f(r["x_m"]), _f(r["y_m"]), hint)
            hint = pr.get("index")
            ct.append(abs(pr["lateral_m"]))
    states = [s for s, _n in _compress(r.get("state", "") for r in run) if s]
    blind = [b for b in (_f(r.get("blind_m")) for r in run) if b is not None]
    ex = exit_of(geom, run)
    last = run[-1]
    sig = [tuple(v for k, v in r.items() if k not in _SKIP_SAME) for r in run]
    same = next((k for k, s in earlier if s == sig), None)
    earlier.append((n, sig))
    return {
        "run": n, "run_id": rid,
        "start": datetime.fromtimestamp(t0).strftime("%H:%M:%S") if t0 else "",
        "time_s": round((t1 or 0) - (t0 or 0), 1),
        "track": track or "?", "track_how": how,
        "route_env": m.get("route", "") if m else None,
        "states": " > ".join(states),
        "junctions": "; ".join(f"{k or '-'}[{o.replace(' ', '')}]x{c}" for k, o, c in junction_views(run)),
        "choices": ",".join(route_choices(run)) or "-",
        "blind_max_m": round(max(blind), 2) if blind else None,
        "ct_max_cm": round(100 * max(ct), 1) if ct else None,
        "ct_mean_cm": round(100 * sum(ct) / len(ct), 1) if ct else None,
        "outcome": (f"exit {ex}" if ex else
                    f"no exit: last state {last.get('state') or '?'} at ({_f(last['x_m']):.2f}, {_f(last['y_m']):.2f})"),
        "end_xy": (round(_f(last["x_m"]), 2), round(_f(last["y_m"]), 2)),
        "same_as_run": same,
        "warnings": m.get("warnings", "") if m else "",
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", nargs="?", type=Path, default=DEFAULT_LOG)
    ap.add_argument("--track", help="track for every run (name in tracks/ or a .json); default: logged or guessed")
    ap.add_argument("--runs-file", type=Path, help=f"run metadata CSV (default: {META_NAME} next to the log)")
    ap.add_argument("--last", type=int, default=0, help="only the N newest runs")
    ap.add_argument("--min-rows", type=int, default=5)
    a = ap.parse_args(argv)
    if not a.log.is_file():
        print(f"Log not found: {a.log}", file=sys.stderr)
        return 1
    runs = [r for r in load_runs(a.log) if len(r) >= a.min_rows]
    meta = load_meta(a.runs_file or a.log.parent / META_NAME)
    tracks = {}
    for name in TRACK_JUNCTION:
        try:
            tracks[name] = _load_track(name)
        except Exception:
            pass
    earlier: list = []
    stats = [summarize(i + 1, r, meta, a.track, tracks, earlier) for i, r in enumerate(runs)]
    if a.last:
        stats = stats[-a.last:]
    print(f"{a.log}  ({sum(len(r) for r in runs)} rows, {len(runs)} runs"
          + (f", run metadata: {len(meta)} runs" if meta else ", no lane-vision-runs.csv: tracks are guessed") + ")\n")
    for s in stats:
        tr = s["track"] + (f" ({s['track_how']})" if s["track_how"] else "")
        print(f"run {s['run']}  {s['run_id']}  start {s['start']}  {s['time_s']} s  track {tr}"
              + (f"  RBM_ROUTE={s['route_env'] or '(unset)'}" if s["route_env"] is not None else ""))
        print(f"  states:    {s['states'] or '-'}")
        print(f"  junctions: {s['junctions'] or '-'}")
        print(f"  route:     {s['choices']}" + (f"   blind max {s['blind_max_m']} m" if s["blind_max_m"] else ""))
        if s["ct_max_cm"] is not None:
            print(f"  cross-track: max {s['ct_max_cm']} cm, mean {s['ct_mean_cm']} cm")
        print(f"  outcome:   {s['outcome']}")
        if s["same_as_run"]:
            print(f"  NOTE: every row identical to run {s['same_as_run']} (same world and route replayed) "
                  "— check which world was really loaded")
        if s["warnings"]:
            print(f"  WARNING:   {s['warnings']}")
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
