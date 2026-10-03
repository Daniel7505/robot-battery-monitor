#!/usr/bin/env python3
"""Per-run summary of lane-vision.csv, junction columns included.

For each run (one ``run_id`` per controller start) it prints: the track,
states visited, junction classifications / openings seen, route choices
(every decision, S,S included) vs. junctions driven through, blind distance, max / mean cross-track error, time and the outcome (exit
crossed or where it stopped). ``drift_report.py`` scores drift only; this
one is for the intersection runs. Runs logged with the obstacle columns
(``obstacle_state, obstacle_m, obstacle_band, obstacle_conf``; empty when
RBM_OBSTACLES was off) also get an ``obstacles:`` line: every slow / stop
with where, how far, which height band, how long it waited and how it
resumed.

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
                  "gap15": "plus", "gap25": "plus", "gap35": "plus", "gap45": "plus",
                  "warehouse": "plus"}
JN_PASS_M = 0.6  # GPS within this of a junction node = the robot drove through it
JN_DECISION_M = 2.5  # a decision is logged ~1-1.5 m before the junction centre
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


def _ways(openings: str) -> int:
    o = openings or ""
    return int("L" in o) + int("S" in o) + int("R" in o)


def route_decisions(run) -> list[dict]:
    """Junction decisions visible in the log, in order: {choice, x, y, kind}.

    ``route_choice`` is sticky (the controller keeps the last choice), so two
    equal choices in a row (S,S) don't show as a change. A decision is:
      * ``route_choice`` changing, unless the junction offers one way only
        (the end of the paint, ``lane_lost [-S?-]``: no route entry used), or
        it changes inside GAP_CROSS (a line across the gap: the same junction
        turned, not a new entry: the last decision is corrected);
      * leaving JUNCTION_APPROACH for GAP_CROSS / CORNER_APPROACH at a junction
        with >= 2 ways, when that approach had no decision yet (S then S).
    A straight pass of a side branch that repeats the previous choice leaves no
    trace in the CSV (the controller drops the junction in the same frame), so
    compare with the GPS junction count.
    """
    out: list[dict] = []
    prev_rc, prev_state, prev_kind, prev_op = "", "", "", ""
    decided = False
    for r in run:
        st, rc = r.get("state", ""), r.get("route_choice", "")
        kind, op = r.get("junction_type", ""), r.get("openings", "")
        if st == "JUNCTION_APPROACH" and prev_state != "JUNCTION_APPROACH":
            decided = False
        x, y = _f(r.get("x_m")), _f(r.get("y_m"))
        if rc and rc != prev_rc:
            if prev_state == "GAP_CROSS" and out:
                out[-1]["choice"] = rc
                out[-1]["note"] = "gap T: turned"
            elif not (kind and _ways(op) < 2):
                if not (decided and out):
                    out.append({"choice": rc, "x": x, "y": y, "kind": kind or "side"})
                else:
                    out[-1]["choice"] = rc
                decided = True
        elif (st in ("GAP_CROSS", "CORNER_APPROACH") and prev_state == "JUNCTION_APPROACH" and not decided
              and _ways(op or prev_op) >= 2):
            out.append({"choice": rc or "S", "x": x, "y": y, "kind": kind or prev_kind})
            decided = True
        if st != "JUNCTION_APPROACH":
            decided = False  # the approach is over: the next change is the next junction
        prev_rc, prev_state, prev_kind, prev_op = rc or prev_rc, st, kind, op
    return out


def route_choices(run) -> list[str]:
    return [d["choice"] for d in route_decisions(run)]


def obstacle_events(run) -> dict | None:
    """Slow / stop events from the obstacle columns; None when the log has none (older file)."""
    if not any("obstacle_state" in r for r in run):
        return None
    on = any(r.get("obstacle_state") for r in run)
    stops, slows = [], 0
    prev = ""
    cur = None
    dists = [d for d in (_f(r.get("obstacle_m")) for r in run) if d is not None]
    bands = []
    for r in run:
        st = r.get("obstacle_state", "")
        b = r.get("obstacle_band", "")
        if b and b not in bands:
            bands.append(b)
        if st != prev:
            t = _f(r.get("unix_s"))
            if st == "SLOW_FOR_OBSTACLE" and prev != "STOPPED_FOR_OBSTACLE":
                slows += 1
            if st == "STOPPED_FOR_OBSTACLE":
                cur = {"t": t, "x": _f(r.get("x_m")), "y": _f(r.get("y_m")), "m": _f(r.get("obstacle_m")),
                       "band": b, "wait_s": None, "how": "still stopped at the end of the log"}
                stops.append(cur)
            elif prev == "STOPPED_FOR_OBSTACLE" and cur is not None:
                cur["wait_s"] = None if (t is None or cur["t"] is None) else round(t - cur["t"], 1)
                cur["how"] = "corridor cleared" if st == "CLEAR" else "obstacle moved off (resumed slow)"
                cur = None
        prev = st
    if cur is not None and cur["t"] is not None:
        cur["wait_s"] = round((_f(run[-1].get("unix_s")) or cur["t"]) - cur["t"], 1)
    return {"on": on, "slows": slows, "stops": stops, "nearest_m": round(min(dists), 2) if dists else None,
            "bands": bands}


def obstacle_line(o: dict | None) -> str | None:
    if o is None:
        return None
    if not o["on"]:
        return "obstacles: off (RBM_OBSTACLES)"
    parts = [f"{len(o['stops'])} stop(s), {o['slows']} slow-down(s)"]
    if o["nearest_m"] is not None:
        parts.append(f"nearest {o['nearest_m']} m ({'/'.join(o['bands']) or '-'})")
    for k, st in enumerate(o["stops"], 1):
        where = "" if st["x"] is None else f" at ({st['x']:.2f}, {st['y']:.2f})"
        parts.append(f"stop {k}{where}: {st['band'] or '?'} obstacle "
                     + ("" if st["m"] is None else f"{st['m']:.2f} m ahead") + ", waited "
                     + ("?" if st["wait_s"] is None else f"{st['wait_s']} s") + f", {st['how']}")
    return "obstacles: " + "; ".join(parts)


def _graph(geom):
    if geom is None or not hasattr(geom, "roads"):
        return None
    try:
        from src.route_planner import RoadGraph

        return RoadGraph(geom)
    except Exception:
        return None


def _node_name(geom, node) -> str:
    names = sorted(geom.roads[k].spec.name for k in node.roads)
    return " x ".join(names) + f" ({node.xy[0]:g}, {node.xy[1]:g})"


def junctions_passed(graph, run) -> list | None:
    """Junction nodes the GPS track drove through, in order (a node counts
    again after the robot has been 2 m away from it)."""
    if graph is None:
        return None
    nodes = graph.junctions
    out, inside = [], set()
    for r in run:
        x, y = _f(r["x_m"]), _f(r["y_m"])
        for nd in nodes:
            d = math.hypot(x - nd.xy[0], y - nd.xy[1])
            if d <= JN_PASS_M and nd.id not in inside:
                inside.add(nd.id)
                out.append(nd)
            elif d > 2.0:
                inside.discard(nd.id)
    return out


def silent_junctions(passed, decisions, radius_m: float = JN_DECISION_M) -> list:
    """Junctions driven through with no logged decision within ``radius_m``
    of them (each decision explains at most one pass)."""
    left = list(decisions)
    out = []
    for nd in passed or []:
        k = next((i for i, d in enumerate(left) if d["x"] is not None
                  and math.hypot(d["x"] - nd.xy[0], d["y"] - nd.xy[1]) <= radius_m), None)
        if k is None:
            out.append(nd)
        else:
            left.pop(k)
    return out


def nearest_feature(geom, graph, x, y) -> str:
    if graph is None:
        return ""
    cands = [(math.hypot(x - nd.xy[0], y - nd.xy[1]), "junction " + _node_name(geom, nd)) for nd in graph.junctions]
    cands += [(math.hypot(x - p[0], y - p[1]), f"corner {geom.roads[k].spec.name} ({p[0]:g}, {p[1]:g})")
              for k, p, _a in graph.corners()]
    if not cands:
        return ""
    d, what = min(cands)
    return f"{d:.1f} m from {what}"


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
    graph = _graph(geom)
    worst = ""
    if ct:
        i = max(range(len(ct)), key=ct.__getitem__)
        wx, wy = _f(run[i]["x_m"]), _f(run[i]["y_m"])
        near = nearest_feature(geom, graph, wx, wy)
        worst = f"({wx:.2f}, {wy:.2f}) {run[i].get('state', '')}" + (f", {near}" if near else "")
    decisions = route_decisions(run)
    passed = junctions_passed(graph, run)
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
        "choices": ",".join(d["choice"] for d in decisions) or "-",
        "n_decisions": len(decisions),
        "jn_passed": None if passed is None else len(passed),
        "jn_silent": [_node_name(geom, nd) for nd in silent_junctions(passed, decisions)],
        "ct_worst_at": worst,
        "blind_max_m": round(max(blind), 2) if blind else None,
        "ct_max_cm": round(100 * max(ct), 1) if ct else None,
        "ct_mean_cm": round(100 * sum(ct) / len(ct), 1) if ct else None,
        "outcome": (f"exit {ex}" if ex else
                    f"no exit: last state {last.get('state') or '?'} at ({_f(last['x_m']):.2f}, {_f(last['y_m']):.2f})"),
        "end_xy": (round(_f(last["x_m"]), 2), round(_f(last["y_m"]), 2)),
        "same_as_run": same,
        "warnings": m.get("warnings", "") if m else "",
        "obstacles": obstacle_events(run),
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
        jn = f"  decisions: {s['n_decisions']} in the log"
        if s["jn_passed"] is not None:
            jn += f", {s['jn_passed']} junctions driven through (GPS)"
        planned = len([c for c in (s["route_env"] or "").replace(";", ",").replace(" ", ",").split(",") if c.strip()])
        if planned:
            jn += f", RBM_ROUTE has {planned}"
        print(jn)
        if s["jn_silent"]:
            print(f"  NOTE: no decision logged at {len(s['jn_silent'])} junction(s): " + "; ".join(s["jn_silent"])
                  + ". A straight pass of a side branch that repeats the previous choice leaves no trace in the CSV"
                  " (check the next choices still line up with RBM_ROUTE), or the junction was missed")
        if s["ct_max_cm"] is not None:
            print(f"  cross-track: max {s['ct_max_cm']} cm, mean {s['ct_mean_cm']} cm"
                  + (f"  (worst at {s['ct_worst_at']})" if s["ct_worst_at"] else ""))
        ol = obstacle_line(s["obstacles"])
        if ol:
            print(f"  {ol}")
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
