#!/usr/bin/env python3
"""Offline matrix for the intersection work (scripts/rowfit_sim.py in the loop).

  python scripts/intersection_matrix.py            # routes x courses + gap series + robustness + regressions
  python scripts/intersection_matrix.py --only routes
  python scripts/intersection_matrix.py --jobs 8
  python scripts/intersection_matrix.py --only warehouse   # every mission in tracks/warehouse.json

Prints markdown tables (paste into docs / PR). Success on an intersection
course = finished AND crossed the exit the route asks for (course ``routes``).
"""
from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

ROUTE_CASES = [
    ("plus", "S"), ("plus", "L"), ("plus", "R"), ("plus", None),
    ("t_end", "L"), ("t_end", "R"), ("t_end", "S"), ("t_end", None),
    ("t_left", "S"), ("t_left", "L"), ("t_left", None),
    ("t_right", "S"), ("t_right", "R"), ("t_right", None),
]
GAP_CASES = [("gap15", None), ("gap25", None), ("gap35", None), ("gap45", None)]
ROBUST = [  # (label, kwargs)
    ("IMU drift +0.25 deg/s", {"imu_drift_dps": 0.25}),
    ("IMU drift +0.5 deg/s", {"imu_drift_dps": 0.5}),
    ("IMU drift -0.5 deg/s, noise 0.3 deg", {"imu_drift_dps": -0.5, "imu_noise_deg": 0.3}),
    ("tyre scrub turn_eff 0.8", {"turn_eff": 0.8}),
    ("offset start +0.15 m", {"start_offset": 0.15}),
    ("offset start -0.15 m, yaw +5 deg", {"start_offset": -0.15, "start_yaw": 0.0873}),
]
ROBUST_CASES = [("plus", "S"), ("plus", "L"), ("t_end", "R"), ("t_left", "L"), ("t_right", "S"), ("gap25", None),
                ("gap45", None)]
WAREHOUSE = "warehouse"  # missions (RBM_ROUTE strings) are read from the track file


def warehouse_cases(track: str = WAREHOUSE) -> list:
    """(course, route, kwargs) per mission in the track file; time limit from the route length."""
    import json

    from src.track_geometry import track_path

    with open(track_path(track), encoding="utf-8") as fh:
        d = json.load(fh)
    return [(track, m["route"], {"max_s": 120.0 + 4.0 * float(m["length_m"])}) for m in d.get("missions", [])]


REGRESSION = ["corner90", "corner_mix", "corner90_right", "corner90_wide", "s", "widen", "turn90", "s_finish"]


def _one(args):
    course, route, kw = args
    from rowfit_sim import run

    os.environ.setdefault("RBM_JUNCTION_STRICT", "1")
    r = run(course, route=route, **kw)
    r.pop("events", None)
    return course, route, kw, r


def _fmt_case(course, route, r):
    worst = r.get("max_lateral_m", r["max_ct_m"])
    piv = r.get("pivots") or []
    perr = " ".join(f"{p['err_deg']:+.1f}/{p.get('true_err_deg', float('nan')):+.1f}" for p in piv) or "-"
    jn = "; ".join(f"{j['kind']}[{j['openings'].replace(' ', '')}]→{j['choice']}"
                   + (f" blind {j['blind_m']:.2f}m" if "blind_m" in j else "") for j in r.get("junctions", [])) or "-"
    ok = r.get("ok", r["finished"])
    ex = r.get("exit")
    return (f"| {course} | {route or 'default'} | {r.get('expected_exit', '-')} | {ex or '-'} | "
            f"{'OK' if ok else 'FAIL'} | {r['time_s']:.1f} | {100 * worst:.1f} | {perr} | {jn} |")


HDR = ("| course | route | want exit | exit | result | time s | worst cm | pivot err imu/true ° | junctions |\n"
       "|---|---|---|---|---|---|---|---|---|")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--only", choices=("routes", "gaps", "robust", "regress", "blind", "warehouse"))
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    a = ap.parse_args(argv)
    jobs = []
    sec = []
    if a.only in (None, "routes"):
        sec.append(("Route matrix", [(c, r, {}) for c, r in ROUTE_CASES]))
    if a.only in (None, "gaps"):
        sec.append(("Large-gap series (straight, default route)", [(c, r, {}) for c, r in GAP_CASES]))
    if a.only in (None, "blind"):
        sec.append(("Max blind too short (RBM_MAX_BLIND_M=2.0 on gap45: expect look-around + stop)",
                    [("gap45", None, {"max_blind": 2.0})]))
    if a.only in (None, "robust"):
        for label, kw in ROBUST:
            sec.append((f"Robustness: {label}", [(c, r, kw) for c, r in ROBUST_CASES]))
    if a.only in (None, "warehouse"):
        sec.append(("Warehouse missions (tracks/warehouse.json)", warehouse_cases()))
    if a.only in (None, "regress"):
        sec.append(("Regression (old courses)", [(c, None, {}) for c in REGRESSION]))
    for _t, cases in sec:
        jobs.extend(cases)
    with ProcessPoolExecutor(max_workers=max(1, a.jobs)) as ex:
        res = list(ex.map(_one, jobs))
    i = 0
    worst_all = 0.0
    for title, cases in sec:
        print(f"\n### {title}\n")
        print(HDR)
        for _ in cases:
            course, route, _kw, r = res[i]
            i += 1
            print(_fmt_case(course, route, r))
            if "exit" in r and r.get("ok") and "start_offset" not in _kw:
                worst_all = max(worst_all, r.get("max_lateral_m", 0.0))
            if not r.get("ok", r["finished"]):
                print(f"|  ↳ | final {r.get('final_state')} braked {r.get('braked_at')} |||||||")
    print(f"\nworst lateral error on successful intersection runs (offset starts excluded: their worst is the "
          f"start offset): {100 * worst_all:.1f} cm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
