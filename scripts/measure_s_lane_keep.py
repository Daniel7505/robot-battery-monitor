#!/usr/bin/env python3
"""Arm lane-keep on the gentle S and report cross-track + finish."""

from __future__ import annotations

import json
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s_track import FINISH_X_M, cross_track_m  # noqa: E402

BASE = "http://127.0.0.1:5000"


def req(path: str, method: str = "GET", body: dict | None = None) -> dict:
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode()
        headers["Content-Type"] = "application/json"
    r = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(r, timeout=8) as resp:
        return json.loads(resp.read().decode())


def snap() -> dict:
    st = req("/api/twin/state")
    feed = st.get("external_feed") or {}
    p = feed.get("pose") or {}
    s = feed.get("sensors") or {}
    d = st.get("control_diag") or {}
    x = float(p.get("x_m") or 0)
    y = float(p.get("y_m") or 0)
    return {
        "x": x,
        "y": y,
        "ct": cross_track_m(x, y),
        "red": float(s.get("finish_red") or 0),
        "yL": float(s.get("left_yellow") or 0),
        "yR": float(s.get("right_yellow") or 0),
        "fL": None if s.get("left_fill") is None else float(s.get("left_fill")),
        "fR": None if s.get("right_fill") is None else float(s.get("right_fill")),
        "phase": s.get("mark_phase"),
        "lane": s.get("lane_phase"),
        "rem": s.get("mark_remaining_m"),
        "src": d.get("cmd_source"),
        "v": float(d.get("gps_speed_m_s") or 0),
        "locks": bool(d.get("locks_engaged")),
        "wL": None if s.get("left_wall_dist_m") is None else float(s.get("left_wall_dist_m")),
        "wR": None if s.get("right_wall_dist_m") is None else float(s.get("right_wall_dist_m")),
        "mCt": None if s.get("metric_ct") is None else float(s.get("metric_ct")),
        "metric": bool(s.get("metric_active")),
        "esrc": s.get("error_source") or "",
        "sim": d.get("sim_time_s"),
        "wall": time.time(),
    }


def main() -> int:
    time.sleep(3.5)
    s0 = snap()
    print(
        f"START x={s0['x']:.3f} y={s0['y']:.3f} ct={s0['ct']:.3f} "
        f"red={s0['red']:.3f} locks={s0['locks']} "
        f"wL={s0['wL']} wR={s0['wR']} mCt={s0['mCt']} src={s0['esrc']}"
    )
    req("/api/twin/command", "POST", {"lane_keep": True, "source": "s_lane_keep"})
    timeout_s = 900.0
    print("LANE KEEP ARMED  finish_x=", FINISH_X_M, "timeout_s=", int(timeout_s))
    t0 = time.time()
    seen_coast = False
    seen_brake = False
    seen_yellow = s0["yL"] >= 0.12 or s0["yR"] >= 0.12
    lost_since = None
    abort_lost = False
    last_print = -1
    max_abs_ct = abs(s0["ct"])
    metric_on = 0
    metric_off = 0
    n_samp = 0
    n_5 = 0
    n_10 = 0
    first_lobe_peak = 0.0
    crest_15 = None
    while time.time() - t0 < timeout_s:
        s = snap()
        t = time.time() - t0
        max_abs_ct = max(max_abs_ct, abs(s["ct"]))
        if 3.0 <= s["x"] <= 12.0:
            first_lobe_peak = max(first_lobe_peak, abs(s["ct"]))
        if 14.8 <= s["x"] <= 15.8:
            if crest_15 is None or abs(s["x"] - 15.3) < abs(crest_15["x"] - 15.3):
                crest_15 = dict(s)
        if s["x"] >= 1.0:
            n_samp += 1
            if abs(s["ct"]) <= 0.05:
                n_5 += 1
            if abs(s["ct"]) <= 0.10:
                n_10 += 1
        if s.get("metric"):
            metric_on += 1
        else:
            metric_off += 1
        near_finish = s["x"] >= FINISH_X_M - 3.5
        mark_owns = s["phase"] in ("coast", "brake") or near_finish
        if (
            s.get("lane") in ("lookout", "lost", "watch")
            and s["x"] > 2.0
            and not seen_brake
            and not mark_owns
        ):
            abort_lost = True
            tag = {
                "lookout": "LOOKOUT_ABORT",
                "lost": "FILL_LOST_ABORT",
                "watch": "WATCH_ABORT",
            }.get(str(s.get("lane")), "LOST_PAINT_ABORT")
            print(
                f"  {tag} t={t:.1f}s x={s['x']:.3f} y={s['y']:.3f} "
                f"ct={s['ct']:.3f} lane={s.get('lane')} "
                f"yL={s['yL']:.2f} yR={s['yR']:.2f}  SIM FINISHED"
            )
            break
        if s["phase"] == "coast" and not seen_coast:
            seen_coast = True
            print(
                f"  COAST t={t:.1f}s x={s['x']:.3f} y={s['y']:.3f} "
                f"ct={s['ct']:.3f} red={s['red']:.3f} rem={s['rem']}"
            )
        if (
            (s["src"] in ("abs", "abs_park") or s["phase"] == "brake")
            and not seen_brake
            and t > 2
            and s["x"] > FINISH_X_M - 3.0
        ):
            seen_brake = True
            print(
                f"  BRAKE t={t:.1f}s x={s['x']:.3f} y={s['y']:.3f} "
                f"ct={s['ct']:.3f} rem={s['rem']} src={s['src']}"
            )
        fill_known = s["fL"] is not None and s["fR"] is not None
        both_dark = (s["yL"] < 0.12 and s["yR"] < 0.12) or (
            fill_known
            and s["fL"] < 0.03
            and s["fR"] < 0.03
            and s["x"] > 2.0
        )
        if not both_dark:
            seen_yellow = True
            lost_since = None
        elif (
            seen_yellow
            and s["x"] > 2.0
            and s["red"] < 0.28
            and not seen_brake
            and not mark_owns
        ):
            if lost_since is None:
                lost_since = time.time()
            held = time.time() - lost_since
            stopped = s["v"] < 0.05
            if held >= 2.0 or (stopped and held >= 0.4):
                abort_lost = True
                print(
                    f"  LOST_PAINT_ABORT t={t:.1f}s x={s['x']:.3f} y={s['y']:.3f} "
                    f"ct={s['ct']:.3f} yL={s['yL']:.2f} yR={s['yR']:.2f} "
                    f"v={s['v']:.3f}  SIM FINISHED (both cameras dark)"
                )
                break
        if int(t) != last_print and int(t) % 10 == 0:
            last_print = int(t)
            fl = "None" if s["fL"] is None else f"{s['fL']:.3f}"
            fr = "None" if s["fR"] is None else f"{s['fR']:.3f}"
            print(
                f"  t={t:5.1f}s x={s['x']:.2f} y={s['y']:.2f} ct={s['ct']:+.3f} "
                f"yL={s['yL']:.2f} yR={s['yR']:.2f} fL={fl} fR={fr} "
                f"red={s['red']:.2f} msrc={s['esrc']} "
                f"phase={s['phase']} src={s['src']}"
            )
        if seen_brake and s["locks"] and s["v"] < 0.03 and s["x"] > FINISH_X_M - 3.0:
            print("  PARKED")
            break
        time.sleep(0.3)

    req("/api/twin/command", "POST", {"lane_keep": False, "drive_stop": True})
    time.sleep(1.2)
    end = snap()
    err = abs(end["x"] - FINISH_X_M)
    in_lane = max_abs_ct < 0.70
    on_line = err < 0.40 and abs(end["y"]) < 0.35
    print("==== RESULT ====")
    pct5 = 0.0 if n_samp <= 0 else 100.0 * n_5 / n_samp
    pct10 = 0.0 if n_samp <= 0 else 100.0 * n_10 / n_samp
    print(
        f"end x={end['x']:.3f} y={end['y']:.3f} |x-finish|={err:.3f} "
        f"ct={end['ct']:.3f} max|ct|={max_abs_ct:.3f} locks={end['locks']}"
    )
    print(
        f"share @5cm={pct5:.0f}% @10cm={pct10:.0f}% "
        f"({n_5}/{n_samp} , {n_10}/{n_samp})"
    )
    print(f"first_lobe_peak={first_lobe_peak:.3f}")
    if crest_15 is not None:
        print(
            f"crest_x15.3 x={crest_15['x']:.3f} y={crest_15['y']:.3f} "
            f"ct={crest_15['ct']:+.3f} yL={crest_15['yL']:.2f} "
            f"yR={crest_15['yR']:.2f} fL={crest_15['fL']} fR={crest_15['fR']}"
        )
    if s0.get("sim") is not None and end.get("sim") is not None:
        dsim = float(end["sim"]) - float(s0["sim"])
        dwall = float(end["wall"]) - float(s0["wall"])
        rt = None if dwall <= 1e-6 else dsim / dwall
        print(
            f"INTEGRATED dsim={dsim:.3f} dwall={dwall:.3f} "
            f"rt={rt if rt is None else f'{rt:.3f}'}"
        )
    metric_n = metric_on + metric_off
    metric_pct = 0.0 if metric_n <= 0 else 100.0 * metric_on / metric_n
    print(
        f"coast_seen={seen_coast} brake_seen={seen_brake} "
        f"lost_paint_abort={abort_lost} elapsed={time.time()-t0:.1f}s"
    )
    print(
        f"metric_active={metric_on}/{metric_n} ({metric_pct:.0f}%) "
        f"picture_fallback={metric_off}"
    )
    if abort_lost:
        print("LOST_PAINT_ABORT LEFT_LANE NOT_ON_LINE")
        return 1
    print("IN_LANE" if in_lane else "LEFT_LANE", "ON_LINE" if on_line else "NOT_ON_LINE")
    return 0 if in_lane and on_line else 1


if __name__ == "__main__":
    raise SystemExit(main())
