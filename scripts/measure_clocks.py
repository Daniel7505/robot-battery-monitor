#!/usr/bin/env python3
"""15 s straight: wall vs sim time, v_gps vs v_odo vs v_cmd. No S."""

from __future__ import annotations

import json
import time
import urllib.request

BASE = "http://127.0.0.1:5000"
WHEEL_V = 5.5
HOLD_S = 16.0


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
    d = st.get("control_diag") or {}
    return {
        "wall": time.time(),
        "sim": d.get("sim_time_s"),
        "x": float(p.get("x_m") or 0),
        "y": float(p.get("y_m") or 0),
        "v_gps": d.get("gps_speed_m_s"),
        "v_odo": d.get("odo_speed_m_s"),
        "v_cmd": d.get("cmd_speed_m_s"),
        "src": d.get("cmd_source"),
        "wL": d.get("hub_left_rad_s"),
        "wR": d.get("hub_right_rad_s"),
    }


def fmt(v) -> str:
    if v is None:
        return "   None"
    return f"{float(v):7.3f}"


def main() -> int:
    s0 = snap()
    print(
        f"PRE  x={s0['x']:.3f} y={s0['y']:.3f} sim={s0['sim']} "
        f"src={s0['src']} (need sim_time_s — reload Webots if None)"
    )
    req(
        "/api/twin/command",
        "POST",
        {
            "drive": {"left": WHEEL_V, "right": WHEEL_V, "duration_s": HOLD_S},
            "source": "clocks",
        },
    )
    print("ARMED  ω=5.5 both hubs  hold=16s")
    print(
        f"{'wall':>7} {'sim':>7} {'dsim':>6} {'dwall':>6} {'rt':>5} "
        f"{'x':>6} {'v_gps':>7} {'v_odo':>7} {'v_cmd':>7} {'wL':>6} {'wR':>6} src"
    )
    prev = None
    t_end = time.time() + 15.0
    while time.time() < t_end:
        s = snap()
        dsim = dwall = rt = None
        if prev is not None and s["sim"] is not None and prev["sim"] is not None:
            dsim = float(s["sim"]) - float(prev["sim"])
            dwall = float(s["wall"]) - float(prev["wall"])
            if dwall > 1e-6:
                rt = dsim / dwall
        print(
            f"{s['wall']-s0['wall']:7.2f} {fmt(s['sim'])} {fmt(dsim)} {fmt(dwall)} "
            f"{fmt(rt)} {s['x']:6.2f} {fmt(s['v_gps'])} {fmt(s['v_odo'])} "
            f"{fmt(s['v_cmd'])} {fmt(s['wL'])} {fmt(s['wR'])} {s['src']}"
        )
        prev = s
        time.sleep(1.0)
    req("/api/twin/command", "POST", {"drive_stop": True, "lane_keep": False, "source": "clocks"})
    time.sleep(0.8)
    end = snap()
    print(
        f"POST x={end['x']:.3f} y={end['y']:.3f} sim={end['sim']} "
        f"v_gps={end['v_gps']} v_odo={end['v_odo']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
