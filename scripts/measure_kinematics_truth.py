#!/usr/bin/env python3
"""
Kinematics truth test: known wheel speed × duration ≥ 5 s vs measured distance.

Physics assumptions (no-slip differential-drive, planar):
  v = ω · r                    wheel radius r = 0.08 m (butlerbot.wbt)
  With constant acceleration 0 → v_cruise over rise time t_r:
      s_rise  = ½ · v_cruise · t_r
      s_flat  = v_cruise · (T − t_r − t_f)   if coast/flat segment exists
      s_fall  = ½ · v_cruise · t_f           constant decel to stop
  Ideal constant-velocity (wrong if accel ignored):
      s_cv    = v_cmd · T

Compares:
  * Command model (ω_cmd · r)
  * Wheel-odometry integral Σ (ω_avg · r · Δt)   [encoder rates]
  * GPS / pose chord length and path length
  * Accel-aware closed form using measured rise/fall times

Requires dashboard + linked Webots twin.

Usage:
  python scripts/measure_kinematics_truth.py
  python scripts/measure_kinematics_truth.py --duration 6 --wheel-v 5.5
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request

# Must match butlerbot.wbt wheel cylinder radius / WHEEL_RADIUS_M in controller
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


def pose(st: dict) -> tuple[float, float, float]:
    feed = st.get("external_feed") or {}
    p = feed.get("pose") or {}
    return (
        float(p.get("x_m") or 0.0),
        float(p.get("y_m") or 0.0),
        float(p.get("heading_rad") or 0.0),
    )


def legs_w(st: dict) -> float:
    for ch in st.get("channels") or []:
        if ch.get("id") == "Legs":
            return float(ch.get("draw_w") or 0.0)
    return 0.0


def wait_park(base: str, timeout_s: float = 14.0) -> dict:
    for _ in range(3):
        cmd(base, {"drive_stop": True})
        time.sleep(0.3)
    deadline = time.time() + timeout_s
    last = twin(base)
    while time.time() < deadline:
        time.sleep(0.55)
        now = twin(base)
        x0, y0, _ = pose(last)
        x1, y1, _ = pose(now)
        dist = math.hypot(x1 - x0, y1 - y0)
        d = diag(now)
        hubs = abs(float(d.get("hub_left_rad_s") or 0)) + abs(
            float(d.get("hub_right_rad_s") or 0)
        )
        if dist < 0.035 and hubs < 0.25:
            return now
        cmd(base, {"drive_stop": True})
        last = now
    return twin(base)


def find_rise_time(samples: list[dict], v_cmd: float, frac: float = 0.90) -> float:
    """First time hub odometry speed reaches frac * v_cmd (seconds from t0)."""
    thr = frac * v_cmd
    for s in samples:
        if s["v_odo"] >= thr:
            return float(s["t"])
    return float(samples[-1]["t"]) if samples else 0.0


def find_fall_start(samples: list[dict], v_cmd: float, frac: float = 0.90) -> float:
    """Last time still above frac*v_cmd (approx start of deceleration)."""
    thr = frac * v_cmd
    last_t = 0.0
    for s in samples:
        if s["v_odo"] >= thr:
            last_t = float(s["t"])
    return last_t


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:5000")
    ap.add_argument("--duration", type=float, default=6.0, help="Commanded drive time (s), min 5")
    ap.add_argument("--wheel-v", type=float, default=5.5, help="Wheel cmd rad/s (known ω)")
    ap.add_argument("--sample-hz", type=float, default=10.0)
    ap.add_argument("--label", default="kinematics_truth_v1")
    ap.add_argument("--radius", type=float, default=WHEEL_RADIUS_M)
    args = ap.parse_args()

    duration = max(5.0, float(args.duration))
    omega_cmd = float(args.wheel_v)
    r = float(args.radius)
    v_cmd = omega_cmd * r  # no-slip command speed (m/s)
    base = args.url.rstrip("/")
    dt_s = 1.0 / max(args.sample_hz, 2.0)

    print("=" * 60)
    print(" KINEMATICS TRUTH TEST")
    print("=" * 60)
    print(f"  wheel radius r     = {r:.4f} m")
    print(f"  commanded ω        = {omega_cmd:.3f} rad/s")
    print(f"  commanded v=ω·r    = {v_cmd:.4f} m/s  ({v_cmd*3.6:.2f} km/h)")
    print(f"  duration T         = {duration:.2f} s")
    print(f"  naive s=v·T        = {v_cmd * duration:.4f} m  (ignores accel)")
    print()

    st = twin(base)
    if not (st.get("bridge") or {}).get("external_active"):
        print("FAIL: Webots twin not linked")
        return 1

    print("Park + holdoff...")
    wait_park(base)
    time.sleep(3.5)  # park_holdoff so drive is accepted

    st0 = twin(base)
    x0, y0, h0 = pose(st0)
    batt0 = float((st0.get("robot") or {}).get("main_battery_pct") or 0)
    print(f"  start pose x={x0:.4f} y={y0:.4f} batt={batt0:.2f}%")

    print(f"Drive ω={omega_cmd} for {duration}s...")
    cmd(
        base,
        {
            "drive": {
                "left": omega_cmd,
                "right": omega_cmd,
                "duration_s": duration + 0.8,
            }
        },
    )
    time.sleep(0.3)
    tel = twin(base).get("teleop") or {}
    print(f"  teleop active={tel.get('active')} L={tel.get('left_v')} R={tel.get('right_v')}")

    t0 = time.time()
    samples: list[dict] = []
    energy_j = 0.0
    peak_legs = 0.0

    while time.time() - t0 < duration:
        st = twin(base)
        t = time.time() - t0
        x, y, h = pose(st)
        d = diag(st)
        wl = abs(float(d.get("hub_left_rad_s") or 0.0))
        wr = abs(float(d.get("hub_right_rad_s") or 0.0))
        w_avg = 0.5 * (wl + wr)
        v_odo = w_avg * r
        gps = float(d.get("gps_speed_m_s") or 0.0)
        lg = legs_w(st)
        peak_legs = max(peak_legs, lg)
        # total draw for energy (optional)
        tw = float((st.get("power") or {}).get("total_draw_w") or 0.0)
        energy_j += tw * dt_s
        samples.append(
            {
                "t": round(t, 4),
                "x": x,
                "y": y,
                "heading": h,
                "omega_l": wl,
                "omega_r": wr,
                "v_odo": v_odo,
                "v_gps": gps,
                "legs_w": lg,
            }
        )
        if len(samples) % 5 == 1:
            print(
                f"  t={t:5.2f}s  v_odo={v_odo:.3f}  v_gps={gps:.3f}  "
                f"ω={w_avg:.2f}  legs={lg:.1f}W  pose=({x:.3f},{y:.3f})"
            )
        time.sleep(dt_s)

    print("Stop + park...")
    st1 = wait_park(base)
    x1, y1, h1 = pose(st1)
    batt1 = float((st1.get("robot") or {}).get("main_battery_pct") or 0)
    w_end = (diag(st1).get("wheels") or {})
    locked = bool(w_end.get("both_locked"))

    # --- Path integrals ---
    path_gps = 0.0
    path_odo = 0.0
    for i in range(1, len(samples)):
        a, b = samples[i - 1], samples[i]
        dt = max(1e-6, b["t"] - a["t"])
        path_gps += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        path_odo += 0.5 * (a["v_odo"] + b["v_odo"]) * dt  # trapezoid

    chord = math.hypot(x1 - x0, y1 - y0)
    T_meas = samples[-1]["t"] if samples else duration

    # Cruise stats (middle 50% of run to avoid edges)
    mid = [s for s in samples if 0.25 * T_meas <= s["t"] <= 0.85 * T_meas]
    if not mid:
        mid = samples
    v_odo_cruise = sum(s["v_odo"] for s in mid) / len(mid)
    v_gps_cruise = sum(s["v_gps"] for s in mid) / len(mid)
    omega_cruise = sum(0.5 * (s["omega_l"] + s["omega_r"]) for s in mid) / len(mid)

    # Accel-aware closed form using measured rise/fall from odometry speed
    t_rise = find_rise_time(samples, v_cmd, 0.90)
    t_high = find_fall_start(samples, v_cmd, 0.90)
    # If never dropped, fall starts at end
    t_fall = max(0.0, T_meas - t_high)
    # Constant-accel rise: s = ½ v t_r ; flat: v*(t_high - t_rise); fall: ½ v t_f
    # Use measured cruise speed for honesty
    v_c = v_odo_cruise if v_odo_cruise > 0.05 else v_cmd
    t_flat = max(0.0, t_high - t_rise)
    s_accel_model = 0.5 * v_c * t_rise + v_c * t_flat + 0.5 * v_c * t_fall
    s_const_v = v_cmd * T_meas
    s_const_v_meas = v_c * T_meas

    # Slip / consistency ratios
    def pct_err(meas: float, pred: float) -> float:
        if abs(pred) < 1e-9:
            return float("nan")
        return 100.0 * (meas - pred) / pred

    print()
    print("=" * 60)
    print(" RESULTS — is the twin truthful?")
    print("=" * 60)
    print(f"  samples            = {len(samples)}  T_meas={T_meas:.3f}s")
    print(f"  commanded v=ωr     = {v_cmd:.4f} m/s")
    print(f"  measured ω cruise  = {omega_cruise:.3f} rad/s  (cmd {omega_cmd:.3f})")
    print(f"  measured v_odo mid = {v_odo_cruise:.4f} m/s")
    print(f"  measured v_gps mid = {v_gps_cruise:.4f} m/s")
    print()
    print("  --- distances ---")
    print(f"  GPS chord (start→end)     = {chord:.4f} m")
    print(f"  GPS path length (∫ds)     = {path_gps:.4f} m")
    print(f"  Wheel odometry path ∫ωr   = {path_odo:.4f} m")
    print(f"  Model s=v_cmd·T (no accel)= {s_const_v:.4f} m")
    print(f"  Model s=v_cruise·T        = {s_const_v_meas:.4f} m")
    print(
        f"  Model accel-aware         = {s_accel_model:.4f} m"
        f"  (t_rise={t_rise:.2f}s flat={t_flat:.2f}s t_fall≈{t_fall:.2f}s)"
    )
    print()
    print("  --- agreement ---")
    print(f"  odo vs v_cmd·T            err = {pct_err(path_odo, s_const_v):+.1f}%")
    print(f"  odo vs accel model        err = {pct_err(path_odo, s_accel_model):+.1f}%")
    print(f"  GPS path vs odo path      err = {pct_err(path_gps, path_odo):+.1f}%")
    print(f"  GPS chord vs odo path     err = {pct_err(chord, path_odo):+.1f}%")
    print(f"  ω cruise vs ω_cmd         err = {pct_err(omega_cruise, omega_cmd):+.1f}%")
    print(f"  v_odo vs v_cmd            err = {pct_err(v_odo_cruise, v_cmd):+.1f}%")
    print()
    print(f"  wheels locked after stop  = {locked}")
    print(f"  battery %                 = {batt0:.2f} → {batt1:.2f}")
    print(f"  peak Legs W               = {peak_legs:.1f}")
    print(f"  energy (total draw) Wh    = {energy_j/3600.0:.6f}")

    # Truth verdict (loose engineering bands for a first twin)
    ok_speed = abs(pct_err(omega_cruise, omega_cmd)) < 15.0
    ok_odo_model = abs(pct_err(path_odo, s_accel_model)) < 20.0
    ok_gps_odo = abs(pct_err(path_gps, path_odo)) < 35.0  # GPS noisier / slip
    ok_moved = path_odo > 0.5 * v_cmd * duration * 0.5  # actually drove

    print()
    print("  VERDICT:")
    print(f"    hub tracks command ω     : {'PASS' if ok_speed else 'FAIL'}")
    print(f"    odometry ≈ accel model   : {'PASS' if ok_odo_model else 'FAIL'}")
    print(f"    GPS path ≈ wheel odo     : {'PASS' if ok_gps_odo else 'FAIL'} (slip/noise band 35%)")
    print(f"    motion occurred          : {'PASS' if ok_moved else 'FAIL'}")
    truth = ok_speed and ok_odo_model and ok_moved
    print(f"    overall twin kinematics  : {'PLAUSIBLE' if truth else 'SUSPECT'}")
    print("=" * 60)

    row = {
        "label": args.label,
        "duration_s": round(T_meas, 3),
        "distance_m": round(path_gps, 4),
        "energy_wh": round(energy_j / 3600.0, 6),
        "avg_legs_w": round(sum(s["legs_w"] for s in samples) / max(len(samples), 1), 2),
        "peak_legs_w": round(peak_legs, 2),
        "avg_total_w": None,
        "battery_start_pct": batt0,
        "battery_end_pct": batt1,
        "wheels_locked_after": locked,
        "hub_left_max_abs": max(s["omega_l"] for s in samples) if samples else 0,
        "hub_right_max_abs": max(s["omega_r"] for s in samples) if samples else 0,
        "rot_left_since_stop_deg": (w_end.get("left") or {}).get("rot_since_stop_deg")
        if isinstance(w_end, dict)
        else None,
        "rot_right_since_stop_deg": (w_end.get("right") or {}).get("rot_since_stop_deg")
        if isinstance(w_end, dict)
        else None,
        "details": {
            "test": "kinematics_truth",
            "r_m": r,
            "omega_cmd": omega_cmd,
            "v_cmd": v_cmd,
            "v_odo_cruise": v_odo_cruise,
            "v_gps_cruise": v_gps_cruise,
            "omega_cruise": omega_cruise,
            "path_gps_m": path_gps,
            "path_odo_m": path_odo,
            "chord_m": chord,
            "s_const_v": s_const_v,
            "s_accel_model": s_accel_model,
            "t_rise_s": t_rise,
            "t_flat_s": t_flat,
            "t_fall_s": t_fall,
            "err_odo_vs_accel_pct": pct_err(path_odo, s_accel_model),
            "err_gps_vs_odo_pct": pct_err(path_gps, path_odo),
            "err_omega_pct": pct_err(omega_cruise, omega_cmd),
            "verdict": "plausible" if truth else "suspect",
            "samples_n": len(samples),
            "samples_head": samples[:8],
            "samples_tail": samples[-4:],
        },
    }
    try:
        res = _req(f"{base}/api/measurements", method="POST", body=row)
        print(f"  DB measurement id = {res.get('id')}")
    except urllib.error.URLError as e:
        print(f"  DB write skipped/failed: {e}")

    return 0 if truth else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)
