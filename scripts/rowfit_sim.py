#!/usr/bin/env python3
"""Offline closed-loop check of the rowfit lane keep (no Webots).

Kinematic diff-drive robot, 8 ms ticks, both nadir cameras rendered from
the SAME camera model every 320 ms (src/lane_vision.render_lane_bgra).
Camera poses come from webots/worlds/butlerbot.wbt (parse_wbt_cameras, the
same fallback the controller uses; ``--constants`` uses NADIR_*_POSE) and the
look-ahead window is derived from their coverage, as in the controller.
LaneTracker + RowfitController in the loop. The course geometry is only
used to paint the synthetic images and to score cross-track error.

    python scripts/rowfit_sim.py            # both courses
    python scripts/rowfit_sim.py turn90 --csv out.csv

Not a substitute for Webots (no tyre slip, lighting, motion blur, body
occlusion), but it catches sign errors, instability and speed logic.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from src.lane_keep import WHEEL_RADIUS_M  # noqa: E402
from src.lane_vision import (  # noqa: E402
    NADIR_CAMS,
    CameraModel,
    LaneTracker,
    camera_coverage,
    parse_wbt_cameras,
    lane_polyline_robot,
    offset_polyline,
    render_lane_bgra,
)
from src.rowfit_control import TRACK_M, RowfitController, lookahead_bounds_from_coverage  # noqa: E402


def sim_cameras(use_constants: bool = False, wbt_path=None) -> dict:
    """Nadir camera models from the world file (fallback: constants)."""
    if not use_constants:
        try:
            w = parse_wbt_cameras(wbt_path)
            cams = {
                n: CameraModel(n, w[n]["translation"], w[n]["rotation"], w[n]["width"], w[n]["height"], w[n]["fov"])
                for n in NADIR_CAMS
            }
            return cams
        except (OSError, KeyError):
            pass
    return dict(NADIR_CAMS)


def path_from_segments(segs, start=(-2.0, 0.0), h0=0.0, ds=0.04):
    """segs: [(length_m, curvature_1pm), ...] -> centre polyline (world)."""
    x, y, h = start[0], start[1], h0
    pts = [(x, y)]
    for L, k in segs:
        n = max(1, int(round(L / ds)))
        s = L / n
        for _ in range(n):
            h += k * s / 2.0
            x += s * math.cos(h)
            y += s * math.sin(h)
            h += k * s / 2.0
            pts.append((x, y))
    return pts


def course(name: str):
    if name == "turn90":
        return path_from_segments([(4.0, 0.0), (1.0 * math.pi / 2, 1.0), (4.0, 0.0)])
    if name == "turn90r08":
        return path_from_segments([(4.0, 0.0), (0.8 * math.pi / 2, 1.25), (4.0, 0.0)])
    if name == "s":
        from s_track import FINISH_X_M, centerline

        pts = [(-2.0 + 0.04 * i, 0.0) for i in range(50)]
        x = 0.0
        while x <= FINISH_X_M + 2.0:
            pts.append((x, centerline(x)[0]))
            x += 0.04
        return pts
    raise SystemExit(f"unknown course {name}")


def _dist_and_index(p, poly, hint=0):
    best, bi = 1e9, hint
    lo, hi = max(0, hint - 40), min(len(poly) - 1, hint + 120)
    for i in range(lo, hi):
        (x0, y0), (x1, y1) = poly[i], poly[i + 1]
        vx, vy = x1 - x0, y1 - y0
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - x0) * vx + (p[1] - y0) * vy) / L2))
        d = math.hypot(p[0] - x0 - t * vx, p[1] - y0 - t * vy)
        if d < best:
            best, bi = d, i
    return best, bi


def _path_curvature(poly):
    """|curvature| at each polyline vertex (heading change / step)."""
    out = [0.0] * len(poly)
    for i in range(1, len(poly) - 1):
        (x0, y0), (x1, y1), (x2, y2) = poly[i - 1], poly[i], poly[i + 1]
        h1 = math.atan2(y1 - y0, x1 - x0)
        h2 = math.atan2(y2 - y1, x2 - x1)
        dh = math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))
        ds = 0.5 * (math.hypot(x1 - x0, y1 - y0) + math.hypot(x2 - x1, y2 - y1)) or 1.0
        out[i] = abs(dh) / ds
    return out


def run(name: str, *, half_w=0.65, start_offset=0.0, start_yaw=0.0, max_s=90.0, csv=None, latency_frames=0,
        cams=None):
    centre = course(name)
    curv = _path_curvature(centre)
    left = offset_polyline(centre, half_w)
    right = offset_polyline(centre, -half_w)
    x, y, yaw = 0.0, float(start_offset), float(start_yaw)
    cams = dict(cams or sim_cameras())
    tracker = LaneTracker(cams)
    ld_min, ld_max = lookahead_bounds_from_coverage([camera_coverage(c) for c in cams.values()])
    ctl = RowfitController(ld_min=ld_min, ld_max=ld_max)
    bend_v = []
    dt, frame_every = 0.008, 40
    tick, idx = 0, 0
    max_ct, sum_ct, n_ct = 0.0, 0.0, 0
    speeds = []
    rows = []
    pending = []
    last_frame_pose = (x, y, yaw)
    end = centre[-1]
    done = False
    while tick * dt < max_s:
        new = tick % frame_every == 0
        est = None
        if new:
            near = [p for p in centre if abs(p[0] - x) < 4.0 and abs(p[1] - y) < 4.0]
            lines = []
            for poly in (left, right):
                seg = [p for p in poly if math.hypot(p[0] - x, p[1] - y) < 3.5]
                if len(seg) > 1:
                    lines.append(lane_polyline_robot(seg, (x, y), yaw))
            imgs = {n: (render_lane_bgra(c, lines), c.width, c.height) for n, c in cams.items()}
            px, py, pyaw = last_frame_pose
            dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
            dx = math.hypot(x - px, y - py)
            last_frame_pose = (x, y, yaw)
            est = tracker.process(imgs, dx_m=dx, dyaw_rad=dyaw)
            pending.append(est)
            est = pending.pop(0) if len(pending) > latency_frames else None
            del near
        cmd = ctl.step(est, new_frame=new and est is not None, dt=dt)
        if cmd["brake"]:
            break
        wl, wr = cmd["left"], cmd["right"]
        v = 0.5 * (wl + wr) * WHEEL_RADIUS_M
        w = (wr - wl) * WHEEL_RADIUS_M / TRACK_M
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        ct, idx = _dist_and_index((x, y), centre, idx)
        max_ct = max(max_ct, ct)
        sum_ct += ct
        n_ct += 1
        speeds.append(v)
        if curv[min(idx, len(curv) - 1)] > 0.3:
            bend_v.append(v)
        if new and csv is not None:
            rows.append((round(tick * dt, 3), round(x, 3), round(y, 3), round(ct, 3),
                         None if est is None else round(est.offset_m, 3),
                         None if est is None else round(est.curvature_1pm, 3),
                         cmd["steer"], cmd.get("target_speed"), round(v, 3)))
        if idx >= len(centre) - 30 or math.hypot(x - end[0], y - end[1]) < 0.5:
            done = True
            break
        tick += 1
    if csv is not None:
        with open(csv, "w", encoding="ascii") as fh:
            fh.write("t_s,x,y,ct_m,offset_m,curv,steer,target_speed,v\n")
            for r in rows:
                fh.write(",".join(str(v) for v in r) + "\n")
    return {
        "course": name,
        "finished": done,
        "time_s": round(tick * dt, 2),
        "max_ct_m": round(max_ct, 3),
        "mean_ct_m": round(sum_ct / max(1, n_ct), 3),
        "min_v": round(min(speeds), 3) if speeds else 0.0,
        "max_v": round(max(speeds), 3) if speeds else 0.0,
        "bend_min_v": round(min(bend_v), 3) if bend_v else None,
        "bend_mean_v": round(sum(bend_v) / len(bend_v), 3) if bend_v else None,
        "ld_window_m": (round(ld_min, 2), round(ld_max, 2)),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("courses", nargs="*", default=["s", "turn90"])
    ap.add_argument("--csv")
    ap.add_argument("--latency", type=int, default=0, help="frames of extra camera latency")
    ap.add_argument("--constants", action="store_true", help="use NADIR_*_POSE instead of the .wbt")
    ap.add_argument("--wbt", help="world file to read the camera poses from")
    a = ap.parse_args(argv)
    cams = sim_cameras(a.constants, a.wbt)
    for n, c in cams.items():
        print(f"camera {n}: t={c.translation} rot={c.rotation} fov={c.fov_rad}")
    for c in a.courses:
        print(run(c, csv=a.csv, latency_frames=a.latency, cams=cams))
    return 0


if __name__ == "__main__":
    sys.exit(main())
