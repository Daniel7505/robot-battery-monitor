"""Short headless Webots checks for the apartment mapper.

One command runs every scenario. Each one is a copy of the apartment
world with a fixed spawn, 30–90 s of simulation, and a pass/fail read
from the pose logger. The mapper is not told these poses. A body-frame
drive name (``RBM_DRIVE``) is the only extra signal, and it carries no
coordinates.

World copies are written next to ``apartment.wbt`` because the proto
path is relative, and deleted when the run ends. Logs stay under /tmp.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from eval.apartment_gt_grid import load_solids
from src.mapping.world_guard import require_skid_world, swap_caster_instance


WORLD = ROOT / "webots" / "worlds" / "apartment.wbt"
WORLDS = ROOT / "webots" / "worlds"
WEBOTS = ROOT / "webots"

# Door G is the shared-wall opening into the hall. Scoring only.
_DOOR_X = 3.66
_EAST_WALL_X = 8.4328
_TAIL_WALL_X = 3.7592
_NOSE_M = 0.17
_TAIL_M = 0.16


@dataclass(frozen=True)
class Scenario:
    name: str
    seconds: float
    x: float
    y: float
    yaw: float
    drive: str
    port: int


SCENARIOS = (
    Scenario("door", 90.0, 4.40, 1.778, math.pi, "", 1241),
    Scenario("tail", 45.0, _TAIL_WALL_X + _TAIL_M + 0.05, 4.50, 0.0, "", 1242),
    Scenario("loop", 50.0, 6.1, 4.7, 1.5708, "square", 1243),
    Scenario("bubble", 30.0, 7.20, 3.60, 0.0, "plunge", 1244),
    Scenario("backup", 30.0, 7.40, 3.60, math.pi, "backup", 1245),
    # Hall centre, facing the bath door. The bedroom is through that
    # door and the walkway at the north end of the bath, or back out
    # and around to the bedroom door. The long way reaches the
    # threshold late, so the cap leaves time to roll up the carpet
    # step. Scoring only; the mapper is not given this pose.
    Scenario("bedroom", 150.0, 2.794, 1.778, math.pi, "", 1246),
)


def _patch_world(text: str, x: float, y: float, yaw: float) -> str:
    match = re.search(r"ButlerBot\s*\{", text)
    if match is None:
        raise RuntimeError("ButlerBot instance not found")
    start = match.end()
    trans = re.search(
        r"translation\s+[-+0-9.eE]+\s+[-+0-9.eE]+\s+[-+0-9.eE]+",
        text[start:],
    )
    if trans is None:
        raise RuntimeError("ButlerBot translation not found")
    prefix = text[: start + trans.start()] + f"translation {x:.4f} {y:.4f} 0"
    tail = text[start + trans.end() :]
    rot = re.search(
        r"rotation\s+[-+0-9.eE]+\s+[-+0-9.eE]+\s+[-+0-9.eE]+\s+[-+0-9.eE]+",
        tail,
    )
    if rot is None:
        raise RuntimeError("ButlerBot rotation not found")
    tail = tail[: rot.start()] + f"rotation 0 0 1 {yaw:.6f}" + tail[rot.end() :]
    return prefix + tail


def _read_csv(path: Path) -> list[dict[str, float]]:
    if not path.is_file():
        return []
    with path.open(encoding="utf-8", newline="") as handle:
        rows = []
        for row in csv.DictReader(handle):
            parsed: dict[str, float] = {}
            for key, value in row.items():
                if key is None:
                    continue
                try:
                    parsed[key] = float(value)
                except (TypeError, ValueError):
                    continue
            if parsed:
                rows.append(parsed)
    return rows


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        out.append(out[-1] + _wrap(value - out[-1]))
    return out


def _world_xy(odom_x: float, odom_y: float, spawn_x: float, spawn_y: float, yaw0: float) -> tuple[float, float]:
    c, s = math.cos(yaw0), math.sin(yaw0)
    return spawn_x + c * odom_x - s * odom_y, spawn_y + s * odom_x + c * odom_y


def _nearest(rows: list[dict[str, float]], t: float) -> dict[str, float]:
    best = rows[0]
    best_dt = abs(best["t_s"] - t)
    # Times are sorted. A binary search would also do; the files are short.
    lo, hi = 0, len(rows) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        dt = rows[mid]["t_s"] - t
        if abs(dt) < best_dt:
            best = rows[mid]
            best_dt = abs(dt)
        if dt < 0:
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def _episodes(walls: list[dict[str, float]]) -> list[tuple[float, float, float]]:
    """(start, end, duration) for contact bursts. A 0.25 s gap splits them."""
    times = [
        row["t_s"]
        for row in walls
        if row.get("left_wall", 0.0) > 0 or row.get("right_wall", 0.0) > 0 or row.get("caster_wall", 0.0) > 0
    ]
    if not times:
        return []
    bursts: list[tuple[float, float, float]] = []
    start = prev = times[0]
    for t in times[1:]:
        if t - prev > 0.25:
            bursts.append((start, prev, prev - start))
            start = t
        prev = t
    bursts.append((start, prev, prev - start))
    return bursts


def _inside(solids, name: str, x: float, y: float, inset: float = 0.05) -> bool:
    for solid in solids:
        if solid.name != name:
            continue
        if solid.xmin + inset < x < solid.xmax - inset and solid.ymin + inset < y < solid.ymax - inset:
            return True
    return False


def _stuck_walk(gt: list[dict[str, float]], pose: list[dict[str, float]], yaw0: float, sx: float, sy: float) -> float:
    """Net odometry drift while the true body stays inside a 3 cm box.

    A pin that snaps the pose back to an anchor makes a sawtooth. The
    path length of that sawtooth is not travel. The chord from the start
    of the still interval to the end is.
    """
    if len(pose) < 2 or not gt:
        return 0.0
    worst = 0.0
    # Pose logs are 8 ms. A stride keeps this on the order of the run, not its square.
    step = 5
    for right in range(0, len(pose), step):
        here = _nearest(gt, pose[right]["t_s"])
        anchor = (here["x_m"], here["y_m"])
        k = right
        while k - step >= 0 and pose[right]["t_s"] - pose[k - step]["t_s"] <= 6.0:
            earlier = _nearest(gt, pose[k - step]["t_s"])
            if math.hypot(earlier["x_m"] - anchor[0], earlier["y_m"] - anchor[1]) > 0.03:
                break
            k -= step
        if pose[right]["t_s"] - pose[k]["t_s"] < 0.4:
            continue
        start_o = _world_xy(pose[k]["x_m"], pose[k]["y_m"], sx, sy, yaw0)
        end_o = _world_xy(pose[right]["x_m"], pose[right]["y_m"], sx, sy, yaw0)
        start_g = _nearest(gt, pose[k]["t_s"])
        net_o = math.hypot(end_o[0] - start_o[0], end_o[1] - start_o[1])
        net_g = math.hypot(here["x_m"] - start_g["x_m"], here["y_m"] - start_g["y_m"])
        worst = max(worst, net_o - net_g)
    return worst


def _end_errors(gt, pose, yaw_rows, yaw0, sx, sy) -> tuple[float, float, float]:
    """Position error (m), heading error (deg), and true path length (m)."""
    if not gt or not pose:
        return float("inf"), float("inf"), 0.0
    last = pose[-1]
    true = _nearest(gt, last["t_s"])
    wx, wy = _world_xy(last["x_m"], last["y_m"], sx, sy, yaw0)
    pos = math.hypot(wx - true["x_m"], wy - true["y_m"])
    gt_yaw = _unwrap([row["yaw_rad"] for row in gt])
    # Heading of the fused yaw against the true yaw, both relative to spawn.
    if yaw_rows:
        fused = yaw_rows[-1].get("fused_yaw", pose[-1]["yaw_rad"])
        # Match the yaw-source row to the last pose time when possible.
        fused = _nearest(yaw_rows, last["t_s"]).get("fused_yaw", fused)
    else:
        fused = _unwrap([row["yaw_rad"] for row in pose])[-1]
    true_index = min(range(len(gt)), key=lambda i: abs(gt[i]["t_s"] - last["t_s"]))
    heading = math.degrees(_wrap(fused - (gt_yaw[true_index] - yaw0)))
    length = 0.0
    for a, b in zip(gt, gt[1:]):
        length += math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"])
    return pos, heading, length


def _peak_speed(gt: list[dict[str, float]]) -> float:
    peak = 0.0
    for a, b in zip(gt, gt[1:]):
        dt = b["t_s"] - a["t_s"]
        if dt <= 1e-4:
            continue
        peak = max(peak, math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]) / dt)
    return peak


def judge(name: str, folder: Path, scenario: Scenario) -> tuple[bool, str]:
    gt = _read_csv(folder / "gt_pose_log.csv")
    pose = _read_csv(folder / "pose_log.csv")
    yaw_rows = _read_csv(folder / "yaw_sources.csv")
    walls = _read_csv(folder / "wall_contacts.csv")
    done = (folder / "mapping_done").is_file()
    if len(gt) < 10 or len(pose) < 10:
        return False, "pose logs missing or short"
    solids = load_solids()
    bursts = _episodes(walls)
    longest = max((item[2] for item in bursts), default=0.0)
    pos, heading, length = _end_errors(gt, pose, yaw_rows, scenario.yaw, scenario.x, scenario.y)
    final = gt[-1]
    min_x = min(row["x_m"] for row in gt)
    max_x = max(row["x_m"] for row in gt)
    speed = _peak_speed(gt)
    moved = math.hypot(final["x_m"] - scenario.x, final["y_m"] - scenario.y)
    visual = yaw_rows[-1].get("visual_applied_yaw", 0.0) if yaw_rows else 0.0
    wall_yaw = yaw_rows[-1].get("wall_applied_yaw", 0.0) if yaw_rows else 0.0
    common = (
        f"pos={pos:.3f}m head={heading:+.2f}deg path={length:.2f}m "
        f"press={longest:.2f}s contacts={len(walls)} speed={speed:.2f} "
        f"final=({final['x_m']:.2f},{final['y_m']:.2f}) visual_yaw={visual:+.3f} "
        f"wall_yaw={wall_yaw:+.3f} done={int(done)}"
    )
    if name == "door":
        entered = any(
            _inside(solids, "APT_FLOOR_HALL", row["x_m"], row["y_m"])
            or _inside(solids, "APT_FLOOR_BEDROOM", row["x_m"], row["y_m"])
            for row in gt
        )
        stayed = final["x_m"] < _DOOR_X
        crossed = min_x < 3.40
        ok = entered and stayed and crossed
        return ok, common + f" min_x={min_x:.2f} entered={int(entered)} stayed={int(stayed)}"
    if name == "tail":
        walk = _stuck_walk(gt, pose, scenario.yaw, scenario.x, scenario.y)
        ok = longest <= 2.0 and walk <= 0.05 and moved >= 0.15
        return ok, common + f" walk={walk:.3f}m moved={moved:.2f}m max_x={max_x:.2f}"
    if name == "loop":
        ok = done and abs(heading) < 2.0 and pos < 0.10 and length > 2.5
        return ok, common
    if name == "bubble":
        nose = max(row["x_m"] + _NOSE_M * math.cos(row["yaw_rad"]) for row in gt)
        air = _EAST_WALL_X - nose
        end_speed = 0.0
        if len(gt) >= 2:
            dt = gt[-1]["t_s"] - gt[-2]["t_s"]
            if dt > 1e-4:
                end_speed = math.hypot(gt[-1]["x_m"] - gt[-2]["x_m"], gt[-1]["y_m"] - gt[-2]["y_m"]) / dt
        ok = len(walls) == 0 and speed >= 0.22 and end_speed < 0.08 and 0.01 < air < 0.40
        return ok, common + f" air={air:.3f}m end_speed={end_speed:.3f}"
    if name == "backup":
        tail_x = max(row["x_m"] - math.cos(row["yaw_rad"]) * _TAIL_M for row in gt)
        air = _EAST_WALL_X - tail_x
        end_speed = 0.0
        if len(gt) >= 2:
            dt = gt[-1]["t_s"] - gt[-2]["t_s"]
            if dt > 1e-4:
                end_speed = math.hypot(gt[-1]["x_m"] - gt[-2]["x_m"], gt[-1]["y_m"] - gt[-2]["y_m"]) / dt
        backed = final["x_m"] > scenario.x + 0.15
        ok = len(walls) == 0 and speed >= 0.10 and end_speed < 0.08 and 0.01 < air < 0.45 and backed
        return ok, common + f" tail_air={air:.3f}m end_speed={end_speed:.3f} backed={int(backed)}"
    if name == "bedroom":
        inside = [
            row for row in gt if _inside(solids, "APT_FLOOR_BEDROOM", row["x_m"], row["y_m"], inset=0.12)
        ]
        ok = len(inside) >= 50 and longest <= 2.0
        return ok, common + f" bedroom_s={len(inside) * 0.008:.2f}"
    return False, "unknown scenario " + name


def _run_one(scenario: Scenario, folder: Path, world_path: Path) -> subprocess.Popen:
    folder.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    env["RBM_MAP_MAX_S"] = str(scenario.seconds)
    env["RBM_MAP_DIR"] = str(folder)
    env.pop("RBM_STRAIGHT_S", None)
    if scenario.drive:
        env["RBM_DRIVE"] = scenario.drive
    else:
        env.pop("RBM_DRIVE", None)
    log_path = folder / "webots.log"
    handle = log_path.open("w", encoding="utf-8")
    cmd = [
        "xvfb-run",
        "-a",
        "webots",
        "--mode=fast",
        "--no-rendering",
        "--batch",
        "--stdout",
        "--stderr",
        f"--port={scenario.port}",
        str(world_path.relative_to(WEBOTS)),
    ]
    return subprocess.Popen(
        cmd,
        cwd=str(WEBOTS),
        env=env,
        stdout=handle,
        stderr=subprocess.STDOUT,
    )


def run_scenarios(names: list[str] | None = None) -> list[tuple[str, bool, str]]:
    chosen = [item for item in SCENARIOS if not names or item.name in names]
    if not chosen:
        raise SystemExit(f"no scenarios named {names}; choose from {[s.name for s in SCENARIOS]}")
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    root = Path("/tmp/map_scenarios") / stamp
    root.mkdir(parents=True, exist_ok=True)
    source = WORLD.read_text(encoding="utf-8")
    worlds: list[Path] = []
    procs: list[tuple[Scenario, subprocess.Popen, Path]] = []
    results: list[tuple[str, bool, str]] = []
    try:
        for scenario in chosen:
            world_path = WORLDS / f"_scenario_{scenario.name}.wbt"
            text = swap_caster_instance(_patch_world(source, scenario.x, scenario.y, scenario.yaw))
            require_skid_world(text, world_path.name)
            world_path.write_text(text, encoding="utf-8")
            worlds.append(world_path)
            folder = root / scenario.name
            procs.append((scenario, _run_one(scenario, folder, world_path), folder))
        deadline = time.time() + max(scenario.seconds for scenario in chosen) * 3.0 + 90.0
        pending = list(procs)
        while pending and time.time() < deadline:
            still = []
            for scenario, proc, folder in pending:
                code = proc.poll()
                if code is None:
                    still.append((scenario, proc, folder))
                    continue
                ok, detail = judge(scenario.name, folder, scenario)
                if code != 0 and not ok:
                    detail = f"webots exit {code}; " + detail
                results.append((scenario.name, ok, detail))
                print(f"SCENARIO {scenario.name} {'PASS' if ok else 'FAIL'} {detail}", flush=True)
            pending = still
            if pending:
                time.sleep(2.0)
        for scenario, proc, folder in pending:
            proc.kill()
            proc.wait(timeout=20)
            ok, detail = judge(scenario.name, folder, scenario)
            results.append((scenario.name, False, "timeout; " + detail))
            print(f"SCENARIO {scenario.name} FAIL timeout; {detail}", flush=True)
    finally:
        for path in worlds:
            path.unlink(missing_ok=True)
        latest = Path("/tmp/map_scenarios/latest")
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        try:
            latest.symlink_to(root)
        except OSError:
            pass
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the short apartment mapping scenarios.")
    parser.add_argument("--only", nargs="*", default=None, help="scenario names, default all")
    args = parser.parse_args(argv)
    if not shutil.which("webots"):
        print("webots is not on PATH", file=sys.stderr)
        return 2
    results = run_scenarios(args.only)
    failed = [name for name, ok, _detail in results if not ok]
    print(f"SCENARIOS {len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
