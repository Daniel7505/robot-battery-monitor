"""Headless 600 s seeds on ButlerBotSkid. Worlds are deleted. Logs stay in /tmp.

The apartment world is copied and the robot instance is swapped to
ButlerBotSkid. ButlerBot.proto and apartment.wbt are not modified.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.mapping.world_guard import require_skid_world, swap_caster_instance
WEBOTS = ROOT / "webots"
WORLDS = WEBOTS / "worlds"
SOURCE = (WORLDS / "apartment.wbt").read_text(encoding="utf-8")
OUT = Path(os.environ.get("RBM_SKID_SEEDS", "/tmp/map_seeds/skid"))
SEEDS = [
    ("stock", 6.10, 4.70, 1.5708),
    ("p40", 6.10, 4.70, 1.9708),
    ("m35", 6.10, 4.70, 1.2208),
    ("east", 5.40, 3.80, 0.0),
    ("west", 6.60, 3.90, math.pi),
    ("sw", 4.90, 4.50, 0.7),
    ("north", 5.80, 5.60, 2.2),
    ("south", 6.40, 6.30, -1.2),
]


def patch(x: float, y: float, yaw: float) -> str:
    text = SOURCE
    old_t = "  translation 6.1 4.7 0\n"
    old_r = "  rotation 0 0 1 1.5708\n"
    if old_t not in text or old_r not in text or "\nButlerBot {\n" not in text:
        raise SystemExit("apartment spawn or proto block missing")
    text = swap_caster_instance(text)
    text = text.replace(old_t, f"  translation {x:.4f} {y:.4f} 0\n", 1)
    text = text.replace(old_r, f"  rotation 0 0 1 {yaw:.4f}\n", 1)
    require_skid_world(text, "apartment seed world")
    return text


def launch(name: str, x: float, y: float, yaw: float, port: int):
    folder = OUT / name
    folder.mkdir(parents=True, exist_ok=True)
    world = WORLDS / f"_seed_{name}.wbt"
    world.write_text(patch(x, y, yaw), encoding="utf-8")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    env["RBM_MAP_MAX_S"] = "600"
    env["RBM_MAP_DIR"] = str(folder)
    env["RBM_DECISION_LOG"] = "1"
    env["RBM_MOVIE"] = "1"
    env["RBM_DEPTH_SEED"] = str(1 + (port % 97))
    env.pop("RBM_DRIVE", None)
    env.pop("RBM_STRAIGHT_S", None)
    log = open(folder / "webots.log", "w", encoding="utf-8")
    proc = subprocess.Popen(
        [
            "xvfb-run",
            "-a",
            "-s",
            "-screen 0 1280x720x24",
            "webots",
            "--mode=fast",
            "--fullscreen",
            "--batch",
            "--stdout",
            "--stderr",
            f"--port={port}",
            f"worlds/_seed_{name}.wbt",
        ],
        cwd=str(WEBOTS),
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    return name, proc, world, log


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    pending = list(SEEDS)
    running = []
    port = 1711
    wave = 4
    try:
        while pending or running:
            while pending and len(running) < wave:
                name, x, y, yaw = pending.pop(0)
                running.append(launch(name, x, y, yaw, port))
                print(f"START {name} port={port}", flush=True)
                port += 1
            still = []
            for name, proc, world, log in running:
                code = proc.poll()
                if code is None:
                    still.append((name, proc, world, log))
                    continue
                log.close()
                world.unlink(missing_ok=True)
                print(f"DONE {name} exit={code}", flush=True)
            running = still
            if running:
                time.sleep(5)
    finally:
        for _name, proc, world, log in running:
            proc.kill()
            log.close()
            world.unlink(missing_ok=True)
        for path in WORLDS.glob("_seed_*.wbt"):
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
