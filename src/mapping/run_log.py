"""Per-run map and pose files.

The folder is under ``logs/mapping_runs/``, which is gitignored. Nothing
here is ground truth. ``CURRENT`` is a pointer so the separate pose
logger can append to the same folder.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def default_run_dir(repo_root: Path, stamp: str | None = None) -> Path:
    if stamp is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path(repo_root) / "logs" / "mapping_runs" / stamp


def publish_current(repo_root: Path, run_dir: Path) -> None:
    pointer = Path(repo_root) / "logs" / "mapping_runs" / "CURRENT"
    pointer.parent.mkdir(parents=True, exist_ok=True)
    pointer.write_text(str(Path(run_dir).resolve()) + "\n", encoding="utf-8")


class RunLog:
    def __init__(self, directory: Path) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.pose_path = self.directory / "pose_log.csv"
        self._pose = self.pose_path.open("w", encoding="utf-8", newline="\n")
        self._pose.write("t_s,x_m,y_m,yaw_rad\n")
        self.rows = 0

    def write_pose(self, t_s: float, x_m: float, y_m: float, yaw_rad: float) -> None:
        self._pose.write(f"{t_s:.4f},{x_m:.5f},{y_m:.5f},{yaw_rad:.6f}\n")
        self.rows += 1
        if self.rows % 25 == 0:
            self._pose.flush()

    def save_map(self, labels: np.ndarray, meta: dict) -> None:
        np.save(self.directory / "map.npy", np.asarray(labels, dtype=np.int8))
        (self.directory / "map_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
        self._pose.flush()

    def close(self) -> None:
        self._pose.flush()
        self._pose.close()
