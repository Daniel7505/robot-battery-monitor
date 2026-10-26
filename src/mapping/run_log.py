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
        self.yaw_path = self.directory / "yaw_sources.csv"
        self._yaw = self.yaw_path.open("w", encoding="utf-8", newline="\n")
        self._yaw.write(
            "t_s,fused_yaw,gyro_raw_yaw,gyro_unbiased_yaw,wheel_yaw,"
            "visual_meas_yaw,visual_applied_yaw,floor_applied_yaw,wall_applied_yaw,"
            "bias_rad_s,still_s,moving_s\n"
        )
        # Encoder angles are the mapper's own sensors. The pose logger's
        # physics_log.csv has the true body speed for the slip comparison.
        self.wheel_path = self.directory / "wheel_enc.csv"
        self._wheel = self.wheel_path.open("w", encoding="utf-8", newline="\n")
        self._wheel.write("t_s,left_rad,right_rad,left_cmd,right_cmd\n")
        self.rows = 0

    def write_pose(self, t_s: float, x_m: float, y_m: float, yaw_rad: float) -> None:
        self._pose.write(f"{t_s:.4f},{x_m:.5f},{y_m:.5f},{yaw_rad:.6f}\n")
        self.rows += 1
        if self.rows % 25 == 0:
            self._pose.flush()

    def write_yaw(
        self,
        t_s: float,
        fused_yaw: float,
        gyro_raw_yaw: float,
        gyro_unbiased_yaw: float,
        wheel_yaw: float,
        visual_meas_yaw: float,
        visual_applied_yaw: float,
        floor_applied_yaw: float,
        wall_applied_yaw: float,
        bias_rad_s: float,
        still_s: float,
        moving_s: float,
    ) -> None:
        self._yaw.write(
            f"{t_s:.4f},{fused_yaw:.6f},{gyro_raw_yaw:.6f},{gyro_unbiased_yaw:.6f},"
            f"{wheel_yaw:.6f},{visual_meas_yaw:.6f},{visual_applied_yaw:.6f},"
            f"{floor_applied_yaw:.6f},{wall_applied_yaw:.6f},{bias_rad_s:.6f},"
            f"{still_s:.3f},{moving_s:.3f}\n"
        )

    def write_wheels(
        self,
        t_s: float,
        left_rad: float,
        right_rad: float,
        left_cmd: float,
        right_cmd: float,
    ) -> None:
        self._wheel.write(
            f"{t_s:.4f},{left_rad:.6f},{right_rad:.6f},{left_cmd:.4f},{right_cmd:.4f}\n"
        )

    def save_map(self, labels: np.ndarray, meta: dict) -> None:
        np.save(self.directory / "map.npy", np.asarray(labels, dtype=np.int8))
        (self.directory / "map_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
        self._pose.flush()

    def save_rooms(self, labels, names: dict) -> None:
        if labels is None:
            return
        np.save(self.directory / "rooms.npy", np.asarray(labels, dtype=np.int16))
        payload = {"names": {str(key): value for key, value in names.items()}, "frame": "odometry"}
        (self.directory / "rooms_meta.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def close(self) -> None:
        self._pose.flush()
        self._pose.close()
        self._yaw.flush()
        self._yaw.close()
        self._wheel.flush()
        self._wheel.close()
