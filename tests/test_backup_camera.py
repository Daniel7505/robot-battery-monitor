"""The backup-camera overlay is the driver's path, drawn and nothing else."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from backup_camera import crosses_occupied, path_kind, planned_path  # noqa: E402


def _corridor() -> tuple[np.ndarray, tuple[float, float], float]:
    # Wider than the 0.20 m planning disk, so the centre line stays free.
    labels = np.full((80, 80), -1, dtype=np.int8)
    labels[30:50, 10:70] = 0
    labels[29, 10:70] = 1
    labels[50, 10:70] = 1
    return labels, (-2.0, -2.0), 0.05


def test_free_goal_uses_the_driver_path():
    labels, origin, res = _corridor()
    pose = (origin[0] + 15.5 * res, origin[1] + 40.5 * res, 0.0)
    goal = (origin[0] + 60.5 * res, origin[1] + 40.5 * res)
    path = planned_path(labels, origin, res, pose, goal)
    assert path_kind(labels, origin, res, pose, goal, path) == "astar"
    assert path[-1] == goal
    assert len(path) > 2
    assert crosses_occupied(labels, origin, res, path) == 0


def test_a_mapped_wall_is_not_a_straight_aim():
    labels, origin, res = _corridor()
    pose = (origin[0] + 15.5 * res, origin[1] + 40.5 * res, 0.0)
    goal_col = 40
    labels[40, goal_col] = 1
    goal = (origin[0] + (goal_col + 0.5) * res, origin[1] + 40.5 * res)
    path = planned_path(labels, origin, res, pose, goal)
    assert path != [goal]
    assert crosses_occupied(labels, origin, res, path) == 0
    assert path_kind(labels, origin, res, pose, goal, path) == "astar"


def test_unknown_goal_is_a_straight_aim():
    labels, origin, res = _corridor()
    pose = (origin[0] + 15.5 * res, origin[1] + 40.5 * res, 0.0)
    goal = (origin[0] + 60.5 * res, origin[1] + 10.5 * res)
    path = planned_path(labels, origin, res, pose, goal)
    assert path == [goal]
    assert path_kind(labels, origin, res, pose, goal, path) == "straight"
