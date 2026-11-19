"""Waypoint driver and the snapshot port. No simulator pose."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from src.mapping.brain_port import latest_stop, publish_command, read_status, wait_for_command, write_png, write_stop
from src.mapping.bubble import BubbleView
from src.mapping.driver import STALL_WINDOW_S, WaypointDriver, parse_command
from src.mapping.explore import Explorer
from src.mapping.grid import OccupancyGrid


ROOT = Path(__file__).resolve().parents[1]
_BANNED = ("getposition", "apartment.wbt", "gt_pose", "supervisor", "apartment_ground_truth")


class _Odom:
    def __init__(self) -> None:
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.yaw_unwrapped_rad = 0.0
        self.last_ds_m = 0.0


class _Fusion:
    def __init__(self) -> None:
        self.scene_floor_m = None


class _Bot:
    def __init__(self) -> None:
        self.odometry = _Odom()
        self._fusion = _Fusion()
        self.grid = OccupancyGrid()
        self.wheels_lifted = 0
        self.t_s = 0.0
        self._why = None

    def pose(self):
        return (self.odometry.x_m, self.odometry.y_m, self.odometry.yaw_rad)

    def _stuck_reason(self):
        return self._why


def _open():
    return BubbleView(5.0, 5.0, 5.0, 5.0, 5.0)


def _shut():
    return BubbleView(0.05, 0.05, 0.05, 0.05, 0.05)


def test_parse_accepts_the_four_commands():
    assert parse_command({"cmd": "go_to", "x": 1, "y": -0.5}) == {"cmd": "go_to", "x": 1.0, "y": -0.5}
    assert parse_command({"cmd": "turn_to", "heading": 1.2})["yaw"] == pytest.approx(1.2)
    assert parse_command({"cmd": "scan"}) == {"cmd": "look"}
    assert parse_command({"cmd": "stop"}) == {"cmd": "stop"}
    with pytest.raises(ValueError):
        parse_command({"cmd": "wander"})


def test_go_to_arrives_within_five_centimetres():
    bot = _Bot()
    bot.odometry.x_m = 0.56
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 0.6, "y": 0.0}, bot)
    left, right = driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert (left, right) == (0.0, 0.0)
    assert driver.outcome == "arrived"
    assert driver.remaining_m == pytest.approx(0.04, abs=1e-6)


def test_the_frontier_radius_is_not_arrival():
    """0.28 m short is the old frontier gate. A waypoint is still driving."""
    bot = _Bot()
    bot.odometry.x_m = 0.32
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 0.6, "y": 0.0}, bot)
    left, right = driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert driver.outcome is None
    assert left > 0.0 and right > 0.0
    far = _Bot()
    far_driver = WaypointDriver()
    far_driver.arm({"cmd": "go_to", "x": 2.0, "y": 0.0}, far)
    far_left, far_right = far_driver.wheels(far, 5.0, 5.0, _open(), 0.05)
    close = _Bot()
    close.odometry.x_m = 1.85
    close_driver = WaypointDriver()
    close_driver.arm({"cmd": "go_to", "x": 2.0, "y": 0.0}, close)
    close_left, close_right = close_driver.wheels(close, 5.0, 5.0, _open(), 0.05)
    far_v = 0.5 * (far_left + far_right) * 0.08
    close_v = 0.5 * (close_left + close_right) * 0.08
    assert far_v > 0.2
    assert close_v < far_v * 0.7


def test_an_open_command_the_pose_does_not_follow_is_stuck():
    bot = _Bot()
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 2.0, "y": 0.0}, bot)
    left, right = driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert abs(left) > 0.1 and abs(right) > 0.1
    for _ in range(50):
        driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
        if driver.outcome:
            break
    assert driver.outcome == "stuck"
    assert driver._t >= STALL_WINDOW_S - 1e-6


def test_a_straight_door_the_rectangle_clips_still_drives():
    """The nose test leaves a centred doorway open. The rectangle does not.

    Zeroing that straight command sits the shell in the opening.
    """
    from src.mapping.footprint import footprint_clips
    from src.mapping.tick import _ahead_blocked

    bot = _Bot()
    for x in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35):
        for y in (0.20, -0.20):
            cell = bot.grid.cell_of(x, y)
            bot.grid.log[cell] = bot.grid.l_max
    pose = bot.pose()
    assert footprint_clips(bot.grid, pose, 0.28, 0.0)
    assert _ahead_blocked(bot.grid, pose) is False
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 1.5, "y": 0.0}, bot)
    left, right = driver.wheels(bot, 0.6, 0.6, _open(), 0.05)
    assert driver.outcome is None
    assert left > 0.0 and right > 0.0


def _vw(left: float, right: float) -> tuple[float, float]:
    v = 0.5 * (left + right) * 0.08
    w = (right - left) * 0.08 / 0.34
    return v, w


def test_a_door_back_off_is_not_redecided_every_frame():
    """A side gap that flickers used to swap creep and reverse every tick.

    The back-off stays for the 2 s clock. The next command is a turn
    toward the opening, not another creep on the heading that failed.
    """
    bot = _Bot()
    for x in (0.25, 0.45, 0.65):
        bot.grid.log[bot.grid.cell_of(x, 0.20)] = bot.grid.l_max
        bot.grid.log[bot.grid.cell_of(x, -0.55)] = bot.grid.l_max
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 1.5, "y": 0.0}, bot)
    close = BubbleView(1.5, 0.08, 1.2, 1.5, 0.08)
    opened = BubbleView(1.5, 0.40, 1.2, 1.5, 0.40)
    left, right = driver.wheels(bot, 0.08, 1.2, close, 0.05)
    held = _vw(left, right)
    assert held[0] < -0.02
    saw_turn = False
    for _ in range(80):
        left, right = driver.wheels(bot, 0.40, 1.2, opened, 0.05)
        v, w = _vw(left, right)
        if driver._backoff_vw is None:
            assert abs(v) < 0.02
            assert abs(w) > 0.05
            saw_turn = True
            break
        assert v == pytest.approx(held[0], abs=1e-6)
        assert w == pytest.approx(held[1], abs=1e-6)
    assert saw_turn
    assert driver._t >= STALL_WINDOW_S - 1e-6
    assert driver.outcome is None


def test_a_vetoed_go_to_stops_short_with_the_gap():
    bot = _Bot()
    bot.grid.log[:] = bot.grid.l_max
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 2.0, "y": 0.0}, bot)
    for _ in range(50):
        driver.wheels(bot, 0.05, 0.05, _shut(), 0.05)
        if driver.outcome:
            break
    assert driver.outcome == "stopped_short"
    assert driver.remaining_m == pytest.approx(2.0, abs=0.05)
    assert driver.report(bot)["remaining_m"] == pytest.approx(2.0, abs=0.05)


def test_timeout_fires_before_the_two_second_clock():
    bot = _Bot()
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 2.0, "y": 0.0}, bot, timeout_s=0.25)
    outcome = None
    for _ in range(10):
        driver.wheels(bot, 5.0, 5.0, _open(), 0.1)
        outcome = driver.outcome
        if outcome:
            break
    assert outcome == "stopped_short"
    assert driver.remaining_m == pytest.approx(2.0, abs=0.05)
    assert driver._t < STALL_WINDOW_S
    turn = WaypointDriver()
    turn.arm({"cmd": "turn_to", "yaw": 1.0}, bot, timeout_s=0.2)
    for _ in range(8):
        # Hold the heading so the turn does not finish.
        bot.odometry.yaw_rad = 0.0
        bot.odometry.yaw_unwrapped_rad = 0.0
        turn.wheels(bot, 5.0, 5.0, _open(), 0.1)
        if turn.outcome:
            break
    assert turn.outcome == "stopped_short"
    assert turn.remaining_rad == pytest.approx(1.0, abs=0.05)


def test_wheels_lifted_is_a_tilt_warning_and_not_a_new_pitch_input():
    bot = _Bot()
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 1.0, "y": 0.0}, bot)
    bot.wheels_lifted = 1
    left, right = driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert (left, right) == (0.0, 0.0)
    assert driver.outcome == "tilt_warning"
    text = (ROOT / "src" / "mapping" / "driver.py").read_text(encoding="utf-8")
    assert "oak_imu" not in text
    assert "pitch" not in text.lower()


def test_twelve_degrees_short_is_not_a_finished_turn():
    """The smoke stopped at 0.194 for a 0.40 command. That is the 12° pivot gate."""
    bot = _Bot()
    bot.odometry.yaw_rad = 0.194
    bot.odometry.yaw_unwrapped_rad = 0.194
    driver = WaypointDriver()
    driver.arm({"cmd": "turn_to", "yaw": 0.40}, bot)
    left, right = driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert driver.outcome is None
    w = (right - left) * 0.08 / 0.34
    assert 0.1 < abs(w) < 0.70
    done = _Bot()
    done.odometry.yaw_rad = 0.36
    done.odometry.yaw_unwrapped_rad = 0.36
    finished = WaypointDriver()
    finished.arm({"cmd": "turn_to", "yaw": 0.40}, done)
    assert finished.wheels(done, 5.0, 5.0, _open(), 0.05) == (0.0, 0.0)
    assert finished.outcome == "arrived"
    assert finished.remaining_rad == pytest.approx(0.04, abs=1e-6)


def test_a_spin_does_not_book_floor_shift_as_travel():
    bot = _Bot()
    driver = WaypointDriver()
    driver.arm({"cmd": "look"}, bot)
    driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    bot._fusion.scene_floor_m = 0.20
    bot.odometry.yaw_unwrapped_rad += 0.15
    bot.odometry.yaw_rad += 0.15
    driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert driver.floor_m == pytest.approx(0.0)
    report = driver.report(bot)
    assert report["travel"]["floor_reliable"] is False
    straight = _Bot()
    straight._fusion.scene_floor_m = 0.004
    other = WaypointDriver()
    other.arm({"cmd": "go_to", "x": 1.0, "y": 0.0}, straight)
    other.wheels(straight, 5.0, 5.0, _open(), 0.05)
    assert other.floor_m == pytest.approx(0.004)
    assert other.report(straight)["travel"]["floor_reliable"] is True


def test_turn_and_look_finish_on_the_estimate():
    bot = _Bot()
    driver = WaypointDriver()
    driver.arm({"cmd": "turn_to", "yaw": 0.0}, bot)
    assert driver.wheels(bot, 5.0, 5.0, _open(), 0.05) == (0.0, 0.0)
    assert driver.outcome == "arrived"
    driver.arm({"cmd": "look"}, bot)
    for _ in range(40):
        bot.odometry.yaw_unwrapped_rad += 0.2
        bot.odometry.yaw_rad += 0.2
        driver.wheels(bot, 5.0, 5.0, _open(), 0.02)
        if driver.outcome:
            break
    assert driver.outcome == "arrived"
    assert driver.detail == "full turn"


def test_stop_and_the_existing_stuck_reason():
    bot = _Bot()
    driver = WaypointDriver()
    driver.arm({"cmd": "stop"}, bot)
    assert driver.outcome == "stopped"
    driver.arm({"cmd": "go_to", "x": 1.0, "y": 0.0}, bot)
    bot._why = "camera_stuck"
    driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert driver.outcome == "stuck"
    assert driver.detail == "camera_stuck"


def test_travel_books_and_battery_use_the_robot_measurements():
    bot = _Bot()
    bot.odometry.last_ds_m = 0.01
    bot._fusion.scene_floor_m = 0.004
    driver = WaypointDriver()
    driver.arm({"cmd": "go_to", "x": 2.0, "y": 0.0}, bot)
    driver.wheels(bot, 5.0, 5.0, _open(), 0.05)
    assert driver.wheel_m == pytest.approx(0.01)
    assert driver.floor_m == pytest.approx(0.004)
    assert driver.commanded_m > 0.0
    assert driver.wh_used > 0.0
    # A short step is a fraction of a watt-hour. The percent moves once the book does.
    driver._note_command(3.5, 3.5, 60.0)
    assert driver.battery()["pct"] < 100.0
    assert driver.battery()["pack_wh"] == 480.0


def test_the_stall_clock_is_the_existing_two_seconds():
    assert STALL_WINDOW_S == 2.0
    text = (ROOT / "src" / "mapping" / "driver.py").read_text(encoding="utf-8")
    assert "STALL_WINDOW_S" in text


def test_snapshot_has_frames_map_and_no_simulator_pose(tmp_path):
    labels = np.array([[-1, 0], [1, 0]], dtype=np.int8)
    meta = {
        "resolution_m": 0.05,
        "origin_xy_m": [0.0, 0.0],
        "width": 2,
        "height": 2,
        "frame": "odometry",
    }
    report = {
        "status": "arrived",
        "detail": "within 0.28 m",
        "tilt_warning": False,
        "pose": {"x_m": 0.02, "y_m": 0.02, "yaw_rad": 0.1, "frame": "odometry"},
        "travel": {"commanded_m": 0.2, "wheel_m": 0.18, "floor_m": 0.15},
        "battery": {"pct": 99.9, "wh_used": 0.1, "pack_wh": 480.0},
        "command": {"cmd": "go_to", "x": 0.2, "y": 0.0},
    }
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    image[0, 0] = (10, 20, 30)
    stop = write_stop(tmp_path, 1, report, 4.5, {"oak": image, "floor": image[:, :, 0]}, labels, meta)
    assert (stop / "oak.png").read_bytes().startswith(b"\x89PNG")
    assert (stop / "floor.png").is_file()
    assert (stop / "map.png").is_file()
    status = read_status(stop)
    assert status["pose"]["frame"] == "odometry"
    assert status["travel"]["wheel_m"] == 0.18
    assert status["battery"]["pct"] == 99.9
    blob = (stop / "status.json").read_text(encoding="utf-8") + (stop / "map.json").read_text(encoding="utf-8")
    for token in _BANNED:
        assert token not in blob.lower()
    assert latest_stop(tmp_path) == stop
    grid = json.loads((stop / "map.json").read_text(encoding="utf-8"))
    assert grid["rows"][0].startswith("-.")
    write_png(tmp_path / "alone.png", np.array([[0, 255], [128, 64]], dtype=np.uint8))
    assert (tmp_path / "alone.png").read_bytes().startswith(b"\x89PNG")


def test_wait_unblocks_when_a_command_file_appears(tmp_path):
    def _write():
        time.sleep(0.1)
        publish_command(tmp_path, {"cmd": "go_to", "x": 0.5, "y": 0.0})

    thread = threading.Thread(target=_write)
    thread.start()
    cmd = wait_for_command(tmp_path, poll_s=0.02)
    thread.join()
    assert cmd == {"cmd": "go_to", "x": 0.5, "y": 0.0}
    assert not (tmp_path / "driver" / "command.json").exists()
    assert (tmp_path / "driver" / "command.accepted.json").is_file()


def test_cli_reads_a_snapshot_and_writes_a_command(tmp_path, capsys):
    import importlib.util

    labels = np.zeros((2, 2), dtype=np.int8)
    meta = {"resolution_m": 0.05, "origin_xy_m": [0.0, 0.0], "width": 2, "height": 2}
    report = {
        "status": "ready",
        "detail": "",
        "tilt_warning": False,
        "pose": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "frame": "odometry"},
        "travel": {"commanded_m": 0.0, "wheel_m": 0.0, "floor_m": 0.0},
        "battery": {"pct": 100.0, "wh_used": 0.0, "pack_wh": 480.0},
        "command": None,
    }
    write_stop(tmp_path, 0, report, 1.0, {}, labels, meta)
    spec = importlib.util.spec_from_file_location("driver_command", ROOT / "scripts" / "driver_command.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.main(["--run", str(tmp_path), "turn_to", "0.4"]) == 0
    written = json.loads((tmp_path / "driver" / "command.json").read_text(encoding="utf-8"))
    assert written == {"cmd": "turn_to", "yaw": 0.4}
    assert "odometry" in capsys.readouterr().out


def test_driver_flag_defaults_off_and_a_bound_explorer_still_steps():
    mapper = (ROOT / "webots" / "controllers" / "apartment_mapper" / "apartment_mapper.py").read_text(encoding="utf-8")
    assert '_flag("RBM_DRIVER", False)' in mapper
    plain = Explorer(fusion=False, floor=False, loop=False)
    assert plain._waypoint is None
    driven = Explorer(fusion=False, floor=False, loop=False)
    driven._waypoint = WaypointDriver()
    driven._waypoint.arm({"cmd": "go_to", "x": 1.0, "y": 0.0}, driven)
    left, right = driven.step(dt_s=0.05, left_wheel_rad=0.0, right_wheel_rad=0.0, yaw_rate_rad_s=0.0)
    assert left > 0.0 and right > 0.0
    assert driven._waypoint.outcome is None
    assert driven.done is False
