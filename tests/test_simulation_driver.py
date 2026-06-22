import time

from src.simulation_driver import SimulationDriver, BUTLERBOT_SCRIPT
from src.mission_tasks import MissionTaskManager, TICK_SECONDS
from src.hardware import reset_hardware_source
from src.hardware_ros2 import ROS2BatterySource


def test_butlerbot_script_has_full_cycle():
    tasks = [s["task"] for s in BUTLERBOT_SCRIPT]
    assert tasks == ["idle", "moving", "balanced", "high_load", "idle"]


def test_draw_profiles_realistic_totals():
    driver = SimulationDriver()
    idle = sum(driver.draw_targets_for("idle").values())
    moving = sum(driver.draw_targets_for("moving").values())
    balanced = sum(driver.draw_targets_for("balanced").values())
    high = sum(driver.draw_targets_for("high_load").values())
    assert 15 <= idle <= 25
    assert 40 <= moving <= 55
    assert 30 <= balanced <= 45
    assert 55 <= high <= 65
    assert moving > balanced > idle


def test_driver_advances_through_phases():
    driver = SimulationDriver()
    driver._script = [
        {"task": "idle", "duration_s": 6, "label": "Idle"},
        {"task": "moving", "duration_s": 6, "label": "Transit"},
        {"task": "balanced", "duration_s": 6, "label": "Patrol"},
    ]
    driver._loop = False
    mission = MissionTaskManager(start_task="idle")
    mission.attach_simulation_driver(driver)
    driver.start(mission)

    seen = {mission.task_id}
    for _ in range(30):
        driver.advance(mission)
        seen.add(mission.task_id)

    assert "idle" in seen
    assert "moving" in seen
    assert "balanced" in seen


def test_ros2_butlerbot_simulation_integrated():
    reset_hardware_source()
    source = ROS2BatterySource()
    source.start()
    time.sleep(5)
    status = source.simulation_status
    assert status.get("driver") == "butlerbot"
    assert status.get("robot_name") == "ButlerBot"
    assert len(status.get("script", [])) >= 4
    assert source.allocation_status.get("task") in {
        "idle", "moving", "balanced", "high_load"
    }
    draws = [source.last_readings[ch]["draw"] for ch in source.last_readings]
    assert sum(draws) > 10
    source.stop()
    reset_hardware_source()