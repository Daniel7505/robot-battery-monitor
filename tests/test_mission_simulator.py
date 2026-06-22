import time

from src.mission_simulator import MissionSimulator
from src.mission_tasks import MissionTaskManager, TICK_SECONDS
from src.hardware import reset_hardware_source
from src.hardware_ros2 import ROS2BatterySource


def test_simulator_runs_script_sequence():
    sim = MissionSimulator()
    sim._enabled = True
    sim._loop = False
    sim._script = [
        {"task": "idle", "duration_s": 6, "label": "Idle"},
        {"task": "moving", "duration_s": 6, "label": "Transit"},
        {"task": "high_load", "duration_s": 6, "label": "Load"},
    ]
    mission = MissionTaskManager(start_task="idle")
    sim.start(mission)

    assert mission.task_id == "idle"
    ticks = 6 // TICK_SECONDS
    for _ in range(ticks):
        sim.advance(mission)

    changed = sim.advance(mission)
    assert changed or mission.task_id == "moving"

    for _ in range(ticks):
        sim.advance(mission)
    sim.advance(mission)
    assert mission.task_id == "high_load"


def test_simulator_loops_when_enabled():
    sim = MissionSimulator()
    sim._enabled = True
    sim._loop = True
    sim._script = [
        {"task": "idle", "duration_s": 3, "label": "Idle"},
        {"task": "moving", "duration_s": 3, "label": "Transit"},
    ]
    mission = MissionTaskManager(start_task="idle")
    sim.start(mission)

    for _ in range(20):
        sim.advance(mission)

    assert sim.running
    assert sim._loops_completed >= 1


def test_ros2_source_simulation_status():
    reset_hardware_source()
    source = ROS2BatterySource()
    source.start()
    time.sleep(4)
    assert source.simulation_status.get("enabled") is True
    assert source.simulation_status.get("script")
    assert len(source.last_readings) == 4
    source.stop_simulation()
    assert source.simulation_status.get("running") is False
    source.start_simulation()
    assert source.simulation_status.get("running") is True
    source.stop()
    reset_hardware_source()