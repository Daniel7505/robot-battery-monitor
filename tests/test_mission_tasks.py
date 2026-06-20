from src.mission_tasks import MissionTaskManager, TASK_PROFILES, predict_runtime
from src.power_allocator import PowerAllocator


def test_all_task_types_defined():
    assert set(TASK_PROFILES.keys()) == {"idle", "moving", "balanced", "high_load"}


def test_idle_draw_targets_lower_than_moving():
    idle = TASK_PROFILES["idle"].draw_targets
    moving = TASK_PROFILES["moving"].draw_targets
    assert idle["Legs"] < moving["Legs"]
    assert idle["Compute"] > idle["Legs"]


def test_balanced_between_idle_and_moving():
    balanced = TASK_PROFILES["balanced"].draw_targets
    assert balanced["Legs"] > TASK_PROFILES["idle"].draw_targets["Legs"]
    assert balanced["Legs"] < TASK_PROFILES["moving"].draw_targets["Legs"]


def test_mission_manager_returns_profile_info():
    mgr = MissionTaskManager("moving")
    info = mgr.mission_info(battery_pct=90, capacity_wh=1000, current_draw_w=55)
    assert info["task"] == "moving"
    assert "Transit" in info["task_label"]
    assert info["task_remaining_s"] > 0
    assert info["energy_wh_remaining"] == 900.0
    assert info["runtime_min_at_current_draw"] is not None


def test_predict_runtime_scales_with_draw():
    low = predict_runtime(80, 1000, 40, 40)
    high = predict_runtime(80, 1000, 80, 80)
    assert low["runtime_min_at_current_draw"] > high["runtime_min_at_current_draw"]


def test_allocator_uses_task_budget_factor():
    channels = [
        {"id": "Legs", "max_draw_w": 35},
        {"id": "Compute", "max_draw_w": 15},
    ]
    alloc = PowerAllocator(channels, system_budget_w=100)
    idle_result = alloc.allocate("idle", {"Legs": 30, "Compute": 10})
    moving_result = alloc.allocate("moving", {"Legs": 30, "Compute": 10})
    assert idle_result["budget_w"] < moving_result["budget_w"]


def test_force_task_changes_mission():
    mgr = MissionTaskManager(start_task="idle")
    assert mgr.force_task("high_load", duration_s=12)
    assert mgr.task_id == "high_load"
    assert mgr.seconds_remaining == 12
    assert not mgr.force_task("invalid_task")