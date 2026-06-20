from src.power_allocator import PowerAllocator

_CHANNELS = [
    {"id": "Legs", "max_draw_w": 35, "nominal_voltage": 48},
    {"id": "Arms", "max_draw_w": 25, "nominal_voltage": 48},
    {"id": "Torso", "max_draw_w": 20, "nominal_voltage": 48},
    {"id": "Compute", "max_draw_w": 15, "nominal_voltage": 24},
]


def test_allocate_within_budget():
    alloc = PowerAllocator(_CHANNELS, system_budget_w=80)
    result = alloc.allocate("idle", {"Legs": 10, "Arms": 8, "Torso": 6, "Compute": 9})
    assert result["status"] == "ok"
    assert result["total_allocated_w"] == result["total_requested_w"]
    assert not result["throttled_channels"]


def test_allocate_throttles_over_budget():
    alloc = PowerAllocator(_CHANNELS, system_budget_w=60)
    result = alloc.allocate(
        "high_load",
        {"Legs": 34, "Arms": 20, "Torso": 18, "Compute": 13},
    )
    assert result["total_allocated_w"] <= 60
    assert result["status"] == "throttled"
    assert result["throttled_channels"]


def test_allocate_caps_channel_max():
    alloc = PowerAllocator(_CHANNELS, system_budget_w=95)
    result = alloc.allocate("moving", {"Legs": 50, "Arms": 10, "Torso": 8, "Compute": 9})
    assert result["allocated"]["Legs"] == 35
    assert any("channel cap" in d for d in result["decisions"])