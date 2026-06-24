from src.mission_context import (
    filter_lru_result,
    filter_requirements,
    is_standby_lru,
    throttle_exempt_channels,
)


def test_drive_transit_marks_arms_torso_standby():
    assert is_standby_lru("arms", "drive_transit")
    assert is_standby_lru("torso", "drive_transit")
    assert not is_standby_lru("locomotion", "drive_transit")
    assert "Arms" in throttle_exempt_channels("drive_transit")


def test_filter_lru_suppresses_standby_warnings():
    raw = {
        "faults": [],
        "warnings": ["LRU Arms voltage sag: 21.0V (88% nominal)"],
        "lrus": [
            {"id": "arms", "label": "Arms", "status": "warning", "utilization_pct": 12},
            {"id": "locomotion", "label": "Locomotion", "status": "ok", "utilization_pct": 55},
        ],
        "degradation_level": "caution",
    }
    out = filter_lru_result(raw, "drive_transit")
    assert out["lrus"][0]["mission_role"] == "standby"
    assert out["degradation_level"] == "normal"


def test_filter_requirements_allows_low_idle_draw():
    raw = {
        "task": "moving",
        "violations": ["Arms: 5.0W below min (6.0W)"],
        "overall_compliant": False,
        "eps": {"compliant": True},
        "lru_requirements": [
            {
                "id": "arms",
                "label": "Arms",
                "draw_w": 5.0,
                "min_draw_w": 6.0,
                "compliant": False,
                "status": "warning",
            }
        ],
    }
    out = filter_requirements(raw, "drive_transit")
    assert out["overall_compliant"] is True
    assert out["lru_requirements"][0]["status"] == "standby"