from src.onboard_agent import OnboardAgent
from src.twin.bridge import DigitalTwinBridge
from src.twin.control import build_twin_control_status, webots_phase_flow


class _FakeHardware:
    allocation_status = {
        "task": "moving",
        "status": "throttled",
        "utilization_pct": 72,
        "throttled_channels": ["Legs", "Compute"],
    }
    mission_info = {"task": "moving", "task_label": "Walking / Transit"}
    agent_status = {
        "posture": "cautious",
        "controlling": True,
        "recommendations": [{"action": "throttle_system", "factor": 0.85}],
        "applied_actions": [],
    }


def test_webots_phase_flow_has_five_steps():
    flow = webots_phase_flow()
    assert len(flow) == 5
    assert flow[0]["phase"] == "standby"
    assert flow[1]["phase"] == "drive_transit"


def test_build_twin_control_status_with_external_feed():
    bridge = DigitalTwinBridge()
    bridge.ingest_telemetry(
        {
            "source": "webots",
            "phase": "patrol",
            "gait": "patrol",
            "locomotion": {"gait": "patrol", "phase": "patrol", "speed_m_s": 0.2},
            "mission": {"task": "balanced"},
            "channel_draws": {"Legs": 12, "Arms": 8, "Torso": 7, "Compute": 9},
        },
        adapter="webots",
    )
    hw = _FakeHardware()
    status = build_twin_control_status(bridge, hw)
    assert status["active"] is True
    assert status["sim_phase"] == "patrol"
    assert status["active_phase_index"] == 2
    assert status["agent_controlling"] is True
    assert "PMS throttling" in status["power_influence"]


def test_twin_stress_phase_detection():
    from src.twin.control import is_twin_stress_phase
    assert is_twin_stress_phase("drive_transit")
    assert is_twin_stress_phase("walk_transit")
    assert is_twin_stress_phase("manipulate")
    assert not is_twin_stress_phase("standby")


def test_agent_logs_phase_change():
    agent = OnboardAgent()
    agent.record_phase_change("walk_transit", "walk", "moving")
    agent.record_phase_change("walk_transit", "walk", "moving")
    agent.record_phase_change("patrol", "patrol", "balanced")
    log = agent._recent_log
    assert len(log) == 2
    assert log[0]["action"] == "phase_change"
    assert log[0]["sim_phase"] == "patrol"
    assert log[0]["influence"] == "twin"