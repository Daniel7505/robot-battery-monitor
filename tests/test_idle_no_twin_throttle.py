"""
Idle + no external twin must not permanently latch safety-fault / agent-throttle.

Root cause covered: soft LRU task budgets below profile draw targets were
promoted to hard faults, then agent re-applied system ×70% every tick.
"""

from src.onboard_agent import OnboardAgent, AgentRecommendation
from src.power_requirements import PowerRequirements
from src.safety_monitor import SafetyMonitor

_CHANNELS = [
    {"id": "Legs", "max_draw_w": 28, "nominal_voltage": 48},
    {"id": "Arms", "max_draw_w": 18, "nominal_voltage": 48},
    {"id": "Torso", "max_draw_w": 14, "nominal_voltage": 48},
    {"id": "Compute", "max_draw_w": 12, "nominal_voltage": 24},
    {"id": "Cooling", "max_draw_w": 10, "nominal_voltage": 24},
]

# Typical idle profile draws (TASK_PROFILES idle ≈ 5/8/5.5/10.5/2)
_IDLE_ALLOCATED = {
    "Legs": 5.0,
    "Arms": 8.0,
    "Torso": 5.5,
    "Compute": 10.5,
    "Cooling": 2.0,
}


def test_soft_budget_overshoot_is_warning_not_fault():
    req = PowerRequirements(_CHANNELS, system_budget_w=62)
    lru_states = [
        {
            "id": "locomotion",
            "label": "Locomotion (LRUA)",
            "draw_w": 9.0,  # over soft budget, under hard max
            "utilization_pct": 32,
            "status": "ok",
        },
        {
            "id": "compute",
            "label": "Compute",
            "draw_w": 10.5,
            "utilization_pct": 87,
            "status": "ok",
        },
    ]
    result = req.evaluate(
        "idle",
        lru_states,
        total_draw_w=sum(_IDLE_ALLOCATED.values()),
        task_budget_w=43.4,
    )
    soft = [v for v in result["violations"] if "soft budget" in v.lower()]
    hard = [v for v in result["violations"] if "hard max" in v.lower()]
    assert soft or any(
        e.get("status") == "warning" for e in result["lru_requirements"]
    )
    assert not hard
    # No requirement entry should be fault for soft-only overshoot
    assert all(e.get("status") != "fault" for e in result["lru_requirements"])


def test_idle_draws_do_not_force_safety_fault_throttle():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=62)
    total = sum(_IDLE_ALLOCATED.values())
    allocation = {
        "budget_w": 43.4,
        "utilization_pct": round(total / 43.4 * 100, 1),
        "status": "ok",
    }
    # Several ticks — residual / soft budgets / voltage sag must not latch
    # fault + critical 70% throttle in no-twin idle.
    result = None
    for _ in range(5):
        result = mon.evaluate(
            battery_pct=80.0,
            requested=dict(_IDLE_ALLOCATED),
            allocated=dict(_IDLE_ALLOCATED),
            allocation=allocation,
            task_id="idle",
            task_budget_w=43.4,
            twin_phase=None,
            tick_seconds=3.0,
        )
    assert result is not None
    assert result["status"] != "fault"
    assert result.get("throttle_required") is False
    assert result.get("throttle_reason") != "safety fault"
    assert result.get("faults") == []


def test_agent_does_not_stack_throttle_when_safety_already_applied():
    agent = OnboardAgent()
    agent._cfg["auto_apply_throttle"] = True
    result = type("R", (), {})()
    result.recommendations = [
        AgentRecommendation(
            action="throttle_system",
            priority="high",
            reason="safety fault",
            factor=0.70,
            rule_id="safety_mirror",
        )
    ]
    result.applied_actions = []
    allocated = {"Legs": 20.0, "Arms": 10.0, "Torso": 8.0, "Compute": 10.0}
    out, applied = agent.apply_throttle(
        allocated, result, safety_already_throttled=True
    )
    assert applied == []
    assert out == allocated


def test_safety_mirror_skips_when_already_throttled_or_fault():
    agent = OnboardAgent()
    for status in ("throttled", "fault"):
        result = agent.evaluate(
            battery_pct=80,
            task_id="idle",
            allocation={
                "task": "idle",
                "total_allocated_w": 31.0,
                "utilization_pct": 50,
                "status": status,
                "throttled_channels": ["Legs"],
                "budget_w": 43.4,
            },
            safety={
                "status": "fault" if status == "fault" else "warning",
                "thermal_c": 35.0,
                "thermal_status": "normal",
                "alerts": [],
                "faults": ["legacy soft"] if status == "fault" else [],
                "warnings": [],
                "spike_channels": [],
                "throttle_required": True,
                "throttle_factor": 0.70,
                "throttle_reason": "safety fault",
                "lru": {"lrus": []},
            },
            prediction={"risk_level": "low", "mission_energy_ok": True},
            mission={"task": "idle"},
            readings={},
        )
        system_throttles = [
            r
            for r in result.recommendations
            if r.action == "throttle_system" and r.rule_id == "safety_mirror"
        ]
        assert system_throttles == [], f"status={status} still mirrored throttle"
