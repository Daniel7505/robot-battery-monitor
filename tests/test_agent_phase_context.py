"""Agent respects twin phase — no throttle on intentionally idle LRUs."""

from src.onboard_agent import AgentRecommendation, OnboardAgent


def _agent_ctx(**overrides):
    base = {
        "battery_pct": 88.0,
        "task_id": "moving",
        "total_draw_w": 58.0,
        "utilization_pct": 92.0,
        "allocation_status": "ok",
        "throttled_channels": [],
        "safety": {
            "status": "warning",
            "lru": {
                "lrus": [
                    {
                        "id": "arms",
                        "label": "Arms",
                        "status": "standby",
                        "mission_role": "standby",
                        "channels": ["Arms"],
                    },
                    {
                        "id": "locomotion",
                        "label": "Locomotion (LRUA)",
                        "status": "warning",
                        "mission_role": "active",
                        "channels": ["Legs"],
                    },
                ]
            },
            "spike_channels": ["Arms", "Legs"],
        },
        "prediction": {},
        "mission": {},
        "readings": {},
        "twin_phase": "drive_transit",
        "twin_gait": "drive",
        "twin_source": "webots",
        "loop_forecast": {},
    }
    base.update(overrides)
    return base


def test_lru_degraded_skips_standby_during_drive_transit():
    agent = OnboardAgent()
    from src.onboard_agent import AgentContext

    ctx = AgentContext(**_agent_ctx())
    recs = agent._rule_lru_degraded(ctx, agent._cfg)
    channels = {r.channel for r in recs}
    assert "Arms" not in channels
    assert "Legs" in channels


def test_phase_filter_drops_exempt_channel_throttles():
    agent = OnboardAgent()
    from src.onboard_agent import AgentContext

    ctx = AgentContext(
        battery_pct=80,
        task_id="moving",
        total_draw_w=50,
        utilization_pct=70,
        allocation_status="ok",
        throttled_channels=[],
        safety={},
        prediction={},
        mission={},
        readings={},
        twin_phase="drive_transit",
        twin_gait="drive",
        twin_source="webots",
    )
    recs = [
        AgentRecommendation(action="throttle_channel", channel="Arms", priority="high", reason="x"),
        AgentRecommendation(action="throttle_channel", channel="Legs", priority="high", reason="y"),
    ]
    filtered = agent._phase_filter_recommendations(recs, ctx)
    assert len(filtered) == 1
    assert filtered[0].channel == "Legs"