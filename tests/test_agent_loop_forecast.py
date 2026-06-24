from src.onboard_agent import OnboardAgent, AgentContext


def _ctx(**kwargs):
    defaults = dict(
        battery_pct=50.0,
        task_id="moving",
        total_draw_w=45.0,
        utilization_pct=70.0,
        allocation_status="ok",
        throttled_channels=[],
        safety={"thermal_status": "normal", "thermal_c": 30, "alerts": []},
        prediction={},
        mission={},
        readings={},
        twin_phase="drive_transit",
        twin_gait="drive",
        twin_source="webots",
    )
    defaults.update(kwargs)
    return AgentContext(**defaults)


def test_loop_forecast_rule_fires_when_margin_low():
    agent = OnboardAgent()
    ctx = _ctx(
        loop_forecast={
            "ok": True,
            "margin_pct": 6.0,
            "can_complete_loop": False,
            "loop_wh_remaining": 0.45,
            "energy_wh_remaining": 0.12,
            "finish_battery_pct": 2.0,
            "feasibility_status": "insufficient",
        }
    )
    recs = agent._rule_loop_forecast(ctx, agent._cfg)
    assert recs
    assert any(r.action == "suggest_task" and r.task == "idle" for r in recs)


def test_negotiator_suggests_balanced_on_tight_transit():
    agent = OnboardAgent()
    ctx = _ctx(
        loop_forecast={
            "ok": True,
            "margin_pct": 12.0,
            "can_complete_loop": True,
            "finish_battery_pct": 14.0,
            "feasibility_status": "tight",
        }
    )
    recs = agent._rule_negotiator(ctx, agent._cfg)
    assert any(r.action == "suggest_task" and r.task == "balanced" for r in recs)