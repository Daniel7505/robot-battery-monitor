"""
ButlerBot wheeled robot — example digital twin data flow.

Demonstrates how a compact wheeled mobile manipulator cycles through
standby → drive → patrol → manipulate phases with reference-hardware draws.
"""

from __future__ import annotations

from datetime import datetime, timezone

# Wheeled mission cycle for ButlerBot (reference-hardware watts)
BUTLERBOT_MISSION_FLOW = [
    {
        "phase": "standby",
        "label": "Standby",
        "task": "idle",
        "duration_s": 6,
        "locomotion": {"gait": "stand", "speed_m_s": 0.0},
        "channel_draws": {"Legs": 4.0, "Arms": 5.5, "Torso": 3.5, "Compute": 9.0, "Cooling": 1.5},
    },
    {
        "phase": "drive_transit",
        "label": "Drive / Transit",
        "task": "moving",
        "duration_s": 14,
        "locomotion": {"gait": "drive", "speed_m_s": 0.42},
        "channel_draws": {"Legs": 22.0, "Arms": 6.0, "Torso": 8.0, "Compute": 12.0, "Cooling": 4.5},
    },
    {
        "phase": "patrol",
        "label": "Patrol",
        "task": "balanced",
        "duration_s": 12,
        "locomotion": {"gait": "patrol", "speed_m_s": 0.28},
        "channel_draws": {"Legs": 14.0, "Arms": 7.5, "Torso": 7.0, "Compute": 11.5, "Cooling": 3.5},
    },
    {
        "phase": "manipulate",
        "label": "Manipulate",
        "task": "high_load",
        "duration_s": 10,
        "locomotion": {"gait": "manipulate", "speed_m_s": 0.0},
        "channel_draws": {"Legs": 6.0, "Arms": 18.0, "Torso": 12.0, "Compute": 13.0, "Cooling": 8.0},
    },
    {
        "phase": "return_idle",
        "label": "Return to Idle",
        "task": "idle",
        "duration_s": 10,
        "locomotion": {"gait": "stand", "speed_m_s": 0.0},
        "channel_draws": {"Legs": 5.0, "Arms": 5.5, "Torso": 3.5, "Compute": 9.0, "Cooling": 2.0},
    },
]

# Backward-compatible alias
BUTLERBOT_WALKING_FLOW = BUTLERBOT_MISSION_FLOW


def butlerbot_telemetry_step(
    step_index: int,
    *,
    source: str = "custom",
    adapter: str = "butlerbot",
    battery_pct: float = 88.0,
    robot_name: str = "ButlerBot",
) -> dict:
    """Build a twin telemetry payload for one step of the ButlerBot mission flow."""
    step = BUTLERBOT_MISSION_FLOW[step_index % len(BUTLERBOT_MISSION_FLOW)]
    total_draw = round(sum(step["channel_draws"].values()), 1)
    return {
        "schema_version": "1.1",
        "source": source,
        "adapter": adapter,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "robot": {
            "name": robot_name,
            "main_battery_pct": round(battery_pct, 1),
            "battery_capacity_wh": 480,
        },
        "mission": {
            "task": step["task"],
            "phase": step["phase"],
            "phase_label": step.get("label") or step["phase"].replace("_", " ").title(),
        },
        "channel_draws": dict(step["channel_draws"]),
        "power": {
            "total_draw_w": total_draw,
            "channel_draws": dict(step["channel_draws"]),
        },
        "locomotion": dict(step["locomotion"]),
        "pose": {
            "x_m": 0.0,
            "y_m": 0.0,
            "heading_rad": 0.0,
        },
    }


def butlerbot_flow_description() -> dict:
    """Document the example ButlerBot twin data flow for integrators."""
    return {
        "robot": "ButlerBot",
        "description": "Wheeled mobile manipulator — standby → drive → patrol → manipulate → idle",
        "locomotion": "wheeled (differential drive)",
        "cycle_steps": len(BUTLERBOT_MISSION_FLOW),
        "phases": [
            {
                "phase": s["phase"],
                "label": s.get("label", s["phase"]),
                "task": s["task"],
                "duration_s": s["duration_s"],
                "expected_draw_w": round(sum(s["channel_draws"].values()), 1),
                "locomotion": s["locomotion"],
            }
            for s in BUTLERBOT_MISSION_FLOW
        ],
        "integration": {
            "ingest": "POST /api/twin/telemetry",
            "poll_state": "GET /api/twin/state",
            "send_commands": "POST /api/twin/command",
            "example_script": "examples/butlerbot_twin_feed.py",
        },
    }