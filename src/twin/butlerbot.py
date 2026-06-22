"""
ButlerBot walking robot — example digital twin data flow.

Demonstrates how a compact mobile manipulator cycles through stand → walk →
patrol → manipulate phases with realistic per-channel draw for external simulators.
"""

from __future__ import annotations

from datetime import datetime, timezone

# Example walking cycle for ButlerBot (servo-scale watts)
BUTLERBOT_WALKING_FLOW = [
    {
        "phase": "standby",
        "task": "idle",
        "duration_s": 8,
        "locomotion": {"gait": "stand", "speed_m_s": 0.0},
        "channel_draws": {"Legs": 3.5, "Arms": 5.0, "Torso": 3.5, "Compute": 7.5},
    },
    {
        "phase": "walk_transit",
        "task": "moving",
        "duration_s": 12,
        "locomotion": {"gait": "walk", "speed_m_s": 0.45, "step_hz": 1.2},
        "channel_draws": {"Legs": 20.0, "Arms": 8.0, "Torso": 10.5, "Compute": 9.0},
    },
    {
        "phase": "patrol",
        "task": "balanced",
        "duration_s": 10,
        "locomotion": {"gait": "patrol", "speed_m_s": 0.25},
        "channel_draws": {"Legs": 12.0, "Arms": 9.0, "Torso": 7.5, "Compute": 8.5},
    },
    {
        "phase": "manipulate",
        "task": "high_load",
        "duration_s": 8,
        "locomotion": {"gait": "stand", "speed_m_s": 0.0, "payload_kg": 2.5},
        "channel_draws": {"Legs": 24.0, "Arms": 16.0, "Torso": 12.0, "Compute": 10.0},
    },
    {
        "phase": "return_idle",
        "task": "idle",
        "duration_s": 6,
        "locomotion": {"gait": "stand", "speed_m_s": 0.0},
        "channel_draws": {"Legs": 3.5, "Arms": 5.0, "Torso": 3.5, "Compute": 7.5},
    },
]


def butlerbot_telemetry_step(
    step_index: int,
    *,
    source: str = "custom",
    adapter: str = "butlerbot",
    battery_pct: float = 88.0,
    robot_name: str = "ButlerBot",
) -> dict:
    """Build a twin telemetry payload for one step of the ButlerBot walking flow."""
    step = BUTLERBOT_WALKING_FLOW[step_index % len(BUTLERBOT_WALKING_FLOW)]
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
            "phase_label": step["phase"].replace("_", " ").title(),
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
        "description": "Compact walking manipulator — stand → walk → patrol → manipulate → idle",
        "cycle_steps": len(BUTLERBOT_WALKING_FLOW),
        "phases": [
            {
                "phase": s["phase"],
                "task": s["task"],
                "duration_s": s["duration_s"],
                "expected_draw_w": round(sum(s["channel_draws"].values()), 1),
                "locomotion": s["locomotion"],
            }
            for s in BUTLERBOT_WALKING_FLOW
        ],
        "integration": {
            "ingest": "POST /api/twin/telemetry",
            "poll_state": "GET /api/twin/state",
            "send_commands": "POST /api/twin/command",
            "example_script": "examples/butlerbot_twin_feed.py",
        },
    }