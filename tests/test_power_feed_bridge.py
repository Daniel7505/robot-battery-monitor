"""Power feed contract: external/Webots telemetry → hardware.last_readings."""

from src.hardware import reset_hardware_source
from src.hardware_ros2 import ROS2BatterySource
from src.twin import get_twin_bridge, reset_twin_bridge
from src.twin.adapters import get_adapter
from src.twin.power_feed import POWER_FEED_SCHEMA_VERSION, PowerFeed, power_feed_from_telemetry
from src.twin.butlerbot import butlerbot_telemetry_step


def test_power_feed_contract_shape():
    feed = PowerFeed(
        source="webots",
        battery_pct=88.5,
        channel_draws={"Legs": 20.0, "Arms": 6.0, "Torso": 8.0, "Compute": 10.0},
        task="moving",
    )
    assert feed.usable() is True
    d = feed.to_dict()
    assert d["schema_version"] == POWER_FEED_SCHEMA_VERSION
    assert d["battery_pct"] == 88.5
    assert d["channel_draws"]["Legs"] == 20.0


def test_webots_adapter_prefers_prebuilt_channel_draws():
    adapter = get_adapter("webots")
    tel = adapter.normalize({
        "source": "webots",
        "robot": {"name": "ButlerBot", "main_battery_pct": 73.0},
        "channel_draws": {
            "Legs": 19.5,
            "Arms": 5.5,
            "Torso": 7.0,
            "Compute": 9.0,
            "Cooling": 2.5,
        },
        "motor_power_w": {"left_wheel": 99.0, "right_wheel": 99.0},  # must not override
        "locomotion": {"gait": "drive", "speed_m_s": 0.4, "phase": "drive_transit"},
        "mission": {"task": "moving"},
    })
    assert tel.channel_draws["Legs"] == 19.5
    assert tel.channel_draws["Cooling"] == 2.5
    assert tel.battery_pct == 73.0
    assert tel.locomotion.get("phase") == "drive_transit"


def test_ingest_updates_last_readings_end_to_end():
    reset_twin_bridge()
    reset_hardware_source()
    source = ROS2BatterySource()
    # Do not start full telemetry loop — avoid DB/sim side effects
    bridge = get_twin_bridge()
    payload = {
        "source": "webots",
        "adapter": "webots",
        "robot": {"name": "ButlerBot", "main_battery_pct": 66.0},
        "mission": {"task": "moving"},
        "channel_draws": {
            "Legs": 22.0,
            "Arms": 6.0,
            "Torso": 8.0,
            "Compute": 11.0,
            "Cooling": 4.0,
        },
        "locomotion": {"gait": "drive", "speed_m_s": 0.35, "phase": "drive_transit"},
    }
    result = bridge.ingest_telemetry(payload, adapter="webots")
    assert result["ok"] is True

    feed = bridge.get_power_feed()
    assert feed is not None
    assert feed.source == "webots"
    assert feed.battery_pct == 66.0
    assert feed.channel_draws["Legs"] == 22.0

    assert bridge.sync_to_hardware(source) is True
    assert source.power_source == "webots"
    assert source._main_battery == 66.0
    assert source.last_readings["Legs"]["draw"] == 22.0
    assert source.last_readings["Legs"]["battery"] == 66.0
    assert source.last_readings["Legs"]["power_source"] == "webots"
    assert source.last_readings["Compute"]["draw"] == 11.0

    reset_hardware_source()
    reset_twin_bridge()


def test_fallback_when_no_external_feed():
    reset_twin_bridge()
    reset_hardware_source()
    source = ROS2BatterySource()
    bridge = get_twin_bridge()
    assert bridge.get_power_feed() is None
    assert bridge.sync_to_hardware(source) is False
    assert source.power_source == "internal"
    reset_hardware_source()
    reset_twin_bridge()


def test_butlerbot_step_becomes_power_feed():
    reset_twin_bridge()
    bridge = get_twin_bridge()
    payload = butlerbot_telemetry_step(1, source="custom", battery_pct=81.0)
    assert bridge.ingest_telemetry(payload)["ok"] is True
    feed = bridge.get_power_feed()
    assert feed is not None
    assert feed.battery_pct == 81.0
    assert feed.channel_draws["Legs"] == 22.0
    assert power_feed_from_telemetry(bridge._last_telemetry).usable()
    reset_twin_bridge()
