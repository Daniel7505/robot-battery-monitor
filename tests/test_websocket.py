"""Live payload contract. The browser polls /api/live; nothing emits a socket."""

import json

import pytest

from src.database import init_db, log_channel_reading


def test_build_battery_payload_returns_correct_structure(clean_database):
    from src.dashboard import _build_battery_payload

    init_db()
    log_channel_reading("Legs", 91, 18)
    log_channel_reading("Arms", 85, 12)

    payload = _build_battery_payload()

    assert isinstance(payload, dict)
    assert "main_battery" in payload
    assert "timestamp" in payload
    assert "channels" in payload
    assert isinstance(payload["channels"], list)
    assert payload["main_battery"] >= 0
    assert "live" in payload
    assert "proof" in payload


def test_broadcast_payload_matches_frontend_expectations(clean_database):
    from src.dashboard import _build_battery_payload

    init_db()
    log_channel_reading("Legs", 88, 14)
    log_channel_reading("Compute", 91, 7)

    payload = _build_battery_payload()

    required_keys = {"main_battery", "timestamp", "channels", "live", "proof"}
    assert required_keys.issubset(payload.keys())
    assert isinstance(payload["channels"], list)
    for ch in payload["channels"]:
        assert "id" in ch
        assert "name" in ch
        assert "draw" in ch
        assert "battery" in ch
    assert payload["live"]["speed_m_s"] is None or payload["live"]["active"]


def test_api_live_matches_battery_payload(clean_database):
    from src.dashboard import app

    with app.test_client() as client:
        resp = client.get("/api/live")
    assert resp.status_code == 200
    data = resp.get_json()
    assert "main_battery" in data
    assert "timestamp" in data
    assert "channels" in data
    assert "live" in data
    assert "proof" in data
    assert isinstance(data["channels"], list)


def test_api_live_reads_speed_and_steer_from_the_bridge():
    from src.dashboard import app
    from src.hardware import reset_hardware_source
    from src.twin import get_twin_bridge, reset_twin_bridge

    reset_twin_bridge()
    reset_hardware_source()
    bridge = get_twin_bridge()
    result = bridge.ingest_telemetry(
        {
            "source": "webots",
            "adapter": "webots",
            "robot": {"name": "ButlerBot", "main_battery_pct": 90},
            "mission": {"task": "moving"},
            "channel_draws": {"Legs": 10, "Arms": 1, "Torso": 1, "Compute": 9},
            "locomotion": {
                "gait": "drive",
                "speed_m_s": 0.44,
                "phase": "teleop",
                "cross_track_m": 0.032,
            },
            "pose": {"x_m": 1.5, "y_m": -0.2},
            "sensors": {"steer": 0.15, "lane_keep": True, "cross_track_m": 0.032},
        },
        adapter="webots",
    )
    assert result["ok"] is True

    with app.test_client() as client:
        data = client.get("/api/live").get_json()
    assert data["live"]["active"] is True
    assert data["live"]["speed_m_s"] == pytest.approx(0.44)
    assert data["live"]["steer"] == pytest.approx(0.15)
    assert data["live"]["pose"]["x_m"] == pytest.approx(1.5)
    assert data["live"]["phase_label"] == "Teleop Drive"
    assert data["proof"]["ok"] is True
    assert data["proof"]["source"] == "This lap"
    assert data["proof"]["ct_max_m"] == pytest.approx(0.032)

    reset_twin_bridge()
    with app.test_client() as client:
        idle = client.get("/api/live").get_json()
    assert idle["live"]["active"] is False
    assert idle["live"]["speed_m_s"] is None
    reset_hardware_source()


def test_page_does_not_pretend_to_be_a_websocket():
    from src.dashboard import HTML_TEMPLATE
    import src.dashboard as dashboard

    assert not hasattr(dashboard, "broadcast_updates")
    assert "cdn.socket.io" not in HTML_TEMPLATE
    assert "Real-time via WebSocket" not in HTML_TEMPLATE
    assert "setInterval(pullLive, 500)" in HTML_TEMPLATE
    assert "liveInFlight" in HTML_TEMPLATE
    assert 'id="proof-panel"' in HTML_TEMPLATE
    assert "analytics-panel" not in HTML_TEMPLATE
    assert "nadir" not in HTML_TEMPLATE.lower()


def test_latest_mission_summary_file_fills_proof(tmp_path):
    from src.mission_summary_file import latest_proof_from_disk

    line = (
        "MISSION_SUMMARY build=light err_pct=-4.2 "
        "wh_measured_drive_elec=1.20 wh_model_drive=1.25 "
        "steer_frac_camera=0.80 steer_frac_held=0.10 steer_frac_track=0.10 "
        "ct_max_m=0.041 width_err_rms_m=0.012 width_err_max_m=0.020 "
        "center_err_rms_m=0.008 center_err_max_m=0.015"
    )
    (tmp_path / "mission_summary_test.json").write_text(
        json.dumps({"lines": [line]}), encoding="utf-8"
    )
    proof = latest_proof_from_disk(str(tmp_path))
    assert proof["err_pct"] == pytest.approx(-4.2)
    assert proof["wh_measured_drive_elec"] == pytest.approx(1.20)
    assert proof["steer_frac_camera"] == pytest.approx(0.80)
    assert proof["steer_frac_held"] == pytest.approx(0.10)
    assert proof["steer_frac_track"] == pytest.approx(0.10)
    assert proof["ct_max_m"] == pytest.approx(0.041)
    assert proof["width_err_rms_m"] == pytest.approx(0.012)
    assert proof["center_err_rms_m"] == pytest.approx(0.008)


def test_db_connect_passes_a_timeout(monkeypatch):
    import psycopg2

    from src.database import get_db_connection

    captured = {}

    def fake_connect(*args, **kwargs):
        captured["args"] = args
        captured["kwargs"] = kwargs
        raise RuntimeError("no db")

    monkeypatch.setattr(psycopg2, "connect", fake_connect)
    with pytest.raises(RuntimeError):
        get_db_connection()
    assert captured["kwargs"].get("connect_timeout") == 5
