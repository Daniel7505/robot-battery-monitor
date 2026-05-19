import warnings
import pytest

# Ignore the thread exception warning caused by forcefully stopping loops in tests
pytestmark = pytest.mark.filterwarnings("ignore::pytest.PytestUnhandledThreadExceptionWarning")
from unittest.mock import patch, MagicMock
from src.dashboard import broadcast_updates
from src.database import log_channel_reading, init_db


def test_broadcast_updates_emits_correct_structure(clean_database):
    """Test that broadcast_updates builds and would emit the expected payload structure."""
    init_db()
    log_channel_reading("Legs", 92, 15)
    log_channel_reading("Arms", 88, 9)

    # We patch socketio.emit so we can inspect what would be sent
    with patch('src.dashboard.socketio.emit') as mock_emit:
        # Call the function once (it normally loops forever)
        # We'll break out after one iteration by patching time.sleep
        with patch('src.dashboard.time.sleep', side_effect=Exception("Stop loop")):
            try:
                broadcast_updates()
            except Exception as e:
                if "Stop loop" not in str(e):
                    raise

        # Check that emit was called
        assert mock_emit.called
        event_name, payload = mock_emit.call_args[0]

        assert event_name == "battery_update"
        assert "main_battery" in payload
        assert "timestamp" in payload
        assert "channels" in payload
        assert isinstance(payload["channels"], list)


def test_broadcast_updates_handles_empty_database(clean_database):
    """Should emit a fallback payload (or at least not crash) when database is empty."""
    with patch('src.dashboard.socketio.emit') as mock_emit:
        with patch('src.dashboard.time.sleep', side_effect=Exception("Stop loop")):
            try:
                broadcast_updates()
            except Exception as e:
                if "Stop loop" not in str(e):
                    raise

        # It should have attempted to emit at least once (even if fallback)
        assert mock_emit.called


def test_broadcast_updates_payload_has_expected_keys(clean_database):
    """The emitted payload should contain the keys the frontend expects."""
    init_db()
    log_channel_reading("Compute", 75, 22)

    with patch('src.dashboard.socketio.emit') as mock_emit:
        with patch('src.dashboard.time.sleep', side_effect=Exception("Stop")):
            try:
                broadcast_updates()
            except Exception:
                pass

        _, payload = mock_emit.call_args[0]

        # These are the keys the frontend JavaScript expects
        assert "main_battery" in payload
        assert "timestamp" in payload
        assert "channels" in payload
        
def test_build_battery_payload_returns_correct_structure(clean_database):
    """_build_battery_payload should return a well-formed payload."""
    from src.dashboard import _build_battery_payload
    from src.database import init_db, log_channel_reading

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
    
def test_broadcast_payload_matches_frontend_expectations(clean_database):
    """
    This test protects the contract between backend and frontend.
    If this test fails, the live dashboard will likely stop updating.
    """
    from src.dashboard import _build_battery_payload
    from src.database import init_db, log_channel_reading

    init_db()
    log_channel_reading("Legs", 88, 14)
    log_channel_reading("Compute", 91, 7)

    payload = _build_battery_payload()

    # These are the exact keys the frontend JavaScript expects
    required_keys = {"main_battery", "timestamp", "channels"}
    assert required_keys.issubset(payload.keys())

    # channels should be a list
    assert isinstance(payload["channels"], list)

    # If there are channels, each should have these keys
    for ch in payload["channels"]:
        assert "id" in ch
        assert "name" in ch
        assert "draw" in ch
        assert "battery" in ch