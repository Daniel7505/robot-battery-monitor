import pytest
from unittest.mock import patch
from src.database import init_db, log_channel_reading
from robot_battery_monitor import print_summary, show_channel_history


def test_print_summary_outputs_data(capsys, clean_database):
    """print_summary should output battery info when data exists."""
    init_db()
    log_channel_reading("Legs", 85, 20)
    log_channel_reading("Arms", 78, 12)

    print_summary()

    captured = capsys.readouterr()
    output = captured.out

    assert "Robot Battery Monitor" in output
    assert "Legs" in output or "Arms" in output
    assert "%" in output


def test_show_channel_history_outputs_data(capsys, clean_database):
    """show_channel_history should print history for a valid channel."""
    init_db()
    log_channel_reading("Compute", 92, 8)

    show_channel_history("Compute", limit=5)

    captured = capsys.readouterr()
    output = captured.out

    assert "History for Compute" in output or "Compute" in output


def test_show_channel_history_handles_unknown_channel(capsys):
    """Should handle a channel that has no data."""
    show_channel_history("NonExistentChannel", limit=5)

    captured = capsys.readouterr()
    output = captured.out

    assert "No history" in output or "NonExistentChannel" in output


def test_cli_archive_command(monkeypatch, clean_database):
    """Test that calling archive via main() works without crashing."""
    from robot_battery_monitor import main

    init_db()
    log_channel_reading("Torso", 70, 5)

    # Simulate running: python robot_battery_monitor.py --archive
    monkeypatch.setattr("sys.argv", ["robot_battery_monitor.py", "--archive"])

    # We just want to make sure it doesn't crash
    try:
        main()
    except SystemExit:
        pass  # argparse sometimes calls sys.exit