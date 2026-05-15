import pytest
from robot_battery_monitor import BatteryLogEntry

@pytest.fixture
def battery_entries():
    """
    pytest fixture to provide common BatteryLogEntry objects for tests.
    """
    # Create and return a dictionary of common entry objects
    return {
        "low": BatteryLogEntry("2025-01-01 10:00:00", "R001", 15),
        "full": BatteryLogEntry("2025-01-01 11:00:00", "R002", 100),
        "normal": BatteryLogEntry("2025-01-01 12:00:00", "R003", 75),
        "at_default": BatteryLogEntry("2025-01-01 14:01:00", "R008", 20),
        "below_default": BatteryLogEntry("2025-01-01 14:00:00", "R007", 15),
        "above_default": BatteryLogEntry("2025-01-01 14:02:00", "R009", 21),
        "at_custom": BatteryLogEntry("2025-01-01 15:01:00", "R011", 30),
        "below_custom": BatteryLogEntry("2025-01-01 15:00:00", "R010", 29),
        "above_custom": BatteryLogEntry("2025-01-01 15:02:00", "R012", 31)
    }

# pytest test functions simply start with 'test_'
# ... (battery_entries fixture) ...

def test_is_fully_charged_pytest(battery_entries): # <-- Add battery_entries as argument
    """
    Test that is_fully_charged() returns True for 100% and False otherwise using pytest.
    """
    # Act & Assert (combined in pytest often using direct 'assert')
    assert battery_entries["full"].is_fully_charged() is True
    assert battery_entries["normal"].is_fully_charged() is False
    assert battery_entries["low"].is_fully_charged() is False

def test_normal_battery_status_pytest(battery_entries): # <-- Add battery_entries as argument
    """
    Test that get_status_description correctly identifies a normal battery using pytest.
    """
    # Act:
    status = battery_entries["normal"].get_status_description() # Use the fixture's object

    # Assert:
    expected_status = "Ready for Work - Monitor Battery"
    assert status == expected_status
    
def test_low_battery_status_pytest(battery_entries): # Request the fixture
    """
    Test that get_status_description correctly identifies a low battery using pytest.
    """
    status = battery_entries["low"].get_status_description() # Use fixture object
    expected_status = "Low - Consider Recharging Soon"
    assert status == expected_status

def test_is_low_battery_default_threshold_pytest(battery_entries): # Request the fixture
    """
    Test is_low_battery() with default threshold (20%) using pytest.
    """
    # Using specific entries from the fixture
    assert battery_entries["below_default"].is_low_battery() is True
    assert battery_entries["at_default"].is_low_battery() is True # Updated to True per our earlier discussion
    assert battery_entries["above_default"].is_low_battery() is False

def test_is_low_battery_custom_threshold_pytest(battery_entries): # Request the fixture
    """
    Test is_low_battery() with a custom threshold using pytest.
    """
    custom_threshold = 30 # This specific threshold is still defined here as it's test-specific
    assert battery_entries["below_custom"].is_low_battery(custom_threshold) is True
    assert battery_entries["at_custom"].is_low_battery(custom_threshold) is True # Updated to True
    assert battery_entries["above_custom"].is_low_battery(custom_threshold) is False