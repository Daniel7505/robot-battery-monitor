import unittest
# We need to import the class we want to test from our main script
from robot_battery_monitor import BatteryLogEntry

class TestBatteryLogEntry(unittest.TestCase):
    """
    Tests for the BatteryLogEntry class in robot_battery_monitor.py
    """

    def test_low_battery_status(self):
        """
        Test that get_status_description correctly identifies a low battery.
        """
        # Arrange: Set up the data needed for the test
        entry = BatteryLogEntry("2025-01-01 10:00:00", "R001", 15) # Battery level 15 (low)

        # Act: Call the method we want to test
        status = entry.get_status_description()

        # Assert: Check if the actual result matches the expected result
        expected_status = "Low - Consider Recharging Soon"
        self.assertEqual(status, expected_status)

    def test_full_battery_status(self):
        """
        Test that get_status_description correctly identifies a full battery.
        """
        entry = BatteryLogEntry("2025-01-01 11:00:00", "R002", 100) # Battery level 100 (full)
        status = entry.get_status_description()
        expected_status = "Fully Charged - Ready to Kick Butt!"
        self.assertEqual(status, expected_status)

    def test_normal_battery_status(self):
        """
        Test that get_status_description correctly identifies a normal battery.
        """
        entry = BatteryLogEntry("2025-01-01 12:00:00", "R003", 75) # Battery level 75 (normal)
        status = entry.get_status_description()
        expected_status = "Ready for Work - Monitor Battery"
        self.assertEqual(status, expected_status)

# This part makes the tests runnable directly from the command line
if __name__ == '__main__':
    unittest.main()