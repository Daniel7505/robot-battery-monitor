import unittest
from robot_battery_monitor import BatteryLogEntry

class TestBatteryLogEntry(unittest.TestCase):
    """
    Tests for the BatteryLogEntry class in robot_battery_monitor.py
    """

    # --- NEW: setUp method ---
    def setUp(self):
        """
        Set up common BatteryLogEntry objects for use in tests.
        This method runs before *each* test method.
        """
        self.entry_low = BatteryLogEntry("2025-01-01 10:00:00", "R001", 15)
        self.entry_full = BatteryLogEntry("2025-01-01 11:00:00", "R002", 100)
        self.entry_normal = BatteryLogEntry("2025-01-01 12:00:00", "R003", 75)
        self.entry_at_default_threshold = BatteryLogEntry("2025-01-01 14:01:00", "R008", 20)
        self.entry_below_default_threshold = BatteryLogEntry("2025-01-01 14:00:00", "R007", 15)
        self.entry_above_default_threshold = BatteryLogEntry("2025-01-01 14:02:00", "R009", 21)
        self.entry_at_custom_threshold = BatteryLogEntry("2025-01-01 15:01:00", "R011", 30)
        self.entry_below_custom_threshold = BatteryLogEntry("2025-01-01 15:00:00", "R010", 29)
        self.entry_above_custom_threshold = BatteryLogEntry("2025-01-01 15:02:00", "R012", 31)

    # --- MODIFIED: Existing test methods ---

    def test_low_battery_status(self):
        """
        Test that get_status_description correctly identifies a low battery.
        """
        # Arrange is now mostly done in setUp!
        status = self.entry_low.get_status_description() # Use self.entry_low
        expected_status = "Low - Consider Recharging Soon"
        self.assertEqual(status, expected_status)

    def test_full_battery_status(self):
        """
        Test that get_status_description correctly identifies a full battery.
        """
        status = self.entry_full.get_status_description() # Use self.entry_full
        expected_status = "Fully Charged - Ready to Kick Butt!"
        self.assertEqual(status, expected_status)

    def test_normal_battery_status(self):
        """
        Test that get_status_description correctly identifies a normal battery.
        """
        status = self.entry_normal.get_status_description() # Use self.entry_normal
        expected_status = "Ready for Work - Monitor Battery"
        self.assertEqual(status, expected_status)

    def test_is_fully_charged(self):
        """
        Test that is_fully_charged() returns True for 100% and False otherwise.
        """
        self.assertTrue(self.entry_full.is_fully_charged())
        self.assertFalse(self.entry_normal.is_fully_charged()) # Use self.entry_normal
        self.assertFalse(self.entry_low.is_fully_charged()) # Use self.entry_low

    def test_is_low_battery_default_threshold(self):
        """
        Test is_low_battery() with default threshold (20%).
        """
        # Arrange is now mostly done in setUp!
        self.assertTrue(self.entry_below_default_threshold.is_low_battery())
        self.assertTrue(self.entry_at_default_threshold.is_low_battery()) # Corrected earlier
        self.assertFalse(self.entry_above_default_threshold.is_low_battery())

    def test_is_low_battery_custom_threshold(self):
        """
        Test is_low_battery() with a custom threshold.
        """
        custom_threshold = 30 # This still needs to be defined within the test if specific to it
        self.assertTrue(self.entry_below_custom_threshold.is_low_battery(custom_threshold))
        self.assertTrue(self.entry_at_custom_threshold.is_low_battery(custom_threshold)) # Corrected earlier
        self.assertFalse(self.entry_above_custom_threshold.is_low_battery(custom_threshold))

# This part makes the tests runnable directly from the command line
if __name__ == '__main__':
    unittest.main()