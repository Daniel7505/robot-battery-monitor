# Robot Battery Monitor Project

This project contains a Python script to analyze robot battery log data, identify battery status (low, full, normal), and provide a clear report.

## Features

* Parses a simple text log file (`robot_battery_log.log`) containing battery information.
* Creates `BatteryLogEntry` objects for each log entry, encapsulating data.
* Identifies if a robot's battery is critically low, fully charged, or in a normal operating range.
* Generates a human-readable status description for each log entry.

## How to Run

1.  **Ensure Python is installed:** This project requires Python 3.x.
2.  **Save files:** Place `robot_battery_monitor.py` and `robot_battery_log.log` in the same directory.
3.  **Run from terminal:**
    Open your terminal or command prompt, navigate to the project directory, and run the script using:
    ```bash
    python robot_battery_monitor.py
    ```

## Technologies Used

* Python 3.x

## Author

* Daniel Dew