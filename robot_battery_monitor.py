# This script analyzes robot battery log data.
import os


class BatteryLogEntry:
    """ this is gonna be awsome""" # Keep your awesome comment!

    def __init__(self, time_stamp, robot_id, battery_level):
        self.time_stamp = time_stamp.strip()
        self.robot_id = robot_id.strip()

        # Check if battery_level is a string and needs stripping
        if isinstance(battery_level, str):
            self.battery_level = int(battery_level.strip('%').strip()) # .strip() again to remove any extra spaces
        else:
            # If it's not a string, assume it's already an int or can be directly converted
            self.battery_level = int(battery_level)
        
    def __str__(self):
        """Return the easy answer for the user to understand what the log entry is."""
        return f"[{self.time_stamp}] Robot id: {self.robot_id} Battery level : {self.battery_level}%"
    
    def is_low_battery(self, threshold=20):
        if self.battery_level <= threshold:
            return True
        else:
            return False
        
    def is_fully_charged(self):
        if self.battery_level == 100:
            return True
        else:
            return False
        
    def get_status_description(self, threshold=20): 
        if self.is_fully_charged(): 
            return "Fully Charged - Ready to Kick Butt!"
        elif self.is_low_battery(threshold): 
            return "Low - Consider Recharging Soon"
        else:
            return "Ready for Work - Monitor Battery" 
    
def read_battery_log(battery_log_path):
    """ Reads Robot battery log and parses each line into BatteryLogEntry object.
    """
    log_entries = []
    print(f" Reading Log file:{battery_log_path}")
    try:
        with open (battery_log_path, 'r') as file:
            for line_num, line in enumerate(file, 1 ):
                line = line.strip() #remove leading and trailing white spcaes. 
                if not line: #skip empty lines
                    continue
                
                parts = line.split(' - ',2)
                
                if len(parts) == 3:
                    time_stamp =  parts[0].strip('[]')
                    robot_id = parts[1].strip(" ")
                    battery_level = parts[2].strip(" ")
                    log_entries.append(BatteryLogEntry(time_stamp,robot_id,battery_level))
                else:
                    print(f"Warning , skipped malformed line {line_num}:{line}")
                    
    except FileNotFoundError:
        print(f" Error: the log file {battery_log_path} not found.")
    except Exception as e:
        print(f"An unnexpected error has occured while reading the log file: {e}")
    return log_entries

# ... (all your BatteryLogEntry class definition and read_battery_log function definition above) ...


# --- Main execution block ---
if __name__ == "__main__":
    # 1. Define the path to your log file
    log_file_path = "robot_battery_log.log" # Assumes log file is in the same directory

    print("\n--- Analyzing Robot Battery Log ---")

    # 2. Call read_battery_log and store the list of BatteryLogEntry objects
    battery_log_data = read_battery_log(log_file_path)

    # 3. Check if any data was read
    if not battery_log_data:
        print("No battery log entries were found or processed.")
    else:
        # 4. Loop through each entry in the list
        for entry in battery_log_data:
            # 5. Print the entry's __str__ representation
            print(entry)

            # 6. Print the status description
            print(f"Status: {entry.get_status_description()}")

            # 7. Print the boolean checks
            print(f"Is Low Battery? {entry.is_low_battery()}") # Uses default threshold (20)
            print(f"Is Fully Charged? {entry.is_fully_charged()}")

            print("-" * 30) # A separator for readability

    print("\n--- Analysis Complete ---")