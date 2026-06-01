from src.hardware import get_hardware_source
import time

print("=== Testing ROS2 Hardware Source ===")

source = get_hardware_source()
print(f"Using: {type(source).__name__}")

source.start()

print("Waiting for data to flow...")
time.sleep(10)  # Give it time to run

print("\nLatest readings from ROS2 source:")
for channel, data in source.last_readings.items():
    print(f"  {channel}: {data.get('battery')}% | Draw: {data.get('draw')}W")

print(f"\nHealth: {source.health_status}")