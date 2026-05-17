# run_dashboard.py
from src.dashboard import run_dashboard
from src.hardware import get_hardware_source

if __name__ == "__main__":
    print("🤖 Starting Robot Battery Monitoring System\n")

    # Start the correct hardware source (simulator or real)
    hardware = get_hardware_source()
    hardware.start()

    # Start the web dashboard
    run_dashboard()