# run_dashboard.py
"""
Main process entry point for the Robot Battery Monitor / ButlerBot system.

Boot sequence (order matters):
  1. Initialize PostgreSQL schema and analytics views (`init_db`).
  2. Resolve the hardware source via factory (`get_hardware_source`) —
     simulator, ROS2 physics loop, or generic real hardware depending on config.
  3. Start the hardware telemetry thread (background daemon).
  4. Hand control to the Flask/SocketIO dashboard (`run_dashboard`), which
     blocks until the process is stopped.

This is the process operators launch for live monitoring. Offline history
analysis uses `robot_battery_monitor.py` instead.
"""

from src.logger import logger
from src.database import init_db
from src.hardware import get_hardware_source
from src.dashboard import run_dashboard

if __name__ == "__main__":
    logger.info("🤖 Starting Robot Battery Monitoring System")

    try:
        # Schema + views must exist before hardware begins writing snapshots.
        init_db()
        hardware = get_hardware_source()
        hardware.start()
        logger.info("✅ Hardware source started")

        logger.info("🌐 Launching dashboard...")
        # Blocks on Flask/SocketIO event loop until process exit.
        run_dashboard()

    except Exception as e:
        logger.error(f"❌ Failed to start system: {e}", exc_info=True)
        raise
