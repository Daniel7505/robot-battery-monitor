# run_dashboard.py
from src.logger import logger
from src.database import init_db
from src.hardware import get_hardware_source
from src.dashboard import run_dashboard

if __name__ == "__main__":
    logger.info("🤖 Starting Robot Battery Monitoring System")
    
    try:
        init_db()
        hardware = get_hardware_source()
        hardware.start()
        logger.info("✅ Hardware source started")
        
        logger.info("🌐 Launching dashboard...")
        run_dashboard()
        
    except Exception as e:
        logger.error(f"❌ Failed to start system: {e}", exc_info=True)
        raise
    