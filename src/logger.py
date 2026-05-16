import logging
import os
from datetime import datetime
import sys

def setup_logger():
    os.makedirs("logs", exist_ok=True)
    
    logger = logging.getLogger("robot_monitor")
    logger.setLevel(logging.INFO)
    
    # Remove any existing handlers
    logger.handlers.clear()
    
    # File handler (always safe)
    file_handler = logging.FileHandler(f"logs/robot_monitor_{datetime.now().strftime('%Y-%m-%d')}.log", encoding='utf-8')
    file_handler.setFormatter(logging.Formatter(
        '%(asctime)s | %(levelname)s | %(message)s'
    ))
    
    # Console handler with safe encoding
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(logging.Formatter(
        '%(asctime)s | %(levelname)s | %(message)s'
    ))
    
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

logger = setup_logger()