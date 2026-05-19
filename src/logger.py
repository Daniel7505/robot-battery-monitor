# src/logger.py
import logging
import os
from datetime import datetime
import sys
from src.config import config

def setup_logger(name="robot_monitor"):
    os.makedirs("logs", exist_ok=True)
    
    level_name = config.get("monitoring", "log_level", "INFO")
    level = getattr(logging, level_name.upper(), logging.INFO)
    
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.handlers.clear()

    formatter = logging.Formatter(
        '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s'
    )

    # Daily file handler
    file_handler = logging.FileHandler(
        f"logs/robot_monitor_{datetime.now().strftime('%Y-%m-%d')}.log",
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


logger = setup_logger()