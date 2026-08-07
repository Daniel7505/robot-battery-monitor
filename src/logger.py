# src/logger.py
"""
Process logging setup for the Robot Battery Monitor.

Role
----
Creates a named logger (``robot_monitor``) with dual handlers:
  - Daily rotating file under ``logs/robot_monitor_YYYY-MM-DD.log``
  - Console (stdout) for Docker / terminal operators

Log level comes from ``config.monitoring.log_level`` (default INFO).

Outputs
-------
Module-level ``logger`` is imported everywhere (``from src.logger import logger``).
Calling ``setup_logger`` again replaces handlers (idempotent clear + re-add).
"""

import logging
import os
from datetime import datetime
import sys
from src.config import config


def setup_logger(name="robot_monitor"):
    """Configure and return the application logger with file + console handlers."""
    os.makedirs("logs", exist_ok=True)

    level_name = config.get("monitoring", "log_level", "INFO")
    level = getattr(logging, level_name.upper(), logging.INFO)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    # Clear any prior handlers so reloads / tests don't duplicate output lines.
    logger.handlers.clear()

    formatter = logging.Formatter(
        '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s'
    )

    # One file per calendar day — simple ops retention without extra deps.
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
