import pytest
import os
import tempfile
import shutil
from src.config import Config
from src.database import init_db, get_db_connection


@pytest.fixture(scope="session")
def test_config():
    """Provides a loaded config instance for tests."""
    return Config()


@pytest.fixture
def clean_database(tmp_path, monkeypatch):
    """
    Creates a fresh, isolated database for each test.
    This prevents tests from polluting the real logs/robot_battery.db
    """
    # Create a temporary directory for this test
    temp_dir = tmp_path / "logs"
    temp_dir.mkdir()

    temp_db = temp_dir / "robot_battery.db"

    # Monkeypatch the DB_PATH so all database functions use the temp one
    import src.database as db_module
    monkeypatch.setattr(db_module, "DB_PATH", str(temp_db))

    # Initialize a clean database
    init_db()

    yield str(temp_db)

    # Cleanup happens automatically because tmp_path is deleted after test