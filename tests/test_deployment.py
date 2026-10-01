"""Deployment configuration and script validation."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_deployment_files_exist():
    required = [
        "docker-compose.yml",
        "Dockerfile",
        ".env.example",
        ".dockerignore",
        "scripts/docker-entrypoint.sh",
        "scripts/wait_for_postgres.py",
        "scripts/ros2_sim_node.py",
        "scripts/ros2_sim_entrypoint.sh",
        "scripts/start.sh",
        "scripts/start.ps1",
        "scripts/stop.sh",
        "scripts/stop.ps1",
    ]
    for rel in required:
        assert (ROOT / rel).is_file(), f"Missing deployment file: {rel}"


def test_docker_compose_structure():
    compose = yaml.safe_load((ROOT / "docker-compose.yml").read_text(encoding="utf-8"))
    services = compose.get("services", {})
    assert "postgres" in services
    assert "dashboard" in services
    assert "ros2-sim" in services
    assert services["ros2-sim"].get("profiles") == ["full"]

    dashboard = services["dashboard"]
    assert dashboard.get("depends_on", {}).get("postgres", {}).get("condition") == "service_healthy"
    env = dashboard.get("environment", {})
    assert "DATABASE_URL" in env
    assert env.get("ROS2_MOCK") in ("true", "${ROS2_MOCK:-true}")


def test_env_example_documents_core_vars():
    content = (ROOT / ".env.example").read_text(encoding="utf-8")
    for key in (
        "POSTGRES_USER",
        "DATABASE_URL",
        "HARDWARE_MODE",
        "HARDWARE_TYPE",
        "ROS2_MOCK",
        "DASHBOARD_PORT",
    ):
        assert key in content


def test_ros2_mock_env_override():
    os.environ["ROS2_MOCK"] = "true"
    try:
        from src.config import Config

        cfg = Config()
        assert cfg.get("hardware", "ros2", {}).get("mock") is True
    finally:
        os.environ.pop("ROS2_MOCK", None)


@pytest.mark.skipif(shutil.which("docker") is None, reason="docker CLI not on PATH")
def test_docker_compose_config_valid():
    result = subprocess.run(
        ["docker", "compose", "config"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    stderr = (result.stderr or "").lower()
    # No .env yet (fresh clone / CI): seed it from .env.example like start.sh does.
    # Compose fails either on the missing env_file or on the required
    # POSTGRES_PASSWORD interpolation, depending on which it checks first.
    if result.returncode != 0 and ("env file" in stderr or "required variable" in stderr):
        example = ROOT / ".env.example"
        target = ROOT / ".env"
        if not target.exists() and example.exists():
            target.write_text(example.read_text(encoding="utf-8"), encoding="utf-8")
        result = subprocess.run(
            ["docker", "compose", "config"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=30,
        )
    assert result.returncode == 0, result.stderr or result.stdout