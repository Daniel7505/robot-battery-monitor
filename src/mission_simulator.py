"""
Backward-compatible alias for the ButlerBot simulation driver.

Historical name ``MissionSimulator`` now points at ``SimulationDriver`` in
``src.simulation_driver``. Prefer importing SimulationDriver directly in new code.
"""

from src.simulation_driver import SimulationDriver as MissionSimulator

__all__ = ["MissionSimulator"]
