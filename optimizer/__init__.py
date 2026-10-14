"""Overnight Webots loop: diagnose, pick one allowlisted change, run, score.

Webots itself is Windows-only. Everything imported here (diagnosis, scoring,
logging, the rule picker) runs without a simulator.
"""

from optimizer.loop import run_loop

__all__ = ["run_loop"]
