"""Expected BoM body mass for the pinned optimizer run."""

from __future__ import annotations

MASS_TOLERANCE_KG = 0.06


def expected_body_mass_kg(run_body: str) -> float | None:
    try:
        from src.webots_body import bom_body_physics
    except ImportError:
        return None
    physics = bom_body_physics(run_body or None)
    if physics is None:
        return None
    return float(physics.mass_kg)


def mass_matches_run(summary, run_body: str) -> tuple[bool, str | None]:
    expected = expected_body_mass_kg(run_body)
    observed = None if summary is None else summary.body_mass_kg
    if expected is None:
        return True, None
    if observed is None:
        return False, f"lap log is missing body_mass_kg (expected {expected:g} kg for {run_body})"
    if abs(float(observed) - expected) > MASS_TOLERANCE_KG:
        return (
            False,
            f"body_mass_kg={observed:g} does not match {run_body} ({expected:g} kg expected)",
        )
    return True, None
