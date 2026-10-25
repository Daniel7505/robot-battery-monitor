"""Held-out confirm pair helpers."""

from __future__ import annotations

from optimizer.bad_lap import has_phrase

MAX_CONFIRM_PAIR_RETRIES = 1


def confirm_invalid_retryable(reason: str) -> bool:
    """True when the confirm pair should be retried instead of refuting the keep."""
    text = reason or ""
    if not text.strip():
        return False
    if has_phrase(text, "turn_err", "turn segment", "segment_energy"):
        return True
    if has_phrase(text, "stale", "opt_run", "opt_trial"):
        return True
    if has_phrase(text, "45s", "instant", "cached", "short"):
        return True
    if has_phrase(text, "no mission", "no summary", "missing opt_"):
        return True
    if has_phrase(text, "launcher exited", "crash", "timeout"):
        return True
    if "config" in text.lower() and has_phrase(text, "before launch"):
        return True
    return False
