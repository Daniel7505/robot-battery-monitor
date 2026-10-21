"""Held-out confirm pair helpers."""

from __future__ import annotations

MAX_CONFIRM_PAIR_RETRIES = 1


def confirm_invalid_retryable(reason: str) -> bool:
    """True when the confirm pair should be retried instead of refuting the keep."""
    text = (reason or "").lower()
    if not text:
        return False
    if "turn_err" in text or "turn segment" in text or "segment_energy" in text:
        return True
    if "stale" in text or "opt_run" in text or "opt_trial" in text:
        return True
    if "45s" in text or "instant" in text or "cached" in text or "short" in text:
        return True
    if "no mission" in text or "no summary" in text or "missing opt_" in text:
        return True
    if "launcher exited" in text or "crash" in text or "timeout" in text:
        return True
    if "config" in text and "before launch" in text:
        return True
    return False
