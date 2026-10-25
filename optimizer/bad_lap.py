"""Phrases that mean a lap did not produce a scoreable summary.

One list. Confirm-pair retry and the postmortem lesson test the phrases
that match their wording. The overnight loop does not.

PR #27: a bad lap is not a reason to start Webots again. ``_retry_reason``
relaunches only when the launcher process itself exits (or raises
Error/Exception). A stale, cached, or short summary stays a single attempt.
That second start is what killed trials 2–6 of run 10 within a minute.
"""

from __future__ import annotations

BAD_LAP_PHRASES = (
    "turn_err",
    "turn segment",
    "segment_energy",
    "stale",
    "opt_run",
    "opt_trial",
    "45s",
    "instant",
    "cached",
    "cached summary",
    "short",
    "too quickly",
    "too short",
    "(<",
    "no mission",
    "no summary",
    "missing opt_",
    "launcher exited",
    "crash",
    "timeout",
    "before launch",
)


def has_phrase(text: str, *phrases: str) -> bool:
    """True when ``text`` contains one of ``phrases`` (case-insensitive).

    Every phrase has to be in :data:`BAD_LAP_PHRASES`.
    """
    unknown = [phrase for phrase in phrases if phrase not in BAD_LAP_PHRASES]
    if unknown:
        raise KeyError(unknown)
    lowered = (text or "").lower()
    return any(phrase in lowered for phrase in phrases)
