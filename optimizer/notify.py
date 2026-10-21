"""Optional webhook when an overnight run finishes."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from pathlib import Path


def notify_run_finished(
    url: str | None,
    *,
    status: str,
    iterations: int,
    report_path: Path | str | None,
    summary: str,
) -> None:
    """POST a small JSON body. Never raises; no-op when ``url`` is empty."""
    target = (url or "").strip()
    if not target:
        return
    payload = {
        "status": status,
        "iterations": int(iterations),
        "report_path": "" if report_path is None else str(report_path),
        "summary": summary,
    }
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        target,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            resp.read()
    except (OSError, urllib.error.URLError, urllib.error.HTTPError, ValueError):
        return
