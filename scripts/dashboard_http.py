"""JSON HTTP helpers shared by the dashboard drive scripts. Stdlib only.

``_req`` defaults to a 5 s timeout, matching ``agent_short_drive`` and
``agent_extended_drive``. ``twin`` and ``cmd`` use 8 s, matching
``measure_track_run``.
"""

from __future__ import annotations

import json
import os
import urllib.request


def _req(url: str, method: str = "GET", body: dict | None = None, timeout: float = 5.0) -> dict:
    """Minimal JSON HTTP helper (stdlib only — no requests dependency)."""
    data = None
    headers = {}
    if os.environ.get("RBM_API_TOKEN"):
        headers["X-API-Token"] = os.environ["RBM_API_TOKEN"]
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def twin(base: str) -> dict:
    """GET ``/api/twin/state``."""
    return _req(f"{base}/api/twin/state", timeout=8.0)


def cmd(base: str, body: dict) -> dict:
    """POST ``/api/twin/command``."""
    return _req(f"{base}/api/twin/command", method="POST", body=body, timeout=8.0)
