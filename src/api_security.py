"""
HTTP hardening for the PMS dashboard: bind host, CORS allowlist, API token.

Secure by default, frictionless for local dev:

* **Bind host** — ``DASHBOARD_HOST`` env, else ``dashboard.host`` in config,
  else ``127.0.0.1``. Docker sets ``DASHBOARD_HOST=0.0.0.0`` inside the
  container and compose publishes the port on ``127.0.0.1`` only.
* **CORS (SocketIO)** — ``RBM_CORS_ORIGINS`` (comma list), else
  ``dashboard.cors_origins``, else ``http://127.0.0.1:<port>`` and
  ``http://localhost:<port>``.
* **API token** — optional ``RBM_API_TOKEN``. When set, every POST / PUT /
  PATCH / DELETE needs ``X-API-Token: <token>`` (Webots controller, scripts)
  **or** the ``rbm_api_token`` cookie (browser UI). The browser gets the
  cookie by opening ``/?token=<token>`` once; it is HttpOnly + SameSite=Strict
  so other sites cannot ride it (CSRF). When unset, writes are allowed and a
  one-time warning is logged.
* **Localhost-only routes** — ``localhost_only`` rejects any client whose
  ``remote_addr`` is not 127.0.0.1 / ::1 (e.g. host-side Webots launch).
"""

from __future__ import annotations

import functools
import hmac
import os
import threading

from flask import jsonify, redirect, request

from src.config import config
from src.logger import logger

TOKEN_ENV = "RBM_API_TOKEN"
TOKEN_HEADER = "X-API-Token"
TOKEN_COOKIE = "rbm_api_token"
TOKEN_QUERY = "token"
WRITE_METHODS = frozenset({"POST", "PUT", "PATCH", "DELETE"})
LOCALHOST_ADDRS = frozenset({"127.0.0.1", "::1", "::ffff:127.0.0.1"})

_warned_lock = threading.Lock()
_warned_no_token = False


def api_token() -> str | None:
    """Configured shared token, or None when auth is disabled (local dev)."""
    tok = (os.getenv(TOKEN_ENV) or "").strip()
    return tok or None


def bind_host() -> str:
    """Interface the HTTP server listens on (default loopback only)."""
    env = (os.getenv("DASHBOARD_HOST") or "").strip()
    if env:
        return env
    return str(config.get("dashboard", "host", "127.0.0.1") or "127.0.0.1")


def allowed_origins() -> list[str]:
    """Browser origins allowed to open the SocketIO connection."""
    env = os.getenv("RBM_CORS_ORIGINS")
    if env is not None and env.strip():
        return [o.strip().rstrip("/") for o in env.split(",") if o.strip()]
    cfg = config.get("dashboard", "cors_origins", None)
    if isinstance(cfg, (list, tuple)) and cfg:
        return [str(o).strip().rstrip("/") for o in cfg if str(o).strip()]
    port = config.get("dashboard", "port", 5000)
    return [f"http://127.0.0.1:{port}", f"http://localhost:{port}"]


def _tokens_match(provided: str | None, expected: str) -> bool:
    if not provided:
        return False
    return hmac.compare_digest(provided.encode("utf-8"), expected.encode("utf-8"))


def _warn_once_no_token() -> None:
    global _warned_no_token
    if _warned_no_token:
        return
    with _warned_lock:
        if not _warned_no_token:
            _warned_no_token = True
            logger.warning(
                "%s is not set — state-changing API endpoints are UNAUTHENTICATED. "
                "Fine for local dev on 127.0.0.1; set %s to require the %s header.",
                TOKEN_ENV, TOKEN_ENV, TOKEN_HEADER,
            )


def check_write_auth():
    """``before_request`` hook. Returns a Flask response to block, else None."""
    expected = api_token()

    # Browser bootstrap: /?token=<t> sets the UI cookie, then strips the token
    # from the URL so it does not linger in history / screenshots.
    if request.method == "GET" and request.path == "/" and TOKEN_QUERY in request.args:
        if expected and _tokens_match(request.args.get(TOKEN_QUERY), expected):
            resp = redirect("/", code=302)
            resp.set_cookie(
                TOKEN_COOKIE, expected, httponly=True, samesite="Strict", path="/"
            )
            return resp
        return None  # wrong / unused token: just render the page (UI stays locked)

    if request.method not in WRITE_METHODS:
        return None
    if expected is None:
        _warn_once_no_token()
        return None

    if _tokens_match(request.headers.get(TOKEN_HEADER), expected):
        return None
    if _tokens_match(request.cookies.get(TOKEN_COOKIE), expected):
        # Cookie auth is browser-only; refuse foreign origins as defense in depth.
        origin = (request.headers.get("Origin") or "").rstrip("/")
        if not origin or origin in allowed_origins():
            return None
        return jsonify({"ok": False, "error": "origin not allowed"}), 403
    return jsonify({
        "ok": False,
        "error": f"missing or invalid {TOKEN_HEADER}",
        "errors": [f"unauthorized: send {TOKEN_HEADER} or open /?token=... once"],
    }), 401


def localhost_only(view):
    """Reject requests whose client address is not loopback."""

    @functools.wraps(view)
    def wrapper(*args, **kwargs):
        if request.remote_addr not in LOCALHOST_ADDRS:
            return jsonify({
                "ok": False,
                "launched": False,
                "error": "localhost only",
                "launch_message": (
                    "This action is only allowed from the same machine (127.0.0.1). "
                    "If the dashboard runs in Docker, run scripts/launch_webots_twin.ps1 "
                    "(or .sh) on the host instead."
                ),
            }), 403
        return view(*args, **kwargs)

    return wrapper


def init_app(app) -> None:
    """Register the auth hook on a Flask app."""
    app.before_request(check_write_auth)
