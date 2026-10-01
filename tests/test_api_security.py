"""API hardening: optional RBM_API_TOKEN on writes, localhost-only launch, CORS, bind host."""

import pytest

from src import api_security
from src.dashboard import app, socketio

TOKEN = "test-token-123"
LAN = {"REMOTE_ADDR": "192.168.1.50"}


@pytest.fixture
def client(monkeypatch):
    monkeypatch.delenv("RBM_API_TOKEN", raising=False)
    app.config["TESTING"] = True
    with app.test_client() as c:
        yield c


@pytest.fixture
def token(monkeypatch):
    monkeypatch.setenv("RBM_API_TOKEN", TOKEN)
    return TOKEN


# --- token unset: frictionless local dev -------------------------------------

def test_writes_allowed_without_token_configured(client):
    assert client.post("/api/demo/deactivate").status_code == 200


def test_missing_token_warns_only_once(client, monkeypatch):
    calls = []
    monkeypatch.setattr(api_security, "_warned_no_token", False)
    monkeypatch.setattr(api_security.logger, "warning", lambda *a, **k: calls.append(a))
    client.post("/api/demo/deactivate")
    client.post("/api/demo/deactivate")
    assert len(calls) == 1


# --- token set ---------------------------------------------------------------

@pytest.mark.parametrize(
    "path",
    [
        "/api/twin/command",
        "/api/twin/telemetry",
        "/api/simulation/start",
        "/api/measurements",
        "/api/demo/activate",
        "/api/demo/deactivate",
        "/api/demo/launch-webots",
    ],
)
def test_write_endpoints_reject_missing_token(client, token, path):
    resp = client.post(path, json={})
    assert resp.status_code == 401
    assert resp.get_json()["ok"] is False


@pytest.mark.parametrize("method", ["put", "patch", "delete"])
def test_other_write_methods_are_gated(client, token, method):
    # Route only allows POST; the auth hook must still answer first.
    assert getattr(client, method)("/api/twin/command").status_code == 401


def test_wrong_token_rejected(client, token):
    resp = client.post("/api/demo/deactivate", headers={"X-API-Token": "nope"})
    assert resp.status_code == 401


def test_correct_header_accepted(client, token):
    resp = client.post("/api/demo/deactivate", headers={"X-API-Token": TOKEN})
    assert resp.status_code == 200


def test_reads_do_not_need_token(client, token):
    assert client.get("/api/demo/status").status_code == 200


def test_browser_bootstrap_sets_strict_httponly_cookie(client, token):
    resp = client.get(f"/?token={TOKEN}")
    assert resp.status_code == 302
    assert resp.headers["Location"].endswith("/")
    cookie = resp.headers["Set-Cookie"]
    assert f"{api_security.TOKEN_COOKIE}={TOKEN}" in cookie
    assert "HttpOnly" in cookie
    assert "SameSite=Strict" in cookie


def test_bootstrap_with_wrong_token_sets_no_cookie(client, token):
    resp = client.get("/?token=wrong")
    assert "Set-Cookie" not in resp.headers


def test_cookie_authorizes_same_origin_ui(client, token):
    client.set_cookie(api_security.TOKEN_COOKIE, TOKEN)
    resp = client.post(
        "/api/demo/deactivate", headers={"Origin": "http://127.0.0.1:5000"}
    )
    assert resp.status_code == 200


def test_cookie_refused_from_foreign_origin(client, token):
    client.set_cookie(api_security.TOKEN_COOKIE, TOKEN)
    resp = client.post("/api/demo/deactivate", headers={"Origin": "http://evil.example"})
    assert resp.status_code == 403


# --- localhost-only launch ---------------------------------------------------

def test_launch_webots_refused_from_non_localhost(client):
    resp = client.post("/api/demo/launch-webots", environ_base=LAN)
    assert resp.status_code == 403
    body = resp.get_json()
    assert body["launched"] is False
    assert "localhost" in body["error"]


def test_launch_webots_refused_remote_even_with_token(client, token):
    resp = client.post(
        "/api/demo/launch-webots", headers={"X-API-Token": TOKEN}, environ_base=LAN
    )
    assert resp.status_code == 403


def test_launch_webots_allowed_from_localhost(client, monkeypatch):
    import src.demo_mode as demo_mode

    monkeypatch.setattr(demo_mode, "launch_webots_on_host", lambda: {"ok": True, "launched": False})
    resp = client.post("/api/demo/launch-webots", environ_base={"REMOTE_ADDR": "127.0.0.1"})
    assert resp.status_code == 200
    client.post("/api/demo/deactivate")


# --- CORS + bind host --------------------------------------------------------

def test_default_cors_origins_are_loopback_only(monkeypatch):
    monkeypatch.delenv("RBM_CORS_ORIGINS", raising=False)
    origins = api_security.allowed_origins()
    assert "*" not in origins
    port = api_security.config.get("dashboard", "port", 5000)
    assert set(origins) == {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}


def test_cors_origins_env_override(monkeypatch):
    monkeypatch.setenv("RBM_CORS_ORIGINS", "http://a.test:5000, http://b.test:5000/")
    assert api_security.allowed_origins() == ["http://a.test:5000", "http://b.test:5000"]


def test_socketio_uses_allowlist_not_wildcard():
    configured = socketio.server.eio.cors_allowed_origins
    assert configured != "*"
    assert "http://127.0.0.1:5000" in configured
    assert "http://localhost:5000" in configured


def test_bind_host_defaults_to_loopback(monkeypatch):
    monkeypatch.delenv("DASHBOARD_HOST", raising=False)
    assert api_security.bind_host() == "127.0.0.1"


def test_bind_host_env_override(monkeypatch):
    monkeypatch.setenv("DASHBOARD_HOST", "0.0.0.0")
    assert api_security.bind_host() == "0.0.0.0"


# --- clients send the header -------------------------------------------------

def test_twin_publisher_sends_token_header(monkeypatch):
    import importlib.util
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[1]
        / "webots/controllers/butlerbot_controller/twin_publisher.py"
    )
    spec = importlib.util.spec_from_file_location("twin_publisher_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    monkeypatch.setenv("RBM_API_TOKEN", TOKEN)
    assert mod._auth_headers({"Content-Type": "application/json"}) == {
        "Content-Type": "application/json",
        "X-API-Token": TOKEN,
    }
    monkeypatch.setenv("RBM_API_TOKEN", "")
    monkeypatch.setattr(mod, "_PROJECT_ENV_FILE", "/nonexistent/.env")
    assert "X-API-Token" not in mod._auth_headers()


def test_twin_publisher_reads_token_from_repo_env_file(monkeypatch, tmp_path):
    import importlib.util
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[1]
        / "webots/controllers/butlerbot_controller/twin_publisher.py"
    )
    spec = importlib.util.spec_from_file_location("twin_publisher_env_file", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    env_file = tmp_path / ".env"
    env_file.write_text("POSTGRES_USER=robot\nRBM_API_TOKEN=\"from-file\"\n", encoding="utf-8")
    monkeypatch.delenv("RBM_API_TOKEN", raising=False)
    monkeypatch.setattr(mod, "_PROJECT_ENV_FILE", str(env_file))
    assert mod.api_token() == "from-file"
