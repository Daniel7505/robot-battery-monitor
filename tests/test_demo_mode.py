from src.demo_mode import activate_demo, deactivate_demo, status


def test_demo_mode_toggle():
    activate_demo()
    st = status()
    assert st["active"] is True
    assert st["started_at"]
    deactivate_demo()
    assert status()["active"] is False