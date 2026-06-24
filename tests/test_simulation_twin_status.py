from src.simulation_driver import status_for_external_twin


def test_external_twin_status_reflects_webots_phase():
    st = status_for_external_twin(None, "drive_transit", "drive", source="webots")
    assert st["external_twin"] is True
    assert st["webots_phase"] == "drive_transit"
    assert st["segment_label"] == "Drive / Transit"
    assert st["segment_index"] == 2
    assert "Webots" in st["note"]