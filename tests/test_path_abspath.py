"""path_abspath must not lowercase Windows paths."""

from __future__ import annotations

from optimizer.lap_folder import path_abspath, paths_equal


def test_path_abspath_preserves_mixed_case(monkeypatch):
    monkeypatch.setattr(
        "optimizer.lap_folder.os.path.abspath",
        lambda _p: "C:\\Users\\MixedCase\\lap",
    )
    assert path_abspath("ignored") == "C:\\Users\\MixedCase\\lap"
    assert "MixedCase" in path_abspath("ignored")


def test_paths_equal_ignores_case_on_windows(monkeypatch):
    monkeypatch.setattr("optimizer.lap_folder.sys.platform", "win32")

    def abspath(p):
        return str(p).replace("/", "\\")

    monkeypatch.setattr("optimizer.lap_folder.os.path.abspath", abspath)
    assert paths_equal("C:\\Users\\A", "c:\\users\\a")
