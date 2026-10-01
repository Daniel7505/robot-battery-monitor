"""Guard: the inline dashboard JS must parse.

HTML_TEMPLATE is a normal (non-raw) Python string, so a JS '\\n' written as
'\n' becomes a real newline and breaks the whole <script> block (dead buttons,
no live data). This test syntax-checks the served script with node when present.
"""
import re
import shutil
import subprocess

import pytest

from src.dashboard import HTML_TEMPLATE


@pytest.mark.skipif(shutil.which("node") is None, reason="node not installed")
def test_inline_dashboard_js_parses(tmp_path):
    scripts = re.findall(r"<script[^>]*>(.*?)</script>", HTML_TEMPLATE, re.S)
    assert scripts, "no inline <script> found"
    for i, js in enumerate(scripts):
        f = tmp_path / f"s{i}.js"
        f.write_text(js, encoding="utf-8")
        r = subprocess.run(["node", "--check", str(f)], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
