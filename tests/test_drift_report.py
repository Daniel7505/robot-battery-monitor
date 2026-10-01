import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "drift_report", Path(__file__).resolve().parent.parent / "scripts" / "drift_report.py")
dr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dr)


def _write(tmp_path, rows):
    p = tmp_path / "log.csv"
    lines = ["unix_s,x_m,y_m,steer,oL,oR"]
    lines += [f"{t},{x},{y},{s},0,0" for t, x, y, s in rows]
    p.write_text("\n".join(lines) + "\n")
    return p


def test_splits_on_reset_and_gap(tmp_path):
    rows = [(i, i * 0.5, 0.01 * i, 0) for i in range(10)]            # run 1
    rows += [(20 + i, i * 0.5, -0.02, 0.1) for i in range(10)]       # reset -> run 2
    rows += [(200 + i, 5 + i * 0.5, 0.3 if i == 4 else 0, 0) for i in range(10)]  # gap -> run 3
    runs = dr.split_runs(dr.load_rows(_write(tmp_path, rows)))
    assert [len(r) for r in runs] == [10, 10, 10]
    s3 = dr.summarize(runs[2], 3)
    assert s3["max_drift_m"] == 0.3 and s3["worst_side"] == "left" and s3["worst_at_x_m"] == 7.0


def test_skips_junk_and_repeated_headers(tmp_path):
    p = _write(tmp_path, [(1, 0.1, 0.0, 0), (2, 0.2, -0.1, 0)])
    p.write_text(p.read_text() + "unix_s,x_m,y_m,steer\nbad,row,,\n")
    assert len(dr.load_rows(p)) == 2


def test_cli_runs(tmp_path, capsys):
    p = _write(tmp_path, [(i, i * 2.0, 0.05, 0) for i in range(10)])
    out = tmp_path / "s.csv"
    assert dr.main([str(p), "--out", str(out)]) == 0
    assert "yes" in capsys.readouterr().out and out.is_file()
