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


def test_run_id_splits_runs_and_is_reported(tmp_path, capsys):
    p = tmp_path / "lane-vision.csv"
    hdr = ("unix_s,x_m,y_m,mode,offset_m,heading_rad,curvature,lookahead_m,confidence,"
           "nL_pts,nR_pts,steer,target_speed,new_frame,run_id")
    lines = [hdr]
    # two runs back to back: no x reset and no time gap -> only run_id separates them
    for i in range(10):
        lines.append(f"{i},{i * 0.5},0.01,rowfit,0,0,0,1.9,0.9,50,50,0.1,0.44,1,20261001-180000")
    for i in range(10):
        lines.append(f"{10 + i},{5 + i * 0.5},-0.2,rowfit,0,0,0,1.9,0.9,50,50,0.1,0.44,0,20261001-181500")
    p.write_text("\n".join(lines) + "\n")
    runs = dr.split_runs(dr.load_rows(p))
    assert [len(r) for r in runs] == [10, 10]
    s2 = dr.summarize(runs[1], 2)
    assert s2["run_id"] == "20261001-181500" and s2["mode"] == "rowfit"
    assert dr.main([str(p)]) == 0
    out = capsys.readouterr().out
    assert "20261001-180000" in out and "run_id" in out


def test_old_logs_have_no_run_id_column_in_summary(tmp_path):
    p = _write(tmp_path, [(i, i * 0.5, 0.0, 0) for i in range(10)])
    s = dr.summarize(dr.split_runs(dr.load_rows(p))[0], 1)
    assert "run_id" not in s and "mode" not in s


def test_track_s_scores_vs_centreline(tmp_path, capsys):
    from s_track import centerline

    rows = [(i, 3.0 + 0.25 * i, centerline(3.0 + 0.25 * i)[0], 0) for i in range(20)]
    p = _write(tmp_path, rows)
    assert dr.main([str(p), "--track", "s"]) == 0
    assert "S-track" in capsys.readouterr().out
