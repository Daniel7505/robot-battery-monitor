"""scripts/run_report.py: per-run summary of lane-vision.csv with the junction columns."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("run_report", ROOT / "scripts" / "run_report.py")
rr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rr)

HDR = ("unix_s,x_m,y_m,mode,offset_m,heading_rad,curvature,lookahead_m,confidence,nL_pts,nR_pts,steer,"
       "target_speed,new_frame,run_id,state,corner_dir,corner_m,corner_conf,yaw_deg,yaw_target_deg,"
       "junction_type,junction_m,openings,route_choice,blind_m")


def _row(t, x, y, rid, state, jt="", op="", ch="", blind=""):
    return f"{t:.3f},{x:.3f},{y:.3f},rowfit,0,0,0,1.9,0.9,100,100,0,0.3,1,{rid},{state},,,0,0,,{jt},,{op},{ch},{blind}"


def _plus_s(rid, t0):
    rows = [_row(t0 + i, 0.5 * i, 0.01, rid, "LANE") for i in range(5)]
    rows += [_row(t0 + 5, 2.5, 0.0, rid, "JUNCTION_APPROACH", "plus", "L S? R"),
             _row(t0 + 6, 3.0, 0.0, rid, "JUNCTION_APPROACH", "plus", "L S R", "S"),
             _row(t0 + 7, 4.0, 0.0, rid, "GAP_CROSS", "", "", "S", "0.3"),
             _row(t0 + 8, 6.0, -0.02, rid, "LANE", "", "", "S"),
             _row(t0 + 9, 7.96, 0.0, rid, "JUNCTION_APPROACH", "lane_lost", "- S? -", "S")]
    return rows


def _t_end_l(rid, t0):
    rows = [_row(t0 + i, 0.5 * i, 0.0, rid, "LANE") for i in range(5)]
    rows += [_row(t0 + 5, 2.6, 0.0, rid, "JUNCTION_APPROACH", "t_end", "L - R", "L"),
             _row(t0 + 6, 4.0, 0.0, rid, "PIVOT", "", "", "L"),
             _row(t0 + 7, 4.03, 2.0, rid, "LANE", "", "", "L"),
             _row(t0 + 8, 4.0, 3.98, rid, "LANE", "", "", "L")]
    return rows


def test_report_splits_runs_guesses_tracks_and_flags_replays(tmp_path, capsys):
    log = tmp_path / "lane-vision.csv"
    log.write_text("\n".join([HDR] + _plus_s("A", 100) + _t_end_l("B", 200) + _plus_s("C", 300)) + "\n")
    assert rr.main([str(log)]) == 0
    out = capsys.readouterr().out
    runs = out.split("\nrun ")[1:]
    assert len(runs) == 3
    assert "track plus (guess)" in runs[0] and "exit east" in runs[0] and "plus[LSR]" in runs[0]
    assert "route:     S" in runs[0] and "blind max 0.3 m" in runs[0]
    assert "track t_end (guess)" in runs[1] and "exit north" in runs[1] and "t_end[L-R]" in runs[1]
    assert "PIVOT" in runs[1]
    assert "identical to run 1" in runs[2] and "identical" not in runs[1]


def test_report_uses_logged_world_and_track(tmp_path, capsys):
    log = tmp_path / "lane-vision.csv"
    log.write_text("\n".join([HDR] + _plus_s("A", 100)) + "\n")
    (tmp_path / "lane-vision-runs.csv").write_text(
        "run_id,unix_s,world,track,track_source,route,max_blind_m,look_around,warnings\n"
        "A,100,butlerbot_plus.wbt,plus,world butlerbot_plus.wbt,,4.00,1,"
        "Webots loaded butlerbot_plus.wbt but the launcher asked for butlerbot_t_end.wbt\n")
    assert rr.main([str(log)]) == 0
    out = capsys.readouterr().out
    assert "track plus (logged (butlerbot_plus.wbt))" in out and "RBM_ROUTE=(unset)" in out
    assert "WARNING:   Webots loaded butlerbot_plus.wbt" in out
    assert "cross-track: max 2.0 cm" in out
    assert rr.main([str(log), "--track", "t_end"]) == 0
    assert "track t_end (--track)" in capsys.readouterr().out
