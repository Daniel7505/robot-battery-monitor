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


def _rows_to_dicts(lines):
    keys = HDR.split(",")
    return [dict(zip(keys, ln.split(","))) for ln in lines]


def test_route_decisions_keep_repeated_choices():
    """route_choice is sticky: S at two junctions in a row is one value in the
    CSV. Leaving JUNCTION_APPROACH again is the second S; the end-of-paint
    lane_lost (one way) is not a decision; a gap that turns out to be a T
    corrects the last decision instead of adding one."""
    rid = "A"
    rows = [_row(100, 0.0, 0.0, rid, "LANE"),
            _row(101, 2.4, 0.0, rid, "JUNCTION_APPROACH", "plus", "LS?R"),
            _row(102, 2.5, 0.0, rid, "GAP_CROSS", "plus", "LSR", "S", "0"),
            _row(103, 4.0, 0.0, rid, "LANE", "", "", "S"),
            _row(104, 6.4, 0.0, rid, "JUNCTION_APPROACH", "plus", "LS?R", "S"),
            _row(105, 8.2, 0.0, rid, "GAP_CROSS", "plus", "LS?R", "S", "0"),
            _row(106, 11.0, 0.0, rid, "LANE", "", "", "S"),
            _row(107, 14.0, 0.0, rid, "JUNCTION_APPROACH", "plus", "LS?R", "S"),
            _row(108, 15.0, 0.0, rid, "GAP_CROSS", "plus", "LSR", "S", "0.5"),
            _row(109, 15.5, 0.0, rid, "CORNER_APPROACH", "plus", "LSR", "L"),
            _row(110, 16.0, 2.0, rid, "LANE", "", "", "L"),
            _row(111, 16.0, 4.9, rid, "LANE", "", "", "S"),  # side branch passed straight
            _row(112, 16.0, 9.0, rid, "JUNCTION_APPROACH", "lane_lost", "-S?-", "S"),
            _row(113, 16.0, 9.5, rid, "GAP_CROSS", "lane_lost", "-S?-", "S", "0")]
    ds = rr.route_decisions(_rows_to_dicts(rows))
    assert [d["choice"] for d in ds] == ["S", "S", "L", "S"]
    assert ds[2].get("note") == "gap T: turned" and ds[3]["kind"] == "side"
    # the end of the paint after a turn changes route_choice to the only way: still no entry
    rows2 = rows[:11] + [_row(112, 16.0, 9.0, rid, "JUNCTION_APPROACH", "lane_lost", "-S?-", "L"),
                         _row(113, 16.0, 9.5, rid, "GAP_CROSS", "lane_lost", "-S?-", "S", "0")]
    assert rr.route_choices(_rows_to_dicts(rows2)) == ["S", "S", "L"]


def _warehouse_front(rid, t0):
    """RCV-1 -> S (dock lane) -> S (forklift crossing) -> L (front cross aisle)
    -> S past aisle 3 -> S past aisle 4 (no trace: same choice, side branch)."""
    rows, t = [], t0
    def add(x, y, st, jt="", op="", ch="", blind=""):
        nonlocal t
        rows.append(_row(t, x, y, rid, st, jt, op, ch, blind))
        t += 1
    for x in (0.0, 1.0):
        add(x, 0.0, "LANE")
    add(2.4, 0.0, "JUNCTION_APPROACH", "plus", "LS?R")
    add(2.5, 0.0, "GAP_CROSS", "plus", "LSR", "S", "0")
    for x in (3.5, 4.0, 4.5, 5.5):
        add(x, 0.0, "GAP_CROSS" if x < 4.6 else "LANE", ch="S")
    add(6.4, 0.0, "JUNCTION_APPROACH", "plus", "LS?R", "S")
    for x in (8.2, 9.5, 10.0, 10.5):
        add(x, 0.0, "GAP_CROSS", "plus", "LS?R", "S", "0")
    for x in (12.0, 14.0, 15.3):
        add(x, 0.0, "LANE", ch="S")
    add(16.4, 0.0, "CORNER_APPROACH", "plus", "LSR", "L")
    add(17.94, 0.03, "PIVOT", "plus", "LSR", "L")
    add(18.0, 0.3, "REACQUIRE", "", "", "L")
    for y in (1.0, 3.0):
        add(18.0, y, "LANE", ch="L")
    for y in (4.9, 5.5, 6.0, 6.5, 8.0, 10.0, 11.5, 12.0, 12.5, 14.0):
        add(18.0, y, "LANE", ch="S")
    return rows


def test_report_warehouse_counts_silent_junctions_and_worst_spot(tmp_path, capsys):
    log = tmp_path / "lane-vision.csv"
    log.write_text("\n".join([HDR] + _warehouse_front("W", 100)) + "\n")
    assert rr.main([str(log), "--track", "warehouse"]) == 0
    out = capsys.readouterr().out
    assert "route:     S,S,L,S" in out
    assert "decisions: 4 in the log, 5 junctions driven through (GPS)" in out
    assert "no decision logged at 1 junction(s): aisle_4 x perimeter (18, 12)" in out
    assert "worst at (17.94, 0.03) PIVOT, 0.1 m from junction perimeter x warehouse (18, 0)" in out


def test_guess_knows_the_warehouse():
    import sys

    sys.path.insert(0, str(ROOT))
    from src.track_geometry import load_track

    run = _rows_to_dicts(_warehouse_front("W", 100)
                         + [_row(200, -2.0, 36.0, "W", "LANE"), _row(201, -3.0, 36.0, "W", "GAP_CROSS")])
    assert rr.guess_track(run, {"plus": load_track("plus"), "warehouse": load_track("warehouse")}) == "warehouse"


OBS_HDR = HDR + ",obstacle_state,obstacle_m,obstacle_band,obstacle_conf"


def _obs_row(t, x, rid, state, obs_state, m="", band="", conf=""):
    return _row(t, x, 0.0, rid, state) + f",{obs_state},{m},{band},{conf}"


def test_report_summarizes_obstacle_stops(tmp_path, capsys):
    rows = [_obs_row(100 + i, 0.5 * i, "A", "LANE", "CLEAR") for i in range(4)]
    rows += [_obs_row(104, 2.0, "A", "LANE", "SLOW_FOR_OBSTACLE", 1.70, "low", 1.0),
             _obs_row(105, 2.6, "A", "LANE", "SLOW_FOR_OBSTACLE", 1.05, "low", 1.0),
             _obs_row(106, 3.1, "A", "LANE", "STOPPED_FOR_OBSTACLE", 0.56, "low", 1.0),
             _obs_row(112, 3.1, "A", "LANE", "STOPPED_FOR_OBSTACLE", 0.55, "low", 1.0),
             _obs_row(114, 3.1, "A", "LANE", "CLEAR"),
             _obs_row(115, 3.5, "A", "LANE", "CLEAR")]
    log = tmp_path / "lane-vision.csv"
    log.write_text("\n".join([OBS_HDR] + rows) + "\n")
    assert rr.main([str(log)]) == 0
    out = capsys.readouterr().out
    line = next(ln for ln in out.splitlines() if "obstacles:" in ln)
    assert "1 stop(s), 1 slow-down(s)" in line and "nearest 0.55 m (low)" in line
    assert "stop 1 at (3.10, 0.00): low obstacle 0.56 m ahead, waited 8.0 s, corridor cleared" in line


def test_report_obstacles_off_and_old_logs(tmp_path, capsys):
    log = tmp_path / "lane-vision.csv"
    log.write_text("\n".join([OBS_HDR] + [_obs_row(100 + i, 0.5 * i, "A", "LANE", "") for i in range(5)]) + "\n")
    assert rr.main([str(log)]) == 0
    assert "obstacles: off" in capsys.readouterr().out
    log.write_text("\n".join([HDR] + _plus_s("A", 100)) + "\n")  # pre-obstacle log: no line at all
    assert rr.main([str(log)]) == 0
    assert "obstacles:" not in capsys.readouterr().out
