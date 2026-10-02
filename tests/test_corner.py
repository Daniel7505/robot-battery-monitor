"""Sharp-corner handling: lane_vision corner cue + rowfit_control state machine.

Dan's idea: in the look-ahead rows the line 'widens, then disappears' — one
row's yellow run spans far more than a stripe (the line now runs across the
row), smeared toward the turn, with no yellow beyond it.
"""
import importlib.util
import math
import sys
from pathlib import Path

import pytest

from src import rowfit_control as rc
from src.lane_vision import (
    CornerEstimate,
    LaneEstimate,
    LaneTracker,
    corner_cues,
    fuse_corner_cues,
    lane_polyline_robot,
    render_lane_bgra,
    yellow_mask,
)

ROOT = Path(__file__).resolve().parent.parent
CTRL_DIR = ROOT / "webots" / "controllers" / "butlerbot_controller"


def _sim():
    spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    return sim


SIM = _sim()
CAMS = SIM.sim_cameras()


def _scene(track, x, y=0.0, yaw_deg=0.0, yellow_mode=None):
    geom = SIM.track_geometry(track)
    _centre, left, right, bars = SIM.track_course(geom)
    yaw = math.radians(yaw_deg)
    lines = [lane_polyline_robot([p for p in poly if math.hypot(p[0] - x, p[1] - y) < 3.5], (x, y), yaw)
             for poly in (left, right)]
    marks = [(lane_polyline_robot(b, (x, y), yaw), w, c) for b, w, c in bars]
    imgs = {n: (render_lane_bgra(c, lines, marks=marks), c.width, c.height) for n, c in CAMS.items()}
    return imgs


def _cues(imgs, yellow_mode=None):
    out = []
    for n, c in CAMS.items():
        img = imgs[n][0]
        out += corner_cues(c, yellow_mask(img, c.width, c.height, mode=yellow_mode), c.width, c.height)
    return out


def test_sharp_left_outer_line_widens_then_disappears():
    cues = _cues(_scene("corner90", 3.0))
    outer = [c for c in cues if c.role == "outer"]
    assert len(outer) == 1
    c = outer[0]
    assert c.cam == "nadir_right" and c.direction == 1  # right shoulder cam, smear to the left
    assert c.x_m == pytest.approx(1.65, abs=0.05)  # outer line at x=4.65, robot at 3.0
    assert c.wide_m > 3 * c.stripe_m and c.empty_beyond is True and c.ramp_m < 0.05
    d, dist, conf, cams, _used = fuse_corner_cues(cues)
    assert d == 1 and dist == pytest.approx(1.65, abs=0.05) and conf >= 0.9
    assert "nadir_right outer" in cams and "nadir_left inner" in cams


def test_sharp_right_mirrors():
    d, dist, conf, cams, _ = fuse_corner_cues(_cues(_scene("corner90_right", 3.0)))
    assert d == -1 and dist == pytest.approx(1.65, abs=0.05) and conf >= 0.9
    assert cams.startswith("nadir_left outer")


def test_wide_lane_corner_distance():
    d, dist, conf, _cams, _ = fuse_corner_cues(_cues(_scene("corner90_wide", 3.0)))
    assert d == 1 and dist == pytest.approx(1.80, abs=0.05) and conf >= 0.6


@pytest.mark.parametrize("track,x,y,yaw", [
    ("corner_mix", 2.92, 0.0, 0.0),  # rounded r=0.5 left: outer line widens gradually (inner cue only)
    ("corner_mix", 3.3, 0.05, 10.0),
    ("widen", 3.0, 0.0, 0.0),  # widening lane
    ("widen", 6.8, 0.0, 0.0),  # narrowing again
    ("corner90", 0.2, 0.0, 0.0),  # green start bar in view
    ("corner90", 4.0, 3.0, 90.0),  # red finish bar + end of paint
])
def test_no_false_corner(track, x, y, yaw):
    assert fuse_corner_cues(_cues(_scene(track, x, y, yaw))) is None


def test_finish_bar_scored_as_yellow_is_not_a_corner():
    """Even if the red bar passed as paint (gap-mode yellow), a bar across the
    lane makes BOTH lines smear inward in opposite directions: no corner."""
    imgs = _scene("corner90", 4.0, 2.6, 90.0)
    cues = _cues(imgs, yellow_mode="lane_keep")
    assert {c.direction for c in cues if c.role == "outer"} in (set(), {1, -1})
    assert fuse_corner_cues(cues) is None


def test_tracker_persistence_and_fit_clip():
    tr = LaneTracker(CAMS)
    e1 = tr.process(_scene("corner90", 2.86), dx_m=0.0)
    assert e1.corner is not None and not e1.corner.seen and e1.corner.frames == 1
    e2 = tr.process(_scene("corner90", 3.0), dx_m=0.14)
    assert e2.corner.seen and e2.corner.frames == 2 and e2.corner.side == "left"
    assert e2.valid and e2.lookahead_m < e2.corner.clip_x_m + 0.01  # fit stops before the crossbar
    assert abs(e2.heading_rad) < math.radians(3) and abs(e2.offset_m) < 0.03
    # inconsistent distance (not what odometry predicts) restarts the count
    e3 = tr.process(_scene("corner90", 3.14), dx_m=0.6)
    assert e3.corner.frames == 1 and not e3.corner.seen


def test_relaxed_gate_after_pivot():
    tr = LaneTracker(CAMS)
    tr.process(_scene("corner90", 1.0), dx_m=0.0)
    tr.begin_reacquire(relax_frames=2)
    assert tr.prior_near is None and tr.lane_w is not None and tr.relax_frames == 2
    e = tr.process(_scene("corner90", 4.0, 0.3, 80.0), dx_m=0.0)  # new leg, 10 deg off
    assert e.valid and not e.held and tr.relax_frames == 1


# --------------------------------------------------------------------------
# state machine (synthetic estimates + an integrated yaw)
# --------------------------------------------------------------------------


def _lane(conf=0.9, heading=0.0, corner=None, n=60):
    psi = -heading
    return LaneEstimate(valid=True, offset_m=0.0, heading_rad=heading, lookahead_m=1.2, confidence=conf,
                        n_left=n, n_right=n, lane_width_m=1.3, width_measured=True, a0=0.0, psi=psi,
                        corner=corner)


def _corner(dist, d=1, conf=0.95, frames=2):
    return CornerEstimate(seen=frames >= 2, direction=d, distance_m=dist, confidence=conf, frames=frames,
                          cams="nadir_right outer", clip_x_m=dist - 0.15)


class _Bot:
    """Integrates the controller's wheel output (no camera)."""

    def __init__(self):
        self.ctl = rc.RowfitController()
        self.yaw = 0.0
        self.x = 0.0
        self.events = []

    def tick(self, est=None, new=False, yaw_ok=True):
        cmd = self.ctl.step(est, new_frame=new, dt=0.008, yaw=self.yaw if yaw_ok else None)
        wl, wr = cmd["left"], cmd["right"]
        v = 0.5 * (wl + wr) * rc.WHEEL_RADIUS_M
        self.yaw += (wr - wl) * rc.WHEEL_RADIUS_M / rc.TRACK_M * 0.008
        self.x += v * 0.008
        self.events += self.ctl.drain_events()
        return cmd


def test_state_machine_full_corner():
    b = _Bot()
    for _ in range(3):
        b.tick(_lane(), new=True)
        for _ in range(39):
            b.tick()
    assert b.ctl.state == rc.STATE_LANE
    b.ctl.gov.v = 0.44
    x0 = b.x
    cmd = b.tick(_lane(corner=_corner(1.40)), new=True)
    assert b.ctl.state == rc.STATE_APPROACH and cmd["state"] == "CORNER_APPROACH"
    assert b.ctl.remaining_m == pytest.approx(1.40 - 0.65, abs=0.01)  # outer line - half lane
    assert b.events[-1].startswith("CORNER seen left 1.40 m conf 0.95")
    speeds = []
    n = 0
    while b.ctl.state == rc.STATE_APPROACH and n < 4000:
        new = n % 40 == 39
        est = _lane(corner=_corner(1.40 - (b.x - x0))) if new else None
        cmd = b.tick(est, new=new)
        speeds.append(cmd.get("speed_cmd", 0.0))
        n += 1
    assert b.ctl.state == rc.STATE_PIVOT
    assert b.x - x0 == pytest.approx(0.75, abs=0.03)  # stopped at the pivot point
    assert max(speeds[-20:]) < 0.08 and not any(c is True for c in [cmd["brake"]])
    assert any(e.startswith("PIVOT start left") for e in b.events)
    max_rate, n = 0.0, 0
    while b.ctl.state == rc.STATE_PIVOT and n < 2000:
        y0 = b.yaw
        cmd = b.tick()
        assert not cmd["brake"]
        assert cmd["left"] == pytest.approx(-cmd["right"], abs=1e-3)  # in place
        max_rate = max(max_rate, abs(b.yaw - y0) / 0.008)
        n += 1
    assert b.ctl.state == rc.STATE_REACQUIRE
    assert math.degrees(b.yaw) == pytest.approx(90.0, abs=1.5)
    assert max_rate <= rc.PIVOT_W_MAX * 1.01  # (wheel commands are rounded to 1e-3)
    assert b.ctl.last_pivot["exit"] == "yaw on target" and b.ctl.last_pivot["time_s"] < 4.0
    assert any(e.startswith(("PIVOT done yaw +89", "PIVOT done yaw +90")) for e in b.events)
    for i in range(3):
        b.tick(_lane(), new=True)
        for _ in range(39):
            cmd = b.tick()
            if cmd["state"] == "REACQUIRE":
                assert cmd["speed_cmd"] <= rc.V_REACQUIRE_M_S + 1e-6
    assert b.ctl.state == rc.STATE_LANE and any(e.startswith("REACQUIRE ok") for e in b.events)
    # the cue right after the corner is ignored (cool-down)
    b.tick(_lane(corner=_corner(1.0)), new=True)
    assert b.ctl.state == rc.STATE_LANE


def test_no_lane_after_pivot_brakes():
    b = _Bot()
    b.tick(_lane(corner=_corner(0.70)), new=True)
    n = 0
    while b.ctl.state != rc.STATE_REACQUIRE and n < 3000:
        b.tick()
        n += 1
    assert b.ctl.state == rc.STATE_REACQUIRE
    cmd = None
    for _ in range(rc.LOST_FRAMES_STOP):
        cmd = b.tick(LaneEstimate(valid=False), new=True)
    assert cmd["brake"] and any(e.startswith("REACQUIRE failed") for e in b.events)


def test_without_imu_it_stops_at_the_corner():
    b = _Bot()
    b.tick(_lane(corner=_corner(1.0)), new=True, yaw_ok=False)
    assert b.ctl.state == rc.STATE_APPROACH and "NO IMU" in b.events[-1]
    cmd, n = None, 0
    while n < 4000:
        cmd = b.tick(yaw_ok=False)
        n += 1
        if cmd["brake"]:
            break
    assert cmd["brake"] and b.x == pytest.approx(0.35, abs=0.03)
    assert b.events[-1].startswith("CORNER reached but no IMU yaw")


def test_weak_or_single_frame_cue_ignored():
    b = _Bot()
    b.tick(_lane(corner=_corner(1.0, frames=1)), new=True)
    b.tick(_lane(corner=_corner(1.0, conf=0.5)), new=True)
    assert b.ctl.state == rc.STATE_LANE


def test_pivot_lane_exit_when_vision_sees_aligned_lane():
    b = _Bot()
    b.tick(_lane(corner=_corner(0.70)), new=True)
    n = 0
    while b.ctl.state != rc.STATE_PIVOT and n < 3000:
        b.tick()
        n += 1
    while abs(math.degrees(b.yaw) - 90.0) > 10.0:
        b.tick()
    b.tick(_lane(heading=math.radians(1.0)), new=True)
    assert b.ctl.state == rc.STATE_REACQUIRE and b.ctl.last_pivot["exit"].startswith("lane seen aligned")


def test_runtime_hud_and_log_carry_state(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(CTRL_DIR))
    for m in ("twin_publisher", "controller_rowfit"):
        sys.modules.pop(m, None)
    import twin_publisher

    monkeypatch.setattr(twin_publisher, "_PROJECT_ENV_FILE", str(tmp_path / "none.env"))
    import controller_rowfit as glue

    rt = glue.RowfitRuntime()
    assert rt.ctl.tracker is rt.tracker
    rt.est = _lane(corner=_corner(1.12, conf=0.8))
    rt._pending, rt._has_pending = rt.est, True
    printed = []
    cmd = rt.command(0.008, yaw=0.0, log=printed.append)
    assert printed and printed[0].startswith("CORNER seen left 1.12 m conf 0.80")
    eyes = {}
    rt.fill_eyes(eyes)
    assert eyes["rowfit_text"][0].startswith("CORNER L 1.12m") and eyes["rowfit_state"] == "CORNER_APPROACH"
    p = tmp_path / "lane-vision.csv"
    glue.LaneVisionLog(str(p), run_id="r1").write(x_m=1.0, y_m=0.0, mode="rowfit", est=rt.est, cmd=cmd,
                                                    new_frame=True, yaw=0.01)
    hdr, row = p.read_text().splitlines()
    rec = dict(zip(hdr.split(","), row.split(",")))
    assert rec["state"] == "CORNER_APPROACH" and rec["corner_dir"] == "left"
    assert float(rec["corner_m"]) == pytest.approx(1.12) and float(rec["corner_conf"]) == pytest.approx(0.8)
    assert rec["run_id"] == "r1" and float(rec["yaw_deg"]) == pytest.approx(math.degrees(0.01), abs=0.01)
