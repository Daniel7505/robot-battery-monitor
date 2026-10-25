"""
Robot Battery Monitor — operator dashboard.

Flask serves one page and the twin HTTP API. The browser polls ``GET /api/live``
every half second. That route reads pose, speed, steer, and mission efficiency
from the twin bridge, not from the 3 s power-model tick.

There is no browser socket. A blocked ``cdn.socket.io`` script used to freeze
the page, so that script stays out. The process serves HTTP with Flask.

Pause is client-side only. The hardware tick and twin ingest keep running.
"""

from flask import Flask, render_template_string, jsonify
from datetime import datetime
import warnings
import threading
import time

from src.config import config
from src.logger import logger
from src.database import (
    log_drive_measurement,
    get_drive_measurements,
)

warnings.filterwarnings("ignore")

from src import api_security

app = Flask(__name__)
api_security.init_app(app)


HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>{{ robot_name }} Live Monitor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #0a0a0a; color: #ddd; }
        h1 { color: #9df; text-align: center; font-size: 1.4em; }
        h2 { color: #9cf; font-size: 1em; margin: 0 0 8px; }
        .status { text-align: center; color: #889; }
        .live-controls { text-align: center; margin: 8px 0 14px; }
        button { background: #1a2a3a; border: 1px solid #456; color: #cde; padding: 6px 12px; border-radius: 5px; cursor: pointer; margin: 0 4px; }
        button:hover { background: #243448; }
        .pause-btn.paused { border-color: #a80; color: #fc8; }
        .hint { color: #678; font-size: 0.85em; margin-left: 8px; }
        .warning { background: #440000; color: #ff4444; padding: 12px; border-radius: 8px; text-align: center; font-weight: bold; margin: 10px 0; display: none; }
        .cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin: 12px 0; }
        .ess-card { background: #0d1117; border: 1px solid #334; border-radius: 8px; padding: 12px 14px; }
        .ess-card strong { display: block; font-size: 1.25em; color: #9df; margin-top: 4px; }
        .ess-card span, .sub { font-size: 0.78em; color: #778; }
        .ess-card.ok strong { color: #8f8; }
        .ess-card.warn strong { color: #fc8; }
        .ess-card.crit strong { color: #f88; }
        .panel { background: #0d1117; border: 1px solid #345; border-radius: 8px; padding: 14px 16px; margin: 12px 0; }
        .badge { display: inline-block; padding: 3px 8px; border-radius: 4px; font-size: 0.8em; margin-left: 8px; background: #222; color: #888; }
        .badge.on { background: #132; color: #8f8; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 10px; margin-top: 10px; }
        .stat span { display: block; color: #778; font-size: 0.75em; text-transform: uppercase; }
        .stat strong { color: #def; font-size: 1.05em; }
        .steer-bar { position: relative; height: 10px; background: #222; border-radius: 5px; margin: 8px 0; }
        .steer-center { position: absolute; left: 50%; top: 0; width: 1px; height: 100%; background: #456; }
        .steer-needle { position: absolute; top: -3px; width: 4px; height: 16px; background: #9df; border-radius: 2px; }
        .note { color: #678; font-size: 0.82em; margin-top: 6px; }
        .proof-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 12px; }
    </style>
</head>
<body>
    <h1>{{ robot_name }}</h1>
    <div class="live-controls">
        <button type="button" id="pause-resume-btn" class="pause-btn">Pause</button>
        <span class="hint" id="pause-hint">Numbers follow the robot. Pause holds them so you can read.</span>
    </div>
    <p class="status">Last update: <span id="last-update">—</span> · Hardware: <span id="hw-mode">—</span></p>
    <div id="warning" class="warning"></div>

    <div class="cards">
        <div class="ess-card" id="ess-battery"><span>Battery</span><strong id="ess-battery-val">—</strong></div>
        <div class="ess-card" id="ess-draw"><span>System draw</span><strong id="ess-draw-val">—</strong><div class="sub" id="ess-draw-sub"></div></div>
        <div class="ess-card" id="ess-runtime"><span>Runtime left</span><strong id="ess-runtime-val">—</strong></div>
        <div class="ess-card" id="ess-energy"><span>Energy left</span><strong id="ess-energy-val">—</strong></div>
    </div>

    <div class="panel" id="twin-panel">
        <strong>This lap</strong>
        <span id="twin-badge" class="badge">AWAITING TWIN</span>
        <div class="grid">
            <div class="stat"><span>Phase</span><strong id="twin-phase">—</strong></div>
            <div class="stat"><span>Speed</span><strong id="twin-speed">—</strong></div>
            <div class="stat"><span>Position</span><strong id="twin-pose">—</strong></div>
            <div class="stat"><span>Mission time</span><strong id="eff-elapsed">—</strong></div>
            <div class="stat"><span>Energy used</span><strong id="eff-energy">—</strong></div>
            <div class="stat"><span>Battery change</span><strong id="eff-batt">—</strong></div>
            <div class="stat"><span>Distance</span><strong id="eff-distance">—</strong></div>
            <div class="stat"><span>Average speed</span><strong id="eff-avg">—</strong></div>
        </div>
        <p class="note" id="eff-note">Waiting for the twin.</p>
        <div style="margin-top:10px">
            <button type="button" id="battery-replenish-btn">Replenish battery</button>
            <button type="button" id="teleop-forward-btn">Forward</button>
            <button type="button" id="teleop-back-btn">Back</button>
            <button type="button" id="teleop-left-btn">Left</button>
            <button type="button" id="teleop-right-btn">Right</button>
            <button type="button" id="teleop-stop-btn">Stop</button>
            <button type="button" id="lane-keep-btn">Lane keep</button>
        </div>
        <p class="note" id="lane-keep-status">Lane keep off. GPS stop at 16.5 m.</p>
        <div class="steer-bar"><div class="steer-center"></div><div id="steer-needle" class="steer-needle" style="left:50%"></div></div>
        <div id="steer-readout">steer —</div>
    </div>

    <div class="panel" id="proof-panel">
        <h2>Proof <span id="proof-source" class="badge">No lap yet</span></h2>
        <div class="proof-grid">
            <div class="stat"><span>Measured vs model</span><strong id="proof-err">—</strong><div class="sub" id="proof-wh"></div></div>
            <div class="stat"><span>Steer source</span><strong id="proof-steer">—</strong><div class="sub">camera · held · track</div></div>
            <div class="stat"><span>Cross-track</span><strong id="proof-cross">—</strong></div>
            <div class="stat"><span>Width error</span><strong id="proof-width">—</strong></div>
            <div class="stat"><span>Center error</span><strong id="proof-center">—</strong></div>
        </div>
        <p class="note" id="proof-note">Model versus measured is the drive watt-hours. Lane error is from the camera fit.</p>
    </div>

<script>
    let livePaused = false;
    let liveInFlight = false;
    let laneKeepArmed = false;

    function setText(id, value) {
        const el = document.getElementById(id);
        if (!el) return;
        const next = (value === null || value === undefined || value === '') ? '\\u2014' : String(value);
        if (el.textContent !== next) el.textContent = next;
    }

    function setRaw(id, value) {
        const el = document.getElementById(id);
        if (!el) return;
        const next = value == null ? '' : String(value);
        if (el.textContent !== next) el.textContent = next;
    }

    function fmtMin(m) {
        if (m == null || Number.isNaN(Number(m))) return null;
        const n = Number(m);
        return n >= 60 ? (n / 60).toFixed(1) + ' hr' : Math.round(n) + ' min';
    }

    function cm(metres) {
        if (metres == null || Number.isNaN(Number(metres))) return null;
        return (Number(metres) * 100).toFixed(1) + ' cm';
    }

    function share(frac) {
        if (frac == null || Number.isNaN(Number(frac))) return '\\u2014';
        return Math.round(Number(frac) * 100) + '%';
    }

    function applyLivePayload(data) {
        if (!data || livePaused) return;
        setText('last-update', data.timestamp || null);
        if (data.hardware_mode) {
            setText('hw-mode', data.hardware_mode.mode + ' / ' + data.hardware_mode.type);
        }

        const batPct = data.main_battery;
        const batCard = document.getElementById('ess-battery');
        let batCls = 'ess-card';
        if (batPct != null) {
            batCls = batPct <= 10 ? 'ess-card crit' : (batPct <= 20 ? 'ess-card warn' : 'ess-card ok');
        }
        if (batCard.className !== batCls) batCard.className = batCls;
        setText('ess-battery-val', batPct == null ? null : batPct + '%');
        const warn = document.getElementById('warning');
        if (batPct != null && batPct <= 20) {
            if (warn.style.display !== 'block') warn.style.display = 'block';
            setText('warning', 'Battery low');
        } else if (warn.style.display !== 'none') {
            warn.style.display = 'none';
        }

        let draw = null;
        if (data.channels && data.channels.length) {
            draw = data.channels.reduce(function (sum, ch) { return sum + Number(ch.draw || 0); }, 0);
            draw = Math.round(draw * 10) / 10;
        }
        setText('ess-draw-val', draw == null ? null : draw + ' W');
        const alloc = data.allocation || {};
        const util = alloc.utilization_pct;
        setRaw('ess-draw-sub', util == null ? '' : (util + '% of budget'));

        const mission = data.mission || alloc;
        setText('ess-runtime-val', fmtMin(mission.runtime_min_at_current_draw));
        const whLeft = mission.energy_wh_remaining;
        setText('ess-energy-val', whLeft == null ? null : whLeft + ' Wh');

        const live = data.live || {};
        const badge = document.getElementById('twin-badge');
        const badgeCls = live.active ? 'badge on' : 'badge';
        const badgeTxt = live.active ? 'TWIN LIVE' : 'AWAITING TWIN';
        if (badge.className !== badgeCls) badge.className = badgeCls;
        setText('twin-badge', badgeTxt);
        setText('twin-phase', live.active ? (live.phase_label || null) : null);

        if (!live.active || live.speed_m_s == null) {
            setText('twin-speed', null);
            setText('twin-pose', null);
        } else {
            setText('twin-speed', Number(live.speed_m_s).toFixed(2) + ' m/s');
            const pose = live.pose || {};
            const poseTxt = (pose.x_m == null) ? null
                : '(' + Number(pose.x_m).toFixed(2) + ', ' + Number(pose.y_m == null ? 0 : pose.y_m).toFixed(2) + ')';
            setText('twin-pose', poseTxt);
        }

        const eff = live.efficiency || {};
        if (live.active && eff.active) {
            setText('eff-elapsed', eff.elapsed_label || (eff.elapsed_s != null ? Number(eff.elapsed_s).toFixed(1) + ' s' : null));
            let eTxt = null;
            if (eff.energy_wh_used != null) {
                eTxt = Number(eff.energy_wh_used).toFixed(3) + ' Wh';
                if (eff.energy_source === 'integrated_draw') eTxt += ' (from draw)';
                else if (eff.energy_source === 'soc_delta') eTxt += ' (from battery)';
            }
            setText('eff-energy', eTxt);
            if (eff.battery_delta_pct != null) {
                const d = Number(eff.battery_delta_pct);
                setText('eff-batt', (d >= 0 ? '\\u2212' : '+') + Math.abs(d).toFixed(2) + '%');
            } else {
                setText('eff-batt', null);
            }
            setText('eff-distance', eff.distance_m != null ? Number(eff.distance_m).toFixed(2) + ' m' : null);
            setText('eff-avg', eff.avg_speed_m_s != null ? Number(eff.avg_speed_m_s).toFixed(2) + ' m/s' : null);
            setText('eff-note', eff.parked ? 'Lap parked.' : 'Lap running.');
        } else {
            setText('eff-elapsed', null);
            setText('eff-energy', null);
            setText('eff-batt', null);
            setText('eff-distance', null);
            setText('eff-avg', null);
            setText('eff-note', 'Waiting for the twin.');
        }

        laneKeepArmed = !!live.lane_keep;
        setText('lane-keep-status', (laneKeepArmed ? 'Lane keep on' : 'Lane keep off') + '. GPS stop at 16.5 m.');
        const lkBtn = document.getElementById('lane-keep-btn');
        const lkTxt = laneKeepArmed ? 'Lane keep on' : 'Lane keep';
        if (lkBtn.textContent !== lkTxt) lkBtn.textContent = lkTxt;

        const steer = (live.active && live.steer != null) ? Number(live.steer) : null;
        const steerOk = steer != null && !Number.isNaN(steer);
        setText('steer-readout', steerOk ? ('steer ' + steer.toFixed(3)) : null);
        const needle = document.getElementById('steer-needle');
        if (needle) {
            const left = steerOk ? (Math.max(0, Math.min(100, 50 + steer * 50)).toFixed(1) + '%') : '50%';
            if (needle.style.left !== left) needle.style.left = left;
        }

        const proof = data.proof || {};
        const src = document.getElementById('proof-source');
        const srcTxt = proof.ok ? (proof.source || 'This lap') : 'No lap yet';
        const srcCls = proof.ok ? 'badge on' : 'badge';
        if (src.className !== srcCls) src.className = srcCls;
        setText('proof-source', srcTxt);
        if (proof.err_pct == null) {
            setText('proof-err', null);
        } else {
            const n = Number(proof.err_pct);
            setText('proof-err', (n > 0 ? '+' : '') + n.toFixed(1) + '%');
        }
        const whBits = [];
        if (proof.wh_measured_drive_elec != null) whBits.push('measured ' + Number(proof.wh_measured_drive_elec).toFixed(2) + ' Wh');
        if (proof.wh_model_drive != null) whBits.push('model ' + Number(proof.wh_model_drive).toFixed(2) + ' Wh');
        setRaw('proof-wh', whBits.join(' · '));
        if (proof.steer_frac_camera == null && proof.steer_frac_held == null && proof.steer_frac_track == null) {
            setText('proof-steer', null);
        } else {
            setText('proof-steer', share(proof.steer_frac_camera) + ' · ' + share(proof.steer_frac_held) + ' · ' + share(proof.steer_frac_track));
        }
        const nowCt = cm(proof.cross_track_m);
        const maxCt = cm(proof.ct_max_m);
        if (nowCt == null && maxCt == null) setText('proof-cross', null);
        else setText('proof-cross', (nowCt || '\\u2014') + ' now · ' + (maxCt || '\\u2014') + ' max');
        const wR = cm(proof.width_err_rms_m);
        const wM = cm(proof.width_err_max_m);
        if (wR == null && wM == null) setText('proof-width', null);
        else setText('proof-width', (wR || '\\u2014') + ' rms · ' + (wM || '\\u2014') + ' max');
        const cR = cm(proof.center_err_rms_m);
        const cM = cm(proof.center_err_max_m);
        if (cR == null && cM == null) setText('proof-center', null);
        else setText('proof-center', (cR || '\\u2014') + ' rms · ' + (cM || '\\u2014') + ' max');
    }

    function pullLive() {
        if (livePaused || liveInFlight) return;
        liveInFlight = true;
        fetch('/api/live')
            .then(function (r) { return r.json(); })
            .then(function (data) {
                try { applyLivePayload(data); }
                catch (err) { console.warn('live paint', err); }
            })
            .catch(function (err) { console.warn('live poll failed', err); })
            .finally(function () { liveInFlight = false; });
    }
    pullLive();
    setInterval(pullLive, 500);

    document.getElementById('pause-resume-btn').addEventListener('click', function () {
        livePaused = !livePaused;
        const btn = document.getElementById('pause-resume-btn');
        const hint = document.getElementById('pause-hint');
        if (livePaused) {
            btn.textContent = 'Resume';
            btn.className = 'pause-btn paused';
            hint.textContent = 'Updates paused. Numbers are holding.';
        } else {
            btn.textContent = 'Pause';
            btn.className = 'pause-btn';
            hint.textContent = 'Numbers follow the robot. Pause holds them so you can read.';
            pullLive();
        }
    });

    function postTwinCommand(body) {
        return fetch('/api/twin/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
        }).then(function (r) { return r.json(); });
    }

    document.getElementById('battery-replenish-btn').addEventListener('click', function () {
        postTwinCommand({ battery_reset: true, battery_pct: 100, source: 'dashboard' });
    });
    document.getElementById('teleop-forward-btn').addEventListener('click', function () {
        postTwinCommand({ drive: { left: 5.5, right: 5.5, duration_s: 3 }, source: 'dashboard' });
    });
    document.getElementById('teleop-back-btn').addEventListener('click', function () {
        postTwinCommand({ drive: { left: -5.5, right: -5.5, duration_s: 3 }, source: 'dashboard' });
    });
    document.getElementById('teleop-left-btn').addEventListener('click', function () {
        postTwinCommand({ drive: { left: -4.5, right: 4.5, duration_s: 3 }, source: 'dashboard' });
    });
    document.getElementById('teleop-right-btn').addEventListener('click', function () {
        postTwinCommand({ drive: { left: 4.5, right: -4.5, duration_s: 3 }, source: 'dashboard' });
    });
    document.getElementById('teleop-stop-btn').addEventListener('click', function () {
        postTwinCommand({ drive_stop: true, source: 'dashboard' });
    });
    document.getElementById('lane-keep-btn').addEventListener('click', function () {
        const next = !laneKeepArmed;
        postTwinCommand({ lane_keep: next, source: 'dashboard' }).then(function (data) {
            if (data && data.ok) laneKeepArmed = next;
        });
    });
</script>
</body>
</html>
'''


# ===========================================================================
# REST routes
# ===========================================================================

@app.route('/')
def dashboard():
    """First paint. Live numbers arrive from GET /api/live, not from Postgres."""
    return render_template_string(
        HTML_TEMPLATE,
        robot_name=config.get('robot', 'name', 'Robot'),
    )


# --- Demo / sim helpers ----------------------------------------------------

@app.route('/api/demo/activate', methods=['POST'])
def api_demo_activate():
    from src.demo_mode import activate_demo
    return jsonify(activate_demo())


@app.route('/api/demo/deactivate', methods=['POST'])
def api_demo_deactivate():
    from src.demo_mode import deactivate_demo
    return jsonify(deactivate_demo())


@app.route('/api/demo/status')
def api_demo_status():
    from src.demo_mode import status as demo_status
    return jsonify(demo_status())


@app.route('/api/demo/launch-webots', methods=['POST'])
@api_security.localhost_only
def api_demo_launch_webots():
    """Activate demo mode and attempt host-side Webots launch."""
    from src.demo_mode import activate_demo, launch_webots_on_host
    activate_demo()
    return jsonify(launch_webots_on_host())


@app.route('/api/simulation/<action>', methods=['POST'])
def api_simulation(action):
    """Start/stop the PMS mission script (Idle→Patrol). Not the Webots S.

    ``action`` is ``start`` or ``stop``. Only available when the hardware
    backend exposes simulation controls.
    """
    from src.hardware import get_hardware_source

    hardware = get_hardware_source()
    if action == 'start' and hasattr(hardware, 'start_simulation'):
        return jsonify(hardware.start_simulation())
    if action == 'stop' and hasattr(hardware, 'stop_simulation'):
        return jsonify(hardware.stop_simulation())
    return jsonify({"error": "Simulation not available for this hardware mode"}), 400


# --- Digital twin API (Webots / external simulators) -----------------------
# Contract summary for first-time readers:
#
#   GET  /api/twin/state
#       Full PMS + teleop snapshot for the controller to poll.
#       Includes agent/safety throttle, teleop wheel cmds, stop_epoch,
#       and one-shot battery_pct / reset_thermal after battery_reset.
#
#   POST /api/twin/telemetry?adapter=webots
#       Body: joints, locomotion, channel_draws, robot.main_battery_pct, …
#       Ingests external power feed; syncs hardware readings immediately so
#       the UI does not wait for the 3 s hardware tick.
#
#   POST /api/twin/command
#       Body fields (any subset):
#         drive: {left, right, duration_s}  — rad/s wheel velocities
#         drive_stop: true                 — zeros drive; bumps stop_epoch
#         battery_reset: true, battery_pct — one-shot replenish for Webots
#         source: string                   — operator / script label
#       stop_epoch: monotonic counter the controller compares so a late poll
#       still sees a new stop even if wheel cmds already read as zero.
#
#   GET  /api/twin/schema
#       Discovery helper for adapters and demos. The CLI ``--report`` path
#       still prints analytics; this process does not serve /api/analytics.

@app.route('/api/twin/state')
def api_twin_state():
    """Export twin + hardware state for Webots controller / agent scripts."""
    from src.hardware import get_hardware_source
    from src.twin import get_twin_bridge

    bridge = get_twin_bridge()
    if not bridge.enabled:
        return jsonify({"error": "Digital twin bridge disabled"}), 503
    hardware = get_hardware_source()
    return jsonify(bridge.export_state(hardware))


@app.route('/api/twin/telemetry', methods=['POST'])
def api_twin_telemetry():
    """Ingest external twin telemetry and push it into the live power model."""
    from flask import request
    from src.hardware import get_hardware_source
    from src.twin import get_twin_bridge

    bridge = get_twin_bridge()
    payload = request.get_json(silent=True) or {}
    adapter = request.args.get('adapter')
    result = bridge.ingest_telemetry(payload, adapter=adapter)
    # Push battery % + channel watts into hardware.last_readings on this POST.
    # Pose, speed, and steer are read from the bridge by GET /api/live.
    if result.get("ok"):
        try:
            hardware = get_hardware_source()
            applied = bridge.sync_to_hardware(hardware)
            result["applied_to_hardware"] = bool(applied)
            # Update PMS mission from twin motion on every POST (~0.5s), not only
            # the 3s telemetry tick — so the dashboard leaves Idle while driving.
            if hasattr(hardware, "_apply_twin_mission_task") and bridge._last_telemetry:
                if hardware._apply_twin_mission_task(bridge._last_telemetry):
                    result["mission_task"] = getattr(
                        getattr(hardware, "_mission", None), "task_id", None
                    )
            feed = bridge.get_power_feed()
            if feed is not None:
                result["power_source"] = feed.source
                result["battery_pct"] = feed.battery_pct
        except Exception as exc:
            result["applied_to_hardware"] = False
            result["apply_error"] = str(exc)
    status = 200 if result.get("ok") else 400
    return jsonify(result), status


@app.route('/api/twin/command', methods=['POST'])
def api_twin_command():
    """Apply drive / stop / battery_reset (and related) commands to the bridge.

    Webots does not receive this POST directly — the controller polls
    ``/api/twin/state`` and reads ``teleop`` fields including ``stop_epoch``.
    """
    from flask import request
    from src.hardware import get_hardware_source
    from src.twin import get_twin_bridge

    bridge = get_twin_bridge()
    payload = request.get_json(silent=True) or {}
    result = bridge.apply_command(get_hardware_source(), payload)
    status = 200 if result.get("ok") else 400
    return jsonify(result), status


@app.route('/api/twin/schema')
def api_twin_schema():
    """JSON schema / field guide for twin telemetry and commands."""
    from src.twin import get_twin_bridge

    return jsonify(get_twin_bridge().schema())


# --- Lightweight data snapshot --------------------------------------------

@app.route('/api/measurements', methods=['GET'])
def api_measurements_list():
    """List recent short-drive energy / wheel-lock measurements from Postgres."""
    from flask import request
    limit = 20
    try:
        limit = min(100, max(1, int(request.args.get("limit", 20))))
    except (TypeError, ValueError):
        pass
    return jsonify({"ok": True, "measurements": get_drive_measurements(limit=limit)})


@app.route('/api/measurements', methods=['POST'])
def api_measurements_create():
    """Store one drive measurement (energy, distance, wheel lock sensors)."""
    from flask import request
    body = request.get_json(silent=True) or {}
    if not isinstance(body, dict):
        return jsonify({"ok": False, "error": "JSON object required"}), 400
    new_id = log_drive_measurement(body)
    if new_id is None:
        return jsonify({"ok": False, "error": "DB write failed"}), 500
    return jsonify({"ok": True, "id": new_id})


@app.route('/api/live')
def api_live():
    """Live page payload. The browser polls this every 500 ms."""
    return jsonify(_build_battery_payload())


# ===========================================================================
# Process entry + background services
# ===========================================================================
# The browser polls GET /api/live. Nothing in this process emits to a socket.
# Pause is entirely client-side. Hardware and twin routes keep working.

def run_dashboard():
    """Start hardware, archiver, then serve HTTP."""
    from src.hardware import get_hardware_source

    hardware = get_hardware_source()
    if not hardware.running:
        hardware.start()

    start_auto_archiver()

    port = config.get("dashboard", "port", 5000)
    host = api_security.bind_host()
    logger.info(f"Dashboard started on {host}:{port}")
    if api_security.api_token() is None:
        api_security._warn_once_no_token()

    # Werkzeug's dev server. Acceptable here because it is a single-user lab
    # tool bound to 127.0.0.1 by default (0.0.0.0 only inside Docker, where
    # compose publishes the port on 127.0.0.1). Do not expose it to a network
    # as-is; put a real WSGI server / reverse proxy in front first.
    app.run(host=host, port=port, debug=False, threaded=True, use_reloader=False)
    


import threading
import time

def start_auto_archiver():
    """Background thread that periodically archives old data."""
    archive_days = config.get("monitoring", "archive_after_days", 30)
    interval_hours = config.get("monitoring", "archive_interval_hours", 24)

    def _archiver():
        while True:
            try:
                from src.database import archive_old_data
                logger.info(f"Running scheduled archive (older than {archive_days} days)...")
                archive_old_data(days=archive_days)
            except Exception as e:
                logger.error(f"Auto-archive job failed: {e}")
            time.sleep(interval_hours * 3600)

    thread = threading.Thread(target=_archiver, daemon=True, name="AutoArchiver")
    thread.start()
    logger.info(f"✅ Auto-archiver started (every {interval_hours}h, archive data older than {archive_days} days)")


def _phase_label(phase, gait, speed) -> str | None:
    """Short phase name. Teleop while moving stays 'Teleop Drive'."""
    from src.twin.control import PHASE_LABELS

    phase_l = str(phase or "").lower()
    gait_l = str(gait or "").lower()
    try:
        speed_f = float(speed) if speed is not None else 0.0
    except (TypeError, ValueError):
        speed_f = 0.0
    if phase_l in ("", "standby") and (
        gait_l in ("drive", "transit", "turn") or speed_f >= 0.08
    ):
        phase = "teleop_turn" if gait_l == "turn" else "teleop"
        phase_l = phase
    if not phase_l:
        return None
    return PHASE_LABELS.get(phase_l, phase_l.replace("_", " ").title())


def _num(value):
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _live_from_bridge() -> dict:
    """Pose, speed, steer, and efficiency from the latest twin post.

    These fields used to wait for the 3 s power-model tick. The page polls
    twice a second, so they are read here on each ``GET /api/live``.
    """
    from src.twin import get_twin_bridge

    bridge = get_twin_bridge()
    tel = getattr(bridge, "_last_telemetry", None)
    active = bool(bridge._is_external_active()) if tel is not None else False
    loc = (getattr(tel, "locomotion", None) or {}) if tel else {}
    pose = (getattr(tel, "pose", None) or {}) if tel else {}
    raw = getattr(tel, "raw", None) if tel else None
    sensors = raw.get("sensors") if isinstance(raw, dict) and isinstance(raw.get("sensors"), dict) else {}
    speed = _num(loc.get("speed_m_s"))
    steer = _num(sensors.get("steer"))
    if steer is None:
        steer = _num(loc.get("steer"))
    phase = loc.get("phase") or sensors.get("phase")
    eff = {}
    try:
        eff = bridge._efficiency.snapshot()
    except Exception:
        eff = {}
    if not active:
        return {
            "active": False,
            "phase_label": None,
            "speed_m_s": None,
            "pose": {},
            "steer": None,
            "lane_keep": False,
            "efficiency": {},
        }
    return {
        "active": True,
        "phase_label": _phase_label(phase, loc.get("gait"), speed),
        "speed_m_s": speed,
        "pose": {
            "x_m": _num(pose.get("x_m")),
            "y_m": _num(pose.get("y_m")),
        },
        "steer": steer,
        "lane_keep": bool(
            sensors.get("lane_keep")
            or (getattr(bridge, "_webots_teleop", None) or {}).get("lane_keep")
        ),
        "efficiency": eff,
    }


def _instant_lane(tel) -> dict:
    """Latest cross-track and fit errors on this post, before the lap averages."""
    if tel is None:
        return {}
    loc = getattr(tel, "locomotion", None) or {}
    raw = getattr(tel, "raw", None)
    sensors = raw.get("sensors") if isinstance(raw, dict) and isinstance(raw.get("sensors"), dict) else {}

    def pick(key):
        if loc.get(key) is not None:
            return loc.get(key)
        return sensors.get(key)

    return {
        "cross_track_m": _num(pick("cross_track_m")),
        "ct_max_m": _num(pick("ct_max_m")),
        "width_err_m": _num(pick("width_err_m")),
        "center_err_m": _num(pick("center_err_m")),
    }


def _proof_payload() -> dict:
    """This lap from the bridge energy log, else the newest mission summary file."""
    from src.mission_summary_file import latest_proof_from_disk
    from src.twin import get_twin_bridge

    bridge = get_twin_bridge()
    tel = getattr(bridge, "_last_telemetry", None)
    instant = _instant_lane(tel)
    live = {}
    try:
        live = bridge._energy_log.proof_status()
    except Exception:
        live = {"ok": False}
    if live.get("ok"):
        out = dict(live)
        out["source"] = "This lap"
        # The running max can lag one post behind the sample just ingested.
        if instant.get("cross_track_m") is not None:
            out["cross_track_m"] = instant["cross_track_m"]
        if instant.get("ct_max_m") is not None:
            prev = out.get("ct_max_m")
            got = abs(instant["ct_max_m"])
            out["ct_max_m"] = got if prev is None else max(float(prev), got)
        interesting = (
            "err_pct",
            "steer_frac_camera",
            "steer_frac_held",
            "steer_frac_track",
            "cross_track_m",
            "ct_max_m",
            "width_err_rms_m",
            "center_err_rms_m",
        )
        if any(out.get(key) is not None for key in interesting):
            return out
    saved = latest_proof_from_disk()
    if saved:
        saved["source"] = "Last saved lap"
        return saved
    return {"ok": False, "source": "No lap yet"}


def _build_battery_payload():
    """JSON for ``GET /api/live``.

    Channel watts and battery come from the latest hardware readings (updated
    on each twin post). Pose, speed, steer, efficiency, and proof come from
    the twin bridge, not the 3 s model tick.
    """
    try:
        from src.hardware import get_hardware_mode, get_hardware_source

        hardware = get_hardware_source()
        latest = getattr(hardware, "last_readings", None) or {}
        channels = []
        main_battery = None
        if latest:
            batteries = [d.get("battery", 85) for d in latest.values()]
            main_battery = round(sum(batteries) / len(batteries), 1) if batteries else None
            for ch in config.get("power_channels") or []:
                ch_id = ch.get("id")
                data = latest.get(ch_id, {})
                channels.append({
                    "id": ch_id,
                    "name": ch.get("name", ch_id),
                    "draw": data.get("draw", 0),
                    "battery": data.get("battery", main_battery),
                })
        allocation = getattr(hardware, "allocation_status", {}) or {}
        mission = getattr(hardware, "mission_info", {}) or {}
        if main_battery is None:
            main_battery = 85
        return {
            "main_battery": main_battery,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "hardware_mode": get_hardware_mode(),
            "channels": channels,
            "allocation": {
                "utilization_pct": allocation.get("utilization_pct"),
                "status": allocation.get("status"),
                "energy_wh_remaining": allocation.get("energy_wh_remaining"),
                "runtime_min_at_current_draw": allocation.get("runtime_min_at_current_draw"),
            },
            "mission": {
                "energy_wh_remaining": mission.get("energy_wh_remaining"),
                "runtime_min_at_current_draw": mission.get("runtime_min_at_current_draw"),
            },
            "live": _live_from_bridge(),
            "proof": _proof_payload(),
        }
    except Exception as exc:
        logger.warning(f"Live payload failed: {exc}")

    return {
        "main_battery": 85,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "channels": [],
        "allocation": {},
        "mission": {},
        "live": {"active": False, "speed_m_s": None, "steer": None, "pose": {}, "efficiency": {}},
        "proof": {"ok": False, "source": "No lap yet"},
    }


if __name__ == "__main__":
    run_dashboard()


