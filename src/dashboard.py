# src/dashboard.py
from flask import Flask, render_template_string, jsonify
from flask_socketio import SocketIO
from datetime import datetime
import warnings
import threading
import time

from src.config import config
from src.logger import logger
from src.database import get_all_readings, get_latest_allocation

warnings.filterwarnings("ignore")

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading', logger=False, engineio_logger=False)



HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>🤖 {{ robot_name }} Live Monitor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #0a0a0a; color: #0f0; }
        h1 { color: #0f0; text-align: center; }
        .warning { background: #440000; color: #ff4444; padding: 15px; border-radius: 8px; text-align: center; font-weight: bold; margin: 10px 0; display: none; }
        table { width: 100%; border-collapse: collapse; margin: 20px 0; }
        th, td { padding: 12px; border: 1px solid #333; text-align: left; }
        th { background: #222; }
        .status { text-align: center; color: #666; font-size: 0.9em; }
        .live-dot { display: inline-block; width: 10px; height: 10px; background: #0f0; border-radius: 50%; margin-left: 8px; animation: pulse 1.5s infinite; }
        .live-dot.paused { background: #fa0; animation: none; box-shadow: 0 0 8px #fa0; }
        @keyframes pulse { 0%,100% {opacity:1;} 50% {opacity:0.4;} }
        .live-controls { text-align: center; margin: 12px 0 16px; }
        .pause-btn { font-size: 1.05em; font-weight: bold; padding: 10px 28px; border-radius: 8px; cursor: pointer; border: 2px solid #3a3; }
        .pause-btn.live { background: #132; color: #8f8; border-color: #3a3; }
        .pause-btn.live:hover { background: #1a3a1a; }
        .pause-btn.paused { background: #432; color: #fc8; border-color: #a80; }
        .pause-btn.paused:hover { background: #533; }
        .pause-banner { background: #3a2a00; color: #fc8; border: 2px solid #a80; border-radius: 8px; padding: 12px 18px; text-align: center; font-weight: bold; margin: 0 0 16px; }
        .pause-hint { display: block; color: #888; font-size: 0.82em; margin-top: 6px; }
        .allocation-panel { background: #111; border: 1px solid #333; border-radius: 8px; padding: 16px; margin: 16px 0; }
        .allocation-panel.ok { border-color: #0f0; }
        .allocation-panel.warning { border-color: #fa0; }
        .allocation-panel.throttled { border-color: #f44; }
        .allocation-panel.fault { border-color: #f00; background: #1a0808; }
        .alloc-bar { height: 10px; background: #222; border-radius: 5px; overflow: hidden; margin: 8px 0; }
        .alloc-bar-fill { height: 100%; background: #0f0; transition: width 0.4s; }
        .alloc-warnings { color: #fa0; font-size: 0.9em; margin-top: 8px; }
        .tag { display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 0.8em; background: #333; }
        .tag.throttled { background: #522; color: #f88; }
        .mission-banner { text-align: center; padding: 14px 20px; margin: 16px 0; border-radius: 8px; border: 2px solid #333; background: #111; }
        .mission-banner.idle { border-color: #3a3; color: #8f8; }
        .mission-banner.moving { border-color: #38f; color: #8cf; }
        .mission-banner.balanced { border-color: #3aa; color: #8dd; }
        .mission-banner.high_load { border-color: #f80; color: #fc8; }
        .mission-title { font-size: 1.3em; font-weight: bold; margin-bottom: 6px; }
        .mission-desc { font-size: 0.95em; color: #aaa; max-width: 640px; margin: 0 auto; }
        .mission-meta { display: flex; justify-content: center; gap: 24px; flex-wrap: wrap; margin-top: 10px; font-size: 0.9em; color: #888; }
        .mission-meta strong { color: #ccc; }
        .sim-panel { background: #101820; border: 1px solid #356; border-radius: 8px; padding: 12px 16px; margin: 12px 0; }
        .sim-badge { display: inline-block; padding: 3px 10px; border-radius: 4px; font-size: 0.8em; margin-left: 8px; }
        .sim-badge.running { background: #132; color: #8f8; border: 1px solid #3a3; }
        .sim-badge.stopped { background: #331; color: #fc8; border: 1px solid #a80; }
        .sim-badge.off { background: #222; color: #888; border: 1px solid #444; }
        .sim-timeline { display: flex; gap: 8px; flex-wrap: wrap; margin: 10px 0; }
        .sim-step { flex: 1; min-width: 120px; background: #111; border: 1px solid #333; border-radius: 6px; padding: 8px 10px; font-size: 0.82em; color: #888; }
        .sim-step.active { border-color: #58f; color: #cef; background: #152030; }
        .sim-step.done { border-color: #363; color: #6a8; }
        .sim-controls button { background: #1a2a3a; border: 1px solid #456; color: #9cf; padding: 5px 14px; border-radius: 5px; cursor: pointer; margin-right: 8px; font-size: 0.85em; }
        .sim-controls button:hover { background: #243448; }
        .sim-meta { font-size: 0.85em; color: #8ab; margin-top: 6px; }
        .prediction-panel { background: #0d1117; border: 1px solid #334; border-radius: 8px; padding: 14px 18px; margin: 12px 0; }
        .confidence-bar { height: 8px; background: #222; border-radius: 4px; overflow: hidden; margin: 6px 0; max-width: 280px; display: inline-block; vertical-align: middle; }
        .confidence-fill { height: 100%; background: #6cf; transition: width 0.4s; }
        .prediction-ok { color: #8f8; }
        .prediction-warn { color: #fa0; }
        .risk-badge { display: inline-block; padding: 4px 10px; border-radius: 5px; font-size: 0.85em; font-weight: bold; margin-left: 8px; }
        .risk-low { background: #132; color: #8f8; border: 1px solid #3a3; }
        .risk-medium { background: #331; color: #fc8; border: 1px solid #a80; }
        .risk-high { background: #411; color: #f88; border: 1px solid #a44; }
        .risk-critical { background: #511; color: #f44; border: 1px solid #f00; }
        .horizon-chart { display: flex; align-items: flex-end; gap: 4px; height: 56px; margin: 10px 0 4px; }
        .horizon-bar { flex: 1; background: #1a3a5c; border-radius: 3px 3px 0 0; position: relative; min-width: 28px; transition: height 0.4s; }
        .horizon-bar span { position: absolute; bottom: -18px; left: 50%; transform: translateX(-50%); font-size: 0.7em; color: #888; white-space: nowrap; }
        .horizon-range { font-size: 0.9em; color: #9ab; }
        .phase-outlook { color: #8cf; font-size: 0.92em; margin-top: 6px; }
        .analytics-panel { background: #0d1117; border: 1px solid #334; border-radius: 8px; padding: 14px 18px; margin: 16px 0; }
        .analytics-grid { display: flex; gap: 20px; flex-wrap: wrap; margin: 10px 0; }
        .analytics-stat { background: #111; border: 1px solid #333; border-radius: 6px; padding: 10px 14px; min-width: 120px; }
        .analytics-stat strong { display: block; font-size: 1.1em; color: #8cf; }
        .analytics-stat span { font-size: 0.8em; color: #888; }
        .trend-chart { display: flex; align-items: flex-end; gap: 3px; height: 64px; margin: 12px 0 4px; }
        .trend-bar { flex: 1; background: #2a4a6a; border-radius: 2px 2px 0 0; min-width: 6px; transition: height 0.4s; }
        .analytics-table { width: 100%; font-size: 0.85em; margin-top: 8px; border-collapse: collapse; }
        .analytics-table th { text-align: left; color: #8ab; padding: 4px 8px; border-bottom: 1px solid #333; }
        .analytics-table td { padding: 4px 8px; border-bottom: 1px solid #222; }
        .analytics-section { margin-top: 14px; }
        .analytics-section h3 { font-size: 0.92em; color: #9ab; margin: 0 0 6px; font-weight: normal; }
        .analytics-hours-btn { background: #1a2433; border: 1px solid #345; color: #9cf; padding: 3px 10px; border-radius: 4px; cursor: pointer; margin-right: 6px; font-size: 0.82em; }
        .analytics-hours-btn.active { background: #234; border-color: #58f; color: #def; }
        .battery-trend { display: flex; align-items: flex-end; gap: 3px; height: 40px; margin: 8px 0; }
        .battery-trend-bar { flex: 1; background: #2a5a3a; border-radius: 2px 2px 0 0; min-width: 6px; }
        .ros2-topics { font-size: 0.78em; color: #678; margin-top: 6px; }
        .ros2-cmd-log { font-size: 0.78em; color: #7a9; margin-top: 4px; }
        .ros2-panel { background: #0a1018; border: 1px solid #345; border-radius: 8px; padding: 12px 16px; margin: 12px 0; font-size: 0.9em; }
        .ros2-badge { display: inline-block; padding: 3px 8px; border-radius: 4px; font-size: 0.8em; margin-left: 8px; }
        .ros2-live { background: #132; color: #8f8; border: 1px solid #3a3; }
        .ros2-mock { background: #223; color: #8cf; border: 1px solid #38f; }
        .ros2-off { background: #222; color: #888; border: 1px solid #444; }
        .safety-panel { background: #111; border: 1px solid #333; border-radius: 8px; padding: 16px; margin: 16px 0; }
        .safety-panel.ok { border-color: #0f0; }
        .safety-panel.warning { border-color: #fa0; }
        .safety-panel.fault { border-color: #f44; background: #1a0808; }
        .safety-indicators { display: flex; gap: 16px; flex-wrap: wrap; margin: 10px 0; }
        .safety-badge { padding: 6px 12px; border-radius: 6px; font-size: 0.85em; background: #222; border: 1px solid #444; }
        .safety-badge.ok { border-color: #3a3; color: #8f8; }
        .safety-badge.warning { border-color: #a80; color: #fc8; }
        .safety-badge.critical, .safety-badge.fault { border-color: #a33; color: #f88; }
        .thermal-bar { height: 10px; background: #222; border-radius: 5px; overflow: hidden; margin: 8px 0; max-width: 320px; }
        .thermal-fill { height: 100%; transition: width 0.4s, background 0.4s; }
        .safety-alerts { color: #f88; font-weight: bold; margin: 8px 0; }
        .safety-warnings { color: #fa0; font-size: 0.9em; margin-top: 6px; }
        .lru-panel { background: #0d1117; border: 1px solid #334; border-radius: 8px; padding: 14px 18px; margin: 14px 0; }
        .lru-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 12px; margin-top: 10px; }
        .lru-card { background: #111; border: 1px solid #333; border-radius: 8px; padding: 12px; }
        .lru-card.ok { border-color: #3a3; }
        .lru-card.warning { border-color: #a80; }
        .lru-card.fault { border-color: #a33; background: #1a0808; }
        .lru-card-title { font-weight: bold; margin-bottom: 6px; display: flex; justify-content: space-between; align-items: center; }
        .lru-status-pill { font-size: 0.75em; padding: 2px 8px; border-radius: 4px; background: #222; }
        .lru-status-pill.ok { color: #8f8; }
        .lru-status-pill.warning { color: #fc8; }
        .lru-status-pill.fault { color: #f88; }
        .lru-bar { height: 6px; background: #222; border-radius: 3px; overflow: hidden; margin: 6px 0; }
        .lru-bar-fill { height: 100%; transition: width 0.4s, background 0.4s; }
        .lru-meta { font-size: 0.82em; color: #888; line-height: 1.5; }
        .degrade-badge { display: inline-block; padding: 3px 10px; border-radius: 5px; font-size: 0.8em; margin-left: 8px; background: #132; color: #8f8; border: 1px solid #3a3; }
        .degrade-badge.caution { background: #331; color: #fc8; border-color: #a80; }
        .degrade-badge.degraded, .degrade-badge.critical { background: #511; color: #f88; border-color: #a33; }
        .lru-card.eps { border-width: 2px; grid-column: 1 / -1; background: #0a1018; }
        .lru-card.tier1 { border-color: #48f; }
        .req-row { font-size: 0.78em; color: #7a9; margin-top: 4px; }
        .req-ok { color: #8f8; }
        .req-warn { color: #fc8; }
        .req-fault { color: #f88; }
        .lru-summary { display: flex; gap: 16px; flex-wrap: wrap; margin: 8px 0 12px; font-size: 0.88em; color: #9ab; }
        .lru-summary strong { color: #cde; }
    </style>
</head>
<body>
    <h1>🤖 {{ robot_name }} Live Monitor <span class="live-dot" id="live-dot"></span></h1>
    <div class="live-controls">
        <button type="button" id="pause-resume-btn" class="pause-btn live">⏸ Pause Live Updates</button>
        <span class="pause-hint" id="pause-hint">Freeze the dashboard to inspect allocation, LRU, and forecasts</span>
    </div>
    <div id="pause-banner" class="pause-banner" style="display:none">
        ⏸ LIVE UPDATES PAUSED — frozen snapshot at <strong id="pause-frozen-at">—</strong>
        &nbsp;·&nbsp; Allocation, LRU, and forecast panels hold their last values
    </div>
    <p class="status"><span id="live-status-label">Real-time via WebSocket</span> • Last update: <span id="last-update">—</span>
        • Hardware: <span id="hw-mode">—</span></p>

    <div id="ros2-panel" class="ros2-panel" style="display:none">
        <strong>ROS2 Integration</strong>
        <span id="ros2-badge" class="ros2-badge ros2-off">—</span>
        <span style="color:#666;margin-left:12px">
            Publishes: <span id="ros2-pub-count">0</span>
            &nbsp;|&nbsp; Mode: <span id="ros2-mode">—</span>
            &nbsp;|&nbsp; Node: <span id="ros2-node">—</span>
            &nbsp;|&nbsp; Last task: <span id="ros2-last-task">—</span>
            &nbsp;|&nbsp; Draw: <span id="ros2-last-draw">—</span>W
        </span>
        <div id="ros2-topics" class="ros2-topics"></div>
        <div id="ros2-cmd-log" class="ros2-cmd-log"></div>
    </div>
    
    <div id="warning" class="warning"></div>

    <div id="safety-panel" class="safety-panel ok">
        <h2>Safety &amp; Thermal Status <span id="degrade-badge" class="degrade-badge">NORMAL</span></h2>
        <div class="safety-indicators">
            <span id="safety-status-badge" class="safety-badge ok">SYSTEM OK</span>
            <span id="thermal-badge" class="safety-badge ok">Thermal: —°C</span>
            <span id="battery-safety-badge" class="safety-badge ok">Battery: OK</span>
            <span id="draw-safety-badge" class="safety-badge ok">Power Draw: OK</span>
        </div>
        <p>
            Estimated temperature:
            <strong><span id="thermal-value">—</span>°C</strong>
            (ambient <span id="thermal-ambient">22</span>°C, warn <span id="thermal-warn">55</span>°C, crit <span id="thermal-crit">68</span>°C)
        </p>
        <div class="thermal-bar"><div id="thermal-fill" class="thermal-fill" style="width:0%; background:#0f0"></div></div>
        <div id="safety-alerts" class="safety-alerts"></div>
        <div id="safety-warnings" class="safety-warnings"></div>
    </div>

    <div id="lru-panel" class="lru-panel">
        <strong>LRU Hierarchy &amp; Requirements</strong>
        <span id="req-compliant-badge" class="lru-status-pill ok" style="margin-left:8px">REQUIREMENTS OK</span>
        <div id="lru-summary" class="lru-summary"></div>
        <div id="lru-grid" class="lru-grid"></div>
    </div>

    <div id="sim-panel" class="sim-panel">
        <strong>Mission Simulation</strong>
        <span id="sim-badge" class="sim-badge off">OFF</span>
        <div class="sim-controls" style="margin-top:8px">
            <button type="button" id="sim-start-btn">Start Script</button>
            <button type="button" id="sim-stop-btn">Stop Script</button>
        </div>
        <div id="sim-meta" class="sim-meta">Script: Idle → Transit → Idle → High Load</div>
        <div id="sim-timeline" class="sim-timeline"></div>
    </div>

    <div id="mission-banner" class="mission-banner idle">
        <div class="mission-title">Mission: <span id="mission-label">—</span></div>
        <div class="mission-desc" id="mission-desc">Waiting for telemetry…</div>
        <div class="mission-meta">
            <span>Task ends in: <strong><span id="mission-remaining">—</span>s</strong></span>
            <span>Battery runtime: <strong><span id="mission-runtime">—</span></strong> (at current draw)</span>
            <span>At task avg: <strong><span id="mission-runtime-task">—</span></strong></span>
            <span>Energy left: <strong><span id="mission-energy">—</span> Wh</strong></span>
        </div>
    </div>

    <div id="prediction-panel" class="prediction-panel">
        <strong>Energy Forecast</strong>
        <span id="pred-risk-badge" class="risk-badge risk-low">RISK: LOW</span>
        <p>
            Now: <span id="pred-draw">—</span> W
            &nbsp;|&nbsp; 30s: <span id="pred-30s">—</span>
            <span id="pred-30s-range" class="horizon-range"></span>
            &nbsp;|&nbsp; 60s: <span id="pred-60s">—</span>
            <span id="pred-60s-range" class="horizon-range"></span>
        </p>
        <p>
            Est. runtime: <strong><span id="pred-runtime">—</span></strong>
            <span id="pred-runtime-range" class="horizon-range"></span>
            &nbsp;|&nbsp; Battery in 60s: <span id="pred-battery-60s">—</span>%
            &nbsp;|&nbsp; Mission ends at: <span id="pred-mission-battery">—</span>%
        </p>
        <p>
            Confidence:
            <span class="confidence-bar"><span id="confidence-fill" class="confidence-fill" style="width:0%"></span></span>
            <strong><span id="pred-confidence">—</span>%</strong>
            &nbsp;|&nbsp; Mission energy: <span id="pred-mission-ok">—</span>
            &nbsp;|&nbsp; Trend: <span id="pred-trend">—</span> W/s
        </p>
        <div id="horizon-chart" class="horizon-chart"></div>
        <div id="phase-outlook" class="phase-outlook"></div>
    </div>

    <h2>Main Battery: <span id="main-battery">—</span>%</h2>

    <div id="allocation-panel" class="allocation-panel ok">
        <h2>Power Allocation</h2>
        <p>
            Task: <span id="alloc-task" class="tag">—</span>
            &nbsp;|&nbsp; Budget: <span id="alloc-used">—</span> / <span id="alloc-budget">—</span> W
            (<span id="alloc-pct">—</span>%)
            &nbsp;|&nbsp; System cap: <span id="alloc-system-budget">—</span> W
        </p>
        <div class="alloc-bar"><div id="alloc-bar-fill" class="alloc-bar-fill" style="width:0%"></div></div>
        <p>Status: <span id="alloc-status">—</span></p>
        <div id="alloc-warnings" class="alloc-warnings"></div>
    </div>
    
    <div id="analytics-panel" class="analytics-panel">
        <strong>Historical Analytics</strong>
        <span style="color:#666;font-size:0.85em;margin-left:8px">(last <span id="analytics-hours">1</span>h)</span>
        <span style="margin-left:12px">
            <button class="analytics-hours-btn active" data-hours="1">1h</button>
            <button class="analytics-hours-btn" data-hours="6">6h</button>
            <button class="analytics-hours-btn" data-hours="24">24h</button>
        </span>
        <div class="analytics-grid">
            <div class="analytics-stat"><strong id="hist-snapshots">—</strong><span>Snapshots</span></div>
            <div class="analytics-stat"><strong id="hist-avg-draw">—</strong><span>Avg Draw (W)</span></div>
            <div class="analytics-stat"><strong id="hist-peak-draw">—</strong><span>Peak Draw (W)</span></div>
            <div class="analytics-stat"><strong id="hist-battery-range">—</strong><span>Battery Range</span></div>
            <div class="analytics-stat"><strong id="hist-throttles">—</strong><span>Throttle Events</span></div>
            <div class="analytics-stat"><strong id="hist-tasks">—</strong><span>Distinct Tasks</span></div>
        </div>
        <div class="analytics-section">
            <h3>Power draw trend (minute buckets)</h3>
            <div id="trend-chart" class="trend-chart"></div>
        </div>
        <div class="analytics-section">
            <h3>Battery trend</h3>
            <div id="battery-trend-chart" class="battery-trend"></div>
        </div>
        <div class="analytics-section">
            <h3>LRU group summary</h3>
            <table class="analytics-table" id="lru-summary-table">
                <tr><th>LRU</th><th>Avg W</th><th>Peak W</th><th>Alerts</th><th>Throttles</th></tr>
            </table>
        </div>
        <div class="analytics-section">
            <h3>Channel summary</h3>
            <table class="analytics-table" id="channel-summary-table">
                <tr><th>Channel</th><th>Readings</th><th>Avg W</th><th>Peak W</th><th>Throttles</th></tr>
            </table>
        </div>
        <div class="analytics-section">
            <h3>Mission summaries</h3>
            <table class="analytics-table" id="mission-summary-table">
                <tr><th>Task</th><th>Samples</th><th>Avg W</th><th>Util %</th><th>Alerts</th></tr>
            </table>
        </div>
        <div class="analytics-section">
            <h3>Recent mission history</h3>
            <table class="analytics-table" id="mission-history-table">
                <tr><th>Time</th><th>Task</th><th>Draw W</th><th>Battery %</th><th>Status</th></tr>
            </table>
        </div>
    </div>

    <h2>Power Channels</h2>
    <table id="channels-table">
        <tr><th>Channel</th><th>Current Draw</th><th>Battery %</th></tr>
    </table>

    <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<script>
    const socket = io({
        transports: ['websocket', 'polling'],
        reconnection: true
    });

    let analyticsHours = 1;

    function fillTable(tableId, headers, rows, rowHtml) {
        const table = document.getElementById(tableId);
        table.innerHTML = '<tr>' + headers.map(h => `<th>${h}</th>`).join('') + '</tr>';
        rows.forEach(item => {
            const row = table.insertRow();
            row.innerHTML = rowHtml(item);
        });
        if (!rows.length) {
            const row = table.insertRow();
            row.innerHTML = `<td colspan="${headers.length}" style="color:#666">No data yet — keep the dashboard running.</td>`;
        }
    }

    function loadAnalytics() {
        fetch('/api/analytics?hours=' + analyticsHours)
            .then(r => r.json())
            .then(data => {
                const s = data.summary || {};
                document.getElementById('analytics-hours').innerText = data.window_hours || analyticsHours;
                document.getElementById('hist-snapshots').innerText = s.snapshot_count ?? '—';
                document.getElementById('hist-avg-draw').innerText = s.avg_draw_w ?? '—';
                document.getElementById('hist-peak-draw').innerText = s.peak_draw_w ?? '—';
                document.getElementById('hist-battery-range').innerText =
                    (s.min_battery_pct != null && s.max_battery_pct != null)
                        ? s.min_battery_pct + '–' + s.max_battery_pct + '%' : '—';
                document.getElementById('hist-throttles').innerText = s.throttle_events ?? '—';
                document.getElementById('hist-tasks').innerText = s.distinct_tasks ?? '—';

                const trends = data.power_trends || [];
                const chart = document.getElementById('trend-chart');
                if (trends.length) {
                    const maxW = Math.max(...trends.map(t => t.avg_draw_w || 0), 1);
                    chart.innerHTML = trends.map(t => {
                        const h = Math.max(4, ((t.avg_draw_w || 0) / maxW) * 56);
                        return `<div class="trend-bar" style="height:${h}px" title="${t.bucket}: ${t.avg_draw_w}W"></div>`;
                    }).join('');
                } else {
                    chart.innerHTML = '<span style="color:#666;font-size:0.85em">Collecting trend data…</span>';
                }

                const batChart = document.getElementById('battery-trend-chart');
                if (trends.length) {
                    const minB = Math.min(...trends.map(t => t.avg_battery_pct || 100));
                    const maxB = Math.max(...trends.map(t => t.avg_battery_pct || 0));
                    const span = Math.max(1, maxB - minB);
                    batChart.innerHTML = trends.map(t => {
                        const pct = t.avg_battery_pct || 0;
                        const h = Math.max(4, ((pct - minB) / span) * 36 + 4);
                        return `<div class="battery-trend-bar" style="height:${h}px" title="${t.bucket}: ${pct}%"></div>`;
                    }).join('');
                } else {
                    batChart.innerHTML = '<span style="color:#666;font-size:0.85em">—</span>';
                }

                fillTable('lru-summary-table',
                    ['LRU', 'Avg W', 'Peak W', 'Alerts', 'Throttles'],
                    data.lru_groups || [],
                    l => `<td>${l.lru_label}</td><td>${l.avg_draw_w ?? '—'}</td>`
                        + `<td>${l.max_draw_w ?? '—'}</td><td>${l.alert_count ?? 0}</td>`
                        + `<td>${l.throttle_count ?? 0}</td>`);

                fillTable('channel-summary-table',
                    ['Channel', 'Readings', 'Avg W', 'Peak W', 'Throttles'],
                    data.channels || [],
                    c => `<td>${c.channel}</td><td>${c.reading_count}</td>`
                        + `<td>${c.avg_draw_w ?? '—'}</td><td>${c.max_draw_w ?? '—'}</td>`
                        + `<td>${c.throttle_count ?? 0}</td>`);

                fillTable('mission-summary-table',
                    ['Task', 'Samples', 'Avg W', 'Util %', 'Alerts'],
                    data.missions || [],
                    m => `<td>${m.task}</td><td>${m.snapshot_count}</td>`
                        + `<td>${m.avg_allocated_w ?? '—'}</td>`
                        + `<td>${m.avg_utilization_pct ?? '—'}</td>`
                        + `<td>${m.alert_events ?? 0}</td>`);

                fillTable('mission-history-table',
                    ['Time', 'Task', 'Draw W', 'Battery %', 'Status'],
                    data.mission_history || [],
                    h => `<td>${h.time}</td><td>${h.task}</td>`
                        + `<td>${h.allocated_w ?? '—'}</td><td>${h.battery_pct ?? '—'}</td>`
                        + `<td>${h.status ?? '—'}</td>`);
            })
            .catch(err => console.warn('Analytics fetch failed:', err));
    }

    document.querySelectorAll('.analytics-hours-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.analytics-hours-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            analyticsHours = parseFloat(btn.dataset.hours);
            loadAnalytics();
        });
    });

    function simControl(action) {
        fetch('/api/simulation/' + action, { method: 'POST' })
            .then(r => r.json())
            .then(data => console.log('Simulation', action, data))
            .catch(err => console.warn('Simulation control failed:', err));
    }
    document.getElementById('sim-start-btn').addEventListener('click', () => simControl('start'));
    document.getElementById('sim-stop-btn').addEventListener('click', () => simControl('stop'));

    let livePaused = false;
    let analyticsTimer = null;

    function setLivePaused(paused) {
        livePaused = paused;
        const btn = document.getElementById('pause-resume-btn');
        const dot = document.getElementById('live-dot');
        const banner = document.getElementById('pause-banner');
        const hint = document.getElementById('pause-hint');
        const statusLabel = document.getElementById('live-status-label');

        if (paused) {
            document.getElementById('pause-frozen-at').innerText =
                document.getElementById('last-update').innerText || '—';
            btn.className = 'pause-btn paused';
            btn.innerText = '▶ Resume Live Updates';
            dot.classList.add('paused');
            banner.style.display = 'block';
            hint.innerText = 'WebSocket updates frozen — scroll and inspect any panel';
            statusLabel.innerText = 'Paused (snapshot frozen)';
            if (analyticsTimer) {
                clearInterval(analyticsTimer);
                analyticsTimer = null;
            }
        } else {
            btn.className = 'pause-btn live';
            btn.innerText = '⏸ Pause Live Updates';
            dot.classList.remove('paused');
            banner.style.display = 'none';
            hint.innerText = 'Freeze the dashboard to inspect allocation, LRU, and forecasts';
            statusLabel.innerText = 'Real-time via WebSocket';
            if (!analyticsTimer) {
                analyticsTimer = setInterval(loadAnalytics, 30000);
            }
        }
    }

    document.getElementById('pause-resume-btn').addEventListener('click', () => {
        setLivePaused(!livePaused);
    });

    socket.on('connect', () => {
        console.log('%c[WebSocket] Connected successfully!', 'color: lime');
        loadAnalytics();
        if (!livePaused && !analyticsTimer) {
            analyticsTimer = setInterval(loadAnalytics, 30000);
        }
    });

    socket.on('battery_update', function(data) {
        if (livePaused) return;
        console.log('[WebSocket] Received:', data);

        // Update Main Battery
        document.getElementById('main-battery').innerText = data.main_battery || '--';

        // Update timestamp
        document.getElementById('last-update').innerText = data.timestamp;

        if (data.hardware_mode) {
            document.getElementById('hw-mode').innerText =
                data.hardware_mode.mode + ' / ' + data.hardware_mode.type;
        }

        if (data.ros2) {
            const panel = document.getElementById('ros2-panel');
            panel.style.display = 'block';
            const r = data.ros2;
            const badge = document.getElementById('ros2-badge');
            if (r.active && r.mode === 'live') {
                badge.className = 'ros2-badge ros2-live';
                badge.innerText = 'ROS2 LIVE';
            } else if (r.active) {
                badge.className = 'ros2-badge ros2-mock';
                badge.innerText = 'ROS2 MOCK';
            } else {
                badge.className = 'ros2-badge ros2-off';
                badge.innerText = 'ROS2 OFF';
            }
            document.getElementById('ros2-pub-count').innerText = r.publish_count ?? 0;
            document.getElementById('ros2-mode').innerText = r.mode ?? '—';
            document.getElementById('ros2-node').innerText = r.node_name ?? '—';
            document.getElementById('ros2-last-task').innerText = r.last_payload_task ?? '—';
            document.getElementById('ros2-last-draw').innerText = r.last_total_draw_w ?? '—';
            const topics = r.topics || {};
            document.getElementById('ros2-topics').innerText =
                'Topics: ' + (topics.main_battery || '—') + ' · ' + (topics.mission_command || '—');
            const cmds = (r.recent_commands || []).slice(-3).map(c =>
                (c.accepted ? '✓' : '✗') + ' ' + c.kind + '=' + JSON.stringify(c.value)
            ).join(' · ');
            document.getElementById('ros2-cmd-log').innerText =
                (r.mock_feed_enabled ? 'Mock feed on. ' : '') + (cmds || 'No recent commands');
        }

        // Safety & thermal panel
        if (data.safety) {
            const s = data.safety;
            const panel = document.getElementById('safety-panel');
            const status = s.status || 'ok';
            panel.className = 'safety-panel ' + status;

            const statusBadge = document.getElementById('safety-status-badge');
            statusBadge.className = 'safety-badge ' + status;
            statusBadge.innerText = status === 'fault' ? 'FAULT DETECTED' :
                status === 'warning' ? 'WARNINGS ACTIVE' : 'SYSTEM OK';

            const temp = s.thermal_c ?? 22;
            const tStatus = s.thermal_status || 'normal';
            const thermalBadge = document.getElementById('thermal-badge');
            thermalBadge.className = 'safety-badge ' + (tStatus === 'critical' ? 'critical' : tStatus === 'warning' ? 'warning' : 'ok');
            thermalBadge.innerText = 'Thermal: ' + temp + '°C (' + tStatus.toUpperCase() + ')';
            document.getElementById('thermal-value').innerText = temp;
            if (s.thermal_ambient_c != null) document.getElementById('thermal-ambient').innerText = s.thermal_ambient_c;
            if (s.thermal_warning_c != null) document.getElementById('thermal-warn').innerText = s.thermal_warning_c;
            if (s.thermal_critical_c != null) document.getElementById('thermal-crit').innerText = s.thermal_critical_c;

            const maxTemp = s.thermal_critical_c || 68;
            const ambient = s.thermal_ambient_c || 22;
            const pct = Math.min(100, Math.max(0, ((temp - ambient) / (maxTemp - ambient)) * 100));
            const fill = document.getElementById('thermal-fill');
            fill.style.width = pct + '%';
            fill.style.background = tStatus === 'critical' ? '#f44' : tStatus === 'warning' ? '#fa0' : '#0f0';

            const batBadge = document.getElementById('battery-safety-badge');
            const batPct = data.main_battery ?? s.battery_pct ?? 100;
            if (batPct <= 10) { batBadge.className = 'safety-badge critical'; batBadge.innerText = 'Battery: CRITICAL'; }
            else if (batPct <= 20) { batBadge.className = 'safety-badge warning'; batBadge.innerText = 'Battery: LOW'; }
            else { batBadge.className = 'safety-badge ok'; batBadge.innerText = 'Battery: OK'; }

            const drawBadge = document.getElementById('draw-safety-badge');
            if ((s.faults || []).some(f => f.includes('over-draw'))) {
                drawBadge.className = 'safety-badge fault'; drawBadge.innerText = 'Power Draw: OVER-LIMIT';
            } else if ((s.spike_channels || []).length) {
                drawBadge.className = 'safety-badge warning'; drawBadge.innerText = 'Power Draw: SPIKE';
            } else if (s.throttle_required) {
                drawBadge.className = 'safety-badge warning'; drawBadge.innerText = 'Power Draw: THROTTLED';
            } else {
                drawBadge.className = 'safety-badge ok'; drawBadge.innerText = 'Power Draw: OK';
            }

            const alertEl = document.getElementById('safety-alerts');
            alertEl.innerHTML = (s.alerts || []).map(a => `<div>⚠ ${a}</div>`).join('');
            const warnEl = document.getElementById('safety-warnings');
            const allWarns = [...(s.faults || []), ...(s.warnings || [])];
            warnEl.innerHTML = allWarns.map(w => `<div>• ${w}</div>`).join('');

            const topWarn = document.getElementById('warning');
            if ((s.alerts || []).length) {
                topWarn.style.display = 'block';
                topWarn.innerText = '⚠️ ' + s.alerts[0];
            } else if (batPct <= 20) {
                topWarn.style.display = 'block';
                topWarn.innerText = '⚠️ CRITICAL ALERT — Main battery low!';
            } else {
                topWarn.style.display = 'none';
            }

            const deg = s.degradation_level || 'normal';
            const degEl = document.getElementById('degrade-badge');
            degEl.className = 'degrade-badge ' + deg;
            degEl.innerText = deg.toUpperCase();

            const req = s.requirements || data.requirements || {};
            const reqBadge = document.getElementById('req-compliant-badge');
            if (req.overall_compliant === true) {
                reqBadge.className = 'lru-status-pill ok';
                reqBadge.innerText = 'REQUIREMENTS OK';
            } else if (req.overall_compliant === false) {
                reqBadge.className = 'lru-status-pill fault';
                reqBadge.innerText = 'REQUIREMENTS VIOLATION';
            }

            const summary = document.getElementById('lru-summary');
            if (req.task) {
                summary.innerHTML =
                    `Task: <strong>${req.task}</strong> &nbsp;|&nbsp; `
                    + `Budget: <strong>${req.task_budget_w ?? '—'}W</strong> &nbsp;|&nbsp; `
                    + `Startup: <strong>${req.startup_cost_wh ?? '—'}Wh</strong>`
                    + (req.startup_applied ? ' (applied)' : '');
            }

            const reqMap = {};
            (req.lru_requirements || []).forEach(r => { reqMap[r.id] = r; });
            if (req.eps) reqMap.eps = req.eps;

            const lruData = (s.lru && s.lru.lrus) || [];
            const grid = document.getElementById('lru-grid');
            grid.innerHTML = lruData.map(lru => {
                const st = lru.status || 'ok';
                const barColor = st === 'fault' ? '#f44' : st === 'warning' ? '#fa0' : '#0f0';
                const util = lru.utilization_pct || 0;
                const r = reqMap[lru.id] || {};
                const reqSt = r.status || (r.compliant === false ? 'warning' : 'ok');
                const reqCls = reqSt === 'fault' ? 'req-fault' : reqSt === 'warning' ? 'req-warn' : 'req-ok';
                const tierCls = lru.tier === 1 ? ' eps tier1' : '';
                const reqLine = r.budget_w != null
                    ? `<div class="req-row ${reqCls}">Req: ${r.min_draw_w ?? '—'}–${r.budget_w}–${r.max_draw_w ?? '—'} W</div>`
                    : '';
                const voltLine = lru.id !== 'eps'
                    ? `Est. ${lru.estimated_voltage}V (${lru.voltage_pct}% nominal)`
                    : `Reserve ${r.reserve_pct ?? 12}% · Task cap ${r.budget_w ?? '—'}W`;
                return `<div class="lru-card ${st}${tierCls}">
                    <div class="lru-card-title">
                        ${lru.label}${lru.tier === 1 ? ' <span style="font-size:0.7em;color:#68f">TIER 1</span>' : ''}
                        <span class="lru-status-pill ${st}">${st.toUpperCase()}</span>
                    </div>
                    <div class="lru-bar"><div class="lru-bar-fill" style="width:${Math.min(util,100)}%;background:${barColor}"></div></div>
                    <div class="lru-meta">
                        ${lru.draw_w}W / ${lru.max_draw_w}W (${util}%)<br>
                        ${voltLine}
                    </div>
                    ${reqLine}
                </div>`;
            }).join('');
        }

        // Mission simulation panel
        const sim = data.simulation || (data.mission && data.mission.simulation) || {};
        if (sim.enabled !== false) {
            const simBadge = document.getElementById('sim-badge');
            if (sim.running) {
                simBadge.className = 'sim-badge running';
                simBadge.innerText = 'SCRIPT RUNNING';
            } else if (sim.enabled) {
                simBadge.className = 'sim-badge stopped';
                simBadge.innerText = 'STOPPED';
            } else {
                simBadge.className = 'sim-badge off';
                simBadge.innerText = 'DISABLED';
            }
            const loopTxt = sim.loop ? 'loop on' : 'loop off';
            const loops = sim.loops_completed ? ` · ${sim.loops_completed} loop(s)` : '';
            document.getElementById('sim-meta').innerText =
                `Segment ${sim.segment_index || 0}/${sim.segment_total || 0}: ${sim.segment_label || '—'}`
                + ` · ${sim.segment_remaining_s ?? '—'}s left · ${loopTxt}${loops}`;

            const timeline = document.getElementById('sim-timeline');
            const script = sim.script || [];
            const activeIdx = (sim.segment_index || 1) - 1;
            timeline.innerHTML = script.map((step, i) => {
                const cls = i === activeIdx && sim.running ? 'active'
                    : i < activeIdx && sim.running ? 'done' : '';
                return `<div class="sim-step ${cls}"><strong>${step.label || step.task}</strong><br>`
                    + `${step.duration_s}s · ${step.task}</div>`;
            }).join('');
        }

        // Update mission banner
        if (data.mission || data.allocation) {
            const m = data.mission || data.allocation;
            const taskId = m.task || 'idle';
            const banner = document.getElementById('mission-banner');
            banner.className = 'mission-banner ' + taskId;
            document.getElementById('mission-label').innerText = m.task_label || taskId.toUpperCase();
            document.getElementById('mission-desc').innerText = m.task_description || '';
            document.getElementById('mission-remaining').innerText = m.task_remaining_s ?? '—';
            const rt = m.runtime_min_at_current_draw;
            document.getElementById('mission-runtime').innerText =
                rt != null ? (rt >= 60 ? (rt / 60).toFixed(1) + ' hr' : Math.round(rt) + ' min') : '—';
            const rtt = m.runtime_min_at_task_avg;
            document.getElementById('mission-runtime-task').innerText =
                rtt != null ? Math.round(rtt) + ' min' : '—';
            document.getElementById('mission-energy').innerText = m.energy_wh_remaining ?? '—';
        }

        if (data.prediction || data.mission) {
            const p = data.prediction || data.mission;
            const fmtRuntime = (m) => m != null ? (m >= 60 ? (m / 60).toFixed(1) + ' hr' : Math.round(m) + ' min') : '—';
            const fmtRange = (pt) => pt ? ` (${pt.draw_low_w}–${pt.draw_high_w} W)` : '';

            document.getElementById('pred-draw').innerText = p.predicted_draw_w ?? '—';

            const f30 = p.forecast_30s;
            const f60 = p.forecast_60s;
            document.getElementById('pred-30s').innerText = f30 ? f30.draw_w + ' W' : '—';
            document.getElementById('pred-30s-range').innerText = fmtRange(f30);
            document.getElementById('pred-60s').innerText = f60 ? f60.draw_w + ' W' : '—';
            document.getElementById('pred-60s-range').innerText = fmtRange(f60);

            const pr = p.runtime_min_blended ?? p.predicted_runtime_min ?? p.mission_forecast_min;
            document.getElementById('pred-runtime').innerText = fmtRuntime(pr);
            const rLo = p.runtime_min_low, rHi = p.runtime_min_high;
            document.getElementById('pred-runtime-range').innerText =
                (rLo != null && rHi != null) ? ` (${fmtRuntime(rHi)} – ${fmtRuntime(rLo)})` : '';

            document.getElementById('pred-mission-battery').innerText =
                p.mission_battery_pct_at_end != null ? p.mission_battery_pct_at_end : '—';
            document.getElementById('pred-battery-60s').innerText =
                p.battery_pct_at_60s != null ? p.battery_pct_at_60s : '—';

            const conf = p.confidence_pct ?? 0;
            document.getElementById('pred-confidence').innerText = conf;
            document.getElementById('confidence-fill').style.width = conf + '%';

            const trend = p.draw_trend_w_per_s;
            document.getElementById('pred-trend').innerText =
                trend != null ? (trend >= 0 ? '+' : '') + trend.toFixed(2) : '—';

            const risk = (p.risk_level || 'low').toLowerCase();
            const riskEl = document.getElementById('pred-risk-badge');
            riskEl.className = 'risk-badge risk-' + risk;
            riskEl.innerText = 'RISK: ' + risk.toUpperCase();

            const okEl = document.getElementById('pred-mission-ok');
            if (p.mission_energy_ok === true) {
                okEl.innerHTML = '<span class="prediction-ok">Sufficient</span>';
            } else if (p.mission_energy_ok === false) {
                okEl.innerHTML = '<span class="prediction-warn">Marginal — budget reduced</span>';
            } else {
                okEl.innerText = '—';
            }

            const chart = document.getElementById('horizon-chart');
            const points = p.horizon_points || [];
            if (points.length) {
                const maxW = Math.max(...points.map(pt => pt.draw_high_w), 1);
                chart.innerHTML = points.map(pt => {
                    const h = Math.max(8, (pt.draw_w / maxW) * 48);
                    return `<div class="horizon-bar" style="height:${h}px" title="${pt.draw_w}W (${pt.draw_low_w}–${pt.draw_high_w})"><span>${pt.t_s}s</span></div>`;
                }).join('');
            }

            const outlook = p.locomotion_outlook;
            document.getElementById('phase-outlook').innerText =
                outlook && outlook.outlook ? '🔮 ' + outlook.outlook : '';
        }

        // Update allocation panel
        if (data.allocation) {
            const a = data.allocation;
            const panel = document.getElementById('allocation-panel');
            panel.className = 'allocation-panel ' + (a.status || 'ok');
            document.getElementById('alloc-task').innerText = (a.task_label || a.task || '—');
            document.getElementById('alloc-used').innerText = a.total_allocated_w ?? '—';
            document.getElementById('alloc-budget').innerText = a.budget_w ?? '—';
            document.getElementById('alloc-system-budget').innerText = a.system_budget_w ?? '—';
            document.getElementById('alloc-pct').innerText = a.utilization_pct ?? '—';
            document.getElementById('alloc-status').innerText = (a.status || 'ok').toUpperCase();
            document.getElementById('alloc-bar-fill').style.width = Math.min(a.utilization_pct || 0, 100) + '%';
            document.getElementById('alloc-bar-fill').style.background =
                a.status === 'fault' || a.status === 'throttled' ? '#f44' :
                a.status === 'warning' ? '#fa0' : '#0f0';
            const warnEl = document.getElementById('alloc-warnings');
            warnEl.innerHTML = (a.warnings || []).map(w => `<div>⚠ ${w}</div>`).join('');
        }

        // Update channels table
        const table = document.getElementById('channels-table');
        if (table && data.channels) {
            table.innerHTML = `
                <tr>
                    <th>Channel</th>
                    <th>Draw (W)</th>
                    <th>Req / Alloc</th>
                    <th>Amps</th>
                    <th>% of Max</th>
                    <th>Status</th>
                </tr>`;
            
            data.channels.forEach(ch => {
                const colors = {critical: 'red', warning: 'orange', throttled: '#f88', normal: 'lime'};
                const statusColor = colors[ch.status] || 'lime';
                const throttleTag = ch.throttled ? ' <span class="tag throttled">THROTTLED</span>' : '';
                const row = table.insertRow();
                row.innerHTML = `
                    <td>${ch.name}${throttleTag}</td>
                    <td>${ch.draw}W</td>
                    <td>${ch.requested_w ?? ch.draw} / ${ch.allocated_w ?? ch.draw} W</td>
                    <td>${ch.amps}A</td>
                    <td>${ch.allocation_pct ?? '—'}%</td>
                    <td style="color:${statusColor}">${ch.status.toUpperCase()}</td>
                `;
            });
        }
    });

    socket.on('connect_error', (err) => console.error('Connection error:', err));
</script>
</body>
</html>
'''

@app.route('/')
def dashboard():
    entries = get_all_readings(limit=300)
    main_battery = entries[0]["battery"] if entries else 94

    latest = {}
    for e in entries:
        if e["channel"] not in latest:
            latest[e["channel"]] = e

    channels = []
    power_channels = config.get('power_channels') or []
    for ch in power_channels:
        data = latest.get(ch.get('id', ''), {})
        channels.append({
            "name": ch.get('name', ch.get('id')),
            "draw": data.get("draw", 0),
            "battery": main_battery
        })

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return render_template_string(HTML_TEMPLATE,
                                  robot_name=config.get('robot', 'name', 'Robot'),
                                  main_battery=main_battery,
                                  channels=channels,
                                  now=now)


@app.route('/api/simulation/<action>', methods=['POST'])
def api_simulation(action):
    from src.hardware import get_hardware_source

    hardware = get_hardware_source()
    if action == 'start' and hasattr(hardware, 'start_simulation'):
        return jsonify(hardware.start_simulation())
    if action == 'stop' and hasattr(hardware, 'stop_simulation'):
        return jsonify(hardware.stop_simulation())
    return jsonify({"error": "Simulation not available for this hardware mode"}), 400


@app.route('/api/analytics')
def api_analytics():
    from flask import request
    from src.analytics import build_report

    hours = request.args.get('hours', default=1, type=float)
    hours = max(0.25, min(hours, 168))
    report = build_report(hours=hours)
    return jsonify(report)


@app.route('/api/data')
def api_data():
    entries = get_all_readings(limit=300)
    main_battery = entries[0]["battery"] if entries else 94

    latest = {}
    for e in entries:
        if e["channel"] not in latest:
            latest[e["channel"]] = e

    channels = []
    power_channels = config.get('power_channels') or []
    for ch in power_channels:
        data = latest.get(ch.get('id', ''), {})
        channels.append({
            "draw": data.get("draw", 0),
            "battery": main_battery
        })

    allocation = get_latest_allocation()
    return jsonify({
        "main_battery": main_battery,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "channels": channels,
        "allocation": {
            "task": allocation.get("task") if allocation else None,
            "budget_w": allocation.get("budget_w") if allocation else None,
            "total_allocated_w": allocation.get("total_allocated_w") if allocation else None,
            "utilization_pct": allocation.get("utilization_pct") if allocation else None,
            "status": allocation.get("status") if allocation else None,
            "warnings": allocation.get("warnings") if allocation else [],
        } if allocation else None,
    })


def run_dashboard():
    from src.hardware import get_hardware_source

    hardware = get_hardware_source()
    if not hardware.running:
        hardware.start()

    # Start background services
    start_auto_archiver()
    
    # Start WebSocket broadcaster
    broadcaster_thread = threading.Thread(
        target=broadcast_updates, 
        daemon=True, 
        name="WebSocketBroadcaster"
    )
    broadcaster_thread.start()
    print("[DEBUG] WebSocket broadcaster thread started successfully")

    port = config.get("dashboard", "port", 5000)
    logger.info(f"🚀 Dashboard + WebSocket started on port {port}")
    
    socketio.run(app, host='0.0.0.0', port=port, debug=False, allow_unsafe_werkzeug=True)
    


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
    
def _build_battery_payload():
    """Enhanced payload with engineering metrics"""
    try:
        from src.hardware import get_hardware_source, get_hardware_mode
        hardware = get_hardware_source()
        
        if hasattr(hardware, 'last_readings') and hardware.last_readings:
            latest = hardware.last_readings
            
            # Calculate main battery
            batteries = [d.get('battery', 85) for d in latest.values()]
            main_battery = int(sum(batteries) / len(batteries)) if batteries else 85

            channels = []
            for ch in config.get('power_channels') or []:
                ch_id = ch.get('id')
                data = latest.get(ch_id, {})
                
                channels.append({
                    "id": ch_id,
                    "name": ch.get('name', ch_id),
                    "draw": data.get("draw", 0),
                    "amps": data.get("amps", 0),
                    "battery": data.get("battery", main_battery),
                    "max_draw_w": ch.get("max_draw_w", 30),
                    "voltage": ch.get("nominal_voltage", 48),
                    "requested_w": data.get("requested_w", data.get("draw", 0)),
                    "allocated_w": data.get("allocated_w", data.get("draw", 0)),
                    "allocation_pct": data.get("allocation_pct", 0),
                    "throttled": data.get("throttled", False),
                    "status": data.get("status", "normal")
                })

            allocation = getattr(hardware, "allocation_status", {}) or {}
            mission = getattr(hardware, "mission_info", {}) or {}
            prediction = getattr(hardware, "prediction_status", {}) or {}
            safety = getattr(hardware, "safety_status", {}) or {}
            requirements = getattr(hardware, "requirements_status", {}) or safety.get("requirements", {})
            ros2 = getattr(hardware, "ros2_status", {}) or {}
            simulation = getattr(hardware, "simulation_status", {}) or mission.get("simulation", {})

            return {
                "main_battery": main_battery,
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "hardware_mode": get_hardware_mode(),
                "ros2": ros2,
                "simulation": simulation,
                "channels": channels,
                "mission": mission,
                "prediction": prediction,
                "safety": safety,
                "requirements": requirements,
                "allocation": {
                    "task": allocation.get("task", "unknown"),
                    "task_label": allocation.get("task_label", ""),
                    "task_description": allocation.get("task_description", ""),
                    "task_remaining_s": allocation.get("task_remaining_s", 0),
                    "energy_wh_remaining": allocation.get("energy_wh_remaining"),
                    "runtime_min_at_current_draw": allocation.get("runtime_min_at_current_draw"),
                    "runtime_min_at_task_avg": allocation.get("runtime_min_at_task_avg"),
                    "task_avg_draw_w": allocation.get("task_avg_draw_w"),
                    "predicted_draw_w": prediction.get("predicted_draw_w"),
                    "confidence_pct": prediction.get("confidence_pct"),
                    "mission_energy_ok": prediction.get("mission_energy_ok"),
                    "dynamic_budget_applied": allocation.get("dynamic_budget_applied", False),
                    "budget_w": allocation.get("budget_w", 0),
                    "base_budget_w": allocation.get("base_budget_w", 0),
                    "system_budget_w": allocation.get("system_budget_w", 0),
                    "total_allocated_w": allocation.get("total_allocated_w", 0),
                    "total_requested_w": allocation.get("total_requested_w", 0),
                    "utilization_pct": allocation.get("utilization_pct", 0),
                    "status": allocation.get("status", "ok"),
                    "warnings": allocation.get("warnings", []),
                    "throttled_channels": allocation.get("throttled_channels", []),
                    "safety": allocation.get("safety"),
                }
            }
    except Exception as e:
        print(f"[DEBUG] Payload error: {e}")

    # Fallback
    return {
        "main_battery": 85,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "channels": []
    }

def broadcast_updates():
    """Stable broadcaster with strong debug"""
    print("[DEBUG] WebSocket broadcaster ACTIVE - sending every 4 seconds")
    while True:
        try:
            payload = _build_battery_payload()
            socketio.emit('battery_update', payload)
            print(f"[DEBUG] Emitted update → Main: {payload.get('main_battery')}%")  # Remove later
        except Exception as e:
            print(f"[DEBUG] Broadcast error: {e}")
        
        time.sleep(4)
if __name__ == "__main__":
    run_dashboard()