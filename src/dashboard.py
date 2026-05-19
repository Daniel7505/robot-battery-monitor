# src/dashboard.py
from flask import Flask, render_template_string, jsonify
from flask_socketio import SocketIO, emit
from datetime import datetime
import warnings
import threading
import time

from src.config import config
from src.logger import logger
from src.database import get_all_readings, get_channel_history

warnings.filterwarnings("ignore")

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

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
        @keyframes pulse { 0%,100% {opacity:1;} 50% {opacity:0.4;} }
    </style>
</head>
<body>
    <h1>🤖 {{ robot_name }} Live Monitor <span class="live-dot"></span></h1>
    <p class="status">Real-time via WebSocket • Last update: <span id="last-update">—</span></p>
    
    <div id="warning" class="warning">⚠️ CRITICAL ALERT — Main battery low!</div>

    <h2>Main Battery: <span id="main-battery">—</span>%</h2>
    
    <h2>Power Channels</h2>
    <table id="channels-table">
        <tr><th>Channel</th><th>Current Draw</th><th>Battery %</th></tr>
    </table>

    <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
    <script>
        console.log("Socket.IO script loaded");

        const socket = io({
            transports: ['websocket', 'polling'],
            reconnection: true,
            reconnectionAttempts: 5,
            reconnectionDelay: 1000
        });

        socket.on('connect', () => {
            console.log('%c[WebSocket] Connected successfully!', 'color: lime');
        });

        socket.on('connect_error', (err) => {
            console.error('[WebSocket] Connection error:', err);
        });

        socket.on('battery_update', function(data) {
            console.log('[WebSocket] Received battery_update:', data);

            const mainEl = document.getElementById('main-battery');
            if (mainEl) mainEl.innerText = data.main_battery;

            const timeEl = document.getElementById('last-update');
            if (timeEl) timeEl.innerText = data.timestamp;

            const warn = document.getElementById('warning');
            if (warn) warn.style.display = (data.main_battery <= 20) ? 'block' : 'none';

            const table = document.getElementById('channels-table');
            if (table) {
                table.innerHTML = '<tr><th>Channel</th><th>Current Draw</th><th>Battery %</th></tr>';
                data.channels.forEach(ch => {
                    const row = table.insertRow();
                    row.innerHTML = `<td>${ch.name}</td><td>${ch.draw}W</td><td>${ch.battery}%</td>`;
                });
            }
        });

        socket.on('disconnect', () => {
            console.warn('[WebSocket] Disconnected');
        });
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

    return jsonify({
        "main_battery": main_battery,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "channels": channels
    })


def run_dashboard():
    from src.hardware import get_hardware_source

    hardware = get_hardware_source()
    hardware.start()

    # Start auto archiver + websocket broadcaster
    start_auto_archiver()
    threading.Thread(target=broadcast_updates, daemon=True, name="WebSocketBroadcaster").start()

    port = config.get("dashboard", "port", 5000)
    logger.info(f"🚀 Dashboard + WebSocket started on port {port}")
    socketio.run(app, host='0.0.0.0', port=port, debug=False)
    


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
    """Builds the data payload for WebSocket emission."""
    entries = get_all_readings(limit=50)

    if not entries:
        return {
            "main_battery": 0,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "channels": []
        }

    main_battery = entries[0]["battery"]

    latest = {}
    for e in entries:
        if e["channel"] not in latest:
            latest[e["channel"]] = e

    channels = []
    power_channels = config.get('power_channels') or []
    for ch in power_channels:
        data = latest.get(ch.get('id', ''), {})
        channels.append({
            "id": ch.get('id'),
            "name": ch.get('name', ch.get('id')),
            "draw": data.get("draw", 0),
            "battery": main_battery
        })

    return {
        "main_battery": main_battery,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "channels": channels
    }


def broadcast_updates():
    """Background thread that emits battery updates via WebSocket."""
    while True:
        try:
            payload = _build_battery_payload()
            socketio.emit('battery_update', payload)

        except Exception as e:
            logger.error(f"WebSocket broadcast error: {e}")

        time.sleep(2)
    
if __name__ == "__main__":
    run_dashboard()