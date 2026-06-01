# src/dashboard.py
from flask import Flask, render_template_string, jsonify
from flask_socketio import SocketIO
from datetime import datetime
import warnings
import threading
import time

from src.config import config
from src.logger import logger
from src.database import get_all_readings

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
    const socket = io({
        transports: ['websocket', 'polling'],
        reconnection: true
    });

    socket.on('connect', () => {
        console.log('%c[WebSocket] Connected successfully!', 'color: lime');
    });

    socket.on('battery_update', function(data) {
        console.log('[WebSocket] Received:', data);

        // Update Main Battery
        document.getElementById('main-battery').innerText = data.main_battery || '--';

        // Update timestamp
        document.getElementById('last-update').innerText = data.timestamp;

        // Update channels table
        const table = document.getElementById('channels-table');
        if (table && data.channels) {
            table.innerHTML = `
                <tr>
                    <th>Channel</th>
                    <th>Draw (W)</th>
                    <th>Amps</th>
                    <th>Battery %</th>
                    <th>Status</th>
                </tr>`;
            
            data.channels.forEach(ch => {
                const statusColor = ch.status === 'critical' ? 'red' : ch.status === 'warning' ? 'orange' : 'lime';
                const row = table.insertRow();
                row.innerHTML = `
                    <td>${ch.name}</td>
                    <td>${ch.draw}W</td>
                    <td>${ch.amps}A</td>
                    <td>${ch.battery}%</td>
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
        from src.hardware import get_hardware_source
        hardware = get_hardware_source()
        
        if hasattr(hardware, 'last_readings') and hardware.last_readings:
            latest = hardware.last_readings
            
            # Calculate main battery
            batteries = [d.get('battery', 85) for d in latest.values()]
            main_battery = int(sum(batteries) / len(batteries)) if batteries else 85

            channels = []
            for ch in config.get('power_channels', []):
                ch_id = ch.get('id')
                data = latest.get(ch_id, {})
                
                channels.append({
                    "name": ch.get('name', ch_id),
                    "draw": data.get("draw", 0),
                    "amps": data.get("amps", 0),
                    "battery": data.get("battery", main_battery),
                    "max_draw_w": ch.get("max_draw_w", 30),
                    "voltage": ch.get("nominal_voltage", 48),
                    "status": data.get("status", "normal")
                })
            
            return {
                "main_battery": main_battery,
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "channels": channels
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