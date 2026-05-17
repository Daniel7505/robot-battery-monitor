# src/dashboard.py - Reliable polling version (no SocketIO threading issues)
from flask import Flask, render_template_string, jsonify
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io
import base64
from datetime import datetime
import warnings
import yaml

warnings.filterwarnings("ignore")

with open('config/config.yaml', 'r') as f:
    config = yaml.safe_load(f)

from src.database import get_all_readings, get_channel_history
from src.hardware import get_hardware_source

app = Flask(__name__)

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>🤖 {{ robot_name }} Live Monitor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #0a0a0a; color: #0f0; }
        h1 { color: #0f0; text-align: center; }
        .warning { background: #440000; color: #ff4444; padding: 15px; border-radius: 8px; text-align: center; font-weight: bold; margin: 10px 0; }
        button { padding: 8px 16px; margin: 4px; background: #222; color: #0f0; border: 1px solid #444; cursor: pointer; }
        button:hover { background: #333; }
        table { width: 100%; border-collapse: collapse; margin: 20px 0; }
        th, td { padding: 12px; border: 1px solid #333; text-align: left; }
        th { background: #222; }
        .low { color: #ff4444; font-weight: bold; }
        .full { color: #00ff88; font-weight: bold; }
        .status { text-align: center; color: #666; font-size: 0.9em; }
        .live-dot { display: inline-block; width: 10px; height: 10px; background: #0f0; border-radius: 50%; margin-left: 8px; animation: pulse 1.5s infinite; }
        @keyframes pulse { 0%,100% {opacity:1;} 50% {opacity:0.4;} }
        .history-section { display: none; }
    </style>
</head>
<body>
    <h1>🤖 {{ robot_name }} Live Monitor <span class="live-dot"></span></h1>
    <p class="status">Live updates every 2 seconds • Last update: <span id="last-update">{{ now }}</span></p>
    
    <div id="warning" style="display: none;" class="warning">⚠️ CRITICAL ALERT — Main battery low!</div>

    <h2>Main Battery: <span id="main-battery">{{ main_battery }}</span>%</h2>
    
    <h2>Power Channels</h2>
    <table id="channels-table">
        <tr><th>Channel</th><th>Current Draw</th><th>Battery Impact</th><th>Action</th></tr>
        {% for ch in channels %}
        <tr id="row-{{ ch.id }}">
            <td>{{ ch.name }}</td>
            <td id="draw-{{ ch.id }}">{{ ch.draw }}W</td>
            <td id="bat-{{ ch.id }}">{{ ch.battery }}%</td>
            <td><button onclick="toggleHistory('{{ ch.id }}')">Toggle History</button></td>
        </tr>
        {% endfor %}
    </table>

    {% for ch in channels %}
    <div id="history-{{ ch.id }}" class="history-section">
        <h2>{{ ch.name }} — Last 30 Readings</h2>
        <table>
            <tr><th>Time</th><th>Battery %</th><th>Draw (W)</th></tr>
            {% for e in ch.history %}
            <tr><td>{{ e.time }}</td><td>{{ e.battery }}%</td><td>{{ e.draw }}W</td></tr>
            {% endfor %}
        </table>
    </div>
    {% endfor %}

    <script>
        function updateDashboard(data) {
            // Main battery
            const mainEl = document.getElementById('main-battery');
            if (mainEl) mainEl.innerText = data.main_battery;
            
            // Warning
            const warn = document.getElementById('warning');
            if (warn) warn.style.display = (data.main_battery <= 20) ? 'block' : 'none';
            
            // Timestamp
            const timeEl = document.getElementById('last-update');
            if (timeEl) timeEl.innerText = data.timestamp;
            
            // Update each channel
            if (data.channels) {
                data.channels.forEach(ch => {
                    const drawEl = document.getElementById('draw-' + ch.id);
                    const batEl = document.getElementById('bat-' + ch.id);
                    if (drawEl) drawEl.innerText = ch.draw + 'W';
                    if (batEl) batEl.innerText = ch.battery + '%';
                });
            }
        }

        async function fetchAndUpdate() {
            try {
                const res = await fetch('/api/data');
                const data = await res.json();
                updateDashboard(data);
            } catch (e) {
                console.log('Update error (will retry):', e);
            }
        }

        // Initial load + poll every 2 seconds
        setInterval(fetchAndUpdate, 2000);
        fetchAndUpdate(); // run immediately

        function toggleHistory(channel) {
            const section = document.getElementById('history-' + channel);
            section.style.display = (section.style.display === 'none' || section.style.display === '') ? 'block' : 'none';
        }
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
    for ch in config.get('power_channels', []):
        data = latest.get(ch['id'], {})
        history = get_channel_history(ch['id'], limit=30)
        channels.append({
            "id": ch['id'],
            "name": ch['name'],
            "draw": data.get("draw", 0),
            "battery": main_battery,
            "history": history
        })

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return render_template_string(HTML_TEMPLATE,
                                  robot_name=config['robot']['name'],
                                  main_battery=main_battery,
                                  channels=channels,
                                  now=now)

@app.route('/api/data')
def api_data():
    """JSON endpoint for live polling"""
    entries = get_all_readings(limit=300)
    main_battery = entries[0]["battery"] if entries else 94

    latest = {}
    for e in entries:
        if e["channel"] not in latest:
            latest[e["channel"]] = e

    channels = []
    for ch in config.get('power_channels', []):
        data = latest.get(ch['id'], {})
        channels.append({
            "id": ch['id'],
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
    import threading
    
    # Start the hardware source (simulator or real)
    hardware = get_hardware_source()
    hardware.start()

    print("🚀 Real-time Robot Battery Monitor started inside Docker")
    print("🌐 Listening on http://0.0.0.0:5000")
    print("   → Open in browser: http://127.0.0.1:5000\n")
    
    # CRITICAL for Docker: bind to all interfaces
    app.run(host='0.0.0.0', port=config['dashboard']['port'], debug=False)
    

if __name__ == "__main__":
    run_dashboard()