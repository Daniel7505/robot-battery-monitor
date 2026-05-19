# src/dashboard.py
from flask import Flask, render_template_string, jsonify
from datetime import datetime
import warnings

from src.config import config
from src.logger import logger
from src.database import get_all_readings, get_channel_history

warnings.filterwarnings("ignore")

app = Flask(__name__)

HTML_TEMPLATE = '''<!DOCTYPE html>
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
    </style>
</head>
<body>
    <h1>🤖 {{ robot_name }} Live Monitor <span class="live-dot"></span></h1>
    <p class="status">Live updates every 2 seconds • Last update: <span id="last-update">{{ now }}</span></p>
    
    <div id="warning" style="display: none;" class="warning">⚠️ CRITICAL ALERT — Main battery low!</div>

    <h2>Main Battery: <span id="main-battery">{{ main_battery }}</span>%</h2>
    
    <h2>Power Channels</h2>
    <table>
        <tr><th>Channel</th><th>Current Draw</th><th>Battery %</th></tr>
        {% for ch in channels %}
        <tr>
            <td>{{ ch.name }}</td>
            <td>{{ ch.draw }}W</td>
            <td>{{ ch.battery }}%</td>
        </tr>
        {% endfor %}
    </table>

    <script>
        async function fetchAndUpdate() {
            try {
                const res = await fetch('/api/data');
                const data = await res.json();
                
                document.getElementById('main-battery').innerText = data.main_battery;
                
                const warn = document.getElementById('warning');
                warn.style.display = (data.main_battery <= 20) ? 'block' : 'none';
                
                document.getElementById('last-update').innerText = data.timestamp;
            } catch(e) {}
        }
        setInterval(fetchAndUpdate, 2000);
        fetchAndUpdate();
    </script>
</body>
</html>'''

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
            "id": ch.get('id'),
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
            "id": ch.get('id'),
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
    hardware