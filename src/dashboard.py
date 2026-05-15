from flask import Flask, render_template_string, request
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io
import base64
from datetime import datetime
import warnings
import yaml
import threading

warnings.filterwarnings("ignore")

with open('config/config.yaml', 'r') as f:
    config = yaml.safe_load(f)

from src.database import get_all_readings, get_channel_history, log_channel_reading

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
        .refresh { text-align: center; color: #666; font-size: 0.9em; }
        .history-section { display: none; }
    </style>
    <meta http-equiv="refresh" content="{{ refresh }}">
    <script>
        function toggleHistory(channel) {
            var section = document.getElementById('history-' + channel);
            if (section.style.display === "none" || section.style.display === "") {
                section.style.display = "block";
            } else {
                section.style.display = "none";
            }
        }
    </script>
</head>
<body>
    <h1>🤖 {{ robot_name }} Live Monitor</h1>
    <p class="refresh">Auto-refreshing every {{ refresh }}s • {{ now }}</p>
    
    {% if any_low %}
    <div class="warning">⚠️ CRITICAL ALERT — Main battery low!</div>
    {% endif %}

    <h2>Main Battery: {{ main_battery }}%</h2>
    
    <h2>Power Channels</h2>
    <table>
        <tr><th>Channel</th><th>Current Draw</th><th>Battery Impact</th><th>Action</th></tr>
        {% for ch in channels %}
        <tr>
            <td>{{ ch.name }}</td>
            <td>{{ ch.draw }}W</td>
            <td>{{ ch.battery }}%</td>
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
            <tr>
                <td>{{ e.time }}</td>
                <td>{{ e.battery }}%</td>
                <td>{{ e.draw }}W</td>
            </tr>
            {% endfor %}
        </table>
    </div>
    {% endfor %}
</body>
</html>
'''

@app.route('/')
def dashboard():
    entries = get_all_readings(limit=300)
    main_battery = entries[0]["battery"] if entries else 94
    any_low = main_battery <= 20

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
    refresh = config['dashboard']['refresh_seconds']

    return render_template_string(HTML_TEMPLATE, 
                                robot_name=config['robot']['name'],
                                main_battery=main_battery,
                                channels=channels,
                                now=now, refresh=refresh, any_low=any_low)

def run_dashboard():
    from src.simulator import simulate_robot_data
    sim_thread = threading.Thread(target=simulate_robot_data, daemon=True)
    sim_thread.start()
    
    print("🚀 Single Robot Multi-Channel System v4.8 - Toggle History")
    print(f"🌐 Dashboard → http://{config['dashboard']['host']}:{config['dashboard']['port']}")
    print("Click 'Toggle History' on any channel\n")
    
    app.run(debug=False, host=config['dashboard']['host'], port=config['dashboard']['port'])

if __name__ == "__main__":
    run_dashboard()