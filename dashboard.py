from flask import Flask, render_template_string
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io
import base64
import re
import os
from datetime import datetime
import warnings

warnings.filterwarnings("ignore")

app = Flask(__name__)

def parse_log_file():
    log_path = 'robot_battery_log.log'
    entries = []
    if not os.path.exists(log_path):
        return []
    with open(log_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            match = re.search(r'\[(.+?)\]\s*-\s*(\S+)\s*-\s*(\d+)%', line)
            if match:
                timestamp = match.group(1)
                robot_id = match.group(2)
                try:
                    battery = int(match.group(3))
                    entries.append({"time": timestamp, "robot": robot_id, "battery": battery})
                except:
                    continue
    return entries

def generate_plot(entries):
    if not entries:
        return None
    times = [e["time"][-8:] for e in entries]
    levels = [e["battery"] for e in entries]
    
    plt.figure(figsize=(10, 5))
    plt.plot(times, levels, marker='o', linestyle='-', color='cyan', linewidth=2)
    plt.title('Live Robot Battery Levels')
    plt.xlabel('Time')
    plt.ylabel('Battery %')
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.tight_layout()
    
    buf = io.BytesIO()
    plt.savefig(buf, format='png', dpi=120)
    buf.seek(0)
    img_base64 = base64.b64encode(buf.read()).decode('utf-8')
    plt.close()
    return img_base64

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>🤖 Robot Battery Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #0a0a0a; color: #0f0; }
        h1 { color: #0f0; text-align: center; }
        table { width: 100%; border-collapse: collapse; margin: 20px 0; }
        th, td { padding: 12px; border: 1px solid #333; text-align: left; }
        th { background: #222; }
        .low { color: #ff4444; font-weight: bold; }
        .full { color: #00ff88; font-weight: bold; }
        img { max-width: 100%; border: 2px solid #333; margin: 15px 0; }
        .refresh { text-align: center; color: #666; font-size: 0.9em; }
    </style>
    <meta http-equiv="refresh" content="5">
</head>
<body>
    <h1>🤖 Robot Battery Live Dashboard</h1>
    <p class="refresh">Auto-refreshing every 5 seconds • Last updated: {{ now }}</p>
    
    <h2>Current Status</h2>
    <table>
        <tr><th>Time</th><th>Robot</th><th>Battery %</th><th>Status</th></tr>
        {% for e in entries %}
        <tr>
            <td>{{ e.time }}</td>
            <td>{{ e.robot }}</td>
            <td>{{ e.battery }}%</td>
            <td class="{% if e.battery <= 20 %}low{% elif e.battery >= 95 %}full{% endif %}">
                {% if e.battery <= 20 %}CRITICALLY LOW{% elif e.battery >= 95 %}FULLY CHARGED{% else %}Normal{% endif %}
            </td>
        </tr>
        {% endfor %}
    </table>

    <h2>Live Graph</h2>
    {% if plot %}
        <img src="data:image/png;base64,{{ plot }}" alt="Battery Graph">
    {% else %}
        <p>No data yet...</p>
    {% endif %}
</body>
</html>
'''

@app.route('/')
def dashboard():
    entries = parse_log_file()
    plot = generate_plot(entries)
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return render_template_string(HTML_TEMPLATE, entries=entries, plot=plot, now=now)

if __name__ == '__main__':
    print("🚀 Robot Battery Dashboard v4.0 Ready!")
    print("🌐 Open browser → http://127.0.0.1:5000")
    print("Leave this window open!\n")
    app.run(debug=False, host='127.0.0.1', port=5000)