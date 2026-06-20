# Robot Battery Monitor — dashboard container
FROM python:3.12-slim

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

RUN mkdir -p logs archives \
    && chmod +x scripts/docker-entrypoint.sh scripts/ros2_sim_entrypoint.sh scripts/start.sh scripts/stop.sh

EXPOSE 5000

HEALTHCHECK --interval=15s --timeout=5s --start-period=25s --retries=5 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:5000/')" || exit 1

ENTRYPOINT ["/bin/sh", "scripts/docker-entrypoint.sh"]