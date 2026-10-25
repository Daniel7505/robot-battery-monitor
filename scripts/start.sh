#!/usr/bin/env sh
# =============================================================================
# start.sh — bring up the Robot Battery Monitor Docker stack (Linux/macOS)
# =============================================================================
# Core (default): Postgres + PMS dashboard on DASHBOARD_PORT (default 5000)
# Full:           ./scripts/start.sh full  → also starts ROS2 sim
#
# After success: open http://127.0.0.1:<DASHBOARD_PORT> ; optional Webots via launch_webots_twin.sh
# =============================================================================
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# Seed env on first run so compose has DB credentials
if [ ! -f .env ] && [ -f .env.example ]; then
  echo "No .env found — copying .env.example"
  cp .env.example .env
fi

PROFILE="${1:-}"
if [ "$PROFILE" = "full" ]; then
  echo "Starting full stack (dashboard + Postgres + ROS2 sim)..."
  docker compose --profile full up --build -d
else
  echo "Starting core stack (dashboard + Postgres)..."
  docker compose up --build -d
fi

# Same port compose publishes: shell env, else .env, else 5000.
PORT="${DASHBOARD_PORT:-}"
if [ -z "$PORT" ] && [ -f .env ]; then
  PORT="$(sed -n 's/^[[:space:]]*DASHBOARD_PORT[[:space:]]*=[[:space:]]*\([0-9][0-9]*\)[[:space:]]*$/\1/p' .env | head -n 1)"
fi
PORT="${PORT:-5000}"
DASH_URL="http://127.0.0.1:${PORT}/"

# Wait until Flask answers (entrypoint may still be initializing Postgres tables)
echo ""
echo "Waiting for dashboard on ${DASH_URL} ..."
attempt=0
while [ "$attempt" -lt 30 ]; do
  if curl -sf "$DASH_URL" > /dev/null 2>&1; then
    echo ""
    echo "Dashboard ready: ${DASH_URL}"
    docker compose ps
    exit 0
  fi
  attempt=$((attempt + 1))
  sleep 2
done

echo "Dashboard did not respond in time — check logs with: docker compose logs dashboard"
docker compose ps
exit 1