#!/usr/bin/env sh
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

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

echo ""
echo "Waiting for dashboard..."
attempt=0
while [ "$attempt" -lt 30 ]; do
  if curl -sf http://127.0.0.1:5000/ > /dev/null 2>&1; then
    echo ""
    echo "Dashboard ready: http://127.0.0.1:5000"
    docker compose ps
    exit 0
  fi
  attempt=$((attempt + 1))
  sleep 2
done

echo "Dashboard did not respond in time — check logs with: docker compose logs dashboard"
docker compose ps
exit 1