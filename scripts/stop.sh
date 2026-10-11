#!/usr/bin/env sh
# stop.sh — tear down the Docker stack (includes optional full-profile services)
# Usage: ./scripts/stop.sh
set -e
cd "$(dirname "$0")/.."
docker compose --profile full down
echo "Stack stopped."