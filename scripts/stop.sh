#!/usr/bin/env sh
set -e
cd "$(dirname "$0")/.."
docker compose --profile full down
echo "Stack stopped."