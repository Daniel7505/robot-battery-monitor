#!/bin/sh
# =============================================================================
# docker-entrypoint.sh — dashboard container boot sequence
# =============================================================================
# Ordered startup inside the `dashboard` service image:
#   1. Block until Postgres accepts connections (wait_for_postgres)
#   2. Create/migrate schema if needed (setup_postgres)
#   3. Replace this shell with the Flask+SocketIO process (run_dashboard)
# =============================================================================
set -e

echo "==> Robot Battery Monitor — container startup"
python scripts/wait_for_postgres.py
python scripts/setup_postgres.py
echo "==> Starting dashboard..."
exec python run_dashboard.py