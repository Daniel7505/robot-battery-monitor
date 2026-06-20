#!/bin/sh
set -e

echo "==> Robot Battery Monitor — container startup"
python scripts/wait_for_postgres.py
python scripts/setup_postgres.py
echo "==> Starting dashboard..."
exec python run_dashboard.py