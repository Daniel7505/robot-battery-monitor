#!/usr/bin/env bash
# =============================================================================
# launch_webots_twin.sh — open ButlerBot world + twin bridge to the dashboard
# =============================================================================
# Prerequisites: dashboard (./scripts/start.sh) and Webots installed.
# Exports TWIN_DASHBOARD_URL for the controller's twin_publisher HTTP client.
#
# Usage: ./scripts/launch_webots_twin.sh [dashboard_url] [world]
#   world: butlerbot (default), corner90, corner_mix, widen, plus, t_end, gap45, warehouse, warehouse_obstacles, radar_motion, or a .wbt path (RBM_WORLD too)
#   RBM_TRACK is set from the world's # TRACK_FILE tag.
# The controller lane-keeps on its own (rowfit); the agent drives via the twin API.
# =============================================================================

set -euo pipefail
DASHBOARD_URL="${1:-http://127.0.0.1:5000}"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WEBOTS_DIR="$PROJECT_ROOT/webots"
WORLD_ARG="${2:-${RBM_WORLD:-butlerbot}}"
WORLD=""
for cand in "$WORLD_ARG" "$WEBOTS_DIR/worlds/$WORLD_ARG" "$WEBOTS_DIR/worlds/$WORLD_ARG.wbt" \
            "$WEBOTS_DIR/worlds/butlerbot_$WORLD_ARG.wbt"; do
  if [ -f "$cand" ] && [[ "$cand" == *.wbt ]]; then WORLD="$(cd "$(dirname "$cand")" && pwd)/$(basename "$cand")"; break; fi
done
if [ -z "$WORLD" ]; then
  echo "ERROR: world '$WORLD_ARG' not found in $WEBOTS_DIR/worlds" >&2
  exit 1
fi

# Track for the finish referee from the world's "# TRACK_FILE" tag; a different
# RBM_TRACK left in the shell is a leftover from an earlier run: override it.
TAG="$(sed -n 's/^# TRACK_FILE \(.*\)$/\1/p' "$WORLD" | head -n 1 | tr -d '\r')"
if [ -n "$TAG" ]; then
  TAG_NAME="$(basename "$TAG" .json)"
  if [ -n "${RBM_TRACK:-}" ] && [ "$(basename "${RBM_TRACK//\\//}" .json)" != "$TAG_NAME" ]; then
    echo "WARNING: RBM_TRACK=$RBM_TRACK does not match world $(basename "$WORLD") (track $TAG_NAME) — using $TAG_NAME" >&2
  fi
  export RBM_TRACK="$TAG_NAME"
fi
export RBM_WORLD_FILE="$WORLD"

echo "ButlerBot Webots Digital Twin"
echo "Dashboard: $DASHBOARD_URL"
echo "World:     $WORLD"
echo "Track:     ${RBM_TRACK:-(none: S default)}"
echo "Route:     RBM_ROUTE=${RBM_ROUTE:-(unset)}  RBM_MAX_BLIND_M=${RBM_MAX_BLIND_M:-(4.0)}"
echo ""

if curl -sf "$DASHBOARD_URL/api/twin/schema" >/dev/null 2>&1; then
  echo "Dashboard twin API OK"
else
  echo "WARNING: Dashboard not reachable at $DASHBOARD_URL"
  echo "Start it first: ./scripts/start.sh"
  echo ""
fi

export TWIN_DASHBOARD_URL="$DASHBOARD_URL"
export WEBOTS_PROJECT_HOME="$WEBOTS_DIR"

WEBOTS_BIN="${WEBOTS_HOME:-/usr/local/webots}/webots"
if ! command -v webots >/dev/null 2>&1 && [ -x "$WEBOTS_BIN" ]; then
  WEBOTS_CMD="$WEBOTS_BIN"
elif command -v webots >/dev/null 2>&1; then
  WEBOTS_CMD="webots"
else
  echo "ERROR: Webots not found. Install from https://cyberbotics.com/download"
  exit 1
fi

# Avoid two Webots instances fighting over the same world/controller
if pgrep -x webots >/dev/null 2>&1; then
  echo "Closing existing Webots process(es)..."
  pkill -x webots || true
  sleep 2
fi

cd "$WEBOTS_DIR"
echo "Launching ButlerBot world (realtime) — close Webots window to exit."
exec "$WEBOTS_CMD" --mode=realtime --stdout --stderr "$WORLD"