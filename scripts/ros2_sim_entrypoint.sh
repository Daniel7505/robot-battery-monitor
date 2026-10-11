#!/bin/bash
set -e

source /opt/ros/rolling/setup.bash

echo "==> ROS2 sim — installing Python deps"
apt-get update -qq
apt-get install -y -qq python3-pip > /dev/null
pip3 install pyyaml --break-system-packages -q

export PYTHONPATH="/app:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "==> Waiting for dashboard..."
sleep "${ROS2_START_DELAY:-8}"

echo "==> Starting ROS2 simulation node (domain ${ROS_DOMAIN_ID})"
exec python3 scripts/ros2_sim_node.py