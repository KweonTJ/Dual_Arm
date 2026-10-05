#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ "${1:-}" == --sim ]]; then
  shift
  exec ./simulation.sh "$@"
fi
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/log/monitor_ros}"
export PYTHONNOUSERSITE=1
# Run the current source so display changes are reflected without a rebuild.
exec /usr/bin/python3 src/dual_arm_monitor/scripts/monitor_node.py "$@"
