#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
action="${1:-plan}"
if [[ $# -gt 0 ]]; then shift; fi
case "$action" in
  plan|check|apply|run) ;;
  *) printf '%s\n' 'Usage: ./hardware.sh [plan|check|apply|run] [configuration.yaml]' >&2; exit 2 ;;
esac
hardware_config="${1:-$PWD/src/dual_arm_hardware/config/hardware.yaml}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/log/hardware_ros}"
export PYTHONNOUSERSITE=1
if [[ "$action" == run ]]; then
  # Fail on an incomplete configuration before building or opening a serial port.
  /usr/bin/python3 src/dual_arm_hardware/scripts/hardware_limits.py plan \
    --config "$hardware_config" --description-config "$PWD/src/dual_arm_description/config"
  colcon build --symlink-install --packages-select dual_arm_description dual_arm_hardware
  source install/local_setup.bash
  exec ros2 launch dual_arm_hardware hardware.launch.py "hardware_config:=$hardware_config"
fi
exec /usr/bin/python3 src/dual_arm_hardware/scripts/hardware_limits.py "$action" \
  --config "$hardware_config" --description-config "$PWD/src/dual_arm_description/config"
