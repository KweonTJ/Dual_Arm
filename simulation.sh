#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# MuJoCo is installed in the user's Python site in this workspace environment.
unset PYTHONNOUSERSITE
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/log/simulation_ros}"
simulation_dir="$PWD/log/simulation"
mkdir -p "$simulation_dir"
# Rebuild the physics input from current Xacro and YAML on every launch.
xacro "$PWD/src/dual_arm_description/urdf/dual_arm.urdf.xacro" -o "$simulation_dir/current.urdf"
exec /usr/bin/python3 src/dual_arm_simulation/scripts/simulation_app.py \
  --urdf "$simulation_dir/current.urdf" \
  --description-dir "$PWD/src/dual_arm_description" \
  --config "$PWD/src/dual_arm_simulation/config/simulation.yaml" \
  --output-dir "$simulation_dir" "$@"
