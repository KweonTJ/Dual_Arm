#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# MuJoCo is installed in the user's Python site in this workspace environment.
unset PYTHONNOUSERSITE
# Real twin mode uses the dedicated SDK/pyserial installation, while MuJoCo/Qt
# continue to run in the existing desktop runtime.
twin_real=false
previous_arg=''
for argument in "$@"; do
  if [[ "$argument" == --twin=real || ( "$previous_arg" == --twin && "$argument" == real ) ]]; then twin_real=true; fi
  previous_arg="$argument"
done
if "$twin_real" && [[ -x "$PWD/.venv-hardware/bin/python" ]]; then
  hardware_site="$("$PWD/.venv-hardware/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  export PYTHONPATH="$hardware_site${PYTHONPATH:+:$PYTHONPATH}"
fi
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
