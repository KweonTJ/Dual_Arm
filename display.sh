#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

preview_only=false
launch_args=()
model_args=()
for arg in "$@"; do
  case "$arg" in
    --preview-only) preview_only=true ;;
    --help|-h)
      printf '%s\n' \
        'Usage: ./display.sh [--preview-only] [ROS launch arguments]' \
        'Regenerates meshes, URDF, PNG and HTML before opening RViz.' \
        '--preview-only: update files without starting ROS nodes or GUI.' \
        'RViz: Move Camera = left drag rotate, wheel zoom, Shift+left drag pan.' \
        'Focus Camera: click a part to set the rotation centre, then zoom.' \
        'Example: ./display.sh mount_spacing:=0.30 mount_height:=0.45'
      exit 0 ;;
    *)
      launch_args+=("$arg")
      case "$arg" in
        dimensions_file:=*|mount_spacing:=*|mount_height:=*) model_args+=("$arg") ;;
      esac ;;
  esac
done

description="$PWD/src/dual_arm_description"
printf '%s\n' '[1/4] Updating part meshes'
PYTHONNOUSERSITE=1 /usr/bin/python3 "$description/scripts/generate_meshes.py"

source /opt/ros/humble/setup.bash
printf '%s\n' '[2/4] Building description package'
colcon build --symlink-install --packages-select dual_arm_description
source install/local_setup.bash

printf '%s\n' '[3/4] Updating URDF with the launch dimensions'
urdf_temp="$(mktemp "$description/urdf/.dual_arm.XXXXXX.urdf")"
trap 'rm -f -- "$urdf_temp"' EXIT
xacro "$description/urdf/dual_arm.urdf.xacro" "${model_args[@]}" -o "$urdf_temp"
mv -- "$urdf_temp" "$description/urdf/dual_arm.urdf"
trap - EXIT

printf '%s\n' '[4/4] Updating PNG and interactive preview'
preview_args=()
comparison_urdf="$PWD/preview/revisions/before_frame_detail/urdf/dual_arm.urdf"
if [[ -f "$comparison_urdf" ]]; then
  preview_args+=(--compare-to "$comparison_urdf")
fi
surface_comparison_urdf="$PWD/preview/revisions/before_solid_faces/urdf/dual_arm.urdf"
if [[ -f "$surface_comparison_urdf" ]]; then
  preview_args+=(--surface-compare-to "$surface_comparison_urdf")
fi
wrist_comparison_urdf="$PWD/preview/revisions/before_wrist_direction_reverse/urdf/dual_arm.urdf"
if [[ -f "$wrist_comparison_urdf" ]]; then
  preview_args+=(--wrist-compare-to "$wrist_comparison_urdf")
fi
PYTHONNOUSERSITE=1 MPLCONFIGDIR="$PWD/log/matplotlib" \
  /usr/bin/python3 "$description/scripts/export_preview.py" "${preview_args[@]}"
printf 'Preview: %s\n' "$PWD/preview/01_양팔_전체.png" "$PWD/preview/dual_arm_viewer.html"
if "$preview_only"; then
  exit 0
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/log/ros}"
# RViz receives precisely the URDF used to generate the PNG/HTML above.
printf 'RViz model: %s\n' "$description/urdf/dual_arm.urdf"
printf '%s\n' \
  'RViz: Move Camera = left drag rotate, wheel zoom, Shift+left drag pan.' \
  'Focus Camera: click a part to set the rotation centre, then zoom.'
exec ros2 launch dual_arm_description display.launch.py \
  "urdf_file:=$description/urdf/dual_arm.urdf" "${launch_args[@]}"
