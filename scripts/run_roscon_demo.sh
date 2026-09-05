#!/usr/bin/env bash
set -euo pipefail

lmmg_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
lmmg_package_dir=$(cd -- "${lmmg_script_dir}/.." && pwd)
lmmg_workspace_dir=$(cd -- "${lmmg_package_dir}/.." && pwd)
lmmg_output_root=${1:-/tmp/lmmg_roscon_demo}
lmmg_mode=${2:-generate}

if [[ "${lmmg_mode}" != "generate" && "${lmmg_mode}" != "review" && \
      "${lmmg_mode}" != "generate-and-review" ]]; then
  echo "usage: $0 [OUTPUT_ROOT] [generate|review|generate-and-review]" >&2
  exit 2
fi
if [[ "${lmmg_output_root}" != /* ]]; then
  echo "error: OUTPUT_ROOT must be an absolute path" >&2
  exit 2
fi

lmmg_input_dir="${lmmg_output_root}/synthetic_input"
lmmg_output_dir="${lmmg_output_root}/generated_map"
lmmg_ros_distro=${ROS_DISTRO:-jazzy}
lmmg_ros_setup="/opt/ros/${lmmg_ros_distro}/setup.bash"

set +u
if ! command -v ros2 >/dev/null 2>&1; then
  if [[ ! -f "${lmmg_ros_setup}" ]]; then
    echo "error: ROS setup not found: ${lmmg_ros_setup}" >&2
    exit 1
  fi
  source "${lmmg_ros_setup}"
fi
if ! ros2 pkg prefix lidar_mobility_map_generator >/dev/null 2>&1; then
  if [[ -f "${lmmg_workspace_dir}/install/setup.bash" ]]; then
    source "${lmmg_workspace_dir}/install/setup.bash"
  else
    echo "error: build and source lidar_mobility_map_generator first" >&2
    exit 1
  fi
fi
set -u
lmmg_share_dir=$(ros2 pkg prefix --share lidar_mobility_map_generator)

mkdir -p "${lmmg_output_root}"
export ROS_LOG_DIR=${ROS_LOG_DIR:-"${lmmg_output_root}/ros_logs"}
mkdir -p "${ROS_LOG_DIR}"

lmmg_generate() {
  python3 "${lmmg_script_dir}/create_roscon_demo_input.py" \
    --output-directory "${lmmg_input_dir}" --replace

  ros2 run lidar_mobility_map_generator generate_vector_map --ros-args \
    --params-file "${lmmg_share_dir}/config/glim.yaml" \
    -p "input.glim.map_path:=${lmmg_input_dir}/map.ply" \
    -p "input.glim.trajectory_path:=${lmmg_input_dir}/traj_lidar.txt" \
    -p input.glim.trajectory_frame:=base \
    -p extrinsics.source:=parameters \
    -p extrinsics.calibration_source:=measured \
    -p extrinsics.calibration_confidence:=high \
    -p extrinsics.verified:=true \
    -p "extrinsics.translation:=[0.0, 0.0, 0.0]" \
    -p "extrinsics.quaternion_xyzw:=[0.0, 0.0, 0.0, 1.0]" \
    -p robot.profile:=custom \
    -p robot.base_reference:=rear_axle_ground_projection \
    -p robot.footprint_model:=rectangle \
    -p robot.width:=1.0 \
    -p robot.front_extent:=0.8 \
    -p robot.rear_extent:=0.4 \
    -p robot.clearance_margin:=0.15 \
    -p robot.minimum_collision_height:=0.08 \
    -p robot.maximum_collision_height:=1.2 \
    -p robot.dimensions_source:=measured \
    -p robot.dimensions_confidence:=high \
    -p robot.dimensions_verified:=true \
    -p robot.minimum_turning_radius:=0.5 \
    -p robot.allow_in_place_rotation:=false \
    -p robot.allow_reverse_motion:=false \
    -p trajectory.smoothing_window:=0.20 \
    -p traversability.free_space_evidence_mode:=combined \
    -p traversability.trajectory_free_space_model:=footprint \
    -p traversability.trajectory_footprint_erosion_margin:=0.05 \
    -p traversability.minimum_obstacle_observations:=1 \
    -p topology.maximum_edge_length:=2.0 \
    -p topology.geometry_smoothing_window:=0.50 \
    -p lanelet2.speed_limit_mps:=0.50 \
    -p output.target_mode:=both \
    -p output.nav2_free_space_verified:=true \
    -p output.lanelet2_physical_boundaries_verified:=false \
    -p "output.directory:=${lmmg_output_dir}"

  python3 "${lmmg_script_dir}/check_roscon_demo.py" "${lmmg_output_dir}"
}

lmmg_review() {
  python3 "${lmmg_script_dir}/check_roscon_demo.py" "${lmmg_output_dir}"
  ros2 launch lidar_mobility_map_generator edit_and_review.launch.py \
    "output_directory:=${lmmg_output_dir}" frame_id:=map
}

if [[ "${lmmg_mode}" == "generate" || "${lmmg_mode}" == "generate-and-review" ]]; then
  lmmg_generate
fi
if [[ "${lmmg_mode}" == "review" || "${lmmg_mode}" == "generate-and-review" ]]; then
  lmmg_review
fi

printf 'ROSCon demo output: %s\n' "${lmmg_output_dir}"
