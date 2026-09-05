#!/usr/bin/env bash
set -euo pipefail

# Public beta workflow for LiDAR Mobility Map Generator.
# The product, ROS package, executable, and public LMMG_* settings use the
# same name because this project has no released compatibility surface yet.

lmmg_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
lmmg_input_path_base=$(pwd -P)
lmmg_mode=${1:-}
lmmg_params_arg=${2:-}
lmmg_vehicle_arg=${3:-}
lmmg_output_arg=${4:-}

if [[ "${lmmg_mode}" == "-h" || "${lmmg_mode}" == "--help" ]]; then
  echo "Usage: $0 <generate|review|generate-and-review> PARAMS_YAML VEHICLE_INFO_YAML OUTPUT_DIRECTORY"
  echo ""
  echo "PARAMS_YAML contains a LiDAR SLAM result (GLIM-format) or rosbag2 input,"
  echo "LiDAR extrinsics,"
  echo "clearance, collision-height, and evidence settings."
  echo "VEHICLE_INFO_YAML is the same vehicle_info.param.yaml passed to Autoware."
  echo "Set LMMG_ACQUISITION_VEHICLE_IS_TARGET=true only when the recording"
  echo "vehicle and the target vehicle are the same (default: false)."
  exit 0
fi
if [[ -z "${lmmg_mode}" || -z "${lmmg_params_arg}" ||
      -z "${lmmg_vehicle_arg}" || -z "${lmmg_output_arg}" ]]; then
  echo "Usage: $0 <generate|review|generate-and-review> PARAMS_YAML VEHICLE_INFO_YAML OUTPUT_DIRECTORY" >&2
  echo "" >&2
  echo "PARAMS_YAML contains a LiDAR SLAM result (GLIM-format) or rosbag2 input," >&2
  echo "LiDAR extrinsics," >&2
  echo "clearance, collision-height, and evidence settings." >&2
  echo "VEHICLE_INFO_YAML is the same vehicle_info.param.yaml passed to Autoware." >&2
  exit 2
fi
case "${lmmg_mode}" in
  generate|review|generate-and-review) ;;
  *)
    echo "error: mode must be generate, review, or generate-and-review" >&2
    exit 2
    ;;
esac

for lmmg_command in ros2 python3 realpath sha256sum install
do
  if ! command -v "${lmmg_command}" >/dev/null 2>&1; then
    echo "error: required command is not installed or the ROS environment is not sourced: ${lmmg_command}" >&2
    exit 2
  fi
done
if [[ -L "${lmmg_params_arg}" || ! -f "${lmmg_params_arg}" ]]; then
  echo "error: generator parameter file is missing: ${lmmg_params_arg}" >&2
  exit 2
fi
if [[ -L "${lmmg_vehicle_arg}" || ! -f "${lmmg_vehicle_arg}" ]]; then
  echo "error: target vehicle_info.param.yaml is missing: ${lmmg_vehicle_arg}" >&2
  exit 2
fi
if [[ "$(realpath -s -- "${lmmg_params_arg}")" != "$(realpath -- "${lmmg_params_arg}")" ]]; then
  echo "error: generator parameter path must not contain a symlink: ${lmmg_params_arg}" >&2
  exit 2
fi
if [[ "$(realpath -s -- "${lmmg_vehicle_arg}")" != "$(realpath -- "${lmmg_vehicle_arg}")" ]]; then
  echo "error: target vehicle path must not contain a symlink: ${lmmg_vehicle_arg}" >&2
  exit 2
fi

lmmg_params=$(realpath "${lmmg_params_arg}")
lmmg_vehicle=$(realpath "${lmmg_vehicle_arg}")
lmmg_output=$(realpath -m "${lmmg_output_arg}")
lmmg_dataset=${lmmg_output##*/}
if [[ -z "${lmmg_dataset}" ]]; then
  echo "error: OUTPUT_DIRECTORY must not be the filesystem root" >&2
  exit 2
fi
lmmg_acquisition_vehicle_is_target=${LMMG_ACQUISITION_VEHICLE_IS_TARGET:-false}
if [[ "${lmmg_acquisition_vehicle_is_target}" != "true" &&
      "${lmmg_acquisition_vehicle_is_target}" != "false" ]]; then
  echo "error: LMMG_ACQUISITION_VEHICLE_IS_TARGET must be true or false" >&2
  exit 2
fi
lmmg_resolver="${lmmg_script_dir}/resolve_autoware_vehicle_info.py"
lmmg_binding_verifier="${lmmg_script_dir}/verify_target_vehicle_map_binding.py"
lmmg_contract_tool="${lmmg_script_dir}/generation_calibration_contract.py"
lmmg_stage="${lmmg_script_dir}/stage_autoware_closed_course_map.sh"
for lmmg_tool in "${lmmg_resolver}" "${lmmg_binding_verifier}" "${lmmg_stage}"
do
  if [[ ! -x "${lmmg_tool}" ]]; then
    echo "error: required workflow tool is missing or not executable: ${lmmg_tool}" >&2
    exit 2
  fi
done
if [[ ! -f "${lmmg_contract_tool}" ]]; then
  echo "error: required workflow tool is missing: ${lmmg_contract_tool}" >&2
  exit 2
fi

lmmg_resolved_tsv=$("${lmmg_resolver}" "${lmmg_vehicle}" --format tsv)
lmmg_width=
lmmg_front=
lmmg_rear=
lmmg_turning_radius=
lmmg_height=
lmmg_vehicle_sha256=
while IFS=$'\t' read -r lmmg_key lmmg_value lmmg_extra
do
  if [[ -n "${lmmg_extra}" ]]; then
    echo "error: malformed vehicle-info resolver output" >&2
    exit 2
  fi
  case "${lmmg_key}" in
    width_m) lmmg_width=${lmmg_value} ;;
    front_extent_m) lmmg_front=${lmmg_value} ;;
    rear_extent_m) lmmg_rear=${lmmg_value} ;;
    minimum_turning_radius_m) lmmg_turning_radius=${lmmg_value} ;;
    vehicle_height_m) lmmg_height=${lmmg_value} ;;
    sha256) lmmg_vehicle_sha256=${lmmg_value} ;;
    *)
      echo "error: unexpected vehicle-info resolver key: ${lmmg_key}" >&2
      exit 2
      ;;
  esac
done <<< "${lmmg_resolved_tsv}"
if [[ -z "${lmmg_width}" || -z "${lmmg_front}" || -z "${lmmg_rear}" ||
      -z "${lmmg_turning_radius}" || -z "${lmmg_height}" ||
      ! "${lmmg_vehicle_sha256}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "error: vehicle_info.param.yaml did not resolve to one complete vehicle" >&2
  exit 2
fi

lmmg_record_contract() {
  python3 "${lmmg_contract_tool}" create-direct \
    --dataset "${lmmg_dataset}" \
    --map-type vector_map \
    --generation-report "${lmmg_output}/generation_report.yaml" \
    --generator-parameters "${lmmg_params}" \
    --input-path-base "${lmmg_input_path_base}" \
    --target-vehicle-info "${lmmg_output}/target_vehicle_info.param.yaml" \
    --acquisition-vehicle-is-target "${lmmg_acquisition_vehicle_is_target}" \
    --output "${lmmg_output}/acquisition_vehicle_target_contract.json" \
    --sha256-output "${lmmg_output}/acquisition_vehicle_target_contract.sha256"
}

lmmg_verify_contract() {
  local lmmg_verify_inputs=${1:-false}
  local lmmg_target
  local lmmg_canonical_target="${lmmg_output}/target_vehicle_info.param.yaml"
  local -a lmmg_targets=("${lmmg_canonical_target}")
  local -a lmmg_verify_command
  # Verify both the vehicle YAML supplied to this invocation and the canonical
  # copy that the editor is about to display.  The large source data is hashed
  # only once, through the canonical-copy verification below.
  if [[ ! "${lmmg_vehicle}" -ef "${lmmg_canonical_target}" ]]; then
    lmmg_targets=("${lmmg_vehicle}" "${lmmg_canonical_target}")
  fi
  for lmmg_target in "${lmmg_targets[@]}"
  do
    lmmg_verify_command=(
      python3 "${lmmg_contract_tool}" verify-direct
      --expected-dataset "${lmmg_dataset}"
      --expected-map-type vector_map
      --contract "${lmmg_output}/acquisition_vehicle_target_contract.json"
      --sha256 "${lmmg_output}/acquisition_vehicle_target_contract.sha256"
      --generator-parameters "${lmmg_params}"
      --target-vehicle-info "${lmmg_target}"
      --format json
    )
    if [[ "${lmmg_verify_inputs}" == "true" &&
          "${lmmg_target}" == "${lmmg_canonical_target}" ]]; then
      lmmg_verify_command+=(--verify-inputs)
    fi
    "${lmmg_verify_command[@]}" >/dev/null
  done
}

lmmg_generate() {
  echo "LiDAR Mobility Map Generator: generating Vector Map (Beta)..."
  local lmmg_params_sha_before
  local lmmg_params_sha_after
  local lmmg_vehicle_sha_after
  lmmg_params_sha_before=$(sha256sum -- "${lmmg_params}")
  lmmg_params_sha_before=${lmmg_params_sha_before%% *}
  ros2 run lidar_mobility_map_generator lidar_mobility_map_generator \
    --ros-args \
    --params-file "${lmmg_params}" \
    -p "output.directory:=${lmmg_output}" \
    -p output.target_mode:=vector_map \
    -p robot.profile:=custom \
    -p robot.base_reference:=rear_axle_ground_projection \
    -p robot.footprint_model:=rectangle \
    -p "robot.width:=${lmmg_width}" \
    -p "robot.front_extent:=${lmmg_front}" \
    -p "robot.rear_extent:=${lmmg_rear}" \
    -p "robot.minimum_turning_radius:=${lmmg_turning_radius}" \
    -p robot.allow_in_place_rotation:=false \
    -p "robot.maximum_collision_height:=${lmmg_height}"

  lmmg_params_sha_after=$(sha256sum -- "${lmmg_params}")
  lmmg_params_sha_after=${lmmg_params_sha_after%% *}
  lmmg_vehicle_sha_after=$(sha256sum -- "${lmmg_vehicle}")
  lmmg_vehicle_sha_after=${lmmg_vehicle_sha_after%% *}
  if [[ "${lmmg_params_sha_before}" != "${lmmg_params_sha_after}" ]]; then
    echo "error: generator parameter YAML changed while generation was running" >&2
    return 2
  fi
  if [[ "${lmmg_vehicle_sha256}" != "${lmmg_vehicle_sha_after}" ]]; then
    echo "error: target vehicle YAML changed while generation was running" >&2
    return 2
  fi

  install -m 0644 "${lmmg_vehicle}" "${lmmg_output}/target_vehicle_info.param.yaml"
  "${lmmg_resolver}" "${lmmg_vehicle}" \
    --audit-output "${lmmg_output}/target_vehicle_info.audit.json" >/dev/null
  lmmg_record_contract
  "${lmmg_binding_verifier}" \
    --vehicle-yaml "${lmmg_output}/target_vehicle_info.param.yaml" \
    --osm "${lmmg_output}/lanelet2_map_closed_course_experimental.osm" \
    --output "${lmmg_output}/target_vehicle_map_binding.json"
  "${lmmg_stage}" "${lmmg_output}" --replace
  echo "LiDAR Mobility Map Generator: Vector Map output is ready under ${lmmg_output}"
  if [[ "${lmmg_acquisition_vehicle_is_target}" != "true" ]]; then
    echo "warning: the recording vehicle was not confirmed as the target vehicle; do not start a motion test" >&2
  fi
}

lmmg_validate_marker() {
  local lmmg_marker=$1
  local lmmg_expected_session=$2
  local lmmg_source_selection=$3
  local lmmg_magic_count=0
  local lmmg_session_count=0
  local lmmg_source_count=0
  local lmmg_fingerprint_count=0
  local lmmg_marker_session=
  local lmmg_marker_source=
  local lmmg_marker_fingerprint=
  local lmmg_key=
  local lmmg_value=
  local lmmg_extra=
  while IFS=$'\t' read -r lmmg_key lmmg_value lmmg_extra
  do
    if [[ -n "${lmmg_extra}" ]]; then
      return 1
    fi
    case "${lmmg_key}" in
      LMMG_AUTOWARE_ONE_CLICK_EXPORT)
        lmmg_magic_count=$((lmmg_magic_count + 1))
        [[ "${lmmg_value}" == "1" ]] || return 1
        ;;
      SESSION)
        lmmg_session_count=$((lmmg_session_count + 1))
        lmmg_marker_session=${lmmg_value}
        ;;
      SOURCE)
        lmmg_source_count=$((lmmg_source_count + 1))
        lmmg_marker_source=${lmmg_value}
        ;;
      GRAPH_FINGERPRINT)
        lmmg_fingerprint_count=$((lmmg_fingerprint_count + 1))
        lmmg_marker_fingerprint=${lmmg_value}
        ;;
      *) return 1 ;;
    esac
  done < "${lmmg_marker}"
  [[ ${lmmg_magic_count} -eq 1 &&
     ${lmmg_session_count} -eq 1 &&
     ${lmmg_source_count} -eq 1 &&
     ${lmmg_fingerprint_count} -eq 1 &&
     "${lmmg_marker_session}" == "${lmmg_expected_session}" &&
     "${lmmg_marker_source}" =~ ^(recorded_trajectory|edited_topology)$ &&
     "${lmmg_marker_fingerprint}" =~ ^[0-9a-f]{16}$ ]] || return 1

  # The default recorded-trajectory source predates the persisted selection
  # file.  A user-authored source, however, is valid only when the editor has
  # atomically saved the exact source/fingerprint binding that it requested.
  if [[ ! -e "${lmmg_source_selection}" ]]; then
    [[ "${lmmg_marker_source}" == "recorded_trajectory" ]]
    return
  fi
  [[ -f "${lmmg_source_selection}" && ! -L "${lmmg_source_selection}" ]] || return 1

  local lmmg_selection_header_count=0
  local lmmg_selection_source_count=0
  local lmmg_selection_frame_count=0
  local lmmg_selection_fingerprint_count=0
  local lmmg_selection_source=
  local lmmg_selection_frame=
  local lmmg_selection_fingerprint=
  while IFS=$'\t' read -r lmmg_key lmmg_value lmmg_extra
  do
    [[ -z "${lmmg_extra}" ]] || return 1
    case "${lmmg_key}" in
      LMMG_VECTOR_MAP_SOURCE)
        lmmg_selection_header_count=$((lmmg_selection_header_count + 1))
        [[ "${lmmg_value}" == "1" ]] || return 1
        ;;
      SOURCE)
        lmmg_selection_source_count=$((lmmg_selection_source_count + 1))
        lmmg_selection_source=${lmmg_value}
        ;;
      FRAME)
        lmmg_selection_frame_count=$((lmmg_selection_frame_count + 1))
        lmmg_selection_frame=${lmmg_value}
        ;;
      GRAPH_FINGERPRINT)
        lmmg_selection_fingerprint_count=$((lmmg_selection_fingerprint_count + 1))
        lmmg_selection_fingerprint=${lmmg_value}
        ;;
      *) return 1 ;;
    esac
  done < "${lmmg_source_selection}"
  [[ ${lmmg_selection_header_count} -eq 1 &&
     ${lmmg_selection_source_count} -eq 1 &&
     ${lmmg_selection_frame_count} -eq 1 &&
     ${lmmg_selection_fingerprint_count} -eq 1 &&
     "${lmmg_selection_source}" =~ ^(recorded_trajectory|edited_topology)$ &&
     -n "${lmmg_selection_frame}" &&
     "${lmmg_selection_fingerprint}" =~ ^[0-9a-f]{16}$ &&
     "${lmmg_marker_source}" == "${lmmg_selection_source}" &&
     "${lmmg_marker_fingerprint}" == "${lmmg_selection_fingerprint}" ]]
}

lmmg_review() {
  if [[ ! -d "${lmmg_output}" ]]; then
    echo "error: generated output directory is missing: ${lmmg_output}" >&2
    return 2
  fi
  if ! lmmg_verify_contract false; then
    echo "error: input data, parameters, or vehicle information changed after generation; generate the map again" >&2
    return 2
  fi
  local lmmg_required
  for lmmg_required in \
    pointcloud_map.pcd trajectory_processed.tum \
    route_graph_autoware_replay_candidate.geojson \
    lanelet2_map_closed_course_experimental.osm \
    navigation_target_readiness.yaml route_body_passage_planning_report.json \
    target_vehicle_info.param.yaml target_vehicle_info.audit.json \
    acquisition_vehicle_target_contract.json \
    acquisition_vehicle_target_contract.sha256
  do
    if [[ ! -s "${lmmg_output}/${lmmg_required}" ]]; then
      echo "error: Vector Map output is incomplete: ${lmmg_output}/${lmmg_required}" >&2
      return 2
    fi
  done

  local lmmg_marker="${lmmg_output}/.autoware_one_click_export_requested"
  local lmmg_session
  printf -v lmmg_session '%s-%s-%s-%s' \
    "${BASHPID}" "${RANDOM}" "${RANDOM}" "${RANDOM}"
  rm -f -- "${lmmg_marker}"
  local lmmg_review_status=0
  if ros2 launch lidar_mobility_map_generator edit_vector_map.launch.py \
      "output_directory:=${lmmg_output}" \
      frame_id:=map \
      enable_autoware_one_click_export:=true \
      "autoware_one_click_session:=${lmmg_session}"; then
    lmmg_review_status=0
  else
    lmmg_review_status=$?
  fi

  if [[ ! -f "${lmmg_marker}" ]]; then
    return "${lmmg_review_status}"
  fi
  if [[ ${lmmg_review_status} -ne 0 ]]; then
    rm -f -- "${lmmg_marker}"
    echo "error: Vector Map regeneration was cancelled because the editor failed" >&2
    return "${lmmg_review_status}"
  fi
  if ! lmmg_validate_marker \
      "${lmmg_marker}" "${lmmg_session}" "${lmmg_output}/vector_map_source.tsv"; then
    rm -f -- "${lmmg_marker}"
    echo "error: the GUI regeneration request is invalid or no longer matches the current files" >&2
    return 1
  fi
  rm -f -- "${lmmg_marker}"
  if ! lmmg_verify_contract true; then
    echo "error: regeneration rejected changed LiDAR SLAM/rosbag2 input data" >&2
    return 2
  fi
  lmmg_generate
}

case "${lmmg_mode}" in
  generate) lmmg_generate ;;
  review) lmmg_review ;;
  generate-and-review)
    lmmg_generate
    lmmg_review
    ;;
esac
