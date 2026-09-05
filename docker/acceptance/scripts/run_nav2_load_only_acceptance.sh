#!/usr/bin/env bash
set -euo pipefail

lmmg_artifacts=${1:-/artifacts}
lmmg_reports=${2:-/reports}
lmmg_timeout=${LMMG_NAV2_LOAD_TIMEOUT_SEC:-45}
lmmg_probe=${LMMG_NAV2_LOAD_ONLY_PROBE:-/opt/lmmg_acceptance/nav2_load_only_probe.py}
lmmg_waypoint_client=${LMMG_FOLLOW_NAV2_WAYPOINTS_CLIENT:-/opt/lmmg_acceptance/follow_nav2_waypoints.py}

lmmg_fail() {
  echo "error: $*" >&2
  exit 2
}

lmmg_require_read_only_mount() {
  local lmmg_options
  lmmg_options=$(findmnt -T "$1" -n -o OPTIONS) || \
    lmmg_fail "cannot inspect artifact mount options for $1"
  if [[ ",${lmmg_options}," != *,ro,* ]]; then
    lmmg_fail "artifact mount is not read-only: $1 (${lmmg_options})"
  fi
}

[[ -d "${lmmg_artifacts}" ]] || lmmg_fail "artifact directory is missing: ${lmmg_artifacts}"
[[ -x "${lmmg_probe}" ]] || lmmg_fail "load-only probe is unavailable: ${lmmg_probe}"
[[ -x "${lmmg_waypoint_client}" ]] || \
  lmmg_fail "waypoint dry-run client is unavailable: ${lmmg_waypoint_client}"
[[ "${lmmg_timeout}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  lmmg_fail "LMMG_NAV2_LOAD_TIMEOUT_SEC must be a positive number"
lmmg_require_read_only_mount "${lmmg_artifacts}"
mkdir -p "${lmmg_reports}/node_logs"

lmmg_map_yaml="${lmmg_artifacts}/nav2_map_closed_course_experimental.yaml"
lmmg_graph="${lmmg_artifacts}/nav2_route_graph_closed_course_experimental.geojson"
lmmg_waypoints="${lmmg_artifacts}/nav2_waypoints_closed_course_experimental.yaml"
lmmg_params="${lmmg_artifacts}/nav2_closed_course_experimental_params.yaml"
lmmg_readiness="${lmmg_artifacts}/nav2_closed_course_experimental_readiness.yaml"
for lmmg_required in \
  "${lmmg_map_yaml}" "${lmmg_graph}" "${lmmg_waypoints}" \
  "${lmmg_params}" "${lmmg_readiness}"
do
  [[ -f "${lmmg_required}" ]] || lmmg_fail "required artifact is missing: ${lmmg_required}"
done

lmmg_waypoint_args=()
if [[ -n "${LMMG_NAV2_ROUTE_ID:-}" ]]; then
  lmmg_waypoint_args+=(--route-id "${LMMG_NAV2_ROUTE_ID}")
fi
python3 "${lmmg_waypoint_client}" "${lmmg_waypoints}" \
  "${lmmg_waypoint_args[@]}" --dry-run > "${lmmg_reports}/waypoint_dry_run.json"

lmmg_pids=()
lmmg_cleanup() {
  local lmmg_pid
  for lmmg_pid in "${lmmg_pids[@]}"; do
    if kill -0 "${lmmg_pid}" 2>/dev/null; then
      kill -INT "${lmmg_pid}" 2>/dev/null || true
    fi
  done
  for _ in {1..40}; do
    local lmmg_running=false
    for lmmg_pid in "${lmmg_pids[@]}"; do
      if kill -0 "${lmmg_pid}" 2>/dev/null; then
        lmmg_running=true
      fi
    done
    [[ "${lmmg_running}" == false ]] && break
    sleep 0.1
  done
  for lmmg_pid in "${lmmg_pids[@]}"; do
    if kill -0 "${lmmg_pid}" 2>/dev/null; then
      kill -TERM "${lmmg_pid}" 2>/dev/null || true
    fi
    wait "${lmmg_pid}" 2>/dev/null || true
  done
}
trap lmmg_cleanup EXIT

# Load-only means exactly these two lifecycle nodes.  No planner, controller,
# navigator, waypoint follower, route request, action goal, or motion simulator
# is started by this acceptance path.
ros2 run nav2_map_server map_server --ros-args \
  --params-file "${lmmg_params}" \
  -p "yaml_filename:=${lmmg_map_yaml}" > \
  "${lmmg_reports}/node_logs/map_server.log" 2>&1 &
lmmg_pids+=("$!")

ros2 run nav2_route route_server --ros-args \
  --params-file "${lmmg_params}" \
  -p "graph_filepath:=${lmmg_graph}" > \
  "${lmmg_reports}/node_logs/route_server.log" 2>&1 &
lmmg_pids+=("$!")

python3 "${lmmg_probe}" \
  --artifact-dir "${lmmg_artifacts}" \
  --waypoint-dry-run "${lmmg_reports}/waypoint_dry_run.json" \
  --report "${lmmg_reports}/acceptance.json" \
  --timeout "${lmmg_timeout}"

echo "Nav2 alpha load-only acceptance passed: ${lmmg_reports}/acceptance.json"
