#!/usr/bin/env bash
set -euo pipefail

lmmg_script_path=$(realpath "${BASH_SOURCE[0]}")
lmmg_script_dir=$(cd -- "$(dirname -- "${lmmg_script_path}")" && pwd)
lmmg_package_dir=$(cd -- "${lmmg_script_dir}/.." && pwd)
lmmg_artifact_arg=${1:-}
lmmg_report_arg=${2:-}
lmmg_nav2_image_lock_arg=${3:-${lmmg_package_dir}/docker/acceptance/locks/nav2-load-only-image.lock.env.example}

if [[ -z "${lmmg_artifact_arg}" || -z "${lmmg_report_arg}" ]]; then
  echo "Usage: $0 <generated-navigation-map-dir> <fresh-report-dir> [nav2-image-lock]" >&2
  exit 2
fi

lmmg_artifacts=$(realpath "${lmmg_artifact_arg}")
lmmg_nav2_image_lock=$(realpath "${lmmg_nav2_image_lock_arg}")
lmmg_report_parent=$(realpath -m "$(dirname -- "${lmmg_report_arg}")")
lmmg_reports=$(realpath -m "${lmmg_report_arg}")
mkdir -p "${lmmg_report_parent}"
if [[ -e "${lmmg_reports}" ]]; then
  echo "error: report directory must be fresh: ${lmmg_reports}" >&2
  exit 2
fi
mkdir "${lmmg_reports}"

lmmg_image=$(python3 \
  "${lmmg_package_dir}/docker/acceptance/scripts/validate_lock.py" \
  "${lmmg_nav2_image_lock}" --mode nav2 --get LMMG_NAV2_ACCEPTANCE_IMAGE)
[[ "${lmmg_image}" =~ @sha256:[0-9a-f]{64}$ ]] || {
  echo "error: reviewed Nav2 image is not digest-qualified" >&2
  exit 2
}
docker image inspect "${lmmg_image}" > "${lmmg_reports}/container-image-inspect.json"

lmmg_inner_runner=$(realpath \
  "${lmmg_package_dir}/docker/acceptance/scripts/run_nav2_load_only_acceptance.sh")
lmmg_probe=$(realpath \
  "${lmmg_package_dir}/docker/acceptance/scripts/nav2_load_only_probe.py")
lmmg_waypoint_client=$(realpath \
  "${lmmg_package_dir}/scripts/follow_nav2_waypoints.py")

docker run --rm --pull never \
  --network none \
  --read-only \
  --user "$(id -u):$(id -g)" \
  --cap-drop ALL \
  --security-opt no-new-privileges:true \
  --pids-limit 256 \
  --tmpfs /tmp:rw,nosuid,nodev,mode=1777,size=256m \
  --env HOME=/tmp/lmmg-home \
  --env ROS_HOME=/tmp/lmmg-ros-home \
  --env ROS_LOG_DIR=/tmp/lmmg-ros-log \
  --env ROS_LOCALHOST_ONLY=1 \
  --env LMMG_NAV2_LOAD_ONLY_PROBE=/evidence/nav2_load_only_probe.py \
  --env LMMG_FOLLOW_NAV2_WAYPOINTS_CLIENT=/evidence/follow_nav2_waypoints.py \
  --env "LMMG_NAV2_LOAD_TIMEOUT_SEC=${LMMG_NAV2_LOAD_TIMEOUT_SEC:-45}" \
  --env "LMMG_NAV2_ROUTE_ID=${LMMG_NAV2_ROUTE_ID:-}" \
  --volume "${lmmg_artifacts}:/artifacts:ro" \
  --volume "${lmmg_reports}:/reports:rw" \
  --volume "${lmmg_inner_runner}:/evidence/run_nav2_load_only_acceptance.sh:ro" \
  --volume "${lmmg_probe}:/evidence/nav2_load_only_probe.py:ro" \
  --volume "${lmmg_waypoint_client}:/evidence/follow_nav2_waypoints.py:ro" \
  --entrypoint /bin/bash \
  "${lmmg_image}" \
  -c 'set -e; source /opt/ros/jazzy/setup.bash; exec "$@"' \
  lmmg-nav2-load-only \
  /evidence/run_nav2_load_only_acceptance.sh /artifacts /reports

echo "Nav2 alpha load-only report: ${lmmg_reports}/acceptance.json"
