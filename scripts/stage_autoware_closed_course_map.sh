#!/usr/bin/env bash
set -euo pipefail

lmmg_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
lmmg_candidate_validator="${lmmg_script_dir}/validate_autoware_candidate.py"
lmmg_lanelet_smoke=${LMMG_AUTOWARE_LANELET_SMOKE_EXECUTABLE:-}
lmmg_require_lanelet_smoke=${LMMG_REQUIRE_AUTOWARE_LANELET_SMOKE:-false}
if [[ -z "${lmmg_lanelet_smoke}" && -x "${lmmg_script_dir}/autoware_lanelet_smoke" ]]; then
  lmmg_lanelet_smoke="${lmmg_script_dir}/autoware_lanelet_smoke"
fi
lmmg_output_arg=${1:-}
lmmg_replace=${2:-}
if [[ -z "${lmmg_output_arg}" || "${lmmg_output_arg}" == "-h" ||
      "${lmmg_output_arg}" == "--help" ]]; then
  echo "usage: $0 OUTPUT_DIRECTORY [--replace]" >&2
  echo "Prepares a Vector Map folder for Autoware loading checks; this does not prove that the map is runnable." >&2
  exit 2
fi
if [[ -n "${lmmg_replace}" && "${lmmg_replace}" != "--replace" ]]; then
  echo "error: unknown option: ${lmmg_replace}" >&2
  exit 2
fi
for lmmg_command in xmllint sha256sum python3
do
  if ! command -v "${lmmg_command}" >/dev/null 2>&1; then
    echo "error: required command is not installed: ${lmmg_command}" >&2
    exit 2
  fi
done
if [[ ! -x "${lmmg_candidate_validator}" ]]; then
  echo "error: Vector Map validation tool is missing: ${lmmg_candidate_validator}" >&2
  exit 2
fi
if [[ -n "${lmmg_lanelet_smoke}" && ! -x "${lmmg_lanelet_smoke}" ]]; then
  echo "error: configured Autoware Lanelet smoke checker is not executable: ${lmmg_lanelet_smoke}" >&2
  exit 2
fi
if [[ "${lmmg_require_lanelet_smoke}" != "true" &&
      "${lmmg_require_lanelet_smoke}" != "false" ]]; then
  echo "error: LMMG_REQUIRE_AUTOWARE_LANELET_SMOKE must be true or false" >&2
  exit 2
fi
if [[ ! -d "${lmmg_output_arg}" ]]; then
  echo "error: output directory does not exist: ${lmmg_output_arg}" >&2
  exit 2
fi
lmmg_output=$(cd -- "${lmmg_output_arg}" && pwd)
lmmg_readiness="${lmmg_output}/navigation_target_readiness.yaml"
lmmg_pointcloud="${lmmg_output}/pointcloud_map.pcd"
lmmg_lanelet="${lmmg_output}/lanelet2_map_closed_course_experimental.osm"
lmmg_projector="${lmmg_output}/map_projector_info.yaml"
lmmg_vector_map_source="${lmmg_output}/vector_map_source.tsv"
if [[ ! -f "${lmmg_readiness}" ]]; then
  echo "error: required artifact is missing (the required file was not generated): ${lmmg_readiness}" >&2
  exit 2
fi

# Read only the Autoware section. Production readiness is deliberately not
# accepted as a substitute for the separately named experimental gate.
lmmg_generation_complete=$(awk '$1 == "generation_complete:" {print $2; exit}' \
  "${lmmg_readiness}")
lmmg_target_mode=$(awk '$1 == "requested_target_mode:" {gsub(/\"/, "", $2); print $2; exit}' \
  "${lmmg_readiness}")
lmmg_autoware_enabled=$(awk '
  /^autoware:/ { in_autoware = 1; next }
  in_autoware && /^[^ ]/ { in_autoware = 0 }
  in_autoware && $1 == "enabled:" { print $2; exit }
' "${lmmg_readiness}")
lmmg_centerline_source=$(awk '
  /^autoware:/ { in_autoware = 1; next }
  in_autoware && /^[^ ]/ { in_autoware = 0 }
  in_autoware && $1 == "centerline_source:" {
    gsub(/"/, "", $2); print $2; exit
  }
' "${lmmg_readiness}")
if [[ -z "${lmmg_centerline_source}" ]]; then
  # Compatibility for older recorded-trajectory readiness reports.
  lmmg_centerline_source=recorded_trajectory
fi
case "${lmmg_centerline_source}" in
  recorded_trajectory)
    lmmg_user_authored=false
    lmmg_full_map_source_graph="${lmmg_output}/route_graph_autoware_replay_candidate.geojson"
    lmmg_lossless_replay_source_graph="${lmmg_output}/route_graph_closed_course_replay_candidate.geojson"
    lmmg_named_mission_source_graph="${lmmg_output}/route_graph_autoware_selected_mission.geojson"
    lmmg_autoware_replay_metadata="${lmmg_output}/route_graph_autoware_replay_candidate_metadata.yaml"
    lmmg_navigation_authoring="${lmmg_output}/navigation_authoring_autoware_replay.json"
    lmmg_navigation_authoring_status="${lmmg_output}/navigation_authoring_closed_course_status.json"
    lmmg_authoring_scope=autoware_lossless_replay
    lmmg_semantic_features_tsv="${lmmg_output}/semantic_features.tsv"
    lmmg_semantic_features_geojson="${lmmg_output}/semantic_features.geojson"
    lmmg_autoware_semantic_graph="${lmmg_output}/route_graph_autoware_semantic_lanelet_candidate.geojson"
    lmmg_autoware_semantic_review="${lmmg_output}/review_geometry_autoware_semantic_lanelet_candidate.tsv"
    ;;
  edited_topology)
    lmmg_user_authored=true
    lmmg_full_map_source_graph="${lmmg_output}/route_graph_autoware_topology_source.geojson"
    lmmg_lossless_replay_source_graph=
    lmmg_named_mission_source_graph="${lmmg_output}/route_graph_autoware_topology_selected_mission.geojson"
    lmmg_autoware_replay_metadata=
    lmmg_navigation_authoring="${lmmg_output}/navigation_authoring_autoware_topology.json"
    lmmg_navigation_authoring_status="${lmmg_output}/navigation_authoring_autoware_topology_status.json"
    lmmg_authoring_scope=autoware_edited_topology
    lmmg_semantic_features_tsv="${lmmg_output}/semantic_features_autoware_topology.tsv"
    lmmg_semantic_features_geojson="${lmmg_output}/semantic_features_autoware_topology.geojson"
    lmmg_autoware_semantic_graph="${lmmg_output}/route_graph_autoware_topology_semantic_candidate.geojson"
    lmmg_autoware_semantic_review="${lmmg_output}/review_geometry_autoware_topology_semantic_candidate.tsv"
    ;;
  *)
    echo "error: unsupported Vector Map centerline_source: ${lmmg_centerline_source}" >&2
    exit 4
    ;;
esac
if [[ "${lmmg_generation_complete}" != "true" ]]; then
  echo "error: map generation is incomplete; no Autoware map folder was prepared" >&2
  exit 3
fi
if [[ ( "${lmmg_target_mode}" != "autoware" && "${lmmg_target_mode}" != "both" ) ||
      "${lmmg_autoware_enabled}" != "true" ]]; then
  echo "error: Vector Map is not selected by the requested map type" >&2
  exit 3
fi
lmmg_required_artifacts=(
  "${lmmg_pointcloud}" "${lmmg_lanelet}" "${lmmg_projector}"
  "${lmmg_full_map_source_graph}" "${lmmg_named_mission_source_graph}"
)
if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
  lmmg_required_artifacts+=(
    "${lmmg_lossless_replay_source_graph}" "${lmmg_autoware_replay_metadata}"
  )
else
  lmmg_required_artifacts+=("${lmmg_vector_map_source}")
fi
for lmmg_required in "${lmmg_required_artifacts[@]}"
do
  if [[ ! -f "${lmmg_required}" ]]; then
    echo "error: required artifact is missing (the required file was not generated): ${lmmg_required}" >&2
    exit 2
  fi
done
if [[ "${lmmg_centerline_source}" == "edited_topology" ]]; then
  lmmg_selected_source=$(awk -F '\t' '$1 == "SOURCE" {print $2; exit}' \
    "${lmmg_vector_map_source}")
  if [[ "${lmmg_selected_source}" != "${lmmg_centerline_source}" ]]; then
    echo "error: vector_map_source.tsv and readiness centerline_source disagree" >&2
    exit 4
  fi
fi
lmmg_autoware_experimental_ready=$(awk '
  /^autoware:/ { in_autoware = 1; next }
  in_autoware && /^[^ ]/ { in_autoware = 0 }
  in_autoware && $1 == "closed_course_experimental_ready:" { print $2; exit }
' "${lmmg_readiness}")
if [[ "${lmmg_autoware_experimental_ready}" != "true" ]]; then
  echo "error: Vector Map review output is not ready" >&2
  exit 3
fi

lmmg_terminal_settling_verified=
lmmg_terminal_tail_omitted=
lmmg_omitted_planar_length_m=
if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
  lmmg_terminal_settling_verified=$(awk '
    /^autoware_replay_derivative:/ { in_candidate = 1; next }
    in_candidate && /^[^ ]/ { in_candidate = 0 }
    in_candidate && $1 == "terminal_localization_settling_verified:" { print $2; exit }
  ' "${lmmg_autoware_replay_metadata}")
  lmmg_terminal_tail_omitted=$(awk '
    /^autoware_replay_derivative:/ { in_candidate = 1; next }
    in_candidate && /^[^ ]/ { in_candidate = 0 }
    in_candidate && $1 == "terminal_tail_omitted:" { print $2; exit }
  ' "${lmmg_autoware_replay_metadata}")
  lmmg_omitted_planar_length_m=$(awk '
    /^autoware_replay_derivative:/ { in_candidate = 1; next }
    in_candidate && /^[^ ]/ { in_candidate = 0 }
    in_candidate && $1 == "omitted_planar_length_m:" { print $2; exit }
  ' "${lmmg_autoware_replay_metadata}")
  if [[ ! "${lmmg_terminal_settling_verified}" =~ ^(true|false)$ ||
        ! "${lmmg_terminal_tail_omitted}" =~ ^(true|false)$ ||
        ! "${lmmg_omitted_planar_length_m}" =~ ^[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$ ]]; then
    echo "error: generated map lacks valid terminal-settling evidence (the end-of-route localization check was not confirmed)" >&2
    exit 4
  fi
  if ! awk -v value="${lmmg_omitted_planar_length_m}" \
      'BEGIN { exit !(value + 0 >= 0) }'; then
    echo "error: omitted_planar_length_m must be nonnegative" >&2
    exit 4
  fi
fi
lmmg_candidate_acceptance="${lmmg_output}/autoware_candidate_acceptance.json"
if ! "${lmmg_candidate_validator}" "${lmmg_output}" \
    --report "${lmmg_candidate_acceptance}"; then
  if [[ -f "${lmmg_candidate_acceptance}" ]] &&
      [[ "$(python3 -c \
        'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("coverage_reference", ""))' \
        "${lmmg_candidate_acceptance}")" == "named_route_source_graph" ]] &&
      [[ ! -f "${lmmg_named_mission_source_graph}" ]]; then
    echo "error: required target-route check file is missing: ${lmmg_named_mission_source_graph}" >&2
  fi
  echo "error: Vector Map failed post-export geometry or trajectory-coverage checks" >&2
  # Exit 3 is reserved for a run that does not request Vector Map output. A map
  # that fails its post-export checks is a generation error (exit 4).
  exit 4
fi
lmmg_accepted_centerline_source=$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("centerline_source", ""))' \
  "${lmmg_candidate_acceptance}")
if [[ "${lmmg_accepted_centerline_source}" != "${lmmg_centerline_source}" ]]; then
  echo "error: candidate validation used a different Vector Map centerline source" >&2
  exit 4
fi
lmmg_synthetic_support_count=$(python3 -c \
  'import json,sys; value=json.load(open(sys.argv[1], encoding="utf-8")); print(value.get("counts", {}).get("synthetic_planning_support_lanelets", -1))' \
  "${lmmg_candidate_acceptance}")
lmmg_synthetic_support_present=$(python3 -c \
  'import json,sys; value=json.load(open(sys.argv[1], encoding="utf-8")); support=value.get("planning_support") or {}; print(str(bool(support.get("present", False))).lower())' \
  "${lmmg_candidate_acceptance}")
if [[ "${lmmg_synthetic_support_count}" != "0" ||
      "${lmmg_synthetic_support_present}" != "false" ]]; then
  echo "error: Vector Map contains forbidden synthetic planning-support Lanelets" >&2
  exit 4
fi
lmmg_coverage_reference=$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("coverage_reference", ""))' \
  "${lmmg_candidate_acceptance}")
lmmg_accepted_full_map_hash=$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("input", {}).get("full_map_source_graph_sha256", ""))' \
  "${lmmg_candidate_acceptance}")
lmmg_current_full_map_hash=$(sha256sum \
  "${lmmg_full_map_source_graph}" | awk '{print $1}')
if [[ "${lmmg_accepted_full_map_hash}" != "${lmmg_current_full_map_hash}" ]]; then
  echo "error: source route files changed after validation; generate the map again" >&2
  exit 4
fi
if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
  lmmg_accepted_lossless_replay_hash=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("input", {}).get("lossless_replay_source_graph_sha256", ""))' \
    "${lmmg_candidate_acceptance}")
  lmmg_current_lossless_replay_hash=$(sha256sum \
    "${lmmg_lossless_replay_source_graph}" | awk '{print $1}')
  if [[ "${lmmg_accepted_lossless_replay_hash}" != \
        "${lmmg_current_lossless_replay_hash}" ]]; then
    echo "error: lossless replay source changed after validation; generate the map again" >&2
    exit 4
  fi
else
  lmmg_accepted_vector_map_source_hash=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("input", {}).get("vector_map_source_sha256", ""))' \
    "${lmmg_candidate_acceptance}")
  lmmg_current_vector_map_source_hash=$(sha256sum \
    "${lmmg_vector_map_source}" | awk '{print $1}')
  if [[ "${lmmg_accepted_vector_map_source_hash}" != \
        "${lmmg_current_vector_map_source_hash}" ]]; then
    echo "error: Vector Map source selection changed after validation" >&2
    exit 4
  fi
fi
lmmg_named_route=false
lmmg_named_route_id=
lmmg_named_route_name_json=
lmmg_named_route_target=
lmmg_named_route_edges=
lmmg_named_route_stops=
lmmg_named_route_coverage=
lmmg_terminal_support_source_graph=
lmmg_maximal_authoring_fixture_manifest="${lmmg_output}/maximal_autoware_authoring_fixture_manifest.json"
lmmg_maximal_authoring_fixture_valid=false
if [[ "${lmmg_coverage_reference}" == "named_route_source_graph" ]]; then
  lmmg_named_route=true
  for lmmg_named_required in \
    "${lmmg_named_mission_source_graph}" "${lmmg_navigation_authoring}" \
    "${lmmg_navigation_authoring_status}" \
    "${lmmg_semantic_features_tsv}" "${lmmg_semantic_features_geojson}" \
    "${lmmg_autoware_semantic_graph}" "${lmmg_autoware_semantic_review}"
  do
    if [[ ! -f "${lmmg_named_required}" ]]; then
      echo "error: required target-route check file is missing: ${lmmg_named_required}" >&2
      exit 4
    fi
  done
  lmmg_named_route_id=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["named_route"]["id"])' \
    "${lmmg_candidate_acceptance}")
  lmmg_named_route_name_json=$(python3 -c \
    'import json,sys; print(json.dumps(json.load(open(sys.argv[1], encoding="utf-8"))["named_route"]["name"], ensure_ascii=False))' \
    "${lmmg_candidate_acceptance}")
  lmmg_named_route_target=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["named_route"]["target"])' \
    "${lmmg_candidate_acceptance}")
  lmmg_named_route_edges=$(python3 -c \
    'import json,sys; print(len(json.load(open(sys.argv[1], encoding="utf-8"))["named_route"]["ordered_edge_ids"]))' \
    "${lmmg_candidate_acceptance}")
  lmmg_named_route_stops=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["counts"]["authored_stop_lines"])' \
    "${lmmg_candidate_acceptance}")
  lmmg_named_route_coverage=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["metrics"]["acceptance_centerline_coverage"])' \
    "${lmmg_candidate_acceptance}")
  if [[ ! "${lmmg_named_route_id}" =~ ^[1-9][0-9]*$ ||
        ! "${lmmg_named_route_edges}" =~ ^[1-9][0-9]*$ ||
        ! "${lmmg_named_route_stops}" =~ ^[0-9]+$ ||
        "${lmmg_named_route_target}" != "autoware" ||
        ! "${lmmg_named_route_coverage}" =~ ^[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$ ]]; then
    echo "error: target-route validation report is incomplete or malformed" >&2
    exit 4
  fi
  lmmg_terminal_support_present=$(python3 -c \
    'import json,sys; print(str(bool(json.load(open(sys.argv[1], encoding="utf-8")).get("terminal_support", {}).get("present", False))).lower())' \
    "${lmmg_candidate_acceptance}")
  if [[ "${lmmg_terminal_support_present}" == "true" &&
        "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
    lmmg_terminal_support_source_graph="${lmmg_output}/route_graph_closed_course_topology_candidate.geojson"
    if [[ ! -f "${lmmg_terminal_support_source_graph}" ]]; then
      echo "error: validated end-of-route support file is missing: ${lmmg_terminal_support_source_graph}" >&2
      exit 4
    fi
  fi
  lmmg_accepted_authoring_scope=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("input", {}).get("navigation_authoring_scope", ""))' \
    "${lmmg_candidate_acceptance}")
  lmmg_accepted_authoring_hash=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("input", {}).get("navigation_authoring_sha256", ""))' \
    "${lmmg_candidate_acceptance}")
  lmmg_accepted_authoring_status_hash=$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("input", {}).get("navigation_authoring_status_sha256", ""))' \
    "${lmmg_candidate_acceptance}")
  lmmg_actual_authoring_hash=$(sha256sum "${lmmg_navigation_authoring}" | awk '{print $1}')
  lmmg_actual_authoring_status_hash=$(sha256sum \
    "${lmmg_navigation_authoring_status}" | awk '{print $1}')
  if [[ "${lmmg_accepted_authoring_scope}" != "${lmmg_authoring_scope}" ||
        "${lmmg_accepted_authoring_hash}" != "${lmmg_actual_authoring_hash}" ||
        "${lmmg_accepted_authoring_status_hash}" != \
          "${lmmg_actual_authoring_status_hash}" ]]; then
    echo "error: saved target-route editing files changed after validation; generate the map again" >&2
    exit 4
  fi
  if [[ "${lmmg_centerline_source}" == "recorded_trajectory" &&
        -f "${lmmg_maximal_authoring_fixture_manifest}" ]]; then
    if python3 - \
        "${lmmg_maximal_authoring_fixture_manifest}" \
        "${lmmg_navigation_authoring}" \
        "${lmmg_semantic_features_tsv}" \
        "${lmmg_full_map_source_graph}" \
        "${lmmg_candidate_acceptance}" <<'PY'
import hashlib
import json
import pathlib
import sys


class InvalidFixture(ValueError):
    pass


def load_object(path: pathlib.Path, label: str) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise InvalidFixture(f"{label} is not valid JSON: {error}") from error
    if not isinstance(value, dict):
        raise InvalidFixture(f"{label} is not a JSON object")
    return value


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def edge_ids(value: object, label: str) -> list[int]:
    if (
        not isinstance(value, list)
        or not value
        or any(not isinstance(item, int) or isinstance(item, bool) for item in value)
        or len(set(value)) != len(value)
    ):
        raise InvalidFixture(f"{label} must be a nonempty unique integer list")
    return value


def bound_file(record: dict, expected: pathlib.Path, label: str) -> None:
    filename = record.get("file")
    if not isinstance(filename, str) or not filename:
        raise InvalidFixture(f"{label}.file is missing")
    declared = pathlib.Path(filename)
    if not declared.is_absolute():
        declared = manifest_path.parent / declared
    if declared.resolve() != expected.resolve():
        raise InvalidFixture(f"{label}.file does not identify the staged source artifact")
    if record.get("sha256") != sha256(expected):
        raise InvalidFixture(f"{label}.sha256 does not match the staged source artifact")


manifest_path, authoring_path, semantic_path, graph_path, acceptance_path = (
    pathlib.Path(value) for value in sys.argv[1:]
)
try:
    manifest = load_object(manifest_path, "maximal authoring fixture manifest")
    authoring = manifest.get("authoring")
    semantic = manifest.get("semantic_authoring")
    source = manifest.get("source")
    selection = manifest.get("route_selection")
    preflight = manifest.get("terminal_planning_support_preflight")
    stop_line = manifest.get("stop_line")
    if (
        manifest.get("schema_version") != 1
        or manifest.get("kind")
        != "gui_equivalent_exact_autoware_replay_authoring_fixture"
        or manifest.get("scenario") != "full_map_route"
        or manifest.get("scope") != "autoware_lossless_replay"
        or manifest.get("operational_claim") is not False
        or not all(
            isinstance(value, dict)
            for value in (authoring, semantic, source, selection, preflight, stop_line)
        )
    ):
        raise InvalidFixture("fixture envelope or required sections differ")

    if (
        authoring.get("schema_version") != 1
        or authoring.get("gui_schema_equivalent") is not True
        or authoring.get("scope") != "autoware_lossless_replay"
    ):
        raise InvalidFixture("authoring record differs from the GUI-equivalent contract")
    bound_file(authoring, authoring_path, "authoring")
    if (
        semantic.get("schema_version") != 2
        or semantic.get("gui_schema_equivalent") is not True
    ):
        raise InvalidFixture("semantic authoring record differs from the GUI contract")
    bound_file(semantic, semantic_path, "semantic_authoring")

    graph = load_object(graph_path, "full-map source graph")
    graph_features = graph.get("features")
    if graph.get("type") != "FeatureCollection" or not isinstance(graph_features, list):
        raise InvalidFixture("full-map source graph is not a FeatureCollection")
    graph_edge_ids = []
    for feature in graph_features:
        if not isinstance(feature, dict):
            continue
        geometry = feature.get("geometry")
        properties = feature.get("properties")
        if (
            not isinstance(geometry, dict)
            or geometry.get("type") not in ("LineString", "MultiLineString")
        ):
            continue
        if not isinstance(properties, dict):
            raise InvalidFixture("full-map source graph Edge properties are malformed")
        identifier = properties.get("id")
        if not isinstance(identifier, int) or isinstance(identifier, bool):
            raise InvalidFixture("full-map source graph has a non-integer Edge ID")
        graph_edge_ids.append(identifier)
    graph_edge_ids = edge_ids(graph_edge_ids, "full-map source graph Edge IDs")

    full_ids = edge_ids(
        selection.get("full_ordered_edge_ids"),
        "route_selection.full_ordered_edge_ids",
    )
    selected_ids = edge_ids(
        selection.get("selected_mission_edge_ids"),
        "route_selection.selected_mission_edge_ids",
    )
    if (
        selection.get("algorithm")
        != "exact_chronological_replay_file_order_all_edges"
        or selection.get("edge_id_order_assumed") is not False
        or selection.get("source_order_preserved") is not True
        or selection.get("is_cycle") is not False
        or selection.get("full_edge_count") != len(full_ids)
        or selection.get("selected_edge_count") != len(selected_ids)
        or selection.get("excluded_terminal_support_edge_ids") != []
        or selection.get("excluded_terminal_support_edge_count") != 0
        or selection.get("selected_to_full_arc_length_ratio") != 1.0
        or full_ids != graph_edge_ids
        or selected_ids != full_ids
    ):
        raise InvalidFixture("route selection is not the exact complete replay Edge order")
    if (
        source.get("route_graph_sha256") != sha256(graph_path)
        or source.get("graph_fingerprint_recomputed") is not True
        or preflight.get("production_ready") is not False
        or preflight.get("deployment_ready") is not False
        or preflight.get("synthetic_support_is_part_of_named_route") is not False
        or stop_line.get("classification") != "closed_course_virtual_unsurveyed"
        or stop_line.get("physical_stop_line_verified") is not False
    ):
        raise InvalidFixture("source, preflight, or stop-line evidence differs")

    acceptance = load_object(acceptance_path, "candidate acceptance report")
    accepted_route = acceptance.get("named_route")
    if (
        acceptance.get("accepted") is not True
        or not isinstance(accepted_route, dict)
        or accepted_route.get("ordered_edge_ids") != selected_ids
    ):
        raise InvalidFixture("candidate acceptance does not bind the maximal Edge order")
except (InvalidFixture, OSError) as error:
    print(f"warning: maximal authoring fixture manifest ignored: {error}", file=sys.stderr)
    raise SystemExit(1)
PY
    then
      lmmg_maximal_authoring_fixture_valid=true
    else
      echo "warning: staging the named Route as a schema-5 GUI authoring bundle" >&2
    fi
  fi
elif [[ "${lmmg_coverage_reference}" != "full_processed_trajectory" &&
        "${lmmg_coverage_reference}" != "edited_topology_source_graph" ]]; then
  echo "error: unknown Vector Map trajectory-coverage mode: ${lmmg_coverage_reference}" >&2
  exit 4
fi
if ! xmllint --noout "${lmmg_lanelet}"; then
  echo "error: experimental Lanelet2 OSM is not well-formed XML" >&2
  exit 4
fi
lmmg_lanelet_count=$(xmllint --xpath \
  'count(/osm/relation[tag[@k="type" and @v="lanelet"]])' "${lmmg_lanelet}")
if [[ ! "${lmmg_lanelet_count}" =~ ^[0-9]+$ || "${lmmg_lanelet_count}" -eq 0 ]]; then
  echo "error: experimental Lanelet2 OSM contains no Lanelet relation" >&2
  exit 4
fi
if ! grep -Eq '<tag k="type" v="lanelet"/>' "${lmmg_lanelet}"; then
  echo "error: experimental OSM relations are not tagged as Lanelets" >&2
  exit 4
fi
if ! awk '$1 == "projector_type:" && $2 == "Local" { found = 1 } END { exit !found }' \
    "${lmmg_projector}"; then
  echo "error: map_projector_info.yaml must contain exact projector_type: Local" >&2
  exit 4
fi
lmmg_pcd_points=$(sed -n '/^POINTS /{s/^POINTS //;p}; /^DATA /q' "${lmmg_pointcloud}")
if [[ ! "${lmmg_pcd_points}" =~ ^[0-9]+$ || "${lmmg_pcd_points}" -eq 0 ]]; then
  echo "error: pointcloud_map.pcd has no positive POINTS declaration" >&2
  exit 4
fi

lmmg_lanelet_smoke_verified=false
lmmg_lanelet_smoke_report="${lmmg_output}/autoware_lanelet_smoke_report.txt"
if [[ -n "${lmmg_lanelet_smoke}" ]]; then
  lmmg_lanelet_smoke_temp="${lmmg_output}/.autoware_lanelet_smoke_report.tmp.$$"
  if "${lmmg_lanelet_smoke}" "${lmmg_lanelet}" >"${lmmg_lanelet_smoke_temp}" 2>&1 &&
      grep -qx 'AUTOWARE_LANELET_SMOKE=PASS' "${lmmg_lanelet_smoke_temp}"; then
    mv -- "${lmmg_lanelet_smoke_temp}" "${lmmg_lanelet_smoke_report}"
    lmmg_lanelet_smoke_verified=true
  else
    if [[ -f "${lmmg_lanelet_smoke_temp}" ]]; then
      mv -- "${lmmg_lanelet_smoke_temp}" "${lmmg_lanelet_smoke_report}"
    fi
    echo "error: Autoware Lanelet2 loading and routing check failed" >&2
    echo "       report: ${lmmg_lanelet_smoke_report}" >&2
    exit 4
  fi
elif [[ "${lmmg_require_lanelet_smoke}" == "true" ]]; then
  echo "error: the required Autoware Lanelet2 loading check is unavailable" >&2
  exit 4
else
  echo "warning: Autoware Lanelet2 loading check is unavailable; only static file checks were run" >&2
fi

# Never place the experimental OSM beside generated/canonical candidates under
# their original names. Autoware's default filename is lanelet2_map.osm, and
# some loaders merge every OSM in a directory.
lmmg_target="${lmmg_output}/autoware_closed_course_experimental_map"
if [[ -e "${lmmg_target}" && "${lmmg_replace}" != "--replace" ]]; then
  echo "error: staging directory already exists: ${lmmg_target}" >&2
  echo "       rerun with --replace after reviewing the regenerated source" >&2
  exit 4
fi
if [[ -e "${lmmg_target}" &&
      ! -f "${lmmg_target}/EXPERIMENTAL_ONLY.yaml" ]]; then
  echo "error: refusing to replace a directory without EXPERIMENTAL_ONLY.yaml" >&2
  exit 4
fi
lmmg_stage=$(mktemp -d \
  "${lmmg_output}/.autoware_closed_course_experimental_map.XXXXXX")
lmmg_cleanup_stage() {
  if [[ -n "${lmmg_stage:-}" && -d "${lmmg_stage}" &&
        "${lmmg_stage}" == "${lmmg_output}/.autoware_closed_course_experimental_map."* ]]; then
    rm -rf -- "${lmmg_stage}"
  fi
}
trap lmmg_cleanup_stage EXIT

# The stage lives on the same filesystem, so a hard link avoids duplicating a
# multi-hundred-megabyte PCD while presenting Autoware's expected filename.
ln -- "${lmmg_pointcloud}" "${lmmg_stage}/pointcloud_map.pcd"
cp -- "${lmmg_lanelet}" "${lmmg_stage}/lanelet2_map.osm"
cp -- "${lmmg_projector}" "${lmmg_stage}/map_projector_info.yaml"
lmmg_full_map_source_graph_name=$(basename -- "${lmmg_full_map_source_graph}")
lmmg_named_mission_source_graph_name=$(basename -- "${lmmg_named_mission_source_graph}")
cp -- "${lmmg_full_map_source_graph}" \
  "${lmmg_stage}/${lmmg_full_map_source_graph_name}"
cp -- "${lmmg_named_mission_source_graph}" \
  "${lmmg_stage}/${lmmg_named_mission_source_graph_name}"
if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
  cp -- "${lmmg_lossless_replay_source_graph}" \
    "${lmmg_stage}/route_graph_closed_course_replay_candidate.geojson"
  cp -- "${lmmg_autoware_replay_metadata}" \
    "${lmmg_stage}/route_graph_autoware_replay_candidate_metadata.yaml"
else
  cp -- "${lmmg_vector_map_source}" "${lmmg_stage}/vector_map_source.tsv"
fi
cp -- "${lmmg_candidate_acceptance}" \
  "${lmmg_stage}/autoware_candidate_acceptance.json"
if [[ "${lmmg_named_route}" == "true" ]]; then
  lmmg_navigation_authoring_name=$(basename -- "${lmmg_navigation_authoring}")
  lmmg_navigation_authoring_status_name=$(basename -- \
    "${lmmg_navigation_authoring_status}")
  lmmg_semantic_features_tsv_name=$(basename -- "${lmmg_semantic_features_tsv}")
  lmmg_semantic_features_geojson_name=$(basename -- \
    "${lmmg_semantic_features_geojson}")
  lmmg_autoware_semantic_graph_name=$(basename -- "${lmmg_autoware_semantic_graph}")
  lmmg_autoware_semantic_review_name=$(basename -- "${lmmg_autoware_semantic_review}")
  cp -- "${lmmg_navigation_authoring}" \
    "${lmmg_stage}/${lmmg_navigation_authoring_name}"
  cp -- "${lmmg_navigation_authoring_status}" \
    "${lmmg_stage}/${lmmg_navigation_authoring_status_name}"
  if [[ "${lmmg_maximal_authoring_fixture_valid}" == "true" ]]; then
    cp -- "${lmmg_maximal_authoring_fixture_manifest}" \
      "${lmmg_stage}/maximal_autoware_authoring_fixture_manifest.json"
  fi
  cp -- "${lmmg_semantic_features_tsv}" \
    "${lmmg_stage}/${lmmg_semantic_features_tsv_name}"
  cp -- "${lmmg_semantic_features_geojson}" \
    "${lmmg_stage}/${lmmg_semantic_features_geojson_name}"
  cp -- "${lmmg_autoware_semantic_graph}" \
    "${lmmg_stage}/${lmmg_autoware_semantic_graph_name}"
  cp -- "${lmmg_autoware_semantic_review}" \
    "${lmmg_stage}/${lmmg_autoware_semantic_review_name}"
  if [[ -n "${lmmg_terminal_support_source_graph}" ]]; then
    cp -- "${lmmg_terminal_support_source_graph}" \
      "${lmmg_stage}/route_graph_closed_course_topology_candidate.geojson"
  fi
fi
if [[ "${lmmg_lanelet_smoke_verified}" == "true" ]]; then
  cp -- "${lmmg_lanelet_smoke_report}" "${lmmg_stage}/autoware_lanelet_smoke_report.txt"
fi
lmmg_pointcloud_hash=$(sha256sum "${lmmg_pointcloud}" | awk '{print $1}')
lmmg_lanelet_hash=$(sha256sum "${lmmg_lanelet}" | awk '{print $1}')
lmmg_full_map_source_graph_hash=$(sha256sum \
  "${lmmg_full_map_source_graph}" | awk '{print $1}')
lmmg_lossless_replay_source_graph_hash=
lmmg_manifest_schema_version=2
if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
  lmmg_lossless_replay_source_graph_hash=$(sha256sum \
    "${lmmg_lossless_replay_source_graph}" | awk '{print $1}')
else
  # Schema 8 is the first source-bound user-authored topology staging bundle.
  lmmg_manifest_schema_version=8
fi
lmmg_named_mission_source_graph_hash=
lmmg_navigation_authoring_hash=
lmmg_navigation_authoring_status_hash=
lmmg_candidate_acceptance_hash=
lmmg_maximal_authoring_fixture_manifest_hash=
lmmg_terminal_support_source_graph_hash=
if [[ "${lmmg_named_route}" == "true" ]]; then
  # Schema 5 makes the dedicated Autoware replay authoring scope explicit.
  # Schema 3/4 bundles used navigation_authoring.json and are intentionally
  # not accepted as equivalent evidence by new strict runtime checks.
  # Schema 7 additionally stages and hash-binds the exact GUI-equivalent
  # maximal authoring fixture and enumerates every regular bundle artifact.
  if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
    lmmg_manifest_schema_version=5
  fi
  lmmg_named_mission_source_graph_hash=$(sha256sum \
    "${lmmg_named_mission_source_graph}" | awk '{print $1}')
  lmmg_navigation_authoring_hash=$(sha256sum "${lmmg_navigation_authoring}" | awk '{print $1}')
  lmmg_navigation_authoring_status_hash=$(sha256sum \
    "${lmmg_navigation_authoring_status}" | awk '{print $1}')
  lmmg_candidate_acceptance_hash=$(sha256sum \
    "${lmmg_candidate_acceptance}" | awk '{print $1}')
  if [[ "${lmmg_maximal_authoring_fixture_valid}" == "true" ]]; then
    lmmg_manifest_schema_version=7
    lmmg_maximal_authoring_fixture_manifest_hash=$(sha256sum \
      "${lmmg_maximal_authoring_fixture_manifest}" | awk '{print $1}')
  fi
  if [[ -n "${lmmg_terminal_support_source_graph}" ]]; then
    lmmg_terminal_support_source_graph_hash=$(sha256sum \
      "${lmmg_terminal_support_source_graph}" | awk '{print $1}')
    lmmg_accepted_terminal_support_hash=$(python3 -c \
      'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["terminal_support"]["source_graph_sha256"])' \
      "${lmmg_candidate_acceptance}")
    if [[ "${lmmg_accepted_terminal_support_hash}" != \
          "${lmmg_terminal_support_source_graph_hash}" ]]; then
      echo "error: terminal support source hash changed after candidate validation" >&2
      exit 4
    fi
  fi
fi
{
  printf 'schema_version: %s\n' "${lmmg_manifest_schema_version}"
  printf '%s\n' 'bundle: "autoware_closed_course_experimental_map"'
  printf '%s\n' 'experimental_only: true'
  printf '%s\n' 'production_ready: false'
  printf '%s\n' 'deployment_ready: false'
  printf '%s\n' 'acceptance_scope: "static_format_geometry_coverage_only"'
  printf '%s\n' 'accepted_for_autoware_motion_test: false'
  printf '%s\n' 'synthetic_planning_support_present: false'
  printf 'centerline_source: "%s"\n' "${lmmg_centerline_source}"
  printf 'user_authored: %s\n' "${lmmg_user_authored}"
  if [[ "${lmmg_centerline_source}" == "edited_topology" ]]; then
    printf '%s\n' \
      'runtime_route_geometry: "user_authored_vehicle_footprint_validated_lanelet_centerlines"'
  else
    printf '%s\n' \
      'runtime_route_geometry: "observed_raw_lanelet_centerlines_only"'
  fi
  printf 'lanelet2_local_loader_verified: %s\n' "${lmmg_lanelet_smoke_verified}"
  printf 'traffic_rules_verified: %s\n' "${lmmg_lanelet_smoke_verified}"
  printf 'routing_graph_verified: %s\n' "${lmmg_lanelet_smoke_verified}"
  if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
    printf '%s\n' 'autoware_replay_candidate_metadata: "route_graph_autoware_replay_candidate_metadata.yaml"'
    printf 'terminal_localization_settling_verified: %s\n' \
      "${lmmg_terminal_settling_verified}"
    printf 'terminal_tail_omitted: %s\n' "${lmmg_terminal_tail_omitted}"
    printf 'omitted_planar_length_m: %s\n' "${lmmg_omitted_planar_length_m}"
  else
    printf '%s\n' 'vector_map_source: "vector_map_source.tsv"'
    printf 'vector_map_source_sha256: "%s"\n' \
      "${lmmg_current_vector_map_source_hash}"
  fi
  printf '%s\n' 'physical_lane_boundaries_verified: false'
  printf 'source_output_directory: "%s"\n' "${lmmg_output}"
  printf 'pointcloud_points: %s\n' "${lmmg_pcd_points}"
  printf 'lanelet_relations: %s\n' "${lmmg_lanelet_count}"
  printf 'pointcloud_sha256: "%s"\n' "${lmmg_pointcloud_hash}"
  printf 'lanelet2_sha256: "%s"\n' "${lmmg_lanelet_hash}"
  printf 'full_map_source_graph: "%s"\n' "${lmmg_full_map_source_graph_name}"
  printf 'full_map_source_graph_sha256: "%s"\n' \
    "${lmmg_full_map_source_graph_hash}"
  if [[ "${lmmg_centerline_source}" == "recorded_trajectory" ]]; then
    printf '%s\n' \
      'lossless_replay_source_graph: "route_graph_closed_course_replay_candidate.geojson"'
    printf 'lossless_replay_source_graph_sha256: "%s"\n' \
      "${lmmg_lossless_replay_source_graph_hash}"
  fi
  if [[ "${lmmg_named_route}" == "true" ]]; then
    printf '%s\n' 'named_route:'
    printf '  id: %s\n' "${lmmg_named_route_id}"
    printf '  name: %s\n' "${lmmg_named_route_name_json}"
    printf '  target: "%s"\n' "${lmmg_named_route_target}"
    printf '  ordered_edges: %s\n' "${lmmg_named_route_edges}"
    printf '  authored_stop_lines: %s\n' "${lmmg_named_route_stops}"
    printf '  coverage_reference: "named_route_source_graph"\n'
    printf '  bidirectional_centerline_coverage: %s\n' "${lmmg_named_route_coverage}"
    printf '  mission_source_graph: "%s"\n' \
      "${lmmg_named_mission_source_graph_name}"
    printf '  authoring_scope: "%s"\n' "${lmmg_authoring_scope}"
    printf '  navigation_authoring: "%s"\n' \
      "${lmmg_navigation_authoring_name}"
    printf '  navigation_authoring_status: "%s"\n' \
      "${lmmg_navigation_authoring_status_name}"
    printf '  acceptance_report: "autoware_candidate_acceptance.json"\n'
    printf '  mission_source_graph_sha256: "%s"\n' \
      "${lmmg_named_mission_source_graph_hash}"
    printf '  navigation_authoring_sha256: "%s"\n' \
      "${lmmg_navigation_authoring_hash}"
    printf '  navigation_authoring_status_sha256: "%s"\n' \
      "${lmmg_navigation_authoring_status_hash}"
    printf '  acceptance_report_sha256: "%s"\n' "${lmmg_candidate_acceptance_hash}"
    if [[ "${lmmg_maximal_authoring_fixture_valid}" == "true" ]]; then
      printf '%s\n' \
        '  maximal_authoring_fixture_manifest: "maximal_autoware_authoring_fixture_manifest.json"'
      printf '  maximal_authoring_fixture_manifest_sha256: "%s"\n' \
        "${lmmg_maximal_authoring_fixture_manifest_hash}"
    fi
    if [[ -n "${lmmg_terminal_support_source_graph}" ]]; then
      lmmg_terminal_support_final_edge=$(python3 -c \
        'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["terminal_support"]["final_named_edge_id"])' \
        "${lmmg_candidate_acceptance}")
      lmmg_terminal_support_edge_ids=$(python3 -c \
        'import json,sys; print("[" + ", ".join(str(v) for v in json.load(open(sys.argv[1], encoding="utf-8"))["terminal_support"]["support_edge_ids"]) + "]")' \
        "${lmmg_candidate_acceptance}")
      lmmg_terminal_support_length=$(python3 -c \
        'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["terminal_support"]["support_length_m"])' \
        "${lmmg_candidate_acceptance}")
      printf '%s\n' '  terminal_planning_support:'
      printf '%s\n' '    present: true'
      printf '    final_named_edge_id: %s\n' "${lmmg_terminal_support_final_edge}"
      printf '    support_edge_ids: %s\n' "${lmmg_terminal_support_edge_ids}"
      printf '    support_length_m: %s\n' "${lmmg_terminal_support_length}"
      printf '%s\n' \
        '    source: "closed_course_semantic_topology"'
      printf '%s\n' \
        '    support_is_part_of_named_route: false'
      printf '%s\n' \
        '    source_graph: "route_graph_closed_course_topology_candidate.geojson"'
      printf '    source_graph_sha256: "%s"\n' \
        "${lmmg_terminal_support_source_graph_hash}"
    fi
  fi
  # Hash every regular staged artifact except this manifest itself.  Consumers
  # require the declared set to equal the directory set; an unlisted or
  # removed file is therefore a failure, not an ignored extra.
  printf '%s\n' 'artifact_sha256:'
  for lmmg_staged_artifact in "${lmmg_stage}"/*; do
    [[ -f "${lmmg_staged_artifact}" && ! -L "${lmmg_staged_artifact}" ]] || {
      echo "error: staged artifact is not a regular non-symlink file: ${lmmg_staged_artifact}" >&2
      exit 4
    }
    lmmg_staged_name=$(basename -- "${lmmg_staged_artifact}")
    [[ "${lmmg_staged_name}" != "EXPERIMENTAL_ONLY.yaml" ]] || continue
    lmmg_staged_hash=$(sha256sum -- "${lmmg_staged_artifact}" | awk '{print $1}')
    printf '  "%s": "%s"\n' "${lmmg_staged_name}" "${lmmg_staged_hash}"
  done
  printf '%s\n' 'required_before_motion:'
  printf '%s\n' '  - "controlled_closed_course"'
  printf '%s\n' '  - "map_and_localization_alignment_reviewed"'
  printf '%s\n' '  - "measured_vehicle_geometry_or_operator_acceptance_of_estimate"'
  printf '%s\n' '  - "live_obstacle_detection_and_emergency_stop_tested"'
} > "${lmmg_stage}/EXPERIMENTAL_ONLY.yaml"

lmmg_previous=
if [[ -e "${lmmg_target}" ]]; then
  lmmg_previous="${lmmg_output}/.autoware_closed_course_experimental_map.previous.$$"
  mv -- "${lmmg_target}" "${lmmg_previous}"
fi
if ! mv -- "${lmmg_stage}" "${lmmg_target}"; then
  if [[ -n "${lmmg_previous}" && -d "${lmmg_previous}" ]]; then
    mv -- "${lmmg_previous}" "${lmmg_target}"
  fi
  exit 5
fi
lmmg_stage=
trap - EXIT
if [[ -n "${lmmg_previous}" && -d "${lmmg_previous}" ]]; then
  rm -rf -- "${lmmg_previous}"
fi
echo "Vector Map runtime directory prepared under: ${lmmg_output}"
echo "warning: files were prepared, but vehicle motion was not tested; complete a separate planning test before enabling motion"
