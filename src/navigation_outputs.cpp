#include "lidar_mobility_map_generator/navigation_outputs.hpp"

#include "lidar_mobility_map_generator/nav2_experimental.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

std::string jsonEscape(const std::string & input)
{
  std::string output;
  output.reserve(input.size());
  for (const char character : input) {
    switch (character) {
      case '"': output += "\\\""; break;
      case '\\': output += "\\\\"; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default: output += character; break;
    }
  }
  return output;
}

std::string yamlQuote(const std::string & input)
{
  return '"' + jsonEscape(input) + '"';
}

void addReason(std::vector<std::string> & reasons, const std::string & reason)
{
  if (std::find(reasons.begin(), reasons.end(), reason) == reasons.end()) {
    reasons.push_back(reason);
  }
}

bool weaklyConnected(const RouteGraph & graph)
{
  if (graph.edges.empty()) {
    return false;
  }
  std::map<std::uint64_t, std::vector<std::uint64_t>> adjacency;
  for (const RouteEdge & edge : graph.edges) {
    adjacency[edge.from].push_back(edge.to);
    adjacency[edge.to].push_back(edge.from);
  }
  std::set<std::uint64_t> visited;
  std::queue<std::uint64_t> pending;
  pending.push(graph.edges.front().from);
  visited.insert(graph.edges.front().from);
  while (!pending.empty()) {
    const std::uint64_t current = pending.front();
    pending.pop();
    for (const std::uint64_t next : adjacency[current]) {
      if (visited.insert(next).second) {
        pending.push(next);
      }
    }
  }
  return visited.size() == adjacency.size();
}

bool hasExportableReplayCenterlines(const RouteGraph & graph)
{
  return !graph.edges.empty() && std::all_of(
    graph.edges.begin(), graph.edges.end(),
    [](const RouteEdge & edge) {
      return edge.passable && edge.centerline.size() >= 2U &&
             std::all_of(
        edge.centerline.begin(), edge.centerline.end(),
        [](const Vec3 & point) {return finite(point);});
    });
}

std::optional<Vec2> startDirection(const RouteEdge & edge)
{
  if (edge.centerline.size() < 2U) {
    return std::nullopt;
  }
  const Vec3 & start = edge.centerline.front();
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec2 direction{
      edge.centerline[index].x - start.x,
      edge.centerline[index].y - start.y};
    if (norm(direction) > 1.0e-12) {
      return normalized(direction);
    }
  }
  return std::nullopt;
}

std::optional<Vec2> endDirection(const RouteEdge & edge)
{
  if (edge.centerline.size() < 2U) {
    return std::nullopt;
  }
  const Vec3 & end = edge.centerline.back();
  for (std::size_t index = edge.centerline.size() - 1U; index > 0U; --index) {
    const Vec2 direction{
      end.x - edge.centerline[index - 1U].x,
      end.y - edge.centerline[index - 1U].y};
    if (norm(direction) > 1.0e-12) {
      return normalized(direction);
    }
  }
  return std::nullopt;
}

bool hasForwardIncompatibleCusp(const RouteGraph & graph)
{
  for (const RouteEdge & incoming : graph.edges) {
    const std::optional<Vec2> incoming_direction = endDirection(incoming);
    if (!incoming_direction) {
      continue;
    }
    for (const RouteEdge & outgoing : graph.edges) {
      if (incoming.id == outgoing.id || incoming.to != outgoing.from) {
        continue;
      }
      const std::optional<Vec2> outgoing_direction = startDirection(outgoing);
      if (!outgoing_direction) {
        continue;
      }
      const double jump = std::abs(normalizeAngle(
          std::atan2(outgoing_direction->y, outgoing_direction->x) -
          std::atan2(incoming_direction->y, incoming_direction->x))) * 180.0 / kPi;
      if (jump + 1.0e-9 >= 150.0) {
        return true;
      }
    }
  }
  return false;
}

void evaluateNav2ProductionReadiness(
  NavigationTargetReadiness & readiness,
  const MappingDataset & dataset,
  const PipelineResult & pipeline,
  const RouteValidationResult & route_validation,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & route_check_controls)
{
  const TraversabilityGridResult & grids = pipeline.grids;
  readiness.nav2_map_server_compatible =
    !grids.obstacle_grid.empty() &&
    hasMatchingGridGeometry(grids.obstacle_grid, grids.observed_free_grid) &&
    hasMatchingGridGeometry(grids.obstacle_grid, grids.unknown_grid);
  if (!readiness.nav2_map_server_compatible) {
    addReason(readiness.nav2_reasons, "nav2_grid_geometry_invalid");
  }
  readiness.nav2_route_server_compatible =
    config.output.nav2_route_max_chord_error > 0.0 &&
    config.output.nav2_route_max_segment_length > 0.0;
  if (!readiness.nav2_route_server_compatible) {
    addReason(readiness.nav2_reasons, "nav2_route_segmentation_parameters_invalid");
  }
  if (dataset.world_frame != config.output.frame_id || config.output.frame_id != "map") {
    addReason(readiness.nav2_reasons, "navigation_frame_is_not_map");
  }
  if (!config.extrinsics.verified) {
    addReason(readiness.nav2_reasons, "lidar_extrinsics_unverified");
  } else if (!evidenceSupportsProductionVerification(
      config.extrinsics.calibration_source, config.extrinsics.calibration_confidence))
  {
    addReason(readiness.nav2_reasons, "lidar_extrinsics_production_evidence_invalid");
  }
  if (!config.generator.robot.dimensions_verified) {
    addReason(readiness.nav2_reasons, "vehicle_dimensions_unverified");
  } else if (!evidenceSupportsProductionVerification(
      config.generator.robot.dimensions_source,
      config.generator.robot.dimensions_confidence))
  {
    addReason(readiness.nav2_reasons, "vehicle_dimensions_production_evidence_invalid");
  }
  if (!config.output.nav2_free_space_verified) {
    addReason(readiness.nav2_reasons, "nav2_free_space_evidence_unverified");
  }
  if (grids.observed_free_grid.occupiedCellCount() == 0U) {
    addReason(readiness.nav2_reasons, "no_explicit_observed_free_cells");
  }
  if (config.generator.traversability.unknown_space_policy != "occupied") {
    addReason(readiness.nav2_reasons, "unknown_space_policy_not_fail_closed");
  }
  if (!route_validation.operational_ready) {
    addReason(readiness.nav2_reasons, "no_valid_operational_route");
  }
  if (!weaklyConnected(route_validation.operational_graph)) {
    addReason(readiness.nav2_reasons, "operational_route_graph_disconnected");
  }
  const Nav2ClosedCourseAssessment operational_route_assessment =
    evaluateNav2ClosedCourseExperiment(
    pipeline, route_validation.operational_graph, config, route_check_controls);
  for (const std::string & blocker : operational_route_assessment.route_server_blockers) {
    addReason(readiness.nav2_reasons, blocker);
  }
  readiness.nav2_navigation_ready = readiness.nav2_map_server_compatible &&
    readiness.nav2_route_server_compatible && readiness.nav2_reasons.empty();
}

void evaluateNav2ExperimentalReadiness(
  NavigationTargetReadiness & readiness,
  const MappingDataset & dataset,
  const PipelineResult & pipeline,
  const RouteValidationResult * closed_course_route_validation,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & route_check_controls)
{
  const TraversabilityGridResult & grids = pipeline.grids;
  if (!readiness.nav2_map_server_compatible) {
    addReason(readiness.nav2_experimental_reasons, "nav2_grid_geometry_invalid");
  }
  if (!readiness.nav2_route_server_compatible) {
    addReason(
      readiness.nav2_experimental_reasons,
      "nav2_route_segmentation_parameters_invalid");
  }
  if (dataset.world_frame != config.output.frame_id || config.output.frame_id != "map") {
    addReason(readiness.nav2_experimental_reasons, "navigation_frame_is_not_map");
  }
  if (!evidenceSupportsClosedCourseExperiment(
      config.extrinsics.calibration_source, config.extrinsics.calibration_confidence))
  {
    addReason(
      readiness.nav2_experimental_reasons,
      "lidar_extrinsics_evidence_below_closed_course_threshold");
  } else if (!config.extrinsics.verified) {
    addReason(
      readiness.nav2_experimental_warnings,
      "lidar_extrinsics_not_production_verified");
  }
  if (!evidenceSupportsClosedCourseExperiment(
      config.generator.robot.dimensions_source,
      config.generator.robot.dimensions_confidence))
  {
    addReason(
      readiness.nav2_experimental_reasons,
      "vehicle_dimensions_evidence_below_closed_course_threshold");
  } else if (!config.generator.robot.dimensions_verified) {
    addReason(
      readiness.nav2_experimental_warnings,
      "vehicle_dimensions_not_production_verified");
  }
  const std::string & free_mode =
    config.generator.traversability.free_space_evidence_mode;
  if (free_mode != "ground_observations" && free_mode != "combined") {
    addReason(
      readiness.nav2_experimental_reasons,
      "closed_course_ground_free_space_evidence_not_enabled");
  }
  if (grids.observed_free_grid.occupiedCellCount() == 0U) {
    addReason(readiness.nav2_experimental_reasons, "no_explicit_observed_free_cells");
  }
  if (config.generator.traversability.unknown_space_policy != "occupied") {
    addReason(readiness.nav2_experimental_reasons, "unknown_space_policy_not_fail_closed");
  }
  if (closed_course_route_validation == nullptr) {
    addReason(
      readiness.nav2_experimental_reasons,
      "closed_course_route_validation_not_provided");
  } else {
    if (!closed_course_route_validation->operational_ready) {
      addReason(readiness.nav2_experimental_reasons, "no_valid_closed_course_route");
    }
    if (!weaklyConnected(closed_course_route_validation->operational_graph)) {
      addReason(
        readiness.nav2_experimental_warnings,
        "closed_course_route_graph_disconnected_partial_coverage");
    }
    const Nav2ClosedCourseAssessment experimental_route_assessment =
      evaluateNav2ClosedCourseExperiment(
      pipeline, closed_course_route_validation->operational_graph,
      config, route_check_controls);
    for (const std::string & blocker : experimental_route_assessment.route_server_blockers) {
      addReason(readiness.nav2_experimental_reasons, blocker);
    }
  }
  readiness.nav2_closed_course_experimental_ready =
    readiness.nav2_map_server_compatible && readiness.nav2_route_server_compatible &&
    readiness.nav2_experimental_reasons.empty();
}

void evaluateAutowareProductionReadiness(
  NavigationTargetReadiness & readiness,
  const MappingDataset & dataset,
  const RouteValidationResult & route_validation,
  const ApplicationConfig & config)
{
  const Lanelet2Config & lanelet = config.generator.lanelet2;
  readiness.autoware_map_loader_compatible =
    dataset.world_frame == config.output.frame_id && config.output.frame_id == "map" &&
    lanelet.one_way && (lanelet.location == "urban" || lanelet.location == "nonurban");
  if (dataset.world_frame != config.output.frame_id || config.output.frame_id != "map") {
    addReason(readiness.autoware_reasons, "navigation_frame_is_not_map");
  }
  if (!lanelet.one_way) {
    addReason(readiness.autoware_reasons, "autoware_requires_directed_one_way_lanelets");
  }
  if (lanelet.location != "urban" && lanelet.location != "nonurban") {
    addReason(readiness.autoware_reasons, "lanelet_location_not_standard");
  }
  if (!config.extrinsics.verified) {
    addReason(readiness.autoware_reasons, "lidar_extrinsics_unverified");
  } else if (!evidenceSupportsProductionVerification(
      config.extrinsics.calibration_source, config.extrinsics.calibration_confidence))
  {
    addReason(readiness.autoware_reasons, "lidar_extrinsics_production_evidence_invalid");
  }
  if (!config.generator.robot.dimensions_verified) {
    addReason(readiness.autoware_reasons, "vehicle_dimensions_unverified");
  } else if (!evidenceSupportsProductionVerification(
      config.generator.robot.dimensions_source,
      config.generator.robot.dimensions_confidence))
  {
    addReason(readiness.autoware_reasons, "vehicle_dimensions_production_evidence_invalid");
  }
  if (config.generator.robot.base_reference != "rear_axle_ground_projection") {
    addReason(readiness.autoware_reasons, "base_reference_is_not_rear_axle_ground_projection");
  }
  if (!config.output.lanelet2_physical_boundaries_verified) {
    addReason(readiness.autoware_reasons, "lanelet_physical_boundaries_unverified");
  }
  if (!route_validation.operational_ready) {
    addReason(readiness.autoware_reasons, "no_valid_operational_route");
  }
  if (!weaklyConnected(route_validation.operational_graph)) {
    addReason(readiness.autoware_reasons, "operational_route_graph_disconnected");
  }
  if (std::any_of(
      route_validation.operational_graph.nodes.begin(),
      route_validation.operational_graph.nodes.end(),
      [](const RouteNode & node) {return node.type == RouteNodeType::kJunction;}))
  {
    addReason(readiness.autoware_reasons, "junction_lanelet_topology_not_authored");
  }
  if (std::any_of(
      route_validation.operational_graph.edges.begin(),
      route_validation.operational_graph.edges.end(),
      [](const RouteEdge & edge) {return edge.reverse_of.has_value();}))
  {
    addReason(readiness.autoware_reasons, "bidirectional_lanelet_topology_not_authored");
  }
  readiness.autoware_navigation_ready = readiness.autoware_map_loader_compatible &&
    readiness.autoware_reasons.empty();
}

void evaluateAutowareExperimentalReadiness(
  NavigationTargetReadiness & readiness,
  const MappingDataset & dataset,
  const RouteValidationResult * autoware_candidate,
  const ApplicationConfig & config)
{
  const Lanelet2Config & lanelet = config.generator.lanelet2;
  if (dataset.world_frame != config.output.frame_id || config.output.frame_id != "map") {
    addReason(readiness.autoware_experimental_reasons, "navigation_frame_is_not_map");
  }
  if (!lanelet.one_way) {
    addReason(
      readiness.autoware_experimental_reasons,
      "autoware_requires_directed_one_way_lanelets");
  }
  if (lanelet.location != "urban" && lanelet.location != "nonurban") {
    addReason(readiness.autoware_experimental_reasons, "lanelet_location_not_standard");
  }
  if (!evidenceSupportsClosedCourseExperiment(
      config.extrinsics.calibration_source, config.extrinsics.calibration_confidence))
  {
    addReason(
      readiness.autoware_experimental_reasons,
      "lidar_extrinsics_evidence_below_closed_course_threshold");
  } else if (!config.extrinsics.verified) {
    addReason(
      readiness.autoware_experimental_warnings,
      "lidar_extrinsics_not_production_verified");
  }
  if (!evidenceSupportsClosedCourseExperiment(
      config.generator.robot.dimensions_source,
      config.generator.robot.dimensions_confidence))
  {
    addReason(
      readiness.autoware_experimental_reasons,
      "vehicle_dimensions_evidence_below_closed_course_threshold");
  } else if (!config.generator.robot.dimensions_verified) {
    addReason(
      readiness.autoware_experimental_warnings,
      "vehicle_dimensions_not_production_verified");
  }
  if (config.generator.robot.base_reference != "rear_axle_ground_projection") {
    addReason(
      readiness.autoware_experimental_reasons,
      "base_reference_is_not_rear_axle_ground_projection");
  }
  if (!config.output.lanelet2_physical_boundaries_verified) {
    addReason(
      readiness.autoware_experimental_warnings,
      "lanelet_physical_boundaries_unverified_experimental_corridors_only");
  }
  if (autoware_candidate == nullptr) {
    addReason(
      readiness.autoware_experimental_reasons,
      "closed_course_route_validation_not_provided");
  } else {
    const RouteGraph & experimental_graph = autoware_candidate->operational_graph;
    if (!autoware_candidate->operational_ready) {
      addReason(
        readiness.autoware_experimental_reasons,
        "no_valid_closed_course_route");
    }
    if (!hasExportableReplayCenterlines(experimental_graph)) {
      addReason(
        readiness.autoware_experimental_reasons,
        "observed_replay_centerlines_unavailable");
    }
    if (!weaklyConnected(experimental_graph)) {
      addReason(
        readiness.autoware_experimental_warnings,
        "closed_course_route_graph_disconnected_partial_coverage");
    }
    if (std::any_of(
        experimental_graph.nodes.begin(), experimental_graph.nodes.end(),
        [](const RouteNode & node) {return node.type == RouteNodeType::kJunction;}))
    {
      addReason(
        readiness.autoware_experimental_reasons,
        "junction_lanelet_topology_not_authored");
    }
    if (std::any_of(
        experimental_graph.edges.begin(), experimental_graph.edges.end(),
        [](const RouteEdge & edge) {return edge.reverse_of.has_value();}))
    {
      addReason(
        readiness.autoware_experimental_reasons,
        "bidirectional_lanelet_topology_not_authored");
    }
    if (hasForwardIncompatibleCusp(experimental_graph)) {
      addReason(
        readiness.autoware_experimental_reasons,
        "forward_replay_contains_unhandled_cusp");
    }
  }
  readiness.autoware_closed_course_experimental_ready =
    readiness.autoware_map_loader_compatible &&
    readiness.autoware_experimental_reasons.empty();
}

void applyTargetSelectionReadiness(NavigationTargetReadiness & readiness)
{
  // Compatibility describes the source data, but a non-selected target must
  // never be promoted or staged. Record one deterministic blocker in both
  // readiness paths while retaining compatibility diagnostics for audits.
  if (!readiness.nav2_enabled) {
    addReason(readiness.nav2_reasons, "target_not_selected");
    addReason(readiness.nav2_experimental_reasons, "target_not_selected");
    readiness.nav2_navigation_ready = false;
    readiness.nav2_closed_course_experimental_ready = false;
  }
  if (!readiness.autoware_enabled) {
    addReason(readiness.autoware_reasons, "target_not_selected");
    addReason(readiness.autoware_experimental_reasons, "target_not_selected");
    readiness.autoware_navigation_ready = false;
    readiness.autoware_closed_course_experimental_ready = false;
  }
}

}  // namespace

NavigationTargetReadiness evaluateNavigationTargetReadiness(
  const MappingDataset & dataset,
  const PipelineResult & pipeline,
  const RouteValidationResult & route_validation,
  const ApplicationConfig & config,
  const RouteValidationResult * closed_course_route_validation,
  const RouteValidationResult * autoware_closed_course_route_validation)
{
  NavigationTargetReadiness readiness;
  readiness.nav2_enabled = config.output.nav2Enabled();
  readiness.autoware_enabled = config.output.autowareEnabled();
  const RouteValidationResult * autoware_candidate =
    autoware_closed_course_route_validation != nullptr ?
    autoware_closed_course_route_validation : closed_course_route_validation;
  Nav2ClosedCourseControls route_check_controls;
  route_check_controls.enabled = true;

  evaluateNav2ProductionReadiness(
    readiness, dataset, pipeline, route_validation, config, route_check_controls);
  evaluateNav2ExperimentalReadiness(
    readiness, dataset, pipeline, closed_course_route_validation, config,
    route_check_controls);
  evaluateAutowareProductionReadiness(
    readiness, dataset, route_validation, config);
  evaluateAutowareExperimentalReadiness(
    readiness, dataset, autoware_candidate, config);
  applyTargetSelectionReadiness(readiness);
  return readiness;
}

void saveNavigationTargetReadinessYaml(
  const std::filesystem::path & path,
  const NavigationTargetReadiness & readiness,
  const MappingDataset & dataset,
  const PipelineResult & pipeline,
  const RouteValidationResult & route_validation,
  const ApplicationConfig & config,
  const RouteValidationResult * closed_course_route_validation,
  const RouteValidationResult * autoware_closed_course_route_validation,
  const std::string & autoware_centerline_source)
{
  const RouteValidationResult * autoware_candidate =
    autoware_closed_course_route_validation != nullptr ?
    autoware_closed_course_route_validation : closed_course_route_validation;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const std::filesystem::path temporary = path.string() + ".lmmg.tmp";
  std::ofstream stream(temporary, std::ios::trunc);
  if (!stream) {
    throw std::runtime_error(
            "failed to create navigation readiness report: " + temporary.string());
  }
  stream << std::setprecision(12)
         << "schema_version: 3\n"
         << "generation_complete: true\n"
         << "frame_id: " << yamlQuote(dataset.world_frame) << '\n'
         << "requested_target_mode: " << yamlQuote(config.output.target_mode) << '\n'
         << "canonical_outputs_fail_closed: true\n"
         << "calibration_evidence:\n"
         << "  vehicle_dimensions:\n"
         << "    source: " << yamlQuote(config.generator.robot.dimensions_source) << '\n'
         << "    confidence: " <<
    yamlQuote(config.generator.robot.dimensions_confidence) << '\n'
         << "    production_verification_claim: " <<
    (config.generator.robot.dimensions_verified ? "true" : "false") << '\n'
         << "    production_verified: " <<
    ((config.generator.robot.dimensions_verified && evidenceSupportsProductionVerification(
      config.generator.robot.dimensions_source,
      config.generator.robot.dimensions_confidence)) ? "true" : "false") << '\n'
         << "  lidar_extrinsics:\n"
         << "    transform_load_source: " << yamlQuote(config.extrinsics.source) << '\n'
         << "    calibration_source: " <<
    yamlQuote(config.extrinsics.calibration_source) << '\n'
         << "    confidence: " <<
    yamlQuote(config.extrinsics.calibration_confidence) << '\n'
         << "    production_verification_claim: " <<
    (config.extrinsics.verified ? "true" : "false") << '\n'
         << "    production_verified: " <<
    ((config.extrinsics.verified && evidenceSupportsProductionVerification(
      config.extrinsics.calibration_source,
      config.extrinsics.calibration_confidence)) ? "true" : "false") << '\n'
         << "nav2:\n"
         << "  enabled: " << (readiness.nav2_enabled ? "true" : "false") << '\n'
         << "  map_server_compatible: " <<
    (readiness.nav2_map_server_compatible ? "true" : "false") << '\n'
         << "  route_server_compatible: " <<
    (readiness.nav2_route_server_compatible ? "true" : "false") << '\n'
         << "  navigation_ready: " <<
    (readiness.nav2_navigation_ready ? "true" : "false") << '\n'
         << "  production_ready: " <<
    (readiness.nav2_navigation_ready ? "true" : "false") << '\n'
         << "  closed_course_experimental_ready: " <<
    (readiness.nav2_closed_course_experimental_ready ? "true" : "false") << '\n'
         << "  canonical_map: \"nav2_map.yaml\"\n"
         << "  candidate_map: \"nav2_map_generated.yaml\"\n"
         << "  canonical_route_graph: \"nav2_route_graph.geojson\"\n"
         << "  candidate_route_graph: \"nav2_route_graph_generated.geojson\"\n"
         << "  free_space_verified: " <<
    (config.output.nav2_free_space_verified ? "true" : "false") << '\n'
         << "  free_space_evidence_mode: " <<
    yamlQuote(config.generator.traversability.free_space_evidence_mode) << '\n'
         << "  trajectory_free_space_model: " <<
    yamlQuote(config.generator.traversability.trajectory_free_space_model) << '\n'
         << "  trajectory_footprint_erosion_margin: " <<
    config.generator.traversability.trajectory_footprint_erosion_margin << '\n'
         << "  explicit_free_cells: " <<
    pipeline.grids.observed_free_grid.occupiedCellCount() << '\n'
         << "  direct_ground_free_cells: " <<
    pipeline.grids.ground_observation_free_cells << '\n'
         << "  unknown_cells: " << pipeline.grids.unknown_grid.occupiedCellCount() << '\n'
         << "  reasons:";
  if (readiness.nav2_reasons.empty()) {
    stream << " []\n";
  } else {
    stream << '\n';
    for (const std::string & reason : readiness.nav2_reasons) {
      stream << "    - " << yamlQuote(reason) << '\n';
    }
  }
  stream << "  closed_course_route_edges: " <<
    (closed_course_route_validation == nullptr ? 0U :
  closed_course_route_validation->operational_graph.edges.size()) << '\n'
         << "  experimental_reasons:";
  if (readiness.nav2_experimental_reasons.empty()) {
    stream << " []\n";
  } else {
    stream << '\n';
    for (const std::string & reason : readiness.nav2_experimental_reasons) {
      stream << "    - " << yamlQuote(reason) << '\n';
    }
  }
  stream << "  experimental_warnings:";
  if (readiness.nav2_experimental_warnings.empty()) {
    stream << " []\n";
  } else {
    stream << '\n';
    for (const std::string & warning : readiness.nav2_experimental_warnings) {
      stream << "    - " << yamlQuote(warning) << '\n';
    }
  }
  stream << "autoware:\n"
         << "  enabled: " << (readiness.autoware_enabled ? "true" : "false") << '\n'
         << "  centerline_source: " << yamlQuote(autoware_centerline_source) << '\n'
         << "  map_loader_compatible: " <<
    (readiness.autoware_map_loader_compatible ? "true" : "false") << '\n'
         << "  navigation_ready: " <<
    (readiness.autoware_navigation_ready ? "true" : "false") << '\n'
         << "  production_ready: " <<
    (readiness.autoware_navigation_ready ? "true" : "false") << '\n'
         << "  closed_course_experimental_ready: " <<
    (readiness.autoware_closed_course_experimental_ready ? "true" : "false") << '\n'
         << "  canonical_lanelet2_map: \"lanelet2_map.osm\"\n"
         << "  candidate_lanelet2_map: \"lanelet2_map_generated.osm\"\n"
         << "  closed_course_experimental_lanelet2_map: "
         << "\"lanelet2_map_closed_course_experimental.osm\"\n"
         << "  replay_candidate_graph: "
         << "\"route_graph_autoware_replay_candidate.geojson\"\n"
         << "  replay_candidate_metadata: "
         << "\"route_graph_autoware_replay_candidate_metadata.yaml\"\n"
         << "  selected_source_graph: "
         << "\"route_graph_autoware_selected_source.geojson\"\n"
         << "  projector_info: \"map_projector_info.yaml\"\n"
         << "  physical_boundaries_verified: " <<
    (config.output.lanelet2_physical_boundaries_verified ? "true" : "false") << '\n'
         << "  estimated_corridors_available: " <<
    ((readiness.autoware_enabled && autoware_candidate != nullptr && hasExportableReplayCenterlines(
      autoware_candidate->operational_graph)) ? "true" : "false") << '\n'
         << "  base_reference: " << yamlQuote(config.generator.robot.base_reference) << '\n'
         << "  operational_edges: " <<
    (readiness.autoware_enabled ? route_validation.operational_graph.edges.size() : 0U) << '\n'
         << "  reasons:";
  if (readiness.autoware_reasons.empty()) {
    stream << " []\n";
  } else {
    stream << '\n';
    for (const std::string & reason : readiness.autoware_reasons) {
      stream << "    - " << yamlQuote(reason) << '\n';
    }
  }
  stream << "  closed_course_route_edges: " <<
    (!readiness.autoware_enabled || autoware_candidate == nullptr ? 0U :
  autoware_candidate->operational_graph.edges.size()) << '\n'
         << "  experimental_reasons:";
  if (readiness.autoware_experimental_reasons.empty()) {
    stream << " []\n";
  } else {
    stream << '\n';
    for (const std::string & reason : readiness.autoware_experimental_reasons) {
      stream << "    - " << yamlQuote(reason) << '\n';
    }
  }
  stream << "  experimental_warnings:";
  if (readiness.autoware_experimental_warnings.empty()) {
    stream << " []\n";
  } else {
    stream << '\n';
    for (const std::string & warning : readiness.autoware_experimental_warnings) {
      stream << "    - " << yamlQuote(warning) << '\n';
    }
  }
  stream.close();
  if (!stream) {
    std::error_code cleanup_error;
    std::filesystem::remove(temporary, cleanup_error);
    throw std::runtime_error("failed to finish navigation readiness report: " + path.string());
  }
  std::error_code rename_error;
  std::filesystem::rename(temporary, path, rename_error);
  if (rename_error) {
    std::error_code cleanup_error;
    std::filesystem::remove(temporary, cleanup_error);
    throw std::runtime_error(
            "failed to atomically install navigation readiness report: " +
            rename_error.message());
  }
}

}  // namespace lidar_mobility_map_generator
