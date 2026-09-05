#include "lidar_mobility_map_generator/pipeline.hpp"

#include "lidar_mobility_map_generator/observed_route_graph.hpp"
#include "lidar_mobility_map_generator/route_graph.hpp"
#include "lidar_mobility_map_generator/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

constexpr double kGeometryAuditEpsilon = 1.0e-12;
constexpr double kGeometryAuditTolerance = 1.0e-8;

double planarLength(const std::vector<Vec3> & points)
{
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result += distance2d(points[index - 1U], points[index]);
  }
  return result;
}

void recordGeometryAuditFailure(
  RouteGeometryAudit & audit,
  const std::uint64_t edge_id,
  const std::size_t segment_index,
  const std::string & reason)
{
  audit.valid = false;
  if (audit.first_invalid_reason.empty()) {
    audit.first_invalid_edge_id = edge_id;
    audit.first_invalid_segment_index = segment_index;
    audit.first_invalid_reason = reason;
  }
}

RouteGeometryAudit auditRouteGeometry(
  const RouteGraph & graph,
  const std::vector<TimedPose> * source_trajectory)
{
  RouteGeometryAudit audit;
  std::vector<Vec3> route_points;
  for (std::size_t edge_index = 0U; edge_index < graph.edges.size(); ++edge_index) {
    const RouteEdge & edge = graph.edges[edge_index];
    if (edge.centerline.size() < 2U) {
      recordGeometryAuditFailure(audit, edge.id, 0U, "centerline_has_fewer_than_two_points");
      continue;
    }
    if (source_trajectory != nullptr && edge_index > 0U) {
      const RouteEdge & previous = graph.edges[edge_index - 1U];
      if (previous.to != edge.from ||
        distance3d(previous.centerline.back(), edge.centerline.front()) >
        kGeometryAuditTolerance)
      {
        recordGeometryAuditFailure(audit, edge.id, 0U, "chronological_edge_chain_disconnected");
      }
    }
    audit.route_planar_length += planarLength(edge.centerline);
    audit.route_spatial_length += polylineLength(edge.centerline);
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      ++audit.segments_evaluated;
      const Vec3 & first = edge.centerline[index - 1U];
      const Vec3 & second = edge.centerline[index];
      if (!finite(first) || !finite(second)) {
        ++audit.nonfinite_segments;
        recordGeometryAuditFailure(audit, edge.id, index - 1U, "nonfinite_centerline_segment");
        continue;
      }
      const double delta_z = std::abs(second.z - first.z);
      const double horizontal_distance = distance2d(first, second);
      if (!std::isfinite(delta_z) || !std::isfinite(horizontal_distance)) {
        ++audit.nonfinite_segments;
        recordGeometryAuditFailure(audit, edge.id, index - 1U, "nonfinite_centerline_metric");
        continue;
      }
      if (delta_z > audit.maximum_absolute_delta_z) {
        audit.maximum_absolute_delta_z = delta_z;
        audit.maximum_delta_z_edge_id = edge.id;
        audit.maximum_delta_z_segment_index = index - 1U;
      }
      if (horizontal_distance <= kGeometryAuditEpsilon) {
        if (delta_z > kGeometryAuditEpsilon) {
          ++audit.zero_horizontal_distance_z_change_segments;
          recordGeometryAuditFailure(
            audit, edge.id, index - 1U, "z_change_at_zero_horizontal_distance");
        }
        continue;
      }
      const double grade = delta_z / horizontal_distance;
      if (!std::isfinite(grade)) {
        ++audit.nonfinite_segments;
        recordGeometryAuditFailure(audit, edge.id, index - 1U, "nonfinite_centerline_grade");
      } else if (grade > audit.maximum_absolute_grade) {
        audit.maximum_absolute_grade = grade;
        audit.maximum_grade_edge_id = edge.id;
        audit.maximum_grade_segment_index = index - 1U;
      }
    }
    if (source_trajectory != nullptr) {
      const std::size_t begin = edge_index == 0U ? 0U : 1U;
      route_points.insert(
        route_points.end(), edge.centerline.begin() + static_cast<std::ptrdiff_t>(begin),
        edge.centerline.end());
    }
  }

  if (source_trajectory == nullptr) {
    return audit;
  }
  audit.source_pose_count = source_trajectory->size();
  audit.source_segments_evaluated = audit.source_pose_count > 0U ?
    audit.source_pose_count - 1U : 0U;
  std::vector<Vec3> source_points;
  source_points.reserve(source_trajectory->size());
  for (const TimedPose & pose : *source_trajectory) {
    source_points.push_back(pose.world_from_body.translation);
  }
  if (!source_points.empty()) {
    audit.source_start = source_points.front();
    audit.source_end = source_points.back();
  }
  audit.source_planar_length = planarLength(source_points);
  audit.source_spatial_length = polylineLength(source_points);
  audit.planar_length_coverage = audit.source_planar_length > kGeometryAuditEpsilon ?
    audit.route_planar_length / audit.source_planar_length : 0.0;
  audit.spatial_length_coverage = audit.source_spatial_length > kGeometryAuditEpsilon ?
    audit.route_spatial_length / audit.source_spatial_length : 0.0;

  std::size_t route_index = 0U;
  for (const Vec3 & source : source_points) {
    while (route_index < route_points.size() &&
      distance3d(source, route_points[route_index]) > kGeometryAuditTolerance)
    {
      ++route_index;
    }
    if (route_index == route_points.size()) {
      break;
    }
    ++audit.represented_source_pose_count;
    ++route_index;
  }
  audit.source_pose_projection_coverage = audit.source_pose_count > 0U ?
    static_cast<double>(audit.represented_source_pose_count) /
    static_cast<double>(audit.source_pose_count) : 0.0;
  if (audit.represented_source_pose_count != audit.source_pose_count) {
    recordGeometryAuditFailure(audit, 0U, 0U, "processed_pose_order_or_coverage_changed");
  }
  const double planar_tolerance = std::max(
    kGeometryAuditTolerance, audit.source_planar_length * 1.0e-10);
  const double spatial_tolerance = std::max(
    kGeometryAuditTolerance, audit.source_spatial_length * 1.0e-10);
  if (std::abs(audit.route_planar_length - audit.source_planar_length) > planar_tolerance) {
    recordGeometryAuditFailure(audit, 0U, 0U, "planar_source_length_changed");
  }
  if (std::abs(audit.route_spatial_length - audit.source_spatial_length) > spatial_tolerance) {
    recordGeometryAuditFailure(audit, 0U, 0U, "spatial_source_length_changed");
  }
  return audit;
}

void requireValidGeometryAudit(const RouteGeometryAudit & audit, const std::string & label)
{
  if (audit.valid) {
    return;
  }
  throw std::runtime_error(
          label + " geometry audit failed: " + audit.first_invalid_reason +
          " (edge=" + std::to_string(audit.first_invalid_edge_id) +
          ", segment=" + std::to_string(audit.first_invalid_segment_index) + ")");
}

}  // namespace

PipelineResult runVectorMapPipeline(
  const MappingDataset & dataset,
  const GeneratorConfig & config)
{
  if (dataset.map_points.empty()) {
    throw std::runtime_error("input point cloud map is empty");
  }
  if (dataset.trajectory.size() < 2U) {
    throw std::runtime_error("input trajectory contains fewer than two poses");
  }

  PipelineResult result;
  result.generation.warnings = dataset.warnings;
  result.generation.statistics.raw_trajectory_poses = dataset.trajectory.size();
  const std::vector<TimedPose> observed_free_space_trajectory =
    normalizeTrajectory(dataset.trajectory);
  TrajectoryProcessingAudit trajectory_processing_audit;
  result.generation.processed_trajectory = processTrajectory(
    dataset.trajectory, config.trajectory, &result.generation.warnings,
    &trajectory_processing_audit);
  result.generation.statistics.processed_trajectory_poses =
    result.generation.processed_trajectory.size();
  result.generation.statistics.raw_trajectory_preserved =
    trajectory_processing_audit.raw_trajectory_preserved;
  result.generation.statistics.corrected_position_jitter_poses =
    trajectory_processing_audit.corrected_position_jitter_poses;
  result.generation.statistics.corrected_position_jitter_runs =
    trajectory_processing_audit.corrected_position_jitter_runs;
  result.generation.statistics.maximum_planar_position_correction_m =
    trajectory_processing_audit.maximum_planar_position_correction_m;
  result.generation.statistics.planar_length_before_position_jitter_correction_m =
    trajectory_processing_audit.planar_length_before_position_jitter_correction_m;
  result.generation.statistics.planar_length_after_position_jitter_correction_m =
    trajectory_processing_audit.planar_length_after_position_jitter_correction_m;

  result.grids = buildTraversabilityGrid(
    dataset.map_points,
    result.generation.processed_trajectory,
    observed_free_space_trajectory,
    config.traversability,
    config.robot);
  result.generation.statistics.obstacle_points = result.grids.obstacle_points;
  result.generation.statistics.obstacle_cells = result.grids.obstacle_grid.occupiedCellCount();
  result.generation.statistics.trajectory_cleared_obstacle_cells =
    result.grids.trajectory_cleared_obstacle_cells;
  result.generation.statistics.inflated_obstacle_cells =
    result.grids.inflated_grid.occupiedCellCount();

  if (result.grids.fallback_ground_samples > 0U) {
    const double ratio = static_cast<double>(result.grids.fallback_ground_samples) /
      static_cast<double>(result.generation.processed_trajectory.size());
    result.generation.warnings.push_back(
      "floor height fallback was used at " +
      std::to_string(result.grids.fallback_ground_samples) + " trajectory samples (" +
      std::to_string(100.0 * ratio) + " percent)");
  }
  if (result.grids.trajectory_cleared_obstacle_cells > 0U) {
    result.generation.warnings.push_back(
      std::to_string(result.grids.trajectory_cleared_obstacle_cells) +
      " obstacle cells were cleared using observed-trajectory free-space evidence; "
      "verify the configured disk or eroded footprint model against the platform");
  }
  const double footprint_inradius = config.robot.footprint_model == "circle" ?
    0.5 * config.robot.width :
    std::min({
      0.5 * config.robot.width, config.robot.front_extent, config.robot.rear_extent});
  if (config.traversability.observed_trajectory_clearance_radius >
    footprint_inradius + 1.0e-9)
  {
    result.generation.warnings.push_back(
      "observed-trajectory clearance extends outside the configured robot body; cleared cells "
      "cannot be justified as vehicle self-space");
  }
  if (!config.robot.dimensions_verified) {
    if (evidenceSupportsClosedCourseExperiment(
        config.robot.dimensions_source, config.robot.dimensions_confidence))
    {
      result.generation.warnings.push_back(
        "robot footprint uses identified estimated dimensions; closed-course experimental "
        "artifacts may be reviewed, but production operation remains unverified");
    } else {
      result.generation.warnings.push_back(
        "robot footprint evidence is below the closed-course threshold; base_link-relative "
        "extents and collision heights remain diagnostic only");
    }
  }
  if (result.grids.low_support_obstacle_cells > 0U) {
    result.generation.warnings.push_back(
      std::to_string(result.grids.low_support_obstacle_cells) +
      " isolated/low-support obstacle cells were rejected and remain UNKNOWN unless "
      "independently covered by the explicitly selected driven-footprint evidence");
  }
  if (result.grids.ground_unknown_points > 0U) {
    result.generation.warnings.push_back(
      std::to_string(result.grids.ground_unknown_points) +
      " map points could not be evaluated because no bounded local ground model was available");
  }
  if (config.traversability.minimum_obstacle_observations > 1U &&
    !result.grids.has_multi_scan_observation_support)
  {
    result.generation.warnings.push_back(
      "minimum_obstacle_observations exceeds 1, but the input point map has no retained "
      "multi-scan observation history; candidate obstacles cannot pass persistence filtering");
  }
  const std::string & free_mode = config.traversability.free_space_evidence_mode;
  const bool direct_ground_enabled =
    free_mode == "ground_observations" || free_mode == "combined";
  const bool trajectory_footprint_enabled =
    (free_mode == "trajectory" || free_mode == "combined") &&
    config.traversability.trajectory_free_space_model == "footprint";
  const bool trajectory_disk_enabled =
    (free_mode == "trajectory" || free_mode == "combined") &&
    config.traversability.trajectory_free_space_model == "disk" &&
    config.traversability.observed_trajectory_clearance_radius > 0.0;
  if (!direct_ground_enabled && !trajectory_footprint_enabled && !trajectory_disk_enabled) {
    result.generation.warnings.push_back(
      "no explicit free-space evidence is configured; completed point maps do not contain "
      "sensor origins/rays, so non-obstacle cells remain UNKNOWN");
  } else if (direct_ground_enabled && result.grids.ground_observation_free_cells == 0U) {
    result.generation.warnings.push_back(
      "direct-ground FREE evidence was enabled but no cell met its support threshold");
  }
  if (direct_ground_enabled) {
    result.generation.warnings.push_back(
      "direct ground returns and/or an eroded driven footprint provide experimental FREE "
      "evidence; interpolated ground and all remaining non-obstacle cells stay UNKNOWN");
  }
  if (config.traversability.unknown_space_policy == "allow") {
    result.generation.warnings.push_back(
      "UNKNOWN space is allowed by migration policy; clearance results are not fail-closed and "
      "must not be used as an operational safety claim");
  }

  // The processed base_link poses are the direct observation of the driven
  // route. Local floor estimates are independent traversability evidence and
  // must not inject discontinuous Z into Route/Lanelet centerlines.
  const std::vector<TimedPose> & route_trajectory = result.generation.processed_trajectory;
  if (result.grids.ground_z.size() != route_trajectory.size()) {
    throw std::runtime_error("floor-height result does not match the processed trajectory");
  }
  result.generation.observed_route_graph = buildObservedDrivenRouteGraph(
    route_trajectory,
    config.topology,
    dataset.world_frame,
    config.lanelet2.speed_limit_mps);
  result.generation.graph = buildRouteGraph(
    route_trajectory,
    config.topology,
    dataset.world_frame);
  result.generation.observed_route_geometry_audit = auditRouteGeometry(
    result.generation.observed_route_graph, &route_trajectory);
  requireValidGeometryAudit(
    result.generation.observed_route_geometry_audit, "observed replay");
  result.generation.topology_route_geometry_audit = auditRouteGeometry(
    result.generation.graph, nullptr);
  requireValidGeometryAudit(
    result.generation.topology_route_geometry_audit, "physical topology");
  result.generation.warnings.push_back(
    "Route/Lanelet centerline Z preserves the processed base_link trajectory; local floor "
    "estimates are traversability-only. Verify pose Z and T_base_sensor before operation");
  computeRouteClearance(
    result.generation.graph,
    result.grids.inflated_grid,
    result.grids.unknown_grid,
    config.traversability,
    config.lanelet2.speed_limit_mps);

  const auto physicalEdgeHasError = [&](const RouteEdge & edge, const std::string & error) {
      return (!edge.reverse_of || edge.id < *edge.reverse_of) &&
             std::find(edge.validation_errors.begin(), edge.validation_errors.end(), error) !=
             edge.validation_errors.end();
    };
  const std::size_t unknown_clearance_edges = static_cast<std::size_t>(std::count_if(
      result.generation.graph.edges.begin(), result.generation.graph.edges.end(),
      [&](const RouteEdge & edge) {return physicalEdgeHasError(edge, "unknown_clearance");}));
  const std::size_t passable_unknown_clearance_edges = static_cast<std::size_t>(std::count_if(
      result.generation.graph.edges.begin(), result.generation.graph.edges.end(),
      [&](const RouteEdge & edge) {
        return edge.passable && physicalEdgeHasError(edge, "unknown_clearance");
      }));
  const std::size_t invalid_corridor_edges = static_cast<std::size_t>(std::count_if(
      result.generation.graph.edges.begin(), result.generation.graph.edges.end(),
      [&](const RouteEdge & edge) {return
          (!edge.reverse_of || edge.id < *edge.reverse_of) && !edge.corridor_geometry_valid;}));
  if (unknown_clearance_edges > 0U) {
    result.generation.warnings.push_back(
      std::to_string(unknown_clearance_edges) +
      " physical route geometries reached an UNKNOWN boundary; " +
      std::to_string(passable_unknown_clearance_edges) +
      " remain warning-status candidates because the required width fits entirely inside "
      "the contiguous observed-free strip");
  }
  if (invalid_corridor_edges > 0U) {
    result.generation.warnings.push_back(
      std::to_string(invalid_corridor_edges) +
      " physical corridor polygons failed geometry validation; they were excluded from operational outputs");
  }
  GenerationStatistics & statistics = result.generation.statistics;
  statistics.route_nodes = result.generation.graph.nodes.size();
  statistics.route_edges = result.generation.graph.edges.size();
  statistics.trajectory_length = 0.0;
  for (std::size_t index = 1U; index < result.generation.processed_trajectory.size(); ++index) {
    statistics.trajectory_length += distance3d(
      result.generation.processed_trajectory[index - 1U].world_from_body.translation,
      result.generation.processed_trajectory[index].world_from_body.translation);
  }
  for (const RouteEdge & edge : result.generation.graph.edges) {
    if (!edge.passable) {
      ++statistics.impassable_edges;
    }
    const bool physical_representative = !edge.reverse_of || edge.id < *edge.reverse_of;
    if (physical_representative) {
      ++statistics.physical_route_edges;
      if (edge.passable) {
        ++statistics.passable_physical_edges;
      } else {
        ++statistics.impassable_physical_edges;
      }
    }
    statistics.minimum_safe_width = std::min(
      statistics.minimum_safe_width, edge.minimum_safe_width);
  }
  if (!std::isfinite(statistics.minimum_safe_width)) {
    statistics.minimum_safe_width = 0.0;
  }
  if (statistics.impassable_physical_edges > 0U) {
    result.generation.warnings.push_back(
      std::to_string(statistics.impassable_physical_edges) + " of " +
      std::to_string(statistics.physical_route_edges) +
      " physical route geometries failed the configured clearance check");
  }
  if (statistics.passable_physical_edges == 0U) {
    result.generation.warnings.push_back(
      "no physical route geometry passed clearance; operational Route Graph and Lanelet2 "
      "outputs contain no traversable edge");
  }
  if (result.grids.obstacle_points == 0U) {
    result.generation.warnings.push_back(
      "no obstacle points were classified; verify floor-height and obstacle-height parameters");
  }
  return result;
}

}  // namespace lidar_mobility_map_generator
