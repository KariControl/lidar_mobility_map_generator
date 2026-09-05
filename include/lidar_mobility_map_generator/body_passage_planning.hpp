#pragma once

#include "lidar_mobility_map_generator/occupancy_grid.hpp"
#include "lidar_mobility_map_generator/route_editor.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

struct BodyPassageCellEvidence
{
  std::int64_t x{0};
  std::int64_t y{0};
  Vec2 center{};
};

struct BodyPassageEdgeEvidence
{
  std::uint64_t edge_id{0U};
  double planar_length_m{0.0};
  std::size_t sample_count{0U};
  std::size_t obstacle_overlap_sample_count{0U};
  std::size_t obstacle_overlap_cell_count{0U};
  std::size_t unknown_overlap_sample_count{0U};
  std::size_t unknown_overlap_cell_count{0U};
  std::size_t outside_grid_sample_count{0U};
  std::optional<BodyPassageCellEvidence> first_obstacle_overlap;
  std::optional<BodyPassageCellEvidence> first_unknown_overlap;
  double maximum_curvature_inv_m{0.0};
  bool minimum_turning_radius_violation{false};
  bool hard_valid{false};
  std::vector<std::string> hard_errors;
  std::vector<std::string> warnings;
};

struct RouteBodyPassagePlanningReport
{
  std::string frame_id{"map"};
  std::string route_graph_fingerprint;
  double sample_step_m{0.10};
  RobotConfig robot;
  bool planning_body_passage_ready{false};
  std::size_t valid_edges{0U};
  std::size_t warning_edges{0U};
  std::size_t hard_invalid_edges{0U};
  std::size_t total_samples{0U};
  std::size_t obstacle_overlap_samples{0U};
  std::size_t obstacle_overlap_cells{0U};
  std::size_t unknown_overlap_samples{0U};
  std::size_t unknown_overlap_cells{0U};
  std::size_t outside_grid_samples{0U};
  std::map<std::string, std::vector<std::uint64_t>>
    additional_clearance_warning_edges;
  std::vector<BodyPassageEdgeEvidence> edges;
};

// Planning-test-only evidence for the exact chronological replay. The raw
// post-clear obstacle and UNKNOWN masks are checked against the physical body
// with zero additional clearance. This does not approve production use.
[[nodiscard]] RouteBodyPassagePlanningReport evaluateRouteBodyPassagePlanning(
  const RouteGraph & full_replay_route,
  const std::vector<TimedPose> & replay_body_poses,
  const OccupancyGrid2D & raw_obstacle_grid,
  const OccupancyGrid2D & raw_unknown_grid,
  const RobotConfig & robot,
  const RouteValidationResult & clearance_validation,
  double sample_step_m = 0.10);

void saveRouteBodyPassagePlanningReportJson(
  const std::filesystem::path & path,
  const RouteBodyPassagePlanningReport & report);

}  // namespace lidar_mobility_map_generator
