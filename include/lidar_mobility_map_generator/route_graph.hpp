#pragma once

#include "lidar_mobility_map_generator/occupancy_grid.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

// Sampling-density-invariant curvature over a fixed arc-length baseline.
// Adjacent-vertex curvature grows artificially after clearance densification,
// even though the represented polyline has not changed.
[[nodiscard]] double maximumPolylineCurvature(
  const std::vector<Vec3> & points,
  double half_span = 0.50);

[[nodiscard]] RouteGraph buildRouteGraph(
  const std::vector<TimedPose> & processed_trajectory,
  const TopologyConfig & config,
  const std::string & frame_id);

void computeRouteClearance(
  RouteGraph & graph,
  const OccupancyGrid2D & inflated_obstacle_grid,
  const OccupancyGrid2D & unknown_grid,
  const TraversabilityConfig & config,
  double speed_limit_mps);

// Conservative compatibility overload.  Without an explicit UNKNOWN mask,
// empty obstacle-grid cells are not accepted as observed free.
void computeRouteClearance(
  RouteGraph & graph,
  const OccupancyGrid2D & inflated_obstacle_grid,
  const TraversabilityConfig & config,
  double speed_limit_mps);

}  // namespace lidar_mobility_map_generator
