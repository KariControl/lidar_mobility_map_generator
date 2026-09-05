#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{

class OccupancyGrid2D
{
public:
  OccupancyGrid2D() = default;
  OccupancyGrid2D(
    double origin_x, double origin_y, double resolution,
    std::size_t width, std::size_t height);

  [[nodiscard]] double originX() const {return origin_x_;}
  [[nodiscard]] double originY() const {return origin_y_;}
  [[nodiscard]] double resolution() const {return resolution_;}
  [[nodiscard]] std::size_t width() const {return width_;}
  [[nodiscard]] std::size_t height() const {return height_;}
  [[nodiscard]] bool empty() const {return cells_.empty();}

  [[nodiscard]] bool containsCell(std::int64_t x, std::int64_t y) const;
  [[nodiscard]] std::optional<std::pair<std::int64_t, std::int64_t>> worldToCell(
    double x, double y) const;
  [[nodiscard]] Vec2 cellCenter(std::int64_t x, std::int64_t y) const;
  void setOccupied(std::int64_t x, std::int64_t y, bool occupied = true);
  std::size_t setDisk(double center_x, double center_y, double radius_m, bool occupied = true);
  std::size_t clearDisk(double center_x, double center_y, double radius_m);
  // Out-of-grid cells are occupied by definition.  This is the fail-closed
  // boundary rule used by collision and clearance checks.
  [[nodiscard]] bool isOccupied(std::int64_t x, std::int64_t y) const;
  [[nodiscard]] bool isOccupiedWorld(double x, double y) const;
  [[nodiscard]] std::size_t occupiedCellCount() const;
  [[nodiscard]] OccupancyGrid2D inflated(double radius_m) const;
  void savePgm(const std::filesystem::path & path) const;

private:
  [[nodiscard]] std::size_t index(std::int64_t x, std::int64_t y) const;

  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.1};
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::vector<std::uint8_t> cells_;
};

// Grid products are constructed independently in a few pipeline stages.  Use
// one comparison rule for readiness checks and writers so a map cannot pass a
// preflight check and then fail during serialization because of harmless
// floating-point round-off in its origin or resolution.
[[nodiscard]] bool hasMatchingGridGeometry(
  const OccupancyGrid2D & lhs, const OccupancyGrid2D & rhs);

// Save a Nav2 map_server-compatible trinary P5 PGM. The three input masks must
// have identical geometry. Obstacle, unknown, and explicitly observed-free
// cells are encoded as 0, 205, and 254 respectively; cells not classified by
// any mask are conservatively encoded as unknown. When fail_closed is true,
// every output cell is encoded as unknown.
void saveNav2TrinaryPgm(
  const std::filesystem::path & path,
  const OccupancyGrid2D & obstacle_grid,
  const OccupancyGrid2D & observed_free_grid,
  const OccupancyGrid2D & unknown_grid,
  bool fail_closed);

struct TraversabilityGridResult
{
  OccupancyGrid2D obstacle_grid;
  OccupancyGrid2D inflated_grid;
  // Set cells are backed by the configured explicit FREE evidence: direct
  // supported ground returns, a trajectory sweep, or their union.
  OccupancyGrid2D observed_free_grid;
  // Cells with neither a supported obstacle nor configured explicit FREE
  // evidence. A set cell means UNKNOWN, not OCCUPIED. An interpolated ground
  // plane by itself never manufactures FREE evidence.
  OccupancyGrid2D unknown_grid;
  std::vector<double> ground_z;
  std::vector<PointXYZI> classified_obstacle_points;
  std::size_t obstacle_points{0U};
  std::size_t obstacle_candidate_points{0U};
  std::size_t low_support_obstacle_points{0U};
  std::size_t low_support_obstacle_cells{0U};
  std::size_t ground_unknown_points{0U};
  std::size_t locally_modelled_ground_cells{0U};
  // FREE cells backed by one or more direct ground returns.  This excludes
  // cells that are free only because a local ground plane was interpolated.
  std::size_t ground_observation_free_cells{0U};
  std::size_t unknown_cells{0U};
  std::size_t trajectory_cleared_obstacle_cells{0U};
  std::size_t fallback_ground_samples{0U};
  bool has_multi_scan_observation_support{false};
  bool unknown_treated_as_occupied{false};
  bool orientation_aware_footprint{false};
};

[[nodiscard]] TraversabilityGridResult buildTraversabilityGrid(
  const std::vector<PointXYZI> & map_points,
  const std::vector<TimedPose> & trajectory,
  const std::vector<TimedPose> & observed_free_space_trajectory,
  const TraversabilityConfig & config,
  const RobotConfig & robot);

// Convenience overload for callers that do not maintain a separate observed
// trajectory. The pipeline uses the overload above so smoothing cannot create
// free-space evidence away from the measured path.
[[nodiscard]] TraversabilityGridResult buildTraversabilityGrid(
  const std::vector<PointXYZI> & map_points,
  const std::vector<TimedPose> & trajectory,
  const TraversabilityConfig & config,
  const RobotConfig & robot);

}  // namespace lidar_mobility_map_generator
