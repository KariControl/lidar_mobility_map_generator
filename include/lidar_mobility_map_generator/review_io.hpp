#pragma once

#include "lidar_mobility_map_generator/occupancy_grid.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

struct Lanelet2ReviewLanelet
{
  std::uint64_t relation_id{0U};
  std::uint64_t route_edge_id{0U};
  bool passable{true};
  double confidence{1.0};
  double minimum_safe_width{0.0};
  std::vector<Vec3> left_boundary;
  std::vector<Vec3> right_boundary;
  std::vector<Vec3> centerline;
};

struct LoadedOccupancyGrid
{
  OccupancyGrid2D grid;
  std::filesystem::path image_path;
  // ROS occupancy values in grid order: 0 free, 100 occupied, -1 unknown.
  // Keeping this separately from the binary OccupancyGrid2D lets the review
  // node display Nav2 trinary maps without turning UNKNOWN pixels into free
  // space.
  std::vector<std::int8_t> occupancy_values;
};

// A lossless, implementation-owned representation used by the review node.
// The public GeoJSON and Lanelet2 outputs remain unchanged; this file keeps
// impassable edges and all 3-D center/boundary samples for human inspection.
void saveReviewGeometryTsv(
  const std::filesystem::path & path,
  const RouteGraph & graph);

[[nodiscard]] RouteGraph loadReviewGeometryTsv(
  const std::filesystem::path & path);

[[nodiscard]] LoadedOccupancyGrid loadOccupancyGridYaml(
  const std::filesystem::path & path);

// Parses the local-coordinate Lanelet2 OSM emitted by saveLanelet2Osm().
// It intentionally targets this generator's deterministic OSM representation,
// not arbitrary geographic OSM maps.
[[nodiscard]] std::vector<Lanelet2ReviewLanelet> loadGeneratedLanelet2Osm(
  const std::filesystem::path & path);

}  // namespace lidar_mobility_map_generator
