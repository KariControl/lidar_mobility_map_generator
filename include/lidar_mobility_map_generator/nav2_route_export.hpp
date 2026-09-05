#pragma once

#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <filesystem>

namespace lidar_mobility_map_generator
{

// Nav2's GeoJSON graph loader connects edge endpoint nodes and does not
// reconstruct intermediate MultiLineString vertices. This exporter therefore
// retains every source centerline vertex and linearly subdivides each original
// 3-D segment into an explicit node/edge chain. maximum_chord_error remains in
// the API for configuration compatibility but never authorizes simplification.
void saveNav2RouteGraphGeoJson(
  const std::filesystem::path & path,
  const RouteGraph & graph,
  double maximum_chord_error,
  double maximum_segment_length,
  const NamedNavigationRoute * named_route = nullptr);

}  // namespace lidar_mobility_map_generator
