#pragma once

#include "lidar_mobility_map_generator/config.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <filesystem>
#include <vector>

namespace lidar_mobility_map_generator
{

[[nodiscard]] std::vector<TimedPose> loadTumTrajectory(
  const std::filesystem::path & path);

[[nodiscard]] MappingDataset readGlimDataset(
  const GlimInputConfig & input_config,
  const ExtrinsicsConfig & extrinsics,
  const MapBuilderConfig & map_builder_config);

}  // namespace lidar_mobility_map_generator
