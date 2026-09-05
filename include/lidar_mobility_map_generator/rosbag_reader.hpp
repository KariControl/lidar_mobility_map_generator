#pragma once

#include "lidar_mobility_map_generator/config.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <string>

namespace lidar_mobility_map_generator
{

// Returns whether a direct dynamic-TF endpoint may carry the numeric pose
// selected by pose_reference_frame. In sensor-reference mode, base_frame is
// also accepted intentionally: some estimators label a LiDAR-origin pose as
// base_link. resolveRawPoses() warns when that explicit label override occurs.
[[nodiscard]] bool acceptsDirectTfPoseEndpoint(
  const std::string & frame_id,
  const RosbagInputConfig & input_config);

[[nodiscard]] MappingDataset readRosbagDataset(
  const RosbagInputConfig & input_config,
  const ExtrinsicsConfig & extrinsics,
  const MapBuilderConfig & map_builder_config);

}  // namespace lidar_mobility_map_generator
