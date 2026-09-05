#include "lidar_mobility_map_generator/glim_reader.hpp"

#include "lidar_mobility_map_generator/pointcloud_io.hpp"
#include "lidar_mobility_map_generator/trajectory.hpp"
#include "lidar_mobility_map_generator/voxel_map.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

std::vector<TimedPose> loadTumTrajectory(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open TUM trajectory: " + path.string());
  }

  std::vector<TimedPose> result;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(stream, line)) {
    ++line_number;
    const std::size_t first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::istringstream values(line);
    long double stamp = 0.0L;
    TimedPose pose;
    Quaternion quaternion;
    if (!(values >> stamp
      >> pose.world_from_body.translation.x
      >> pose.world_from_body.translation.y
      >> pose.world_from_body.translation.z
      >> quaternion.x >> quaternion.y >> quaternion.z >> quaternion.w))
    {
      throw std::runtime_error(
              "invalid TUM trajectory line " + std::to_string(line_number) +
              " in " + path.string());
    }
    if (!std::isfinite(stamp) || !finite(pose.world_from_body.translation) ||
      !quaternion.isFinite() || quaternion.squaredNorm() < 1.0e-15)
    {
      continue;
    }
    const long double stamp_ns = stamp * 1.0e9L;
    if (stamp_ns < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      stamp_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
    {
      continue;
    }
    pose.stamp_ns = static_cast<std::int64_t>(std::llround(stamp_ns));
    pose.world_from_body.rotation = quaternion.normalized();
    result.push_back(pose);
  }
  result = normalizeTrajectory(result);
  if (result.size() < 2U) {
    throw std::runtime_error("TUM trajectory contains fewer than two valid poses");
  }
  return result;
}

MappingDataset readGlimDataset(
  const GlimInputConfig & input_config,
  const ExtrinsicsConfig & extrinsics,
  const MapBuilderConfig & map_builder_config)
{
  if (input_config.map_path.empty() || input_config.trajectory_path.empty()) {
    throw std::runtime_error("GLIM input requires map_path and trajectory_path");
  }
  if (!(map_builder_config.voxel_size > 0.0) || map_builder_config.point_stride == 0U ||
    !std::isfinite(map_builder_config.minimum_z) ||
    !std::isfinite(map_builder_config.maximum_z) ||
    !(map_builder_config.minimum_z < map_builder_config.maximum_z))
  {
    throw std::runtime_error("invalid GLIM voxel size, point stride, or Z limits");
  }

  MappingDataset dataset;
  dataset.world_frame = input_config.world_frame;
  const std::vector<PointXYZI> loaded = loadPointCloud(input_config.map_path);
  dataset.statistics.decoded_points = loaded.size();
  std::vector<PointXYZI> filtered;
  filtered.reserve(
    (loaded.size() + map_builder_config.point_stride - 1U) /
    map_builder_config.point_stride);
  for (std::size_t index = 0U; index < loaded.size(); index += map_builder_config.point_stride) {
    const PointXYZI & point = loaded[index];
    if (point.z < map_builder_config.minimum_z || point.z > map_builder_config.maximum_z) {
      continue;
    }
    filtered.push_back(point);
  }
  dataset.map_points = voxelize(filtered, map_builder_config.voxel_size);
  dataset.statistics.accepted_points = filtered.size();
  dataset.statistics.map_voxels = dataset.map_points.size();
  dataset.statistics.pointcloud_messages = 1U;
  dataset.statistics.used_pointcloud_messages = 1U;

  dataset.trajectory = loadTumTrajectory(input_config.trajectory_path);
  dataset.statistics.pose_messages = dataset.trajectory.size();
  if (input_config.trajectory_frame == "sensor") {
    dataset.trajectory = sensorPosesToBase(dataset.trajectory, extrinsics.base_from_sensor);
  } else if (input_config.trajectory_frame != "base") {
    throw std::runtime_error("glim.trajectory_frame must be 'sensor' or 'base'");
  }

  if (filtered.size() != loaded.size()) {
    dataset.warnings.push_back(
      "GLIM point cloud filtering/stride accepted " + std::to_string(filtered.size()) +
      " of " + std::to_string(loaded.size()) + " points");
  }
  if (filtered.size() != dataset.map_points.size()) {
    dataset.warnings.push_back(
      "GLIM point cloud was voxelized from " + std::to_string(filtered.size()) + " to " +
      std::to_string(dataset.map_points.size()) + " points");
  }
  if (map_builder_config.scan_stride != 1U) {
    dataset.warnings.push_back(
      "map_builder.scan_stride has no effect for a completed GLIM map");
  }
  if (map_builder_config.minimum_observations_per_voxel != 1U) {
    dataset.warnings.push_back(
      "map_builder.minimum_observations_per_voxel has no observation history in a completed "
      "GLIM map and was treated as 1");
  }
  dataset.warnings.push_back(
    "map_builder.minimum_range and maximum_range cannot be applied to a completed GLIM map "
    "because per-point sensor origins are unavailable");
  return dataset;
}

}  // namespace lidar_mobility_map_generator
