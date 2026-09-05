#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <filesystem>
#include <vector>

namespace lidar_mobility_map_generator
{

[[nodiscard]] std::vector<PointXYZI> loadPointCloudFile(const std::filesystem::path & path);
[[nodiscard]] std::vector<PointXYZI> loadPcdFile(const std::filesystem::path & path);
[[nodiscard]] std::vector<PointXYZI> loadPlyFile(const std::filesystem::path & path);
void savePcdBinary(const std::filesystem::path & path, const std::vector<PointXYZI> & points);

inline std::vector<PointXYZI> loadPointCloud(const std::filesystem::path & path)
{
  return loadPointCloudFile(path);
}

inline void saveBinaryPcd(
  const std::filesystem::path & path, const std::vector<PointXYZI> & points)
{
  savePcdBinary(path, points);
}

}  // namespace lidar_mobility_map_generator
