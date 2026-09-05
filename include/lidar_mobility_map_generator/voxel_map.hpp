#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace lidar_mobility_map_generator
{

class VoxelMapAccumulator
{
public:
  explicit VoxelMapAccumulator(double voxel_size);

  void add(const Vec3 & point, double intensity, std::uint64_t observation_id);
  void add(const PointXYZI & point, std::uint64_t observation_id);
  // For completed map files that already store an aggregate support count.
  void addAggregated(const PointXYZI & point);

  [[nodiscard]] std::size_t voxelCount() const {return voxels_.size();}
  [[nodiscard]] std::vector<PointXYZI> points(std::size_t minimum_observations) const;

private:
  struct Key
  {
    std::int64_t x{0};
    std::int64_t y{0};
    std::int64_t z{0};

    bool operator==(const Key & rhs) const {return x == rhs.x && y == rhs.y && z == rhs.z;}
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const noexcept;
  };

  struct Accumulator
  {
    Vec3 position_sum{};
    double intensity_sum{0.0};
    std::size_t point_count{0U};
    std::size_t observation_count{0U};
    std::uint64_t last_observation_id{0U};
    bool has_observation{false};
  };

  [[nodiscard]] Key key(const Vec3 & point) const;

  double voxel_size_{0.1};
  std::unordered_map<Key, Accumulator, KeyHash> voxels_;
};

[[nodiscard]] std::vector<PointXYZI> voxelize(
  const std::vector<PointXYZI> & points,
  double voxel_size);

}  // namespace lidar_mobility_map_generator
