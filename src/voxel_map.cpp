#include "lidar_mobility_map_generator/voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lidar_mobility_map_generator
{
namespace
{

std::uint64_t mix(std::uint64_t value)
{
  value ^= value >> 30U;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27U;
  value *= 0x94d049bb133111ebULL;
  value ^= value >> 31U;
  return value;
}

}  // namespace

VoxelMapAccumulator::VoxelMapAccumulator(const double voxel_size)
: voxel_size_(voxel_size)
{
  if (!(voxel_size_ > 0.0)) {
    throw std::invalid_argument("voxel size must be positive");
  }
}

std::size_t VoxelMapAccumulator::KeyHash::operator()(const Key & key_value) const noexcept
{
  const std::uint64_t x = mix(static_cast<std::uint64_t>(key_value.x));
  const std::uint64_t y = mix(static_cast<std::uint64_t>(key_value.y));
  const std::uint64_t z = mix(static_cast<std::uint64_t>(key_value.z));
  return static_cast<std::size_t>(x ^ (y << 1U) ^ (z << 7U));
}

VoxelMapAccumulator::Key VoxelMapAccumulator::key(const Vec3 & point) const
{
  return {
    static_cast<std::int64_t>(std::floor(point.x / voxel_size_)),
    static_cast<std::int64_t>(std::floor(point.y / voxel_size_)),
    static_cast<std::int64_t>(std::floor(point.z / voxel_size_))};
}

void VoxelMapAccumulator::add(
  const Vec3 & point, const double intensity, const std::uint64_t observation_id)
{
  if (!finite(point) || !std::isfinite(intensity)) {
    return;
  }
  Accumulator & accumulator = voxels_[key(point)];
  accumulator.position_sum += point;
  accumulator.intensity_sum += intensity;
  ++accumulator.point_count;
  if (!accumulator.has_observation || accumulator.last_observation_id != observation_id) {
    ++accumulator.observation_count;
    accumulator.last_observation_id = observation_id;
    accumulator.has_observation = true;
  }
}

void VoxelMapAccumulator::add(const PointXYZI & point, const std::uint64_t observation_id)
{
  add({point.x, point.y, point.z}, point.intensity, observation_id);
}

void VoxelMapAccumulator::addAggregated(const PointXYZI & point)
{
  if (!point.finite()) {
    return;
  }
  Accumulator & accumulator = voxels_[key(point.position())];
  accumulator.position_sum += point.position();
  accumulator.intensity_sum += point.intensity;
  ++accumulator.point_count;
  accumulator.observation_count = std::max<std::size_t>(
    accumulator.observation_count, std::max<std::uint32_t>(1U, point.observation_count));
  accumulator.has_observation = true;
}

std::vector<PointXYZI> VoxelMapAccumulator::points(const std::size_t minimum_observations) const
{
  std::vector<PointXYZI> result;
  result.reserve(voxels_.size());
  for (const auto & entry : voxels_) {
    const Accumulator & accumulator = entry.second;
    if (accumulator.point_count == 0U || accumulator.observation_count < minimum_observations) {
      continue;
    }
    const double count = static_cast<double>(accumulator.point_count);
    const Vec3 mean = accumulator.position_sum / count;
    result.push_back({
      static_cast<float>(mean.x),
      static_cast<float>(mean.y),
      static_cast<float>(mean.z),
      static_cast<float>(accumulator.intensity_sum / count),
      static_cast<std::uint32_t>(std::min<std::size_t>(
          accumulator.observation_count,
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())))});
  }
  return result;
}

std::vector<PointXYZI> voxelize(
  const std::vector<PointXYZI> & input,
  const double voxel_size)
{
  VoxelMapAccumulator accumulator(voxel_size);
  for (const PointXYZI & point : input) {
    accumulator.addAggregated(point);
  }
  return accumulator.points(1U);
}

}  // namespace lidar_mobility_map_generator
