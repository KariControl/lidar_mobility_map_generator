#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace lidar_mobility_map_generator
{

// Records the only geometry-changing cleanup performed while deriving the
// processed centerline.  The input samples remain immutable and are exported
// separately as trajectory_raw.tum.
struct TrajectoryProcessingAudit
{
  bool raw_trajectory_preserved{true};
  std::size_t corrected_position_jitter_poses{0U};
  std::size_t corrected_position_jitter_runs{0U};
  double maximum_planar_position_correction_m{0.0};
  double planar_length_before_position_jitter_correction_m{0.0};
  double planar_length_after_position_jitter_correction_m{0.0};
};

class PoseBuffer
{
public:
  PoseBuffer() = default;
  explicit PoseBuffer(std::vector<TimedPose> poses);

  [[nodiscard]] bool empty() const {return poses_.empty();}
  [[nodiscard]] std::size_t size() const {return poses_.size();}
  [[nodiscard]] const std::vector<TimedPose> & poses() const {return poses_;}
  [[nodiscard]] std::optional<Transform> lookup(
    std::int64_t stamp_ns, double maximum_gap_sec) const;

private:
  std::vector<TimedPose> poses_;
};

[[nodiscard]] std::vector<TimedPose> normalizeTrajectory(
  const std::vector<TimedPose> & input);

// Convert T_world_sensor samples into T_world_base using T_base_sensor.
[[nodiscard]] std::vector<TimedPose> sensorPosesToBase(
  const std::vector<TimedPose> & sensor_poses,
  const Transform & base_from_sensor);

[[nodiscard]] std::vector<TimedPose> processTrajectory(
  const std::vector<TimedPose> & input,
  const TrajectoryConfig & config,
  std::vector<std::string> * warnings = nullptr,
  TrajectoryProcessingAudit * audit = nullptr);

}  // namespace lidar_mobility_map_generator
