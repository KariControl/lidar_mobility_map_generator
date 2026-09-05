#include "lidar_mobility_map_generator/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lidar_mobility_map_generator
{
namespace
{

std::vector<TimedPose> removeStationary(
  const std::vector<TimedPose> & input, const double minimum_translation)
{
  if (input.empty()) {
    return {};
  }
  std::vector<TimedPose> result;
  result.reserve(input.size());
  result.push_back(input.front());
  for (std::size_t index = 1U; index < input.size(); ++index) {
    const bool last = index + 1U == input.size();
    if (last || distance3d(
        result.back().world_from_body.translation,
        input[index].world_from_body.translation) >= minimum_translation)
    {
      result.push_back(input[index]);
    }
  }
  return result;
}

std::vector<TimedPose> longestContinuousSegment(
  const std::vector<TimedPose> & input,
  const TrajectoryConfig & config,
  std::size_t * discarded_segments)
{
  if (input.size() < 2U) {
    return input;
  }

  std::vector<std::vector<TimedPose>> segments(1U);
  segments.back().push_back(input.front());
  for (std::size_t index = 1U; index < input.size(); ++index) {
    const TimedPose & previous = input[index - 1U];
    const TimedPose & current = input[index];
    const double dt = static_cast<double>(current.stamp_ns - previous.stamp_ns) * 1.0e-9;
    const double displacement = distance3d(
      current.world_from_body.translation, previous.world_from_body.translation);
    const double speed = dt > 1.0e-9 ? displacement / dt : std::numeric_limits<double>::infinity();
    const bool discontinuity =
      displacement > config.maximum_pose_jump || speed > config.maximum_speed_mps;
    if (discontinuity) {
      segments.emplace_back();
    }
    segments.back().push_back(current);
  }

  auto length = [](const std::vector<TimedPose> & poses) {
      double value = 0.0;
      for (std::size_t index = 1U; index < poses.size(); ++index) {
        value += distance3d(
          poses[index - 1U].world_from_body.translation,
          poses[index].world_from_body.translation);
      }
      return value;
    };

  std::size_t best = 0U;
  double best_length = -1.0;
  for (std::size_t index = 0U; index < segments.size(); ++index) {
    const double candidate = length(segments[index]);
    if (candidate > best_length ||
      (std::abs(candidate - best_length) < 1.0e-9 &&
      segments[index].size() > segments[best].size()))
    {
      best = index;
      best_length = candidate;
    }
  }
  if (discarded_segments != nullptr) {
    *discarded_segments = segments.size() > 0U ? segments.size() - 1U : 0U;
  }
  return segments[best];
}

std::vector<TimedPose> resample(
  const std::vector<TimedPose> & input, const double interval)
{
  if (input.size() < 2U) {
    return input;
  }

  std::vector<double> cumulative(input.size(), 0.0);
  for (std::size_t index = 1U; index < input.size(); ++index) {
    cumulative[index] = cumulative[index - 1U] + distance3d(
      input[index - 1U].world_from_body.translation,
      input[index].world_from_body.translation);
  }
  const double total = cumulative.back();
  if (total < interval) {
    return {input.front(), input.back()};
  }

  std::vector<TimedPose> result;
  result.reserve(static_cast<std::size_t>(std::ceil(total / interval)) + 1U);
  std::size_t right = 1U;
  for (double target = 0.0; target < total; target += interval) {
    while (right < cumulative.size() && cumulative[right] < target) {
      ++right;
    }
    if (right >= cumulative.size()) {
      break;
    }
    const std::size_t left = right - 1U;
    const double span = cumulative[right] - cumulative[left];
    const double ratio = span > 1.0e-12 ? (target - cumulative[left]) / span : 0.0;
    TimedPose pose;
    pose.stamp_ns = input[left].stamp_ns + static_cast<std::int64_t>(std::llround(
      ratio * static_cast<double>(input[right].stamp_ns - input[left].stamp_ns)));
    pose.world_from_body = Transform::interpolate(
      input[left].world_from_body, input[right].world_from_body, ratio);
    result.push_back(pose);
  }
  if (result.empty() || distance3d(
      result.back().world_from_body.translation,
      input.back().world_from_body.translation) > 1.0e-6)
  {
    result.push_back(input.back());
  }
  return result;
}

double planarTrajectoryLength(const std::vector<TimedPose> & poses)
{
  double result = 0.0;
  for (std::size_t index = 1U; index < poses.size(); ++index) {
    result += distance2d(
      poses[index - 1U].world_from_body.translation,
      poses[index].world_from_body.translation);
  }
  return result;
}

double planarHeading(const Vec3 & from, const Vec3 & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

// Forward and reverse motion are both aligned with the body longitudinal
// axis.  Using this undirected error is what prevents a genuine reverse
// traversal from being mistaken for localization jitter.
double bodyAxisHeadingError(const double path_heading, const double body_yaw)
{
  const double directed_error = std::abs(normalizeAngle(path_heading - body_yaw));
  return std::min(directed_error, kPi - directed_error);
}

void correctIsolatedPositionJitter(
  std::vector<TimedPose> & poses,
  TrajectoryProcessingAudit & audit)
{
  audit.planar_length_before_position_jitter_correction_m =
    planarTrajectoryLength(poses);
  audit.planar_length_after_position_jitter_correction_m =
    audit.planar_length_before_position_jitter_correction_m;
  if (poses.size() < 4U) {
    return;
  }

  // These fixed, deliberately conservative thresholds identify only a
  // two-sample lateral position excursion bracketed by a continuous travel
  // direction.  They are acquisition-quality checks, not vehicle or Autoware
  // tuning parameters.
  constexpr double maximum_jitter_span_m = 0.50;
  constexpr double minimum_double_turn_rad = 90.0 * kPi / 180.0;
  constexpr double maximum_outer_heading_change_rad = 45.0 * kPi / 180.0;
  constexpr double maximum_outer_body_axis_error_rad = 30.0 * kPi / 180.0;
  constexpr double minimum_jitter_body_axis_error_rad = 45.0 * kPi / 180.0;
  constexpr double maximum_pair_body_yaw_change_rad = 30.0 * kPi / 180.0;
  constexpr double minimum_segment_length_m = 1.0e-9;

  const std::vector<TimedPose> measured = poses;
  std::vector<bool> corrected(poses.size(), false);
  for (std::size_t first = 1U; first + 2U < measured.size(); ++first) {
    const std::size_t second = first + 1U;
    if (corrected[first - 1U] || corrected[first] || corrected[second] ||
      corrected[second + 1U])
    {
      continue;
    }

    const Vec3 & before = measured[first - 1U].world_from_body.translation;
    const Vec3 & first_position = measured[first].world_from_body.translation;
    const Vec3 & second_position = measured[second].world_from_body.translation;
    const Vec3 & after = measured[second + 1U].world_from_body.translation;
    const double incoming_length = distance2d(before, first_position);
    const double jitter_length = distance2d(first_position, second_position);
    const double outgoing_length = distance2d(second_position, after);
    if (incoming_length <= minimum_segment_length_m ||
      jitter_length <= minimum_segment_length_m ||
      outgoing_length <= minimum_segment_length_m ||
      jitter_length > maximum_jitter_span_m)
    {
      continue;
    }

    const double incoming_heading = planarHeading(before, first_position);
    const double jitter_heading = planarHeading(first_position, second_position);
    const double outgoing_heading = planarHeading(second_position, after);
    const double first_turn = std::abs(normalizeAngle(jitter_heading - incoming_heading));
    const double second_turn = std::abs(normalizeAngle(outgoing_heading - jitter_heading));
    const double outer_change = std::abs(normalizeAngle(outgoing_heading - incoming_heading));
    if (first_turn + 1.0e-12 < minimum_double_turn_rad ||
      second_turn + 1.0e-12 < minimum_double_turn_rad ||
      outer_change > maximum_outer_heading_change_rad)
    {
      continue;
    }

    const double first_yaw = measured[first].world_from_body.rotation.yaw();
    const double second_yaw = measured[second].world_from_body.rotation.yaw();
    if (bodyAxisHeadingError(incoming_heading, first_yaw) >
      maximum_outer_body_axis_error_rad ||
      bodyAxisHeadingError(outgoing_heading, second_yaw) >
      maximum_outer_body_axis_error_rad ||
      bodyAxisHeadingError(jitter_heading, first_yaw) <=
      minimum_jitter_body_axis_error_rad ||
      bodyAxisHeadingError(jitter_heading, second_yaw) <=
      minimum_jitter_body_axis_error_rad ||
      std::abs(normalizeAngle(second_yaw - first_yaw)) >
      maximum_pair_body_yaw_change_rad)
    {
      continue;
    }

    const std::int64_t span_ns =
      measured[second + 1U].stamp_ns - measured[first - 1U].stamp_ns;
    const auto interpolationRatio = [&](const std::size_t index, const double fallback) {
        if (span_ns <= 0) {
          return fallback;
        }
        return std::clamp(
          static_cast<double>(measured[index].stamp_ns - measured[first - 1U].stamp_ns) /
          static_cast<double>(span_ns), 0.0, 1.0);
      };
    const double first_ratio = interpolationRatio(first, 1.0 / 3.0);
    const double second_ratio = interpolationRatio(second, 2.0 / 3.0);
    const auto correctPosition = [&](const std::size_t index, const double ratio) {
        const Vec3 original = poses[index].world_from_body.translation;
        Vec3 replacement = original;
        replacement.x = before.x + ratio * (after.x - before.x);
        replacement.y = before.y + ratio * (after.y - before.y);
        poses[index].world_from_body.translation = replacement;
        audit.maximum_planar_position_correction_m = std::max(
          audit.maximum_planar_position_correction_m,
          distance2d(original, replacement));
      };
    correctPosition(first, first_ratio);
    correctPosition(second, second_ratio);
    corrected[first] = true;
    corrected[second] = true;
    audit.corrected_position_jitter_poses += 2U;
    ++audit.corrected_position_jitter_runs;
  }
  audit.planar_length_after_position_jitter_correction_m =
    planarTrajectoryLength(poses);
}

std::vector<bool> sharpTurnMask(const std::vector<TimedPose> & poses)
{
  std::vector<bool> result(poses.size(), false);
  constexpr double break_angle = 90.0 * kPi / 180.0;
  for (std::size_t index = 1U; index + 1U < poses.size(); ++index) {
    const Vec3 incoming = poses[index].world_from_body.translation -
      poses[index - 1U].world_from_body.translation;
    const Vec3 outgoing = poses[index + 1U].world_from_body.translation -
      poses[index].world_from_body.translation;
    if (std::hypot(incoming.x, incoming.y) < 1.0e-9 ||
      std::hypot(outgoing.x, outgoing.y) < 1.0e-9)
    {
      continue;
    }
    const double change = std::abs(normalizeAngle(
        std::atan2(outgoing.y, outgoing.x) - std::atan2(incoming.y, incoming.x)));
    result[index] = change >= break_angle;
  }
  return result;
}

void smoothPositions(
  std::vector<TimedPose> & poses,
  const double window,
  const double interval,
  const std::vector<bool> & sharp_turns)
{
  if (poses.size() < 3U || window <= 0.0 || interval <= 0.0) {
    return;
  }
  std::size_t radius = static_cast<std::size_t>(std::llround(window / (2.0 * interval)));
  radius = std::max<std::size_t>(1U, radius);
  radius = std::min(radius, (poses.size() - 1U) / 2U);
  if (radius == 0U) {
    return;
  }

  std::vector<Vec3> positions(poses.size());
  positions.front() = poses.front().world_from_body.translation;
  positions.back() = poses.back().world_from_body.translation;
  for (std::size_t index = 1U; index + 1U < poses.size(); ++index) {
    if (index < sharp_turns.size() && sharp_turns[index]) {
      positions[index] = poses[index].world_from_body.translation;
      continue;
    }
    std::size_t begin = index > radius ? index - radius : 0U;
    std::size_t end = std::min(poses.size() - 1U, index + radius);
    // Never average across a cusp or an in-place/sharp turn.  Both sides may
    // use the turn point as their endpoint, but samples from the opposite leg
    // cannot pull the path through the corner.
    for (std::size_t sample = begin; sample < index; ++sample) {
      if (sample < sharp_turns.size() && sharp_turns[sample]) {
        begin = sample;
      }
    }
    for (std::size_t sample = index + 1U; sample <= end; ++sample) {
      if (sample < sharp_turns.size() && sharp_turns[sample]) {
        end = sample;
        break;
      }
    }
    Vec3 sum{};
    for (std::size_t sample = begin; sample <= end; ++sample) {
      sum += poses[sample].world_from_body.translation;
    }
    positions[index] = sum / static_cast<double>(end - begin + 1U);
  }
  for (std::size_t index = 0U; index < poses.size(); ++index) {
    poses[index].world_from_body.translation = positions[index];
  }
}

}  // namespace

std::vector<TimedPose> normalizeTrajectory(const std::vector<TimedPose> & input)
{
  std::vector<TimedPose> result;
  result.reserve(input.size());
  for (const TimedPose & pose : input) {
    const Quaternion & rotation = pose.world_from_body.rotation;
    if (!finite(pose.world_from_body.translation) || !rotation.isFinite() ||
      rotation.squaredNorm() < 1.0e-15)
    {
      continue;
    }
    TimedPose normalized_pose = pose;
    normalized_pose.world_from_body.rotation =
      normalized_pose.world_from_body.rotation.normalized();
    result.push_back(normalized_pose);
  }
  // Keep input order for equal timestamps so duplicate resolution is deterministic.
  std::stable_sort(result.begin(), result.end(), [](const TimedPose & lhs, const TimedPose & rhs) {
      return lhs.stamp_ns < rhs.stamp_ns;
    });
  std::vector<TimedPose> deduplicated;
  deduplicated.reserve(result.size());
  for (const TimedPose & pose : result) {
    if (!deduplicated.empty() && deduplicated.back().stamp_ns == pose.stamp_ns) {
      deduplicated.back() = pose;
    } else {
      deduplicated.push_back(pose);
    }
  }
  return deduplicated;
}

std::vector<TimedPose> sensorPosesToBase(
  const std::vector<TimedPose> & sensor_poses,
  const Transform & base_from_sensor)
{
  const Transform sensor_from_base = base_from_sensor.inverse();
  std::vector<TimedPose> result = sensor_poses;
  for (TimedPose & pose : result) {
    pose.world_from_body = pose.world_from_body * sensor_from_base;
  }
  return result;
}

PoseBuffer::PoseBuffer(std::vector<TimedPose> poses)
: poses_(normalizeTrajectory(poses))
{
}

std::optional<Transform> PoseBuffer::lookup(
  const std::int64_t stamp_ns, const double maximum_gap_sec) const
{
  if (poses_.empty()) {
    return std::nullopt;
  }
  const auto right = std::lower_bound(
    poses_.begin(), poses_.end(), stamp_ns,
    [](const TimedPose & pose, const std::int64_t stamp) {return pose.stamp_ns < stamp;});

  if (right != poses_.end() && right->stamp_ns == stamp_ns) {
    return right->world_from_body;
  }
  if (right == poses_.begin() || right == poses_.end()) {
    return std::nullopt;
  }
  const TimedPose & upper = *right;
  const TimedPose & lower = *(right - 1);
  const double lower_gap = static_cast<double>(stamp_ns - lower.stamp_ns) * 1.0e-9;
  const double upper_gap = static_cast<double>(upper.stamp_ns - stamp_ns) * 1.0e-9;
  if (lower_gap < 0.0 || upper_gap < 0.0 ||
    lower_gap > maximum_gap_sec || upper_gap > maximum_gap_sec)
  {
    return std::nullopt;
  }
  const double span = static_cast<double>(upper.stamp_ns - lower.stamp_ns);
  if (span <= 0.0) {
    return std::nullopt;
  }
  const double ratio = static_cast<double>(stamp_ns - lower.stamp_ns) / span;
  return Transform::interpolate(lower.world_from_body, upper.world_from_body, ratio);
}

std::vector<TimedPose> processTrajectory(
  const std::vector<TimedPose> & input,
  const TrajectoryConfig & config,
  std::vector<std::string> * warnings,
  TrajectoryProcessingAudit * audit)
{
  TrajectoryProcessingAudit local_audit;
  TrajectoryProcessingAudit & processing_audit = audit == nullptr ? local_audit : *audit;
  processing_audit = TrajectoryProcessingAudit{};
  if (!(config.resample_interval > 0.0)) {
    throw std::invalid_argument("trajectory.resample_interval must be positive");
  }
  std::vector<TimedPose> normalized_input = normalizeTrajectory(input);
  if (normalized_input.size() < 2U) {
    throw std::runtime_error("trajectory contains fewer than two valid, time-ordered poses");
  }

  std::size_t discarded_segments = 0U;
  std::vector<TimedPose> continuous = longestContinuousSegment(
    normalized_input, config, &discarded_segments);
  if (discarded_segments > 0U && warnings != nullptr) {
    warnings->push_back(
      "trajectory contained discontinuities; only the longest continuous segment was used");
  }
  continuous = removeStationary(continuous, config.minimum_translation);
  if (continuous.size() < 2U) {
    throw std::runtime_error("trajectory contains fewer than two moving poses after filtering");
  }

  std::vector<TimedPose> result = resample(continuous, config.resample_interval);
  correctIsolatedPositionJitter(result, processing_audit);
  if (processing_audit.corrected_position_jitter_poses > 0U && warnings != nullptr) {
    warnings->push_back(
      "corrected " +
      std::to_string(processing_audit.corrected_position_jitter_poses) +
      " processed trajectory poses in " +
      std::to_string(processing_audit.corrected_position_jitter_runs) +
      " isolated position-jitter run(s) using body-yaw consistency "
      "(maximum planar correction " +
      std::to_string(processing_audit.maximum_planar_position_correction_m) +
      " m); the raw input trajectory was preserved");
  }
  const std::vector<bool> sharp_turns = sharpTurnMask(result);
  const std::size_t sharp_turn_count = static_cast<std::size_t>(std::count(
      sharp_turns.begin(), sharp_turns.end(), true));
  if (sharp_turn_count > 0U && warnings != nullptr) {
    warnings->push_back(
      "trajectory smoothing was segmented at " + std::to_string(sharp_turn_count) +
      " sharp turn/cusp samples");
  }
  smoothPositions(
    result, config.smoothing_window, config.resample_interval, sharp_turns);
  // Preserve the measured body orientation.  Route tangents are derived from
  // positions where needed; replacing body yaw with the path tangent loses
  // forward/reverse and in-place-turn semantics.
  return result;
}

}  // namespace lidar_mobility_map_generator
