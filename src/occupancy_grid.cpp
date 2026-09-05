#include "lidar_mobility_map_generator/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

class NearestTrajectoryGrid
{
public:
  NearestTrajectoryGrid(
    const double origin_x, const double origin_y, const double resolution,
    const std::size_t width, const std::size_t height)
  : origin_x_(origin_x), origin_y_(origin_y), resolution_(resolution),
    width_(width), height_(height), nearest_(width * height, -1),
    distance_squared_(width * height, std::numeric_limits<float>::infinity())
  {
  }

  void rasterize(const std::vector<TimedPose> & trajectory, const double radius)
  {
    const std::int64_t cells = static_cast<std::int64_t>(std::ceil(radius / resolution_));
    const double radius_squared = radius * radius;
    for (std::size_t sample = 0U; sample < trajectory.size(); ++sample) {
      const Vec3 position = trajectory[sample].world_from_body.translation;
      const std::int64_t center_x = static_cast<std::int64_t>(
        std::floor((position.x - origin_x_) / resolution_));
      const std::int64_t center_y = static_cast<std::int64_t>(
        std::floor((position.y - origin_y_) / resolution_));
      for (std::int64_t dy = -cells; dy <= cells; ++dy) {
        for (std::int64_t dx = -cells; dx <= cells; ++dx) {
          const std::int64_t x = center_x + dx;
          const std::int64_t y = center_y + dy;
          if (!contains(x, y)) {
            continue;
          }
          const double world_x = origin_x_ + (static_cast<double>(x) + 0.5) * resolution_;
          const double world_y = origin_y_ + (static_cast<double>(y) + 0.5) * resolution_;
          const double distance_squared =
            (world_x - position.x) * (world_x - position.x) +
            (world_y - position.y) * (world_y - position.y);
          if (distance_squared > radius_squared) {
            continue;
          }
          const std::size_t flat = index(x, y);
          if (distance_squared < static_cast<double>(distance_squared_[flat])) {
            distance_squared_[flat] = static_cast<float>(distance_squared);
            nearest_[flat] = static_cast<std::int32_t>(sample);
          }
        }
      }
    }
  }

  [[nodiscard]] std::optional<std::size_t> lookup(const double world_x, const double world_y) const
  {
    const std::int64_t x = static_cast<std::int64_t>(
      std::floor((world_x - origin_x_) / resolution_));
    const std::int64_t y = static_cast<std::int64_t>(
      std::floor((world_y - origin_y_) / resolution_));
    if (!contains(x, y)) {
      return std::nullopt;
    }
    const std::int32_t sample = nearest_[index(x, y)];
    if (sample < 0) {
      return std::nullopt;
    }
    return static_cast<std::size_t>(sample);
  }

private:
  [[nodiscard]] bool contains(const std::int64_t x, const std::int64_t y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<std::int64_t>(width_) &&
           y < static_cast<std::int64_t>(height_);
  }

  [[nodiscard]] std::size_t index(const std::int64_t x, const std::int64_t y) const
  {
    return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
  }

  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.5};
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::vector<std::int32_t> nearest_;
  std::vector<float> distance_squared_;
};

class LocalGroundSurface
{
public:
  LocalGroundSurface(
    const double origin_x, const double origin_y, const double resolution,
    const std::size_t width, const std::size_t height)
  : origin_x_(origin_x), origin_y_(origin_y), resolution_(resolution),
    width_(width), height_(height), candidates_(width * height),
    heights_(width * height, std::numeric_limits<double>::quiet_NaN())
  {
  }

  void add(
    const double x, const double y, const float z,
    const std::uint32_t observation_weight)
  {
    const auto cell = worldToCell(x, y);
    if (cell) {
      candidates_[index(cell->first, cell->second)].push_back(
        {z, std::max<std::uint32_t>(1U, observation_weight)});
    }
  }

  void finalize(const double quantile, const std::size_t minimum_points)
  {
    for (std::size_t flat = 0U; flat < candidates_.size(); ++flat) {
      std::vector<WeightedHeight> & values = candidates_[flat];
      if (values.size() < minimum_points) {
        continue;
      }
      // A scan-integrated voxel represents all scans that observed it. Treating
      // every voxel as one vote lets many one-off vertically smeared returns
      // drag a low quantile below the persistent road surface, after which the
      // real road is misclassified as a solid obstacle sheet. Completed GLIM
      // maps have weight 1, so this is exactly the former unweighted quantile
      // for that input type.
      std::sort(
        values.begin(), values.end(),
        [](const WeightedHeight & lhs, const WeightedHeight & rhs) {
          return lhs.height < rhs.height;
        });
      std::uint64_t total_weight = 0U;
      for (const WeightedHeight & value : values) {
        total_weight += static_cast<std::uint64_t>(value.weight);
      }
      const long double rank = std::floor(
        static_cast<long double>(clamp(quantile, 0.0, 1.0)) *
        static_cast<long double>(total_weight - 1U));
      std::uint64_t cumulative_weight = 0U;
      for (const WeightedHeight & value : values) {
        cumulative_weight += static_cast<std::uint64_t>(value.weight);
        if (static_cast<long double>(cumulative_weight) > rank) {
          heights_[flat] = static_cast<double>(value.height);
          break;
        }
      }
      std::vector<WeightedHeight>().swap(values);
    }
  }

  [[nodiscard]] std::optional<double> estimate(
    const double world_x, const double world_y, const double radius,
    const std::size_t minimum_cells, const double maximum_slope,
    const double maximum_residual) const
  {
    const auto center = worldToCell(world_x, world_y);
    if (!center) {
      return std::nullopt;
    }
    const std::int64_t radius_cells = static_cast<std::int64_t>(std::ceil(radius / resolution_));
    const double radius_squared = radius * radius;
    struct GroundSample
    {
      double dx;
      double dy;
      double z;
      double weight;
    };
    std::vector<GroundSample> samples;
    samples.reserve(static_cast<std::size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1)));
    for (std::int64_t dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (std::int64_t dx = -radius_cells; dx <= radius_cells; ++dx) {
        const std::int64_t x = center->first + dx;
        const std::int64_t y = center->second + dy;
        if (!contains(x, y)) {
          continue;
        }
        const double z = heights_[index(x, y)];
        if (!std::isfinite(z)) {
          continue;
        }
        const double cell_x = origin_x_ + (static_cast<double>(x) + 0.5) * resolution_;
        const double cell_y = origin_y_ + (static_cast<double>(y) + 0.5) * resolution_;
        const double sample_dx = cell_x - world_x;
        const double sample_dy = cell_y - world_y;
        const double distance_squared = sample_dx * sample_dx + sample_dy * sample_dy;
        if (distance_squared > radius_squared) {
          continue;
        }
        samples.push_back({
          sample_dx, sample_dy, z,
          1.0 / (1.0 + distance_squared / std::max(radius_squared, 1.0e-9))});
      }
    }

    const double direct = heights_[index(center->first, center->second)];
    if (samples.size() < minimum_cells) {
      return std::isfinite(direct) ? std::optional<double>(direct) : std::nullopt;
    }

    // Weighted least-squares z = ax + by + c around the query point.
    double matrix[3][4]{};
    for (const GroundSample & sample : samples) {
      const double basis[3]{sample.dx, sample.dy, 1.0};
      for (std::size_t row = 0U; row < 3U; ++row) {
        for (std::size_t column = 0U; column < 3U; ++column) {
          matrix[row][column] += sample.weight * basis[row] * basis[column];
        }
        matrix[row][3] += sample.weight * basis[row] * sample.z;
      }
    }
    for (std::size_t pivot = 0U; pivot < 3U; ++pivot) {
      std::size_t best = pivot;
      for (std::size_t row = pivot + 1U; row < 3U; ++row) {
        if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot])) {
          best = row;
        }
      }
      if (std::abs(matrix[best][pivot]) < 1.0e-12) {
        return std::isfinite(direct) ? std::optional<double>(direct) : std::nullopt;
      }
      if (best != pivot) {
        for (std::size_t column = pivot; column < 4U; ++column) {
          std::swap(matrix[pivot][column], matrix[best][column]);
        }
      }
      const double divisor = matrix[pivot][pivot];
      for (std::size_t column = pivot; column < 4U; ++column) {
        matrix[pivot][column] /= divisor;
      }
      for (std::size_t row = 0U; row < 3U; ++row) {
        if (row == pivot) {
          continue;
        }
        const double factor = matrix[row][pivot];
        for (std::size_t column = pivot; column < 4U; ++column) {
          matrix[row][column] -= factor * matrix[pivot][column];
        }
      }
    }
    const double slope_x = matrix[0][3];
    const double slope_y = matrix[1][3];
    const double intercept = matrix[2][3];
    if (std::hypot(slope_x, slope_y) > maximum_slope) {
      return std::isfinite(direct) ? std::optional<double>(direct) : std::nullopt;
    }
    double weighted_squared_error = 0.0;
    double weight_sum = 0.0;
    for (const GroundSample & sample : samples) {
      const double residual =
        sample.z - (slope_x * sample.dx + slope_y * sample.dy + intercept);
      weighted_squared_error += sample.weight * residual * residual;
      weight_sum += sample.weight;
    }
    const double rms = std::sqrt(weighted_squared_error / std::max(weight_sum, 1.0e-12));
    if (rms > maximum_residual) {
      return std::isfinite(direct) ? std::optional<double>(direct) : std::nullopt;
    }
    return intercept;
  }

private:
  struct WeightedHeight
  {
    float height{0.0F};
    std::uint32_t weight{1U};
  };

  [[nodiscard]] bool contains(const std::int64_t x, const std::int64_t y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<std::int64_t>(width_) &&
           y < static_cast<std::int64_t>(height_);
  }

  [[nodiscard]] std::optional<std::pair<std::int64_t, std::int64_t>> worldToCell(
    const double x, const double y) const
  {
    const std::int64_t cell_x = static_cast<std::int64_t>(std::floor((x - origin_x_) / resolution_));
    const std::int64_t cell_y = static_cast<std::int64_t>(std::floor((y - origin_y_) / resolution_));
    if (!contains(cell_x, cell_y)) {
      return std::nullopt;
    }
    return std::make_pair(cell_x, cell_y);
  }

  [[nodiscard]] std::size_t index(const std::int64_t x, const std::int64_t y) const
  {
    return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
  }

  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.5};
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::vector<std::vector<WeightedHeight>> candidates_;
  std::vector<double> heights_;
};

std::vector<std::vector<std::pair<std::int64_t, std::int64_t>>> rectangleFootprintStencils(
  const OccupancyGrid2D & grid, const RobotConfig & robot, const std::size_t orientation_bins)
{
  std::vector<std::vector<std::pair<std::int64_t, std::int64_t>>> result(orientation_bins);
  const double half_bin_angle = kPi / static_cast<double>(orientation_bins);
  const double corner_radius = std::hypot(
    std::max(robot.front_extent, robot.rear_extent), 0.5 * robot.width);
  const double discretization_margin =
    0.5 * std::sqrt(2.0) * grid.resolution() + corner_radius * std::sin(half_bin_angle);
  const double front = robot.front_extent + robot.clearance_margin + discretization_margin;
  const double rear = robot.rear_extent + robot.clearance_margin + discretization_margin;
  const double side = 0.5 * robot.width + robot.clearance_margin + discretization_margin;
  const double radius = std::hypot(std::max(front, rear), side);
  const std::int64_t cells = static_cast<std::int64_t>(std::ceil(radius / grid.resolution()));
  for (std::size_t bin = 0U; bin < orientation_bins; ++bin) {
    const double yaw = 2.0 * kPi * static_cast<double>(bin) /
      static_cast<double>(orientation_bins);
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    std::vector<std::pair<std::int64_t, std::int64_t>> & stencil = result[bin];
    for (std::int64_t dy = -cells; dy <= cells; ++dy) {
      for (std::int64_t dx = -cells; dx <= cells; ++dx) {
        const double world_dx = static_cast<double>(dx) * grid.resolution();
        const double world_dy = static_cast<double>(dy) * grid.resolution();
        const double body_x = cosine * world_dx + sine * world_dy;
        const double body_y = -sine * world_dx + cosine * world_dy;
        if (body_x >= -rear && body_x <= front && std::abs(body_y) <= side) {
          stencil.emplace_back(dx, dy);
        }
      }
    }
  }
  return result;
}

OccupancyGrid2D inflateForPlatform(
  const OccupancyGrid2D & obstacles, const NearestTrajectoryGrid & nearest,
  const std::vector<TimedPose> & trajectory, const RobotConfig & robot,
  bool * orientation_aware)
{
  if (robot.footprint_model == "circle") {
    if (orientation_aware != nullptr) {
      *orientation_aware = false;
    }
    return obstacles.inflated(0.5 * robot.width + robot.clearance_margin);
  }
  if (robot.footprint_model != "rectangle") {
    throw std::invalid_argument("robot footprint_model must be circle or rectangle");
  }
  if (orientation_aware != nullptr) {
    *orientation_aware = true;
  }
  constexpr std::size_t kOrientationBins = 72U;
  const auto stencils = rectangleFootprintStencils(obstacles, robot, kOrientationBins);
  OccupancyGrid2D result(
    obstacles.originX(), obstacles.originY(), obstacles.resolution(),
    obstacles.width(), obstacles.height());
  for (std::int64_t y = 0; y < static_cast<std::int64_t>(obstacles.height()); ++y) {
    for (std::int64_t x = 0; x < static_cast<std::int64_t>(obstacles.width()); ++x) {
      const Vec2 center = obstacles.cellCenter(x, y);
      const auto sample = nearest.lookup(center.x, center.y);
      if (!sample) {
        continue;
      }
      const double yaw = trajectory[*sample].world_from_body.rotation.yaw();
      double normalized_yaw = std::fmod(yaw, 2.0 * kPi);
      if (normalized_yaw < 0.0) {
        normalized_yaw += 2.0 * kPi;
      }
      const std::size_t bin = static_cast<std::size_t>(std::llround(
          normalized_yaw * static_cast<double>(kOrientationBins) / (2.0 * kPi))) %
        kOrientationBins;
      for (const auto & offset : stencils[bin]) {
        if (obstacles.isOccupied(x + offset.first, y + offset.second)) {
          result.setOccupied(x, y);
          break;
        }
      }
    }
  }
  return result;
}

std::size_t setOrientedRectangle(
  OccupancyGrid2D & grid, const TimedPose & pose, const double front,
  const double rear, const double half_width, const bool occupied)
{
  const double yaw = pose.world_from_body.rotation.yaw();
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const Vec3 & position = pose.world_from_body.translation;
  const double radius = std::hypot(std::max(front, rear), half_width);
  const std::int64_t minimum_x = static_cast<std::int64_t>(std::floor(
      (position.x - radius - grid.originX()) / grid.resolution()));
  const std::int64_t maximum_x = static_cast<std::int64_t>(std::floor(
      (position.x + radius - grid.originX()) / grid.resolution()));
  const std::int64_t minimum_y = static_cast<std::int64_t>(std::floor(
      (position.y - radius - grid.originY()) / grid.resolution()));
  const std::int64_t maximum_y = static_cast<std::int64_t>(std::floor(
      (position.y + radius - grid.originY()) / grid.resolution()));
  std::size_t changed = 0U;
  for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
    for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
      if (!grid.containsCell(x, y)) {
        continue;
      }
      const Vec2 center = grid.cellCenter(x, y);
      const double dx = center.x - position.x;
      const double dy = center.y - position.y;
      const double local_x = cosine * dx + sine * dy;
      const double local_y = -sine * dx + cosine * dy;
      if (local_x < -rear || local_x > front || std::abs(local_y) > half_width) {
        continue;
      }
      if (grid.isOccupied(x, y) != occupied) {
        grid.setOccupied(x, y, occupied);
        ++changed;
      }
    }
  }
  return changed;
}

void applyTrajectoryFootprintFreeSpace(
  OccupancyGrid2D & obstacle_grid, OccupancyGrid2D & observed_free_grid,
  OccupancyGrid2D & unknown_grid, const std::vector<TimedPose> & trajectory,
  const TraversabilityConfig & config, const RobotConfig & robot,
  std::size_t & cleared_obstacle_cells)
{
  const double erosion = config.trajectory_footprint_erosion_margin;
  // Grid cells are represented by their centres. The half-cell-diagonal
  // margin keeps pose stamps consistent with later centre-sampled footprint
  // tests. Between poses, bound the *combined* translation and rotating-body
  // motion to half a cell. Bounding those components independently can leave
  // holes when both move the same body corner at once.
  const double raster_margin =
    0.5 * std::sqrt(2.0) * observed_free_grid.resolution();
  const double maximum_sweep_boundary_step =
    0.5 * observed_free_grid.resolution();
  std::vector<TimedPose> continuous_trajectory;
  continuous_trajectory.reserve(trajectory.size());
  if (!trajectory.empty()) {
    for (std::size_t index = 1U; index < trajectory.size(); ++index) {
      const TimedPose & start = trajectory[index - 1U];
      const TimedPose & end = trajectory[index];
      const double planar_distance = distance2d(
        start.world_from_body.translation, end.world_from_body.translation);
      const double start_yaw = start.world_from_body.rotation.yaw();
      const double yaw_delta = normalizeAngle(end.world_from_body.rotation.yaw() - start_yaw);
      const double body_radius = robot.footprint_model == "circle" ? 0.0 :
        std::hypot(std::max(robot.front_extent, robot.rear_extent), 0.5 * robot.width);
      const double maximum_body_boundary_motion =
        planar_distance + std::abs(yaw_delta) * body_radius;
      const std::size_t pieces = std::max<std::size_t>(
        1U, static_cast<std::size_t>(
          std::ceil(maximum_body_boundary_motion / maximum_sweep_boundary_step)));
      for (std::size_t piece = 0U; piece < pieces; ++piece) {
        const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
        TimedPose pose;
        pose.stamp_ns = start.stamp_ns + static_cast<std::int64_t>(std::llround(
          ratio * static_cast<double>(end.stamp_ns - start.stamp_ns)));
        pose.world_from_body.translation =
          start.world_from_body.translation +
          (end.world_from_body.translation - start.world_from_body.translation) * ratio;
        pose.world_from_body.rotation = Quaternion::fromYaw(
          normalizeAngle(start_yaw + ratio * yaw_delta));
        continuous_trajectory.push_back(std::move(pose));
      }
    }
    continuous_trajectory.push_back(trajectory.back());
  }
  if (robot.footprint_model == "circle") {
    const double radius = 0.5 * robot.width - erosion + raster_margin;
    for (const TimedPose & pose : continuous_trajectory) {
      observed_free_grid.setDisk(
        pose.world_from_body.translation.x, pose.world_from_body.translation.y, radius);
      unknown_grid.clearDisk(
        pose.world_from_body.translation.x, pose.world_from_body.translation.y, radius);
      cleared_obstacle_cells += obstacle_grid.clearDisk(
        pose.world_from_body.translation.x, pose.world_from_body.translation.y, radius);
    }
    return;
  }
  const double front = robot.front_extent - erosion + raster_margin;
  const double rear = robot.rear_extent - erosion + raster_margin;
  const double half_width = 0.5 * robot.width - erosion + raster_margin;
  for (const TimedPose & pose : continuous_trajectory) {
    setOrientedRectangle(observed_free_grid, pose, front, rear, half_width, true);
    setOrientedRectangle(unknown_grid, pose, front, rear, half_width, false);
    cleared_obstacle_cells += setOrientedRectangle(
      obstacle_grid, pose, front, rear, half_width, false);
  }
}

}  // namespace

OccupancyGrid2D::OccupancyGrid2D(
  const double origin_x, const double origin_y, const double resolution,
  const std::size_t width, const std::size_t height)
: origin_x_(origin_x), origin_y_(origin_y), resolution_(resolution),
  width_(width), height_(height), cells_(width * height, 0U)
{
  if (!(resolution_ > 0.0) || width_ == 0U || height_ == 0U) {
    throw std::invalid_argument("occupancy grid requires positive resolution and dimensions");
  }
}

bool OccupancyGrid2D::containsCell(const std::int64_t x, const std::int64_t y) const
{
  return x >= 0 && y >= 0 && x < static_cast<std::int64_t>(width_) &&
         y < static_cast<std::int64_t>(height_);
}

std::optional<std::pair<std::int64_t, std::int64_t>> OccupancyGrid2D::worldToCell(
  const double x, const double y) const
{
  const std::int64_t cell_x = static_cast<std::int64_t>(std::floor((x - origin_x_) / resolution_));
  const std::int64_t cell_y = static_cast<std::int64_t>(std::floor((y - origin_y_) / resolution_));
  if (!containsCell(cell_x, cell_y)) {
    return std::nullopt;
  }
  return std::make_pair(cell_x, cell_y);
}

Vec2 OccupancyGrid2D::cellCenter(const std::int64_t x, const std::int64_t y) const
{
  return {
    origin_x_ + (static_cast<double>(x) + 0.5) * resolution_,
    origin_y_ + (static_cast<double>(y) + 0.5) * resolution_};
}

std::size_t OccupancyGrid2D::index(const std::int64_t x, const std::int64_t y) const
{
  return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
}

void OccupancyGrid2D::setOccupied(
  const std::int64_t x, const std::int64_t y, const bool occupied)
{
  if (containsCell(x, y)) {
    cells_[index(x, y)] = occupied ? 1U : 0U;
  }
}

std::size_t OccupancyGrid2D::clearDisk(
  const double center_x, const double center_y, const double radius_m)
{
  return setDisk(center_x, center_y, radius_m, false);
}

std::size_t OccupancyGrid2D::setDisk(
  const double center_x, const double center_y, const double radius_m,
  const bool occupied)
{
  if (!(radius_m >= 0.0) || !std::isfinite(radius_m)) {
    throw std::invalid_argument("clear radius must be finite and nonnegative");
  }
  const auto center = worldToCell(center_x, center_y);
  if (!center) {
    return 0U;
  }
  const std::int64_t cells = static_cast<std::int64_t>(std::ceil(radius_m / resolution_));
  const double radius_squared = radius_m * radius_m;
  std::size_t changed_cells = 0U;
  for (std::int64_t dy = -cells; dy <= cells; ++dy) {
    for (std::int64_t dx = -cells; dx <= cells; ++dx) {
      const std::int64_t x = center->first + dx;
      const std::int64_t y = center->second + dy;
      if (!containsCell(x, y) || isOccupied(x, y) == occupied) {
        continue;
      }
      const Vec2 cell = cellCenter(x, y);
      const double offset_x = cell.x - center_x;
      const double offset_y = cell.y - center_y;
      if (offset_x * offset_x + offset_y * offset_y <= radius_squared + 1.0e-12) {
        setOccupied(x, y, occupied);
        ++changed_cells;
      }
    }
  }
  return changed_cells;
}

bool OccupancyGrid2D::isOccupied(const std::int64_t x, const std::int64_t y) const
{
  return !containsCell(x, y) || cells_[index(x, y)] != 0U;
}

bool OccupancyGrid2D::isOccupiedWorld(const double x, const double y) const
{
  const auto cell = worldToCell(x, y);
  return !cell || isOccupied(cell->first, cell->second);
}

std::size_t OccupancyGrid2D::occupiedCellCount() const
{
  return static_cast<std::size_t>(std::count(cells_.begin(), cells_.end(), static_cast<std::uint8_t>(1U)));
}

OccupancyGrid2D OccupancyGrid2D::inflated(const double radius_m) const
{
  OccupancyGrid2D result(origin_x_, origin_y_, resolution_, width_, height_);
  if (radius_m <= 0.0) {
    result.cells_ = cells_;
    return result;
  }
  const std::int64_t radius_cells = static_cast<std::int64_t>(std::ceil(radius_m / resolution_));
  const double radius_squared = radius_m * radius_m;
  std::vector<std::pair<std::int64_t, std::int64_t>> offsets;
  for (std::int64_t dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (std::int64_t dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double world_dx = static_cast<double>(dx) * resolution_;
      const double world_dy = static_cast<double>(dy) * resolution_;
      if (world_dx * world_dx + world_dy * world_dy <= radius_squared + 1.0e-12) {
        offsets.emplace_back(dx, dy);
      }
    }
  }

  for (std::int64_t y = 0; y < static_cast<std::int64_t>(height_); ++y) {
    for (std::int64_t x = 0; x < static_cast<std::int64_t>(width_); ++x) {
      if (!isOccupied(x, y)) {
        continue;
      }
      for (const auto & offset : offsets) {
        result.setOccupied(x + offset.first, y + offset.second);
      }
    }
  }
  return result;
}

void OccupancyGrid2D::savePgm(const std::filesystem::path & path) const
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to create occupancy image: " + path.string());
  }
  stream << "P5\n" << width_ << " " << height_ << "\n255\n";
  for (std::int64_t y = static_cast<std::int64_t>(height_) - 1; y >= 0; --y) {
    for (std::size_t x = 0U; x < width_; ++x) {
      const std::uint8_t value = cells_[index(static_cast<std::int64_t>(x), y)] != 0U ? 0U : 254U;
      stream.write(reinterpret_cast<const char *>(&value), 1);
    }
  }
}

bool hasMatchingGridGeometry(
  const OccupancyGrid2D & lhs, const OccupancyGrid2D & rhs)
{
  constexpr double kOriginTolerance = 1.0e-9;
  constexpr double kResolutionTolerance = 1.0e-12;
  return lhs.width() == rhs.width() && lhs.height() == rhs.height() &&
         std::abs(lhs.originX() - rhs.originX()) <= kOriginTolerance &&
         std::abs(lhs.originY() - rhs.originY()) <= kOriginTolerance &&
         std::abs(lhs.resolution() - rhs.resolution()) <= kResolutionTolerance;
}

void saveNav2TrinaryPgm(
  const std::filesystem::path & path,
  const OccupancyGrid2D & obstacle_grid,
  const OccupancyGrid2D & observed_free_grid,
  const OccupancyGrid2D & unknown_grid,
  bool fail_closed)
{
  if (!hasMatchingGridGeometry(obstacle_grid, observed_free_grid) ||
    !hasMatchingGridGeometry(obstacle_grid, unknown_grid))
  {
    throw std::invalid_argument("Nav2 trinary PGM input grids must have identical geometry");
  }
  if (obstacle_grid.empty()) {
    throw std::invalid_argument("Nav2 trinary PGM input grids must be non-empty");
  }

  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to create Nav2 trinary PGM: " + path.string());
  }
  stream << "P5\n" << obstacle_grid.width() << " " << obstacle_grid.height() << "\n255\n";

  constexpr std::uint8_t kObstacle = 0U;
  constexpr std::uint8_t kUnknown = 205U;
  constexpr std::uint8_t kFree = 254U;
  for (std::int64_t y = static_cast<std::int64_t>(obstacle_grid.height()) - 1; y >= 0; --y) {
    for (std::size_t x = 0U; x < obstacle_grid.width(); ++x) {
      std::uint8_t pixel = kUnknown;
      if (!fail_closed) {
        if (obstacle_grid.isOccupied(static_cast<std::int64_t>(x), y)) {
          pixel = kObstacle;
        } else if (unknown_grid.isOccupied(static_cast<std::int64_t>(x), y)) {
          pixel = kUnknown;
        } else if (observed_free_grid.isOccupied(static_cast<std::int64_t>(x), y)) {
          pixel = kFree;
        }
      }
      stream.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }
  if (!stream) {
    throw std::runtime_error("failed to write Nav2 trinary PGM: " + path.string());
  }
}

TraversabilityGridResult buildTraversabilityGrid(
  const std::vector<PointXYZI> & map_points,
  const std::vector<TimedPose> & trajectory,
  const std::vector<TimedPose> & observed_free_space_trajectory,
  const TraversabilityConfig & config,
  const RobotConfig & robot)
{
  if (trajectory.size() < 2U) {
    throw std::runtime_error("traversability grid requires at least two trajectory poses");
  }
  if (!(config.grid_resolution > 0.0) || !(config.trajectory_crop_radius > 0.0)) {
    throw std::invalid_argument("invalid traversability grid resolution or crop radius");
  }

  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const TimedPose & pose : trajectory) {
    const Vec3 position = pose.world_from_body.translation;
    minimum_x = std::min(minimum_x, position.x - config.trajectory_crop_radius);
    minimum_y = std::min(minimum_y, position.y - config.trajectory_crop_radius);
    maximum_x = std::max(maximum_x, position.x + config.trajectory_crop_radius);
    maximum_y = std::max(maximum_y, position.y + config.trajectory_crop_radius);
  }

  const std::size_t width = static_cast<std::size_t>(
    std::ceil((maximum_x - minimum_x) / config.grid_resolution)) + 1U;
  const std::size_t height = static_cast<std::size_t>(
    std::ceil((maximum_y - minimum_y) / config.grid_resolution)) + 1U;
  if (width > 0U && height > config.maximum_grid_cells / width) {
    throw std::runtime_error(
            "traversability grid exceeds maximum_grid_cells; increase resolution or reduce crop radius");
  }

  OccupancyGrid2D obstacle_grid(
    minimum_x, minimum_y, config.grid_resolution, width, height);

  const double assignment_resolution = std::max(
    config.grid_resolution, std::min(0.5, std::max(0.1, config.ground_estimation_radius * 0.5)));
  const std::size_t assignment_width = static_cast<std::size_t>(
    std::ceil((maximum_x - minimum_x) / assignment_resolution)) + 1U;
  const std::size_t assignment_height = static_cast<std::size_t>(
    std::ceil((maximum_y - minimum_y) / assignment_resolution)) + 1U;
  if (assignment_width > 0U && assignment_height > config.maximum_grid_cells / assignment_width) {
    throw std::runtime_error("trajectory assignment grid exceeds maximum_grid_cells");
  }
  NearestTrajectoryGrid nearest(
    minimum_x, minimum_y, assignment_resolution, assignment_width, assignment_height);
  nearest.rasterize(trajectory, config.trajectory_crop_radius);

  const std::size_t ground_width = static_cast<std::size_t>(
    std::ceil((maximum_x - minimum_x) / config.ground_cell_resolution)) + 1U;
  const std::size_t ground_height = static_cast<std::size_t>(
    std::ceil((maximum_y - minimum_y) / config.ground_cell_resolution)) + 1U;
  if (ground_width > 0U && ground_height > config.maximum_grid_cells / ground_width) {
    throw std::runtime_error("local ground surface exceeds maximum_grid_cells");
  }
  LocalGroundSurface ground_surface(
    minimum_x, minimum_y, config.ground_cell_resolution, ground_width, ground_height);
  for (const PointXYZI & point : map_points) {
    const auto sample = nearest.lookup(point.x, point.y);
    if (!sample) {
      continue;
    }
    const double relative_z = static_cast<double>(point.z) -
      trajectory[*sample].world_from_body.translation.z;
    if (relative_z >= config.ground_search_min_offset &&
      relative_z <= config.ground_search_max_offset)
    {
      ground_surface.add(
        point.x, point.y, point.z,
        std::max<std::uint32_t>(1U, point.observation_count));
    }
  }
  // Keep the pre-2.5D minimum_ground_points setting safety-effective as a
  // lower bound; existing configurations therefore cannot silently weaken
  // the new per-cell estimator.
  ground_surface.finalize(
    config.ground_quantile,
    std::max(config.minimum_ground_points, config.minimum_ground_points_per_cell));

  TraversabilityGridResult result;
  result.ground_z.resize(trajectory.size());
  for (std::size_t sample = 0U; sample < trajectory.size(); ++sample) {
    const Vec3 position = trajectory[sample].world_from_body.translation;
    const auto ground = ground_surface.estimate(
      position.x, position.y, config.ground_plane_radius,
      config.minimum_ground_cells_for_plane, config.maximum_ground_slope,
      config.maximum_ground_plane_residual);
    if (ground) {
      result.ground_z[sample] = *ground;
    } else {
      result.ground_z[sample] =
        position.z + config.fallback_ground_z_offset;
      ++result.fallback_ground_samples;
    }
  }

  const std::size_t cell_count = width * height;
  std::vector<double> local_ground(cell_count, std::numeric_limits<double>::quiet_NaN());
  for (std::int64_t y = 0; y < static_cast<std::int64_t>(height); ++y) {
    for (std::int64_t x = 0; x < static_cast<std::int64_t>(width); ++x) {
      const Vec2 center = obstacle_grid.cellCenter(x, y);
      if (!nearest.lookup(center.x, center.y)) {
        continue;
      }
      const auto ground = ground_surface.estimate(
        center.x, center.y, config.ground_plane_radius,
        config.minimum_ground_cells_for_plane, config.maximum_ground_slope,
        config.maximum_ground_plane_residual);
      if (ground) {
        local_ground[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] = *ground;
        ++result.locally_modelled_ground_cells;
      }
    }
  }

  struct ObstacleCandidate
  {
    std::size_t point_index{0U};
    std::size_t flat_cell{0U};
  };
  std::vector<ObstacleCandidate> obstacle_candidates;
  std::vector<std::uint32_t> candidate_counts(cell_count, 0U);
  std::vector<std::uint32_t> ground_return_counts(cell_count, 0U);
  std::vector<std::uint32_t> maximum_observations(cell_count, 0U);
  const double minimum_obstacle_height = std::max(
    config.minimum_obstacle_height, robot.minimum_collision_height);
  const double maximum_obstacle_height = std::min(
    config.maximum_obstacle_height, robot.maximum_collision_height);
  if (!(minimum_obstacle_height < maximum_obstacle_height)) {
    throw std::invalid_argument(
            "robot vertical collision envelope does not overlap traversability obstacle heights");
  }
  const double crop_radius_squared =
    config.trajectory_crop_radius * config.trajectory_crop_radius;
  for (std::size_t point_index = 0U; point_index < map_points.size(); ++point_index) {
    const PointXYZI & point = map_points[point_index];
    if (point.observation_count > 1U) {
      result.has_multi_scan_observation_support = true;
    }
    const auto sample = nearest.lookup(point.x, point.y);
    if (!sample) {
      continue;
    }
    const Vec3 trajectory_position = trajectory[*sample].world_from_body.translation;
    const double dx = static_cast<double>(point.x) - trajectory_position.x;
    const double dy = static_cast<double>(point.y) - trajectory_position.y;
    if (dx * dx + dy * dy > crop_radius_squared) {
      continue;
    }
    const auto cell = obstacle_grid.worldToCell(point.x, point.y);
    if (!cell) {
      continue;
    }
    const std::size_t flat = static_cast<std::size_t>(cell->second) * width +
      static_cast<std::size_t>(cell->first);
    if (!std::isfinite(local_ground[flat])) {
      ++result.ground_unknown_points;
      continue;
    }
    const double height_above_ground = static_cast<double>(point.z) - local_ground[flat];
    // A direct return on the locally fitted ground is positive free-space
    // evidence for this exact 2-D cell.  Merely having an interpolated ground
    // model is intentionally insufficient: completed point maps do not retain
    // the rays needed to declare every non-obstacle cell free.
    if (std::abs(height_above_ground) <= config.maximum_ground_free_height) {
      ground_return_counts[flat] = ground_return_counts[flat] ==
        std::numeric_limits<std::uint32_t>::max() ? ground_return_counts[flat] :
        ground_return_counts[flat] + 1U;
    }
    if (height_above_ground < minimum_obstacle_height ||
      height_above_ground > maximum_obstacle_height)
    {
      continue;
    }
    obstacle_candidates.push_back({point_index, flat});
    candidate_counts[flat] = candidate_counts[flat] ==
      std::numeric_limits<std::uint32_t>::max() ? candidate_counts[flat] : candidate_counts[flat] + 1U;
    maximum_observations[flat] = std::max(
      maximum_observations[flat], point.observation_count);
  }
  result.obstacle_candidate_points = obstacle_candidates.size();

  std::vector<std::uint8_t> supported_cells(cell_count, 0U);
  const std::int64_t support_radius = static_cast<std::int64_t>(
    config.obstacle_support_radius_cells);
  for (std::int64_t y = 0; y < static_cast<std::int64_t>(height); ++y) {
    for (std::int64_t x = 0; x < static_cast<std::int64_t>(width); ++x) {
      const std::size_t flat = static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
      if (candidate_counts[flat] == 0U) {
        continue;
      }
      std::size_t neighborhood_points = 0U;
      for (std::int64_t dy = -support_radius; dy <= support_radius; ++dy) {
        for (std::int64_t dx = -support_radius; dx <= support_radius; ++dx) {
          const std::int64_t neighbor_x = x + dx;
          const std::int64_t neighbor_y = y + dy;
          if (!obstacle_grid.containsCell(neighbor_x, neighbor_y)) {
            continue;
          }
          neighborhood_points += candidate_counts[
            static_cast<std::size_t>(neighbor_y) * width + static_cast<std::size_t>(neighbor_x)];
        }
      }
      const bool spatially_supported =
        candidate_counts[flat] >= config.minimum_obstacle_points_per_cell ||
        neighborhood_points >= config.minimum_obstacle_neighbor_points;
      const bool temporally_supported =
        maximum_observations[flat] >= config.minimum_obstacle_observations;
      if (spatially_supported && temporally_supported) {
        supported_cells[flat] = 1U;
        obstacle_grid.setOccupied(x, y);
      } else {
        ++result.low_support_obstacle_cells;
      }
    }
  }

  result.observed_free_grid = OccupancyGrid2D(
    minimum_x, minimum_y, config.grid_resolution, width, height);
  result.unknown_grid = OccupancyGrid2D(
    minimum_x, minimum_y, config.grid_resolution, width, height);
  const bool use_ground_observations =
    config.free_space_evidence_mode == "ground_observations" ||
    config.free_space_evidence_mode == "combined";
  const bool use_trajectory_sweep =
    config.free_space_evidence_mode == "trajectory" ||
    config.free_space_evidence_mode == "combined";
  for (std::int64_t y = 0; y < static_cast<std::int64_t>(height); ++y) {
    for (std::int64_t x = 0; x < static_cast<std::int64_t>(width); ++x) {
      const std::size_t flat = static_cast<std::size_t>(y) * width +
        static_cast<std::size_t>(x);
      result.unknown_grid.setOccupied(x, y);
      if (obstacle_grid.isOccupied(x, y)) {
        result.unknown_grid.setOccupied(x, y, false);
      } else if (candidate_counts[flat] > 0U && supported_cells[flat] == 0U) {
        // A return inside the collision-height band that lacks temporal or
        // spatial support is uncertainty, not proof of ground. Keep it UNKNOWN
        // instead of allowing a coincident ground return to promote the cell to
        // FREE. An explicitly selected driven-footprint sweep may still clear
        // it later, with that separate evidence recorded in the report.
      } else if (use_ground_observations &&
        ground_return_counts[flat] >= config.minimum_ground_free_points_per_cell)
      {
        result.observed_free_grid.setOccupied(x, y);
        result.unknown_grid.setOccupied(x, y, false);
        ++result.ground_observation_free_cells;
      }
    }
  }
  if (use_trajectory_sweep) {
    if (config.trajectory_free_space_model == "footprint") {
      applyTrajectoryFootprintFreeSpace(
        obstacle_grid, result.observed_free_grid, result.unknown_grid,
        observed_free_space_trajectory, config, robot,
        result.trajectory_cleared_obstacle_cells);
    } else if (config.observed_trajectory_clearance_radius > 0.0) {
      for (const TimedPose & pose : observed_free_space_trajectory) {
        result.observed_free_grid.setDisk(
          pose.world_from_body.translation.x,
          pose.world_from_body.translation.y,
          config.observed_trajectory_clearance_radius);
        result.unknown_grid.clearDisk(
          pose.world_from_body.translation.x,
          pose.world_from_body.translation.y,
          config.observed_trajectory_clearance_radius);
        result.trajectory_cleared_obstacle_cells += obstacle_grid.clearDisk(
          pose.world_from_body.translation.x,
          pose.world_from_body.translation.y,
          config.observed_trajectory_clearance_radius);
      }
    }
  }

  // Keep the debug point cloud exactly consistent with the post-clear raw grid.
  result.classified_obstacle_points.reserve(obstacle_candidates.size());
  for (const ObstacleCandidate & candidate : obstacle_candidates) {
    const std::int64_t x = static_cast<std::int64_t>(candidate.flat_cell % width);
    const std::int64_t y = static_cast<std::int64_t>(candidate.flat_cell / width);
    if (supported_cells[candidate.flat_cell] == 0U) {
      ++result.low_support_obstacle_points;
    } else if (obstacle_grid.isOccupied(x, y)) {
      result.classified_obstacle_points.push_back(map_points[candidate.point_index]);
    }
  }
  result.obstacle_points = result.classified_obstacle_points.size();
  result.unknown_cells = result.unknown_grid.occupiedCellCount();
  result.unknown_treated_as_occupied = config.unknown_space_policy == "occupied";
  result.obstacle_grid = std::move(obstacle_grid);
  result.inflated_grid = inflateForPlatform(
    result.obstacle_grid, nearest, trajectory, robot, &result.orientation_aware_footprint);
  return result;
}

TraversabilityGridResult buildTraversabilityGrid(
  const std::vector<PointXYZI> & map_points,
  const std::vector<TimedPose> & trajectory,
  const TraversabilityConfig & config,
  const RobotConfig & robot)
{
  return buildTraversabilityGrid(map_points, trajectory, trajectory, config, robot);
}

}  // namespace lidar_mobility_map_generator
