#include "lidar_mobility_map_generator/route_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

// Autoware Core 1.9 removes consecutive path points whose 3-D separation is
// below 1 mm.  A topology Edge cut is an artificial sample, so place it on a
// nearby retained source vertex instead of creating a sub-millimetre segment.
// Only the cut moves; the source vertices, complete arc, Edge order, and
// endpoints remain present.
constexpr double kAutowareOverlappingPointDistanceM = 0.001;
constexpr double kTopologySplitSnapToleranceM =
  kAutowareOverlappingPointDistanceM + 1.0e-9;

class DisjointSet
{
public:
  explicit DisjointSet(const std::size_t size)
  : parent_(size), rank_(size, 0U), members_(size)
  {
    std::iota(parent_.begin(), parent_.end(), 0U);
    for (std::size_t index = 0U; index < size; ++index) {
      members_[index].push_back(index);
    }
  }

  std::size_t find(const std::size_t value)
  {
    if (parent_[value] != value) {
      parent_[value] = find(parent_[value]);
    }
    return parent_[value];
  }

  template<typename Compatible>
  bool uniteIfComplete(
    const std::size_t lhs_value, const std::size_t rhs_value,
    const Compatible & compatible)
  {
    std::size_t lhs = find(lhs_value);
    std::size_t rhs = find(rhs_value);
    if (lhs == rhs) {
      return true;
    }
    // A plain disjoint-set union implements single-link clustering: A can be
    // close to B and B close to C while A and C are arbitrarily far apart.
    // Route nodes are placed at the cluster centroid, so that transitive
    // growth can move a node away from every measured visit and can connect
    // otherwise distinct parallel traversals. Require every cross-cluster
    // pair to satisfy the same geometric/heading merge predicate instead.
    for (const std::size_t lhs_member : members_[lhs]) {
      for (const std::size_t rhs_member : members_[rhs]) {
        if (!compatible(lhs_member, rhs_member)) {
          return false;
        }
      }
    }
    if (rank_[lhs] < rank_[rhs]) {
      std::swap(lhs, rhs);
    }
    uniteRoots(lhs, rhs);
    return true;
  }

  template<typename Compatible>
  bool mergeIfClusters(
    const std::size_t lhs_value, const std::size_t rhs_value,
    const Compatible & compatible)
  {
    std::size_t lhs = find(lhs_value);
    std::size_t rhs = find(rhs_value);
    if (lhs == rhs || !compatible(members_[lhs], members_[rhs])) {
      return false;
    }
    if (rank_[lhs] < rank_[rhs]) {
      std::swap(lhs, rhs);
    }
    uniteRoots(lhs, rhs);
    return true;
  }

private:
  void uniteRoots(const std::size_t lhs, const std::size_t rhs)
  {
    parent_[rhs] = lhs;
    members_[lhs].insert(members_[lhs].end(), members_[rhs].begin(), members_[rhs].end());
    members_[rhs].clear();
    if (rank_[lhs] == rank_[rhs]) {
      ++rank_[lhs];
    }
  }

  std::vector<std::size_t> parent_;
  std::vector<std::uint8_t> rank_;
  std::vector<std::vector<std::size_t>> members_;
};

struct CellKey
{
  std::int64_t x{0};
  std::int64_t y{0};

  bool operator==(const CellKey & rhs) const {return x == rhs.x && y == rhs.y;}
};

struct CellHash
{
  std::size_t operator()(const CellKey & key) const noexcept
  {
    const std::uint64_t x = static_cast<std::uint64_t>(key.x) * 0x9e3779b185ebca87ULL;
    const std::uint64_t y = static_cast<std::uint64_t>(key.y) * 0xc2b2ae3d27d4eb4fULL;
    return static_cast<std::size_t>(x ^ (y + (x << 6U) + (x >> 2U)));
  }
};

struct BaseEdge
{
  std::size_t a{0U};
  std::size_t b{0U};
  std::size_t observed_from{0U};
  std::size_t observed_to{0U};
  Vec3 from_position{};
  Vec3 to_position{};
};

struct Chain
{
  std::size_t start_cluster{0U};
  std::size_t end_cluster{0U};
  std::vector<Vec3> points;
};

std::vector<double> cumulativeArc(const std::vector<TimedPose> & trajectory)
{
  std::vector<double> result(trajectory.size(), 0.0);
  for (std::size_t index = 1U; index < trajectory.size(); ++index) {
    result[index] = result[index - 1U] + distance2d(
      trajectory[index - 1U].world_from_body.translation,
      trajectory[index].world_from_body.translation);
  }
  return result;
}

double planarPolylineLength(const std::vector<Vec3> & points)
{
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result += distance2d(points[index - 1U], points[index]);
  }
  return result;
}

std::vector<double> trajectoryHeadings(const std::vector<TimedPose> & trajectory)
{
  std::vector<double> result(trajectory.size(), 0.0);
  for (std::size_t index = 0U; index < trajectory.size(); ++index) {
    Vec3 tangent{};
    if (index == 0U) {
      tangent = trajectory[1U].world_from_body.translation - trajectory[0U].world_from_body.translation;
    } else if (index + 1U == trajectory.size()) {
      tangent = trajectory[index].world_from_body.translation -
        trajectory[index - 1U].world_from_body.translation;
    } else {
      tangent = trajectory[index + 1U].world_from_body.translation -
        trajectory[index - 1U].world_from_body.translation;
    }
    result[index] = std::atan2(tangent.y, tangent.x);
  }
  return result;
}

double curvature(const Vec3 & a, const Vec3 & b, const Vec3 & c)
{
  const double ab = distance2d(a, b);
  const double bc = distance2d(b, c);
  const double ac = distance2d(a, c);
  const double denominator = ab * bc * ac;
  if (denominator < 1.0e-12) {
    return 0.0;
  }
  const double twice_area = std::abs(
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  return 2.0 * twice_area / denominator;
}

void simplifyConsecutiveDuplicates(std::vector<Vec3> & points)
{
  if (points.empty()) {
    return;
  }
  std::vector<Vec3> result;
  result.reserve(points.size());
  result.push_back(points.front());
  for (std::size_t index = 1U; index < points.size(); ++index) {
    if (distance3d(result.back(), points[index]) > 1.0e-6) {
      result.push_back(points[index]);
    }
  }
  points = std::move(result);
}

std::vector<Vec3> smoothTopologyPolyline(
  const std::vector<Vec3> & input, const double window)
{
  if (!(window > 0.0) || input.size() < 3U) {
    return input;
  }
  const bool closed = distance2d(input.front(), input.back()) <= 1.0e-6;
  const std::size_t sample_count = closed ? input.size() - 1U : input.size();
  if (sample_count < 3U) {
    return input;
  }
  std::vector<Vec3> current(input.begin(), input.begin() + sample_count);
  const double radius = 0.5 * window;
  for (std::size_t pass = 0U; pass < 2U; ++pass) {
    std::vector<double> arc(sample_count, 0.0);
    for (std::size_t index = 1U; index < sample_count; ++index) {
      arc[index] = arc[index - 1U] + distance2d(current[index - 1U], current[index]);
    }
    const double total = arc.back() +
      (closed ? distance2d(current.back(), current.front()) : 0.0);
    if (!(total > 1.0e-9)) {
      break;
    }
    std::vector<Vec3> next = current;
    for (std::size_t index = 0U; index < sample_count; ++index) {
      if (index == 0U || (!closed && index + 1U == sample_count)) {
        continue;
      }
      Vec3 weighted_sum{};
      double weight_sum = 0.0;
      for (std::size_t candidate = 0U; candidate < sample_count; ++candidate) {
        double separation = std::abs(arc[candidate] - arc[index]);
        if (closed) {
          separation = std::min(separation, total - separation);
        }
        if (separation > radius) {
          continue;
        }
        const double weight = 1.0 - separation / std::max(radius, 1.0e-12);
        weighted_sum += current[candidate] * weight;
        weight_sum += weight;
      }
      if (weight_sum > 1.0e-12) {
        next[index] = weighted_sum / weight_sum;
      }
    }
    current = std::move(next);
  }
  if (closed) {
    current.push_back(current.front());
  }
  return current;
}

std::vector<Vec3> densifyPolyline(
  const std::vector<Vec3> & points, const double maximum_xy_spacing)
{
  if (points.size() < 2U) {
    return points;
  }
  if (!(maximum_xy_spacing > 0.0) || !std::isfinite(maximum_xy_spacing)) {
    throw std::invalid_argument("polyline densification spacing must be finite and positive");
  }

  std::vector<Vec3> result;
  result.push_back(points.front());
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const Vec3 & start = points[index - 1U];
    const Vec3 & end = points[index];
    const double segment_length = distance2d(start, end);
    const std::size_t intervals = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(segment_length / maximum_xy_spacing)));
    for (std::size_t interval = 1U; interval < intervals; ++interval) {
      const double ratio = static_cast<double>(interval) / static_cast<double>(intervals);
      result.push_back(start + (end - start) * ratio);
    }
    // Preserve the supplied vertex exactly. In particular, this keeps edge
    // endpoints and the geometry shared by generated reverse edges unchanged.
    result.push_back(end);
  }
  return result;
}

struct GridTraceResult
{
  bool outside{false};
  bool occupied{false};
};

GridTraceResult traceSegmentCells(
  const OccupancyGrid2D & grid, const Vec3 & start, const Vec3 & end);

struct RaycastResult
{
  double distance{0.0};
  bool clearance_complete{false};
  bool map_boundary{false};
};

RaycastResult raycast(
  const OccupancyGrid2D & grid,
  const OccupancyGrid2D * unknown_grid,
  const Vec3 & origin,
  const Vec2 & direction,
  const TraversabilityConfig & config)
{
  // Bound segment length for predictable work, then supercover every segment;
  // finite point sampling alone can miss an arbitrarily short cell-corner hit.
  const double step = std::min(config.ray_step, grid.resolution() * 0.5);
  const std::size_t pieces = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(
      config.maximum_corridor_half_width / step)));
  Vec3 previous = origin;
  double previous_distance = 0.0;
  for (std::size_t piece = 1U; piece <= pieces; ++piece) {
    const double distance = config.maximum_corridor_half_width *
      static_cast<double>(piece) / static_cast<double>(pieces);
    const Vec3 current{
      origin.x + direction.x * distance,
      origin.y + direction.y * distance,
      origin.z};
    const GridTraceResult obstacle_trace = traceSegmentCells(grid, previous, current);
    if (obstacle_trace.outside) {
      // The binary grid has no observation mask.  Reaching its extent is not
      // evidence that any positive lateral distance is free, so collapse this
      // side to the centerline and let the caller fail closed.
      return {0.0, false, true};
    }
    const bool enforce_unknown = config.unknown_space_policy != "allow";
    const GridTraceResult unknown_trace = enforce_unknown && unknown_grid != nullptr ?
      traceSegmentCells(*unknown_grid, previous, current) : GridTraceResult{};
    const bool unknown = enforce_unknown &&
      (unknown_grid == nullptr || unknown_trace.outside || unknown_trace.occupied);
    if (unknown) {
      // Keep the diagnostic boundary at the end of the contiguous, explicitly
      // known-free run. The exact entry lies somewhere in this segment, so use
      // its already-verified start as a conservative lower bound.
      return {
        std::max(0.0, previous_distance - config.boundary_margin), false, false};
    }
    if (obstacle_trace.occupied) {
      return {
        std::max(0.0, previous_distance - config.boundary_margin), true, false};
    }
    previous = current;
    previous_distance = distance;
  }
  // Reaching the configured ray length means the full closed interval was
  // checked; it still says nothing about space beyond that configured limit.
  return {config.maximum_corridor_half_width, true, false};
}

Vec2 tangentAt(const std::vector<Vec3> & line, const std::size_t index)
{
  Vec2 tangent{};
  if (index == 0U) {
    tangent = {line[1U].x - line[0U].x, line[1U].y - line[0U].y};
  } else if (index + 1U == line.size()) {
    tangent = {
      line[index].x - line[index - 1U].x,
      line[index].y - line[index - 1U].y};
  } else {
    tangent = {
      line[index + 1U].x - line[index - 1U].x,
      line[index + 1U].y - line[index - 1U].y};
  }
  return normalized(tangent);
}

double cross2d(const Vec2 & lhs, const Vec2 & rhs)
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

double headingChangeAt(
  const std::vector<TimedPose> & trajectory, const std::size_t index)
{
  if (index == 0U || index + 1U >= trajectory.size()) {
    return 0.0;
  }
  const Vec3 incoming3 = trajectory[index].world_from_body.translation -
    trajectory[index - 1U].world_from_body.translation;
  const Vec3 outgoing3 = trajectory[index + 1U].world_from_body.translation -
    trajectory[index].world_from_body.translation;
  const Vec2 incoming{incoming3.x, incoming3.y};
  const Vec2 outgoing{outgoing3.x, outgoing3.y};
  if (norm(incoming) < 1.0e-9 || norm(outgoing) < 1.0e-9) {
    return 0.0;
  }
  return std::abs(normalizeAngle(
      std::atan2(outgoing.y, outgoing.x) - std::atan2(incoming.y, incoming.x)));
}

void addValidationError(RouteEdge & edge, const std::string & error)
{
  if (std::find(edge.validation_errors.begin(), edge.validation_errors.end(), error) ==
    edge.validation_errors.end())
  {
    edge.validation_errors.push_back(error);
  }
}

bool segmentIntersectsClosedInterval(
  const double start, const double delta,
  const double minimum, const double maximum,
  double & first, double & last)
{
  constexpr double tolerance = 1.0e-12;
  if (std::abs(delta) <= tolerance) {
    return start >= minimum - tolerance && start <= maximum + tolerance;
  }
  double enter = (minimum - start) / delta;
  double leave = (maximum - start) / delta;
  if (enter > leave) {
    std::swap(enter, leave);
  }
  first = std::max(first, enter);
  last = std::min(last, leave);
  return first <= last + tolerance;
}

bool segmentIntersectsClosedCell(
  const Vec3 & start, const Vec3 & end,
  const OccupancyGrid2D & grid,
  const std::int64_t cell_x, const std::int64_t cell_y)
{
  const double minimum_x = grid.originX() +
    static_cast<double>(cell_x) * grid.resolution();
  const double minimum_y = grid.originY() +
    static_cast<double>(cell_y) * grid.resolution();
  double first = 0.0;
  double last = 1.0;
  if (!segmentIntersectsClosedInterval(
      start.x, end.x - start.x,
      minimum_x, minimum_x + grid.resolution(), first, last))
  {
    return false;
  }
  return segmentIntersectsClosedInterval(
    start.y, end.y - start.y,
    minimum_y, minimum_y + grid.resolution(), first, last);
}

GridTraceResult traceSegmentCells(
  const OccupancyGrid2D & grid, const Vec3 & start, const Vec3 & end)
{
  GridTraceResult result;
  if (!finite(start) || !finite(end)) {
    result.outside = true;
    return result;
  }
  const auto start_cell = grid.worldToCell(start.x, start.y);
  const auto end_cell = grid.worldToCell(end.x, end.y);
  if (!start_cell || !end_cell) {
    result.outside = true;
    return result;
  }

  // Include one neighboring row/column so a centerline exactly on a cell
  // boundary conservatively checks both closed cells. The caller densifies to
  // at most half a cell, so this bounded supercover scan remains constant-size
  // while catching arbitrarily short corner crossings that point samples miss.
  const std::int64_t minimum_x =
    std::min(start_cell->first, end_cell->first) - 1;
  const std::int64_t maximum_x =
    std::max(start_cell->first, end_cell->first) + 1;
  const std::int64_t minimum_y =
    std::min(start_cell->second, end_cell->second) - 1;
  const std::int64_t maximum_y =
    std::max(start_cell->second, end_cell->second) + 1;
  for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
    for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
      if (!segmentIntersectsClosedCell(start, end, grid, x, y)) {
        continue;
      }
      if (!grid.containsCell(x, y)) {
        result.outside = true;
      } else if (grid.isOccupied(x, y)) {
        result.occupied = true;
      }
    }
  }
  return result;
}

GridTraceResult tracePolylineCells(
  const OccupancyGrid2D & grid, const std::vector<Vec3> & polyline)
{
  GridTraceResult result;
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    const GridTraceResult segment = traceSegmentCells(
      grid, polyline[index - 1U], polyline[index]);
    result.outside = result.outside || segment.outside;
    result.occupied = result.occupied || segment.occupied;
    if (result.outside && result.occupied) {
      break;
    }
  }
  return result;
}

std::vector<double> regularizeClearance(
  const std::vector<double> & raw,
  const std::vector<Vec3> & centerline,
  const double maximum_slope)
{
  if (raw.size() != centerline.size() || raw.size() < 2U) {
    return raw;
  }
  std::vector<double> result = raw;
  // Two one-sided passes form the greatest slope-limited profile that never
  // exceeds a raw clearance measurement.  Narrow observations therefore
  // produce a conservative taper instead of a saw tooth, while smoothing can
  // never create space that the raycast did not report.
  for (std::size_t index = 1U; index < result.size(); ++index) {
    const double allowed_increase = maximum_slope * distance2d(
      centerline[index - 1U], centerline[index]);
    result[index] = std::min(result[index], result[index - 1U] + allowed_increase);
  }
  for (std::size_t index = result.size() - 1U; index > 0U; --index) {
    const double allowed_increase = maximum_slope * distance2d(
      centerline[index - 1U], centerline[index]);
    result[index - 1U] = std::min(result[index - 1U], result[index] + allowed_increase);
  }
  return result;
}

int orientationSign(const Vec3 & a, const Vec3 & b, const Vec3 & c)
{
  const Vec2 ab{b.x - a.x, b.y - a.y};
  const Vec2 ac{c.x - a.x, c.y - a.y};
  const double value = cross2d(ab, ac);
  const double scale = std::max({1.0, norm(ab), norm(ac)});
  const double tolerance = 1.0e-10 * scale * scale;
  if (value > tolerance) {
    return 1;
  }
  if (value < -tolerance) {
    return -1;
  }
  return 0;
}

bool pointOnSegment(const Vec3 & a, const Vec3 & b, const Vec3 & point)
{
  if (orientationSign(a, b, point) != 0) {
    return false;
  }
  constexpr double tolerance = 1.0e-9;
  return point.x >= std::min(a.x, b.x) - tolerance &&
         point.x <= std::max(a.x, b.x) + tolerance &&
         point.y >= std::min(a.y, b.y) - tolerance &&
         point.y <= std::max(a.y, b.y) + tolerance;
}

bool segmentsIntersect(const Vec3 & a, const Vec3 & b, const Vec3 & c, const Vec3 & d)
{
  const int abc = orientationSign(a, b, c);
  const int abd = orientationSign(a, b, d);
  const int cda = orientationSign(c, d, a);
  const int cdb = orientationSign(c, d, b);
  if (abc != abd && cda != cdb) {
    return true;
  }
  return (abc == 0 && pointOnSegment(a, b, c)) ||
         (abd == 0 && pointOnSegment(a, b, d)) ||
         (cda == 0 && pointOnSegment(c, d, a)) ||
         (cdb == 0 && pointOnSegment(c, d, b));
}

bool polylineSelfIntersects(const std::vector<Vec3> & points, const bool closed)
{
  if ((!closed && points.size() < 4U) || (closed && points.size() < 3U)) {
    return false;
  }
  const std::size_t segment_count = closed ? points.size() : points.size() - 1U;
  for (std::size_t first = 0U; first < segment_count; ++first) {
    const std::size_t first_end = (first + 1U) % points.size();
    for (std::size_t second = first + 1U; second < segment_count; ++second) {
      const std::size_t second_end = (second + 1U) % points.size();
      const bool adjacent = first_end == second || second_end == first;
      const bool closure_adjacent = closed && first == 0U && second_end == 0U;
      if (adjacent || closure_adjacent) {
        continue;
      }
      if (segmentsIntersect(
          points[first], points[first_end], points[second], points[second_end]))
      {
        return true;
      }
    }
  }
  return false;
}

double signedPolygonArea(const std::vector<Vec3> & polygon)
{
  if (polygon.empty()) {
    return 0.0;
  }
  double twice_area = 0.0;
  const Vec3 origin = polygon.front();
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    const Vec3 & current = polygon[index];
    const Vec3 & next = polygon[(index + 1U) % polygon.size()];
    twice_area += (current.x - origin.x) * (next.y - origin.y) -
      (current.y - origin.y) * (next.x - origin.x);
  }
  return 0.5 * twice_area;
}

std::vector<Vec3> polylineSlice(
  const std::vector<Vec3> & points,
  const std::vector<double> & arc,
  const double begin,
  const double end)
{
  auto sample = [&](const double target) {
      const auto right = std::lower_bound(arc.begin(), arc.end(), target);
      if (right == arc.begin()) {
        return points.front();
      }
      if (right == arc.end()) {
        return points.back();
      }
      const std::size_t right_index = static_cast<std::size_t>(right - arc.begin());
      const std::size_t left_index = right_index - 1U;
      const double span = arc[right_index] - arc[left_index];
      const double ratio = span > 1.0e-12 ? (target - arc[left_index]) / span : 0.0;
      return points[left_index] + (points[right_index] - points[left_index]) * ratio;
    };

  std::vector<Vec3> result{sample(begin)};
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    if (arc[index] > begin + 1.0e-9 && arc[index] < end - 1.0e-9) {
      result.push_back(points[index]);
    }
  }
  result.push_back(sample(end));
  simplifyConsecutiveDuplicates(result);
  return result;
}

double snapTopologyPartitionArcToSourceVertex(
  const std::vector<double> & source_arc, const double requested)
{
  if (source_arc.size() < 3U || requested <= 0.0 || requested >= source_arc.back()) {
    return requested;
  }
  const auto right = std::lower_bound(source_arc.begin(), source_arc.end(), requested);
  double best = requested;
  double best_difference = kTopologySplitSnapToleranceM;
  const auto consider = [&](const std::vector<double>::const_iterator candidate) {
      if (candidate == source_arc.begin() || candidate == source_arc.end() ||
        std::next(candidate) == source_arc.end())
      {
        return;
      }
      const double difference = std::abs(*candidate - requested);
      if (difference <= best_difference) {
        best = *candidate;
        best_difference = difference;
      }
    };
  if (right != source_arc.end()) {
    consider(right);
  }
  if (right != source_arc.begin()) {
    consider(std::prev(right));
  }
  return best;
}

}  // namespace

double maximumPolylineCurvature(
  const std::vector<Vec3> & points,
  const double requested_half_span)
{
  if (points.size() < 3U || !(requested_half_span > 0.0) ||
    !std::isfinite(requested_half_span))
  {
    return 0.0;
  }
  std::vector<double> arc(points.size(), 0.0);
  for (std::size_t index = 1U; index < points.size(); ++index) {
    arc[index] = arc[index - 1U] + distance2d(points[index - 1U], points[index]);
  }
  const double total = arc.back();
  if (!(total > 1.0e-9)) {
    return 0.0;
  }
  const double half_span = std::min(requested_half_span, 0.5 * total);
  const auto sample = [&](const double target) {
      const auto right = std::lower_bound(arc.begin(), arc.end(), target);
      if (right == arc.begin()) {
        return points.front();
      }
      if (right == arc.end()) {
        return points.back();
      }
      const std::size_t right_index = static_cast<std::size_t>(right - arc.begin());
      const std::size_t left_index = right_index - 1U;
      const double length = arc[right_index] - arc[left_index];
      const double ratio = length > 1.0e-12 ?
        (target - arc[left_index]) / length : 0.0;
      return points[left_index] + (points[right_index] - points[left_index]) * ratio;
    };

  std::vector<double> targets;
  targets.reserve(points.size() + static_cast<std::size_t>(total / 0.10) + 1U);
  for (const double value : arc) {
    if (value >= half_span && value <= total - half_span) {
      targets.push_back(value);
    }
  }
  for (double value = half_span; value <= total - half_span + 1.0e-9; value += 0.10) {
    targets.push_back(std::min(value, total - half_span));
  }
  double result = 0.0;
  for (const double target : targets) {
    result = std::max(
      result,
      curvature(
        sample(target - half_span), sample(target), sample(target + half_span)));
  }
  return result;
}

RouteGraph buildRouteGraph(
  const std::vector<TimedPose> & trajectory,
  const TopologyConfig & config,
  const std::string & frame_id)
{
  if (trajectory.size() < 2U) {
    throw std::runtime_error("route graph requires at least two processed trajectory poses");
  }

  const std::vector<double> arc = cumulativeArc(trajectory);
  const std::vector<double> heading = trajectoryHeadings(trajectory);
  const double same_heading_threshold = config.same_path_heading_threshold_deg * kPi / 180.0;
  const double intersection_heading_threshold =
    config.intersection_heading_threshold_deg * kPi / 180.0;
  const double edge_split_heading_change =
    config.edge_split_heading_change_deg * kPi / 180.0;
  const double cusp_heading_change = config.cusp_heading_change_deg * kPi / 180.0;
  std::vector<bool> sharp_turn_sample(trajectory.size(), false);
  std::vector<bool> cusp_sample(trajectory.size(), false);
  for (std::size_t index = 1U; index + 1U < trajectory.size(); ++index) {
    const double change = headingChangeAt(trajectory, index);
    sharp_turn_sample[index] = change >= edge_split_heading_change;
    cusp_sample[index] = change >= cusp_heading_change;
  }
  const double cell_size = std::max(
    0.05, std::max(config.node_merge_distance, config.intersection_merge_distance));
  const std::int64_t neighbor_radius = 1;

  DisjointSet sets(trajectory.size());
  const auto same_path_pair = [&](const std::size_t lhs, const std::size_t rhs) {
      if (cusp_sample[lhs] || cusp_sample[rhs]) {
        return false;
      }
      const Vec3 & lhs_position = trajectory[lhs].world_from_body.translation;
      const Vec3 & rhs_position = trajectory[rhs].world_from_body.translation;
      return distance2d(lhs_position, rhs_position) <= config.node_merge_distance &&
             std::abs(lhs_position.z - rhs_position.z) <= config.node_merge_distance &&
             undirectedHeadingDifference(heading[lhs], heading[rhs]) <= same_heading_threshold;
    };
  const auto intersection_pair = [&](const std::size_t lhs, const std::size_t rhs) {
      if (cusp_sample[lhs] || cusp_sample[rhs]) {
        return false;
      }
      const Vec3 & lhs_position = trajectory[lhs].world_from_body.translation;
      const Vec3 & rhs_position = trajectory[rhs].world_from_body.translation;
      return distance2d(lhs_position, rhs_position) <= config.intersection_merge_distance &&
             std::abs(lhs_position.z - rhs_position.z) <=
             config.intersection_merge_distance &&
             undirectedHeadingDifference(heading[lhs], heading[rhs]) >=
             intersection_heading_threshold;
    };
  const auto merge_compatible = [&](const std::size_t lhs, const std::size_t rhs) {
      return same_path_pair(lhs, rhs) || intersection_pair(lhs, rhs);
    };
  std::unordered_map<CellKey, std::vector<std::size_t>, CellHash> spatial;
  for (std::size_t index = 0U; index < trajectory.size(); ++index) {
    const Vec3 position = trajectory[index].world_from_body.translation;
    const CellKey cell{
      static_cast<std::int64_t>(std::floor(position.x / cell_size)),
      static_cast<std::int64_t>(std::floor(position.y / cell_size))};
    for (std::int64_t dy = -neighbor_radius; dy <= neighbor_radius; ++dy) {
      for (std::int64_t dx = -neighbor_radius; dx <= neighbor_radius; ++dx) {
        const auto found = spatial.find({cell.x + dx, cell.y + dy});
        if (found == spatial.end()) {
          continue;
        }
        for (const std::size_t other : found->second) {
          if (std::abs(arc[index] - arc[other]) < config.minimum_loop_separation) {
            continue;
          }
          // A reversal point must remain a unique topology anchor.  Merging it
          // into a remote visit is a common source of tiny self-loop edges.
          if (same_path_pair(index, other) || intersection_pair(index, other)) {
            sets.uniteIfComplete(index, other, merge_compatible);
          }
        }
      }
    }
    spatial[cell].push_back(index);
  }

  // Complete-link clustering deliberately prevents a chain of nearby samples
  // from absorbing parallel traversals.  At a real crossing, however, dense
  // sampling can produce two adjacent mixed-heading clusters: each contains
  // independent visits through the intersection, but their outer samples are
  // slightly farther apart than intersection_merge_distance. Consolidate only
  // this bounded, evidence-rich case, including reciprocal A->B/B->A
  // transitions on remote visits. Single-heading parallel routes, sharp
  // turns/cusps, and vertically separated crossings cannot satisfy it.
  const auto cluster_centroid = [&](const std::vector<std::size_t> & members) {
      Vec3 sum{};
      for (const std::size_t member : members) {
        sum += trajectory[member].world_from_body.translation;
      }
      return sum / static_cast<double>(members.size());
    };
  const auto cluster_has_intersection_evidence =
    [&](const std::vector<std::size_t> & members) {
      for (std::size_t lhs = 0U; lhs < members.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1U; rhs < members.size(); ++rhs) {
          if (std::abs(arc[members[lhs]] - arc[members[rhs]]) >=
            config.minimum_loop_separation &&
            intersection_pair(members[lhs], members[rhs]))
          {
            return true;
          }
        }
      }
      return false;
    };
  const auto cluster_has_sharp_or_cusp_sample =
    [&](const std::vector<std::size_t> & members) {
      return std::any_of(
        members.begin(), members.end(), [&](const std::size_t member) {
          return sharp_turn_sample[member] || cusp_sample[member];
        });
    };
  const auto has_reciprocal_remote_transitions =
    [&](const std::vector<std::size_t> & lhs, const std::vector<std::size_t> & rhs) {
      struct ClusterTransition
      {
        double arc_position{0.0};
        double heading{0.0};
      };
      const auto collect_transitions =
        [&](const std::vector<std::size_t> & from_members,
        const std::vector<std::size_t> & to_members)
        {
          std::vector<ClusterTransition> result;
          for (const std::size_t from : from_members) {
            const std::size_t to = from + 1U;
            if (to >= trajectory.size() ||
              std::find(to_members.begin(), to_members.end(), to) == to_members.end())
            {
              continue;
            }
            const Vec3 delta = trajectory[to].world_from_body.translation -
              trajectory[from].world_from_body.translation;
            if (std::hypot(delta.x, delta.y) <= 1.0e-9) {
              continue;
            }
            result.push_back({
              0.5 * (arc[from] + arc[to]), std::atan2(delta.y, delta.x)});
          }
          return result;
        };
      const std::vector<ClusterTransition> lhs_to_rhs = collect_transitions(lhs, rhs);
      const std::vector<ClusterTransition> rhs_to_lhs = collect_transitions(rhs, lhs);
      for (const ClusterTransition & forward : lhs_to_rhs) {
        for (const ClusterTransition & reverse : rhs_to_lhs) {
          if (std::abs(forward.arc_position - reverse.arc_position) >=
            config.minimum_loop_separation &&
            undirectedHeadingDifference(forward.heading, reverse.heading) >=
            intersection_heading_threshold)
          {
            return true;
          }
        }
      }
      return false;
    };
  const auto bounded_intersection_fragments =
    [&](const std::vector<std::size_t> & lhs, const std::vector<std::size_t> & rhs) {
      const Vec3 lhs_centroid = cluster_centroid(lhs);
      const Vec3 rhs_centroid = cluster_centroid(rhs);
      if (distance2d(lhs_centroid, rhs_centroid) >
        config.intersection_merge_distance + 1.0e-9 ||
        std::abs(lhs_centroid.z - rhs_centroid.z) >
        config.intersection_merge_distance + 1.0e-9 ||
        cluster_has_sharp_or_cusp_sample(lhs) ||
        cluster_has_sharp_or_cusp_sample(rhs) ||
        !cluster_has_intersection_evidence(lhs) ||
        !cluster_has_intersection_evidence(rhs) ||
        !has_reciprocal_remote_transitions(lhs, rhs))
      {
        return false;
      }
      bool cross_pair_evidence = false;
      const double maximum_cluster_diameter = 2.0 * config.intersection_merge_distance;
      for (const std::size_t lhs_member : lhs) {
        for (const std::size_t rhs_member : rhs) {
          const Vec3 & lhs_position = trajectory[lhs_member].world_from_body.translation;
          const Vec3 & rhs_position = trajectory[rhs_member].world_from_body.translation;
          if (distance2d(lhs_position, rhs_position) > maximum_cluster_diameter + 1.0e-9 ||
            std::abs(lhs_position.z - rhs_position.z) >
            config.intersection_merge_distance + 1.0e-9)
          {
            return false;
          }
          cross_pair_evidence = cross_pair_evidence ||
            intersection_pair(lhs_member, rhs_member);
        }
      }
      return cross_pair_evidence;
    };
  bool merged_intersection_fragment = true;
  while (merged_intersection_fragment) {
    merged_intersection_fragment = false;
    std::vector<std::size_t> roots;
    roots.reserve(trajectory.size());
    for (std::size_t index = 0U; index < trajectory.size(); ++index) {
      if (sets.find(index) == index) {
        roots.push_back(index);
      }
    }
    for (std::size_t lhs = 0U;
      lhs < roots.size() && !merged_intersection_fragment; ++lhs)
    {
      for (std::size_t rhs = lhs + 1U; rhs < roots.size(); ++rhs) {
        if (sets.mergeIfClusters(
            roots[lhs], roots[rhs], bounded_intersection_fragments))
        {
          merged_intersection_fragment = true;
          break;
        }
      }
    }
  }

  std::unordered_map<std::size_t, std::size_t> root_to_cluster;
  std::vector<std::size_t> sample_cluster(trajectory.size(), 0U);
  std::vector<Vec3> cluster_sum;
  std::vector<std::size_t> cluster_count;
  for (std::size_t index = 0U; index < trajectory.size(); ++index) {
    const std::size_t root = sets.find(index);
    auto insertion = root_to_cluster.emplace(root, root_to_cluster.size());
    const std::size_t cluster = insertion.first->second;
    if (insertion.second) {
      cluster_sum.push_back({});
      cluster_count.push_back(0U);
    }
    sample_cluster[index] = cluster;
    cluster_sum[cluster] += trajectory[index].world_from_body.translation;
    ++cluster_count[cluster];
  }

  std::vector<Vec3> cluster_position(cluster_sum.size());
  for (std::size_t cluster = 0U; cluster < cluster_sum.size(); ++cluster) {
    cluster_position[cluster] = cluster_sum[cluster] / static_cast<double>(cluster_count[cluster]);
  }

  std::map<std::pair<std::size_t, std::size_t>, std::size_t> unique_edges;
  std::vector<BaseEdge> base_edges;
  for (std::size_t index = 1U; index < trajectory.size(); ++index) {
    const std::size_t from = sample_cluster[index - 1U];
    const std::size_t to = sample_cluster[index];
    if (from == to) {
      continue;
    }
    const auto key = std::minmax(from, to);
    if (unique_edges.find(key) != unique_edges.end()) {
      continue;
    }
    unique_edges[key] = base_edges.size();
    base_edges.push_back({
      key.first, key.second, from, to,
      trajectory[index - 1U].world_from_body.translation,
      trajectory[index].world_from_body.translation});
  }
  if (base_edges.empty()) {
    throw std::runtime_error("route topology collapsed to zero edges");
  }

  std::vector<std::vector<std::size_t>> adjacency(cluster_position.size());
  for (std::size_t edge = 0U; edge < base_edges.size(); ++edge) {
    adjacency[base_edges[edge].a].push_back(edge);
    adjacency[base_edges[edge].b].push_back(edge);
  }
  std::vector<bool> anchor(cluster_position.size(), false);
  for (std::size_t cluster = 0U; cluster < adjacency.size(); ++cluster) {
    anchor[cluster] = adjacency[cluster].size() != 2U;
  }
  for (std::size_t index = 1U; index + 1U < trajectory.size(); ++index) {
    if (sharp_turn_sample[index] || cusp_sample[index]) {
      // Keeping a sharp turn inside one degree-two chain makes its left/right
      // normal discontinuous.  Make it an explicit edge endpoint instead.
      anchor[sample_cluster[index]] = true;
    }
  }
  anchor[sample_cluster.front()] = true;
  anchor[sample_cluster.back()] = true;
  if (std::none_of(anchor.begin(), anchor.end(), [](const bool value) {return value;})) {
    anchor[sample_cluster.front()] = true;
  }

  std::vector<bool> visited(base_edges.size(), false);
  std::vector<Chain> chains;
  auto walk = [&](const std::size_t start_cluster, const std::size_t start_edge) {
      Chain chain;
      chain.start_cluster = start_cluster;
      std::size_t current_cluster = start_cluster;
      std::size_t current_edge = start_edge;
      std::vector<std::pair<std::size_t, bool>> traversed;
      while (true) {
        if (visited[current_edge]) {
          break;
        }
        visited[current_edge] = true;
        const BaseEdge & edge = base_edges[current_edge];
        const std::size_t next_cluster = edge.a == current_cluster ? edge.b : edge.a;
        const bool along_observation =
          edge.observed_from == current_cluster && edge.observed_to == next_cluster;
        traversed.emplace_back(current_edge, along_observation);
        if (chain.points.empty()) {
          chain.points.push_back(cluster_position[current_cluster]);
        }
        // Route geometry must follow the ordered topology-cluster centres.
        // Inserting one visit's raw endpoint before every cluster centre made
        // merged/revisited paths alternate between the visit and the centroid,
        // producing a saw-tooth centreline and self-intersecting offset
        // polygons even though the processed trajectory itself was smooth.
        if (distance3d(chain.points.back(), cluster_position[next_cluster]) > 1.0e-6) {
          chain.points.push_back(cluster_position[next_cluster]);
        }
        current_cluster = next_cluster;
        if (anchor[current_cluster]) {
          break;
        }
        std::optional<std::size_t> next_edge;
        for (const std::size_t candidate : adjacency[current_cluster]) {
          if (!visited[candidate]) {
            next_edge = candidate;
            break;
          }
        }
        if (!next_edge) {
          break;
        }
        current_edge = *next_edge;
      }
      chain.end_cluster = current_cluster;
      std::size_t forward_votes = 0U;
      for (const auto & value : traversed) {
        if (value.second) {
          ++forward_votes;
        }
      }
      if (!traversed.empty() && forward_votes * 2U < traversed.size()) {
        std::reverse(chain.points.begin(), chain.points.end());
        std::swap(chain.start_cluster, chain.end_cluster);
      }
      simplifyConsecutiveDuplicates(chain.points);
      if (chain.points.size() >= 2U) {
        chains.push_back(std::move(chain));
      }
    };

  for (std::size_t cluster = 0U; cluster < adjacency.size(); ++cluster) {
    if (!anchor[cluster]) {
      continue;
    }
    for (const std::size_t edge : adjacency[cluster]) {
      if (!visited[edge]) {
        walk(cluster, edge);
      }
    }
  }
  for (std::size_t edge = 0U; edge < base_edges.size(); ++edge) {
    if (!visited[edge]) {
      walk(base_edges[edge].a, edge);
    }
  }

  RouteGraph graph;
  graph.frame_id = frame_id;
  std::unordered_map<std::size_t, std::uint64_t> anchor_node;
  auto get_anchor_node = [&](const std::size_t cluster) {
      const auto found = anchor_node.find(cluster);
      if (found != anchor_node.end()) {
        return found->second;
      }
      const std::uint64_t id = static_cast<std::uint64_t>(graph.nodes.size() + 1U);
      graph.nodes.push_back({id, cluster_position[cluster], RouteNodeType::kNormal});
      anchor_node[cluster] = id;
      return id;
    };

  struct PendingEdge
  {
    std::uint64_t from{0U};
    std::uint64_t to{0U};
    std::vector<Vec3> points;
  };
  std::vector<PendingEdge> pending;
  for (const Chain & chain : chains) {
    const std::vector<Vec3> regularized_points = smoothTopologyPolyline(
      chain.points, config.geometry_smoothing_window);
    // Route topology is partitioned by planar navigation distance. Retained Z
    // remains part of the output geometry but cannot create extra Edges.
    const double total_length = planarPolylineLength(regularized_points);
    if (total_length < config.minimum_edge_length) {
      continue;
    }
    const bool closed_chain = chain.start_cluster == chain.end_cluster;
    // Reserve enough length for two adjacent cut snaps.  A near-exact
    // multiple may therefore gain one topology Edge, but no source geometry
    // is removed and every final Edge remains within maximum_edge_length.
    const double partition_limit =
      config.maximum_edge_length - 2.0 * kTopologySplitSnapToleranceM;
    if (!(partition_limit > kAutowareOverlappingPointDistanceM)) {
      throw std::invalid_argument(
              "topology maximum_edge_length is too small for Autoware "
              "1 mm centerline point separation");
    }
    std::size_t part_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(total_length / partition_limit)));
    // A closed degree-two chain would otherwise be emitted as from == to.  It
    // is valid geometry but unsafe graph topology for downstream planners, so
    // preserve it as two or more ordinary edges around an explicit node.
    if (closed_chain) {
      part_count = std::max<std::size_t>(2U, part_count);
    }
    const double part_length = total_length / static_cast<double>(part_count);
    if (part_length + 1.0e-9 < config.minimum_edge_length) {
      continue;
    }

    std::vector<double> chain_arc(regularized_points.size(), 0.0);
    for (std::size_t index = 1U; index < regularized_points.size(); ++index) {
      chain_arc[index] = chain_arc[index - 1U] +
        distance2d(regularized_points[index - 1U], regularized_points[index]);
    }
    std::vector<double> partition_arcs(part_count + 1U, 0.0);
    partition_arcs.back() = total_length;
    for (std::size_t part = 1U; part < part_count; ++part) {
      const double requested = total_length * static_cast<double>(part) /
        static_cast<double>(part_count);
      partition_arcs[part] = snapTopologyPartitionArcToSourceVertex(
        chain_arc, requested);
      if (!(partition_arcs[part] > partition_arcs[part - 1U] + 1.0e-12)) {
        throw std::logic_error("topology cut snapping changed Edge order");
      }
    }
    std::uint64_t from_node = get_anchor_node(chain.start_cluster);
    for (std::size_t part = 0U; part < part_count; ++part) {
      const double begin = partition_arcs[part];
      const double end = partition_arcs[part + 1U];
      std::vector<Vec3> geometry = polylineSlice(regularized_points, chain_arc, begin, end);
      if (geometry.size() < 2U ||
        planarPolylineLength(geometry) < config.minimum_edge_length)
      {
        continue;
      }
      std::uint64_t to_node = 0U;
      if (part + 1U == part_count) {
        to_node = get_anchor_node(chain.end_cluster);
        geometry.back() = graph.nodes[static_cast<std::size_t>(to_node - 1U)].position;
      } else {
        to_node = static_cast<std::uint64_t>(graph.nodes.size() + 1U);
        graph.nodes.push_back({to_node, geometry.back(), RouteNodeType::kNormal});
      }
      geometry.front() = graph.nodes[static_cast<std::size_t>(from_node - 1U)].position;
      if (planarPolylineLength(geometry) > config.maximum_edge_length + 1.0e-9) {
        throw std::logic_error("topology cut snapping exceeded maximum_edge_length");
      }
      if (from_node != to_node &&
        planarPolylineLength(geometry) >= config.minimum_edge_length)
      {
        pending.push_back({from_node, to_node, std::move(geometry)});
      }
      from_node = to_node;
    }
  }
  if (pending.empty()) {
    throw std::runtime_error("route graph contains no edge after length filtering");
  }

  std::uint64_t next_edge_id = static_cast<std::uint64_t>(graph.nodes.size() + 1U);
  for (PendingEdge & value : pending) {
    RouteEdge edge;
    edge.id = next_edge_id++;
    edge.from = value.from;
    edge.to = value.to;
    edge.centerline = std::move(value.points);
    edge.length = polylineLength(edge.centerline);
    edge.maximum_curvature = maximumPolylineCurvature(edge.centerline);
    graph.edges.push_back(std::move(edge));
  }

  if (config.generate_reverse_edges) {
    const std::size_t forward_count = graph.edges.size();
    for (std::size_t index = 0U; index < forward_count; ++index) {
      RouteEdge reverse = graph.edges[index];
      reverse.id = next_edge_id++;
      reverse.from = graph.edges[index].to;
      reverse.to = graph.edges[index].from;
      reverse.reverse_of = graph.edges[index].id;
      graph.edges[index].reverse_of = reverse.id;
      std::reverse(reverse.centerline.begin(), reverse.centerline.end());
      graph.edges.push_back(std::move(reverse));
    }
  }

  std::unordered_map<std::uint64_t, std::set<std::uint64_t>> neighbors;
  for (const RouteEdge & edge : graph.edges) {
    if (edge.reverse_of && edge.id > *edge.reverse_of) {
      continue;
    }
    neighbors[edge.from].insert(edge.to);
    neighbors[edge.to].insert(edge.from);
  }
  for (RouteNode & node : graph.nodes) {
    const std::size_t degree = neighbors[node.id].size();
    if (degree <= 1U) {
      node.type = RouteNodeType::kEndpoint;
    } else if (degree >= 3U) {
      node.type = RouteNodeType::kJunction;
    } else {
      node.type = RouteNodeType::kNormal;
    }
  }
  return graph;
}

void computeRouteClearanceImpl(
  RouteGraph & graph,
  const OccupancyGrid2D & grid,
  const OccupancyGrid2D * unknown_grid,
  const TraversabilityConfig & config,
  const double speed_limit_mps)
{
  // Densification bounds the lateral-ray spacing and keeps the conservative
  // centerline supercover scan below constant work per segment. Respect a
  // smaller configured ray step so longitudinal and lateral checks have
  // comparable spatial resolution.
  const double clearance_sample_spacing = std::min(
    config.ray_step, grid.resolution() * 0.5);

  for (RouteEdge & edge : graph.edges) {
    edge.left_boundary.clear();
    edge.right_boundary.clear();
    edge.left_clearance.clear();
    edge.right_clearance.clear();
    edge.left_clearance_observed.clear();
    edge.right_clearance_observed.clear();
    edge.validation_errors.clear();
    edge.corridor_geometry_valid = false;
    edge.minimum_safe_width = std::numeric_limits<double>::infinity();
    edge.passable = true;
    std::size_t observed_boundaries = 0U;
    std::size_t ray_count = 0U;

    if (edge.centerline.size() < 2U) {
      edge.passable = false;
      edge.confidence = 0.0;
      edge.minimum_safe_width = 0.0;
      addValidationError(edge, "centerline_too_short");
      continue;
    }
    edge.centerline = densifyPolyline(edge.centerline, clearance_sample_spacing);
    edge.maximum_curvature = maximumPolylineCurvature(edge.centerline);

    // Point samples alone can miss an arbitrarily short crossing through a
    // grid-cell corner. Trace every centerline segment against all closed cells
    // it touches before computing lateral clearance.
    const GridTraceResult obstacle_trace = tracePolylineCells(grid, edge.centerline);
    if (obstacle_trace.outside) {
      edge.passable = false;
      addValidationError(edge, "centerline_outside_grid");
    }
    if (obstacle_trace.occupied) {
      edge.passable = false;
      addValidationError(edge, "centerline_occupied");
    }
    if (unknown_grid != nullptr && config.unknown_space_policy != "allow") {
      const GridTraceResult unknown_trace = tracePolylineCells(*unknown_grid, edge.centerline);
      if (unknown_trace.outside || unknown_trace.occupied) {
        edge.passable = false;
        addValidationError(edge, "centerline_unknown");
      }
    }

    std::vector<Vec2> left_normals;
    std::vector<Vec2> right_normals;
    left_normals.reserve(edge.centerline.size());
    right_normals.reserve(edge.centerline.size());
    for (std::size_t index = 0U; index < edge.centerline.size(); ++index) {
      const Vec3 center = edge.centerline[index];
      const Vec2 tangent = tangentAt(edge.centerline, index);
      if (norm(tangent) < 1.0e-12) {
        edge.passable = false;
        addValidationError(edge, "degenerate_tangent");
      }
      const Vec2 left_normal{-tangent.y, tangent.x};
      const Vec2 right_normal{tangent.y, -tangent.x};
      left_normals.push_back(left_normal);
      right_normals.push_back(right_normal);
      const RaycastResult left = raycast(
        grid, unknown_grid, center, left_normal, config);
      const RaycastResult right = raycast(
        grid, unknown_grid, center, right_normal, config);
      edge.left_clearance.push_back(left.distance);
      edge.right_clearance.push_back(right.distance);
      edge.left_clearance_observed.push_back(left.clearance_complete ? 1U : 0U);
      edge.right_clearance_observed.push_back(right.clearance_complete ? 1U : 0U);
      if (!left.clearance_complete || !right.clearance_complete) {
        // UNKNOWN is a conservative corridor boundary, not automatically a
        // collision.  The edge remains eligible only when the contiguous
        // known-free width below satisfies minimum_safe_center_width.
        // The centerline supercover above handles UNKNOWN directly under the
        // Route, while this warning describes incomplete lateral width.
        addValidationError(edge, "unknown_clearance");
      }
      observed_boundaries += left.clearance_complete ? 1U : 0U;
      observed_boundaries += right.clearance_complete ? 1U : 0U;
      ray_count += 2U;
    }

    edge.left_clearance = regularizeClearance(
      edge.left_clearance, edge.centerline, config.maximum_clearance_slope);
    edge.right_clearance = regularizeClearance(
      edge.right_clearance, edge.centerline, config.maximum_clearance_slope);
    for (std::size_t index = 0U; index < edge.centerline.size(); ++index) {
      const Vec3 center = edge.centerline[index];
      const double left_distance = edge.left_clearance[index];
      const double right_distance = edge.right_clearance[index];
      edge.left_boundary.push_back({
        center.x + left_normals[index].x * left_distance,
        center.y + left_normals[index].y * left_distance,
        center.z});
      edge.right_boundary.push_back({
        center.x + right_normals[index].x * right_distance,
        center.y + right_normals[index].y * right_distance,
        center.z});
      const double safe_width = left_distance + right_distance;
      edge.minimum_safe_width = std::min(edge.minimum_safe_width, safe_width);
      if (safe_width < config.minimum_safe_center_width) {
        edge.passable = false;
        addValidationError(edge, "insufficient_clearance");
      }
    }

    bool tangent_reversal = false;
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      const Vec2 previous = tangentAt(edge.centerline, index - 1U);
      const Vec2 current = tangentAt(edge.centerline, index);
      if (norm(previous) < 1.0e-12 || norm(current) < 1.0e-12 ||
        dot(previous, current) <= 0.0)
      {
        tangent_reversal = true;
        break;
      }
    }
    if (tangent_reversal) {
      edge.passable = false;
      addValidationError(edge, "corridor_tangent_reversal");
    }
    if (polylineSelfIntersects(edge.centerline, false)) {
      edge.passable = false;
      addValidationError(edge, "centerline_self_intersection");
    }

    std::vector<Vec3> corridor_polygon = edge.left_boundary;
    corridor_polygon.insert(
      corridor_polygon.end(), edge.right_boundary.rbegin(), edge.right_boundary.rend());
    const bool corridor_degenerate = corridor_polygon.size() < 4U ||
      std::abs(signedPolygonArea(corridor_polygon)) < 1.0e-8;
    const bool corridor_self_intersection =
      !corridor_degenerate && polylineSelfIntersects(corridor_polygon, true);
    edge.corridor_geometry_valid =
      !corridor_degenerate && !corridor_self_intersection && !tangent_reversal;
    if (corridor_degenerate) {
      edge.passable = false;
      addValidationError(edge, "corridor_degenerate");
    }
    if (corridor_self_intersection) {
      edge.passable = false;
      addValidationError(edge, "corridor_self_intersection");
    }
    edge.confidence = ray_count > 0U ?
      static_cast<double>(observed_boundaries) / static_cast<double>(ray_count) : 0.0;
    const double curvature_factor = 1.0 / (1.0 + 3.0 * edge.maximum_curvature);
    edge.recommended_speed_mps = std::max(0.1, speed_limit_mps * curvature_factor);
  }
}

void computeRouteClearance(
  RouteGraph & graph,
  const OccupancyGrid2D & inflated_obstacle_grid,
  const OccupancyGrid2D & unknown_grid,
  const TraversabilityConfig & config,
  const double speed_limit_mps)
{
  computeRouteClearanceImpl(
    graph, inflated_obstacle_grid, &unknown_grid, config, speed_limit_mps);
}

void computeRouteClearance(
  RouteGraph & graph,
  const OccupancyGrid2D & inflated_obstacle_grid,
  const TraversabilityConfig & config,
  const double speed_limit_mps)
{
  // Backward-compatible entry point for callers that have no observation
  // mask.  Such empty cells are UNKNOWN, never assumed observed free.
  computeRouteClearanceImpl(
    graph, inflated_obstacle_grid, nullptr, config, speed_limit_mps);
}

}  // namespace lidar_mobility_map_generator
