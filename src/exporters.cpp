#include "lidar_mobility_map_generator/exporters.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#ifdef __CPPCHECK__
#define LMMG_PROJECT_VERSION "cppcheck"
#elif !defined(LMMG_PROJECT_VERSION)
#error "LMMG_PROJECT_VERSION must be provided by the build system"
#endif

namespace lidar_mobility_map_generator
{
namespace
{

// These guards compensate separately for linear interpolation between the
// discrete boundary cross-sections and for finite-precision tangents at an
// open component's two end caps. Both are part of the generated geometry
// contract, so saveLanelet2OsmInternal() records them on every closed-course
// Lanelet rather than hiding them as implementation details.
constexpr double kEstimatedBoundaryInterpolationGuardM = 0.05;
constexpr double kEstimatedLongitudinalEndpointGuardM = 0.05;

std::ofstream openOutput(const std::filesystem::path & path)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create output file: " + path.string());
  }
  stream << std::setprecision(12);
  return stream;
}

std::string jsonEscape(const std::string & input)
{
  std::string result;
  result.reserve(input.size());
  for (const char character : input) {
    switch (character) {
      case '"': result += "\\\""; break;
      case '\\': result += "\\\\"; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default: result += character; break;
    }
  }
  return result;
}

std::string xmlEscape(const std::string & input)
{
  std::string result;
  result.reserve(input.size());
  for (const char character : input) {
    switch (character) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      case '"': result += "&quot;"; break;
      case '\'': result += "&apos;"; break;
      default: result += character; break;
    }
  }
  return result;
}

void writeCoordinate2d(std::ostream & stream, const Vec3 & point)
{
  stream << '[' << point.x << ',' << point.y << ']';
}

void writeCoordinate3d(std::ostream & stream, const Vec3 & point)
{
  stream << '[' << point.x << ',' << point.y << ',' << point.z << ']';
}

std::string yamlQuote(const std::string & input)
{
  std::string result = "\"";
  for (const char character : input) {
    if (character == '\\' || character == '"') {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  result.push_back('"');
  return result;
}

std::string commaSeparatedIds(const std::vector<std::uint64_t> & ids)
{
  std::string result;
  for (const std::uint64_t id : ids) {
    if (!result.empty()) {
      result += ',';
    }
    result += std::to_string(id);
  }
  return result;
}

void writeValidationErrorsJson(
  std::ostream & stream, const std::vector<std::string> & errors)
{
  stream << '[';
  for (std::size_t index = 0U; index < errors.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << '"' << jsonEscape(errors[index]) << '"';
  }
  stream << ']';
}

bool clearanceComplete(const RouteEdge & edge)
{
  return edge.left_clearance_observed.size() == edge.centerline.size() &&
         edge.right_clearance_observed.size() == edge.centerline.size() &&
         std::all_of(
    edge.left_clearance_observed.begin(), edge.left_clearance_observed.end(),
    [](const std::uint8_t value) {return value != 0U;}) &&
         std::all_of(
    edge.right_clearance_observed.begin(), edge.right_clearance_observed.end(),
    [](const std::uint8_t value) {return value != 0U;});
}

enum class Lanelet2OutputKind
{
  kCandidate,
  kProductionReady,
  kClosedCourseExperimental
};

struct ExperimentalLanelet2Metadata
{
  ClosedCourseLanelet2ExportOptions options;
  ClosedCourseLanelet2ExportSummary summary;
  double corridor_width{0.0};
};

struct StopLinePlacement
{
  Vec3 center{};
  Vec2 tangent{};
  bool valid{false};
};

double polylineLength2d(const std::vector<Vec3> & points)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length += distance2d(points[index - 1U], points[index]);
  }
  return length;
}

StopLinePlacement projectStopLineToCenterline(
  const RouteEdge & edge, const AuthoredStopLine & stop)
{
  StopLinePlacement best;
  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc_difference = std::numeric_limits<double>::infinity();
  double traversed = 0.0;
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec3 & start = edge.centerline[index - 1U];
    const Vec3 & end = edge.centerline[index];
    const Vec3 delta = end - start;
    const double segment_length = distance3d(start, end);
    const Vec2 planar_delta{delta.x, delta.y};
    const double planar_squared = normSquared(planar_delta);
    if (!(segment_length > 1.0e-12) || planar_squared <= 1.0e-18 ||
      !finite(start) || !finite(end))
    {
      traversed += std::isfinite(segment_length) ? segment_length : 0.0;
      continue;
    }
    double ratio = 0.0;
    if (finite(stop.anchor)) {
      const Vec2 relative{stop.anchor.x - start.x, stop.anchor.y - start.y};
      ratio = clamp(dot(relative, planar_delta) / planar_squared, 0.0, 1.0);
    } else {
      ratio = clamp((stop.s - traversed) / segment_length, 0.0, 1.0);
    }
    const Vec3 point = start + delta * ratio;
    const double distance = finite(stop.anchor) ? distance3d(stop.anchor, point) : 0.0;
    const double arc_difference = std::abs((traversed + ratio * segment_length) - stop.s);
    if (distance + 1.0e-9 < best_distance ||
      (std::abs(distance - best_distance) <= 1.0e-9 &&
      arc_difference < best_arc_difference))
    {
      best.center = point;
      best.tangent = normalized(planar_delta);
      best.valid = norm(best.tangent) > 1.0e-12;
      best_distance = distance;
      best_arc_difference = arc_difference;
    }
    traversed += segment_length;
  }
  return best;
}

Vec2 stablePolylineTangent(
  const std::vector<Vec3> & points, const std::size_t index)
{
  std::optional<std::size_t> previous;
  for (std::size_t candidate = index; candidate > 0U; --candidate) {
    if (distance2d(points[candidate - 1U], points[index]) > 1.0e-9) {
      previous = candidate - 1U;
      break;
    }
  }
  std::optional<std::size_t> next;
  for (std::size_t candidate = index + 1U; candidate < points.size(); ++candidate) {
    if (distance2d(points[candidate], points[index]) > 1.0e-9) {
      next = candidate;
      break;
    }
  }
  if (previous && next) {
    return normalized(Vec2{
      points[*next].x - points[*previous].x,
      points[*next].y - points[*previous].y});
  }
  if (next) {
    return normalized(Vec2{
      points[*next].x - points[index].x,
      points[*next].y - points[index].y});
  }
  if (previous) {
    return normalized(Vec2{
      points[index].x - points[*previous].x,
      points[index].y - points[*previous].y});
  }
  return {};
}

Vec2 stablePolylineTangentOverSpan(
  const std::vector<Vec3> & points, const std::size_t index,
  const double minimum_span)
{
  std::size_t previous = index;
  double previous_span = 0.0;
  while (previous > 0U && previous_span < minimum_span) {
    previous_span += distance2d(points[previous], points[previous - 1U]);
    --previous;
  }
  std::size_t next = index;
  double next_span = 0.0;
  while (next + 1U < points.size() && next_span < minimum_span) {
    next_span += distance2d(points[next], points[next + 1U]);
    ++next;
  }
  if (previous < index && next > index) {
    return normalized(Vec2{
      points[next].x - points[previous].x,
      points[next].y - points[previous].y});
  }
  if (next > index) {
    return normalized(Vec2{
      points[next].x - points[index].x,
      points[next].y - points[index].y});
  }
  if (previous < index) {
    return normalized(Vec2{
      points[index].x - points[previous].x,
      points[index].y - points[previous].y});
  }
  return {};
}

double pointToSegmentDistance2d(
  const Vec3 & point, const Vec3 & start, const Vec3 & end)
{
  const Vec2 segment{end.x - start.x, end.y - start.y};
  const double denominator = normSquared(segment);
  if (denominator <= 1.0e-18) {
    return distance2d(point, start);
  }
  const Vec2 relative{point.x - start.x, point.y - start.y};
  const double ratio = clamp(dot(relative, segment) / denominator, 0.0, 1.0);
  const Vec3 closest = start + (end - start) * ratio;
  return distance2d(point, closest);
}

void retainRdpPoints(
  const std::vector<Vec3> & points, const std::size_t first,
  const std::size_t last, const double tolerance, std::vector<bool> & retained)
{
  if (last <= first + 1U) {
    return;
  }
  double largest_distance = -1.0;
  std::size_t largest_index = first;
  for (std::size_t index = first + 1U; index < last; ++index) {
    const double distance = pointToSegmentDistance2d(points[index], points[first], points[last]);
    if (distance > largest_distance) {
      largest_distance = distance;
      largest_index = index;
    }
  }
  if (largest_distance > tolerance) {
    retained[largest_index] = true;
    retainRdpPoints(points, first, largest_index, tolerance, retained);
    retainRdpPoints(points, largest_index, last, tolerance, retained);
  }
}

std::vector<Vec3> regularizeExperimentalCenterline(const std::vector<Vec3> & source)
{
  if (source.size() < 3U) {
    return source;
  }
  constexpr double maximum_chord_error = 0.05;
  constexpr double maximum_segment_length = 0.50;
  std::vector<bool> retained(source.size(), false);
  retained.front() = true;
  retained.back() = true;
  retainRdpPoints(
    source, 0U, source.size() - 1U, maximum_chord_error, retained);
  std::vector<Vec3> simplified;
  for (std::size_t index = 0U; index < source.size(); ++index) {
    if (retained[index] &&
      (simplified.empty() || distance3d(simplified.back(), source[index]) > 1.0e-9))
    {
      simplified.push_back(source[index]);
    }
  }
  if (simplified.size() < 2U) {
    return {source.front(), source.back()};
  }
  std::vector<Vec3> result{simplified.front()};
  for (std::size_t index = 1U; index < simplified.size(); ++index) {
    const Vec3 & start = simplified[index - 1U];
    const Vec3 & end = simplified[index];
    const double length = distance3d(start, end);
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / maximum_segment_length)));
    for (std::size_t piece = 1U; piece <= pieces; ++piece) {
      result.push_back(
        start + (end - start) *
        (static_cast<double>(piece) / static_cast<double>(pieces)));
    }
  }
  return result;
}

std::vector<Vec3> envelopeReferenceCenterline(const std::vector<Vec3> & source)
{
  std::vector<Vec3> reference = source;
  if (source.size() < 4U) {
    return reference;
  }
  constexpr double maximum_isolated_excursion_length_m = 0.50;
  constexpr double minimum_turn_angle_rad = 75.0 * kPi / 180.0;
  const auto turn_angle = [&](const std::size_t index) {
      const Vec2 incoming{
        source[index].x - source[index - 1U].x,
        source[index].y - source[index - 1U].y};
      const Vec2 outgoing{
        source[index + 1U].x - source[index].x,
        source[index + 1U].y - source[index].y};
      if (norm(incoming) <= 1.0e-12 || norm(outgoing) <= 1.0e-12) {
        return 0.0;
      }
      return std::abs(normalizeAngle(
          std::atan2(outgoing.y, outgoing.x) -
          std::atan2(incoming.y, incoming.x)));
    };
  for (std::size_t first = 1U; first + 2U < source.size(); ++first) {
    const std::size_t second = first + 1U;
    if (distance2d(source[first], source[second]) + 1.0e-12 >=
      maximum_isolated_excursion_length_m ||
      turn_angle(first) + 1.0e-12 < minimum_turn_angle_rad ||
      turn_angle(second) + 1.0e-12 < minimum_turn_angle_rad)
    {
      continue;
    }
    const double incoming_length = distance2d(source[first - 1U], source[first]);
    const double excursion_length = distance2d(source[first], source[second]);
    const double outgoing_length = distance2d(source[second], source[second + 1U]);
    const double local_length = incoming_length + excursion_length + outgoing_length;
    if (!(local_length > 1.0e-12) ||
      distance2d(source[first - 1U], source[second + 1U]) <= 1.0e-12)
    {
      continue;
    }
    reference[first] = source[first - 1U] +
      (source[second + 1U] - source[first - 1U]) *
      (incoming_length / local_length);
    reference[second] = source[first - 1U] +
      (source[second + 1U] - source[first - 1U]) *
      ((incoming_length + excursion_length) / local_length);
    ++first;
  }
  return reference;
}

double cross2d(const Vec2 & first, const Vec2 & second, const Vec2 & third)
{
  return (second.x - first.x) * (third.y - first.y) -
         (second.y - first.y) * (third.x - first.x);
}

bool pointOnSegment2d(
  const Vec2 & point, const Vec2 & first, const Vec2 & second)
{
  constexpr double tolerance = 1.0e-9;
  return std::abs(cross2d(first, second, point)) <= tolerance &&
         point.x >= std::min(first.x, second.x) - tolerance &&
         point.x <= std::max(first.x, second.x) + tolerance &&
         point.y >= std::min(first.y, second.y) - tolerance &&
         point.y <= std::max(first.y, second.y) + tolerance;
}

bool segmentsIntersect2d(
  const Vec3 & first_start, const Vec3 & first_end,
  const Vec3 & second_start, const Vec3 & second_end)
{
  const Vec2 a{first_start.x, first_start.y};
  const Vec2 b{first_end.x, first_end.y};
  const Vec2 c{second_start.x, second_start.y};
  const Vec2 d{second_end.x, second_end.y};
  const double ab_c = cross2d(a, b, c);
  const double ab_d = cross2d(a, b, d);
  const double cd_a = cross2d(c, d, a);
  const double cd_b = cross2d(c, d, b);
  constexpr double tolerance = 1.0e-9;
  if (((ab_c > tolerance && ab_d < -tolerance) ||
    (ab_c < -tolerance && ab_d > tolerance)) &&
    ((cd_a > tolerance && cd_b < -tolerance) ||
    (cd_a < -tolerance && cd_b > tolerance)))
  {
    return true;
  }
  return (std::abs(ab_c) <= tolerance && pointOnSegment2d(c, a, b)) ||
         (std::abs(ab_d) <= tolerance && pointOnSegment2d(d, a, b)) ||
         (std::abs(cd_a) <= tolerance && pointOnSegment2d(a, c, d)) ||
         (std::abs(cd_b) <= tolerance && pointOnSegment2d(b, c, d));
}

bool boundaryPolylineSelfIntersects(const std::vector<Vec3> & points)
{
  const bool closed = points.size() > 3U &&
    distance2d(points.front(), points.back()) <= 1.0e-9;
  for (std::size_t first = 1U; first < points.size(); ++first) {
    for (std::size_t second = first + 2U; second < points.size(); ++second) {
      if (closed && first == 1U && second + 1U == points.size()) {
        continue;
      }
      if (segmentsIntersect2d(
          points[first - 1U], points[first],
          points[second - 1U], points[second]))
      {
        return true;
      }
    }
  }
  return false;
}

bool estimatedBoundaryPolygonIsValid(const RouteEdge & edge)
{
  if (edge.left_boundary.size() != edge.centerline.size() ||
    edge.right_boundary.size() != edge.centerline.size() ||
    edge.centerline.size() < 2U ||
    boundaryPolylineSelfIntersects(edge.left_boundary) ||
    boundaryPolylineSelfIntersects(edge.right_boundary))
  {
    return false;
  }
  for (std::size_t left_segment = 1U;
    left_segment < edge.left_boundary.size(); ++left_segment)
  {
    for (std::size_t right_segment = 1U;
      right_segment < edge.right_boundary.size(); ++right_segment)
    {
      if (segmentsIntersect2d(
          edge.left_boundary[left_segment - 1U],
          edge.left_boundary[left_segment],
          edge.right_boundary[right_segment - 1U],
          edge.right_boundary[right_segment]))
      {
        return false;
      }
    }
  }
  double twice_area = 0.0;
  std::vector<Vec3> polygon = edge.left_boundary;
  polygon.insert(
    polygon.end(), edge.right_boundary.rbegin(), edge.right_boundary.rend());
  // Checking the two side polylines and their mutual crossings is not enough:
  // a very short Edge can have individually simple sides while its start and
  // end cap segments overlap.  Close the complete Lanelet ring explicitly so
  // the same self-intersection class rejected by the post-export validator is
  // caught before an OSM artifact is written.
  polygon.push_back(polygon.front());
  if (boundaryPolylineSelfIntersects(polygon)) {
    return false;
  }
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    const Vec3 & first = polygon[index];
    const Vec3 & second = polygon[(index + 1U) % polygon.size()];
    twice_area += first.x * second.y - second.x * first.y;
  }
  return std::abs(twice_area) > 1.0e-6;
}

struct RoadPolygon2d
{
  std::uint64_t edge_id{0U};
  std::vector<Vec3> vertices;
  double minimum_x{std::numeric_limits<double>::infinity()};
  double maximum_x{-std::numeric_limits<double>::infinity()};
  double minimum_y{std::numeric_limits<double>::infinity()};
  double maximum_y{-std::numeric_limits<double>::infinity()};
};

struct NonadjacentRawCenterlineIsolation
{
  double distance_m{std::numeric_limits<double>::infinity()};
  std::size_t centerline_count{0U};
  std::vector<std::uint64_t> nearest_edge_ids;
};

NonadjacentRawCenterlineIsolation auditNonadjacentRawCenterlineIsolation(
  const Vec3 & outer_pose, const std::uint64_t adjacent_raw_edge_id,
  const std::vector<RouteEdge> & raw_edges)
{
  constexpr double tie_tolerance_m = 1.0e-9;
  NonadjacentRawCenterlineIsolation result;
  for (const RouteEdge & edge : raw_edges) {
    if (edge.id == adjacent_raw_edge_id) {
      continue;
    }
    ++result.centerline_count;
    double edge_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      edge_distance = std::min(
        edge_distance,
        pointToSegmentDistance2d(
          outer_pose, edge.centerline[index - 1U], edge.centerline[index]));
    }
    if (edge_distance + tie_tolerance_m < result.distance_m) {
      result.distance_m = edge_distance;
      result.nearest_edge_ids = {edge.id};
    } else if (std::abs(edge_distance - result.distance_m) <= tie_tolerance_m) {
      result.nearest_edge_ids.push_back(edge.id);
    }
  }
  std::sort(result.nearest_edge_ids.begin(), result.nearest_edge_ids.end());
  result.nearest_edge_ids.erase(
    std::unique(result.nearest_edge_ids.begin(), result.nearest_edge_ids.end()),
    result.nearest_edge_ids.end());
  return result;
}

double requiredOuterPoseNonadjacentRawCenterlineIsolation(
  const ClosedCourseLanelet2ExportOptions & options)
{
  const double half_width_with_margin =
    0.5 * options.estimated_vehicle_width + options.lateral_clearance_margin;
  const double maximum_longitudinal_extent = std::max(
    options.estimated_front_extent, options.estimated_rear_extent);
  const double footprint_circumradius = std::hypot(
    maximum_longitudinal_extent, half_width_with_margin);
  return footprint_circumradius + options.planning_endpoint_allowance;
}

RoadPolygon2d roadPolygonForEdge(const RouteEdge & edge)
{
  RoadPolygon2d result;
  result.edge_id = edge.id;
  result.vertices = edge.left_boundary;
  result.vertices.insert(
    result.vertices.end(), edge.right_boundary.rbegin(), edge.right_boundary.rend());
  for (const Vec3 & point : result.vertices) {
    result.minimum_x = std::min(result.minimum_x, point.x);
    result.maximum_x = std::max(result.maximum_x, point.x);
    result.minimum_y = std::min(result.minimum_y, point.y);
    result.maximum_y = std::max(result.maximum_y, point.y);
  }
  return result;
}

bool pointInRoadPolygonInclusive(const Vec3 & point, const RoadPolygon2d & polygon)
{
  constexpr double tolerance = 1.0e-9;
  if (polygon.vertices.size() < 3U ||
    point.x < polygon.minimum_x - tolerance ||
    point.x > polygon.maximum_x + tolerance ||
    point.y < polygon.minimum_y - tolerance ||
    point.y > polygon.maximum_y + tolerance)
  {
    return false;
  }
  const Vec2 query{point.x, point.y};
  bool inside = false;
  for (std::size_t index = 0U; index < polygon.vertices.size(); ++index) {
    const Vec3 & first = polygon.vertices[index];
    const Vec3 & second = polygon.vertices[(index + 1U) % polygon.vertices.size()];
    if (pointOnSegment2d(
        query, Vec2{first.x, first.y}, Vec2{second.x, second.y}))
    {
      return true;
    }
    const bool crosses = (first.y > point.y) != (second.y > point.y);
    if (crosses) {
      const double crossing_x = first.x +
        (point.y - first.y) * (second.x - first.x) / (second.y - first.y);
      if (crossing_x >= point.x - tolerance) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool polygonsOverlap2d(
  const std::vector<Vec3> & first, const RoadPolygon2d & second)
{
  if (first.size() < 3U || second.vertices.size() < 3U) {
    return false;
  }
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const Vec3 & point : first) {
    minimum_x = std::min(minimum_x, point.x);
    maximum_x = std::max(maximum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_y = std::max(maximum_y, point.y);
  }
  constexpr double tolerance = 1.0e-9;
  if (maximum_x < second.minimum_x - tolerance ||
    minimum_x > second.maximum_x + tolerance ||
    maximum_y < second.minimum_y - tolerance ||
    minimum_y > second.maximum_y + tolerance)
  {
    return false;
  }
  for (const Vec3 & point : first) {
    if (pointInRoadPolygonInclusive(point, second)) {
      return true;
    }
  }
  RoadPolygon2d first_polygon;
  first_polygon.vertices = first;
  first_polygon.minimum_x = minimum_x;
  first_polygon.maximum_x = maximum_x;
  first_polygon.minimum_y = minimum_y;
  first_polygon.maximum_y = maximum_y;
  for (const Vec3 & point : second.vertices) {
    if (pointInRoadPolygonInclusive(point, first_polygon)) {
      return true;
    }
  }
  for (std::size_t first_index = 0U; first_index < first.size(); ++first_index) {
    const Vec3 & first_start = first[first_index];
    const Vec3 & first_end = first[(first_index + 1U) % first.size()];
    for (std::size_t second_index = 0U;
      second_index < second.vertices.size(); ++second_index)
    {
      if (segmentsIntersect2d(
          first_start, first_end, second.vertices[second_index],
          second.vertices[(second_index + 1U) % second.vertices.size()]))
      {
        return true;
      }
    }
  }
  return false;
}

std::vector<Vec3> vehicleFootprintPolygon(
  const Vec3 & center, const Vec2 & tangent,
  const ClosedCourseLanelet2ExportOptions & options)
{
  const Vec2 direction = normalized(tangent);
  const Vec2 normal{-direction.y, direction.x};
  const double half_width =
    0.5 * options.estimated_vehicle_width + options.lateral_clearance_margin;
  const auto corner = [&](const double longitudinal, const double lateral) {
      return Vec3{
        center.x + direction.x * longitudinal + normal.x * lateral,
        center.y + direction.y * longitudinal + normal.y * lateral,
        center.z};
    };
  return {
    corner(options.estimated_front_extent, half_width),
    corner(options.estimated_front_extent, -half_width),
    corner(-options.estimated_rear_extent, -half_width),
    corner(-options.estimated_rear_extent, half_width)};
}

std::vector<std::uint64_t> overlappingRoadPolygonIds(
  const std::vector<Vec3> & polygon,
  const std::vector<RoadPolygon2d> & roads)
{
  std::vector<std::uint64_t> result;
  for (const RoadPolygon2d & road : roads) {
    if (polygonsOverlap2d(polygon, road)) {
      result.push_back(road.edge_id);
    }
  }
  return result;
}

std::vector<std::uint64_t> containingRoadPolygonIds(
  const Vec3 & point, const std::vector<RoadPolygon2d> & roads)
{
  std::vector<std::uint64_t> result;
  for (const RoadPolygon2d & road : roads) {
    if (pointInRoadPolygonInclusive(point, road)) {
      result.push_back(road.edge_id);
    }
  }
  return result;
}

bool polygonContainedInRoadPolygon(
  const std::vector<Vec3> & subject, const RoadPolygon2d & container)
{
  constexpr double sample_spacing_m = 0.05;
  if (subject.size() < 3U) {
    return false;
  }
  for (std::size_t index = 0U; index < subject.size(); ++index) {
    const Vec3 & first = subject[index];
    const Vec3 & second = subject[(index + 1U) % subject.size()];
    const double length = distance2d(first, second);
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / sample_spacing_m)));
    for (std::size_t piece = 0U; piece <= pieces; ++piece) {
      const Vec3 point = first + (second - first) *
        (static_cast<double>(piece) / static_cast<double>(pieces));
      if (!pointInRoadPolygonInclusive(point, container)) {
        return false;
      }
    }
  }
  return true;
}

bool polygonContainedInAnyRoadPolygon(
  const std::vector<Vec3> & subject,
  const std::vector<RoadPolygon2d> & containers)
{
  constexpr double sample_spacing_m = 0.05;
  if (subject.size() < 3U || containers.empty()) {
    return false;
  }
  for (std::size_t index = 0U; index < subject.size(); ++index) {
    const Vec3 & first = subject[index];
    const Vec3 & second = subject[(index + 1U) % subject.size()];
    const double length = distance2d(first, second);
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / sample_spacing_m)));
    for (std::size_t piece = 0U; piece <= pieces; ++piece) {
      const Vec3 point = first + (second - first) *
        (static_cast<double>(piece) / static_cast<double>(pieces));
      const bool contained = std::any_of(
        containers.begin(), containers.end(),
        [&](const RoadPolygon2d & polygon) {
          return pointInRoadPolygonInclusive(point, polygon);
        });
      if (!contained) {
        return false;
      }
    }
  }
  return true;
}

bool replaceWithEstimatedDrivableBoundaries(
  RouteEdge & edge, const ClosedCourseLanelet2ExportOptions & options,
  const bool regularize_centerline = true, const bool validate_polygon = true)
{
  if (edge.centerline.size() < 2U ||
    !(options.estimated_vehicle_width > 0.0) ||
    !(options.estimated_front_extent > 0.0) ||
    !(options.estimated_rear_extent > 0.0) ||
    options.lateral_clearance_margin < 0.0)
  {
    return false;
  }
  if (regularize_centerline) {
    edge.centerline = regularizeExperimentalCenterline(edge.centerline);
  }
  const std::vector<Vec3> envelope_centerline = regularize_centerline ?
    edge.centerline : envelopeReferenceCenterline(edge.centerline);
  std::vector<Vec2> tangents;
  tangents.reserve(envelope_centerline.size());
  const bool closed_centerline = envelope_centerline.size() > 3U &&
    distance2d(envelope_centerline.front(), envelope_centerline.back()) <= 1.0e-9;
  for (std::size_t index = 0U; index < envelope_centerline.size(); ++index) {
    const Vec3 & center = envelope_centerline[index];
    if (!finite(center)) {
      return false;
    }
    // Exact audited composite samples can be much denser than the normal
    // regularized centerline.  Use a bounded chord for their boundary tangent
    // so centimetre-scale localization jitter cannot fold a metre-wide offset
    // polygon while the centerline itself remains byte-for-byte traceable.
    constexpr double exact_composite_tangent_span = 0.50;
    Vec2 tangent;
    if (closed_centerline && (index == 0U || index + 1U == envelope_centerline.size())) {
      tangent = normalized(Vec2{
          envelope_centerline[1U].x -
          envelope_centerline[envelope_centerline.size() - 2U].x,
          envelope_centerline[1U].y -
          envelope_centerline[envelope_centerline.size() - 2U].y});
    } else {
      tangent = regularize_centerline ?
        stablePolylineTangent(envelope_centerline, index) :
        stablePolylineTangentOverSpan(
        envelope_centerline, index, exact_composite_tangent_span);
    }
    if (norm(tangent) <= 1.0e-12) {
      return false;
    }
    tangents.push_back(tangent);
  }

  // Build a cross-section envelope of the yawed configured rectangle, rather
  // than offsetting the centreline by one constant width.  Every regularized
  // centreline sample is a vehicle pose. Dense samples on all four footprint
  // sides are projected onto nearby route cross-sections; their signed lateral
  // extrema become the left/right offsets. This retains longitudinal-overhang
  // sweep through curves without pretending that the estimated boundaries are
  // surveyed paint or kerbs.
  constexpr double footprint_sample_spacing = 0.10;
  double maximum_centerline_step = 0.0;
  for (std::size_t index = 1U; index < envelope_centerline.size(); ++index) {
    maximum_centerline_step = std::max(
      maximum_centerline_step,
      distance2d(envelope_centerline[index - 1U], envelope_centerline[index]));
  }
  const double cross_section_capture =
    std::max(0.10, 0.5 * maximum_centerline_step + footprint_sample_spacing);
  const double half_vehicle_width = 0.5 * options.estimated_vehicle_width;
  std::vector<double> route_arcs(envelope_centerline.size(), 0.0);
  std::vector<double> source_arcs(edge.centerline.size(), 0.0);
  for (std::size_t index = 1U; index < envelope_centerline.size(); ++index) {
    route_arcs[index] = route_arcs[index - 1U] +
      distance2d(envelope_centerline[index - 1U], envelope_centerline[index]);
    source_arcs[index] = source_arcs[index - 1U] +
      distance2d(edge.centerline[index - 1U], edge.centerline[index]);
  }
  const double route_length = route_arcs.back();
  const double source_length = source_arcs.back();
  std::vector<double> left_offsets(
    envelope_centerline.size(), half_vehicle_width);
  std::vector<double> right_offsets(
    envelope_centerline.size(), half_vehicle_width);

  auto observe_footprint_point = [&](const Vec2 & point, const double pose_arc) {
      for (std::size_t station = 0U; station < envelope_centerline.size(); ++station) {
        double station_from_pose = route_arcs[station] - pose_arc;
        if (closed_centerline && route_length > 1.0e-9) {
          if (station_from_pose > 0.5 * route_length) {
            station_from_pose -= route_length;
          } else if (station_from_pose < -0.5 * route_length) {
            station_from_pose += route_length;
          }
        }
        if (station_from_pose <
          -options.estimated_rear_extent - cross_section_capture ||
          station_from_pose >
          options.estimated_front_extent + cross_section_capture)
        {
          continue;
        }
        const Vec3 & center = envelope_centerline[station];
        const Vec2 delta{point.x - center.x, point.y - center.y};
        const Vec2 tangent = tangents[station];
        const double longitudinal = dot(delta, tangent);
        // The terminal boundary nodes are translated to the rear/front cap
        // after the lateral envelope is accumulated.  A footprint corner on
        // a curved approach can already be beyond the terminal cross-section
        // and therefore failed the ordinary narrow station-capture test.  It
        // still contributes to the cap's lateral support function.  Include
        // those points explicitly; otherwise the long final boundary segment
        // can cut a few millimetres inside the very sweep it claims to bound.
        const bool terminal_cap_point = !closed_centerline &&
          ((station == 0U && longitudinal <= 0.0 &&
          longitudinal >=
          -(options.estimated_rear_extent + kEstimatedLongitudinalEndpointGuardM)) ||
          (station + 1U == envelope_centerline.size() && longitudinal >= 0.0 &&
          longitudinal <=
          options.estimated_front_extent + kEstimatedLongitudinalEndpointGuardM));
        if (!terminal_cap_point && std::abs(longitudinal) > cross_section_capture) {
          continue;
        }
        const Vec2 normal{-tangent.y, tangent.x};
        const double lateral = dot(delta, normal);
        left_offsets[station] = std::max(left_offsets[station], lateral);
        right_offsets[station] = std::max(right_offsets[station], -lateral);
      }
    };
  const double longitudinal_span =
    options.estimated_front_extent + options.estimated_rear_extent;
  const std::size_t longitudinal_pieces = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(longitudinal_span / footprint_sample_spacing)));
  const std::size_t lateral_pieces = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(options.estimated_vehicle_width / footprint_sample_spacing)));
  const auto position_at_arc = [&](
      const std::vector<Vec3> & points, const std::vector<double> & arcs,
      double arc, const bool closed) {
      const double length = arcs.back();
      if (closed && length > 1.0e-9) {
        arc = std::fmod(arc, length);
        if (arc < 0.0) {arc += length;}
      } else {
        arc = clamp(arc, 0.0, length);
      }
      const auto upper = std::upper_bound(arcs.begin(), arcs.end(), arc);
      const std::size_t second = upper == arcs.end() ?
        arcs.size() - 1U :
        static_cast<std::size_t>(std::distance(arcs.begin(), upper));
      const std::size_t first = second == 0U ? 0U : second - 1U;
      const double span = arcs[second] - arcs[first];
      const double ratio = span > 1.0e-12 ?
        (arc - arcs[first]) / span : 0.0;
      return points[first] + (points[second] - points[first]) * ratio;
    };
  constexpr double envelope_pose_spacing = 0.10;
  constexpr double envelope_tangent_span = 0.25;
  const std::size_t pose_pieces = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(source_length / envelope_pose_spacing)));
  for (std::size_t pose = 0U; pose <= pose_pieces; ++pose) {
    const double fraction = static_cast<double>(pose) /
      static_cast<double>(pose_pieces);
    const double pose_arc = source_length * fraction;
    const double envelope_arc = route_length * fraction;
    const Vec3 center = position_at_arc(
      edge.centerline, source_arcs, pose_arc, closed_centerline);
    const Vec3 before = position_at_arc(
      edge.centerline, source_arcs, pose_arc - envelope_tangent_span,
      closed_centerline);
    const Vec3 after = position_at_arc(
      edge.centerline, source_arcs, pose_arc + envelope_tangent_span,
      closed_centerline);
    const Vec2 tangent = normalized(Vec2{after.x - before.x, after.y - before.y});
    if (norm(tangent) <= 1.0e-12) {
      return false;
    }
    const Vec2 normal{-tangent.y, tangent.x};
    auto world_point = [&](const double longitudinal, const double lateral) {
        return Vec2{
          center.x + tangent.x * longitudinal + normal.x * lateral,
          center.y + tangent.y * longitudinal + normal.y * lateral};
      };
    for (std::size_t piece = 0U; piece <= longitudinal_pieces; ++piece) {
      const double longitudinal = -options.estimated_rear_extent +
        longitudinal_span * static_cast<double>(piece) /
        static_cast<double>(longitudinal_pieces);
      observe_footprint_point(
        world_point(longitudinal, half_vehicle_width), envelope_arc);
      observe_footprint_point(
        world_point(longitudinal, -half_vehicle_width), envelope_arc);
    }
    for (std::size_t piece = 0U; piece <= lateral_pieces; ++piece) {
      const double lateral = -half_vehicle_width +
        options.estimated_vehicle_width * static_cast<double>(piece) /
        static_cast<double>(lateral_pieces);
      observe_footprint_point(
        world_point(options.estimated_front_extent, lateral), envelope_arc);
      observe_footprint_point(
        world_point(-options.estimated_rear_extent, lateral), envelope_arc);
    }
  }

  // A one-station maximum filter prevents linear interpolation between dense
  // cross-sections from cutting inside a sampled footprint extremum.
  const std::vector<double> raw_left = left_offsets;
  const std::vector<double> raw_right = right_offsets;
  for (std::size_t index = 0U; index < envelope_centerline.size(); ++index) {
    const std::size_t first = index == 0U ? 0U : index - 1U;
    const std::size_t last = std::min(index + 1U, envelope_centerline.size() - 1U);
    for (std::size_t neighbor = first; neighbor <= last; ++neighbor) {
      left_offsets[index] = std::max(left_offsets[index], raw_left[neighbor]);
      right_offsets[index] = std::max(right_offsets[index], raw_right[neighbor]);
    }
    left_offsets[index] +=
      options.lateral_clearance_margin + kEstimatedBoundaryInterpolationGuardM;
    right_offsets[index] +=
      options.lateral_clearance_margin + kEstimatedBoundaryInterpolationGuardM;
  }

  edge.left_boundary.clear();
  edge.right_boundary.clear();
  edge.left_boundary.reserve(envelope_centerline.size());
  edge.right_boundary.reserve(envelope_centerline.size());
  for (std::size_t index = 0U; index < envelope_centerline.size(); ++index) {
    const Vec3 & center = envelope_centerline[index];
    const Vec2 left_normal{-tangents[index].y, tangents[index].x};
    edge.left_boundary.push_back({
      center.x + left_normal.x * left_offsets[index],
      center.y + left_normal.y * left_offsets[index],
      center.z});
    edge.right_boundary.push_back({
      center.x - left_normal.x * right_offsets[index],
      center.y - left_normal.y * right_offsets[index],
      center.z});
  }
  if (!closed_centerline) {
    // The raw replay includes both terminal base poses.  Its boundary must
    // therefore contain the complete rear overhang at arc 0 and the complete
    // front overhang at arc L; excluding those poses would make an open map
    // look safer by silently shortening the verified sweep.  Extend only the
    // outer boundary caps (never the immutable centerline), with the tagged
    // guard beyond the configured physical extents for numeric closure.
    const Vec2 head_extension = tangents.front() *
      -(options.estimated_rear_extent + kEstimatedLongitudinalEndpointGuardM);
    const Vec2 tail_extension = tangents.back() *
      (options.estimated_front_extent + kEstimatedLongitudinalEndpointGuardM);
    edge.left_boundary.front().x += head_extension.x;
    edge.left_boundary.front().y += head_extension.y;
    edge.right_boundary.front().x += head_extension.x;
    edge.right_boundary.front().y += head_extension.y;
    edge.left_boundary.back().x += tail_extension.x;
    edge.left_boundary.back().y += tail_extension.y;
    edge.right_boundary.back().x += tail_extension.x;
    edge.right_boundary.back().y += tail_extension.y;
  }
  return !validate_polygon || estimatedBoundaryPolygonIsValid(edge);
}

bool replaceOrderedRouteWithEstimatedDrivableBoundaries(
  std::vector<RouteEdge> & edges,
  const std::vector<std::uint64_t> & ordered_edge_ids,
  const ClosedCourseLanelet2ExportOptions & options)
{
  if (ordered_edge_ids.empty()) {
    return false;
  }
  std::map<std::uint64_t, std::size_t> edge_index;
  for (std::size_t index = 0U; index < edges.size(); ++index) {
    if (!edge_index.emplace(edges[index].id, index).second) {
      return false;
    }
  }
  RouteEdge combined;
  combined.id = ordered_edge_ids.front();
  std::vector<std::pair<std::size_t, std::size_t>> ranges;
  ranges.reserve(ordered_edge_ids.size());
  for (const std::uint64_t edge_id : ordered_edge_ids) {
    const auto found = edge_index.find(edge_id);
    if (found == edge_index.end()) {
      return false;
    }
    const RouteEdge & edge = edges[found->second];
    if (edge.centerline.size() < 2U) {
      return false;
    }
    const std::size_t start = combined.centerline.empty() ?
      0U : combined.centerline.size() - 1U;
    if (combined.centerline.empty()) {
      combined.centerline = edge.centerline;
    } else {
      if (distance3d(combined.centerline.back(), edge.centerline.front()) > 1.0e-6) {
        return false;
      }
      combined.centerline.insert(
        combined.centerline.end(), edge.centerline.begin() + 1,
        edge.centerline.end());
    }
    ranges.emplace_back(start, combined.centerline.size() - 1U);
  }
  if (!replaceWithEstimatedDrivableBoundaries(combined, options, false, false)) {
    throw std::invalid_argument(
            "combined ordered Route swept-envelope construction failed");
  }
  for (std::size_t order = 0U; order < ordered_edge_ids.size(); ++order) {
    RouteEdge & edge = edges[edge_index.at(ordered_edge_ids[order])];
    const auto [first, last] = ranges[order];
    edge.centerline.assign(
      combined.centerline.begin() + static_cast<std::ptrdiff_t>(first),
      combined.centerline.begin() + static_cast<std::ptrdiff_t>(last + 1U));
    edge.left_boundary.assign(
      combined.left_boundary.begin() + static_cast<std::ptrdiff_t>(first),
      combined.left_boundary.begin() + static_cast<std::ptrdiff_t>(last + 1U));
    edge.right_boundary.assign(
      combined.right_boundary.begin() + static_cast<std::ptrdiff_t>(first),
      combined.right_boundary.begin() + static_cast<std::ptrdiff_t>(last + 1U));
    if (!estimatedBoundaryPolygonIsValid(edge)) {
      const std::string reason = boundaryPolylineSelfIntersects(edge.left_boundary) ?
        "left boundary self-intersects" :
        (boundaryPolylineSelfIntersects(edge.right_boundary) ?
        "right boundary self-intersects" : "left/right boundaries cross or area is zero");
      throw std::invalid_argument(
              "ordered Route swept-envelope polygon is invalid for Edge " +
              std::to_string(edge.id) + " (" +
              std::to_string(edge.centerline.size()) + " centerline samples): " + reason);
    }
  }
  return true;
}

bool replaceUnbranchedComponentsWithEstimatedDrivableBoundaries(
  std::vector<RouteEdge> & edges,
  const ClosedCourseLanelet2ExportOptions & options)
{
  if (edges.empty()) {
    return true;
  }
  std::map<std::uint64_t, std::size_t> edge_index;
  std::map<std::uint64_t, std::vector<std::uint64_t>> outgoing;
  std::map<std::uint64_t, std::vector<std::uint64_t>> incoming;
  for (std::size_t index = 0U; index < edges.size(); ++index) {
    const RouteEdge & edge = edges[index];
    if (!edge_index.emplace(edge.id, index).second) {
      return false;
    }
    outgoing[edge.from].push_back(edge.id);
    incoming[edge.to].push_back(edge.id);
    outgoing.try_emplace(edge.to);
    incoming.try_emplace(edge.from);
  }
  if (std::any_of(
      outgoing.begin(), outgoing.end(),
      [](const auto & item) {return item.second.size() > 1U;}) ||
    std::any_of(
      incoming.begin(), incoming.end(),
      [](const auto & item) {return item.second.size() > 1U;}))
  {
    // A branched topology has multiple transition sweeps. The acceptance
    // validator rejects it for closed-course readiness; retain the old
    // per-Edge diagnostic export below instead of choosing one branch.
    return false;
  }

  std::set<std::uint64_t> visited;
  const auto sweep_from = [&](const std::uint64_t first_id) {
      std::vector<std::uint64_t> ordered;
      std::uint64_t edge_id = first_id;
      while (visited.insert(edge_id).second) {
        ordered.push_back(edge_id);
        const RouteEdge & edge = edges[edge_index.at(edge_id)];
        const auto successors = outgoing.find(edge.to);
        if (successors == outgoing.end() || successors->second.empty()) {
          break;
        }
        edge_id = successors->second.front();
      }
      if (!ordered.empty() &&
        !replaceOrderedRouteWithEstimatedDrivableBoundaries(
          edges, ordered, options))
      {
        throw std::invalid_argument(
                "unbranched full-map swept-envelope construction failed");
      }
    };

  // Open component heads first, preserving the serialized Edge order. Any
  // remaining component is an explicitly closed cycle and is anchored by its
  // first serialized Edge. Every eligible Edge is swept exactly once.
  for (const RouteEdge & edge : edges) {
    const auto predecessors = incoming.find(edge.from);
    if (predecessors == incoming.end() || predecessors->second.empty()) {
      sweep_from(edge.id);
    }
  }
  for (const RouteEdge & edge : edges) {
    if (visited.count(edge.id) == 0U) {
      sweep_from(edge.id);
    }
  }
  return visited.size() == edges.size();
}

struct ExperimentalGraphSelection
{
  RouteGraph graph;
  ClosedCourseLanelet2ExportSummary summary;
};

struct SyntheticSupportBuildResult
{
  std::vector<RouteNode> nodes;
  std::vector<SyntheticOpenRoutePlanningSupport> records;
};

double endpointVerticalSlope(
  const std::vector<Vec3> & points, const bool at_tail,
  const Vec2 & directed_tangent)
{
  constexpr double minimum_span_m = 0.50;
  const Vec3 & endpoint = at_tail ? points.back() : points.front();
  double best_span = 0.0;
  double best_slope = 0.0;
  if (at_tail) {
    for (std::size_t index = points.size() - 1U; index > 0U; --index) {
      const Vec3 & sample = points[index - 1U];
      const double span = dot(
        Vec2{endpoint.x - sample.x, endpoint.y - sample.y}, directed_tangent);
      if (span > best_span) {
        best_span = span;
        best_slope = (endpoint.z - sample.z) / span;
      }
      if (span >= minimum_span_m) {break;}
    }
  } else {
    for (std::size_t index = 1U; index < points.size(); ++index) {
      const Vec3 & sample = points[index];
      const double span = dot(
        Vec2{sample.x - endpoint.x, sample.y - endpoint.y}, directed_tangent);
      if (span > best_span) {
        best_span = span;
        best_slope = (sample.z - endpoint.z) / span;
      }
      if (span >= minimum_span_m) {break;}
    }
  }
  return best_span > 1.0e-9 && std::isfinite(best_slope) ? best_slope : 0.0;
}

constexpr double kPlanningSupportSearchStepM = 0.25;
constexpr double kPlanningSupportPathSampleSpacingM = 0.10;

struct SyntheticStagingCandidate
{
  std::vector<Vec3> outward_centerline;
  std::vector<Vec3> route_centerline;
  Vec2 outer_route_tangent{};
  std::string geometry_kind{"straight"};
  std::size_t search_index{0U};
  double planar_length_m{0.0};
  double turn_radius_m{0.0};
  double turn_angle_rad{0.0};
  double straight_length_m{0.0};
  double maximum_curvature_inv_m{0.0};
  double actual_maximum_curvature_inv_m{0.0};
  double raw_overlap_transition_length_m{0.0};
  std::vector<std::uint64_t> nonadjacent_raw_overlap_edge_ids;
  double nonadjacent_raw_overlap_transition_length_m{0.0};
  double required_outer_pose_nonadjacent_raw_centerline_isolation_m{0.0};
  double actual_outer_pose_nonadjacent_raw_centerline_isolation_m{0.0};
  std::size_t outer_pose_nonadjacent_raw_centerline_count{0U};
  std::vector<std::uint64_t>
  outer_pose_nearest_nonadjacent_raw_centerline_edge_ids;
  bool outer_footprint_contained{false};
};

struct SyntheticStagingSearchResult
{
  SyntheticStagingCandidate selected;
  std::size_t candidate_count_tested{0U};
  std::size_t rejected_kinematic_candidates{0U};
  std::size_t rejected_invalid_geometry_candidates{0U};
  std::size_t rejected_outer_raw_overlap_candidates{0U};
  std::size_t rejected_insufficient_outer_pose_isolation_candidates{0U};
  std::size_t rejected_raw_polygon_reentry_candidates{0U};
  std::size_t rejected_nonadjacent_transition_candidates{0U};
  double maximum_length_m{0.0};
};

std::vector<Vec3> densifyPolyline(
  const std::vector<Vec3> & points, const double maximum_step_m)
{
  if (points.size() < 2U || !(maximum_step_m > 0.0)) {
    return points;
  }
  std::vector<Vec3> result{points.front()};
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const Vec3 & first = points[index - 1U];
    const Vec3 & second = points[index];
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(distance2d(first, second) / maximum_step_m)));
    for (std::size_t piece = 1U; piece <= pieces; ++piece) {
      result.push_back(first + (second - first) *
        (static_cast<double>(piece) / static_cast<double>(pieces)));
    }
  }
  return result;
}

double maximumDiscretePolylineCurvature(const std::vector<Vec3> & points)
{
  double maximum = 0.0;
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    const Vec2 first{
      points[index].x - points[index - 1U].x,
      points[index].y - points[index - 1U].y};
    const Vec2 second{
      points[index + 1U].x - points[index].x,
      points[index + 1U].y - points[index].y};
    const Vec2 chord{
      points[index + 1U].x - points[index - 1U].x,
      points[index + 1U].y - points[index - 1U].y};
    const double denominator = norm(first) * norm(second) * norm(chord);
    if (denominator <= 1.0e-12) {
      continue;
    }
    const double twice_area = std::abs(first.x * second.y - first.y * second.x);
    maximum = std::max(maximum, 2.0 * twice_area / denominator);
  }
  return maximum;
}

SyntheticStagingCandidate makeSyntheticStagingCandidate(
  const Vec3 & raw_endpoint, const Vec2 & route_tangent,
  const double vertical_slope, const bool head,
  const double total_length_m, const double turn_radius_m,
  const double turn_angle_rad, const int turn_sign,
  const std::size_t search_index)
{
  SyntheticStagingCandidate result;
  result.search_index = search_index;
  result.planar_length_m = total_length_m;
  result.turn_radius_m = turn_radius_m;
  result.turn_angle_rad = turn_angle_rad;
  const double arc_length_m = turn_radius_m * turn_angle_rad;
  result.straight_length_m = total_length_m - arc_length_m;
  result.maximum_curvature_inv_m =
    turn_angle_rad > 1.0e-12 ? 1.0 / turn_radius_m : 0.0;
  if (turn_sign < 0) {
    result.geometry_kind = "right_arc_straight";
  } else if (turn_sign > 0) {
    result.geometry_kind = "left_arc_straight";
  }

  const Vec2 initial_outward = head ?
    Vec2{-route_tangent.x, -route_tangent.y} : route_tangent;
  const double initial_heading = std::atan2(initial_outward.y, initial_outward.x);
  const double signed_curvature = turn_angle_rad > 1.0e-12 ?
    static_cast<double>(turn_sign) / turn_radius_m : 0.0;
  const double final_heading = initial_heading +
    static_cast<double>(turn_sign) * turn_angle_rad;
  const Vec2 final_outward{std::cos(final_heading), std::sin(final_heading)};
  const std::size_t pieces = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(total_length_m / kPlanningSupportPathSampleSpacingM)));
  result.outward_centerline.reserve(pieces + 1U);
  for (std::size_t piece = 0U; piece <= pieces; ++piece) {
    const double arc = total_length_m * static_cast<double>(piece) /
      static_cast<double>(pieces);
    Vec3 point = raw_endpoint;
    if (turn_angle_rad <= 1.0e-12) {
      point.x += initial_outward.x * arc;
      point.y += initial_outward.y * arc;
    } else if (arc <= arc_length_m || result.straight_length_m <= 1.0e-12) {
        const double heading = initial_heading + signed_curvature * arc;
        point.x += (std::sin(heading) - std::sin(initial_heading)) /
          signed_curvature;
        point.y += -(std::cos(heading) - std::cos(initial_heading)) /
          signed_curvature;
    } else {
      const double arc_heading = initial_heading +
        static_cast<double>(turn_sign) * turn_angle_rad;
      point.x += (std::sin(arc_heading) - std::sin(initial_heading)) /
        signed_curvature;
      point.y += -(std::cos(arc_heading) - std::cos(initial_heading)) /
        signed_curvature;
      point.x += final_outward.x * (arc - arc_length_m);
      point.y += final_outward.y * (arc - arc_length_m);
    }
    point.z += (head ? -vertical_slope : vertical_slope) * arc;
    result.outward_centerline.push_back(point);
  }
  // Keep the serialized support on the same audited 0.10 m sampling used by
  // the collision and transition checks.  Simplifying an analytic arc into
  // long chords and then subdividing those chords concentrates the heading
  // change at the retained vertices: the discrete-curvature audit can report
  // roughly three times the requested curvature even though the generated
  // arc itself satisfies the configured turning radius.  A two-point straight
  // also contradicts the exported path-sample-spacing contract.  The dense
  // outward polyline avoids both representation errors without changing any
  // raw replay Edge or its coverage.
  result.route_centerline = result.outward_centerline;
  if (head) {
    std::reverse(result.route_centerline.begin(), result.route_centerline.end());
    result.outer_route_tangent = {-final_outward.x, -final_outward.y};
  } else {
    result.outer_route_tangent = final_outward;
  }
  return result;
}

bool candidateHasSingleRawPolygonTransition(
  SyntheticStagingCandidate & candidate, const bool head,
  const std::uint64_t adjacent_raw_edge_id,
  const ClosedCourseLanelet2ExportOptions & options,
  const std::vector<RoadPolygon2d> & raw_roads)
{
  bool cleared_raw_polygons = false;
  std::set<std::uint64_t> nonadjacent;
  candidate.raw_overlap_transition_length_m = 0.0;
  candidate.nonadjacent_raw_overlap_transition_length_m = 0.0;
  double arc = 0.0;
  for (std::size_t index = 0U; index < candidate.outward_centerline.size(); ++index) {
    const Vec3 & center = candidate.outward_centerline[index];
    if (index > 0U) {
      arc += distance2d(
        candidate.outward_centerline[index - 1U], center);
    }
    const Vec2 outward_tangent = stablePolylineTangentOverSpan(
      candidate.outward_centerline, index, 0.25);
    if (norm(outward_tangent) <= 1.0e-12) {
      return false;
    }
    const Vec2 route_direction = head ?
      Vec2{-outward_tangent.x, -outward_tangent.y} : outward_tangent;
    const std::vector<std::uint64_t> overlaps = overlappingRoadPolygonIds(
      vehicleFootprintPolygon(center, route_direction, options), raw_roads);
    if (overlaps.empty()) {
      cleared_raw_polygons = true;
      continue;
    }
    if (cleared_raw_polygons) {
      return false;
    }
    candidate.raw_overlap_transition_length_m = arc;
    for (const std::uint64_t edge_id : overlaps) {
      if (edge_id != adjacent_raw_edge_id) {
        nonadjacent.insert(edge_id);
        candidate.nonadjacent_raw_overlap_transition_length_m = arc;
      }
    }
  }
  candidate.nonadjacent_raw_overlap_edge_ids.assign(
    nonadjacent.begin(), nonadjacent.end());
  return cleared_raw_polygons;
}

std::vector<SyntheticStagingSearchResult> searchSyntheticStagingCandidates(
  const Vec3 & raw_endpoint, const Vec2 & route_tangent,
  const double vertical_slope, const bool head,
  const std::uint64_t adjacent_raw_edge_id,
  const std::uint64_t support_edge_id,
  const ClosedCourseLanelet2ExportOptions & options,
  const std::vector<RoadPolygon2d> & raw_roads,
  const std::vector<RouteEdge> & raw_centerline_edges,
  const double maximum_length_m, const std::size_t valid_candidate_limit)
{
  std::vector<SyntheticStagingSearchResult> valid_candidates;
  SyntheticStagingSearchResult result;
  result.maximum_length_m = maximum_length_m;
  const std::vector<double> radius_multipliers{1.0, 1.5, 2.0};
  const std::vector<double> turn_angles{
    15.0 * kPi / 180.0, 30.0 * kPi / 180.0,
    45.0 * kPi / 180.0, 60.0 * kPi / 180.0,
    75.0 * kPi / 180.0, 90.0 * kPi / 180.0,
    120.0 * kPi / 180.0, 150.0 * kPi / 180.0};
  std::size_t search_index = 0U;
  const std::size_t length_steps = static_cast<std::size_t>(std::ceil(
      (maximum_length_m - options.planning_endpoint_allowance) /
      kPlanningSupportSearchStepM));
  for (std::size_t length_step = 0U; length_step <= length_steps; ++length_step) {
    const double total_length_m = std::min(
      maximum_length_m,
      options.planning_endpoint_allowance +
      static_cast<double>(length_step) * kPlanningSupportSearchStepM);
    struct Family
    {
      double radius_m;
      double angle_rad;
      int turn_sign;
    };
    std::vector<Family> families{{0.0, 0.0, 0}};
    for (const double radius_multiplier : radius_multipliers) {
      for (const double angle : turn_angles) {
        families.push_back({
          options.estimated_minimum_turning_radius * radius_multiplier,
          angle, 1});
        families.push_back({
          options.estimated_minimum_turning_radius * radius_multiplier,
          angle, -1});
      }
    }
    for (const Family & family : families) {
      ++result.candidate_count_tested;
      const double arc_length_m = family.radius_m * family.angle_rad;
      if (family.angle_rad > 0.0 &&
        (!(family.radius_m >= options.estimated_minimum_turning_radius) ||
        arc_length_m > total_length_m + 1.0e-12))
      {
        ++result.rejected_kinematic_candidates;
        ++search_index;
        continue;
      }
      SyntheticStagingCandidate candidate = makeSyntheticStagingCandidate(
        raw_endpoint, route_tangent, vertical_slope, head, total_length_m,
        family.radius_m, family.angle_rad, family.turn_sign, search_index);
      candidate.actual_maximum_curvature_inv_m =
        maximumDiscretePolylineCurvature(candidate.route_centerline);
      if (!std::isfinite(candidate.actual_maximum_curvature_inv_m) ||
        candidate.actual_maximum_curvature_inv_m >
        1.0 / options.estimated_minimum_turning_radius + 1.0e-6)
      {
        ++result.rejected_kinematic_candidates;
        ++search_index;
        continue;
      }
      const Vec3 & outer = candidate.outward_centerline.back();
      const std::vector<Vec3> outer_footprint = vehicleFootprintPolygon(
        outer, candidate.outer_route_tangent, options);
      if (!containingRoadPolygonIds(outer, raw_roads).empty() ||
        !overlappingRoadPolygonIds(outer_footprint, raw_roads).empty())
      {
        ++result.rejected_outer_raw_overlap_candidates;
        ++search_index;
        continue;
      }
      candidate.required_outer_pose_nonadjacent_raw_centerline_isolation_m =
        requiredOuterPoseNonadjacentRawCenterlineIsolation(options);
      const NonadjacentRawCenterlineIsolation isolation =
        auditNonadjacentRawCenterlineIsolation(
        outer, adjacent_raw_edge_id, raw_centerline_edges);
      candidate.outer_pose_nonadjacent_raw_centerline_count =
        isolation.centerline_count;
      candidate.outer_pose_nearest_nonadjacent_raw_centerline_edge_ids =
        isolation.nearest_edge_ids;
      candidate.actual_outer_pose_nonadjacent_raw_centerline_isolation_m =
        isolation.centerline_count == 0U ?
        candidate.required_outer_pose_nonadjacent_raw_centerline_isolation_m :
        isolation.distance_m;
      if (!std::isfinite(
          candidate.actual_outer_pose_nonadjacent_raw_centerline_isolation_m) ||
        candidate.actual_outer_pose_nonadjacent_raw_centerline_isolation_m + 1.0e-9 <
        candidate.required_outer_pose_nonadjacent_raw_centerline_isolation_m)
      {
        ++result.rejected_insufficient_outer_pose_isolation_candidates;
        ++search_index;
        continue;
      }
      if (!candidateHasSingleRawPolygonTransition(
          candidate, head, adjacent_raw_edge_id, options, raw_roads))
      {
        ++result.rejected_raw_polygon_reentry_candidates;
        ++search_index;
        continue;
      }
      const double maximum_nonadjacent_transition_m =
        options.estimated_front_extent + options.estimated_rear_extent +
        2.0 * options.estimated_minimum_turning_radius;
      if (candidate.nonadjacent_raw_overlap_transition_length_m >
        maximum_nonadjacent_transition_m + 1.0e-9)
      {
        ++result.rejected_nonadjacent_transition_candidates;
        ++search_index;
        continue;
      }
      RouteEdge support;
      support.id = support_edge_id;
      support.centerline = candidate.route_centerline;
      if (!replaceWithEstimatedDrivableBoundaries(support, options, false)) {
        ++result.rejected_invalid_geometry_candidates;
        ++search_index;
        continue;
      }
      const RoadPolygon2d support_polygon = roadPolygonForEdge(support);
      candidate.outer_footprint_contained =
        pointInRoadPolygonInclusive(outer, support_polygon) &&
        polygonContainedInRoadPolygon(outer_footprint, support_polygon);
      if (!candidate.outer_footprint_contained) {
        ++result.rejected_invalid_geometry_candidates;
        ++search_index;
        continue;
      }
      result.selected = std::move(candidate);
      valid_candidates.push_back(result);
      if (valid_candidates.size() >= valid_candidate_limit) {
        return valid_candidates;
      }
      ++search_index;
    }
  }
  if (valid_candidates.empty()) {
    throw std::invalid_argument(
            std::string("open-route planning support found no deterministic ") +
            (head ? "head" : "tail") +
            " kinematic staging candidate with a unique outer vehicle pose; tested=" +
            std::to_string(result.candidate_count_tested) +
            ", rejected_kinematic=" +
            std::to_string(result.rejected_kinematic_candidates) +
            ", rejected_invalid_geometry=" +
            std::to_string(result.rejected_invalid_geometry_candidates) +
            ", rejected_outer_raw_overlap=" +
            std::to_string(result.rejected_outer_raw_overlap_candidates) +
            ", rejected_insufficient_outer_pose_isolation=" +
            std::to_string(
              result.rejected_insufficient_outer_pose_isolation_candidates) +
            ", rejected_raw_polygon_reentry=" +
            std::to_string(result.rejected_raw_polygon_reentry_candidates) +
            ", rejected_nonadjacent_transition=" +
            std::to_string(result.rejected_nonadjacent_transition_candidates));
  }
  return valid_candidates;
}

SyntheticSupportBuildResult appendSyntheticOpenRoutePlanningSupport(
  std::vector<RouteEdge> & edges,
  const RouteGraph & graph,
  const ClosedCourseLanelet2ExportOptions & options,
  const NamedNavigationRoute * named_route,
  const RouteGraph * semantic_source_graph,
  const std::vector<SemanticRouteEdgeProvenance> * semantic_provenance)
{
  SyntheticSupportBuildResult result;
  if (!options.test_only_add_open_route_planning_support) {
    return result;
  }
  if (!(options.planning_endpoint_allowance > 0.0) ||
    !std::isfinite(options.planning_endpoint_allowance))
  {
    throw std::invalid_argument(
            "open-route planning endpoint allowance must be finite and positive");
  }
  if (!(options.estimated_minimum_turning_radius > 0.0) ||
    !std::isfinite(options.estimated_minimum_turning_radius))
  {
    throw std::invalid_argument(
            "open-route planning support requires a finite positive minimum turning radius");
  }
  if (edges.empty()) {
    throw std::invalid_argument(
            "open-route planning support requires a non-empty complete replay");
  }
  std::vector<std::uint64_t> complete_replay_order;
  complete_replay_order.reserve(edges.size());
  if (named_route == nullptr) {
    for (const RouteEdge & edge : edges) {
      complete_replay_order.push_back(edge.id);
    }
  } else {
    complete_replay_order = named_route->ordered_edge_ids;
  }
  if (complete_replay_order.size() != edges.size()) {
    throw std::invalid_argument(
            "open-route planning support requires every exported raw Lanelet segment");
  }
  for (std::size_t index = 0U; index < edges.size(); ++index) {
    if (edges[index].id != complete_replay_order[index]) {
      throw std::invalid_argument(
              "open-route planning support Mission does not preserve full map order");
    }
    if (index > 0U && edges[index - 1U].to != edges[index].from) {
      throw std::invalid_argument(
              "open-route planning support requires one directed unbranched chain");
    }
  }
  if (edges.front().from == edges.back().to) {
    // A closed lap already supplies predecessor/successor planning context.
    return result;
  }
  if (named_route != nullptr &&
    (named_route->start_node_id != edges.front().from ||
    named_route->end_node_id != edges.back().to))
  {
    throw std::invalid_argument(
            "open-route planning support Mission endpoints differ from the full chain");
  }

  std::map<std::uint64_t, const RouteNode *> nodes_by_id;
  std::uint64_t maximum_id = 0U;
  for (const RouteNode & node : graph.nodes) {
    if (!nodes_by_id.emplace(node.id, &node).second) {
      throw std::invalid_argument("open-route planning support found duplicate Node IDs");
    }
    maximum_id = std::max(maximum_id, node.id);
  }
  for (const RouteEdge & edge : graph.edges) {
    maximum_id = std::max(maximum_id, edge.id);
  }
  if (maximum_id > std::numeric_limits<std::uint64_t>::max() - 4U) {
    throw std::invalid_argument("open-route planning support ID space is exhausted");
  }
  const auto first_node = nodes_by_id.find(edges.front().from);
  const auto last_node = nodes_by_id.find(edges.back().to);
  if (first_node == nodes_by_id.end() || last_node == nodes_by_id.end()) {
    throw std::invalid_argument("open-route planning support endpoint Node is missing");
  }
  if (distance3d(first_node->second->position, edges.front().centerline.front()) > 1.0e-6 ||
    distance3d(last_node->second->position, edges.back().centerline.back()) > 1.0e-6)
  {
    throw std::invalid_argument(
            "open-route planning support endpoint Node geometry is inconsistent");
  }

  std::map<std::uint64_t, SemanticRouteEdgeProvenance> provenance_by_output;
  std::map<std::uint64_t, double> source_lengths;
  if ((semantic_source_graph == nullptr) != (semantic_provenance == nullptr)) {
    throw std::invalid_argument(
            "open-route planning support semantic lineage is incomplete");
  }
  if (semantic_source_graph != nullptr) {
    for (const RouteEdge & source : semantic_source_graph->edges) {
      source_lengths.emplace(source.id, polylineLength(source.centerline));
    }
    for (const SemanticRouteEdgeProvenance & provenance : *semantic_provenance) {
      provenance_by_output.emplace(provenance.edge_id, provenance);
    }
  }

  auto source_lineage = [&](const RouteEdge & adjacent, const bool tail) {
      std::uint64_t source_id = adjacent.id;
      double source_length = polylineLength(adjacent.centerline);
      double source_s = tail ? source_length : 0.0;
      const auto provenance = provenance_by_output.find(adjacent.id);
      if (provenance != provenance_by_output.end()) {
        source_id = provenance->second.source_edge_id;
        const auto length = source_lengths.find(source_id);
        if (length == source_lengths.end()) {
          throw std::invalid_argument(
                  "open-route planning support source Edge length is unavailable");
        }
        source_length = length->second;
        source_s = tail ? provenance->second.source_end_s : provenance->second.source_start_s;
      }
      return std::tuple<std::uint64_t, double, double>{source_id, source_length, source_s};
    };

  constexpr double tangent_span_m = 0.50;
  const Vec2 head_tangent = stablePolylineTangentOverSpan(
    edges.front().centerline, 0U, tangent_span_m);
  const Vec2 tail_tangent = stablePolylineTangentOverSpan(
    edges.back().centerline, edges.back().centerline.size() - 1U, tangent_span_m);
  if (norm(head_tangent) <= 1.0e-12 || norm(tail_tangent) <= 1.0e-12) {
    throw std::invalid_argument(
            "open-route planning support endpoint tangent is degenerate");
  }
  const double head_slope = endpointVerticalSlope(
    edges.front().centerline, false, head_tangent);
  const double tail_slope = endpointVerticalSlope(
    edges.back().centerline, true, tail_tangent);
  const Vec3 raw_head = edges.front().centerline.front();
  const Vec3 raw_tail = edges.back().centerline.back();

  const std::uint64_t head_node_id = maximum_id + 1U;
  const std::uint64_t tail_node_id = maximum_id + 2U;
  const std::uint64_t head_edge_id = maximum_id + 3U;
  const std::uint64_t tail_edge_id = maximum_id + 4U;

  // Build an immutable raw-only swept-envelope reference for candidate
  // rejection.  The selected support is added only after this copy has been
  // constructed, so no search candidate can feed back into raw Edge geometry
  // or raw coverage.  The raw-only terminal caps are conservative relative to
  // the final connected chain.
  std::vector<RouteEdge> raw_boundary_edges = edges;
  if (!replaceOrderedRouteWithEstimatedDrivableBoundaries(
      raw_boundary_edges, complete_replay_order, options))
  {
    throw std::invalid_argument(
            "open-route planning support could not construct raw reference polygons");
  }
  std::vector<RoadPolygon2d> raw_roads;
  raw_roads.reserve(raw_boundary_edges.size());
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const RouteEdge & raw_edge : raw_boundary_edges) {
    RoadPolygon2d polygon = roadPolygonForEdge(raw_edge);
    minimum_x = std::min(minimum_x, polygon.minimum_x);
    maximum_x = std::max(maximum_x, polygon.maximum_x);
    minimum_y = std::min(minimum_y, polygon.minimum_y);
    maximum_y = std::max(maximum_y, polygon.maximum_y);
    raw_roads.push_back(std::move(polygon));
  }
  const double raw_bbox_diagonal = std::hypot(
    maximum_x - minimum_x, maximum_y - minimum_y);
  const double unrounded_maximum_length = options.planning_endpoint_allowance +
    raw_bbox_diagonal + 2.0 * (
    options.estimated_front_extent + options.estimated_rear_extent +
    options.estimated_vehicle_width + options.lateral_clearance_margin +
    kPi * options.estimated_minimum_turning_radius);
  const double maximum_search_length = options.planning_endpoint_allowance +
    std::ceil(
    (unrounded_maximum_length - options.planning_endpoint_allowance) /
    kPlanningSupportSearchStepM) * kPlanningSupportSearchStepM;
  const RouteEdge raw_first = edges.front();
  const RouteEdge raw_last = edges.back();
  auto make_support_edge = [&](const std::uint64_t id, const std::uint64_t from,
      const std::uint64_t to, const std::vector<Vec3> & centerline,
      const RouteEdge & adjacent) {
      RouteEdge support;
      support.id = id;
      support.from = from;
      support.to = to;
      support.centerline = centerline;
      support.length = polylineLength(centerline);
      support.minimum_safe_width = options.estimated_vehicle_width +
        2.0 * options.lateral_clearance_margin;
      support.maximum_curvature = adjacent.maximum_curvature;
      support.confidence = 0.0;
      support.recommended_speed_mps = adjacent.recommended_speed_mps;
      support.passable = true;
      support.corridor_geometry_valid = true;
      return support;
    };
  constexpr std::size_t candidate_pool_limit = 512U;
  constexpr std::size_t candidate_pair_evaluation_limit = 512U;
  const std::vector<SyntheticStagingSearchResult> head_candidates =
    searchSyntheticStagingCandidates(
    raw_head, head_tangent, head_slope, true, raw_first.id,
    head_edge_id, options, raw_roads, raw_boundary_edges, maximum_search_length,
    candidate_pool_limit);
  const std::vector<SyntheticStagingSearchResult> tail_candidates =
    searchSyntheticStagingCandidates(
    raw_tail, tail_tangent, tail_slope, false, raw_last.id,
    tail_edge_id, options, raw_roads, raw_boundary_edges, maximum_search_length,
    candidate_pool_limit);

  struct PairAudit
  {
    std::size_t tested{0U};
    std::size_t selected_rank{0U};
    std::size_t rejected_boundary{0U};
    std::size_t rejected_outer{0U};
    std::size_t rejected_transition{0U};
    std::size_t rejected_containment{0U};
  } pair_audit;
  enum class PairValidation
  {
    kValid,
    kBoundary,
    kOuter,
    kTransition,
    kContainment
  };
  const auto validate_pair = [&](const SyntheticStagingSearchResult & head,
      const SyntheticStagingSearchResult & tail) {
      RouteEdge trial_head = make_support_edge(
        head_edge_id, head_node_id, raw_first.from,
        head.selected.route_centerline, raw_first);
      trial_head.maximum_curvature = head.selected.maximum_curvature_inv_m;
      RouteEdge trial_tail = make_support_edge(
        tail_edge_id, raw_last.to, tail_node_id,
        tail.selected.route_centerline, raw_last);
      trial_tail.maximum_curvature = tail.selected.maximum_curvature_inv_m;
      std::vector<RouteEdge> trial_edges;
      trial_edges.reserve(edges.size() + 2U);
      trial_edges.push_back(std::move(trial_head));
      trial_edges.insert(trial_edges.end(), edges.begin(), edges.end());
      trial_edges.push_back(std::move(trial_tail));
      std::vector<std::uint64_t> trial_order{head_edge_id};
      trial_order.insert(
        trial_order.end(), complete_replay_order.begin(),
        complete_replay_order.end());
      trial_order.push_back(tail_edge_id);
      try {
        if (!replaceOrderedRouteWithEstimatedDrivableBoundaries(
            trial_edges, trial_order, options))
        {
          return PairValidation::kBoundary;
        }
      } catch (const std::invalid_argument &) {
        return PairValidation::kBoundary;
      }
      std::vector<RoadPolygon2d> trial_all_roads;
      std::vector<RoadPolygon2d> trial_raw_roads;
      for (const RouteEdge & trial_edge : trial_edges) {
        const RoadPolygon2d polygon = roadPolygonForEdge(trial_edge);
        trial_all_roads.push_back(polygon);
        if (trial_edge.id != head_edge_id && trial_edge.id != tail_edge_id) {
          trial_raw_roads.push_back(polygon);
        }
      }
      const auto validate_role = [&](const SyntheticStagingSearchResult & candidate,
          const std::uint64_t support_id, const std::uint64_t adjacent_id,
          const bool head_role) {
          const auto support = std::find_if(
            trial_edges.begin(), trial_edges.end(),
            [&](const RouteEdge & edge) {return edge.id == support_id;});
          if (support == trial_edges.end() ||
            !estimatedBoundaryPolygonIsValid(*support))
          {
            return PairValidation::kBoundary;
          }
          const Vec3 outer = candidate.selected.outward_centerline.back();
          if (containingRoadPolygonIds(outer, trial_all_roads) !=
            std::vector<std::uint64_t>{support_id})
          {
            return PairValidation::kOuter;
          }
          const std::vector<Vec3> outer_footprint = vehicleFootprintPolygon(
            outer, candidate.selected.outer_route_tangent, options);
          if (overlappingRoadPolygonIds(outer_footprint, trial_all_roads) !=
            std::vector<std::uint64_t>{support_id} ||
            !polygonContainedInRoadPolygon(
              outer_footprint, roadPolygonForEdge(*support)))
          {
            return PairValidation::kOuter;
          }
          SyntheticStagingCandidate final_candidate = candidate.selected;
          final_candidate.route_centerline = support->centerline;
          final_candidate.outward_centerline = support->centerline;
          if (head_role) {
            std::reverse(
              final_candidate.outward_centerline.begin(),
              final_candidate.outward_centerline.end());
          }
          final_candidate.outward_centerline = densifyPolyline(
            final_candidate.outward_centerline,
            kPlanningSupportPathSampleSpacingM);
          final_candidate.planar_length_m = polylineLength2d(support->centerline);
          if (!candidateHasSingleRawPolygonTransition(
              final_candidate, head_role, adjacent_id, options, trial_raw_roads) ||
            final_candidate.nonadjacent_raw_overlap_transition_length_m >
            options.estimated_front_extent + options.estimated_rear_extent +
            2.0 * options.estimated_minimum_turning_radius + 1.0e-9)
          {
            return PairValidation::kTransition;
          }
          for (std::size_t index = 0U; index < support->centerline.size(); ++index) {
            const Vec2 direction = stablePolylineTangentOverSpan(
              support->centerline, index, 0.25);
            if (norm(direction) <= 1.0e-12 ||
              !polygonContainedInAnyRoadPolygon(
                vehicleFootprintPolygon(
                  support->centerline[index], direction, options),
                trial_all_roads))
            {
              return PairValidation::kContainment;
            }
          }
          return PairValidation::kValid;
        };
      const PairValidation head_validation = validate_role(
        head, head_edge_id, raw_first.id, true);
      if (head_validation != PairValidation::kValid) {
        return head_validation;
      }
      return validate_role(tail, tail_edge_id, raw_last.id, false);
    };

  struct CandidatePair
  {
    std::size_t head{0U};
    std::size_t tail{0U};
    double total_length_m{0.0};
  };
  const auto pair_compare = [&](const CandidatePair & lhs, const CandidatePair & rhs) {
      if (std::abs(lhs.total_length_m - rhs.total_length_m) > 1.0e-12) {
        return lhs.total_length_m > rhs.total_length_m;
      }
      if (head_candidates[lhs.head].selected.search_index !=
        head_candidates[rhs.head].selected.search_index)
      {
        return head_candidates[lhs.head].selected.search_index >
               head_candidates[rhs.head].selected.search_index;
      }
      return tail_candidates[lhs.tail].selected.search_index >
             tail_candidates[rhs.tail].selected.search_index;
    };
  std::priority_queue<
    CandidatePair, std::vector<CandidatePair>, decltype(pair_compare)> queue(pair_compare);
  std::set<std::pair<std::size_t, std::size_t>> queued;
  const auto enqueue = [&](const std::size_t head, const std::size_t tail) {
      if (head >= head_candidates.size() || tail >= tail_candidates.size() ||
        !queued.emplace(head, tail).second)
      {
        return;
      }
      queue.push({
        head, tail,
        head_candidates[head].selected.planar_length_m +
        tail_candidates[tail].selected.planar_length_m});
    };
  enqueue(0U, 0U);
  std::optional<std::pair<std::size_t, std::size_t>> selected_pair;
  while (!queue.empty() &&
    pair_audit.tested < candidate_pair_evaluation_limit)
  {
    const CandidatePair candidate = queue.top();
    queue.pop();
    ++pair_audit.tested;
    const PairValidation validation = validate_pair(
      head_candidates[candidate.head], tail_candidates[candidate.tail]);
    if (validation == PairValidation::kValid) {
      selected_pair = std::make_pair(candidate.head, candidate.tail);
      pair_audit.selected_rank = pair_audit.tested;
      break;
    }
    if (validation == PairValidation::kBoundary) {
      ++pair_audit.rejected_boundary;
    } else if (validation == PairValidation::kOuter) {
      ++pair_audit.rejected_outer;
    } else if (validation == PairValidation::kTransition) {
      ++pair_audit.rejected_transition;
    } else {
      ++pair_audit.rejected_containment;
    }
    enqueue(candidate.head + 1U, candidate.tail);
    enqueue(candidate.head, candidate.tail + 1U);
  }
  if (!selected_pair) {
    throw std::invalid_argument(
            "open-route planning support found no valid final combined head/tail "
            "candidate pair within the deterministic bound; tested=" +
            std::to_string(pair_audit.tested) + ", limit=" +
            std::to_string(candidate_pair_evaluation_limit) +
            ", head_pool=" + std::to_string(head_candidates.size()) +
            ", tail_pool=" + std::to_string(tail_candidates.size()) +
            ", rejected_boundary=" +
            std::to_string(pair_audit.rejected_boundary) +
            ", rejected_outer=" + std::to_string(pair_audit.rejected_outer) +
            ", rejected_transition=" +
            std::to_string(pair_audit.rejected_transition) +
            ", rejected_containment=" +
            std::to_string(pair_audit.rejected_containment));
  }
  const SyntheticStagingSearchResult & head_search =
    head_candidates[selected_pair->first];
  const SyntheticStagingSearchResult & tail_search =
    tail_candidates[selected_pair->second];
  const Vec3 synthetic_head = head_search.selected.outward_centerline.back();
  const Vec3 synthetic_tail = tail_search.selected.outward_centerline.back();
  result.nodes = {
    {head_node_id, synthetic_head, RouteNodeType::kEndpoint},
    {tail_node_id, synthetic_tail, RouteNodeType::kEndpoint}};

  RouteEdge head_edge = make_support_edge(
    head_edge_id, head_node_id, raw_first.from,
    head_search.selected.route_centerline, raw_first);
  head_edge.maximum_curvature = head_search.selected.maximum_curvature_inv_m;
  RouteEdge tail_edge = make_support_edge(
    tail_edge_id, raw_last.to, tail_node_id,
    tail_search.selected.route_centerline, raw_last);
  tail_edge.maximum_curvature = tail_search.selected.maximum_curvature_inv_m;

  const auto [head_source_id, head_source_length, head_source_s] =
    source_lineage(raw_first, false);
  const auto [tail_source_id, tail_source_length, tail_source_s] =
    source_lineage(raw_last, true);
  const auto make_record = [&](const RouteEdge & support_edge,
      const RouteEdge & adjacent_edge, const std::uint64_t source_id,
      const double source_length, const double source_s, const bool head,
      const SyntheticStagingSearchResult & search) {
      SyntheticOpenRoutePlanningSupport record;
      record.edge_id = support_edge.id;
      record.adjacent_output_edge_id = adjacent_edge.id;
      record.adjacent_source_edge_id = source_id;
      record.raw_endpoint_node_id = head ? adjacent_edge.from : adjacent_edge.to;
      record.role = head ? "head" : "tail";
      record.raw_endpoint = head ? raw_head : raw_tail;
      record.synthetic_endpoint = search.selected.outward_centerline.back();
      record.directed_tangent = head_tangent;
      if (!head) {record.directed_tangent = tail_tangent;}
      record.outer_directed_tangent = search.selected.outer_route_tangent;
      record.source_edge_length_m = source_length;
      record.raw_endpoint_s_m = source_s;
      record.centerline_planar_length_m = polylineLength2d(support_edge.centerline);
      record.centerline_3d_length_m = polylineLength(support_edge.centerline);
      record.required_boundary_beyond_raw_endpoint_m =
        (head ? options.estimated_rear_extent : options.estimated_front_extent) +
        options.planning_endpoint_allowance;
      record.geometry_kind = search.selected.geometry_kind;
      record.selected_candidate_index = search.selected.search_index;
      record.candidate_count_tested = search.candidate_count_tested;
      record.individually_valid_candidate_rank =
        (head ? selected_pair->first : selected_pair->second) + 1U;
      record.rejected_kinematic_candidates = search.rejected_kinematic_candidates;
      record.rejected_invalid_geometry_candidates =
        search.rejected_invalid_geometry_candidates;
      record.rejected_outer_raw_overlap_candidates =
        search.rejected_outer_raw_overlap_candidates;
      record.rejected_insufficient_outer_pose_isolation_candidates =
        search.rejected_insufficient_outer_pose_isolation_candidates;
      record.rejected_raw_polygon_reentry_candidates =
        search.rejected_raw_polygon_reentry_candidates;
      record.rejected_nonadjacent_transition_candidates =
        search.rejected_nonadjacent_transition_candidates;
      record.search_step_m = kPlanningSupportSearchStepM;
      record.search_max_length_m = search.maximum_length_m;
      record.turn_radius_m = search.selected.turn_radius_m;
      record.turn_angle_rad = search.selected.turn_angle_rad;
      record.straight_length_m = search.selected.straight_length_m;
      record.maximum_curvature_inv_m = search.selected.maximum_curvature_inv_m;
      record.actual_maximum_curvature_inv_m =
        search.selected.actual_maximum_curvature_inv_m;
      record.kinematic_valid =
        record.maximum_curvature_inv_m <=
        1.0 / options.estimated_minimum_turning_radius + 1.0e-9;
      record.raw_overlap_single_transition = true;
      record.raw_overlap_transition_length_m =
        search.selected.raw_overlap_transition_length_m;
      record.nonadjacent_raw_overlap_edge_ids =
        search.selected.nonadjacent_raw_overlap_edge_ids;
      record.nonadjacent_raw_overlap_transition_length_m =
        search.selected.nonadjacent_raw_overlap_transition_length_m;
      record.required_outer_pose_nonadjacent_raw_centerline_isolation_m =
        search.selected.required_outer_pose_nonadjacent_raw_centerline_isolation_m;
      record.actual_outer_pose_nonadjacent_raw_centerline_isolation_m =
        search.selected.actual_outer_pose_nonadjacent_raw_centerline_isolation_m;
      record.outer_pose_nonadjacent_raw_centerline_count =
        search.selected.outer_pose_nonadjacent_raw_centerline_count;
      record.outer_pose_nearest_nonadjacent_raw_centerline_edge_ids =
        search.selected.outer_pose_nearest_nonadjacent_raw_centerline_edge_ids;
      record.maximum_nonadjacent_raw_overlap_transition_length_m =
        options.estimated_front_extent + options.estimated_rear_extent +
        2.0 * options.estimated_minimum_turning_radius;
      record.outer_footprint_contained = search.selected.outer_footprint_contained;
      record.candidate_pool_limit = candidate_pool_limit;
      record.head_candidate_pool_size = head_candidates.size();
      record.tail_candidate_pool_size = tail_candidates.size();
      record.candidate_pair_evaluation_limit = candidate_pair_evaluation_limit;
      record.candidate_pairs_tested = pair_audit.tested;
      record.selected_candidate_pair_rank = pair_audit.selected_rank;
      record.rejected_final_boundary_pairs = pair_audit.rejected_boundary;
      record.rejected_final_outer_membership_pairs = pair_audit.rejected_outer;
      record.rejected_final_transition_pairs = pair_audit.rejected_transition;
      record.rejected_final_containment_pairs = pair_audit.rejected_containment;
      return record;
    };
  result.records = {
    make_record(
      head_edge, raw_first, head_source_id, head_source_length,
      head_source_s, true, head_search),
    make_record(
      tail_edge, raw_last, tail_source_id, tail_source_length,
      tail_source_s, false, tail_search)};

  std::vector<RouteEdge> with_support;
  with_support.reserve(edges.size() + 2U);
  with_support.push_back(std::move(head_edge));
  with_support.insert(with_support.end(), edges.begin(), edges.end());
  with_support.push_back(std::move(tail_edge));
  edges = std::move(with_support);
  return result;
}

void validateSyntheticOpenRoutePlanningSupportBoundaries(
  const std::vector<RouteEdge> & edges,
  const ClosedCourseLanelet2ExportOptions & options,
  std::vector<SyntheticOpenRoutePlanningSupport> & records)
{
  std::map<std::uint64_t, const RouteEdge *> by_id;
  for (const RouteEdge & edge : edges) {
    by_id.emplace(edge.id, &edge);
  }
  std::set<std::uint64_t> support_edge_ids;
  for (const SyntheticOpenRoutePlanningSupport & record : records) {
    support_edge_ids.insert(record.edge_id);
  }
  std::vector<RoadPolygon2d> all_roads;
  std::vector<RoadPolygon2d> raw_roads;
  std::vector<RouteEdge> raw_centerline_edges;
  all_roads.reserve(edges.size());
  raw_roads.reserve(edges.size());
  for (const RouteEdge & edge : edges) {
    const RoadPolygon2d polygon = roadPolygonForEdge(edge);
    all_roads.push_back(polygon);
    if (support_edge_ids.count(edge.id) == 0U) {
      raw_roads.push_back(polygon);
      raw_centerline_edges.push_back(edge);
    }
  }
  for (SyntheticOpenRoutePlanningSupport & record : records) {
    const auto found = by_id.find(record.edge_id);
    if (found == by_id.end()) {
      throw std::invalid_argument("synthetic planning-support Edge disappeared");
    }
    const RouteEdge & edge = *found->second;
    if (!estimatedBoundaryPolygonIsValid(edge) || edge.centerline.size() < 2U) {
      throw std::invalid_argument("synthetic planning-support polygon is invalid");
    }
    const bool head = record.role == "head";
    if (!head && record.role != "tail") {
      throw std::invalid_argument("synthetic planning-support role is invalid");
    }
    const Vec2 outward = head ?
      Vec2{-record.directed_tangent.x, -record.directed_tangent.y} :
      record.directed_tangent;
    const Vec3 & left = head ? edge.left_boundary.front() : edge.left_boundary.back();
    const Vec3 & right = head ? edge.right_boundary.front() : edge.right_boundary.back();
    record.actual_left_boundary_beyond_raw_endpoint_m = dot(
      Vec2{left.x - record.raw_endpoint.x, left.y - record.raw_endpoint.y}, outward);
    record.actual_right_boundary_beyond_raw_endpoint_m = dot(
      Vec2{right.x - record.raw_endpoint.x, right.y - record.raw_endpoint.y}, outward);
    const double tolerance = 1.0e-9;
    if (record.geometry_kind == "straight" &&
      (record.actual_left_boundary_beyond_raw_endpoint_m + tolerance <
      record.required_boundary_beyond_raw_endpoint_m ||
      record.actual_right_boundary_beyond_raw_endpoint_m + tolerance <
      record.required_boundary_beyond_raw_endpoint_m))
    {
      throw std::invalid_argument(
              "synthetic planning-support boundary does not contain the raw endpoint footprint");
    }

    record.actual_maximum_curvature_inv_m =
      maximumDiscretePolylineCurvature(edge.centerline);
    if (record.planning_support_contract_version != 2U ||
      record.candidate_count_tested != record.selected_candidate_index + 1U ||
      record.individually_valid_candidate_rank == 0U ||
      record.rejected_kinematic_candidates +
      record.rejected_invalid_geometry_candidates +
      record.rejected_outer_raw_overlap_candidates +
      record.rejected_insufficient_outer_pose_isolation_candidates +
      record.rejected_raw_polygon_reentry_candidates +
      record.rejected_nonadjacent_transition_candidates +
      record.individually_valid_candidate_rank - 1U !=
      record.selected_candidate_index ||
      record.candidate_pool_limit == 0U ||
      record.head_candidate_pool_size == 0U ||
      record.tail_candidate_pool_size == 0U ||
      record.candidate_pairs_tested != record.selected_candidate_pair_rank ||
      record.rejected_final_boundary_pairs +
      record.rejected_final_outer_membership_pairs +
      record.rejected_final_transition_pairs +
      record.rejected_final_containment_pairs + 1U !=
      record.selected_candidate_pair_rank ||
      !record.kinematic_valid ||
      record.maximum_curvature_inv_m >
      1.0 / options.estimated_minimum_turning_radius + tolerance ||
      !std::isfinite(record.actual_maximum_curvature_inv_m) ||
      record.actual_maximum_curvature_inv_m >
      1.0 / options.estimated_minimum_turning_radius + 1.0e-6)
    {
      throw std::invalid_argument(
              "synthetic planning-support kinematic search audit is inconsistent");
    }

    const RoadPolygon2d support_polygon = roadPolygonForEdge(edge);
    record.outer_endpoint_route_polygon_edge_ids = containingRoadPolygonIds(
      record.synthetic_endpoint, all_roads);
    record.outer_endpoint_unique =
      record.outer_endpoint_route_polygon_edge_ids ==
      std::vector<std::uint64_t>{record.edge_id};
    const std::vector<Vec3> outer_footprint = vehicleFootprintPolygon(
      record.synthetic_endpoint, record.outer_directed_tangent, options);
    record.outer_footprint_raw_overlap_edge_ids = overlappingRoadPolygonIds(
      outer_footprint, raw_roads);
    const std::vector<std::uint64_t> outer_footprint_all_overlaps =
      overlappingRoadPolygonIds(outer_footprint, all_roads);
    record.outer_footprint_contained =
      polygonContainedInRoadPolygon(outer_footprint, support_polygon);
    if (!record.outer_endpoint_unique ||
      !record.outer_footprint_raw_overlap_edge_ids.empty() ||
      outer_footprint_all_overlaps != std::vector<std::uint64_t>{record.edge_id} ||
      !record.outer_footprint_contained)
    {
      throw std::invalid_argument(
              "synthetic planning-support " + record.role +
              " outer vehicle pose is not uniquely contained; center_memberships=" +
              commaSeparatedIds(record.outer_endpoint_route_polygon_edge_ids) +
              ", raw_footprint_overlaps=" +
              commaSeparatedIds(record.outer_footprint_raw_overlap_edge_ids) +
              ", all_footprint_overlaps=" +
              commaSeparatedIds(outer_footprint_all_overlaps));
    }

    const double expected_isolation =
      requiredOuterPoseNonadjacentRawCenterlineIsolation(options);
    const NonadjacentRawCenterlineIsolation isolation =
      auditNonadjacentRawCenterlineIsolation(
      record.synthetic_endpoint, record.adjacent_output_edge_id,
      raw_centerline_edges);
    const double actual_isolation = isolation.centerline_count == 0U ?
      expected_isolation : isolation.distance_m;
    if (!std::isfinite(actual_isolation) ||
      std::abs(
        record.required_outer_pose_nonadjacent_raw_centerline_isolation_m -
        expected_isolation) > tolerance ||
      std::abs(
        record.actual_outer_pose_nonadjacent_raw_centerline_isolation_m -
        actual_isolation) > tolerance ||
      record.outer_pose_nonadjacent_raw_centerline_count !=
      isolation.centerline_count ||
      record.outer_pose_nearest_nonadjacent_raw_centerline_edge_ids !=
      isolation.nearest_edge_ids ||
      actual_isolation + tolerance < expected_isolation)
    {
      throw std::invalid_argument(
              "synthetic planning-support " + record.role +
              " outer pose is insufficiently isolated from nonadjacent raw centerlines");
    }

    SyntheticStagingCandidate final_candidate;
    final_candidate.route_centerline = edge.centerline;
    final_candidate.outward_centerline = edge.centerline;
    if (head) {
      std::reverse(
        final_candidate.outward_centerline.begin(),
        final_candidate.outward_centerline.end());
    }
    final_candidate.outward_centerline = densifyPolyline(
      final_candidate.outward_centerline, kPlanningSupportPathSampleSpacingM);
    final_candidate.planar_length_m = polylineLength2d(edge.centerline);
    if (!candidateHasSingleRawPolygonTransition(
        final_candidate, head, record.adjacent_output_edge_id, options, raw_roads))
    {
      throw std::invalid_argument(
              "synthetic planning-support path re-enters a raw road polygon");
    }
    record.raw_overlap_single_transition = true;
    record.raw_overlap_transition_length_m =
      final_candidate.raw_overlap_transition_length_m;
    record.nonadjacent_raw_overlap_edge_ids =
      final_candidate.nonadjacent_raw_overlap_edge_ids;
    record.nonadjacent_raw_overlap_transition_length_m =
      final_candidate.nonadjacent_raw_overlap_transition_length_m;
    if (record.maximum_nonadjacent_raw_overlap_transition_length_m !=
      options.estimated_front_extent + options.estimated_rear_extent +
      2.0 * options.estimated_minimum_turning_radius ||
      record.nonadjacent_raw_overlap_transition_length_m >
      record.maximum_nonadjacent_raw_overlap_transition_length_m + tolerance)
    {
      throw std::invalid_argument(
              "synthetic planning-support nonadjacent overlap transition is too long");
    }

    record.connection_footprint_contained = true;
    for (std::size_t index = 0U; index < edge.centerline.size(); ++index) {
      const Vec2 route_direction = stablePolylineTangentOverSpan(
        edge.centerline, index, 0.25);
      if (norm(route_direction) <= 1.0e-12 ||
        !polygonContainedInAnyRoadPolygon(
          vehicleFootprintPolygon(edge.centerline[index], route_direction, options),
          all_roads))
      {
        record.connection_footprint_contained = false;
        break;
      }
    }
    if (!record.connection_footprint_contained) {
      throw std::invalid_argument(
              "synthetic planning-support connection does not contain the vehicle footprint");
    }
  }
}

ExperimentalGraphSelection buildClosedCourseGraphSelection(
  const RouteGraph & graph, const ClosedCourseLanelet2ExportOptions & options,
  const NamedNavigationRoute * named_route,
  const RouteGraph * semantic_source_graph,
  const std::vector<SemanticRouteEdgeProvenance> * semantic_provenance)
{
  ExperimentalGraphSelection selection;
  selection.graph.frame_id = graph.frame_id;

  std::set<std::uint64_t> route_node_ids;
  for (const RouteNode & node : graph.nodes) {
    route_node_ids.insert(node.id);
  }

  std::vector<RouteEdge> eligible_edges;
  eligible_edges.reserve(graph.edges.size());
  std::set<std::uint64_t> authored_edge_ids;
  if (named_route != nullptr) {
    authored_edge_ids.insert(
      named_route->ordered_edge_ids.begin(), named_route->ordered_edge_ids.end());
  }
  for (const RouteEdge & source : graph.edges) {
    // The caller supplies the hard-validated experimental operational graph.
    // Keep the normal physical-edge/reverse selection rule unchanged.
    bool omit_reverse = false;
    if (source.reverse_of) {
      const bool source_is_authored = authored_edge_ids.count(source.id) != 0U;
      const bool reverse_is_authored = authored_edge_ids.count(*source.reverse_of) != 0U;
      omit_reverse = source_is_authored != reverse_is_authored ?
        !source_is_authored : source.id > *source.reverse_of;
    }
    if (!source.passable || omit_reverse ||
      route_node_ids.count(source.from) == 0U || route_node_ids.count(source.to) == 0U)
    {
      continue;
    }
    RouteEdge edge = source;
    // The replay candidate is an audited record of the observed, ordered
    // trajectory.  Its exact samples are part of the map-coverage contract:
    // simplifying/resampling each roughly five-metre Edge here changes both
    // its geometry and length and can manufacture a shorter route.  Build the
    // swept boundary around those exact samples instead.  The composite
    // boundary implementation below uses a bounded-span tangent so it does
    // not need to rewrite the centreline to suppress localisation jitter.
    edge.left_boundary.clear();
    edge.right_boundary.clear();
    edge.length = polylineLength2d(edge.centerline);
    if (!(edge.length > 1.0e-9) || !std::isfinite(edge.length)) {
      continue;
    }
    eligible_edges.push_back(std::move(edge));
  }

  selection.summary.source_physical_edges = eligible_edges.size();
  for (const RouteEdge & edge : eligible_edges) {
    selection.summary.source_length += edge.length;
  }
  SyntheticSupportBuildResult synthetic_support =
    appendSyntheticOpenRoutePlanningSupport(
    eligible_edges, graph, options, named_route,
    semantic_source_graph, semantic_provenance);

  const bool full_components_swept =
    replaceUnbranchedComponentsWithEstimatedDrivableBoundaries(
    eligible_edges, options);
  if (!full_components_swept) {
    std::set<std::uint64_t> jointly_swept_edge_ids;
    if (named_route != nullptr) {
      if (!replaceOrderedRouteWithEstimatedDrivableBoundaries(
          eligible_edges, named_route->ordered_edge_ids, options))
      {
        throw std::invalid_argument(
                "named Route oriented swept-footprint boundary construction failed");
      }
      jointly_swept_edge_ids.insert(
        named_route->ordered_edge_ids.begin(), named_route->ordered_edge_ids.end());
    }
    for (RouteEdge & edge : eligible_edges) {
      if (jointly_swept_edge_ids.count(edge.id) == 0U &&
        !replaceWithEstimatedDrivableBoundaries(edge, options, false))
      {
        throw std::invalid_argument(
                "oriented swept-footprint boundary construction failed for Edge " +
                std::to_string(edge.id));
      }
    }
  }

  validateSyntheticOpenRoutePlanningSupportBoundaries(
    eligible_edges, options, synthetic_support.records);
  selection.summary.synthetic_planning_support = std::move(synthetic_support.records);
  std::set<std::uint64_t> synthetic_edge_ids;
  for (const SyntheticOpenRoutePlanningSupport & support :
    selection.summary.synthetic_planning_support)
  {
    synthetic_edge_ids.insert(support.edge_id);
  }
  std::set<std::uint64_t> selected_node_ids;
  // A Lanelet2 map may legitimately contain multiple route components (for
  // example, either side of an authored no-entry span). Silently choosing only
  // the largest component erased valid measured coverage in earlier releases.
  // Export every eligible physical edge and let the routing graph report
  // reachability for the requested start/goal pair.
  for (RouteEdge & edge : eligible_edges) {
    if (synthetic_edge_ids.count(edge.id) == 0U) {
      selection.summary.exported_length += edge.length;
      ++selection.summary.exported_physical_edges;
      ++selection.summary.exported_lanelet_segments;
    }
    selected_node_ids.insert(edge.from);
    selected_node_ids.insert(edge.to);
    selection.graph.edges.push_back(std::move(edge));
  }
  for (const RouteNode & node : graph.nodes) {
    if (selected_node_ids.count(node.id) != 0U) {
      selection.graph.nodes.push_back(node);
    }
  }
  for (const RouteNode & node : synthetic_support.nodes) {
    selection.graph.nodes.push_back(node);
  }
  return selection;
}

void applyTerminalSupportToSelection(
  ExperimentalGraphSelection & selection,
  const RouteGraph & exact_named_graph,
  const ClosedCourseLanelet2ExportOptions & options,
  const NamedNavigationRoute * named_route,
  const ClosedCourseAutowareTerminalSupport & support)
{
  constexpr double endpoint_tolerance = 1.0e-3;
  if (named_route == nullptr || named_route->ordered_edge_ids.empty()) {
    throw std::invalid_argument(
            "terminal support requires a non-empty named Autoware Route");
  }
  if (support.source != "closed_course_semantic_topology") {
    throw std::invalid_argument("terminal support provenance is not recognized");
  }
  if (support.support_edge_ids.size() != 1U ||
    support.support_edge_ids.front() != support.successor_edge.id)
  {
    throw std::invalid_argument("terminal support must contain exactly one audited successor");
  }
  if (support.named_terminal_edge_id != named_route->ordered_edge_ids.back()) {
    throw std::invalid_argument("terminal support does not match the final named Route Edge");
  }
  if (!(support.terminal_support_length_m > 1.0e-9) ||
    !std::isfinite(support.terminal_support_length_m) ||
    !(support.named_route_source_length_m > 1.0e-9) ||
    !std::isfinite(support.named_route_source_length_m))
  {
    throw std::invalid_argument("terminal support audit lengths are invalid");
  }
  if (!support.successor_edge.passable ||
    support.successor_edge.centerline.size() < 2U ||
    support.successor_end_node.id != support.successor_edge.to ||
    !finite(support.successor_end_node.position))
  {
    throw std::invalid_argument("terminal support successor geometry is invalid");
  }
  const double measured_support_length = polylineLength2d(
    support.successor_edge.centerline);
  if (!(measured_support_length > 1.0e-9) ||
    std::abs(measured_support_length - support.terminal_support_length_m) >
    1.0e-6 * std::max(1.0, measured_support_length))
  {
    throw std::invalid_argument("terminal support length does not match its geometry");
  }
  if (distance3d(
      support.successor_edge.centerline.back(), support.successor_end_node.position) >
    endpoint_tolerance)
  {
    throw std::invalid_argument("terminal support end node does not match its geometry");
  }

  const auto terminal = std::find_if(
    selection.graph.edges.begin(), selection.graph.edges.end(),
    [&](const RouteEdge & edge) {return edge.id == support.named_terminal_edge_id;});
  if (terminal == selection.graph.edges.end()) {
    throw std::invalid_argument("final named Route Edge is not exportable for terminal support");
  }
  const auto exact_terminal = std::find_if(
    exact_named_graph.edges.begin(), exact_named_graph.edges.end(),
    [&](const RouteEdge & edge) {return edge.id == support.named_terminal_edge_id;});
  if (exact_terminal == exact_named_graph.edges.end() ||
    exact_terminal->centerline.size() < 2U)
  {
    throw std::invalid_argument("exact final named Route Edge geometry is unavailable");
  }
  const double exact_named_length = polylineLength2d(exact_terminal->centerline);
  if (!(exact_named_length > 1.0e-9) ||
    std::abs(exact_named_length - support.named_route_source_length_m) >
    1.0e-6 * std::max(1.0, exact_named_length))
  {
    throw std::invalid_argument(
            "terminal support named source length does not match export geometry");
  }
  if (std::any_of(
      selection.graph.edges.begin(), selection.graph.edges.end(),
      [&](const RouteEdge & edge) {
        return edge.id == support.successor_edge.id;
      }))
  {
    throw std::invalid_argument("terminal support successor would be exported twice");
  }
  if (distance3d(
      exact_terminal->centerline.back(), support.successor_edge.centerline.front()) >
    endpoint_tolerance)
  {
    throw std::invalid_argument("terminal support is disconnected from the final Lanelet");
  }

  // Preserve the exact audited source samples for this composite.  The normal
  // closed-course RDP regularization is intentionally bypassed: independently
  // simplifying the final named Edge and its successor changes their summed
  // arc length, making the OSM geometry disagree with its provenance tags.
  std::vector<Vec3> composite = exact_terminal->centerline;
  composite.insert(
    composite.end(), support.successor_edge.centerline.begin() + 1,
    support.successor_edge.centerline.end());
  terminal->centerline = std::move(composite);
  terminal->to = support.successor_edge.to;
  terminal->reverse_of.reset();
  terminal->left_clearance.clear();
  terminal->right_clearance.clear();
  terminal->left_clearance_observed.clear();
  terminal->right_clearance_observed.clear();
  terminal->minimum_safe_width = std::min(
    terminal->minimum_safe_width, support.successor_edge.minimum_safe_width);
  terminal->maximum_curvature = std::max(
    terminal->maximum_curvature, support.successor_edge.maximum_curvature);
  terminal->confidence = std::min(terminal->confidence, support.successor_edge.confidence);
  if (!std::isfinite(terminal->recommended_speed_mps) ||
    !std::isfinite(support.successor_edge.recommended_speed_mps))
  {
    throw std::invalid_argument("terminal support speed metadata is not finite");
  }
  if (support.successor_edge.recommended_speed_mps > 0.0 &&
    (!(terminal->recommended_speed_mps > 0.0) ||
    support.successor_edge.recommended_speed_mps < terminal->recommended_speed_mps))
  {
    // One composite Lanelet can carry only one speed limit.  Preserve the
    // most restrictive positive source value over the named and support
    // geometries; zero still means "use the configured fallback".
    terminal->recommended_speed_mps = support.successor_edge.recommended_speed_mps;
  }
  terminal->corridor_geometry_valid =
    terminal->corridor_geometry_valid && support.successor_edge.corridor_geometry_valid;
  if (!replaceWithEstimatedDrivableBoundaries(*terminal, options, false)) {
    throw std::invalid_argument("terminal support composite boundary construction failed");
  }
  terminal->length = polylineLength2d(terminal->centerline);

  std::set<std::uint64_t> retained_node_ids;
  for (const RouteEdge & edge : selection.graph.edges) {
    retained_node_ids.insert(edge.from);
    retained_node_ids.insert(edge.to);
  }
  selection.graph.nodes.erase(
    std::remove_if(
      selection.graph.nodes.begin(), selection.graph.nodes.end(),
      [&](const RouteNode & node) {
        return retained_node_ids.count(node.id) == 0U;
      }),
    selection.graph.nodes.end());
  const auto existing_end = std::find_if(
    selection.graph.nodes.begin(), selection.graph.nodes.end(),
    [&](const RouteNode & node) {return node.id == support.successor_end_node.id;});
  if (existing_end == selection.graph.nodes.end()) {
    selection.graph.nodes.push_back(support.successor_end_node);
  } else if (distance3d(existing_end->position, support.successor_end_node.position) >
    endpoint_tolerance)
  {
    throw std::invalid_argument("terminal support end node ID has conflicting geometry");
  }

  selection.summary.terminal_support_applied = true;
  selection.summary.terminal_support_named_edge_id = support.named_terminal_edge_id;
  selection.summary.terminal_support_edge_ids = support.support_edge_ids;
  selection.summary.terminal_support_length_m = support.terminal_support_length_m;
  selection.summary.named_route_source_length_m = support.named_route_source_length_m;
}

void writeTumTimestamp(std::ostream & stream, const std::int64_t stamp_ns)
{
  const bool negative = stamp_ns < 0;
  const std::uint64_t magnitude = negative ?
    static_cast<std::uint64_t>(-(stamp_ns + 1)) + 1U :
    static_cast<std::uint64_t>(stamp_ns);
  const std::uint64_t seconds = magnitude / 1000000000ULL;
  const std::uint64_t nanoseconds = magnitude % 1000000000ULL;
  if (negative) {
    stream << '-';
  }
  stream << seconds << '.' << std::setw(9) << std::setfill('0') << nanoseconds <<
    std::setfill(' ');
}

}  // namespace

void saveTrajectoryTum(
  const std::filesystem::path & path,
  const std::vector<TimedPose> & trajectory)
{
  std::ofstream stream = openOutput(path);
  for (const TimedPose & pose : trajectory) {
    const Quaternion quaternion = pose.world_from_body.rotation.normalized();
    writeTumTimestamp(stream, pose.stamp_ns);
    stream << ' ' << pose.world_from_body.translation.x << ' '
           << pose.world_from_body.translation.y << ' '
           << pose.world_from_body.translation.z << ' '
           << quaternion.x << ' ' << quaternion.y << ' '
           << quaternion.z << ' ' << quaternion.w << '\n';
  }
}

void saveRouteGraphGeoJson(
  const std::filesystem::path & path,
  const RouteGraph & graph)
{
  std::ofstream stream = openOutput(path);
  stream << "{\n  \"type\": \"FeatureCollection\",\n  \"features\": [\n";
  bool first_feature = true;
  auto separator = [&]() {
      if (!first_feature) {
        stream << ",\n";
      }
      first_feature = false;
    };

  for (const RouteNode & node : graph.nodes) {
    separator();
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"id\":" << node.id << ','
           << "\"frame\":\"" << jsonEscape(graph.frame_id) << "\","
           << "\"metadata\":{"
           << "\"node_type\":\"" << toString(node.type) << "\","
           << "\"z\":" << node.position.z << "}},"
           << "\"geometry\":{\"type\":\"Point\",\"coordinates\":";
    writeCoordinate2d(stream, node.position);
    stream << "}}";
  }

  // Impassable edges are deliberately omitted from the Nav2 graph. They remain in
  // route_graph_metadata.yaml and drivable_corridors.geojson for review.
  for (const RouteEdge & edge : graph.edges) {
    if (!edge.passable) {
      continue;
    }
    separator();
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"id\":" << edge.id << ','
           << "\"startid\":" << edge.from << ','
           << "\"endid\":" << edge.to << ','
           << "\"cost\":" << edge.length << ','
           << "\"overridable\":true,"
           << "\"metadata\":{"
           << "\"class\":\"autogenerated_route\","
           << "\"passable\":true,"
           << "\"abs_speed_limit\":" << edge.recommended_speed_mps << ','
           << "\"minimum_safe_width\":" << edge.minimum_safe_width << ','
           << "\"maximum_curvature\":" << edge.maximum_curvature << ','
           << "\"confidence\":" << edge.confidence << ','
           << "\"clearance_complete\":" << (clearanceComplete(edge) ? "true" : "false") << ','
           << "\"corridor_geometry_valid\":" <<
      (edge.corridor_geometry_valid ? "true" : "false") << ','
           << "\"z_start\":" << edge.centerline.front().z << ','
           << "\"z_end\":" << edge.centerline.back().z;
    if (edge.reverse_of) {
      stream << ",\"reverse_of\":" << *edge.reverse_of;
    }
    stream << "}},\"geometry\":{\"type\":\"MultiLineString\",\"coordinates\":[[";
    for (std::size_t index = 0U; index < edge.centerline.size(); ++index) {
      if (index > 0U) {
        stream << ',';
      }
      writeCoordinate2d(stream, edge.centerline[index]);
    }
    stream << "]]}}";
  }
  stream << "\n  ]\n}\n";
}

void saveCorridorsGeoJson(
  const std::filesystem::path & path,
  const RouteGraph & graph)
{
  std::ofstream stream = openOutput(path);
  stream << "{\n  \"type\": \"FeatureCollection\",\n  \"features\": [\n";
  bool first = true;
  for (const RouteEdge & edge : graph.edges) {
    if (edge.reverse_of && edge.id > *edge.reverse_of) {
      continue;
    }
    if (edge.left_boundary.size() < 2U || edge.right_boundary.size() < 2U) {
      continue;
    }
    if (!first) {
      stream << ",\n";
    }
    first = false;
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"edge_id\":" << edge.id << ','
           << "\"passable\":" << (edge.passable ? "true" : "false") << ','
           << "\"minimum_safe_width\":" << edge.minimum_safe_width << ','
           << "\"confidence\":" << edge.confidence << ','
           << "\"clearance_complete\":" << (clearanceComplete(edge) ? "true" : "false") << ','
           << "\"corridor_geometry_valid\":" <<
      (edge.corridor_geometry_valid ? "true" : "false") << ','
           << "\"validation_errors\":";
    writeValidationErrorsJson(stream, edge.validation_errors);
    stream << "},\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[";
    bool first_point = true;
    for (const Vec3 & point : edge.left_boundary) {
      if (!first_point) {
        stream << ',';
      }
      first_point = false;
      writeCoordinate3d(stream, point);
    }
    for (auto iterator = edge.right_boundary.rbegin(); iterator != edge.right_boundary.rend(); ++iterator) {
      stream << ',';
      writeCoordinate3d(stream, *iterator);
    }
    stream << ',';
    writeCoordinate3d(stream, edge.left_boundary.front());
    stream << "]]}}";
  }
  stream << "\n  ]\n}\n";
}

void saveRouteGraphMetadataYaml(
  const std::filesystem::path & path,
  const RouteGraph & graph)
{
  std::ofstream stream = openOutput(path);
  stream << "frame_id: " << yamlQuote(graph.frame_id) << "\n";
  stream << "nodes:\n";
  for (const RouteNode & node : graph.nodes) {
    stream << "  " << node.id << ":\n"
           << "    type: " << toString(node.type) << "\n"
           << "    position: [" << node.position.x << ", " << node.position.y << ", "
           << node.position.z << "]\n";
  }
  stream << "edges:\n";
  for (const RouteEdge & edge : graph.edges) {
    stream << "  " << edge.id << ":\n"
           << "    from: " << edge.from << "\n"
           << "    to: " << edge.to << "\n"
           << "    length: " << edge.length << "\n"
           << "    minimum_safe_width: " << edge.minimum_safe_width << "\n"
           << "    maximum_curvature: " << edge.maximum_curvature << "\n"
           << "    confidence: " << edge.confidence << "\n"
           << "    clearance_complete: " << (clearanceComplete(edge) ? "true" : "false") << "\n"
           << "    corridor_geometry_valid: " <<
      (edge.corridor_geometry_valid ? "true" : "false") << "\n"
           << "    recommended_speed_mps: " << edge.recommended_speed_mps << "\n"
           << "    passable: " << (edge.passable ? "true" : "false") << "\n";
    stream << "    validation_errors:";
    if (edge.validation_errors.empty()) {
      stream << " []\n";
    } else {
      stream << '\n';
      for (const std::string & error : edge.validation_errors) {
        stream << "      - " << yamlQuote(error) << "\n";
      }
    }
    if (edge.reverse_of) {
      stream << "    reverse_of: " << *edge.reverse_of << "\n";
    }
  }
}

void saveAutowareReplayCandidateMetadataYaml(
  const std::filesystem::path & path,
  const AutowareReplayCandidateResult & candidate)
{
  saveRouteGraphMetadataYaml(path, candidate.graph);
  std::ofstream stream(path, std::ios::app);
  if (!stream) {
    throw std::runtime_error(
            "failed to append Autoware replay candidate metadata: " + path.string());
  }
  stream << std::setprecision(12)
         << "autoware_replay_derivative:\n"
         << "  scope: \"autoware_lanelet_only\"\n"
         << "  lossless_observed_route_unchanged: true\n"
         << "  nav2_replay_unchanged: true\n"
         << "  terminal_localization_settling_verified: "
         << (candidate.explicit_verification ? "true" : "false") << "\n"
         << "  verification_provenance: "
         << yamlQuote(candidate.verification_provenance) << "\n"
         << "  terminal_tail_omitted: "
         << (candidate.terminal_tail_omitted ? "true" : "false") << "\n"
         << "  reason: " << yamlQuote(candidate.reason) << "\n"
         << "  source_edges: " << candidate.source_edges << "\n"
         << "  retained_edges: " << candidate.retained_edges << "\n"
         << "  omitted_edges: " << candidate.omitted_edges << "\n"
         << "  source_planar_length_m: " << candidate.source_length << "\n"
         << "  retained_planar_length_m: " << candidate.retained_length << "\n"
         << "  omitted_planar_length_m: " << candidate.omitted_length << "\n"
         << "  omitted_length_ratio: " << candidate.omitted_length_ratio << "\n"
         << "  connection_heading_jump_deg: "
         << candidate.connection_heading_jump_deg << "\n"
         << "  terminal_body_yaw_change_deg: "
         << candidate.terminal_body_yaw_change_deg << "\n"
         << "  fixed_gates:\n"
         << "    minimum_heading_jump_deg: "
         << kAutowareTerminalCuspMinimumHeadingJumpDeg << "\n"
         << "    maximum_tail_length_m: "
         << kAutowareTerminalSettlingMaximumLength << "\n"
         << "    maximum_tail_length_ratio: "
         << kAutowareTerminalSettlingMaximumLengthRatio << "\n"
         << "    maximum_terminal_body_yaw_change_deg: "
         << kAutowareTerminalSettlingMaximumBodyYawChangeDeg << "\n";
}

void saveLanelet2OsmImpl(
  const std::filesystem::path & path,
  const RouteGraph & graph,
  const Lanelet2Config & config,
  const Lanelet2OutputKind output_kind,
  const ExperimentalLanelet2Metadata * experimental_metadata,
  const Lanelet2AuthoringOptions & authoring)
{
  const bool autoware_ready = output_kind == Lanelet2OutputKind::kProductionReady;
  const bool closed_course_experimental =
    output_kind == Lanelet2OutputKind::kClosedCourseExperimental;
  if (closed_course_experimental && experimental_metadata == nullptr) {
    throw std::invalid_argument("closed-course Lanelet2 metadata is required");
  }
  std::ofstream stream = openOutput(path);
  stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  stream <<
    "<osm version=\"0.6\" generator=\"lidar_mobility_map_generator\" upload=\"false\">\n"
    "  <MetaInfo format_version=\"1.1\" map_version=\"" LMMG_PROJECT_VERSION "\"/>\n";

  struct LaneletObjects
  {
    std::uint64_t left_way{0U};
    std::uint64_t right_way{0U};
    std::uint64_t center_way{0U};
    std::uint64_t relation{0U};
    std::vector<std::uint64_t> left_nodes;
    std::vector<std::uint64_t> right_nodes;
    std::vector<std::uint64_t> center_nodes;
    std::vector<std::uint64_t> regulatory_relations;
    const RouteEdge * edge{nullptr};
  };

  std::map<std::uint64_t, std::size_t> named_route_order;
  if (authoring.named_route != nullptr) {
    for (std::size_t index = 0U;
      index < authoring.named_route->ordered_edge_ids.size(); ++index)
    {
      named_route_order.emplace(authoring.named_route->ordered_edge_ids[index], index);
    }
  }

  if ((authoring.semantic_source_graph == nullptr) !=
    (authoring.semantic_edge_provenance == nullptr))
  {
    throw std::invalid_argument(
            "Lanelet2 semantic source graph and provenance must be supplied together");
  }
  std::map<std::uint64_t, SemanticRouteEdgeProvenance> semantic_provenance_by_edge;
  std::map<std::uint64_t, double> semantic_source_length_by_edge;
  std::map<std::uint64_t, SyntheticOpenRoutePlanningSupport>
    synthetic_planning_support_by_edge;
  if (experimental_metadata != nullptr) {
    for (const SyntheticOpenRoutePlanningSupport & support :
      experimental_metadata->summary.synthetic_planning_support)
    {
      if (!synthetic_planning_support_by_edge.emplace(support.edge_id, support).second) {
        throw std::invalid_argument(
                "Lanelet2 synthetic planning support has duplicate Edge IDs");
      }
    }
  }
  if (authoring.semantic_edge_provenance != nullptr) {
    for (const RouteEdge & source : authoring.semantic_source_graph->edges) {
      semantic_source_length_by_edge.emplace(
        source.id, polylineLength(source.centerline));
    }
    for (const SemanticRouteEdgeProvenance & provenance :
      *authoring.semantic_edge_provenance)
    {
      if (!semantic_provenance_by_edge.emplace(provenance.edge_id, provenance).second) {
        throw std::invalid_argument("Lanelet2 semantic provenance has duplicate output Edge IDs");
      }
    }
  }

  std::vector<const RouteEdge *> selected_edges;
  for (const RouteEdge & edge : graph.edges) {
    // Match the operational Route Graph exporter: unsafe geometry remains in
    // review_geometry.tsv/corridors but must not become a routable Lanelet.
    if (!edge.passable) {
      continue;
    }
    // A physical corridor is exported once.  When one_way is true this keeps
    // the originally observed direction; exporting its generated mirror as a
    // second one-way Lanelet would silently make the corridor bidirectional
    // and would also mix left/right endpoint sharing across the mirror pair.
    if (edge.reverse_of) {
      const bool edge_is_authored = named_route_order.count(edge.id) != 0U;
      const bool reverse_is_authored = named_route_order.count(*edge.reverse_of) != 0U;
      if (edge_is_authored != reverse_is_authored) {
        if (!edge_is_authored) {
          continue;
        }
      } else if (edge.id > *edge.reverse_of) {
        continue;
      }
    }
    if (edge.left_boundary.size() == edge.centerline.size() &&
      edge.right_boundary.size() == edge.centerline.size() && edge.centerline.size() >= 2U)
    {
      if (authoring.semantic_edge_provenance != nullptr &&
        semantic_provenance_by_edge.count(edge.id) == 0U &&
        synthetic_planning_support_by_edge.count(edge.id) == 0U)
      {
        throw std::invalid_argument(
                "Lanelet2 output Edge lacks lossless semantic provenance: " +
                std::to_string(edge.id));
      }
      selected_edges.push_back(&edge);
    }
  }

  // ``named_route_order`` is a Lanelet-chain order, not a raw authored-Edge
  // order. GUI speed rules may replace one authored Edge with several
  // semantic children, so assign one global consecutive order after sorting
  // by raw Mission order and source arc. ``source_route_edge_id`` below keeps
  // the independent many-to-one raw lineage.
  struct NamedLaneletCandidate
  {
    const RouteEdge * edge{nullptr};
    std::size_t raw_order{0U};
    double source_start_s{0.0};
  };
  std::vector<NamedLaneletCandidate> named_lanelet_candidates;
  for (const RouteEdge * edge : selected_edges) {
    std::optional<std::size_t> raw_order;
    const auto exact_order = named_route_order.find(edge->id);
    if (exact_order != named_route_order.end()) {
      raw_order = exact_order->second;
    } else if (edge->reverse_of) {
      const auto reverse_order = named_route_order.find(*edge->reverse_of);
      if (reverse_order != named_route_order.end()) {
        raw_order = reverse_order->second;
      }
    }
    double source_start_s = 0.0;
    const auto semantic = semantic_provenance_by_edge.find(edge->id);
    if (semantic != semantic_provenance_by_edge.end()) {
      const auto source_order = named_route_order.find(
        semantic->second.source_edge_id);
      if (source_order != named_route_order.end()) {
        raw_order = source_order->second;
        source_start_s = semantic->second.source_start_s;
      }
    }
    if (raw_order) {
      named_lanelet_candidates.push_back({edge, *raw_order, source_start_s});
    }
  }
  std::sort(
    named_lanelet_candidates.begin(), named_lanelet_candidates.end(),
    [](const NamedLaneletCandidate & lhs, const NamedLaneletCandidate & rhs) {
      return std::tie(lhs.raw_order, lhs.source_start_s, lhs.edge->id) <
             std::tie(rhs.raw_order, rhs.source_start_s, rhs.edge->id);
    });
  std::map<std::uint64_t, std::size_t> named_lanelet_order;
  for (std::size_t index = 0U; index < named_lanelet_candidates.size(); ++index) {
    named_lanelet_order.emplace(named_lanelet_candidates[index].edge->id, index);
  }

  std::map<std::uint64_t, RouteNodeType> node_types;
  std::map<std::uint64_t, Vec3> route_node_positions;
  for (const RouteNode & node : graph.nodes) {
    node_types[node.id] = node.type;
    route_node_positions[node.id] = node.position;
  }
  std::map<std::uint64_t, std::size_t> selected_incoming_count;
  std::map<std::uint64_t, std::size_t> selected_outgoing_count;
  for (const RouteEdge * edge : selected_edges) {
    ++selected_outgoing_count[edge->from];
    ++selected_incoming_count[edge->to];
  }

  // Average each side's incident endpoints and reuse one OSM node. This
  // preserves Lanelet2 adjacency along unambiguous 1-in/1-out chains.
  // A branch cannot have one common cross-section without potentially folding
  // one of its incident Lanelets, so its boundary endpoints stay independent;
  // the shared route-centre node still preserves the authored graph junction.
  using SharedKey = std::pair<std::uint64_t, int>;  // side: 0=left, 1=right
  struct Average
  {
    Vec3 sum{};
    std::size_t count{0U};
    std::vector<Vec3> points;
  };
  std::map<SharedKey, Average> endpoint_averages;
  auto add_endpoint = [&](const std::uint64_t node_id, const int side, const Vec3 & point) {
      const auto found = node_types.find(node_id);
      if (found == node_types.end() ||
        (!closed_course_experimental && found->second == RouteNodeType::kJunction) ||
        (closed_course_experimental &&
        (selected_incoming_count[node_id] != 1U ||
        selected_outgoing_count[node_id] != 1U)))
      {
        return;
      }
      Average & average = endpoint_averages[{node_id, side}];
      average.sum += point;
      ++average.count;
      average.points.push_back(point);
    };
  for (const RouteEdge * edge : selected_edges) {
    if (edge->from != edge->to) {
      add_endpoint(edge->from, 0, edge->left_boundary.front());
      add_endpoint(edge->from, 1, edge->right_boundary.front());
      add_endpoint(edge->to, 0, edge->left_boundary.back());
      add_endpoint(edge->to, 1, edge->right_boundary.back());
    }
  }

  std::uint64_t next_id = 1U;
  std::map<std::uint64_t, Vec3> osm_nodes;
  auto allocate_node = [&](const Vec3 & point) {
      const std::uint64_t id = next_id++;
      osm_nodes[id] = point;
      return id;
    };

  std::map<SharedKey, std::uint64_t> shared_boundary_nodes;
  if (closed_course_experimental) {
    for (const auto & route_node : route_node_positions) {
      const auto left_entry = endpoint_averages.find({route_node.first, 0});
      const auto right_entry = endpoint_averages.find({route_node.first, 1});
      if (left_entry == endpoint_averages.end() || right_entry == endpoint_averages.end() ||
        left_entry->second.count == 0U || right_entry->second.count == 0U)
      {
        continue;
      }
      if (left_entry->second.count == 1U && right_entry->second.count == 1U) {
        // An open component's terminal boundary is deliberately extended by
        // the tagged longitudinal guard. Reprojecting that sole endpoint onto
        // the RouteNode would erase the cap extension during serialization.
        // Allocate the Edge's exact boundary endpoint below; only junctions
        // shared by multiple incident Lanelets need a shared OSM boundary ID.
        continue;
      }
      // Share only incident endpoints that already describe the same boundary
      // cross-section (apart from round-off). Reprojecting a non-coincident
      // section to the RouteNode as a geometric miter can turn a
      // centimetre-long first segment inside out.
      const Vec3 average_left = left_entry->second.sum /
        static_cast<double>(left_entry->second.count);
      const Vec3 average_right = right_entry->second.sum /
        static_cast<double>(right_entry->second.count);
      constexpr double shared_endpoint_tolerance_m = 1.0e-6;
      const auto coincident = [&](const Average & average, const Vec3 & mean) {
          return std::all_of(
            average.points.begin(), average.points.end(),
            [&](const Vec3 & point) {
              return distance3d(point, mean) <= shared_endpoint_tolerance_m;
            });
        };
      if (distance2d(average_left, average_right) <= 1.0e-12 ||
        !coincident(left_entry->second, average_left) ||
        !coincident(right_entry->second, average_right))
      {
        continue;
      }
      shared_boundary_nodes[{route_node.first, 0}] = allocate_node({
        average_left.x, average_left.y, average_left.z});
      shared_boundary_nodes[{route_node.first, 1}] = allocate_node({
        average_right.x, average_right.y, average_right.z});
    }
  } else {
    for (const auto & entry : endpoint_averages) {
      if (entry.second.count > 0U) {
        shared_boundary_nodes[entry.first] = allocate_node(
          entry.second.sum / static_cast<double>(entry.second.count));
      }
    }
  }
  std::map<std::uint64_t, std::uint64_t> shared_center_nodes;
  for (const auto & entry : route_node_positions) {
    shared_center_nodes[entry.first] = allocate_node(entry.second);
  }

  auto endpoint_or_new = [&](const std::uint64_t route_node, const int side, const Vec3 & point) {
      const auto found = shared_boundary_nodes.find({route_node, side});
      return found != shared_boundary_nodes.end() ? found->second : allocate_node(point);
    };

  std::vector<LaneletObjects> objects;
  for (const RouteEdge * edge : selected_edges) {
    LaneletObjects object;
    object.edge = edge;
    object.left_nodes.reserve(edge->centerline.size());
    object.right_nodes.reserve(edge->centerline.size());
    object.center_nodes.reserve(edge->centerline.size());
    for (std::size_t index = 0U; index < edge->centerline.size(); ++index) {
      const bool first = index == 0U;
      const bool last = index + 1U == edge->centerline.size();
      if (first) {
        object.left_nodes.push_back(endpoint_or_new(edge->from, 0, edge->left_boundary[index]));
        object.right_nodes.push_back(endpoint_or_new(edge->from, 1, edge->right_boundary[index]));
        object.center_nodes.push_back(shared_center_nodes.at(edge->from));
      } else if (last) {
        object.left_nodes.push_back(endpoint_or_new(edge->to, 0, edge->left_boundary[index]));
        object.right_nodes.push_back(endpoint_or_new(edge->to, 1, edge->right_boundary[index]));
        object.center_nodes.push_back(shared_center_nodes.at(edge->to));
      } else {
        object.left_nodes.push_back(allocate_node(edge->left_boundary[index]));
        object.right_nodes.push_back(allocate_node(edge->right_boundary[index]));
        object.center_nodes.push_back(allocate_node(edge->centerline[index]));
      }
    }
    object.left_way = next_id++;
    object.right_way = next_id++;
    object.center_way = next_id++;
    object.relation = next_id++;
    objects.push_back(std::move(object));
  }
  if (closed_course_experimental) {
    const auto serialized_edge = [&](const LaneletObjects & object) {
      RouteEdge serialized = *object.edge;
      serialized.left_boundary.clear();
      serialized.right_boundary.clear();
      serialized.centerline.clear();
      for (const std::uint64_t node : object.left_nodes) {
        serialized.left_boundary.push_back(osm_nodes.at(node));
      }
      for (const std::uint64_t node : object.right_nodes) {
        serialized.right_boundary.push_back(osm_nodes.at(node));
      }
      for (const std::uint64_t node : object.center_nodes) {
        serialized.centerline.push_back(osm_nodes.at(node));
      }
      return serialized;
    };
    const auto invalid_object_indices = [&]() {
      std::vector<std::size_t> invalid;
      for (std::size_t index = 0U; index < objects.size(); ++index) {
        if (!estimatedBoundaryPolygonIsValid(serialized_edge(objects[index]))) {
          invalid.push_back(index);
        }
      }
      return invalid;
    };
    std::vector<std::size_t> invalid = invalid_object_indices();
    if (!invalid.empty()) {
      throw std::invalid_argument(
              "serialized closed-course Lanelet boundary polygon is invalid for Edge " +
              std::to_string(objects[invalid.front()].edge->id));
    }
  }

  struct StopLineObjects
  {
    AuthoredStopLine stop;
    std::size_t lanelet_index{0U};
    std::uint64_t way{0U};
    std::uint64_t regulatory_relation{0U};
    std::vector<std::uint64_t> nodes;
  };
  std::map<std::uint64_t, std::size_t> lanelet_by_edge;
  for (std::size_t index = 0U; index < objects.size(); ++index) {
    lanelet_by_edge.emplace(objects[index].edge->id, index);
  }
  std::map<std::uint64_t, const RouteEdge *> graph_edge_by_id;
  for (const RouteEdge & edge : graph.edges) {
    graph_edge_by_id.emplace(edge.id, &edge);
  }
  auto lanelet_for_edge = [&](const std::uint64_t edge_id) -> std::optional<std::size_t> {
      const auto exact = lanelet_by_edge.find(edge_id);
      if (exact != lanelet_by_edge.end()) {
        return exact->second;
      }
      const auto source = graph_edge_by_id.find(edge_id);
      if (source != graph_edge_by_id.end() && source->second->reverse_of) {
        const auto reverse = lanelet_by_edge.find(*source->second->reverse_of);
        if (reverse != lanelet_by_edge.end()) {
          return reverse->second;
        }
      }
      for (std::size_t index = 0U; index < objects.size(); ++index) {
        if (objects[index].edge->reverse_of && *objects[index].edge->reverse_of == edge_id) {
          return index;
        }
      }
      return std::nullopt;
    };

  std::vector<StopLineObjects> stop_line_objects;
  for (const AuthoredStopLine & stop : authoring.resolved_stop_lines) {
    if (!(stop.width_m > 0.0) || !std::isfinite(stop.width_m) ||
      !std::isfinite(stop.s))
    {
      throw std::invalid_argument(
              "resolved Lanelet2 stop line has invalid width or arc length");
    }
    const std::optional<std::size_t> lanelet_index = lanelet_for_edge(stop.edge_id);
    if (!lanelet_index) {
      throw std::invalid_argument(
              "resolved Lanelet2 stop line edge is not exportable: " +
              std::to_string(stop.edge_id));
    }
    AuthoredStopLine geometric_stop = stop;
    const RouteEdge & output_edge = *objects[*lanelet_index].edge;
    if (output_edge.id != stop.edge_id) {
      double output_length = 0.0;
      for (std::size_t index = 1U; index < output_edge.centerline.size(); ++index) {
        output_length += distance3d(
          output_edge.centerline[index - 1U], output_edge.centerline[index]);
      }
      geometric_stop.s = clamp(output_length - stop.s, 0.0, output_length);
    }
    const StopLinePlacement placement = projectStopLineToCenterline(
      output_edge, geometric_stop);
    if (!placement.valid) {
      throw std::invalid_argument(
              "resolved Lanelet2 stop line cannot be placed on edge: " +
              std::to_string(stop.edge_id));
    }
    const Vec2 normal{-placement.tangent.y, placement.tangent.x};
    const double half_width = 0.5 * stop.width_m;
    StopLineObjects output;
    output.stop = stop;
    output.lanelet_index = *lanelet_index;
    output.nodes = {
      allocate_node({
        placement.center.x + normal.x * half_width,
        placement.center.y + normal.y * half_width,
        placement.center.z}),
      allocate_node({
        placement.center.x - normal.x * half_width,
        placement.center.y - normal.y * half_width,
        placement.center.z})};
    output.way = next_id++;
    output.regulatory_relation = next_id++;
    objects[*lanelet_index].regulatory_relations.push_back(output.regulatory_relation);
    stop_line_objects.push_back(std::move(output));
  }

  for (const auto & entry : osm_nodes) {
    const Vec3 & point = entry.second;
    stream << "  <node id=\"" << entry.first <<
      "\" visible=\"true\" version=\"1\" lat=\"0\" lon=\"0\">\n"
           << "    <tag k=\"local_x\" v=\"" << point.x << "\"/>\n"
           << "    <tag k=\"local_y\" v=\"" << point.y << "\"/>\n"
           << "    <tag k=\"ele\" v=\"" << point.z << "\"/>\n"
           << "  </node>\n";
  }

  auto write_way = [&stream](
      const std::uint64_t id, const std::vector<std::uint64_t> & nodes,
      const std::string & type, const std::string & subtype) {
      stream << "  <way id=\"" << id << "\" visible=\"true\" version=\"1\">\n";
      for (const std::uint64_t node : nodes) {
        stream << "    <nd ref=\"" << node << "\"/>\n";
      }
      stream << "    <tag k=\"type\" v=\"" << xmlEscape(type) << "\"/>\n";
      if (!subtype.empty()) {
        stream << "    <tag k=\"subtype\" v=\"" << xmlEscape(subtype) << "\"/>\n";
      }
      stream << "  </way>\n";
    };

  for (const LaneletObjects & object : objects) {
    const std::string boundary_type = closed_course_experimental ?
      "virtual" : config.boundary_type;
    const std::string boundary_subtype = closed_course_experimental ?
      "" : config.boundary_subtype;
    write_way(
      object.left_way, object.left_nodes,
      boundary_type, boundary_subtype);
    write_way(
      object.right_way, object.right_nodes,
      boundary_type, boundary_subtype);
    write_way(object.center_way, object.center_nodes, "virtual", "");
  }

  for (const StopLineObjects & output : stop_line_objects) {
    stream << "  <way id=\"" << output.way <<
      "\" visible=\"true\" version=\"1\">\n";
    for (const std::uint64_t node : output.nodes) {
      stream << "    <nd ref=\"" << node << "\"/>\n";
    }
    stream << "    <tag k=\"type\" v=\"stop_line\"/>\n"
           << "    <tag k=\"subtype\" v=\"solid\"/>\n"
           << "    <tag k=\"name\" v=\"" << xmlEscape(output.stop.name) << "\"/>\n"
           << "    <tag k=\"virtual\" v=\"yes\"/>\n"
           << "    <tag k=\"autogenerated\" v=\"yes\"/>\n"
           << "    <tag k=\"audit_source\" v=\"navigation_authoring_gui\"/>\n"
           << "    <tag k=\"physical_stop_sign_observed\" v=\"no\"/>\n"
           << "    <tag k=\"authored_stop_line_id\" v=\"" << output.stop.id << "\"/>\n"
           << "    <tag k=\"route_edge_id\" v=\"" << output.stop.edge_id << "\"/>\n"
           << "    <tag k=\"route_edge_s_m\" v=\"" << output.stop.s << "\"/>\n"
           << "    <tag k=\"authored_width_m\" v=\"" << output.stop.width_m << "\"/>\n"
           << "    <tag k=\"navigation_target\" v=\"" <<
      toString(output.stop.target) << "\"/>\n";
    const auto stop_provenance = semantic_provenance_by_edge.find(output.stop.edge_id);
    if (stop_provenance != semantic_provenance_by_edge.end()) {
      stream << "    <tag k=\"source_route_edge_id\" v=\"" <<
        stop_provenance->second.source_edge_id << "\"/>\n"
             << "    <tag k=\"source_route_edge_s_m\" v=\"" <<
        stop_provenance->second.source_start_s + output.stop.s << "\"/>\n";
    }
    stream << "  </way>\n";
  }

  for (const StopLineObjects & output : stop_line_objects) {
    stream << "  <relation id=\"" << output.regulatory_relation <<
      "\" visible=\"true\" version=\"1\">\n"
           << "    <member type=\"way\" ref=\"" << output.way <<
      "\" role=\"ref_line\"/>\n"
           << "    <tag k=\"type\" v=\"regulatory_element\"/>\n"
           << "    <tag k=\"subtype\" v=\"traffic_sign\"/>\n"
           << "    <tag k=\"sign_type\" v=\"stop_sign\"/>\n"
           << "    <tag k=\"name\" v=\"" << xmlEscape(output.stop.name) << "\"/>\n"
           << "    <tag k=\"virtual\" v=\"yes\"/>\n"
           << "    <tag k=\"autogenerated\" v=\"yes\"/>\n"
           << "    <tag k=\"audit_source\" v=\"navigation_authoring_gui\"/>\n"
           << "    <tag k=\"physical_stop_sign_observed\" v=\"no\"/>\n"
           << "    <tag k=\"authored_stop_line_id\" v=\"" << output.stop.id << "\"/>\n"
           << "  </relation>\n";
  }

  for (const LaneletObjects & object : objects) {
    const RouteEdge & edge = *object.edge;
    const double speed_limit_mps = edge.recommended_speed_mps > 0.0 ?
      edge.recommended_speed_mps : config.speed_limit_mps;
    stream << "  <relation id=\"" << object.relation <<
      "\" visible=\"true\" version=\"1\">\n"
           << "    <member type=\"way\" ref=\"" << object.left_way << "\" role=\"left\"/>\n"
           << "    <member type=\"way\" ref=\"" << object.right_way << "\" role=\"right\"/>\n"
           << "    <member type=\"way\" ref=\"" << object.center_way << "\" role=\"centerline\"/>\n";
    for (const std::uint64_t regulatory_relation : object.regulatory_relations) {
      stream << "    <member type=\"relation\" ref=\"" << regulatory_relation <<
        "\" role=\"regulatory_element\"/>\n";
    }
    stream << "    <tag k=\"type\" v=\"lanelet\"/>\n"
           << "    <tag k=\"subtype\" v=\"" << xmlEscape(config.subtype) << "\"/>\n"
           << "    <tag k=\"location\" v=\"" << xmlEscape(config.location) << "\"/>\n"
           << "    <tag k=\"one_way\" v=\"" << (config.one_way ? "yes" : "no") << "\"/>\n"
           // Unitless Lanelet2 speed limits are km/h. This form is understood
           // by Lanelet2 TrafficRules and also passes Autoware's current map
           // validator, whose strict numeric parser rejects otherwise legal
           // values such as "0.25 m/s".
           << "    <tag k=\"speed_limit\" v=\"" <<
      speed_limit_mps * 3.6 << "\"/>\n"
           << "    <tag k=\"generator_speed_limit_mps\" v=\"" <<
      speed_limit_mps << "\"/>\n";
    if (!config.participant.empty()) {
      stream << "    <tag k=\"participant:" << xmlEscape(config.participant)
             << "\" v=\"yes\"/>\n";
    }
    const auto authored_order = named_lanelet_order.find(edge.id);
    if (authoring.named_route != nullptr &&
      authored_order != named_lanelet_order.end())
    {
      stream << "    <tag k=\"named_route_id\" v=\"" <<
        authoring.named_route->id << "\"/>\n"
             << "    <tag k=\"named_route_name\" v=\"" <<
        xmlEscape(authoring.named_route->name) << "\"/>\n"
             << "    <tag k=\"named_route_target\" v=\"" <<
        toString(authoring.named_route->target) << "\"/>\n"
             << "    <tag k=\"named_route_order\" v=\"" <<
        authored_order->second << "\"/>\n";
    }
    stream << "    <tag k=\"autogenerated\" v=\"yes\"/>\n"
           << "    <tag k=\"autoware_ready\" v=\"" <<
      (autoware_ready ? "yes" : "no") << "\"/>\n"
           << "    <tag k=\"boundary_model\" v=\"" <<
      (autoware_ready ? "independently_verified_physical_boundary" :
      (closed_course_experimental ?
      "trajectory_derived_estimated_drivable_corridor" :
      "center_feasible_corridor")) << "\"/>\n";
    if (closed_course_experimental) {
      const ClosedCourseLanelet2ExportOptions & options = experimental_metadata->options;
      const ClosedCourseLanelet2ExportSummary & summary = experimental_metadata->summary;
      const auto synthetic_support = synthetic_planning_support_by_edge.find(edge.id);
      const bool is_synthetic_support =
        synthetic_support != synthetic_planning_support_by_edge.end();
      const bool user_authored_centerline =
        options.centerline_source == "edited_topology";
      const double coverage = summary.source_length > 1.0e-12 ?
        summary.exported_length / summary.source_length : 0.0;
      stream << "    <tag k=\"production_ready\" v=\"no\"/>\n"
             << "    <tag k=\"experimental_use\" v=\"closed_course_only\"/>\n"
             << "    <tag k=\"experimental_ready\" v=\"" <<
        (options.experimental_ready ? "yes" : "no") << "\"/>\n"
             << "    <tag k=\"physical_boundaries_verified\" v=\"no\"/>\n"
             << "    <tag k=\"vehicle_dimensions_source\" v=\"" <<
        xmlEscape(options.vehicle_dimensions_evidence_source) << "\"/>\n"
             << "    <tag k=\"vehicle_profile\" v=\"" <<
        xmlEscape(options.vehicle_profile) << "\"/>\n"
             << "    <tag k=\"vehicle_base_reference\" v=\"" <<
        xmlEscape(options.vehicle_base_reference) << "\"/>\n"
             << "    <tag k=\"vehicle_dimensions_evidence_source\" v=\"" <<
        xmlEscape(options.vehicle_dimensions_evidence_source) << "\"/>\n"
             << "    <tag k=\"vehicle_dimensions_evidence_confidence\" v=\"" <<
        xmlEscape(options.vehicle_dimensions_evidence_confidence) << "\"/>\n"
             << "    <tag k=\"vehicle_dimensions_verified\" v=\"" <<
        (options.vehicle_dimensions_verified ? "yes" : "no") << "\"/>\n"
             << "    <tag k=\"centerline_source\" v=\"" <<
        xmlEscape(options.centerline_source) << "\"/>\n"
             << "    <tag k=\"provenance\" v=\"" <<
        (is_synthetic_support ?
        "synthetic_test_kinematic_staging" :
        (user_authored_centerline ?
        "user_authored_centerline" : "observed_driven_trajectory")) << "\"/>\n"
             << "    <tag k=\"observed_driven\" v=\"" <<
        (is_synthetic_support || user_authored_centerline ? "no" : "yes") << "\"/>\n"
             << "    <tag k=\"validation_status\" v=\"" <<
        (is_synthetic_support ?
        "estimated_synthetic_test_staging" :
        (user_authored_centerline ?
        "user_authored_vehicle_footprint_validated_candidate" :
        "observed_driven_replay_candidate")) << "\"/>\n"
             << "    <tag k=\"operational_safety_margin_validated\" v=\"no\"/>\n"
             << "    <tag k=\"estimated_vehicle_width_m\" v=\"" <<
        options.estimated_vehicle_width << "\"/>\n"
             << "    <tag k=\"estimated_front_extent_m\" v=\"" <<
        options.estimated_front_extent << "\"/>\n"
             << "    <tag k=\"estimated_rear_extent_m\" v=\"" <<
        options.estimated_rear_extent << "\"/>\n"
             << "    <tag k=\"vehicle_minimum_turning_radius_m\" v=\"" <<
        options.estimated_minimum_turning_radius << "\"/>\n"
             << "    <tag k=\"estimated_lateral_margin_m\" v=\"" <<
        options.lateral_clearance_margin << "\"/>\n"
             << "    <tag k=\"estimated_boundary_interpolation_guard_m\" v=\"" <<
        kEstimatedBoundaryInterpolationGuardM << "\"/>\n"
             << "    <tag k=\"estimated_longitudinal_endpoint_guard_m\" v=\"" <<
        kEstimatedLongitudinalEndpointGuardM << "\"/>\n"
             << "    <tag k=\"estimated_effective_lateral_margin_m\" v=\"" <<
        options.lateral_clearance_margin + kEstimatedBoundaryInterpolationGuardM <<
        "\"/>\n"
             << "    <tag k=\"estimated_boundary_algorithm\" "
        "v=\"oriented_rectangular_swept_envelope\"/>\n"
             << "    <tag k=\"estimated_corridor_width_m\" v=\"" <<
        experimental_metadata->corridor_width << "\"/>\n"
             << "    <tag k=\"source_physical_edge_count\" v=\"" <<
        summary.source_physical_edges << "\"/>\n"
             << "    <tag k=\"exported_edge_count\" v=\"" <<
        summary.exported_physical_edges << "\"/>\n"
             << "    <tag k=\"exported_lanelet_segment_count\" v=\"" <<
        summary.exported_lanelet_segments << "\"/>\n"
             << "    <tag k=\"exported_length_m\" v=\"" <<
        summary.exported_length << "\"/>\n"
             << "    <tag k=\"source_physical_length_m\" v=\"" <<
        summary.source_length << "\"/>\n"
             << "    <tag k=\"exported_length_coverage\" v=\"" <<
        coverage << "\"/>\n"
             << "    <tag k=\"synthetic_planning_support_count\" v=\"" <<
        summary.synthetic_planning_support.size() << "\"/>\n";
      if (is_synthetic_support) {
        const SyntheticOpenRoutePlanningSupport & support = synthetic_support->second;
        stream << "    <tag k=\"synthetic_planning_support\" v=\"yes\"/>\n"
               << "    <tag k=\"synthetic_test_staging\" v=\"yes\"/>\n"
               << "    <tag k=\"deployment_ready\" v=\"no\"/>\n"
               << "    <tag k=\"surveyed\" v=\"no\"/>\n"
               << "    <tag k=\"support_is_part_of_raw_counts\" v=\"no\"/>\n"
               << "    <tag k=\"support_is_part_of_named_route\" v=\"no\"/>\n"
               << "    <tag k=\"support_is_raw_coverage\" v=\"no\"/>\n"
               << "    <tag k=\"planning_support_contract_version\" v=\"" <<
          support.planning_support_contract_version << "\"/>\n"
               << "    <tag k=\"planning_support_role\" v=\"" <<
          support.role << "\"/>\n"
               << "    <tag k=\"planning_support_source\" "
          "v=\"deterministic_kinematic_staging_search\"/>\n"
               << "    <tag k=\"planning_support_estimated\" v=\"yes\"/>\n"
               << "    <tag k=\"planning_support_geometry_kind\" v=\"" <<
          support.geometry_kind << "\"/>\n"
               << "    <tag k=\"planning_support_adjacent_output_edge_id\" v=\"" <<
          support.adjacent_output_edge_id << "\"/>\n"
               << "    <tag k=\"planning_support_adjacent_source_edge_id\" v=\"" <<
          support.adjacent_source_edge_id << "\"/>\n"
               << "    <tag k=\"planning_support_raw_endpoint_node_id\" v=\"" <<
          support.raw_endpoint_node_id << "\"/>\n"
               << "    <tag k=\"planning_support_source_edge_length_m\" v=\"" <<
          support.source_edge_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_raw_endpoint_s_m\" v=\"" <<
          support.raw_endpoint_s_m << "\"/>\n"
               << "    <tag k=\"planning_support_raw_endpoint_x\" v=\"" <<
          support.raw_endpoint.x << "\"/>\n"
               << "    <tag k=\"planning_support_raw_endpoint_y\" v=\"" <<
          support.raw_endpoint.y << "\"/>\n"
               << "    <tag k=\"planning_support_raw_endpoint_z\" v=\"" <<
          support.raw_endpoint.z << "\"/>\n"
               << "    <tag k=\"planning_support_synthetic_endpoint_x\" v=\"" <<
          support.synthetic_endpoint.x << "\"/>\n"
               << "    <tag k=\"planning_support_synthetic_endpoint_y\" v=\"" <<
          support.synthetic_endpoint.y << "\"/>\n"
               << "    <tag k=\"planning_support_synthetic_endpoint_z\" v=\"" <<
          support.synthetic_endpoint.z << "\"/>\n"
               << "    <tag k=\"planning_support_tangent_x\" v=\"" <<
          support.directed_tangent.x << "\"/>\n"
               << "    <tag k=\"planning_support_tangent_y\" v=\"" <<
          support.directed_tangent.y << "\"/>\n"
               << "    <tag k=\"planning_support_outer_tangent_x\" v=\"" <<
          support.outer_directed_tangent.x << "\"/>\n"
               << "    <tag k=\"planning_support_outer_tangent_y\" v=\"" <<
          support.outer_directed_tangent.y << "\"/>\n"
               << "    <tag k=\"planning_support_centerline_planar_length_m\" v=\"" <<
          support.centerline_planar_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_centerline_3d_length_m\" v=\"" <<
          support.centerline_3d_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_endpoint_allowance_m\" v=\"" <<
          options.planning_endpoint_allowance << "\"/>\n"
               << "    <tag k=\"planning_support_required_boundary_beyond_raw_endpoint_m\" v=\"" <<
          support.required_boundary_beyond_raw_endpoint_m << "\"/>\n"
               << "    <tag k=\"planning_support_actual_left_boundary_beyond_raw_endpoint_m\" v=\"" <<
          support.actual_left_boundary_beyond_raw_endpoint_m << "\"/>\n"
               << "    <tag k=\"planning_support_actual_right_boundary_beyond_raw_endpoint_m\" v=\"" <<
          support.actual_right_boundary_beyond_raw_endpoint_m << "\"/>\n"
               << "    <tag k=\"planning_support_search_step_m\" v=\"" <<
          support.search_step_m << "\"/>\n"
               << "    <tag k=\"planning_support_path_sample_spacing_m\" v=\"" <<
          kPlanningSupportPathSampleSpacingM << "\"/>\n"
               << "    <tag k=\"planning_support_search_max_length_m\" v=\"" <<
          support.search_max_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_selected_candidate_index\" v=\"" <<
          support.selected_candidate_index << "\"/>\n"
               << "    <tag k=\"planning_support_candidate_count_tested\" v=\"" <<
          support.candidate_count_tested << "\"/>\n"
               << "    <tag k=\"planning_support_individually_valid_candidate_rank\" v=\"" <<
          support.individually_valid_candidate_rank << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_kinematic_candidates\" v=\"" <<
          support.rejected_kinematic_candidates << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_invalid_geometry_candidates\" v=\"" <<
          support.rejected_invalid_geometry_candidates << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_outer_raw_overlap_candidates\" v=\"" <<
          support.rejected_outer_raw_overlap_candidates << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_insufficient_outer_pose_isolation_candidates\" v=\"" <<
          support.rejected_insufficient_outer_pose_isolation_candidates << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_raw_polygon_reentry_candidates\" v=\"" <<
          support.rejected_raw_polygon_reentry_candidates << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_nonadjacent_transition_candidates\" v=\"" <<
          support.rejected_nonadjacent_transition_candidates << "\"/>\n"
               << "    <tag k=\"planning_support_turn_radius_m\" v=\"" <<
          support.turn_radius_m << "\"/>\n"
               << "    <tag k=\"planning_support_turn_angle_rad\" v=\"" <<
          support.turn_angle_rad << "\"/>\n"
               << "    <tag k=\"planning_support_straight_length_m\" v=\"" <<
          support.straight_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_maximum_curvature_inv_m\" v=\"" <<
          support.maximum_curvature_inv_m << "\"/>\n"
               << "    <tag k=\"planning_support_actual_maximum_curvature_inv_m\" v=\"" <<
          support.actual_maximum_curvature_inv_m << "\"/>\n"
               << "    <tag k=\"planning_support_kinematic_valid\" v=\"" <<
          (support.kinematic_valid ? "yes" : "no") << "\"/>\n"
               << "    <tag k=\"planning_support_outer_endpoint_unique\" v=\"" <<
          (support.outer_endpoint_unique ? "yes" : "no") << "\"/>\n"
               << "    <tag k=\"planning_support_outer_endpoint_route_polygon_edge_ids\" v=\"" <<
          commaSeparatedIds(support.outer_endpoint_route_polygon_edge_ids) << "\"/>\n"
               << "    <tag k=\"planning_support_outer_footprint_raw_overlap_edge_ids\" v=\"" <<
          commaSeparatedIds(support.outer_footprint_raw_overlap_edge_ids) << "\"/>\n"
               << "    <tag k=\"planning_support_outer_pose_isolation_scope\" "
          "v=\"nonadjacent_raw_route_centerlines\"/>\n"
               << "    <tag k=\"planning_support_outer_pose_isolation_derivation\" "
          "v=\"vehicle_footprint_circumradius_plus_endpoint_allowance\"/>\n"
               << "    <tag k=\"planning_support_required_outer_pose_nonadjacent_raw_centerline_isolation_m\" v=\"" <<
          support.required_outer_pose_nonadjacent_raw_centerline_isolation_m << "\"/>\n"
               << "    <tag k=\"planning_support_actual_outer_pose_nonadjacent_raw_centerline_isolation_m\" v=\"" <<
          support.actual_outer_pose_nonadjacent_raw_centerline_isolation_m << "\"/>\n"
               << "    <tag k=\"planning_support_outer_pose_nonadjacent_raw_centerline_count\" v=\"" <<
          support.outer_pose_nonadjacent_raw_centerline_count << "\"/>\n"
               << "    <tag k=\"planning_support_outer_pose_nearest_nonadjacent_raw_centerline_edge_ids\" v=\"" <<
          commaSeparatedIds(
            support.outer_pose_nearest_nonadjacent_raw_centerline_edge_ids) << "\"/>\n"
               << "    <tag k=\"planning_support_raw_overlap_single_transition\" v=\"" <<
          (support.raw_overlap_single_transition ? "yes" : "no") << "\"/>\n"
               << "    <tag k=\"planning_support_raw_overlap_transition_length_m\" v=\"" <<
          support.raw_overlap_transition_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_nonadjacent_raw_overlap_edge_ids\" v=\"" <<
          commaSeparatedIds(support.nonadjacent_raw_overlap_edge_ids) << "\"/>\n"
               << "    <tag k=\"planning_support_nonadjacent_raw_overlap_transition_length_m\" v=\"" <<
          support.nonadjacent_raw_overlap_transition_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_maximum_nonadjacent_raw_overlap_transition_length_m\" v=\"" <<
          support.maximum_nonadjacent_raw_overlap_transition_length_m << "\"/>\n"
               << "    <tag k=\"planning_support_outer_footprint_contained\" v=\"" <<
          (support.outer_footprint_contained ? "yes" : "no") << "\"/>\n"
               << "    <tag k=\"planning_support_connection_footprint_contained\" v=\"" <<
          (support.connection_footprint_contained ? "yes" : "no") << "\"/>\n"
               << "    <tag k=\"planning_support_candidate_pool_limit\" v=\"" <<
          support.candidate_pool_limit << "\"/>\n"
               << "    <tag k=\"planning_support_head_candidate_pool_size\" v=\"" <<
          support.head_candidate_pool_size << "\"/>\n"
               << "    <tag k=\"planning_support_tail_candidate_pool_size\" v=\"" <<
          support.tail_candidate_pool_size << "\"/>\n"
               << "    <tag k=\"planning_support_candidate_pair_evaluation_limit\" v=\"" <<
          support.candidate_pair_evaluation_limit << "\"/>\n"
               << "    <tag k=\"planning_support_candidate_pairs_tested\" v=\"" <<
          support.candidate_pairs_tested << "\"/>\n"
               << "    <tag k=\"planning_support_selected_candidate_pair_rank\" v=\"" <<
          support.selected_candidate_pair_rank << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_final_boundary_pairs\" v=\"" <<
          support.rejected_final_boundary_pairs << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_final_outer_membership_pairs\" v=\"" <<
          support.rejected_final_outer_membership_pairs << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_final_transition_pairs\" v=\"" <<
          support.rejected_final_transition_pairs << "\"/>\n"
               << "    <tag k=\"planning_support_rejected_final_containment_pairs\" v=\"" <<
          support.rejected_final_containment_pairs << "\"/>\n"
               << "    <tag k=\"planning_support_collision_scope\" "
          "v=\"route_polygon_single_transition_only\"/>\n";
      }
      if (summary.terminal_support_applied &&
        edge.id == summary.terminal_support_named_edge_id)
      {
        stream << "    <tag k=\"autoware_terminal_support\" v=\"yes\"/>\n"
               << "    <tag k=\"terminal_support_edge_ids\" v=\"" <<
          commaSeparatedIds(summary.terminal_support_edge_ids) << "\"/>\n"
               << "    <tag k=\"terminal_support_length_m\" v=\"" <<
          summary.terminal_support_length_m << "\"/>\n"
               << "    <tag k=\"named_route_source_length_m\" v=\"" <<
          summary.named_route_source_length_m << "\"/>\n"
               << "    <tag k=\"terminal_support_source\" "
          "v=\"closed_course_semantic_topology\"/>\n";
      }
    }
    const auto semantic_provenance = semantic_provenance_by_edge.find(edge.id);
    if (semantic_provenance != semantic_provenance_by_edge.end()) {
      stream << "    <tag k=\"source_route_edge_id\" v=\"" <<
        semantic_provenance->second.source_edge_id << "\"/>\n"
             << "    <tag k=\"source_start_s_m\" v=\"" <<
        semantic_provenance->second.source_start_s << "\"/>\n"
             << "    <tag k=\"source_end_s_m\" v=\"" <<
        semantic_provenance->second.source_end_s << "\"/>\n"
             << "    <tag k=\"source_edge_length_m\" v=\"" <<
        semantic_source_length_by_edge.at(
        semantic_provenance->second.source_edge_id) << "\"/>\n"
             << "    <tag k=\"source_arc_metric\" "
        "v=\"spatial_3d_processed_base_link\"/>\n"
             << "    <tag k=\"edge_partition_arc_metric\" v=\"planar_xy\"/>\n"
             << "    <tag k=\"semantic_segment\" v=\"yes\"/>\n";
    }
    stream
           << "    <tag k=\"route_edge_id\" v=\"" << edge.id << "\"/>\n"
           << "    <tag k=\"passable\" v=\"" << (edge.passable ? "yes" : "no") << "\"/>\n"
           << "    <tag k=\"generator_confidence\" v=\"" << edge.confidence << "\"/>\n"
           << "    <tag k=\"minimum_safe_width\" v=\"" << edge.minimum_safe_width << "\"/>\n"
           << "  </relation>\n";
  }
  stream << "</osm>\n";
}

void saveLanelet2Osm(
  const std::filesystem::path & path,
  const RouteGraph & graph,
  const Lanelet2Config & config,
  const bool autoware_ready,
  const Lanelet2AuthoringOptions & authoring)
{
  saveLanelet2OsmImpl(
    path, graph, config,
    autoware_ready ? Lanelet2OutputKind::kProductionReady :
    Lanelet2OutputKind::kCandidate,
    nullptr, authoring);
}

ClosedCourseLanelet2ExportSummary saveClosedCourseExperimentalLanelet2Osm(
  const std::filesystem::path & path,
  const RouteGraph & experimental_operational_graph,
  const Lanelet2Config & config,
  const ClosedCourseLanelet2ExportOptions & options,
  const Lanelet2AuthoringOptions & authoring)
{
  if (!(options.estimated_vehicle_width > 0.0) ||
    !std::isfinite(options.estimated_vehicle_width) ||
    !(options.estimated_front_extent > 0.0) ||
    !std::isfinite(options.estimated_front_extent) ||
    !(options.estimated_rear_extent > 0.0) ||
    !std::isfinite(options.estimated_rear_extent) ||
    !(options.estimated_minimum_turning_radius > 0.0) ||
    !std::isfinite(options.estimated_minimum_turning_radius) ||
    options.lateral_clearance_margin < 0.0 ||
    !std::isfinite(options.lateral_clearance_margin))
  {
    throw std::invalid_argument(
            "closed-course Lanelet2 vehicle dimensions/turning radius must be positive "
            "and margin nonnegative");
  }
  const std::set<std::string> valid_profiles{"custom", "small_robot", "car", "yaris"};
  const std::set<std::string> valid_base_references{
    "unspecified", "body_center", "rear_axle_ground_projection"};
  const std::set<std::string> valid_evidence_sources{
    "unknown", "measured", "catalog_estimated", "inferred"};
  const std::set<std::string> valid_evidence_confidences{
    "unknown", "low", "medium", "high"};
  const std::set<std::string> valid_centerline_sources{
    "recorded_trajectory", "edited_topology"};
  if (valid_profiles.count(options.vehicle_profile) == 0U ||
    valid_base_references.count(options.vehicle_base_reference) == 0U ||
    valid_evidence_sources.count(options.vehicle_dimensions_evidence_source) == 0U ||
    valid_evidence_confidences.count(
      options.vehicle_dimensions_evidence_confidence) == 0U ||
    valid_centerline_sources.count(options.centerline_source) == 0U ||
    ((options.vehicle_dimensions_evidence_source == "unknown") !=
    (options.vehicle_dimensions_evidence_confidence == "unknown")) ||
    (options.vehicle_dimensions_verified &&
    (options.vehicle_dimensions_evidence_source != "measured" ||
    options.vehicle_dimensions_evidence_confidence != "high")))
  {
    throw std::invalid_argument(
            "closed-course Lanelet2 vehicle profile/base/evidence/source metadata is invalid");
  }
  const double corridor_width = options.estimated_vehicle_width +
    2.0 * options.lateral_clearance_margin;
  std::optional<LosslessSemanticRouteGraphAudit> semantic_audit;
  if ((authoring.semantic_source_graph == nullptr) !=
    (authoring.semantic_edge_provenance == nullptr))
  {
    throw std::invalid_argument(
            "closed-course semantic source graph and provenance must be supplied together");
  }
  if (authoring.semantic_source_graph != nullptr) {
    if (authoring.terminal_support) {
      throw std::invalid_argument(
              "terminal support is not composable with lossless semantic segmentation");
    }
    SemanticRouteGraphResult semantic_graph;
    semantic_graph.graph = experimental_operational_graph;
    semantic_graph.edge_provenance = *authoring.semantic_edge_provenance;
    semantic_audit = validateLosslessSemanticRouteGraph(
      *authoring.semantic_source_graph, semantic_graph);
  }
  ExperimentalGraphSelection selection = buildClosedCourseGraphSelection(
    experimental_operational_graph, options, authoring.named_route,
    authoring.semantic_source_graph, authoring.semantic_edge_provenance);
  if (authoring.terminal_support) {
    applyTerminalSupportToSelection(
      selection, experimental_operational_graph, options,
      authoring.named_route,
      *authoring.terminal_support);
  }
  if (semantic_audit) {
    std::set<std::uint64_t> selected_output_ids;
    for (const RouteEdge & edge : selection.graph.edges) {
      selected_output_ids.insert(edge.id);
    }
    std::set<std::uint64_t> represented_source_ids;
    for (const SemanticRouteEdgeProvenance & provenance :
      *authoring.semantic_edge_provenance)
    {
      if (selected_output_ids.count(provenance.edge_id) != 0U) {
        represented_source_ids.insert(provenance.source_edge_id);
      }
    }
    selection.summary.source_physical_edges = semantic_audit->source_edges;
    selection.summary.exported_physical_edges = represented_source_ids.size();
    selection.summary.exported_lanelet_segments =
      selection.graph.edges.size() - selection.summary.synthetic_planning_support.size();
  }
  const ExperimentalLanelet2Metadata metadata{options, selection.summary, corridor_width};
  saveLanelet2OsmImpl(
    path, selection.graph, config,
    Lanelet2OutputKind::kClosedCourseExperimental, &metadata, authoring);
  return selection.summary;
}

void saveMapProjectorInfo(const std::filesystem::path & path)
{
  std::ofstream stream = openOutput(path);
  stream << "projector_type: Local\n";
}

void saveOccupancyGridYaml(
  const std::filesystem::path & path,
  const std::string & image_filename,
  const OccupancyGrid2D & grid)
{
  std::ofstream stream = openOutput(path);
  stream << "image: " << yamlQuote(image_filename) << "\n"
         << "mode: trinary\n"
         << "resolution: " << grid.resolution() << "\n"
         << "origin: [" << grid.originX() << ", " << grid.originY() << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
}

void saveGenerationReport(
  const std::filesystem::path & path,
  const MappingDataset & dataset,
  const PipelineResult & result,
  const ApplicationConfig & application_config,
  const AutowareReplayCandidateResult * autoware_replay_candidate)
{
  std::ofstream stream = openOutput(path);
  const DatasetStatistics & reader = dataset.statistics;
  const GenerationStatistics & generation = result.generation.statistics;
  const GeneratorConfig & config = application_config.generator;
  double observed_route_length = 0.0;
  double observed_route_planar_length = 0.0;
  for (const RouteEdge & edge : result.generation.observed_route_graph.edges) {
    observed_route_length += edge.length;
    observed_route_planar_length += polylineLength2d(edge.centerline);
  }
  double trajectory_planar_length = 0.0;
  for (std::size_t index = 1U;
    index < result.generation.processed_trajectory.size(); ++index)
  {
    trajectory_planar_length += distance2d(
      result.generation.processed_trajectory[index - 1U].world_from_body.translation,
      result.generation.processed_trajectory[index].world_from_body.translation);
  }
  const char * traversability_status = generation.passable_physical_edges == 0U ?
    "failed" :
    (generation.impassable_physical_edges == 0U ? "pass" : "partial");
  const char * output_map_type = application_config.output.target_mode == "autoware" ?
    "vector_map" :
    (application_config.output.target_mode == "nav2" ? "navigation_map" : "both");
  stream << "schema_version: 6\n"
         << "generator_version: \"" LMMG_PROJECT_VERSION "\"\n"
         << "generation_completed: true\n"
         << "traversability_status: " << yamlQuote(traversability_status) << "\n"
         << "input_type: " << yamlQuote(application_config.input_type) << "\n"
         << "world_frame: " << yamlQuote(dataset.world_frame) << "\n"
         << "input:\n"
         << "  type: " << yamlQuote(application_config.input_type) << "\n";
  if (application_config.input_type == "glim") {
    stream << "  map_path: " << yamlQuote(application_config.glim.map_path.string()) << "\n"
           << "  trajectory_path: "
           << yamlQuote(application_config.glim.trajectory_path.string()) << "\n"
           << "  trajectory_frame: "
           << yamlQuote(application_config.glim.trajectory_frame) << "\n";
  } else if (application_config.input_type == "rosbag2") {
    const RosbagInputConfig & input = application_config.rosbag2;
    stream << "  bag_path: " << yamlQuote(input.bag_path.string()) << "\n"
           << "  storage_id: " << yamlQuote(input.storage_id) << "\n"
           << "  pointcloud_topic: " << yamlQuote(input.pointcloud_topic) << "\n"
           << "  pointcloud_mode: " << yamlQuote(input.pointcloud_mode) << "\n"
           << "  pose_source: " << yamlQuote(input.pose_source) << "\n"
           << "  pose_topic: " << yamlQuote(input.pose_topic) << "\n"
           << "  tf_topic: " << yamlQuote(input.tf_topic) << "\n"
           << "  tf_static_topic: " << yamlQuote(input.tf_static_topic) << "\n"
           << "  configured_world_frame: " << yamlQuote(input.world_frame) << "\n"
           << "  base_frame: " << yamlQuote(input.base_frame) << "\n"
           << "  sensor_frame: " << yamlQuote(input.sensor_frame) << "\n"
           << "  pose_reference_frame: " << yamlQuote(input.pose_reference_frame) << "\n"
           << "  strict_frame_check: " << (input.strict_frame_check ? "true" : "false") << "\n"
           << "  use_header_stamp: " << (input.use_header_stamp ? "true" : "false") << "\n"
           << "  maximum_pose_gap_sec: " << input.maximum_pose_gap_sec << "\n"
           << "  deskew:\n"
           << "    enabled: " << (input.deskew.enabled ? "true" : "false") << "\n"
           << "    point_time_field: " << yamlQuote(input.deskew.point_time_field) << "\n"
           << "    point_time_reference: "
           << yamlQuote(input.deskew.point_time_reference) << "\n"
           << "    point_time_scale_sec: " << input.deskew.point_time_scale_sec << "\n"
           << "    point_time_offset_sec: " << input.deskew.point_time_offset_sec << "\n";
  }
  stream << "extrinsics:\n"
         << "  source: " << yamlQuote(application_config.extrinsics.source) << "\n"
         << "  calibration_source: " <<
    yamlQuote(application_config.extrinsics.calibration_source) << "\n"
         << "  calibration_confidence: " <<
    yamlQuote(application_config.extrinsics.calibration_confidence) << "\n"
         << "  verified: " <<
    (application_config.extrinsics.verified ? "true" : "false") << "\n";
  if (application_config.extrinsics.source == "parameters") {
    if (!application_config.extrinsics.requested_base_from_sensor) {
      throw std::invalid_argument(
              "parameter extrinsics report requires the exact requested transform");
    }
    const Transform & requested_extrinsic =
      *application_config.extrinsics.requested_base_from_sensor;
    const Transform & effective_extrinsic = application_config.extrinsics.base_from_sensor;
    // This transform is restored verbatim by the generation contract for GUI
    // regeneration.  Unlike review-only report values, it must survive a
    // double -> text -> double round trip exactly; otherwise normalizing the
    // rounded quaternion again can perturb generated geometry and invalidate
    // fail-closed Route fingerprints.
    stream << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "  translation: [" << requested_extrinsic.translation.x << ", "
           << requested_extrinsic.translation.y << ", "
           << requested_extrinsic.translation.z << "]\n"
           << "  quaternion_xyzw: [" << requested_extrinsic.rotation.x << ", "
           << requested_extrinsic.rotation.y << ", "
           << requested_extrinsic.rotation.z << ", "
           << requested_extrinsic.rotation.w << "]\n"
           << "  effective_quaternion_xyzw: [" << effective_extrinsic.rotation.x << ", "
           << effective_extrinsic.rotation.y << ", "
           << effective_extrinsic.rotation.z << ", "
           << effective_extrinsic.rotation.w << "]\n"
           << std::setprecision(12);
  } else {
    stream << "  transform_recorded_in_report: false\n";
  }
  stream << "reader:\n"
         << "  pose_messages: " << reader.pose_messages << "\n"
         << "  pointcloud_messages: " << reader.pointcloud_messages << "\n"
         << "  used_pointcloud_messages: " << reader.used_pointcloud_messages << "\n"
         << "  dropped_pointcloud_messages: " << reader.dropped_pointcloud_messages << "\n"
         << "  decoded_points: " << reader.decoded_points << "\n"
         << "  accepted_points: " << reader.accepted_points << "\n"
         << "  map_voxels: " << reader.map_voxels << "\n"
         << "generation:\n"
         << "  raw_trajectory_poses: " << generation.raw_trajectory_poses << "\n"
         << "  processed_trajectory_poses: " << generation.processed_trajectory_poses << "\n"
         << "  raw_trajectory_preserved: " <<
    (generation.raw_trajectory_preserved ? "true" : "false") << "\n"
         << "  corrected_position_jitter_poses: " <<
    generation.corrected_position_jitter_poses << "\n"
         << "  corrected_position_jitter_runs: " <<
    generation.corrected_position_jitter_runs << "\n"
         << "  maximum_planar_position_correction_m: " <<
    generation.maximum_planar_position_correction_m << "\n"
         << "  planar_length_before_position_jitter_correction_m: " <<
    generation.planar_length_before_position_jitter_correction_m << "\n"
         << "  planar_length_after_position_jitter_correction_m: " <<
    generation.planar_length_after_position_jitter_correction_m << "\n"
         << "  trajectory_length: " << generation.trajectory_length << "\n"
         << "  trajectory_planar_length: " << trajectory_planar_length << "\n"
         << "  observed_driven_route_nodes: " <<
    result.generation.observed_route_graph.nodes.size() << "\n"
         << "  observed_driven_route_edges: " <<
    result.generation.observed_route_graph.edges.size() << "\n"
         << "  observed_driven_route_length: " << observed_route_length << "\n"
         << "  observed_driven_route_planar_length: " <<
    observed_route_planar_length << "\n"
         << "  observed_driven_route_length_coverage: " <<
    (trajectory_planar_length > 1.0e-12 ?
    observed_route_planar_length / trajectory_planar_length : 0.0) << "\n"
         << "  observed_driven_route_spatial_length_coverage: " <<
    (generation.trajectory_length > 1.0e-12 ?
    observed_route_length / generation.trajectory_length : 0.0) << "\n"
         << "  obstacle_points: " << generation.obstacle_points << "\n"
         << "  obstacle_candidate_points: " << result.grids.obstacle_candidate_points << "\n"
         << "  low_support_obstacle_points: " << result.grids.low_support_obstacle_points << "\n"
         << "  low_support_obstacle_cells: " << result.grids.low_support_obstacle_cells << "\n"
         << "  obstacle_cells: " << generation.obstacle_cells << "\n"
         << "  trajectory_cleared_obstacle_cells: "
         << generation.trajectory_cleared_obstacle_cells << "\n"
         << "  inflated_obstacle_cells: " << generation.inflated_obstacle_cells << "\n"
         << "  locally_modelled_ground_cells: "
         << result.grids.locally_modelled_ground_cells << "\n"
         << "  ground_observation_free_cells: "
         << result.grids.ground_observation_free_cells << "\n"
         << "  explicit_free_cells: "
         << result.grids.observed_free_grid.occupiedCellCount() << "\n"
         << "  ground_unknown_points: " << result.grids.ground_unknown_points << "\n"
         << "  unknown_cells: " << result.grids.unknown_cells << "\n"
         << "  has_multi_scan_observation_support: "
         << (result.grids.has_multi_scan_observation_support ? "true" : "false") << "\n"
         << "  unknown_treated_as_occupied: "
         << (result.grids.unknown_treated_as_occupied ? "true" : "false") << "\n"
         << "  orientation_aware_footprint: "
         << (result.grids.orientation_aware_footprint ? "true" : "false") << "\n"
         << "  floor_fallback_samples: " << result.grids.fallback_ground_samples << "\n"
         << "  route_nodes: " << generation.route_nodes << "\n"
         << "  route_edges: " << generation.route_edges << "\n"
         << "  physical_route_edges: " << generation.physical_route_edges << "\n"
         << "  passable_physical_edges: " << generation.passable_physical_edges << "\n"
         << "  impassable_edges: " << generation.impassable_edges << "\n"
         << "  impassable_physical_edges: " << generation.impassable_physical_edges << "\n"
         << "  minimum_safe_width: " << generation.minimum_safe_width << "\n";
  const RouteGeometryAudit & observed_audit =
    result.generation.observed_route_geometry_audit;
  const RouteGeometryAudit & topology_audit =
    result.generation.topology_route_geometry_audit;
  stream << "centerline_geometry_audit:\n"
         << "  elevation_source: \"processed_base_link_trajectory\"\n"
         << "  local_ground_role: \"traversability_only\"\n"
         << "  edge_partition_arc_metric: \"planar_xy\"\n"
         << "  semantic_source_arc_metric: \"spatial_3d_processed_base_link\"\n"
         << "  observed_replay:\n"
         << "    valid: " << (observed_audit.valid ? "true" : "false") << "\n"
         << "    source_pose_count: " << observed_audit.source_pose_count << "\n"
         << "    source_segments_evaluated: " <<
    observed_audit.source_segments_evaluated << "\n"
         << "    represented_source_pose_count: " <<
    observed_audit.represented_source_pose_count << "\n"
         << "    source_pose_projection_coverage: " <<
    observed_audit.source_pose_projection_coverage << "\n"
         << "    source_planar_length_m: " << observed_audit.source_planar_length << "\n"
         << "    route_planar_length_m: " << observed_audit.route_planar_length << "\n"
         << "    planar_length_coverage: " << observed_audit.planar_length_coverage << "\n"
         << "    source_spatial_length_m: " << observed_audit.source_spatial_length << "\n"
         << "    route_spatial_length_m: " << observed_audit.route_spatial_length << "\n"
         << "    spatial_length_coverage: " << observed_audit.spatial_length_coverage << "\n"
         << "    source_start_xyz: [" << observed_audit.source_start.x << ", " <<
    observed_audit.source_start.y << ", " << observed_audit.source_start.z << "]\n"
         << "    source_end_xyz: [" << observed_audit.source_end.x << ", " <<
    observed_audit.source_end.y << ", " << observed_audit.source_end.z << "]\n"
         << "    segments_evaluated: " << observed_audit.segments_evaluated << "\n"
         << "    maximum_absolute_delta_z_m: " <<
    observed_audit.maximum_absolute_delta_z << "\n"
         << "    maximum_delta_z_edge_id: " <<
    observed_audit.maximum_delta_z_edge_id << "\n"
         << "    maximum_delta_z_segment_index: " <<
    observed_audit.maximum_delta_z_segment_index << "\n"
         << "    maximum_absolute_grade: " << observed_audit.maximum_absolute_grade << "\n"
         << "    maximum_grade_edge_id: " << observed_audit.maximum_grade_edge_id << "\n"
         << "    maximum_grade_segment_index: " <<
    observed_audit.maximum_grade_segment_index << "\n"
         << "    nonfinite_segments: " << observed_audit.nonfinite_segments << "\n"
         << "    zero_horizontal_distance_z_change_segments: " <<
    observed_audit.zero_horizontal_distance_z_change_segments << "\n"
         << "  physical_topology:\n"
         << "    valid: " << (topology_audit.valid ? "true" : "false") << "\n"
         << "    route_planar_length_m: " << topology_audit.route_planar_length << "\n"
         << "    route_spatial_length_m: " << topology_audit.route_spatial_length << "\n"
         << "    segments_evaluated: " << topology_audit.segments_evaluated << "\n"
         << "    maximum_absolute_delta_z_m: " <<
    topology_audit.maximum_absolute_delta_z << "\n"
         << "    maximum_delta_z_edge_id: " << topology_audit.maximum_delta_z_edge_id << "\n"
         << "    maximum_delta_z_segment_index: " <<
    topology_audit.maximum_delta_z_segment_index << "\n"
         << "    maximum_absolute_grade: " << topology_audit.maximum_absolute_grade << "\n"
         << "    maximum_grade_edge_id: " << topology_audit.maximum_grade_edge_id << "\n"
         << "    maximum_grade_segment_index: " <<
    topology_audit.maximum_grade_segment_index << "\n"
         << "    nonfinite_segments: " << topology_audit.nonfinite_segments << "\n"
         << "    zero_horizontal_distance_z_change_segments: " <<
    topology_audit.zero_horizontal_distance_z_change_segments << "\n";
  if (autoware_replay_candidate != nullptr) {
    const AutowareReplayCandidateResult & candidate = *autoware_replay_candidate;
    stream << "autoware_replay_candidate:\n"
           << "  derivative_scope: \"autoware_lanelet_only\"\n"
           << "  lossless_observed_route_unchanged: true\n"
           << "  nav2_replay_unchanged: true\n"
           << "  terminal_localization_settling_verified: "
           << (candidate.explicit_verification ? "true" : "false") << "\n"
           << "  verification_provenance: "
           << yamlQuote(candidate.verification_provenance) << "\n"
           << "  terminal_tail_omitted: "
           << (candidate.terminal_tail_omitted ? "true" : "false") << "\n"
           << "  reason: " << yamlQuote(candidate.reason) << "\n"
           << "  source_edges: " << candidate.source_edges << "\n"
           << "  retained_edges: " << candidate.retained_edges << "\n"
           << "  omitted_edges: " << candidate.omitted_edges << "\n"
           << "  source_planar_length_m: " << candidate.source_length << "\n"
           << "  retained_planar_length_m: " << candidate.retained_length << "\n"
           << "  omitted_planar_length_m: " << candidate.omitted_length << "\n"
           << "  omitted_length_ratio: " << candidate.omitted_length_ratio << "\n"
           << "  connection_heading_jump_deg: "
           << candidate.connection_heading_jump_deg << "\n"
           << "  terminal_body_yaw_change_deg: "
           << candidate.terminal_body_yaw_change_deg << "\n";
  }
  stream << "parameters:\n"
         << "  map_voxel_size: " << config.map_builder.voxel_size << "\n"
         << "  map_minimum_range: " << config.map_builder.minimum_range << "\n"
         << "  map_maximum_range: " << config.map_builder.maximum_range << "\n"
         << "  map_minimum_z: " << config.map_builder.minimum_z << "\n"
         << "  map_maximum_z: " << config.map_builder.maximum_z << "\n"
         << "  map_minimum_observations_per_voxel: "
         << config.map_builder.minimum_observations_per_voxel << "\n"
         << "  map_scan_stride: " << config.map_builder.scan_stride << "\n"
         << "  map_point_stride: " << config.map_builder.point_stride << "\n"
         << "  trajectory_resample_interval: " << config.trajectory.resample_interval << "\n"
         << "  robot_profile: " << yamlQuote(config.robot.profile) << "\n"
         << "  robot_base_reference: " << yamlQuote(config.robot.base_reference) << "\n"
         << "  robot_footprint_model: " << yamlQuote(config.robot.footprint_model) << "\n"
         << "  robot_width: " << config.robot.width << "\n"
         << "  robot_front_extent: " << config.robot.front_extent << "\n"
         << "  robot_rear_extent: " << config.robot.rear_extent << "\n"
         << "  clearance_margin: " << config.robot.clearance_margin << "\n"
         << "  robot_minimum_collision_height: "
         << config.robot.minimum_collision_height << "\n"
         << "  robot_maximum_collision_height: "
         << config.robot.maximum_collision_height << "\n"
         << "  robot_dimensions_source: " <<
    yamlQuote(config.robot.dimensions_source) << "\n"
         << "  robot_dimensions_confidence: " <<
    yamlQuote(config.robot.dimensions_confidence) << "\n"
         << "  robot_dimensions_verified: "
         << (config.robot.dimensions_verified ? "true" : "false") << "\n"
         << "  robot_minimum_turning_radius: "
         << config.robot.minimum_turning_radius << "\n"
         << "  robot_allow_in_place_rotation: "
         << (config.robot.allow_in_place_rotation ? "true" : "false") << "\n"
         << "  robot_allow_reverse_motion: "
         << (config.robot.allow_reverse_motion ? "true" : "false") << "\n"
         << "  observed_trajectory_clearance_radius: "
         << config.traversability.observed_trajectory_clearance_radius << "\n"
         << "  free_space_evidence_mode: " <<
    yamlQuote(config.traversability.free_space_evidence_mode) << "\n"
         << "  minimum_ground_free_points_per_cell: " <<
    config.traversability.minimum_ground_free_points_per_cell << "\n"
         << "  maximum_ground_free_height: " <<
    config.traversability.maximum_ground_free_height << "\n"
         << "  trajectory_free_space_model: " <<
    yamlQuote(config.traversability.trajectory_free_space_model) << "\n"
         << "  trajectory_footprint_erosion_margin: " <<
    config.traversability.trajectory_footprint_erosion_margin << "\n"
         << "  grid_resolution: " << config.traversability.grid_resolution << "\n"
         << "  ground_cell_resolution: "
         << config.traversability.ground_cell_resolution << "\n"
         << "  ground_quantile: " << config.traversability.ground_quantile << "\n"
         << "  ground_quantile_weighting: \"voxel_observation_count\"\n"
         << "  ground_plane_radius: " << config.traversability.ground_plane_radius << "\n"
         << "  maximum_ground_slope: "
         << config.traversability.maximum_ground_slope << "\n"
         << "  maximum_ground_plane_residual: "
         << config.traversability.maximum_ground_plane_residual << "\n"
         << "  minimum_obstacle_points_per_cell: "
         << config.traversability.minimum_obstacle_points_per_cell << "\n"
         << "  obstacle_support_radius_cells: "
         << config.traversability.obstacle_support_radius_cells << "\n"
         << "  minimum_obstacle_neighbor_points: "
         << config.traversability.minimum_obstacle_neighbor_points << "\n"
         << "  minimum_obstacle_observations: "
         << config.traversability.minimum_obstacle_observations << "\n"
         << "  low_support_obstacle_cell_policy: \"unknown_unless_trajectory_swept\"\n"
         << "  unknown_space_policy: "
         << yamlQuote(config.traversability.unknown_space_policy) << "\n"
         << "  output_frame_id: " << yamlQuote(application_config.output.frame_id) << "\n"
         << "  output_map_type: " << yamlQuote(output_map_type) << "\n"
         << "  nav2_free_space_verified: " <<
    (application_config.output.nav2_free_space_verified ? "true" : "false") << "\n"
         << "  lanelet2_physical_boundaries_verified: " <<
    (application_config.output.lanelet2_physical_boundaries_verified ? "true" : "false") << "\n"
         << "  lanelet2_terminal_localization_settling_verified: " <<
    (config.lanelet2.terminal_localization_settling_verified ? "true" : "false") << "\n"
         << "warnings:";
  const bool has_autoware_candidate_warning =
    autoware_replay_candidate != nullptr &&
    (autoware_replay_candidate->terminal_tail_omitted ||
    autoware_replay_candidate->explicit_verification);
  if (result.generation.warnings.empty() && !has_autoware_candidate_warning) {
    stream << " []\n";
  } else {
    stream << '\n';
    for (const std::string & warning : result.generation.warnings) {
      stream << "  - " << yamlQuote(warning) << "\n";
    }
    if (autoware_replay_candidate != nullptr &&
      autoware_replay_candidate->terminal_tail_omitted)
    {
      stream << "  - " << yamlQuote(
        "Autoware Lanelet replay omitted a verified terminal localization-settling tail (" +
        std::to_string(autoware_replay_candidate->omitted_length) +
        " planar m); route_graph_observed_driven and Nav2 replay remain lossless") << "\n";
    } else if (autoware_replay_candidate != nullptr &&
      autoware_replay_candidate->explicit_verification)
    {
      stream << "  - " << yamlQuote(
        "lanelet2.terminal_localization_settling_verified=true did not satisfy the fixed "
        "Autoware terminal-tail gates; the full replay was retained (" +
        autoware_replay_candidate->reason + ")") << "\n";
    }
  }
}

}  // namespace lidar_mobility_map_generator
