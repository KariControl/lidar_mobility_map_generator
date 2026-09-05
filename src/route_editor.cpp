#include "lidar_mobility_map_generator/route_editor.hpp"

#include "lidar_mobility_map_generator/route_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

constexpr std::uint32_t kRouteEditOverlayVersion = 1U;

template<typename Container>
auto findById(Container & values, const std::uint64_t id)
{
  return std::find_if(values.begin(), values.end(), [&](const auto & value) {return value.id == id;});
}

void requireFinite(const Vec3 & point, const std::string & label)
{
  if (!finite(point)) {
    throw std::runtime_error(label + " must contain finite coordinates");
  }
}

bool hasDirectionReversal(const std::vector<Vec3> & points)
{
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    const Vec2 incoming{
      points[index].x - points[index - 1U].x,
      points[index].y - points[index - 1U].y};
    const Vec2 outgoing{
      points[index + 1U].x - points[index].x,
      points[index + 1U].y - points[index].y};
    if (norm(incoming) > 1.0e-9 && norm(outgoing) > 1.0e-9 && dot(incoming, outgoing) <= 0.0) {
      return true;
    }
  }
  return false;
}

void resetDerivedGeometry(RouteEdge & edge)
{
  edge.left_boundary.clear();
  edge.right_boundary.clear();
  edge.left_clearance.clear();
  edge.right_clearance.clear();
  edge.left_clearance_observed.clear();
  edge.right_clearance_observed.clear();
  edge.length = polylineLength(edge.centerline);
  edge.minimum_safe_width = 0.0;
  edge.maximum_curvature = maximumPolylineCurvature(edge.centerline);
  edge.confidence = 0.0;
  edge.recommended_speed_mps = 0.0;
  edge.passable = false;
  edge.corridor_geometry_valid = false;
  edge.validation_errors.clear();
}

void addUniqueError(std::vector<std::string> & errors, const std::string & error)
{
  if (std::find(errors.begin(), errors.end(), error) == errors.end()) {
    errors.push_back(error);
  }
}

void addEdgeError(RouteEdge & edge, RouteEntityMetadata & metadata, const std::string & error)
{
  addUniqueError(edge.validation_errors, error);
  addUniqueError(metadata.validation_errors, error);
  edge.passable = false;
}

bool isAdvisoryValidationError(const std::string & error)
{
  // The contiguous known-free strip can be wide enough for the platform even
  // when its outer extent terminates at UNKNOWN. Keep that route operational,
  // but expose the incomplete-clearance evidence to the operator.
  return error == "unknown_clearance";
}

bool isOperationalStatus(const RouteValidationStatus status)
{
  return status == RouteValidationStatus::kValid ||
         status == RouteValidationStatus::kWarning;
}

bool matchingGridGeometry(
  const OccupancyGrid2D & lhs, const OccupancyGrid2D & rhs)
{
  const double tolerance = 1.0e-9;
  return lhs.width() == rhs.width() && lhs.height() == rhs.height() &&
    std::abs(lhs.originX() - rhs.originX()) <= tolerance &&
    std::abs(lhs.originY() - rhs.originY()) <= tolerance &&
    std::abs(lhs.resolution() - rhs.resolution()) <= tolerance;
}

void requireMatchingGridGeometry(
  const OccupancyGrid2D & lhs, const OccupancyGrid2D & rhs)
{
  if (!matchingGridGeometry(lhs, rhs)) {
    throw std::invalid_argument(
            "inflated obstacle and UNKNOWN grids must have identical geometry");
  }
}

double unknownConfigurationRadius(
  const RobotConfig & robot, const double grid_resolution)
{
  const double discretization_margin = 0.5 * std::sqrt(2.0) * grid_resolution;
  if (robot.footprint_model == "circle") {
    return 0.5 * robot.width + robot.clearance_margin + discretization_margin;
  }
  if (robot.footprint_model == "rectangle") {
    const double longitudinal =
      std::max(robot.front_extent, robot.rear_extent) + robot.clearance_margin;
    const double lateral = 0.5 * robot.width + robot.clearance_margin;
    // UNKNOWN has no observation orientation. Its circumscribed radius is a
    // conservative, yaw-independent configuration-space transform.
    return std::hypot(longitudinal, lateral) + discretization_margin;
  }
  throw std::invalid_argument("robot footprint_model must be circle or rectangle");
}

OccupancyGrid2D configurationSpaceUnknownGrid(
  const OccupancyGrid2D & unknown_grid, const RobotConfig & robot)
{
  const double radius = unknownConfigurationRadius(robot, unknown_grid.resolution());
  OccupancyGrid2D result = unknown_grid.inflated(radius);

  // Space beyond the grid is UNKNOWN too. Erode the usable map extent by the
  // same footprint radius so a center cell near the boundary cannot be marked
  // safe while part of the platform lies outside all evidence.
  const double minimum_x = unknown_grid.originX();
  const double minimum_y = unknown_grid.originY();
  const double maximum_x = minimum_x +
    static_cast<double>(unknown_grid.width()) * unknown_grid.resolution();
  const double maximum_y = minimum_y +
    static_cast<double>(unknown_grid.height()) * unknown_grid.resolution();
  for (std::int64_t y = 0; y < static_cast<std::int64_t>(unknown_grid.height()); ++y) {
    for (std::int64_t x = 0; x < static_cast<std::int64_t>(unknown_grid.width()); ++x) {
      const Vec2 center = unknown_grid.cellCenter(x, y);
      if (center.x - minimum_x <= radius || maximum_x - center.x <= radius ||
        center.y - minimum_y <= radius || maximum_y - center.y <= radius)
      {
        result.setOccupied(x, y);
      }
    }
  }
  return result;
}

bool rectangleFootprintTouchesUnknownAtPose(
  const OccupancyGrid2D & unknown_grid, const Vec3 & position, const double yaw,
  const RobotConfig & robot, const bool include_clearance)
{
  // The closed-course swept-footprint mode classifies cells by their centres,
  // as the trajectory FREE raster does. Production additionally expands by a
  // half-cell diagonal and the configured clearance margin.
  const double half_cell_diagonal =
    0.5 * std::sqrt(2.0) * unknown_grid.resolution();
  // The trajectory sweep and the UNKNOWN mask are centre-sampled rasters. An
  // inward half-cell tolerance avoids rejecting the same swept boundary due
  // only to sub-cell placement. The separately supplied inflated obstacle
  // grid still collision-checks the full body plus clearance.
  const double raster_margin = include_clearance ?
    half_cell_diagonal : -half_cell_diagonal;
  const double clearance = include_clearance ? robot.clearance_margin : 0.0;
  const double front = robot.front_extent + clearance + raster_margin;
  const double rear = robot.rear_extent + clearance + raster_margin;
  const double lateral = 0.5 * robot.width + clearance + raster_margin;
  const double search_radius = std::hypot(std::max(front, rear), lateral);
  const std::int64_t minimum_x = static_cast<std::int64_t>(std::floor(
      (position.x - search_radius - unknown_grid.originX()) / unknown_grid.resolution()));
  const std::int64_t maximum_x = static_cast<std::int64_t>(std::floor(
      (position.x + search_radius - unknown_grid.originX()) / unknown_grid.resolution()));
  const std::int64_t minimum_y = static_cast<std::int64_t>(std::floor(
      (position.y - search_radius - unknown_grid.originY()) / unknown_grid.resolution()));
  const std::int64_t maximum_y = static_cast<std::int64_t>(std::floor(
      (position.y + search_radius - unknown_grid.originY()) / unknown_grid.resolution()));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
    for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
      const double world_x = unknown_grid.originX() +
        (static_cast<double>(x) + 0.5) * unknown_grid.resolution();
      const double world_y = unknown_grid.originY() +
        (static_cast<double>(y) + 0.5) * unknown_grid.resolution();
      const double dx = world_x - position.x;
      const double dy = world_y - position.y;
      const double local_x = cosine * dx + sine * dy;
      const double local_y = -sine * dx + cosine * dy;
      if (local_x < -rear || local_x > front || std::abs(local_y) > lateral) {
        continue;
      }
      // The map exterior is UNKNOWN by definition, as is every set cell in
      // the supplied mask.
      if (!unknown_grid.containsCell(x, y) || unknown_grid.isOccupied(x, y)) {
        return true;
      }
    }
  }
  return false;
}

bool rectangleRouteFootprintTouchesUnknown(
  const RouteEdge & edge, const OccupancyGrid2D & unknown_grid,
  const RobotConfig & robot, const bool include_clearance)
{
  if (edge.centerline.size() < 2U) {
    return true;
  }
  const double sample_step = std::max(0.01, 0.5 * unknown_grid.resolution());
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec3 & start = edge.centerline[index - 1U];
    const Vec3 & end = edge.centerline[index];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::hypot(dx, dy);
    if (length <= 1.0e-9) {
      continue;
    }
    const double yaw = std::atan2(dy, dx);
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / sample_step)));
    for (std::size_t piece = 0U; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      if (rectangleFootprintTouchesUnknownAtPose(
          unknown_grid, start + (end - start) * ratio, yaw, robot, include_clearance))
      {
        return true;
      }
    }
  }
  return false;
}

std::vector<double> routeTangentYaws(const std::vector<Vec3> & points)
{
  std::vector<double> result(points.size(), 0.0);
  if (points.size() < 2U) {
    return result;
  }
  const auto direction = [&](const std::size_t first, const std::size_t second) {
      return normalized(Vec2{
          points[second].x - points[first].x,
          points[second].y - points[first].y});
    };
  for (std::size_t index = 0U; index < points.size(); ++index) {
    Vec2 tangent;
    if (index == 0U) {
      tangent = direction(0U, 1U);
    } else if (index + 1U == points.size()) {
      tangent = direction(index - 1U, index);
    } else {
      const Vec2 incoming = direction(index - 1U, index);
      const Vec2 outgoing = direction(index, index + 1U);
      tangent = normalized(incoming + outgoing);
      if (norm(tangent) <= 1.0e-12) {
        tangent = norm(outgoing) > 1.0e-12 ? outgoing : incoming;
      }
    }
    if (norm(tangent) > 1.0e-12) {
      result[index] = std::atan2(tangent.y, tangent.x);
    } else if (index > 0U) {
      result[index] = result[index - 1U];
    }
  }
  return result;
}

bool rectangleFootprintIntersectsCell(
  const Vec3 & base, const double yaw, const RobotConfig & robot,
  const Vec2 & cell_center, const double cell_half_width)
{
  const double front = robot.front_extent + robot.clearance_margin;
  const double rear = robot.rear_extent + robot.clearance_margin;
  const double half_length = 0.5 * (front + rear);
  const double half_width = 0.5 * robot.width + robot.clearance_margin;
  const double center_offset = 0.5 * (front - rear);
  const Vec2 longitudinal{std::cos(yaw), std::sin(yaw)};
  const Vec2 lateral{-longitudinal.y, longitudinal.x};
  const Vec2 rectangle_center{
    base.x + center_offset * longitudinal.x,
    base.y + center_offset * longitudinal.y};
  const Vec2 delta{
    cell_center.x - rectangle_center.x,
    cell_center.y - rectangle_center.y};
  constexpr double tolerance = 1.0e-12;

  // Separating-axis test between the oriented body+clearance rectangle and an
  // axis-aligned occupancy-grid cell. Boundary contact counts as collision.
  if (std::abs(dot(delta, longitudinal)) >
    half_length + cell_half_width *
    (std::abs(longitudinal.x) + std::abs(longitudinal.y)) + tolerance)
  {
    return false;
  }
  if (std::abs(dot(delta, lateral)) >
    half_width + cell_half_width *
    (std::abs(lateral.x) + std::abs(lateral.y)) + tolerance)
  {
    return false;
  }
  if (std::abs(delta.x) >
    cell_half_width + half_length * std::abs(longitudinal.x) +
    half_width * std::abs(lateral.x) + tolerance)
  {
    return false;
  }
  if (std::abs(delta.y) >
    cell_half_width + half_length * std::abs(longitudinal.y) +
    half_width * std::abs(lateral.y) + tolerance)
  {
    return false;
  }
  return true;
}

bool rectangleFootprintTouchesObstacleAtPose(
  const OccupancyGrid2D & obstacle_grid, const Vec3 & position,
  const double yaw, const RobotConfig & robot)
{
  const double front = robot.front_extent + robot.clearance_margin;
  const double rear = robot.rear_extent + robot.clearance_margin;
  const double lateral = 0.5 * robot.width + robot.clearance_margin;
  const double search_radius = std::hypot(std::max(front, rear), lateral) +
    std::sqrt(0.5) * obstacle_grid.resolution();
  const std::int64_t minimum_x = static_cast<std::int64_t>(std::floor(
      (position.x - search_radius - obstacle_grid.originX()) /
      obstacle_grid.resolution()));
  const std::int64_t maximum_x = static_cast<std::int64_t>(std::floor(
      (position.x + search_radius - obstacle_grid.originX()) /
      obstacle_grid.resolution()));
  const std::int64_t minimum_y = static_cast<std::int64_t>(std::floor(
      (position.y - search_radius - obstacle_grid.originY()) /
      obstacle_grid.resolution()));
  const std::int64_t maximum_y = static_cast<std::int64_t>(std::floor(
      (position.y + search_radius - obstacle_grid.originY()) /
      obstacle_grid.resolution()));
  const double half_cell = 0.5 * obstacle_grid.resolution();
  for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
    for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
      if (!obstacle_grid.containsCell(x, y) || !obstacle_grid.isOccupied(x, y)) {
        continue;
      }
      if (rectangleFootprintIntersectsCell(
          position, yaw, robot, obstacle_grid.cellCenter(x, y), half_cell))
      {
        return true;
      }
    }
  }
  return false;
}

bool rectangleRouteFootprintTouchesObstacle(
  const RouteEdge & edge, const OccupancyGrid2D & obstacle_grid,
  const RobotConfig & robot)
{
  if (edge.centerline.size() < 2U) {
    return true;
  }
  const std::vector<double> vertex_yaws = routeTangentYaws(edge.centerline);
  const double sample_step = std::max(0.01, 0.5 * obstacle_grid.resolution());
  const double body_radius = std::hypot(
    std::max(robot.front_extent, robot.rear_extent) + robot.clearance_margin,
    0.5 * robot.width + robot.clearance_margin);
  bool sampled = false;
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec3 & start = edge.centerline[index - 1U];
    const Vec3 & end = edge.centerline[index];
    const double length = distance2d(start, end);
    if (length <= 1.0e-9) {
      continue;
    }
    const double start_yaw = vertex_yaws[index - 1U];
    const double yaw_delta = normalizeAngle(vertex_yaws[index] - start_yaw);
    // Bound the combined translation and corner motion, not the two terms
    // independently. Otherwise simultaneous translation and rotation can
    // leave a narrow unstamped gap between consecutive footprints.
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
        (length + std::abs(yaw_delta) * body_radius) / sample_step)));
    for (std::size_t piece = 0U; piece <= pieces; ++piece) {
      sampled = true;
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      if (rectangleFootprintTouchesObstacleAtPose(
          obstacle_grid, start + (end - start) * ratio,
          normalizeAngle(start_yaw + ratio * yaw_delta), robot))
      {
        return true;
      }
    }
  }
  // Degenerate XY geometry cannot establish a collision-free orientation.
  return !sampled;
}

bool circleFootprintTouchesUnknownAtPose(
  const OccupancyGrid2D & unknown_grid, const Vec3 & position,
  const RobotConfig & robot, const bool include_clearance)
{
  // Match the centre-sampled trajectory FREE raster. The closed-course path
  // uses an inward half-cell tolerance so the same swept boundary is not
  // rejected solely because the platform centre falls on another sub-cell
  // phase. Known obstacles are still tested on the independently supplied
  // full body-plus-clearance configuration-space grid.
  const double half_cell_diagonal =
    0.5 * std::sqrt(2.0) * unknown_grid.resolution();
  const double clearance = include_clearance ? robot.clearance_margin : 0.0;
  const double raster_margin = include_clearance ?
    half_cell_diagonal : -half_cell_diagonal;
  const double radius = std::max(
    0.0, 0.5 * robot.width + clearance + raster_margin);
  const std::int64_t minimum_x = static_cast<std::int64_t>(std::floor(
      (position.x - radius - unknown_grid.originX()) / unknown_grid.resolution()));
  const std::int64_t maximum_x = static_cast<std::int64_t>(std::floor(
      (position.x + radius - unknown_grid.originX()) / unknown_grid.resolution()));
  const std::int64_t minimum_y = static_cast<std::int64_t>(std::floor(
      (position.y - radius - unknown_grid.originY()) / unknown_grid.resolution()));
  const std::int64_t maximum_y = static_cast<std::int64_t>(std::floor(
      (position.y + radius - unknown_grid.originY()) / unknown_grid.resolution()));
  const double radius_squared = radius * radius;
  for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
    for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
      const double world_x = unknown_grid.originX() +
        (static_cast<double>(x) + 0.5) * unknown_grid.resolution();
      const double world_y = unknown_grid.originY() +
        (static_cast<double>(y) + 0.5) * unknown_grid.resolution();
      const double dx = world_x - position.x;
      const double dy = world_y - position.y;
      if (dx * dx + dy * dy > radius_squared + 1.0e-12) {
        continue;
      }
      if (!unknown_grid.containsCell(x, y) || unknown_grid.isOccupied(x, y)) {
        return true;
      }
    }
  }
  return false;
}

bool circleRouteFootprintTouchesUnknown(
  const RouteEdge & edge, const OccupancyGrid2D & unknown_grid,
  const RobotConfig & robot, const bool include_clearance)
{
  if (edge.centerline.size() < 2U) {
    return true;
  }
  const double sample_step = std::max(0.01, 0.5 * unknown_grid.resolution());
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec3 & start = edge.centerline[index - 1U];
    const Vec3 & end = edge.centerline[index];
    const double length = std::hypot(end.x - start.x, end.y - start.y);
    if (length <= 1.0e-9) {
      continue;
    }
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / sample_step)));
    for (std::size_t piece = 0U; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      if (circleFootprintTouchesUnknownAtPose(
          unknown_grid, start + (end - start) * ratio, robot, include_clearance))
      {
        return true;
      }
    }
  }
  return false;
}

void updateNodeTypes(RouteGraph & graph)
{
  std::map<std::uint64_t, std::set<std::uint64_t>> neighbors;
  for (const RouteEdge & edge : graph.edges) {
    neighbors[edge.from].insert(edge.to);
    neighbors[edge.to].insert(edge.from);
  }
  for (RouteNode & node : graph.nodes) {
    const std::size_t degree = neighbors[node.id].size();
    node.type = degree <= 1U ? RouteNodeType::kEndpoint :
      (degree >= 3U ? RouteNodeType::kJunction : RouteNodeType::kNormal);
  }
}

void validateStructure(const RouteGraph & graph)
{
  std::unordered_set<std::uint64_t> ids;
  for (const RouteNode & node : graph.nodes) {
    if (node.id == 0U || !ids.insert(node.id).second) {
      throw std::runtime_error("route graph contains a zero or duplicate node ID");
    }
    requireFinite(node.position, "route node position");
  }
  for (const RouteEdge & edge : graph.edges) {
    if (edge.id == 0U || !ids.insert(edge.id).second) {
      throw std::runtime_error("route graph IDs must be unique across nodes and edges");
    }
    if (edge.from == edge.to) {
      throw std::runtime_error("route graph self-loop edges are not allowed");
    }
    const auto from = findById(graph.nodes, edge.from);
    const auto to = findById(graph.nodes, edge.to);
    if (from == graph.nodes.end() || to == graph.nodes.end()) {
      throw std::runtime_error("route edge references a missing node");
    }
    if (edge.centerline.size() < 2U) {
      throw std::runtime_error("route edge centerline requires at least two points");
    }
    for (const Vec3 & point : edge.centerline) {
      requireFinite(point, "route edge centerline");
    }
    if (distance3d(edge.centerline.front(), from->position) > 1.0e-6 ||
      distance3d(edge.centerline.back(), to->position) > 1.0e-6)
    {
      throw std::runtime_error("route edge endpoints do not match referenced nodes");
    }
    if (edge.reverse_of) {
      const auto reverse = findById(graph.edges, *edge.reverse_of);
      if (reverse == graph.edges.end() || !reverse->reverse_of || *reverse->reverse_of != edge.id ||
        reverse->from != edge.to || reverse->to != edge.from)
      {
        throw std::runtime_error("route edge reverse_of relationship is not reciprocal");
      }
    }
  }
}

std::vector<double> cumulativeArc(const std::vector<Vec3> & points)
{
  std::vector<double> result(points.size(), 0.0);
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result[index] = result[index - 1U] + distance3d(points[index - 1U], points[index]);
  }
  return result;
}

Vec3 sampleAtArc(
  const std::vector<Vec3> & points, const std::vector<double> & arc, const double target)
{
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
}

std::vector<Vec3> slicePolyline(
  const std::vector<Vec3> & points, const double begin, const double end)
{
  const std::vector<double> arc = cumulativeArc(points);
  std::vector<Vec3> result{sampleAtArc(points, arc, begin)};
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    if (arc[index] > begin + 1.0e-9 && arc[index] < end - 1.0e-9) {
      result.push_back(points[index]);
    }
  }
  result.push_back(sampleAtArc(points, arc, end));
  result.erase(std::unique(
      result.begin(), result.end(),
      [](const Vec3 & lhs, const Vec3 & rhs) {return distance3d(lhs, rhs) <= 1.0e-9;}),
    result.end());
  return result;
}

std::vector<std::uint64_t> combinedSourceIds(
  const RouteEntityMetadata & metadata, const RouteEdge & edge)
{
  std::vector<std::uint64_t> result = metadata.source_ids;
  if (result.empty()) {
    result.push_back(edge.id);
  }
  if (edge.reverse_of && std::find(result.begin(), result.end(), *edge.reverse_of) == result.end()) {
    result.push_back(*edge.reverse_of);
  }
  return result;
}

RouteProvenance derivedProvenance(const RouteEntityMetadata & source)
{
  return source.provenance == RouteProvenance::kManual ?
    RouteProvenance::kManual : RouteProvenance::kEditedGenerated;
}

void eraseEdge(EditedRouteGraph & edited, const std::uint64_t edge_id)
{
  edited.graph.edges.erase(std::remove_if(
      edited.graph.edges.begin(), edited.graph.edges.end(),
      [&](const RouteEdge & edge) {return edge.id == edge_id;}), edited.graph.edges.end());
  edited.edge_metadata.erase(edge_id);
}

bool idExists(const RouteGraph & graph, const std::uint64_t id)
{
  return findById(graph.nodes, id) != graph.nodes.end() ||
         findById(graph.edges, id) != graph.edges.end();
}

RouteEdge makeEdge(
  const std::uint64_t id, const std::uint64_t from, const std::uint64_t to,
  std::vector<Vec3> centerline)
{
  RouteEdge edge;
  edge.id = id;
  edge.from = from;
  edge.to = to;
  edge.centerline = std::move(centerline);
  resetDerivedGeometry(edge);
  return edge;
}

void applyOperation(EditedRouteGraph & edited, const RouteEditOperation & operation)
{
  switch (operation.type) {
    case RouteEditOperationType::kClearGraph: {
        edited.graph.nodes.clear();
        edited.graph.edges.clear();
        edited.node_metadata.clear();
        edited.edge_metadata.clear();
        break;
      }
    case RouteEditOperationType::kAddNode: {
        if (operation.node_id == 0U || idExists(edited.graph, operation.node_id)) {
          throw std::runtime_error("ADD_NODE uses an invalid or duplicate ID");
        }
        requireFinite(operation.position, "ADD_NODE position");
        edited.graph.nodes.push_back({operation.node_id, operation.position, RouteNodeType::kEndpoint});
        edited.node_metadata[operation.node_id] = {
          RouteProvenance::kManual, RouteValidationStatus::kUnvalidated, {}, {}, false};
        break;
      }
    case RouteEditOperationType::kMoveNode: {
        auto node = findById(edited.graph.nodes, operation.node_id);
        if (node == edited.graph.nodes.end()) {
          throw std::runtime_error("MOVE_NODE references a missing node");
        }
        requireFinite(operation.position, "MOVE_NODE position");
        node->position = operation.position;
        RouteEntityMetadata & node_metadata = edited.node_metadata[operation.node_id];
        node_metadata.provenance = derivedProvenance(node_metadata);
        node_metadata.validation_status = RouteValidationStatus::kUnvalidated;
        for (RouteEdge & edge : edited.graph.edges) {
          if (edge.from == operation.node_id) {
            edge.centerline.front() = operation.position;
          }
          if (edge.to == operation.node_id) {
            edge.centerline.back() = operation.position;
          }
          if (edge.from == operation.node_id || edge.to == operation.node_id) {
            RouteEntityMetadata & metadata = edited.edge_metadata[edge.id];
            metadata.provenance = derivedProvenance(metadata);
            metadata.validation_status = RouteValidationStatus::kUnvalidated;
            metadata.requires_orientation_collision_validation = true;
            resetDerivedGeometry(edge);
          }
        }
        break;
      }
    case RouteEditOperationType::kDeleteNode: {
        const auto node = findById(edited.graph.nodes, operation.node_id);
        if (node == edited.graph.nodes.end()) {
          throw std::runtime_error("DELETE_NODE references a missing node");
        }
        std::vector<std::uint64_t> incident;
        for (const RouteEdge & edge : edited.graph.edges) {
          if (edge.from == operation.node_id || edge.to == operation.node_id) {
            incident.push_back(edge.id);
          }
        }
        if (!incident.empty() && !operation.cascade) {
          throw std::runtime_error("DELETE_NODE has incident edges and cascade is false");
        }
        for (const std::uint64_t id : incident) {
          eraseEdge(edited, id);
        }
        edited.graph.nodes.erase(node);
        edited.node_metadata.erase(operation.node_id);
        break;
      }
    case RouteEditOperationType::kAddEdge: {
        if (operation.edge_id == 0U || idExists(edited.graph, operation.edge_id) ||
          operation.from_id == operation.to_id)
        {
          throw std::runtime_error("ADD_EDGE uses an invalid, duplicate, or self-loop ID");
        }
        const auto from = findById(edited.graph.nodes, operation.from_id);
        const auto to = findById(edited.graph.nodes, operation.to_id);
        if (from == edited.graph.nodes.end() || to == edited.graph.nodes.end()) {
          throw std::runtime_error("ADD_EDGE references a missing node");
        }
        if (operation.polyline.size() < 2U) {
          throw std::runtime_error("ADD_EDGE polyline requires at least two points");
        }
        std::vector<Vec3> geometry = operation.polyline;
        for (const Vec3 & point : geometry) {
          requireFinite(point, "ADD_EDGE polyline");
        }
        geometry.front() = from->position;
        geometry.back() = to->position;
        RouteEdge forward = makeEdge(
          operation.edge_id, operation.from_id, operation.to_id, std::move(geometry));
        RouteEntityMetadata forward_metadata{
          RouteProvenance::kManual, RouteValidationStatus::kUnvalidated, {}, {}, false, true};
        if (operation.direction == RouteDirection::kBidirectional) {
          if (operation.reverse_edge_id == 0U ||
            operation.reverse_edge_id == operation.edge_id || idExists(edited.graph, operation.reverse_edge_id))
          {
            throw std::runtime_error("ADD_EDGE bidirectional reverse ID is invalid or duplicate");
          }
          RouteEdge reverse = forward;
          reverse.id = operation.reverse_edge_id;
          reverse.from = forward.to;
          reverse.to = forward.from;
          forward.reverse_of = reverse.id;
          reverse.reverse_of = forward.id;
          std::reverse(reverse.centerline.begin(), reverse.centerline.end());
          edited.graph.edges.push_back(forward);
          edited.graph.edges.push_back(reverse);
          edited.edge_metadata[forward.id] = forward_metadata;
          forward_metadata.reverse_direction = true;
          edited.edge_metadata[reverse.id] = forward_metadata;
        } else {
          edited.graph.edges.push_back(forward);
          edited.edge_metadata[forward.id] = forward_metadata;
        }
        break;
      }
    case RouteEditOperationType::kDeleteEdge: {
        const auto edge = findById(edited.graph.edges, operation.edge_id);
        if (edge == edited.graph.edges.end()) {
          throw std::runtime_error("DELETE_EDGE references a missing edge");
        }
        const std::optional<std::uint64_t> reverse = edge->reverse_of;
        eraseEdge(edited, operation.edge_id);
        if (reverse && operation.include_reverse) {
          eraseEdge(edited, *reverse);
        } else if (reverse) {
          auto partner = findById(edited.graph.edges, *reverse);
          if (partner != edited.graph.edges.end()) {
            partner->reverse_of.reset();
            RouteEntityMetadata & metadata = edited.edge_metadata[partner->id];
            metadata.provenance = derivedProvenance(metadata);
            metadata.reverse_direction = false;
          }
        }
        break;
      }
    case RouteEditOperationType::kSplitEdge: {
        auto selected = findById(edited.graph.edges, operation.edge_id);
        if (selected == edited.graph.edges.end()) {
          throw std::runtime_error("SPLIT_EDGE references a missing edge");
        }
        const RouteEdge source = *selected;
        const RouteEntityMetadata source_metadata = edited.edge_metadata[source.id];
        const double length = polylineLength(source.centerline);
        if (!(operation.split_s > 1.0e-9) || !(operation.split_s < length - 1.0e-9)) {
          throw std::runtime_error("SPLIT_EDGE split_s must be inside the edge");
        }
        const std::vector<Vec3> first_geometry = slicePolyline(
          source.centerline, 0.0, operation.split_s);
        const std::vector<Vec3> second_geometry = slicePolyline(
          source.centerline, operation.split_s, length);
        const Vec3 split_position = first_geometry.back();
        const std::vector<std::uint64_t> requested_ids{
          operation.node_id, operation.first_edge_id, operation.second_edge_id,
          operation.first_reverse_edge_id, operation.second_reverse_edge_id};
        std::unordered_set<std::uint64_t> unique;
        for (const std::uint64_t id : requested_ids) {
          if (id == 0U) {
            continue;
          }
          if (!unique.insert(id).second || idExists(edited.graph, id)) {
            throw std::runtime_error("SPLIT_EDGE allocated IDs are zero, duplicate, or already used");
          }
        }
        if (operation.node_id == 0U || operation.first_edge_id == 0U ||
          operation.second_edge_id == 0U)
        {
          throw std::runtime_error("SPLIT_EDGE is missing allocated result IDs");
        }
        const std::optional<std::uint64_t> old_reverse = source.reverse_of;
        if (old_reverse) {
          if (operation.first_reverse_edge_id == 0U || operation.second_reverse_edge_id == 0U) {
            throw std::runtime_error("SPLIT_EDGE bidirectional source requires reverse result IDs");
          }
        } else if (operation.first_reverse_edge_id != 0U || operation.second_reverse_edge_id != 0U) {
          throw std::runtime_error("SPLIT_EDGE one-way source must not allocate reverse result IDs");
        }
        const std::vector<std::uint64_t> source_ids = combinedSourceIds(source_metadata, source);
        eraseEdge(edited, source.id);
        if (old_reverse) {
          eraseEdge(edited, *old_reverse);
        }
        edited.graph.nodes.push_back({operation.node_id, split_position, RouteNodeType::kNormal});
        edited.node_metadata[operation.node_id] = {
          derivedProvenance(source_metadata), RouteValidationStatus::kUnvalidated,
          source_ids, {}, false};
        RouteEdge first = makeEdge(
          operation.first_edge_id, source.from, operation.node_id, first_geometry);
        RouteEdge second = makeEdge(
          operation.second_edge_id, operation.node_id, source.to, second_geometry);
        RouteEntityMetadata metadata{
          derivedProvenance(source_metadata), RouteValidationStatus::kUnvalidated,
          source_ids, {}, false,
          source_metadata.requires_orientation_collision_validation};
        if (old_reverse) {
          RouteEdge first_reverse = first;
          RouteEdge second_reverse = second;
          first_reverse.id = operation.first_reverse_edge_id;
          first_reverse.from = first.to;
          first_reverse.to = first.from;
          second_reverse.id = operation.second_reverse_edge_id;
          second_reverse.from = second.to;
          second_reverse.to = second.from;
          first.reverse_of = first_reverse.id;
          first_reverse.reverse_of = first.id;
          second.reverse_of = second_reverse.id;
          second_reverse.reverse_of = second.id;
          std::reverse(first_reverse.centerline.begin(), first_reverse.centerline.end());
          std::reverse(second_reverse.centerline.begin(), second_reverse.centerline.end());
          edited.graph.edges.push_back(first);
          edited.graph.edges.push_back(second);
          edited.graph.edges.push_back(first_reverse);
          edited.graph.edges.push_back(second_reverse);
          edited.edge_metadata[first.id] = metadata;
          edited.edge_metadata[second.id] = metadata;
          metadata.reverse_direction = true;
          edited.edge_metadata[first_reverse.id] = metadata;
          edited.edge_metadata[second_reverse.id] = metadata;
        } else {
          edited.graph.edges.push_back(first);
          edited.graph.edges.push_back(second);
          edited.edge_metadata[first.id] = metadata;
          edited.edge_metadata[second.id] = metadata;
        }
        break;
      }
    case RouteEditOperationType::kSetDirection: {
        auto selected = findById(edited.graph.edges, operation.edge_id);
        if (selected == edited.graph.edges.end()) {
          throw std::runtime_error("SET_DIRECTION references a missing edge");
        }
        if (operation.direction == RouteDirection::kOneWay) {
          const std::optional<std::uint64_t> reverse = selected->reverse_of;
          selected->reverse_of.reset();
          RouteEntityMetadata & metadata = edited.edge_metadata[selected->id];
          metadata.provenance = derivedProvenance(metadata);
          metadata.reverse_direction = false;
          if (reverse) {
            eraseEdge(edited, *reverse);
          }
        } else if (!selected->reverse_of) {
          if (operation.reverse_edge_id == 0U || idExists(edited.graph, operation.reverse_edge_id)) {
            throw std::runtime_error("SET_DIRECTION reverse ID is invalid or duplicate");
          }
          const std::uint64_t selected_id = selected->id;
          RouteEdge reverse = *selected;
          reverse.id = operation.reverse_edge_id;
          reverse.from = selected->to;
          reverse.to = selected->from;
          selected->reverse_of = reverse.id;
          reverse.reverse_of = selected_id;
          std::reverse(reverse.centerline.begin(), reverse.centerline.end());
          RouteEntityMetadata metadata = edited.edge_metadata[selected_id];
          metadata.provenance = derivedProvenance(metadata);
          metadata.reverse_direction = false;
          edited.edge_metadata[selected_id] = metadata;
          metadata.reverse_direction = true;
          edited.graph.edges.push_back(reverse);
          edited.edge_metadata[reverse.id] = metadata;
        }
        break;
      }
  }
  updateNodeTypes(edited.graph);
}

std::uint64_t maximumGraphId(const RouteGraph & graph)
{
  std::uint64_t result = 0U;
  for (const RouteNode & node : graph.nodes) {
    result = std::max(result, node.id);
  }
  for (const RouteEdge & edge : graph.edges) {
    result = std::max(result, edge.id);
  }
  return result;
}

std::uint64_t maximumOperationEntityId(const RouteEditOverlay & overlay)
{
  std::uint64_t result = 0U;
  for (const RouteEditOperation & operation : overlay.operations) {
    result = std::max({
      result, operation.node_id, operation.edge_id, operation.reverse_edge_id,
      operation.first_edge_id, operation.second_edge_id,
      operation.first_reverse_edge_id, operation.second_reverse_edge_id});
  }
  return result;
}

std::string sanitizeField(std::string value)
{
  for (char & character : value) {
    if (character == '\t' || character == '\n' || character == '\r') {
      character = ' ';
    }
  }
  return value;
}

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> result;
  std::size_t begin = 0U;
  while (begin <= line.size()) {
    const std::size_t end = line.find('\t', begin);
    result.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return result;
}

std::string encodePolyline(const std::vector<Vec3> & points)
{
  std::ostringstream stream;
  stream << std::setprecision(17);
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (index > 0U) {
      stream << ';';
    }
    stream << points[index].x << ',' << points[index].y << ',' << points[index].z;
  }
  return stream.str();
}

std::vector<Vec3> decodePolyline(const std::string & encoded)
{
  std::vector<Vec3> result;
  std::size_t begin = 0U;
  while (begin < encoded.size()) {
    const std::size_t end = encoded.find(';', begin);
    const std::string point = encoded.substr(
      begin, end == std::string::npos ? end : end - begin);
    const std::size_t first_comma = point.find(',');
    const std::size_t second_comma = point.find(',', first_comma == std::string::npos ? 0U : first_comma + 1U);
    if (first_comma == std::string::npos || second_comma == std::string::npos) {
      throw std::runtime_error("invalid encoded route polyline point");
    }
    result.push_back({
      std::stod(point.substr(0U, first_comma)),
      std::stod(point.substr(first_comma + 1U, second_comma - first_comma - 1U)),
      std::stod(point.substr(second_comma + 1U))});
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return result;
}

RouteEditOperationType operationTypeFromString(const std::string & value)
{
  if (value == "clear_graph") {return RouteEditOperationType::kClearGraph;}
  if (value == "add_node") {return RouteEditOperationType::kAddNode;}
  if (value == "move_node") {return RouteEditOperationType::kMoveNode;}
  if (value == "delete_node") {return RouteEditOperationType::kDeleteNode;}
  if (value == "add_edge") {return RouteEditOperationType::kAddEdge;}
  if (value == "delete_edge") {return RouteEditOperationType::kDeleteEdge;}
  if (value == "split_edge") {return RouteEditOperationType::kSplitEdge;}
  if (value == "set_direction") {return RouteEditOperationType::kSetDirection;}
  throw std::runtime_error("unknown route edit operation type: " + value);
}

RouteDirection directionFromString(const std::string & value)
{
  if (value == "one_way") {return RouteDirection::kOneWay;}
  if (value == "bidirectional") {return RouteDirection::kBidirectional;}
  throw std::runtime_error("unknown route direction: " + value);
}

std::string operationRecord(const RouteEditOperation & operation)
{
  std::ostringstream stream;
  stream << std::setprecision(17)
         << "OP\t" << operation.operation_id << '\t' << toString(operation.type) << '\t'
         << operation.node_id << '\t' << operation.edge_id << '\t'
         << operation.reverse_edge_id << '\t' << operation.from_id << '\t' << operation.to_id << '\t'
         << operation.first_edge_id << '\t' << operation.second_edge_id << '\t'
         << operation.first_reverse_edge_id << '\t' << operation.second_reverse_edge_id << '\t'
         << operation.position.x << '\t' << operation.position.y << '\t' << operation.position.z << '\t'
         << operation.split_s << '\t' << toString(operation.direction) << '\t'
         << (operation.include_reverse ? 1 : 0) << '\t' << (operation.cascade ? 1 : 0) << '\t'
         << encodePolyline(operation.polyline);
  return stream.str();
}

RouteEditOperation parseOperationRecord(const std::string & record)
{
  const std::vector<std::string> fields = splitTabs(record);
  if (fields.size() != 20U || fields[0] != "OP") {
    throw std::runtime_error("invalid route edit OP record");
  }
  RouteEditOperation operation;
  operation.operation_id = std::stoull(fields[1]);
  operation.type = operationTypeFromString(fields[2]);
  operation.node_id = std::stoull(fields[3]);
  operation.edge_id = std::stoull(fields[4]);
  operation.reverse_edge_id = std::stoull(fields[5]);
  operation.from_id = std::stoull(fields[6]);
  operation.to_id = std::stoull(fields[7]);
  operation.first_edge_id = std::stoull(fields[8]);
  operation.second_edge_id = std::stoull(fields[9]);
  operation.first_reverse_edge_id = std::stoull(fields[10]);
  operation.second_reverse_edge_id = std::stoull(fields[11]);
  operation.position = {std::stod(fields[12]), std::stod(fields[13]), std::stod(fields[14])};
  operation.split_s = std::stod(fields[15]);
  operation.direction = directionFromString(fields[16]);
  operation.include_reverse = fields[17] == "1";
  operation.cascade = fields[18] == "1";
  operation.polyline = decodePolyline(fields[19]);
  return operation;
}

std::string jsonEscape(const std::string & input)
{
  std::string result;
  for (const char character : input) {
    switch (character) {
      case '\\': result += "\\\\"; break;
      case '"': result += "\\\""; break;
      case '\t': result += "\\t"; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      default: result += character; break;
    }
  }
  return result;
}

std::string jsonUnescape(const std::string & input)
{
  std::string result;
  bool escaped = false;
  for (const char character : input) {
    if (!escaped) {
      if (character == '\\') {
        escaped = true;
      } else {
        result.push_back(character);
      }
      continue;
    }
    switch (character) {
      case 't': result.push_back('\t'); break;
      case 'n': result.push_back('\n'); break;
      case 'r': result.push_back('\r'); break;
      default: result.push_back(character); break;
    }
    escaped = false;
  }
  if (escaped) {
    throw std::runtime_error("unterminated JSON escape");
  }
  return result;
}

std::string extractJsonString(
  const std::string & input, const std::string & key, const std::size_t start = 0U,
  std::size_t * end_position = nullptr)
{
  const std::string pattern = "\"" + key + "\":\"";
  const std::size_t value_begin = input.find(pattern, start);
  if (value_begin == std::string::npos) {
    throw std::runtime_error("GeoJSON key is missing: " + key);
  }
  std::size_t index = value_begin + pattern.size();
  std::string encoded;
  bool escaped = false;
  for (; index < input.size(); ++index) {
    const char character = input[index];
    if (!escaped && character == '"') {
      if (end_position != nullptr) {
        *end_position = index + 1U;
      }
      return jsonUnescape(encoded);
    }
    encoded.push_back(character);
    if (character == '\\' && !escaped) {
      escaped = true;
    } else {
      escaped = false;
    }
  }
  throw std::runtime_error("unterminated GeoJSON string: " + key);
}

std::uint64_t extractJsonUnsigned(const std::string & input, const std::string & key)
{
  const std::string pattern = "\"" + key + "\":";
  const std::size_t begin = input.find(pattern);
  if (begin == std::string::npos) {
    throw std::runtime_error("GeoJSON numeric key is missing: " + key);
  }
  return std::stoull(input.substr(begin + pattern.size()));
}

std::string readFile(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open route edit file: " + path.string());
  }
  std::ostringstream result;
  result << stream.rdbuf();
  return result.str();
}

void writeJsonCoordinate(std::ostream & stream, const Vec3 & point)
{
  stream << '[' << point.x << ',' << point.y << ',' << point.z << ']';
}

void writeJsonStringArray(std::ostream & stream, const std::vector<std::string> & values)
{
  stream << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {stream << ',';}
    stream << '"' << jsonEscape(values[index]) << '"';
  }
  stream << ']';
}

void writeJsonIdArray(std::ostream & stream, const std::vector<std::uint64_t> & values)
{
  stream << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {stream << ',';}
    stream << values[index];
  }
  stream << ']';
}

}  // namespace

const char * toString(const RouteEditOperationType type)
{
  switch (type) {
    case RouteEditOperationType::kClearGraph: return "clear_graph";
    case RouteEditOperationType::kAddNode: return "add_node";
    case RouteEditOperationType::kMoveNode: return "move_node";
    case RouteEditOperationType::kDeleteNode: return "delete_node";
    case RouteEditOperationType::kAddEdge: return "add_edge";
    case RouteEditOperationType::kDeleteEdge: return "delete_edge";
    case RouteEditOperationType::kSplitEdge: return "split_edge";
    case RouteEditOperationType::kSetDirection: return "set_direction";
  }
  return "unknown";
}

const char * toString(const RouteDirection direction)
{
  return direction == RouteDirection::kBidirectional ? "bidirectional" : "one_way";
}

const char * toString(const RouteProvenance provenance)
{
  switch (provenance) {
    case RouteProvenance::kGenerated: return "generated";
    case RouteProvenance::kManual: return "manual";
    case RouteProvenance::kEditedGenerated: return "edited_generated";
  }
  return "unknown";
}

const char * toString(const RouteValidationStatus status)
{
  switch (status) {
    case RouteValidationStatus::kUnvalidated: return "unvalidated";
    case RouteValidationStatus::kValid: return "valid";
    case RouteValidationStatus::kWarning: return "warning";
    case RouteValidationStatus::kInvalid: return "invalid";
  }
  return "unknown";
}

std::string routeGraphFingerprint(const RouteGraph & graph)
{
  // FNV-1a over the fields that identify the generated topology and geometry.
  // Numeric values are hashed by IEEE bytes to avoid locale-dependent text.
  std::uint64_t hash = 1469598103934665603ULL;
  auto append = [&](const void * data, const std::size_t size) {
      const auto * bytes = static_cast<const unsigned char *>(data);
      for (std::size_t index = 0U; index < size; ++index) {
        hash ^= static_cast<std::uint64_t>(bytes[index]);
        hash *= 1099511628211ULL;
      }
    };
  append(graph.frame_id.data(), graph.frame_id.size());
  for (const RouteNode & node : graph.nodes) {
    append(&node.id, sizeof(node.id));
    append(&node.position.x, sizeof(double));
    append(&node.position.y, sizeof(double));
    append(&node.position.z, sizeof(double));
  }
  for (const RouteEdge & edge : graph.edges) {
    append(&edge.id, sizeof(edge.id));
    append(&edge.from, sizeof(edge.from));
    append(&edge.to, sizeof(edge.to));
    const std::uint64_t reverse = edge.reverse_of.value_or(0U);
    append(&reverse, sizeof(reverse));
    for (const Vec3 & point : edge.centerline) {
      append(&point.x, sizeof(double));
      append(&point.y, sizeof(double));
      append(&point.z, sizeof(double));
    }
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << hash;
  return stream.str();
}

EditedRouteGraph applyRouteEdits(
  const RouteGraph & generated_graph, const RouteEditOverlay & overlay)
{
  validateStructure(generated_graph);
  if (overlay.version != kRouteEditOverlayVersion) {
    throw std::runtime_error("unsupported route edit overlay version");
  }
  if (overlay.frame_id != generated_graph.frame_id) {
    throw std::runtime_error("route edit overlay frame does not match generated graph");
  }
  if (overlay.base_graph_fingerprint != routeGraphFingerprint(generated_graph)) {
    throw std::runtime_error("route edit overlay was created for a different generated graph");
  }

  EditedRouteGraph edited;
  edited.graph = generated_graph;
  for (const RouteNode & node : generated_graph.nodes) {
    edited.node_metadata[node.id] = {
      RouteProvenance::kGenerated, RouteValidationStatus::kUnvalidated, {node.id}, {}, false};
  }
  for (const RouteEdge & edge : generated_graph.edges) {
    edited.edge_metadata[edge.id] = {
      RouteProvenance::kGenerated, RouteValidationStatus::kUnvalidated, {edge.id}, {},
      edge.reverse_of && edge.id > *edge.reverse_of};
  }

  std::unordered_set<std::uint64_t> operation_ids;
  for (const RouteEditOperation & operation : overlay.operations) {
    if (operation.operation_id == 0U || !operation_ids.insert(operation.operation_id).second) {
      throw std::runtime_error("route edit operation IDs must be nonzero and unique");
    }
    applyOperation(edited, operation);
    validateStructure(edited.graph);
  }
  return edited;
}

RouteValidationResult validateEditedRouteGraph(
  const EditedRouteGraph & edited_graph,
  const OccupancyGrid2D & inflated_obstacle_grid,
  const OccupancyGrid2D & unknown_grid,
  const GeneratorConfig & config,
  const RouteValidationOptions & options,
  const OccupancyGrid2D * raw_obstacle_grid)
{
  validateStructure(edited_graph.graph);
  requireMatchingGridGeometry(inflated_obstacle_grid, unknown_grid);
  const bool direct_obstacle_footprint =
    options.use_orientation_aware_obstacle_footprint &&
    config.robot.footprint_model == "rectangle";
  if (direct_obstacle_footprint) {
    if (raw_obstacle_grid == nullptr) {
      throw std::invalid_argument(
              "raw obstacle grid is required for orientation-aware Route collision validation");
    }
    if (!matchingGridGeometry(inflated_obstacle_grid, *raw_obstacle_grid)) {
      throw std::invalid_argument(
              "raw and inflated obstacle grids must have identical geometry");
    }
  }
  RouteValidationResult result;
  result.edited = edited_graph;
  for (RouteEdge & edge : result.edited.graph.edges) {
    resetDerivedGeometry(edge);
    RouteEntityMetadata & metadata = result.edited.edge_metadata[edge.id];
    metadata.validation_status = RouteValidationStatus::kUnvalidated;
    metadata.validation_errors.clear();
  }

  result.direct_route_footprint_obstacle_validation = direct_obstacle_footprint;
  result.obstacle_footprint_policy = direct_obstacle_footprint ?
    "route_tangent_rectangle_raw_obstacle_grid" :
    "trajectory_yaw_configuration_space_grid";
  const bool direct_footprint_unknown =
    options.use_orientation_aware_unknown_footprint &&
    (config.robot.footprint_model == "rectangle" ||
    config.robot.footprint_model == "circle");
  result.direct_route_footprint_unknown_validation = direct_footprint_unknown;
  result.unknown_footprint_includes_clearance =
    options.include_clearance_in_unknown_footprint;
  result.unknown_footprint_policy = direct_footprint_unknown ?
    (config.robot.footprint_model == "rectangle" ?
    "route_tangent_rectangle" : "route_sample_circle") :
    "configuration_space_radius";
  std::optional<OccupancyGrid2D> configuration_unknown;
  if (!direct_footprint_unknown) {
    configuration_unknown.emplace(configurationSpaceUnknownGrid(unknown_grid, config.robot));
  }
  computeRouteClearance(
    result.edited.graph, inflated_obstacle_grid,
    direct_footprint_unknown ? unknown_grid : *configuration_unknown,
    config.traversability, config.lanelet2.speed_limit_mps);

  for (RouteEdge & edge : result.edited.graph.edges) {
    RouteEntityMetadata & metadata = result.edited.edge_metadata[edge.id];
    metadata.validation_errors = edge.validation_errors;
    if (edge.length + 1.0e-9 < config.topology.minimum_edge_length) {
      addEdgeError(edge, metadata, "edge_below_minimum_length");
    }
    if (edge.length > config.topology.maximum_edge_length + 1.0e-9) {
      addEdgeError(edge, metadata, "edge_above_maximum_length");
    }
    if (options.require_verified_vehicle_dimensions && !config.robot.dimensions_verified) {
      addEdgeError(edge, metadata, "vehicle_dimensions_unverified");
    }
    if (direct_footprint_unknown && config.robot.footprint_model == "rectangle" &&
      rectangleRouteFootprintTouchesUnknown(
        edge, unknown_grid, config.robot,
        options.include_clearance_in_unknown_footprint))
    {
      addEdgeError(edge, metadata, "vehicle_footprint_overlaps_unknown");
    }
    if (direct_footprint_unknown && config.robot.footprint_model == "circle" &&
      circleRouteFootprintTouchesUnknown(
        edge, unknown_grid, config.robot,
        options.include_clearance_in_unknown_footprint))
    {
      addEdgeError(edge, metadata, "vehicle_footprint_overlaps_unknown");
    }
    if (direct_obstacle_footprint && metadata.requires_orientation_collision_validation &&
      rectangleRouteFootprintTouchesObstacle(edge, *raw_obstacle_grid, config.robot))
    {
      addEdgeError(edge, metadata, "vehicle_footprint_overlaps_obstacle");
    }
    // `allow` is useful for visual diagnostics, but an absence of returns is
    // not observed free space.  Never promote such a graph to the canonical
    // planner-facing output.
    if (config.traversability.unknown_space_policy != "occupied") {
      addEdgeError(edge, metadata, "unknown_space_policy_not_operational");
    }
    if (config.robot.minimum_turning_radius > 0.0) {
      const double maximum_allowed_curvature = 1.0 / config.robot.minimum_turning_radius;
      if (edge.maximum_curvature > maximum_allowed_curvature + 1.0e-9) {
        addEdgeError(edge, metadata, "minimum_turning_radius_violation");
      }
    }
    if (!config.robot.allow_in_place_rotation && hasDirectionReversal(edge.centerline)) {
      addEdgeError(edge, metadata, "in_place_rotation_not_allowed");
    }
    if (!config.robot.allow_reverse_motion && metadata.reverse_direction) {
      addEdgeError(edge, metadata, "reverse_motion_not_allowed");
    }
    if (!direct_obstacle_footprint && config.robot.footprint_model == "rectangle" &&
      metadata.requires_orientation_collision_validation)
    {
      // The supplied inflated grid uses the measured trajectory yaw. It cannot
      // prove collision freedom for a newly oriented manual/moved route.
      addEdgeError(edge, metadata, "route_orientation_collision_unvalidated");
    }
    const bool has_hard_error = std::any_of(
      metadata.validation_errors.begin(), metadata.validation_errors.end(),
      [](const std::string & error) {return !isAdvisoryValidationError(error);});
    if (!edge.passable || has_hard_error) {
      metadata.validation_status = RouteValidationStatus::kInvalid;
    } else if (!metadata.validation_errors.empty()) {
      metadata.validation_status = RouteValidationStatus::kWarning;
    } else {
      metadata.validation_status = RouteValidationStatus::kValid;
    }
    if (metadata.validation_status == RouteValidationStatus::kInvalid) {
      for (const std::string & error : metadata.validation_errors) {
        addUniqueError(
          result.validation_errors,
          "edge_" + std::to_string(edge.id) + ":" + error);
      }
    }
  }

  for (RouteNode & node : result.edited.graph.nodes) {
    RouteEntityMetadata & metadata = result.edited.node_metadata[node.id];
    metadata.validation_errors.clear();
    std::size_t operational_incident = 0U;
    std::size_t warning_incident = 0U;
    std::size_t invalid_incident = 0U;
    for (const RouteEdge & edge : result.edited.graph.edges) {
      if (edge.from != node.id && edge.to != node.id) {
        continue;
      }
      const auto edge_metadata = result.edited.edge_metadata.find(edge.id);
      if (edge_metadata != result.edited.edge_metadata.end() &&
        isOperationalStatus(edge_metadata->second.validation_status))
      {
        ++operational_incident;
        if (edge_metadata->second.validation_status == RouteValidationStatus::kWarning) {
          ++warning_incident;
        }
      } else {
        ++invalid_incident;
      }
    }
    if (operational_incident == 0U && invalid_incident == 0U) {
      metadata.validation_status = RouteValidationStatus::kInvalid;
      metadata.validation_errors.push_back("isolated_node");
    } else if (operational_incident == 0U) {
      metadata.validation_status = RouteValidationStatus::kInvalid;
      metadata.validation_errors.push_back("no_valid_incident_edge");
    } else if (invalid_incident > 0U || warning_incident > 0U) {
      metadata.validation_status = RouteValidationStatus::kWarning;
      if (invalid_incident > 0U) {
        metadata.validation_errors.push_back("has_invalid_incident_edge");
      }
      if (warning_incident > 0U) {
        metadata.validation_errors.push_back("has_warning_incident_edge");
      }
    } else {
      metadata.validation_status = RouteValidationStatus::kValid;
    }
  }

  result.operational_graph.frame_id = result.edited.graph.frame_id;
  std::set<std::uint64_t> referenced_nodes;
  std::set<std::uint64_t> valid_edge_ids;
  for (const RouteEdge & edge : result.edited.graph.edges) {
    const auto metadata = result.edited.edge_metadata.find(edge.id);
    if (metadata == result.edited.edge_metadata.end() ||
      !isOperationalStatus(metadata->second.validation_status))
    {
      continue;
    }
    result.operational_graph.edges.push_back(edge);
    referenced_nodes.insert(edge.from);
    referenced_nodes.insert(edge.to);
    valid_edge_ids.insert(edge.id);
  }
  for (RouteEdge & edge : result.operational_graph.edges) {
    if (edge.reverse_of && valid_edge_ids.find(*edge.reverse_of) == valid_edge_ids.end()) {
      edge.reverse_of.reset();
    }
  }
  for (const RouteNode & node : result.edited.graph.nodes) {
    if (referenced_nodes.find(node.id) != referenced_nodes.end()) {
      result.operational_graph.nodes.push_back(node);
    }
  }
  updateNodeTypes(result.operational_graph);
  result.operational_ready = !result.operational_graph.edges.empty();
  if (!result.operational_ready) {
    addUniqueError(result.validation_errors, "no_valid_operational_edge");
  }
  return result;
}

RouteEditSession::RouteEditSession(const RouteGraph & generated_graph)
: generated_graph_(generated_graph)
{
  validateStructure(generated_graph_);
  overlay_.frame_id = generated_graph_.frame_id;
  overlay_.base_graph_fingerprint = routeGraphFingerprint(generated_graph_);
  overlay_.next_entity_id = maximumGraphId(generated_graph_) + 1U;
}

RouteEditSession::RouteEditSession(const RouteGraph & generated_graph, RouteEditOverlay overlay)
: generated_graph_(generated_graph), overlay_(std::move(overlay))
{
  static_cast<void>(applyRouteEdits(generated_graph_, overlay_));
  std::uint64_t maximum_operation_id = 0U;
  for (const RouteEditOperation & operation : overlay_.operations) {
    maximum_operation_id = std::max(maximum_operation_id, operation.operation_id);
  }
  overlay_.next_operation_id = std::max(overlay_.next_operation_id, maximum_operation_id + 1U);
  overlay_.next_entity_id = std::max(
    overlay_.next_entity_id,
    std::max(maximumGraphId(generated_graph_), maximumOperationEntityId(overlay_)) + 1U);
}

EditedRouteGraph RouteEditSession::editedGraph() const
{
  return applyRouteEdits(generated_graph_, overlay_);
}

void RouteEditSession::clearGraph()
{
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kClearGraph;
  appendAndVerify(std::move(operation));
}

std::uint64_t RouteEditSession::allocateEntityId()
{
  if (overlay_.next_entity_id == 0U ||
    overlay_.next_entity_id == std::numeric_limits<std::uint64_t>::max())
  {
    throw std::overflow_error("route entity ID space is exhausted");
  }
  return overlay_.next_entity_id++;
}

std::uint64_t RouteEditSession::allocateOperationId()
{
  if (overlay_.next_operation_id == 0U ||
    overlay_.next_operation_id == std::numeric_limits<std::uint64_t>::max())
  {
    throw std::overflow_error("route edit operation ID space is exhausted");
  }
  return overlay_.next_operation_id++;
}

void RouteEditSession::appendAndVerify(RouteEditOperation operation)
{
  overlay_.operations.push_back(std::move(operation));
  try {
    static_cast<void>(applyRouteEdits(generated_graph_, overlay_));
  } catch (...) {
    overlay_.operations.pop_back();
    throw;
  }
}

std::uint64_t RouteEditSession::addNode(const Vec3 & position)
{
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kAddNode;
  operation.node_id = allocateEntityId();
  operation.position = position;
  const std::uint64_t id = operation.node_id;
  appendAndVerify(std::move(operation));
  return id;
}

void RouteEditSession::moveNode(const std::uint64_t node_id, const Vec3 & position)
{
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kMoveNode;
  operation.node_id = node_id;
  operation.position = position;
  appendAndVerify(std::move(operation));
}

void RouteEditSession::deleteNode(const std::uint64_t node_id, const bool cascade)
{
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kDeleteNode;
  operation.node_id = node_id;
  operation.cascade = cascade;
  appendAndVerify(std::move(operation));
}

std::pair<std::uint64_t, std::optional<std::uint64_t>> RouteEditSession::addEdge(
  const std::uint64_t from_id,
  const std::uint64_t to_id,
  std::vector<Vec3> polyline,
  const RouteDirection direction)
{
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kAddEdge;
  operation.edge_id = allocateEntityId();
  operation.from_id = from_id;
  operation.to_id = to_id;
  operation.direction = direction;
  operation.polyline = std::move(polyline);
  if (direction == RouteDirection::kBidirectional) {
    operation.reverse_edge_id = allocateEntityId();
  }
  const std::uint64_t forward = operation.edge_id;
  const std::optional<std::uint64_t> reverse = operation.reverse_edge_id == 0U ?
    std::nullopt : std::make_optional(operation.reverse_edge_id);
  appendAndVerify(std::move(operation));
  return {forward, reverse};
}

void RouteEditSession::deleteEdge(
  const std::uint64_t edge_id, const bool include_reverse)
{
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kDeleteEdge;
  operation.edge_id = edge_id;
  operation.include_reverse = include_reverse;
  appendAndVerify(std::move(operation));
}

SplitRouteEdgeResult RouteEditSession::splitEdge(
  const std::uint64_t edge_id, const double split_s)
{
  const EditedRouteGraph current = editedGraph();
  const auto edge = findById(current.graph.edges, edge_id);
  if (edge == current.graph.edges.end()) {
    throw std::runtime_error("cannot split a missing route edge");
  }
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kSplitEdge;
  operation.edge_id = edge_id;
  operation.split_s = split_s;
  operation.node_id = allocateEntityId();
  operation.first_edge_id = allocateEntityId();
  operation.second_edge_id = allocateEntityId();
  if (edge->reverse_of) {
    operation.first_reverse_edge_id = allocateEntityId();
    operation.second_reverse_edge_id = allocateEntityId();
  }
  SplitRouteEdgeResult result;
  result.node_id = operation.node_id;
  result.first_edge_id = operation.first_edge_id;
  result.second_edge_id = operation.second_edge_id;
  if (operation.first_reverse_edge_id != 0U) {
    result.first_reverse_edge_id = operation.first_reverse_edge_id;
    result.second_reverse_edge_id = operation.second_reverse_edge_id;
  }
  appendAndVerify(std::move(operation));
  return result;
}

std::optional<std::uint64_t> RouteEditSession::setEdgeDirection(
  const std::uint64_t edge_id, const RouteDirection direction)
{
  const EditedRouteGraph current = editedGraph();
  const auto edge = findById(current.graph.edges, edge_id);
  if (edge == current.graph.edges.end()) {
    throw std::runtime_error("cannot change direction of a missing route edge");
  }
  RouteEditOperation operation;
  operation.operation_id = allocateOperationId();
  operation.type = RouteEditOperationType::kSetDirection;
  operation.edge_id = edge_id;
  operation.direction = direction;
  std::optional<std::uint64_t> created_reverse;
  if (direction == RouteDirection::kBidirectional && !edge->reverse_of) {
    operation.reverse_edge_id = allocateEntityId();
    created_reverse = operation.reverse_edge_id;
  }
  appendAndVerify(std::move(operation));
  return created_reverse;
}

void saveRouteEditOverlayTsv(
  const std::filesystem::path & path, const RouteEditOverlay & overlay)
{
  if (overlay.version != kRouteEditOverlayVersion) {
    throw std::runtime_error("cannot save unsupported route edit overlay version");
  }
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create route edit TSV: " + path.string());
  }
  stream << "VERSION\t" << overlay.version << '\n'
         << "FRAME\t" << sanitizeField(overlay.frame_id) << '\n'
         << "BASE\t" << sanitizeField(overlay.base_graph_fingerprint) << '\n'
         << "NEXT_OPERATION_ID\t" << overlay.next_operation_id << '\n'
         << "NEXT_ENTITY_ID\t" << overlay.next_entity_id << '\n';
  for (const RouteEditOperation & operation : overlay.operations) {
    stream << operationRecord(operation) << '\n';
  }
}

RouteEditOverlay loadRouteEditOverlayTsv(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open route edit TSV: " + path.string());
  }
  RouteEditOverlay overlay;
  overlay.version = 0U;
  bool frame_seen = false;
  bool base_seen = false;
  bool next_operation_seen = false;
  bool next_entity_seen = false;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(stream, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line.front() == '#') {
      continue;
    }
    try {
      const std::vector<std::string> fields = splitTabs(line);
      if (fields[0] == "VERSION") {
        if (fields.size() != 2U) {throw std::runtime_error("invalid VERSION record");}
        overlay.version = static_cast<std::uint32_t>(std::stoul(fields[1]));
      } else if (fields[0] == "FRAME") {
        if (fields.size() != 2U) {throw std::runtime_error("invalid FRAME record");}
        overlay.frame_id = fields[1];
        frame_seen = true;
      } else if (fields[0] == "BASE") {
        if (fields.size() != 2U) {throw std::runtime_error("invalid BASE record");}
        overlay.base_graph_fingerprint = fields[1];
        base_seen = true;
      } else if (fields[0] == "NEXT_OPERATION_ID") {
        if (fields.size() != 2U) {throw std::runtime_error("invalid NEXT_OPERATION_ID record");}
        overlay.next_operation_id = std::stoull(fields[1]);
        next_operation_seen = true;
      } else if (fields[0] == "NEXT_ENTITY_ID") {
        if (fields.size() != 2U) {throw std::runtime_error("invalid NEXT_ENTITY_ID record");}
        overlay.next_entity_id = std::stoull(fields[1]);
        next_entity_seen = true;
      } else if (fields[0] == "OP") {
        overlay.operations.push_back(parseOperationRecord(line));
      } else {
        throw std::runtime_error("unknown route edit record: " + fields[0]);
      }
    } catch (const std::exception & exception) {
      throw std::runtime_error(
              "invalid route edit TSV line " + std::to_string(line_number) +
              ": " + exception.what());
    }
  }
  if (overlay.version != kRouteEditOverlayVersion || !frame_seen || !base_seen ||
    !next_operation_seen || !next_entity_seen)
  {
    throw std::runtime_error("route edit TSV header is incomplete or unsupported");
  }
  return overlay;
}

void saveRouteEditOverlayGeoJson(
  const std::filesystem::path & path, const RouteEditOverlay & overlay)
{
  if (overlay.version != kRouteEditOverlayVersion) {
    throw std::runtime_error("cannot save unsupported route edit overlay version");
  }
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create route edit GeoJSON: " + path.string());
  }
  stream << std::setprecision(17)
         << "{\"type\":\"FeatureCollection\","
         << "\"route_edit_overlay_version\":" << overlay.version << ','
         << "\"frame_id\":\"" << jsonEscape(overlay.frame_id) << "\","
         << "\"base_graph_fingerprint\":\"" <<
    jsonEscape(overlay.base_graph_fingerprint) << "\","
         << "\"next_operation_id\":" << overlay.next_operation_id << ','
         << "\"next_entity_id\":" << overlay.next_entity_id << ','
         << "\"features\":[\n";
  for (std::size_t index = 0U; index < overlay.operations.size(); ++index) {
    const RouteEditOperation & operation = overlay.operations[index];
    if (index > 0U) {
      stream << ",\n";
    }
    stream << "{\"type\":\"Feature\",\"properties\":{"
           << "\"operation_id\":" << operation.operation_id << ','
           << "\"operation_type\":\"" << toString(operation.type) << "\","
           << "\"record\":\"" << jsonEscape(operationRecord(operation)) << "\"},"
           << "\"geometry\":";
    if (operation.type == RouteEditOperationType::kAddNode ||
      operation.type == RouteEditOperationType::kMoveNode)
    {
      stream << "{\"type\":\"Point\",\"coordinates\":";
      writeJsonCoordinate(stream, operation.position);
      stream << '}';
    } else if (operation.type == RouteEditOperationType::kAddEdge &&
      operation.polyline.size() >= 2U)
    {
      stream << "{\"type\":\"LineString\",\"coordinates\":[";
      for (std::size_t point = 0U; point < operation.polyline.size(); ++point) {
        if (point > 0U) {stream << ',';}
        writeJsonCoordinate(stream, operation.polyline[point]);
      }
      stream << "]}";
    } else {
      stream << "null";
    }
    stream << '}';
  }
  stream << "\n]}\n";
}

RouteEditOverlay loadRouteEditOverlayGeoJson(const std::filesystem::path & path)
{
  const std::string input = readFile(path);
  RouteEditOverlay overlay;
  overlay.version = static_cast<std::uint32_t>(extractJsonUnsigned(
      input, "route_edit_overlay_version"));
  if (overlay.version != kRouteEditOverlayVersion) {
    throw std::runtime_error("unsupported route edit GeoJSON version");
  }
  overlay.frame_id = extractJsonString(input, "frame_id");
  overlay.base_graph_fingerprint = extractJsonString(input, "base_graph_fingerprint");
  overlay.next_operation_id = extractJsonUnsigned(input, "next_operation_id");
  overlay.next_entity_id = extractJsonUnsigned(input, "next_entity_id");
  std::size_t search = 0U;
  while (true) {
    const std::size_t found = input.find("\"record\":\"", search);
    if (found == std::string::npos) {
      break;
    }
    std::size_t end = 0U;
    const std::string record = extractJsonString(input, "record", found, &end);
    overlay.operations.push_back(parseOperationRecord(record));
    search = end;
  }
  return overlay;
}

void saveEditedRouteGraphGeoJson(
  const std::filesystem::path & path, const EditedRouteGraph & graph)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create edited route GeoJSON: " + path.string());
  }
  stream << std::setprecision(17)
         << "{\"type\":\"FeatureCollection\",\"edited_route_graph_version\":1,"
         << "\"frame_id\":\"" << jsonEscape(graph.graph.frame_id) << "\",\"features\":[\n";
  bool first = true;
  for (const RouteNode & node : graph.graph.nodes) {
    if (!first) {stream << ",\n";}
    first = false;
    const auto found = graph.node_metadata.find(node.id);
    const RouteEntityMetadata metadata = found == graph.node_metadata.end() ?
      RouteEntityMetadata{} : found->second;
    stream << "{\"type\":\"Feature\",\"properties\":{"
           << "\"entity_type\":\"node\",\"id\":" << node.id << ','
           << "\"provenance\":\"" << toString(metadata.provenance) << "\","
           << "\"validation_status\":\"" << toString(metadata.validation_status) << "\","
           << "\"source_ids\":";
    writeJsonIdArray(stream, metadata.source_ids);
    stream << ",\"validation_errors\":";
    writeJsonStringArray(stream, metadata.validation_errors);
    stream << "},\"geometry\":{\"type\":\"Point\",\"coordinates\":";
    writeJsonCoordinate(stream, node.position);
    stream << "}}";
  }
  for (const RouteEdge & edge : graph.graph.edges) {
    if (!first) {stream << ",\n";}
    first = false;
    const auto found = graph.edge_metadata.find(edge.id);
    const RouteEntityMetadata metadata = found == graph.edge_metadata.end() ?
      RouteEntityMetadata{} : found->second;
    stream << "{\"type\":\"Feature\",\"properties\":{"
           << "\"entity_type\":\"edge\",\"id\":" << edge.id << ','
           << "\"from\":" << edge.from << ",\"to\":" << edge.to << ','
           << "\"provenance\":\"" << toString(metadata.provenance) << "\","
           << "\"validation_status\":\"" << toString(metadata.validation_status) << "\","
           << "\"reverse_direction\":" << (metadata.reverse_direction ? "true" : "false") << ','
           << "\"requires_orientation_collision_validation\":" <<
      (metadata.requires_orientation_collision_validation ? "true" : "false") << ','
           << "\"source_ids\":";
    writeJsonIdArray(stream, metadata.source_ids);
    stream << ",\"validation_errors\":";
    writeJsonStringArray(stream, metadata.validation_errors);
    if (edge.reverse_of) {
      stream << ",\"reverse_of\":" << *edge.reverse_of;
    }
    stream << "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";
    for (std::size_t point = 0U; point < edge.centerline.size(); ++point) {
      if (point > 0U) {stream << ',';}
      writeJsonCoordinate(stream, edge.centerline[point]);
    }
    stream << "]}}";
  }
  stream << "\n]}\n";
}

void saveRouteValidationReportYaml(
  const std::filesystem::path & path,
  const RouteValidationResult & result,
  const GeneratorConfig & config)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create route validation report: " + path.string());
  }

  std::size_t valid_edges = 0U;
  std::size_t warning_edges = 0U;
  std::size_t invalid_edges = 0U;
  for (const auto & [id, metadata] : result.edited.edge_metadata) {
    static_cast<void>(id);
    if (metadata.validation_status == RouteValidationStatus::kValid) {
      ++valid_edges;
    } else if (metadata.validation_status == RouteValidationStatus::kWarning) {
      ++warning_edges;
    } else if (metadata.validation_status == RouteValidationStatus::kInvalid) {
      ++invalid_edges;
    }
  }
  std::size_t valid_nodes = 0U;
  std::size_t warning_nodes = 0U;
  std::size_t invalid_nodes = 0U;
  for (const auto & [id, metadata] : result.edited.node_metadata) {
    static_cast<void>(id);
    if (metadata.validation_status == RouteValidationStatus::kValid) {
      ++valid_nodes;
    } else if (metadata.validation_status == RouteValidationStatus::kWarning) {
      ++warning_nodes;
    } else if (metadata.validation_status == RouteValidationStatus::kInvalid) {
      ++invalid_nodes;
    }
  }

  stream << std::setprecision(17)
         << "route_validation_version: 2\n"
         << "navigation_ready: " << (result.operational_ready ? "true" : "false") << '\n'
         << "vehicle_dimensions_verified: " <<
    (config.robot.dimensions_verified ? "true" : "false") << '\n'
         << "minimum_turning_radius: " << config.robot.minimum_turning_radius << '\n'
         << "allow_in_place_rotation: " <<
    (config.robot.allow_in_place_rotation ? "true" : "false") << '\n'
         << "allow_reverse_motion: " <<
    (config.robot.allow_reverse_motion ? "true" : "false") << '\n'
         << "direct_route_footprint_obstacle_validation: " <<
    (result.direct_route_footprint_obstacle_validation ? "true" : "false") << '\n'
         << "obstacle_footprint_policy: \"" <<
    jsonEscape(result.obstacle_footprint_policy) << "\"\n"
         << "unknown_configuration_space_radius: " <<
    unknownConfigurationRadius(config.robot, config.traversability.grid_resolution) << '\n'
         << "unknown_configuration_space_radius_applied: " <<
    (result.direct_route_footprint_unknown_validation ? "false" : "true") << '\n'
         << "unknown_footprint_policy: \"" <<
    jsonEscape(result.unknown_footprint_policy) << "\"\n"
         << "unknown_footprint_includes_clearance: " <<
    (result.unknown_footprint_includes_clearance ? "true" : "false") << '\n'
         << "rectangle_edited_route_policy: \"" <<
    (result.direct_route_footprint_obstacle_validation ?
    "route_tangent_raw_obstacle_grid" :
    "fail_closed_without_route_orientation_collision") << "\"\n"
         << "edited_nodes: " << result.edited.graph.nodes.size() << '\n'
         << "edited_edges: " << result.edited.graph.edges.size() << '\n'
         << "valid_nodes: " << valid_nodes << '\n'
         << "warning_nodes: " << warning_nodes << '\n'
         << "invalid_nodes: " << invalid_nodes << '\n'
         << "valid_edges: " << valid_edges << '\n'
         << "warning_edges: " << warning_edges << '\n'
         << "invalid_edges: " << invalid_edges << '\n'
         << "operational_nodes: " << result.operational_graph.nodes.size() << '\n'
         << "operational_edges: " << result.operational_graph.edges.size() << '\n'
         << "reasons:\n";
  if (result.validation_errors.empty()) {
    stream << "  []\n";
  } else {
    for (const std::string & error : result.validation_errors) {
      stream << "  - \"" << jsonEscape(error) << "\"\n";
    }
  }
}

}  // namespace lidar_mobility_map_generator
