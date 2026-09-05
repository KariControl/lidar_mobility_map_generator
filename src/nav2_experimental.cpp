#include "lidar_mobility_map_generator/nav2_experimental.hpp"

#include "lidar_mobility_map_generator/exporters.hpp"
#include "lidar_mobility_map_generator/nav2_route_export.hpp"
#include "lidar_mobility_map_generator/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

std::string yamlQuote(const std::string & input)
{
  std::string result{"\""};
  for (const char character : input) {
    switch (character) {
      case '\\': result += "\\\\"; break;
      case '"': result += "\\\""; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default: result += character; break;
    }
  }
  result += '"';
  return result;
}

std::string yamlFloatingPoint(const double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("cannot serialize a non-finite YAML floating-point value");
  }
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  std::string result = stream.str();
  if (result.find_first_of(".eE") == std::string::npos) {
    result += ".0";
  }
  return result;
}

void ensureParentDirectory(const std::filesystem::path & path)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
}

void addUnique(std::vector<std::string> & values, const std::string & value)
{
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

bool finitePolyline(const std::vector<Vec3> & points)
{
  return std::all_of(points.begin(), points.end(), [](const Vec3 & point) {return finite(point);});
}

bool edgeEndpointGeometryValid(
  const RouteEdge & edge,
  const std::map<std::uint64_t, Vec3> & nodes)
{
  const auto from = nodes.find(edge.from);
  const auto to = nodes.find(edge.to);
  if (from == nodes.end() || to == nodes.end() || edge.centerline.size() < 2U) {
    return false;
  }
  const double forward_error =
    distance2d(edge.centerline.front(), from->second) +
    distance2d(edge.centerline.back(), to->second);
  const double reverse_error =
    distance2d(edge.centerline.back(), from->second) +
    distance2d(edge.centerline.front(), to->second);
  return std::min(forward_error, reverse_error) <= 2.0e-3;
}

double derivedInflationRadius(const RobotConfig & robot)
{
  // For a circle this is the usual center-point planner inflation. For a
  // rectangle, Nav2 already collision-checks the polygon footprint; using its
  // circumscribed radius here would count the long vehicle extent again and
  // over-inflate a Yaris-sized vehicle by several metres.
  return 0.5 * robot.width + robot.clearance_margin;
}

std::vector<Vec3> orientedCenterline(
  const RouteEdge & edge,
  const std::map<std::uint64_t, Vec3> & nodes)
{
  std::vector<Vec3> points = edge.centerline;
  const auto from = nodes.find(edge.from);
  if (from != nodes.end() && points.size() >= 2U &&
    distance2d(points.back(), from->second) < distance2d(points.front(), from->second))
  {
    std::reverse(points.begin(), points.end());
  }
  return points;
}

std::vector<Vec3> densifyPolyline(
  const std::vector<Vec3> & points,
  const double maximum_spacing)
{
  if (points.size() < 2U) {
    return {};
  }
  std::vector<Vec3> result{points.front()};
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const Vec3 start = result.back();
    const Vec3 end = points[index];
    const double length = distance3d(start, end);
    if (length <= 1.0e-9) {
      continue;
    }
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / maximum_spacing)));
    for (std::size_t piece = 1U; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      result.push_back(start + (end - start) * ratio);
    }
  }
  return result;
}

struct RouteGridIntersection
{
  std::size_t obstacle_samples{0U};
  std::size_t unknown_samples{0U};
  std::size_t off_map_samples{0U};
};

RouteGridIntersection classifyRouteAgainstGrid(
  const std::vector<Nav2WaypointRoute> & routes,
  const OccupancyGrid2D & obstacle_grid,
  const OccupancyGrid2D & unknown_grid)
{
  RouteGridIntersection result;
  // Quarter-cell sampling is stricter than Nav2's normal CostmapScorer
  // sampling and catches short diagonal/corner crossings without depending on
  // an installed Nav2 implementation.
  const double maximum_step = 0.25 * obstacle_grid.resolution();
  for (const Nav2WaypointRoute & route : routes) {
    for (std::size_t index = 1U; index < route.waypoints.size(); ++index) {
      const Vec3 & start = route.waypoints[index - 1U].position;
      const Vec3 & end = route.waypoints[index].position;
      const double length = distance2d(start, end);
      const std::size_t pieces = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(length / maximum_step)));
      for (std::size_t piece = index == 1U ? 0U : 1U; piece <= pieces; ++piece) {
        const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
        const Vec3 sample = start + (end - start) * ratio;
        const auto cell = obstacle_grid.worldToCell(sample.x, sample.y);
        if (!cell) {
          ++result.off_map_samples;
        } else if (obstacle_grid.isOccupied(cell->first, cell->second)) {
          ++result.obstacle_samples;
        } else if (unknown_grid.isOccupied(cell->first, cell->second)) {
          ++result.unknown_samples;
        }
      }
    }
  }
  return result;
}

double waypointYaw(
  const std::vector<Vec3> & points,
  const std::size_t index,
  const bool closed_loop)
{
  if (points.size() < 2U) {
    return 0.0;
  }
  if (index + 1U < points.size()) {
    return std::atan2(
      points[index + 1U].y - points[index].y,
      points[index + 1U].x - points[index].x);
  }
  if (closed_loop && distance2d(points.back(), points.front()) > 1.0e-9) {
    return std::atan2(
      points.front().y - points.back().y,
      points.front().x - points.back().x);
  }
  return std::atan2(
    points.back().y - points[points.size() - 2U].y,
    points.back().x - points[points.size() - 2U].x);
}

bool passableGraphWeaklyConnected(const RouteGraph & graph)
{
  std::map<std::uint64_t, std::vector<std::uint64_t>> adjacency;
  for (const RouteEdge & edge : graph.edges) {
    if (!edge.passable || edge.centerline.size() < 2U) {
      continue;
    }
    adjacency[edge.from].push_back(edge.to);
    adjacency[edge.to].push_back(edge.from);
  }
  if (adjacency.empty()) {
    return false;
  }
  std::set<std::uint64_t> visited;
  std::queue<std::uint64_t> pending;
  pending.push(adjacency.begin()->first);
  visited.insert(adjacency.begin()->first);
  while (!pending.empty()) {
    const std::uint64_t current = pending.front();
    pending.pop();
    for (const std::uint64_t next : adjacency[current]) {
      if (visited.insert(next).second) {
        pending.push(next);
      }
    }
  }
  return visited.size() == adjacency.size();
}

void writeStringList(
  std::ostream & stream,
  const std::string & key,
  const std::vector<std::string> & values)
{
  stream << key << ':';
  if (values.empty()) {
    stream << " []\n";
    return;
  }
  stream << '\n';
  for (const std::string & value : values) {
    stream << "    - " << yamlQuote(value) << '\n';
  }
}

RouteGraph speedLimitedGraph(
  const RouteGraph & graph,
  const double maximum_linear_speed_mps)
{
  RouteGraph result = graph;
  for (RouteEdge & edge : result.edges) {
    if (!(edge.recommended_speed_mps > 0.0) ||
      edge.recommended_speed_mps > maximum_linear_speed_mps)
    {
      edge.recommended_speed_mps = maximum_linear_speed_mps;
    }
  }
  return result;
}

double waypointPolylineLength(const std::vector<Vec3> & points)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length += distance3d(points[index - 1U], points[index]);
  }
  return length;
}

Vec3 sampleWaypointPolylineAtArc(
  const std::vector<Vec3> & points, const double requested)
{
  if (points.size() < 2U || !std::isfinite(requested)) {
    throw std::invalid_argument("cannot sample a degenerate or non-finite waypoint edge");
  }
  const double length = waypointPolylineLength(points);
  if (!(length > 1.0e-12) || !std::isfinite(length)) {
    throw std::invalid_argument("cannot sample a zero-length waypoint edge");
  }
  const double target = clamp(requested, 0.0, length);
  double traversed = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double segment = distance3d(points[index - 1U], points[index]);
    if (target <= traversed + segment || index + 1U == points.size()) {
      const double ratio = segment > 1.0e-12 ?
        clamp((target - traversed) / segment, 0.0, 1.0) : 0.0;
      return points[index - 1U] + (points[index] - points[index - 1U]) * ratio;
    }
    traversed += segment;
  }
  return points.back();
}

std::vector<Vec3> waypointPolylineInterval(
  const std::vector<Vec3> & points, const double requested_start,
  const double requested_end)
{
  constexpr double tolerance = 1.0e-9;
  const double length = waypointPolylineLength(points);
  const double start = clamp(requested_start, 0.0, length);
  const double end = clamp(requested_end, 0.0, length);
  if (end + tolerance < start) {
    throw std::invalid_argument("waypoint edge interval is reversed");
  }
  std::vector<Vec3> result{sampleWaypointPolylineAtArc(points, start)};
  double traversed = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    traversed += distance3d(points[index - 1U], points[index]);
    if (traversed > start + tolerance && traversed < end - tolerance &&
      distance3d(result.back(), points[index]) > tolerance)
    {
      result.push_back(points[index]);
    }
  }
  const Vec3 endpoint = sampleWaypointPolylineAtArc(points, end);
  if (distance3d(result.back(), endpoint) > tolerance) {
    result.push_back(endpoint);
  }
  return result;
}

struct OrderedStopMarker
{
  const AuthoredStopLine * stop{nullptr};
  double travel_s{0.0};
  Vec3 sampled_position{};
};

void rebuildNamedWaypointRouteWithStops(
  Nav2WaypointRoute & route,
  const RouteGraph & graph,
  const NamedNavigationRoute & named_route,
  const std::vector<AuthoredStopLine> & stops,
  const double maximum_waypoint_spacing,
  const double maximum_chord_error)
{
  constexpr double arc_tolerance = 1.0e-8;
  constexpr double duplicate_tolerance = 1.0e-9;
  // Kept in the public API for configuration compatibility. Stop intervals
  // retain every original centerline vertex; chord-error simplification is not
  // permitted because it shortens the selected mission.
  static_cast<void>(maximum_chord_error);
  std::map<std::uint64_t, Vec3> nodes;
  for (const RouteNode & node : graph.nodes) {
    if (!finite(node.position) || !nodes.emplace(node.id, node.position).second) {
      throw std::invalid_argument("named Nav2 Route contains an invalid or duplicate node");
    }
  }
  std::map<std::uint64_t, const RouteEdge *> edges;
  for (const RouteEdge & edge : graph.edges) {
    if (!edges.emplace(edge.id, &edge).second) {
      throw std::invalid_argument("named Nav2 Route graph contains duplicate Edge IDs");
    }
  }

  const std::set<std::uint64_t> built_edge_ids(
    route.source_edge_ids.begin(), route.source_edge_ids.end());
  const std::set<std::uint64_t> authored_edge_ids(
    named_route.ordered_edge_ids.begin(), named_route.ordered_edge_ids.end());
  if (built_edge_ids.size() != route.source_edge_ids.size() ||
    authored_edge_ids.size() != named_route.ordered_edge_ids.size() ||
    built_edge_ids != authored_edge_ids)
  {
    throw std::invalid_argument(
            "selected named Nav2 Route does not exactly match the built waypoint chain");
  }

  std::map<std::uint64_t, std::vector<const AuthoredStopLine *>> stops_by_edge;
  std::set<std::uint64_t> stop_ids;
  for (const AuthoredStopLine & stop : stops) {
    if (stop.id == 0U || !stop_ids.insert(stop.id).second) {
      throw std::invalid_argument("resolved Nav2 stop-line IDs must be unique and nonzero");
    }
    if (stop.name.empty() || !std::isfinite(stop.s) ||
      !(stop.width_m > 0.0) || !std::isfinite(stop.width_m) || !finite(stop.anchor))
    {
      throw std::invalid_argument("resolved Nav2 stop line contains non-finite or empty data");
    }
    if (!includesTarget(stop.target, NavigationAuthoringTarget::kNav2)) {
      throw std::invalid_argument("resolved Nav2 stop line has a non-Nav2 target");
    }
    if (authored_edge_ids.count(stop.edge_id) == 0U || edges.count(stop.edge_id) == 0U) {
      throw std::invalid_argument(
              "resolved Nav2 stop line Edge is absent from the selected named Route: " +
              std::to_string(stop.edge_id));
    }
    stops_by_edge[stop.edge_id].push_back(&stop);
  }

  route.source_edge_ids = named_route.ordered_edge_ids;
  route.waypoints.clear();
  const RouteEdge * previous_edge = nullptr;
  std::size_t inserted_stops = 0U;
  auto append_position = [&](const Vec3 & point) -> std::size_t {
      if (!finite(point)) {
        throw std::invalid_argument("named Nav2 waypoint is non-finite");
      }
      if (route.waypoints.empty() ||
        distance3d(route.waypoints.back().position, point) > duplicate_tolerance)
      {
        Nav2WaypointPose waypoint;
        waypoint.position = point;
        route.waypoints.push_back(std::move(waypoint));
      }
      return route.waypoints.size() - 1U;
    };
  auto append_interval = [&](const std::vector<Vec3> & oriented_points,
      const double start, const double end) {
      const std::vector<Vec3> interval = waypointPolylineInterval(
        oriented_points, start, end);
      const std::vector<Vec3> dense = interval.size() >= 2U ?
        densifyPolyline(interval, maximum_waypoint_spacing) : interval;
      if (dense.empty()) {
        throw std::invalid_argument("named Nav2 waypoint interval cannot be materialized");
      }
      for (const Vec3 & point : dense) {
        append_position(point);
      }
    };

  for (std::size_t edge_order = 0U;
    edge_order < named_route.ordered_edge_ids.size(); ++edge_order)
  {
    const std::uint64_t edge_id = named_route.ordered_edge_ids[edge_order];
    const auto found = edges.find(edge_id);
    if (found == edges.end()) {
      throw std::invalid_argument(
              "selected named Nav2 Route Edge is absent: " + std::to_string(edge_id));
    }
    const RouteEdge & edge = *found->second;
    if (!edge.passable || edge.centerline.size() < 2U || !finitePolyline(edge.centerline) ||
      nodes.count(edge.from) == 0U || nodes.count(edge.to) == 0U ||
      !edgeEndpointGeometryValid(edge, nodes))
    {
      throw std::invalid_argument("selected named Nav2 Route Edge is not exportable");
    }
    if (edge_order == 0U && edge.from != named_route.start_node_id) {
      throw std::invalid_argument("named Nav2 Route start node does not match its first Edge");
    }
    if (previous_edge != nullptr && previous_edge->to != edge.from) {
      throw std::invalid_argument("named Nav2 Route Edge order is disconnected");
    }
    if (edge_order + 1U == named_route.ordered_edge_ids.size() &&
      edge.to != named_route.end_node_id)
    {
      throw std::invalid_argument("named Nav2 Route end node does not match its last Edge");
    }

    const std::vector<Vec3> oriented_points = orientedCenterline(edge, nodes);
    const double length = waypointPolylineLength(oriented_points);
    if (!(length > 1.0e-12) || !std::isfinite(length)) {
      throw std::invalid_argument("selected named Nav2 Route Edge has zero length");
    }
    const bool raw_reversed =
      distance2d(edge.centerline.back(), nodes.at(edge.from)) <
      distance2d(edge.centerline.front(), nodes.at(edge.from));
    std::vector<OrderedStopMarker> markers;
    for (const AuthoredStopLine * stop : stops_by_edge[edge.id]) {
      if (stop->s < -arc_tolerance || stop->s > length + arc_tolerance) {
        throw std::invalid_argument(
                "resolved Nav2 stop-line arc length is outside its Edge");
      }
      const double clamped_s = clamp(stop->s, 0.0, length);
      markers.push_back({
        stop,
        raw_reversed ? length - clamped_s : clamped_s,
        sampleWaypointPolylineAtArc(edge.centerline, clamped_s)});
    }
    std::sort(
      markers.begin(), markers.end(),
      [](const OrderedStopMarker & lhs, const OrderedStopMarker & rhs) {
        if (lhs.travel_s != rhs.travel_s) {
          return lhs.travel_s < rhs.travel_s;
        }
        return lhs.stop->id < rhs.stop->id;
      });
    for (std::size_t index = 1U; index < markers.size(); ++index) {
      if (std::abs(markers[index].travel_s - markers[index - 1U].travel_s) <=
        duplicate_tolerance)
      {
        throw std::invalid_argument(
                "multiple resolved Nav2 stop lines occupy the same Route position");
      }
    }

    double interval_start = 0.0;
    for (const OrderedStopMarker & marker : markers) {
      append_interval(oriented_points, interval_start, marker.travel_s);
      const std::size_t waypoint_index = append_position(marker.sampled_position);
      Nav2WaypointPose & waypoint = route.waypoints[waypoint_index];
      if (waypoint.authored_stop_line_id) {
        throw std::invalid_argument(
                "multiple resolved Nav2 stop lines map to one waypoint");
      }
      waypoint.position = marker.sampled_position;
      waypoint.authored_stop_line_id = marker.stop->id;
      waypoint.authored_stop_line_name = marker.stop->name;
      waypoint.authored_stop_edge_id = marker.stop->edge_id;
      waypoint.authored_stop_edge_s_m = marker.stop->s;
      waypoint.authored_stop_width_m = marker.stop->width_m;
      ++inserted_stops;
      interval_start = marker.travel_s;
    }
    append_interval(oriented_points, interval_start, length);
    previous_edge = &edge;
  }

  if (route.waypoints.size() < 2U || inserted_stops != stops.size()) {
    throw std::runtime_error(
            "resolved Nav2 stop-line insertion count does not match its input");
  }
  std::vector<Vec3> positions;
  positions.reserve(route.waypoints.size());
  for (const Nav2WaypointPose & waypoint : route.waypoints) {
    positions.push_back(waypoint.position);
  }
  route.closed_loop = previous_edge != nullptr &&
    named_route.start_node_id == named_route.end_node_id;
  for (std::size_t index = 0U; index < route.waypoints.size(); ++index) {
    route.waypoints[index].yaw = waypointYaw(positions, index, route.closed_loop);
  }
}

void validateArtifactPaths(const Nav2ClosedCourseArtifacts & artifacts)
{
  const std::vector<std::filesystem::path> paths{
    artifacts.map_pgm,
    artifacts.map_yaml,
    artifacts.route_graph_geojson,
    artifacts.waypoint_routes_yaml,
    artifacts.nav2_params_overlay_yaml,
    artifacts.readiness_yaml};
  if (std::any_of(paths.begin(), paths.end(), [](const auto & path) {return path.empty();})) {
    throw std::invalid_argument("Nav2 closed-course artifact paths must not be empty");
  }
}

}  // namespace

std::vector<Nav2WaypointRoute> buildNav2WaypointRoutes(
  const RouteGraph & graph,
  const double maximum_waypoint_spacing,
  const double maximum_chord_error)
{
  if (!(maximum_waypoint_spacing > 0.0) || !std::isfinite(maximum_waypoint_spacing)) {
    throw std::invalid_argument("maximum Nav2 waypoint spacing must be finite and positive");
  }
  if (!(maximum_chord_error > 0.0) || !std::isfinite(maximum_chord_error)) {
    throw std::invalid_argument("maximum Nav2 waypoint chord error must be finite and positive");
  }
  // Retain this validated argument for configuration/API compatibility. The
  // lossless waypoint builder never removes source vertices by chord error.
  static_cast<void>(maximum_chord_error);

  std::map<std::uint64_t, Vec3> nodes;
  for (const RouteNode & node : graph.nodes) {
    if (!finite(node.position) || !nodes.emplace(node.id, node.position).second) {
      throw std::invalid_argument("Nav2 waypoint graph contains an invalid or duplicate node");
    }
  }

  std::vector<const RouteEdge *> edges;
  std::set<std::uint64_t> edge_ids;
  for (const RouteEdge & edge : graph.edges) {
    if (!edge.passable) {
      continue;
    }
    if (nodes.count(edge.from) == 0U || nodes.count(edge.to) == 0U ||
      edge.centerline.size() < 2U || !finitePolyline(edge.centerline) ||
      !edgeEndpointGeometryValid(edge, nodes) ||
      !edge_ids.insert(edge.id).second)
    {
      throw std::invalid_argument("Nav2 waypoint graph contains an invalid passable edge");
    }
    edges.push_back(&edge);
  }
  std::sort(
    edges.begin(), edges.end(),
    [](const RouteEdge * lhs, const RouteEdge * rhs) {return lhs->id < rhs->id;});

  std::map<std::uint64_t, std::vector<std::size_t>> outgoing;
  std::map<std::uint64_t, std::size_t> incoming_count;
  for (std::size_t index = 0U; index < edges.size(); ++index) {
    outgoing[edges[index]->from].push_back(index);
    ++incoming_count[edges[index]->to];
  }
  for (auto & [node_id, edge_indices] : outgoing) {
    static_cast<void>(node_id);
    std::sort(
      edge_indices.begin(), edge_indices.end(),
      [&](const std::size_t lhs, const std::size_t rhs) {
        return edges[lhs]->id < edges[rhs]->id;
      });
  }

  std::vector<bool> visited(edges.size(), false);
  std::vector<std::vector<std::size_t>> chains;
  const auto walk = [&](const std::size_t start) {
      std::vector<std::size_t> chain;
      std::size_t current = start;
      while (!visited[current]) {
        visited[current] = true;
        chain.push_back(current);
        const std::uint64_t node = edges[current]->to;
        const auto next = outgoing.find(node);
        if (incoming_count[node] != 1U || next == outgoing.end() || next->second.size() != 1U) {
          break;
        }
        current = next->second.front();
      }
      return chain;
    };

  for (std::size_t index = 0U; index < edges.size(); ++index) {
    const std::uint64_t start_node = edges[index]->from;
    const std::size_t outgoing_count = outgoing[start_node].size();
    if (!visited[index] && (incoming_count[start_node] != 1U || outgoing_count != 1U)) {
      chains.push_back(walk(index));
    }
  }
  // The remaining edges are directed cycles; choose the lowest edge ID as the
  // deterministic first edge of each cycle.
  for (std::size_t index = 0U; index < edges.size(); ++index) {
    if (!visited[index]) {
      chains.push_back(walk(index));
    }
  }

  std::vector<Nav2WaypointRoute> routes;
  for (const std::vector<std::size_t> & chain : chains) {
    if (chain.empty()) {
      continue;
    }
    std::vector<Vec3> polyline;
    Nav2WaypointRoute route;
    route.id = routes.size();
    route.closed_loop =
      edges[chain.front()]->from == edges[chain.back()]->to;
    for (const std::size_t edge_index : chain) {
      const RouteEdge & edge = *edges[edge_index];
      route.source_edge_ids.push_back(edge.id);
      const std::vector<Vec3> edge_points = orientedCenterline(edge, nodes);
      // Preserve every original point in each source Edge. The final linear
      // densification adds poses only within an original segment, so it cannot
      // cut a corner or change the 2-D/3-D source arc length.
      for (const Vec3 & point : edge_points) {
        if (polyline.empty() || distance3d(polyline.back(), point) > 1.0e-9) {
          polyline.push_back(point);
        }
      }
    }
    const std::vector<Vec3> dense = densifyPolyline(polyline, maximum_waypoint_spacing);
    if (dense.size() < 2U) {
      continue;
    }
    route.waypoints.reserve(dense.size());
    for (std::size_t index = 0U; index < dense.size(); ++index) {
      Nav2WaypointPose waypoint;
      waypoint.position = dense[index];
      waypoint.yaw = waypointYaw(dense, index, route.closed_loop);
      route.waypoints.push_back(std::move(waypoint));
    }
    routes.push_back(std::move(route));
  }
  return routes;
}

Nav2ClosedCourseAssessment evaluateNav2ClosedCourseExperiment(
  const PipelineResult & pipeline,
  const RouteGraph & closed_course_graph,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & controls)
{
  Nav2ClosedCourseAssessment assessment;
  const TraversabilityGridResult & grids = pipeline.grids;
  assessment.map_server_compatible =
    !grids.obstacle_grid.empty() &&
    hasMatchingGridGeometry(grids.obstacle_grid, grids.observed_free_grid) &&
    hasMatchingGridGeometry(grids.obstacle_grid, grids.unknown_grid);
  if (!assessment.map_server_compatible) {
    addUnique(assessment.map_blockers, "nav2_grid_geometry_invalid");
  }

  if (!grids.obstacle_grid.empty()) {
    for (std::size_t y = 0U; y < grids.obstacle_grid.height(); ++y) {
      for (std::size_t x = 0U; x < grids.obstacle_grid.width(); ++x) {
        const auto cell_x = static_cast<std::int64_t>(x);
        const auto cell_y = static_cast<std::int64_t>(y);
        const bool obstacle = grids.obstacle_grid.isOccupied(cell_x, cell_y);
        const bool free = grids.observed_free_grid.isOccupied(cell_x, cell_y);
        const bool unknown = grids.unknown_grid.isOccupied(cell_x, cell_y);
        assessment.obstacle_cells += obstacle ? 1U : 0U;
        assessment.explicit_free_cells += free ? 1U : 0U;
        assessment.unknown_cells += unknown ? 1U : 0U;
        const unsigned int classifications =
          static_cast<unsigned int>(obstacle) + static_cast<unsigned int>(free) +
          static_cast<unsigned int>(unknown);
        assessment.unclassified_cells += classifications == 0U ? 1U : 0U;
        assessment.overlapping_classification_cells += classifications > 1U ? 1U : 0U;
      }
    }
  }
  assessment.classification_partition_complete = assessment.map_server_compatible &&
    assessment.unclassified_cells == 0U && assessment.overlapping_classification_cells == 0U;
  if (!assessment.classification_partition_complete) {
    addUnique(assessment.map_blockers, "occupancy_classification_partition_invalid");
  }
  if (assessment.explicit_free_cells == 0U) {
    addUnique(assessment.map_blockers, "no_explicit_free_cells");
  }

  const std::string & free_mode = config.generator.traversability.free_space_evidence_mode;
  assessment.direct_free_space_evidence_selected =
    free_mode == "ground_observations" || free_mode == "combined";
  if (!assessment.direct_free_space_evidence_selected) {
    addUnique(
      assessment.map_blockers,
      free_mode == "trajectory" ?
      "trajectory_only_free_space_evidence_not_promotable" :
      "direct_free_space_evidence_not_selected");
  }
  if (config.generator.traversability.unknown_space_policy != "occupied") {
    addUnique(assessment.map_blockers, "unknown_space_policy_not_fail_closed");
  }
  if (config.output.frame_id != "map" || closed_course_graph.frame_id != "map") {
    addUnique(assessment.map_blockers, "navigation_frame_is_not_map");
  }

  const RobotConfig & robot = config.generator.robot;
  const bool dimensions_supported = robot.dimensions_verified ||
    evidenceSupportsClosedCourseExperiment(
    robot.dimensions_source, robot.dimensions_confidence);
  const bool extrinsics_supported = config.extrinsics.verified ||
    evidenceSupportsClosedCourseExperiment(
    config.extrinsics.calibration_source, config.extrinsics.calibration_confidence);
  if (!dimensions_supported) {
    addUnique(assessment.map_blockers, "robot_dimension_evidence_below_experimental_threshold");
  }
  if (!extrinsics_supported) {
    addUnique(assessment.map_blockers, "lidar_extrinsic_evidence_below_experimental_threshold");
  }
  if (!controls.enabled) {
    addUnique(assessment.map_blockers, "closed_course_experimental_output_not_enabled");
  }
  if (controls.base_frame.empty() || controls.odom_frame.empty() ||
    controls.base_frame == "map" || controls.odom_frame == controls.base_frame)
  {
    addUnique(assessment.map_blockers, "nav2_runtime_frames_invalid");
  }
  if (controls.obstacle_pointcloud_topic.empty()) {
    addUnique(assessment.map_blockers, "local_obstacle_pointcloud_topic_empty");
  }
  if (!(controls.maximum_linear_speed_mps > 0.0) ||
    !std::isfinite(controls.maximum_linear_speed_mps) ||
    controls.maximum_linear_speed_mps > 0.50)
  {
    addUnique(assessment.map_blockers, "closed_course_linear_speed_limit_invalid");
  }
  if (!(controls.maximum_angular_speed_rps > 0.0) ||
    !std::isfinite(controls.maximum_angular_speed_rps) ||
    controls.maximum_angular_speed_rps > 1.50)
  {
    addUnique(assessment.map_blockers, "closed_course_angular_speed_limit_invalid");
  }
  if (!(controls.maximum_waypoint_spacing > 0.0) ||
    !std::isfinite(controls.maximum_waypoint_spacing) ||
    controls.maximum_waypoint_spacing > 2.0)
  {
    addUnique(assessment.map_blockers, "closed_course_waypoint_spacing_invalid");
  }
  if (!(controls.cost_scaling_factor > 0.0) ||
    !std::isfinite(controls.cost_scaling_factor))
  {
    addUnique(assessment.map_blockers, "nav2_cost_scaling_factor_invalid");
  }
  assessment.costmap_inflation_radius = derivedInflationRadius(robot);
  if (!(assessment.costmap_inflation_radius > 0.0) ||
    !std::isfinite(assessment.costmap_inflation_radius))
  {
    addUnique(assessment.map_blockers, "derived_costmap_inflation_radius_invalid");
  }

  std::map<std::uint64_t, Vec3> route_nodes;
  std::set<std::uint64_t> duplicate_nodes;
  for (const RouteNode & node : closed_course_graph.nodes) {
    if (!finite(node.position) || !route_nodes.emplace(node.id, node.position).second) {
      duplicate_nodes.insert(node.id);
    }
  }
  std::set<std::uint64_t> route_edge_ids;
  for (const RouteEdge & edge : closed_course_graph.edges) {
    if (!edge.passable) {
      continue;
    }
    ++assessment.passable_route_edges;
    const bool valid = route_nodes.count(edge.from) != 0U && route_nodes.count(edge.to) != 0U &&
      duplicate_nodes.count(edge.from) == 0U && duplicate_nodes.count(edge.to) == 0U &&
      edge.centerline.size() >= 2U && finitePolyline(edge.centerline) &&
      edgeEndpointGeometryValid(edge, route_nodes) &&
      route_edge_ids.insert(edge.id).second;
    assessment.invalid_passable_route_edges += valid ? 0U : 1U;
  }
  if (assessment.passable_route_edges == 0U) {
    addUnique(assessment.follow_waypoints_blockers, "no_valid_closed_course_route");
    addUnique(assessment.route_server_blockers, "no_valid_closed_course_route");
  }
  if (assessment.invalid_passable_route_edges != 0U || !duplicate_nodes.empty()) {
    addUnique(assessment.follow_waypoints_blockers, "closed_course_route_graph_invalid");
    addUnique(assessment.route_server_blockers, "closed_course_route_graph_invalid");
  }
  if (closed_course_graph.frame_id != "map") {
    addUnique(assessment.follow_waypoints_blockers, "waypoint_frame_is_not_map");
    addUnique(assessment.route_server_blockers, "route_graph_frame_is_not_map");
  }
  if (!(config.output.nav2_route_max_chord_error > 0.0) ||
    !(config.output.nav2_route_max_segment_length > 0.0) ||
    !std::isfinite(config.output.nav2_route_max_chord_error) ||
    !std::isfinite(config.output.nav2_route_max_segment_length))
  {
    addUnique(
      assessment.route_server_blockers,
      "nav2_route_segmentation_parameters_invalid");
  }

  if (assessment.invalid_passable_route_edges == 0U &&
    assessment.passable_route_edges != 0U &&
    controls.maximum_waypoint_spacing > 0.0 &&
    std::isfinite(controls.maximum_waypoint_spacing))
  {
    try {
      const std::vector<Nav2WaypointRoute> routes = buildNav2WaypointRoutes(
        closed_course_graph, controls.maximum_waypoint_spacing,
        config.output.nav2_route_max_chord_error);
      assessment.waypoint_routes = routes.size();
      for (const Nav2WaypointRoute & route : routes) {
        assessment.waypoints += route.waypoints.size();
      }
      if (assessment.map_server_compatible) {
        const RouteGridIntersection intersections = classifyRouteAgainstGrid(
          routes, grids.obstacle_grid, grids.unknown_grid);
        assessment.route_obstacle_samples = intersections.obstacle_samples;
        assessment.route_unknown_samples = intersections.unknown_samples;
        assessment.route_off_map_samples = intersections.off_map_samples;
        if (assessment.route_obstacle_samples != 0U) {
          addUnique(assessment.follow_waypoints_blockers, "route_intersects_static_obstacle");
          addUnique(assessment.route_server_blockers, "route_intersects_static_obstacle");
        }
        if (assessment.route_unknown_samples != 0U) {
          addUnique(assessment.follow_waypoints_blockers, "route_intersects_unknown_space");
          addUnique(assessment.route_server_blockers, "route_intersects_unknown_space");
        }
        if (assessment.route_off_map_samples != 0U) {
          addUnique(assessment.follow_waypoints_blockers, "route_leaves_static_map");
          addUnique(assessment.route_server_blockers, "route_leaves_static_map");
        }
      }
    } catch (const std::exception &) {
      addUnique(assessment.follow_waypoints_blockers, "waypoint_route_conversion_failed");
    }
  }
  assessment.follow_waypoints_compatible =
    assessment.waypoint_routes != 0U && assessment.waypoints >= 2U &&
    assessment.follow_waypoints_blockers.empty();
  if (!assessment.follow_waypoints_compatible && assessment.waypoint_routes == 0U) {
    addUnique(assessment.follow_waypoints_blockers, "no_connected_waypoint_route");
  }
  assessment.route_server_compatible =
    assessment.passable_route_edges != 0U &&
    assessment.route_server_blockers.empty();

  assessment.static_map_artifact_ready = assessment.map_blockers.empty();
  assessment.follow_waypoints_artifact_ready =
    assessment.static_map_artifact_ready && assessment.follow_waypoints_compatible;
  assessment.route_server_artifact_ready =
    assessment.static_map_artifact_ready && assessment.route_server_compatible;
  // FollowWaypoints and Route Server are alternative consumers of the same
  // connected route evidence. The action-goal artifact is the conservative
  // baseline because it does not require Route Server BT integration.
  assessment.closed_course_artifact_ready = assessment.follow_waypoints_artifact_ready;

  if (!controls.operator_acknowledged_experimental_only) {
    addUnique(assessment.deployment_blockers, "operator_experimental_only_acknowledgement_missing");
  }
  if ((!robot.dimensions_verified || !config.extrinsics.verified) &&
    !controls.estimated_geometry_acknowledged)
  {
    addUnique(assessment.deployment_blockers, "estimated_geometry_acknowledgement_missing");
  }
  if (!controls.closed_course_access_controlled) {
    addUnique(assessment.deployment_blockers, "closed_course_access_control_not_confirmed");
  }
  if (!controls.free_space_reviewed_for_session) {
    addUnique(assessment.deployment_blockers, "session_free_space_review_missing");
  }
  if (!controls.localization_alignment_checked_for_session) {
    addUnique(assessment.deployment_blockers, "session_localization_alignment_check_missing");
  }
  if (!controls.emergency_stop_available) {
    addUnique(assessment.deployment_blockers, "emergency_stop_not_confirmed");
  }
  assessment.static_map_deployment_ready =
    assessment.static_map_artifact_ready && assessment.deployment_blockers.empty();
  assessment.follow_waypoints_deployment_ready =
    assessment.follow_waypoints_artifact_ready && assessment.deployment_blockers.empty();
  assessment.route_server_deployment_ready =
    assessment.route_server_artifact_ready && assessment.deployment_blockers.empty();
  assessment.closed_course_deployment_ready = assessment.follow_waypoints_deployment_ready;

  addUnique(assessment.limitations, "closed_course_experimental_only_not_production_ready");
  addUnique(assessment.limitations, "global_costmap_uses_static_obstacle_map_only");
  addUnique(
    assessment.limitations,
    "local_live_obstacle_avoidance_requires_current_registered_pointcloud");
  addUnique(assessment.limitations, "unknown_cells_remain_impassable");
  addUnique(assessment.limitations, "offline_map_is_not_robot_footprint_inflated");
  addUnique(assessment.limitations, "one_nav2_inflation_layer_per_costmap_required");
  addUnique(
    assessment.limitations,
    "route_server_behavior_tree_integration_must_match_installed_nav2_distribution");
  if (!robot.dimensions_verified) {
    addUnique(assessment.limitations, "robot_dimensions_are_estimated");
  }
  if (!config.extrinsics.verified) {
    addUnique(assessment.limitations, "lidar_extrinsics_are_estimated");
  }
  if (!config.output.nav2_free_space_verified) {
    addUnique(assessment.limitations, "free_space_is_not_production_verified");
  }
  if (!grids.has_multi_scan_observation_support) {
    addUnique(assessment.limitations, "obstacle_map_lacks_multi_scan_support");
  }
  if (!passableGraphWeaklyConnected(closed_course_graph)) {
    addUnique(assessment.limitations, "closed_course_route_graph_has_multiple_components");
  }
  if (assessment.waypoint_routes > 1U) {
    addUnique(assessment.limitations, "branches_are_exported_as_separate_waypoint_routes");
  }
  return assessment;
}

void saveNav2WaypointRoutesYaml(
  const std::filesystem::path & path,
  const std::string & frame_id,
  const std::vector<Nav2WaypointRoute> & routes,
  const bool experiment_ready)
{
  ensureParentDirectory(path);
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create Nav2 waypoint route file: " + path.string());
  }
  stream << std::setprecision(12)
         << "schema_version: 2\n"
         << "format: \"lmmg_nav2_follow_waypoints_routes\"\n"
         << "action_type: \"nav2_msgs/action/FollowWaypoints\"\n"
         << "action_name: \"/follow_waypoints\"\n"
         << "frame_id: " << yamlQuote(frame_id) << '\n'
         << "experimental_only: true\n"
         << "production_ready: false\n"
         << "artifact_ready: " << (experiment_ready ? "true" : "false") << '\n'
         << "routes:";
  if (routes.empty()) {
    stream << " []\n";
    return;
  }
  stream << '\n';
  for (const Nav2WaypointRoute & route : routes) {
    stream << "  - route_id: " << route.id << '\n';
    if (route.source_named_route_id) {
      stream << "    named_route_id: " << *route.source_named_route_id << '\n';
    }
    if (route.name) {
      stream << "    name: " << yamlQuote(*route.name) << '\n';
    }
    if (route.target) {
      stream << "    target: " << yamlQuote(toString(*route.target)) << '\n';
    }
    const std::size_t authored_stop_waypoint_count = static_cast<std::size_t>(
      std::count_if(
        route.waypoints.begin(), route.waypoints.end(),
        [](const Nav2WaypointPose & waypoint) {
          return waypoint.authored_stop_line_id.has_value();
        }));
    if (route.source_named_route_id || authored_stop_waypoint_count != 0U) {
      stream << "    authored_stop_waypoint_count: " <<
        authored_stop_waypoint_count << '\n'
             << "    stop_behavior: \"waypoint_arrival_only\"\n";
    }
    stream << "    closed_loop: " << (route.closed_loop ? "true" : "false") << '\n'
           << "    source_edge_ids: [";
    for (std::size_t index = 0U; index < route.source_edge_ids.size(); ++index) {
      if (index > 0U) {stream << ", ";}
      stream << route.source_edge_ids[index];
    }
    stream << "]\n"
           << "    waypoints:\n";
    for (const Nav2WaypointPose & waypoint : route.waypoints) {
      stream << "      - {x: " << waypoint.position.x
             << ", y: " << waypoint.position.y
             << ", z: 0.0"
             << ", source_map_z: " << waypoint.position.z
             << ", yaw: " << waypoint.yaw;
      if (waypoint.authored_stop_line_id) {
        stream << ", authored_stop_line_id: " << *waypoint.authored_stop_line_id;
      }
      if (waypoint.authored_stop_line_name) {
        stream << ", authored_stop_line_name: " <<
          yamlQuote(*waypoint.authored_stop_line_name);
      }
      if (waypoint.authored_stop_edge_id) {
        stream << ", authored_stop_edge_id: " << *waypoint.authored_stop_edge_id;
      }
      if (waypoint.authored_stop_edge_s_m) {
        stream << ", authored_stop_edge_s_m: " << *waypoint.authored_stop_edge_s_m;
      }
      if (waypoint.authored_stop_width_m) {
        stream << ", authored_stop_width_m: " << *waypoint.authored_stop_width_m;
      }
      stream << "}\n";
    }
  }
}

void saveNav2ClosedCourseParamsOverlayYaml(
  const std::filesystem::path & path,
  const std::filesystem::path & map_yaml,
  const std::filesystem::path & route_graph_geojson,
  const RobotConfig & robot,
  const Nav2ClosedCourseControls & controls,
  const Nav2ClosedCourseAssessment & assessment)
{
  ensureParentDirectory(path);
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create Nav2 closed-course params: " + path.string());
  }
  const std::filesystem::path absolute_map = std::filesystem::absolute(map_yaml);
  const std::filesystem::path absolute_graph = std::filesystem::absolute(route_graph_geojson);
  const double half_width = 0.5 * robot.width;
  const double reverse_speed = robot.allow_reverse_motion ? -controls.maximum_linear_speed_mps : 0.0;

  stream << std::setprecision(12)
         << "# CLOSED-COURSE EXPERIMENTAL OVERLAY; never use as production certification.\n"
         << "# The referenced PGM contains raw obstacles, explicit FREE and UNKNOWN.\n"
         << "# Do not replace it with obstacles_inflated.pgm: each costmap inflates once below.\n"
         << "lmmg_closed_course_contract:\n"
         << "  ros__parameters:\n"
         << "    production_ready: false\n"
         << "    artifact_ready: " <<
    (assessment.closed_course_artifact_ready ? "true" : "false") << '\n'
         << "    deployment_ready: " <<
    (assessment.closed_course_deployment_ready ? "true" : "false") << '\n'
         << "    offline_obstacle_inflation_applied: false\n"
         << "    unknown_promoted_to_free: false\n"
         << "map_server:\n"
         << "  ros__parameters:\n"
         << "    yaml_filename: " << yamlQuote(absolute_map.string()) << '\n'
         << "planner_server:\n"
         << "  ros__parameters:\n"
         << "    planner_plugins: [\"GridBased\"]\n"
         << "    GridBased:\n"
         << "      plugin: \"nav2_navfn_planner::NavfnPlanner\"\n"
         << "      tolerance: 0.25\n"
         << "      use_astar: true\n"
         << "      allow_unknown: false\n";

  const auto write_costmap = [
    &stream, &controls, &robot, &assessment, half_width](
    const std::string & name, const std::string & global_frame, const bool rolling_window) {
      stream << name << ":\n"
             << "  " << name << ":\n"
             << "    ros__parameters:\n"
             << "      global_frame: " << yamlQuote(global_frame) << '\n'
             << "      robot_base_frame: " << yamlQuote(controls.base_frame) << '\n'
             << "      rolling_window: " << (rolling_window ? "true" : "false") << '\n';
      if (rolling_window) {
        stream << "      width: 10.0\n"
               << "      height: 10.0\n";
      }
      if (robot.footprint_model == "circle") {
        stream << "      robot_radius: " << yamlFloatingPoint(0.5 * robot.width) << '\n';
      } else {
        std::ostringstream footprint;
        footprint << std::setprecision(12)
                  << "[[" << robot.front_extent << ", " << half_width << "], ["
                  << robot.front_extent << ", " << -half_width << "], ["
                  << -robot.rear_extent << ", " << -half_width << "], ["
                  << -robot.rear_extent << ", " << half_width << "]]";
        stream << "      footprint: " << yamlQuote(footprint.str()) << '\n';
      }
      stream << "      footprint_padding: 0.0\n"
             << "      track_unknown_space: true\n"
             << "      plugins: [\"static_layer\"";
      if (rolling_window) {
        stream << ", \"obstacle_layer\"";
      }
      stream << ", \"inflation_layer\"]\n"
             << "      static_layer:\n"
             << "        plugin: \"nav2_costmap_2d::StaticLayer\"\n"
             << "        map_subscribe_transient_local: true\n"
             << "        footprint_clearing_enabled: false\n";
      if (rolling_window) {
        stream << "      obstacle_layer:\n"
               << "        plugin: \"nav2_costmap_2d::ObstacleLayer\"\n"
               << "        enabled: true\n"
               << "        footprint_clearing_enabled: true\n"
               << "        observation_sources: \"points\"\n"
               << "        points:\n"
               << "          topic: " << yamlQuote(controls.obstacle_pointcloud_topic) << '\n'
               << "          data_type: \"PointCloud2\"\n"
               << "          clearing: true\n"
               << "          marking: true\n"
               << "          min_obstacle_height: "
               << yamlFloatingPoint(robot.minimum_collision_height) << '\n'
               << "          max_obstacle_height: "
               << yamlFloatingPoint(robot.maximum_collision_height) << '\n'
               << "          obstacle_max_range: 8.0\n"
               << "          raytrace_max_range: 10.0\n"
               << "          observation_persistence: 0.0\n"
               << "          expected_update_rate: 0.0\n";
      }
      stream << "      inflation_layer:\n"
             << "        plugin: \"nav2_costmap_2d::InflationLayer\"\n"
             << "        inflation_radius: "
             << yamlFloatingPoint(assessment.costmap_inflation_radius) << '\n'
             << "        cost_scaling_factor: "
             << yamlFloatingPoint(controls.cost_scaling_factor) << '\n'
             << "        inflate_unknown: false\n"
             << "        inflate_around_unknown: true\n"
             << "      always_send_full_costmap: true\n";
    };
  write_costmap("global_costmap", "map", false);
  write_costmap("local_costmap", controls.odom_frame, true);

  stream << "velocity_smoother:\n"
         << "  ros__parameters:\n"
         << "    max_velocity: ["
         << yamlFloatingPoint(controls.maximum_linear_speed_mps)
         << ", 0.0, " << yamlFloatingPoint(controls.maximum_angular_speed_rps) << "]\n"
         << "    min_velocity: [" << yamlFloatingPoint(reverse_speed)
         << ", 0.0, " << yamlFloatingPoint(-controls.maximum_angular_speed_rps) << "]\n"
         << "# Route Server is optional; FollowWaypoints remains the portable action path.\n"
         << "route_server:\n"
         << "  ros__parameters:\n"
         << "    base_frame: " << yamlQuote(controls.base_frame) << '\n'
         << "    route_frame: \"map\"\n"
         << "    graph_filepath: " << yamlQuote(absolute_graph.string()) << '\n'
         << "    graph_file_loader: \"GeoJsonGraphFileLoader\"\n"
         << "    GeoJsonGraphFileLoader:\n"
         << "      plugin: \"nav2_route::GeoJsonGraphFileLoader\"\n"
         << "    path_density: 0.05\n"
         << "    edge_cost_functions: [\"TimeScorer\", \"CostmapScorer\"]\n"
         << "    TimeScorer:\n"
         << "      plugin: \"nav2_route::TimeScorer\"\n"
         << "      speed_tag: \"abs_speed_limit\"\n"
         << "      max_vel: " << yamlFloatingPoint(controls.maximum_linear_speed_mps) << '\n'
         << "    CostmapScorer:\n"
         << "      plugin: \"nav2_route::CostmapScorer\"\n"
         << "      costmap_topic: \"global_costmap/costmap_raw\"\n"
         << "      invalid_on_collision: true\n"
         << "      invalid_off_map: true\n"
         << "      max_cost: 253.0\n"
         << "    operations: [\"CollisionMonitor\"]\n"
         << "    CollisionMonitor:\n"
         << "      plugin: \"nav2_route::CollisionMonitor\"\n"
         << "      costmap_topic: \"global_costmap/costmap_raw\"\n"
         << "      max_cost: 253.0\n"
         << "      reroute_on_collision: true\n";
}

void saveNav2ClosedCourseReadinessYaml(
  const std::filesystem::path & path,
  const Nav2ClosedCourseAssessment & assessment,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & controls)
{
  ensureParentDirectory(path);
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create Nav2 closed-course readiness: " + path.string());
  }
  stream << std::setprecision(12)
         << "schema_version: 2\n"
         << "target: \"nav2_closed_course_experimental\"\n"
         << "production_ready: false\n"
         << "artifact:\n"
         << "  ready: " << (assessment.closed_course_artifact_ready ? "true" : "false") << '\n'
         << "  static_map_ready: " <<
    (assessment.static_map_artifact_ready ? "true" : "false") << '\n'
         << "  follow_waypoints_ready: " <<
    (assessment.follow_waypoints_artifact_ready ? "true" : "false") << '\n'
         << "  route_server_ready: " <<
    (assessment.route_server_artifact_ready ? "true" : "false") << '\n'
         << "deployment:\n"
         << "  ready: " << (assessment.closed_course_deployment_ready ? "true" : "false") << '\n'
         << "  static_map_ready: " <<
    (assessment.static_map_deployment_ready ? "true" : "false") << '\n'
         << "  follow_waypoints_ready: " <<
    (assessment.follow_waypoints_deployment_ready ? "true" : "false") << '\n'
         << "  route_server_ready: " <<
    (assessment.route_server_deployment_ready ? "true" : "false") << '\n'
         << "compatibility:\n"
         << "  map_server: " << (assessment.map_server_compatible ? "true" : "false") << '\n'
         << "  follow_waypoints_action_goal: " <<
    (assessment.follow_waypoints_compatible ? "true" : "false") << '\n'
         << "  route_server_geojson: " <<
    (assessment.route_server_compatible ? "true" : "false") << '\n'
         << "evidence:\n"
         << "  free_space_mode: " <<
    yamlQuote(config.generator.traversability.free_space_evidence_mode) << '\n'
         << "  direct_free_space_selected: " <<
    (assessment.direct_free_space_evidence_selected ? "true" : "false") << '\n'
         << "  robot_dimensions_source: " <<
    yamlQuote(config.generator.robot.dimensions_source) << '\n'
         << "  robot_dimensions_confidence: " <<
    yamlQuote(config.generator.robot.dimensions_confidence) << '\n'
         << "  robot_dimensions_production_verified: " <<
    (config.generator.robot.dimensions_verified ? "true" : "false") << '\n'
         << "  lidar_extrinsics_source: " <<
    yamlQuote(config.extrinsics.calibration_source) << '\n'
         << "  lidar_extrinsics_confidence: " <<
    yamlQuote(config.extrinsics.calibration_confidence) << '\n'
         << "  lidar_extrinsics_production_verified: " <<
    (config.extrinsics.verified ? "true" : "false") << '\n'
         << "map:\n"
         << "  obstacle_source: \"raw_obstacle_grid\"\n"
         << "  offline_inflation_applied: false\n"
         << "  unknown_promoted_to_free: false\n"
         << "  classification_partition_complete: " <<
    (assessment.classification_partition_complete ? "true" : "false") << '\n'
         << "  obstacle_cells: " << assessment.obstacle_cells << '\n'
         << "  explicit_free_cells: " << assessment.explicit_free_cells << '\n'
         << "  unknown_cells: " << assessment.unknown_cells << '\n'
         << "  unclassified_cells: " << assessment.unclassified_cells << '\n'
         << "  overlapping_classification_cells: " <<
    assessment.overlapping_classification_cells << '\n'
         << "  nav2_costmap_inflation_radius: " <<
    assessment.costmap_inflation_radius << '\n'
         << "route:\n"
         << "  passable_edges: " << assessment.passable_route_edges << '\n'
         << "  invalid_passable_edges: " << assessment.invalid_passable_route_edges << '\n'
         << "  waypoint_routes: " << assessment.waypoint_routes << '\n'
         << "  waypoints: " << assessment.waypoints << '\n'
         << "  obstacle_samples: " << assessment.route_obstacle_samples << '\n'
         << "  unknown_samples: " << assessment.route_unknown_samples << '\n'
         << "  off_map_samples: " << assessment.route_off_map_samples << '\n'
         << "session_controls:\n"
         << "  enabled: " << (controls.enabled ? "true" : "false") << '\n'
         << "  operator_acknowledged_experimental_only: " <<
    (controls.operator_acknowledged_experimental_only ? "true" : "false") << '\n'
         << "  estimated_geometry_acknowledged: " <<
    (controls.estimated_geometry_acknowledged ? "true" : "false") << '\n'
         << "  closed_course_access_controlled: " <<
    (controls.closed_course_access_controlled ? "true" : "false") << '\n'
         << "  free_space_reviewed_for_session: " <<
    (controls.free_space_reviewed_for_session ? "true" : "false") << '\n'
         << "  localization_alignment_checked_for_session: " <<
    (controls.localization_alignment_checked_for_session ? "true" : "false") << '\n'
         << "  emergency_stop_available: " <<
    (controls.emergency_stop_available ? "true" : "false") << '\n'
         << "  obstacle_pointcloud_topic: " <<
    yamlQuote(controls.obstacle_pointcloud_topic) << '\n'
         << "  maximum_linear_speed_mps: " << controls.maximum_linear_speed_mps << '\n'
         << "  maximum_angular_speed_rps: " << controls.maximum_angular_speed_rps << '\n';
  writeStringList(stream, "map_blockers", assessment.map_blockers);
  writeStringList(
    stream, "follow_waypoints_blockers", assessment.follow_waypoints_blockers);
  writeStringList(stream, "route_server_blockers", assessment.route_server_blockers);
  writeStringList(stream, "deployment_blockers", assessment.deployment_blockers);
  writeStringList(stream, "limitations", assessment.limitations);
}

Nav2ClosedCourseAssessment saveNav2ClosedCourseExperimentalBundle(
  const Nav2ClosedCourseArtifacts & artifacts,
  const PipelineResult & pipeline,
  const RouteGraph & closed_course_graph,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & controls,
  const NamedNavigationRoute * named_route,
  const std::vector<AuthoredStopLine> * resolved_stop_lines)
{
  validateArtifactPaths(artifacts);
  Nav2ClosedCourseAssessment assessment = evaluateNav2ClosedCourseExperiment(
    pipeline, closed_course_graph, config, controls);
  const std::vector<AuthoredStopLine> no_stop_lines;
  const std::vector<AuthoredStopLine> & stops = resolved_stop_lines != nullptr ?
    *resolved_stop_lines : no_stop_lines;
  if (named_route == nullptr && !stops.empty()) {
    throw std::invalid_argument(
            "resolved Nav2 stop lines require a selected named Route");
  }

  // Validate and materialize every authored waypoint before writing any part
  // of the bundle. An invalid stop must not leave a partially promoted map.
  std::vector<Nav2WaypointRoute> waypoint_routes;
  if (assessment.follow_waypoints_artifact_ready) {
    // Route Server consumes the complete eligible map below. FollowWaypoints
    // consumes one explicitly selected mission when authoring is present.
    // Building its poses from the full graph first made a branched/loop map
    // either fail the "one chain" assertion or silently conflate map scope
    // with mission scope.
    RouteGraph waypoint_graph = closed_course_graph;
    if (named_route != nullptr) {
      waypoint_graph = selectNamedNavigationRouteGraph(
        closed_course_graph, *named_route);
    }
    waypoint_routes = buildNav2WaypointRoutes(
      waypoint_graph, controls.maximum_waypoint_spacing,
      config.output.nav2_route_max_chord_error);
    if (named_route != nullptr) {
      if (waypoint_routes.size() != 1U) {
        throw std::runtime_error(
                "a selected named Nav2 Route must materialize as exactly one waypoint chain");
      }
      rebuildNamedWaypointRouteWithStops(
        waypoint_routes.front(), closed_course_graph, *named_route, stops,
        controls.maximum_waypoint_spacing,
        config.output.nav2_route_max_chord_error);
      waypoint_routes.front().name = named_route->name;
      waypoint_routes.front().target = named_route->target;
      waypoint_routes.front().source_named_route_id = named_route->id;
    }
    assessment.waypoint_routes = waypoint_routes.size();
    assessment.waypoints = 0U;
    for (const Nav2WaypointRoute & route : waypoint_routes) {
      assessment.waypoints += route.waypoints.size();
    }
  } else if (!stops.empty()) {
    throw std::runtime_error(
            "resolved Nav2 stop lines cannot be inserted into a non-ready waypoint artifact");
  }

  // The experimental map uses the raw obstacle mask. Costmap inflation is the
  // sole footprint/inflation application. Session controls affect deployment
  // readiness, but do not erase a technically valid artifact.
  saveNav2TrinaryPgm(
    artifacts.map_pgm,
    pipeline.grids.obstacle_grid,
    pipeline.grids.observed_free_grid,
    pipeline.grids.unknown_grid,
    !assessment.static_map_artifact_ready);
  saveOccupancyGridYaml(
    artifacts.map_yaml, artifacts.map_pgm.filename(), pipeline.grids.obstacle_grid);

  RouteGraph route_graph;
  route_graph.frame_id = "map";
  if (assessment.route_server_artifact_ready) {
    route_graph = speedLimitedGraph(closed_course_graph, controls.maximum_linear_speed_mps);
  }
  saveNav2RouteGraphGeoJson(
    artifacts.route_graph_geojson,
    route_graph,
    config.output.nav2_route_max_chord_error,
    config.output.nav2_route_max_segment_length,
    named_route);

  saveNav2WaypointRoutesYaml(
    artifacts.waypoint_routes_yaml, "map", waypoint_routes,
    assessment.follow_waypoints_artifact_ready);
  saveNav2ClosedCourseParamsOverlayYaml(
    artifacts.nav2_params_overlay_yaml,
    artifacts.map_yaml,
    artifacts.route_graph_geojson,
    config.generator.robot,
    controls,
    assessment);
  saveNav2ClosedCourseReadinessYaml(
    artifacts.readiness_yaml, assessment, config, controls);
  return assessment;
}

}  // namespace lidar_mobility_map_generator
