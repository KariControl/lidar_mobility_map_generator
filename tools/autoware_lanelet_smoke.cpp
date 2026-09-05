// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/stop_line.hpp>
#include <autoware/map_loader/lanelet2_map_loader_node.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include "lidar_mobility_map_generator/exporters.hpp"

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <rclcpp/time.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

using LaneletId = lanelet::Id;

struct Report
{
  std::size_t points{0U};
  std::size_t lanelets{0U};
  std::size_t map_bin_bytes{0U};
  std::size_t passable_lanelets{0U};
  std::size_t routing_edges{0U};
  std::size_t routing_heads{0U};
  std::size_t routing_tails{0U};
  std::size_t routing_components{0U};
  std::size_t ordered_lanelets{0U};
  std::size_t authored_stop_line_ways{0U};
  std::size_t stop_sign_regulatory_elements{0U};
  std::size_t lanelets_with_stop_sign{0U};
  std::size_t autoware_stop_line_api_matches{0U};
  std::size_t cusp_transitions{0U};
  double minimum_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_speed_mps{0.0};
  double maximum_speed_metadata_error_mps{0.0};
  double maximum_heading_jump_degrees{0.0};
  std::vector<std::string> errors;
};

double strictDouble(const std::string & value)
{
  std::size_t consumed = 0U;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument("not a finite, unitless number: '" + value + "'");
  }
  return result;
}

void addError(Report & report, const std::string & message)
{
  report.errors.push_back(message);
}

void verifyLocalCoordinates(const lanelet::LaneletMapConstPtr & map, Report & report)
{
  constexpr double tolerance = 1.0e-9;
  for (const auto & point : map->pointLayer) {
    if (!point.hasAttribute("local_x") || !point.hasAttribute("local_y")) {
      addError(
        report, "point " + std::to_string(point.id()) +
        " is missing local_x or local_y");
      continue;
    }
    const auto local_x = point.attribute("local_x").asDouble();
    const auto local_y = point.attribute("local_y").asDouble();
    if (!local_x || !local_y) {
      addError(
        report, "point " + std::to_string(point.id()) +
        " has a non-numeric local coordinate");
      continue;
    }
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
      std::abs(point.x() - *local_x) > tolerance ||
      std::abs(point.y() - *local_y) > tolerance)
    {
      std::ostringstream message;
      message << "Autoware Local projection did not preserve local_x/local_y for point " <<
        point.id() << " (loaded=" << point.x() << ',' << point.y() <<
        ", tags=" << *local_x << ',' << *local_y << ')';
      addError(report, message.str());
    }
  }
}

std::vector<lanelet::ConstLanelet> collectLanelets(
  const lanelet::LaneletMapConstPtr & map, Report & report)
{
  std::vector<lanelet::ConstLanelet> lanelets;
  lanelets.reserve(map->laneletLayer.size());
  for (const auto & lanelet : map->laneletLayer) {
    if (lanelet.leftBound().size() < 2U || lanelet.rightBound().size() < 2U) {
      addError(
        report, "lanelet " + std::to_string(lanelet.id()) +
        " has a boundary with fewer than two points");
    }
    if (!lanelet.hasCustomCenterline() || lanelet.centerline().size() < 2U) {
      addError(
        report, "lanelet " + std::to_string(lanelet.id()) +
        " has no usable explicit centerline");
    }
    const double length = lanelet::geometry::length2d(lanelet);
    if (!std::isfinite(length) || length <= 1.0e-6) {
      addError(
        report, "lanelet " + std::to_string(lanelet.id()) +
        " has zero or non-finite centerline length");
    }
    lanelets.emplace_back(lanelet);
  }
  return lanelets;
}

std::optional<std::pair<double, double>> endpointTangent(
  const lanelet::ConstLineString3d & centerline, const bool at_end)
{
  constexpr double minimum_norm = 1.0e-6;
  if (centerline.size() < 2U) {
    return std::nullopt;
  }
  if (at_end) {
    const auto & end = centerline.back();
    for (std::size_t offset = 1U; offset < centerline.size(); ++offset) {
      const auto & begin = centerline[centerline.size() - 1U - offset];
      const double dx = end.x() - begin.x();
      const double dy = end.y() - begin.y();
      if (std::hypot(dx, dy) > minimum_norm) {
        return std::pair<double, double>{dx, dy};
      }
    }
    return std::nullopt;
  }
  const auto & begin = centerline.front();
  for (std::size_t index = 1U; index < centerline.size(); ++index) {
    const auto & end = centerline[index];
    const double dx = end.x() - begin.x();
    const double dy = end.y() - begin.y();
    if (std::hypot(dx, dy) > minimum_norm) {
      return std::pair<double, double>{dx, dy};
    }
  }
  return std::nullopt;
}

double headingJumpDegrees(
  const lanelet::ConstLineString3d & from, const lanelet::ConstLineString3d & to)
{
  constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
  const auto incoming = endpointTangent(from, true);
  const auto outgoing = endpointTangent(to, false);
  if (!incoming || !outgoing) {
    return 0.0;
  }
  const double denominator =
    std::hypot(incoming->first, incoming->second) *
    std::hypot(outgoing->first, outgoing->second);
  const double cosine = std::clamp(
    (incoming->first * outgoing->first + incoming->second * outgoing->second) /
    denominator, -1.0, 1.0);
  return std::acos(cosine) * radians_to_degrees;
}

void verifySpeeds(
  const std::vector<lanelet::ConstLanelet> & lanelets,
  const lanelet::traffic_rules::TrafficRules & traffic_rules, Report & report)
{
  constexpr double metadata_tolerance_mps = 1.0e-6;
  for (const auto & lanelet : lanelets) {
    if (!traffic_rules.canPass(lanelet)) {
      addError(
        report, "German vehicle TrafficRules reject lanelet " +
        std::to_string(lanelet.id()));
      continue;
    }
    ++report.passable_lanelets;

    double raw_speed_kmh = 0.0;
    try {
      raw_speed_kmh = strictDouble(lanelet.attribute("speed_limit").value());
    } catch (const std::exception & error) {
      addError(
        report, "lanelet " + std::to_string(lanelet.id()) +
        " has an Autoware-validator-incompatible speed_limit: " + error.what());
      continue;
    }
    if (!(raw_speed_kmh > 0.0)) {
      addError(
        report, "lanelet " + std::to_string(lanelet.id()) +
        " has a non-positive speed_limit");
      continue;
    }

    const double parsed_speed_mps = traffic_rules.speedLimit(lanelet).speedLimit.value();
    if (!std::isfinite(parsed_speed_mps) || !(parsed_speed_mps > 0.0)) {
      addError(
        report, "TrafficRules produced an invalid speed for lanelet " +
        std::to_string(lanelet.id()));
      continue;
    }
    report.minimum_speed_mps = std::min(report.minimum_speed_mps, parsed_speed_mps);
    report.maximum_speed_mps = std::max(report.maximum_speed_mps, parsed_speed_mps);

    const double unit_error = std::abs(parsed_speed_mps - raw_speed_kmh / 3.6);
    if (unit_error > metadata_tolerance_mps) {
      std::ostringstream message;
      message << "TrafficRules did not interpret unitless speed_limit as km/h for lanelet " <<
        lanelet.id() << " (raw=" << raw_speed_kmh << " km/h, parsed=" <<
        parsed_speed_mps << " m/s)";
      addError(report, message.str());
    }

    if (!lanelet.hasAttribute("generator_speed_limit_mps")) {
      addError(
        report, "lanelet " + std::to_string(lanelet.id()) +
        " is missing generator_speed_limit_mps provenance");
      continue;
    }
    try {
      const double expected_speed_mps =
        strictDouble(lanelet.attribute("generator_speed_limit_mps").value());
      const double error = std::abs(parsed_speed_mps - expected_speed_mps);
      report.maximum_speed_metadata_error_mps =
        std::max(report.maximum_speed_metadata_error_mps, error);
      if (error > metadata_tolerance_mps) {
        std::ostringstream message;
        message << "speed metadata mismatch for lanelet " << lanelet.id() <<
          " (TrafficRules=" << parsed_speed_mps << " m/s, generator=" <<
          expected_speed_mps << " m/s)";
        addError(report, message.str());
      }
    } catch (const std::exception & error) {
      addError(
        report, "lanelet " + std::to_string(lanelet.id()) +
        " has invalid generator_speed_limit_mps: " + error.what());
    }
  }
}

void verifyAuthoredStopLines(
  const lanelet::LaneletMapConstPtr & map,
  const std::vector<lanelet::ConstLanelet> & lanelets,
  Report & report)
{
  std::set<LaneletId> authored_stop_way_ids;
  for (const auto & line : map->lineStringLayer) {
    if (line.hasAttribute("authored_stop_line_id")) {
      authored_stop_way_ids.insert(line.id());
      if (!line.hasAttribute("type") || line.attribute("type").value() != "stop_line") {
        addError(
          report, "authored stop LineString " + std::to_string(line.id()) +
          " is not tagged type=stop_line");
      }
    }
  }
  report.authored_stop_line_ways = authored_stop_way_ids.size();

  std::set<LaneletId> regulatory_ids;
  std::set<LaneletId> referenced_stop_way_ids;
  for (const lanelet::ConstLanelet & lane : lanelets) {
    bool lane_has_stop = false;
    const auto traffic_signs =
      lane.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto & traffic_sign : traffic_signs) {
      if (!traffic_sign->hasAttribute("audit_source") ||
        traffic_sign->attribute("audit_source").value() != "navigation_authoring_gui")
      {
        continue;
      }
      lane_has_stop = true;
      regulatory_ids.insert(traffic_sign->id());
      if (traffic_sign->type() != "stop_sign") {
        addError(
          report, "authored TrafficSign " + std::to_string(traffic_sign->id()) +
          " did not parse as type()=stop_sign");
      }
      const auto ref_lines = traffic_sign->refLines();
      if (ref_lines.empty()) {
        addError(
          report, "authored TrafficSign " + std::to_string(traffic_sign->id()) +
          " has no refLines()");
      }
      for (const auto & line : ref_lines) {
        referenced_stop_way_ids.insert(line.id());
        if (!line.hasAttribute("type") || line.attribute("type").value() != "stop_line") {
          addError(
            report, "TrafficSign ref_line " + std::to_string(line.id()) +
            " is not type=stop_line");
        }
      }
    }
    if (lane_has_stop) {
      ++report.lanelets_with_stop_sign;
      const auto autoware_stop =
        autoware::experimental::lanelet2_utils::get_stop_lines_from_stop_sign(lane);
      if (!autoware_stop) {
        addError(
          report, "Autoware 1.9 stop-line utility did not return the authored stop for lanelet " +
          std::to_string(lane.id()));
      } else {
        ++report.autoware_stop_line_api_matches;
      }
    }
  }
  report.stop_sign_regulatory_elements = regulatory_ids.size();
  if (report.authored_stop_line_ways != referenced_stop_way_ids.size()) {
    addError(
      report, "authored stop LineString/ref_line count differs (ways=" +
      std::to_string(report.authored_stop_line_ways) + ", refs=" +
      std::to_string(referenced_stop_way_ids.size()) + ")");
  }
  for (const LaneletId way_id : authored_stop_way_ids) {
    if (referenced_stop_way_ids.count(way_id) == 0U) {
      addError(
        report, "authored stop LineString " + std::to_string(way_id) +
        " is not referenced by a parsed TrafficSign");
    }
  }
}

void verifyTransition(
  const lanelet::ConstLanelet & from, const lanelet::ConstLanelet & to,
  const lanelet::traffic_rules::TrafficRules & traffic_rules, Report & report)
{
  const std::string transition =
    std::to_string(from.id()) + " -> " + std::to_string(to.id());
  if (!lanelet::geometry::follows(from, to)) {
    addError(report, "Lanelet2 geometry::follows is false for " + transition);
  }
  if (!traffic_rules.canPass(from, to)) {
    addError(report, "TrafficRules::canPass is false for " + transition);
  }
  if (from.leftBound().back().id() != to.leftBound().front().id() ||
    from.rightBound().back().id() != to.rightBound().front().id())
  {
    addError(report, "left/right boundary endpoint IDs are not shared for " + transition);
  }
  if (!from.hasCustomCenterline() || !to.hasCustomCenterline() ||
    from.centerline().back().id() != to.centerline().front().id())
  {
    addError(report, "explicit centerline endpoint IDs are not shared for " + transition);
  }
  const double heading_jump = headingJumpDegrees(from.centerline(), to.centerline());
  report.maximum_heading_jump_degrees =
    std::max(report.maximum_heading_jump_degrees, heading_jump);
  if (heading_jump >= 150.0) {
    ++report.cusp_transitions;
    std::ostringstream message;
    message << "forward-only replay chain contains a " << heading_jump <<
      " degree cusp at " << transition <<
      "; model the stop/reversal as separate route phases";
    addError(report, message.str());
  }
}

std::string laneletIds(
  const std::vector<LaneletId> & ids,
  const std::map<LaneletId, lanelet::ConstLanelet> & by_id)
{
  std::ostringstream stream;
  stream << '[';
  const std::size_t limit = std::min<std::size_t>(ids.size(), 12U);
  for (std::size_t index = 0U; index < limit; ++index) {
    if (index != 0U) {
      stream << ',';
    }
    stream << ids[index];
    const auto & lanelet = by_id.at(ids[index]);
    if (lanelet.hasAttribute("route_edge_id")) {
      stream << "(route_edge=" << lanelet.attribute("route_edge_id").value() << ')';
    }
  }
  if (ids.size() > limit) {
    stream << ",...";
  }
  stream << ']';
  return stream.str();
}

void verifyOpenRoutingChain(
  const std::vector<lanelet::ConstLanelet> & lanelets,
  const lanelet::routing::RoutingGraph & routing_graph,
  const lanelet::traffic_rules::TrafficRules & traffic_rules, Report & report)
{
  std::map<LaneletId, lanelet::ConstLanelet> by_id;
  for (const auto & lanelet : lanelets) {
    by_id.emplace(lanelet.id(), lanelet);
  }

  std::map<LaneletId, std::set<LaneletId>> successors;
  std::map<LaneletId, std::size_t> indegree;
  for (const auto & entry : by_id) {
    successors[entry.first] = {};
    indegree[entry.first] = 0U;
  }
  for (const auto & entry : by_id) {
    for (const auto & following : routing_graph.following(entry.second, false)) {
      if (following.inverted()) {
        addError(
          report, "routing graph returned inverted lanelet " +
          std::to_string(following.id()) + " after " + std::to_string(entry.first));
        continue;
      }
      if (by_id.count(following.id()) == 0U) {
        addError(
          report, "routing graph references a lanelet outside the loaded map: " +
          std::to_string(following.id()));
        continue;
      }
      if (successors[entry.first].insert(following.id()).second) {
        ++indegree[following.id()];
        ++report.routing_edges;
      }
    }
  }

  std::vector<LaneletId> heads;
  std::vector<LaneletId> tails;
  for (const auto & entry : by_id) {
    const std::size_t outdegree = successors.at(entry.first).size();
    if (indegree.at(entry.first) == 0U) {
      heads.push_back(entry.first);
    }
    if (outdegree == 0U) {
      tails.push_back(entry.first);
    }
    if (indegree.at(entry.first) > 1U || outdegree > 1U) {
      std::ostringstream message;
      message << "lanelet " << entry.first << " branches in the routing graph (in=" <<
        indegree.at(entry.first) << ", out=" << outdegree << ')';
      addError(report, message.str());
    }
  }
  report.routing_heads = heads.size();
  report.routing_tails = tails.size();

  std::map<LaneletId, std::set<LaneletId>> undirected;
  for (const auto & entry : successors) {
    for (const LaneletId next : entry.second) {
      undirected[entry.first].insert(next);
      undirected[next].insert(entry.first);
    }
  }
  std::set<LaneletId> component_visited;
  for (const auto & entry : by_id) {
    if (component_visited.count(entry.first) != 0U) {
      continue;
    }
    ++report.routing_components;
    std::queue<LaneletId> pending;
    pending.push(entry.first);
    component_visited.insert(entry.first);
    while (!pending.empty()) {
      const LaneletId id = pending.front();
      pending.pop();
      for (const LaneletId neighbour : undirected[id]) {
        if (component_visited.insert(neighbour).second) {
          pending.push(neighbour);
        }
      }
    }
  }
  if (report.routing_components != 1U) {
    addError(
      report, "routing graph has " + std::to_string(report.routing_components) +
      " weakly connected components instead of one");
  }

  const std::size_t expected_edges = lanelets.empty() ? 0U : lanelets.size() - 1U;
  if (heads.size() != 1U || tails.size() != 1U || report.routing_edges != expected_edges) {
    std::ostringstream message;
    if (heads.empty() && tails.empty() && report.routing_edges == lanelets.size()) {
      message << "routing graph is a closed cycle; the temporal replay seam must retain "
        "distinct first/last endpoint IDs";
    } else {
      message << "expected one open sequential routing chain, got heads=" <<
        laneletIds(heads, by_id) << ", tails=" << laneletIds(tails, by_id) <<
        ", routing_edges=" << report.routing_edges << " (expected " <<
        expected_edges << ')';
    }
    addError(report, message.str());
    return;
  }

  std::vector<LaneletId> order;
  std::set<LaneletId> visited;
  LaneletId current = heads.front();
  while (visited.insert(current).second) {
    order.push_back(current);
    const auto & next = successors.at(current);
    if (next.empty()) {
      break;
    }
    if (next.size() != 1U) {
      break;
    }
    const LaneletId next_id = *next.begin();
    verifyTransition(by_id.at(current), by_id.at(next_id), traffic_rules, report);
    current = next_id;
  }
  report.ordered_lanelets = order.size();
  if (order.size() != lanelets.size() || current != tails.front()) {
    addError(
      report, "routing traversal does not visit every lanelet exactly once in temporal order");
    return;
  }

  const auto shortest_path = routing_graph.shortestPath(
    by_id.at(heads.front()), by_id.at(tails.front()), 0U, false);
  if (!shortest_path || shortest_path->size() != order.size()) {
    addError(report, "RoutingGraph::shortestPath cannot reproduce the complete replay chain");
    return;
  }
  for (std::size_t index = 0U; index < order.size(); ++index) {
    if ((*shortest_path)[index].id() != order[index] || (*shortest_path)[index].inverted()) {
      addError(report, "shortest path order differs from the unique replay chain");
      break;
    }
  }
}

void printReport(const std::filesystem::path & osm_path, const Report & report)
{
  std::cout << std::setprecision(12)
            << "osm=" << osm_path.string() << '\n'
            << "projector_type=Local\n"
            << "points=" << report.points << '\n'
            << "lanelets=" << report.lanelets << '\n'
            << "map_bin_bytes=" << report.map_bin_bytes << '\n'
            << "passable_lanelets=" << report.passable_lanelets << '\n'
            << "routing_edges=" << report.routing_edges << '\n'
            << "routing_heads=" << report.routing_heads << '\n'
            << "routing_tails=" << report.routing_tails << '\n'
            << "routing_components=" << report.routing_components << '\n'
            << "ordered_lanelets=" << report.ordered_lanelets << '\n'
            << "authored_stop_line_ways=" << report.authored_stop_line_ways << '\n'
            << "stop_sign_regulatory_elements=" <<
    report.stop_sign_regulatory_elements << '\n'
            << "lanelets_with_stop_sign=" << report.lanelets_with_stop_sign << '\n'
            << "autoware_stop_line_api_matches=" <<
    report.autoware_stop_line_api_matches << '\n'
            << "cusp_transitions=" << report.cusp_transitions << '\n'
            << "maximum_heading_jump_degrees=" <<
    report.maximum_heading_jump_degrees << '\n'
            << "minimum_speed_mps=" <<
    (std::isfinite(report.minimum_speed_mps) ? report.minimum_speed_mps : 0.0) << '\n'
            << "maximum_speed_mps=" << report.maximum_speed_mps << '\n'
            << "maximum_speed_metadata_error_mps=" <<
    report.maximum_speed_metadata_error_mps << '\n';
  if (report.errors.empty()) {
    std::cout << "AUTOWARE_LANELET_SMOKE=PASS\n";
  } else {
    std::cout << "AUTOWARE_LANELET_SMOKE=FAIL\n";
    for (const auto & error : report.errors) {
      std::cerr << "FAIL: " << error << '\n';
    }
  }
}

std::filesystem::path generateAuthoredStopFixture()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {5.0, 0.0, 0.0}, lmmg::RouteNodeType::kNormal},
    {3U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  const auto make_edge = [](const std::uint64_t id, const std::uint64_t from,
      const std::uint64_t to, const double start_x, const double end_x) {
      lmmg::RouteEdge edge;
      edge.id = id;
      edge.from = from;
      edge.to = to;
      edge.centerline = {{start_x, 0.0, 0.0}, {end_x, 0.0, 0.0}};
      edge.left_boundary = {{start_x, 1.0, 0.0}, {end_x, 1.0, 0.0}};
      edge.right_boundary = {{start_x, -1.0, 0.0}, {end_x, -1.0, 0.0}};
      edge.length = end_x - start_x;
      edge.minimum_safe_width = 2.0;
      edge.recommended_speed_mps = 1.0;
      edge.confidence = 1.0;
      edge.corridor_geometry_valid = true;
      edge.passable = true;
      return edge;
    };
  graph.edges = {
    make_edge(10U, 1U, 2U, 0.0, 5.0),
    make_edge(11U, 2U, 3U, 5.0, 10.0)};

  lmmg::NamedNavigationRoute route;
  route.id = 42U;
  route.name = "Autoware 1.9 stop fixture";
  route.target = lmmg::NavigationAuthoringTarget::kAutoware;
  route.start_node_id = 1U;
  route.end_node_id = 3U;
  route.ordered_edge_ids = {10U, 11U};
  lmmg::AuthoredStopLine stop;
  stop.id = 77U;
  stop.name = "virtual authored stop";
  stop.edge_id = 10U;
  stop.s = 4.0;
  stop.width_m = 2.5;
  stop.anchor = {4.0, 0.0, 0.0};
  stop.target = lmmg::NavigationAuthoringTarget::kAutoware;
  lmmg::Lanelet2AuthoringOptions authoring;
  authoring.named_route = &route;
  authoring.resolved_stop_lines = {stop};
  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 2.0;
  options.estimated_front_extent = 3.0;
  options.estimated_rear_extent = 1.0;
  options.estimated_minimum_turning_radius = 5.1;
  options.lateral_clearance_margin = 0.25;
  options.experimental_ready = true;
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_autoware_190_authored_stop_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm");
  (void)lmmg::saveClosedCourseExperimentalLanelet2Osm(
    path, graph, lmmg::Lanelet2Config{}, options, authoring);
  return path;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr <<
      "Usage: autoware_lanelet_smoke <lanelet2_map.osm>|--self-test-authored-stop\n";
    return 2;
  }

  const bool self_test_authored_stop =
    std::string{argv[1]} == "--self-test-authored-stop";
  const std::filesystem::path osm_path = self_test_authored_stop ?
    generateAuthoredStopFixture() : std::filesystem::path{argv[1]};
  if (!std::filesystem::is_regular_file(osm_path)) {
    std::cerr << "OSM file does not exist: " << osm_path << '\n';
    return 2;
  }

  Report report;
  try {
    autoware_map_msgs::msg::MapProjectorInfo projector_info;
    projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL;

    const auto loaded_map =
      autoware::map_loader::Lanelet2MapLoaderNode::load_map(osm_path.string(), projector_info);
    if (!loaded_map) {
      throw std::runtime_error("Autoware Local map loader returned nullptr");
    }
    report.points = loaded_map->pointLayer.size();
    report.lanelets = loaded_map->laneletLayer.size();
    if (report.points == 0U || report.lanelets == 0U) {
      addError(report, "Autoware Local loader produced an empty point or lanelet layer");
    }
    verifyLocalCoordinates(loaded_map, report);

    const auto map_bin = autoware::map_loader::Lanelet2MapLoaderNode::create_map_bin_msg(
      loaded_map, osm_path.string(), rclcpp::Time(0, 0, RCL_ROS_TIME));
    report.map_bin_bytes = map_bin.data.size();
    if (map_bin.data.empty()) {
      addError(report, "Autoware LaneletMapBin serialization produced an empty message");
    }
    const auto map =
      autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_bin);
    if (!map || map->pointLayer.size() != report.points ||
      map->laneletLayer.size() != report.lanelets)
    {
      throw std::runtime_error("LaneletMapBin round trip changed map primitive counts");
    }
    verifyLocalCoordinates(map, report);

    const auto lanelets = collectLanelets(map, report);
    const auto graph_and_rules =
      autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(map);
    const auto & routing_graph = graph_and_rules.first;
    const auto & traffic_rules = graph_and_rules.second;
    if (!routing_graph || !traffic_rules) {
      throw std::runtime_error("Autoware failed to instantiate RoutingGraph or TrafficRules");
    }
    for (const auto & error : routing_graph->checkValidity(false)) {
      addError(report, "RoutingGraph validity error: " + error);
    }
    verifySpeeds(lanelets, *traffic_rules, report);
    verifyAuthoredStopLines(map, lanelets, report);
    if (self_test_authored_stop &&
      (report.authored_stop_line_ways != 1U ||
      report.stop_sign_regulatory_elements != 1U ||
      report.lanelets_with_stop_sign != 1U ||
      report.autoware_stop_line_api_matches != 1U))
    {
      addError(
        report,
        "self-test expected exactly one authored stop way, TrafficSign, affected Lanelet, "
        "and Autoware stop-line API match");
    }
    verifyOpenRoutingChain(lanelets, *routing_graph, *traffic_rules, report);
  } catch (const std::exception & error) {
    addError(report, error.what());
  }

  printReport(osm_path, report);
  if (self_test_authored_stop) {
    std::error_code cleanup_error;
    std::filesystem::remove(osm_path, cleanup_error);
  }
  return report.errors.empty() ? 0 : 1;
}
