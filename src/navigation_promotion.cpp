#include "lidar_mobility_map_generator/navigation_promotion.hpp"

#include "lidar_mobility_map_generator/semantic_route_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

struct MaterializedRoute
{
  RouteGraph graph;
  NamedNavigationRoute route;
};

double planarPolylineLength(const std::vector<Vec3> & points)
{
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result += distance2d(points[index - 1U], points[index]);
  }
  return result;
}

Vec2 directedEndpointTangent(
  const std::vector<Vec3> & points, const bool at_end,
  const double minimum_span)
{
  if (points.size() < 2U) {
    return {};
  }
  if (at_end) {
    std::size_t previous = points.size() - 1U;
    double span = 0.0;
    while (previous > 0U && span < minimum_span) {
      span += distance2d(points[previous], points[previous - 1U]);
      --previous;
    }
    return normalized(Vec2{
      points.back().x - points[previous].x,
      points.back().y - points[previous].y});
  }
  std::size_t next = 0U;
  double span = 0.0;
  while (next + 1U < points.size() && span < minimum_span) {
    span += distance2d(points[next], points[next + 1U]);
    ++next;
  }
  return normalized(Vec2{
    points[next].x - points.front().x,
    points[next].y - points.front().y});
}

std::string joinedIds(const std::vector<std::uint64_t> & ids)
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

const RouteEdge * findEdge(const RouteGraph & graph, const std::uint64_t id)
{
  const auto found = std::find_if(
    graph.edges.begin(), graph.edges.end(),
    [id](const RouteEdge & edge) {return edge.id == id;});
  return found == graph.edges.end() ? nullptr : &*found;
}

const RouteNode * findUniqueNode(const RouteGraph & graph, const std::uint64_t id)
{
  const RouteNode * result = nullptr;
  for (const RouteNode & node : graph.nodes) {
    if (node.id != id) {
      continue;
    }
    if (result != nullptr) {
      throw std::runtime_error("semantic_topology_duplicate_node_id:" + std::to_string(id));
    }
    result = &node;
  }
  return result;
}

bool hasNoEntryEvidence(const RouteEdge & edge)
{
  return std::find(
    edge.validation_errors.begin(), edge.validation_errors.end(),
    "semantic_no_entry") != edge.validation_errors.end();
}

ClosedCourseAutowareTerminalSupport deriveTerminalSupport(
  const RouteGraph & named_graph,
  const NamedNavigationRoute & named_route,
  const RouteGraph & semantic_topology)
{
  constexpr double endpoint_tolerance = 1.0e-3;
  constexpr double tangent_span = 0.50;
  constexpr double maximum_heading_jump_deg = 30.0;
  if (named_graph.frame_id != semantic_topology.frame_id) {
    throw std::runtime_error("semantic_topology_frame_mismatch");
  }
  if (named_route.ordered_edge_ids.empty()) {
    throw std::runtime_error("named_route_empty");
  }

  std::set<std::uint64_t> topology_edge_ids;
  for (const RouteEdge & edge : semantic_topology.edges) {
    if (!topology_edge_ids.insert(edge.id).second) {
      throw std::runtime_error(
              "semantic_topology_duplicate_edge_id:" + std::to_string(edge.id));
    }
  }

  std::set<std::uint64_t> named_edge_ids;
  std::set<std::uint64_t> named_node_ids;
  const RouteEdge * terminal = nullptr;
  for (const std::uint64_t edge_id : named_route.ordered_edge_ids) {
    if (!named_edge_ids.insert(edge_id).second) {
      throw std::runtime_error("named_route_edge_reused:" + std::to_string(edge_id));
    }
    const RouteEdge * edge = findEdge(named_graph, edge_id);
    if (edge == nullptr) {
      throw std::runtime_error("named_route_edge_missing:" + std::to_string(edge_id));
    }
    const double length = planarPolylineLength(edge->centerline);
    if (!(length > 1.0e-9) || !std::isfinite(length)) {
      throw std::runtime_error("named_route_edge_geometry_invalid:" + std::to_string(edge_id));
    }
    named_node_ids.insert(edge->from);
    named_node_ids.insert(edge->to);
    terminal = edge;
  }
  if (terminal == nullptr || terminal->id != named_route.ordered_edge_ids.back()) {
    throw std::runtime_error("named_route_terminal_edge_missing");
  }

  std::vector<const RouteEdge *> outgoing;
  std::vector<std::uint64_t> no_entry_ids;
  for (const RouteEdge & edge : semantic_topology.edges) {
    if (edge.from != terminal->to) {
      continue;
    }
    if (!edge.passable || hasNoEntryEvidence(edge)) {
      no_entry_ids.push_back(edge.id);
    } else {
      outgoing.push_back(&edge);
    }
  }
  if (!no_entry_ids.empty()) {
    throw std::runtime_error(
            "terminal_successor_no_entry_or_impassable:" + joinedIds(no_entry_ids));
  }
  if (outgoing.empty()) {
    throw std::runtime_error(
            "terminal_successor_missing:" + std::to_string(terminal->to));
  }
  if (outgoing.size() != 1U) {
    std::vector<std::uint64_t> ids;
    ids.reserve(outgoing.size());
    for (const RouteEdge * edge : outgoing) {
      ids.push_back(edge->id);
    }
    throw std::runtime_error("terminal_successor_branch_ambiguous:" + joinedIds(ids));
  }

  const RouteEdge & successor = *outgoing.front();
  bool reverses_named_edge = successor.reverse_of &&
    named_edge_ids.count(*successor.reverse_of) != 0U;
  for (const std::uint64_t edge_id : named_edge_ids) {
    const RouteEdge * named_edge = findEdge(named_graph, edge_id);
    reverses_named_edge = reverses_named_edge ||
      (named_edge != nullptr && named_edge->reverse_of &&
      *named_edge->reverse_of == successor.id);
  }
  if (named_edge_ids.count(successor.id) != 0U || reverses_named_edge) {
    throw std::runtime_error(
            "terminal_successor_reuses_named_edge:" + std::to_string(successor.id));
  }
  if (successor.from == successor.to || named_node_ids.count(successor.to) != 0U) {
    throw std::runtime_error("terminal_successor_cycle:" + std::to_string(successor.id));
  }
  if (terminal->centerline.size() < 2U || successor.centerline.size() < 2U ||
    !finite(terminal->centerline.back()) || !finite(successor.centerline.front()) ||
    !finite(successor.centerline.back()))
  {
    throw std::runtime_error(
            "terminal_successor_geometry_invalid:" + std::to_string(successor.id));
  }
  if (distance3d(terminal->centerline.back(), successor.centerline.front()) >
    endpoint_tolerance)
  {
    throw std::runtime_error(
            "terminal_successor_endpoint_mismatch:" + std::to_string(successor.id));
  }
  const Vec2 terminal_tangent = directedEndpointTangent(
    terminal->centerline, true, tangent_span);
  const Vec2 successor_tangent = directedEndpointTangent(
    successor.centerline, false, tangent_span);
  if (norm(terminal_tangent) <= 1.0e-12 || norm(successor_tangent) <= 1.0e-12) {
    throw std::runtime_error(
            "terminal_successor_tangent_invalid:" + std::to_string(successor.id));
  }
  const double heading_jump_deg = std::abs(normalizeAngle(
      std::atan2(successor_tangent.y, successor_tangent.x) -
      std::atan2(terminal_tangent.y, terminal_tangent.x))) * 180.0 / kPi;
  if (heading_jump_deg > maximum_heading_jump_deg + 1.0e-9) {
    throw std::runtime_error(
            "terminal_successor_heading_discontinuity:" +
            std::to_string(successor.id) + ":" + std::to_string(heading_jump_deg));
  }
  const RouteNode * successor_end = findUniqueNode(semantic_topology, successor.to);
  if (successor_end == nullptr || !finite(successor_end->position)) {
    throw std::runtime_error(
            "terminal_successor_node_missing:" + std::to_string(successor.to));
  }
  if (distance3d(successor.centerline.back(), successor_end->position) > endpoint_tolerance) {
    throw std::runtime_error(
            "terminal_successor_end_node_mismatch:" + std::to_string(successor.id));
  }
  const double support_length = planarPolylineLength(successor.centerline);
  const double terminal_named_length = planarPolylineLength(terminal->centerline);
  if (!(support_length > 1.0e-9) || !std::isfinite(support_length) ||
    !(terminal_named_length > 1.0e-9) || !std::isfinite(terminal_named_length))
  {
    throw std::runtime_error(
            "terminal_successor_geometry_invalid:" + std::to_string(successor.id));
  }

  ClosedCourseAutowareTerminalSupport result;
  result.named_terminal_edge_id = terminal->id;
  result.support_edge_ids = {successor.id};
  result.terminal_support_length_m = support_length;
  result.named_route_source_length_m = terminal_named_length;
  result.successor_edge = successor;
  result.successor_end_node = *successor_end;
  return result;
}

// Planner maps are independent from the mission selected inside them.
// Materialize semantics over the complete eligible map and remap only the
// NamedNavigationRoute metadata onto its derived Edge IDs. Selecting the
// RouteGraph before this step used to erase every unselected Lanelet/Route
// Server Edge and made a short mission look like the whole generated map.
MaterializedRoute materializeMapWithRoute(
  const RouteGraph & source,
  const NamedNavigationRoute & authored_route,
  const SemanticMap * semantic_map)
{
  if (semantic_map == nullptr) {
    throw std::runtime_error("semantic_layer_not_ready");
  }
  const RouteGraph selected_source =
    selectNamedNavigationRouteGraph(source, authored_route);
  const SemanticGraphFilterResult filtered =
    filterSemanticMapForGraph(*semantic_map, source);
  const SemanticRouteGraphResult materialized =
    materializeSemanticRouteGraph(source, filtered.map);
  if (materialized.graph.edges.empty()) {
    throw std::runtime_error("eligible_map_empty_after_semantic_materialization");
  }
  return {
    materialized.graph,
    remapNamedNavigationRouteAfterSemantics(
      selected_source, authored_route, materialized)};
}

void blockTarget(
  NavigationAuthoringValidationResult & validation,
  const NavigationAuthoringTarget target,
  const std::string & reason)
{
  const std::optional<std::uint64_t> selected =
    target == NavigationAuthoringTarget::kAutoware ?
    validation.selected_autoware_route_id : validation.selected_nav2_route_id;
  if (selected) {
    const auto status = std::find_if(
      validation.route_statuses.begin(), validation.route_statuses.end(),
      [&](const NamedNavigationRouteStatus & value) {return value.id == *selected;});
    if (status != validation.route_statuses.end()) {
      status->valid = false;
      status->promotion_eligible = false;
      status->errors.push_back(reason);
    }
  }
  validation.errors.push_back(reason);
  if (target == NavigationAuthoringTarget::kAutoware) {
    validation.selected_autoware_route_id.reset();
  } else {
    validation.selected_nav2_route_id.reset();
  }
}

std::vector<AuthoredStopLine> resolveAllStops(
  const NavigationAuthoringValidationResult & validation,
  const RouteGraph & graph,
  const NavigationAuthoringTarget target,
  const NamedNavigationRoute & materialized_route,
  const NamedNavigationRoute & source_route)
{
  // Stop-line IDs and arc lengths are authored on the source graph. Resolve
  // their persisted anchors on the materialized mission chain, whose Edge
  // objects are also owned by the complete map. Restricting geometric
  // projection to this chain avoids ambiguity from coincident reverse or
  // revisited map edges, while passing nullptr below avoids incorrectly
  // pre-filtering a source Edge ID that a semantic split has replaced.
  const RouteGraph mission_graph =
    selectNamedNavigationRouteGraph(graph, materialized_route);
  std::vector<AuthoredStopLine> resolved =
    resolveStopLinesForGraph(validation, mission_graph, target, nullptr);
  const std::size_t expected = expectedStopLineCountForRoute(
    validation, target, source_route);
  if (resolved.size() != expected) {
    throw std::runtime_error(
            "authored_stop_line_resolution_incomplete:" +
            std::to_string(resolved.size()) + "/" + std::to_string(expected));
  }
  return resolved;
}

}  // namespace

NavigationPromotionResult materializeNavigationPromotions(
  const NavigationAuthoringValidationResult & production_validation,
  const NavigationAuthoringValidationResult & closed_course_validation,
  const RouteGraph & production_graph,
  const RouteGraph & closed_course_graph,
  const RouteGraph & empty_graph,
  const SemanticMap * semantic_map,
  const bool nav2_requested,
  const bool autoware_requested,
  const RouteGraph * closed_course_semantic_topology)
{
  NavigationPromotionResult result;
  result.production_validation = production_validation;
  result.closed_course_validation = closed_course_validation;
  result.production_nav2_graph = production_graph;
  result.production_autoware_graph = production_graph;
  result.closed_course_nav2_graph.frame_id = closed_course_graph.frame_id;
  result.closed_course_autoware_graph.frame_id = closed_course_graph.frame_id;

  const NamedNavigationRoute * production_nav2_route = selectedNamedNavigationRoute(
    result.production_validation, NavigationAuthoringTarget::kNav2);
  const NamedNavigationRoute * production_autoware_route = selectedNamedNavigationRoute(
    result.production_validation, NavigationAuthoringTarget::kAutoware);
  const NamedNavigationRoute * closed_course_nav2_route = selectedNamedNavigationRoute(
    result.closed_course_validation, NavigationAuthoringTarget::kNav2);
  const NamedNavigationRoute * closed_course_autoware_route = selectedNamedNavigationRoute(
    result.closed_course_validation, NavigationAuthoringTarget::kAutoware);

  if (nav2_requested) {
    result.production_nav2_graph = empty_graph;
    if (production_nav2_route != nullptr) {
      try {
        MaterializedRoute materialized = materializeMapWithRoute(
          production_graph,
          *production_nav2_route, semantic_map);
        result.production_nav2_graph = std::move(materialized.graph);
        result.production_nav2_route = std::move(materialized.route);
      } catch (const std::exception & exception) {
        blockTarget(
          result.production_validation, NavigationAuthoringTarget::kNav2,
          std::string{"nav2_production_materialization_failed:"} + exception.what());
        production_nav2_route = nullptr;
      }
    }
    if (closed_course_nav2_route != nullptr) {
      try {
        MaterializedRoute materialized = materializeMapWithRoute(
          closed_course_graph,
          *closed_course_nav2_route, semantic_map);
        result.closed_course_nav2_graph = std::move(materialized.graph);
        result.closed_course_nav2_route = std::move(materialized.route);
      } catch (const std::exception & exception) {
        blockTarget(
          result.closed_course_validation, NavigationAuthoringTarget::kNav2,
          std::string{"nav2_closed_course_materialization_failed:"} + exception.what());
        closed_course_nav2_route = nullptr;
      }
    }
  }

  if (autoware_requested) {
    result.production_autoware_graph = empty_graph;
    if (production_autoware_route != nullptr) {
      try {
        MaterializedRoute materialized = materializeMapWithRoute(
          production_graph,
          *production_autoware_route, semantic_map);
        result.production_autoware_graph = std::move(materialized.graph);
        result.production_autoware_route = std::move(materialized.route);
      } catch (const std::exception & exception) {
        blockTarget(
          result.production_validation, NavigationAuthoringTarget::kAutoware,
          std::string{"autoware_production_materialization_failed:"} + exception.what());
        production_autoware_route = nullptr;
      }
    }
    if (closed_course_autoware_route != nullptr) {
      try {
        MaterializedRoute materialized = materializeMapWithRoute(
          closed_course_graph,
          *closed_course_autoware_route, semantic_map);
        result.closed_course_autoware_graph = std::move(materialized.graph);
        result.closed_course_autoware_route = std::move(materialized.route);
      } catch (const std::exception & exception) {
        blockTarget(
          result.closed_course_validation, NavigationAuthoringTarget::kAutoware,
          std::string{"autoware_closed_course_materialization_failed:"} + exception.what());
        closed_course_autoware_route = nullptr;
      }
      if (closed_course_autoware_route != nullptr &&
        closed_course_semantic_topology != nullptr)
      {
        try {
          result.closed_course_autoware_terminal_support = deriveTerminalSupport(
            result.closed_course_autoware_graph,
            *result.closed_course_autoware_route,
            *closed_course_semantic_topology);
        } catch (const std::exception &) {
          // Terminal support was needed only when the map itself was reduced
          // to the Named Route.  The full map already retains all available
          // successors, so inability to derive one audited continuation must
          // not delete an otherwise valid mission or any unrelated Lanelets.
          result.closed_course_autoware_terminal_support.reset();
        }
      }
    }
  }

  if (production_nav2_route != nullptr && result.production_nav2_route) {
    try {
      result.production_nav2_stop_lines = resolveAllStops(
        result.production_validation, result.production_nav2_graph,
        NavigationAuthoringTarget::kNav2, *result.production_nav2_route,
        *production_nav2_route);
      if (!result.production_nav2_stop_lines.empty()) {
        throw std::runtime_error("authored_virtual_stop_line_not_physically_verified");
      }
    } catch (const std::exception & exception) {
      blockTarget(
        result.production_validation, NavigationAuthoringTarget::kNav2,
        std::string{"nav2_production_stop_line_validation_failed:"} + exception.what());
      result.production_nav2_route.reset();
      result.production_nav2_graph = empty_graph;
      result.production_nav2_stop_lines.clear();
    }
  }
  if (closed_course_nav2_route != nullptr && result.closed_course_nav2_route) {
    try {
      result.closed_course_nav2_stop_lines = resolveAllStops(
        result.closed_course_validation, result.closed_course_nav2_graph,
        NavigationAuthoringTarget::kNav2, *result.closed_course_nav2_route,
        *closed_course_nav2_route);
    } catch (const std::exception & exception) {
      blockTarget(
        result.closed_course_validation, NavigationAuthoringTarget::kNav2,
        std::string{"nav2_closed_course_stop_line_validation_failed:"} + exception.what());
      result.closed_course_nav2_route.reset();
      result.closed_course_nav2_graph = empty_graph;
      result.closed_course_nav2_stop_lines.clear();
    }
  }
  if (production_autoware_route != nullptr && result.production_autoware_route) {
    try {
      result.production_autoware_stop_lines = resolveAllStops(
        result.production_validation, result.production_autoware_graph,
        NavigationAuthoringTarget::kAutoware, *result.production_autoware_route,
        *production_autoware_route);
      if (!result.production_autoware_stop_lines.empty()) {
        throw std::runtime_error("authored_virtual_stop_line_not_physically_verified");
      }
    } catch (const std::exception & exception) {
      blockTarget(
        result.production_validation, NavigationAuthoringTarget::kAutoware,
        std::string{"autoware_production_stop_line_validation_failed:"} + exception.what());
      result.production_autoware_route.reset();
      result.production_autoware_graph = empty_graph;
      result.production_autoware_stop_lines.clear();
    }
  }
  if (closed_course_autoware_route != nullptr && result.closed_course_autoware_route) {
    try {
      result.closed_course_autoware_stop_lines = resolveAllStops(
        result.closed_course_validation, result.closed_course_autoware_graph,
        NavigationAuthoringTarget::kAutoware, *result.closed_course_autoware_route,
        *closed_course_autoware_route);
    } catch (const std::exception & exception) {
      blockTarget(
        result.closed_course_validation, NavigationAuthoringTarget::kAutoware,
        std::string{"autoware_closed_course_stop_line_validation_failed:"} + exception.what());
      result.closed_course_autoware_route.reset();
      result.closed_course_autoware_graph = empty_graph;
      result.closed_course_autoware_stop_lines.clear();
      result.closed_course_autoware_terminal_support.reset();
    }
  }

  return result;
}

}  // namespace lidar_mobility_map_generator
