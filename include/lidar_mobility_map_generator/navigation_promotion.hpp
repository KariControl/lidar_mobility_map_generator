#pragma once

#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/semantic_map.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

// Audited, observed continuation for Autoware's closed-course terminal Lanelet.
// It supplies measured downstream geometry without claiming that longitudinal
// support alone satisfies the planner's vehicle-footprint/drivable-area gates.
// The support Edge is not promoted into the named Route and is not exported as
// a separate Lanelet.  Instead, the exporter composites its centerline into
// the final named Lanelet and records every field below as OSM provenance.
// `support_edge_ids` currently contains exactly one uniquely selected outgoing
// successor; the vector keeps the audit representation explicit and
// forwards-compatible without weakening that gate.
struct ClosedCourseAutowareTerminalSupport
{
  std::uint64_t named_terminal_edge_id{0U};
  std::vector<std::uint64_t> support_edge_ids;
  double terminal_support_length_m{0.0};
  // Planar source length of `named_terminal_edge_id` before compositing.  This
  // makes the final relation's source + support = composite length auditable.
  double named_route_source_length_m{0.0};
  std::string source{"closed_course_semantic_topology"};

  // Geometry carrier for the closed-course Lanelet2 exporter.  It remains
  // separate from the promoted RouteGraph so the named Route's ordered Edge
  // IDs and authored end node retain their original meaning.
  RouteEdge successor_edge;
  RouteNode successor_end_node;
};

// Fully owned result of projecting GUI-authored Routes onto production and
// closed-course graphs. Both Autoware and Nav2 graphs retain the complete
// eligible map; their optional NamedNavigationRoute values identify only the
// mission subset inside it. Optional Routes are present only after every
// selection, semantic-materialization, and stop-line gate has succeeded.
// Keeping the Routes in this value object avoids pointers into temporary
// validation/materialization objects.
struct NavigationPromotionResult
{
  NavigationAuthoringValidationResult production_validation;
  NavigationAuthoringValidationResult closed_course_validation;

  RouteGraph production_nav2_graph;
  RouteGraph production_autoware_graph;
  RouteGraph closed_course_nav2_graph;
  RouteGraph closed_course_autoware_graph;

  std::optional<NamedNavigationRoute> production_nav2_route;
  std::optional<NamedNavigationRoute> production_autoware_route;
  std::optional<NamedNavigationRoute> closed_course_nav2_route;
  std::optional<NamedNavigationRoute> closed_course_autoware_route;
  std::optional<ClosedCourseAutowareTerminalSupport>
    closed_course_autoware_terminal_support;

  std::vector<AuthoredStopLine> production_nav2_stop_lines;
  std::vector<AuthoredStopLine> production_autoware_stop_lines;
  std::vector<AuthoredStopLine> closed_course_nav2_stop_lines;
  std::vector<AuthoredStopLine> closed_course_autoware_stop_lines;
};

// Apply the final promotion gates to already validated authoring data.
// `semantic_map == nullptr` means the semantic layer is unavailable and every
// requested promotion fails closed. `empty_graph` supplies the canonical frame
// used for blocked planner-facing outputs.
[[nodiscard]] NavigationPromotionResult materializeNavigationPromotions(
  const NavigationAuthoringValidationResult & production_validation,
  const NavigationAuthoringValidationResult & closed_course_validation,
  const RouteGraph & production_graph,
  const RouteGraph & closed_course_graph,
  const RouteGraph & empty_graph,
  const SemanticMap * semantic_map,
  bool nav2_requested,
  bool autoware_requested,
  // Full semantic physical topology, before selecting the named Route.  When
  // supplied, a closed-course Autoware promotion fails closed unless its final
  // node has exactly one valid, previously unused outgoing successor.
  const RouteGraph * closed_course_semantic_topology = nullptr);

}  // namespace lidar_mobility_map_generator
