#pragma once

#include "lidar_mobility_map_generator/types.hpp"
#include "lidar_mobility_map_generator/semantic_route_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

enum class NavigationAuthoringTarget
{
  kAutoware,
  kNav2,
  kBoth
};

struct NamedNavigationRoute
{
  std::uint64_t id{0U};
  std::string name;
  NavigationAuthoringTarget target{NavigationAuthoringTarget::kBoth};
  std::uint64_t start_node_id{0U};
  std::uint64_t end_node_id{0U};
  std::vector<std::uint64_t> ordered_edge_ids;
  bool validation_requested{false};
  bool promotion_requested{false};
};

// A stop line is authored in the directed arc-length convention of one Route
// Edge.  `anchor` is mandatory persisted geometry evidence: after a semantic
// split changes Edge IDs it can be rebound to one unambiguous derived edge.
struct AuthoredStopLine
{
  std::uint64_t id{0U};
  std::string name;
  std::uint64_t edge_id{0U};
  double s{0.0};
  double width_m{0.0};
  Vec3 anchor{};
  NavigationAuthoringTarget target{NavigationAuthoringTarget::kBoth};
};

struct NavigationAuthoring
{
  std::uint32_t schema_version{1U};
  std::string frame_id{"map"};
  // Fingerprint of the complete edited Route Graph used by the GUI.  Edge
  // identifiers are only meaningful in that exact graph revision.
  std::string graph_fingerprint;
  std::vector<NamedNavigationRoute> routes;
  std::vector<AuthoredStopLine> stop_lines;
};

struct NamedNavigationRouteStatus
{
  std::uint64_t id{0U};
  bool valid{false};
  bool promotion_eligible{false};
  std::vector<std::string> errors;
};

struct AuthoredStopLineStatus
{
  std::uint64_t id{0U};
  bool valid{false};
  std::vector<std::string> errors;
};

struct NavigationAuthoringValidationResult
{
  NavigationAuthoring authoring;
  std::vector<NamedNavigationRouteStatus> route_statuses;
  std::vector<AuthoredStopLineStatus> stop_line_statuses;
  std::optional<std::uint64_t> selected_autoware_route_id;
  std::optional<std::uint64_t> selected_nav2_route_id;
  bool autoware_stop_lines_valid{true};
  bool nav2_stop_lines_valid{true};
  bool autoware_promoted{false};
  bool nav2_promoted{false};
  std::vector<std::string> errors;
};

[[nodiscard]] const char * toString(NavigationAuthoringTarget target);
[[nodiscard]] NavigationAuthoringTarget navigationAuthoringTargetFromString(
  const std::string & value);
[[nodiscard]] bool includesTarget(
  NavigationAuthoringTarget authored, NavigationAuthoringTarget requested);

// Build the one-click default Mission from every Edge of a chronological
// replay graph, in the graph's persisted order.  This deliberately does not
// run a shortest-path search or discard branches: the complete graph must
// already be one non-cyclic, passable, directed open chain.  Any ambiguity or
// incomplete ordering is rejected so a convenient GUI action cannot shorten
// the measured trajectory silently.  The edited-topology GUI may explicitly
// set require_passable_edges=false only while persisting a pre-regeneration
// authoring request: newly drawn Edges are intentionally unvalidated until the
// Generator reruns.  Operational promotion and staging must still validate
// those Edges against the generated operational graph.
[[nodiscard]] NamedNavigationRoute makeCompleteOpenChainNavigationRoute(
  const RouteGraph & graph,
  std::uint64_t route_id,
  std::string route_name,
  NavigationAuthoringTarget target,
  bool require_passable_edges = true);

void saveNavigationAuthoringJson(
  const std::filesystem::path & path, const NavigationAuthoring & authoring);
[[nodiscard]] NavigationAuthoring loadNavigationAuthoringJson(
  const std::filesystem::path & path);

// Validate a persisted authoring layer against the graph that is eligible for
// the requested output.  Selection is fail-closed: exactly one valid Route
// with validation_requested=true and promotion_requested=true may select each
// target. `both` participates in both target selections.
[[nodiscard]] NavigationAuthoringValidationResult validateNavigationAuthoring(
  const NavigationAuthoring & authoring, const RouteGraph & graph,
  double maximum_anchor_distance = 1.0);

// Second stage for promotion: after the document has been fingerprint-checked
// against the complete edited graph, require every requested Route Edge to be
// present and passable in the planner-facing operational graph.  This does
// not compare fingerprints because the operational graph is intentionally a
// safety-filtered subset of the authored graph.
void applyOperationalGraphSafetyValidation(
  NavigationAuthoringValidationResult & validation,
  const RouteGraph & operational_graph);

// GUI-authored stop lines currently carry no surveyed physical-sign evidence.
// They are valid for the separately labelled closed-course candidate only and
// must not be promoted into a production canonical output.
void applyVirtualStopLineProductionPolicy(
  NavigationAuthoringValidationResult & validation,
  NavigationAuthoringTarget target);

[[nodiscard]] const NamedNavigationRoute * selectedNamedNavigationRoute(
  const NavigationAuthoringValidationResult & validation,
  NavigationAuthoringTarget target);

[[nodiscard]] bool hasPromotionRequest(
  const NavigationAuthoring & authoring, NavigationAuthoringTarget target);

// Return an ordered, single-chain graph.  Structural validation is repeated
// defensively so callers cannot accidentally export a disconnected Route.
[[nodiscard]] RouteGraph selectNamedNavigationRouteGraph(
  const RouteGraph & graph, const NamedNavigationRoute & route);

// Replace authored source Edge IDs with the ordered semantic child Edge IDs.
// `materialized` may contain the complete planner map, not only the selected
// Route. Every selected source arc must remain covered and the selected
// result must be exactly one directed passable chain. Unrelated materialized
// map edges are ignored; a no-entry gap on the selected Route still throws
// instead of silently promoting a shortened/disconnected mission.
[[nodiscard]] NamedNavigationRoute remapNamedNavigationRouteAfterSemantics(
  const RouteGraph & selected_source_graph,
  const NamedNavigationRoute & authored_route,
  const SemanticRouteGraphResult & materialized);

// Rebind already validated source-replay stops to the unique semantic child
// interval that contains their source arc position. The source Edge/s value is
// recoverable from SemanticRouteEdgeProvenance; no nearest-geometry projection
// or whole-Edge rounding is used.
[[nodiscard]] std::vector<AuthoredStopLine> remapResolvedStopLinesAfterSemantics(
  const RouteGraph & source_graph,
  const std::vector<AuthoredStopLine> & resolved_source_stops,
  const SemanticRouteGraphResult & materialized);

// Select target-compatible stop lines on a named Route and rebind their
// anchors to split/derived edges when the original ID no longer exists.
// Missing or ambiguous anchors are omitted instead of being attached to an
// unrelated Lanelet.
[[nodiscard]] std::vector<AuthoredStopLine> resolveStopLinesForGraph(
  const NavigationAuthoringValidationResult & validation,
  const RouteGraph & graph,
  NavigationAuthoringTarget target,
  const NamedNavigationRoute * selected_route = nullptr,
  double maximum_anchor_distance = 1.0);

[[nodiscard]] std::size_t expectedStopLineCountForRoute(
  const NavigationAuthoringValidationResult & validation,
  NavigationAuthoringTarget target,
  const NamedNavigationRoute & selected_route);

void saveNavigationAuthoringStatusJson(
  const std::filesystem::path & path,
  const NavigationAuthoringValidationResult & validation);

}  // namespace lidar_mobility_map_generator
