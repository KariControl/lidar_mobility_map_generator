#include "lidar_mobility_map_generator/exporters.hpp"
#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/navigation_promotion.hpp"
#include "lidar_mobility_map_generator/nav2_route_export.hpp"
#include "lidar_mobility_map_generator/review_io.hpp"

#include "lidar_mobility_map_generator/route_editor.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

void check(const bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

lmmg::RouteEdge edge(
  const std::uint64_t id, const std::uint64_t from, const std::uint64_t to,
  const double start_x, const double end_x)
{
  lmmg::RouteEdge result;
  result.id = id;
  result.from = from;
  result.to = to;
  result.centerline = {{start_x, 0.0, 0.0}, {end_x, 0.0, 0.0}};
  result.left_boundary = {{start_x, 1.0, 0.0}, {end_x, 1.0, 0.0}};
  result.right_boundary = {{start_x, -1.0, 0.0}, {end_x, -1.0, 0.0}};
  result.length = std::abs(end_x - start_x);
  result.passable = true;
  result.corridor_geometry_valid = true;
  result.minimum_safe_width = 2.0;
  result.recommended_speed_mps = 1.0;
  return result;
}

lmmg::RouteGraph graph()
{
  lmmg::RouteGraph result;
  result.frame_id = "map";
  result.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {5.0, 0.0, 0.0}, lmmg::RouteNodeType::kJunction},
    {3U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  result.edges = {edge(10U, 1U, 2U, 0.0, 5.0), edge(11U, 2U, 3U, 5.0, 10.0)};
  return result;
}

lmmg::NavigationAuthoring authoringFor(const lmmg::RouteGraph & source)
{
  lmmg::NavigationAuthoring result;
  result.frame_id = source.frame_id;
  result.graph_fingerprint = lmmg::routeGraphFingerprint(source);
  lmmg::NamedNavigationRoute route;
  route.id = 42U;
  route.name = "構内 Route \"A\"";
  route.target = lmmg::NavigationAuthoringTarget::kBoth;
  route.start_node_id = 1U;
  route.end_node_id = 3U;
  route.ordered_edge_ids = {10U, 11U};
  route.validation_requested = true;
  route.promotion_requested = true;
  result.routes = {route};
  lmmg::AuthoredStopLine stop;
  stop.id = 9U;
  stop.name = "停止線 1";
  stop.edge_id = 11U;
  stop.s = 2.0;
  stop.width_m = 2.4;
  stop.anchor = {7.0, 0.0, 0.0};
  stop.target = lmmg::NavigationAuthoringTarget::kAutoware;
  result.stop_lines = {stop};
  return result;
}

lmmg::RouteGraph topologyWithTerminalSuccessor()
{
  lmmg::RouteGraph result = graph();
  result.nodes.push_back(
    {4U, {15.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint});
  result.edges.push_back(edge(12U, 3U, 4U, 10.0, 15.0));
  return result;
}

double graphPlanarLength(const lmmg::RouteGraph & route_graph)
{
  double result = 0.0;
  for (const lmmg::RouteEdge & route_edge : route_graph.edges) {
    result += lmmg::polylineLength(route_edge.centerline);
  }
  return result;
}

std::size_t countOccurrences(const std::string & text, const std::string & needle)
{
  std::size_t result = 0U;
  std::size_t offset = 0U;
  while ((offset = text.find(needle, offset)) != std::string::npos) {
    ++result;
    offset += needle.size();
  }
  return result;
}

lmmg::NavigationPromotionResult promoteWithTerminalTopology(
  const lmmg::RouteGraph & topology)
{
  const lmmg::RouteGraph edited = graph();
  lmmg::NavigationAuthoring authored = authoringFor(edited);
  authored.stop_lines.clear();
  lmmg::NavigationAuthoringValidationResult production =
    lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(production, edited);
  lmmg::NavigationAuthoringValidationResult closed_course = production;
  lmmg::RouteGraph empty;
  empty.frame_id = "map";
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  return lmmg::materializeNavigationPromotions(
    production, closed_course, edited, edited, empty, &semantics,
    false, true, &topology);
}

void testSchemaRoundTrip()
{
  const lmmg::RouteGraph source = graph();
  const lmmg::NavigationAuthoring authored = authoringFor(source);
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_navigation_authoring_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  lmmg::saveNavigationAuthoringJson(path, authored);
  std::ifstream stream(path, std::ios::binary);
  const std::string json{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  check(json.find("\"schema_version\":1") != std::string::npos, "schema version missing");
  check(json.find("\"graph_fingerprint\":") != std::string::npos, "fingerprint missing");
  check(json.find("\"routes\":[") != std::string::npos, "GUI routes key missing");
  check(json.find("route_requests") == std::string::npos, "obsolete GUI key was emitted");
  const lmmg::NavigationAuthoring loaded = lmmg::loadNavigationAuthoringJson(path);
  std::filesystem::remove(path);
  check(loaded.graph_fingerprint == authored.graph_fingerprint, "fingerprint round trip failed");
  check(loaded.routes.size() == 1U && loaded.routes.front().id == 42U, "route round trip failed");
  check(loaded.routes.front().name == authored.routes.front().name, "UTF-8 route name changed");
  check(loaded.stop_lines.size() == 1U && loaded.stop_lines.front().id == 9U,
    "stop-line round trip failed");
}

void testCompleteOpenChainRouteDerivation()
{
  lmmg::RouteGraph source = graph();
  source.nodes.push_back(
    {4U, {15.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint});
  source.edges.push_back(edge(12U, 3U, 4U, 10.0, 15.0));
  const lmmg::NamedNavigationRoute route =
    lmmg::makeCompleteOpenChainNavigationRoute(
    source, 77U, "走行軌跡全体",
    lmmg::NavigationAuthoringTarget::kAutoware);
  check(route.id == 77U && route.start_node_id == 1U && route.end_node_id == 4U,
    "complete replay Route endpoints changed");
  check(route.ordered_edge_ids == std::vector<std::uint64_t>({10U, 11U, 12U}),
    "complete replay Route did not retain every Edge in persisted order");
  check(route.validation_requested && route.promotion_requested,
    "complete replay Route did not request validation and promotion");

  const auto rejected = [](const lmmg::RouteGraph & candidate) {
      try {
        static_cast<void>(lmmg::makeCompleteOpenChainNavigationRoute(
            candidate, 1U, "full", lmmg::NavigationAuthoringTarget::kAutoware));
      } catch (const std::invalid_argument &) {
        return true;
      }
      return false;
    };

  lmmg::RouteGraph disconnected = source;
  disconnected.edges[1U].from = 1U;
  check(rejected(disconnected), "disconnected complete replay was accepted");

  lmmg::RouteGraph branch = source;
  branch.nodes.push_back(
    {5U, {10.0, 1.0, 0.0}, lmmg::RouteNodeType::kEndpoint});
  branch.edges.push_back(edge(13U, 2U, 5U, 5.0, 10.0));
  check(rejected(branch), "branched complete replay was shortened or reordered");

  lmmg::RouteGraph duplicate = source;
  duplicate.edges.back().id = duplicate.edges.front().id;
  check(rejected(duplicate), "duplicate complete replay Edge ID was accepted");

  lmmg::RouteGraph cycle = source;
  cycle.edges.back().to = 1U;
  check(rejected(cycle), "cyclic complete replay was accepted as an open Mission");

  lmmg::RouteGraph non_passable = source;
  non_passable.edges[1U].passable = false;
  check(rejected(non_passable), "non-passable complete replay Edge was accepted");
  const lmmg::NamedNavigationRoute unvalidated_topology_route =
    lmmg::makeCompleteOpenChainNavigationRoute(
    non_passable, 78U, "edited topology", lmmg::NavigationAuthoringTarget::kAutoware,
    false);
  check(
    unvalidated_topology_route.ordered_edge_ids ==
    std::vector<std::uint64_t>({10U, 11U, 12U}),
    "pre-regeneration edited-topology Route did not retain every Edge");
  lmmg::NavigationAuthoring unvalidated_topology_authoring;
  unvalidated_topology_authoring.frame_id = non_passable.frame_id;
  unvalidated_topology_authoring.graph_fingerprint =
    lmmg::routeGraphFingerprint(non_passable);
  unvalidated_topology_authoring.routes.push_back(unvalidated_topology_route);
  lmmg::NavigationAuthoringValidationResult unvalidated_topology_validation =
    lmmg::validateNavigationAuthoring(unvalidated_topology_authoring, non_passable);
  check(
    unvalidated_topology_validation.selected_autoware_route_id ==
    unvalidated_topology_route.id,
    "pre-regeneration edited-topology Route was not saved as an authoring request");
  lmmg::applyOperationalGraphSafetyValidation(
    unvalidated_topology_validation, non_passable);
  check(
    !unvalidated_topology_validation.selected_autoware_route_id,
    "unvalidated edited-topology Route bypassed operational safety validation");

  lmmg::RouteGraph unvalidated_branch = branch;
  for (lmmg::RouteEdge & edge : unvalidated_branch.edges) {
    edge.passable = false;
  }
  try {
    static_cast<void>(lmmg::makeCompleteOpenChainNavigationRoute(
        unvalidated_branch, 79U, "invalid edited topology",
        lmmg::NavigationAuthoringTarget::kAutoware, false));
    check(false, "branched unvalidated edited topology was accepted");
  } catch (const std::invalid_argument &) {
  }

  lmmg::RouteGraph empty;
  empty.frame_id = "map";
  check(rejected(empty), "empty complete replay graph was accepted");
}

void testFingerprintAndTwoStageValidation()
{
  const lmmg::RouteGraph edited = graph();
  lmmg::NavigationAuthoring authored = authoringFor(edited);
  lmmg::NavigationAuthoringValidationResult validation =
    lmmg::validateNavigationAuthoring(authored, edited);
  check(validation.errors.empty(), "valid edited document was rejected");
  check(validation.selected_autoware_route_id == 42U, "Autoware route was not selected");
  check(validation.selected_nav2_route_id == 42U, "Nav2 route was not selected");
  lmmg::applyOperationalGraphSafetyValidation(validation, edited);
  check(validation.selected_autoware_route_id == 42U, "operational Autoware selection failed");

  lmmg::RouteGraph incomplete = edited;
  incomplete.edges.pop_back();
  incomplete.nodes.pop_back();
  validation = lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(validation, incomplete);
  check(!validation.selected_autoware_route_id, "unsafe partial route was promoted");
  check(!validation.route_statuses.front().valid, "operational rejection lacks invalid status");

  authored.graph_fingerprint = "stale";
  validation = lmmg::validateNavigationAuthoring(authored, edited);
  check(!validation.selected_autoware_route_id, "stale fingerprint selected a route");
  check(!validation.route_statuses.front().valid, "stale document route remained valid");
}

void testOrderedSelectionAndStopRebinding()
{
  const lmmg::RouteGraph edited = graph();
  const lmmg::NavigationAuthoring authored = authoringFor(edited);
  const lmmg::NavigationAuthoringValidationResult validation =
    lmmg::validateNavigationAuthoring(authored, edited);
  const lmmg::RouteGraph selected =
    lmmg::selectNamedNavigationRouteGraph(edited, authored.routes.front());
  check(selected.edges.size() == 2U, "selected chain edge count changed");
  check(selected.edges[0U].id == 10U && selected.edges[1U].id == 11U,
    "selected chain order changed");

  lmmg::RouteGraph derived;
  derived.frame_id = "map";
  derived.nodes = {
    {20U, {5.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {21U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  derived.edges = {edge(101U, 20U, 21U, 5.0, 10.0)};
  const std::vector<lmmg::AuthoredStopLine> stops = lmmg::resolveStopLinesForGraph(
    validation, derived, lmmg::NavigationAuthoringTarget::kAutoware,
    &authored.routes.front());
  check(stops.size() == 1U, "anchored stop line was not rebound");
  check(stops.front().edge_id == 101U, "stop line rebound to the wrong edge");
  check(std::abs(stops.front().s - 2.0) < 1.0e-9, "rebound stop-line arc is wrong");
}

void testDisconnectedAndAmbiguousPromotionFailClosed()
{
  const lmmg::RouteGraph edited = graph();
  lmmg::NavigationAuthoring authored = authoringFor(edited);
  authored.routes.front().ordered_edge_ids = {11U, 10U};
  lmmg::NavigationAuthoringValidationResult validation =
    lmmg::validateNavigationAuthoring(authored, edited);
  check(!validation.route_statuses.front().valid, "disconnected order was accepted");
  check(!validation.selected_nav2_route_id, "disconnected route was selected");

  authored = authoringFor(edited);
  lmmg::NamedNavigationRoute second = authored.routes.front();
  second.id = 43U;
  second.name = "second";
  authored.routes.push_back(second);
  validation = lmmg::validateNavigationAuthoring(authored, edited);
  check(!validation.selected_autoware_route_id, "ambiguous Autoware promotion was selected");
  check(!validation.selected_nav2_route_id, "ambiguous Nav2 promotion was selected");
}

void testDuplicateAndInvalidStopsFailClosed()
{
  const lmmg::RouteGraph edited = graph();
  lmmg::NavigationAuthoring authored = authoringFor(edited);
  lmmg::AuthoredStopLine duplicate = authored.stop_lines.front();
  duplicate.name = "duplicate ID";
  authored.stop_lines.push_back(duplicate);
  lmmg::NavigationAuthoringValidationResult validation =
    lmmg::validateNavigationAuthoring(authored, edited);
  check(!validation.autoware_stop_lines_valid, "duplicate stop IDs remained valid");
  check(!validation.selected_autoware_route_id, "route with duplicate stop IDs was selected");
  const std::vector<lmmg::AuthoredStopLine> resolved =
    lmmg::resolveStopLinesForGraph(
    validation, edited, lmmg::NavigationAuthoringTarget::kAutoware,
    &authored.routes.front());
  check(resolved.empty(), "first duplicate stop escaped lookup validation");

  authored = authoringFor(edited);
  authored.stop_lines.front().width_m = 1001.0;
  validation = lmmg::validateNavigationAuthoring(authored, edited);
  check(!validation.autoware_stop_lines_valid, "absurd stop width remained valid");
  check(!validation.selected_autoware_route_id, "route with absurd stop width was selected");
}

void testVirtualStopLinesAreClosedCourseOnly()
{
  const lmmg::RouteGraph edited = graph();
  lmmg::NavigationAuthoring authored = authoringFor(edited);
  lmmg::NavigationAuthoringValidationResult production =
    lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(production, edited);
  lmmg::applyVirtualStopLineProductionPolicy(
    production, lmmg::NavigationAuthoringTarget::kAutoware);
  check(!production.selected_autoware_route_id,
    "GUI virtual stop line was promoted to production Autoware");
  check(production.selected_nav2_route_id == 42U,
    "Autoware-only stop line incorrectly blocked the Nav2 Route");
  check(production.route_statuses.front().valid,
    "shared Route was globally invalidated while Nav2 remained eligible");
  check(
    !production.errors.empty() &&
    production.errors.back() ==
    "authored_virtual_stop_line_not_physically_verified:autoware",
    "production virtual-stop blocker was not reported");

  // The same authored data remains eligible for the explicitly labelled
  // closed-course candidate, where virtual GUI stops are permitted.
  lmmg::NavigationAuthoringValidationResult closed_course =
    lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(closed_course, edited);
  check(closed_course.selected_autoware_route_id == 42U,
    "virtual stop line was incorrectly rejected from closed-course output");

  authored.stop_lines.front().target = lmmg::NavigationAuthoringTarget::kBoth;
  production = lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(production, edited);
  lmmg::applyVirtualStopLineProductionPolicy(
    production, lmmg::NavigationAuthoringTarget::kAutoware);
  lmmg::applyVirtualStopLineProductionPolicy(
    production, lmmg::NavigationAuthoringTarget::kNav2);
  check(!production.selected_autoware_route_id && !production.selected_nav2_route_id,
    "both-target virtual stop line escaped production blocking");
  check(!production.route_statuses.front().valid,
    "fully blocked Route status remained valid");
}

lmmg::SemanticFeature semanticSpan(
  const lmmg::SemanticFeatureType type, const std::uint64_t edge_id,
  const double start_s, const double end_s)
{
  lmmg::SemanticFeature feature;
  feature.id = 700U;
  feature.type = type;
  feature.geometry = lmmg::SemanticGeometryType::kRouteEdges;
  feature.value = 0.4;
  feature.route_edge_ids = {edge_id};
  feature.route_edge_spans = {{edge_id, start_s, end_s, std::nullopt, std::nullopt}};
  return feature;
}

void testSemanticRemapPreservesOrderAndRejectsGap()
{
  const lmmg::RouteGraph edited = graph();
  const lmmg::NavigationAuthoring authored = authoringFor(edited);
  const lmmg::RouteGraph selected =
    lmmg::selectNamedNavigationRouteGraph(edited, authored.routes.front());
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    semanticSpan(lmmg::SemanticFeatureType::kSpeedLimit, 10U, 1.0, 4.0)};
  lmmg::SemanticRouteGraphResult materialized =
    lmmg::materializeSemanticRouteGraph(selected, semantics);
  const lmmg::NamedNavigationRoute remapped =
    lmmg::remapNamedNavigationRouteAfterSemantics(
    selected, authored.routes.front(), materialized);
  check(remapped.ordered_edge_ids.size() == materialized.graph.edges.size(),
    "semantic child edge coverage was not preserved");
  check(remapped.ordered_edge_ids.size() == 4U, "speed split did not preserve ordered children");

  semantics.features = {
    semanticSpan(lmmg::SemanticFeatureType::kNoEntry, 10U, 2.0, 3.0)};
  materialized = lmmg::materializeSemanticRouteGraph(selected, semantics);
  bool rejected = false;
  try {
    (void)lmmg::remapNamedNavigationRouteAfterSemantics(
      selected, authored.routes.front(), materialized);
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected, "semantic no-entry gap was promoted as a named Route");
}

void testPromotionOwnsMaterializedRoutesAndClosedCourseStops()
{
  const lmmg::RouteGraph edited = graph();
  const lmmg::NavigationAuthoring authored = authoringFor(edited);
  lmmg::NavigationAuthoringValidationResult production =
    lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(production, edited);
  lmmg::applyVirtualStopLineProductionPolicy(
    production, lmmg::NavigationAuthoringTarget::kAutoware);
  lmmg::applyVirtualStopLineProductionPolicy(
    production, lmmg::NavigationAuthoringTarget::kNav2);
  lmmg::NavigationAuthoringValidationResult closed_course =
    lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(closed_course, edited);
  lmmg::RouteGraph empty;
  empty.frame_id = "map";
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";

  const lmmg::NavigationPromotionResult result =
    lmmg::materializeNavigationPromotions(
    production, closed_course, edited, edited, empty, &semantics, true, true);

  check(result.production_nav2_route.has_value(), "valid production Nav2 Route was lost");
  check(result.production_nav2_graph.edges.size() == 2U,
    "production Nav2 graph changed during promotion");
  check(!result.production_autoware_route,
    "virtual stop line escaped the Autoware production policy");
  check(result.production_autoware_graph.edges.empty(),
    "blocked production Autoware graph was not empty");
  check(result.closed_course_nav2_route.has_value(),
    "valid closed-course Nav2 Route was lost");
  check(result.closed_course_autoware_route.has_value(),
    "valid closed-course Autoware Route was lost");
  check(result.closed_course_autoware_stop_lines.size() == 1U,
    "closed-course stop line was not resolved");
  check(result.closed_course_autoware_stop_lines.front().edge_id == 11U,
    "closed-course stop line resolved onto the wrong edge");
}

void testNamedRouteDoesNotShrinkAutowareLaneletMap()
{
  // The map and the mission have deliberately different scopes. The third
  // Edge must remain routable in the Lanelet2 map even though the named
  // mission ends after Edge 11.
  const lmmg::RouteGraph full_map = topologyWithTerminalSuccessor();
  lmmg::NavigationAuthoring authored = authoringFor(full_map);
  authored.stop_lines.clear();
  lmmg::NavigationAuthoringValidationResult validation =
    lmmg::validateNavigationAuthoring(authored, full_map);
  lmmg::applyOperationalGraphSafetyValidation(validation, full_map);
  lmmg::RouteGraph empty;
  empty.frame_id = "map";
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";

  const std::size_t source_edge_count = full_map.edges.size();
  const double source_length = graphPlanarLength(full_map);
  const lmmg::NavigationPromotionResult promotion =
    lmmg::materializeNavigationPromotions(
    validation, validation, full_map, full_map, empty, &semantics, false, true);

  check(promotion.closed_course_autoware_route.has_value(),
    "valid named Autoware mission was lost");
  check(
    promotion.closed_course_autoware_route->ordered_edge_ids ==
    std::vector<std::uint64_t>({10U, 11U}),
    "named mission Edge order changed");
  check(promotion.closed_course_autoware_graph.edges.size() == source_edge_count,
    "named mission selection shrank the full closed-course Autoware map");
  check(
    std::abs(graphPlanarLength(promotion.closed_course_autoware_graph) - source_length) <=
    1.0e-9,
    "named mission selection changed the full closed-course Autoware map length");

  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 1.8;
  options.estimated_front_extent = 1.0;
  options.estimated_rear_extent = 1.0;
  options.estimated_minimum_turning_radius = 4.8;
  options.lateral_clearance_margin = 0.15;
  options.experimental_ready = true;
  lmmg::Lanelet2AuthoringOptions export_authoring;
  export_authoring.named_route = &*promotion.closed_course_autoware_route;
  const std::string suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm";
  const std::filesystem::path baseline_path = std::filesystem::temp_directory_path() /
    ("lmmg_full_map_without_mission_" + suffix);
  const std::filesystem::path authored_path = std::filesystem::temp_directory_path() /
    ("lmmg_full_map_named_mission_" + suffix);
  const lmmg::ClosedCourseLanelet2ExportSummary baseline_summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    baseline_path, full_map, lmmg::Lanelet2Config{}, options);
  const std::vector<lmmg::Lanelet2ReviewLanelet> baseline_lanelets =
    lmmg::loadGeneratedLanelet2Osm(baseline_path);
  const lmmg::ClosedCourseLanelet2ExportSummary summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    authored_path, promotion.closed_course_autoware_graph, lmmg::Lanelet2Config{}, options,
    export_authoring);
  const std::vector<lmmg::Lanelet2ReviewLanelet> lanelets =
    lmmg::loadGeneratedLanelet2Osm(authored_path);
  std::ifstream stream(authored_path, std::ios::binary);
  const std::string osm{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  std::filesystem::remove(baseline_path);
  std::filesystem::remove(authored_path);

  check(summary.source_physical_edges == source_edge_count &&
    summary.exported_physical_edges == source_edge_count,
    "Lanelet2 export dropped a non-mission map Edge");
  check(baseline_lanelets.size() == source_edge_count &&
    lanelets.size() == baseline_lanelets.size(),
    "Lanelet2 relation count changed after named mission authoring");
  check(std::abs(baseline_summary.source_length - source_length) <= 1.0e-9 &&
    std::abs(summary.source_length - baseline_summary.source_length) <= 1.0e-9 &&
    std::abs(summary.exported_length - baseline_summary.exported_length) <= 1.0e-9,
    "Lanelet2 map length changed after named mission authoring");
  check(countOccurrences(osm, "k=\"named_route_id\" v=\"42\"") == 2U,
    "named Route metadata was not limited to the two mission Lanelets");
  check(countOccurrences(osm, "k=\"named_route_order\"") == 2U &&
    osm.find("k=\"named_route_order\" v=\"0\"") != std::string::npos &&
    osm.find("k=\"named_route_order\" v=\"1\"") != std::string::npos,
    "named mission order tags are incomplete");
}

void testNamedRouteDoesNotShrinkNav2RouteMap()
{
  // Route Server's map has three eligible Edges, while the operator-selected
  // Mission deliberately uses only the first two. Promotion and canonical
  // export must retain Edge 12 and its length.
  const lmmg::RouteGraph full_map = topologyWithTerminalSuccessor();
  lmmg::NavigationAuthoring authored = authoringFor(full_map);
  authored.stop_lines.clear();
  lmmg::NavigationAuthoringValidationResult validation =
    lmmg::validateNavigationAuthoring(authored, full_map);
  lmmg::applyOperationalGraphSafetyValidation(validation, full_map);
  lmmg::RouteGraph empty;
  empty.frame_id = "map";
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";

  const std::size_t source_edge_count = full_map.edges.size();
  const double source_length = graphPlanarLength(full_map);
  const lmmg::NavigationPromotionResult promotion =
    lmmg::materializeNavigationPromotions(
    validation, validation, full_map, full_map, empty, &semantics, true, false);

  check(promotion.production_nav2_route.has_value() &&
    promotion.closed_course_nav2_route.has_value(),
    "valid named Nav2 Mission was lost");
  check(
    promotion.closed_course_nav2_route->ordered_edge_ids ==
    std::vector<std::uint64_t>({10U, 11U}),
    "named Nav2 Mission Edge order changed");
  check(promotion.production_nav2_graph.edges.size() == source_edge_count &&
    promotion.closed_course_nav2_graph.edges.size() == source_edge_count,
    "named Nav2 Mission selection shrank a full Route Server map");
  check(
    std::abs(graphPlanarLength(promotion.production_nav2_graph) - source_length) <= 1.0e-9 &&
    std::abs(graphPlanarLength(promotion.closed_course_nav2_graph) - source_length) <= 1.0e-9,
    "named Nav2 Mission selection changed full map length");

  const lmmg::RouteGraph selected = lmmg::selectNamedNavigationRouteGraph(
    promotion.production_nav2_graph, *promotion.production_nav2_route);
  const std::string suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count()) + ".geojson";
  const std::filesystem::path full_path = std::filesystem::temp_directory_path() /
    ("lmmg_nav2_full_map_" + suffix);
  const std::filesystem::path mission_path = std::filesystem::temp_directory_path() /
    ("lmmg_nav2_selected_mission_" + suffix);
  lmmg::saveNav2RouteGraphGeoJson(
    full_path, promotion.production_nav2_graph, 0.05, 10.0,
    &*promotion.production_nav2_route);
  lmmg::saveNav2RouteGraphGeoJson(
    mission_path, selected, 0.05, 10.0, &*promotion.production_nav2_route);
  std::ifstream full_stream(full_path, std::ios::binary);
  const std::string full_geojson{
    std::istreambuf_iterator<char>(full_stream), std::istreambuf_iterator<char>()};
  std::ifstream mission_stream(mission_path, std::ios::binary);
  const std::string mission_geojson{
    std::istreambuf_iterator<char>(mission_stream), std::istreambuf_iterator<char>()};
  std::filesystem::remove(full_path);
  std::filesystem::remove(mission_path);

  check(countOccurrences(full_geojson, "\"source_route_edge_id\"") == source_edge_count,
    "canonical Nav2 export omitted a non-Mission map Edge");
  check(countOccurrences(full_geojson, "\"named_route_id\":42") == 2U,
    "canonical Nav2 export did not limit named metadata to Mission Edges");
  check(countOccurrences(mission_geojson, "\"source_route_edge_id\"") == 2U &&
    mission_geojson.find("\"source_route_edge_id\":12") == std::string::npos,
    "Nav2 selected-Mission sidecar is not the exact two-Edge chain");
}

void testEditedTopologyLaneletProvenance()
{
  const lmmg::RouteGraph source = graph();
  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 1.8;
  options.estimated_front_extent = 1.0;
  options.estimated_rear_extent = 1.0;
  options.estimated_minimum_turning_radius = 4.8;
  options.lateral_clearance_margin = 0.15;
  options.experimental_ready = true;
  options.centerline_source = "edited_topology";
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_edited_topology_provenance_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm");
  const lmmg::ClosedCourseLanelet2ExportSummary summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    path, source, lmmg::Lanelet2Config{}, options);
  check(summary.exported_physical_edges == source.edges.size(),
    "edited-topology provenance test did not export every source Edge");
  std::ifstream stream(path, std::ios::binary);
  const std::string osm{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  std::filesystem::remove(path);

  check(
    countOccurrences(osm, "k=\"centerline_source\" v=\"edited_topology\"") ==
    source.edges.size() &&
    countOccurrences(osm, "k=\"provenance\" v=\"user_authored_centerline\"") ==
    source.edges.size() &&
    countOccurrences(osm, "k=\"observed_driven\" v=\"no\"") ==
    source.edges.size() &&
    countOccurrences(
      osm,
      "k=\"validation_status\" "
      "v=\"user_authored_vehicle_footprint_validated_candidate\"") ==
    source.edges.size(),
    "edited-topology Lanelets did not preserve their distinct provenance");
}

void testGeneratedOpenRouteNeverAddsSyntheticPlanningSupport()
{
  const lmmg::RouteGraph source = graph();
  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 1.8;
  options.estimated_front_extent = 3.0;
  options.estimated_rear_extent = 2.0;
  options.estimated_minimum_turning_radius = 4.8;
  options.lateral_clearance_margin = 0.15;
  options.experimental_ready = true;

  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_no_synthetic_open_support_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm");
  const auto verify = [&](const lmmg::Lanelet2AuthoringOptions & authoring,
      const std::string & context) {
      const lmmg::ClosedCourseLanelet2ExportSummary summary =
        lmmg::saveClosedCourseExperimentalLanelet2Osm(
        path, source, lmmg::Lanelet2Config{}, options, authoring);
      const std::vector<lmmg::Lanelet2ReviewLanelet> lanelets =
        lmmg::loadGeneratedLanelet2Osm(path);
      std::ifstream stream(path, std::ios::binary);
      const std::string osm{
        std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};

      check(summary.synthetic_planning_support.empty(),
        context + " export added unobserved synthetic planning support");
      check(summary.source_physical_edges == source.edges.size() &&
        summary.exported_physical_edges == source.edges.size() &&
        lanelets.size() == source.edges.size(),
        context + " export changed the measured replay Edge count");
      check(countOccurrences(
          osm, "k=\"synthetic_planning_support_count\" v=\"0\"") ==
        source.edges.size() &&
        osm.find("k=\"synthetic_planning_support\" v=\"yes\"") ==
        std::string::npos &&
        osm.find("synthetic_test_kinematic_staging") == std::string::npos &&
        osm.find("k=\"observed_driven\" v=\"no\"") == std::string::npos,
        context + " OSM serialized forbidden synthetic route geometry");

      const lmmg::Lanelet2ReviewLanelet & first = lanelets.front();
      const lmmg::Lanelet2ReviewLanelet & last = lanelets.back();
      check(first.centerline.front().x == 0.0 && last.centerline.back().x == 10.0,
        context + " export changed measured centerline endpoints");
      check(first.left_boundary.front().x < -2.049 &&
        first.right_boundary.front().x < -2.049 &&
        last.left_boundary.back().x > 13.049 &&
        last.right_boundary.back().x > 13.049,
        context + " export lost the vehicle swept-envelope endpoint caps");
    };

  verify(lmmg::Lanelet2AuthoringOptions{}, "initial replay");

  lmmg::NavigationAuthoring authored = authoringFor(source);
  authored.stop_lines.clear();
  authored.routes.front().target = lmmg::NavigationAuthoringTarget::kAutoware;
  lmmg::Lanelet2AuthoringOptions exact_mission;
  exact_mission.named_route = &authored.routes.front();
  verify(exact_mission, "exact full-replay Mission");
  std::filesystem::remove(path);
}

void testSyntheticOpenRoutePlanningSupportPreservesRawReplay()
{
  const lmmg::RouteGraph source = graph();
  lmmg::NavigationAuthoring authored = authoringFor(source);
  authored.stop_lines.clear();
  authored.routes.front().target = lmmg::NavigationAuthoringTarget::kAutoware;
  const lmmg::NamedNavigationRoute & route = authored.routes.front();
  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 1.8;
  options.estimated_front_extent = 3.0;
  options.estimated_rear_extent = 2.0;
  options.estimated_minimum_turning_radius = 4.8;
  options.lateral_clearance_margin = 0.15;
  options.experimental_ready = true;
  options.test_only_add_open_route_planning_support = true;
  options.planning_endpoint_allowance = 0.50;
  lmmg::Lanelet2AuthoringOptions export_authoring;
  export_authoring.named_route = &route;
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_synthetic_open_support_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm");
  const lmmg::ClosedCourseLanelet2ExportSummary summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    path, source, lmmg::Lanelet2Config{}, options, export_authoring);
  const std::vector<lmmg::Lanelet2ReviewLanelet> lanelets =
    lmmg::loadGeneratedLanelet2Osm(path);
  std::ifstream stream(path, std::ios::binary);
  const std::string osm{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  std::filesystem::remove(path);

  check(summary.source_physical_edges == 2U &&
    summary.exported_physical_edges == 2U &&
    summary.exported_lanelet_segments == 2U,
    "synthetic planning support changed raw replay counts");
  check(std::abs(summary.source_length - 10.0) <= 1.0e-12 &&
    std::abs(summary.exported_length - 10.0) <= 1.0e-12,
    "synthetic planning support changed raw replay lengths");
  check(summary.synthetic_planning_support.size() == 2U && lanelets.size() == 4U,
    "synthetic head/tail Lanelets were not exported separately");
  check(summary.synthetic_planning_support[0].role == "head" &&
    summary.synthetic_planning_support[1].role == "tail" &&
    summary.synthetic_planning_support[0].adjacent_source_edge_id == 10U &&
    summary.synthetic_planning_support[1].adjacent_source_edge_id == 11U,
    "synthetic planning support raw provenance is incomplete");
  check(summary.synthetic_planning_support[0].actual_left_boundary_beyond_raw_endpoint_m >=
    1.50 &&
    summary.synthetic_planning_support[0].actual_right_boundary_beyond_raw_endpoint_m >=
    1.50 &&
    summary.synthetic_planning_support[1].actual_left_boundary_beyond_raw_endpoint_m >=
    2.50 &&
    summary.synthetic_planning_support[1].actual_right_boundary_beyond_raw_endpoint_m >=
    2.50,
    "synthetic boundaries do not contain raw endpoint vehicle poses plus allowance");
  const auto raw_first = std::find_if(
    lanelets.begin(), lanelets.end(),
    [](const lmmg::Lanelet2ReviewLanelet & lanelet) {return lanelet.route_edge_id == 10U;});
  const auto raw_last = std::find_if(
    lanelets.begin(), lanelets.end(),
    [](const lmmg::Lanelet2ReviewLanelet & lanelet) {return lanelet.route_edge_id == 11U;});
  const auto same_points = [](const std::vector<lmmg::Vec3> & first,
      const std::vector<lmmg::Vec3> & second) {
      return first.size() == second.size() && std::equal(
        first.begin(), first.end(), second.begin(),
        [](const lmmg::Vec3 & lhs, const lmmg::Vec3 & rhs) {
          return lmmg::distance3d(lhs, rhs) <= 1.0e-12;
        });
    };
  check(raw_first != lanelets.end() && raw_last != lanelets.end() &&
    same_points(raw_first->centerline, source.edges[0].centerline) &&
    same_points(raw_last->centerline, source.edges[1].centerline),
    "synthetic planning support changed raw serialized vertices");
  check(countOccurrences(osm, "k=\"synthetic_planning_support\" v=\"yes\"") == 2U &&
    countOccurrences(osm, "k=\"planning_support_contract_version\" v=\"2\"") == 2U &&
    countOccurrences(osm, "k=\"synthetic_test_staging\" v=\"yes\"") == 2U &&
    countOccurrences(osm, "k=\"support_is_part_of_raw_counts\" v=\"no\"") == 2U &&
    countOccurrences(osm, "k=\"support_is_part_of_named_route\" v=\"no\"") == 2U &&
    countOccurrences(osm, "k=\"support_is_raw_coverage\" v=\"no\"") == 2U &&
    countOccurrences(
      osm,
      "k=\"planning_support_outer_pose_isolation_scope\" "
      "v=\"nonadjacent_raw_route_centerlines\"") == 2U &&
    countOccurrences(
      osm,
      "k=\"planning_support_outer_pose_isolation_derivation\" "
      "v=\"vehicle_footprint_circumradius_plus_endpoint_allowance\"") == 2U &&
    countOccurrences(osm, "k=\"surveyed\" v=\"no\"") == 2U &&
    countOccurrences(osm, "k=\"deployment_ready\" v=\"no\"") == 2U &&
    countOccurrences(osm, "k=\"named_route_order\"") == 2U &&
    countOccurrences(osm, "k=\"observed_driven\" v=\"no\"") == 2U,
    "synthetic support tags or raw named-Route membership are incorrect");
  for (const lmmg::SyntheticOpenRoutePlanningSupport & support :
    summary.synthetic_planning_support)
  {
    check(
      support.planning_support_contract_version == 2U &&
      support.candidate_count_tested == support.selected_candidate_index + 1U &&
      support.individually_valid_candidate_rank >= 1U &&
      support.rejected_kinematic_candidates +
      support.rejected_invalid_geometry_candidates +
      support.rejected_outer_raw_overlap_candidates +
      support.rejected_insufficient_outer_pose_isolation_candidates +
      support.rejected_raw_polygon_reentry_candidates +
      support.rejected_nonadjacent_transition_candidates +
      support.individually_valid_candidate_rank - 1U ==
      support.selected_candidate_index &&
      support.kinematic_valid && support.outer_endpoint_unique &&
      support.outer_endpoint_route_polygon_edge_ids ==
      std::vector<std::uint64_t>{support.edge_id} &&
      support.outer_footprint_raw_overlap_edge_ids.empty() &&
      support.actual_outer_pose_nonadjacent_raw_centerline_isolation_m + 1.0e-9 >=
      support.required_outer_pose_nonadjacent_raw_centerline_isolation_m &&
      support.outer_pose_nonadjacent_raw_centerline_count == 1U &&
      support.outer_pose_nearest_nonadjacent_raw_centerline_edge_ids.size() == 1U &&
      support.raw_overlap_single_transition &&
      support.nonadjacent_raw_overlap_transition_length_m <=
      support.maximum_nonadjacent_raw_overlap_transition_length_m + 1.0e-9 &&
      support.outer_footprint_contained &&
      support.connection_footprint_contained,
      "synthetic staging search/containment audit is incomplete");
  }

  const lmmg::ClosedCourseLanelet2ExportSummary initial_summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    path, source, lmmg::Lanelet2Config{}, options, lmmg::Lanelet2AuthoringOptions{});
  std::ifstream initial_stream(path, std::ios::binary);
  const std::string initial_osm{
    std::istreambuf_iterator<char>(initial_stream),
    std::istreambuf_iterator<char>()};
  std::filesystem::remove(path);
  check(initial_summary.synthetic_planning_support.size() == 2U &&
    countOccurrences(initial_osm, "k=\"synthetic_planning_support\" v=\"yes\"") == 2U &&
    countOccurrences(initial_osm, "k=\"named_route_order\"") == 0U,
    "initial unselected replay did not receive separate endpoint support");

  lmmg::NamedNavigationRoute shortened = route;
  shortened.ordered_edge_ids.pop_back();
  shortened.end_node_id = 2U;
  export_authoring.named_route = &shortened;
  bool rejected = false;
  try {
    const lmmg::ClosedCourseLanelet2ExportSummary ignored =
      lmmg::saveClosedCourseExperimentalLanelet2Osm(
      path, source, lmmg::Lanelet2Config{}, options, export_authoring);
    (void)ignored;
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  std::filesystem::remove(path);
  check(rejected, "synthetic support accepted a shortened Mission");
}

void testKinematicStagingTurnsAroundSpatialSeamWithoutTrimmingRawRoute()
{
  lmmg::RouteGraph source;
  source.frame_id = "map";
  source.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kJunction},
    {3U, {10.0, 20.0, 0.0}, lmmg::RouteNodeType::kJunction},
    {4U, {-10.0, 20.0, 0.0}, lmmg::RouteNodeType::kJunction},
    {5U, {-10.0, 0.0, 0.0}, lmmg::RouteNodeType::kJunction},
    {6U, {-0.25, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  const auto make_edge = [](const std::uint64_t id, const std::uint64_t from,
      const std::uint64_t to, const std::vector<lmmg::Vec3> & centerline) {
      lmmg::RouteEdge result;
      result.id = id;
      result.from = from;
      result.to = to;
      result.centerline = {centerline.front()};
      for (std::size_t index = 1U; index < centerline.size(); ++index) {
        const lmmg::Vec3 & first = centerline[index - 1U];
        const lmmg::Vec3 & second = centerline[index];
        const std::size_t pieces = std::max<std::size_t>(
          1U, static_cast<std::size_t>(
            std::ceil(lmmg::distance2d(first, second) / 0.50)));
        for (std::size_t piece = 1U; piece <= pieces; ++piece) {
          result.centerline.push_back(first + (second - first) *
            (static_cast<double>(piece) / static_cast<double>(pieces)));
        }
      }
      result.length = lmmg::polylineLength(result.centerline);
      result.passable = true;
      result.corridor_geometry_valid = true;
      result.minimum_safe_width = 1.0;
      result.recommended_speed_mps = 1.0;
      return result;
    };
  std::vector<lmmg::Vec3> first_arc;
  for (std::size_t index = 0U; index <= 40U; ++index) {
    const double angle = -0.5 * lmmg::kPi + lmmg::kPi *
      static_cast<double>(index) / 40.0;
    first_arc.push_back({
      10.0 + 10.0 * std::cos(angle),
      10.0 + 10.0 * std::sin(angle), 0.0});
  }
  std::vector<lmmg::Vec3> second_arc;
  for (std::size_t index = 0U; index <= 40U; ++index) {
    const double angle = 0.5 * lmmg::kPi + lmmg::kPi *
      static_cast<double>(index) / 40.0;
    second_arc.push_back({
      -10.0 + 10.0 * std::cos(angle),
      10.0 + 10.0 * std::sin(angle), 0.0});
  }
  source.edges = {
    make_edge(10U, 1U, 2U, {source.nodes[0].position, source.nodes[1].position}),
    make_edge(11U, 2U, 3U, first_arc),
    make_edge(12U, 3U, 4U, {source.nodes[2].position, source.nodes[3].position}),
    make_edge(13U, 4U, 5U, second_arc),
    make_edge(14U, 5U, 6U, {source.nodes[4].position, source.nodes[5].position})};
  lmmg::NamedNavigationRoute route;
  route.id = 84U;
  route.name = "spatial seam full route";
  route.target = lmmg::NavigationAuthoringTarget::kAutoware;
  route.start_node_id = 1U;
  route.end_node_id = 6U;
  route.ordered_edge_ids = {10U, 11U, 12U, 13U, 14U};
  route.validation_requested = true;
  route.promotion_requested = true;

  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 0.10;
  options.estimated_front_extent = 2.0;
  options.estimated_rear_extent = 1.0;
  options.estimated_minimum_turning_radius = 4.8;
  options.lateral_clearance_margin = 0.05;
  options.experimental_ready = true;
  options.test_only_add_open_route_planning_support = true;
  options.planning_endpoint_allowance = 0.50;
  lmmg::Lanelet2AuthoringOptions authoring;
  authoring.named_route = &route;
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_kinematic_staging_seam_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm");
  const lmmg::ClosedCourseLanelet2ExportSummary summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    path, source, lmmg::Lanelet2Config{}, options, authoring);
  const std::vector<lmmg::Lanelet2ReviewLanelet> lanelets =
    lmmg::loadGeneratedLanelet2Osm(path);
  std::filesystem::remove(path);

  check(summary.source_physical_edges == source.edges.size() &&
    summary.exported_physical_edges == source.edges.size() &&
    std::abs(summary.source_length - graphPlanarLength(source)) <= 1.0e-12 &&
    std::abs(summary.exported_length - graphPlanarLength(source)) <= 1.0e-12,
    "kinematic staging changed raw Edge counts or length at a spatial seam");
  const bool selected_turn = std::any_of(
    summary.synthetic_planning_support.begin(),
    summary.synthetic_planning_support.end(),
    [](const lmmg::SyntheticOpenRoutePlanningSupport & support) {
      return support.geometry_kind != "straight";
    });
  check(selected_turn,
    "spatial seam fixture did not exercise a multi-point kinematic support");
  check(std::all_of(
      summary.synthetic_planning_support.begin(),
      summary.synthetic_planning_support.end(),
      [](const lmmg::SyntheticOpenRoutePlanningSupport & support) {
        return support.geometry_kind != "straight";
      }),
    "spatial seam fixture did not exercise both curved endpoint supports");
  check(std::all_of(
      summary.synthetic_planning_support.begin(),
      summary.synthetic_planning_support.end(),
      [](const lmmg::SyntheticOpenRoutePlanningSupport & support) {
        return support.actual_outer_pose_nonadjacent_raw_centerline_isolation_m +
               1.0e-9 >=
               support.required_outer_pose_nonadjacent_raw_centerline_isolation_m &&
               support.actual_outer_pose_nonadjacent_raw_centerline_isolation_m > 2.23;
      }),
    "spatial seam staging did not isolate both outer poses beyond the old 2.23 m case");
  check(summary.synthetic_planning_support.size() == 2U &&
    std::any_of(
      summary.synthetic_planning_support.begin(),
      summary.synthetic_planning_support.end(),
      [](const lmmg::SyntheticOpenRoutePlanningSupport & support) {
        return support.geometry_kind != "straight" &&
               support.centerline_planar_length_m > 0.5 &&
               support.maximum_curvature_inv_m > 0.0;
      }) &&
    summary.synthetic_planning_support.front().candidate_pairs_tested ==
    summary.synthetic_planning_support.back().candidate_pairs_tested &&
    summary.synthetic_planning_support.front().selected_candidate_pair_rank > 0U &&
    summary.synthetic_planning_support.front().rejected_final_boundary_pairs +
    summary.synthetic_planning_support.front().rejected_final_outer_membership_pairs +
    summary.synthetic_planning_support.front().rejected_final_transition_pairs +
    summary.synthetic_planning_support.front().rejected_final_containment_pairs + 1U ==
    summary.synthetic_planning_support.front().selected_candidate_pair_rank,
    "final combined-geometry pair search was not audited at the spatial seam");
  std::vector<std::uint64_t> serialized_order;
  serialized_order.reserve(lanelets.size());
  for (const lmmg::Lanelet2ReviewLanelet & lanelet : lanelets) {
    serialized_order.push_back(lanelet.route_edge_id);
  }
  std::vector<std::uint64_t> expected_order{
    summary.synthetic_planning_support.front().edge_id};
  expected_order.insert(
    expected_order.end(), route.ordered_edge_ids.begin(), route.ordered_edge_ids.end());
  expected_order.push_back(summary.synthetic_planning_support.back().edge_id);
  check(serialized_order == expected_order,
    "final staging sequence is not head support plus every raw Edge plus tail support");
  for (const lmmg::SyntheticOpenRoutePlanningSupport & support :
    summary.synthetic_planning_support)
  {
    const auto serialized = std::find_if(
      lanelets.begin(), lanelets.end(),
      [&](const lmmg::Lanelet2ReviewLanelet & lanelet) {
        return lanelet.route_edge_id == support.edge_id;
      });
    bool spacing_valid = serialized != lanelets.end() &&
      serialized->centerline.size() >= 2U;
    if (spacing_valid) {
      for (std::size_t index = 1U; index < serialized->centerline.size(); ++index) {
        if (lmmg::distance2d(
            serialized->centerline[index - 1U], serialized->centerline[index]) >
          0.100001)
        {
          spacing_valid = false;
          break;
        }
      }
    }
    check(spacing_valid,
      "serialized planning support exceeds its audited 0.10 m sample spacing");
    double maximum_curvature = 0.0;
    if (serialized != lanelets.end()) {
      for (std::size_t index = 1U; index + 1U < serialized->centerline.size(); ++index) {
        const lmmg::Vec2 first{
          serialized->centerline[index].x - serialized->centerline[index - 1U].x,
          serialized->centerline[index].y - serialized->centerline[index - 1U].y};
        const lmmg::Vec2 second{
          serialized->centerline[index + 1U].x - serialized->centerline[index].x,
          serialized->centerline[index + 1U].y - serialized->centerline[index].y};
        const lmmg::Vec2 chord{
          serialized->centerline[index + 1U].x - serialized->centerline[index - 1U].x,
          serialized->centerline[index + 1U].y - serialized->centerline[index - 1U].y};
        const double denominator = lmmg::norm(first) * lmmg::norm(second) *
          lmmg::norm(chord);
        if (denominator > 1.0e-12) {
          const double twice_area = std::abs(
            first.x * second.y - first.y * second.x);
          maximum_curvature = std::max(
            maximum_curvature, 2.0 * twice_area / denominator);
        }
      }
    }
    check(maximum_curvature <= 1.0 / options.estimated_minimum_turning_radius + 1.0e-6 &&
      std::abs(maximum_curvature - support.actual_maximum_curvature_inv_m) <= 1.0e-6,
      "serialized planning-support curvature differs from its audited kinematic path");
  }
  for (const lmmg::RouteEdge & raw : source.edges) {
    const auto serialized = std::find_if(
      lanelets.begin(), lanelets.end(),
      [&](const lmmg::Lanelet2ReviewLanelet & lanelet) {
        return lanelet.route_edge_id == raw.id;
      });
    check(serialized != lanelets.end() &&
      serialized->centerline.size() == raw.centerline.size() &&
      std::equal(
        serialized->centerline.begin(), serialized->centerline.end(),
        raw.centerline.begin(),
        [](const lmmg::Vec3 & lhs, const lmmg::Vec3 & rhs) {
          return lmmg::distance3d(lhs, rhs) <= 1.0e-9;
        }),
      "multi-point kinematic staging changed raw centerline Edge " +
      std::to_string(raw.id) + " (serialized=" +
      std::to_string(
        serialized == lanelets.end() ? 0U : serialized->centerline.size()) +
      ", source=" + std::to_string(raw.centerline.size()) + ")");
  }

  lmmg::ClosedCourseLanelet2ExportOptions invalid_radius = options;
  invalid_radius.estimated_minimum_turning_radius = 0.0;
  bool rejected = false;
  try {
    const auto ignored = lmmg::saveClosedCourseExperimentalLanelet2Osm(
      path, source, lmmg::Lanelet2Config{}, invalid_radius, authoring);
    (void)ignored;
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  std::filesystem::remove(path);
  check(rejected, "kinematic staging accepted a zero minimum turning radius");
}

void testPromotionFailsClosedWithoutSemanticLayer()
{
  const lmmg::RouteGraph edited = graph();
  lmmg::NavigationAuthoring authored = authoringFor(edited);
  authored.stop_lines.clear();
  lmmg::NavigationAuthoringValidationResult production =
    lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(production, edited);
  lmmg::NavigationAuthoringValidationResult closed_course = production;
  lmmg::RouteGraph empty;
  empty.frame_id = "map";

  const lmmg::NavigationPromotionResult result =
    lmmg::materializeNavigationPromotions(
    production, closed_course, edited, edited, empty, nullptr, true, true);

  check(!result.production_nav2_route && !result.production_autoware_route,
    "production Route survived an unavailable semantic layer");
  check(!result.closed_course_nav2_route && !result.closed_course_autoware_route,
    "closed-course Route survived an unavailable semantic layer");
  check(result.production_nav2_graph.edges.empty() &&
    result.production_autoware_graph.edges.empty(),
    "fail-closed production graphs were not empty");
  check(!result.production_validation.selected_nav2_route_id &&
    !result.production_validation.selected_autoware_route_id,
    "blocked production selections were retained");
  check(result.production_validation.errors.size() >= 2U,
    "materialization failures were not reported");
  check(
    result.production_validation.errors.front() ==
    "nav2_production_materialization_failed:semantic_layer_not_ready",
    "Nav2 materialization failure reason changed");
  check(
    result.production_validation.errors.back() ==
    "autoware_production_materialization_failed:semantic_layer_not_ready",
    "Autoware materialization failure reason changed");
}

void testPromotionDefensivelyRejectsProductionVirtualStops()
{
  const lmmg::RouteGraph edited = graph();
  const lmmg::NavigationAuthoring authored = authoringFor(edited);
  lmmg::NavigationAuthoringValidationResult validation =
    lmmg::validateNavigationAuthoring(authored, edited);
  lmmg::applyOperationalGraphSafetyValidation(validation, edited);
  lmmg::RouteGraph empty;
  empty.frame_id = "map";
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";

  const lmmg::NavigationPromotionResult result =
    lmmg::materializeNavigationPromotions(
    validation, validation, edited, edited, empty, &semantics, false, true);

  check(!result.production_autoware_route,
    "production virtual stop line passed the defensive promotion gate");
  check(result.production_autoware_graph.edges.empty(),
    "production graph remained populated after stop-line rejection");
  check(
    !result.production_validation.errors.empty() &&
    result.production_validation.errors.back() ==
    "autoware_production_stop_line_validation_failed:"
    "authored_virtual_stop_line_not_physically_verified",
    "production stop-line rejection reason changed");
  check(result.closed_course_autoware_route.has_value() &&
    result.closed_course_autoware_stop_lines.size() == 1U,
    "production rejection incorrectly blocked the closed-course stop line");
}

void testClosedCourseAutowareTerminalSupportIsAuditedAndCompositeOnly()
{
  lmmg::NavigationPromotionResult result = promoteWithTerminalTopology(
    topologyWithTerminalSuccessor());
  check(result.closed_course_autoware_route.has_value(),
    "unique terminal successor blocked the closed-course Autoware Route");
  check(result.closed_course_autoware_terminal_support.has_value(),
    "unique terminal successor did not produce an audit record");
  lmmg::ClosedCourseAutowareTerminalSupport support =
    *result.closed_course_autoware_terminal_support;
  check(
    result.closed_course_autoware_route->ordered_edge_ids ==
    std::vector<std::uint64_t>({10U, 11U}),
    "terminal support changed the named Route Edge order");
  check(result.closed_course_autoware_graph.edges.size() == 2U,
    "terminal support was inserted into the promoted RouteGraph");
  check(result.closed_course_autoware_graph.edges.back().centerline.back().x == 10.0,
    "terminal support changed the named Route geometry before export");
  check(
    support.named_terminal_edge_id == 11U &&
    support.support_edge_ids == std::vector<std::uint64_t>({12U}) &&
    support.source == "closed_course_semantic_topology",
    "terminal support provenance or Edge IDs are incorrect");
  check(std::abs(support.terminal_support_length_m - 5.0) <= 1.0e-9,
    "terminal support length is incorrect");
  check(std::abs(support.named_route_source_length_m - 5.0) <= 1.0e-9,
    "final named Route Edge source length is incorrect");

  // Regression: both 4 cm deviations are below the normal 5 cm RDP threshold.
  // The terminal composite must retain these exact audited source samples,
  // otherwise its centerline length differs from the two provenance tags.
  lmmg::RouteEdge & exact_terminal = result.closed_course_autoware_graph.edges.back();
  exact_terminal.centerline = {
    {5.0, 0.0, 0.0}, {7.5, 0.04, 0.0}, {10.0, 0.0, 0.0}};
  exact_terminal.left_boundary = {
    {5.0, 1.0, 0.0}, {7.5, 1.04, 0.0}, {10.0, 1.0, 0.0}};
  exact_terminal.right_boundary = {
    {5.0, -1.0, 0.0}, {7.5, -0.96, 0.0}, {10.0, -1.0, 0.0}};
  exact_terminal.length = lmmg::polylineLength(exact_terminal.centerline);
  exact_terminal.recommended_speed_mps = 0.8;
  support.successor_edge.centerline = {
    {10.0, 0.0, 0.0}, {12.5, -0.04, 0.0}, {15.0, 0.0, 0.0}};
  support.successor_edge.recommended_speed_mps = 0.35;
  support.terminal_support_length_m = lmmg::polylineLength(
    support.successor_edge.centerline);
  support.named_route_source_length_m = exact_terminal.length;
  const double expected_composite_length =
    support.named_route_source_length_m + support.terminal_support_length_m;

  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 1.8;
  options.estimated_front_extent = 1.0;
  options.estimated_rear_extent = 1.0;
  options.estimated_minimum_turning_radius = 4.8;
  options.lateral_clearance_margin = 0.15;
  options.experimental_ready = true;
  lmmg::Lanelet2AuthoringOptions authoring;
  authoring.named_route = &*result.closed_course_autoware_route;
  authoring.terminal_support = support;
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_terminal_support_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm");
  const lmmg::ClosedCourseLanelet2ExportSummary summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    path, result.closed_course_autoware_graph, lmmg::Lanelet2Config{}, options, authoring);
  const std::vector<lmmg::Lanelet2ReviewLanelet> lanelets =
    lmmg::loadGeneratedLanelet2Osm(path);
  std::ifstream stream(path, std::ios::binary);
  const std::string osm{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  std::filesystem::remove(path);

  check(summary.terminal_support_applied &&
    summary.terminal_support_named_edge_id == 11U &&
    summary.terminal_support_edge_ids == std::vector<std::uint64_t>({12U}),
    "terminal support export summary is incomplete");
  check(lanelets.size() == 2U,
    "terminal support was incorrectly exported as a separate Lanelet");
  const auto final_lanelet = std::find_if(
    lanelets.begin(), lanelets.end(),
    [](const lmmg::Lanelet2ReviewLanelet & lanelet) {
      return lanelet.route_edge_id == 11U;
    });
  check(final_lanelet != lanelets.end() &&
    std::abs(
      lmmg::polylineLength(final_lanelet->centerline) - expected_composite_length) <=
    1.0e-9 &&
    std::abs(final_lanelet->centerline.back().x - 15.0) <= 1.0e-9,
    "final named Lanelet did not receive the measured support centerline");
  check(osm.find("k=\"autoware_terminal_support\" v=\"yes\"") != std::string::npos &&
    osm.find("k=\"terminal_support_edge_ids\" v=\"12\"") != std::string::npos &&
    osm.find("k=\"terminal_support_length_m\" v=\"") != std::string::npos &&
    osm.find("k=\"named_route_source_length_m\" v=\"") != std::string::npos &&
    osm.find("k=\"generator_speed_limit_mps\" v=\"0.35\"") != std::string::npos &&
    osm.find(
      "k=\"terminal_support_source\" "
      "v=\"closed_course_semantic_topology\"") != std::string::npos,
    "final Lanelet terminal support provenance tags are incomplete");
}

void testClosedCourseAutowareTerminalSupportFailureDoesNotEraseMission()
{
  struct Case
  {
    std::string name;
    lmmg::RouteGraph topology;
  };
  std::vector<Case> cases;

  lmmg::RouteGraph branch = topologyWithTerminalSuccessor();
  branch.nodes.push_back(
    {5U, {15.0, 1.0, 0.0}, lmmg::RouteNodeType::kEndpoint});
  lmmg::RouteEdge branch_edge = edge(13U, 3U, 5U, 10.0, 15.0);
  branch_edge.centerline.back().y = 1.0;
  branch_edge.left_boundary.back().y = 2.0;
  branch_edge.right_boundary.back().y = 0.0;
  branch.edges.push_back(branch_edge);
  cases.push_back({"branch", branch});

  lmmg::RouteGraph cycle = graph();
  lmmg::RouteEdge cycle_edge = edge(12U, 3U, 1U, 10.0, 0.0);
  cycle.edges.push_back(cycle_edge);
  cases.push_back({"cycle", cycle});

  lmmg::RouteGraph reused = topologyWithTerminalSuccessor();
  reused.edges.back().reverse_of = 11U;
  cases.push_back({"reused", reused});

  lmmg::RouteGraph no_entry = topologyWithTerminalSuccessor();
  no_entry.edges.back().passable = false;
  no_entry.edges.back().validation_errors.push_back("semantic_no_entry");
  cases.push_back({"no_entry", no_entry});

  lmmg::RouteGraph sharp_turn = topologyWithTerminalSuccessor();
  sharp_turn.nodes.back().position = {10.0, 5.0, 0.0};
  sharp_turn.edges.back().centerline = {
    {10.0, 0.0, 0.0}, {10.0, 5.0, 0.0}};
  sharp_turn.edges.back().left_boundary = {
    {9.0, 0.0, 0.0}, {9.0, 5.0, 0.0}};
  sharp_turn.edges.back().right_boundary = {
    {11.0, 0.0, 0.0}, {11.0, 5.0, 0.0}};
  sharp_turn.edges.back().length = 5.0;
  cases.push_back({"sharp_turn", sharp_turn});

  for (const Case & test_case : cases) {
    const lmmg::NavigationPromotionResult result = promoteWithTerminalTopology(
      test_case.topology);
    check(result.closed_course_autoware_route.has_value(),
      test_case.name + " optional terminal support failure erased the valid mission");
    check(
      result.closed_course_autoware_route->ordered_edge_ids ==
      std::vector<std::uint64_t>({10U, 11U}),
      test_case.name + " optional terminal support failure changed mission order");
    check(result.closed_course_autoware_graph.edges.size() == graph().edges.size(),
      test_case.name + " optional terminal support failure erased the map");
    check(!result.closed_course_autoware_terminal_support,
      test_case.name + " terminal support left stale provenance");
  }
}

}  // namespace

int main()
{
  testSchemaRoundTrip();
  testCompleteOpenChainRouteDerivation();
  testFingerprintAndTwoStageValidation();
  testOrderedSelectionAndStopRebinding();
  testDisconnectedAndAmbiguousPromotionFailClosed();
  testDuplicateAndInvalidStopsFailClosed();
  testVirtualStopLinesAreClosedCourseOnly();
  testSemanticRemapPreservesOrderAndRejectsGap();
  testPromotionOwnsMaterializedRoutesAndClosedCourseStops();
  testNamedRouteDoesNotShrinkAutowareLaneletMap();
  testNamedRouteDoesNotShrinkNav2RouteMap();
  testEditedTopologyLaneletProvenance();
  testGeneratedOpenRouteNeverAddsSyntheticPlanningSupport();
  testSyntheticOpenRoutePlanningSupportPreservesRawReplay();
  testKinematicStagingTurnsAroundSpatialSeamWithoutTrimmingRawRoute();
  testPromotionFailsClosedWithoutSemanticLayer();
  testPromotionDefensivelyRejectsProductionVirtualStops();
  testClosedCourseAutowareTerminalSupportIsAuditedAndCompositeOnly();
  testClosedCourseAutowareTerminalSupportFailureDoesNotEraseMission();
  return 0;
}
