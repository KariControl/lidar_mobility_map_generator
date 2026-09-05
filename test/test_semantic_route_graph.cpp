#include "lidar_mobility_map_generator/semantic_route_graph.hpp"

#include "lidar_mobility_map_generator/exporters.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

void check(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void checkNear(
  const double actual, const double expected, const double tolerance,
  const std::string & message)
{
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
            message + ": expected " + std::to_string(expected) +
            ", got " + std::to_string(actual));
  }
}

template<typename CallableT>
void checkThrows(CallableT && callable, const std::string & message)
{
  try {
    callable();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error(message);
}

const lmmg::RouteNode & findNode(const lmmg::RouteGraph & graph, const std::uint64_t id)
{
  const auto found = std::find_if(
    graph.nodes.begin(), graph.nodes.end(),
    [id](const lmmg::RouteNode & node) {return node.id == id;});
  if (found == graph.nodes.end()) {
    throw std::runtime_error("missing RouteNode " + std::to_string(id));
  }
  return *found;
}

const lmmg::RouteEdge & findDerivedEdge(
  const lmmg::SemanticRouteGraphResult & result,
  const std::uint64_t source_id,
  const double start_s,
  const double end_s)
{
  const auto provenance = std::find_if(
    result.edge_provenance.begin(), result.edge_provenance.end(),
    [&](const lmmg::SemanticRouteEdgeProvenance & value) {
      return value.source_edge_id == source_id &&
             std::abs(value.source_start_s - start_s) < 1.0e-9 &&
             std::abs(value.source_end_s - end_s) < 1.0e-9;
    });
  if (provenance == result.edge_provenance.end()) {
    throw std::runtime_error("missing semantic segment provenance");
  }
  const auto edge = std::find_if(
    result.graph.edges.begin(), result.graph.edges.end(),
    [&](const lmmg::RouteEdge & value) {return value.id == provenance->edge_id;});
  if (edge == result.graph.edges.end()) {
    throw std::runtime_error("provenance references missing derived edge");
  }
  return *edge;
}

lmmg::RouteEdge makeEdge(
  const std::uint64_t id, const std::uint64_t from, const std::uint64_t to,
  std::vector<lmmg::Vec3> centerline)
{
  lmmg::RouteEdge edge;
  edge.id = id;
  edge.from = from;
  edge.to = to;
  edge.centerline = std::move(centerline);
  edge.length = lmmg::polylineLength(edge.centerline);
  edge.passable = true;
  edge.corridor_geometry_valid = true;
  edge.recommended_speed_mps = 1.0;
  edge.minimum_safe_width = 2.0;
  edge.confidence = 1.0;
  for (const lmmg::Vec3 & point : edge.centerline) {
    edge.left_boundary.push_back({point.x, point.y + 1.0, point.z});
    edge.right_boundary.push_back({point.x, point.y - 1.0, point.z});
    edge.left_clearance.push_back(1.0);
    edge.right_clearance.push_back(1.0);
    edge.left_clearance_observed.push_back(1U);
    edge.right_clearance_observed.push_back(1U);
  }
  return edge;
}

lmmg::SemanticFeature spanFeature(
  const std::uint64_t id, const lmmg::SemanticFeatureType type,
  const std::uint64_t edge_id, const double start_s, const double end_s,
  const double value = 0.0)
{
  lmmg::SemanticFeature feature;
  feature.id = id;
  feature.type = type;
  feature.geometry = lmmg::SemanticGeometryType::kRouteEdges;
  feature.value = value;
  feature.route_edge_ids = {edge_id};
  feature.route_edge_spans = {{edge_id, start_s, end_s, std::nullopt, std::nullopt}};
  return feature;
}

std::size_t countOccurrences(const std::string & text, const std::string & pattern)
{
  std::size_t count = 0U;
  std::size_t position = 0U;
  while ((position = text.find(pattern, position)) != std::string::npos) {
    ++count;
    position += pattern.size();
  }
  return count;
}

std::string relationForRouteEdge(const std::string & osm, const std::uint64_t edge_id)
{
  const std::string marker =
    "<tag k=\"route_edge_id\" v=\"" + std::to_string(edge_id) + "\"/>";
  std::size_t marker_position = 0U;
  while ((marker_position = osm.find(marker, marker_position)) != std::string::npos) {
    const std::size_t begin = osm.rfind("<relation ", marker_position);
    const std::size_t end = osm.find("</relation>", marker_position);
    if (begin != std::string::npos && end != std::string::npos) {
      const std::string relation =
        osm.substr(begin, end + std::string{"</relation>"}.size() - begin);
      if (relation.find("<tag k=\"type\" v=\"lanelet\"/>") != std::string::npos) {
        return relation;
      }
    }
    marker_position += marker.size();
  }
  return {};
}

void testExactSpeedAndNoEntrySplits()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {makeEdge(3U, 1U, 2U, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}})};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    spanFeature(1U, lmmg::SemanticFeatureType::kSpeedLimit, 3U, 2.0, 5.0, 0.4),
    spanFeature(2U, lmmg::SemanticFeatureType::kSpeedLimit, 3U, 4.0, 8.0, 0.2),
    spanFeature(3U, lmmg::SemanticFeatureType::kNoEntry, 3U, 6.0, 7.0)};

  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  check(result.omitted_no_entry_segments == 1U, "no-entry span was not counted as omitted");
  check(result.graph.edges.size() == 6U, "semantic boundary edge count is incorrect");
  check(result.graph.nodes.size() == 8U, "semantic boundary node count is incorrect");
  checkNear(
    findDerivedEdge(result, 3U, 0.0, 2.0).recommended_speed_mps,
    1.0, 1.0e-12, "base speed changed before semantic span");
  checkNear(
    findDerivedEdge(result, 3U, 2.0, 4.0).recommended_speed_mps,
    0.4, 1.0e-12, "first semantic speed was not materialized");
  checkNear(
    findDerivedEdge(result, 3U, 4.0, 5.0).recommended_speed_mps,
    0.2, 1.0e-12, "overlap did not choose the restrictive speed");
  checkNear(
    findDerivedEdge(result, 3U, 5.0, 6.0).recommended_speed_mps,
    0.2, 1.0e-12, "second semantic speed ended at the wrong boundary");
  checkNear(
    findDerivedEdge(result, 3U, 7.0, 8.0).recommended_speed_mps,
    0.2, 1.0e-12, "speed after omitted no-entry span changed");
  const lmmg::RouteEdge & before_gap = findDerivedEdge(result, 3U, 5.0, 6.0);
  const lmmg::RouteEdge & after_gap = findDerivedEdge(result, 3U, 7.0, 8.0);
  checkNear(findNode(result.graph, before_gap.to).position.x, 6.0, 1.0e-12, "gap start node");
  checkNear(findNode(result.graph, after_gap.from).position.x, 7.0, 1.0e-12, "gap end node");
  for (const lmmg::RouteEdge & edge : result.graph.edges) {
    checkNear(
      edge.centerline.front().x, findNode(result.graph, edge.from).position.x,
      1.0e-12, "derived edge start does not equal its RouteNode");
    checkNear(
      edge.centerline.back().x, findNode(result.graph, edge.to).position.x,
      1.0e-12, "derived edge end does not equal its RouteNode");
    check(edge.left_boundary.size() == edge.centerline.size(), "left boundary was not sliced");
    check(edge.right_boundary.size() == edge.centerline.size(), "right boundary was not sliced");
  }

  lmmg::SemanticRouteGraphOptions diagnostic_options;
  diagnostic_options.no_entry_policy = lmmg::SemanticNoEntryPolicy::kKeepImpassable;
  const lmmg::SemanticRouteGraphResult diagnostic =
    lmmg::materializeSemanticRouteGraph(graph, semantics, diagnostic_options);
  const lmmg::RouteEdge & no_entry = findDerivedEdge(diagnostic, 3U, 6.0, 7.0);
  check(!no_entry.passable, "kept no-entry segment remained passable");
  check(
    std::find(
      no_entry.validation_errors.begin(), no_entry.validation_errors.end(),
      "semantic_no_entry") != no_entry.validation_errors.end(),
    "kept no-entry segment lacks a diagnostic reason");
}

void testAuthoredSpeedMayExceedGeneratedDefaultWithoutBoundaryGap()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {8.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {3U, {18.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  lmmg::RouteEdge first = makeEdge(
    75U, 1U, 2U, {{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}});
  lmmg::RouteEdge second = makeEdge(
    83U, 2U, 3U, {{8.0, 0.0, 0.0}, {18.0, 0.0, 0.0}});
  first.recommended_speed_mps = 0.5;
  second.recommended_speed_mps = 0.5;
  graph.edges = {first, second};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    spanFeature(1U, lmmg::SemanticFeatureType::kSpeedLimit, 75U, 0.0, 8.0, 0.9),
    spanFeature(2U, lmmg::SemanticFeatureType::kSpeedLimit, 83U, 0.0, 10.0, 0.3)};

  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  check(result.graph.edges.size() == 2U, "full-edge speed limits added a gap segment");
  checkNear(
    findDerivedEdge(result, 75U, 0.0, 8.0).recommended_speed_mps,
    0.9, 1.0e-12, "authored 0.9 m/s was capped by generated 0.5 m/s default");
  checkNear(
    findDerivedEdge(result, 83U, 0.0, 10.0).recommended_speed_mps,
    0.3, 1.0e-12, "adjacent authored 0.3 m/s was not preserved");
  check(
    findDerivedEdge(result, 75U, 0.0, 8.0).to ==
    findDerivedEdge(result, 83U, 0.0, 10.0).from,
    "adjacent 0.9/0.3 speed intervals do not share their exact boundary");

  const std::vector<lmmg::EdgeSemanticRule> summary =
    lmmg::deriveEdgeSemanticRules(semantics, graph);
  const auto first_summary = std::find_if(
    summary.begin(), summary.end(),
    [](const lmmg::EdgeSemanticRule & rule) {return rule.edge_id == 75U;});
  check(first_summary != summary.end(), "missing whole-edge speed summary");
  checkNear(
    first_summary->effective_speed_limit_mps, 0.9, 1.0e-12,
    "whole-edge summary capped authored speed by generated default");
}

void testAlignedGeometryInterpolation()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {3.0, 4.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  lmmg::RouteEdge edge = makeEdge(
    3U, 1U, 2U, {{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {3.0, 4.0, 0.0}});
  // Use deliberately non-parallel values so the test verifies sampling by
  // centerline cross-section, rather than boundary arc length.
  edge.left_boundary = {{0.0, 1.0, 0.0}, {2.0, 1.0, 0.0}, {2.0, 4.0, 0.0}};
  edge.right_boundary = {{0.0, -1.0, 0.0}, {4.0, -1.0, 0.0}, {4.0, 4.0, 0.0}};
  graph.edges = {edge};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    spanFeature(1U, lmmg::SemanticFeatureType::kSpeedLimit, 3U, 2.0, 5.0, 0.3)};
  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  const lmmg::RouteEdge & middle = findDerivedEdge(result, 3U, 2.0, 5.0);
  check(middle.centerline.size() == 3U, "interior centerline vertex was lost");
  checkNear(middle.centerline.front().x, 2.0, 1.0e-12, "slice start x");
  checkNear(middle.centerline.back().x, 3.0, 1.0e-12, "slice end x");
  checkNear(middle.centerline.back().y, 2.0, 1.0e-12, "slice end y");
  checkNear(middle.left_boundary.front().x, 4.0 / 3.0, 1.0e-12, "left start interpolation");
  checkNear(middle.left_boundary.back().y, 2.5, 1.0e-12, "left end interpolation");
  checkNear(middle.right_boundary.back().y, 1.5, 1.0e-12, "right end interpolation");
}

void testAutowareSemanticBreakSnapsWithoutShorteningRawRoute()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {makeEdge(
      3U, 1U, 2U,
      {{0.0, 0.0, 0.0}, {2.0005, 0.0, 0.0}, {10.0, 0.0, 0.0}})};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    spanFeature(
      1U, lmmg::SemanticFeatureType::kSpeedLimit, 3U, 2.0, 5.0, 0.25)};
  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  const lmmg::LosslessSemanticRouteGraphAudit audit =
    lmmg::validateLosslessSemanticRouteGraph(graph, result);

  const lmmg::RouteEdge & speed = findDerivedEdge(result, 3U, 2.0005, 5.0);
  checkNear(speed.centerline.front().x, 2.0005, 1.0e-12,
    "semantic break was not snapped to the observed source vertex");
  checkNear(audit.source_length, 10.0, 1.0e-12,
    "semantic snap changed source Route length");
  checkNear(audit.output_length, 10.0, 1.0e-12,
    "semantic snap shortened output Route length");
  for (const lmmg::RouteEdge & edge : result.graph.edges) {
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      check(lmmg::distance3d(edge.centerline[index - 1U], edge.centerline[index]) >= 0.001,
        "semantic split retained a sub-millimetre centerline segment");
    }
  }
}

void testAutowareSpacingGateDoesNotBlockGenericTopologyMaterialization()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {1.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {makeEdge(
      3U, 1U, 2U,
      {{0.0, 0.0, 0.0}, {0.0005, 0.0, 0.0}, {1.0, 0.0, 0.0}})};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  check(result.graph.edges.size() == 1U,
    "generic topology semantic materialization unexpectedly dropped an Edge");

  bool rejected_by_autoware_gate = false;
  try {
    (void)lmmg::validateLosslessSemanticRouteGraph(graph, result);
  } catch (const std::invalid_argument & exception) {
    rejected_by_autoware_gate =
      std::string{exception.what()}.find("1 mm overlap threshold") != std::string::npos;
  }
  check(rejected_by_autoware_gate,
    "explicit Autoware lossless gate accepted a sub-millimetre centerline segment");
}

void testReversePairsShareSplitNodes()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  lmmg::RouteEdge forward = makeEdge(10U, 1U, 2U, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}});
  lmmg::RouteEdge reverse = forward;
  forward.reverse_of = 11U;
  reverse.id = 11U;
  reverse.from = 2U;
  reverse.to = 1U;
  reverse.reverse_of = 10U;
  std::reverse(reverse.centerline.begin(), reverse.centerline.end());
  std::reverse(reverse.left_boundary.begin(), reverse.left_boundary.end());
  std::reverse(reverse.right_boundary.begin(), reverse.right_boundary.end());
  std::reverse(reverse.left_clearance.begin(), reverse.left_clearance.end());
  std::reverse(reverse.right_clearance.begin(), reverse.right_clearance.end());
  std::reverse(reverse.left_clearance_observed.begin(), reverse.left_clearance_observed.end());
  std::reverse(reverse.right_clearance_observed.begin(), reverse.right_clearance_observed.end());
  graph.edges = {forward, reverse};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    spanFeature(1U, lmmg::SemanticFeatureType::kSpeedLimit, 10U, 2.0, 5.0, 0.25)};
  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  check(result.graph.edges.size() == 6U, "reverse edge was not synchronized at semantic splits");
  check(result.graph.nodes.size() == 4U, "forward/reverse splits did not share nodes");

  const lmmg::RouteEdge & forward_middle = findDerivedEdge(result, 10U, 2.0, 5.0);
  const lmmg::RouteEdge & reverse_middle = findDerivedEdge(result, 11U, 5.0, 8.0);
  check(forward_middle.reverse_of == reverse_middle.id, "forward reverse_of was not restored");
  check(reverse_middle.reverse_of == forward_middle.id, "reverse reverse_of was not restored");
  check(
    forward_middle.from == reverse_middle.to && forward_middle.to == reverse_middle.from,
    "reverse segments do not share split RouteNodes");
  checkNear(forward_middle.recommended_speed_mps, 0.25, 1.0e-12, "forward speed override");
  checkNear(reverse_middle.recommended_speed_mps, 1.0, 1.0e-12, "directed speed leaked to reverse");
}

void testOneFeatureSpansConnectedEdges()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {5.0, 0.0, 0.0}, lmmg::RouteNodeType::kNormal},
    {3U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {
    makeEdge(4U, 1U, 2U, {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}}),
    makeEdge(5U, 2U, 3U, {{5.0, 0.0, 0.0}, {10.0, 0.0, 0.0}})};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  lmmg::SemanticFeature speed;
  speed.id = 9U;
  speed.type = lmmg::SemanticFeatureType::kSpeedLimit;
  speed.geometry = lmmg::SemanticGeometryType::kRouteEdges;
  speed.value = 0.33;
  speed.route_edge_ids = {4U, 5U};
  speed.route_edge_spans = {
    {4U, 3.0, 5.0, std::nullopt, std::nullopt},
    {5U, 0.0, 2.0, std::nullopt, std::nullopt}};
  semantics.features.push_back(speed);

  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  const lmmg::RouteEdge & first = findDerivedEdge(result, 4U, 3.0, 5.0);
  const lmmg::RouteEdge & second = findDerivedEdge(result, 5U, 0.0, 2.0);
  check(first.to == 2U && second.from == 2U, "multi-edge semantic lost source connectivity");
  checkNear(first.recommended_speed_mps, 0.33, 1.0e-12, "first edge span speed");
  checkNear(second.recommended_speed_mps, 0.33, 1.0e-12, "second edge span speed");
  const auto provenance = std::find_if(
    result.edge_provenance.begin(), result.edge_provenance.end(),
    [&](const lmmg::SemanticRouteEdgeProvenance & value) {
      return value.edge_id == first.id;
    });
  check(
    provenance != result.edge_provenance.end() &&
    provenance->source_feature_ids == std::vector<std::uint64_t>{9U},
    "multi-edge semantic provenance was not retained");
}

void testSemanticLaneletOsmIntegration()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {makeEdge(3U, 1U, 2U, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}})};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    spanFeature(1U, lmmg::SemanticFeatureType::kSpeedLimit, 3U, 2.0, 5.0, 0.25),
    spanFeature(2U, lmmg::SemanticFeatureType::kNoEntry, 3U, 6.0, 7.0)};
  lmmg::SemanticRouteGraphOptions options;
  options.no_entry_policy = lmmg::SemanticNoEntryPolicy::kKeepImpassable;
  const lmmg::SemanticRouteGraphResult materialized =
    lmmg::materializeSemanticRouteGraph(graph, semantics, options);
  const lmmg::RouteEdge & speed_edge = findDerivedEdge(materialized, 3U, 2.0, 5.0);
  const lmmg::RouteEdge & no_entry_edge = findDerivedEdge(materialized, 3U, 6.0, 7.0);
  check(no_entry_edge.passable == false, "materialized no-entry edge remained passable");

  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path output = std::filesystem::temp_directory_path() /
    ("lmmg_semantic_lanelet_" + std::to_string(nonce) + ".osm");
  lmmg::Lanelet2Config config;
  config.speed_limit_mps = 1.0;
  lmmg::saveLanelet2Osm(output, materialized.graph, config);
  std::ifstream stream(output);
  const std::string osm{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  std::filesystem::remove(output);

  check(!osm.empty(), "semantic Lanelet2 integration export is empty");
  const std::string speed_relation = relationForRouteEdge(osm, speed_edge.id);
  check(!speed_relation.empty(), "speed-span child edge was not exported as a Lanelet");
  check(
    speed_relation.find("<tag k=\"speed_limit\" v=\"0.9\"/>") != std::string::npos,
    "speed-span Lanelet does not contain unitless 0.9 km/h");
  check(
    speed_relation.find("<tag k=\"generator_speed_limit_mps\" v=\"0.25\"/>") !=
    std::string::npos,
    "speed-span Lanelet lost its 0.25 m/s generator value");
  check(
    relationForRouteEdge(osm, no_entry_edge.id).empty(),
    "no-entry child edge was exported as a Lanelet relation");
  check(
    countOccurrences(osm, "<tag k=\"type\" v=\"lanelet\"/>") == 4U,
    "unexpected Lanelet relation count after excluding one no-entry interval");
}

void testUnsplitEdgeKeepsIdentity()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {2.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {makeEdge(3U, 1U, 2U, {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}})};
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  const lmmg::SemanticRouteGraphResult result =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  check(result.graph.edges.size() == 1U, "unrestricted edge count changed");
  check(result.graph.edges.front().id == 3U, "unrestricted edge identity changed");
  check(result.edge_provenance.front().source_edge_id == 3U, "unrestricted provenance missing");
}

void testLosslessAutowareSemanticLaneletSegmentation()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {makeEdge(
      3U, 1U, 2U,
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {4.0, 0.0, 0.0},
        {7.0, 0.0, 0.0}, {10.0, 0.0, 0.0}})};

  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  semantics.features = {
    spanFeature(1U, lmmg::SemanticFeatureType::kSpeedLimit, 3U, 2.0, 5.0, 0.25)};
  const lmmg::SemanticRouteGraphResult materialized =
    lmmg::materializeSemanticRouteGraph(graph, semantics);
  const lmmg::LosslessSemanticRouteGraphAudit audit =
    lmmg::validateLosslessSemanticRouteGraph(graph, materialized);
  check(audit.source_edges == 1U && audit.output_edges == 3U, "lossless split counts");
  checkNear(audit.source_length, audit.output_length, 1.0e-9, "lossless full arc");

  lmmg::NamedNavigationRoute raw_route;
  raw_route.id = 20U;
  raw_route.name = "full_raw_route";
  raw_route.target = lmmg::NavigationAuthoringTarget::kAutoware;
  raw_route.start_node_id = 1U;
  raw_route.end_node_id = 2U;
  raw_route.ordered_edge_ids = {3U};
  raw_route.validation_requested = true;
  raw_route.promotion_requested = true;
  const lmmg::NamedNavigationRoute semantic_route =
    lmmg::remapNamedNavigationRouteAfterSemantics(graph, raw_route, materialized);
  check(semantic_route.ordered_edge_ids.size() == 3U, "raw Mission was not expanded");

  lmmg::AuthoredStopLine raw_stop;
  raw_stop.id = 30U;
  raw_stop.name = "test_stop";
  raw_stop.edge_id = 3U;
  raw_stop.s = 3.0;
  raw_stop.width_m = 1.2;
  raw_stop.anchor = {3.0, 0.0, 0.0};
  raw_stop.target = lmmg::NavigationAuthoringTarget::kAutoware;
  const std::vector<lmmg::AuthoredStopLine> semantic_stops =
    lmmg::remapResolvedStopLinesAfterSemantics(graph, {raw_stop}, materialized);
  check(semantic_stops.size() == 1U, "semantic stop remap count");
  check(semantic_stops.front().edge_id == semantic_route.ordered_edge_ids[1],
    "semantic stop did not bind to speed-span Lanelet");
  checkNear(semantic_stops.front().s, 1.0, 1.0e-9, "semantic stop local arc");

  lmmg::Lanelet2AuthoringOptions authoring;
  authoring.named_route = &semantic_route;
  authoring.resolved_stop_lines = semantic_stops;
  authoring.semantic_source_graph = &graph;
  authoring.semantic_edge_provenance = &materialized.edge_provenance;
  lmmg::ClosedCourseLanelet2ExportOptions export_options;
  export_options.estimated_vehicle_width = 1.0;
  export_options.estimated_front_extent = 1.0;
  export_options.estimated_rear_extent = 0.5;
  export_options.estimated_minimum_turning_radius = 2.0;
  export_options.lateral_clearance_margin = 0.1;
  export_options.experimental_ready = true;
  lmmg::Lanelet2Config config;
  config.speed_limit_mps = 1.0;
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path output = std::filesystem::temp_directory_path() /
    ("lmmg_lossless_semantic_lanelet_" + std::to_string(nonce) + ".osm");
  const lmmg::ClosedCourseLanelet2ExportSummary summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    output, materialized.graph, config, export_options, authoring);
  std::ifstream stream(output);
  const std::string osm{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  std::filesystem::remove(output);
  check(summary.source_physical_edges == 1U, "raw physical source count changed");
  check(summary.exported_physical_edges == 1U, "raw physical coverage count changed");
  check(summary.exported_lanelet_segments == 3U, "semantic Lanelet count is incorrect");
  check(countOccurrences(osm, "<tag k=\"type\" v=\"lanelet\"/>") == 3U,
    "partial speed span did not produce three Lanelets");
  check(countOccurrences(osm, "<tag k=\"named_route_id\" v=\"20\"") == 3U,
    "semantic named Mission did not tag every Lanelet child");
  for (std::size_t index = 0U; index < semantic_route.ordered_edge_ids.size(); ++index) {
    const std::string relation = relationForRouteEdge(
      osm, semantic_route.ordered_edge_ids[index]);
    check(
      relation.find(
        "<tag k=\"named_route_order\" v=\"" + std::to_string(index) + "\"/>") !=
      std::string::npos,
      "semantic named Mission Lanelet order is not global and consecutive");
  }
  const lmmg::RouteEdge & speed_edge = findDerivedEdge(materialized, 3U, 2.0, 5.0);
  const std::string speed_relation = relationForRouteEdge(osm, speed_edge.id);
  check(speed_relation.find("<tag k=\"generator_speed_limit_mps\" v=\"0.25\"/>") !=
    std::string::npos, "partial speed Lanelet lost its effective limit");
  check(speed_relation.find("<tag k=\"source_route_edge_id\" v=\"3\"/>") !=
    std::string::npos, "partial speed Lanelet lost source Edge lineage");
  check(speed_relation.find("<tag k=\"source_start_s_m\" v=\"2\"/>") !=
    std::string::npos, "partial speed Lanelet lost source start arc");
  check(speed_relation.find("<tag k=\"source_end_s_m\" v=\"5\"/>") !=
    std::string::npos, "partial speed Lanelet lost source end arc");
  check(speed_relation.find("<tag k=\"source_edge_length_m\" v=\"10\"/>") !=
    std::string::npos, "partial speed Lanelet lost source 3-D arc length");
  const lmmg::RouteEdge & before = findDerivedEdge(materialized, 3U, 0.0, 2.0);
  const lmmg::RouteEdge & after = findDerivedEdge(materialized, 3U, 5.0, 10.0);
  for (const std::uint64_t edge_id : {before.id, after.id}) {
    check(relationForRouteEdge(osm, edge_id).find(
        "<tag k=\"generator_speed_limit_mps\" v=\"1\"/>") != std::string::npos,
      "outside speed span did not retain the default speed");
  }
  check(osm.find("<tag k=\"source_route_edge_s_m\" v=\"3\"/>") !=
    std::string::npos, "semantic stop line lost its raw source arc");

  lmmg::SemanticMap empty_semantics;
  empty_semantics.frame_id = "map";
  const lmmg::SemanticRouteGraphResult unsplit =
    lmmg::materializeSemanticRouteGraph(graph, empty_semantics);
  const lmmg::LosslessSemanticRouteGraphAudit unsplit_audit =
    lmmg::validateLosslessSemanticRouteGraph(graph, unsplit);
  check(unsplit_audit.output_edges == 1U && unsplit.graph.edges.front().id == 3U,
    "no-semantic replay did not remain exact 1:1");

  lmmg::SemanticRouteGraphResult tampered = materialized;
  tampered.edge_provenance[1].source_start_s += 0.1;
  checkThrows(
    [&]() {(void)lmmg::validateLosslessSemanticRouteGraph(graph, tampered);},
    "semantic provenance gap was accepted");
  tampered = materialized;
  tampered.edge_provenance[1].source_start_s -= 0.1;
  checkThrows(
    [&]() {(void)lmmg::validateLosslessSemanticRouteGraph(graph, tampered);},
    "semantic provenance overlap was accepted");
  tampered = materialized;
  std::swap(tampered.edge_provenance[0], tampered.edge_provenance[1]);
  checkThrows(
    [&]() {(void)lmmg::validateLosslessSemanticRouteGraph(graph, tampered);},
    "semantic provenance reorder was accepted");
  tampered = materialized;
  tampered.graph.edges[1].centerline.back().x += 0.2;
  checkThrows(
    [&]() {(void)lmmg::validateLosslessSemanticRouteGraph(graph, tampered);},
    "semantic segment geometry/length tamper was accepted");
}

void testExactReplaySemanticCoordinateRecognition()
{
  lmmg::RouteGraph replay;
  replay.frame_id = "map";
  replay.nodes = {
    lmmg::RouteNode{1U, {0.0, 0.0, 0.0}},
    lmmg::RouteNode{2U, {5.0, 0.0, 0.0}}};
  replay.edges.push_back(makeEdge(65U, 1U, 2U, {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}}));

  constexpr double start_s = 0.605574963596;
  constexpr double seam_s = 2.64556809788;
  lmmg::SemanticMap semantics;
  semantics.frame_id = "map";
  lmmg::SemanticFeature speed = spanFeature(
    1U, lmmg::SemanticFeatureType::kSpeedLimit, 65U, start_s, seam_s, 0.9);
  speed.route_edge_spans.front().start_anchor = lmmg::Vec3{start_s, 0.0, 0.0};
  speed.route_edge_spans.front().end_anchor = lmmg::Vec3{seam_s, 0.0, 0.0};
  semantics.features.push_back(speed);

  check(
    lmmg::semanticRouteSpansUseGraphCoordinates(semantics, replay),
    "exact replay-authored semantic span was not recognized");
  const lmmg::SemanticRouteGraphResult materialized =
    lmmg::materializeSemanticRouteGraph(replay, semantics);
  const auto speed_provenance = std::find_if(
    materialized.edge_provenance.begin(), materialized.edge_provenance.end(),
    [&](const lmmg::SemanticRouteEdgeProvenance & value) {
      return value.source_start_s == start_s && value.source_end_s == seam_s;
    });
  check(
    speed_provenance != materialized.edge_provenance.end(),
    "exact authored replay decimals moved during semantic segmentation");

  lmmg::RouteGraph differently_partitioned = replay;
  differently_partitioned.nodes[0].position.x = 2.0;
  differently_partitioned.nodes[1].position.x = 7.0;
  differently_partitioned.edges[0] = makeEdge(
    65U, 1U, 2U, {{2.0, 0.0, 0.0}, {7.0, 0.0, 0.0}});
  check(
    !lmmg::semanticRouteSpansUseGraphCoordinates(semantics, differently_partitioned),
    "reused Edge ID on different geometry was mistaken for replay coordinates");

  lmmg::SemanticMap mismatched_anchor = semantics;
  mismatched_anchor.features.front().route_edge_spans.front().start_anchor->x += 2.0e-6;
  check(
    !lmmg::semanticRouteSpansUseGraphCoordinates(mismatched_anchor, replay),
    "span with a mismatched anchor was mistaken for replay coordinates");
}

}  // namespace

int main()
{
  try {
    testExactSpeedAndNoEntrySplits();
    testAuthoredSpeedMayExceedGeneratedDefaultWithoutBoundaryGap();
    testAlignedGeometryInterpolation();
    testAutowareSemanticBreakSnapsWithoutShorteningRawRoute();
    testAutowareSpacingGateDoesNotBlockGenericTopologyMaterialization();
    testReversePairsShareSplitNodes();
    testOneFeatureSpansConnectedEdges();
    testSemanticLaneletOsmIntegration();
    testUnsplitEdgeKeepsIdentity();
    testLosslessAutowareSemanticLaneletSegmentation();
    testExactReplaySemanticCoordinateRecognition();
    std::cout << "semantic route graph tests passed\n";
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "semantic route graph test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
