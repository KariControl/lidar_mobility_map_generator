#include "lidar_mobility_map_generator/observed_route_graph.hpp"
#include "lidar_mobility_map_generator/exporters.hpp"
#include "lidar_mobility_map_generator/review_io.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
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
            message + ": actual=" + std::to_string(actual) +
            " expected=" + std::to_string(expected));
  }
}

std::vector<lmmg::TimedPose> poses(const std::vector<lmmg::Vec3> & positions)
{
  std::vector<lmmg::TimedPose> result;
  result.reserve(positions.size());
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    lmmg::TimedPose pose;
    pose.stamp_ns = static_cast<std::int64_t>(index) * 100000000LL;
    pose.world_from_body.translation = positions[index];
    result.push_back(pose);
  }
  return result;
}

std::vector<lmmg::TimedPose> terminalTailPoses(
  const double prefix_length, const double tail_length,
  const double terminal_yaw_change_deg = 0.0)
{
  std::vector<lmmg::TimedPose> result = poses({
    {0.0, 0.0, 0.0},
    {0.5 * prefix_length, 0.0, 0.0},
    {prefix_length, 0.0, 0.0},
    {prefix_length - tail_length, 0.0, 0.0}});
  result.back().world_from_body.rotation = lmmg::Quaternion::fromYaw(
    terminal_yaw_change_deg * lmmg::kPi / 180.0);
  return result;
}

lmmg::RouteGraph terminalTailGraph(
  const std::vector<lmmg::TimedPose> & trajectory)
{
  lmmg::TopologyConfig config;
  config.maximum_edge_length = 20.0;
  config.edge_split_heading_change_deg = 75.0;
  config.cusp_heading_change_deg = 120.0;
  return lmmg::buildObservedDrivenRouteGraph(
    trajectory, config, "map", 0.25);
}

double graphLength(const lmmg::RouteGraph & graph)
{
  double result = 0.0;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    result += edge.length;
  }
  return result;
}

double planarGraphLength(const lmmg::RouteGraph & graph)
{
  double result = 0.0;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      result += lmmg::distance2d(
        edge.centerline[index - 1U], edge.centerline[index]);
    }
  }
  return result;
}

void checkSourceVerticesRetainedInOrder(
  const lmmg::RouteGraph & graph, const std::vector<lmmg::Vec3> & source,
  const std::string & context)
{
  std::vector<lmmg::Vec3> reconstructed;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    const auto begin = reconstructed.empty() ?
      edge.centerline.begin() : std::next(edge.centerline.begin());
    reconstructed.insert(reconstructed.end(), begin, edge.centerline.end());
  }
  std::size_t source_index = 0U;
  for (const lmmg::Vec3 & point : reconstructed) {
    if (source_index < source.size() &&
      lmmg::distance3d(point, source[source_index]) <= 1.0e-12)
    {
      ++source_index;
    }
  }
  check(source_index == source.size(),
    context + " did not retain every processed trajectory vertex in order");
}

void checkSequentialChain(const lmmg::RouteGraph & graph)
{
  check(graph.nodes.size() == graph.edges.size() + 1U, "replay graph is not a chain");
  check(graph.nodes.front().type == lmmg::RouteNodeType::kEndpoint,
    "replay graph start is not an endpoint");
  check(graph.nodes.back().type == lmmg::RouteNodeType::kEndpoint,
    "replay graph end is not an endpoint");
  for (std::size_t index = 0U; index < graph.edges.size(); ++index) {
    const lmmg::RouteEdge & edge = graph.edges[index];
    check(edge.from == graph.nodes[index].id, "edge source is out of sequence");
    check(edge.to == graph.nodes[index + 1U].id, "edge target is out of sequence");
    check(!edge.reverse_of.has_value(), "replay graph unexpectedly generated a reverse edge");
    check(edge.passable, "observed replay edge is not retained as passable");
    checkNear(edge.confidence, 0.0, 0.0, "unvalidated replay edge confidence is not zero");
    check(!edge.corridor_geometry_valid,
      "observed replay edge unexpectedly claims valid safety boundaries");
    check(edge.validation_errors.size() == 1U &&
      edge.validation_errors.front() == lmmg::kObservedDrivenRouteDiagnostic,
      "observed replay provenance diagnostic is missing");
    check(lmmg::distance3d(edge.centerline.front(), graph.nodes[index].position) < 1.0e-12,
      "edge start geometry does not match its node");
    check(lmmg::distance3d(edge.centerline.back(), graph.nodes[index + 1U].position) < 1.0e-12,
      "edge end geometry does not match its node");
    if (index > 0U) {
      check(graph.edges[index - 1U].to == edge.from,
        "successive replay edges are disconnected");
    }
  }
}

void testFullLengthAndDiagnostics()
{
  const std::vector<lmmg::Vec3> source{
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 2.0, 0.0}, {4.0, 2.0, 0.0}};
  lmmg::TopologyConfig config;
  config.maximum_edge_length = 100.0;
  config.edge_split_heading_change_deg = 170.0;
  config.cusp_heading_change_deg = 180.0;
  config.generate_reverse_edges = true;
  const lmmg::RouteGraph graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), config, "odom", 0.25);

  check(graph.frame_id == "odom", "observed replay frame was not retained");
  check(graph.edges.size() == 1U, "ordinary corners caused an unexpected replay split");
  checkSequentialChain(graph);
  checkNear(graphLength(graph), lmmg::polylineLength(source), 1.0e-12,
    "observed replay did not preserve full length");
  check(graph.edges.front().centerline.size() == source.size(),
    "unsplit replay did not retain source vertices");
  checkNear(graph.edges.front().recommended_speed_mps, 0.25, 0.0,
    "replay speed was not retained");
}

void testMaximumEdgeLength()
{
  const std::vector<lmmg::Vec3> source{{0.0, 0.0, 0.0}, {10.5, 0.0, 0.0}};
  lmmg::TopologyConfig config;
  config.minimum_edge_length = 9.0;  // Must not discard replay coverage.
  config.maximum_edge_length = 2.0;
  const lmmg::RouteGraph graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), config, "map", 0.1);

  check(graph.edges.size() == 6U, "long replay span was not split at maximum length");
  checkSequentialChain(graph);
  for (const lmmg::RouteEdge & edge : graph.edges) {
    check(edge.length <= config.maximum_edge_length + 1.0e-12,
      "replay edge exceeds configured maximum length");
  }
  checkNear(graphLength(graph), 10.5, 1.0e-12,
    "maximum-length splitting changed total route length");
}

void testFiveAndTwentyMetrePartitionsRetainCompleteTrajectory()
{
  // These are the two fixed map_ws contracts: 5 m for MID360 and 20 m for
  // automotive replay.  Changing the limit may only add/remove artificial
  // Edge joints; it must not simplify the processed trajectory or its arc.
  const std::vector<lmmg::Vec3> source{
    {0.0, 0.0, 0.00}, {3.2, 0.1, 0.05}, {7.6, 0.4, 0.10},
    {12.1, 0.2, 0.15}, {17.0, 0.5, 0.20}, {22.4, 0.3, 0.25},
    {28.0, 0.6, 0.30}};
  lmmg::TopologyConfig five_metre;
  five_metre.maximum_edge_length = 5.0;
  five_metre.edge_split_heading_change_deg = 170.0;
  five_metre.cusp_heading_change_deg = 180.0;
  lmmg::TopologyConfig twenty_metre = five_metre;
  twenty_metre.maximum_edge_length = 20.0;

  const lmmg::RouteGraph five_graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), five_metre, "map", 0.1);
  const lmmg::RouteGraph twenty_graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), twenty_metre, "map", 0.1);

  check(five_graph.edges.size() > twenty_graph.edges.size(),
    "5 m and 20 m contracts did not change only the expected partition count");
  for (const auto & value : {
      std::pair<const lmmg::RouteGraph *, double>{&five_graph, 5.0},
      std::pair<const lmmg::RouteGraph *, double>{&twenty_graph, 20.0}})
  {
    checkSequentialChain(*value.first);
    for (const lmmg::RouteEdge & edge : value.first->edges) {
      double planar_edge_length = 0.0;
      for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
        planar_edge_length += lmmg::distance2d(
          edge.centerline[index - 1U], edge.centerline[index]);
      }
      check(planar_edge_length <= value.second + 1.0e-9,
        "fixed map_ws partition exceeded maximum_edge_length");
    }
  }
  checkSourceVerticesRetainedInOrder(five_graph, source, "5 m partition");
  checkSourceVerticesRetainedInOrder(twenty_graph, source, "20 m partition");
  checkNear(graphLength(five_graph), lmmg::polylineLength(source), 1.0e-12,
    "5 m partition changed complete spatial trajectory length");
  checkNear(graphLength(twenty_graph), lmmg::polylineLength(source), 1.0e-12,
    "20 m partition changed complete spatial trajectory length");
  checkNear(planarGraphLength(five_graph), planarGraphLength(twenty_graph), 1.0e-12,
    "5 m and 20 m partitions changed planar trajectory coverage");
}

void testPlanarPartitionIgnoresElevation()
{
  // A large but finite elevation change must be reported by the geometry
  // audit, not converted into extra raw Edge IDs by the partitioner.
  const std::vector<lmmg::Vec3> source{
    {0.0, 0.0, 0.0}, {4.9, 0.0, 0.0}, {5.1, 0.0, 100.0}};
  lmmg::TopologyConfig config;
  config.maximum_edge_length = 5.0;
  config.edge_split_heading_change_deg = 170.0;
  config.cusp_heading_change_deg = 180.0;
  const lmmg::RouteGraph graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), config, "map", 0.1);

  check(graph.edges.size() == 2U,
    "elevation inflated the planar raw Edge partition count");
  checkSequentialChain(graph);
  checkNear(graphLength(graph), lmmg::polylineLength(source), 1.0e-9,
    "planar partitioning changed the complete spatial source geometry");
  double planar_length = 0.0;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      planar_length += lmmg::distance2d(
        edge.centerline[index - 1U], edge.centerline[index]);
    }
  }
  checkNear(planar_length, 5.1, 1.0e-12,
    "planar partitioning changed the complete XY source interval");
}

void testPartitionSnapsSubMillimetreAutowareJointWithoutLosingRoute()
{
  // The nominal 4.95 m cut lies 0.5 mm before an observed source vertex.
  // Leaving both points in the next Lanelet makes Autoware's 1 mm overlap
  // filter duplicate its Lanelet ID.  The cut must move to the observed
  // vertex while the complete 0..9.9 m source arc and Edge order remain.
  const std::vector<lmmg::Vec3> source{
    {0.0, 0.0, 0.0}, {4.9505, 0.0, 0.0}, {9.9, 0.0, 0.0}};
  lmmg::TopologyConfig config;
  config.maximum_edge_length = 5.0;
  config.edge_split_heading_change_deg = 170.0;
  config.cusp_heading_change_deg = 180.0;
  const lmmg::RouteGraph graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), config, "map", 0.1);

  check(graph.edges.size() == 2U, "Autoware joint snap changed raw Edge count");
  checkSequentialChain(graph);
  checkNear(graph.edges.front().centerline.back().x, 4.9505, 1.0e-12,
    "raw Edge cut was not snapped to the source vertex");
  checkNear(graph.edges.back().centerline.front().x, 4.9505, 1.0e-12,
    "successor did not start at the snapped shared vertex");
  checkNear(graphLength(graph), 9.9, 1.0e-12,
    "Autoware-compatible cut snapping shortened the raw Route");
  for (const lmmg::RouteEdge & edge : graph.edges) {
    check(edge.length <= config.maximum_edge_length + 1.0e-12,
      "Autoware-compatible cut exceeds maximum_edge_length");
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      check(lmmg::distance3d(edge.centerline[index - 1U], edge.centerline[index]) >= 0.001,
        "sub-millimetre centerline segment survived raw partition snapping");
    }
  }
}

void testZeroHorizontalDistanceZChangeFailsClosed()
{
  lmmg::TopologyConfig config;
  bool threw = false;
  try {
    static_cast<void>(lmmg::buildObservedDrivenRouteGraph(
        poses({{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}}),
        config, "map", 0.1));
  } catch (const std::runtime_error & error) {
    threw = std::string(error.what()).find("zero horizontal distance") != std::string::npos;
  }
  check(threw, "zero-horizontal-distance Z change did not fail closed");
}

void testLoopCrossingAndRevisitPreservation()
{
  // The start is revisited and the first segment is traversed again.  A
  // topology builder may merge these visits; a replay builder must not.
  const std::vector<lmmg::Vec3> source{
    {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0},
    {0.0, 2.0, 0.0}, {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  lmmg::TopologyConfig config;
  config.maximum_edge_length = 100.0;
  config.edge_split_heading_change_deg = 170.0;
  config.cusp_heading_change_deg = 180.0;
  const lmmg::RouteGraph graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), config, "map", 0.2);

  checkSequentialChain(graph);
  check(graph.edges.size() == 1U, "revisit was topology-merged or unexpectedly split");
  check(graph.edges.front().centerline.size() == source.size(),
    "loop/revisit vertices were not preserved");
  for (std::size_t index = 0U; index < source.size(); ++index) {
    check(lmmg::distance3d(graph.edges.front().centerline[index], source[index]) < 1.0e-12,
      "loop/revisit observation order changed");
  }
  checkNear(graphLength(graph), lmmg::polylineLength(source), 1.0e-12,
    "loop/revisit length was collapsed");

  const std::vector<lmmg::Vec3> closed_source{
    {-1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0},
    {1.0, -1.0, 0.0}, {-1.0, -1.0, 0.0}};
  const lmmg::RouteGraph closed_graph = lmmg::buildObservedDrivenRouteGraph(
    poses(closed_source), config, "map", 0.2);
  checkSequentialChain(closed_graph);
  check(closed_graph.nodes.front().id != closed_graph.nodes.back().id,
    "closed replay was collapsed into a graph self-loop");
  check(lmmg::distance3d(
      closed_graph.nodes.front().position, closed_graph.nodes.back().position) < 1.0e-12,
    "closed replay did not retain its coincident terminal visit");
  checkNear(graphLength(closed_graph), lmmg::polylineLength(closed_source), 1.0e-12,
    "self-crossing closed replay length was collapsed");
}

void testSharpAndReversalSplits()
{
  const std::vector<lmmg::Vec3> source{
    {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  lmmg::TopologyConfig config;
  config.maximum_edge_length = 100.0;
  config.edge_split_heading_change_deg = 75.0;
  config.cusp_heading_change_deg = 120.0;
  const lmmg::RouteGraph graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), config, "map", 0.1);

  checkSequentialChain(graph);
  check(graph.edges.size() == 2U, "direction reversal was not made an edge boundary");
  checkNear(graph.edges[0U].length, 2.0, 1.0e-12, "reversal incoming length changed");
  checkNear(graph.edges[1U].length, 2.0, 1.0e-12, "reversal outgoing length changed");
  check(graph.nodes.front().id != graph.nodes.back().id,
    "out-and-back visits were merged into one node");
  check(lmmg::distance3d(
      graph.nodes.front().position, graph.nodes.back().position) < 1.0e-12,
    "out-and-back endpoint position changed");
}

void testAdjacentSharpTurnsRetainLosslessRawEdgesAndExportLanelets()
{
  const std::vector<lmmg::Vec3> source{
    {0.0, 0.0, 0.0}, {0.0, -8.0, 0.0},
    {-0.1, -7.98, 0.0}, {-0.1, -48.0, 0.0}};
  lmmg::TopologyConfig config;
  config.minimum_edge_length = 0.5;
  config.maximum_edge_length = 5.0;
  config.edge_split_heading_change_deg = 75.0;
  config.cusp_heading_change_deg = 120.0;
  const lmmg::RouteGraph graph = lmmg::buildObservedDrivenRouteGraph(
    poses(source), config, "map", 0.1);

  checkSequentialChain(graph);
  check(graph.edges.size() == 12U,
    "adjacent sharp turns did not retain all raw replay Edges");
  checkNear(graphLength(graph), lmmg::polylineLength(source), 1.0e-12,
    "adjacent sharp turns changed full replay length");
  std::size_t short_edges = 0U;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    if (edge.length < config.minimum_edge_length) {
      ++short_edges;
    }
    check(edge.length <= config.maximum_edge_length + 1.0e-12,
      "adjacent sharp turn exceeded maximum Edge length");
  }
  check(short_edges == 1U,
    "lossless replay did not retain exactly one observed subminimum Edge");
  std::vector<lmmg::Vec3> reconstructed;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    if (reconstructed.empty()) {
      reconstructed = edge.centerline;
    } else {
      reconstructed.insert(
        reconstructed.end(), edge.centerline.begin() + 1, edge.centerline.end());
    }
  }
  for (const lmmg::Vec3 & point : source) {
    check(std::any_of(
        reconstructed.begin(), reconstructed.end(), [&](const lmmg::Vec3 & candidate) {
          return lmmg::distance3d(point, candidate) <= 1.0e-12;
        }),
      "adjacent sharp-turn replay removed an observed trajectory vertex");
  }

  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 1.8;
  options.estimated_front_extent = 3.2;
  options.estimated_rear_extent = 1.0;
  options.estimated_minimum_turning_radius = 5.0;
  options.lateral_clearance_margin = 0.15;
  options.experimental_ready = true;
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("lmmg_adjacent_sharp_turns_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".osm");
  const lmmg::ClosedCourseLanelet2ExportSummary summary =
    lmmg::saveClosedCourseExperimentalLanelet2Osm(
    path, graph, lmmg::Lanelet2Config{}, options);
  const std::vector<lmmg::Lanelet2ReviewLanelet> lanelets =
    lmmg::loadGeneratedLanelet2Osm(path);
  std::filesystem::remove(path);
  check(summary.source_physical_edges == 12U &&
    summary.exported_physical_edges == 12U && lanelets.size() == 12U,
    "adjacent sharp-turn replay did not export all lossless Lanelets");
  checkNear(summary.source_length, lmmg::polylineLength(source), 1.0e-12,
    "Lanelet export changed adjacent sharp-turn source length");
  checkNear(summary.exported_length, summary.source_length, 1.0e-12,
    "Lanelet export shortened adjacent sharp-turn replay");
}

void testVerifiedTerminalLocalizationSettlingTrim()
{
  const std::vector<lmmg::TimedPose> trajectory = terminalTailPoses(100.0, 0.5);
  const lmmg::RouteGraph full = terminalTailGraph(trajectory);
  const lmmg::AutowareReplayCandidateResult candidate =
    lmmg::materializeAutowareReplayCandidate(full, trajectory, true);

  check(candidate.explicit_verification, "explicit settling verification was not recorded");
  check(candidate.terminal_tail_omitted, "verified valid terminal tail was not omitted");
  check(
    candidate.reason == "verified_terminal_localization_settling_tail_omitted",
    "terminal settling trim reason is missing");
  checkNear(candidate.source_length, 100.5, 1.0e-9, "source length changed");
  checkNear(candidate.retained_length, 100.0, 1.0e-9, "retained length is wrong");
  checkNear(candidate.omitted_length, 0.5, 1.0e-9, "omitted length is wrong");
  checkNear(
    candidate.omitted_length_ratio, 0.5 / 100.5, 1.0e-12,
    "omitted ratio is wrong");
  checkNear(
    candidate.connection_heading_jump_deg, 180.0, 1.0e-9,
    "cusp heading jump is wrong");
  checkNear(
    candidate.terminal_body_yaw_change_deg, 0.0, 1.0e-12,
    "stationary body yaw unexpectedly changed");
  check(candidate.graph.edges.size() + candidate.omitted_edges == full.edges.size(),
    "trimmed edge accounting is inconsistent");
  check(candidate.graph.nodes.size() == candidate.graph.edges.size() + 1U,
    "trimmed Autoware candidate is not a chain");
  check(candidate.graph.nodes.back().type == lmmg::RouteNodeType::kEndpoint,
    "trimmed cusp node was not changed to a terminal endpoint");
  checkNear(graphLength(full), 100.5, 1.0e-9,
    "materializing the Autoware derivative mutated the lossless graph");
}

void testTerminalLocalizationSettlingFailuresRetainFullReplay()
{
  const auto check_retained = [](
      const lmmg::RouteGraph & full,
      const lmmg::AutowareReplayCandidateResult & candidate,
      const std::string & reason) {
      check(!candidate.terminal_tail_omitted, reason + " unexpectedly trimmed the graph");
      check(candidate.reason == reason, "unexpected fail-closed reason: " + candidate.reason);
      check(candidate.graph.edges.size() == full.edges.size(),
        reason + " did not retain every edge");
      checkNear(graphLength(candidate.graph), graphLength(full), 1.0e-9,
        reason + " changed full replay length");
    };

  const std::vector<lmmg::TimedPose> valid_trajectory = terminalTailPoses(100.0, 0.5);
  const lmmg::RouteGraph valid_graph = terminalTailGraph(valid_trajectory);
  check_retained(
    valid_graph,
    lmmg::materializeAutowareReplayCandidate(valid_graph, valid_trajectory, false),
    "explicit_verification_not_provided");

  const std::vector<lmmg::TimedPose> straight = poses({
    {0.0, 0.0, 0.0}, {50.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
  const lmmg::RouteGraph straight_graph = terminalTailGraph(straight);
  check_retained(
    straight_graph,
    lmmg::materializeAutowareReplayCandidate(straight_graph, straight, true),
    "no_terminal_cusp");

  const std::vector<lmmg::TimedPose> long_tail = terminalTailPoses(200.0, 1.1);
  const lmmg::RouteGraph long_tail_graph = terminalTailGraph(long_tail);
  check_retained(
    long_tail_graph,
    lmmg::materializeAutowareReplayCandidate(long_tail_graph, long_tail, true),
    "terminal_tail_exceeds_1m");

  const std::vector<lmmg::TimedPose> excessive_ratio = terminalTailPoses(20.0, 0.5);
  const lmmg::RouteGraph excessive_ratio_graph = terminalTailGraph(excessive_ratio);
  check_retained(
    excessive_ratio_graph,
    lmmg::materializeAutowareReplayCandidate(
      excessive_ratio_graph, excessive_ratio, true),
    "terminal_tail_exceeds_1_percent");

  const std::vector<lmmg::TimedPose> yaw_change = terminalTailPoses(100.0, 0.5, 20.0);
  const lmmg::RouteGraph yaw_change_graph = terminalTailGraph(yaw_change);
  check_retained(
    yaw_change_graph,
    lmmg::materializeAutowareReplayCandidate(yaw_change_graph, yaw_change, true),
    "terminal_tail_body_yaw_change_exceeds_15deg");
}

}  // namespace

int main()
{
  testFullLengthAndDiagnostics();
  testMaximumEdgeLength();
  testFiveAndTwentyMetrePartitionsRetainCompleteTrajectory();
  testPlanarPartitionIgnoresElevation();
  testPartitionSnapsSubMillimetreAutowareJointWithoutLosingRoute();
  testZeroHorizontalDistanceZChangeFailsClosed();
  testLoopCrossingAndRevisitPreservation();
  testSharpAndReversalSplits();
  testAdjacentSharpTurnsRetainLosslessRawEdgesAndExportLanelets();
  testVerifiedTerminalLocalizationSettlingTrim();
  testTerminalLocalizationSettlingFailuresRetainFullReplay();
  return 0;
}
