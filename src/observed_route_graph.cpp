#include "lidar_mobility_map_generator/observed_route_graph.hpp"

#include "lidar_mobility_map_generator/route_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

constexpr double kLengthEpsilon = 1.0e-12;
// Autoware Core 1.9 RouteHandler::removeOverlappingPoints() merges points
// whose 3-D separation is below 1 mm.  If an equal-length raw-Edge cut is
// placed just before/after an existing trajectory vertex, the outgoing or
// incoming Lanelet otherwise starts/ends with a sub-millimetre segment and
// Autoware appends the same Lanelet ID twice to one PathWithLaneId point.
// Snapping only the artificial cut to that existing source vertex keeps the
// complete source polyline and [0, L] coverage; no trajectory sample or arc is
// discarded.
constexpr double kAutowareOverlappingPointDistanceM = 0.001;
constexpr double kSplitSnapToleranceM =
  kAutowareOverlappingPointDistanceM + 1.0e-9;

std::optional<Vec2> incomingDirection(
  const std::vector<Vec3> & points, const std::size_t index)
{
  for (std::size_t candidate = index; candidate > 0U; --candidate) {
    const Vec2 direction{
      points[index].x - points[candidate - 1U].x,
      points[index].y - points[candidate - 1U].y};
    if (norm(direction) > kLengthEpsilon) {
      return normalized(direction);
    }
  }
  return std::nullopt;
}

std::optional<Vec2> outgoingDirection(
  const std::vector<Vec3> & points, const std::size_t index)
{
  for (std::size_t candidate = index + 1U; candidate < points.size(); ++candidate) {
    const Vec2 direction{
      points[candidate].x - points[index].x,
      points[candidate].y - points[index].y};
    if (norm(direction) > kLengthEpsilon) {
      return normalized(direction);
    }
  }
  return std::nullopt;
}

double headingChangeAt(const std::vector<Vec3> & points, const std::size_t index)
{
  const std::optional<Vec2> incoming = incomingDirection(points, index);
  const std::optional<Vec2> outgoing = outgoingDirection(points, index);
  if (!incoming || !outgoing) {
    return 0.0;
  }
  return std::abs(normalizeAngle(
      std::atan2(outgoing->y, outgoing->x) -
      std::atan2(incoming->y, incoming->x)));
}

std::vector<double> cumulativeLength(
  const std::vector<Vec3> & points,
  const std::size_t begin_index,
  const std::size_t end_index)
{
  std::vector<double> arc(end_index - begin_index + 1U, 0.0);
  for (std::size_t index = begin_index + 1U; index <= end_index; ++index) {
    arc[index - begin_index] = arc[index - begin_index - 1U] +
      distance2d(points[index - 1U], points[index]);
  }
  return arc;
}

void appendIfDistinct(std::vector<Vec3> & points, const Vec3 & point)
{
  if (points.empty() || distance3d(points.back(), point) > kLengthEpsilon) {
    points.push_back(point);
  }
}

std::vector<Vec3> polylineSlice(
  const std::vector<Vec3> & source,
  const std::size_t source_begin,
  const std::size_t source_end,
  const std::vector<double> & arc,
  const double begin,
  const double end)
{
  const auto sample = [&](const double target) {
      if (target <= 0.0) {
        return source[source_begin];
      }
      if (target >= arc.back()) {
        return source[source_end];
      }
      const auto right = std::upper_bound(arc.begin(), arc.end(), target);
      const std::size_t right_local = static_cast<std::size_t>(right - arc.begin());
      const std::size_t left_local = right_local - 1U;
      const double span = arc[right_local] - arc[left_local];
      if (!(span > kLengthEpsilon)) {
        return source[source_begin + right_local];
      }
      const double ratio = (target - arc[left_local]) / span;
      const Vec3 & left = source[source_begin + left_local];
      const Vec3 & right_point = source[source_begin + right_local];
      return left + (right_point - left) * ratio;
    };

  std::vector<Vec3> result;
  appendIfDistinct(result, sample(begin));
  for (std::size_t local = 1U; local + 1U < arc.size(); ++local) {
    if (arc[local] > begin + kLengthEpsilon && arc[local] < end - kLengthEpsilon) {
      appendIfDistinct(result, source[source_begin + local]);
    }
  }
  appendIfDistinct(result, sample(end));
  return result;
}

double snapPartitionArcToSourceVertex(
  const std::vector<double> & source_arc, const double requested)
{
  if (source_arc.size() < 3U || requested <= 0.0 || requested >= source_arc.back()) {
    return requested;
  }
  const auto right = std::lower_bound(source_arc.begin(), source_arc.end(), requested);
  double best = requested;
  double best_difference = kSplitSnapToleranceM;
  const auto consider = [&](const std::vector<double>::const_iterator candidate) {
      if (candidate == source_arc.begin() || candidate == source_arc.end() ||
        std::next(candidate) == source_arc.end())
      {
        return;
      }
      const double difference = std::abs(*candidate - requested);
      if (difference <= best_difference) {
        best = *candidate;
        best_difference = difference;
      }
    };
  if (right != source_arc.end()) {
    consider(right);
  }
  if (right != source_arc.begin()) {
    consider(std::prev(right));
  }
  return best;
}

void validateAutowareCenterlinePointSpacing(
  const std::vector<Vec3> & points, const std::string & context)
{
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double spacing = distance3d(points[index - 1U], points[index]);
    if (!(spacing + 1.0e-12 >= kAutowareOverlappingPointDistanceM)) {
      throw std::runtime_error(
              context + " contains a consecutive centerline segment below the "
              "Autoware 1 mm overlap threshold at point " + std::to_string(index));
    }
  }
}

void validateConfiguration(
  const TopologyConfig & topology, const double recommended_speed_mps)
{
  if (!std::isfinite(topology.maximum_edge_length) ||
    !(topology.maximum_edge_length > 0.0))
  {
    throw std::invalid_argument(
            "observed route maximum_edge_length must be finite and positive");
  }
  if (!std::isfinite(topology.edge_split_heading_change_deg) ||
    topology.edge_split_heading_change_deg <= 0.0 ||
    topology.edge_split_heading_change_deg >= 180.0 ||
    !std::isfinite(topology.cusp_heading_change_deg) ||
    topology.cusp_heading_change_deg <= 0.0 ||
    topology.cusp_heading_change_deg > 180.0 ||
    topology.cusp_heading_change_deg < topology.edge_split_heading_change_deg)
  {
    throw std::invalid_argument(
            "observed route turn thresholds must satisfy "
            "0 < edge_split <= cusp <= 180 degrees");
  }
  if (!std::isfinite(recommended_speed_mps) || recommended_speed_mps < 0.0) {
    throw std::invalid_argument(
            "observed route recommended speed must be finite and nonnegative");
  }
}

double planarPolylineLength(const std::vector<Vec3> & points)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length += distance2d(points[index - 1U], points[index]);
  }
  return length;
}

double planarGraphLength(const RouteGraph & graph)
{
  double length = 0.0;
  for (const RouteEdge & edge : graph.edges) {
    length += planarPolylineLength(edge.centerline);
  }
  return length;
}

std::optional<Vec2> edgeStartDirection(const RouteEdge & edge)
{
  if (edge.centerline.size() < 2U) {
    return std::nullopt;
  }
  const Vec3 & start = edge.centerline.front();
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec2 direction{
      edge.centerline[index].x - start.x,
      edge.centerline[index].y - start.y};
    if (norm(direction) > kLengthEpsilon) {
      return normalized(direction);
    }
  }
  return std::nullopt;
}

std::optional<Vec2> edgeEndDirection(const RouteEdge & edge)
{
  if (edge.centerline.size() < 2U) {
    return std::nullopt;
  }
  const Vec3 & end = edge.centerline.back();
  for (std::size_t index = edge.centerline.size() - 1U; index > 0U; --index) {
    const Vec2 direction{
      end.x - edge.centerline[index - 1U].x,
      end.y - edge.centerline[index - 1U].y};
    if (norm(direction) > kLengthEpsilon) {
      return normalized(direction);
    }
  }
  return std::nullopt;
}

double edgeConnectionHeadingJumpDegrees(
  const RouteEdge & incoming, const RouteEdge & outgoing)
{
  const std::optional<Vec2> incoming_direction = edgeEndDirection(incoming);
  const std::optional<Vec2> outgoing_direction = edgeStartDirection(outgoing);
  if (!incoming_direction || !outgoing_direction) {
    return 0.0;
  }
  return std::abs(normalizeAngle(
      std::atan2(outgoing_direction->y, outgoing_direction->x) -
      std::atan2(incoming_direction->y, incoming_direction->x))) * 180.0 / kPi;
}

std::optional<std::vector<std::size_t>> orderedOpenChainEdges(const RouteGraph & graph)
{
  if (graph.edges.empty()) {
    return std::nullopt;
  }
  std::map<std::uint64_t, std::size_t> outgoing;
  std::map<std::uint64_t, std::size_t> indegree;
  std::map<std::uint64_t, std::size_t> outdegree;
  std::set<std::uint64_t> route_nodes;
  for (std::size_t index = 0U; index < graph.edges.size(); ++index) {
    const RouteEdge & edge = graph.edges[index];
    if (edge.from == edge.to || outgoing.count(edge.from) != 0U) {
      return std::nullopt;
    }
    outgoing.emplace(edge.from, index);
    ++outdegree[edge.from];
    ++indegree[edge.to];
    route_nodes.insert(edge.from);
    route_nodes.insert(edge.to);
  }
  std::vector<std::uint64_t> heads;
  std::vector<std::uint64_t> tails;
  for (const std::uint64_t node : route_nodes) {
    const std::size_t incoming = indegree[node];
    const std::size_t outgoing_count = outdegree[node];
    if (incoming > 1U || outgoing_count > 1U) {
      return std::nullopt;
    }
    if (incoming == 0U && outgoing_count == 1U) {
      heads.push_back(node);
    }
    if (incoming == 1U && outgoing_count == 0U) {
      tails.push_back(node);
    }
  }
  if (heads.size() != 1U || tails.size() != 1U) {
    return std::nullopt;
  }
  std::vector<std::size_t> order;
  order.reserve(graph.edges.size());
  std::set<std::size_t> visited;
  std::uint64_t node = heads.front();
  while (outgoing.count(node) != 0U) {
    const std::size_t edge_index = outgoing.at(node);
    if (!visited.insert(edge_index).second) {
      return std::nullopt;
    }
    order.push_back(edge_index);
    node = graph.edges[edge_index].to;
  }
  if (order.size() != graph.edges.size() || node != tails.front()) {
    return std::nullopt;
  }
  return order;
}

double maximumUnwrappedYawRangeDegrees(
  const std::vector<TimedPose> & trajectory, const std::size_t begin)
{
  if (begin >= trajectory.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double previous = trajectory[begin].world_from_body.rotation.yaw();
  double unwrapped = previous;
  double minimum = unwrapped;
  double maximum = unwrapped;
  for (std::size_t index = begin + 1U; index < trajectory.size(); ++index) {
    const double current = trajectory[index].world_from_body.rotation.yaw();
    unwrapped += normalizeAngle(current - previous);
    previous = current;
    minimum = std::min(minimum, unwrapped);
    maximum = std::max(maximum, unwrapped);
  }
  return (maximum - minimum) * 180.0 / kPi;
}

}  // namespace

RouteGraph buildObservedDrivenRouteGraph(
  const std::vector<TimedPose> & processed_trajectory,
  const TopologyConfig & topology,
  const std::string & frame_id,
  const double recommended_speed_mps)
{
  validateConfiguration(topology, recommended_speed_mps);
  if (processed_trajectory.size() < 2U) {
    throw std::runtime_error(
            "observed route requires at least two processed trajectory poses");
  }

  std::vector<Vec3> positions;
  positions.reserve(processed_trajectory.size());
  for (const TimedPose & pose : processed_trajectory) {
    if (!finite(pose.world_from_body.translation)) {
      throw std::runtime_error("observed route contains a non-finite trajectory position");
    }
    positions.push_back(pose.world_from_body.translation);
  }
  for (std::size_t index = 1U; index < positions.size(); ++index) {
    const double horizontal_distance = distance2d(positions[index - 1U], positions[index]);
    const double delta_z = std::abs(positions[index].z - positions[index - 1U].z);
    if (horizontal_distance <= kLengthEpsilon && delta_z > kLengthEpsilon) {
      throw std::runtime_error(
              "observed route contains a Z change at zero horizontal distance between poses " +
              std::to_string(index - 1U) + " and " + std::to_string(index));
    }
  }
  // Edge partitioning is a navigation-map operation and therefore uses
  // planar XY arc.  Elevation is preserved in every sampled Vec3, but cannot
  // manufacture extra Edge IDs by inflating the split metric.
  const double source_length = planarPolylineLength(positions);
  if (!(source_length > kLengthEpsilon)) {
    throw std::runtime_error("observed route trajectory has zero length");
  }

  const double split_threshold = topology.edge_split_heading_change_deg * kPi / 180.0;
  const double cusp_threshold = topology.cusp_heading_change_deg * kPi / 180.0;
  std::vector<std::size_t> break_indices{0U};
  for (std::size_t index = 1U; index + 1U < positions.size(); ++index) {
    const double change = headingChangeAt(positions, index);
    // Keep the explicit cusp comparison: it documents that a reversal is a
    // mandatory boundary even when future configurations distinguish it from
    // ordinary sharp-turn splitting.
    if (change >= split_threshold || change >= cusp_threshold) {
      if (break_indices.back() != index) {
        break_indices.push_back(index);
      }
    }
  }
  break_indices.push_back(positions.size() - 1U);

  std::vector<std::vector<Vec3>> pieces;
  for (std::size_t span_index = 1U; span_index < break_indices.size(); ++span_index) {
    const std::size_t source_begin = break_indices[span_index - 1U];
    const std::size_t source_end = break_indices[span_index];
    if (source_end <= source_begin) {
      continue;
    }
    const std::vector<double> arc = cumulativeLength(positions, source_begin, source_end);
    const double span_length = arc.back();
    if (!(span_length > kLengthEpsilon)) {
      continue;
    }
    // Leave enough length headroom for two adjacent cut snaps while keeping
    // every raw Edge within maximum_edge_length.  This may add an Edge near an
    // exact multiple of the maximum, but can never remove source coverage.
    const double partition_limit =
      topology.maximum_edge_length - 2.0 * kSplitSnapToleranceM;
    if (!(partition_limit > kAutowareOverlappingPointDistanceM)) {
      throw std::invalid_argument(
              "observed route maximum_edge_length is too small for Autoware "
              "1 mm centerline point separation");
    }
    const std::size_t part_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(span_length / partition_limit)));
    std::vector<double> partition_arcs(part_count + 1U, 0.0);
    partition_arcs.back() = span_length;
    for (std::size_t part = 1U; part < part_count; ++part) {
      const double requested = span_length * static_cast<double>(part) /
        static_cast<double>(part_count);
      partition_arcs[part] = snapPartitionArcToSourceVertex(arc, requested);
      if (!(partition_arcs[part] > partition_arcs[part - 1U] + kLengthEpsilon)) {
        throw std::logic_error("observed route cut snapping changed Edge order");
      }
    }
    for (std::size_t part = 0U; part < part_count; ++part) {
      const double begin = partition_arcs[part];
      const double end = partition_arcs[part + 1U];
      std::vector<Vec3> geometry = polylineSlice(
        positions, source_begin, source_end, arc, begin, end);
      if (geometry.size() >= 2U && planarPolylineLength(geometry) > kLengthEpsilon) {
        validateAutowareCenterlinePointSpacing(
          geometry, "observed route Edge part " + std::to_string(part));
        if (planarPolylineLength(geometry) > topology.maximum_edge_length + 1.0e-9) {
          throw std::logic_error(
                  "observed route cut snapping exceeded maximum_edge_length");
        }
        pieces.push_back(std::move(geometry));
      }
    }
  }
  if (pieces.empty()) {
    throw std::runtime_error("observed route contains no positive-length edge");
  }

  RouteGraph graph;
  graph.frame_id = frame_id;
  graph.nodes.reserve(pieces.size() + 1U);
  graph.edges.reserve(pieces.size());
  graph.nodes.push_back({1U, pieces.front().front(), RouteNodeType::kEndpoint});
  for (std::size_t index = 0U; index < pieces.size(); ++index) {
    if (distance3d(pieces[index].front(), graph.nodes.back().position) > 1.0e-9) {
      throw std::logic_error("observed route split produced a disconnected chain");
    }
    // Use the preceding node position verbatim at every shared boundary.
    pieces[index].front() = graph.nodes.back().position;
    const RouteNodeType type = index + 1U == pieces.size() ?
      RouteNodeType::kEndpoint : RouteNodeType::kNormal;
    const std::uint64_t node_id = static_cast<std::uint64_t>(graph.nodes.size() + 1U);
    graph.nodes.push_back({node_id, pieces[index].back(), type});
  }

  const std::uint64_t first_edge_id = static_cast<std::uint64_t>(graph.nodes.size() + 1U);
  for (std::size_t index = 0U; index < pieces.size(); ++index) {
    RouteEdge edge;
    edge.id = first_edge_id + static_cast<std::uint64_t>(index);
    edge.from = graph.nodes[index].id;
    edge.to = graph.nodes[index + 1U].id;
    edge.centerline = std::move(pieces[index]);
    edge.centerline.back() = graph.nodes[index + 1U].position;
    edge.length = polylineLength(edge.centerline);
    edge.minimum_safe_width = 0.0;
    edge.maximum_curvature = maximumPolylineCurvature(edge.centerline);
    edge.confidence = 0.0;
    edge.recommended_speed_mps = recommended_speed_mps;
    edge.passable = true;
    edge.corridor_geometry_valid = false;
    edge.validation_errors = {kObservedDrivenRouteDiagnostic};
    graph.edges.push_back(std::move(edge));
  }

  const double graph_length = [&graph]() {
      double length = 0.0;
      for (const RouteEdge & edge : graph.edges) {
        length += planarPolylineLength(edge.centerline);
      }
      return length;
    }();
  const double tolerance = std::max(1.0e-9, source_length * 1.0e-10);
  if (std::abs(graph_length - source_length) > tolerance) {
    throw std::logic_error("observed route split did not preserve trajectory length");
  }
  return graph;
}

AutowareReplayCandidateResult materializeAutowareReplayCandidate(
  const RouteGraph & chronological_replay,
  const std::vector<TimedPose> & processed_trajectory,
  const bool terminal_localization_settling_verified)
{
  AutowareReplayCandidateResult result;
  result.graph = chronological_replay;
  result.explicit_verification = terminal_localization_settling_verified;
  result.source_edges = chronological_replay.edges.size();
  result.retained_edges = chronological_replay.edges.size();
  result.source_length = planarGraphLength(chronological_replay);
  result.retained_length = result.source_length;

  if (!terminal_localization_settling_verified) {
    result.reason = "explicit_verification_not_provided";
    return result;
  }
  if (processed_trajectory.size() < 2U) {
    result.reason = "processed_trajectory_unavailable";
    return result;
  }
  const std::optional<std::vector<std::size_t>> order =
    orderedOpenChainEdges(chronological_replay);
  if (!order || order->size() < 2U) {
    result.reason = "route_is_not_a_single_open_chain";
    return result;
  }

  std::optional<std::size_t> cusp_transition;
  double cusp_heading_jump = 0.0;
  for (std::size_t transition = 0U; transition + 1U < order->size(); ++transition) {
    const RouteEdge & incoming = chronological_replay.edges[(*order)[transition]];
    const RouteEdge & outgoing = chronological_replay.edges[(*order)[transition + 1U]];
    if (incoming.to != outgoing.from) {
      result.reason = "route_chain_has_unshared_transition";
      return result;
    }
    const double heading_jump = edgeConnectionHeadingJumpDegrees(incoming, outgoing);
    if (heading_jump + 1.0e-9 >= kAutowareTerminalCuspMinimumHeadingJumpDeg) {
      // Use the first unresolved cusp.  Trimming after a later one would leave
      // an earlier forward-incompatible transition in the exported map.
      cusp_transition = transition;
      cusp_heading_jump = heading_jump;
      break;
    }
  }
  if (!cusp_transition) {
    result.reason = "no_terminal_cusp";
    return result;
  }
  result.connection_heading_jump_deg = cusp_heading_jump;

  double retained_length = 0.0;
  double omitted_length = 0.0;
  for (std::size_t position = 0U; position < order->size(); ++position) {
    const double length = planarPolylineLength(
      chronological_replay.edges[(*order)[position]].centerline);
    if (position <= *cusp_transition) {
      retained_length += length;
    } else {
      omitted_length += length;
    }
  }
  result.omitted_length = omitted_length;
  result.omitted_length_ratio = result.source_length > kLengthEpsilon ?
    omitted_length / result.source_length : std::numeric_limits<double>::infinity();
  if (!(omitted_length > kLengthEpsilon)) {
    result.reason = "terminal_tail_is_empty";
    return result;
  }
  if (omitted_length > kAutowareTerminalSettlingMaximumLength + 1.0e-9) {
    result.reason = "terminal_tail_exceeds_1m";
    return result;
  }
  if (result.omitted_length_ratio >
    kAutowareTerminalSettlingMaximumLengthRatio + 1.0e-12)
  {
    result.reason = "terminal_tail_exceeds_1_percent";
    return result;
  }

  const RouteEdge & cusp_incoming =
    chronological_replay.edges[(*order)[*cusp_transition]];
  const Vec3 & cusp_position = cusp_incoming.centerline.back();
  std::vector<double> trajectory_arc(processed_trajectory.size(), 0.0);
  for (std::size_t index = 1U; index < processed_trajectory.size(); ++index) {
    trajectory_arc[index] = trajectory_arc[index - 1U] + distance2d(
      processed_trajectory[index - 1U].world_from_body.translation,
      processed_trajectory[index].world_from_body.translation);
  }
  std::size_t trajectory_cusp = 0U;
  double best_score = std::numeric_limits<double>::infinity();
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < processed_trajectory.size(); ++index) {
    const double position_distance = distance2d(
      processed_trajectory[index].world_from_body.translation, cusp_position);
    const double arc_difference = std::abs(trajectory_arc[index] - retained_length);
    const double score = position_distance + 1.0e-6 * arc_difference;
    if (score < best_score) {
      best_score = score;
      best_distance = position_distance;
      trajectory_cusp = index;
    }
  }
  if (best_distance > 1.0e-3) {
    result.reason = "terminal_cusp_pose_not_found";
    return result;
  }
  result.terminal_body_yaw_change_deg = maximumUnwrappedYawRangeDegrees(
    processed_trajectory, trajectory_cusp);
  if (!std::isfinite(result.terminal_body_yaw_change_deg) ||
    result.terminal_body_yaw_change_deg >
    kAutowareTerminalSettlingMaximumBodyYawChangeDeg + 1.0e-9)
  {
    result.reason = "terminal_tail_body_yaw_change_exceeds_15deg";
    return result;
  }

  std::set<std::uint64_t> retained_node_ids;
  RouteGraph trimmed;
  trimmed.frame_id = chronological_replay.frame_id;
  trimmed.edges.reserve(*cusp_transition + 1U);
  for (std::size_t position = 0U; position <= *cusp_transition; ++position) {
    const RouteEdge & edge = chronological_replay.edges[(*order)[position]];
    trimmed.edges.push_back(edge);
    retained_node_ids.insert(edge.from);
    retained_node_ids.insert(edge.to);
  }
  const std::uint64_t terminal_node_id = trimmed.edges.back().to;
  trimmed.nodes.reserve(retained_node_ids.size());
  for (RouteNode node : chronological_replay.nodes) {
    if (retained_node_ids.count(node.id) == 0U) {
      continue;
    }
    if (node.id == terminal_node_id) {
      node.type = RouteNodeType::kEndpoint;
    }
    trimmed.nodes.push_back(std::move(node));
  }
  result.graph = std::move(trimmed);
  result.terminal_tail_omitted = true;
  result.retained_length = retained_length;
  result.omitted_edges = result.source_edges - result.graph.edges.size();
  result.retained_edges = result.graph.edges.size();
  result.reason = "verified_terminal_localization_settling_tail_omitted";
  return result;
}

ObservedDrivenCandidateResult materializeObservedDrivenCandidate(
  const RouteValidationResult & closed_course_validation)
{
  ObservedDrivenCandidateResult result;
  result.graph.frame_id = closed_course_validation.edited.graph.frame_id;
  std::set<std::uint64_t> retained_nodes;
  for (const RouteEdge & source : closed_course_validation.edited.graph.edges) {
    const auto metadata = closed_course_validation.edited.edge_metadata.find(source.id);
    if (metadata == closed_course_validation.edited.edge_metadata.end()) {
      ++result.excluded_edges;
      continue;
    }
    const RouteEntityMetadata & evidence = metadata->second;
    const bool observed_lineage =
      evidence.provenance != RouteProvenance::kManual &&
      !evidence.requires_orientation_collision_validation &&
      !evidence.reverse_direction && !evidence.source_ids.empty();
    const bool independently_validated =
      evidence.validation_status == RouteValidationStatus::kValid ||
      evidence.validation_status == RouteValidationStatus::kWarning;
    if ((!observed_lineage && !independently_validated) || evidence.reverse_direction) {
      ++result.excluded_edges;
      continue;
    }
    RouteEdge edge = source;
    edge.passable = true;
    auto add_diagnostic = [&](const std::string & diagnostic) {
        if (std::find(
            edge.validation_errors.begin(), edge.validation_errors.end(), diagnostic) ==
          edge.validation_errors.end())
        {
          edge.validation_errors.push_back(diagnostic);
        }
      };
    if (observed_lineage) {
      ++result.observed_evidence_edges;
      add_diagnostic(kObservedDrivenRouteDiagnostic);
      if (!independently_validated) {
        add_diagnostic("observed_passage_not_operational_safety_clearance");
      }
    } else {
      ++result.independently_validated_edges;
      add_diagnostic("closed_course_clearance_validated_manual_route");
    }
    edge.length = polylineLength(edge.centerline);
    result.retained_length += edge.length;
    retained_nodes.insert(edge.from);
    retained_nodes.insert(edge.to);
    result.graph.edges.push_back(std::move(edge));
  }
  for (const RouteNode & node : closed_course_validation.edited.graph.nodes) {
    if (retained_nodes.count(node.id) != 0U) {
      result.graph.nodes.push_back(node);
    }
  }
  return result;
}

}  // namespace lidar_mobility_map_generator
