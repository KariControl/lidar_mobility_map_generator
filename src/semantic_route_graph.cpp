#include "lidar_mobility_map_generator/semantic_route_graph.hpp"

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
#include <string>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

constexpr double kArcTolerance = 1.0e-8;
constexpr double kAutowareOverlappingPointDistanceM = 0.001;
constexpr double kAutowareSemanticBreakSnapToleranceM =
  kAutowareOverlappingPointDistanceM + 1.0e-9;

struct SliceSample
{
  std::size_t lower{0U};
  std::size_t upper{0U};
  double ratio{0.0};
};

struct MaterializedSegment
{
  EdgeSemanticSegmentRule rule;
  const RouteEdge * source{nullptr};
  RouteEdge edge;
};

std::vector<double> cumulativeLengths(const std::vector<Vec3> & points)
{
  std::vector<double> result(points.size(), 0.0);
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result[index] = result[index - 1U] + distance3d(points[index - 1U], points[index]);
  }
  return result;
}

SliceSample sampleAtArcLength(
  const std::vector<double> & cumulative, const double requested_distance)
{
  if (cumulative.size() < 2U) {
    throw std::invalid_argument("cannot sample a RouteEdge with fewer than two points");
  }
  const double distance = clamp(requested_distance, 0.0, cumulative.back());
  if (distance <= kArcTolerance) {
    return {0U, 0U, 0.0};
  }
  if (distance >= cumulative.back() - kArcTolerance) {
    const std::size_t last = cumulative.size() - 1U;
    return {last, last, 0.0};
  }
  const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), distance);
  const std::size_t upper_index = static_cast<std::size_t>(upper - cumulative.begin());
  const std::size_t lower_index = upper_index - 1U;
  const double interval = cumulative[upper_index] - cumulative[lower_index];
  if (!(interval > 1.0e-12)) {
    return {upper_index, upper_index, 0.0};
  }
  const double ratio = clamp(
    (distance - cumulative[lower_index]) / interval, 0.0, 1.0);
  if (ratio <= 1.0e-12) {
    return {lower_index, lower_index, 0.0};
  }
  if (ratio >= 1.0 - 1.0e-12) {
    return {upper_index, upper_index, 0.0};
  }
  return {lower_index, upper_index, ratio};
}

Vec3 interpolateSample(const std::vector<Vec3> & values, const SliceSample & sample)
{
  if (sample.lower == sample.upper) {
    return values.at(sample.lower);
  }
  return values.at(sample.lower) +
         (values.at(sample.upper) - values.at(sample.lower)) * sample.ratio;
}

double interpolateSample(const std::vector<double> & values, const SliceSample & sample)
{
  if (sample.lower == sample.upper) {
    return values.at(sample.lower);
  }
  return values.at(sample.lower) +
         (values.at(sample.upper) - values.at(sample.lower)) * sample.ratio;
}

std::uint8_t conservativeSample(
  const std::vector<std::uint8_t> & values, const SliceSample & sample)
{
  if (sample.lower == sample.upper) {
    return values.at(sample.lower);
  }
  // A newly interpolated cross-section is considered observed only when both
  // surrounding observations were complete.
  return values.at(sample.lower) != 0U && values.at(sample.upper) != 0U ? 1U : 0U;
}

std::vector<SliceSample> sliceSamples(
  const std::vector<double> & cumulative, const double start_s, const double end_s)
{
  if (!(end_s > start_s + kArcTolerance)) {
    return {};
  }
  std::vector<SliceSample> result;
  result.push_back(sampleAtArcLength(cumulative, start_s));
  for (std::size_t index = 1U; index + 1U < cumulative.size(); ++index) {
    if (cumulative[index] > start_s + kArcTolerance &&
      cumulative[index] < end_s - kArcTolerance)
    {
      result.push_back({index, index, 0.0});
    }
  }
  const SliceSample last = sampleAtArcLength(cumulative, end_s);
  if (result.empty() || result.back().lower != last.lower ||
    result.back().upper != last.upper ||
    std::abs(result.back().ratio - last.ratio) > 1.0e-12)
  {
    result.push_back(last);
  } else {
    result.back() = last;
  }
  return result;
}

std::vector<Vec3> sliceValues(
  const std::vector<Vec3> & values, const std::vector<SliceSample> & samples)
{
  std::vector<Vec3> result;
  result.reserve(samples.size());
  for (const SliceSample & sample : samples) {
    result.push_back(interpolateSample(values, sample));
  }
  return result;
}

std::vector<double> sliceValues(
  const std::vector<double> & values, const std::vector<SliceSample> & samples)
{
  std::vector<double> result;
  result.reserve(samples.size());
  for (const SliceSample & sample : samples) {
    result.push_back(interpolateSample(values, sample));
  }
  return result;
}

std::vector<std::uint8_t> sliceValues(
  const std::vector<std::uint8_t> & values, const std::vector<SliceSample> & samples)
{
  std::vector<std::uint8_t> result;
  result.reserve(samples.size());
  for (const SliceSample & sample : samples) {
    result.push_back(conservativeSample(values, sample));
  }
  return result;
}

void addUniqueError(std::vector<std::string> & errors, const std::string & error)
{
  if (std::find(errors.begin(), errors.end(), error) == errors.end()) {
    errors.push_back(error);
  }
}

const RouteEdge * findEdge(const RouteGraph & graph, const std::uint64_t id)
{
  const auto found = std::find_if(
    graph.edges.begin(), graph.edges.end(),
    [id](const RouteEdge & edge) {return edge.id == id;});
  return found == graph.edges.end() ? nullptr : &*found;
}

const EdgeSemanticSegmentRule * ruleAt(
  const std::vector<EdgeSemanticSegmentRule> & rules, const double midpoint)
{
  const auto found = std::find_if(
    rules.begin(), rules.end(), [&](const EdgeSemanticSegmentRule & rule) {
      return midpoint >= rule.start_s - kArcTolerance &&
             midpoint <= rule.end_s + kArcTolerance;
    });
  return found == rules.end() ? nullptr : &*found;
}

void appendBreak(std::vector<double> & breaks, const double value, const double length)
{
  breaks.push_back(clamp(value, 0.0, length));
}

void normalizeBreaks(std::vector<double> & breaks)
{
  std::sort(breaks.begin(), breaks.end());
  breaks.erase(
    std::unique(
      breaks.begin(), breaks.end(), [](const double lhs, const double rhs) {
        return std::abs(lhs - rhs) <= kArcTolerance;
      }),
    breaks.end());
}

double snapSemanticBreakToSourceVertex(
  const std::vector<double> & cumulative, const double requested)
{
  if (cumulative.size() < 3U || requested <= 0.0 || requested >= cumulative.back()) {
    return requested;
  }
  const auto right = std::lower_bound(cumulative.begin(), cumulative.end(), requested);
  double best = requested;
  double best_difference = kAutowareSemanticBreakSnapToleranceM;
  const auto consider = [&](const std::vector<double>::const_iterator candidate) {
      if (candidate == cumulative.begin() || candidate == cumulative.end() ||
        std::next(candidate) == cumulative.end())
      {
        return;
      }
      const double difference = std::abs(*candidate - requested);
      if (difference <= best_difference) {
        best = *candidate;
        best_difference = difference;
      }
    };
  if (right != cumulative.end()) {
    consider(right);
  }
  if (right != cumulative.begin()) {
    consider(std::prev(right));
  }
  return best;
}

void snapSemanticBreaksForAutoware(
  std::vector<double> & breaks, const RouteEdge & edge)
{
  if (breaks.size() <= 2U) {
    return;
  }
  const std::vector<double> cumulative = cumulativeLengths(edge.centerline);
  const std::size_t original_count = breaks.size();
  for (std::size_t index = 1U; index + 1U < breaks.size(); ++index) {
    breaks[index] = snapSemanticBreakToSourceVertex(cumulative, breaks[index]);
  }
  normalizeBreaks(breaks);
  if (breaks.size() != original_count) {
    throw std::invalid_argument(
            "semantic boundaries are closer than Autoware's 1 mm "
            "centerline representation permits on source Edge " +
            std::to_string(edge.id));
  }
}

void validateAutowareSemanticCenterlineSpacing(
  const std::vector<Vec3> & points, const std::uint64_t source_edge_id)
{
  for (std::size_t index = 1U; index < points.size(); ++index) {
    if (distance3d(points[index - 1U], points[index]) + 1.0e-12 <
      kAutowareOverlappingPointDistanceM)
    {
      throw std::invalid_argument(
              "semantic Lanelet centerline contains a segment below "
              "Autoware's 1 mm overlap threshold on source Edge " +
              std::to_string(source_edge_id));
    }
  }
}

std::vector<EdgeSemanticSegmentRule> synchronizedRules(
  const RouteGraph & graph,
  const std::vector<EdgeSemanticSegmentRule> & semantic_rules,
  const bool synchronize_reverse_edges)
{
  std::unordered_map<std::uint64_t, std::vector<EdgeSemanticSegmentRule>> rules_by_edge;
  for (const EdgeSemanticSegmentRule & rule : semantic_rules) {
    rules_by_edge[rule.edge_id].push_back(rule);
  }

  std::vector<EdgeSemanticSegmentRule> result;
  for (const RouteEdge & edge : graph.edges) {
    const double length = polylineLength(edge.centerline);
    if (!(length > kArcTolerance)) {
      continue;
    }
    const auto own_found = rules_by_edge.find(edge.id);
    if (own_found == rules_by_edge.end() || own_found->second.empty()) {
      continue;
    }
    std::vector<double> breaks{0.0, length};
    for (const EdgeSemanticSegmentRule & rule : own_found->second) {
      appendBreak(breaks, rule.start_s, length);
      appendBreak(breaks, rule.end_s, length);
    }
    if (synchronize_reverse_edges && edge.reverse_of) {
      const RouteEdge * reverse = findEdge(graph, *edge.reverse_of);
      const auto reverse_rules = rules_by_edge.find(*edge.reverse_of);
      if (reverse != nullptr && reverse_rules != rules_by_edge.end()) {
        const double reverse_length = polylineLength(reverse->centerline);
        if (reverse_length > kArcTolerance) {
          for (const EdgeSemanticSegmentRule & reverse_rule : reverse_rules->second) {
            appendBreak(
              breaks, length - reverse_rule.end_s * length / reverse_length, length);
            appendBreak(
              breaks, length - reverse_rule.start_s * length / reverse_length, length);
          }
        }
      }
    }
    normalizeBreaks(breaks);
    snapSemanticBreaksForAutoware(breaks, edge);
    for (std::size_t index = 1U; index < breaks.size(); ++index) {
      const double start_s = breaks[index - 1U];
      const double end_s = breaks[index];
      if (!(end_s > start_s + kArcTolerance)) {
        continue;
      }
      const EdgeSemanticSegmentRule * source_rule = ruleAt(
        own_found->second, 0.5 * (start_s + end_s));
      if (source_rule == nullptr) {
        throw std::logic_error(
                "semantic segment rules do not cover source edge " + std::to_string(edge.id));
      }
      EdgeSemanticSegmentRule rule = *source_rule;
      rule.start_s = start_s;
      rule.end_s = end_s;
      result.push_back(std::move(rule));
    }
  }
  return result;
}

RouteEdge sliceEdge(
  const RouteEdge & source, const EdgeSemanticSegmentRule & rule)
{
  const std::vector<double> cumulative = cumulativeLengths(source.centerline);
  const std::vector<SliceSample> samples = sliceSamples(cumulative, rule.start_s, rule.end_s);
  if (samples.size() < 2U) {
    throw std::logic_error("semantic split produced a degenerate RouteEdge");
  }

  RouteEdge result = source;
  result.reverse_of.reset();
  result.centerline = sliceValues(source.centerline, samples);
  if (source.left_boundary.size() == source.centerline.size()) {
    result.left_boundary = sliceValues(source.left_boundary, samples);
  } else {
    result.left_boundary.clear();
  }
  if (source.right_boundary.size() == source.centerline.size()) {
    result.right_boundary = sliceValues(source.right_boundary, samples);
  } else {
    result.right_boundary.clear();
  }
  if (source.left_clearance.size() == source.centerline.size()) {
    result.left_clearance = sliceValues(source.left_clearance, samples);
  } else {
    result.left_clearance.clear();
  }
  if (source.right_clearance.size() == source.centerline.size()) {
    result.right_clearance = sliceValues(source.right_clearance, samples);
  } else {
    result.right_clearance.clear();
  }
  if (source.left_clearance_observed.size() == source.centerline.size()) {
    result.left_clearance_observed = sliceValues(source.left_clearance_observed, samples);
  } else {
    result.left_clearance_observed.clear();
  }
  if (source.right_clearance_observed.size() == source.centerline.size()) {
    result.right_clearance_observed = sliceValues(source.right_clearance_observed, samples);
  } else {
    result.right_clearance_observed.clear();
  }

  result.length = polylineLength(result.centerline);
  result.maximum_curvature = maximumPolylineCurvature(result.centerline);
  if (result.left_clearance.size() == result.centerline.size() &&
    result.right_clearance.size() == result.centerline.size())
  {
    result.minimum_safe_width = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < result.centerline.size(); ++index) {
      result.minimum_safe_width = std::min(
        result.minimum_safe_width,
        result.left_clearance[index] + result.right_clearance[index]);
    }
  }
  if (result.left_clearance_observed.size() == result.centerline.size() &&
    result.right_clearance_observed.size() == result.centerline.size())
  {
    std::size_t observed = 0U;
    for (const std::uint8_t value : result.left_clearance_observed) {
      observed += value != 0U ? 1U : 0U;
    }
    for (const std::uint8_t value : result.right_clearance_observed) {
      observed += value != 0U ? 1U : 0U;
    }
    result.confidence = static_cast<double>(observed) /
      static_cast<double>(2U * result.centerline.size());
  }
  result.recommended_speed_mps = rule.effective_speed_limit_mps;
  result.passable = rule.effective_passable;
  if (rule.no_entry) {
    addUniqueError(result.validation_errors, "semantic_no_entry");
  }
  return result;
}

std::uint64_t maximumEntityId(const RouteGraph & graph)
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

std::uint64_t allocateId(std::uint64_t & next_id)
{
  if (next_id == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("route graph ID space is exhausted");
  }
  return next_id++;
}

using PositionKey = std::tuple<long long, long long, long long>;

PositionKey positionKey(const Vec3 & point)
{
  // A nanometre-scale key absorbs reversed-polyline interpolation round-off
  // while remaining far below any mapping resolution used by the generator.
  constexpr double precision = 1.0e9;
  return {
    std::llround(point.x * precision),
    std::llround(point.y * precision),
    std::llround(point.z * precision)};
}

std::uint64_t physicalSourceId(const RouteEdge & edge)
{
  return edge.reverse_of ? std::min(edge.id, *edge.reverse_of) : edge.id;
}

double pointToSegmentDistance3d(
  const Vec3 & point, const Vec3 & first, const Vec3 & second)
{
  const Vec3 delta = second - first;
  const double squared_length = dot(delta, delta);
  if (!(squared_length > 1.0e-24)) {
    return distance3d(point, first);
  }
  const double ratio = clamp(dot(point - first, delta) / squared_length, 0.0, 1.0);
  return distance3d(point, first + delta * ratio);
}

double pointToPolylineDistance3d(
  const Vec3 & point, const std::vector<Vec3> & polyline)
{
  if (polyline.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (polyline.size() == 1U) {
    return distance3d(point, polyline.front());
  }
  double result = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    result = std::min(
      result,
      pointToSegmentDistance3d(point, polyline[index - 1U], polyline[index]));
  }
  return result;
}

}  // namespace

SemanticRouteGraphResult materializeSemanticRouteGraph(
  const RouteGraph & source_graph,
  const SemanticMap & semantic_map,
  const SemanticRouteGraphOptions & options)
{
  const std::vector<EdgeSemanticSegmentRule> derived_rules =
    deriveEdgeSemanticSegmentRules(semantic_map, source_graph);
  const std::vector<EdgeSemanticSegmentRule> rules = synchronizedRules(
    source_graph, derived_rules, options.synchronize_reverse_edge_splits);

  std::unordered_map<std::uint64_t, const RouteEdge *> source_by_id;
  std::unordered_map<std::uint64_t, std::size_t> segment_count;
  for (const RouteEdge & edge : source_graph.edges) {
    source_by_id.emplace(edge.id, &edge);
  }
  for (const EdgeSemanticSegmentRule & rule : rules) {
    ++segment_count[rule.edge_id];
  }

  const std::uint64_t maximum_id = maximumEntityId(source_graph);
  if (maximum_id == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("route graph ID space is exhausted");
  }
  std::uint64_t next_id = maximum_id + 1U;

  SemanticRouteGraphResult result;
  result.graph.frame_id = source_graph.frame_id;
  result.graph.nodes = source_graph.nodes;
  std::unordered_map<std::uint64_t, std::size_t> node_index;
  for (std::size_t index = 0U; index < result.graph.nodes.size(); ++index) {
    node_index[result.graph.nodes[index].id] = index;
  }

  using SplitNodeKey = std::tuple<std::uint64_t, PositionKey>;
  std::map<SplitNodeKey, std::uint64_t> split_nodes;
  const auto ensureSplitNode = [&](const RouteEdge & source, const Vec3 & requested_position) {
      const SplitNodeKey key{physicalSourceId(source), positionKey(requested_position)};
      const auto existing = split_nodes.find(key);
      if (existing != split_nodes.end()) {
        return existing->second;
      }
      const std::uint64_t id = allocateId(next_id);
      split_nodes.emplace(key, id);
      node_index[id] = result.graph.nodes.size();
      result.graph.nodes.push_back({id, requested_position, RouteNodeType::kNormal});
      return id;
    };

  std::vector<MaterializedSegment> segments;
  segments.reserve(rules.size());
  for (const EdgeSemanticSegmentRule & rule : rules) {
    const auto source_found = source_by_id.find(rule.edge_id);
    if (source_found == source_by_id.end()) {
      throw std::logic_error("semantic rule references a missing source edge");
    }
    const RouteEdge & source = *source_found->second;
    if (rule.no_entry && options.no_entry_policy == SemanticNoEntryPolicy::kOmit) {
      ++result.omitted_no_entry_segments;
      continue;
    }
    RouteEdge edge = sliceEdge(source, rule);
    const double source_length = polylineLength(source.centerline);
    edge.from = rule.start_s <= kArcTolerance ? source.from :
      ensureSplitNode(source, edge.centerline.front());
    edge.to = rule.end_s >= source_length - kArcTolerance ? source.to :
      ensureSplitNode(source, edge.centerline.back());
    edge.centerline.front() = result.graph.nodes.at(node_index.at(edge.from)).position;
    edge.centerline.back() = result.graph.nodes.at(node_index.at(edge.to)).position;
    edge.length = polylineLength(edge.centerline);
    edge.maximum_curvature = maximumPolylineCurvature(edge.centerline);
    const bool unsplit = segment_count[source.id] == 1U &&
      rule.start_s <= kArcTolerance && rule.end_s >= source_length - kArcTolerance;
    edge.id = unsplit ? source.id : allocateId(next_id);
    segments.push_back({rule, &source, std::move(edge)});
  }

  // Restore reverse_of only for exactly corresponding retained segments.
  for (MaterializedSegment & segment : segments) {
    if (!segment.source->reverse_of) {
      continue;
    }
    const auto reverse_source_found = source_by_id.find(*segment.source->reverse_of);
    if (reverse_source_found == source_by_id.end()) {
      continue;
    }
    const RouteEdge & reverse_source = *reverse_source_found->second;
    const double source_length = polylineLength(segment.source->centerline);
    const double reverse_length = polylineLength(reverse_source.centerline);
    if (!(source_length > kArcTolerance) || !(reverse_length > kArcTolerance)) {
      continue;
    }
    const double expected_start =
      (source_length - segment.rule.end_s) * reverse_length / source_length;
    const double expected_end =
      (source_length - segment.rule.start_s) * reverse_length / source_length;
    const auto reverse = std::find_if(
      segments.begin(), segments.end(), [&](const MaterializedSegment & candidate) {
        return candidate.source->id == reverse_source.id &&
               std::abs(candidate.rule.start_s - expected_start) <= 1.0e-6 &&
               std::abs(candidate.rule.end_s - expected_end) <= 1.0e-6 &&
               segment.edge.from == candidate.edge.to &&
               segment.edge.to == candidate.edge.from;
      });
    if (reverse != segments.end()) {
      segment.edge.reverse_of = reverse->edge.id;
    }
  }

  result.graph.edges.reserve(segments.size());
  result.edge_provenance.reserve(segments.size());
  std::unordered_set<std::uint64_t> used_node_ids;
  for (MaterializedSegment & segment : segments) {
    used_node_ids.insert(segment.edge.from);
    used_node_ids.insert(segment.edge.to);
    result.edge_provenance.push_back({
      segment.edge.id,
      segment.source->id,
      segment.rule.start_s,
      segment.rule.end_s,
      segment.rule.no_entry,
      segment.rule.source_feature_ids});
    result.graph.edges.push_back(std::move(segment.edge));
  }

  // A no-entry omission can isolate source or split nodes. Exclude those from
  // the operational graph and recompute node types from retained passable
  // connectivity. Diagnostic impassable edges keep their endpoint nodes.
  result.graph.nodes.erase(
    std::remove_if(
      result.graph.nodes.begin(), result.graph.nodes.end(),
      [&](const RouteNode & node) {return used_node_ids.count(node.id) == 0U;}),
    result.graph.nodes.end());
  std::map<std::uint64_t, std::set<std::uint64_t>> neighbours;
  for (const RouteEdge & edge : result.graph.edges) {
    if (!edge.passable) {
      continue;
    }
    neighbours[edge.from].insert(edge.to);
    neighbours[edge.to].insert(edge.from);
  }
  for (RouteNode & node : result.graph.nodes) {
    const std::size_t degree = neighbours[node.id].size();
    node.type = degree <= 1U ? RouteNodeType::kEndpoint :
      (degree >= 3U ? RouteNodeType::kJunction : RouteNodeType::kNormal);
  }

  return result;
}

LosslessSemanticRouteGraphAudit validateLosslessSemanticRouteGraph(
  const RouteGraph & source_graph,
  const SemanticRouteGraphResult & semantic_graph)
{
  constexpr double absolute_arc_tolerance = 1.0e-7;
  constexpr double geometry_tolerance = 1.0e-7;
  if (source_graph.frame_id != semantic_graph.graph.frame_id) {
    throw std::invalid_argument("lossless semantic graph frame differs from source replay");
  }
  if (semantic_graph.edge_provenance.size() != semantic_graph.graph.edges.size()) {
    throw std::invalid_argument(
            "lossless semantic provenance count differs from output Edge count");
  }

  LosslessSemanticRouteGraphAudit audit;
  audit.source_edges = source_graph.edges.size();
  audit.output_edges = semantic_graph.graph.edges.size();
  std::size_t provenance_index = 0U;
  std::unordered_set<std::uint64_t> output_ids;

  for (const RouteEdge & source : source_graph.edges) {
    const double source_length = polylineLength(source.centerline);
    if (source.centerline.size() < 2U || !(source_length > absolute_arc_tolerance) ||
      !std::isfinite(source_length))
    {
      throw std::invalid_argument(
              "lossless semantic source Edge is degenerate: " + std::to_string(source.id));
    }
    audit.source_length += source_length;
    const double arc_tolerance = std::max(
      absolute_arc_tolerance, absolute_arc_tolerance * source_length);
    const std::size_t first_provenance = provenance_index;
    double expected_start_s = 0.0;
    std::vector<Vec3> reconstructed;
    std::uint64_t previous_to = source.from;

    while (provenance_index < semantic_graph.edge_provenance.size() &&
      semantic_graph.edge_provenance[provenance_index].source_edge_id == source.id)
    {
      const SemanticRouteEdgeProvenance & provenance =
        semantic_graph.edge_provenance[provenance_index];
      const RouteEdge & output = semantic_graph.graph.edges[provenance_index];
      if (output.id != provenance.edge_id || !output_ids.insert(output.id).second) {
        throw std::invalid_argument(
                "lossless semantic output Edge order/identity differs from provenance");
      }
      if (!std::isfinite(provenance.source_start_s) ||
        !std::isfinite(provenance.source_end_s) ||
        std::abs(provenance.source_start_s - expected_start_s) > arc_tolerance ||
        !(provenance.source_end_s > provenance.source_start_s + absolute_arc_tolerance) ||
        provenance.source_end_s > source_length + arc_tolerance)
      {
        throw std::invalid_argument(
                "lossless semantic source intervals contain a gap, overlap, or invalid length");
      }
      if (provenance.no_entry || output.passable != source.passable) {
        throw std::invalid_argument(
                "lossless semantic Lanelet segmentation changed source passability");
      }
      if (output.centerline.size() < 2U || output.from != previous_to) {
        throw std::invalid_argument(
                "lossless semantic segments do not form the source directed chain");
      }
      // materializeSemanticRouteGraph() is also used for the generic editable
      // physical-topology preview and for Nav2.  Autoware's 1 mm overlapping-
      // point rule belongs to this explicit lossless replay-to-Lanelet gate,
      // not to those generic graph consumers.  Keeping the check here still
      // fails closed before any chronological semantic Lanelet is exported,
      // while an unrelated sub-millimetre sample on a non-Autoware topology
      // Edge cannot suppress an otherwise valid full replay Route.
      validateAutowareSemanticCenterlineSpacing(output.centerline, source.id);
      const double output_length = polylineLength(output.centerline);
      const double expected_length = provenance.source_end_s - provenance.source_start_s;
      if (!std::isfinite(output_length) ||
        std::abs(output_length - expected_length) > arc_tolerance)
      {
        throw std::invalid_argument(
                "lossless semantic segment arc length differs from its source interval");
      }
      if (reconstructed.empty()) {
        reconstructed = output.centerline;
      } else {
        if (distance3d(reconstructed.back(), output.centerline.front()) > geometry_tolerance) {
          throw std::invalid_argument(
                  "lossless semantic segment centerlines are disconnected");
        }
        reconstructed.insert(
          reconstructed.end(), output.centerline.begin() + 1, output.centerline.end());
      }
      for (const Vec3 & point : output.centerline) {
        if (!finite(point) ||
          pointToPolylineDistance3d(point, source.centerline) > geometry_tolerance)
        {
          throw std::invalid_argument(
                  "lossless semantic segment geometry leaves its source centerline");
        }
      }
      audit.output_length += output_length;
      previous_to = output.to;
      expected_start_s = provenance.source_end_s;
      ++provenance_index;
    }

    if (provenance_index == first_provenance ||
      std::abs(expected_start_s - source_length) > arc_tolerance ||
      previous_to != source.to)
    {
      throw std::invalid_argument(
              "lossless semantic source Edge is omitted or not covered through [0,L]");
    }
    if (distance3d(reconstructed.front(), source.centerline.front()) > geometry_tolerance ||
      distance3d(reconstructed.back(), source.centerline.back()) > geometry_tolerance ||
      std::abs(polylineLength(reconstructed) - source_length) > arc_tolerance)
    {
      throw std::invalid_argument(
              "lossless semantic reconstructed source endpoints/arc length differ");
    }
    for (const Vec3 & point : source.centerline) {
      if (pointToPolylineDistance3d(point, reconstructed) > geometry_tolerance) {
        throw std::invalid_argument(
                "lossless semantic reconstructed centerline omits source geometry");
      }
    }
  }
  if (provenance_index != semantic_graph.edge_provenance.size()) {
    throw std::invalid_argument(
            "lossless semantic provenance contains reordered or unknown source Edges");
  }
  const double total_tolerance = std::max(
    absolute_arc_tolerance, absolute_arc_tolerance * audit.source_length);
  if (std::abs(audit.output_length - audit.source_length) > total_tolerance) {
    throw std::invalid_argument(
            "lossless semantic full-map arc length differs from source replay");
  }
  return audit;
}

}  // namespace lidar_mobility_map_generator
