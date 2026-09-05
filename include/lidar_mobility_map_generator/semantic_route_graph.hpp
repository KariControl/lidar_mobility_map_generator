#pragma once

#include "lidar_mobility_map_generator/semantic_map.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lidar_mobility_map_generator
{

// A semantic no-entry interval can either disappear from the operational
// graph (the normal Lanelet/Nav2 export policy), or remain as an explicitly
// impassable diagnostic edge for review tools.
enum class SemanticNoEntryPolicy
{
  kOmit,
  kKeepImpassable
};

struct SemanticRouteGraphOptions
{
  SemanticNoEntryPolicy no_entry_policy{SemanticNoEntryPolicy::kOmit};
  // Keep forward/reverse physical pairs divided at identical positions even
  // when a directed semantic restriction was authored on only one direction.
  // This preserves well-formed reverse_of pairs without changing the semantic
  // rule applied to the unrestricted direction.
  bool synchronize_reverse_edge_splits{true};
};

// Traceability for a materialized edge. Distances use the original source
// edge's directed spatial-3D arc-length convention over the processed
// base_link geometry. Raw Edge partitioning itself is planar XY; exporters
// record both metrics explicitly.
struct SemanticRouteEdgeProvenance
{
  std::uint64_t edge_id{0U};
  std::uint64_t source_edge_id{0U};
  double source_start_s{0.0};
  double source_end_s{0.0};
  bool no_entry{false};
  std::vector<std::uint64_t> source_feature_ids;
};

struct SemanticRouteGraphResult
{
  RouteGraph graph;
  std::vector<SemanticRouteEdgeProvenance> edge_provenance;
  std::size_t omitted_no_entry_segments{0U};
};

struct LosslessSemanticRouteGraphAudit
{
  std::size_t source_edges{0U};
  std::size_t output_edges{0U};
  double source_length{0.0};
  double output_length{0.0};
};

// Materialize exact semantic span boundaries as RouteNodes/RouteEdges.
//
// - effective speed is stored in RouteEdge::recommended_speed_mps;
// - an authored speed limit overrides the generated RouteEdge default;
// - overlapping authored speed limits use deriveEdgeSemanticSegmentRules()
//   semantics (the most restrictive positive authored limit);
// - no-entry spans follow options.no_entry_policy;
// - source RouteGraph and SemanticMap are never mutated.
//
// A split source edge receives new edge IDs. An edge that needs no split keeps
// its source ID. New IDs are allocated above every source node/edge ID.
[[nodiscard]] SemanticRouteGraphResult materializeSemanticRouteGraph(
  const RouteGraph & source_graph,
  const SemanticMap & semantic_map,
  const SemanticRouteGraphOptions & options = {});

// Prove that a semantic graph is only an ordered one-to-many segmentation of
// `source_graph`. Every source Edge must have a contiguous [0, L] partition;
// output Edge/provenance order and topology must agree; and concatenated
// centerlines must reproduce the source direction, geometry, and arc length.
// Throws std::invalid_argument on any gap, overlap, omission, reorder, or
// geometric change. This is the hard gate used before exporting speed-specific
// Lanelets from the immutable chronological replay.
[[nodiscard]] LosslessSemanticRouteGraphAudit validateLosslessSemanticRouteGraph(
  const RouteGraph & source_graph,
  const SemanticRouteGraphResult & semantic_graph);

}  // namespace lidar_mobility_map_generator
