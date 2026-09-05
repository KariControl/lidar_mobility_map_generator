#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

enum class SemanticFeatureType
{
  kStop,
  kWait,
  kDock,
  kCharger,
  kDoor,
  kSpeedLimit,
  kNoEntry
};

enum class SemanticGeometryType
{
  kPoint,
  kRouteEdges,
  kPolygon
};

// A directed interval on a generated Route Edge. Distances are measured along
// the edge centerline from its `from` node. Keeping the source edge ID and arc
// length makes semantic edits independent of the centerline sample spacing.
struct RouteEdgeSpan
{
  std::uint64_t edge_id{0U};
  double start_s{0.0};
  double end_s{0.0};
  // Optional world-coordinate anchors allow a best-effort geometry remap when
  // generator re-runs assign different edge IDs.
  std::optional<Vec3> start_anchor;
  std::optional<Vec3> end_anchor;
};

struct SemanticFeature
{
  std::uint64_t id{0U};
  SemanticFeatureType type{SemanticFeatureType::kStop};
  SemanticGeometryType geometry{SemanticGeometryType::kPoint};
  bool enabled{true};

  std::string name;
  std::string notes;

  // Used by point features. yaw is expressed in the map frame.
  Vec3 position{};
  double yaw{0.0};

  // speed_limit: m/s. door: physical opening width in metres.
  double value{0.0};
  double extent{0.0};

  // Point features may optionally reference one route edge. Route-edge
  // restrictions created by the version-1 format reference whole directed
  // route edges. The IDs remain available for backwards compatibility.
  std::vector<std::uint64_t> route_edge_ids;

  // Version-2 route restrictions use one or more exact directed intervals.
  // When empty, route_edge_ids retain their legacy whole-edge meaning.
  std::vector<RouteEdgeSpan> route_edge_spans;

  // Used by polygon speed/no-entry zones. The polygon is implicitly closed.
  std::vector<Vec3> polygon;
};

struct SemanticMap
{
  std::string frame_id{"map"};
  std::vector<SemanticFeature> features;
};

// Result of projecting an authoring semantic layer onto a safety-validated
// operational graph. Route spans whose edited source edge did not pass route
// validation are deliberately omitted instead of making the complete export
// fail or leaking an invalid route to a planner.
struct SemanticGraphFilterResult
{
  SemanticMap map;
  std::vector<std::uint64_t> excluded_feature_ids;
  std::vector<std::string> diagnostics;
};

struct EdgeSemanticRule
{
  std::uint64_t edge_id{0U};
  bool base_passable{true};
  bool no_entry{false};
  bool effective_passable{true};
  double base_speed_limit_mps{0.0};
  double effective_speed_limit_mps{0.0};
  std::vector<std::uint64_t> source_feature_ids;
};

// Operational semantic rule for one continuous portion of a source edge.
// Unlike EdgeSemanticRule, this type preserves partial restrictions.
struct EdgeSemanticSegmentRule
{
  std::uint64_t edge_id{0U};
  double start_s{0.0};
  double end_s{0.0};
  bool base_passable{true};
  bool no_entry{false};
  bool effective_passable{true};
  double base_speed_limit_mps{0.0};
  double effective_speed_limit_mps{0.0};
  std::vector<std::uint64_t> source_feature_ids;
};

[[nodiscard]] const char * toString(SemanticFeatureType type);
[[nodiscard]] const char * toString(SemanticGeometryType type);
[[nodiscard]] SemanticFeatureType semanticFeatureTypeFromString(const std::string & value);
[[nodiscard]] SemanticGeometryType semanticGeometryTypeFromString(const std::string & value);

void validateSemanticMap(const SemanticMap & map, const RouteGraph * graph = nullptr);

void saveSemanticMapTsv(
  const std::filesystem::path & path,
  const SemanticMap & map);

[[nodiscard]] SemanticMap loadSemanticMapTsv(
  const std::filesystem::path & path,
  const RouteGraph * graph = nullptr);

void saveSemanticMapGeoJson(
  const std::filesystem::path & path,
  const SemanticMap & map,
  const RouteGraph & graph);

[[nodiscard]] std::vector<EdgeSemanticRule> deriveEdgeSemanticRules(
  const SemanticMap & map,
  const RouteGraph & graph);

[[nodiscard]] std::vector<EdgeSemanticSegmentRule> deriveEdgeSemanticSegmentRules(
  const SemanticMap & map,
  const RouteGraph & graph);

// Return the corresponding interval on reverse_of. Throws if the source edge
// or its reverse is unavailable.
[[nodiscard]] RouteEdgeSpan reverseRouteEdgeSpan(
  const RouteEdgeSpan & span,
  const RouteGraph & graph);

// Rebind anchored spans to a changed graph. A span crossing a newly inserted
// split node becomes multiple connected spans. Missing or ambiguous geometry
// is rejected instead of silently changing the restricted route.
[[nodiscard]] SemanticMap remapSemanticMapToGraph(
  const SemanticMap & map,
  const RouteGraph & graph,
  double maximum_anchor_distance = 1.0);

// Returns true only when every bounded route-edge semantic already expresses
// its Edge ID, arc distances, and anchors in `graph` coordinates. Legacy
// whole-edge rules return false because they carry no geometric proof.
[[nodiscard]] bool semanticRouteSpansUseGraphCoordinates(
  const SemanticMap & map,
  const RouteGraph & graph,
  double maximum_anchor_error = 1.0e-6);

[[nodiscard]] SemanticGraphFilterResult filterSemanticMapForGraph(
  const SemanticMap & map,
  const RouteGraph & graph);

void saveSemanticRouteRulesYaml(
  const std::filesystem::path & path,
  const SemanticMap & map,
  const RouteGraph & graph);

// Nav2-compatible graph where semantic no-entry edges are omitted and semantic
// speed limits override the automatically estimated edge speed.
void saveSemanticRouteGraphGeoJson(
  const std::filesystem::path & path,
  const SemanticMap & map,
  const RouteGraph & graph);

[[nodiscard]] std::uint64_t nextSemanticFeatureId(const SemanticMap & map);

}  // namespace lidar_mobility_map_generator
