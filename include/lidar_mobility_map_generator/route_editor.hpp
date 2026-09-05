#pragma once

#include "lidar_mobility_map_generator/occupancy_grid.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{

enum class RouteEditOperationType
{
  kClearGraph,
  kAddNode,
  kMoveNode,
  kDeleteNode,
  kAddEdge,
  kDeleteEdge,
  kSplitEdge,
  kSetDirection
};

enum class RouteDirection
{
  kOneWay,
  kBidirectional
};

enum class RouteProvenance
{
  kGenerated,
  kManual,
  kEditedGenerated
};

enum class RouteValidationStatus
{
  kUnvalidated,
  kValid,
  kWarning,
  kInvalid
};

// One deterministic command in the edit overlay. Only fields relevant to the
// selected type are populated. Allocated IDs are stored in the command so an
// overlay replays identically after save/reload.
struct RouteEditOperation
{
  std::uint64_t operation_id{0U};
  RouteEditOperationType type{RouteEditOperationType::kAddNode};

  std::uint64_t node_id{0U};
  std::uint64_t edge_id{0U};
  std::uint64_t reverse_edge_id{0U};
  std::uint64_t from_id{0U};
  std::uint64_t to_id{0U};

  // Split result IDs, ordered in the selected edge direction. Reverse IDs are
  // zero when the source edge was one-way.
  std::uint64_t first_edge_id{0U};
  std::uint64_t second_edge_id{0U};
  std::uint64_t first_reverse_edge_id{0U};
  std::uint64_t second_reverse_edge_id{0U};

  Vec3 position{};
  double split_s{0.0};
  RouteDirection direction{RouteDirection::kOneWay};
  bool include_reverse{true};
  bool cascade{true};
  std::vector<Vec3> polyline;
};

struct RouteEditOverlay
{
  std::uint32_t version{1U};
  std::string frame_id{"map"};
  std::string base_graph_fingerprint;
  std::uint64_t next_operation_id{1U};
  std::uint64_t next_entity_id{1U};
  std::vector<RouteEditOperation> operations;
};

struct RouteEntityMetadata
{
  RouteProvenance provenance{RouteProvenance::kGenerated};
  RouteValidationStatus validation_status{RouteValidationStatus::kUnvalidated};
  std::vector<std::uint64_t> source_ids;
  std::vector<std::string> validation_errors;
  bool reverse_direction{false};
  // True when an edit changes/creates centerline geometry whose orientation
  // is not represented by the trajectory-yaw rectangle collision grid.
  bool requires_orientation_collision_validation{false};
};

struct EditedRouteGraph
{
  RouteGraph graph;
  std::map<std::uint64_t, RouteEntityMetadata> node_metadata;
  std::map<std::uint64_t, RouteEntityMetadata> edge_metadata;
};

struct RouteValidationOptions
{
  bool require_verified_vehicle_dimensions{true};
  // Closed-course candidates may validate the physical platform footprint at
  // the actual route sample instead of inflating UNKNOWN by a yaw-independent
  // configuration-space radius.  For rectangles the route tangent supplies
  // yaw; circles use the same direct centre-sampled footprint test. Production
  // deliberately uses the conservative transform unless explicitly enabled.
  bool use_orientation_aware_unknown_footprint{false};
  // A driven-path closed-course candidate may use the physical swept body as
  // its UNKNOWN test while retaining the configured clearance in the already
  // supplied obstacle configuration-space grid. Production keeps this true.
  bool include_clearance_in_unknown_footprint{true};
  // Validate a rectangle at the edited Route tangent directly against the raw
  // obstacle grid.  The normal inflated grid is oriented with the measured
  // trajectory yaw and therefore cannot establish collision freedom after a
  // manual Edge is added or moved to another heading. Kept last so existing
  // positional aggregate initializers retain their meaning.
  bool use_orientation_aware_obstacle_footprint{false};
};

struct RouteValidationResult
{
  EditedRouteGraph edited;
  // Contains kValid and advisory-only kWarning edges, plus the nodes
  // referenced by those edges. Hard-invalid geometry never enters this graph.
  RouteGraph operational_graph;
  bool operational_ready{false};
  bool direct_route_footprint_unknown_validation{false};
  bool unknown_footprint_includes_clearance{true};
  std::string unknown_footprint_policy{"configuration_space_radius"};
  std::vector<std::string> validation_errors;
  bool direct_route_footprint_obstacle_validation{false};
  std::string obstacle_footprint_policy{"trajectory_yaw_configuration_space_grid"};
};

struct SplitRouteEdgeResult
{
  std::uint64_t node_id{0U};
  std::uint64_t first_edge_id{0U};
  std::uint64_t second_edge_id{0U};
  std::optional<std::uint64_t> first_reverse_edge_id;
  std::optional<std::uint64_t> second_reverse_edge_id;
};

[[nodiscard]] const char * toString(RouteEditOperationType type);
[[nodiscard]] const char * toString(RouteDirection direction);
[[nodiscard]] const char * toString(RouteProvenance provenance);
[[nodiscard]] const char * toString(RouteValidationStatus status);

[[nodiscard]] std::string routeGraphFingerprint(const RouteGraph & graph);

// Replays an overlay against an immutable generated graph. Structural or ID
// errors throw std::runtime_error; the input graph is never modified.
[[nodiscard]] EditedRouteGraph applyRouteEdits(
  const RouteGraph & generated_graph,
  const RouteEditOverlay & overlay);

[[nodiscard]] RouteValidationResult validateEditedRouteGraph(
  const EditedRouteGraph & edited_graph,
  const OccupancyGrid2D & inflated_obstacle_grid,
  // Environmental UNKNOWN mask. The validator transforms it into platform
  // configuration space; callers must not pre-inflate it.
  const OccupancyGrid2D & unknown_grid,
  const GeneratorConfig & config,
  const RouteValidationOptions & options = {},
  // Required only when use_orientation_aware_obstacle_footprint is true. This
  // is the non-inflated obstacle grid with the same geometry as the other
  // validation grids.
  const OccupancyGrid2D * raw_obstacle_grid = nullptr);

class RouteEditSession
{
public:
  explicit RouteEditSession(const RouteGraph & generated_graph);
  RouteEditSession(const RouteGraph & generated_graph, RouteEditOverlay overlay);

  [[nodiscard]] const RouteGraph & generatedGraph() const {return generated_graph_;}
  [[nodiscard]] const RouteEditOverlay & overlay() const {return overlay_;}
  [[nodiscard]] EditedRouteGraph editedGraph() const;

  void clearGraph();
  std::uint64_t addNode(const Vec3 & position);
  void moveNode(std::uint64_t node_id, const Vec3 & position);
  void deleteNode(std::uint64_t node_id, bool cascade = true);
  [[nodiscard]] std::pair<std::uint64_t, std::optional<std::uint64_t>> addEdge(
    std::uint64_t from_id,
    std::uint64_t to_id,
    std::vector<Vec3> polyline,
    RouteDirection direction = RouteDirection::kOneWay);
  void deleteEdge(std::uint64_t edge_id, bool include_reverse = true);
  [[nodiscard]] SplitRouteEdgeResult splitEdge(std::uint64_t edge_id, double split_s);
  [[nodiscard]] std::optional<std::uint64_t> setEdgeDirection(
    std::uint64_t edge_id, RouteDirection direction);

private:
  [[nodiscard]] std::uint64_t allocateEntityId();
  [[nodiscard]] std::uint64_t allocateOperationId();
  void appendAndVerify(RouteEditOperation operation);

  RouteGraph generated_graph_;
  RouteEditOverlay overlay_;
};

void saveRouteEditOverlayTsv(
  const std::filesystem::path & path, const RouteEditOverlay & overlay);
[[nodiscard]] RouteEditOverlay loadRouteEditOverlayTsv(
  const std::filesystem::path & path);

void saveRouteEditOverlayGeoJson(
  const std::filesystem::path & path, const RouteEditOverlay & overlay);
[[nodiscard]] RouteEditOverlay loadRouteEditOverlayGeoJson(
  const std::filesystem::path & path);

// Review artifact containing edited geometry plus provenance and validation
// status. It is separate from the operational graph so invalid edits remain
// inspectable without reaching a planner.
void saveEditedRouteGraphGeoJson(
  const std::filesystem::path & path, const EditedRouteGraph & graph);

// Navigation-readiness summary for operators and automated deployment gates.
// This report is deliberately separate from the geometry-generation report:
// generation can succeed while every edited edge fails operational validation.
void saveRouteValidationReportYaml(
  const std::filesystem::path & path,
  const RouteValidationResult & result,
  const GeneratorConfig & config);

}  // namespace lidar_mobility_map_generator
