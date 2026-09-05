#pragma once

#include "lidar_mobility_map_generator/config.hpp"
#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/navigation_promotion.hpp"
#include "lidar_mobility_map_generator/observed_route_graph.hpp"
#include "lidar_mobility_map_generator/pipeline.hpp"
#include "lidar_mobility_map_generator/semantic_route_graph.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

struct ClosedCourseLanelet2ExportOptions
{
  // These dimensions are allowed to be configuration estimates.  The output
  // is consequently a closed-course candidate and is never production-ready.
  double estimated_vehicle_width{0.0};
  double estimated_front_extent{0.0};
  double estimated_rear_extent{0.0};
  // Positive kinematic contract copied from RobotConfig.  This does not
  // change the swept-envelope geometry, but binds the generated map to the
  // same vehicle profile used by downstream planning/control validation.
  double estimated_minimum_turning_radius{0.0};
  double lateral_clearance_margin{0.0};
  std::string vehicle_profile{"custom"};
  std::string vehicle_base_reference{"unspecified"};
  std::string vehicle_dimensions_evidence_source{"unknown"};
  std::string vehicle_dimensions_evidence_confidence{"unknown"};
  bool vehicle_dimensions_verified{false};
  bool experimental_ready{false};
  // Identifies whether the exported Lanelet centerline follows the recorded
  // SLAM trajectory or a user-authored, independently validated topology.
  // Keep the recorded value as the default for backward-compatible exports.
  std::string centerline_source{"recorded_trajectory"};
  // Legacy diagnostic-only exporter exercise.  This creates unobserved,
  // passable predecessor/successor Lanelets outside the measured replay and
  // must remain false for every generated/public map.  It is retained only so
  // low-level tests can audit the old geometry without silently re-enabling it
  // in the application pipeline.
  bool test_only_add_open_route_planning_support{false};
  double planning_endpoint_allowance{0.50};
};

struct SyntheticOpenRoutePlanningSupport
{
  std::uint64_t edge_id{0U};
  std::uint64_t adjacent_output_edge_id{0U};
  std::uint64_t adjacent_source_edge_id{0U};
  std::uint64_t raw_endpoint_node_id{0U};
  std::string role;  // head or tail
  Vec3 raw_endpoint{};
  Vec3 synthetic_endpoint{};
  Vec2 directed_tangent{};
  Vec2 outer_directed_tangent{};
  double source_edge_length_m{0.0};
  double raw_endpoint_s_m{0.0};
  double centerline_planar_length_m{0.0};
  double centerline_3d_length_m{0.0};
  double required_boundary_beyond_raw_endpoint_m{0.0};
  double actual_left_boundary_beyond_raw_endpoint_m{0.0};
  double actual_right_boundary_beyond_raw_endpoint_m{0.0};
  // Contract-v2 deterministic test-only kinematic staging audit.  The
  // candidate index is zero-based in the documented length/family search
  // order. Rejection counts plus prior individually-valid rank account for
  // every earlier candidate; the pair fields separately audit rejection
  // against the final combined head+raw+tail swept geometry.
  std::uint32_t planning_support_contract_version{2U};
  std::string geometry_kind{"straight"};
  std::size_t selected_candidate_index{0U};
  std::size_t candidate_count_tested{0U};
  std::size_t individually_valid_candidate_rank{0U};
  std::size_t rejected_kinematic_candidates{0U};
  std::size_t rejected_invalid_geometry_candidates{0U};
  std::size_t rejected_outer_raw_overlap_candidates{0U};
  std::size_t rejected_insufficient_outer_pose_isolation_candidates{0U};
  std::size_t rejected_raw_polygon_reentry_candidates{0U};
  std::size_t rejected_nonadjacent_transition_candidates{0U};
  double search_step_m{0.0};
  double search_max_length_m{0.0};
  double turn_radius_m{0.0};
  double turn_angle_rad{0.0};
  double straight_length_m{0.0};
  double maximum_curvature_inv_m{0.0};
  double actual_maximum_curvature_inv_m{0.0};
  bool kinematic_valid{false};
  bool outer_endpoint_unique{false};
  std::vector<std::uint64_t> outer_endpoint_route_polygon_edge_ids;
  std::vector<std::uint64_t> outer_footprint_raw_overlap_edge_ids;
  // The outer base pose must not remain close enough to an unrelated replay
  // centerline to be a competing endpoint context.  The required distance is
  // derived from the configured vehicle footprint circumradius plus the same
  // planning endpoint allowance used by staging; it is not an Autoware tuning
  // constant.  With no nonadjacent raw centerline the condition is vacuous and
  // the canonical audited distance equals the required distance.
  double required_outer_pose_nonadjacent_raw_centerline_isolation_m{0.0};
  double actual_outer_pose_nonadjacent_raw_centerline_isolation_m{0.0};
  std::size_t outer_pose_nonadjacent_raw_centerline_count{0U};
  std::vector<std::uint64_t>
  outer_pose_nearest_nonadjacent_raw_centerline_edge_ids;
  bool raw_overlap_single_transition{false};
  double raw_overlap_transition_length_m{0.0};
  std::vector<std::uint64_t> nonadjacent_raw_overlap_edge_ids;
  double nonadjacent_raw_overlap_transition_length_m{0.0};
  double maximum_nonadjacent_raw_overlap_transition_length_m{0.0};
  bool outer_footprint_contained{false};
  bool connection_footprint_contained{false};
  std::size_t candidate_pool_limit{0U};
  std::size_t head_candidate_pool_size{0U};
  std::size_t tail_candidate_pool_size{0U};
  std::size_t candidate_pair_evaluation_limit{0U};
  std::size_t candidate_pairs_tested{0U};
  std::size_t selected_candidate_pair_rank{0U};
  std::size_t rejected_final_boundary_pairs{0U};
  std::size_t rejected_final_outer_membership_pairs{0U};
  std::size_t rejected_final_transition_pairs{0U};
  std::size_t rejected_final_containment_pairs{0U};
};

struct ClosedCourseLanelet2ExportSummary
{
  std::size_t source_physical_edges{0U};
  std::size_t exported_physical_edges{0U};
  // A semantic speed span may split one physical source Edge into multiple
  // Lanelets.  Keep that representation count separate from raw-map coverage.
  std::size_t exported_lanelet_segments{0U};
  double source_length{0.0};
  double exported_length{0.0};
  bool terminal_support_applied{false};
  std::uint64_t terminal_support_named_edge_id{0U};
  std::vector<std::uint64_t> terminal_support_edge_ids;
  double terminal_support_length_m{0.0};
  double named_route_source_length_m{0.0};
  // Explicit synthetic planning geometry is excluded from every raw replay
  // Edge/length/coverage count above.  This vector is the only allow-list for
  // Lanelets that may omit semantic raw-source lineage in a semantic export.
  std::vector<SyntheticOpenRoutePlanningSupport> synthetic_planning_support;
};

// Optional, already-validated navigation authoring projected onto `graph`.
// The pointers are observed only for the duration of the export call.  Stop
// lines must have been rebound with resolveStopLinesForGraph() before use.
struct Lanelet2AuthoringOptions
{
  const NamedNavigationRoute * named_route{nullptr};
  std::vector<AuthoredStopLine> resolved_stop_lines;
  // Legacy route-only export aid. Do not set this for a complete map that
  // already contains the successor Edge; the exporter rejects that duplicate
  // rather than compositing two representations of the same driven segment.
  std::optional<ClosedCourseAutowareTerminalSupport> terminal_support;
  // Optional lossless semantic segmentation lineage. Both pointers must be
  // supplied together. `semantic_source_graph` is the immutable chronological
  // replay used by acceptance; `semantic_edge_provenance` maps every exported
  // Lanelet segment back to an exact directed arc interval on that graph.
  // The exporter rejects gaps, overlaps, reordering, geometry changes, and
  // incomplete coverage before writing the OSM.
  const RouteGraph * semantic_source_graph{nullptr};
  const std::vector<SemanticRouteEdgeProvenance> * semantic_edge_provenance{nullptr};
};

void saveTrajectoryTum(
  const std::filesystem::path & path,
  const std::vector<TimedPose> & trajectory);

void saveRouteGraphGeoJson(
  const std::filesystem::path & path,
  const RouteGraph & graph);

void saveCorridorsGeoJson(
  const std::filesystem::path & path,
  const RouteGraph & graph);

void saveRouteGraphMetadataYaml(
  const std::filesystem::path & path,
  const RouteGraph & graph);

void saveAutowareReplayCandidateMetadataYaml(
  const std::filesystem::path & path,
  const AutowareReplayCandidateResult & candidate);

void saveLanelet2Osm(
  const std::filesystem::path & path,
  const RouteGraph & graph,
  const Lanelet2Config & config,
  bool autoware_ready = false,
  const Lanelet2AuthoringOptions & authoring = {});

// Writes the retained replay-candidate components as a Lanelet2 map that can
// be loaded for supervised closed-course experiments. Passage evidence from
// an observed trajectory is deliberately not presented as a clearance or
// production-safety proof. Impassable edges are omitted. Physical road
// boundaries are not inferred. Instead, each retained trajectory corridor is
// the envelope swept by the configured oriented rectangular footprint
// (front/rear extent and width), lateral clearance margin, and the explicitly
// tagged interpolation guard. Relation tags keep this output distinct from
// the production-gated canonical map.
[[nodiscard]] ClosedCourseLanelet2ExportSummary saveClosedCourseExperimentalLanelet2Osm(
  const std::filesystem::path & path,
  const RouteGraph & experimental_operational_graph,
  const Lanelet2Config & config,
  const ClosedCourseLanelet2ExportOptions & options,
  const Lanelet2AuthoringOptions & authoring = {});

void saveMapProjectorInfo(const std::filesystem::path & path);

void saveOccupancyGridYaml(
  const std::filesystem::path & path,
  const std::string & image_filename,
  const OccupancyGrid2D & grid);

void saveGenerationReport(
  const std::filesystem::path & path,
  const MappingDataset & dataset,
  const PipelineResult & result,
  const ApplicationConfig & config,
  const AutowareReplayCandidateResult * autoware_replay_candidate = nullptr);

}  // namespace lidar_mobility_map_generator
