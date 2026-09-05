#pragma once

#include "lidar_mobility_map_generator/config.hpp"
#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/pipeline.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

// Explicit session controls for a low-speed, access-controlled experiment.
// These are deliberately independent of the production verification flags.
// A caller must collect each acknowledgement for the actual run; merely
// generating a candidate map never makes an experiment ready.
struct Nav2ClosedCourseControls
{
  bool enabled{false};
  bool operator_acknowledged_experimental_only{false};
  bool estimated_geometry_acknowledged{false};
  bool closed_course_access_controlled{false};
  bool free_space_reviewed_for_session{false};
  bool localization_alignment_checked_for_session{false};
  bool emergency_stop_available{false};
  std::string base_frame{"base_link"};
  std::string odom_frame{"odom"};
  // PointCloud2 used by the local ObstacleLayer for live marking and raytrace
  // clearing. The generated static map remains the global planning source.
  std::string obstacle_pointcloud_topic{"/points_raw"};
  double maximum_linear_speed_mps{0.25};
  double maximum_angular_speed_rps{0.60};
  double maximum_waypoint_spacing{0.50};
  double cost_scaling_factor{5.0};
};

struct Nav2WaypointPose
{
  Vec3 position{};
  double yaw{0.0};
  std::optional<std::uint64_t> authored_stop_line_id;
  std::optional<std::string> authored_stop_line_name;
  std::optional<std::uint64_t> authored_stop_edge_id;
  std::optional<double> authored_stop_edge_s_m;
  std::optional<double> authored_stop_width_m;
};

// A maximal, non-branching directed chain. Branches become separate routes so
// FollowWaypoints/NavigateThroughPoses never receives a discontinuous jump.
struct Nav2WaypointRoute
{
  std::size_t id{0U};
  bool closed_loop{false};
  std::vector<std::uint64_t> source_edge_ids;
  std::vector<Nav2WaypointPose> waypoints;
  std::optional<std::string> name;
  std::optional<NavigationAuthoringTarget> target;
  std::optional<std::uint64_t> source_named_route_id;
};

struct Nav2ClosedCourseAssessment
{
  // File-format/geometry compatibility is not a safety or production claim.
  bool map_server_compatible{false};
  bool follow_waypoints_compatible{false};
  bool route_server_compatible{false};
  bool classification_partition_complete{false};
  bool direct_free_space_evidence_selected{false};

  // Artifact readiness covers data/format/evidence quality and may use
  // explicitly identified estimated geometry. Deployment readiness additionally
  // requires the per-session operator controls above. The two must not be
  // collapsed into one "ready" claim.
  bool static_map_artifact_ready{false};
  bool follow_waypoints_artifact_ready{false};
  bool route_server_artifact_ready{false};
  bool closed_course_artifact_ready{false};
  bool static_map_deployment_ready{false};
  bool follow_waypoints_deployment_ready{false};
  bool route_server_deployment_ready{false};
  bool closed_course_deployment_ready{false};

  std::size_t obstacle_cells{0U};
  std::size_t explicit_free_cells{0U};
  std::size_t unknown_cells{0U};
  std::size_t unclassified_cells{0U};
  std::size_t overlapping_classification_cells{0U};
  std::size_t passable_route_edges{0U};
  std::size_t invalid_passable_route_edges{0U};
  std::size_t waypoint_routes{0U};
  std::size_t waypoints{0U};
  std::size_t route_obstacle_samples{0U};
  std::size_t route_unknown_samples{0U};
  std::size_t route_off_map_samples{0U};
  double costmap_inflation_radius{0.0};

  std::vector<std::string> map_blockers;
  std::vector<std::string> follow_waypoints_blockers;
  std::vector<std::string> route_server_blockers;
  std::vector<std::string> deployment_blockers;
  std::vector<std::string> limitations;
};

struct Nav2ClosedCourseArtifacts
{
  std::filesystem::path map_pgm;
  std::filesystem::path map_yaml;
  std::filesystem::path route_graph_geojson;
  std::filesystem::path waypoint_routes_yaml;
  std::filesystem::path nav2_params_overlay_yaml;
  std::filesystem::path readiness_yaml;
};

[[nodiscard]] std::vector<Nav2WaypointRoute> buildNav2WaypointRoutes(
  const RouteGraph & graph,
  double maximum_waypoint_spacing,
  // Retained for configuration compatibility; source vertices are never
  // removed by chord-error simplification.
  double maximum_chord_error = 0.10);

[[nodiscard]] Nav2ClosedCourseAssessment evaluateNav2ClosedCourseExperiment(
  const PipelineResult & pipeline,
  const RouteGraph & closed_course_graph,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & controls);

void saveNav2WaypointRoutesYaml(
  const std::filesystem::path & path,
  const std::string & frame_id,
  const std::vector<Nav2WaypointRoute> & routes,
  bool experiment_ready);

void saveNav2ClosedCourseParamsOverlayYaml(
  const std::filesystem::path & path,
  const std::filesystem::path & map_yaml,
  const std::filesystem::path & route_graph_geojson,
  const RobotConfig & robot,
  const Nav2ClosedCourseControls & controls,
  const Nav2ClosedCourseAssessment & assessment);

void saveNav2ClosedCourseReadinessYaml(
  const std::filesystem::path & path,
  const Nav2ClosedCourseAssessment & assessment,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & controls);

// Writes a separately named experimental bundle. The production canonical
// nav2_map.yaml/nav2_route_graph.geojson are never read or overwritten. When
// the static-map gates fail, the emitted experimental PGM is all UNKNOWN; when
// the route gates fail, the GeoJSON and waypoint route list are empty. Resolved
// stop lines are inserted as exact FollowWaypoints arrival poses on a selected
// named Route; they do not by themselves command a dwell or held stop.
[[nodiscard]] Nav2ClosedCourseAssessment saveNav2ClosedCourseExperimentalBundle(
  const Nav2ClosedCourseArtifacts & artifacts,
  const PipelineResult & pipeline,
  const RouteGraph & closed_course_graph,
  const ApplicationConfig & config,
  const Nav2ClosedCourseControls & controls,
  const NamedNavigationRoute * named_route = nullptr,
  const std::vector<AuthoredStopLine> * resolved_stop_lines = nullptr);

}  // namespace lidar_mobility_map_generator
