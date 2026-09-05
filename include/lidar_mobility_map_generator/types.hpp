#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

constexpr double kPi = 3.141592653589793238462643383279502884;

// Evidence provenance is deliberately separate from the production verification
// booleans.  A nominal catalogue value or a data-derived estimate can support a
// controlled closed-course experiment, but it must never silently become a
// production measurement claim.
inline bool validEvidenceSource(const std::string & source)
{
  return source == "unknown" || source == "measured" ||
         source == "catalog_estimated" || source == "inferred";
}

inline bool validEvidenceConfidence(const std::string & confidence)
{
  return confidence == "unknown" || confidence == "low" ||
         confidence == "medium" || confidence == "high";
}

inline bool evidenceSupportsClosedCourseExperiment(
  const std::string & source, const std::string & confidence)
{
  return source != "unknown" && validEvidenceSource(source) &&
         (confidence == "medium" || confidence == "high");
}

inline bool evidenceSupportsProductionVerification(
  const std::string & source, const std::string & confidence)
{
  return source == "measured" && confidence == "high";
}

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Vec2 operator+(const Vec2 & lhs, const Vec2 & rhs) {return {lhs.x + rhs.x, lhs.y + rhs.y};}
inline Vec2 operator-(const Vec2 & lhs, const Vec2 & rhs) {return {lhs.x - rhs.x, lhs.y - rhs.y};}
inline Vec2 operator*(const Vec2 & value, const double scale) {return {value.x * scale, value.y * scale};}
inline Vec2 operator/(const Vec2 & value, const double scale) {return {value.x / scale, value.y / scale};}
inline Vec3 operator+(const Vec3 & lhs, const Vec3 & rhs) {return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};}
inline Vec3 operator-(const Vec3 & lhs, const Vec3 & rhs) {return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};}
inline Vec3 operator*(const Vec3 & value, const double scale) {return {value.x * scale, value.y * scale, value.z * scale};}
inline Vec3 operator/(const Vec3 & value, const double scale) {return {value.x / scale, value.y / scale, value.z / scale};}
inline Vec3 & operator+=(Vec3 & lhs, const Vec3 & rhs)
{
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  lhs.z += rhs.z;
  return lhs;
}

inline double dot(const Vec2 & lhs, const Vec2 & rhs) {return lhs.x * rhs.x + lhs.y * rhs.y;}
inline double dot(const Vec3 & lhs, const Vec3 & rhs) {return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;}
inline double normSquared(const Vec2 & value) {return dot(value, value);}
inline double normSquared(const Vec3 & value) {return dot(value, value);}
inline double norm(const Vec2 & value) {return std::sqrt(normSquared(value));}
inline double norm(const Vec3 & value) {return std::sqrt(normSquared(value));}
inline double distance2d(const Vec3 & lhs, const Vec3 & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}
inline double distance3d(const Vec3 & lhs, const Vec3 & rhs) {return norm(lhs - rhs);}
inline Vec2 normalized(const Vec2 & value)
{
  const double length = norm(value);
  return length > 1.0e-12 ? value / length : Vec2{};
}
inline Vec3 normalized(const Vec3 & value)
{
  const double length = norm(value);
  return length > 1.0e-12 ? value / length : Vec3{};
}
inline bool finite(const Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
inline double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}
inline double normalizeAngle(double value)
{
  while (value > kPi) {value -= 2.0 * kPi;}
  while (value < -kPi) {value += 2.0 * kPi;}
  return value;
}
inline double undirectedHeadingDifference(const double lhs, const double rhs)
{
  double difference = std::abs(normalizeAngle(lhs - rhs));
  return std::min(difference, std::abs(kPi - difference));
}

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};

  [[nodiscard]] bool isFinite() const;
  [[nodiscard]] double squaredNorm() const;
  [[nodiscard]] Quaternion normalized() const;
  [[nodiscard]] Quaternion inverse() const;
  [[nodiscard]] Quaternion operator*(const Quaternion & rhs) const;
  [[nodiscard]] Vec3 rotate(const Vec3 & point) const;
  [[nodiscard]] double yaw() const;

  static Quaternion fromYaw(double yaw);
  static Quaternion slerp(const Quaternion & lhs, const Quaternion & rhs, double ratio);
};

struct Transform
{
  Vec3 translation{};
  Quaternion rotation{};

  [[nodiscard]] Vec3 apply(const Vec3 & point) const;
  [[nodiscard]] Transform inverse() const;
  [[nodiscard]] Transform operator*(const Transform & rhs) const;

  static Transform interpolate(const Transform & lhs, const Transform & rhs, double ratio);
};

struct TimedPose
{
  std::int64_t stamp_ns{0};
  Transform world_from_body{};
};

struct PointXYZI
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  // Number of distinct input scans which contributed to this representative
  // point.  Completed PLY/PCD maps do not carry this history and therefore
  // keep the conservative default of one observation.
  std::uint32_t observation_count{1U};

  [[nodiscard]] Vec3 position() const
  {
    return {static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)};
  }

  [[nodiscard]] bool finite() const
  {
    return lidar_mobility_map_generator::finite(position()) && std::isfinite(intensity);
  }
};

struct DatasetStatistics
{
  std::size_t pose_messages{0U};
  std::size_t pointcloud_messages{0U};
  std::size_t used_pointcloud_messages{0U};
  std::size_t dropped_pointcloud_messages{0U};
  std::size_t decoded_points{0U};
  std::size_t accepted_points{0U};
  std::size_t map_voxels{0U};
};

struct MappingDataset
{
  std::vector<PointXYZI> map_points;
  std::vector<TimedPose> trajectory;
  std::string world_frame{"map"};
  DatasetStatistics statistics;
  std::vector<std::string> warnings;
};

struct TrajectoryConfig
{
  double minimum_translation{0.03};
  double maximum_pose_jump{2.0};
  double maximum_speed_mps{20.0};
  double resample_interval{0.20};
  // The measured path is the only default FREE-space evidence. Unconstrained
  // smoothing can cut a corner outside that swept body and therefore remains
  // explicit opt-in until containment-constrained smoothing is implemented.
  double smoothing_window{0.0};
};

struct MapBuilderConfig
{
  double voxel_size{0.08};
  double minimum_range{0.5};
  double maximum_range{80.0};
  double minimum_z{-10.0};
  double maximum_z{10.0};
  std::size_t minimum_observations_per_voxel{1U};
  std::size_t scan_stride{1U};
  std::size_t point_stride{1U};
};

struct RobotConfig
{
  double width{0.70};
  double clearance_margin{0.15};
  // Named profiles provide explicit platform semantics while every dimension
  // remains overridable.  "custom" preserves the legacy circular model.
  std::string profile{"custom"};       // custom, small_robot, car, yaris
  // Physical point represented by base_link.  Autoware vehicle models use the
  // rear-axle centre projected onto the ground; "unspecified" prevents a
  // diagnostic template from silently claiming that convention.
  // Values: unspecified, body_center, rear_axle_ground_projection.
  std::string base_reference{"unspecified"};
  std::string footprint_model{"circle"};  // circle or rectangle
  double front_extent{0.35};            // base_link to front bumper [m]
  double rear_extent{0.35};             // base_link to rear bumper [m]
  // Only returns intersecting the physical vertical envelope can make the
  // platform collide.  Values are relative to the locally estimated ground.
  double minimum_collision_height{0.08};
  double maximum_collision_height{2.00};
  // Provenance of width/extents/collision envelope.  `dimensions_verified`
  // remains the explicit production gate; source/confidence only describe the
  // evidence and may qualify a separately labelled closed-course candidate.
  std::string dimensions_source{"unknown"};
  std::string dimensions_confidence{"unknown"};
  bool dimensions_verified{false};
  double minimum_turning_radius{0.0};
  bool allow_in_place_rotation{true};
  bool allow_reverse_motion{true};
};

struct TraversabilityConfig
{
  double grid_resolution{0.10};
  double trajectory_crop_radius{6.0};
  double ground_estimation_radius{1.0};
  double ground_quantile{0.15};
  std::size_t minimum_ground_points{6U};
  double ground_search_min_offset{-2.0};
  double ground_search_max_offset{0.5};
  double fallback_ground_z_offset{-0.30};
  double minimum_obstacle_height{0.08};
  double maximum_obstacle_height{2.00};
  // Optional free-space evidence from the driven path. Zero keeps every
  // classified obstacle; a positive value clears raw obstacle cells within
  // this radius before robot-footprint inflation.
  double observed_trajectory_clearance_radius{0.0};
  // Explicit source selection for FREE cells. `ground_observations` accepts
  // only direct, locally supported ground returns; interpolated ground is not
  // free-space evidence. `combined` unions it with the trajectory evidence.
  std::string free_space_evidence_mode{"trajectory"};
  std::size_t minimum_ground_free_points_per_cell{1U};
  double maximum_ground_free_height{0.08};
  // `disk` retains the legacy radius-based sweep. `footprint` sweeps the
  // orientation-aware configured body after inward erosion for uncertainty.
  std::string trajectory_free_space_model{"disk"};
  double trajectory_footprint_erosion_margin{0.0};
  double maximum_corridor_half_width{3.0};
  double ray_step{0.05};
  double boundary_margin{0.05};
  double minimum_safe_center_width{0.20};
  std::size_t maximum_grid_cells{50000000U};
  // XY-local 2.5D ground surface.  Per-cell low quantiles are combined by a
  // bounded local plane instead of extending one trajectory height laterally.
  double ground_cell_resolution{0.50};
  std::size_t minimum_ground_points_per_cell{3U};
  double ground_plane_radius{1.50};
  std::size_t minimum_ground_cells_for_plane{3U};
  double maximum_ground_slope{0.35};
  double maximum_ground_plane_residual{0.12};
  // Suppress isolated map returns while retaining spatially supported thin
  // structures such as curbs.  Observation support is effective for rosbag
  // scans; completed maps explicitly have observation_count == 1.
  std::size_t minimum_obstacle_points_per_cell{2U};
  std::size_t obstacle_support_radius_cells{1U};
  std::size_t minimum_obstacle_neighbor_points{3U};
  std::size_t minimum_obstacle_observations{1U};
  // Flattened point maps contain no ray-level free-space history.  The only
  // optional free evidence is the explicitly configured swept trajectory.
  std::string unknown_space_policy{"occupied"};  // allow or occupied
  // Maximum longitudinal change of either corridor boundary [lateral m per
  // centerline m].  The limiter only shrinks raw raycast clearances.
  double maximum_clearance_slope{1.0};
};

struct TopologyConfig
{
  double node_merge_distance{0.45};
  double intersection_merge_distance{0.30};
  double same_path_heading_threshold_deg{25.0};
  double intersection_heading_threshold_deg{50.0};
  double minimum_loop_separation{2.0};
  double minimum_edge_length{0.50};
  double maximum_edge_length{20.0};
  // Arc-length window used to regularize cluster-centroid geometry after
  // revisit merging. Zero disables it. This prevents visit/centroid jitter
  // from becoming artificial steering oscillation or invalid Lanelet offsets.
  double geometry_smoothing_window{1.0};
  // Split compressed chains at sharp changes in the observed direction.  A
  // larger reversal is additionally treated as a cusp, so smoothing or graph
  // compression cannot silently connect the two sides through one offset
  // corridor.
  double edge_split_heading_change_deg{75.0};
  double cusp_heading_change_deg{120.0};
  bool generate_reverse_edges{false};
};

struct Lanelet2Config
{
  std::string subtype{"road"};
  std::string location{"urban"};
  std::string participant{"vehicle"};
  std::string boundary_type{"virtual"};
  std::string boundary_subtype{};
  bool one_way{true};
  double speed_limit_mps{1.0};
  // Explicit, data-set-specific verification that a short terminal cusp is a
  // localization-settling artifact.  This never changes the lossless route or
  // Nav2 replay; it only permits the fail-closed Autoware derivative to omit
  // a tail that also passes all fixed geometric gates.
  bool terminal_localization_settling_verified{false};
};

struct GeneratorConfig
{
  TrajectoryConfig trajectory;
  MapBuilderConfig map_builder;
  RobotConfig robot;
  TraversabilityConfig traversability;
  TopologyConfig topology;
  Lanelet2Config lanelet2;
};

enum class RouteNodeType
{
  kEndpoint,
  kJunction,
  kNormal
};

struct RouteNode
{
  std::uint64_t id{0U};
  Vec3 position{};
  RouteNodeType type{RouteNodeType::kNormal};
};

struct RouteEdge
{
  std::uint64_t id{0U};
  std::uint64_t from{0U};
  std::uint64_t to{0U};
  std::optional<std::uint64_t> reverse_of;
  std::vector<Vec3> centerline;
  std::vector<Vec3> left_boundary;
  std::vector<Vec3> right_boundary;
  std::vector<double> left_clearance;
  std::vector<double> right_clearance;
  double length{0.0};
  double minimum_safe_width{0.0};
  double maximum_curvature{0.0};
  double confidence{0.0};
  double recommended_speed_mps{0.0};
  bool passable{false};
  // An empty cell in the binary obstacle grid is not proof that the area was
  // observed free.  These masks record whether each lateral clearance ended
  // at an observed obstacle rather than at the grid/raycast limit.
  std::vector<std::uint8_t> left_clearance_observed;
  std::vector<std::uint8_t> right_clearance_observed;
  bool corridor_geometry_valid{false};
  std::vector<std::string> validation_errors;
};

struct RouteGraph
{
  std::string frame_id{"map"};
  std::vector<RouteNode> nodes;
  std::vector<RouteEdge> edges;
};

struct GenerationStatistics
{
  std::size_t raw_trajectory_poses{0U};
  std::size_t processed_trajectory_poses{0U};
  bool raw_trajectory_preserved{true};
  std::size_t corrected_position_jitter_poses{0U};
  std::size_t corrected_position_jitter_runs{0U};
  double maximum_planar_position_correction_m{0.0};
  double planar_length_before_position_jitter_correction_m{0.0};
  double planar_length_after_position_jitter_correction_m{0.0};
  std::size_t obstacle_points{0U};
  std::size_t obstacle_cells{0U};
  std::size_t trajectory_cleared_obstacle_cells{0U};
  std::size_t inflated_obstacle_cells{0U};
  std::size_t route_nodes{0U};
  std::size_t route_edges{0U};
  std::size_t physical_route_edges{0U};
  std::size_t passable_physical_edges{0U};
  std::size_t impassable_edges{0U};
  std::size_t impassable_physical_edges{0U};
  double trajectory_length{0.0};
  double minimum_safe_width{std::numeric_limits<double>::infinity()};
};

// Audit for the immutable chronological replay geometry.  Routing partitions
// are measured in planar XY, while both planar and spatial lengths are
// retained so a Z correction cannot be mistaken for route shortening.
struct RouteGeometryAudit
{
  bool valid{true};
  std::size_t source_pose_count{0U};
  std::size_t source_segments_evaluated{0U};
  std::size_t represented_source_pose_count{0U};
  std::size_t segments_evaluated{0U};
  std::size_t nonfinite_segments{0U};
  std::size_t zero_horizontal_distance_z_change_segments{0U};
  double source_pose_projection_coverage{0.0};
  double source_planar_length{0.0};
  double route_planar_length{0.0};
  double planar_length_coverage{0.0};
  double source_spatial_length{0.0};
  double route_spatial_length{0.0};
  double spatial_length_coverage{0.0};
  Vec3 source_start;
  Vec3 source_end;
  double maximum_absolute_delta_z{0.0};
  std::uint64_t maximum_delta_z_edge_id{0U};
  std::size_t maximum_delta_z_segment_index{0U};
  double maximum_absolute_grade{0.0};
  std::uint64_t maximum_grade_edge_id{0U};
  std::size_t maximum_grade_segment_index{0U};
  std::uint64_t first_invalid_edge_id{0U};
  std::size_t first_invalid_segment_index{0U};
  std::string first_invalid_reason;
};

struct GenerationResult
{
  std::vector<TimedPose> processed_trajectory;
  // Lossless, chronological replay chain derived from the processed base_link
  // trajectory. It records where the platform was observed to drive; local
  // ground estimates remain traversability evidence and never replace its Z.
  RouteGraph observed_route_graph;
  RouteGeometryAudit observed_route_geometry_audit;
  RouteGraph graph;
  RouteGeometryAudit topology_route_geometry_audit;
  GenerationStatistics statistics;
  std::vector<std::string> warnings;
};

[[nodiscard]] const char * toString(RouteNodeType type);
[[nodiscard]] double polylineLength(const std::vector<Vec3> & points);

}  // namespace lidar_mobility_map_generator
