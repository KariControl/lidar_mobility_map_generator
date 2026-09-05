#include "lidar_mobility_map_generator/ros_parameters.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  return node.declare_parameter<T>(name, default_value);
}

std::size_t nonnegativeSize(const std::int64_t value, const std::string & name)
{
  if (value < 0) {
    throw std::runtime_error(name + " must not be negative");
  }
  return static_cast<std::size_t>(value);
}

void requirePositive(const double value, const std::string & name)
{
  if (!std::isfinite(value) || !(value > 0.0)) {
    throw std::runtime_error(name + " must be finite and greater than zero");
  }
}

void requireNonnegative(const double value, const std::string & name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(name + " must be finite and nonnegative");
  }
}

void requireOrdered(
  const double minimum, const double maximum,
  const std::string & minimum_name, const std::string & maximum_name)
{
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || !(minimum < maximum)) {
    throw std::runtime_error(minimum_name + " must be smaller than " + maximum_name);
  }
}

void requireUnitInterval(const double value, const std::string & name)
{
  if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
    throw std::runtime_error(name + " must be in [0, 1]");
  }
}

RobotConfig robotProfileDefaults(const std::string & profile)
{
  RobotConfig robot;
  robot.profile = profile;
  if (profile == "custom") {
    return robot;
  }
  if (profile == "small_robot") {
    robot.footprint_model = "circle";
    robot.width = 0.50;
    robot.front_extent = 0.25;
    robot.rear_extent = 0.25;
    robot.clearance_margin = 0.10;
    robot.minimum_collision_height = 0.08;
    robot.maximum_collision_height = 0.80;
    robot.minimum_turning_radius = 0.0;
    robot.allow_in_place_rotation = true;
    robot.allow_reverse_motion = true;
    return robot;
  }
  if (profile == "car") {
    robot.footprint_model = "rectangle";
    robot.width = 1.80;
    robot.front_extent = 3.20;
    robot.rear_extent = 1.00;
    robot.clearance_margin = 0.15;
    robot.minimum_collision_height = 0.12;
    robot.maximum_collision_height = 2.20;
    robot.minimum_turning_radius = 4.50;
    robot.allow_in_place_rotation = false;
    robot.allow_reverse_motion = true;
    return robot;
  }
  if (profile == "yaris") {
    // Unverified nominal Yaris envelope expressed about Autoware's base_link
    // convention: the rear-axle centre projected onto the ground.  The
    // 3.35/0.60 m split is a diagnostic hypothesis and still requires direct
    // measurement of this vehicle, including mirrors and mounted equipment.
    robot.base_reference = "rear_axle_ground_projection";
    robot.footprint_model = "rectangle";
    robot.width = 1.695;
    robot.front_extent = 3.35;
    robot.rear_extent = 0.60;
    robot.clearance_margin = 0.15;
    robot.minimum_collision_height = 0.12;
    robot.maximum_collision_height = 2.20;
    robot.dimensions_source = "catalog_estimated";
    robot.dimensions_confidence = "medium";
    robot.minimum_turning_radius = 5.10;
    robot.allow_in_place_rotation = false;
    robot.allow_reverse_motion = true;
    return robot;
  }
  throw std::runtime_error("robot.profile must be custom, small_robot, car, or yaris");
}

}  // namespace

ApplicationConfig loadApplicationConfig(rclcpp::Node & node)
{
  ApplicationConfig config;
  config.input_type = parameter(node, "input.type", config.input_type);

  config.rosbag2.bag_path = parameter(
    node, "input.rosbag2.path", config.rosbag2.bag_path.string());
  config.rosbag2.storage_id = parameter(
    node, "input.rosbag2.storage_id", config.rosbag2.storage_id);
  config.rosbag2.pointcloud_topic = parameter(
    node, "input.rosbag2.pointcloud_topic", config.rosbag2.pointcloud_topic);
  config.rosbag2.pointcloud_mode = parameter(
    node, "input.rosbag2.pointcloud_mode", config.rosbag2.pointcloud_mode);
  config.rosbag2.pose_source = parameter(
    node, "input.rosbag2.pose_source", config.rosbag2.pose_source);
  config.rosbag2.pose_topic = parameter(
    node, "input.rosbag2.pose_topic", config.rosbag2.pose_topic);
  config.rosbag2.tf_topic = parameter(
    node, "input.rosbag2.tf_topic", config.rosbag2.tf_topic);
  config.rosbag2.tf_static_topic = parameter(
    node, "input.rosbag2.tf_static_topic", config.rosbag2.tf_static_topic);
  config.rosbag2.world_frame = parameter(
    node, "input.rosbag2.world_frame", config.rosbag2.world_frame);
  config.rosbag2.base_frame = parameter(
    node, "input.rosbag2.base_frame", config.rosbag2.base_frame);
  config.rosbag2.sensor_frame = parameter(
    node, "input.rosbag2.sensor_frame", config.rosbag2.sensor_frame);
  config.rosbag2.pose_reference_frame = parameter(
    node, "input.rosbag2.pose_reference_frame", config.rosbag2.pose_reference_frame);
  config.rosbag2.strict_frame_check = parameter(
    node, "input.rosbag2.strict_frame_check", config.rosbag2.strict_frame_check);
  config.rosbag2.use_header_stamp = parameter(
    node, "input.rosbag2.use_header_stamp", config.rosbag2.use_header_stamp);
  config.rosbag2.maximum_pose_gap_sec = parameter(
    node, "input.rosbag2.maximum_pose_gap_sec", config.rosbag2.maximum_pose_gap_sec);
  config.rosbag2.deskew.enabled = parameter(
    node, "input.rosbag2.deskew.enabled", config.rosbag2.deskew.enabled);
  config.rosbag2.deskew.point_time_field = parameter(
    node, "input.rosbag2.deskew.point_time_field", config.rosbag2.deskew.point_time_field);
  config.rosbag2.deskew.point_time_reference = parameter(
    node, "input.rosbag2.deskew.point_time_reference",
    config.rosbag2.deskew.point_time_reference);
  config.rosbag2.deskew.point_time_scale_sec = parameter(
    node, "input.rosbag2.deskew.point_time_scale_sec",
    config.rosbag2.deskew.point_time_scale_sec);
  config.rosbag2.deskew.point_time_offset_sec = parameter(
    node, "input.rosbag2.deskew.point_time_offset_sec",
    config.rosbag2.deskew.point_time_offset_sec);

  config.glim.map_path = parameter(
    node, "input.glim.map_path", config.glim.map_path.string());
  config.glim.trajectory_path = parameter(
    node, "input.glim.trajectory_path", config.glim.trajectory_path.string());
  config.glim.trajectory_frame = parameter(
    node, "input.glim.trajectory_frame", config.glim.trajectory_frame);
  config.glim.world_frame = parameter(
    node, "input.glim.world_frame", config.glim.world_frame);

  config.extrinsics.source = parameter(
    node, "extrinsics.source", config.extrinsics.source);
  config.extrinsics.calibration_source = parameter(
    node, "extrinsics.calibration_source", config.extrinsics.calibration_source);
  config.extrinsics.calibration_confidence = parameter(
    node, "extrinsics.calibration_confidence", config.extrinsics.calibration_confidence);
  config.extrinsics.verified = parameter(
    node, "extrinsics.verified", config.extrinsics.verified);
  const std::vector<double> translation = parameter<std::vector<double>>(
    node, "extrinsics.translation", {0.0, 0.0, 0.0});
  const std::vector<double> quaternion = parameter<std::vector<double>>(
    node, "extrinsics.quaternion_xyzw", {0.0, 0.0, 0.0, 1.0});
  if (translation.size() != 3U || quaternion.size() != 4U) {
    throw std::runtime_error(
            "extrinsics.translation requires 3 values and quaternion_xyzw requires 4 values");
  }
  Transform requested_base_from_sensor;
  requested_base_from_sensor.translation = {translation[0], translation[1], translation[2]};
  const Quaternion raw_extrinsic_rotation{
    quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
  if (!raw_extrinsic_rotation.isFinite() ||
    raw_extrinsic_rotation.squaredNorm() < 1.0e-15)
  {
    throw std::runtime_error("extrinsics quaternion must be finite and nonzero");
  }
  requested_base_from_sensor.rotation = raw_extrinsic_rotation;
  config.extrinsics.requested_base_from_sensor = requested_base_from_sensor;
  config.extrinsics.base_from_sensor.translation = requested_base_from_sensor.translation;
  config.extrinsics.base_from_sensor.rotation = raw_extrinsic_rotation.normalized();

  config.generator.map_builder.voxel_size = parameter(
    node, "map_builder.voxel_size", config.generator.map_builder.voxel_size);
  config.generator.map_builder.minimum_range = parameter(
    node, "map_builder.minimum_range", config.generator.map_builder.minimum_range);
  config.generator.map_builder.maximum_range = parameter(
    node, "map_builder.maximum_range", config.generator.map_builder.maximum_range);
  config.generator.map_builder.minimum_z = parameter(
    node, "map_builder.minimum_z", config.generator.map_builder.minimum_z);
  config.generator.map_builder.maximum_z = parameter(
    node, "map_builder.maximum_z", config.generator.map_builder.maximum_z);
  config.generator.map_builder.minimum_observations_per_voxel = nonnegativeSize(parameter(
      node, "map_builder.minimum_observations_per_voxel",
      static_cast<std::int64_t>(config.generator.map_builder.minimum_observations_per_voxel)),
    "map_builder.minimum_observations_per_voxel");
  config.generator.map_builder.scan_stride = nonnegativeSize(parameter(
      node, "map_builder.scan_stride",
      static_cast<std::int64_t>(config.generator.map_builder.scan_stride)),
    "map_builder.scan_stride");
  config.generator.map_builder.point_stride = nonnegativeSize(parameter(
      node, "map_builder.point_stride",
      static_cast<std::int64_t>(config.generator.map_builder.point_stride)),
    "map_builder.point_stride");

  config.generator.trajectory.minimum_translation = parameter(
    node, "trajectory.minimum_translation", config.generator.trajectory.minimum_translation);
  config.generator.trajectory.maximum_pose_jump = parameter(
    node, "trajectory.maximum_pose_jump", config.generator.trajectory.maximum_pose_jump);
  config.generator.trajectory.maximum_speed_mps = parameter(
    node, "trajectory.maximum_speed_mps", config.generator.trajectory.maximum_speed_mps);
  config.generator.trajectory.resample_interval = parameter(
    node, "trajectory.resample_interval", config.generator.trajectory.resample_interval);
  config.generator.trajectory.smoothing_window = parameter(
    node, "trajectory.smoothing_window", config.generator.trajectory.smoothing_window);

  const std::string robot_profile = parameter(
    node, "robot.profile", config.generator.robot.profile);
  config.generator.robot = robotProfileDefaults(robot_profile);
  config.generator.robot.base_reference = parameter(
    node, "robot.base_reference", config.generator.robot.base_reference);
  config.generator.robot.footprint_model = parameter(
    node, "robot.footprint_model", config.generator.robot.footprint_model);
  config.generator.robot.width = parameter(
    node, "robot.width", config.generator.robot.width);
  config.generator.robot.clearance_margin = parameter(
    node, "robot.clearance_margin", config.generator.robot.clearance_margin);
  config.generator.robot.front_extent = parameter(
    node, "robot.front_extent", config.generator.robot.front_extent);
  config.generator.robot.rear_extent = parameter(
    node, "robot.rear_extent", config.generator.robot.rear_extent);
  config.generator.robot.minimum_collision_height = parameter(
    node, "robot.minimum_collision_height",
    config.generator.robot.minimum_collision_height);
  config.generator.robot.maximum_collision_height = parameter(
    node, "robot.maximum_collision_height",
    config.generator.robot.maximum_collision_height);
  config.generator.robot.dimensions_source = parameter(
    node, "robot.dimensions_source", config.generator.robot.dimensions_source);
  config.generator.robot.dimensions_confidence = parameter(
    node, "robot.dimensions_confidence", config.generator.robot.dimensions_confidence);
  config.generator.robot.dimensions_verified = parameter(
    node, "robot.dimensions_verified", config.generator.robot.dimensions_verified);
  config.generator.robot.minimum_turning_radius = parameter(
    node, "robot.minimum_turning_radius", config.generator.robot.minimum_turning_radius);
  config.generator.robot.allow_in_place_rotation = parameter(
    node, "robot.allow_in_place_rotation", config.generator.robot.allow_in_place_rotation);
  config.generator.robot.allow_reverse_motion = parameter(
    node, "robot.allow_reverse_motion", config.generator.robot.allow_reverse_motion);

  config.generator.traversability.grid_resolution = parameter(
    node, "traversability.grid_resolution",
    config.generator.traversability.grid_resolution);
  config.generator.traversability.trajectory_crop_radius = parameter(
    node, "traversability.trajectory_crop_radius",
    config.generator.traversability.trajectory_crop_radius);
  config.generator.traversability.ground_estimation_radius = parameter(
    node, "traversability.ground_estimation_radius",
    config.generator.traversability.ground_estimation_radius);
  config.generator.traversability.ground_quantile = parameter(
    node, "traversability.ground_quantile",
    config.generator.traversability.ground_quantile);
  config.generator.traversability.minimum_ground_points = nonnegativeSize(parameter(
      node, "traversability.minimum_ground_points",
      static_cast<std::int64_t>(config.generator.traversability.minimum_ground_points)),
    "traversability.minimum_ground_points");
  config.generator.traversability.ground_search_min_offset = parameter(
    node, "traversability.ground_search_min_offset",
    config.generator.traversability.ground_search_min_offset);
  config.generator.traversability.ground_search_max_offset = parameter(
    node, "traversability.ground_search_max_offset",
    config.generator.traversability.ground_search_max_offset);
  config.generator.traversability.fallback_ground_z_offset = parameter(
    node, "traversability.fallback_ground_z_offset",
    config.generator.traversability.fallback_ground_z_offset);
  config.generator.traversability.minimum_obstacle_height = parameter(
    node, "traversability.minimum_obstacle_height",
    config.generator.traversability.minimum_obstacle_height);
  config.generator.traversability.maximum_obstacle_height = parameter(
    node, "traversability.maximum_obstacle_height",
    config.generator.traversability.maximum_obstacle_height);
  config.generator.traversability.observed_trajectory_clearance_radius = parameter(
    node, "traversability.observed_trajectory_clearance_radius",
    config.generator.traversability.observed_trajectory_clearance_radius);
  config.generator.traversability.free_space_evidence_mode = parameter(
    node, "traversability.free_space_evidence_mode",
    config.generator.traversability.free_space_evidence_mode);
  config.generator.traversability.minimum_ground_free_points_per_cell = nonnegativeSize(parameter(
      node, "traversability.minimum_ground_free_points_per_cell",
      static_cast<std::int64_t>(
        config.generator.traversability.minimum_ground_free_points_per_cell)),
    "traversability.minimum_ground_free_points_per_cell");
  config.generator.traversability.maximum_ground_free_height = parameter(
    node, "traversability.maximum_ground_free_height",
    config.generator.traversability.maximum_ground_free_height);
  config.generator.traversability.trajectory_free_space_model = parameter(
    node, "traversability.trajectory_free_space_model",
    config.generator.traversability.trajectory_free_space_model);
  config.generator.traversability.trajectory_footprint_erosion_margin = parameter(
    node, "traversability.trajectory_footprint_erosion_margin",
    config.generator.traversability.trajectory_footprint_erosion_margin);
  config.generator.traversability.maximum_corridor_half_width = parameter(
    node, "traversability.maximum_corridor_half_width",
    config.generator.traversability.maximum_corridor_half_width);
  config.generator.traversability.ray_step = parameter(
    node, "traversability.ray_step", config.generator.traversability.ray_step);
  config.generator.traversability.boundary_margin = parameter(
    node, "traversability.boundary_margin",
    config.generator.traversability.boundary_margin);
  config.generator.traversability.minimum_safe_center_width = parameter(
    node, "traversability.minimum_safe_center_width",
    config.generator.traversability.minimum_safe_center_width);
  config.generator.traversability.maximum_grid_cells = nonnegativeSize(parameter(
      node, "traversability.maximum_grid_cells",
      static_cast<std::int64_t>(config.generator.traversability.maximum_grid_cells)),
    "traversability.maximum_grid_cells");
  config.generator.traversability.maximum_clearance_slope = parameter(
    node, "traversability.maximum_clearance_slope",
    config.generator.traversability.maximum_clearance_slope);
  config.generator.traversability.ground_cell_resolution = parameter(
    node, "traversability.ground_cell_resolution",
    config.generator.traversability.ground_cell_resolution);
  config.generator.traversability.minimum_ground_points_per_cell = nonnegativeSize(parameter(
      node, "traversability.minimum_ground_points_per_cell",
      static_cast<std::int64_t>(
        config.generator.traversability.minimum_ground_points_per_cell)),
    "traversability.minimum_ground_points_per_cell");
  config.generator.traversability.ground_plane_radius = parameter(
    node, "traversability.ground_plane_radius",
    config.generator.traversability.ground_plane_radius);
  config.generator.traversability.minimum_ground_cells_for_plane = nonnegativeSize(parameter(
      node, "traversability.minimum_ground_cells_for_plane",
      static_cast<std::int64_t>(
        config.generator.traversability.minimum_ground_cells_for_plane)),
    "traversability.minimum_ground_cells_for_plane");
  config.generator.traversability.maximum_ground_slope = parameter(
    node, "traversability.maximum_ground_slope",
    config.generator.traversability.maximum_ground_slope);
  config.generator.traversability.maximum_ground_plane_residual = parameter(
    node, "traversability.maximum_ground_plane_residual",
    config.generator.traversability.maximum_ground_plane_residual);
  config.generator.traversability.minimum_obstacle_points_per_cell = nonnegativeSize(parameter(
      node, "traversability.minimum_obstacle_points_per_cell",
      static_cast<std::int64_t>(
        config.generator.traversability.minimum_obstacle_points_per_cell)),
    "traversability.minimum_obstacle_points_per_cell");
  config.generator.traversability.obstacle_support_radius_cells = nonnegativeSize(parameter(
      node, "traversability.obstacle_support_radius_cells",
      static_cast<std::int64_t>(
        config.generator.traversability.obstacle_support_radius_cells)),
    "traversability.obstacle_support_radius_cells");
  config.generator.traversability.minimum_obstacle_neighbor_points = nonnegativeSize(parameter(
      node, "traversability.minimum_obstacle_neighbor_points",
      static_cast<std::int64_t>(
        config.generator.traversability.minimum_obstacle_neighbor_points)),
    "traversability.minimum_obstacle_neighbor_points");
  config.generator.traversability.minimum_obstacle_observations = nonnegativeSize(parameter(
      node, "traversability.minimum_obstacle_observations",
      static_cast<std::int64_t>(
        config.generator.traversability.minimum_obstacle_observations)),
    "traversability.minimum_obstacle_observations");
  config.generator.traversability.unknown_space_policy = parameter(
    node, "traversability.unknown_space_policy",
    config.generator.traversability.unknown_space_policy);

  config.generator.topology.node_merge_distance = parameter(
    node, "topology.node_merge_distance",
    config.generator.topology.node_merge_distance);
  config.generator.topology.intersection_merge_distance = parameter(
    node, "topology.intersection_merge_distance",
    config.generator.topology.intersection_merge_distance);
  config.generator.topology.same_path_heading_threshold_deg = parameter(
    node, "topology.same_path_heading_threshold_deg",
    config.generator.topology.same_path_heading_threshold_deg);
  config.generator.topology.intersection_heading_threshold_deg = parameter(
    node, "topology.intersection_heading_threshold_deg",
    config.generator.topology.intersection_heading_threshold_deg);
  config.generator.topology.minimum_loop_separation = parameter(
    node, "topology.minimum_loop_separation",
    config.generator.topology.minimum_loop_separation);
  config.generator.topology.minimum_edge_length = parameter(
    node, "topology.minimum_edge_length",
    config.generator.topology.minimum_edge_length);
  config.generator.topology.maximum_edge_length = parameter(
    node, "topology.maximum_edge_length",
    config.generator.topology.maximum_edge_length);
  config.generator.topology.geometry_smoothing_window = parameter(
    node, "topology.geometry_smoothing_window",
    config.generator.topology.geometry_smoothing_window);
  config.generator.topology.edge_split_heading_change_deg = parameter(
    node, "topology.edge_split_heading_change_deg",
    config.generator.topology.edge_split_heading_change_deg);
  config.generator.topology.cusp_heading_change_deg = parameter(
    node, "topology.cusp_heading_change_deg",
    config.generator.topology.cusp_heading_change_deg);
  config.generator.topology.generate_reverse_edges = parameter(
    node, "topology.generate_reverse_edges",
    config.generator.topology.generate_reverse_edges);

  config.generator.lanelet2.subtype = parameter(
    node, "lanelet2.subtype", config.generator.lanelet2.subtype);
  config.generator.lanelet2.location = parameter(
    node, "lanelet2.location", config.generator.lanelet2.location);
  config.generator.lanelet2.participant = parameter(
    node, "lanelet2.participant", config.generator.lanelet2.participant);
  config.generator.lanelet2.boundary_type = parameter(
    node, "lanelet2.boundary_type", config.generator.lanelet2.boundary_type);
  config.generator.lanelet2.boundary_subtype = parameter(
    node, "lanelet2.boundary_subtype", config.generator.lanelet2.boundary_subtype);
  config.generator.lanelet2.one_way = parameter(
    node, "lanelet2.one_way", config.generator.lanelet2.one_way);
  config.generator.lanelet2.speed_limit_mps = parameter(
    node, "lanelet2.speed_limit_mps", config.generator.lanelet2.speed_limit_mps);
  config.generator.lanelet2.terminal_localization_settling_verified = parameter(
    node, "lanelet2.terminal_localization_settling_verified",
    config.generator.lanelet2.terminal_localization_settling_verified);

  config.output.directory = parameter(
    node, "output.directory", config.output.directory.string());
  config.output.target_mode = parameter(
    node, "output.target_mode", config.output.target_mode);
  if (config.output.target_mode == "vector_map") {
    config.output.target_mode = "autoware";
  } else if (config.output.target_mode == "navigation_map") {
    config.output.target_mode = "nav2";
  }
  config.output.frame_id = parameter(
    node, "output.frame_id", config.output.frame_id);
  config.output.save_pointcloud_map = parameter(
    node, "output.save_pointcloud_map", config.output.save_pointcloud_map);
  config.output.save_debug_grids = parameter(
    node, "output.save_debug_grids", config.output.save_debug_grids);
  config.output.save_lanelet2 = parameter(
    node, "output.save_lanelet2", config.output.save_lanelet2);
  config.output.nav2_free_space_verified = parameter(
    node, "output.nav2_free_space_verified", config.output.nav2_free_space_verified);
  config.output.lanelet2_physical_boundaries_verified = parameter(
    node, "output.lanelet2_physical_boundaries_verified",
    config.output.lanelet2_physical_boundaries_verified);
  config.output.nav2_route_max_chord_error = parameter(
    node, "output.nav2_route_max_chord_error",
    config.output.nav2_route_max_chord_error);
  config.output.nav2_route_max_segment_length = parameter(
    node, "output.nav2_route_max_segment_length",
    config.output.nav2_route_max_segment_length);

  if (config.input_type != "rosbag2" && config.input_type != "glim") {
    throw std::runtime_error("input.type must be 'rosbag2' or 'glim'");
  }
  if (config.extrinsics.source != "tf_static" && config.extrinsics.source != "parameters") {
    throw std::runtime_error("extrinsics.source must be 'tf_static' or 'parameters'");
  }
  if (!validEvidenceSource(config.extrinsics.calibration_source)) {
    throw std::runtime_error(
            "extrinsics.calibration_source must be unknown, measured, "
            "catalog_estimated, or inferred");
  }
  if (!validEvidenceConfidence(config.extrinsics.calibration_confidence)) {
    throw std::runtime_error(
            "extrinsics.calibration_confidence must be unknown, low, medium, or high");
  }
  if ((config.extrinsics.calibration_source == "unknown") !=
    (config.extrinsics.calibration_confidence == "unknown"))
  {
    throw std::runtime_error(
            "extrinsics calibration source/confidence must either both be unknown or both "
            "describe available evidence");
  }
  if (config.extrinsics.verified && !evidenceSupportsProductionVerification(
      config.extrinsics.calibration_source, config.extrinsics.calibration_confidence))
  {
    throw std::runtime_error(
            "extrinsics.verified=true requires calibration_source=measured and "
            "calibration_confidence=high");
  }
  if (config.input_type == "rosbag2") {
    if (config.rosbag2.bag_path.empty()) {
      throw std::runtime_error("input.rosbag2.path is required");
    }
    if (config.rosbag2.pointcloud_topic.empty()) {
      throw std::runtime_error("input.rosbag2.pointcloud_topic must not be empty");
    }
    if (config.rosbag2.pointcloud_mode != "scan" &&
      config.rosbag2.pointcloud_mode != "accumulated_map")
    {
      throw std::runtime_error(
              "input.rosbag2.pointcloud_mode must be 'scan' or 'accumulated_map'");
    }
    if (config.rosbag2.pose_source != "odometry" &&
      config.rosbag2.pose_source != "pose_stamped" &&
      config.rosbag2.pose_source != "path" && config.rosbag2.pose_source != "tf")
    {
      throw std::runtime_error(
              "input.rosbag2.pose_source must be odometry, pose_stamped, path, or tf");
    }
    if (config.rosbag2.pose_source != "tf" && config.rosbag2.pose_topic.empty()) {
      throw std::runtime_error("input.rosbag2.pose_topic must not be empty");
    }
    if (config.rosbag2.pose_source == "tf" && config.rosbag2.tf_topic.empty()) {
      throw std::runtime_error("input.rosbag2.tf_topic must not be empty");
    }
    if (config.rosbag2.world_frame.empty() || config.rosbag2.base_frame.empty()) {
      throw std::runtime_error("input.rosbag2 world_frame and base_frame must not be empty");
    }
    if (config.rosbag2.pose_reference_frame != "base" &&
      config.rosbag2.pose_reference_frame != "sensor")
    {
      throw std::runtime_error(
              "input.rosbag2.pose_reference_frame must be 'base' or 'sensor'");
    }
    if (config.rosbag2.pointcloud_mode == "scan" && config.rosbag2.sensor_frame.empty()) {
      throw std::runtime_error("input.rosbag2.sensor_frame must not be empty in scan mode");
    }
    requirePositive(
      config.rosbag2.maximum_pose_gap_sec, "input.rosbag2.maximum_pose_gap_sec");
    if (config.rosbag2.deskew.enabled) {
      if (config.rosbag2.deskew.point_time_field.empty()) {
        throw std::runtime_error(
                "input.rosbag2.deskew.point_time_field must not be empty when deskew is enabled");
      }
      if (config.rosbag2.deskew.point_time_reference != "relative" &&
        config.rosbag2.deskew.point_time_reference != "absolute")
      {
        throw std::runtime_error(
                "input.rosbag2.deskew.point_time_reference must be 'relative' or 'absolute'");
      }
      if (!std::isfinite(config.rosbag2.deskew.point_time_scale_sec) ||
        config.rosbag2.deskew.point_time_scale_sec == 0.0 ||
        !std::isfinite(config.rosbag2.deskew.point_time_offset_sec))
      {
        throw std::runtime_error("deskew time scale must be nonzero and time values must be finite");
      }
    }
  } else {
    if (config.glim.map_path.empty() || config.glim.trajectory_path.empty()) {
      throw std::runtime_error("input.glim.map_path and trajectory_path are required");
    }
    if (config.glim.trajectory_frame != "sensor" &&
      config.glim.trajectory_frame != "base")
    {
      throw std::runtime_error("input.glim.trajectory_frame must be 'sensor' or 'base'");
    }
    if (config.glim.world_frame.empty()) {
      throw std::runtime_error("input.glim.world_frame must not be empty");
    }
    if (config.glim.trajectory_frame == "sensor" &&
      config.extrinsics.source != "parameters")
    {
      throw std::runtime_error(
              "GLIM sensor-frame trajectories require extrinsics.source='parameters'");
    }
  }

  if (config.output.directory.empty()) {
    throw std::runtime_error("output.directory must not be empty");
  }

  if (config.generator.robot.dimensions_verified && !config.extrinsics.verified) {
    throw std::runtime_error(
            "robot.dimensions_verified=true requires extrinsics.verified=true; "
            "a footprint cannot be operationally verified against an unverified pose/sensor reference");
  }

  const MapBuilderConfig & map = config.generator.map_builder;
  requirePositive(map.voxel_size, "map_builder.voxel_size");
  requireNonnegative(map.minimum_range, "map_builder.minimum_range");
  requireOrdered(
    map.minimum_range, map.maximum_range,
    "map_builder.minimum_range", "map_builder.maximum_range");
  requireOrdered(map.minimum_z, map.maximum_z, "map_builder.minimum_z", "map_builder.maximum_z");
  if (map.scan_stride == 0U || map.point_stride == 0U) {
    throw std::runtime_error("map_builder scan_stride and point_stride must be at least 1");
  }
  if (map.minimum_observations_per_voxel == 0U) {
    throw std::runtime_error("map_builder.minimum_observations_per_voxel must be at least 1");
  }

  const TrajectoryConfig & trajectory = config.generator.trajectory;
  requireNonnegative(trajectory.minimum_translation, "trajectory.minimum_translation");
  requirePositive(trajectory.maximum_pose_jump, "trajectory.maximum_pose_jump");
  requirePositive(trajectory.maximum_speed_mps, "trajectory.maximum_speed_mps");
  requirePositive(trajectory.resample_interval, "trajectory.resample_interval");
  requireNonnegative(trajectory.smoothing_window, "trajectory.smoothing_window");

  const RobotConfig & robot = config.generator.robot;
  if (robot.profile != "custom" && robot.profile != "small_robot" &&
    robot.profile != "car" && robot.profile != "yaris")
  {
    throw std::runtime_error("robot.profile must be custom, small_robot, car, or yaris");
  }
  if (robot.footprint_model != "circle" && robot.footprint_model != "rectangle") {
    throw std::runtime_error("robot.footprint_model must be circle or rectangle");
  }
  if (robot.base_reference != "unspecified" && robot.base_reference != "body_center" &&
    robot.base_reference != "rear_axle_ground_projection")
  {
    throw std::runtime_error(
            "robot.base_reference must be unspecified, body_center, or "
            "rear_axle_ground_projection");
  }
  if (!validEvidenceSource(robot.dimensions_source)) {
    throw std::runtime_error(
            "robot.dimensions_source must be unknown, measured, catalog_estimated, or inferred");
  }
  if (!validEvidenceConfidence(robot.dimensions_confidence)) {
    throw std::runtime_error(
            "robot.dimensions_confidence must be unknown, low, medium, or high");
  }
  if ((robot.dimensions_source == "unknown") !=
    (robot.dimensions_confidence == "unknown"))
  {
    throw std::runtime_error(
            "robot dimension source/confidence must either both be unknown or both describe "
            "available evidence");
  }
  if (robot.dimensions_verified && !evidenceSupportsProductionVerification(
      robot.dimensions_source, robot.dimensions_confidence))
  {
    throw std::runtime_error(
            "robot.dimensions_verified=true requires dimensions_source=measured and "
            "dimensions_confidence=high");
  }
  requirePositive(robot.width, "robot.width");
  requireNonnegative(robot.clearance_margin, "robot.clearance_margin");
  requirePositive(robot.front_extent, "robot.front_extent");
  requirePositive(robot.rear_extent, "robot.rear_extent");
  requireNonnegative(robot.minimum_collision_height, "robot.minimum_collision_height");
  requireOrdered(
    robot.minimum_collision_height, robot.maximum_collision_height,
    "robot.minimum_collision_height", "robot.maximum_collision_height");
  requireNonnegative(robot.minimum_turning_radius, "robot.minimum_turning_radius");
  if (!robot.allow_in_place_rotation && !(robot.minimum_turning_radius > 0.0)) {
    throw std::runtime_error(
            "robot.minimum_turning_radius must be positive when in-place rotation is disabled");
  }

  const TraversabilityConfig & traversability = config.generator.traversability;
  requirePositive(traversability.grid_resolution, "traversability.grid_resolution");
  requirePositive(
    traversability.trajectory_crop_radius, "traversability.trajectory_crop_radius");
  requirePositive(
    traversability.ground_estimation_radius, "traversability.ground_estimation_radius");
  requireUnitInterval(traversability.ground_quantile, "traversability.ground_quantile");
  if (traversability.minimum_ground_points == 0U) {
    throw std::runtime_error("traversability.minimum_ground_points must be at least 1");
  }
  requireOrdered(
    traversability.ground_search_min_offset, traversability.ground_search_max_offset,
    "traversability.ground_search_min_offset",
    "traversability.ground_search_max_offset");
  requireNonnegative(
    traversability.minimum_obstacle_height, "traversability.minimum_obstacle_height");
  requireOrdered(
    traversability.minimum_obstacle_height, traversability.maximum_obstacle_height,
    "traversability.minimum_obstacle_height",
    "traversability.maximum_obstacle_height");
  requireNonnegative(
    traversability.observed_trajectory_clearance_radius,
    "traversability.observed_trajectory_clearance_radius");
  if (traversability.free_space_evidence_mode != "none" &&
    traversability.free_space_evidence_mode != "trajectory" &&
    traversability.free_space_evidence_mode != "ground_observations" &&
    traversability.free_space_evidence_mode != "combined")
  {
    throw std::runtime_error(
            "traversability.free_space_evidence_mode must be none, trajectory, "
            "ground_observations, or combined");
  }
  if (traversability.minimum_ground_free_points_per_cell == 0U) {
    throw std::runtime_error(
            "traversability.minimum_ground_free_points_per_cell must be at least 1");
  }
  requireNonnegative(
    traversability.maximum_ground_free_height,
    "traversability.maximum_ground_free_height");
  if (traversability.maximum_ground_free_height >
    traversability.minimum_obstacle_height + 1.0e-9)
  {
    throw std::runtime_error(
            "traversability.maximum_ground_free_height must not exceed "
            "minimum_obstacle_height");
  }
  if (traversability.trajectory_free_space_model != "disk" &&
    traversability.trajectory_free_space_model != "footprint")
  {
    throw std::runtime_error(
            "traversability.trajectory_free_space_model must be disk or footprint");
  }
  requireNonnegative(
    traversability.trajectory_footprint_erosion_margin,
    "traversability.trajectory_footprint_erosion_margin");
  const double footprint_inradius = robot.footprint_model == "circle" ?
    0.5 * robot.width :
    std::min({0.5 * robot.width, robot.front_extent, robot.rear_extent});
  if (traversability.trajectory_free_space_model == "footprint" &&
    traversability.trajectory_footprint_erosion_margin >= footprint_inradius)
  {
    throw std::runtime_error(
            "traversability.trajectory_footprint_erosion_margin must leave a positive "
            "orientation-aware footprint inradius");
  }
  if (robot.dimensions_verified &&
    traversability.observed_trajectory_clearance_radius > footprint_inradius + 1.0e-9)
  {
    throw std::runtime_error(
            "traversability.observed_trajectory_clearance_radius extends outside the verified "
            "robot footprint; reduce it below the nearest base_link-relative body boundary");
  }
  requirePositive(
    traversability.maximum_corridor_half_width,
    "traversability.maximum_corridor_half_width");
  requirePositive(traversability.ray_step, "traversability.ray_step");
  requireNonnegative(traversability.boundary_margin, "traversability.boundary_margin");
  requireNonnegative(
    traversability.minimum_safe_center_width,
    "traversability.minimum_safe_center_width");
  if (traversability.maximum_grid_cells == 0U) {
    throw std::runtime_error("traversability.maximum_grid_cells must be at least 1");
  }
  requirePositive(
    traversability.maximum_clearance_slope,
    "traversability.maximum_clearance_slope");
  requirePositive(
    traversability.ground_cell_resolution,
    "traversability.ground_cell_resolution");
  if (traversability.minimum_ground_points_per_cell == 0U) {
    throw std::runtime_error(
            "traversability.minimum_ground_points_per_cell must be at least 1");
  }
  requirePositive(traversability.ground_plane_radius, "traversability.ground_plane_radius");
  if (traversability.minimum_ground_cells_for_plane < 3U) {
    throw std::runtime_error(
            "traversability.minimum_ground_cells_for_plane must be at least 3");
  }
  requireNonnegative(
    traversability.maximum_ground_slope, "traversability.maximum_ground_slope");
  requireNonnegative(
    traversability.maximum_ground_plane_residual,
    "traversability.maximum_ground_plane_residual");
  if (traversability.minimum_obstacle_points_per_cell == 0U ||
    traversability.minimum_obstacle_neighbor_points == 0U ||
    traversability.minimum_obstacle_observations == 0U)
  {
    throw std::runtime_error(
            "traversability obstacle support thresholds must be at least 1");
  }
  if (traversability.unknown_space_policy != "occupied" &&
    traversability.unknown_space_policy != "allow")
  {
    throw std::runtime_error(
            "traversability.unknown_space_policy must be occupied or allow");
  }
  if (std::max(
      traversability.minimum_obstacle_height, robot.minimum_collision_height) >=
    std::min(traversability.maximum_obstacle_height, robot.maximum_collision_height))
  {
    throw std::runtime_error(
            "robot collision height envelope does not overlap traversability obstacle heights");
  }

  const TopologyConfig & topology = config.generator.topology;
  requirePositive(topology.node_merge_distance, "topology.node_merge_distance");
  requirePositive(
    topology.intersection_merge_distance, "topology.intersection_merge_distance");
  if (topology.same_path_heading_threshold_deg < 0.0 ||
    topology.same_path_heading_threshold_deg > 90.0 ||
    topology.intersection_heading_threshold_deg < 0.0 ||
    topology.intersection_heading_threshold_deg > 90.0)
  {
    throw std::runtime_error("topology heading thresholds must be in [0, 90] degrees");
  }
  requireNonnegative(topology.minimum_loop_separation, "topology.minimum_loop_separation");
  requirePositive(topology.minimum_edge_length, "topology.minimum_edge_length");
  requirePositive(topology.maximum_edge_length, "topology.maximum_edge_length");
  requireNonnegative(
    topology.geometry_smoothing_window, "topology.geometry_smoothing_window");
  if (topology.maximum_edge_length < topology.minimum_edge_length) {
    throw std::runtime_error(
            "topology.maximum_edge_length must be at least minimum_edge_length");
  }
  if (!std::isfinite(topology.edge_split_heading_change_deg) ||
    topology.edge_split_heading_change_deg <= 0.0 ||
    topology.edge_split_heading_change_deg >= 180.0 ||
    !std::isfinite(topology.cusp_heading_change_deg) ||
    topology.cusp_heading_change_deg <= 0.0 ||
    topology.cusp_heading_change_deg > 180.0 ||
    topology.cusp_heading_change_deg < topology.edge_split_heading_change_deg)
  {
    throw std::runtime_error(
            "topology turn thresholds must satisfy 0 < edge_split <= cusp <= 180 degrees");
  }

  if (config.generator.lanelet2.subtype.empty() ||
    config.generator.lanelet2.participant.empty() ||
    config.generator.lanelet2.boundary_type.empty())
  {
    throw std::runtime_error(
            "lanelet2 subtype, participant, and boundary_type must not be empty");
  }
  requirePositive(config.generator.lanelet2.speed_limit_mps, "lanelet2.speed_limit_mps");
  if (config.output.frame_id.empty()) {
    throw std::runtime_error("output.frame_id must not be empty");
  }
  if (config.output.target_mode != "autoware" && config.output.target_mode != "nav2" &&
    config.output.target_mode != "both")
  {
    throw std::runtime_error(
            "output.target_mode must be 'vector_map', 'navigation_map', or 'both'");
  }
  if (config.output.autowareEnabled() && !config.output.save_lanelet2) {
    throw std::runtime_error(
            "output.save_lanelet2 must be true when the map type includes Vector Map");
  }
  if (config.output.autowareEnabled() && !config.output.save_pointcloud_map) {
    throw std::runtime_error(
            "output.save_pointcloud_map must be true when the map type includes Vector Map");
  }
  requirePositive(
    config.output.nav2_route_max_chord_error,
    "output.nav2_route_max_chord_error");
  requirePositive(
    config.output.nav2_route_max_segment_length,
    "output.nav2_route_max_segment_length");
  if ((config.output.nav2_free_space_verified ||
    config.output.lanelet2_physical_boundaries_verified) &&
    (!config.extrinsics.verified || !config.generator.robot.dimensions_verified))
  {
    throw std::runtime_error(
            "planner-facing verification claims require both measured vehicle dimensions "
            "and verified LiDAR extrinsics");
  }
  return config;
}

}  // namespace lidar_mobility_map_generator
