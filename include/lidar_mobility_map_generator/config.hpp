#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace lidar_mobility_map_generator
{

struct DeskewConfig
{
  bool enabled{false};
  std::string point_time_field{"time"};
  std::string point_time_reference{"relative"};  // relative or absolute
  double point_time_scale_sec{1.0};
  double point_time_offset_sec{0.0};
};

struct ExtrinsicsConfig
{
  std::string source{"tf_static"};  // tf_static or parameters
  Transform base_from_sensor{};     // T_base_sensor
  // Exact parameter tokens before quaternion normalization.  The generation
  // contract restores these values so every run performs the same single
  // normalization from the same requested input.
  std::optional<Transform> requested_base_from_sensor;
  // Physical-calibration provenance.  This is independent of `source`, which
  // only says how the transform was loaded (TF or parameters).
  std::string calibration_source{"unknown"};
  std::string calibration_confidence{"unknown"};
  // Explicit provenance gate.  A transform being present (or identity) does
  // not prove that it represents the physical sensor installation.
  bool verified{false};
};

struct RosbagInputConfig
{
  std::filesystem::path bag_path;
  std::string storage_id{};
  std::string pointcloud_topic{"/points_raw"};
  std::string pointcloud_mode{"scan"};  // scan or accumulated_map
  std::string pose_source{"odometry"};  // odometry, tf, pose_stamped, path
  std::string pose_topic{"/localization/kinematic_state"};
  std::string tf_topic{"/tf"};
  std::string tf_static_topic{"/tf_static"};
  std::string world_frame{"map"};
  std::string base_frame{"base_link"};
  std::string sensor_frame{"lidar"};
  // Physical reference point represented by the numeric localization poses.
  // "sensor" is useful for bags whose TF child is named base_link even though
  // the estimator actually reports the LiDAR-origin pose.
  std::string pose_reference_frame{"base"};  // base or sensor
  bool strict_frame_check{true};
  bool use_header_stamp{true};
  double maximum_pose_gap_sec{0.10};
  DeskewConfig deskew;
};

struct GlimInputConfig
{
  std::filesystem::path map_path;
  std::filesystem::path trajectory_path;
  std::string trajectory_frame{"sensor"};  // sensor or base
  std::string world_frame{"map"};
};

struct OutputConfig
{
  std::filesystem::path directory{"output"};
  // Selects which planner-facing products may be promoted.  The expensive
  // point-cloud/trajectory pipeline remains shared; the non-selected target
  // is still overwritten with fail-closed placeholders so stale products
  // cannot look current after changing modes.
  // Internal compatibility value. Public vector_map/navigation_map aliases are
  // normalized to autoware/nav2 while loading ROS parameters.
  // The first beta release is Vector Map first. Navigation Map remains an
  // explicit alpha selection instead of being generated implicitly.
  std::string target_mode{"autoware"};  // autoware, nav2, or both (internal)
  // Offline inputs may use an estimator-specific world-frame name.  Exported
  // map products use one explicit navigation frame so RViz, Nav2, and
  // Autoware do not silently disagree about the frame label.  This relabels
  // the local Cartesian datum; it does not create a geodetic transform.
  std::string frame_id{"map"};
  bool save_pointcloud_map{true};
  bool save_debug_grids{true};
  bool save_lanelet2{true};
  // Human/independent verification gates.  A parseable candidate is always
  // emitted, while canonical planner inputs fail closed until these claims
  // are explicitly made for the actual data set.
  bool nav2_free_space_verified{false};
  bool lanelet2_physical_boundaries_verified{false};
  double nav2_route_max_chord_error{0.10};
  double nav2_route_max_segment_length{2.0};

  [[nodiscard]] bool autowareEnabled() const noexcept
  {
    return target_mode == "autoware" || target_mode == "both";
  }

  [[nodiscard]] bool nav2Enabled() const noexcept
  {
    return target_mode == "nav2" || target_mode == "both";
  }
};

struct ApplicationConfig
{
  std::string input_type{"rosbag2"};
  RosbagInputConfig rosbag2;
  GlimInputConfig glim;
  ExtrinsicsConfig extrinsics;
  GeneratorConfig generator;
  OutputConfig output;
};

}  // namespace lidar_mobility_map_generator
