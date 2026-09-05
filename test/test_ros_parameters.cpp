#include "lidar_mobility_map_generator/ros_parameters.hpp"
#include "lidar_mobility_map_generator/rosbag_reader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

void check(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

rclcpp::NodeOptions baseOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("input.type", "glim");
  options.append_parameter_override("input.glim.map_path", "/tmp/not_read_by_parameter_test.ply");
  options.append_parameter_override(
    "input.glim.trajectory_path", "/tmp/not_read_by_parameter_test.tum");
  options.append_parameter_override("input.glim.trajectory_frame", "sensor");
  options.append_parameter_override("extrinsics.source", "parameters");
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions estimated_options = baseOptions();
    estimated_options.append_parameter_override("robot.profile", "yaris");
    estimated_options.append_parameter_override("extrinsics.calibration_source", "inferred");
    estimated_options.append_parameter_override("extrinsics.calibration_confidence", "medium");
    estimated_options.append_parameter_override(
      "extrinsics.translation", std::vector<double>{0.77, 0.0, 1.694});
    estimated_options.append_parameter_override(
      "extrinsics.quaternion_xyzw",
      std::vector<double>{0.0, 0.0, -0.00523596, 0.99998629});
    estimated_options.append_parameter_override(
      "traversability.free_space_evidence_mode", "combined");
    estimated_options.append_parameter_override(
      "traversability.trajectory_free_space_model", "footprint");
    estimated_options.append_parameter_override(
      "traversability.trajectory_footprint_erosion_margin", 0.10);
    estimated_options.append_parameter_override(
      "lanelet2.terminal_localization_settling_verified", true);
    rclcpp::Node estimated_node("estimated_parameter_test", estimated_options);
    const lmmg::ApplicationConfig estimated = lmmg::loadApplicationConfig(estimated_node);
    check(
      estimated.generator.robot.width == 1.695 &&
      estimated.generator.robot.front_extent == 3.35 &&
      estimated.generator.robot.rear_extent == 0.60 &&
      estimated.generator.robot.clearance_margin == 0.15,
      "Yaris profile dimensions changed unexpectedly");
    check(
      estimated.generator.robot.dimensions_source == "catalog_estimated" &&
      estimated.generator.robot.dimensions_confidence == "medium" &&
      !estimated.generator.robot.dimensions_verified,
      "Yaris evidence provenance was not retained as a non-production estimate");
    check(
      estimated.extrinsics.calibration_source == "inferred" &&
      estimated.extrinsics.calibration_confidence == "medium" &&
      !estimated.extrinsics.verified,
      "inferred extrinsics were incorrectly promoted to production verification");
    const lmmg::Quaternion requested_rotation{0.0, 0.0, -0.00523596, 0.99998629};
    const lmmg::Quaternion expected_effective_rotation = requested_rotation.normalized();
    check(
      estimated.extrinsics.requested_base_from_sensor &&
      estimated.extrinsics.requested_base_from_sensor->translation.x == 0.77 &&
      estimated.extrinsics.requested_base_from_sensor->translation.y == 0.0 &&
      estimated.extrinsics.requested_base_from_sensor->translation.z == 1.694 &&
      estimated.extrinsics.requested_base_from_sensor->rotation.z == requested_rotation.z &&
      estimated.extrinsics.requested_base_from_sensor->rotation.w == requested_rotation.w &&
      estimated.extrinsics.base_from_sensor.rotation.z == expected_effective_rotation.z &&
      estimated.extrinsics.base_from_sensor.rotation.w == expected_effective_rotation.w,
      "raw parameter extrinsics were not retained separately from the effective transform");
    check(
      estimated.generator.traversability.free_space_evidence_mode == "combined" &&
      estimated.generator.traversability.trajectory_free_space_model == "footprint" &&
      estimated.generator.traversability.trajectory_footprint_erosion_margin == 0.10,
      "closed-course free-space parameters were not loaded");
    check(
      estimated.generator.lanelet2.terminal_localization_settling_verified,
      "explicit terminal localization-settling verification was not loaded");
    check(
      estimated.output.target_mode == "autoware" && estimated.output.autowareEnabled() &&
      !estimated.output.nav2Enabled(),
      "default output target mode is not the Vector Map beta mode");

    rclcpp::NodeOptions vector_map_options = baseOptions();
    vector_map_options.append_parameter_override("output.target_mode", "vector_map");
    rclcpp::Node vector_map_node("vector_map_parameter_test", vector_map_options);
    const lmmg::ApplicationConfig vector_map = lmmg::loadApplicationConfig(vector_map_node);
    check(
      vector_map.output.target_mode == "autoware" &&
      vector_map.output.autowareEnabled() && !vector_map.output.nav2Enabled(),
      "Vector Map alias was not normalized to its internal compatibility value");

    rclcpp::NodeOptions navigation_map_options = baseOptions();
    navigation_map_options.append_parameter_override("output.target_mode", "navigation_map");
    navigation_map_options.append_parameter_override("output.save_lanelet2", false);
    rclcpp::Node navigation_map_node(
      "navigation_map_parameter_test", navigation_map_options);
    const lmmg::ApplicationConfig navigation_map =
      lmmg::loadApplicationConfig(navigation_map_node);
    check(
      navigation_map.output.target_mode == "nav2" &&
      navigation_map.output.nav2Enabled() && !navigation_map.output.autowareEnabled(),
      "Navigation Map alias was not normalized to its internal compatibility value");

    rclcpp::NodeOptions nav2_options = baseOptions();
    nav2_options.append_parameter_override("output.target_mode", "nav2");
    nav2_options.append_parameter_override("output.save_lanelet2", false);
    rclcpp::Node nav2_node("nav2_parameter_test", nav2_options);
    const lmmg::ApplicationConfig nav2 = lmmg::loadApplicationConfig(nav2_node);
    check(
      nav2.output.nav2Enabled() && !nav2.output.autowareEnabled(),
      "legacy navigation-map output target mode was not retained");
    check(
      nav2.generator.traversability.free_space_evidence_mode == "trajectory" &&
      nav2.generator.traversability.trajectory_free_space_model == "disk" &&
      nav2.generator.traversability.observed_trajectory_clearance_radius == 0.0 &&
      nav2.generator.traversability.unknown_space_policy == "occupied",
      "generic FREE/UNKNOWN defaults no longer match their fail-closed contract");

    lmmg::RosbagInputConfig direct_tf_frames;
    direct_tf_frames.base_frame = "base_link";
    direct_tf_frames.sensor_frame = "lidar";
    direct_tf_frames.pose_reference_frame = "base";
    check(
      lmmg::acceptsDirectTfPoseEndpoint("/base_link", direct_tf_frames) &&
      !lmmg::acceptsDirectTfPoseEndpoint("lidar", direct_tf_frames),
      "base-reference direct TF endpoint selection changed unexpectedly");
    direct_tf_frames.pose_reference_frame = "sensor";
    check(
      lmmg::acceptsDirectTfPoseEndpoint("lidar", direct_tf_frames) &&
      lmmg::acceptsDirectTfPoseEndpoint("base_link", direct_tf_frames) &&
      !lmmg::acceptsDirectTfPoseEndpoint("camera", direct_tf_frames),
      "sensor-reference direct TF label override changed unexpectedly");

    rclcpp::NodeOptions invalid_target_options = baseOptions();
    invalid_target_options.append_parameter_override("output.target_mode", "automatic");
    rclcpp::Node invalid_target_node("invalid_target_parameter_test", invalid_target_options);
    bool invalid_target_rejected = false;
    try {
      static_cast<void>(lmmg::loadApplicationConfig(invalid_target_node));
    } catch (const std::runtime_error & error) {
      invalid_target_rejected =
        std::string(error.what()).find("output.target_mode") != std::string::npos;
    }
    check(invalid_target_rejected, "invalid output target mode was accepted");

    rclcpp::NodeOptions invalid_options = baseOptions();
    invalid_options.append_parameter_override("extrinsics.calibration_source", "inferred");
    invalid_options.append_parameter_override("extrinsics.calibration_confidence", "medium");
    invalid_options.append_parameter_override("extrinsics.verified", true);
    rclcpp::Node invalid_node("invalid_parameter_test", invalid_options);
    bool invalid_production_claim_rejected = false;
    try {
      static_cast<void>(lmmg::loadApplicationConfig(invalid_node));
    } catch (const std::runtime_error & error) {
      invalid_production_claim_rejected =
        std::string(error.what()).find("requires calibration_source=measured") !=
        std::string::npos;
    }
    check(
      invalid_production_claim_rejected,
      "inferred/medium extrinsics were accepted with verified=true");

    rclcpp::NodeOptions nonfinite_options = baseOptions();
    nonfinite_options.append_parameter_override(
      "extrinsics.quaternion_xyzw",
      std::vector<double>{0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()});
    rclcpp::Node nonfinite_node("nonfinite_parameter_test", nonfinite_options);
    bool nonfinite_extrinsic_rejected = false;
    try {
      static_cast<void>(lmmg::loadApplicationConfig(nonfinite_node));
    } catch (const std::runtime_error & error) {
      nonfinite_extrinsic_rejected =
        std::string(error.what()).find("finite and nonzero") != std::string::npos;
    }
    check(
      nonfinite_extrinsic_rejected,
      "non-finite extrinsic quaternion parameter was accepted");
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
