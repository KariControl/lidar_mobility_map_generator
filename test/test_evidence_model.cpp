#include "lidar_mobility_map_generator/navigation_outputs.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

void check(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lmmg::RouteValidationResult simpleRouteValidation()
{
  lmmg::RouteValidationResult result;
  result.operational_ready = true;
  result.operational_graph.frame_id = "map";
  result.operational_graph.nodes = {
    {1U, {0.05, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {0.15, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  lmmg::RouteEdge edge;
  edge.id = 3U;
  edge.from = 1U;
  edge.to = 2U;
  edge.centerline = {{0.05, 0.05, 0.0}, {0.15, 0.05, 0.0}};
  edge.left_boundary = {{0.05, 0.10, 0.0}, {0.15, 0.10, 0.0}};
  edge.right_boundary = {{0.05, 0.0, 0.0}, {0.15, 0.0, 0.0}};
  edge.corridor_geometry_valid = true;
  edge.passable = true;
  result.operational_graph.edges.push_back(edge);
  return result;
}

}  // namespace

int main()
{
  try {
    check(
      lmmg::evidenceSupportsClosedCourseExperiment("catalog_estimated", "medium"),
      "catalogue/medium dimensions did not meet the closed-course threshold");
    check(
      lmmg::evidenceSupportsClosedCourseExperiment("inferred", "medium"),
      "inferred/medium extrinsics did not meet the closed-course threshold");
    check(
      !lmmg::evidenceSupportsClosedCourseExperiment("inferred", "low"),
      "low-confidence inference was accepted for closed-course readiness");
    check(
      lmmg::evidenceSupportsProductionVerification("measured", "high") &&
      !lmmg::evidenceSupportsProductionVerification("catalog_estimated", "high"),
      "production evidence threshold did not require a high-confidence measurement");

    lmmg::MappingDataset dataset;
    dataset.world_frame = "map";
    lmmg::PipelineResult pipeline;
    pipeline.grids.obstacle_grid = lmmg::OccupancyGrid2D(0.0, 0.0, 0.1, 2U, 2U);
    pipeline.grids.observed_free_grid = lmmg::OccupancyGrid2D(0.0, 0.0, 0.1, 2U, 2U);
    pipeline.grids.unknown_grid = lmmg::OccupancyGrid2D(0.0, 0.0, 0.1, 2U, 2U);
    pipeline.grids.observed_free_grid.setOccupied(0, 0);
    pipeline.grids.observed_free_grid.setOccupied(1, 0);
    pipeline.grids.unknown_grid.setOccupied(0, 1);
    pipeline.grids.unknown_grid.setOccupied(1, 1);

    const lmmg::RouteValidationResult production_route = simpleRouteValidation();
    const lmmg::RouteValidationResult experimental_route = simpleRouteValidation();
    lmmg::ApplicationConfig config;
    config.output.frame_id = "map";
    // Evidence behavior is checked for both products in this test.  Do not
    // depend on the Vector Map-first public default.
    config.output.target_mode = "both";
    config.output.nav2_free_space_verified = true;
    config.output.lanelet2_physical_boundaries_verified = true;
    config.generator.robot.base_reference = "rear_axle_ground_projection";
    config.generator.robot.dimensions_source = "catalog_estimated";
    config.generator.robot.dimensions_confidence = "medium";
    config.generator.traversability.free_space_evidence_mode = "combined";
    config.generator.lanelet2.location = "urban";
    config.generator.lanelet2.one_way = true;
    config.extrinsics.calibration_source = "inferred";
    config.extrinsics.calibration_confidence = "medium";

    const lmmg::NavigationTargetReadiness estimated =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, pipeline, production_route, config, &experimental_route);
    check(
      !estimated.nav2_navigation_ready && !estimated.autoware_navigation_ready,
      "estimated calibration was incorrectly promoted to production readiness");
    check(
      estimated.nav2_closed_course_experimental_ready &&
      estimated.autoware_closed_course_experimental_ready,
      "otherwise-valid estimated calibration did not qualify for closed-course readiness");

    config.extrinsics.calibration_confidence = "low";
    const lmmg::NavigationTargetReadiness low_confidence =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, pipeline, production_route, config, &experimental_route);
    check(
      !low_confidence.nav2_closed_course_experimental_ready &&
      !low_confidence.autoware_closed_course_experimental_ready &&
      std::find(
        low_confidence.nav2_experimental_reasons.begin(),
        low_confidence.nav2_experimental_reasons.end(),
        "lidar_extrinsics_evidence_below_closed_course_threshold") !=
      low_confidence.nav2_experimental_reasons.end(),
      "low-confidence extrinsics did not fail closed-course readiness with a reason");

    config.extrinsics.calibration_source = "measured";
    config.extrinsics.calibration_confidence = "high";
    config.extrinsics.verified = true;
    config.generator.robot.dimensions_source = "measured";
    config.generator.robot.dimensions_confidence = "high";
    config.generator.robot.dimensions_verified = true;
    const lmmg::NavigationTargetReadiness measured =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, pipeline, production_route, config, &experimental_route);
    check(
      measured.nav2_navigation_ready && measured.autoware_navigation_ready &&
      measured.nav2_closed_course_experimental_ready &&
      measured.autoware_closed_course_experimental_ready,
      "measured/high evidence did not satisfy both readiness levels");

    config.output.target_mode = "nav2";
    const lmmg::NavigationTargetReadiness nav2_only =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, pipeline, production_route, config, &experimental_route);
    check(
      nav2_only.nav2_enabled && !nav2_only.autoware_enabled &&
      nav2_only.nav2_navigation_ready && !nav2_only.autoware_navigation_ready &&
      !nav2_only.autoware_closed_course_experimental_ready &&
      std::find(
        nav2_only.autoware_reasons.begin(), nav2_only.autoware_reasons.end(),
        "target_not_selected") != nav2_only.autoware_reasons.end(),
      "Nav2-only mode did not fail-close the non-selected Autoware target");

  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
