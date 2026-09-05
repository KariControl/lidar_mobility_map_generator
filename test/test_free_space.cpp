#include "lidar_mobility_map_generator/occupancy_grid.hpp"
#include "lidar_mobility_map_generator/route_editor.hpp"
#include "lidar_mobility_map_generator/route_graph.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
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

lmmg::TimedPose poseAt(const double x)
{
  lmmg::TimedPose pose;
  pose.stamp_ns = static_cast<std::int64_t>(std::llround(x * 1.0e9));
  pose.world_from_body.translation = {x, 0.0, 1.0};
  return pose;
}

lmmg::TimedPose poseAtYaw(
  const double x, const double y, const double yaw, const std::int64_t stamp_ns)
{
  lmmg::TimedPose pose;
  pose.stamp_ns = stamp_ns;
  pose.world_from_body.translation = {x, y, 1.0};
  pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(yaw);
  return pose;
}

}  // namespace

int main()
{
  try {
    std::vector<lmmg::TimedPose> trajectory;
    for (int index = 0; index <= 40; ++index) {
      trajectory.push_back(poseAt(0.1 * static_cast<double>(index)));
    }

    std::vector<lmmg::PointXYZI> points;
    for (int x = -10; x <= 50; ++x) {
      for (int y = -20; y <= 20; ++y) {
        points.push_back({
          0.1F * static_cast<float>(x) + 0.05F,
          0.1F * static_cast<float>(y) + 0.05F, 0.0F, 1.0F});
      }
    }
    // A supported obstacle outside the swept vehicle must survive, while the
    // same kind of return inside the driven footprint is an ego/self-map
    // contradiction and is cleared by the explicitly selected sweep model.
    points.push_back({2.0F, 1.5F, 0.50F, 1.0F});
    points.push_back({2.01F, 1.5F, 0.51F, 1.0F});
    points.push_back({2.0F, 0.0F, 0.50F, 1.0F});
    points.push_back({2.01F, 0.0F, 0.51F, 1.0F});

    lmmg::TraversabilityConfig traversability;
    traversability.grid_resolution = 0.10;
    traversability.trajectory_crop_radius = 3.0;
    traversability.ground_estimation_radius = 1.0;
    traversability.minimum_ground_points = 3U;
    traversability.minimum_ground_points_per_cell = 3U;
    traversability.minimum_obstacle_height = 0.10;
    traversability.maximum_obstacle_height = 1.50;
    traversability.minimum_obstacle_points_per_cell = 2U;
    traversability.minimum_obstacle_neighbor_points = 2U;
    traversability.free_space_evidence_mode = "combined";
    traversability.minimum_ground_free_points_per_cell = 1U;
    traversability.maximum_ground_free_height = 0.08;
    traversability.trajectory_free_space_model = "footprint";
    traversability.trajectory_footprint_erosion_margin = 0.10;

    lmmg::RobotConfig robot;
    robot.footprint_model = "rectangle";
    robot.width = 1.0;
    robot.front_extent = 1.0;
    robot.rear_extent = 0.50;
    robot.clearance_margin = 0.10;
    robot.minimum_collision_height = 0.10;
    robot.maximum_collision_height = 1.50;

    const lmmg::TraversabilityGridResult grids = lmmg::buildTraversabilityGrid(
      points, trajectory, trajectory, traversability, robot);
    check(grids.ground_observation_free_cells > 1000U,
      "direct ground returns did not create explicit FREE cells");
    check(grids.observed_free_grid.isOccupiedWorld(2.0, 0.0),
      "trajectory footprint was not explicit FREE");
    check(!grids.obstacle_grid.isOccupiedWorld(2.0, 0.0),
      "trajectory footprint did not clear an ego/self-map contradiction");
    check(grids.obstacle_grid.isOccupiedWorld(2.0, 1.5),
      "supported obstacle outside the trajectory footprint was removed");
    check(grids.unknown_grid.isOccupiedWorld(2.0, 2.8),
      "unobserved map exterior was incorrectly promoted to FREE");

    // A trajectory sweep is continuous evidence, not a collection of
    // disconnected pose stamps. Keep the endpoints far enough apart that the
    // midpoint is outside both endpoint rectangles; the <=0.10 m interpolation
    // must still clear the driven midpoint.
    std::vector<lmmg::TimedPose> sparse_observed{poseAt(0.0), poseAt(4.0)};
    const lmmg::TraversabilityGridResult continuous_grids =
      lmmg::buildTraversabilityGrid(
      points, trajectory, sparse_observed, traversability, robot);
    check(continuous_grids.observed_free_grid.isOccupiedWorld(2.0, 0.0),
      "continuous trajectory footprint left a gap between recorded poses");
    check(!continuous_grids.obstacle_grid.isOccupiedWorld(2.0, 0.0),
      "continuous trajectory footprint did not clear the driven midpoint");

    // Translation and rotation can move the same body corner in the same
    // direction. Limiting each component independently to one grid cell is
    // therefore insufficient: their combined motion can leave a sub-cell gap
    // between footprint stamps. This cell is crossed only near the end of the
    // combined motion and must still be recorded as driven FREE space.
    lmmg::TraversabilityConfig combined_sweep_config = traversability;
    combined_sweep_config.free_space_evidence_mode = "trajectory";
    const double combined_yaw = 0.08854829190899167;
    std::vector<lmmg::TimedPose> combined_motion{
      poseAtYaw(0.0, 0.0, 0.0, 0),
      poseAtYaw(
        0.008628418532018156, 0.09862327511108282, combined_yaw,
        1'000'000'000)};
    const lmmg::TraversabilityGridResult combined_sweep_grids =
      lmmg::buildTraversabilityGrid(
      points, combined_motion, combined_motion, combined_sweep_config, robot);
    check(combined_sweep_grids.observed_free_grid.isOccupiedWorld(0.95, 0.55),
      "combined translation/rotation sweep left a sub-cell FREE-space gap");

    lmmg::RouteGraph graph;
    graph.frame_id = "map";
    graph.nodes = {{1U, {0.5, 0.0, 0.0}}, {2U, {3.5, 0.0, 0.0}}};
    lmmg::RouteEdge edge;
    edge.id = 3U;
    edge.from = 1U;
    edge.to = 2U;
    edge.centerline = {{0.5, 0.0, 0.0}, {3.5, 0.0, 0.0}};
    edge.passable = true;
    graph.edges.push_back(edge);

    lmmg::GeneratorConfig generator;
    generator.robot = robot;
    generator.robot.dimensions_verified = false;
    generator.traversability = traversability;
    generator.topology.minimum_edge_length = 0.10;
    generator.topology.maximum_edge_length = 10.0;
    lmmg::RouteEditSession session(graph);
    lmmg::RouteValidationOptions options;
    options.require_verified_vehicle_dimensions = false;
    options.use_orientation_aware_unknown_footprint = true;
    options.include_clearance_in_unknown_footprint = false;
    const lmmg::RouteValidationResult valid = lmmg::validateEditedRouteGraph(
      session.editedGraph(), grids.inflated_grid, grids.unknown_grid, generator, options);
    if (!valid.operational_ready) {
      for (const std::string & error : valid.validation_errors) {
        std::cerr << "unexpected validation error: " << error << '\n';
      }
    }
    check(valid.operational_ready,
      "orientation-aware rectangular UNKNOWN validation rejected a fully observed route");

    lmmg::OccupancyGrid2D unknown_with_hole = grids.unknown_grid;
    const auto hole = unknown_with_hole.worldToCell(2.0, 0.35);
    check(hole.has_value(), "test UNKNOWN hole is outside the grid");
    unknown_with_hole.setOccupied(hole->first, hole->second);
    const lmmg::RouteValidationResult invalid = lmmg::validateEditedRouteGraph(
      session.editedGraph(), grids.inflated_grid, unknown_with_hole, generator, options);
    check(!invalid.operational_ready,
      "orientation-aware rectangular UNKNOWN validation missed a footprint overlap");
    check(std::any_of(
        invalid.validation_errors.begin(), invalid.validation_errors.end(),
        [](const std::string & error) {
          return error.find("vehicle_footprint_overlaps_unknown") != std::string::npos;
        }),
      "UNKNOWN footprint rejection did not report its cause");

    // The same closed-course direct-footprint policy must work for a circular
    // robot. Previously circles silently fell back to a body+clearance UNKNOWN
    // inflation, making a deliberately eroded driven strip impossible to pass.
    lmmg::GeneratorConfig circle_generator = generator;
    circle_generator.robot.footprint_model = "circle";
    circle_generator.robot.width = 0.30;
    circle_generator.robot.front_extent = 0.15;
    circle_generator.robot.rear_extent = 0.15;
    const lmmg::RouteValidationResult valid_circle = lmmg::validateEditedRouteGraph(
      session.editedGraph(), grids.inflated_grid, grids.unknown_grid,
      circle_generator, options);
    check(valid_circle.operational_ready,
      "direct circular UNKNOWN validation rejected a fully observed route");
    check(
      valid_circle.unknown_footprint_policy == "route_sample_circle" &&
      valid_circle.direct_route_footprint_unknown_validation,
      "circular route validation did not report its direct-footprint policy");

    lmmg::OccupancyGrid2D circle_unknown_with_hole = grids.unknown_grid;
    const auto circle_hole = circle_unknown_with_hole.worldToCell(2.0, 0.05);
    check(circle_hole.has_value(), "circle test UNKNOWN hole is outside the grid");
    circle_unknown_with_hole.setOccupied(circle_hole->first, circle_hole->second);
    const lmmg::RouteValidationResult invalid_circle = lmmg::validateEditedRouteGraph(
      session.editedGraph(), grids.inflated_grid, circle_unknown_with_hole,
      circle_generator, options);
    check(!invalid_circle.operational_ready,
      "direct circular UNKNOWN validation missed a footprint overlap");
    check(std::any_of(
        invalid_circle.validation_errors.begin(), invalid_circle.validation_errors.end(),
        [](const std::string & error) {
          return error.find("vehicle_footprint_overlaps_unknown") != std::string::npos;
        }),
      "circular UNKNOWN footprint rejection did not report its cause");

    // A manually added rectangle Edge has a route orientation that is absent
    // from the trajectory-yaw inflated grid. Validate its swept footprint
    // directly against the raw obstacle mask instead of rejecting it solely
    // for missing orientation evidence.
    lmmg::OccupancyGrid2D manual_inflated_grid(0.0, 0.0, 0.10, 100U, 100U);
    lmmg::OccupancyGrid2D manual_unknown_grid(0.0, 0.0, 0.10, 100U, 100U);
    lmmg::RouteGraph manual_base_graph;
    manual_base_graph.frame_id = "map";
    manual_base_graph.nodes = {
      {401U, {2.0, 2.0, 0.0}}, {402U, {5.0, 2.0, 0.0}}};
    lmmg::RouteEdge manual_base_edge;
    manual_base_edge.id = 403U;
    manual_base_edge.from = 401U;
    manual_base_edge.to = 402U;
    manual_base_edge.centerline = {
      manual_base_graph.nodes.front().position,
      manual_base_graph.nodes.back().position};
    manual_base_graph.edges.push_back(manual_base_edge);
    lmmg::RouteEditSession manual_session(manual_base_graph);
    const std::uint64_t manual_node = manual_session.addNode({5.0, 5.0, 0.0});
    const auto manual_edge = manual_session.addEdge(
      402U, manual_node, {{5.0, 2.0, 0.0}, {5.0, 5.0, 0.0}},
      lmmg::RouteDirection::kOneWay);

    lmmg::GeneratorConfig manual_generator;
    manual_generator.robot.footprint_model = "rectangle";
    manual_generator.robot.width = 0.40;
    manual_generator.robot.front_extent = 0.40;
    manual_generator.robot.rear_extent = 0.30;
    manual_generator.robot.clearance_margin = 0.05;
    manual_generator.robot.dimensions_verified = true;
    manual_generator.traversability.grid_resolution = 0.10;
    manual_generator.traversability.maximum_corridor_half_width = 0.50;
    manual_generator.traversability.ray_step = 0.05;
    manual_generator.traversability.minimum_safe_center_width = 0.20;
    manual_generator.topology.minimum_edge_length = 0.10;
    manual_generator.topology.maximum_edge_length = 10.0;
    lmmg::RouteValidationOptions manual_options;
    manual_options.use_orientation_aware_obstacle_footprint = true;

    const lmmg::RouteValidationResult manual_clear = lmmg::validateEditedRouteGraph(
      manual_session.editedGraph(), manual_inflated_grid, manual_unknown_grid,
      manual_generator, manual_options, &manual_inflated_grid);
    const auto & manual_clear_metadata =
      manual_clear.edited.edge_metadata.at(manual_edge.first);
    check(
      manual_clear_metadata.validation_status == lmmg::RouteValidationStatus::kValid &&
      std::find(
        manual_clear_metadata.validation_errors.begin(),
        manual_clear_metadata.validation_errors.end(),
        "route_orientation_collision_unvalidated") ==
      manual_clear_metadata.validation_errors.end(),
      "clear manual rectangle Edge did not pass raw-obstacle footprint validation");
    check(
      manual_clear.direct_route_footprint_obstacle_validation &&
      manual_clear.obstacle_footprint_policy ==
      "route_tangent_rectangle_raw_obstacle_grid",
      "manual rectangle validation did not report its raw-obstacle policy");

    lmmg::OccupancyGrid2D manual_raw_obstacles = manual_inflated_grid;
    // The raw obstacle is off the centerline but inside body+clearance. Keep
    // the inflated-grid argument empty to isolate the direct footprint test.
    const auto manual_obstacle_cell = manual_raw_obstacles.worldToCell(5.21, 3.5);
    check(manual_obstacle_cell.has_value(),
      "manual rectangle obstacle test cell is outside the grid");
    manual_raw_obstacles.setOccupied(
      manual_obstacle_cell->first, manual_obstacle_cell->second);
    const lmmg::RouteValidationResult manual_blocked = lmmg::validateEditedRouteGraph(
      manual_session.editedGraph(), manual_inflated_grid, manual_unknown_grid,
      manual_generator, manual_options, &manual_raw_obstacles);
    const auto & manual_blocked_metadata =
      manual_blocked.edited.edge_metadata.at(manual_edge.first);
    check(
      manual_blocked_metadata.validation_status == lmmg::RouteValidationStatus::kInvalid &&
      std::find(
        manual_blocked_metadata.validation_errors.begin(),
        manual_blocked_metadata.validation_errors.end(),
        "vehicle_footprint_overlaps_obstacle") !=
      manual_blocked_metadata.validation_errors.end() &&
      std::find(
        manual_blocked_metadata.validation_errors.begin(),
        manual_blocked_metadata.validation_errors.end(),
        "route_orientation_collision_unvalidated") ==
      manual_blocked_metadata.validation_errors.end(),
      "manual rectangle raw-obstacle footprint overlap was not rejected");

    // Production uses a configuration-space UNKNOWN mask, while its lateral
    // rays begin away from the center sample. A sub-cell-radius robot leaves a
    // single UNKNOWN center cell uninflated; lateral clearance on the other
    // side must not let that center overlap satisfy the summed width gate.
    lmmg::OccupancyGrid2D center_obstacle_grid(0.0, 0.0, 0.10, 100U, 100U);
    lmmg::OccupancyGrid2D center_unknown_grid(0.0, 0.0, 0.10, 100U, 100U);
    const auto unknown_center = center_unknown_grid.worldToCell(4.05, 5.09);
    check(unknown_center.has_value(), "center UNKNOWN test cell is outside the grid");
    center_unknown_grid.setOccupied(unknown_center->first, unknown_center->second);

    lmmg::RouteGraph center_unknown_graph;
    center_unknown_graph.frame_id = "map";
    center_unknown_graph.nodes = {
      {101U, {2.05, 5.09, 0.0}}, {102U, {7.05, 5.09, 0.0}}};
    lmmg::RouteEdge center_unknown_edge;
    center_unknown_edge.id = 103U;
    center_unknown_edge.from = 101U;
    center_unknown_edge.to = 102U;
    center_unknown_edge.centerline = {
      center_unknown_graph.nodes.front().position,
      center_unknown_graph.nodes.back().position};
    center_unknown_edge.passable = true;
    center_unknown_graph.edges.push_back(center_unknown_edge);

    lmmg::GeneratorConfig center_unknown_generator;
    center_unknown_generator.robot.footprint_model = "circle";
    center_unknown_generator.robot.width = 0.001;
    center_unknown_generator.robot.front_extent = 0.0005;
    center_unknown_generator.robot.rear_extent = 0.0005;
    center_unknown_generator.robot.clearance_margin = 0.0;
    center_unknown_generator.robot.dimensions_verified = true;
    center_unknown_generator.traversability.grid_resolution = 0.10;
    center_unknown_generator.traversability.ray_step = 0.10;
    center_unknown_generator.traversability.maximum_corridor_half_width = 0.50;
    center_unknown_generator.traversability.minimum_safe_center_width = 0.10;
    center_unknown_generator.traversability.unknown_space_policy = "occupied";
    center_unknown_generator.topology.minimum_edge_length = 0.10;
    center_unknown_generator.topology.maximum_edge_length = 10.0;
    lmmg::RouteEditSession center_unknown_session(center_unknown_graph);
    const lmmg::RouteValidationResult center_unknown_result =
      lmmg::validateEditedRouteGraph(
      center_unknown_session.editedGraph(), center_obstacle_grid,
      center_unknown_grid, center_unknown_generator);
    check(!center_unknown_result.operational_ready,
      "production UNKNOWN validation skipped a centerline UNKNOWN cell");
    check(std::any_of(
        center_unknown_result.validation_errors.begin(),
        center_unknown_result.validation_errors.end(),
        [](const std::string & error) {
          return error.find("centerline_unknown") != std::string::npos;
        }),
      "centerline UNKNOWN rejection did not report its cause");

    // Half-cell point sampling is not sufficient for a diagonal that clips a
    // cell corner for only a few millimetres. The segment supercover must
    // reject both configuration-space UNKNOWN and inflated obstacles.
    lmmg::RouteGraph diagonal_corner_graph;
    diagonal_corner_graph.frame_id = "map";
    diagonal_corner_graph.nodes = {
      {201U, {2.0, 3.095, 0.0}}, {202U, {7.0, 8.095, 0.0}}};
    lmmg::RouteEdge diagonal_corner_edge;
    diagonal_corner_edge.id = 203U;
    diagonal_corner_edge.from = 201U;
    diagonal_corner_edge.to = 202U;
    diagonal_corner_edge.centerline = {
      diagonal_corner_graph.nodes.front().position,
      diagonal_corner_graph.nodes.back().position};
    diagonal_corner_edge.passable = true;
    diagonal_corner_graph.edges.push_back(diagonal_corner_edge);
    lmmg::RouteEditSession diagonal_corner_session(diagonal_corner_graph);

    lmmg::OccupancyGrid2D no_unknown_grid(0.0, 0.0, 0.10, 100U, 100U);
    const lmmg::RouteValidationResult diagonal_clear_result =
      lmmg::validateEditedRouteGraph(
      diagonal_corner_session.editedGraph(), center_obstacle_grid,
      no_unknown_grid, center_unknown_generator);
    check(diagonal_clear_result.operational_ready,
      "segment supercover rejected a known-free diagonal control route");

    const lmmg::RouteValidationResult diagonal_unknown_result =
      lmmg::validateEditedRouteGraph(
      diagonal_corner_session.editedGraph(), center_obstacle_grid,
      center_unknown_grid, center_unknown_generator);
    check(!diagonal_unknown_result.operational_ready,
      "segment supercover missed a short diagonal UNKNOWN-cell corner crossing");
    check(std::any_of(
        diagonal_unknown_result.validation_errors.begin(),
        diagonal_unknown_result.validation_errors.end(),
        [](const std::string & error) {
          return error.find("centerline_unknown") != std::string::npos;
        }),
      "diagonal UNKNOWN-cell rejection did not report its cause");

    lmmg::OccupancyGrid2D diagonal_obstacle_grid = center_obstacle_grid;
    const auto diagonal_obstacle_cell = diagonal_obstacle_grid.worldToCell(4.05, 5.05);
    check(diagonal_obstacle_cell.has_value(),
      "diagonal obstacle test cell is outside the grid");
    diagonal_obstacle_grid.setOccupied(
      diagonal_obstacle_cell->first, diagonal_obstacle_cell->second);
    const lmmg::RouteValidationResult diagonal_obstacle_result =
      lmmg::validateEditedRouteGraph(
      diagonal_corner_session.editedGraph(), diagonal_obstacle_grid,
      no_unknown_grid, center_unknown_generator);
    check(!diagonal_obstacle_result.operational_ready,
      "segment supercover missed a short diagonal obstacle-cell corner crossing");
    check(std::any_of(
        diagonal_obstacle_result.validation_errors.begin(),
        diagonal_obstacle_result.validation_errors.end(),
        [](const std::string & error) {
          return error.find("centerline_occupied") != std::string::npos;
        }),
      "diagonal obstacle-cell rejection did not report its cause");

    // Exercise the same corner clip along a lateral clearance ray. The short
    // edge has an up-right normal y=x+1.095; 5 cm point samples straddle the
    // forbidden cell while the continuous ray intersects it briefly.
    lmmg::RouteGraph ray_corner_graph;
    ray_corner_graph.frame_id = "map";
    constexpr double inverse_sqrt_two = 0.7071067811865476;
    ray_corner_graph.nodes = {
      {301U, {3.5, 4.595, 0.0}},
      {302U, {
          3.5 + 0.001 * inverse_sqrt_two,
          4.595 - 0.001 * inverse_sqrt_two,
          0.0}}};
    lmmg::RouteEdge ray_corner_edge;
    ray_corner_edge.id = 303U;
    ray_corner_edge.from = 301U;
    ray_corner_edge.to = 302U;
    ray_corner_edge.centerline = {
      ray_corner_graph.nodes.front().position,
      ray_corner_graph.nodes.back().position};
    ray_corner_graph.edges.push_back(ray_corner_edge);
    lmmg::TraversabilityConfig ray_corner_config;
    ray_corner_config.grid_resolution = 0.10;
    ray_corner_config.ray_step = 0.10;
    ray_corner_config.maximum_corridor_half_width = 1.0;
    ray_corner_config.minimum_safe_center_width = 1.8;
    ray_corner_config.boundary_margin = 0.0;
    ray_corner_config.maximum_clearance_slope = 100.0;
    ray_corner_config.unknown_space_policy = "occupied";

    lmmg::RouteGraph ray_clear_graph = ray_corner_graph;
    lmmg::computeRouteClearance(
      ray_clear_graph, center_obstacle_grid, no_unknown_grid,
      ray_corner_config, 1.0);
    check(ray_clear_graph.edges.front().passable,
      "lateral-ray supercover rejected a known-free control corridor");

    lmmg::RouteGraph ray_obstacle_graph = ray_corner_graph;
    lmmg::computeRouteClearance(
      ray_obstacle_graph, diagonal_obstacle_grid, no_unknown_grid,
      ray_corner_config, 1.0);
    check(!ray_obstacle_graph.edges.front().passable,
      "lateral-ray supercover missed a short obstacle-cell corner crossing");
    check(std::find(
        ray_obstacle_graph.edges.front().validation_errors.begin(),
        ray_obstacle_graph.edges.front().validation_errors.end(),
        "insufficient_clearance") !=
      ray_obstacle_graph.edges.front().validation_errors.end(),
      "lateral obstacle corner crossing did not reduce safe width");

    lmmg::RouteGraph ray_unknown_graph = ray_corner_graph;
    lmmg::computeRouteClearance(
      ray_unknown_graph, center_obstacle_grid, center_unknown_grid,
      ray_corner_config, 1.0);
    check(!ray_unknown_graph.edges.front().passable,
      "lateral-ray supercover missed a short UNKNOWN-cell corner crossing");
    check(std::find(
        ray_unknown_graph.edges.front().validation_errors.begin(),
        ray_unknown_graph.edges.front().validation_errors.end(),
        "unknown_clearance") !=
      ray_unknown_graph.edges.front().validation_errors.end(),
      "lateral UNKNOWN corner crossing did not report incomplete clearance");

    // A scan-integrated map can contain many one-off vertically smeared low
    // voxels below a persistent road return. Observation-count weighting must
    // keep those outliers from pulling the ground quantile down and turning the
    // road itself into a dense obstacle sheet.
    std::vector<lmmg::TimedPose> weighted_trajectory;
    for (int index = 0; index <= 20; ++index) {
      weighted_trajectory.push_back(poseAt(0.1 * static_cast<double>(index)));
    }
    std::vector<lmmg::PointXYZI> weighted_points;
    for (int coarse_x = -2; coarse_x <= 5; ++coarse_x) {
      for (int coarse_y = -3; coarse_y <= 3; ++coarse_y) {
        const float origin_x = 0.5F * static_cast<float>(coarse_x);
        const float origin_y = 0.5F * static_cast<float>(coarse_y);
        weighted_points.push_back({origin_x + 0.02F, origin_y + 0.02F, -0.40F, 1.0F, 1U});
        weighted_points.push_back({origin_x + 0.08F, origin_y + 0.08F, -0.38F, 1.0F, 1U});
        for (int sample = 0; sample < 6; ++sample) {
          weighted_points.push_back({
            origin_x + 0.12F + 0.05F * static_cast<float>(sample),
            origin_y + 0.12F + 0.04F * static_cast<float>(sample),
            0.0F, 1.0F, 20U});
        }
      }
    }
    // A one-scan obstacle candidate shares a 2-D cell with persistent ground.
    // It is not enough to claim an obstacle, but it must keep the cell UNKNOWN
    // instead of allowing the coincident ground return to promote it to FREE.
    weighted_points.push_back({1.12F, 0.12F, 0.50F, 1.0F, 1U});

    lmmg::TraversabilityConfig weighted_config = traversability;
    weighted_config.free_space_evidence_mode = "ground_observations";
    weighted_config.minimum_obstacle_observations = 2U;
    weighted_config.minimum_obstacle_points_per_cell = 1U;
    weighted_config.minimum_obstacle_neighbor_points = 1U;
    lmmg::RobotConfig weighted_robot = robot;
    weighted_robot.footprint_model = "circle";
    weighted_robot.width = 0.30;
    weighted_robot.front_extent = 0.15;
    weighted_robot.rear_extent = 0.15;
    const lmmg::TraversabilityGridResult weighted_grids = lmmg::buildTraversabilityGrid(
      weighted_points, weighted_trajectory, weighted_trajectory,
      weighted_config, weighted_robot);
    check(weighted_grids.obstacle_candidate_points == 1U,
      "persistent ground was misclassified as obstacle candidates");
    check(weighted_grids.obstacle_grid.occupiedCellCount() == 0U,
      "low-support obstacle candidate entered the obstacle grid");
    check(weighted_grids.low_support_obstacle_cells == 1U,
      "low-support obstacle cell was not recorded");
    check(weighted_grids.unknown_grid.isOccupiedWorld(1.12, 0.12),
      "low-support obstacle candidate was incorrectly promoted to FREE");
    check(!weighted_grids.observed_free_grid.isOccupiedWorld(1.12, 0.12),
      "low-support obstacle candidate leaked into direct-ground FREE evidence");

    std::cout << "Free-space evidence tests passed. ground_free_cells="
              << grids.ground_observation_free_cells << '\n';
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
