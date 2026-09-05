#include "lidar_mobility_map_generator/nav2_experimental.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
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

std::string readFile(const std::filesystem::path & path, const bool binary = false)
{
  std::ifstream stream(path, binary ? std::ios::binary : std::ios::in);
  if (!stream) {
    throw std::runtime_error("failed to open test artifact: " + path.string());
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::size_t countOccurrences(const std::string & text, const std::string & needle)
{
  std::size_t count = 0U;
  std::size_t position = 0U;
  while ((position = text.find(needle, position)) != std::string::npos) {
    ++count;
    position += needle.size();
  }
  return count;
}

lmmg::RouteGraph simpleRouteGraph()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {0.15, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {0.25, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  lmmg::RouteEdge edge;
  edge.id = 3U;
  edge.from = 1U;
  edge.to = 2U;
  edge.centerline = {{0.15, 0.05, 0.0}, {0.25, 0.05, 0.0}};
  edge.recommended_speed_mps = 1.0;
  edge.passable = true;
  graph.edges.push_back(edge);
  return graph;
}

lmmg::RouteGraph fullMapWithSelectedMissionGraph()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {30U, {0.11, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {31U, {0.17, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {32U, {0.23, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {33U, {0.29, 0.05, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  for (const auto & ids :
    std::vector<std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>>{
      {40U, 30U, 31U}, {41U, 31U, 32U}, {42U, 32U, 33U}})
  {
    lmmg::RouteEdge edge;
    edge.id = std::get<0>(ids);
    edge.from = std::get<1>(ids);
    edge.to = std::get<2>(ids);
    edge.passable = true;
    edge.recommended_speed_mps = 0.5;
    const auto from = std::find_if(
      graph.nodes.begin(), graph.nodes.end(),
      [&](const lmmg::RouteNode & node) {return node.id == edge.from;});
    const auto to = std::find_if(
      graph.nodes.begin(), graph.nodes.end(),
      [&](const lmmg::RouteNode & node) {return node.id == edge.to;});
    edge.centerline = {from->position, to->position};
    graph.edges.push_back(std::move(edge));
  }
  return graph;
}

lmmg::PipelineResult classifiedPipeline()
{
  lmmg::PipelineResult pipeline;
  pipeline.grids.obstacle_grid = lmmg::OccupancyGrid2D(0.0, 0.0, 0.1, 4U, 1U);
  pipeline.grids.inflated_grid = lmmg::OccupancyGrid2D(0.0, 0.0, 0.1, 4U, 1U);
  pipeline.grids.observed_free_grid = lmmg::OccupancyGrid2D(0.0, 0.0, 0.1, 4U, 1U);
  pipeline.grids.unknown_grid = lmmg::OccupancyGrid2D(0.0, 0.0, 0.1, 4U, 1U);
  pipeline.grids.obstacle_grid.setOccupied(0, 0);
  pipeline.grids.inflated_grid.setOccupied(0, 0);
  pipeline.grids.inflated_grid.setOccupied(1, 0);
  pipeline.grids.observed_free_grid.setOccupied(1, 0);
  pipeline.grids.observed_free_grid.setOccupied(2, 0);
  pipeline.grids.unknown_grid.setOccupied(3, 0);
  pipeline.grids.ground_observation_free_cells = 2U;
  return pipeline;
}

lmmg::ApplicationConfig experimentalConfig()
{
  lmmg::ApplicationConfig config;
  config.output.frame_id = "map";
  config.generator.traversability.free_space_evidence_mode = "ground_observations";
  config.generator.traversability.unknown_space_policy = "occupied";
  config.generator.robot.profile = "small_robot";
  config.generator.robot.footprint_model = "circle";
  config.generator.robot.width = 0.50;
  config.generator.robot.clearance_margin = 0.10;
  config.generator.robot.dimensions_source = "inferred";
  config.generator.robot.dimensions_confidence = "medium";
  config.extrinsics.calibration_source = "inferred";
  config.extrinsics.calibration_confidence = "medium";
  return config;
}

lmmg::Nav2ClosedCourseControls artifactControls()
{
  lmmg::Nav2ClosedCourseControls controls;
  controls.enabled = true;
  return controls;
}

lmmg::RouteGraph branchRouteGraph()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {10U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {11U, {1.0, 0.0, 0.0}, lmmg::RouteNodeType::kJunction},
    {12U, {2.0, 1.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {13U, {2.0, -1.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  for (const auto & ids :
    std::vector<std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>>{
      {20U, 10U, 11U}, {21U, 11U, 12U}, {22U, 11U, 13U}})
  {
    lmmg::RouteEdge edge;
    edge.id = std::get<0>(ids);
    edge.from = std::get<1>(ids);
    edge.to = std::get<2>(ids);
    edge.passable = true;
    const auto from = std::find_if(
      graph.nodes.begin(), graph.nodes.end(),
      [&](const lmmg::RouteNode & node) {return node.id == edge.from;});
    const auto to = std::find_if(
      graph.nodes.begin(), graph.nodes.end(),
      [&](const lmmg::RouteNode & node) {return node.id == edge.to;});
    edge.centerline = {from->position, to->position};
    graph.edges.push_back(edge);
  }
  return graph;
}

lmmg::RouteGraph denseRouteGraph(const std::vector<lmmg::Vec3> & centerline)
{
  check(centerline.size() >= 2U, "dense route fixture requires at least two points");
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {100U, centerline.front(), lmmg::RouteNodeType::kEndpoint},
    {101U, centerline.back(), lmmg::RouteNodeType::kEndpoint}};
  lmmg::RouteEdge edge;
  edge.id = 102U;
  edge.from = 100U;
  edge.to = 101U;
  edge.centerline = centerline;
  edge.passable = true;
  graph.edges.push_back(std::move(edge));
  return graph;
}

double waypointLength(const std::vector<lmmg::Nav2WaypointPose> & waypoints)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < waypoints.size(); ++index) {
    length += lmmg::distance3d(
      waypoints[index - 1U].position, waypoints[index].position);
  }
  return length;
}

}  // namespace

int main()
{
  try {
    const lmmg::PipelineResult pipeline = classifiedPipeline();
    const lmmg::RouteGraph graph = simpleRouteGraph();
    lmmg::ApplicationConfig config = experimentalConfig();
    lmmg::Nav2ClosedCourseControls controls = artifactControls();

    const lmmg::Nav2ClosedCourseAssessment artifact =
      lmmg::evaluateNav2ClosedCourseExperiment(pipeline, graph, config, controls);
    check(
      artifact.closed_course_artifact_ready && artifact.static_map_artifact_ready &&
      artifact.follow_waypoints_artifact_ready && artifact.route_server_artifact_ready,
      "valid estimated geometry did not produce closed-course artifacts");
    check(
      artifact.route_obstacle_samples == 0U && artifact.route_unknown_samples == 0U &&
      artifact.route_off_map_samples == 0U,
      "valid closed-course route did not remain inside explicit FREE cells");
    check(
      !artifact.closed_course_deployment_ready && !artifact.deployment_blockers.empty(),
      "unset session controls were incorrectly treated as deployment-ready");
    check(
      artifact.obstacle_cells == 1U && artifact.explicit_free_cells == 2U &&
      artifact.unknown_cells == 1U && artifact.unclassified_cells == 0U &&
      artifact.overlapping_classification_cells == 0U,
      "trinary classification accounting changed");
    check(
      std::abs(artifact.costmap_inflation_radius - 0.35) <= 1.0e-12,
      "circle inflation radius no longer includes one footprint radius plus clearance");

    const std::filesystem::path output =
      std::filesystem::temp_directory_path() / "lmmg_nav2_experimental_test";
    config.generator.robot.allow_reverse_motion = false;
    const lmmg::Nav2ClosedCourseArtifacts paths{
      output / "map.pgm", output / "map.yaml", output / "graph.geojson",
      output / "waypoints.yaml", output / "params.yaml", output / "readiness.yaml"};
    const lmmg::Nav2ClosedCourseAssessment saved =
      lmmg::saveNav2ClosedCourseExperimentalBundle(paths, pipeline, graph, config, controls);
    check(saved.closed_course_artifact_ready, "valid experimental bundle failed to save");

    const std::string pgm = readFile(paths.map_pgm, true);
    const std::string header = "P5\n4 1\n255\n";
    check(pgm.rfind(header, 0U) == 0U && pgm.size() == header.size() + 4U,
      "experimental PGM header or dimensions changed");
    const std::vector<unsigned int> expected{0U, 254U, 254U, 205U};
    for (std::size_t index = 0U; index < expected.size(); ++index) {
      check(
        static_cast<unsigned char>(pgm[header.size() + index]) == expected[index],
        "experimental PGM is not raw obstacle/FREE/UNKNOWN trinary data");
    }
    check(
      static_cast<unsigned char>(pgm[header.size() + 1U]) == 254U,
      "offline inflated obstacle leaked into the Nav2 map");

    const std::string params = readFile(paths.nav2_params_overlay_yaml);
    check(
      params.find("allow_unknown: false") != std::string::npos &&
      params.find("track_unknown_space: true") != std::string::npos &&
      params.find("footprint_clearing_enabled: false") != std::string::npos &&
      params.find("plugin: \"nav2_costmap_2d::ObstacleLayer\"") != std::string::npos &&
      params.find("topic: \"/points_raw\"") != std::string::npos &&
      params.find("data_type: \"PointCloud2\"") != std::string::npos &&
      params.find("clearing: true") != std::string::npos &&
      params.find("marking: true") != std::string::npos &&
      params.find("offline_obstacle_inflation_applied: false") != std::string::npos &&
      params.find("global_frame: \"map\"") != std::string::npos &&
      params.find("global_frame: \"odom\"") != std::string::npos,
      "Nav2 static obstacle/UNKNOWN/frame safety overlay changed");
    check(
      params.find("cost_scaling_factor: 5.0") != std::string::npos &&
      params.find("max_velocity: [0.25, 0.0, 0.6]") != std::string::npos &&
      params.find("min_velocity: [0.0, 0.0, -0.6]") != std::string::npos &&
      params.find("min_velocity: [0, 0.0,") == std::string::npos,
      "Nav2 velocity arrays lost homogeneous floating-point YAML types");
    const std::string waypoints = readFile(paths.waypoint_routes_yaml);
    check(
      waypoints.find("format: \"lmmg_nav2_follow_waypoints_routes\"") !=
      std::string::npos &&
      waypoints.find("action_type: \"nav2_msgs/action/FollowWaypoints\"") !=
      std::string::npos &&
      waypoints.find("z: 0.0") != std::string::npos &&
      waypoints.find("artifact_ready: true") != std::string::npos &&
      waypoints.find("source_edge_ids: [3]") != std::string::npos,
      "FollowWaypoints artifact omitted its schema/readiness/provenance");
    check(
      waypoints.find("authored_stop_waypoint_count") == std::string::npos &&
      waypoints.find("stop_behavior") == std::string::npos,
      "authoring-free waypoint YAML unexpectedly gained stop metadata");
    const std::string graph_text = readFile(paths.route_graph_geojson);
    check(
      graph_text.find("\"abs_speed_limit\":0.25") != std::string::npos,
      "experimental Route Server graph was not capped to closed-course speed");
    const std::string readiness = readFile(paths.readiness_yaml);
    check(
      readiness.find("artifact:\n  ready: true") != std::string::npos &&
      readiness.find("deployment:\n  ready: false") != std::string::npos &&
      readiness.find("unknown_promoted_to_free: false") != std::string::npos,
      "artifact/deployment or UNKNOWN contract was omitted from readiness report");

    lmmg::NamedNavigationRoute named_route;
    named_route.id = 70U;
    named_route.name = "yard & inspection";
    named_route.target = lmmg::NavigationAuthoringTarget::kNav2;
    named_route.start_node_id = 1U;
    named_route.end_node_id = 2U;
    named_route.ordered_edge_ids = {3U};
    std::vector<lmmg::AuthoredStopLine> stop_lines;
    for (const auto & definition :
      std::vector<std::tuple<std::uint64_t, const char *, double>>{
        {80U, "route start", 0.0},
        {81U, "first & exact", 0.025},
        {82U, "second exact", 0.075}})
    {
      lmmg::AuthoredStopLine stop;
      stop.id = std::get<0>(definition);
      stop.name = std::get<1>(definition);
      stop.edge_id = 3U;
      stop.s = std::get<2>(definition);
      stop.width_m = 0.50;
      stop.anchor = {0.15 + stop.s, 0.05, 0.0};
      stop.target = lmmg::NavigationAuthoringTarget::kNav2;
      stop_lines.push_back(std::move(stop));
    }
    const lmmg::Nav2ClosedCourseArtifacts named_paths{
      output / "named_map.pgm", output / "named_map.yaml", output / "named_graph.geojson",
      output / "named_waypoints.yaml", output / "named_params.yaml",
      output / "named_readiness.yaml"};
    const lmmg::Nav2ClosedCourseAssessment named_saved =
      lmmg::saveNav2ClosedCourseExperimentalBundle(
      named_paths, pipeline, graph, config, controls, &named_route, &stop_lines);
    check(
      named_saved.follow_waypoints_artifact_ready &&
      named_saved.waypoint_routes == 1U && named_saved.waypoints == 4U,
      "named stop-waypoint bundle readiness counts do not match its emitted Route");
    const std::string named_waypoints = readFile(named_paths.waypoint_routes_yaml);
    const std::size_t stop_80 = named_waypoints.find("authored_stop_line_id: 80");
    const std::size_t stop_81 = named_waypoints.find("authored_stop_line_id: 81");
    const std::size_t stop_82 = named_waypoints.find("authored_stop_line_id: 82");
    check(
      named_waypoints.find("named_route_id: 70") != std::string::npos &&
      named_waypoints.find("name: \"yard & inspection\"") != std::string::npos &&
      named_waypoints.find("authored_stop_waypoint_count: 3") != std::string::npos &&
      named_waypoints.find("stop_behavior: \"waypoint_arrival_only\"") !=
      std::string::npos &&
      named_waypoints.find("authored_stop_line_name: \"first & exact\"") !=
      std::string::npos &&
      named_waypoints.find("authored_stop_edge_id: 3") != std::string::npos &&
      named_waypoints.find("authored_stop_edge_s_m: 0.025") != std::string::npos &&
      named_waypoints.find("authored_stop_width_m: 0.5") != std::string::npos,
      "named stop-waypoint YAML omitted route, behavior, or stop provenance");
    check(
      stop_80 != std::string::npos && stop_80 < stop_81 && stop_81 < stop_82,
      "authored stop waypoints were not emitted in directed Route order");
    check(
      countOccurrences(named_waypoints, "authored_stop_line_id:") == 3U &&
      countOccurrences(named_waypoints, "      - {x:") == 4U,
      "stop insertion count changed or duplicated an existing endpoint waypoint");

    // Stop insertion must split the original polyline at the authored arc
    // position without simplifying away any of its curved/3-D source
    // vertices.  This covers the dedicated named-Mission rebuild path, not
    // only the authoring-free waypoint builder below.
    lmmg::RouteGraph curved_stop_graph = simpleRouteGraph();
    const std::vector<lmmg::Vec3> curved_stop_centerline{
      {0.15, 0.05, 0.00},
      {0.18, 0.08, 0.01},
      {0.22, 0.02, 0.02},
      {0.25, 0.05, 0.00}};
    curved_stop_graph.edges.front().centerline = curved_stop_centerline;
    lmmg::NamedNavigationRoute curved_stop_route = named_route;
    curved_stop_route.id = 71U;
    curved_stop_route.name = "lossless curved stop";
    lmmg::AuthoredStopLine curved_stop;
    curved_stop.id = 84U;
    curved_stop.name = "interior curve stop";
    curved_stop.edge_id = 3U;
    curved_stop.s = 0.02;
    curved_stop.width_m = 0.50;
    curved_stop.anchor = {0.164142135624, 0.064142135624, 0.004714045208};
    curved_stop.target = lmmg::NavigationAuthoringTarget::kNav2;
    const std::vector<lmmg::AuthoredStopLine> curved_stops{curved_stop};
    const lmmg::Nav2ClosedCourseArtifacts curved_stop_paths{
      output / "curved_stop_map.pgm", output / "curved_stop_map.yaml",
      output / "curved_stop_graph.geojson", output / "curved_stop_waypoints.yaml",
      output / "curved_stop_params.yaml", output / "curved_stop_readiness.yaml"};
    const lmmg::Nav2ClosedCourseAssessment curved_stop_saved =
      lmmg::saveNav2ClosedCourseExperimentalBundle(
      curved_stop_paths, pipeline, curved_stop_graph, config, controls,
      &curved_stop_route, &curved_stops);
    check(
      curved_stop_saved.follow_waypoints_artifact_ready &&
      curved_stop_saved.waypoint_routes == 1U && curved_stop_saved.waypoints == 5U,
      "curved named stop Mission did not retain four source vertices plus its stop");
    const std::string curved_stop_waypoints = readFile(curved_stop_paths.waypoint_routes_yaml);
    check(
      curved_stop_waypoints.find(
        "- {x: 0.18, y: 0.08, z: 0.0, source_map_z: 0.01") !=
      std::string::npos &&
      curved_stop_waypoints.find(
        "- {x: 0.22, y: 0.02, z: 0.0, source_map_z: 0.02") !=
      std::string::npos &&
      curved_stop_waypoints.find("authored_stop_line_id: 84") != std::string::npos &&
      countOccurrences(curved_stop_waypoints, "      - {x:") == 5U,
      "named stop insertion simplified, reordered, or changed a curved source vertex");

    // A named FollowWaypoints Mission is a subset of the complete Route
    // Server map. The third map Edge must remain in GeoJSON while it must not
    // leak into the selected waypoint chain.
    const lmmg::RouteGraph full_nav2_map = fullMapWithSelectedMissionGraph();
    lmmg::NamedNavigationRoute subset_mission;
    subset_mission.id = 90U;
    subset_mission.name = "two of three edges";
    subset_mission.target = lmmg::NavigationAuthoringTarget::kNav2;
    subset_mission.start_node_id = 30U;
    subset_mission.end_node_id = 32U;
    subset_mission.ordered_edge_ids = {40U, 41U};
    const lmmg::Nav2ClosedCourseArtifacts subset_paths{
      output / "subset_map.pgm", output / "subset_map.yaml",
      output / "subset_full_graph.geojson", output / "subset_mission_waypoints.yaml",
      output / "subset_params.yaml", output / "subset_readiness.yaml"};
    const lmmg::Nav2ClosedCourseAssessment subset_saved =
      lmmg::saveNav2ClosedCourseExperimentalBundle(
      subset_paths, pipeline, full_nav2_map, config, controls, &subset_mission, nullptr);
    const std::string subset_full_graph = readFile(subset_paths.route_graph_geojson);
    const std::string subset_waypoints = readFile(subset_paths.waypoint_routes_yaml);
    check(
      subset_saved.passable_route_edges == 3U &&
      countOccurrences(subset_full_graph, "\"source_route_edge_id\"") == 3U &&
      subset_full_graph.find("\"source_route_edge_id\":42") != std::string::npos,
      "named Mission shortened the full closed-course Nav2 Route Server graph");
    check(
      countOccurrences(subset_full_graph, "\"named_route_id\":90") == 2U,
      "named Mission metadata was not limited to its two Nav2 map Edges");
    check(
      subset_saved.waypoint_routes == 1U &&
      subset_waypoints.find("source_edge_ids: [40, 41]") != std::string::npos &&
      subset_waypoints.find("source_edge_ids: [40, 41, 42]") == std::string::npos,
      "FollowWaypoints did not remain the exact selected Mission subset");

    const lmmg::Nav2ClosedCourseArtifacts invalid_stop_paths{
      output / "invalid_stop_map.pgm", output / "invalid_stop_map.yaml",
      output / "invalid_stop_graph.geojson", output / "invalid_stop_waypoints.yaml",
      output / "invalid_stop_params.yaml", output / "invalid_stop_readiness.yaml"};
    const auto invalidStopsThrow = [&](const std::vector<lmmg::AuthoredStopLine> & invalid) {
        try {
          static_cast<void>(lmmg::saveNav2ClosedCourseExperimentalBundle(
            invalid_stop_paths, pipeline, graph, config, controls, &named_route, &invalid));
        } catch (const std::exception &) {
          return true;
        }
        return false;
      };
    std::vector<lmmg::AuthoredStopLine> missing_edge_stops = stop_lines;
    missing_edge_stops.front().edge_id = 999U;
    check(invalidStopsThrow(missing_edge_stops),
      "a resolved stop on an absent Edge did not fail closed");
    std::vector<lmmg::AuthoredStopLine> nonfinite_stops = stop_lines;
    nonfinite_stops.front().s = std::numeric_limits<double>::quiet_NaN();
    check(invalidStopsThrow(nonfinite_stops),
      "a non-finite resolved stop did not fail closed");
    std::vector<lmmg::AuthoredStopLine> duplicate_position_stops = stop_lines;
    lmmg::AuthoredStopLine duplicate_position = duplicate_position_stops.back();
    duplicate_position.id = 83U;
    duplicate_position.name = "duplicate position";
    duplicate_position_stops.push_back(std::move(duplicate_position));
    check(invalidStopsThrow(duplicate_position_stops),
      "two stops at one Route position did not fail closed");

    lmmg::Nav2ClosedCourseControls deployment_controls = controls;
    deployment_controls.operator_acknowledged_experimental_only = true;
    deployment_controls.estimated_geometry_acknowledged = true;
    deployment_controls.closed_course_access_controlled = true;
    deployment_controls.free_space_reviewed_for_session = true;
    deployment_controls.localization_alignment_checked_for_session = true;
    deployment_controls.emergency_stop_available = true;
    const lmmg::Nav2ClosedCourseAssessment deployment =
      lmmg::evaluateNav2ClosedCourseExperiment(
      pipeline, graph, config, deployment_controls);
    check(
      deployment.closed_course_deployment_ready,
      "complete closed-course session controls did not satisfy deployment gate");

    config.generator.traversability.free_space_evidence_mode = "trajectory";
    const lmmg::Nav2ClosedCourseAssessment trajectory_only =
      lmmg::evaluateNav2ClosedCourseExperiment(pipeline, graph, config, controls);
    check(
      !trajectory_only.static_map_artifact_ready &&
      std::find(
        trajectory_only.map_blockers.begin(), trajectory_only.map_blockers.end(),
        "trajectory_only_free_space_evidence_not_promotable") !=
      trajectory_only.map_blockers.end(),
      "trajectory-only FREE was promoted to a closed-course artifact");

    config = experimentalConfig();
    config.generator.robot.footprint_model = "rectangle";
    config.generator.robot.width = 1.80;
    config.generator.robot.front_extent = 3.0;
    config.generator.robot.rear_extent = 1.0;
    config.generator.robot.clearance_margin = 0.15;
    const lmmg::Nav2ClosedCourseAssessment rectangle =
      lmmg::evaluateNav2ClosedCourseExperiment(pipeline, graph, config, controls);
    check(
      std::abs(rectangle.costmap_inflation_radius - 1.05) <= 1.0e-12 &&
      rectangle.costmap_inflation_radius < config.generator.robot.front_extent,
      "rectangle polygon footprint was double-counted in inflation radius");

    config = experimentalConfig();
    config.extrinsics.calibration_confidence = "low";
    check(
      !lmmg::evaluateNav2ClosedCourseExperiment(pipeline, graph, config, controls).
      static_map_artifact_ready,
      "low-confidence estimated extrinsics passed the artifact gate");

    config = experimentalConfig();
    lmmg::PipelineResult route_unknown = pipeline;
    route_unknown.grids.observed_free_grid.setOccupied(2, 0, false);
    route_unknown.grids.unknown_grid.setOccupied(2, 0);
    const lmmg::Nav2ClosedCourseAssessment route_unknown_assessment =
      lmmg::evaluateNav2ClosedCourseExperiment(route_unknown, graph, config, controls);
    check(
      !route_unknown_assessment.follow_waypoints_artifact_ready &&
      !route_unknown_assessment.route_server_artifact_ready &&
      route_unknown_assessment.route_unknown_samples != 0U &&
      std::find(
        route_unknown_assessment.follow_waypoints_blockers.begin(),
        route_unknown_assessment.follow_waypoints_blockers.end(),
        "route_intersects_unknown_space") !=
      route_unknown_assessment.follow_waypoints_blockers.end(),
      "UNKNOWN route crossing was not rejected by both Nav2 route consumers");

    config = experimentalConfig();
    lmmg::PipelineResult overlapping = pipeline;
    overlapping.grids.unknown_grid.setOccupied(1, 0);
    check(
      !lmmg::evaluateNav2ClosedCourseExperiment(overlapping, graph, config, controls).
      static_map_artifact_ready,
      "overlapping occupancy classifications passed the artifact gate");
    lmmg::PipelineResult unclassified = pipeline;
    unclassified.grids.unknown_grid.setOccupied(3, 0, false);
    check(
      !lmmg::evaluateNav2ClosedCourseExperiment(unclassified, graph, config, controls).
      static_map_artifact_ready,
      "unclassified occupancy cell passed the artifact gate");

    const lmmg::Nav2ClosedCourseArtifacts fail_paths{
      output / "fail_map.pgm", output / "fail_map.yaml", output / "fail_graph.geojson",
      output / "fail_waypoints.yaml", output / "fail_params.yaml",
      output / "fail_readiness.yaml"};
    const lmmg::Nav2ClosedCourseAssessment failed_artifact =
      lmmg::saveNav2ClosedCourseExperimentalBundle(
      fail_paths, unclassified, graph, config, controls);
    check(!failed_artifact.static_map_artifact_ready, "invalid map artifact unexpectedly passed");
    const std::string fail_pgm = readFile(fail_paths.map_pgm, true);
    check(
      fail_pgm.size() == header.size() + 4U &&
      std::all_of(
        fail_pgm.begin() + static_cast<std::ptrdiff_t>(header.size()), fail_pgm.end(),
        [](const char value) {return static_cast<unsigned char>(value) == 205U;}),
      "failed artifact did not emit an all-UNKNOWN PGM");

    const std::vector<lmmg::Nav2WaypointRoute> branch_routes =
      lmmg::buildNav2WaypointRoutes(branchRouteGraph(), 0.5);
    check(
      branch_routes.size() == 3U &&
      std::all_of(
        branch_routes.begin(), branch_routes.end(),
        [](const lmmg::Nav2WaypointRoute & route) {
          return route.source_edge_ids.size() == 1U && route.waypoints.size() >= 2U;
        }),
      "branch graph was not split into continuous waypoint routes");

    std::vector<lmmg::Vec3> dense_straight;
    for (std::size_t index = 0U; index <= 1000U; ++index) {
      dense_straight.push_back({0.01 * static_cast<double>(index), 0.0, 0.0});
    }
    const std::vector<lmmg::Nav2WaypointRoute> lossless_straight =
      lmmg::buildNav2WaypointRoutes(denseRouteGraph(dense_straight), 0.50, 0.01);
    check(
      lossless_straight.size() == 1U &&
      lossless_straight.front().waypoints.size() == dense_straight.size(),
      "dense straight route did not retain every source vertex");
    for (std::size_t index = 0U; index < dense_straight.size(); ++index) {
      check(
        lmmg::distance3d(
          lossless_straight.front().waypoints[index].position,
          dense_straight[index]) <= 1.0e-12,
        "dense straight route changed a source vertex");
    }
    for (std::size_t index = 1U;
      index < lossless_straight.front().waypoints.size(); ++index)
    {
      check(
        lmmg::distance3d(
          lossless_straight.front().waypoints[index - 1U].position,
          lossless_straight.front().waypoints[index].position) <= 0.50 + 1.0e-12,
        "lossless route violates maximum waypoint spacing");
    }
    check(
      std::abs(
        waypointLength(lossless_straight.front().waypoints) -
        lmmg::polylineLength(dense_straight)) <= 1.0e-12,
      "dense straight route changed its 3-D arc length");

    std::vector<lmmg::Vec3> dense_curve;
    for (std::size_t index = 0U; index <= 400U; ++index) {
      const double x = 4.0 * static_cast<double>(index) / 400.0;
      dense_curve.push_back({
          x, 0.25 * std::sin(0.5 * lmmg::kPi * x),
          0.05 * std::sin(0.25 * lmmg::kPi * x)});
    }
    constexpr double chord_error = 0.02;
    const std::vector<lmmg::Nav2WaypointRoute> lossless_curve =
      lmmg::buildNav2WaypointRoutes(denseRouteGraph(dense_curve), 0.40, chord_error);
    check(lossless_curve.size() == 1U, "dense curved route did not produce one route");
    check(
      lossless_curve.front().waypoints.size() == dense_curve.size(),
      "dense curved route did not retain every source vertex");
    for (std::size_t index = 0U; index < dense_curve.size(); ++index) {
      check(
        lmmg::distance3d(
          lossless_curve.front().waypoints[index].position,
          dense_curve[index]) <= 1.0e-12,
        "dense curved route changed a source vertex");
    }
    check(
      std::abs(
        waypointLength(lossless_curve.front().waypoints) -
        lmmg::polylineLength(dense_curve)) <= 1.0e-12,
      "dense curved route changed its 3-D arc length");

    const std::vector<lmmg::Vec3> sparse_curve{
      {0.0, 0.0, 0.0}, {0.31, 0.23, 0.11}, {0.76, -0.18, 0.34}, {1.2, 0.0, 0.50}};
    const std::vector<lmmg::Nav2WaypointRoute> densified_curve =
      lmmg::buildNav2WaypointRoutes(denseRouteGraph(sparse_curve), 0.20, chord_error);
    check(densified_curve.size() == 1U, "sparse curve did not produce one route");
    std::size_t retained_source_vertices = 0U;
    for (const lmmg::Nav2WaypointPose & waypoint : densified_curve.front().waypoints) {
      if (retained_source_vertices < sparse_curve.size() &&
        lmmg::distance3d(
          waypoint.position, sparse_curve[retained_source_vertices]) <= 1.0e-12)
      {
        ++retained_source_vertices;
      }
    }
    check(
      retained_source_vertices == sparse_curve.size(),
      "linear densification omitted or reordered a source vertex");
    for (std::size_t index = 1U;
      index < densified_curve.front().waypoints.size(); ++index)
    {
      check(
        lmmg::distance3d(
          densified_curve.front().waypoints[index - 1U].position,
          densified_curve.front().waypoints[index].position) <= 0.20 + 1.0e-12,
        "linear densification violates maximum waypoint spacing");
    }
    check(
      std::abs(
        waypointLength(densified_curve.front().waypoints) -
        lmmg::polylineLength(sparse_curve)) <= 1.0e-12,
      "linear densification changed the sparse curve's 3-D arc length");
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
