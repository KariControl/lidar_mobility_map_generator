#include "lidar_mobility_map_generator/exporters.hpp"
#include "lidar_mobility_map_generator/review_io.hpp"

#include <filesystem>
#include <iostream>
#include <map>
#include <string>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

lmmg::RouteEdge edge(
  const std::uint64_t id, const std::uint64_t from, const std::uint64_t to,
  std::vector<lmmg::Vec3> centerline)
{
  lmmg::RouteEdge result;
  result.id = id;
  result.from = from;
  result.to = to;
  result.centerline = std::move(centerline);
  result.passable = true;
  for (std::size_t index = 1U; index < result.centerline.size(); ++index) {
    result.length += lmmg::distance2d(
      result.centerline[index - 1U], result.centerline[index]);
  }
  return result;
}

bool expect(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

}  // namespace

int main()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {-5.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {2U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kJunction},
    {3U, {5.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {4U, {0.0, 5.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {5U, {-10.0, 10.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
    {6U, {-5.0, 10.0, 0.0}, lmmg::RouteNodeType::kNormal},
    {7U, {0.0, 10.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
  graph.edges = {
    edge(100U, 1U, 2U, {{-5.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}),
    edge(101U, 2U, 3U, {
        {0.0, 0.0, 0.0}, {0.009, 0.0, 0.0}, {5.0, 0.0, 0.0}}),
    edge(102U, 2U, 4U, {
        {0.0, 0.0, 0.0}, {0.0, 0.009, 0.0}, {0.0, 5.0, 0.0}}),
    edge(103U, 5U, 6U, {{-10.0, 10.0, 0.0}, {-5.0, 10.0, 0.0}}),
    edge(104U, 6U, 7U, {
        {-5.0, 10.0, 0.0}, {-4.991, 10.0, 0.0}, {0.0, 10.0, 0.0}})};

  lmmg::ClosedCourseLanelet2ExportOptions options;
  options.estimated_vehicle_width = 1.695;
  options.estimated_front_extent = 3.35;
  options.estimated_rear_extent = 0.60;
  options.estimated_minimum_turning_radius = 4.8;
  options.lateral_clearance_margin = 0.15;
  options.vehicle_profile = "yaris";
  options.vehicle_base_reference = "rear_axle_ground_projection";
  options.vehicle_dimensions_evidence_source = "catalog_estimated";
  options.vehicle_dimensions_evidence_confidence = "medium";

  lmmg::Lanelet2Config config;
  const std::filesystem::path output =
    std::filesystem::temp_directory_path() /
    "lmmg_lanelet_boundary_serialization_regression.osm";
  std::error_code remove_error;
  std::filesystem::remove(output, remove_error);
  const auto summary = lmmg::saveClosedCourseExperimentalLanelet2Osm(
    output, graph, config, options);
  const auto lanelets = lmmg::loadGeneratedLanelet2Osm(output);
  std::filesystem::remove(output, remove_error);

  bool passed = true;
  passed &= expect(
    summary.exported_physical_edges == graph.edges.size() &&
    lanelets.size() == graph.edges.size(),
    "branch/short-segment fixture did not export every topology Edge");
  std::map<std::uint64_t, lmmg::Lanelet2ReviewLanelet> by_edge;
  for (const auto & lanelet : lanelets) {
    by_edge.emplace(lanelet.route_edge_id, lanelet);
  }
  passed &= expect(
    by_edge.at(101U).centerline.size() == 3U &&
    lmmg::distance3d(by_edge.at(101U).centerline[1U], {0.009, 0.0, 0.0}) <= 1.0e-12,
    "topology boundary serialization changed the 9 mm source centreline segment");
  passed &= expect(
    lmmg::distance2d(
      by_edge.at(100U).left_boundary.back(),
      by_edge.at(101U).left_boundary.front()) > 1.0e-3,
    "branch serialization fabricated one common boundary cross-section");
  passed &= expect(
    by_edge.at(104U).centerline.size() == 3U &&
    lmmg::distance3d(by_edge.at(104U).centerline[1U], {-4.991, 10.0, 0.0}) <= 1.0e-12 &&
    by_edge.at(104U).left_boundary.size() == by_edge.at(104U).centerline.size() &&
    by_edge.at(104U).right_boundary.size() == by_edge.at(104U).centerline.size(),
    "unbranched short-segment topology was not serialized without changing its centreline");
  return passed ? 0 : 1;
}
