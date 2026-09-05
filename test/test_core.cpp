#include "lidar_mobility_map_generator/exporters.hpp"
#include "lidar_mobility_map_generator/glim_reader.hpp"
#include "lidar_mobility_map_generator/navigation_outputs.hpp"
#include "lidar_mobility_map_generator/nav2_route_export.hpp"
#include "lidar_mobility_map_generator/pipeline.hpp"
#include "lidar_mobility_map_generator/pointcloud_io.hpp"
#include "lidar_mobility_map_generator/route_graph.hpp"
#include "lidar_mobility_map_generator/route_editor.hpp"
#include "lidar_mobility_map_generator/review_io.hpp"
#include "lidar_mobility_map_generator/semantic_map.hpp"
#include "lidar_mobility_map_generator/trajectory.hpp"
#include "lidar_mobility_map_generator/transform_graph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
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

void checkNear(
  const double actual, const double expected, const double tolerance,
  const std::string & message)
{
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            " expected=" + std::to_string(expected));
  }
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

bool pointInPolygonInclusive(
  const lmmg::Vec3 & point, const std::vector<lmmg::Vec3> & polygon)
{
  bool inside = false;
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    const lmmg::Vec3 & first = polygon[index];
    const lmmg::Vec3 & second = polygon[(index + 1U) % polygon.size()];
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length > 1.0e-18) {
      const double ratio = std::clamp(
        ((point.x - first.x) * dx + (point.y - first.y) * dy) /
        squared_length, 0.0, 1.0);
      const double nearest_x = first.x + ratio * dx;
      const double nearest_y = first.y + ratio * dy;
      if (std::hypot(point.x - nearest_x, point.y - nearest_y) <= 1.0e-9) {
        return true;
      }
    }
    if ((second.y > point.y) != (first.y > point.y)) {
      const double crossing_x =
        (first.x - second.x) * (point.y - second.y) /
        (first.y - second.y) + second.x;
      if (point.x < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

std::size_t physicalGraphComponents(const lmmg::RouteGraph & graph)
{
  std::map<std::uint64_t, std::vector<std::uint64_t>> adjacency;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    if (edge.reverse_of && edge.id > *edge.reverse_of) {
      continue;
    }
    adjacency[edge.from].push_back(edge.to);
    adjacency[edge.to].push_back(edge.from);
  }
  std::set<std::uint64_t> visited;
  std::size_t components = 0U;
  for (const auto & [node_id, neighbors] : adjacency) {
    static_cast<void>(neighbors);
    if (!visited.insert(node_id).second) {
      continue;
    }
    ++components;
    std::queue<std::uint64_t> pending;
    pending.push(node_id);
    while (!pending.empty()) {
      const std::uint64_t current = pending.front();
      pending.pop();
      for (const std::uint64_t next : adjacency[current]) {
        if (visited.insert(next).second) {
          pending.push(next);
        }
      }
    }
  }
  return components;
}

lmmg::MappingDataset syntheticDataset()
{
  lmmg::MappingDataset dataset;
  dataset.world_frame = "map";
  for (std::size_t index = 0U; index <= 100U; ++index) {
    lmmg::TimedPose pose;
    pose.stamp_ns = static_cast<std::int64_t>(index) * 100000000LL;
    pose.world_from_body.translation = {0.1 * static_cast<double>(index), 0.0, 0.30};
    pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(0.0);
    dataset.trajectory.push_back(pose);
  }

  for (double x = -1.0; x <= 11.0; x += 0.20) {
    for (double y = -2.0; y <= 2.0; y += 0.20) {
      dataset.map_points.push_back({
          static_cast<float>(x), static_cast<float>(y), 0.0F, 1.0F});
    }
    for (double z = 0.0; z <= 1.6; z += 0.20) {
      dataset.map_points.push_back({
          static_cast<float>(x), -2.0F, static_cast<float>(z), 100.0F});
      dataset.map_points.push_back({
          static_cast<float>(x), 2.0F, static_cast<float>(z), 100.0F});
    }
  }
  return dataset;
}

}  // namespace

int main()
{
  try {
    const lmmg::Quaternion rotation = lmmg::Quaternion::fromYaw(lmmg::kPi * 0.5);
    const lmmg::Vec3 rotated = rotation.rotate({1.0, 0.0, 0.0});
    checkNear(rotated.x, 0.0, 1.0e-9, "quaternion rotation x");
    checkNear(rotated.y, 1.0, 1.0e-9, "quaternion rotation y");

    lmmg::Transform a{{1.0, 0.0, 0.0}, rotation};
    const lmmg::Vec3 restored = a.inverse().apply(a.apply({0.4, -0.2, 1.0}));
    checkNear(restored.x, 0.4, 1.0e-9, "transform inverse x");
    checkNear(restored.y, -0.2, 1.0e-9, "transform inverse y");

    lmmg::StaticTransformGraph transforms;
    transforms.add("base_link", "lidar", {{0.2, 0.0, 0.5}, {}});
    const auto base_from_lidar = transforms.resolve("base_link", "lidar");
    check(base_from_lidar.has_value(), "static transform graph failed");
    checkNear(base_from_lidar->translation.x, 0.2, 1.0e-9, "static transform x");

    lmmg::TimedPose sensor_pose;
    sensor_pose.stamp_ns = 123;
    sensor_pose.world_from_body = {{10.0, 5.0, 0.0}, lmmg::Quaternion::fromYaw(lmmg::kPi * 0.5)};
    const lmmg::Transform body_from_sensor{{1.0, 0.0, 0.0}, {}};
    const std::vector<lmmg::TimedPose> body_poses = lmmg::sensorPosesToBase(
      {sensor_pose}, body_from_sensor);
    check(body_poses.size() == 1U, "sensor-to-base pose conversion lost a pose");
    checkNear(body_poses.front().world_from_body.translation.x, 10.0, 1.0e-9,
      "sensor-to-base pose conversion x");
    checkNear(body_poses.front().world_from_body.translation.y, 4.0, 1.0e-9,
      "sensor-to-base pose conversion y");
    checkNear(body_poses.front().world_from_body.rotation.yaw(), lmmg::kPi * 0.5, 1.0e-9,
      "sensor-to-base pose conversion yaw");

    std::vector<lmmg::TimedPose> interpolation_poses(2U);
    interpolation_poses[0].stamp_ns = 0;
    interpolation_poses[0].world_from_body.translation = {0.0, 0.0, 0.0};
    interpolation_poses[1].stamp_ns = 1000000000LL;
    interpolation_poses[1].world_from_body.translation = {2.0, 0.0, 0.0};
    lmmg::PoseBuffer buffer(interpolation_poses);
    const auto midpoint = buffer.lookup(500000000LL, 1.0);
    check(midpoint.has_value(), "pose interpolation failed");
    checkNear(midpoint->translation.x, 1.0, 1.0e-9, "pose interpolation x");

    std::vector<lmmg::TimedPose> duplicate_stamp_poses(5U);
    duplicate_stamp_poses[0U].stamp_ns = 20;
    duplicate_stamp_poses[0U].world_from_body.translation.x = 1.0;
    duplicate_stamp_poses[1U].stamp_ns = 10;
    duplicate_stamp_poses[1U].world_from_body.translation.x = 2.0;
    duplicate_stamp_poses[2U].stamp_ns = 20;
    duplicate_stamp_poses[2U].world_from_body.translation.x = 3.0;
    duplicate_stamp_poses[3U].stamp_ns = 30;
    duplicate_stamp_poses[3U].world_from_body.translation.x = 4.0;
    duplicate_stamp_poses[4U].stamp_ns = 10;
    duplicate_stamp_poses[4U].world_from_body.translation.x = 5.0;
    const std::vector<lmmg::TimedPose> normalized_duplicates =
      lmmg::normalizeTrajectory(duplicate_stamp_poses);
    check(normalized_duplicates.size() == 3U, "duplicate pose stamps were not removed");
    check(
      normalized_duplicates[0U].stamp_ns == 10 &&
      normalized_duplicates[1U].stamp_ns == 20 &&
      normalized_duplicates[2U].stamp_ns == 30,
      "normalized pose stamps are not sorted");
    checkNear(
      normalized_duplicates[0U].world_from_body.translation.x, 5.0, 1.0e-9,
      "last duplicate pose at stamp 10 was not retained");
    checkNear(
      normalized_duplicates[1U].world_from_body.translation.x, 3.0, 1.0e-9,
      "last duplicate pose at stamp 20 was not retained");

    std::vector<lmmg::TimedPose> invalid_rotation_poses(4U);
    invalid_rotation_poses[0U].stamp_ns = 1;
    invalid_rotation_poses[0U].world_from_body.rotation.x =
      std::numeric_limits<double>::quiet_NaN();
    invalid_rotation_poses[1U].stamp_ns = 2;
    invalid_rotation_poses[1U].world_from_body.rotation.w =
      std::numeric_limits<double>::infinity();
    invalid_rotation_poses[2U].stamp_ns = 3;
    invalid_rotation_poses[2U].world_from_body.rotation = {0.0, 0.0, 0.0, 0.0};
    invalid_rotation_poses[3U].stamp_ns = 4;
    invalid_rotation_poses[3U].world_from_body.rotation =
      lmmg::Quaternion::fromYaw(0.25);
    const std::vector<lmmg::TimedPose> finite_rotation_poses =
      lmmg::normalizeTrajectory(invalid_rotation_poses);
    check(
      finite_rotation_poses.size() == 1U && finite_rotation_poses.front().stamp_ns == 4,
      "trajectory normalization retained a non-finite or zero quaternion");

    lmmg::StaticTransformGraph invalid_transforms;
    lmmg::Transform nonfinite_transform;
    nonfinite_transform.rotation.y = std::numeric_limits<double>::quiet_NaN();
    invalid_transforms.add("base_link", "invalid_lidar", nonfinite_transform);
    check(
      !invalid_transforms.resolve("base_link", "invalid_lidar").has_value(),
      "static transform graph retained a non-finite quaternion");

    std::vector<lmmg::TimedPose> crossing;
    std::int64_t crossing_stamp = 0;
    auto append_line = [&](double x0, double y0, double x1, double y1, std::size_t steps) {
        for (std::size_t step = 0U; step <= steps; ++step) {
          if (!crossing.empty() && step == 0U) {
            continue;
          }
          const double ratio = static_cast<double>(step) / static_cast<double>(steps);
          lmmg::TimedPose pose;
          pose.stamp_ns = crossing_stamp;
          crossing_stamp += 100000000LL;
          pose.world_from_body.translation = {
            x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0), 0.3};
          crossing.push_back(pose);
        }
      };
    append_line(-2.0, 0.0, 2.0, 0.0, 20U);
    append_line(2.0, 0.0, 2.0, 2.0, 10U);
    append_line(2.0, 2.0, 0.0, 2.0, 10U);
    append_line(0.0, 2.0, 0.0, -2.0, 20U);
    lmmg::TopologyConfig crossing_config;
    crossing_config.minimum_loop_separation = 1.0;
    crossing_config.maximum_edge_length = 20.0;
    const lmmg::RouteGraph crossing_graph = lmmg::buildRouteGraph(
      crossing, crossing_config, "map");
    check(std::any_of(
      crossing_graph.nodes.begin(), crossing_graph.nodes.end(),
        [](const lmmg::RouteNode & node) {return node.type == lmmg::RouteNodeType::kJunction;}),
      "crossing trajectory did not produce a junction");

    // An equal-length artificial cut 4.95 m into this 9.9 m chain lies only
    // 0.5 mm before a retained source vertex.  Autoware 1.9 would merge that
    // pair and could duplicate a Lanelet ID in PathWithLaneId.  Snap only the
    // artificial topology cut to the source vertex: all source arc and both
    // final Edges remain present.
    std::vector<lmmg::TimedPose> topology_cut_trajectory(3U);
    topology_cut_trajectory[0U].world_from_body.translation = {0.0, 0.0, 0.0};
    topology_cut_trajectory[1U].world_from_body.translation = {4.9505, 0.0, 0.0};
    topology_cut_trajectory[2U].world_from_body.translation = {9.9, 0.0, 0.0};
    for (std::size_t index = 0U; index < topology_cut_trajectory.size(); ++index) {
      topology_cut_trajectory[index].stamp_ns =
        static_cast<std::int64_t>(index) * 100000000LL;
    }
    lmmg::TopologyConfig topology_cut_config;
    topology_cut_config.node_merge_distance = 0.01;
    topology_cut_config.intersection_merge_distance = 0.01;
    topology_cut_config.minimum_edge_length = 0.1;
    topology_cut_config.maximum_edge_length = 5.0;
    topology_cut_config.geometry_smoothing_window = 0.0;
    const lmmg::RouteGraph topology_cut_graph = lmmg::buildRouteGraph(
      topology_cut_trajectory, topology_cut_config, "map");
    check(topology_cut_graph.edges.size() == 2U,
      "topology cut snapping changed the expected Edge count");
    double topology_cut_total_length = 0.0;
    for (const lmmg::RouteEdge & edge : topology_cut_graph.edges) {
      topology_cut_total_length += edge.length;
      check(edge.length <= topology_cut_config.maximum_edge_length + 1.0e-12,
        "snapped topology Edge exceeds maximum_edge_length");
      for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
        check(lmmg::distance3d(
            edge.centerline[index - 1U], edge.centerline[index]) >= 0.001,
          "topology partition retained a sub-millimetre centerline segment");
      }
    }
    checkNear(topology_cut_total_length, 9.9, 1.0e-12,
      "topology cut snapping shortened the source Route");
    lmmg::TopologyConfig topology_uncut_config = topology_cut_config;
    topology_uncut_config.maximum_edge_length = 20.0;
    const lmmg::RouteGraph topology_uncut_graph = lmmg::buildRouteGraph(
      topology_cut_trajectory, topology_uncut_config, "map");
    check(topology_uncut_graph.edges.size() == 1U,
      "20 m topology contract unexpectedly partitioned a 9.9 m chain");
    check(topology_uncut_graph.edges.front().centerline.size() ==
      topology_cut_trajectory.size(),
      "20 m topology contract removed a processed trajectory vertex");
    checkNear(topology_uncut_graph.edges.front().length,
      topology_cut_total_length, 1.0e-12,
      "5 m and 20 m topology contracts changed full Route length");

    // Three revisits form a single-link chain in lateral distance: the first
    // and second paths are mergeable and the second and third paths are
    // mergeable, but the first and third are farther apart than the configured
    // node merge distance.  A transitive disjoint-set merge collapses all
    // three traversals and drops the outer path from the route geometry.
    std::vector<lmmg::TimedPose> merge_chain;
    std::int64_t merge_chain_stamp = 0;
    auto append_merge_chain_line =
      [&](const double x0, const double y0, const double x1, const double y1,
      const std::size_t steps)
      {
        for (std::size_t step = 0U; step <= steps; ++step) {
          if (!merge_chain.empty() && step == 0U) {
            continue;
          }
          const double ratio = static_cast<double>(step) / static_cast<double>(steps);
          lmmg::TimedPose pose;
          pose.stamp_ns = merge_chain_stamp;
          merge_chain_stamp += 100000000LL;
          pose.world_from_body.translation = {
            x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0), 0.0};
          merge_chain.push_back(pose);
        }
      };
    append_merge_chain_line(0.0, 0.0, 4.0, 0.0, 10U);
    append_merge_chain_line(4.0, 0.0, 6.0, 2.0, 5U);
    append_merge_chain_line(6.0, 2.0, 4.0, 0.4, 5U);
    append_merge_chain_line(4.0, 0.4, 0.0, 0.4, 10U);
    append_merge_chain_line(0.0, 0.4, -2.0, 2.0, 5U);
    append_merge_chain_line(-2.0, 2.0, 0.0, 0.8, 5U);
    append_merge_chain_line(0.0, 0.8, 4.0, 0.8, 10U);
    lmmg::TopologyConfig merge_chain_config;
    merge_chain_config.node_merge_distance = 0.45;
    merge_chain_config.intersection_merge_distance = 0.30;
    merge_chain_config.minimum_loop_separation = 1.0;
    merge_chain_config.minimum_edge_length = 0.10;
    merge_chain_config.maximum_edge_length = 100.0;
    const lmmg::RouteGraph merge_chain_graph = lmmg::buildRouteGraph(
      merge_chain, merge_chain_config, "map");
    const bool retained_outer_traversal = std::any_of(
      merge_chain_graph.edges.begin(), merge_chain_graph.edges.end(),
      [](const lmmg::RouteEdge & edge) {
        return std::any_of(
          edge.centerline.begin(), edge.centerline.end(),
          [](const lmmg::Vec3 & point) {
            return point.x > 1.0 && point.x < 3.0 && point.y > 0.70;
          });
      });
    check(
      retained_outer_traversal,
      "complete-link node merging dropped a traversal beyond node_merge_distance");

    // Regression for the GLIM Velodyne closure. Dense samples at one physical
    // crossing formed two adjacent mixed-heading complete-link clusters. The
    // sub-minimum edge between their 0.28 m centroids was then discarded,
    // leaving the loop and through route as separate graph components.
    const std::vector<lmmg::Vec3> closure_positions{
      {25.5254777642, -30.9918027506, 0.0},
      {25.7143706532, -31.0574058323, 0.0},
      {25.9032990353, -31.1229086053, 0.0},
      {26.0923439977, -31.1880783272, 0.0},
      {26.2811592796, -31.2538696492, 0.0},
      {26.4696957514, -31.3204012007, 0.0},
      {26.6578693799, -31.3879629324, 0.0},
      {28.0, -32.0, 0.0}, {30.0, -34.0, 0.0}, {31.0, -37.0, 0.0},
      {28.0, -39.0, 0.0}, {25.0, -37.0, 0.0},
      {26.3590576005, -31.9976033286, 0.0},
      {26.2878721712, -31.8111759852, 0.0},
      {26.2213813840, -31.6231148430, 0.0},
      {26.1588540142, -31.4337372419, 0.0},
      {26.1000623348, -31.2432586030, 0.0},
      {26.0449622684, -31.0516360759, 0.0},
      {25.9928363009, -30.8590621361, 0.0},
      {25.9427942631, -30.6658830892, 0.0},
      {25.8949986889, -30.4722096124, 0.0}};
    auto trajectoryFromPositions = [](const std::vector<lmmg::Vec3> & positions) {
        std::vector<lmmg::TimedPose> result;
        result.reserve(positions.size());
        for (std::size_t index = 0U; index < positions.size(); ++index) {
          lmmg::TimedPose pose;
          pose.stamp_ns = static_cast<std::int64_t>(index) * 100000000LL;
          pose.world_from_body.translation = positions[index];
          result.push_back(pose);
        }
        return result;
      };
    lmmg::TopologyConfig closure_config;
    closure_config.node_merge_distance = 0.45;
    closure_config.intersection_merge_distance = 0.30;
    closure_config.minimum_loop_separation = 2.0;
    closure_config.minimum_edge_length = 0.50;
    closure_config.maximum_edge_length = 100.0;
    closure_config.geometry_smoothing_window = 0.0;
    const lmmg::RouteGraph closure_graph = lmmg::buildRouteGraph(
      trajectoryFromPositions(closure_positions), closure_config, "map");
    check(physicalGraphComponents(closure_graph) == 1U,
      "bounded physical loop closure remained split into graph components");
    check(std::any_of(
      closure_graph.nodes.begin(), closure_graph.nodes.end(),
        [](const lmmg::RouteNode & node) {
          return node.type == lmmg::RouteNodeType::kJunction &&
                 std::hypot(node.position.x - 26.08, node.position.y + 31.16) < 0.20;
      }),
      "bounded physical loop closure did not produce one intersection node");

    // Identical XY crossings at different elevations are not a physical
    // junction. Both height layers must survive without a centroid between
    // them, even though their projected headings cross.
    std::vector<lmmg::Vec3> grade_separated_positions = closure_positions;
    for (std::size_t index = 12U; index < grade_separated_positions.size(); ++index) {
      grade_separated_positions[index].z = 3.0;
    }
    const lmmg::RouteGraph grade_separated_graph = lmmg::buildRouteGraph(
      trajectoryFromPositions(grade_separated_positions), closure_config, "map");
    bool retained_lower_crossing = false;
    bool retained_upper_crossing = false;
    for (const lmmg::RouteEdge & edge : grade_separated_graph.edges) {
      for (const lmmg::Vec3 & point : edge.centerline) {
        if (std::hypot(point.x - 26.08, point.y + 31.16) < 0.45) {
          retained_lower_crossing = retained_lower_crossing || point.z < 0.5;
          retained_upper_crossing = retained_upper_crossing || point.z > 2.5;
        }
      }
    }
    check(retained_lower_crossing && retained_upper_crossing,
      "grade-separated crossing was collapsed into one physical node");
    check(std::none_of(
      grade_separated_graph.nodes.begin(), grade_separated_graph.nodes.end(),
        [](const lmmg::RouteNode & node) {
          return node.type == lmmg::RouteNodeType::kJunction &&
                 std::hypot(node.position.x - 26.08, node.position.y + 31.16) < 0.50;
      }),
      "grade-separated crossing was incorrectly classified as a junction");

    // A reversal at the apparent closure is a maneuver boundary, not proof of
    // a traversable physical junction. Even reciprocal nearby observations
    // must not consolidate clusters containing a sharp/cusp sample.
    std::vector<lmmg::Vec3> cusp_positions = closure_positions;
    cusp_positions[4U] = cusp_positions[2U];
    const lmmg::RouteGraph cusp_graph = lmmg::buildRouteGraph(
      trajectoryFromPositions(cusp_positions), closure_config, "map");
    check(std::none_of(
      cusp_graph.nodes.begin(), cusp_graph.nodes.end(),
        [](const lmmg::RouteNode & node) {
          return node.type == lmmg::RouteNodeType::kJunction &&
                 std::hypot(node.position.x - 26.08, node.position.y + 31.16) < 0.60;
      }),
      "cusp/reversal was incorrectly consolidated into a physical junction");

    // Nearby single-heading paths do not provide intersection evidence and
    // therefore remain two distinct pieces of route geometry.
    std::vector<lmmg::Vec3> parallel_positions;
    for (std::size_t index = 0U; index <= 20U; ++index) {
      parallel_positions.push_back({0.2 * static_cast<double>(index), 0.0, 0.0});
    }
    parallel_positions.push_back({5.0, 1.5, 0.0});
    parallel_positions.push_back({4.0, 0.25, 0.0});
    for (std::size_t index = 1U; index <= 20U; ++index) {
      parallel_positions.push_back({4.0 - 0.2 * static_cast<double>(index), 0.25, 0.0});
    }
    lmmg::TopologyConfig parallel_config = closure_config;
    parallel_config.node_merge_distance = 0.10;
    parallel_config.minimum_edge_length = 0.10;
    const lmmg::RouteGraph parallel_graph = lmmg::buildRouteGraph(
      trajectoryFromPositions(parallel_positions), parallel_config, "map");
    bool retained_lower_parallel = false;
    bool retained_upper_parallel = false;
    for (const lmmg::RouteEdge & edge : parallel_graph.edges) {
      for (const lmmg::Vec3 & point : edge.centerline) {
        if (point.x > 1.0 && point.x < 3.0) {
          retained_lower_parallel = retained_lower_parallel || std::abs(point.y) < 0.05;
          retained_upper_parallel = retained_upper_parallel || std::abs(point.y - 0.25) < 0.05;
        }
      }
    }
    check(retained_lower_parallel && retained_upper_parallel,
      "nearby parallel paths were incorrectly merged by intersection consolidation");

    lmmg::RouteGraph clearance_graph;
    lmmg::RouteEdge forward_clearance_edge;
    forward_clearance_edge.id = 1U;
    forward_clearance_edge.from = 1U;
    forward_clearance_edge.to = 2U;
    forward_clearance_edge.reverse_of = 2U;
    forward_clearance_edge.centerline = {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    lmmg::RouteEdge reverse_clearance_edge = forward_clearance_edge;
    reverse_clearance_edge.id = 2U;
    reverse_clearance_edge.from = 2U;
    reverse_clearance_edge.to = 1U;
    reverse_clearance_edge.reverse_of = 1U;
    std::reverse(
      reverse_clearance_edge.centerline.begin(), reverse_clearance_edge.centerline.end());
    clearance_graph.edges = {forward_clearance_edge, reverse_clearance_edge};

    lmmg::OccupancyGrid2D clearance_grid(-1.0, -1.0, 0.10, 41U, 21U);
    const auto middle_obstacle_cell = clearance_grid.worldToCell(1.0, 0.0);
    check(middle_obstacle_cell.has_value(), "clearance test obstacle is outside the grid");
    clearance_grid.setOccupied(middle_obstacle_cell->first, middle_obstacle_cell->second);
    lmmg::OccupancyGrid2D swept_space_grid(-1.0, -1.0, 0.10, 41U, 21U);
    swept_space_grid.setOccupied(middle_obstacle_cell->first, middle_obstacle_cell->second);
    const auto distant_obstacle_cell = swept_space_grid.worldToCell(1.5, 0.0);
    check(distant_obstacle_cell.has_value(), "swept-space test obstacle is outside the grid");
    swept_space_grid.setOccupied(distant_obstacle_cell->first, distant_obstacle_cell->second);
    check(
      swept_space_grid.clearDisk(1.0, 0.0, 0.20) == 1U,
      "observed-trajectory clearance did not report the cleared obstacle");
    check(
      !swept_space_grid.isOccupiedWorld(1.0, 0.0) &&
      swept_space_grid.isOccupiedWorld(1.5, 0.0),
      "observed-trajectory clearance erased cells outside its radius");
    lmmg::TraversabilityConfig clearance_config;
    clearance_config.grid_resolution = 0.10;
    clearance_config.ray_step = 0.05;
    clearance_config.maximum_corridor_half_width = 0.50;
    lmmg::OccupancyGrid2D clearance_unknown_grid(-1.0, -1.0, 0.10, 41U, 21U);
    lmmg::computeRouteClearance(
      clearance_graph, clearance_grid, clearance_unknown_grid, clearance_config, 1.0);
    check(
      !clearance_graph.edges[0U].passable && !clearance_graph.edges[1U].passable,
      "clearance sampling skipped an obstacle between centerline vertices");
    check(
      clearance_graph.edges[0U].centerline.size() > 2U &&
      clearance_graph.edges[0U].centerline.size() ==
      clearance_graph.edges[1U].centerline.size(),
      "clearance centerlines were not densified symmetrically");
    checkNear(
      clearance_graph.edges[0U].centerline.front().x, 0.0, 1.0e-12,
      "forward clearance start changed during densification");
    checkNear(
      clearance_graph.edges[0U].centerline.back().x, 2.0, 1.0e-12,
      "forward clearance end changed during densification");
    for (std::size_t index = 0U;
      index < clearance_graph.edges[0U].centerline.size(); ++index)
    {
      const std::size_t reverse_index =
        clearance_graph.edges[1U].centerline.size() - 1U - index;
      checkNear(
        clearance_graph.edges[0U].centerline[index].x,
        clearance_graph.edges[1U].centerline[reverse_index].x, 1.0e-12,
        "reverse clearance geometry changed during densification");
      checkNear(
        clearance_graph.edges[0U].centerline[index].y,
        clearance_graph.edges[1U].centerline[reverse_index].y, 1.0e-12,
        "reverse clearance geometry changed during densification");
    }

    lmmg::RouteGraph lateral_raycast_graph;
    lmmg::RouteEdge lateral_raycast_edge;
    lateral_raycast_edge.id = 1U;
    lateral_raycast_edge.from = 1U;
    lateral_raycast_edge.to = 2U;
    lateral_raycast_edge.centerline = {{0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}};
    lateral_raycast_graph.edges.push_back(lateral_raycast_edge);
    lmmg::OccupancyGrid2D lateral_raycast_grid(-1.0, -1.0, 0.10, 21U, 21U);
    const auto lateral_obstacle_cell = lateral_raycast_grid.worldToCell(0.0, 0.2);
    check(lateral_obstacle_cell.has_value(), "lateral raycast obstacle is outside the grid");
    lateral_raycast_grid.setOccupied(lateral_obstacle_cell->first, lateral_obstacle_cell->second);
    lmmg::TraversabilityConfig coarse_raycast_config;
    coarse_raycast_config.grid_resolution = 0.10;
    coarse_raycast_config.ray_step = 0.30;
    coarse_raycast_config.maximum_corridor_half_width = 0.50;
    lmmg::OccupancyGrid2D lateral_unknown_grid(-1.0, -1.0, 0.10, 21U, 21U);
    lmmg::computeRouteClearance(
      lateral_raycast_graph, lateral_raycast_grid, lateral_unknown_grid,
      coarse_raycast_config, 1.0);
    check(
      lateral_raycast_graph.edges.front().left_clearance.front() < 0.30,
      "coarse lateral ray step skipped an occupied grid cell");

    lmmg::RouteGraph known_free_graph;
    lmmg::RouteEdge known_free_edge;
    known_free_edge.id = 1U;
    known_free_edge.from = 1U;
    known_free_edge.to = 2U;
    known_free_edge.centerline = {{-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0}};
    known_free_graph.edges.push_back(known_free_edge);
    lmmg::OccupancyGrid2D known_free_obstacles(-1.0, -1.0, 0.10, 21U, 21U);
    lmmg::OccupancyGrid2D no_unknown_cells(-1.0, -1.0, 0.10, 21U, 21U);
    lmmg::TraversabilityConfig known_free_config;
    known_free_config.ray_step = 0.05;
    known_free_config.maximum_corridor_half_width = 0.50;
    lmmg::computeRouteClearance(
      known_free_graph, known_free_obstacles, no_unknown_cells, known_free_config, 1.0);
    check(
      known_free_graph.edges.front().passable,
      "explicitly known-free ray extent was not accepted");
    checkNear(
      known_free_graph.edges.front().minimum_safe_width, 1.0, 1.0e-9,
      "known-free corridor width");
    check(
      known_free_graph.edges.front().corridor_geometry_valid &&
      known_free_graph.edges.front().confidence == 1.0,
      "known-free corridor validation metadata is incorrect");

    lmmg::RouteGraph unknown_clearance_graph;
    unknown_clearance_graph.edges.push_back(known_free_edge);
    lmmg::OccupancyGrid2D all_unknown_cells(-1.0, -1.0, 0.10, 21U, 21U);
    for (std::int64_t y = 0; y < 21; ++y) {
      for (std::int64_t x = 0; x < 21; ++x) {
        all_unknown_cells.setOccupied(x, y);
      }
    }
    lmmg::computeRouteClearance(
      unknown_clearance_graph, known_free_obstacles, all_unknown_cells,
      known_free_config, 1.0);
    const lmmg::RouteEdge & unknown_edge = unknown_clearance_graph.edges.front();
    check(!unknown_edge.passable, "UNKNOWN corridor was accepted as passable");
    checkNear(unknown_edge.minimum_safe_width, 0.0, 1.0e-12, "UNKNOWN corridor width");
    check(std::find(
      unknown_edge.validation_errors.begin(), unknown_edge.validation_errors.end(),
      "unknown_clearance") != unknown_edge.validation_errors.end(),
      "UNKNOWN corridor diagnostic is missing");
    checkNear(
      lmmg::distance2d(unknown_edge.left_boundary.front(), unknown_edge.centerline.front()),
      0.0, 1.0e-12, "UNKNOWN left boundary did not collapse to centerline");
    checkNear(
      lmmg::distance2d(unknown_edge.right_boundary.front(), unknown_edge.centerline.front()),
      0.0, 1.0e-12, "UNKNOWN right boundary did not collapse to centerline");

    // UNKNOWN may safely form the outside boundary of a corridor when the
    // contiguous observed-free strip itself is wide enough. It must never be
    // traversed or treated as additional clearance.
    lmmg::RouteGraph bounded_known_free_graph;
    bounded_known_free_graph.edges.push_back(known_free_edge);
    lmmg::OccupancyGrid2D bounded_unknown = all_unknown_cells;
    for (double x = -0.2; x <= 0.2 + 1.0e-9; x += 0.1) {
      bounded_unknown.clearDisk(x, 0.0, 0.30);
    }
    lmmg::computeRouteClearance(
      bounded_known_free_graph, known_free_obstacles, bounded_unknown,
      known_free_config, 1.0);
    const lmmg::RouteEdge & bounded_edge = bounded_known_free_graph.edges.front();
    check(
      bounded_edge.passable,
      "sufficient observed-free width bounded by UNKNOWN was rejected");
    check(
      bounded_edge.minimum_safe_width >= known_free_config.minimum_safe_center_width,
      "bounded observed-free corridor is narrower than the configured minimum");
    check(
      std::find(
        bounded_edge.validation_errors.begin(), bounded_edge.validation_errors.end(),
        "unknown_clearance") != bounded_edge.validation_errors.end(),
      "bounded UNKNOWN diagnostic is missing");

    lmmg::RouteGraph continuity_graph;
    continuity_graph.edges.push_back(known_free_edge);
    lmmg::OccupancyGrid2D continuity_obstacles(-1.0, -1.0, 0.10, 21U, 21U);
    for (std::int64_t x = 0; x < 21; ++x) {
      const auto wall = continuity_obstacles.worldToCell(
        -0.95 + 0.10 * static_cast<double>(x), 0.60);
      if (wall) {
        continuity_obstacles.setOccupied(wall->first, wall->second);
      }
    }
    const auto notch = continuity_obstacles.worldToCell(0.0, 0.20);
    check(notch.has_value(), "continuity notch is outside grid");
    continuity_obstacles.setOccupied(notch->first, notch->second);
    lmmg::computeRouteClearance(
      continuity_graph, continuity_obstacles, no_unknown_cells, known_free_config, 1.0);
    const lmmg::RouteEdge & continuous_edge = continuity_graph.edges.front();
    for (std::size_t index = 1U; index < continuous_edge.left_clearance.size(); ++index) {
      const double spacing = lmmg::distance2d(
        continuous_edge.centerline[index - 1U], continuous_edge.centerline[index]);
      check(
        std::abs(
          continuous_edge.left_clearance[index] -
          continuous_edge.left_clearance[index - 1U]) <= spacing + 1.0e-9,
        "clearance continuity limiter exceeded its conservative slope");
    }

    lmmg::RouteGraph crossed_centerline_graph;
    lmmg::RouteEdge crossed_centerline_edge;
    crossed_centerline_edge.id = 1U;
    crossed_centerline_edge.from = 1U;
    crossed_centerline_edge.to = 2U;
    crossed_centerline_edge.centerline = {
      {-0.4, -0.4, 0.0}, {0.4, 0.4, 0.0},
      {-0.4, 0.4, 0.0}, {0.4, -0.4, 0.0}};
    crossed_centerline_graph.edges.push_back(crossed_centerline_edge);
    lmmg::computeRouteClearance(
      crossed_centerline_graph, known_free_obstacles, no_unknown_cells,
      known_free_config, 1.0);
    check(
      !crossed_centerline_graph.edges.front().passable &&
      std::find(
        crossed_centerline_graph.edges.front().validation_errors.begin(),
        crossed_centerline_graph.edges.front().validation_errors.end(),
        "centerline_self_intersection") !=
      crossed_centerline_graph.edges.front().validation_errors.end(),
      "self-intersecting centerline did not fail closed");

    std::vector<lmmg::TimedPose> closed_loop;
    for (std::size_t index = 0U; index <= 40U; ++index) {
      const double angle = 2.0 * lmmg::kPi * static_cast<double>(index) / 40.0;
      lmmg::TimedPose pose;
      pose.stamp_ns = static_cast<std::int64_t>(index) * 100000000LL;
      pose.world_from_body.translation = {std::cos(angle), std::sin(angle), 0.0};
      pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(angle + lmmg::kPi * 0.5);
      closed_loop.push_back(pose);
    }
    lmmg::TopologyConfig loop_config;
    loop_config.minimum_edge_length = 0.10;
    loop_config.maximum_edge_length = 20.0;
    const lmmg::RouteGraph closed_loop_graph = lmmg::buildRouteGraph(
      closed_loop, loop_config, "map");
    check(closed_loop_graph.edges.size() >= 2U, "closed loop was not split into ordinary edges");
    check(std::none_of(
      closed_loop_graph.edges.begin(), closed_loop_graph.edges.end(),
        [](const lmmg::RouteEdge & edge) {return edge.from == edge.to;}),
      "closed route produced an unsafe self-loop edge");

    std::vector<lmmg::TimedPose> preserved_yaw_input(3U);
    for (std::size_t index = 0U; index < preserved_yaw_input.size(); ++index) {
      preserved_yaw_input[index].stamp_ns = static_cast<std::int64_t>(index) * 1000000000LL;
      preserved_yaw_input[index].world_from_body.translation.x = static_cast<double>(index);
      preserved_yaw_input[index].world_from_body.rotation =
        lmmg::Quaternion::fromYaw(lmmg::kPi * 0.5);
    }
    lmmg::TrajectoryConfig preserved_yaw_config;
    checkNear(
      preserved_yaw_config.smoothing_window, 0.0, 0.0,
      "trajectory smoothing must be opt-in because it can leave the measured body sweep");
    preserved_yaw_config.minimum_translation = 0.01;
    preserved_yaw_config.resample_interval = 1.0;
    preserved_yaw_config.smoothing_window = 0.0;
    const std::vector<lmmg::TimedPose> preserved_yaw = lmmg::processTrajectory(
      preserved_yaw_input, preserved_yaw_config);
    checkNear(
      preserved_yaw[1U].world_from_body.rotation.yaw(), lmmg::kPi * 0.5, 1.0e-9,
      "processed trajectory overwrote measured body yaw with path tangent");

    const auto makeTrajectory = [](
      const std::vector<lmmg::Vec3> & positions, const double yaw) {
        std::vector<lmmg::TimedPose> result;
        result.reserve(positions.size());
        for (std::size_t index = 0U; index < positions.size(); ++index) {
          lmmg::TimedPose pose;
          pose.stamp_ns = static_cast<std::int64_t>(index) * 1000000000LL;
          pose.world_from_body.translation = positions[index];
          pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(yaw);
          result.push_back(pose);
        }
        return result;
      };
    const auto maximumPlanarTurnDeg = [](const std::vector<lmmg::TimedPose> & trajectory) {
        double maximum = 0.0;
        for (std::size_t index = 1U; index + 1U < trajectory.size(); ++index) {
          const lmmg::Vec3 incoming =
            trajectory[index].world_from_body.translation -
            trajectory[index - 1U].world_from_body.translation;
          const lmmg::Vec3 outgoing =
            trajectory[index + 1U].world_from_body.translation -
            trajectory[index].world_from_body.translation;
          if (std::hypot(incoming.x, incoming.y) <= 1.0e-9 ||
            std::hypot(outgoing.x, outgoing.y) <= 1.0e-9)
          {
            continue;
          }
          maximum = std::max(
            maximum,
            std::abs(lmmg::normalizeAngle(
              std::atan2(outgoing.y, outgoing.x) -
              std::atan2(incoming.y, incoming.x))) * 180.0 / lmmg::kPi);
        }
        return maximum;
      };

    const std::vector<lmmg::TimedPose> jitter_input = makeTrajectory(
      {{0.0, 0.0, 0.0}, {0.0, -1.0, 0.0},
        {-0.1, -1.0, 0.0}, {-0.1, -2.0, 0.0}},
      -lmmg::kPi * 0.5);
    lmmg::TrajectoryConfig jitter_config;
    jitter_config.minimum_translation = 0.001;
    jitter_config.resample_interval = 0.10;
    jitter_config.smoothing_window = 0.0;
    lmmg::TrajectoryProcessingAudit jitter_audit;
    const std::vector<lmmg::TimedPose> corrected_jitter = lmmg::processTrajectory(
      jitter_input, jitter_config, nullptr, &jitter_audit);
    check(
      jitter_audit.raw_trajectory_preserved &&
      jitter_audit.corrected_position_jitter_poses == 2U &&
      jitter_audit.corrected_position_jitter_runs == 1U,
      "isolated body-yaw-inconsistent position jitter was not audited");
    check(
      jitter_audit.maximum_planar_position_correction_m > 0.0,
      "position-jitter correction did not report its maximum planar deviation");
    check(
      maximumPlanarTurnDeg(corrected_jitter) < 90.0,
      "position-jitter correction left a sharp centerline reversal");
    checkNear(
      jitter_input[1U].world_from_body.translation.x, 0.0, 0.0,
      "position-jitter correction mutated the raw input trajectory");

    // A short longitudinal backtrack is a genuine reverse observation: its
    // direction remains aligned with the body axis and must not be repaired.
    const std::vector<lmmg::TimedPose> reverse_input = makeTrajectory(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {0.9, 0.0, 0.0}, {1.9, 0.0, 0.0}}, 0.0);
    lmmg::TrajectoryProcessingAudit reverse_audit;
    const std::vector<lmmg::TimedPose> preserved_reverse = lmmg::processTrajectory(
      reverse_input, jitter_config, nullptr, &reverse_audit);
    check(
      reverse_audit.corrected_position_jitter_poses == 0U &&
      reverse_audit.corrected_position_jitter_runs == 0U,
      "body-axis-aligned reverse traversal was mistaken for position jitter");
    check(
      maximumPlanarTurnDeg(preserved_reverse) > 150.0,
      "genuine reverse traversal was not preserved in the processed trajectory");

    lmmg::MappingDataset corner_dataset;
    corner_dataset.world_frame = "map";
    std::int64_t corner_stamp = 0;
    auto append_corner_pose = [&](const double x, const double y) {
        lmmg::TimedPose pose;
        pose.stamp_ns = corner_stamp;
        corner_stamp += 100000000LL;
        pose.world_from_body.translation = {x, y, 0.30};
        pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(0.0);
        corner_dataset.trajectory.push_back(pose);
      };
    for (std::size_t step = 0U; step <= 10U; ++step) {
      append_corner_pose(0.1 * static_cast<double>(step), 0.0);
    }
    for (std::size_t step = 1U; step <= 10U; ++step) {
      append_corner_pose(1.0, 0.1 * static_cast<double>(step));
    }
    for (double x = -0.2; x <= 1.2; x += 0.10) {
      for (double y = -0.2; y <= 1.2; y += 0.10) {
        corner_dataset.map_points.push_back({
          static_cast<float>(x), static_cast<float>(y), 0.0F, 1.0F});
      }
    }
    const lmmg::Vec2 shortcut_obstacle{0.89, 0.11};
    corner_dataset.map_points.push_back({
      static_cast<float>(shortcut_obstacle.x),
      static_cast<float>(shortcut_obstacle.y), 0.60F, 100.0F});

    lmmg::GeneratorConfig corner_config;
    corner_config.trajectory.minimum_translation = 0.01;
    corner_config.trajectory.resample_interval = 0.10;
    corner_config.trajectory.smoothing_window = 0.80;
    corner_config.traversability.grid_resolution = 0.02;
    corner_config.traversability.trajectory_crop_radius = 1.0;
    corner_config.traversability.ground_estimation_radius = 0.35;
    corner_config.traversability.minimum_ground_points = 3U;
    corner_config.traversability.minimum_obstacle_points_per_cell = 1U;
    corner_config.traversability.minimum_obstacle_neighbor_points = 1U;
    corner_config.traversability.observed_trajectory_clearance_radius = 0.08;
    corner_config.robot.width = 0.10;
    corner_config.robot.clearance_margin = 0.02;
    corner_config.topology.minimum_edge_length = 0.05;
    corner_config.topology.maximum_edge_length = 20.0;

    const lmmg::PipelineResult corner_result = lmmg::runVectorMapPipeline(
      corner_dataset, corner_config);
    double raw_shortcut_distance = 10.0;
    for (const lmmg::TimedPose & pose : corner_dataset.trajectory) {
      raw_shortcut_distance = std::min(
        raw_shortcut_distance,
        std::hypot(
          shortcut_obstacle.x - pose.world_from_body.translation.x,
          shortcut_obstacle.y - pose.world_from_body.translation.y));
    }
    double processed_shortcut_distance = 10.0;
    for (const lmmg::TimedPose & pose : corner_result.generation.processed_trajectory) {
      processed_shortcut_distance = std::min(
        processed_shortcut_distance,
        std::hypot(
          shortcut_obstacle.x - pose.world_from_body.translation.x,
          shortcut_obstacle.y - pose.world_from_body.translation.y));
    }
    check(
      raw_shortcut_distance > corner_config.traversability.observed_trajectory_clearance_radius &&
      processed_shortcut_distance <
      corner_config.traversability.observed_trajectory_clearance_radius,
      "corner smoothing fixture does not distinguish observed and processed trajectories");
    check(
      corner_result.grids.obstacle_grid.isOccupiedWorld(
        shortcut_obstacle.x, shortcut_obstacle.y),
      "trajectory smoothing cleared an obstacle in an unobserved corner shortcut");

    // A cross-sloped road must be estimated in XY.  Extending the centerline
    // floor height sideways would classify the high side of this road as an
    // obstacle.  The same fixture also checks support filtering, vertical
    // platform envelopes, explicit free evidence, and UNKNOWN diagnostics.
    std::vector<lmmg::TimedPose> sloped_trajectory;
    for (std::size_t step = 0U; step <= 20U; ++step) {
      lmmg::TimedPose pose;
      pose.stamp_ns = static_cast<std::int64_t>(step) * 100000000LL;
      pose.world_from_body.translation = {0.2 * static_cast<double>(step), 0.0, 1.0};
      pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(0.0);
      sloped_trajectory.push_back(pose);
    }
    std::vector<lmmg::PointXYZI> sloped_points;
    for (double x = -1.0; x <= 5.0; x += 0.10) {
      for (double y = -2.0; y <= 2.0; y += 0.10) {
        sloped_points.push_back({
          static_cast<float>(x), static_cast<float>(y),
          static_cast<float>(0.10 * y), 1.0F});
      }
    }
    // One unsupported return must not become an occupied cell.
    sloped_points.push_back({1.01F, -1.01F, 0.30F, 10.0F});
    // A spatially and temporally supported obstacle must remain.
    for (std::size_t sample = 0U; sample < 3U; ++sample) {
      lmmg::PointXYZI obstacle{
        static_cast<float>(3.01 + 0.02 * static_cast<double>(sample)),
        -1.01F, 0.31F, 20.0F};
      obstacle.observation_count = 3U;
      sloped_points.push_back(obstacle);
    }
    // These returns are above a small robot's collision envelope.
    for (std::size_t sample = 0U; sample < 3U; ++sample) {
      sloped_points.push_back({
        static_cast<float>(2.01 + 0.02 * static_cast<double>(sample)),
        1.01F, 1.30F, 30.0F});
    }

    lmmg::TraversabilityConfig sloped_config;
    sloped_config.grid_resolution = 0.10;
    sloped_config.trajectory_crop_radius = 2.5;
    sloped_config.ground_cell_resolution = 0.40;
    sloped_config.minimum_ground_points_per_cell = 3U;
    sloped_config.ground_plane_radius = 1.0;
    sloped_config.minimum_ground_cells_for_plane = 3U;
    sloped_config.maximum_ground_slope = 0.20;
    sloped_config.maximum_ground_plane_residual = 0.03;
    sloped_config.minimum_obstacle_points_per_cell = 2U;
    sloped_config.minimum_obstacle_neighbor_points = 3U;
    sloped_config.obstacle_support_radius_cells = 1U;
    sloped_config.observed_trajectory_clearance_radius = 0.30;
    lmmg::RobotConfig small_robot;
    small_robot.profile = "small_robot";
    small_robot.width = 0.40;
    small_robot.clearance_margin = 0.05;
    small_robot.minimum_collision_height = 0.08;
    small_robot.maximum_collision_height = 0.80;
    const lmmg::TraversabilityGridResult sloped_grid = lmmg::buildTraversabilityGrid(
      sloped_points, sloped_trajectory, sloped_trajectory, sloped_config, small_robot);
    check(
      !sloped_grid.obstacle_grid.isOccupiedWorld(2.0, 1.9),
      "cross-sloped road was misclassified as an obstacle");
    check(
      !sloped_grid.obstacle_grid.isOccupiedWorld(1.01, -1.01),
      "isolated obstacle return was not filtered");
    check(
      sloped_grid.obstacle_grid.isOccupiedWorld(3.03, -1.01),
      "supported obstacle return was filtered");
    check(
      !sloped_grid.obstacle_grid.isOccupiedWorld(2.03, 1.01),
      "return above the platform collision envelope became an obstacle");
    check(
      sloped_grid.observed_free_grid.isOccupiedWorld(1.0, 0.0) &&
      !sloped_grid.unknown_grid.isOccupiedWorld(1.0, 0.0),
      "observed swept path was not marked known free");
    check(
      sloped_grid.unknown_grid.isOccupiedWorld(1.0, 1.5),
      "completed point map manufactured free-space away from the swept path");
    check(
      sloped_grid.has_multi_scan_observation_support,
      "per-voxel scan observation support was not retained");

    lmmg::TraversabilityConfig temporal_config = sloped_config;
    temporal_config.minimum_obstacle_observations = 2U;
    const lmmg::TraversabilityGridResult temporal_grid = lmmg::buildTraversabilityGrid(
      sloped_points, sloped_trajectory, sloped_trajectory, temporal_config, small_robot);
    check(
      temporal_grid.obstacle_grid.isOccupiedWorld(3.03, -1.01),
      "multi-scan-supported obstacle was rejected by persistence filtering");

    // A rectangle is inflated according to path yaw and asymmetric base_link
    // extents.  An obstacle ahead is a collision while the same distance behind
    // can be clear; a width-only circle cannot represent this distinction.
    std::vector<lmmg::PointXYZI> rectangle_points = sloped_points;
    for (std::size_t sample = 0U; sample < 3U; ++sample) {
      rectangle_points.push_back({
        static_cast<float>(2.01 + 0.02 * static_cast<double>(sample)),
        0.01F, 0.40F, 40.0F});
    }
    lmmg::TraversabilityConfig rectangle_config = sloped_config;
    rectangle_config.observed_trajectory_clearance_radius = 0.0;
    lmmg::RobotConfig rectangle_robot;
    rectangle_robot.profile = "car";
    rectangle_robot.footprint_model = "rectangle";
    rectangle_robot.width = 0.40;
    rectangle_robot.front_extent = 1.00;
    rectangle_robot.rear_extent = 0.30;
    rectangle_robot.clearance_margin = 0.0;
    rectangle_robot.maximum_collision_height = 1.0;
    const lmmg::TraversabilityGridResult rectangle_grid = lmmg::buildTraversabilityGrid(
      rectangle_points, sloped_trajectory, sloped_trajectory,
      rectangle_config, rectangle_robot);
    check(rectangle_grid.orientation_aware_footprint, "rectangle footprint mode was not used");
    check(
      rectangle_grid.inflated_grid.isOccupiedWorld(1.10, 0.0),
      "front extent did not detect an obstacle ahead of base_link");
    check(
      !rectangle_grid.inflated_grid.isOccupiedWorld(2.60, 0.0),
      "rear extent was replaced by a symmetric/circular footprint");

    lmmg::GeneratorConfig config;
    config.trajectory.resample_interval = 0.20;
    config.trajectory.smoothing_window = 0.40;
    config.trajectory.maximum_speed_mps = 100.0;
    config.robot.width = 0.60;
    config.robot.clearance_margin = 0.10;
    config.traversability.grid_resolution = 0.10;
    config.traversability.trajectory_crop_radius = 3.0;
    config.traversability.ground_estimation_radius = 0.70;
    config.traversability.minimum_ground_points = 3U;
    config.traversability.maximum_corridor_half_width = 3.0;
    config.traversability.minimum_safe_center_width = 0.20;
    // This legacy corridor fixture has no scan-origin/free-space record.  It
    // intentionally exercises the migration mode; dedicated tests below cover
    // the default fail-closed UNKNOWN behavior.
    config.traversability.unknown_space_policy = "allow";
    config.topology.generate_reverse_edges = true;
    config.topology.maximum_edge_length = 20.0;

    const lmmg::MappingDataset dataset = syntheticDataset();
    const lmmg::PipelineResult result = lmmg::runVectorMapPipeline(dataset, config);
    const lmmg::RouteGraph & observed_graph = result.generation.observed_route_graph;
    check(observed_graph.frame_id == dataset.world_frame,
      "pipeline observed route frame does not match the dataset");
    check(!observed_graph.edges.empty(), "pipeline observed route graph has no edges");
    check(observed_graph.nodes.size() == observed_graph.edges.size() + 1U,
      "pipeline observed route graph is not a chronological chain");
    double observed_graph_length = 0.0;
    for (std::size_t index = 0U; index < observed_graph.edges.size(); ++index) {
      const lmmg::RouteEdge & edge = observed_graph.edges[index];
      check(edge.from == observed_graph.nodes[index].id &&
        edge.to == observed_graph.nodes[index + 1U].id,
        "pipeline observed route edge order is disconnected");
      check(edge.passable, "pipeline dropped an observed replay edge");
      observed_graph_length += edge.length;
    }
    std::vector<lmmg::Vec3> expected_body_route;
    expected_body_route.reserve(result.generation.processed_trajectory.size());
    check(result.grids.ground_z.size() == result.generation.processed_trajectory.size(),
      "pipeline ground samples do not match the processed trajectory");
    for (std::size_t index = 0U;
      index < result.generation.processed_trajectory.size(); ++index)
    {
      expected_body_route.push_back(
        result.generation.processed_trajectory[index].world_from_body.translation);
    }
    checkNear(observed_graph_length, lmmg::polylineLength(expected_body_route), 1.0e-8,
      "pipeline observed route does not cover the full processed base_link trajectory");
    check(lmmg::distance3d(
        observed_graph.nodes.front().position, expected_body_route.front()) < 1.0e-9,
      "pipeline observed route changed the first chronological visit");
    check(lmmg::distance3d(
        observed_graph.nodes.back().position, expected_body_route.back()) < 1.0e-9,
      "pipeline observed route changed the last chronological visit");
    checkNear(observed_graph.nodes.front().position.z, 0.30, 1.0e-12,
      "local floor evidence overwrote the observed base_link route Z");
    check(result.generation.observed_route_geometry_audit.valid,
      "observed route geometry audit failed");
    check(
      result.generation.observed_route_geometry_audit.source_pose_count ==
      result.generation.processed_trajectory.size() &&
      result.generation.observed_route_geometry_audit.source_segments_evaluated + 1U ==
      result.generation.processed_trajectory.size(),
      "observed geometry source segment count differs from processed poses minus one");
    check(lmmg::distance3d(
        result.generation.observed_route_geometry_audit.source_start,
        expected_body_route.front()) < 1.0e-12 && lmmg::distance3d(
        result.generation.observed_route_geometry_audit.source_end,
        expected_body_route.back()) < 1.0e-12,
      "observed geometry audit source endpoints changed");
    checkNear(
      result.generation.observed_route_geometry_audit.source_pose_projection_coverage,
      1.0, 1.0e-12, "observed replay did not retain every processed pose in order");
    checkNear(
      result.generation.observed_route_geometry_audit.planar_length_coverage,
      1.0, 1.0e-12, "observed replay changed the processed planar length");
    checkNear(
      result.generation.observed_route_geometry_audit.spatial_length_coverage,
      1.0, 1.0e-12, "observed replay changed the processed spatial length");
    check(result.generation.graph.nodes.size() >= 2U, "route graph has too few nodes");
    check(!result.generation.graph.edges.empty(), "route graph has no edges");
    check(result.generation.statistics.impassable_edges == 0U, "synthetic corridor is impassable");
    check(
      result.generation.statistics.minimum_safe_width > 2.5 &&
      result.generation.statistics.minimum_safe_width < 3.5,
      "synthetic corridor width is outside expected range");

    // An independently estimated floor may change abruptly at a support
    // boundary. It remains useful for obstacle height classification, but it
    // must never be copied into the driven base_link/Lanelet centerline.
    lmmg::MappingDataset stepped_floor_dataset = syntheticDataset();
    for (lmmg::PointXYZI & point : stepped_floor_dataset.map_points) {
      if (point.intensity < 2.0F && point.x >= 5.0F) {
        point.z = 0.20F;
      }
    }
    lmmg::GeneratorConfig stepped_floor_config = config;
    stepped_floor_config.traversability.ground_cell_resolution = 0.50;
    stepped_floor_config.traversability.minimum_ground_points = 3U;
    stepped_floor_config.traversability.minimum_ground_points_per_cell = 3U;
    stepped_floor_config.traversability.ground_plane_radius = 0.40;
    stepped_floor_config.traversability.maximum_ground_plane_residual = 0.01;
    const lmmg::PipelineResult stepped_floor_result = lmmg::runVectorMapPipeline(
      stepped_floor_dataset, stepped_floor_config);
    const auto ground_range = std::minmax_element(
      stepped_floor_result.grids.ground_z.begin(),
      stepped_floor_result.grids.ground_z.end());
    check(ground_range.first != stepped_floor_result.grids.ground_z.end() &&
      *ground_range.second - *ground_range.first > 0.15,
      "stepped-floor regression fixture did not produce distinct local ground evidence: min=" +
      std::to_string(*ground_range.first) + " max=" + std::to_string(*ground_range.second));
    double maximum_ground_jump = 0.0;
    for (std::size_t index = 1U;
      index < stepped_floor_result.grids.ground_z.size(); ++index)
    {
      maximum_ground_jump = std::max(
        maximum_ground_jump,
        std::abs(
          stepped_floor_result.grids.ground_z[index] -
          stepped_floor_result.grids.ground_z[index - 1U]));
    }
    check(maximum_ground_jump > 0.10,
      "stepped-floor regression fixture did not create a local ground jump");
    for (const lmmg::RouteEdge & edge :
      stepped_floor_result.generation.observed_route_graph.edges)
    {
      for (const lmmg::Vec3 & point : edge.centerline) {
        checkNear(point.z, 0.30, 1.0e-12,
          "local ground jump leaked into the observed base_link centerline");
      }
    }
    checkNear(
      stepped_floor_result.generation.observed_route_geometry_audit.maximum_absolute_delta_z,
      0.0, 1.0e-12,
      "local ground jump leaked into the centerline elevation audit");

    const std::filesystem::path output =
      std::filesystem::temp_directory_path() / "lidar_mobility_map_generator_test";
    std::filesystem::remove_all(output);
    std::filesystem::create_directories(output);
    lmmg::savePcdBinary(output / "map.pcd", dataset.map_points);
    const auto loaded = lmmg::loadPointCloudFile(output / "map.pcd");
    check(loaded.size() == dataset.map_points.size(), "binary PCD round trip failed");
    const std::filesystem::path staged_pcd = output / "staged_map.pcd";
    std::filesystem::create_hard_link(output / "map.pcd", staged_pcd);
    lmmg::savePcdBinary(output / "map.pcd", {{9.0, 8.0, 7.0, 6.0}});
    check(
      lmmg::loadPointCloudFile(staged_pcd).size() == dataset.map_points.size() &&
      lmmg::loadPointCloudFile(output / "map.pcd").size() == 1U,
      "atomic PCD replacement modified an existing staged hard link");
    lmmg::PointXYZI supported_point{1.0F, 2.0F, 3.0F, 4.0F};
    supported_point.observation_count = 7U;
    lmmg::savePcdBinary(output / "map_with_support.pcd", {supported_point});
    const auto loaded_supported = lmmg::loadPointCloudFile(output / "map_with_support.pcd");
    check(
      loaded_supported.size() == 1U && loaded_supported.front().observation_count == 7U,
      "PCD observation support round trip failed");
    lmmg::saveTrajectoryTum(output / "trajectory.tum", result.generation.processed_trajectory);
    std::vector<lmmg::TimedPose> precise_timestamps(2U);
    precise_timestamps[0U].stamp_ns = 1775656363343542337LL;
    precise_timestamps[1U].stamp_ns = 1775656363443486691LL;
    lmmg::saveTrajectoryTum(output / "trajectory_precise.tum", precise_timestamps);
    const std::vector<lmmg::TimedPose> loaded_precise_timestamps = lmmg::loadTumTrajectory(
      output / "trajectory_precise.tum");
    check(
      loaded_precise_timestamps.size() == precise_timestamps.size(),
      "TUM precise timestamp pose count changed");
    check(
      loaded_precise_timestamps[0U].stamp_ns == precise_timestamps[0U].stamp_ns &&
      loaded_precise_timestamps[1U].stamp_ns == precise_timestamps[1U].stamp_ns,
      "TUM nanosecond timestamp round trip failed");
    lmmg::saveRouteGraphGeoJson(output / "route_graph.geojson", result.generation.graph);
    lmmg::saveCorridorsGeoJson(output / "corridors.geojson", result.generation.graph);
    lmmg::saveLanelet2Osm(output / "map.osm", result.generation.graph, config.lanelet2);
    lmmg::saveLanelet2Osm(
      output / "map_autoware_ready.osm", result.generation.graph, config.lanelet2, true);
    lmmg::saveMapProjectorInfo(output / "map_projector_info.yaml");
    lmmg::saveRouteGraphMetadataYaml(output / "route_graph_metadata.yaml", result.generation.graph);
    lmmg::saveReviewGeometryTsv(output / "review_geometry.tsv", result.generation.graph);
    result.grids.obstacle_grid.savePgm(output / "obstacles.pgm");
    lmmg::saveOccupancyGridYaml(
      output / "obstacles.yaml", "obstacles.pgm", result.grids.obstacle_grid);

    std::ifstream projector_stream(output / "map_projector_info.yaml");
    const std::string projector_text{
      std::istreambuf_iterator<char>(projector_stream), std::istreambuf_iterator<char>()};
    check(
      projector_text == "projector_type: Local\n",
      "Autoware Local projector spelling/case is invalid");
    std::ifstream lanelet_stream(output / "map.osm");
    const std::string lanelet_text{
      std::istreambuf_iterator<char>(lanelet_stream), std::istreambuf_iterator<char>()};
    check(
      lanelet_text.find(
        "<MetaInfo format_version=\"1.1\" map_version=\"" LMMG_PROJECT_VERSION "\"/>") !=
      std::string::npos &&
      lanelet_text.find("k=\"location\" v=\"urban\"") != std::string::npos &&
      lanelet_text.find("k=\"one_way\" v=\"yes\"") != std::string::npos &&
      lanelet_text.find("k=\"autoware_ready\" v=\"no\"") != std::string::npos,
      "Lanelet2 candidate omitted required Autoware compatibility/readiness tags");
    std::ifstream ready_lanelet_stream(output / "map_autoware_ready.osm");
    const std::string ready_lanelet_text{
      std::istreambuf_iterator<char>(ready_lanelet_stream),
      std::istreambuf_iterator<char>()};
    check(
      ready_lanelet_text.find("k=\"autoware_ready\" v=\"yes\"") != std::string::npos &&
      ready_lanelet_text.find(
        "k=\"boundary_model\" v=\"independently_verified_physical_boundary\"") !=
      std::string::npos,
      "verified Lanelet2 output was not marked distinctly from the candidate");

    lmmg::OccupancyGrid2D nav2_obstacles(1.0, 2.0, 0.5, 2U, 2U);
    lmmg::OccupancyGrid2D nav2_free(1.0, 2.0, 0.5, 2U, 2U);
    lmmg::OccupancyGrid2D nav2_unknown(1.0, 2.0, 0.5, 2U, 2U);
    lmmg::OccupancyGrid2D nav2_roundoff_geometry(
      1.0 + 0.5e-9, 2.0 - 0.5e-9, 0.5 + 0.5e-12, 2U, 2U);
    lmmg::OccupancyGrid2D nav2_mismatched_geometry(
      1.0 + 2.0e-9, 2.0, 0.5, 2U, 2U);
    lmmg::OccupancyGrid2D nav2_mismatched_resolution(
      1.0, 2.0, 0.5 + 2.0e-12, 2U, 2U);
    lmmg::OccupancyGrid2D nav2_mismatched_dimensions(
      1.0, 2.0, 0.5, 3U, 2U);
    check(
      lmmg::hasMatchingGridGeometry(nav2_obstacles, nav2_free) &&
      lmmg::hasMatchingGridGeometry(nav2_obstacles, nav2_roundoff_geometry),
      "Nav2 grid geometry rejected exact or round-off-equivalent grids");
    check(
      !lmmg::hasMatchingGridGeometry(nav2_obstacles, nav2_mismatched_geometry) &&
      !lmmg::hasMatchingGridGeometry(nav2_obstacles, nav2_mismatched_resolution) &&
      !lmmg::hasMatchingGridGeometry(nav2_obstacles, nav2_mismatched_dimensions),
      "Nav2 grid geometry accepted an origin, resolution, or size mismatch");
    nav2_obstacles.setOccupied(0, 0);
    nav2_free.setOccupied(1, 0);
    nav2_unknown.setOccupied(0, 1);
    lmmg::saveNav2TrinaryPgm(
      output / "nav2_test.pgm", nav2_obstacles, nav2_free, nav2_unknown, false);
    lmmg::saveNav2TrinaryPgm(
      output / "nav2_roundoff_test.pgm", nav2_obstacles,
      nav2_roundoff_geometry, nav2_unknown, false);
    bool mismatched_grid_rejected = false;
    try {
      lmmg::saveNav2TrinaryPgm(
        output / "nav2_mismatched_test.pgm", nav2_obstacles,
        nav2_mismatched_geometry, nav2_unknown, false);
    } catch (const std::invalid_argument &) {
      mismatched_grid_rejected = true;
    }
    check(mismatched_grid_rejected, "Nav2 PGM writer accepted mismatched grid geometry");
    bool mismatched_resolution_rejected = false;
    try {
      lmmg::saveNav2TrinaryPgm(
        output / "nav2_mismatched_resolution_test.pgm", nav2_obstacles,
        nav2_mismatched_resolution, nav2_unknown, false);
    } catch (const std::invalid_argument &) {
      mismatched_resolution_rejected = true;
    }
    check(
      mismatched_resolution_rejected,
      "Nav2 PGM writer accepted a resolution mismatch");
    bool mismatched_dimensions_rejected = false;
    try {
      lmmg::saveNav2TrinaryPgm(
        output / "nav2_mismatched_dimensions_test.pgm", nav2_obstacles,
        nav2_mismatched_dimensions, nav2_unknown, false);
    } catch (const std::invalid_argument &) {
      mismatched_dimensions_rejected = true;
    }
    check(
      mismatched_dimensions_rejected,
      "Nav2 PGM writer accepted a width/height mismatch");
    std::ifstream nav2_stream(output / "nav2_test.pgm", std::ios::binary);
    const std::string nav2_bytes{
      std::istreambuf_iterator<char>(nav2_stream), std::istreambuf_iterator<char>()};
    const std::string nav2_header = "P5\n2 2\n255\n";
    check(nav2_bytes.rfind(nav2_header, 0U) == 0U, "Nav2 PGM header is invalid");
    check(nav2_bytes.size() == nav2_header.size() + 4U, "Nav2 PGM pixel count changed");
    const auto pixel = [&](const std::size_t index) {
        return static_cast<unsigned int>(
          static_cast<unsigned char>(nav2_bytes[nav2_header.size() + index]));
      };
    check(
      pixel(0U) == 205U && pixel(1U) == 205U &&
      pixel(2U) == 0U && pixel(3U) == 254U,
      "Nav2 trinary obstacle/unknown/free encoding is invalid");
    lmmg::saveOccupancyGridYaml(
      output / "nav2_test.yaml", "nav2_test.pgm", nav2_obstacles);
    const lmmg::LoadedOccupancyGrid reviewed_nav2_grid =
      lmmg::loadOccupancyGridYaml(output / "nav2_test.yaml");
    check(
      reviewed_nav2_grid.occupancy_values ==
      std::vector<std::int8_t>({100, 0, -1, -1}),
      "occupancy review loader did not preserve Nav2 occupied/free/unknown values");
    check(
      reviewed_nav2_grid.grid.occupiedCellCount() == 1U,
      "occupancy review binary obstacle view changed while preserving UNKNOWN");
    lmmg::saveNav2TrinaryPgm(
      output / "nav2_fail_closed.pgm", nav2_obstacles, nav2_free, nav2_unknown, true);
    std::ifstream fail_closed_stream(output / "nav2_fail_closed.pgm", std::ios::binary);
    const std::string fail_closed_bytes{
      std::istreambuf_iterator<char>(fail_closed_stream),
      std::istreambuf_iterator<char>()};
    check(
      fail_closed_bytes.size() == nav2_header.size() + 4U &&
      std::all_of(
        fail_closed_bytes.begin() + static_cast<std::ptrdiff_t>(nav2_header.size()),
        fail_closed_bytes.end(),
        [](const char value) {return static_cast<unsigned char>(value) == 205U;}),
      "fail-closed Nav2 map exposed a known/free cell");

    lmmg::saveNav2RouteGraphGeoJson(
      output / "nav2_route_graph.geojson", result.generation.graph, 0.05, 0.50);
    std::ifstream nav2_route_stream(output / "nav2_route_graph.geojson");
    const std::string nav2_route_text{
      std::istreambuf_iterator<char>(nav2_route_stream),
      std::istreambuf_iterator<char>()};
    check(
      nav2_route_text.find("\"source_route_edge_id\"") != std::string::npos &&
      nav2_route_text.find("\"startid\"") != std::string::npos,
      "Nav2 route graph was not converted to endpoint-node segments");

    const lmmg::RouteGraph reviewed_graph = lmmg::loadReviewGeometryTsv(
      output / "review_geometry.tsv");
    check(reviewed_graph.frame_id == result.generation.graph.frame_id,
      "review frame round trip failed");
    check(reviewed_graph.nodes.size() == result.generation.graph.nodes.size(),
      "review node round trip failed");
    check(reviewed_graph.edges.size() == result.generation.graph.edges.size(),
      "review edge round trip failed");
    check(
      reviewed_graph.edges.front().centerline.size() ==
      result.generation.graph.edges.front().centerline.size(),
      "review geometry sample round trip failed");
    checkNear(
      reviewed_graph.edges.front().minimum_safe_width,
      result.generation.graph.edges.front().minimum_safe_width,
      1.0e-9,
      "review minimum width round trip");
    check(
      reviewed_graph.edges.front().corridor_geometry_valid ==
      result.generation.graph.edges.front().corridor_geometry_valid,
      "review corridor geometry validity round trip failed");
    check(
      reviewed_graph.edges.front().left_clearance_observed ==
      result.generation.graph.edges.front().left_clearance_observed &&
      reviewed_graph.edges.front().right_clearance_observed ==
      result.generation.graph.edges.front().right_clearance_observed,
      "review clearance observation masks round trip failed");
    check(
      reviewed_graph.edges.front().validation_errors ==
      result.generation.graph.edges.front().validation_errors,
      "review validation diagnostics round trip failed");

    const lmmg::LoadedOccupancyGrid reviewed_grid = lmmg::loadOccupancyGridYaml(
      output / "obstacles.yaml");
    check(
      reviewed_grid.grid.occupiedCellCount() == result.grids.obstacle_grid.occupiedCellCount(),
      "occupancy review round trip failed");

    const std::vector<lmmg::Lanelet2ReviewLanelet> reviewed_lanelets =
      lmmg::loadGeneratedLanelet2Osm(output / "map.osm");
    check(!reviewed_lanelets.empty(), "Lanelet2 review parser returned no lanelets");
    check(
      reviewed_lanelets.front().left_boundary.size() >= 2U &&
      reviewed_lanelets.front().right_boundary.size() >= 2U,
      "Lanelet2 review boundaries are incomplete");

    lmmg::Lanelet2Config one_way_lanelet_config = config.lanelet2;
    one_way_lanelet_config.one_way = true;
    lmmg::saveLanelet2Osm(
      output / "map_one_way.osm", result.generation.graph, one_way_lanelet_config);
    const std::vector<lmmg::Lanelet2ReviewLanelet> one_way_lanelets =
      lmmg::loadGeneratedLanelet2Osm(output / "map_one_way.osm");
    const std::size_t passable_physical_edges = static_cast<std::size_t>(std::count_if(
      result.generation.graph.edges.begin(), result.generation.graph.edges.end(),
        [](const lmmg::RouteEdge & edge) {
          return edge.passable && (!edge.reverse_of || edge.id < *edge.reverse_of);
      }));
    check(
      one_way_lanelets.size() == passable_physical_edges,
      "one-way Lanelet export duplicated generated reverse edges");

    // A closed-course Autoware candidate is intentionally separate from the
    // production-gated map.  Feed it a hard-validated graph with two
    // components and degenerate source boundaries: the exporter must retain
    // all route components and construct non-degenerate, estimated-width
    // Lanelet chain without changing production readiness.
    lmmg::RouteGraph experimental_lanelet_graph;
    experimental_lanelet_graph.frame_id = "map";
    experimental_lanelet_graph.nodes = {
      {100U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
      {101U, {2.0, 0.0, 0.0}, lmmg::RouteNodeType::kNormal},
      {102U, {2.0, 2.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
      {103U, {0.0, 10.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
      {104U, {1.0, 10.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
    lmmg::RouteEdge experimental_first;
    experimental_first.id = 200U;
    experimental_first.from = 100U;
    experimental_first.to = 101U;
    experimental_first.centerline = {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    experimental_first.left_boundary = experimental_first.centerline;
    experimental_first.right_boundary = experimental_first.centerline;
    experimental_first.length = 2.0;
    experimental_first.passable = true;
    lmmg::RouteEdge experimental_second = experimental_first;
    experimental_second.id = 201U;
    experimental_second.from = 101U;
    experimental_second.to = 102U;
    experimental_second.centerline = {{2.0, 0.0, 0.0}, {2.0, 2.0, 0.0}};
    experimental_second.left_boundary = experimental_second.centerline;
    experimental_second.right_boundary = experimental_second.centerline;
    lmmg::RouteEdge experimental_disconnected = experimental_first;
    experimental_disconnected.id = 202U;
    experimental_disconnected.from = 103U;
    experimental_disconnected.to = 104U;
    experimental_disconnected.centerline = {{0.0, 10.0, 0.0}, {1.0, 10.0, 0.0}};
    experimental_disconnected.left_boundary = experimental_disconnected.centerline;
    experimental_disconnected.right_boundary = experimental_disconnected.centerline;
    experimental_disconnected.length = 1.0;
    experimental_lanelet_graph.edges = {
      experimental_first, experimental_second, experimental_disconnected};

    lmmg::ClosedCourseLanelet2ExportOptions experimental_options;
    experimental_options.estimated_vehicle_width = 1.80;
    experimental_options.estimated_front_extent = 1.0;
    experimental_options.estimated_rear_extent = 1.0;
    experimental_options.estimated_minimum_turning_radius = 4.8;
    experimental_options.lateral_clearance_margin = 0.15;
    experimental_options.vehicle_profile = "car";
    experimental_options.vehicle_base_reference = "rear_axle_ground_projection";
    experimental_options.vehicle_dimensions_evidence_source = "catalog_estimated";
    experimental_options.vehicle_dimensions_evidence_confidence = "medium";
    experimental_options.vehicle_dimensions_verified = false;
    experimental_options.experimental_ready = false;
    const lmmg::ClosedCourseLanelet2ExportSummary experimental_summary =
      lmmg::saveClosedCourseExperimentalLanelet2Osm(
      output / "map_closed_course_experimental.osm", experimental_lanelet_graph,
      one_way_lanelet_config, experimental_options);
    check(
      experimental_summary.source_physical_edges == 3U &&
      experimental_summary.exported_physical_edges == 3U,
      "closed-course Lanelet export silently dropped a disconnected component");
    checkNear(experimental_summary.source_length, 5.0, 1.0e-9,
      "closed-course source coverage length");
    checkNear(experimental_summary.exported_length, 5.0, 1.0e-9,
      "closed-course exported coverage length");
    const std::vector<lmmg::Lanelet2ReviewLanelet> experimental_lanelets =
      lmmg::loadGeneratedLanelet2Osm(output / "map_closed_course_experimental.osm");
    check(experimental_lanelets.size() == 3U,
      "closed-course Lanelet map dropped a disconnected component");
    check(
      experimental_lanelets[0U].route_edge_id == 200U &&
      experimental_lanelets[1U].route_edge_id == 201U,
      "closed-course Lanelet chain selected the wrong component");
    check(
      distance2d(
        experimental_lanelets[0U].left_boundary.back(),
        experimental_lanelets[1U].left_boundary.front()) <= 1.0e-12 &&
      distance2d(
        experimental_lanelets[0U].right_boundary.back(),
        experimental_lanelets[1U].right_boundary.front()) <= 1.0e-12 &&
      distance2d(
        experimental_lanelets[0U].centerline.back(),
        experimental_lanelets[1U].centerline.front()) <= 1.0e-12,
      "successive closed-course Lanelets do not share routing endpoints");
    for (const lmmg::Lanelet2ReviewLanelet & lanelet : experimental_lanelets) {
      check(
        lanelet.left_boundary.size() == lanelet.centerline.size() &&
        lanelet.right_boundary.size() == lanelet.centerline.size(),
        "closed-course Lanelet left/right/centerline structure is incomplete");
      for (std::size_t index = 0U; index < lanelet.centerline.size(); ++index) {
        check(
          distance2d(lanelet.left_boundary[index], lanelet.right_boundary[index]) >=
          experimental_options.estimated_vehicle_width +
          2.0 * experimental_options.lateral_clearance_margin - 1.0e-9,
          "closed-course estimated corridor is not wider than the vehicle plus margins");
      }
    }
    std::ifstream experimental_lanelet_stream(output / "map_closed_course_experimental.osm");
    const std::string experimental_lanelet_text{
      std::istreambuf_iterator<char>(experimental_lanelet_stream),
      std::istreambuf_iterator<char>()};
    check(
      countOccurrences(experimental_lanelet_text, "role=\"left\"") == 3U &&
      countOccurrences(experimental_lanelet_text, "role=\"right\"") == 3U &&
      countOccurrences(experimental_lanelet_text, "role=\"centerline\"") == 3U &&
      countOccurrences(experimental_lanelet_text, "<node id=\"") == 15U,
      "closed-course OSM members or shared endpoint node identities are invalid");
    check(
      experimental_lanelet_text.find("k=\"subtype\" v=\"road\"") != std::string::npos &&
      experimental_lanelet_text.find("k=\"location\" v=\"urban\"") != std::string::npos &&
      experimental_lanelet_text.find("k=\"one_way\" v=\"yes\"") != std::string::npos &&
      experimental_lanelet_text.find("k=\"participant:vehicle\" v=\"yes\"") !=
      std::string::npos &&
      experimental_lanelet_text.find("k=\"autoware_ready\" v=\"no\"") !=
      std::string::npos &&
      experimental_lanelet_text.find("k=\"production_ready\" v=\"no\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"provenance\" v=\"observed_driven_trajectory\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"validation_status\" v=\"observed_driven_replay_candidate\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"boundary_model\" "
        "v=\"trajectory_derived_estimated_drivable_corridor\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_vehicle_width_m\" v=\"1.8\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_front_extent_m\" v=\"1\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_rear_extent_m\" v=\"1\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"vehicle_minimum_turning_radius_m\" v=\"4.8\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_lateral_margin_m\" v=\"0.15\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_boundary_interpolation_guard_m\" v=\"0.05\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_longitudinal_endpoint_guard_m\" v=\"0.05\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_effective_lateral_margin_m\" v=\"0.2\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"estimated_boundary_algorithm\" "
        "v=\"oriented_rectangular_swept_envelope\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"vehicle_profile\" v=\"car\"") != std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"vehicle_base_reference\" v=\"rear_axle_ground_projection\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"vehicle_dimensions_source\" v=\"catalog_estimated\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"vehicle_dimensions_evidence_source\" v=\"catalog_estimated\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"vehicle_dimensions_evidence_confidence\" v=\"medium\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"vehicle_dimensions_verified\" v=\"no\"") !=
      std::string::npos &&
      experimental_lanelet_text.find(
        "k=\"exported_length_coverage\" v=\"1\"") !=
      std::string::npos,
      "closed-course OSM omitted Autoware routing or non-production provenance tags");

    // The full-map envelope is constructed across Edge transitions, but the
    // exported centreline remains the exact replay geometry.  In particular,
    // boundary construction must not RDP-simplify/resample a dense observed
    // chain or silently shorten/drop one of its source Edges.
    lmmg::RouteGraph exact_replay_graph;
    exact_replay_graph.frame_id = "map";
    const std::vector<double> exact_angles{
      0.0, 0.07, 0.18, 0.31, 0.46, 0.63, 0.78,
      0.92, 1.05, 1.19, 1.31, 1.43, lmmg::kPi * 0.5};
    std::vector<lmmg::Vec3> exact_replay_points;
    exact_replay_points.reserve(exact_angles.size());
    for (const double angle : exact_angles) {
      exact_replay_points.push_back({
        4.0 * std::sin(angle),
        20.0 + 4.0 * (1.0 - std::cos(angle)),
        0.02 * angle});
    }
    constexpr std::array<std::size_t, 4U> exact_split_indices{0U, 4U, 8U, 12U};
    for (std::size_t index = 0U; index < exact_split_indices.size(); ++index) {
      exact_replay_graph.nodes.push_back({
        300U + index, exact_replay_points[exact_split_indices[index]],
        index == 0U || index + 1U == exact_split_indices.size() ?
        lmmg::RouteNodeType::kEndpoint : lmmg::RouteNodeType::kNormal});
    }
    double exact_source_length = 0.0;
    for (std::size_t edge_index = 0U; edge_index + 1U < exact_split_indices.size();
      ++edge_index)
    {
      lmmg::RouteEdge edge;
      edge.id = 400U + edge_index;
      edge.from = 300U + edge_index;
      edge.to = 301U + edge_index;
      const std::size_t first = exact_split_indices[edge_index];
      const std::size_t last = exact_split_indices[edge_index + 1U];
      edge.centerline.assign(
        exact_replay_points.begin() + static_cast<std::ptrdiff_t>(first),
        exact_replay_points.begin() + static_cast<std::ptrdiff_t>(last + 1U));
      for (std::size_t point = 1U; point < edge.centerline.size(); ++point) {
        edge.length += lmmg::distance2d(
          edge.centerline[point - 1U], edge.centerline[point]);
      }
      exact_source_length += edge.length;
      edge.passable = true;
      exact_replay_graph.edges.push_back(std::move(edge));
    }
    const lmmg::ClosedCourseLanelet2ExportSummary exact_replay_summary =
      lmmg::saveClosedCourseExperimentalLanelet2Osm(
      output / "map_closed_course_exact_replay.osm", exact_replay_graph,
      one_way_lanelet_config, experimental_options);
    check(
      exact_replay_summary.source_physical_edges == exact_replay_graph.edges.size() &&
      exact_replay_summary.exported_physical_edges == exact_replay_graph.edges.size(),
      "full-map swept-envelope export dropped a replay Edge");
    checkNear(
      exact_replay_summary.source_length, exact_source_length, 1.0e-12,
      "full-map swept-envelope source geometry was shortened");
    checkNear(
      exact_replay_summary.exported_length, exact_source_length, 1.0e-12,
      "full-map swept-envelope exported geometry was shortened");
    check(
      exact_replay_summary.synthetic_planning_support.empty(),
      "default full-map export added unobserved synthetic planning support");
    std::ifstream exact_replay_stream(
      output / "map_closed_course_exact_replay.osm", std::ios::binary);
    const std::string exact_replay_osm{
      std::istreambuf_iterator<char>(exact_replay_stream),
      std::istreambuf_iterator<char>()};
    check(
      countOccurrences(
        exact_replay_osm,
        "k=\"synthetic_planning_support_count\" v=\"0\"") ==
      exact_replay_graph.edges.size() &&
      exact_replay_osm.find("k=\"synthetic_planning_support\" v=\"yes\"") ==
      std::string::npos &&
      exact_replay_osm.find("synthetic_test_kinematic_staging") == std::string::npos,
      "default full-map OSM serialized unobserved synthetic planning support");
    const std::vector<lmmg::Lanelet2ReviewLanelet> exact_replay_lanelets =
      lmmg::loadGeneratedLanelet2Osm(output / "map_closed_course_exact_replay.osm");
    check(
      exact_replay_lanelets.size() == exact_replay_graph.edges.size(),
      "full-map exact-replay OSM omitted a Lanelet");
    for (std::size_t edge_index = 0U; edge_index < exact_replay_graph.edges.size();
      ++edge_index)
    {
      const lmmg::RouteEdge & source_edge = exact_replay_graph.edges[edge_index];
      const lmmg::Lanelet2ReviewLanelet & lanelet = exact_replay_lanelets[edge_index];
      check(
        lanelet.route_edge_id == source_edge.id,
        "full-map exact-replay OSM changed Edge identity or serialized order");
      check(
        lanelet.centerline.size() == source_edge.centerline.size(),
        "full-map exact-replay OSM resampled a source Edge centreline");
      for (std::size_t point = 0U; point < source_edge.centerline.size(); ++point) {
        check(
          lmmg::distance3d(
            lanelet.centerline[point], source_edge.centerline[point]) <= 1.0e-9,
          "full-map exact-replay OSM changed a source centreline point");
      }
    }
    const lmmg::Lanelet2ReviewLanelet & exact_first_lanelet =
      exact_replay_lanelets.front();
    const lmmg::Vec2 exact_head_tangent = lmmg::normalized(lmmg::Vec2{
      exact_first_lanelet.centerline[1U].x - exact_first_lanelet.centerline[0U].x,
      exact_first_lanelet.centerline[1U].y - exact_first_lanelet.centerline[0U].y});
    const lmmg::Vec2 exact_head_cap_center{
      0.5 * (exact_first_lanelet.left_boundary.front().x +
      exact_first_lanelet.right_boundary.front().x),
      0.5 * (exact_first_lanelet.left_boundary.front().y +
      exact_first_lanelet.right_boundary.front().y)};
    const lmmg::Vec2 exact_head_extension{
      exact_head_cap_center.x - exact_first_lanelet.centerline.front().x,
      exact_head_cap_center.y - exact_first_lanelet.centerline.front().y};
    check(
      lmmg::dot(exact_head_extension, exact_head_tangent) < -0.04,
      "open full-map head lost its tagged longitudinal boundary guard");
    const lmmg::Lanelet2ReviewLanelet & exact_last_lanelet =
      exact_replay_lanelets.back();
    const std::size_t exact_last_point = exact_last_lanelet.centerline.size() - 1U;
    const lmmg::Vec2 exact_tail_tangent = lmmg::normalized(lmmg::Vec2{
      exact_last_lanelet.centerline[exact_last_point].x -
      exact_last_lanelet.centerline[exact_last_point - 1U].x,
      exact_last_lanelet.centerline[exact_last_point].y -
      exact_last_lanelet.centerline[exact_last_point - 1U].y});
    const lmmg::Vec2 exact_tail_cap_center{
      0.5 * (exact_last_lanelet.left_boundary.back().x +
      exact_last_lanelet.right_boundary.back().x),
      0.5 * (exact_last_lanelet.left_boundary.back().y +
      exact_last_lanelet.right_boundary.back().y)};
    const lmmg::Vec2 exact_tail_extension{
      exact_tail_cap_center.x - exact_last_lanelet.centerline.back().x,
      exact_tail_cap_center.y - exact_last_lanelet.centerline.back().y};
    check(
      lmmg::dot(exact_tail_extension, exact_tail_tangent) > 0.04,
      "open full-map tail lost its tagged longitudinal boundary guard");

    // Regression for a real Velodyne tail geometry. Its centimetre-scale yaw
    // jitter made a front-left corner 0.9 m before the end project beyond the
    // narrow terminal cross-section. The old cap logic ignored that corner
    // and cut 8 mm inside the configured swept footprint.
    lmmg::RouteGraph terminal_cap_graph;
    terminal_cap_graph.frame_id = "map";
    lmmg::RouteEdge terminal_cap_edge;
    terminal_cap_edge.id = 500U;
    terminal_cap_edge.from = 501U;
    terminal_cap_edge.to = 502U;
    terminal_cap_edge.centerline = {
      {24.6363813611, 10.5755704281, -1.15854924817},
      {24.6469824955, 10.7751112967, -1.15056291643},
      {24.6566931276, 10.9747378868, -1.14320097304},
      {24.6646330466, 11.1744696001, -1.13659296013},
      {24.6733780433, 11.3742149925, -1.1319536623},
      {24.6802314986, 11.5740688651, -1.12888397154},
      {24.6875276999, 11.7738648817, -1.12362702621},
      {24.6946745178, 11.9736984539, -1.12014773538},
      {24.6996206827, 12.1735813165, -1.11569248875},
      {24.7024666106, 12.3734272312, -1.1085033431},
      {24.7026998951, 12.5734006002, -1.10575905746},
      {24.7027395091, 12.77325129, -1.10127176861},
      {24.7036678333, 12.9729715377, -1.09096950268},
      {24.7035166426, 13.1728549785, -1.08427305023},
      {24.7080266028, 13.3727350501, -1.07944104932},
      {24.7070408125, 13.5725691692, -1.07150742863},
      {24.7025626298, 13.7723207484, -1.06264995874},
      {24.6972800834, 13.9721669364, -1.05770556027},
      {24.6912667881, 14.1720342035, -1.05378951705},
      {24.68530724, 14.3717235421, -1.04451074481},
      {24.6790134115, 14.5714178884, -1.03544504794},
      {24.6744480851, 14.7711264904, -1.02581262854},
      {24.6676312504, 14.9708696075, -1.01849864468},
      {24.6605469871, 15.1705009551, -1.00880869067},
      {24.6531143561, 15.3701329072, -0.999355546655},
      {24.6440169803, 15.5696533991, -0.989102760447},
      {24.6365829156, 15.769132064, -0.977512032476},
      {24.6300870177, 15.9688178254, -0.970014941453},
      {24.6224558533, 16.1684169188, -0.960550126626},
      {24.6132325999, 16.3680944189, -0.95418113339},
      {24.6020726079, 16.5675737122, -0.945345088901},
      {24.60339574, 16.7672013698, -0.938803132842},
      {24.5934913946, 16.9666950365, -0.928909216314},
      {24.5903876987, 17.1662979127, -0.918691139138},
      {24.5889836544, 17.2064057681, -0.919191375118}};
    terminal_cap_edge.passable = true;
    for (std::size_t index = 1U; index < terminal_cap_edge.centerline.size(); ++index) {
      terminal_cap_edge.length += lmmg::distance3d(
        terminal_cap_edge.centerline[index - 1U],
        terminal_cap_edge.centerline[index]);
    }
    terminal_cap_graph.nodes = {
      {terminal_cap_edge.from, terminal_cap_edge.centerline.front(),
        lmmg::RouteNodeType::kEndpoint},
      {terminal_cap_edge.to, terminal_cap_edge.centerline.back(),
        lmmg::RouteNodeType::kEndpoint}};
    terminal_cap_graph.edges = {terminal_cap_edge};
    lmmg::ClosedCourseLanelet2ExportOptions terminal_cap_options = experimental_options;
    terminal_cap_options.estimated_vehicle_width = 1.8;
    terminal_cap_options.estimated_front_extent = 3.2;
    terminal_cap_options.estimated_rear_extent = 1.0;
    terminal_cap_options.lateral_clearance_margin = 0.15;
    const std::filesystem::path terminal_cap_path =
      output / "map_closed_course_terminal_cap_regression.osm";
    static_cast<void>(lmmg::saveClosedCourseExperimentalLanelet2Osm(
        terminal_cap_path, terminal_cap_graph, one_way_lanelet_config,
        terminal_cap_options));
    const std::vector<lmmg::Lanelet2ReviewLanelet> terminal_cap_lanelets =
      lmmg::loadGeneratedLanelet2Osm(terminal_cap_path);
    check(terminal_cap_lanelets.size() == 1U,
      "terminal-cap regression did not export one Lanelet");
    std::vector<lmmg::Vec3> terminal_cap_polygon =
      terminal_cap_lanelets.front().left_boundary;
    terminal_cap_polygon.insert(
      terminal_cap_polygon.end(),
      terminal_cap_lanelets.front().right_boundary.rbegin(),
      terminal_cap_lanelets.front().right_boundary.rend());
    check(
      pointInPolygonInclusive(
        {23.41350583502375, 19.45293840157065, -0.9561270757762881},
        terminal_cap_polygon),
      "terminal cap cuts inside the configured curved-approach swept footprint");

    lmmg::SemanticMap semantic_map;
    semantic_map.frame_id = result.generation.graph.frame_id;

    lmmg::SemanticFeature stop;
    stop.id = 1U;
    stop.type = lmmg::SemanticFeatureType::kStop;
    stop.geometry = lmmg::SemanticGeometryType::kPoint;
    stop.name = "loading stop";
    stop.position = result.generation.graph.edges.front().centerline.front();
    stop.yaw = 0.0;
    stop.route_edge_ids = {result.generation.graph.edges.front().id};
    semantic_map.features.push_back(stop);

    lmmg::SemanticFeature speed;
    speed.id = 2U;
    speed.type = lmmg::SemanticFeatureType::kSpeedLimit;
    speed.geometry = lmmg::SemanticGeometryType::kRouteEdges;
    speed.name = "slow section";
    speed.value = 0.35;
    speed.route_edge_ids = {result.generation.graph.edges.front().id};
    semantic_map.features.push_back(speed);

    lmmg::SemanticFeature no_entry;
    no_entry.id = 3U;
    no_entry.type = lmmg::SemanticFeatureType::kNoEntry;
    no_entry.geometry = lmmg::SemanticGeometryType::kPolygon;
    no_entry.name = "blocked zone";
    no_entry.polygon = {
      {4.0, -1.0, 0.0}, {6.0, -1.0, 0.0}, {6.0, 1.0, 0.0}, {4.0, 1.0, 0.0}};
    semantic_map.features.push_back(no_entry);

    lmmg::saveSemanticMapTsv(output / "semantic_features.tsv", semantic_map);
    const lmmg::SemanticMap loaded_semantic = lmmg::loadSemanticMapTsv(
      output / "semantic_features.tsv", &result.generation.graph);
    check(loaded_semantic.features.size() == 3U, "semantic TSV round trip failed");
    check(loaded_semantic.features[0U].name == "loading stop", "semantic name round trip failed");
    lmmg::saveSemanticMapGeoJson(
      output / "semantic_features.geojson", loaded_semantic, result.generation.graph);
    lmmg::saveSemanticRouteRulesYaml(
      output / "semantic_route_rules.yaml", loaded_semantic, result.generation.graph);
    lmmg::saveSemanticRouteGraphGeoJson(
      output / "route_graph_semantic.geojson", loaded_semantic, result.generation.graph);
    const std::vector<lmmg::EdgeSemanticRule> rules = lmmg::deriveEdgeSemanticRules(
      loaded_semantic, result.generation.graph);
    check(!rules.empty(), "semantic route rules are empty");
    const auto first_rule = std::find_if(
      rules.begin(), rules.end(),
      [&](const lmmg::EdgeSemanticRule & rule) {
        return rule.edge_id == result.generation.graph.edges.front().id;
      });
    check(first_rule != rules.end(), "semantic rule for first edge is missing");
    checkNear(
      first_rule->effective_speed_limit_mps, 0.35, 1.0e-9,
      "semantic speed limit override failed");
    check(first_rule->no_entry, "polygon no-entry rule was not applied");
    check(!first_rule->effective_passable, "no-entry edge remained passable");
    check(lmmg::nextSemanticFeatureId(loaded_semantic) == 4U, "next semantic ID failed");

    lmmg::RouteGraph span_graph;
    span_graph.frame_id = "map";
    span_graph.nodes = {
      {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
      {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
    lmmg::RouteEdge span_forward;
    span_forward.id = 3U;
    span_forward.from = 1U;
    span_forward.to = 2U;
    span_forward.reverse_of = 4U;
    span_forward.centerline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    span_forward.length = 10.0;
    span_forward.passable = true;
    span_forward.recommended_speed_mps = 1.0;
    lmmg::RouteEdge span_reverse = span_forward;
    span_reverse.id = 4U;
    span_reverse.from = 2U;
    span_reverse.to = 1U;
    span_reverse.reverse_of = 3U;
    std::reverse(span_reverse.centerline.begin(), span_reverse.centerline.end());
    span_graph.edges = {span_forward, span_reverse};

    lmmg::SemanticMap span_map;
    span_map.frame_id = "map";
    lmmg::SemanticFeature partial_speed;
    partial_speed.id = 1U;
    partial_speed.type = lmmg::SemanticFeatureType::kSpeedLimit;
    partial_speed.geometry = lmmg::SemanticGeometryType::kRouteEdges;
    partial_speed.value = 0.25;
    partial_speed.route_edge_ids = {3U};
    lmmg::RouteEdgeSpan speed_span;
    speed_span.edge_id = 3U;
    speed_span.start_s = 2.0;
    speed_span.end_s = 5.0;
    speed_span.start_anchor = lmmg::Vec3{2.0, 0.0, 0.0};
    speed_span.end_anchor = lmmg::Vec3{5.0, 0.0, 0.0};
    partial_speed.route_edge_spans = {speed_span};
    span_map.features.push_back(partial_speed);
    lmmg::SemanticFeature partial_polygon;
    partial_polygon.id = 2U;
    partial_polygon.type = lmmg::SemanticFeatureType::kNoEntry;
    partial_polygon.geometry = lmmg::SemanticGeometryType::kPolygon;
    partial_polygon.polygon = {
      {6.0, -1.0, 0.0}, {8.0, -1.0, 0.0},
      {8.0, 1.0, 0.0}, {6.0, 1.0, 0.0}};
    span_map.features.push_back(partial_polygon);

    const lmmg::RouteEdgeSpan reverse_span = lmmg::reverseRouteEdgeSpan(speed_span, span_graph);
    check(reverse_span.edge_id == 4U, "reverse span edge ID is incorrect");
    checkNear(reverse_span.start_s, 5.0, 1.0e-9, "reverse span start");
    checkNear(reverse_span.end_s, 8.0, 1.0e-9, "reverse span end");

    const std::vector<lmmg::EdgeSemanticSegmentRule> span_rules =
      lmmg::deriveEdgeSemanticSegmentRules(span_map, span_graph);
    std::vector<lmmg::EdgeSemanticSegmentRule> forward_rules;
    std::copy_if(
      span_rules.begin(), span_rules.end(), std::back_inserter(forward_rules),
      [](const lmmg::EdgeSemanticSegmentRule & rule) {return rule.edge_id == 3U;});
    check(forward_rules.size() == 5U,
      "partial semantics did not split source edge at all boundaries");
    checkNear(forward_rules[1U].start_s, 2.0, 1.0e-9, "speed span split start");
    checkNear(forward_rules[1U].end_s, 5.0, 1.0e-9, "speed span split end");
    checkNear(
      forward_rules[1U].effective_speed_limit_mps, 0.25, 1.0e-9,
      "partial speed limit was not restricted to selected span");
    check(
      !forward_rules[0U].no_entry && !forward_rules[1U].no_entry &&
      !forward_rules[2U].no_entry && forward_rules[3U].no_entry &&
      !forward_rules[4U].no_entry,
      "polygon no-entry was not clipped to its interior edge interval");

    lmmg::saveSemanticMapTsv(output / "semantic_spans.tsv", span_map);
    const lmmg::SemanticMap loaded_spans = lmmg::loadSemanticMapTsv(
      output / "semantic_spans.tsv", &span_graph);
    check(
      loaded_spans.features.front().route_edge_spans.size() == 1U,
      "semantic span TSV round trip failed");
    checkNear(
      loaded_spans.features.front().route_edge_spans.front().start_s, 2.0, 1.0e-9,
      "semantic span start changed on round trip");
    check(
      loaded_spans.features.front().route_edge_spans.front().start_anchor.has_value(),
      "semantic span anchor was not preserved");

    lmmg::RouteGraph remapped_graph = span_graph;
    remapped_graph.edges[0U].id = 33U;
    remapped_graph.edges[0U].reverse_of = 34U;
    remapped_graph.edges[1U].id = 34U;
    remapped_graph.edges[1U].reverse_of = 33U;
    const lmmg::SemanticMap remapped_spans = lmmg::loadSemanticMapTsv(
      output / "semantic_spans.tsv", &remapped_graph);
    check(
      remapped_spans.features.front().route_edge_spans.front().edge_id == 33U,
      "semantic span was not geometry-remapped after edge ID change");
    lmmg::RouteGraph split_remap_graph;
    split_remap_graph.frame_id = "map";
    split_remap_graph.nodes = {
      {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
      {5U, {4.0, 0.0, 0.0}, lmmg::RouteNodeType::kNormal},
      {2U, {10.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
    lmmg::RouteEdge first_split = span_forward;
    first_split.id = 33U;
    first_split.to = 5U;
    first_split.reverse_of.reset();
    first_split.centerline = {{0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}};
    first_split.length = 4.0;
    lmmg::RouteEdge second_split = span_forward;
    second_split.id = 36U;
    second_split.from = 5U;
    second_split.reverse_of.reset();
    second_split.centerline = {{4.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    second_split.length = 6.0;
    split_remap_graph.edges = {first_split, second_split};
    const lmmg::SemanticMap split_remapped = lmmg::loadSemanticMapTsv(
      output / "semantic_spans.tsv", &split_remap_graph);
    const auto & split_spans = split_remapped.features.front().route_edge_spans;
    check(split_spans.size() == 2U, "semantic span was not divided across edited edge split");
    check(
      split_spans[0U].edge_id == 33U && split_spans[1U].edge_id == 36U,
      "split semantic span has incorrect edited edge IDs");
    checkNear(split_spans[0U].start_s, 2.0, 1.0e-9, "split remap first start");
    checkNear(split_spans[0U].end_s, 4.0, 1.0e-9, "split remap first end");
    checkNear(split_spans[1U].start_s, 0.0, 1.0e-9, "split remap second start");
    checkNear(split_spans[1U].end_s, 1.0, 1.0e-9, "split remap second end");
    lmmg::RouteGraph ambiguous_remap_graph = remapped_graph;
    lmmg::RouteEdge duplicate_forward = ambiguous_remap_graph.edges.front();
    duplicate_forward.id = 35U;
    duplicate_forward.reverse_of.reset();
    ambiguous_remap_graph.edges.push_back(duplicate_forward);
    bool ambiguous_remap_rejected = false;
    try {
      static_cast<void>(lmmg::loadSemanticMapTsv(
        output / "semantic_spans.tsv", &ambiguous_remap_graph));
    } catch (const std::exception &) {
      ambiguous_remap_rejected = true;
    }
    check(ambiguous_remap_rejected, "ambiguous semantic span geometry remap was not rejected");

    lmmg::saveSemanticRouteGraphGeoJson(
      output / "route_graph_semantic_spans.geojson", span_map, span_graph);
    std::ifstream derived_graph_stream(output / "route_graph_semantic_spans.geojson");
    const std::string derived_graph_text{
      std::istreambuf_iterator<char>(derived_graph_stream), std::istreambuf_iterator<char>()};
    check(
      derived_graph_text.find("\"generated_graph_unchanged\": true") != std::string::npos &&
      derived_graph_text.find("\"source_start_s\":2") != std::string::npos &&
      derived_graph_text.find("\"source_end_s\":5") != std::string::npos,
      "derived semantic graph is missing split provenance");

    {
      std::ofstream legacy_stream(output / "semantic_v1.tsv");
      legacy_stream << "LMMG_SEMANTICS\t1\nFRAME\tmap\n"
                    << "FEATURE\t1\tspeed_limit\troute_edges\t1\t0\t0\t0\t0\t0.4\t0"
                    << "\tlegacy\twhole edge\t3\n";
    }
    const lmmg::SemanticMap legacy_map = lmmg::loadSemanticMapTsv(
      output / "semantic_v1.tsv", &span_graph);
    check(
      legacy_map.features.size() == 1U &&
      legacy_map.features.front().route_edge_spans.empty() &&
      legacy_map.features.front().route_edge_ids == std::vector<std::uint64_t>{3U},
      "semantic version-1 whole-edge compatibility failed");

    {
      std::ofstream renamed_product_stream(output / "semantic_pcvm_v2.tsv");
      renamed_product_stream << "PCVM_SEMANTICS\t2\nFRAME\tmap\n"
                             << "FEATURE\t1\tspeed_limit\troute_edges\t1\t0\t0\t0\t0\t0.4\t0"
                             << "\tlegacy product header\twhole edge\t3\n";
    }
    const lmmg::SemanticMap renamed_product_map = lmmg::loadSemanticMapTsv(
      output / "semantic_pcvm_v2.tsv", &span_graph);
    check(
      renamed_product_map.features.size() == 1U &&
      renamed_product_map.features.front().route_edge_ids ==
      std::vector<std::uint64_t>{3U},
      "pre-rename PCVM semantic header compatibility failed");

    lmmg::RouteGraph route_edit_base;
    route_edit_base.frame_id = "map";
    route_edit_base.nodes = {
      {1U, {0.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint},
      {2U, {4.0, 0.0, 0.0}, lmmg::RouteNodeType::kEndpoint}};
    lmmg::RouteEdge route_edit_forward;
    route_edit_forward.id = 3U;
    route_edit_forward.from = 1U;
    route_edit_forward.to = 2U;
    route_edit_forward.reverse_of = 4U;
    route_edit_forward.centerline = {{0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}};
    route_edit_forward.length = 4.0;
    lmmg::RouteEdge route_edit_reverse = route_edit_forward;
    route_edit_reverse.id = 4U;
    route_edit_reverse.from = 2U;
    route_edit_reverse.to = 1U;
    route_edit_reverse.reverse_of = 3U;
    std::reverse(route_edit_reverse.centerline.begin(), route_edit_reverse.centerline.end());
    route_edit_base.edges = {route_edit_forward, route_edit_reverse};
    const std::string immutable_base_fingerprint = lmmg::routeGraphFingerprint(route_edit_base);

    lmmg::RouteEditSession clear_route_session(route_edit_base);
    clear_route_session.clearGraph();
    check(
      clear_route_session.editedGraph().graph.nodes.empty() &&
      clear_route_session.editedGraph().graph.edges.empty() &&
      clear_route_session.overlay().operations.size() == 1U &&
      clear_route_session.overlay().operations.front().type ==
      lmmg::RouteEditOperationType::kClearGraph,
      "clear graph was not stored as one undoable Route operation");
    lmmg::saveRouteEditOverlayTsv(
      output / "route_edits_clear.tsv", clear_route_session.overlay());
    lmmg::RouteEditOverlay clear_overlay = lmmg::loadRouteEditOverlayTsv(
      output / "route_edits_clear.tsv");
    check(
      lmmg::applyRouteEdits(route_edit_base, clear_overlay).graph.edges.empty(),
      "clear graph Route operation did not replay from TSV");
    clear_overlay.operations.pop_back();
    check(
      lmmg::routeGraphFingerprint(
        lmmg::applyRouteEdits(route_edit_base, clear_overlay).graph) ==
      immutable_base_fingerprint,
      "undoing clear graph did not restore the generated Route");

    lmmg::RouteEditSession route_edit_session(route_edit_base);
    const lmmg::SplitRouteEdgeResult split_edit = route_edit_session.splitEdge(3U, 2.0);
    check(
      split_edit.first_reverse_edge_id.has_value() &&
      split_edit.second_reverse_edge_id.has_value(),
      "bidirectional route split did not preserve reverse pairs");
    static_cast<void>(route_edit_session.setEdgeDirection(
      split_edit.first_edge_id, lmmg::RouteDirection::kOneWay));
    const std::uint64_t manual_node = route_edit_session.addNode({2.0, 2.0, 0.0});
    const auto manual_pair = route_edit_session.addEdge(
      split_edit.node_id, manual_node,
      {{2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {2.0, 2.0, 0.0}},
      lmmg::RouteDirection::kBidirectional);
    check(manual_pair.second.has_value(), "manual bidirectional edge has no reverse edge");
    route_edit_session.moveNode(manual_node, {2.0, 2.5, 0.0});
    bool connected_node_delete_rejected = false;
    try {
      route_edit_session.deleteNode(split_edit.node_id, false);
    } catch (const std::exception &) {
      connected_node_delete_rejected = true;
    }
    check(
      connected_node_delete_rejected,
      "non-cascade deletion accepted a node with connected edges");

    const lmmg::EditedRouteGraph route_edited = route_edit_session.editedGraph();
    check(
      lmmg::routeGraphFingerprint(route_edit_base) == immutable_base_fingerprint &&
      route_edit_base.nodes.size() == 2U && route_edit_base.edges.size() == 2U,
      "route edit session modified the generated graph");
    check(std::none_of(
      route_edited.graph.edges.begin(), route_edited.graph.edges.end(),
        [](const lmmg::RouteEdge & edge) {return edge.id == 3U || edge.id == 4U;}),
      "split route retained its source edge IDs");
    check(std::none_of(
      route_edited.graph.edges.begin(), route_edited.graph.edges.end(),
        [](const lmmg::RouteEdge & edge) {return edge.from == edge.to;}),
      "route edits created a self-loop edge");
    std::set<std::uint64_t> route_edit_ids;
    for (const lmmg::RouteNode & node : route_edited.graph.nodes) {
      check(route_edit_ids.insert(node.id).second, "edited route has a duplicate node ID");
    }
    for (const lmmg::RouteEdge & edge : route_edited.graph.edges) {
      check(route_edit_ids.insert(edge.id).second, "edited route has a duplicate edge/entity ID");
    }
    check(
      route_edited.edge_metadata.at(manual_pair.first).provenance ==
      lmmg::RouteProvenance::kManual &&
      route_edited.edge_metadata.at(split_edit.second_edge_id).provenance ==
      lmmg::RouteProvenance::kEditedGenerated,
      "route edit provenance is incorrect");

    lmmg::saveRouteEditOverlayTsv(
      output / "route_edits.tsv", route_edit_session.overlay());
    const lmmg::RouteEditOverlay loaded_route_edits_tsv = lmmg::loadRouteEditOverlayTsv(
      output / "route_edits.tsv");
    const lmmg::EditedRouteGraph replayed_tsv = lmmg::applyRouteEdits(
      route_edit_base, loaded_route_edits_tsv);
    check(
      lmmg::routeGraphFingerprint(replayed_tsv.graph) ==
      lmmg::routeGraphFingerprint(route_edited.graph),
      "route edit TSV did not replay deterministically");
    lmmg::saveRouteEditOverlayGeoJson(
      output / "route_edits.geojson", route_edit_session.overlay());
    const lmmg::RouteEditOverlay loaded_route_edits_geojson =
      lmmg::loadRouteEditOverlayGeoJson(output / "route_edits.geojson");
    const lmmg::EditedRouteGraph replayed_geojson = lmmg::applyRouteEdits(
      route_edit_base, loaded_route_edits_geojson);
    check(
      lmmg::routeGraphFingerprint(replayed_geojson.graph) ==
      lmmg::routeGraphFingerprint(route_edited.graph),
      "route edit GeoJSON did not replay deterministically");
    lmmg::saveEditedRouteGraphGeoJson(output / "route_graph_edited.geojson", route_edited);
    check(
      std::filesystem::file_size(output / "route_graph_edited.geojson") > 0U,
      "edited route review GeoJSON was not written");

    lmmg::RouteEditOverlay wrong_base_overlay = loaded_route_edits_tsv;
    wrong_base_overlay.base_graph_fingerprint = "0000000000000000";
    bool wrong_base_rejected = false;
    try {
      static_cast<void>(lmmg::applyRouteEdits(route_edit_base, wrong_base_overlay));
    } catch (const std::exception &) {
      wrong_base_rejected = true;
    }
    check(wrong_base_rejected, "route edits for a different generated graph were accepted");

    lmmg::RouteGraph route_validation_base;
    route_validation_base.frame_id = "map";
    route_validation_base.nodes = route_edit_base.nodes;
    route_validation_base.edges = {route_edit_forward};
    route_validation_base.edges.front().reverse_of.reset();
    lmmg::RouteEditSession validation_session(route_validation_base);
    const std::uint64_t curve_node = validation_session.addNode({4.2, 0.2, 0.0});
    const auto curved_edge = validation_session.addEdge(
      2U, curve_node,
      {{4.0, 0.0, 0.0}, {4.2, 0.0, 0.0}, {4.2, 0.2, 0.0}},
      lmmg::RouteDirection::kOneWay);
    const std::uint64_t short_node = validation_session.addNode({4.05, -0.05, 0.0});
    const auto short_edge = validation_session.addEdge(
      2U, short_node, {{4.0, 0.0, 0.0}, {4.05, -0.05, 0.0}},
      lmmg::RouteDirection::kOneWay);
    lmmg::OccupancyGrid2D edit_obstacles(-1.0, -1.0, 0.10, 71U, 31U);
    lmmg::OccupancyGrid2D edit_unknown(-1.0, -1.0, 0.10, 71U, 31U);
    lmmg::GeneratorConfig route_validation_config;
    route_validation_config.topology.minimum_edge_length = 0.10;
    route_validation_config.topology.maximum_edge_length = 10.0;
    route_validation_config.traversability.maximum_corridor_half_width = 0.30;
    route_validation_config.traversability.ray_step = 0.05;
    route_validation_config.robot.dimensions_verified = true;
    route_validation_config.robot.minimum_turning_radius = 4.5;
    route_validation_config.robot.allow_in_place_rotation = false;
    route_validation_config.robot.allow_reverse_motion = true;
    const lmmg::RouteValidationResult route_validation = lmmg::validateEditedRouteGraph(
      validation_session.editedGraph(), edit_obstacles, edit_unknown,
      route_validation_config);
    check(route_validation.operational_ready, "valid generated route was not operational");
    check(
      route_validation.edited.edge_metadata.at(3U).validation_status ==
      lmmg::RouteValidationStatus::kValid,
      "straight route failed vehicle/clearance validation");
    check(
      route_validation.edited.edge_metadata.at(curved_edge.first).validation_status ==
      lmmg::RouteValidationStatus::kInvalid &&
      std::find(
        route_validation.edited.edge_metadata.at(curved_edge.first).validation_errors.begin(),
        route_validation.edited.edge_metadata.at(curved_edge.first).validation_errors.end(),
        "minimum_turning_radius_violation") !=
      route_validation.edited.edge_metadata.at(curved_edge.first).validation_errors.end(),
      "curvature/vehicle constraint did not reject a sharp manual route");
    check(std::none_of(
      route_validation.operational_graph.edges.begin(),
      route_validation.operational_graph.edges.end(),
        [&](const lmmg::RouteEdge & edge) {return edge.id == curved_edge.first;}),
      "invalid manual route leaked into the operational graph");
    check(
      route_validation.edited.edge_metadata.at(short_edge.first).validation_status ==
      lmmg::RouteValidationStatus::kInvalid &&
      std::find(
        route_validation.edited.edge_metadata.at(short_edge.first).validation_errors.begin(),
        route_validation.edited.edge_metadata.at(short_edge.first).validation_errors.end(),
        "edge_below_minimum_length") !=
      route_validation.edited.edge_metadata.at(short_edge.first).validation_errors.end(),
      "minimum route length did not reject a short manual route");

    lmmg::GeneratorConfig allow_unknown_route_config = route_validation_config;
    allow_unknown_route_config.traversability.unknown_space_policy = "allow";
    const lmmg::RouteValidationResult allow_unknown_route_validation =
      lmmg::validateEditedRouteGraph(
      lmmg::RouteEditSession(route_validation_base).editedGraph(),
      edit_obstacles, edit_unknown, allow_unknown_route_config);
    check(
      !allow_unknown_route_validation.operational_ready &&
      allow_unknown_route_validation.operational_graph.edges.empty() &&
      std::find(
        allow_unknown_route_validation.edited.edge_metadata.at(3U).validation_errors.begin(),
        allow_unknown_route_validation.edited.edge_metadata.at(3U).validation_errors.end(),
        "unknown_space_policy_not_operational") !=
      allow_unknown_route_validation.edited.edge_metadata.at(3U).validation_errors.end(),
      "UNKNOWN allow migration policy leaked into the canonical route graph");

    lmmg::ApplicationConfig readiness_config;
    readiness_config.input_type = "synthetic";
    readiness_config.extrinsics.verified = true;
    readiness_config.extrinsics.calibration_source = "measured";
    readiness_config.extrinsics.calibration_confidence = "high";
    readiness_config.generator = route_validation_config;
    readiness_config.generator.robot.dimensions_source = "measured";
    readiness_config.generator.robot.dimensions_confidence = "high";
    readiness_config.generator.robot.base_reference = "rear_axle_ground_projection";
    readiness_config.generator.lanelet2.location = "urban";
    readiness_config.generator.lanelet2.one_way = true;
    readiness_config.output.frame_id = "map";
    // This block deliberately verifies both target-specific gates.  The public
    // product default is Vector Map, so the dual-target test must opt in.
    readiness_config.output.target_mode = "both";
    lmmg::PipelineResult readiness_pipeline;
    readiness_pipeline.grids.obstacle_grid = edit_obstacles;
    readiness_pipeline.grids.inflated_grid = edit_obstacles;
    readiness_pipeline.grids.observed_free_grid = lmmg::OccupancyGrid2D(
      -1.0, -1.0, 0.10, 71U, 31U);
    readiness_pipeline.grids.unknown_grid = lmmg::OccupancyGrid2D(
      -1.0, -1.0, 0.10, 71U, 31U);
    for (std::int64_t y = 0; y < 31; ++y) {
      for (std::int64_t x = 0; x < 71; ++x) {
        readiness_pipeline.grids.observed_free_grid.setOccupied(x, y);
      }
    }
    const lmmg::NavigationTargetReadiness gated_readiness =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, readiness_pipeline, route_validation, readiness_config);
    check(
      !gated_readiness.nav2_navigation_ready &&
      !gated_readiness.autoware_navigation_ready,
      "planner target became ready without explicit free-space/boundary verification");
    readiness_config.output.nav2_free_space_verified = true;
    readiness_config.output.lanelet2_physical_boundaries_verified = true;
    const lmmg::NavigationTargetReadiness verified_readiness =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, readiness_pipeline, route_validation, readiness_config);
    check(
      verified_readiness.nav2_navigation_ready &&
      verified_readiness.autoware_navigation_ready,
      "fully verified simple route did not pass target-specific readiness gates");
    lmmg::saveNavigationTargetReadinessYaml(
      output / "navigation_target_readiness.yaml", verified_readiness,
      dataset, readiness_pipeline, route_validation, readiness_config);
    std::ifstream readiness_stream(output / "navigation_target_readiness.yaml");
    const std::string readiness_text{
      std::istreambuf_iterator<char>(readiness_stream),
      std::istreambuf_iterator<char>()};
    check(
      readiness_text.find("nav2:\n  enabled: true\n  map_server_compatible: true") !=
      std::string::npos &&
      readiness_text.find(
        "autoware:\n  enabled: true\n  centerline_source: \"recorded_trajectory\"\n"
        "  map_loader_compatible: true") !=
      std::string::npos,
      "navigation target readiness report omitted compatibility results");

    lmmg::OccupancyGrid2D strip_unknown(-1.0, -1.0, 0.10, 71U, 31U);
    for (std::int64_t y = 0; y < 31; ++y) {
      for (std::int64_t x = 0; x < 71; ++x) {
        strip_unknown.setOccupied(x, y);
      }
    }
    for (std::int64_t y = 0; y < 31; ++y) {
      if (std::abs(strip_unknown.cellCenter(0, y).y) > 0.95) {
        continue;
      }
      for (std::int64_t x = 0; x < 71; ++x) {
        strip_unknown.setOccupied(x, y, false);
      }
    }
    lmmg::GeneratorConfig warning_route_config = route_validation_config;
    warning_route_config.traversability.minimum_safe_center_width = 0.40;
    warning_route_config.traversability.maximum_corridor_half_width = 0.80;
    const lmmg::RouteValidationResult warning_route_validation =
      lmmg::validateEditedRouteGraph(
      lmmg::RouteEditSession(route_validation_base).editedGraph(),
      edit_obstacles, strip_unknown, warning_route_config);
    check(
      warning_route_validation.operational_ready &&
      warning_route_validation.operational_graph.edges.size() == 1U &&
      warning_route_validation.edited.edge_metadata.at(3U).validation_status ==
      lmmg::RouteValidationStatus::kWarning,
      "adequate known-free width with UNKNOWN outer clearance was not operational with warning");
    check(
      warning_route_validation.edited.node_metadata.at(1U).validation_status ==
      lmmg::RouteValidationStatus::kWarning,
      "node incident to an advisory route was not retained as a warning");

    lmmg::OccupancyGrid2D narrow_free_unknown(-1.0, -1.0, 0.10, 71U, 31U);
    for (std::int64_t y = 0; y < 31; ++y) {
      for (std::int64_t x = 0; x < 71; ++x) {
        const bool outside_observed_strip =
          std::abs(narrow_free_unknown.cellCenter(x, y).y) > 0.40;
        narrow_free_unknown.setOccupied(x, y, outside_observed_strip);
      }
    }
    const lmmg::RouteValidationResult footprint_unknown_validation =
      lmmg::validateEditedRouteGraph(
      lmmg::RouteEditSession(route_validation_base).editedGraph(),
      edit_obstacles, narrow_free_unknown, route_validation_config);
    check(
      !footprint_unknown_validation.operational_ready &&
      footprint_unknown_validation.operational_graph.edges.empty(),
      "route whose robot footprint overlaps UNKNOWN leaked into the operational graph");

    lmmg::GeneratorConfig rectangle_route_config = route_validation_config;
    rectangle_route_config.robot.footprint_model = "rectangle";
    rectangle_route_config.robot.width = 0.20;
    rectangle_route_config.robot.clearance_margin = 0.0;
    rectangle_route_config.robot.front_extent = 0.20;
    rectangle_route_config.robot.rear_extent = 0.20;
    const lmmg::RouteValidationResult rectangle_route_validation =
      lmmg::validateEditedRouteGraph(
      validation_session.editedGraph(), edit_obstacles, edit_unknown,
      rectangle_route_config);
    check(
      rectangle_route_validation.edited.edge_metadata.at(3U).validation_status ==
      lmmg::RouteValidationStatus::kValid,
      "unchanged generated route failed rectangle-footprint route validation");
    check(
      rectangle_route_validation.edited.edge_metadata.at(curved_edge.first).validation_status ==
      lmmg::RouteValidationStatus::kInvalid &&
      std::find(
        rectangle_route_validation.edited.edge_metadata.at(curved_edge.first).
        validation_errors.begin(),
        rectangle_route_validation.edited.edge_metadata.at(curved_edge.first).
        validation_errors.end(),
        "route_orientation_collision_unvalidated") !=
      rectangle_route_validation.edited.edge_metadata.at(curved_edge.first).
      validation_errors.end(),
      "manual rectangle-footprint route was accepted with trajectory-yaw collision evidence");

    lmmg::RouteEditSession moved_rectangle_session(route_validation_base);
    moved_rectangle_session.moveNode(2U, {3.8, 0.0, 0.0});
    const lmmg::RouteValidationResult moved_rectangle_validation =
      lmmg::validateEditedRouteGraph(
      moved_rectangle_session.editedGraph(), edit_obstacles, edit_unknown,
      rectangle_route_config);
    check(
      moved_rectangle_validation.edited.edge_metadata.at(3U).validation_status ==
      lmmg::RouteValidationStatus::kInvalid &&
      std::find(
        moved_rectangle_validation.edited.edge_metadata.at(3U).validation_errors.begin(),
        moved_rectangle_validation.edited.edge_metadata.at(3U).validation_errors.end(),
        "route_orientation_collision_unvalidated") !=
      moved_rectangle_validation.edited.edge_metadata.at(3U).validation_errors.end(),
      "moved rectangle-footprint route did not fail closed");

    lmmg::saveRouteValidationReportYaml(
      output / "route_validation_report.yaml", warning_route_validation,
      warning_route_config);
    std::ifstream validation_report_stream(output / "route_validation_report.yaml");
    const std::string validation_report{
      std::istreambuf_iterator<char>(validation_report_stream),
      std::istreambuf_iterator<char>()};
    check(
      validation_report.find("navigation_ready: true") != std::string::npos &&
      validation_report.find("warning_edges: 1") != std::string::npos,
      "route validation report omitted navigation readiness or advisory counts");

    lmmg::OccupancyGrid2D edit_all_unknown(-1.0, -1.0, 0.10, 71U, 31U);
    for (std::int64_t y = 0; y < 31; ++y) {
      for (std::int64_t x = 0; x < 71; ++x) {
        edit_all_unknown.setOccupied(x, y);
      }
    }
    const lmmg::RouteValidationResult unknown_route_validation =
      lmmg::validateEditedRouteGraph(
      lmmg::RouteEditSession(route_validation_base).editedGraph(),
      edit_obstacles, edit_all_unknown, route_validation_config);
    check(
      !unknown_route_validation.operational_ready &&
      unknown_route_validation.operational_graph.edges.empty(),
      "UNKNOWN route leaked into the validated operational graph");

    lmmg::ApplicationConfig report_config;
    report_config.input_type = "synthetic";
    report_config.generator = config;
    report_config.extrinsics.source = "parameters";
    const lmmg::Transform requested_report_extrinsic{
      {0.77, 0.0, 1.694}, {0.0, 0.0, -0.00523596, 0.99998629}};
    report_config.extrinsics.requested_base_from_sensor = requested_report_extrinsic;
    report_config.extrinsics.base_from_sensor.translation = requested_report_extrinsic.translation;
    report_config.extrinsics.base_from_sensor.rotation =
      requested_report_extrinsic.rotation.normalized();
    // The report assertions below intentionally cover the dual-product label.
    report_config.output.target_mode = "both";
    lmmg::saveGenerationReport(
      output / "generation_report.yaml", dataset, result, report_config);
    std::ifstream generation_report_stream(output / "generation_report.yaml");
    const std::string generation_report(
      (std::istreambuf_iterator<char>(generation_report_stream)),
      std::istreambuf_iterator<char>());
    check(
      generation_report.find("schema_version: 6") != std::string::npos &&
      generation_report.find("output_map_type: \"both\"") != std::string::npos &&
      generation_report.find("output_target_mode:") == std::string::npos &&
      generation_report.find("raw_trajectory_preserved: true") != std::string::npos &&
      generation_report.find("corrected_position_jitter_poses:") != std::string::npos &&
      generation_report.find("maximum_planar_position_correction_m:") !=
      std::string::npos &&
      generation_report.find("trajectory_planar_length:") != std::string::npos &&
      generation_report.find("observed_driven_route_planar_length:") != std::string::npos &&
      generation_report.find("observed_driven_route_length_coverage: 1") != std::string::npos &&
      generation_report.find(
        "elevation_source: \"processed_base_link_trajectory\"") != std::string::npos &&
      generation_report.find("edge_partition_arc_metric: \"planar_xy\"") !=
      std::string::npos &&
      generation_report.find("source_pose_projection_coverage: 1") != std::string::npos &&
      generation_report.find("source_segments_evaluated:") != std::string::npos &&
      generation_report.find("source_start_xyz:") != std::string::npos &&
      generation_report.find("source_end_xyz:") != std::string::npos &&
      generation_report.find("maximum_absolute_delta_z_m:") != std::string::npos &&
      generation_report.find("maximum_absolute_grade:") != std::string::npos,
      "generation report omitted trajectory correction, route coverage, or elevation evidence");
    const std::string quaternion_prefix = "  quaternion_xyzw: [";
    const std::size_t quaternion_begin = generation_report.find(quaternion_prefix);
    const std::size_t quaternion_end = generation_report.find(']', quaternion_begin);
    check(
      quaternion_begin != std::string::npos && quaternion_end != std::string::npos,
      "generation report omitted parameter extrinsics quaternion");
    std::string quaternion_values = generation_report.substr(
      quaternion_begin + quaternion_prefix.size(),
      quaternion_end - quaternion_begin - quaternion_prefix.size());
    std::replace(quaternion_values.begin(), quaternion_values.end(), ',', ' ');
    std::istringstream quaternion_stream(quaternion_values);
    lmmg::Quaternion restored_quaternion;
    check(
      static_cast<bool>(quaternion_stream >> restored_quaternion.x >> restored_quaternion.y >>
      restored_quaternion.z >> restored_quaternion.w),
      "generation report parameter quaternion could not be parsed");
    const lmmg::Quaternion renormalized_quaternion = restored_quaternion.normalized();
    const lmmg::Quaternion & requested_quaternion =
      requested_report_extrinsic.rotation;
    const lmmg::Quaternion & expected_quaternion =
      report_config.extrinsics.base_from_sensor.rotation;
    check(
      restored_quaternion.x == requested_quaternion.x &&
      restored_quaternion.y == requested_quaternion.y &&
      restored_quaternion.z == requested_quaternion.z &&
      restored_quaternion.w == requested_quaternion.w &&
      renormalized_quaternion.x == expected_quaternion.x &&
      renormalized_quaternion.y == expected_quaternion.y &&
      renormalized_quaternion.z == expected_quaternion.z &&
      renormalized_quaternion.w == expected_quaternion.w,
      "generation report did not preserve the requested quaternion or its normalized transform");
    const std::string effective_quaternion_prefix = "  effective_quaternion_xyzw: [";
    const std::size_t effective_quaternion_begin =
      generation_report.find(effective_quaternion_prefix);
    const std::size_t effective_quaternion_end =
      generation_report.find(']', effective_quaternion_begin);
    check(
      effective_quaternion_begin != std::string::npos &&
      effective_quaternion_end != std::string::npos,
      "generation report omitted the effective normalized quaternion");
    std::string effective_quaternion_values = generation_report.substr(
      effective_quaternion_begin + effective_quaternion_prefix.size(),
      effective_quaternion_end - effective_quaternion_begin -
      effective_quaternion_prefix.size());
    std::replace(
      effective_quaternion_values.begin(), effective_quaternion_values.end(), ',', ' ');
    std::istringstream effective_quaternion_stream(effective_quaternion_values);
    lmmg::Quaternion reported_effective_quaternion;
    check(
      static_cast<bool>(
        effective_quaternion_stream >> reported_effective_quaternion.x >>
        reported_effective_quaternion.y >> reported_effective_quaternion.z >>
        reported_effective_quaternion.w),
      "generation report effective quaternion could not be parsed");
    check(
      reported_effective_quaternion.x == expected_quaternion.x &&
      reported_effective_quaternion.y == expected_quaternion.y &&
      reported_effective_quaternion.z == expected_quaternion.z &&
      reported_effective_quaternion.w == expected_quaternion.w,
      "generation report did not preserve the effective normalized quaternion");
    check(std::filesystem::file_size(output / "route_graph.geojson") > 0U,
      "GeoJSON was not written");
    check(std::filesystem::file_size(output / "map.osm") > 0U, "Lanelet2 OSM was not written");

    std::cout << "All core tests passed. nodes=" << result.generation.graph.nodes.size()
              << " edges=" << result.generation.graph.edges.size()
              << " minimum_safe_width=" << result.generation.statistics.minimum_safe_width
              << '\n';
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "Test failure: " << exception.what() << '\n';
    return 1;
  }
}
