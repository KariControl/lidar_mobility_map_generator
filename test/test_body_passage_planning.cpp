#include "lidar_mobility_map_generator/body_passage_planning.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

lmmg::RouteGraph straightRoute()
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {{1U, {-1.0, 0.0, 0.0}}, {2U, {1.0, 0.0, 0.0}}};
  lmmg::RouteEdge edge;
  edge.id = 10U;
  edge.from = 1U;
  edge.to = 2U;
  edge.centerline = {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  graph.edges.push_back(edge);
  return graph;
}

std::vector<lmmg::TimedPose> replayPoses(const lmmg::RouteGraph & graph)
{
  std::vector<lmmg::TimedPose> result;
  for (const lmmg::RouteEdge & edge : graph.edges) {
    for (const lmmg::Vec3 & point : edge.centerline) {
      if (!result.empty() && lmmg::distance3d(
          result.back().world_from_body.translation, point) <= 1.0e-12)
      {
        continue;
      }
      lmmg::TimedPose pose;
      pose.world_from_body.translation = point;
      pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(0.0);
      result.push_back(pose);
    }
  }
  return result;
}

lmmg::RouteGraph partitionedRoute(const std::vector<double> & second_edge_x)
{
  lmmg::RouteGraph graph;
  graph.frame_id = "map";
  graph.nodes = {
    {1U, {-1.0, 0.0, 0.0}}, {2U, {-0.5, 0.0, 0.0}}, {3U, {1.0, 0.0, 0.0}}};
  lmmg::RouteEdge first;
  first.id = 20U;
  first.from = 1U;
  first.to = 2U;
  first.centerline = {{-1.0, 0.0, 0.0}, {-0.5, 0.0, 0.0}};
  graph.edges.push_back(first);
  lmmg::RouteEdge second;
  second.id = 21U;
  second.from = 2U;
  second.to = 3U;
  for (const double x : second_edge_x) {
    second.centerline.push_back({x, 0.0, 0.0});
  }
  graph.edges.push_back(second);
  return graph;
}

std::vector<lmmg::TimedPose> threeReplayPoses()
{
  std::vector<lmmg::TimedPose> result(3U);
  for (std::size_t index = 0U; index < result.size(); ++index) {
    result[index].world_from_body.translation = {
      -1.0 + static_cast<double>(index), 0.0, 0.0};
    result[index].world_from_body.rotation =
      lmmg::Quaternion::fromYaw(0.4 * static_cast<double>(index));
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
  lmmg::OccupancyGrid2D obstacle(-5.0, -5.0, 0.1, 100U, 100U);
  lmmg::OccupancyGrid2D unknown(-5.0, -5.0, 0.1, 100U, 100U);
  lmmg::RobotConfig circle;
  circle.profile = "test_circle";
  circle.footprint_model = "circle";
  circle.width = 0.4;
  lmmg::RouteValidationResult clearance;
  clearance.edited.edge_metadata[10U].validation_errors = {
    "insufficient_clearance", "corridor_degenerate"};

  const lmmg::RouteGraph straight = straightRoute();
  const std::vector<lmmg::TimedPose> straight_poses = replayPoses(straight);
  const auto clear = lmmg::evaluateRouteBodyPassagePlanning(
    straight, straight_poses, obstacle, unknown, circle, clearance);
  bool passed = true;
  passed &= expect(clear.planning_body_passage_ready, "clear circle route must pass");
  passed &= expect(clear.valid_edges == 1U, "clear route must have one valid edge");
  passed &= expect(
    clear.additional_clearance_warning_edges.count("insufficient_clearance") == 1U,
    "legacy clearance failure must remain an advisory");

  const std::vector<lmmg::TimedPose> three_poses = threeReplayPoses();
  const lmmg::RouteGraph inserted_cut = partitionedRoute({-0.5, 0.0, 0.5, 1.0});
  const auto inserted_cut_report = lmmg::evaluateRouteBodyPassagePlanning(
    inserted_cut, three_poses, obstacle, unknown, circle, clearance);
  passed &= expect(
    inserted_cut_report.planning_body_passage_ready &&
    inserted_cut_report.edges.size() == 2U,
    "an ordered Edge cut inside a processed trajectory segment must be accepted");

  bool rejected_omitted_pose = false;
  try {
    const lmmg::RouteGraph omitted_pose = partitionedRoute({-0.5, 0.5, 1.0});
    static_cast<void>(lmmg::evaluateRouteBodyPassagePlanning(
      omitted_pose, three_poses, obstacle, unknown, circle, clearance));
  } catch (const std::invalid_argument &) {
    rejected_omitted_pose = true;
  }
  passed &= expect(
    rejected_omitted_pose,
    "a Route that shortcuts over a processed trajectory pose must be rejected");

  bool rejected_reordered_vertex = false;
  try {
    const lmmg::RouteGraph reordered = partitionedRoute({-0.5, 0.0, -0.25, 1.0});
    static_cast<void>(lmmg::evaluateRouteBodyPassagePlanning(
      reordered, three_poses, obstacle, unknown, circle, clearance));
  } catch (const std::invalid_argument &) {
    rejected_reordered_vertex = true;
  }
  passed &= expect(
    rejected_reordered_vertex,
    "a Route vertex reordered against the processed trajectory must be rejected");

  lmmg::RouteGraph empty;
  empty.frame_id = "map";
  const auto disabled = lmmg::evaluateRouteBodyPassagePlanning(
    empty, {}, obstacle, unknown, circle, clearance);
  passed &= expect(
    !disabled.planning_body_passage_ready && disabled.edges.empty(),
    "disabled Vector Map output must retain an empty non-ready body report");

  std::vector<lmmg::TimedPose> poses_with_unrepresented_sample = straight_poses;
  lmmg::TimedPose middle;
  middle.world_from_body.translation = {0.0, 0.0, 0.0};
  poses_with_unrepresented_sample.insert(
    poses_with_unrepresented_sample.begin() + 1, middle);
  bool rejected_unrepresented_pose = false;
  try {
    static_cast<void>(lmmg::evaluateRouteBodyPassagePlanning(
      straight, poses_with_unrepresented_sample, obstacle, unknown, circle, clearance));
  } catch (const std::invalid_argument &) {
    rejected_unrepresented_pose = true;
  }
  passed &= expect(
    rejected_unrepresented_pose,
    "body audit must reject a replay route that skips an acquisition-derived pose");

  const auto hit_cell = obstacle.worldToCell(0.5, 0.0);
  obstacle.setOccupied(hit_cell->first, hit_cell->second);
  const auto obstacle_hit = lmmg::evaluateRouteBodyPassagePlanning(
    straight, straight_poses, obstacle, unknown, circle, clearance);
  passed &= expect(!obstacle_hit.planning_body_passage_ready, "raw obstacle hit must block");
  passed &= expect(
    obstacle_hit.obstacle_overlap_cells > 0U &&
    obstacle_hit.edges.front().hard_errors.front() == "body_footprint_overlaps_obstacle",
    "raw obstacle hit must have machine-readable evidence");

  obstacle.setOccupied(hit_cell->first, hit_cell->second, false);
  unknown.setOccupied(hit_cell->first, hit_cell->second);
  lmmg::RobotConfig rectangle = circle;
  rectangle.profile = "test_rectangle";
  rectangle.footprint_model = "rectangle";
  rectangle.width = 0.4;
  rectangle.front_extent = 0.6;
  rectangle.rear_extent = 0.2;
  const auto unknown_hit = lmmg::evaluateRouteBodyPassagePlanning(
    straight, straight_poses, obstacle, unknown, rectangle, clearance);
  passed &= expect(!unknown_hit.planning_body_passage_ready, "raw UNKNOWN hit must block");
  passed &= expect(unknown_hit.unknown_overlap_cells > 0U, "UNKNOWN cells must be counted");

  unknown.setOccupied(hit_cell->first, hit_cell->second, false);
  std::vector<lmmg::TimedPose> sideways_poses = straight_poses;
  for (lmmg::TimedPose & pose : sideways_poses) {
    pose.world_from_body.rotation = lmmg::Quaternion::fromYaw(0.5 * lmmg::kPi);
  }
  const auto recorded_yaw_cell = unknown.worldToCell(0.0, 0.50);
  unknown.setOccupied(recorded_yaw_cell->first, recorded_yaw_cell->second);
  const auto recorded_yaw_hit = lmmg::evaluateRouteBodyPassagePlanning(
    straight, sideways_poses, obstacle, unknown, rectangle, clearance);
  passed &= expect(
    !recorded_yaw_hit.planning_body_passage_ready &&
    recorded_yaw_hit.unknown_overlap_cells > 0U,
    "body audit must use recorded body yaw rather than centerline tangent");
  unknown.setOccupied(recorded_yaw_cell->first, recorded_yaw_cell->second, false);

  lmmg::RouteGraph bend = straightRoute();
  bend.edges.front().centerline.clear();
  constexpr double pi = 3.14159265358979323846;
  for (int index = 0; index <= 20; ++index) {
    const double angle = -0.5 * pi + 0.5 * pi * static_cast<double>(index) / 20.0;
    bend.edges.front().centerline.push_back({std::cos(angle), 1.0 + std::sin(angle), 0.0});
  }
  rectangle.minimum_turning_radius = 5.0;
  rectangle.dimensions_verified = false;
  const std::vector<lmmg::TimedPose> bend_poses = replayPoses(bend);
  const auto inferred_turn = lmmg::evaluateRouteBodyPassagePlanning(
    bend, bend_poses, obstacle, unknown, rectangle, clearance);
  passed &= expect(
    inferred_turn.planning_body_passage_ready && inferred_turn.warning_edges == 1U &&
    inferred_turn.edges.front().minimum_turning_radius_violation,
    "unverified turning-radius violation must warn without blocking");
  rectangle.dimensions_verified = true;
  const auto verified_turn = lmmg::evaluateRouteBodyPassagePlanning(
    bend, bend_poses, obstacle, unknown, rectangle, clearance);
  passed &= expect(
    !verified_turn.planning_body_passage_ready && verified_turn.hard_invalid_edges == 1U,
    "verified turning-radius violation must block");

  return passed ? 0 : 1;
}
