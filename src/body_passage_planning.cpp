#include "lidar_mobility_map_generator/body_passage_planning.hpp"

#include "lidar_mobility_map_generator/route_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

constexpr double kMaximumPlanningSampleStepM = 0.10;
constexpr double kGeometryTolerance = 1.0e-12;

struct FootprintPose
{
  Vec2 base{};
  double yaw{0.0};
};

struct FootprintBounds
{
  double minimum_x{0.0};
  double minimum_y{0.0};
  double maximum_x{0.0};
  double maximum_y{0.0};
};

std::string jsonEscape(const std::string & input)
{
  std::ostringstream stream;
  for (const unsigned char value : input) {
    switch (value) {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\b': stream << "\\b"; break;
      case '\f': stream << "\\f"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (value < 0x20U) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned int>(value) << std::dec;
        } else {
          stream << static_cast<char>(value);
        }
    }
  }
  return stream.str();
}

void addUnique(std::vector<std::string> & values, const std::string & value)
{
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

double planarLength(const std::vector<Vec3> & points)
{
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result += distance2d(points[index - 1U], points[index]);
  }
  return result;
}

FootprintBounds footprintBounds(const FootprintPose & pose, const RobotConfig & robot)
{
  if (robot.footprint_model == "circle") {
    const double radius = 0.5 * robot.width;
    return {
      pose.base.x - radius, pose.base.y - radius,
      pose.base.x + radius, pose.base.y + radius};
  }

  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const double half_length = 0.5 * (robot.front_extent + robot.rear_extent);
  const double half_width = 0.5 * robot.width;
  const double center_offset = 0.5 * (robot.front_extent - robot.rear_extent);
  const Vec2 center{
    pose.base.x + center_offset * cosine,
    pose.base.y + center_offset * sine};
  const double extent_x = half_length * std::abs(cosine) + half_width * std::abs(sine);
  const double extent_y = half_length * std::abs(sine) + half_width * std::abs(cosine);
  return {
    center.x - extent_x, center.y - extent_y,
    center.x + extent_x, center.y + extent_y};
}

bool rectangleIntersectsCell(
  const FootprintPose & pose, const RobotConfig & robot,
  const Vec2 & cell_center, const double cell_half_width)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const Vec2 longitudinal{cosine, sine};
  const Vec2 lateral{-sine, cosine};
  const double half_length = 0.5 * (robot.front_extent + robot.rear_extent);
  const double half_width = 0.5 * robot.width;
  const double center_offset = 0.5 * (robot.front_extent - robot.rear_extent);
  const Vec2 rectangle_center{
    pose.base.x + center_offset * longitudinal.x,
    pose.base.y + center_offset * longitudinal.y};
  const Vec2 delta{
    cell_center.x - rectangle_center.x,
    cell_center.y - rectangle_center.y};

  // Separating-axis test for an oriented body rectangle and one axis-aligned
  // occupancy-grid cell. Boundary contact counts as an intersection.
  if (std::abs(dot(delta, longitudinal)) >
    half_length + cell_half_width *
    (std::abs(longitudinal.x) + std::abs(longitudinal.y)) + kGeometryTolerance)
  {
    return false;
  }
  if (std::abs(dot(delta, lateral)) >
    half_width + cell_half_width *
    (std::abs(lateral.x) + std::abs(lateral.y)) + kGeometryTolerance)
  {
    return false;
  }
  if (std::abs(delta.x) >
    cell_half_width + half_length * std::abs(longitudinal.x) +
    half_width * std::abs(lateral.x) + kGeometryTolerance)
  {
    return false;
  }
  if (std::abs(delta.y) >
    cell_half_width + half_length * std::abs(longitudinal.y) +
    half_width * std::abs(lateral.y) + kGeometryTolerance)
  {
    return false;
  }
  return true;
}

bool circleIntersectsCell(
  const FootprintPose & pose, const RobotConfig & robot,
  const Vec2 & cell_center, const double cell_half_width)
{
  const double minimum_x = cell_center.x - cell_half_width;
  const double maximum_x = cell_center.x + cell_half_width;
  const double minimum_y = cell_center.y - cell_half_width;
  const double maximum_y = cell_center.y + cell_half_width;
  const double nearest_x = std::max(minimum_x, std::min(pose.base.x, maximum_x));
  const double nearest_y = std::max(minimum_y, std::min(pose.base.y, maximum_y));
  const double dx = pose.base.x - nearest_x;
  const double dy = pose.base.y - nearest_y;
  const double radius = 0.5 * robot.width;
  return dx * dx + dy * dy <= radius * radius + kGeometryTolerance;
}

bool footprintIntersectsCell(
  const FootprintPose & pose, const RobotConfig & robot,
  const Vec2 & cell_center, const double cell_half_width)
{
  return robot.footprint_model == "circle" ?
         circleIntersectsCell(pose, robot, cell_center, cell_half_width) :
         rectangleIntersectsCell(pose, robot, cell_center, cell_half_width);
}

std::uint64_t cellKey(
  const std::int64_t x, const std::int64_t y, const std::size_t width)
{
  return static_cast<std::uint64_t>(y) * static_cast<std::uint64_t>(width) +
         static_cast<std::uint64_t>(x);
}

std::vector<FootprintPose> sampleEdge(
  const RouteEdge & edge, const std::vector<double> & vertex_yaws,
  const RobotConfig & robot, const double sample_step_m)
{
  std::vector<FootprintPose> samples;
  if (edge.centerline.size() < 2U || vertex_yaws.size() != edge.centerline.size()) {
    return samples;
  }
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec3 & start = edge.centerline[index - 1U];
    const Vec3 & end = edge.centerline[index];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::hypot(dx, dy);
    if (!(length > kGeometryTolerance)) {
      continue;
    }
    const double start_yaw = vertex_yaws[index - 1U];
    const double yaw_delta = normalizeAngle(vertex_yaws[index] - start_yaw);
    const double body_radius = robot.footprint_model == "circle" ? 0.0 :
      std::hypot(std::max(robot.front_extent, robot.rear_extent), 0.5 * robot.width);
    const std::size_t translation_pieces = static_cast<std::size_t>(
      std::ceil(length / sample_step_m));
    const std::size_t rotation_pieces = static_cast<std::size_t>(
      std::ceil(std::abs(yaw_delta) * body_radius / sample_step_m));
    const std::size_t pieces = std::max<std::size_t>(
      1U, std::max(translation_pieces, rotation_pieces));
    // Include both ends for every source segment. At a route vertex this
    // checks the recorded body yaw from both chronological pose intervals.
    for (std::size_t piece = 0U; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      samples.push_back(
        {{start.x + ratio * dx, start.y + ratio * dy},
          normalizeAngle(start_yaw + ratio * yaw_delta)});
    }
  }
  return samples;
}

std::vector<std::vector<double>> matchReplayBodyYaws(
  const RouteGraph & route, const std::vector<TimedPose> & poses)
{
  constexpr double kPositionToleranceM = 1.0e-6;
  constexpr double kEndpointToleranceM = 1.0e-12;
  constexpr double kParameterTolerance = 1.0e-12;
  if (poses.size() < 2U || route.edges.empty()) {
    throw std::invalid_argument(
            "body-passage planning requires replay body poses for every route vertex");
  }
  std::vector<std::vector<double>> result;
  result.reserve(route.edges.size());
  std::size_t source_index = 0U;
  double source_segment_ratio = 0.0;
  Vec3 previous_route_point{};
  double previous_yaw = 0.0;
  bool matched_any = false;
  std::uint64_t previous_to = 0U;

  const auto advance = [&](const Vec3 & point) {
      if (source_index + 1U >= poses.size()) {
        throw std::invalid_argument(
                "body-passage route extends beyond the replay body-pose polyline");
      }
      const Vec3 & source_start = poses[source_index].world_from_body.translation;
      const Vec3 & source_end = poses[source_index + 1U].world_from_body.translation;
      if (distance3d(point, source_end) <= kPositionToleranceM) {
        ++source_index;
        source_segment_ratio = 0.0;
        return poses[source_index].world_from_body.rotation.yaw();
      }

      const Vec3 source_delta = source_end - source_start;
      const double squared_length = normSquared(source_delta);
      if (!(squared_length > kGeometryTolerance * kGeometryTolerance)) {
        throw std::invalid_argument(
                "body-passage replay body poses contain a degenerate segment");
      }
      const double ratio = dot(point - source_start, source_delta) / squared_length;
      const Vec3 projection = source_start + source_delta * ratio;
      if (ratio <= source_segment_ratio + kParameterTolerance ||
        ratio >= 1.0 - kParameterTolerance ||
        distance3d(point, projection) > kPositionToleranceM)
      {
        throw std::invalid_argument(
                "body-passage route vertices do not monotonically preserve the replay "
                "body-pose polyline");
      }
      source_segment_ratio = ratio;
      const double start_yaw = poses[source_index].world_from_body.rotation.yaw();
      const double yaw_delta = normalizeAngle(
        poses[source_index + 1U].world_from_body.rotation.yaw() - start_yaw);
      return normalizeAngle(start_yaw + ratio * yaw_delta);
    };

  for (std::size_t edge_index = 0U; edge_index < route.edges.size(); ++edge_index) {
    const RouteEdge & edge = route.edges[edge_index];
    if (edge.centerline.size() < 2U) {
      throw std::invalid_argument(
              "body-passage replay Edge contains fewer than two centerline vertices");
    }
    if (edge_index > 0U && edge.from != previous_to) {
      throw std::invalid_argument(
              "body-passage replay Edges do not form one chronological chain");
    }
    std::vector<double> yaws;
    yaws.reserve(edge.centerline.size());
    for (std::size_t vertex_index = 0U; vertex_index < edge.centerline.size(); ++vertex_index) {
      const Vec3 & point = edge.centerline[vertex_index];
      if (!matched_any) {
        if (distance3d(point, poses.front().world_from_body.translation) >
          kEndpointToleranceM)
        {
          throw std::invalid_argument(
                  "body-passage replay Route does not start at the first body pose");
        }
        previous_yaw = poses.front().world_from_body.rotation.yaw();
        matched_any = true;
      } else if (edge_index > 0U && vertex_index == 0U) {
        if (distance3d(point, previous_route_point) > kEndpointToleranceM) {
          throw std::invalid_argument(
                  "body-passage replay Edges do not share an exact boundary vertex");
        }
        // An Edge partition point may lie inside a processed trajectory
        // segment. Reuse its already-projected yaw without advancing the
        // source cursor a second time.
      } else {
        previous_yaw = advance(point);
      }
      previous_route_point = point;
      yaws.push_back(previous_yaw);
    }
    previous_to = edge.to;
    result.push_back(std::move(yaws));
  }
  if (!matched_any || source_index + 1U != poses.size() ||
    distance3d(previous_route_point, poses.back().world_from_body.translation) >
    kEndpointToleranceM)
  {
    throw std::invalid_argument(
            "body-passage route does not completely cover every replay body-pose segment");
  }
  return result;
}

void writeStringArray(std::ostream & stream, const std::vector<std::string> & values)
{
  stream << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << '"' << jsonEscape(values[index]) << '"';
  }
  stream << ']';
}

void writeIdArray(std::ostream & stream, const std::vector<std::uint64_t> & values)
{
  stream << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << values[index];
  }
  stream << ']';
}

void writeCellEvidence(
  std::ostream & stream, const std::optional<BodyPassageCellEvidence> & evidence)
{
  if (!evidence) {
    stream << "null";
    return;
  }
  stream << "{\"cell_x\":" << evidence->x <<
    ",\"cell_y\":" << evidence->y <<
    ",\"center_x_m\":" << evidence->center.x <<
    ",\"center_y_m\":" << evidence->center.y << '}';
}

}  // namespace

RouteBodyPassagePlanningReport evaluateRouteBodyPassagePlanning(
  const RouteGraph & full_replay_route,
  const std::vector<TimedPose> & replay_body_poses,
  const OccupancyGrid2D & raw_obstacle_grid,
  const OccupancyGrid2D & raw_unknown_grid,
  const RobotConfig & robot,
  const RouteValidationResult & clearance_validation,
  const double sample_step_m)
{
  if (raw_obstacle_grid.empty() || raw_unknown_grid.empty() ||
    !hasMatchingGridGeometry(raw_obstacle_grid, raw_unknown_grid))
  {
    throw std::invalid_argument(
            "body-passage planning audit requires matching non-empty raw grids");
  }
  if (!std::isfinite(sample_step_m) || !(sample_step_m > 0.0) ||
    sample_step_m > kMaximumPlanningSampleStepM + kGeometryTolerance)
  {
    throw std::invalid_argument(
            "body-passage planning sample step must be in (0, 0.10] metres");
  }
  if (!std::isfinite(robot.width) || !(robot.width > 0.0) ||
    (robot.footprint_model != "circle" && robot.footprint_model != "rectangle"))
  {
    throw std::invalid_argument("body-passage planning footprint is invalid");
  }
  if (robot.footprint_model == "rectangle" &&
    (!std::isfinite(robot.front_extent) || !(robot.front_extent > 0.0) ||
    !std::isfinite(robot.rear_extent) || !(robot.rear_extent > 0.0)))
  {
    throw std::invalid_argument("body-passage planning rectangle extents are invalid");
  }

  RouteBodyPassagePlanningReport report;
  report.frame_id = full_replay_route.frame_id;
  report.route_graph_fingerprint = routeGraphFingerprint(full_replay_route);
  report.sample_step_m = sample_step_m;
  report.robot = robot;

  const std::set<std::string> additional_clearance_codes{
    "centerline_occupied",
    "insufficient_clearance",
    "unknown_clearance",
    "corridor_degenerate",
    "corridor_self_intersection"};
  for (const auto & [edge_id, metadata] : clearance_validation.edited.edge_metadata) {
    for (const std::string & error : metadata.validation_errors) {
      if (additional_clearance_codes.count(error) != 0U) {
        report.additional_clearance_warning_edges[error].push_back(edge_id);
      }
    }
  }

  std::set<std::uint64_t> all_obstacle_cells;
  std::set<std::uint64_t> all_unknown_cells;
  const double resolution = raw_obstacle_grid.resolution();
  const double half_cell = 0.5 * resolution;
  const double grid_minimum_x = raw_obstacle_grid.originX();
  const double grid_minimum_y = raw_obstacle_grid.originY();
  const double grid_maximum_x = grid_minimum_x +
    static_cast<double>(raw_obstacle_grid.width()) * resolution;
  const double grid_maximum_y = grid_minimum_y +
    static_cast<double>(raw_obstacle_grid.height()) * resolution;
  const std::vector<std::vector<double>> replay_body_yaws =
    full_replay_route.edges.empty() ? std::vector<std::vector<double>>() :
    matchReplayBodyYaws(full_replay_route, replay_body_poses);

  report.edges.reserve(full_replay_route.edges.size());
  for (std::size_t edge_index = 0U; edge_index < full_replay_route.edges.size(); ++edge_index) {
    const RouteEdge & edge = full_replay_route.edges[edge_index];
    BodyPassageEdgeEvidence evidence;
    evidence.edge_id = edge.id;
    evidence.planar_length_m = planarLength(edge.centerline);
    evidence.maximum_curvature_inv_m = maximumPolylineCurvature(edge.centerline);
    const std::vector<FootprintPose> samples = sampleEdge(
      edge, replay_body_yaws[edge_index], robot, sample_step_m);
    evidence.sample_count = samples.size();
    std::set<std::uint64_t> edge_obstacle_cells;
    std::set<std::uint64_t> edge_unknown_cells;

    if (samples.empty() || !(evidence.planar_length_m > kGeometryTolerance)) {
      addUnique(evidence.hard_errors, "route_edge_geometry_invalid");
    }
    for (const FootprintPose & pose : samples) {
      const FootprintBounds bounds = footprintBounds(pose, robot);
      const bool outside =
        bounds.minimum_x < grid_minimum_x - kGeometryTolerance ||
        bounds.minimum_y < grid_minimum_y - kGeometryTolerance ||
        bounds.maximum_x > grid_maximum_x + kGeometryTolerance ||
        bounds.maximum_y > grid_maximum_y + kGeometryTolerance;
      if (outside) {
        ++evidence.outside_grid_sample_count;
      }
      bool obstacle_overlap = false;
      bool unknown_overlap = outside;
      const std::int64_t minimum_x = static_cast<std::int64_t>(std::floor(
          (bounds.minimum_x - grid_minimum_x) / resolution)) - 1;
      const std::int64_t maximum_x = static_cast<std::int64_t>(std::floor(
          (bounds.maximum_x - grid_minimum_x) / resolution)) + 1;
      const std::int64_t minimum_y = static_cast<std::int64_t>(std::floor(
          (bounds.minimum_y - grid_minimum_y) / resolution)) - 1;
      const std::int64_t maximum_y = static_cast<std::int64_t>(std::floor(
          (bounds.maximum_y - grid_minimum_y) / resolution)) + 1;
      for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
        for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
          if (!raw_obstacle_grid.containsCell(x, y)) {
            continue;
          }
          const Vec2 center = raw_obstacle_grid.cellCenter(x, y);
          if (!footprintIntersectsCell(pose, robot, center, half_cell)) {
            continue;
          }
          const std::uint64_t key = cellKey(x, y, raw_obstacle_grid.width());
          if (raw_obstacle_grid.isOccupied(x, y)) {
            obstacle_overlap = true;
            edge_obstacle_cells.insert(key);
            all_obstacle_cells.insert(key);
            if (!evidence.first_obstacle_overlap) {
              evidence.first_obstacle_overlap = BodyPassageCellEvidence{x, y, center};
            }
          }
          if (raw_unknown_grid.isOccupied(x, y)) {
            unknown_overlap = true;
            edge_unknown_cells.insert(key);
            all_unknown_cells.insert(key);
            if (!evidence.first_unknown_overlap) {
              evidence.first_unknown_overlap = BodyPassageCellEvidence{x, y, center};
            }
          }
        }
      }
      if (obstacle_overlap) {
        ++evidence.obstacle_overlap_sample_count;
      }
      if (unknown_overlap) {
        ++evidence.unknown_overlap_sample_count;
      }
    }
    evidence.obstacle_overlap_cell_count = edge_obstacle_cells.size();
    evidence.unknown_overlap_cell_count = edge_unknown_cells.size();
    if (evidence.obstacle_overlap_sample_count > 0U) {
      addUnique(evidence.hard_errors, "body_footprint_overlaps_obstacle");
    }
    if (evidence.unknown_overlap_sample_count > 0U) {
      addUnique(evidence.hard_errors, "body_footprint_overlaps_unknown");
    }
    if (evidence.outside_grid_sample_count > 0U) {
      addUnique(evidence.hard_errors, "body_footprint_outside_grid");
    }

    if (robot.minimum_turning_radius > 0.0) {
      const double maximum_curvature = 1.0 / robot.minimum_turning_radius;
      evidence.minimum_turning_radius_violation =
        evidence.maximum_curvature_inv_m > maximum_curvature + 1.0e-9;
      if (evidence.minimum_turning_radius_violation) {
        if (robot.dimensions_verified) {
          addUnique(evidence.hard_errors, "minimum_turning_radius_violation");
        } else {
          addUnique(
            evidence.warnings,
            "unverified_minimum_turning_radius_violation");
        }
      }
    }
    evidence.hard_valid = evidence.hard_errors.empty();
    report.total_samples += evidence.sample_count;
    report.obstacle_overlap_samples += evidence.obstacle_overlap_sample_count;
    report.unknown_overlap_samples += evidence.unknown_overlap_sample_count;
    report.outside_grid_samples += evidence.outside_grid_sample_count;
    if (!evidence.hard_valid) {
      ++report.hard_invalid_edges;
    } else if (!evidence.warnings.empty()) {
      ++report.warning_edges;
    } else {
      ++report.valid_edges;
    }
    report.edges.push_back(std::move(evidence));
  }
  report.obstacle_overlap_cells = all_obstacle_cells.size();
  report.unknown_overlap_cells = all_unknown_cells.size();
  report.planning_body_passage_ready =
    !report.edges.empty() && report.hard_invalid_edges == 0U;
  return report;
}

void saveRouteBodyPassagePlanningReportJson(
  const std::filesystem::path & path,
  const RouteBodyPassagePlanningReport & report)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error(
            "failed to create body-passage planning report: " + path.string());
  }
  stream << std::setprecision(17)
         << "{\n"
         << "  \"schema_version\":1,\n"
         << "  \"kind\":\"route_body_passage_planning_evidence\",\n"
         << "  \"scope\":\"non_production_autoware_planning_test\",\n"
         << "  \"planning_body_passage_ready\":" <<
    (report.planning_body_passage_ready ? "true" : "false") << ",\n"
         << "  \"production_ready\":false,\n"
         << "  \"deployment_ready\":false,\n"
         << "  \"route_source\":\"route_graph_autoware_replay_candidate.geojson\",\n"
         << "  \"route_graph_fingerprint\":\"" <<
    jsonEscape(report.route_graph_fingerprint) << "\",\n"
         << "  \"frame_id\":\"" << jsonEscape(report.frame_id) << "\",\n"
         << "  \"grid_sources\":{\"obstacle\":\"obstacles.pgm\","
         << "\"unknown\":\"unknown.pgm\","
         << "\"state\":\"post_trajectory_clear_raw_masks\"},\n"
         << "  \"sampling\":{\"maximum_step_m\":" << report.sample_step_m << ','
         << "\"maximum_body_boundary_step_m\":" << report.sample_step_m << ','
         << "\"route_heading_policy\":\"recorded_body_yaw_shortest_angle_endpoint_both\","
         << "\"angular_sampling_policy\":\"maximum_corner_arc_step\","
         << "\"cell_intersection_policy\":\"exact_body_shape_vs_cell_square\"},\n"
         << "  \"vehicle\":{\"profile\":\"" << jsonEscape(report.robot.profile) << "\","
         << "\"base_reference\":\"" << jsonEscape(report.robot.base_reference) << "\","
         << "\"footprint_model\":\"" << jsonEscape(report.robot.footprint_model) << "\","
         << "\"width_m\":" << report.robot.width << ','
         << "\"front_extent_m\":" << report.robot.front_extent << ','
         << "\"rear_extent_m\":" << report.robot.rear_extent << ','
         << "\"clearance_m\":0,"
         << "\"minimum_turning_radius_m\":" << report.robot.minimum_turning_radius << ','
         << "\"dimensions_source\":\"" <<
    jsonEscape(report.robot.dimensions_source) << "\","
         << "\"dimensions_confidence\":\"" <<
    jsonEscape(report.robot.dimensions_confidence) << "\","
         << "\"dimensions_verified\":" <<
    (report.robot.dimensions_verified ? "true" : "false") << "},\n"
         << "  \"policy\":{"
         << "\"body_obstacle_overlap\":\"hard\","
         << "\"body_unknown_overlap\":\"hard\","
         << "\"additional_clearance\":\"warning\","
         << "\"turning_radius_when_dimensions_verified\":\"hard\","
         << "\"turning_radius_when_dimensions_unverified\":\"warning\"},\n"
         << "  \"counts\":{\"route_edges\":" << report.edges.size() << ','
         << "\"valid_edges\":" << report.valid_edges << ','
         << "\"warning_edges\":" << report.warning_edges << ','
         << "\"hard_invalid_edges\":" << report.hard_invalid_edges << ','
         << "\"samples\":" << report.total_samples << ','
         << "\"obstacle_overlap_samples\":" << report.obstacle_overlap_samples << ','
         << "\"obstacle_overlap_cells\":" << report.obstacle_overlap_cells << ','
         << "\"unknown_overlap_samples\":" << report.unknown_overlap_samples << ','
         << "\"unknown_overlap_cells\":" << report.unknown_overlap_cells << ','
         << "\"outside_grid_samples\":" << report.outside_grid_samples << "},\n"
         << "  \"additional_clearance_warnings\":{"
         << "\"source\":\"route_validation_closed_course_report.yaml\","
         << "\"effect_on_planning_test\":\"warning_only\","
         << "\"source_topology_edges_by_reason\":{";
  bool first_warning = true;
  for (const auto & [reason, edge_ids] : report.additional_clearance_warning_edges) {
    if (!first_warning) {
      stream << ',';
    }
    first_warning = false;
    stream << '"' << jsonEscape(reason) << "\":";
    writeIdArray(stream, edge_ids);
  }
  stream << "}},\n  \"edges\":[\n";
  for (std::size_t index = 0U; index < report.edges.size(); ++index) {
    const BodyPassageEdgeEvidence & edge = report.edges[index];
    if (index > 0U) {
      stream << ",\n";
    }
    const char * status = !edge.hard_valid ? "INVALID" :
      (!edge.warnings.empty() ? "WARNING" : "VALID");
    stream << "    {\"edge_id\":" << edge.edge_id <<
      ",\"status\":\"" << status << "\"" <<
      ",\"planar_length_m\":" << edge.planar_length_m <<
      ",\"sample_count\":" << edge.sample_count <<
      ",\"obstacle_overlap_sample_count\":" <<
      edge.obstacle_overlap_sample_count <<
      ",\"obstacle_overlap_cell_count\":" << edge.obstacle_overlap_cell_count <<
      ",\"unknown_overlap_sample_count\":" << edge.unknown_overlap_sample_count <<
      ",\"unknown_overlap_cell_count\":" << edge.unknown_overlap_cell_count <<
      ",\"outside_grid_sample_count\":" << edge.outside_grid_sample_count <<
      ",\"first_obstacle_overlap\":";
    writeCellEvidence(stream, edge.first_obstacle_overlap);
    stream << ",\"first_unknown_overlap\":";
    writeCellEvidence(stream, edge.first_unknown_overlap);
    stream << ",\"maximum_curvature_inv_m\":" << edge.maximum_curvature_inv_m <<
      ",\"minimum_turning_radius_violation\":" <<
      (edge.minimum_turning_radius_violation ? "true" : "false") <<
      ",\"hard_errors\":";
    writeStringArray(stream, edge.hard_errors);
    stream << ",\"warnings\":";
    writeStringArray(stream, edge.warnings);
    stream << '}';
  }
  stream << "\n  ]\n}\n";
}

}  // namespace lidar_mobility_map_generator
