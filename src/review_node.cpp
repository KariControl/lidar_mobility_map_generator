#include "lidar_mobility_map_generator/glim_reader.hpp"
#include "lidar_mobility_map_generator/pointcloud_io.hpp"
#include "lidar_mobility_map_generator/review_io.hpp"
#include "lidar_mobility_map_generator/semantic_map.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA color(
  const float red, const float green, const float blue, const float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA value;
  value.r = red;
  value.g = green;
  value.b = blue;
  value.a = alpha;
  return value;
}

geometry_msgs::msg::Point point(const lmmg::Vec3 & input, const double z_offset = 0.0)
{
  geometry_msgs::msg::Point output;
  output.x = input.x;
  output.y = input.y;
  output.z = input.z + z_offset;
  return output;
}

builtin_interfaces::msg::Time timeMessage(const std::int64_t stamp_ns)
{
  builtin_interfaces::msg::Time message;
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  std::int64_t seconds = stamp_ns / kNanosecondsPerSecond;
  std::int64_t nanoseconds = stamp_ns % kNanosecondsPerSecond;
  if (nanoseconds < 0) {
    --seconds;
    nanoseconds += kNanosecondsPerSecond;
  }
  message.sec = static_cast<std::int32_t>(seconds);
  message.nanosec = static_cast<std::uint32_t>(nanoseconds);
  return message;
}

Marker baseMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & marker_namespace,
  const std::int32_t id,
  const std::int32_t type)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

Marker deleteAllMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.action = Marker::DELETEALL;
  return marker;
}

lmmg::Vec3 polylineMidpoint(const std::vector<lmmg::Vec3> & points)
{
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1U) {
    return points.front();
  }
  const double total = lmmg::polylineLength(points);
  if (!(total > 1.0e-12)) {
    return points[points.size() / 2U];
  }
  const double target = 0.5 * total;
  double accumulated = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double segment = lmmg::distance3d(points[index - 1U], points[index]);
    if (accumulated + segment >= target && segment > 1.0e-12) {
      const double ratio = (target - accumulated) / segment;
      return points[index - 1U] + (points[index] - points[index - 1U]) * ratio;
    }
    accumulated += segment;
  }
  return points.back();
}

std::optional<std::pair<lmmg::Vec3, lmmg::Vec3>> middleDirectionSegment(
  const std::vector<lmmg::Vec3> & points)
{
  if (points.size() < 2U) {
    return std::nullopt;
  }
  const std::size_t middle = points.size() / 2U;
  for (std::size_t offset = 0U; offset < points.size(); ++offset) {
    const std::size_t candidate = middle + offset < points.size() ? middle + offset : middle;
    if (candidate > 0U && lmmg::distance3d(points[candidate - 1U], points[candidate]) > 1.0e-6) {
      return std::make_pair(points[candidate - 1U], points[candidate]);
    }
    if (middle >= offset + 1U) {
      const std::size_t reverse_candidate = middle - offset;
      if (reverse_candidate > 0U &&
        lmmg::distance3d(points[reverse_candidate - 1U], points[reverse_candidate]) > 1.0e-6)
      {
        return std::make_pair(points[reverse_candidate - 1U], points[reverse_candidate]);
      }
    }
  }
  return std::nullopt;
}

bool mirroredGeometryDuplicate(const lmmg::RouteEdge & edge)
{
  return edge.reverse_of && edge.id > *edge.reverse_of;
}

std::string edgeLabel(const lmmg::RouteEdge & edge)
{
  std::ostringstream stream;
  stream << "E" << edge.id << "  w=" << std::fixed << std::setprecision(2)
         << edge.minimum_safe_width << "m  c=" << edge.confidence;
  if (!edge.passable) {
    stream << "  BLOCKED";
    if (!edge.validation_errors.empty()) {
      stream << " (" << edge.validation_errors.front() << ')';
    }
  }
  return stream.str();
}

}  // namespace

class VectorMapReviewNode : public rclcpp::Node
{
public:
  VectorMapReviewNode()
  : Node("lidar_mobility_map_review")
  {
    output_directory_ = declare_parameter<std::string>("output_directory", "output");
    frame_override_ = declare_parameter<std::string>("frame_id", "");

    publish_pointcloud_ = declare_parameter<bool>("publish.pointcloud", true);
    publish_trajectories_ = declare_parameter<bool>("publish.trajectories", true);
    publish_route_graph_ = declare_parameter<bool>("publish.route_graph", true);
    publish_corridors_ = declare_parameter<bool>("publish.corridors", true);
    publish_lanelet2_ = declare_parameter<bool>("publish.lanelet2", true);
    publish_navigation_map_ = declare_parameter<bool>("publish.navigation_map", true);
    publish_semantics_ = declare_parameter<bool>("publish.semantics", true);
    publish_grids_ = declare_parameter<bool>("publish.occupancy_grids", true);
    publish_diagnostics_ = declare_parameter<bool>("publish.diagnostics", true);
    publish_labels_ = declare_parameter<bool>("publish.labels", true);
    lanelet2_source_ = declare_parameter<std::string>(
      "visualization.lanelet2_source", "auto");
    if (lanelet2_source_ != "auto" && lanelet2_source_ != "closed_course_experimental" &&
      lanelet2_source_ != "validated" && lanelet2_source_ != "generated" &&
      lanelet2_source_ != "canonical")
    {
      throw std::invalid_argument(
              "visualization.lanelet2_source must be auto, closed_course_experimental, "
              "validated, generated, or canonical");
    }
    navigation_map_source_ = declare_parameter<std::string>(
      "visualization.navigation_map_source", "auto");
    if (navigation_map_source_ != "auto" &&
      navigation_map_source_ != "closed_course_experimental" &&
      navigation_map_source_ != "generated" && navigation_map_source_ != "canonical")
    {
      throw std::invalid_argument(
              "visualization.navigation_map_source must be auto, "
              "closed_course_experimental, generated, or canonical");
    }
    semantic_auto_reload_period_sec_ = declare_parameter<double>(
      "semantic_auto_reload_period_sec", 1.0);

    pointcloud_stride_ = std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("visualization.pointcloud_stride", 1));
    pointcloud_max_points_ = std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("visualization.pointcloud_max_points", 2000000));
    z_offset_ = declare_parameter<double>("visualization.z_offset", 0.05);
    route_line_width_ = positiveParameter("visualization.route_line_width", 0.08);
    boundary_line_width_ = positiveParameter("visualization.boundary_line_width", 0.04);
    node_scale_ = positiveParameter("visualization.node_scale", 0.25);
    arrow_scale_ = positiveParameter("visualization.arrow_scale", 0.20);
    label_scale_ = positiveParameter("visualization.label_scale", 0.25);
    semantic_point_scale_ = positiveParameter("visualization.semantic_point_scale", 0.35);
    semantic_line_width_ = positiveParameter("visualization.semantic_line_width", 0.14);
    low_confidence_threshold_ = declare_parameter<double>(
      "visualization.low_confidence_threshold", 0.50);
    low_confidence_threshold_ = lmmg::clamp(low_confidence_threshold_, 0.0, 1.0);

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latched_qos.reliable().transient_local();
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/pointcloud_map", latched_qos);
    classified_obstacle_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/obstacle_points_classified", latched_qos);
    raw_trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "~/trajectory_raw", latched_qos);
    processed_trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "~/trajectory_processed", latched_qos);
    route_graph_publisher_ = create_publisher<MarkerArray>("~/route_graph", latched_qos);
    corridor_publisher_ = create_publisher<MarkerArray>("~/corridors", latched_qos);
    lanelet2_publisher_ = create_publisher<MarkerArray>("~/lanelet2", latched_qos);
    issue_publisher_ = create_publisher<MarkerArray>("~/issues", latched_qos);
    semantic_publisher_ = create_publisher<MarkerArray>("~/semantic_features", latched_qos);
    navigation_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/navigation_map", latched_qos);
    obstacle_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/obstacles", latched_qos);
    inflated_obstacle_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/obstacles_inflated", latched_qos);
    observed_free_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/observed_free", latched_qos);
    unknown_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/unknown", latched_qos);

    reload_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reload",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        try {
          const std::size_t count = loadAndPublish();
          response->success = count > 0U;
          response->message = "published " + std::to_string(count) + " review layers";
        } catch (const std::exception & exception) {
          response->success = false;
          response->message = exception.what();
          RCLCPP_ERROR(get_logger(), "Review reload failed: %s", exception.what());
        }
      });

    if (!std::isfinite(semantic_auto_reload_period_sec_)) {
      throw std::invalid_argument("semantic_auto_reload_period_sec must be finite");
    }
    if (semantic_auto_reload_period_sec_ > 0.0) {
      const auto requested_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(semantic_auto_reload_period_sec_));
      const auto period = std::max(requested_period, std::chrono::milliseconds(50));
      semantic_watch_timer_ = create_wall_timer(
        period, [this]() {checkSemanticFileUpdate();});
    }

    // Transient-local publishers retain these messages for RViz subscribers
    // that join later, so startup does not need to be deferred to a timer.
    // Loading synchronously also lets a missing or corrupt output propagate to
    // main(), which gives automation a non-zero process status.
    const std::size_t count = loadAndPublish();
    RCLCPP_INFO(
      get_logger(), "Published %zu review layers from %s", count,
      output_directory_.string().c_str());
  }

private:
  double positiveParameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!(value > 0.0) || !std::isfinite(value)) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  [[nodiscard]] std::filesystem::path preferredReviewGeometryPath() const
  {
    const std::filesystem::path edited = output_directory_ / "review_geometry_edited.tsv";
    return std::filesystem::exists(edited) ?
           edited : output_directory_ / "review_geometry.tsv";
  }

  std::size_t loadAndPublish()
  {
    if (!std::filesystem::exists(output_directory_)) {
      throw std::runtime_error(
              "output directory does not exist: " + output_directory_.string());
    }

    std::optional<lmmg::RouteGraph> graph;
    const std::filesystem::path review_geometry = preferredReviewGeometryPath();
    if (std::filesystem::exists(review_geometry)) {
      graph = lmmg::loadReviewGeometryTsv(review_geometry);
      review_geometry_path_ = review_geometry;
      review_geometry_last_write_time_ = std::filesystem::last_write_time(review_geometry);
    } else if (publish_route_graph_ || publish_corridors_) {
      review_geometry_path_.clear();
      review_geometry_last_write_time_.reset();
      RCLCPP_WARN(
        get_logger(), "%s is missing; regenerate the map with this package version",
        review_geometry.string().c_str());
    } else {
      review_geometry_path_.clear();
      review_geometry_last_write_time_.reset();
    }

    std::string frame_id = frame_override_;
    if (frame_id.empty() && graph && !graph->frame_id.empty()) {
      frame_id = graph->frame_id;
    }
    if (frame_id.empty()) {
      frame_id = "map";
    }

    const builtin_interfaces::msg::Time stamp = now();
    std::size_t published_layers = 0U;

    if (publish_pointcloud_ && publishPointCloud(frame_id, stamp)) {
      ++published_layers;
    }
    if (publish_diagnostics_ && publishPointCloudFile(
        output_directory_ / "obstacle_points_classified.pcd", frame_id, stamp,
        *classified_obstacle_publisher_, "Classified obstacle", 1U,
        static_cast<std::size_t>(pointcloud_max_points_)))
    {
      ++published_layers;
    }
    if (publish_trajectories_) {
      if (publishTrajectory(
          output_directory_ / "trajectory_raw.tum", frame_id, stamp,
          *raw_trajectory_publisher_))
      {
        ++published_layers;
      }
      if (publishTrajectory(
          output_directory_ / "trajectory_processed.tum", frame_id, stamp,
          *processed_trajectory_publisher_))
      {
        ++published_layers;
      }
    }
    if (graph) {
      if (publish_route_graph_) {
        route_graph_publisher_->publish(makeRouteGraphMarkers(*graph, frame_id, stamp));
        issue_publisher_->publish(makeIssueMarkers(*graph, frame_id, stamp));
        published_layers += 2U;
      }
      if (publish_corridors_) {
        corridor_publisher_->publish(makeCorridorMarkers(*graph, frame_id, stamp));
        ++published_layers;
      }
      if (publish_semantics_ && publishSemanticFeatures(*graph, frame_id, stamp)) {
        ++published_layers;
      }
    } else {
      publishMarkerClear(*route_graph_publisher_, frame_id, stamp);
      publishMarkerClear(*corridor_publisher_, frame_id, stamp);
      publishMarkerClear(*issue_publisher_, frame_id, stamp);
      publishMarkerClear(*semantic_publisher_, frame_id, stamp);
    }

    if (publish_lanelet2_ && publishLanelet2(frame_id, stamp)) {
      ++published_layers;
    }
    if (publish_navigation_map_ && publishNavigationMap(frame_id, stamp)) {
      ++published_layers;
    }
    if (publish_grids_) {
      if (publishOccupancyGrid(
          output_directory_ / "obstacles.yaml", frame_id, stamp,
          *obstacle_publisher_))
      {
        ++published_layers;
      }
      if (publishOccupancyGrid(
          output_directory_ / "obstacles_inflated.yaml", frame_id, stamp,
          *inflated_obstacle_publisher_))
      {
        ++published_layers;
      }
      if (publish_diagnostics_ && publishOccupancyGrid(
          output_directory_ / "observed_free.yaml", frame_id, stamp,
          *observed_free_publisher_))
      {
        ++published_layers;
      }
      if (publish_diagnostics_ && publishOccupancyGrid(
          output_directory_ / "unknown.yaml", frame_id, stamp,
          *unknown_publisher_))
      {
        ++published_layers;
      }
    }
    return published_layers;
  }

  bool publishPointCloud(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    return publishPointCloudFile(
      output_directory_ / "pointcloud_map.pcd", frame_id, stamp,
      *pointcloud_publisher_, "Point cloud map",
      static_cast<std::size_t>(pointcloud_stride_),
      static_cast<std::size_t>(pointcloud_max_points_));
  }

  bool publishPointCloudFile(
    const std::filesystem::path & path,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & publisher,
    const std::string & layer_name,
    const std::size_t requested_stride,
    const std::size_t maximum_points)
  {
    if (!std::filesystem::exists(path)) {
      RCLCPP_WARN(get_logger(), "%s is missing: %s", layer_name.c_str(), path.string().c_str());
      return false;
    }
    const std::vector<lmmg::PointXYZI> points = lmmg::loadPointCloudFile(path);
    if (points.empty()) {
      RCLCPP_WARN(get_logger(), "%s is empty: %s", layer_name.c_str(), path.string().c_str());
      return false;
    }

    const std::size_t capacity_stride =
      points.size() > maximum_points ? (points.size() + maximum_points - 1U) / maximum_points : 1U;
    const std::size_t stride = std::max<std::size_t>(1U,
      std::max(requested_stride, capacity_stride));
    const std::size_t selected_count = (points.size() + stride - 1U) / stride;

    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = frame_id;
    message.header.stamp = stamp;
    message.height = 1U;
    message.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(selected_count);

    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
    for (std::size_t index = 0U; index < points.size();
      index += stride, ++x, ++y, ++z, ++intensity)
    {
      *x = points[index].x;
      *y = points[index].y;
      *z = points[index].z;
      *intensity = points[index].intensity;
    }
    publisher.publish(message);
    RCLCPP_INFO(
      get_logger(), "%s review layer: %zu/%zu points (stride %zu)",
      layer_name.c_str(), selected_count, points.size(), stride);
    return true;
  }

  bool publishTrajectory(
    const std::filesystem::path & path,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp,
    rclcpp::Publisher<nav_msgs::msg::Path> & publisher)
  {
    if (!std::filesystem::exists(path)) {
      RCLCPP_WARN(get_logger(), "Trajectory is missing: %s", path.string().c_str());
      return false;
    }
    const std::vector<lmmg::TimedPose> trajectory = lmmg::loadTumTrajectory(path);
    nav_msgs::msg::Path message;
    message.header.frame_id = frame_id;
    message.header.stamp = stamp;
    message.poses.reserve(trajectory.size());
    for (const lmmg::TimedPose & pose : trajectory) {
      geometry_msgs::msg::PoseStamped output;
      output.header.frame_id = frame_id;
      output.header.stamp = timeMessage(pose.stamp_ns);
      output.pose.position.x = pose.world_from_body.translation.x;
      output.pose.position.y = pose.world_from_body.translation.y;
      output.pose.position.z = pose.world_from_body.translation.z + z_offset_;
      output.pose.orientation.x = pose.world_from_body.rotation.x;
      output.pose.orientation.y = pose.world_from_body.rotation.y;
      output.pose.orientation.z = pose.world_from_body.rotation.z;
      output.pose.orientation.w = pose.world_from_body.rotation.w;
      message.poses.push_back(output);
    }
    publisher.publish(message);
    return true;
  }

  MarkerArray makeRouteGraphMarkers(
    const lmmg::RouteGraph & graph,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    MarkerArray array;
    array.markers.push_back(deleteAllMarker(frame_id, stamp));
    std::int32_t centerline_id = 0;
    std::int32_t arrow_id = 0;
    std::int32_t label_id = 0;

    Marker endpoint_nodes = baseMarker(frame_id, stamp, "nodes_endpoint", 0, Marker::SPHERE_LIST);
    Marker junction_nodes = baseMarker(frame_id, stamp, "nodes_junction", 0, Marker::SPHERE_LIST);
    Marker normal_nodes = baseMarker(frame_id, stamp, "nodes_normal", 0, Marker::SPHERE_LIST);
    for (Marker * marker : {&endpoint_nodes, &junction_nodes, &normal_nodes}) {
      marker->scale.x = node_scale_;
      marker->scale.y = node_scale_;
      marker->scale.z = node_scale_;
    }
    endpoint_nodes.color = color(0.1F, 0.4F, 1.0F, 1.0F);
    junction_nodes.color = color(0.9F, 0.1F, 0.9F, 1.0F);
    normal_nodes.color = color(0.7F, 0.7F, 0.7F, 1.0F);
    for (const lmmg::RouteNode & node : graph.nodes) {
      Marker * target = &normal_nodes;
      if (node.type == lmmg::RouteNodeType::kEndpoint) {
        target = &endpoint_nodes;
      } else if (node.type == lmmg::RouteNodeType::kJunction) {
        target = &junction_nodes;
      }
      target->points.push_back(point(node.position, z_offset_));

      if (publish_labels_) {
        Marker label = baseMarker(frame_id, stamp, "node_labels", label_id++,
          Marker::TEXT_VIEW_FACING);
        label.pose.position = point(node.position, z_offset_ + 0.5 * node_scale_);
        label.scale.z = label_scale_;
        label.color = color(1.0F, 1.0F, 1.0F, 0.95F);
        label.text = "N" + std::to_string(node.id);
        array.markers.push_back(std::move(label));
      }
    }
    if (!endpoint_nodes.points.empty()) {
      array.markers.push_back(std::move(endpoint_nodes));
    }
    if (!junction_nodes.points.empty()) {
      array.markers.push_back(std::move(junction_nodes));
    }
    if (!normal_nodes.points.empty()) {
      array.markers.push_back(std::move(normal_nodes));
    }

    for (const lmmg::RouteEdge & edge : graph.edges) {
      if (edge.centerline.size() < 2U) {
        continue;
      }
      if (!mirroredGeometryDuplicate(edge)) {
        Marker line = baseMarker(
          frame_id, stamp, "route_centerline", centerline_id++, Marker::LINE_STRIP);
        line.scale.x = route_line_width_;
        if (!edge.passable) {
          line.color = color(1.0F, 0.05F, 0.05F, 1.0F);
        } else if (edge.confidence < low_confidence_threshold_) {
          line.color = color(1.0F, 0.65F, 0.05F, 1.0F);
        } else {
          line.color = color(0.1F, 0.95F, 0.25F, 1.0F);
        }
        for (const lmmg::Vec3 & sample : edge.centerline) {
          line.points.push_back(point(sample, z_offset_));
        }
        array.markers.push_back(std::move(line));
      }

      const auto segment = middleDirectionSegment(edge.centerline);
      if (segment) {
        Marker arrow = baseMarker(frame_id, stamp, "route_direction", arrow_id++, Marker::ARROW);
        arrow.scale.x = 0.35 * arrow_scale_;
        arrow.scale.y = arrow_scale_;
        arrow.scale.z = 0.8 * arrow_scale_;
        arrow.color = edge.passable ?
          color(0.1F, 0.65F, 1.0F, 0.95F) : color(1.0F, 0.05F, 0.05F, 0.95F);
        arrow.points.push_back(point(segment->first, z_offset_ + route_line_width_));
        arrow.points.push_back(point(segment->second, z_offset_ + route_line_width_));
        array.markers.push_back(std::move(arrow));
      }

      if (publish_labels_) {
        Marker label = baseMarker(frame_id, stamp, "edge_labels", label_id++,
          Marker::TEXT_VIEW_FACING);
        label.pose.position = point(polylineMidpoint(edge.centerline), z_offset_ + label_scale_);
        label.scale.z = label_scale_;
        label.color = edge.passable ?
          color(1.0F, 1.0F, 1.0F, 0.95F) : color(1.0F, 0.2F, 0.2F, 1.0F);
        label.text = edgeLabel(edge);
        array.markers.push_back(std::move(label));
      }
    }
    return array;
  }

  MarkerArray makeCorridorMarkers(
    const lmmg::RouteGraph & graph,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    MarkerArray array;
    array.markers.push_back(deleteAllMarker(frame_id, stamp));
    std::int32_t id = 0;
    for (const lmmg::RouteEdge & edge : graph.edges) {
      // A self-intersecting or degenerate diagnostic boundary must not be
      // rendered as a corridor.  Its centerline and validation reason remain
      // visible in the Route Graph and Review Issues layers.
      if (mirroredGeometryDuplicate(edge) || !edge.corridor_geometry_valid ||
        edge.left_boundary.size() < 2U ||
        edge.left_boundary.size() != edge.right_boundary.size())
      {
        continue;
      }

      Marker fill = baseMarker(frame_id, stamp, "corridor_fill", id++, Marker::TRIANGLE_LIST);
      fill.color = edge.passable ?
        color(0.1F, 0.8F, 0.25F, 0.18F) : color(1.0F, 0.05F, 0.05F, 0.28F);
      for (std::size_t index = 1U; index < edge.left_boundary.size(); ++index) {
        const geometry_msgs::msg::Point left_previous = point(edge.left_boundary[index - 1U],
          0.5 * z_offset_);
        const geometry_msgs::msg::Point right_previous = point(edge.right_boundary[index - 1U],
          0.5 * z_offset_);
        const geometry_msgs::msg::Point left_current = point(edge.left_boundary[index],
          0.5 * z_offset_);
        const geometry_msgs::msg::Point right_current = point(edge.right_boundary[index],
          0.5 * z_offset_);
        fill.points.push_back(left_previous);
        fill.points.push_back(right_previous);
        fill.points.push_back(left_current);
        fill.points.push_back(left_current);
        fill.points.push_back(right_previous);
        fill.points.push_back(right_current);
      }
      array.markers.push_back(std::move(fill));

      Marker left = baseMarker(frame_id, stamp, "corridor_left", id++, Marker::LINE_STRIP);
      left.scale.x = boundary_line_width_;
      left.color = edge.passable ?
        color(0.1F, 0.55F, 1.0F, 0.95F) : color(1.0F, 0.05F, 0.05F, 0.95F);
      Marker right = baseMarker(frame_id, stamp, "corridor_right", id++, Marker::LINE_STRIP);
      right.scale.x = boundary_line_width_;
      right.color = edge.passable ?
        color(1.0F, 0.8F, 0.1F, 0.95F) : color(1.0F, 0.05F, 0.05F, 0.95F);
      for (std::size_t index = 0U; index < edge.left_boundary.size(); ++index) {
        left.points.push_back(point(edge.left_boundary[index], z_offset_));
        right.points.push_back(point(edge.right_boundary[index], z_offset_));
      }
      array.markers.push_back(std::move(left));
      array.markers.push_back(std::move(right));
    }
    return array;
  }

  MarkerArray makeIssueMarkers(
    const lmmg::RouteGraph & graph,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    MarkerArray array;
    array.markers.push_back(deleteAllMarker(frame_id, stamp));
    std::int32_t id = 0;
    for (const lmmg::RouteEdge & edge : graph.edges) {
      if (mirroredGeometryDuplicate(edge) ||
        (edge.passable && edge.confidence >= low_confidence_threshold_))
      {
        continue;
      }
      const lmmg::Vec3 location = polylineMidpoint(edge.centerline);
      Marker sphere = baseMarker(frame_id, stamp, "issue_location", id++, Marker::SPHERE);
      sphere.pose.position = point(location, z_offset_ + 0.15);
      sphere.scale.x = 0.30;
      sphere.scale.y = 0.30;
      sphere.scale.z = 0.30;
      sphere.color = edge.passable ?
        color(1.0F, 0.65F, 0.0F, 0.95F) : color(1.0F, 0.0F, 0.0F, 1.0F);
      array.markers.push_back(std::move(sphere));

      Marker label = baseMarker(frame_id, stamp, "issue_label", id++, Marker::TEXT_VIEW_FACING);
      label.pose.position = point(location, z_offset_ + 0.45);
      label.scale.z = label_scale_;
      label.color = edge.passable ?
        color(1.0F, 0.75F, 0.1F, 1.0F) : color(1.0F, 0.1F, 0.1F, 1.0F);
      label.text = !edge.passable ?
        "E" + std::to_string(edge.id) + ": " +
        (edge.validation_errors.empty() ? "clearance failed" : edge.validation_errors.front()) :
        "E" + std::to_string(edge.id) + ": low confidence";
      array.markers.push_back(std::move(label));
    }
    return array;
  }


  bool publishSemanticFeatures(
    const lmmg::RouteGraph & graph,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    const std::filesystem::path path = output_directory_ / "semantic_features.tsv";
    if (!std::filesystem::exists(path)) {
      publishMarkerClear(*semantic_publisher_, frame_id, stamp);
      semantic_last_write_time_.reset();
      return false;
    }
    const lmmg::SemanticMap semantic_map = lmmg::loadSemanticMapTsv(path, &graph);
    MarkerArray array;
    array.markers.push_back(deleteAllMarker(frame_id, stamp));
    std::int32_t id = 0;

    auto feature_color = [](const lmmg::SemanticFeatureType type, const float alpha) {
        switch (type) {
          case lmmg::SemanticFeatureType::kStop: return color(1.0F, 0.72F, 0.05F, alpha);
          case lmmg::SemanticFeatureType::kWait: return color(0.35F, 0.75F, 1.0F, alpha);
          case lmmg::SemanticFeatureType::kDock: return color(0.0F, 0.85F, 0.55F, alpha);
          case lmmg::SemanticFeatureType::kCharger: return color(0.0F, 1.0F, 0.85F, alpha);
          case lmmg::SemanticFeatureType::kDoor: return color(0.96F, 0.55F, 0.25F, alpha);
          case lmmg::SemanticFeatureType::kSpeedLimit: return color(0.60F, 0.35F, 0.90F, alpha);
          case lmmg::SemanticFeatureType::kNoEntry: return color(1.0F, 0.12F, 0.25F, alpha);
        }
        return color(1.0F, 1.0F, 1.0F, alpha);
      };
    auto find_edge = [&graph](const std::uint64_t edge_id) -> const lmmg::RouteEdge * {
        const auto found = std::find_if(
          graph.edges.begin(), graph.edges.end(),
          [edge_id](const lmmg::RouteEdge & edge) {return edge.id == edge_id;});
        return found == graph.edges.end() ? nullptr : &*found;
      };
    auto label_text = [](const lmmg::SemanticFeature & feature) {
        std::ostringstream stream;
        stream << "S" << feature.id << " " << lmmg::toString(feature.type);
        if (!feature.name.empty()) {
          stream << " " << feature.name;
        }
        if (feature.type == lmmg::SemanticFeatureType::kSpeedLimit) {
          stream << " " << std::fixed << std::setprecision(2) << feature.value << "m/s";
        } else if (feature.type == lmmg::SemanticFeatureType::kDoor) {
          stream << " width=" << std::fixed << std::setprecision(2) << feature.extent << "m";
        }
        if (!feature.enabled) {
          stream << " DISABLED";
        }
        return stream.str();
      };

    for (const lmmg::SemanticFeature & feature : semantic_map.features) {
      const float alpha = feature.enabled ? 0.95F : 0.30F;
      const std_msgs::msg::ColorRGBA marker_color = feature_color(feature.type, alpha);
      lmmg::Vec3 label_position = feature.position;

      if (feature.geometry == lmmg::SemanticGeometryType::kPoint) {
        if (feature.type == lmmg::SemanticFeatureType::kDoor) {
          const lmmg::Vec3 direction{
            -std::sin(feature.yaw), std::cos(feature.yaw), 0.0};
          const lmmg::Vec3 first = feature.position - direction * (0.5 * feature.extent);
          const lmmg::Vec3 second = feature.position + direction * (0.5 * feature.extent);
          Marker door = baseMarker(frame_id, stamp, "semantic_door", id++, Marker::LINE_LIST);
          door.scale.x = semantic_line_width_;
          door.color = marker_color;
          door.points.push_back(point(first, 2.0 * z_offset_));
          door.points.push_back(point(second, 2.0 * z_offset_));
          array.markers.push_back(std::move(door));
        } else {
          Marker body = baseMarker(frame_id, stamp, "semantic_point", id++, Marker::CYLINDER);
          body.pose.position = point(feature.position, 2.0 * z_offset_);
          body.pose.orientation.z = std::sin(0.5 * feature.yaw);
          body.pose.orientation.w = std::cos(0.5 * feature.yaw);
          body.scale.x = semantic_point_scale_;
          body.scale.y = semantic_point_scale_;
          body.scale.z = 0.10;
          body.color = marker_color;
          array.markers.push_back(std::move(body));

          Marker heading = baseMarker(frame_id, stamp, "semantic_heading", id++, Marker::ARROW);
          const lmmg::Vec3 tip = feature.position + lmmg::Vec3{
            std::cos(feature.yaw), std::sin(feature.yaw), 0.0} *
          (1.3 * semantic_point_scale_);
          heading.scale.x = 0.25 * semantic_point_scale_;
          heading.scale.y = 0.55 * semantic_point_scale_;
          heading.scale.z = 0.55 * semantic_point_scale_;
          heading.color = marker_color;
          heading.points.push_back(point(feature.position, 2.5 * z_offset_));
          heading.points.push_back(point(tip, 2.5 * z_offset_));
          array.markers.push_back(std::move(heading));
        }
      } else if (feature.geometry == lmmg::SemanticGeometryType::kRouteEdges) {
        bool label_set = false;
        for (const std::uint64_t edge_id : feature.route_edge_ids) {
          const lmmg::RouteEdge * edge = find_edge(edge_id);
          if (edge == nullptr || edge->centerline.size() < 2U) {
            continue;
          }
          Marker line = baseMarker(frame_id, stamp, "semantic_route_rule", id++,
            Marker::LINE_STRIP);
          line.scale.x = semantic_line_width_;
          line.color = marker_color;
          for (const lmmg::Vec3 & sample : edge->centerline) {
            line.points.push_back(point(sample, 3.0 * z_offset_));
          }
          array.markers.push_back(std::move(line));
          const auto direction = middleDirectionSegment(edge->centerline);
          if (direction) {
            Marker arrow = baseMarker(frame_id, stamp, "semantic_route_direction", id++,
              Marker::ARROW);
            arrow.scale.x = 0.20 * semantic_point_scale_;
            arrow.scale.y = 0.55 * semantic_point_scale_;
            arrow.scale.z = 0.55 * semantic_point_scale_;
            arrow.color = marker_color;
            arrow.points.push_back(point(direction->first, 3.5 * z_offset_));
            arrow.points.push_back(point(direction->second, 3.5 * z_offset_));
            array.markers.push_back(std::move(arrow));
          }
          if (!label_set) {
            label_position = polylineMidpoint(edge->centerline);
            label_set = true;
          }
        }
      } else {
        if (feature.polygon.size() >= 3U) {
          Marker outline = baseMarker(frame_id, stamp, "semantic_zone", id++, Marker::LINE_STRIP);
          outline.scale.x = semantic_line_width_;
          outline.color = marker_color;
          lmmg::Vec3 centroid{};
          for (const lmmg::Vec3 & vertex : feature.polygon) {
            outline.points.push_back(point(vertex, 3.0 * z_offset_));
            centroid += vertex;
          }
          outline.points.push_back(point(feature.polygon.front(), 3.0 * z_offset_));
          array.markers.push_back(std::move(outline));
          label_position = centroid / static_cast<double>(feature.polygon.size());

          Marker vertices = baseMarker(frame_id, stamp, "semantic_zone_vertices", id++,
            Marker::SPHERE_LIST);
          vertices.scale.x = 0.45 * semantic_point_scale_;
          vertices.scale.y = 0.45 * semantic_point_scale_;
          vertices.scale.z = 0.45 * semantic_point_scale_;
          vertices.color = marker_color;
          for (const lmmg::Vec3 & vertex : feature.polygon) {
            vertices.points.push_back(point(vertex, 3.0 * z_offset_));
          }
          array.markers.push_back(std::move(vertices));
        }
      }

      if (publish_labels_) {
        Marker label = baseMarker(frame_id, stamp, "semantic_labels", id++,
          Marker::TEXT_VIEW_FACING);
        label.pose.position = point(label_position, 4.0 * z_offset_ + label_scale_);
        label.scale.z = label_scale_;
        label.color = marker_color;
        label.text = label_text(feature);
        array.markers.push_back(std::move(label));
      }
    }
    semantic_publisher_->publish(array);
    semantic_last_write_time_ = std::filesystem::last_write_time(path);
    RCLCPP_INFO(get_logger(), "Semantic review layer: %zu features", semantic_map.features.size());
    return true;
  }

  void checkSemanticFileUpdate()
  {
    if (!std::filesystem::exists(output_directory_)) {
      return;
    }
    try {
      const std::filesystem::path geometry_path = preferredReviewGeometryPath();
      if (std::filesystem::exists(geometry_path)) {
        const auto modified = std::filesystem::last_write_time(geometry_path);
        if (!review_geometry_last_write_time_ || review_geometry_path_ != geometry_path ||
          *review_geometry_last_write_time_ != modified)
        {
          // Route edits change the graph, corridor and issue layers as well as
          // the semantic attachment geometry. Reload every review layer so
          // RViz never displays semantics against stale Route Edge IDs.
          static_cast<void>(loadAndPublish());
          return;
        }
      } else if (review_geometry_last_write_time_) {
        static_cast<void>(loadAndPublish());
        return;
      }

      if (!publish_semantics_) {
        return;
      }
      const std::filesystem::path semantic_path = output_directory_ / "semantic_features.tsv";
      if (!std::filesystem::exists(semantic_path)) {
        if (semantic_last_write_time_) {
          const std::string frame_id = frame_override_.empty() ? "map" : frame_override_;
          publishMarkerClear(*semantic_publisher_, frame_id, now());
          semantic_last_write_time_.reset();
        }
        return;
      }
      const auto modified = std::filesystem::last_write_time(semantic_path);
      if (semantic_last_write_time_ && *semantic_last_write_time_ == modified) {
        return;
      }
      if (!std::filesystem::exists(geometry_path)) {
        return;
      }
      const lmmg::RouteGraph graph = lmmg::loadReviewGeometryTsv(geometry_path);
      std::string frame_id = frame_override_;
      if (frame_id.empty()) {
        frame_id = graph.frame_id.empty() ? "map" : graph.frame_id;
      }
      static_cast<void>(publishSemanticFeatures(graph, frame_id, now()));
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Semantic auto-reload failed: %s", exception.what());
    }
  }

  bool publishLanelet2(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    const auto named_path = [this](const std::string & source) {
        if (source == "closed_course_experimental") {
          return output_directory_ / "lanelet2_map_closed_course_experimental.osm";
        }
        if (source == "validated") {
          return output_directory_ / "lanelet2_map_validated.osm";
        }
        if (source == "generated") {
          return output_directory_ / "lanelet2_map_generated.osm";
        }
        return output_directory_ / "lanelet2_map.osm";
      };
    std::filesystem::path path;
    std::vector<lmmg::Lanelet2ReviewLanelet> lanelets;
    if (lanelet2_source_ == "auto") {
      for (const char * candidate : {
        "closed_course_experimental", "validated", "generated", "canonical"
      })
      {
        const std::filesystem::path candidate_path = named_path(candidate);
        if (!std::filesystem::exists(candidate_path)) {
          continue;
        }
        std::vector<lmmg::Lanelet2ReviewLanelet> candidate_lanelets =
          lmmg::loadGeneratedLanelet2Osm(candidate_path);
        if (!candidate_lanelets.empty()) {
          path = candidate_path;
          lanelets = std::move(candidate_lanelets);
          break;
        }
      }
    } else {
      path = named_path(lanelet2_source_);
    }
    if (path.empty() || !std::filesystem::exists(path)) {
      RCLCPP_WARN(
        get_logger(), "No non-empty Lanelet2 map is available for source '%s' in %s",
        lanelet2_source_.c_str(), output_directory_.string().c_str());
      publishMarkerClear(*lanelet2_publisher_, frame_id, stamp);
      return false;
    }
    if (lanelets.empty()) {
      lanelets = lmmg::loadGeneratedLanelet2Osm(path);
    }
    MarkerArray array;
    array.markers.push_back(deleteAllMarker(frame_id, stamp));
    std::int32_t id = 0;
    for (const lmmg::Lanelet2ReviewLanelet & lanelet : lanelets) {
      auto add_line = [&](
        const std::string & marker_namespace,
        const std::vector<lmmg::Vec3> & samples,
        const std_msgs::msg::ColorRGBA & marker_color,
        const double offset)
        {
          if (samples.size() < 2U) {
            return;
          }
          Marker marker = baseMarker(frame_id, stamp, marker_namespace, id++, Marker::LINE_STRIP);
          marker.scale.x = 0.75 * boundary_line_width_;
          marker.color = marker_color;
          for (const lmmg::Vec3 & sample : samples) {
            marker.points.push_back(point(sample, offset));
          }
          array.markers.push_back(std::move(marker));
        };
      add_line(
        "lanelet2_left", lanelet.left_boundary,
        color(0.0F, 1.0F, 1.0F, 0.95F), 1.5 * z_offset_);
      add_line(
        "lanelet2_right", lanelet.right_boundary,
        color(1.0F, 0.15F, 0.85F, 0.95F), 1.5 * z_offset_);
      add_line(
        "lanelet2_center", lanelet.centerline,
        lanelet.passable ? color(1.0F, 1.0F, 1.0F, 0.9F) : color(1.0F, 0.0F, 0.0F, 1.0F),
        1.75 * z_offset_);
    }
    lanelet2_publisher_->publish(array);
    RCLCPP_INFO(
      get_logger(), "Lanelet2 review layer: %zu lanelets from %s",
      lanelets.size(), path.filename().string().c_str());
    return true;
  }

  bool publishNavigationMap(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    const auto named_path = [this](const std::string & source) {
        if (source == "closed_course_experimental") {
          return output_directory_ / "nav2_map_closed_course_experimental.yaml";
        }
        if (source == "generated") {
          return output_directory_ / "nav2_map_generated.yaml";
        }
        return output_directory_ / "nav2_map.yaml";
      };

    std::filesystem::path path;
    std::string selected_source = navigation_map_source_;
    if (navigation_map_source_ == "auto") {
      // Production output is deliberately all UNKNOWN until its safety gate
      // passes. Prefer the separately named closed-course review result, then
      // the generated candidate, so RViz can still show the measured map
      // without implying that either file is approved for deployment.
      for (const char * candidate : {
        "closed_course_experimental", "generated", "canonical"
      })
      {
        const std::filesystem::path candidate_path = named_path(candidate);
        if (std::filesystem::exists(candidate_path)) {
          path = candidate_path;
          selected_source = candidate;
          break;
        }
      }
    } else {
      path = named_path(navigation_map_source_);
    }
    if (path.empty() || !std::filesystem::exists(path)) {
      RCLCPP_WARN(
        get_logger(), "No 2D navigation map is available for source '%s' in %s",
        navigation_map_source_.c_str(), output_directory_.string().c_str());
      return false;
    }
    const bool published = publishOccupancyGrid(
      path, frame_id, stamp, *navigation_map_publisher_);
    if (published) {
      RCLCPP_INFO(
        get_logger(), "2D navigation map review layer: %s (%s)",
        path.filename().string().c_str(), selected_source.c_str());
    }
    return published;
  }

  bool publishOccupancyGrid(
    const std::filesystem::path & path,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp,
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid> & publisher)
  {
    if (!std::filesystem::exists(path)) {
      RCLCPP_WARN(get_logger(), "Occupancy grid is missing: %s", path.string().c_str());
      return false;
    }
    const lmmg::LoadedOccupancyGrid loaded = lmmg::loadOccupancyGridYaml(path);
    const lmmg::OccupancyGrid2D & grid = loaded.grid;
    nav_msgs::msg::OccupancyGrid message;
    message.header.frame_id = frame_id;
    message.header.stamp = stamp;
    message.info.map_load_time = stamp;
    message.info.resolution = static_cast<float>(grid.resolution());
    message.info.width = static_cast<std::uint32_t>(grid.width());
    message.info.height = static_cast<std::uint32_t>(grid.height());
    message.info.origin.position.x = grid.originX();
    message.info.origin.position.y = grid.originY();
    message.info.origin.orientation.w = 1.0;
    if (loaded.occupancy_values.size() != grid.width() * grid.height()) {
      throw std::runtime_error(
              "Occupancy review values have an invalid size: " + path.string());
    }
    message.data = loaded.occupancy_values;
    publisher.publish(message);
    return true;
  }

  template<typename PublisherT>
  void publishMarkerClear(
    PublisherT & publisher,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    MarkerArray clear;
    clear.markers.push_back(deleteAllMarker(frame_id, stamp));
    publisher.publish(clear);
  }

  std::filesystem::path output_directory_;
  std::string frame_override_;
  bool publish_pointcloud_{true};
  bool publish_trajectories_{true};
  bool publish_route_graph_{true};
  bool publish_corridors_{true};
  bool publish_lanelet2_{true};
  bool publish_navigation_map_{true};
  bool publish_semantics_{true};
  bool publish_grids_{true};
  bool publish_diagnostics_{true};
  bool publish_labels_{true};
  std::string lanelet2_source_{"auto"};
  std::string navigation_map_source_{"auto"};
  std::int64_t pointcloud_stride_{1};
  std::int64_t pointcloud_max_points_{2000000};
  double z_offset_{0.05};
  double route_line_width_{0.08};
  double boundary_line_width_{0.04};
  double node_scale_{0.25};
  double arrow_scale_{0.20};
  double label_scale_{0.25};
  double semantic_point_scale_{0.35};
  double semantic_line_width_{0.14};
  double low_confidence_threshold_{0.50};
  double semantic_auto_reload_period_sec_{1.0};
  std::optional<std::filesystem::file_time_type> semantic_last_write_time_;
  std::filesystem::path review_geometry_path_;
  std::optional<std::filesystem::file_time_type> review_geometry_last_write_time_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr classified_obstacle_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processed_trajectory_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr route_graph_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr corridor_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr lanelet2_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr issue_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr semantic_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr navigation_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_obstacle_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr observed_free_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr unknown_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
  rclcpp::TimerBase::SharedPtr semantic_watch_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VectorMapReviewNode>());
  } catch (const std::exception & exception) {
    std::cerr << "review_vector_map fatal error: " << exception.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
