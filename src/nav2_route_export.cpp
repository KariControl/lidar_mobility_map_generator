#include "lidar_mobility_map_generator/nav2_route_export.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

std::string jsonEscape(const std::string & input)
{
  std::string output;
  output.reserve(input.size());
  for (const char character : input) {
    switch (character) {
      case '"': output += "\\\""; break;
      case '\\': output += "\\\\"; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default: output += character; break;
    }
  }
  return output;
}

std::vector<Vec3> segmentCenterline(
  const std::vector<Vec3> & centerline,
  const double maximum_segment_length)
{
  if (centerline.size() < 2U) {
    return {};
  }
  // Route Server represents every exported Edge by its endpoint nodes. Keep
  // every source centerline vertex and subdivide each original 3-D segment
  // independently. RDP/chord simplification cuts corners and shortens the
  // observed route even when its lateral error is small.
  std::vector<Vec3> result{centerline.front()};
  for (std::size_t index = 1U; index < centerline.size(); ++index) {
    const Vec3 start = result.back();
    const Vec3 & end = centerline[index];
    const double length = distance3d(start, end);
    if (length <= 1.0e-9) {
      continue;
    }
    const std::size_t pieces = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / maximum_segment_length)));
    for (std::size_t piece = 1U; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      result.push_back(start + (end - start) * ratio);
    }
  }
  return result;
}

struct Nav2Segment
{
  std::uint64_t id{0U};
  std::uint64_t from{0U};
  std::uint64_t to{0U};
  std::uint64_t source_edge{0U};
  std::size_t segment_index{0U};
  Vec3 start{};
  Vec3 end{};
  double speed_limit{0.0};
};

}  // namespace

void saveNav2RouteGraphGeoJson(
  const std::filesystem::path & path,
  const RouteGraph & graph,
  const double maximum_chord_error,
  const double maximum_segment_length,
  const NamedNavigationRoute * named_route)
{
  if (!(maximum_chord_error > 0.0) || !std::isfinite(maximum_chord_error) ||
    !(maximum_segment_length > 0.0) || !std::isfinite(maximum_segment_length))
  {
    throw std::invalid_argument("Nav2 route segmentation tolerances must be finite and positive");
  }
  // Retain this validated argument for configuration/API compatibility. The
  // lossless exporter deliberately does not simplify by chord error.
  static_cast<void>(maximum_chord_error);
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::uint64_t maximum_id = 0U;
  std::map<std::uint64_t, Vec3> source_nodes;
  for (const RouteNode & node : graph.nodes) {
    maximum_id = std::max(maximum_id, node.id);
    source_nodes[node.id] = node.position;
  }
  for (const RouteEdge & edge : graph.edges) {
    maximum_id = std::max(maximum_id, edge.id);
  }
  if (maximum_id == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("route graph IDs are exhausted");
  }
  std::uint64_t next_id = maximum_id + 1U;
  std::map<std::uint64_t, Vec3> output_nodes;
  std::vector<Nav2Segment> output_edges;
  std::map<std::uint64_t, std::size_t> named_route_order;
  if (named_route != nullptr) {
    for (std::size_t index = 0U; index < named_route->ordered_edge_ids.size(); ++index) {
      named_route_order.emplace(named_route->ordered_edge_ids[index], index);
    }
  }

  for (const RouteEdge & edge : graph.edges) {
    if (!edge.passable || edge.centerline.size() < 2U) {
      continue;
    }
    const std::vector<Vec3> points = segmentCenterline(
      edge.centerline, maximum_segment_length);
    if (points.size() < 2U) {
      continue;
    }
    output_nodes[edge.from] = source_nodes.count(edge.from) != 0U ?
      source_nodes.at(edge.from) : points.front();
    output_nodes[edge.to] = source_nodes.count(edge.to) != 0U ?
      source_nodes.at(edge.to) : points.back();
    std::vector<std::uint64_t> node_ids(points.size(), 0U);
    node_ids.front() = edge.from;
    node_ids.back() = edge.to;
    for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
      if (next_id == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("Nav2 route graph IDs are exhausted");
      }
      node_ids[index] = next_id++;
      output_nodes[node_ids[index]] = points[index];
    }
    for (std::size_t index = 0U; index + 1U < points.size(); ++index) {
      if (next_id == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("Nav2 route graph IDs are exhausted");
      }
      output_edges.push_back({
          next_id++, node_ids[index], node_ids[index + 1U], edge.id, index,
          points[index], points[index + 1U], edge.recommended_speed_mps});
    }
  }

  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create Nav2 route graph: " + path.string());
  }
  stream << std::setprecision(12)
         << "{\n  \"type\": \"FeatureCollection\",\n"
         << "  \"features\": [\n";
  bool first_feature = true;
  auto separator = [&]() {
      if (!first_feature) {
        stream << ",\n";
      }
      first_feature = false;
    };
  for (const auto & [id, point] : output_nodes) {
    separator();
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"id\":" << id << ','
           << "\"frame\":\"" << jsonEscape(graph.frame_id) << "\","
           << "\"metadata\":{\"node_type\":\"route_shape_point\",\"z\":"
           << point.z << "}},\"geometry\":{\"type\":\"Point\",\"coordinates\":["
           << point.x << ',' << point.y << "]}}";
  }
  for (const Nav2Segment & edge : output_edges) {
    separator();
    const double cost = distance3d(edge.start, edge.end);
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"id\":" << edge.id << ','
           << "\"startid\":" << edge.from << ','
           << "\"endid\":" << edge.to << ','
           << "\"cost\":" << cost << ','
           << "\"overridable\":true,\"metadata\":{"
           << "\"class\":\"autogenerated_route_segment\","
           << "\"source_route_edge_id\":" << edge.source_edge << ','
           << "\"source_segment_index\":" << edge.segment_index << ','
           << "\"z_start\":" << edge.start.z << ','
           << "\"z_end\":" << edge.end.z << ','
           << "\"abs_speed_limit\":" << edge.speed_limit;
    const auto authored_order = named_route_order.find(edge.source_edge);
    if (named_route != nullptr && authored_order != named_route_order.end()) {
      stream << ",\"named_route_id\":" << named_route->id
             << ",\"named_route_name\":\"" << jsonEscape(named_route->name) << '"'
             << ",\"named_route_target\":\"" << toString(named_route->target) << '"'
             << ",\"named_route_order\":" << authored_order->second;
    }
    stream << "}},\"geometry\":{\"type\":\"MultiLineString\",\"coordinates\":[[["
           << edge.start.x << ',' << edge.start.y << "],["
           << edge.end.x << ',' << edge.end.y << "]]]}}";
  }
  stream << "\n  ]\n}\n";
}

}  // namespace lidar_mobility_map_generator
