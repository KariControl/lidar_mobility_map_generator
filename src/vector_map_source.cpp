#include "lidar_mobility_map_generator/vector_map_source.hpp"

#include "lidar_mobility_map_generator/route_editor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace lidar_mobility_map_generator
{
namespace
{

constexpr std::uint32_t kVectorMapSourceVersion = 1U;

std::string trim(std::string value)
{
  const auto is_space = [](const unsigned char character) {
      return std::isspace(character) != 0;
    };
  value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_space));
  value.erase(std::find_if_not(value.rbegin(), value.rend(), is_space).base(), value.end());
  return value;
}

bool samePoint(const Vec3 & lhs, const Vec3 & rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool isExactDensification(
  const std::vector<Vec3> & raw, const std::vector<Vec3> & validated)
{
  if (raw.size() < 2U || validated.size() < raw.size()) {
    return false;
  }
  if (!samePoint(raw.front(), validated.front()) ||
    !samePoint(raw.back(), validated.back()))
  {
    return false;
  }
  std::size_t validated_index = 1U;
  for (std::size_t raw_index = 1U; raw_index < raw.size(); ++raw_index) {
    const Vec3 & start = raw[raw_index - 1U];
    const Vec3 & end = raw[raw_index];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;
    const double squared_length = dx * dx + dy * dy + dz * dz;
    double previous_ratio = 0.0;
    bool endpoint_seen = false;
    while (validated_index < validated.size()) {
      const Vec3 & point = validated[validated_index++];
      if (samePoint(point, end)) {
        endpoint_seen = true;
        break;
      }
      if (!(squared_length > 0.0)) {
        return false;
      }
      const double ratio =
        ((point.x - start.x) * dx + (point.y - start.y) * dy +
        (point.z - start.z) * dz) / squared_length;
      const Vec3 projected{
        start.x + ratio * dx, start.y + ratio * dy, start.z + ratio * dz};
      const double error = std::sqrt(
        (point.x - projected.x) * (point.x - projected.x) +
        (point.y - projected.y) * (point.y - projected.y) +
        (point.z - projected.z) * (point.z - projected.z));
      const double tolerance = 1.0e-10 * std::max(1.0, std::sqrt(squared_length));
      if (ratio <= previous_ratio || ratio >= 1.0 || error > tolerance) {
        return false;
      }
      previous_ratio = ratio;
    }
    if (!endpoint_seen) {
      return false;
    }
  }
  return validated_index == validated.size();
}

void validateOperationalGraphLineage(
  const RouteGraph & raw, const RouteGraph & validated)
{
  if (validated.frame_id != raw.frame_id) {
    throw std::runtime_error(
            "validated edited-topology graph frame differs from the raw edited graph");
  }
  std::map<std::uint64_t, const RouteNode *> raw_nodes;
  for (const RouteNode & node : raw.nodes) {
    raw_nodes.emplace(node.id, &node);
  }
  for (const RouteNode & node : validated.nodes) {
    const auto source = raw_nodes.find(node.id);
    if (source == raw_nodes.end() || !samePoint(node.position, source->second->position)) {
      throw std::runtime_error(
              "validated edited-topology graph contains a Node not derived from the raw graph");
    }
  }

  std::map<std::uint64_t, const RouteEdge *> raw_edges;
  for (const RouteEdge & edge : raw.edges) {
    raw_edges.emplace(edge.id, &edge);
  }
  std::set<std::uint64_t> validated_edge_ids;
  for (const RouteEdge & edge : validated.edges) {
    validated_edge_ids.insert(edge.id);
  }
  for (const RouteEdge & edge : validated.edges) {
    const auto source = raw_edges.find(edge.id);
    if (source == raw_edges.end() || edge.from != source->second->from ||
      edge.to != source->second->to ||
      !isExactDensification(source->second->centerline, edge.centerline))
    {
      throw std::runtime_error(
              "validated edited-topology Edge is not an exact densification of the raw graph");
    }
    const std::optional<std::uint64_t> expected_reverse =
      source->second->reverse_of &&
      validated_edge_ids.count(*source->second->reverse_of) != 0U ?
      source->second->reverse_of : std::nullopt;
    if (edge.reverse_of != expected_reverse) {
      throw std::runtime_error(
              "validated edited-topology Edge reverse lineage differs from the raw graph");
    }
  }
}

}  // namespace

const char * toString(const VectorMapCenterlineSource source)
{
  switch (source) {
    case VectorMapCenterlineSource::kRecordedTrajectory:
      return "recorded_trajectory";
    case VectorMapCenterlineSource::kEditedTopology:
      return "edited_topology";
  }
  throw std::invalid_argument("unknown Vector Map centerline source");
}

VectorMapCenterlineSource vectorMapCenterlineSourceFromString(const std::string & value)
{
  if (value == "recorded_trajectory") {
    return VectorMapCenterlineSource::kRecordedTrajectory;
  }
  if (value == "edited_topology") {
    return VectorMapCenterlineSource::kEditedTopology;
  }
  throw std::invalid_argument("unknown Vector Map centerline source: " + value);
}

void saveVectorMapSourceSelection(
  const std::filesystem::path & path,
  const VectorMapSourceSelection & selection)
{
  if (selection.version != kVectorMapSourceVersion) {
    throw std::invalid_argument("unsupported Vector Map source selection version");
  }
  if (selection.frame_id.empty() || selection.graph_fingerprint.empty()) {
    throw std::invalid_argument("Vector Map source selection is incomplete");
  }
  std::ofstream stream(path, std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("failed to create Vector Map source selection: " + path.string());
  }
  stream << "LMMG_VECTOR_MAP_SOURCE\t" << selection.version << '\n'
         << "SOURCE\t" << toString(selection.source) << '\n'
         << "FRAME\t" << selection.frame_id << '\n'
         << "GRAPH_FINGERPRINT\t" << selection.graph_fingerprint << '\n';
  stream.close();
  if (!stream) {
    throw std::runtime_error("failed to finish Vector Map source selection: " + path.string());
  }
}

VectorMapSourceSelection loadVectorMapSourceSelection(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open Vector Map source selection: " + path.string());
  }
  std::map<std::string, std::string> fields;
  std::string line;
  bool header_seen = false;
  while (std::getline(stream, line)) {
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const std::size_t separator = line.find('\t');
    if (separator == std::string::npos || line.find('\t', separator + 1U) != std::string::npos) {
      throw std::runtime_error("malformed Vector Map source selection record");
    }
    const std::string key = line.substr(0U, separator);
    const std::string value = line.substr(separator + 1U);
    if (key == "LMMG_VECTOR_MAP_SOURCE") {
      if (header_seen || value != std::to_string(kVectorMapSourceVersion)) {
        throw std::runtime_error("unsupported or duplicate Vector Map source selection header");
      }
      header_seen = true;
      continue;
    }
    if (!fields.emplace(key, value).second) {
      throw std::runtime_error("duplicate Vector Map source selection field: " + key);
    }
  }
  if (!header_seen || fields.size() != 3U || fields.count("SOURCE") == 0U ||
    fields.count("FRAME") == 0U || fields.count("GRAPH_FINGERPRINT") == 0U)
  {
    throw std::runtime_error("Vector Map source selection is incomplete");
  }
  VectorMapSourceSelection result;
  result.version = kVectorMapSourceVersion;
  result.source = vectorMapCenterlineSourceFromString(fields.at("SOURCE"));
  result.frame_id = fields.at("FRAME");
  result.graph_fingerprint = fields.at("GRAPH_FINGERPRINT");
  if (result.frame_id.empty() || result.graph_fingerprint.empty()) {
    throw std::runtime_error("Vector Map source selection contains an empty binding");
  }
  return result;
}

void validateVectorMapSourceSelection(
  const VectorMapSourceSelection & selection,
  const RouteGraph & recorded_trajectory,
  const RouteGraph & edited_topology)
{
  if (selection.version != kVectorMapSourceVersion) {
    throw std::runtime_error("unsupported Vector Map source selection version");
  }
  const RouteGraph & selected =
    selection.source == VectorMapCenterlineSource::kRecordedTrajectory ?
    recorded_trajectory : edited_topology;
  if (selection.frame_id != selected.frame_id) {
    throw std::runtime_error("Vector Map source selection frame differs from the selected graph");
  }
  if (selection.graph_fingerprint != routeGraphFingerprint(selected)) {
    throw std::runtime_error(
            "Vector Map source selection is stale for the selected centerline graph");
  }
  if (selected.edges.empty()) {
    throw std::runtime_error("selected Vector Map centerline graph is empty");
  }
}

const RouteGraph & validateAndSelectVectorMapSourceGraph(
  const VectorMapSourceSelection & selection,
  const RouteGraph & recorded_trajectory,
  const RouteGraph & raw_edited_topology,
  const RouteGraph & validated_edited_topology)
{
  validateVectorMapSourceSelection(
    selection, recorded_trajectory, raw_edited_topology);
  if (selection.source == VectorMapCenterlineSource::kEditedTopology) {
    // Clearance validation may insert samples and remove non-operational
    // Edges, but it must not silently substitute another topology or geometry
    // after the operator-bound raw fingerprint was checked.
    validateOperationalGraphLineage(raw_edited_topology, validated_edited_topology);
    return validated_edited_topology;
  }
  return recorded_trajectory;
}

}  // namespace lidar_mobility_map_generator
