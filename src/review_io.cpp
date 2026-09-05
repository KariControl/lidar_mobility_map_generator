#include "lidar_mobility_map_generator/review_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(
    input.begin(), input.end(),
    [](const unsigned char character) {return std::isspace(character) != 0;});
  const auto last = std::find_if_not(
    input.rbegin(), input.rend(),
    [](const unsigned char character) {return std::isspace(character) != 0;}).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  while (begin <= line.size()) {
    const std::size_t end = line.find('\t', begin);
    fields.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return fields;
}

std::string sanitizeField(std::string input)
{
  for (char & character : input) {
    if (character == '\t' || character == '\r' || character == '\n') {
      character = ' ';
    }
  }
  return input;
}

std::string joinValidationErrors(const std::vector<std::string> & errors)
{
  std::ostringstream stream;
  for (std::size_t index = 0U; index < errors.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << sanitizeField(errors[index]);
  }
  return stream.str();
}

std::vector<std::string> parseValidationErrors(const std::string & field)
{
  std::vector<std::string> result;
  std::size_t begin = 0U;
  while (begin < field.size()) {
    const std::size_t end = field.find(',', begin);
    const std::string value = field.substr(
      begin, end == std::string::npos ? end : end - begin);
    if (!value.empty()) {
      result.push_back(value);
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return result;
}

RouteNodeType parseNodeType(const std::string & value)
{
  if (value == "endpoint") {
    return RouteNodeType::kEndpoint;
  }
  if (value == "junction") {
    return RouteNodeType::kJunction;
  }
  if (value == "normal") {
    return RouteNodeType::kNormal;
  }
  throw std::runtime_error("unknown route node type in review geometry: " + value);
}

bool parseBool(const std::string & value)
{
  return value == "1" || value == "true" || value == "yes";
}

std::string unquote(std::string value)
{
  value = trim(value);
  if (value.size() >= 2U &&
    ((value.front() == '"' && value.back() == '"') ||
    (value.front() == '\'' && value.back() == '\'')))
  {
    value = value.substr(1U, value.size() - 2U);
  }
  std::string result;
  result.reserve(value.size());
  bool escaped = false;
  for (const char character : value) {
    if (escaped) {
      result.push_back(character);
      escaped = false;
    } else if (character == '\\') {
      escaped = true;
    } else {
      result.push_back(character);
    }
  }
  if (escaped) {
    result.push_back('\\');
  }
  return result;
}

std::string readPgmToken(std::istream & stream)
{
  while (true) {
    stream >> std::ws;
    if (!stream) {
      return {};
    }
    if (stream.peek() == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    std::string token;
    while (stream) {
      const int next = stream.peek();
      if (next == std::char_traits<char>::eof() ||
        std::isspace(static_cast<unsigned char>(next)) != 0)
      {
        break;
      }
      token.push_back(static_cast<char>(stream.get()));
    }
    return token;
  }
}

std::optional<std::string> xmlAttribute(
  const std::string & line, const std::string & name)
{
  const std::string pattern = name + "=\"";
  const std::size_t begin = line.find(pattern);
  if (begin == std::string::npos) {
    return std::nullopt;
  }
  const std::size_t value_begin = begin + pattern.size();
  const std::size_t end = line.find('"', value_begin);
  if (end == std::string::npos) {
    return std::nullopt;
  }
  return line.substr(value_begin, end - value_begin);
}

std::optional<std::pair<std::string, std::string>> xmlTag(const std::string & line)
{
  if (line.find("<tag ") == std::string::npos) {
    return std::nullopt;
  }
  const auto key = xmlAttribute(line, "k");
  const auto value = xmlAttribute(line, "v");
  if (!key || !value) {
    return std::nullopt;
  }
  return std::make_pair(*key, *value);
}

struct OSMRelation
{
  std::uint64_t id{0U};
  std::uint64_t left_way{0U};
  std::uint64_t right_way{0U};
  std::uint64_t center_way{0U};
  std::uint64_t route_edge_id{0U};
  bool passable{true};
  double confidence{1.0};
  double minimum_safe_width{0.0};
};

std::vector<Vec3> resolveWay(
  const std::uint64_t way_id,
  const std::map<std::uint64_t, std::vector<std::uint64_t>> & ways,
  const std::map<std::uint64_t, Vec3> & nodes,
  const std::filesystem::path & path)
{
  const auto way = ways.find(way_id);
  if (way == ways.end()) {
    throw std::runtime_error(
            "Lanelet2 relation references missing way " + std::to_string(way_id) +
            " in " + path.string());
  }
  std::vector<Vec3> result;
  result.reserve(way->second.size());
  for (const std::uint64_t node_id : way->second) {
    const auto node = nodes.find(node_id);
    if (node == nodes.end()) {
      throw std::runtime_error(
              "Lanelet2 way references missing node " + std::to_string(node_id) +
              " in " + path.string());
    }
    result.push_back(node->second);
  }
  return result;
}

}  // namespace

void saveReviewGeometryTsv(
  const std::filesystem::path & path,
  const RouteGraph & graph)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create review geometry: " + path.string());
  }
  stream << std::setprecision(17);
  stream << "VERSION\t2\n";
  stream << "FRAME\t" << sanitizeField(graph.frame_id) << '\n';
  for (const RouteNode & node : graph.nodes) {
    stream << "NODE\t" << node.id << '\t' << toString(node.type) << '\t'
           << node.position.x << '\t' << node.position.y << '\t' << node.position.z << '\n';
  }
  for (const RouteEdge & edge : graph.edges) {
    const std::string reverse = edge.reverse_of ? std::to_string(*edge.reverse_of) : "-";
    stream << "EDGE\t" << edge.id << '\t' << edge.from << '\t' << edge.to << '\t'
           << reverse << '\t' << (edge.passable ? 1 : 0) << '\t' << edge.length << '\t'
           << edge.minimum_safe_width << '\t' << edge.maximum_curvature << '\t'
           << edge.confidence << '\t' << edge.recommended_speed_mps << '\t'
           << edge.centerline.size() << '\t'
           << (edge.corridor_geometry_valid ? 1 : 0) << '\t'
           << joinValidationErrors(edge.validation_errors) << '\n';
    for (std::size_t index = 0U; index < edge.centerline.size(); ++index) {
      const Vec3 & center = edge.centerline[index];
      const Vec3 left = index < edge.left_boundary.size() ? edge.left_boundary[index] : center;
      const Vec3 right = index < edge.right_boundary.size() ? edge.right_boundary[index] : center;
      const double left_clearance =
        index < edge.left_clearance.size() ? edge.left_clearance[index] : 0.0;
      const double right_clearance =
        index < edge.right_clearance.size() ? edge.right_clearance[index] : 0.0;
      stream << "SAMPLE\t" << edge.id << '\t' << index << '\t'
             << center.x << '\t' << center.y << '\t' << center.z << '\t'
             << left.x << '\t' << left.y << '\t' << left.z << '\t'
             << right.x << '\t' << right.y << '\t' << right.z << '\t'
             << left_clearance << '\t' << right_clearance << '\t'
             << (index < edge.left_clearance_observed.size() ?
      static_cast<int>(edge.left_clearance_observed[index]) : 0) << '\t'
             << (index < edge.right_clearance_observed.size() ?
      static_cast<int>(edge.right_clearance_observed[index]) : 0) << '\n';
    }
  }
}

RouteGraph loadReviewGeometryTsv(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open review geometry: " + path.string());
  }

  RouteGraph graph;
  std::unordered_map<std::uint64_t, std::size_t> edge_index;
  std::unordered_map<std::uint64_t, std::size_t> expected_samples;
  std::string line;
  std::size_t line_number = 0U;
  int version = 0;
  while (std::getline(stream, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const std::vector<std::string> fields = splitTabs(line);
    try {
      if (fields[0] == "VERSION") {
        if (fields.size() != 2U || (fields[1] != "1" && fields[1] != "2")) {
          throw std::runtime_error("unsupported review geometry version");
        }
        version = std::stoi(fields[1]);
      } else if (fields[0] == "FRAME") {
        if (fields.size() != 2U) {
          throw std::runtime_error("invalid FRAME record");
        }
        graph.frame_id = fields[1];
      } else if (fields[0] == "NODE") {
        if (fields.size() != 6U) {
          throw std::runtime_error("invalid NODE record");
        }
        RouteNode node;
        node.id = std::stoull(fields[1]);
        node.type = parseNodeType(fields[2]);
        node.position = {std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5])};
        graph.nodes.push_back(node);
      } else if (fields[0] == "EDGE") {
        const std::size_t expected_fields = version >= 2 ? 14U : 12U;
        if (fields.size() != expected_fields) {
          throw std::runtime_error("invalid EDGE record");
        }
        RouteEdge edge;
        edge.id = std::stoull(fields[1]);
        edge.from = std::stoull(fields[2]);
        edge.to = std::stoull(fields[3]);
        if (fields[4] != "-") {
          edge.reverse_of = std::stoull(fields[4]);
        }
        edge.passable = parseBool(fields[5]);
        edge.length = std::stod(fields[6]);
        edge.minimum_safe_width = std::stod(fields[7]);
        edge.maximum_curvature = std::stod(fields[8]);
        edge.confidence = std::stod(fields[9]);
        edge.recommended_speed_mps = std::stod(fields[10]);
        expected_samples[edge.id] = static_cast<std::size_t>(std::stoull(fields[11]));
        if (version >= 2) {
          edge.corridor_geometry_valid = parseBool(fields[12]);
          edge.validation_errors = parseValidationErrors(fields[13]);
        } else {
          edge.corridor_geometry_valid = edge.passable;
        }
        edge_index[edge.id] = graph.edges.size();
        graph.edges.push_back(std::move(edge));
      } else if (fields[0] == "SAMPLE") {
        const std::size_t expected_fields = version >= 2 ? 16U : 14U;
        if (fields.size() != expected_fields) {
          throw std::runtime_error("invalid SAMPLE record");
        }
        const std::uint64_t id = std::stoull(fields[1]);
        const auto found = edge_index.find(id);
        if (found == edge_index.end()) {
          throw std::runtime_error("SAMPLE appears before its EDGE record");
        }
        RouteEdge & edge = graph.edges[found->second];
        const std::size_t sample_index = static_cast<std::size_t>(std::stoull(fields[2]));
        if (sample_index != edge.centerline.size()) {
          throw std::runtime_error("non-contiguous SAMPLE index");
        }
        edge.centerline.push_back({std::stod(fields[3]), std::stod(fields[4]),
            std::stod(fields[5])});
        edge.left_boundary.push_back({std::stod(fields[6]), std::stod(fields[7]),
            std::stod(fields[8])});
        edge.right_boundary.push_back({std::stod(fields[9]), std::stod(fields[10]),
            std::stod(fields[11])});
        edge.left_clearance.push_back(std::stod(fields[12]));
        edge.right_clearance.push_back(std::stod(fields[13]));
        edge.left_clearance_observed.push_back(
          version >= 2 && parseBool(fields[14]) ? 1U : 0U);
        edge.right_clearance_observed.push_back(
          version >= 2 && parseBool(fields[15]) ? 1U : 0U);
      } else {
        throw std::runtime_error("unknown record type '" + fields[0] + "'");
      }
    } catch (const std::exception & exception) {
      throw std::runtime_error(
              "invalid review geometry line " + std::to_string(line_number) +
              " in " + path.string() + ": " + exception.what());
    }
  }

  if (version == 0) {
    throw std::runtime_error("review geometry VERSION record is missing: " + path.string());
  }
  for (const RouteEdge & edge : graph.edges) {
    const auto expected = expected_samples.find(edge.id);
    if (expected == expected_samples.end() || edge.centerline.size() != expected->second) {
      throw std::runtime_error(
              "review geometry sample count mismatch for edge " + std::to_string(edge.id));
    }
  }
  return graph;
}

LoadedOccupancyGrid loadOccupancyGridYaml(const std::filesystem::path & path)
{
  std::ifstream yaml(path);
  if (!yaml) {
    throw std::runtime_error("failed to open occupancy YAML: " + path.string());
  }

  std::filesystem::path image_path;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double occupied_threshold = 0.65;
  double free_threshold = 0.196;
  bool negate = false;
  std::string line;
  while (std::getline(yaml, line)) {
    const std::string stripped = trim(line);
    if (stripped.rfind("image:", 0U) == 0U) {
      image_path = unquote(stripped.substr(std::string("image:").size()));
    } else if (stripped.rfind("resolution:", 0U) == 0U) {
      resolution = std::stod(trim(stripped.substr(std::string("resolution:").size())));
    } else if (stripped.rfind("negate:", 0U) == 0U) {
      negate = parseBool(trim(stripped.substr(std::string("negate:").size())));
    } else if (stripped.rfind("occupied_thresh:", 0U) == 0U) {
      occupied_threshold = std::stod(
        trim(stripped.substr(std::string("occupied_thresh:").size())));
    } else if (stripped.rfind("free_thresh:", 0U) == 0U) {
      free_threshold = std::stod(
        trim(stripped.substr(std::string("free_thresh:").size())));
    } else if (stripped.rfind("origin:", 0U) == 0U) {
      const std::size_t left = stripped.find('[');
      const std::size_t right = stripped.find(']');
      if (left == std::string::npos || right == std::string::npos || right <= left) {
        throw std::runtime_error("invalid occupancy origin in " + path.string());
      }
      std::string values = stripped.substr(left + 1U, right - left - 1U);
      std::replace(values.begin(), values.end(), ',', ' ');
      std::istringstream stream(values);
      double ignored_z = 0.0;
      if (!(stream >> origin_x >> origin_y >> ignored_z)) {
        throw std::runtime_error("invalid occupancy origin values in " + path.string());
      }
    }
  }
  if (image_path.empty() || !(resolution > 0.0)) {
    throw std::runtime_error("occupancy YAML lacks image or positive resolution: " + path.string());
  }
  if (!(free_threshold >= 0.0) || !(occupied_threshold <= 1.0) ||
    !(free_threshold < occupied_threshold))
  {
    throw std::runtime_error("invalid occupancy thresholds in " + path.string());
  }
  if (image_path.is_relative()) {
    image_path = path.parent_path() / image_path;
  }

  std::ifstream pgm(image_path, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("failed to open occupancy image: " + image_path.string());
  }
  const std::string magic = readPgmToken(pgm);
  const std::size_t width = static_cast<std::size_t>(std::stoull(readPgmToken(pgm)));
  const std::size_t height = static_cast<std::size_t>(std::stoull(readPgmToken(pgm)));
  const int maximum_value = std::stoi(readPgmToken(pgm));
  if ((magic != "P5" && magic != "P2") || width == 0U || height == 0U ||
    maximum_value <= 0 || maximum_value > 255)
  {
    throw std::runtime_error("unsupported occupancy PGM header: " + image_path.string());
  }

  OccupancyGrid2D grid(origin_x, origin_y, resolution, width, height);
  std::vector<std::int8_t> occupancy_values(width * height, -1);
  const auto record_pixel = [&](const std::size_t x, const std::size_t row, const int pixel) {
      if (pixel < 0 || pixel > maximum_value) {
        throw std::runtime_error("occupancy PGM pixel is out of range: " + image_path.string());
      }
      const std::size_t cell_y = height - 1U - row;
      const double normalized = static_cast<double>(pixel) /
        static_cast<double>(maximum_value);
      const double occupied_probability = negate ? normalized : 1.0 - normalized;
      std::int8_t value = -1;
      if (occupied_probability > occupied_threshold) {
        value = 100;
        grid.setOccupied(
          static_cast<std::int64_t>(x), static_cast<std::int64_t>(cell_y));
      } else if (occupied_probability < free_threshold) {
        value = 0;
      }
      occupancy_values[cell_y * width + x] = value;
    };
  if (magic == "P5") {
    char header_separator = '\0';
    pgm.get(header_separator);
    if (!pgm || std::isspace(static_cast<unsigned char>(header_separator)) == 0) {
      throw std::runtime_error("invalid binary PGM header separator: " + image_path.string());
    }
    std::vector<std::uint8_t> pixels(width * height);
    pgm.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (pgm.gcount() != static_cast<std::streamsize>(pixels.size())) {
      throw std::runtime_error("truncated occupancy PGM: " + image_path.string());
    }
    for (std::size_t row = 0U; row < height; ++row) {
      for (std::size_t x = 0U; x < width; ++x) {
        record_pixel(x, row, static_cast<int>(pixels[row * width + x]));
      }
    }
  } else {
    for (std::size_t row = 0U; row < height; ++row) {
      for (std::size_t x = 0U; x < width; ++x) {
        const int value = std::stoi(readPgmToken(pgm));
        record_pixel(x, row, value);
      }
    }
  }
  return {std::move(grid), image_path, std::move(occupancy_values)};
}

std::vector<Lanelet2ReviewLanelet> loadGeneratedLanelet2Osm(
  const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open Lanelet2 OSM: " + path.string());
  }

  std::map<std::uint64_t, Vec3> nodes;
  std::map<std::uint64_t, std::vector<std::uint64_t>> ways;
  std::vector<OSMRelation> relations;
  std::string line;
  while (std::getline(stream, line)) {
    const std::string stripped = trim(line);
    if (stripped.rfind("<node ", 0U) == 0U) {
      const auto id_text = xmlAttribute(stripped, "id");
      if (!id_text) {
        throw std::runtime_error("Lanelet2 node without id in " + path.string());
      }
      const std::uint64_t id = std::stoull(*id_text);
      Vec3 point;
      while (std::getline(stream, line)) {
        const std::string child = trim(line);
        if (child == "</node>") {
          break;
        }
        const auto tag = xmlTag(child);
        if (!tag) {
          continue;
        }
        if (tag->first == "local_x") {
          point.x = std::stod(tag->second);
        } else if (tag->first == "local_y") {
          point.y = std::stod(tag->second);
        } else if (tag->first == "ele") {
          point.z = std::stod(tag->second);
        }
      }
      nodes[id] = point;
    } else if (stripped.rfind("<way ", 0U) == 0U) {
      const auto id_text = xmlAttribute(stripped, "id");
      if (!id_text) {
        throw std::runtime_error("Lanelet2 way without id in " + path.string());
      }
      const std::uint64_t id = std::stoull(*id_text);
      std::vector<std::uint64_t> references;
      while (std::getline(stream, line)) {
        const std::string child = trim(line);
        if (child == "</way>") {
          break;
        }
        if (child.rfind("<nd ", 0U) == 0U) {
          const auto reference = xmlAttribute(child, "ref");
          if (reference) {
            references.push_back(std::stoull(*reference));
          }
        }
      }
      ways[id] = std::move(references);
    } else if (stripped.rfind("<relation ", 0U) == 0U) {
      const auto id_text = xmlAttribute(stripped, "id");
      if (!id_text) {
        throw std::runtime_error("Lanelet2 relation without id in " + path.string());
      }
      OSMRelation relation;
      relation.id = std::stoull(*id_text);
      bool is_lanelet = false;
      while (std::getline(stream, line)) {
        const std::string child = trim(line);
        if (child == "</relation>") {
          break;
        }
        if (child.rfind("<member ", 0U) == 0U) {
          const auto role = xmlAttribute(child, "role");
          const auto reference = xmlAttribute(child, "ref");
          if (role && reference) {
            const std::uint64_t id = std::stoull(*reference);
            if (*role == "left") {
              relation.left_way = id;
            } else if (*role == "right") {
              relation.right_way = id;
            } else if (*role == "centerline") {
              relation.center_way = id;
            }
          }
        }
        const auto tag = xmlTag(child);
        if (tag) {
          if (tag->first == "type" && tag->second == "lanelet") {
            is_lanelet = true;
          } else if (tag->first == "route_edge_id") {
            relation.route_edge_id = std::stoull(tag->second);
          } else if (tag->first == "passable") {
            relation.passable = parseBool(tag->second);
          } else if (tag->first == "generator_confidence") {
            relation.confidence = std::stod(tag->second);
          } else if (tag->first == "minimum_safe_width") {
            relation.minimum_safe_width = std::stod(tag->second);
          }
        }
      }
      if (is_lanelet) {
        relations.push_back(relation);
      }
    }
  }

  std::vector<Lanelet2ReviewLanelet> lanelets;
  lanelets.reserve(relations.size());
  for (const OSMRelation & relation : relations) {
    if (relation.left_way == 0U || relation.right_way == 0U) {
      throw std::runtime_error(
              "generated Lanelet2 relation lacks left/right way in " + path.string());
    }
    Lanelet2ReviewLanelet lanelet;
    lanelet.relation_id = relation.id;
    lanelet.route_edge_id = relation.route_edge_id;
    lanelet.passable = relation.passable;
    lanelet.confidence = relation.confidence;
    lanelet.minimum_safe_width = relation.minimum_safe_width;
    lanelet.left_boundary = resolveWay(relation.left_way, ways, nodes, path);
    lanelet.right_boundary = resolveWay(relation.right_way, ways, nodes, path);
    if (relation.center_way != 0U) {
      lanelet.centerline = resolveWay(relation.center_way, ways, nodes, path);
    }
    lanelets.push_back(std::move(lanelet));
  }
  return lanelets;
}

}  // namespace lidar_mobility_map_generator
