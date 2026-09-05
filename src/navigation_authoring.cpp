#include "lidar_mobility_map_generator/navigation_authoring.hpp"

#include "lidar_mobility_map_generator/route_editor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
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

constexpr double kArcTolerance = 1.0e-8;
constexpr std::uintmax_t kMaximumDocumentBytes = 4U * 1024U * 1024U;
constexpr std::size_t kMaximumRoutes = 10000U;
constexpr std::size_t kMaximumStopLines = 10000U;
constexpr std::size_t kMaximumRouteEdges = 100000U;
constexpr std::size_t kMaximumNameBytes = 256U;
constexpr double kMaximumStopLineWidth = 1000.0;

struct JsonValue
{
  enum class Type {kNull, kBoolean, kNumber, kString, kArray, kObject};
  Type type{Type::kNull};
  bool boolean{false};
  std::string scalar;
  std::vector<JsonValue> array;
  std::map<std::string, JsonValue> object;
};

void appendUtf8(std::string & output, const std::uint32_t value)
{
  if (value <= 0x7fU) {
    output.push_back(static_cast<char>(value));
  } else if (value <= 0x7ffU) {
    output.push_back(static_cast<char>(0xc0U | (value >> 6U)));
    output.push_back(static_cast<char>(0x80U | (value & 0x3fU)));
  } else if (value <= 0xffffU) {
    output.push_back(static_cast<char>(0xe0U | (value >> 12U)));
    output.push_back(static_cast<char>(0x80U | ((value >> 6U) & 0x3fU)));
    output.push_back(static_cast<char>(0x80U | (value & 0x3fU)));
  } else {
    output.push_back(static_cast<char>(0xf0U | (value >> 18U)));
    output.push_back(static_cast<char>(0x80U | ((value >> 12U) & 0x3fU)));
    output.push_back(static_cast<char>(0x80U | ((value >> 6U) & 0x3fU)));
    output.push_back(static_cast<char>(0x80U | (value & 0x3fU)));
  }
}

class JsonParser
{
public:
  explicit JsonParser(const std::string & input) : input_(input) {}

  JsonValue parse()
  {
    JsonValue result = parseValue();
    skipWhitespace();
    if (position_ != input_.size()) {
      fail("unexpected trailing JSON data");
    }
    return result;
  }

private:
  [[noreturn]] void fail(const std::string & message) const
  {
    throw std::runtime_error(
            "navigation authoring JSON at byte " + std::to_string(position_) + ": " + message);
  }

  void skipWhitespace()
  {
    while (position_ < input_.size() &&
      (input_[position_] == ' ' || input_[position_] == '\n' ||
      input_[position_] == '\r' || input_[position_] == '\t'))
    {
      ++position_;
    }
  }

  bool consume(const char expected)
  {
    skipWhitespace();
    if (position_ < input_.size() && input_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  void expect(const char expected)
  {
    if (!consume(expected)) {
      fail(std::string{"expected '"} + expected + "'");
    }
  }

  void expectLiteral(const std::string & value)
  {
    if (input_.compare(position_, value.size(), value) != 0) {
      fail("expected " + value);
    }
    position_ += value.size();
  }

  std::uint32_t parseHex4()
  {
    if (position_ + 4U > input_.size()) {
      fail("incomplete unicode escape");
    }
    std::uint32_t result = 0U;
    for (std::size_t index = 0U; index < 4U; ++index) {
      const char value = input_[position_++];
      result <<= 4U;
      if (value >= '0' && value <= '9') {
        result += static_cast<std::uint32_t>(value - '0');
      } else if (value >= 'a' && value <= 'f') {
        result += static_cast<std::uint32_t>(value - 'a' + 10);
      } else if (value >= 'A' && value <= 'F') {
        result += static_cast<std::uint32_t>(value - 'A' + 10);
      } else {
        fail("invalid unicode escape");
      }
    }
    return result;
  }

  std::string parseString()
  {
    skipWhitespace();
    if (position_ >= input_.size() || input_[position_++] != '"') {
      fail("expected string");
    }
    std::string result;
    while (position_ < input_.size()) {
      const unsigned char value = static_cast<unsigned char>(input_[position_++]);
      if (value == '"') {
        return result;
      }
      if (value < 0x20U) {
        fail("unescaped control character in string");
      }
      if (value != '\\') {
        result.push_back(static_cast<char>(value));
        continue;
      }
      if (position_ >= input_.size()) {
        fail("incomplete string escape");
      }
      const char escaped = input_[position_++];
      switch (escaped) {
        case '"': result.push_back('"'); break;
        case '\\': result.push_back('\\'); break;
        case '/': result.push_back('/'); break;
        case 'b': result.push_back('\b'); break;
        case 'f': result.push_back('\f'); break;
        case 'n': result.push_back('\n'); break;
        case 'r': result.push_back('\r'); break;
        case 't': result.push_back('\t'); break;
        case 'u': {
          std::uint32_t codepoint = parseHex4();
          if (codepoint >= 0xd800U && codepoint <= 0xdbffU) {
            if (position_ + 2U > input_.size() || input_[position_] != '\\' ||
              input_[position_ + 1U] != 'u')
            {
              fail("high unicode surrogate lacks a low surrogate");
            }
            position_ += 2U;
            const std::uint32_t low = parseHex4();
            if (low < 0xdc00U || low > 0xdfffU) {
              fail("invalid low unicode surrogate");
            }
            codepoint = 0x10000U + ((codepoint - 0xd800U) << 10U) + (low - 0xdc00U);
          } else if (codepoint >= 0xdc00U && codepoint <= 0xdfffU) {
            fail("unexpected low unicode surrogate");
          }
          appendUtf8(result, codepoint);
          break;
        }
        default: fail("unknown string escape");
      }
    }
    fail("unterminated string");
  }

  JsonValue parseNumber()
  {
    skipWhitespace();
    const std::size_t begin = position_;
    if (position_ < input_.size() && input_[position_] == '-') {
      ++position_;
    }
    if (position_ >= input_.size()) {
      fail("incomplete number");
    }
    if (input_[position_] == '0') {
      ++position_;
    } else {
      if (input_[position_] < '1' || input_[position_] > '9') {
        fail("invalid number");
      }
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        ++position_;
      }
    }
    if (position_ < input_.size() && input_[position_] == '.') {
      ++position_;
      const std::size_t fraction = position_;
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        ++position_;
      }
      if (fraction == position_) {
        fail("number has an empty fraction");
      }
    }
    if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) {
      ++position_;
      if (position_ < input_.size() && (input_[position_] == '+' || input_[position_] == '-')) {
        ++position_;
      }
      const std::size_t exponent = position_;
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        ++position_;
      }
      if (exponent == position_) {
        fail("number has an empty exponent");
      }
    }
    JsonValue result;
    result.type = JsonValue::Type::kNumber;
    result.scalar = input_.substr(begin, position_ - begin);
    return result;
  }

  JsonValue parseArray()
  {
    JsonValue result;
    result.type = JsonValue::Type::kArray;
    expect('[');
    if (consume(']')) {
      return result;
    }
    while (true) {
      result.array.push_back(parseValue());
      if (consume(']')) {
        return result;
      }
      expect(',');
    }
  }

  JsonValue parseObject()
  {
    JsonValue result;
    result.type = JsonValue::Type::kObject;
    expect('{');
    if (consume('}')) {
      return result;
    }
    while (true) {
      const std::string key = parseString();
      expect(':');
      if (!result.object.emplace(key, parseValue()).second) {
        fail("duplicate object key: " + key);
      }
      if (consume('}')) {
        return result;
      }
      expect(',');
    }
  }

  JsonValue parseValue()
  {
    skipWhitespace();
    if (position_ >= input_.size()) {
      fail("expected a JSON value");
    }
    if (input_[position_] == '{') {return parseObject();}
    if (input_[position_] == '[') {return parseArray();}
    if (input_[position_] == '"') {
      JsonValue result;
      result.type = JsonValue::Type::kString;
      result.scalar = parseString();
      return result;
    }
    if (input_[position_] == 't') {
      expectLiteral("true");
      JsonValue result; result.type = JsonValue::Type::kBoolean; result.boolean = true;
      return result;
    }
    if (input_[position_] == 'f') {
      expectLiteral("false");
      JsonValue result; result.type = JsonValue::Type::kBoolean; result.boolean = false;
      return result;
    }
    if (input_[position_] == 'n') {
      expectLiteral("null");
      return {};
    }
    return parseNumber();
  }

  const std::string & input_;
  std::size_t position_{0U};
};

const JsonValue & requireType(
  const JsonValue & value, const JsonValue::Type type, const std::string & description)
{
  if (value.type != type) {
    throw std::runtime_error("navigation authoring " + description + " has the wrong JSON type");
  }
  return value;
}

const JsonValue & requireMember(const JsonValue & object, const std::string & key)
{
  requireType(object, JsonValue::Type::kObject, "object");
  const auto found = object.object.find(key);
  if (found == object.object.end()) {
    throw std::runtime_error("navigation authoring is missing required field: " + key);
  }
  return found->second;
}

const JsonValue * optionalMember(const JsonValue & object, const std::string & key)
{
  requireType(object, JsonValue::Type::kObject, "object");
  const auto found = object.object.find(key);
  return found == object.object.end() ? nullptr : &found->second;
}

std::string asString(const JsonValue & value, const std::string & name)
{
  requireType(value, JsonValue::Type::kString, name);
  return value.scalar;
}

bool asBoolean(const JsonValue & value, const std::string & name)
{
  requireType(value, JsonValue::Type::kBoolean, name);
  return value.boolean;
}

std::uint64_t asUnsigned(const JsonValue & value, const std::string & name)
{
  requireType(value, JsonValue::Type::kNumber, name);
  if (value.scalar.empty() || value.scalar.front() == '-' ||
    value.scalar.find_first_of(".eE") != std::string::npos)
  {
    throw std::runtime_error("navigation authoring " + name + " must be an unsigned integer");
  }
  std::size_t consumed = 0U;
  try {
    const unsigned long long parsed = std::stoull(value.scalar, &consumed, 10);
    if (consumed != value.scalar.size()) {
      throw std::runtime_error("trailing integer data");
    }
    return static_cast<std::uint64_t>(parsed);
  } catch (const std::exception &) {
    throw std::runtime_error("navigation authoring " + name + " is outside uint64 range");
  }
}

double asDouble(const JsonValue & value, const std::string & name)
{
  requireType(value, JsonValue::Type::kNumber, name);
  std::size_t consumed = 0U;
  try {
    const double parsed = std::stod(value.scalar, &consumed);
    if (consumed != value.scalar.size() || !std::isfinite(parsed)) {
      throw std::runtime_error("invalid number");
    }
    return parsed;
  } catch (const std::exception &) {
    throw std::runtime_error("navigation authoring " + name + " must be finite");
  }
}

Vec3 asAnchor(const JsonValue & value)
{
  if (value.type == JsonValue::Type::kArray) {
    if (value.array.size() != 3U) {
      throw std::runtime_error("navigation authoring stop_line.anchor must have three values");
    }
    return {
      asDouble(value.array[0U], "anchor[0]"),
      asDouble(value.array[1U], "anchor[1]"),
      asDouble(value.array[2U], "anchor[2]")};
  }
  if (value.type == JsonValue::Type::kObject) {
    return {
      asDouble(requireMember(value, "x"), "anchor.x"),
      asDouble(requireMember(value, "y"), "anchor.y"),
      asDouble(requireMember(value, "z"), "anchor.z")};
  }
  throw std::runtime_error("navigation authoring stop_line.anchor must be an array or object");
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream output;
  for (const unsigned char value : input) {
    switch (value) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (value < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned int>(value) << std::dec << std::setw(0);
        } else {
          output << static_cast<char>(value);
        }
        break;
    }
  }
  return output.str();
}

std::ofstream openOutput(const std::filesystem::path & path)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path, std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("failed to create navigation authoring artifact: " + path.string());
  }
  stream << std::setprecision(12);
  return stream;
}

void addError(std::vector<std::string> & errors, const std::string & error)
{
  if (std::find(errors.begin(), errors.end(), error) == errors.end()) {
    errors.push_back(error);
  }
}

const RouteNode * findNode(const RouteGraph & graph, const std::uint64_t id)
{
  const auto found = std::find_if(
    graph.nodes.begin(), graph.nodes.end(),
    [id](const RouteNode & node) {return node.id == id;});
  return found == graph.nodes.end() ? nullptr : &*found;
}

const RouteEdge * findEdge(const RouteGraph & graph, const std::uint64_t id)
{
  const auto found = std::find_if(
    graph.edges.begin(), graph.edges.end(),
    [id](const RouteEdge & edge) {return edge.id == id;});
  return found == graph.edges.end() ? nullptr : &*found;
}

Vec3 sampleAtArcLength(const RouteEdge & edge, const double requested)
{
  if (edge.centerline.size() < 2U) {
    throw std::invalid_argument("cannot sample a degenerate Route Edge");
  }
  const double length = polylineLength(edge.centerline);
  const double target = clamp(requested, 0.0, length);
  double traversed = 0.0;
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const double segment = distance3d(edge.centerline[index - 1U], edge.centerline[index]);
    if (target <= traversed + segment || index + 1U == edge.centerline.size()) {
      const double ratio = segment > 1.0e-12 ? clamp((target - traversed) / segment, 0.0, 1.0) : 0.0;
      return edge.centerline[index - 1U] +
             (edge.centerline[index] - edge.centerline[index - 1U]) * ratio;
    }
    traversed += segment;
  }
  return edge.centerline.back();
}

struct Projection
{
  const RouteEdge * edge{nullptr};
  double s{0.0};
  double distance{std::numeric_limits<double>::infinity()};
  Vec3 point{};
  bool ambiguous{false};
};

Projection projectToGraph(const Vec3 & anchor, const RouteGraph & graph)
{
  Projection best;
  for (const RouteEdge & edge : graph.edges) {
    double traversed = 0.0;
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      const Vec3 & start = edge.centerline[index - 1U];
      const Vec3 & end = edge.centerline[index];
      const Vec3 delta = end - start;
      const double squared_length = dot(delta, delta);
      const double ratio = squared_length > 1.0e-18 ?
        clamp(dot(anchor - start, delta) / squared_length, 0.0, 1.0) : 0.0;
      const Vec3 point = start + delta * ratio;
      const double distance = distance3d(anchor, point);
      const double segment = std::sqrt(squared_length);
      if (distance + 1.0e-9 < best.distance) {
        best = {&edge, traversed + ratio * segment, distance, point, false};
      } else if (best.edge != nullptr && edge.id != best.edge->id &&
        std::abs(distance - best.distance) <= 1.0e-9)
      {
        best.ambiguous = true;
      }
      traversed += segment;
    }
  }
  return best;
}

const NamedNavigationRouteStatus * findRouteStatus(
  const NavigationAuthoringValidationResult & validation, const std::uint64_t id)
{
  const auto found = std::find_if(
    validation.route_statuses.begin(), validation.route_statuses.end(),
    [id](const NamedNavigationRouteStatus & status) {return status.id == id;});
  return found == validation.route_statuses.end() ? nullptr : &*found;
}

const AuthoredStopLineStatus * findStopStatus(
  const NavigationAuthoringValidationResult & validation, const std::uint64_t id)
{
  const auto found = std::find_if(
    validation.stop_line_statuses.begin(), validation.stop_line_statuses.end(),
    [id](const AuthoredStopLineStatus & status) {return status.id == id;});
  return found == validation.stop_line_statuses.end() ? nullptr : &*found;
}

void writeErrors(std::ostream & stream, const std::vector<std::string> & errors)
{
  stream << '[';
  for (std::size_t index = 0U; index < errors.size(); ++index) {
    if (index > 0U) {stream << ',';}
    stream << '"' << jsonEscape(errors[index]) << '"';
  }
  stream << ']';
}

}  // namespace

const char * toString(const NavigationAuthoringTarget target)
{
  switch (target) {
    case NavigationAuthoringTarget::kAutoware: return "autoware";
    case NavigationAuthoringTarget::kNav2: return "nav2";
    case NavigationAuthoringTarget::kBoth: return "both";
  }
  return "both";
}

NavigationAuthoringTarget navigationAuthoringTargetFromString(const std::string & value)
{
  if (value == "autoware") {return NavigationAuthoringTarget::kAutoware;}
  if (value == "nav2") {return NavigationAuthoringTarget::kNav2;}
  if (value == "both") {return NavigationAuthoringTarget::kBoth;}
  throw std::runtime_error("unknown navigation authoring target: " + value);
}

bool includesTarget(
  const NavigationAuthoringTarget authored, const NavigationAuthoringTarget requested)
{
  return authored == NavigationAuthoringTarget::kBoth || authored == requested;
}

NamedNavigationRoute makeCompleteOpenChainNavigationRoute(
  const RouteGraph & graph,
  const std::uint64_t route_id,
  std::string route_name,
  const NavigationAuthoringTarget target,
  const bool require_passable_edges)
{
  if (route_id == 0U) {
    throw std::invalid_argument("complete replay Route ID must be nonzero");
  }
  if (route_name.empty() || route_name.size() > kMaximumNameBytes) {
    throw std::invalid_argument("complete replay Route name must contain 1..256 bytes");
  }
  if (target == NavigationAuthoringTarget::kBoth) {
    throw std::invalid_argument(
            "complete replay Route must use one explicit navigation target");
  }
  if (graph.edges.empty()) {
    throw std::invalid_argument("complete replay graph has no Edges");
  }

  std::set<std::uint64_t> known_nodes;
  for (const RouteNode & node : graph.nodes) {
    if (node.id == 0U || !known_nodes.insert(node.id).second) {
      throw std::invalid_argument("complete replay graph has an invalid or duplicate Node ID");
    }
  }

  NamedNavigationRoute result;
  result.id = route_id;
  result.name = std::move(route_name);
  result.target = target;
  result.start_node_id = graph.edges.front().from;
  result.end_node_id = graph.edges.back().to;
  result.validation_requested = true;
  result.promotion_requested = true;
  result.ordered_edge_ids.reserve(graph.edges.size());

  std::set<std::uint64_t> edge_ids;
  std::set<std::uint64_t> visited_nodes;
  visited_nodes.insert(result.start_node_id);
  std::uint64_t expected_from = result.start_node_id;
  for (const RouteEdge & edge : graph.edges) {
    if (edge.id == 0U || !edge_ids.insert(edge.id).second) {
      throw std::invalid_argument("complete replay graph has an invalid or duplicate Edge ID");
    }
    if (known_nodes.count(edge.from) == 0U || known_nodes.count(edge.to) == 0U) {
      throw std::invalid_argument("complete replay Edge references a missing Node");
    }
    if (require_passable_edges && !edge.passable) {
      throw std::invalid_argument(
              "complete replay graph contains a non-passable Edge: " +
              std::to_string(edge.id));
    }
    if (edge.from != expected_from) {
      throw std::invalid_argument(
              "complete replay Edges are not one directed chain in persisted order");
    }
    if (!visited_nodes.insert(edge.to).second) {
      throw std::invalid_argument(
              "complete replay graph contains a cycle or repeated Node");
    }
    result.ordered_edge_ids.push_back(edge.id);
    expected_from = edge.to;
  }
  if (result.start_node_id == result.end_node_id) {
    throw std::invalid_argument("complete replay graph must be an open chain");
  }

  // Reuse the canonical validator as a final defensive check.  The helper is
  // intentionally stricter (all persisted Edges, no cycle) than a general
  // named Route, but it must still satisfy the same Node/Edge contract.
  NavigationAuthoring document;
  document.frame_id = graph.frame_id;
  document.graph_fingerprint = routeGraphFingerprint(graph);
  document.routes.push_back(result);
  const NavigationAuthoringValidationResult validation =
    validateNavigationAuthoring(document, graph);
  if (!validation.errors.empty() || validation.route_statuses.size() != 1U ||
    !validation.route_statuses.front().valid ||
    (target == NavigationAuthoringTarget::kAutoware &&
    validation.selected_autoware_route_id != route_id) ||
    (target == NavigationAuthoringTarget::kNav2 &&
    validation.selected_nav2_route_id != route_id))
  {
    throw std::invalid_argument(
            "complete replay Route failed canonical navigation validation");
  }
  return result;
}

void saveNavigationAuthoringJson(
  const std::filesystem::path & path, const NavigationAuthoring & authoring)
{
  if (authoring.schema_version != 1U) {
    throw std::invalid_argument("unsupported navigation authoring schema version");
  }
  std::ofstream stream = openOutput(path);
  stream << "{\n  \"schema_version\":1,\n  \"frame_id\":\"" <<
    jsonEscape(authoring.frame_id) << "\",\n  \"graph_fingerprint\":\"" <<
    jsonEscape(authoring.graph_fingerprint) << "\",\n  \"routes\":[";
  for (std::size_t index = 0U; index < authoring.routes.size(); ++index) {
    if (index > 0U) {stream << ',';}
    const NamedNavigationRoute & route = authoring.routes[index];
    stream << "\n    {\"id\":" << route.id << ",\"name\":\"" << jsonEscape(route.name) <<
      "\",\"target\":\"" << toString(route.target) << "\",\"start_node_id\":" <<
      route.start_node_id << ",\"end_node_id\":" << route.end_node_id <<
      ",\"ordered_edge_ids\":[";
    for (std::size_t edge = 0U; edge < route.ordered_edge_ids.size(); ++edge) {
      if (edge > 0U) {stream << ',';}
      stream << route.ordered_edge_ids[edge];
    }
    stream << "],\"validation_requested\":" <<
      (route.validation_requested ? "true" : "false") <<
      ",\"promotion_requested\":" <<
      (route.promotion_requested ? "true" : "false") << '}';
  }
  stream << "\n  ],\n  \"stop_lines\":[";
  for (std::size_t index = 0U; index < authoring.stop_lines.size(); ++index) {
    if (index > 0U) {stream << ',';}
    const AuthoredStopLine & stop = authoring.stop_lines[index];
    stream << "\n    {\"id\":" << stop.id << ",\"name\":\"" << jsonEscape(stop.name) <<
      "\",\"edge_id\":" << stop.edge_id << ",\"s\":" << stop.s <<
      ",\"width_m\":" << stop.width_m << ",\"anchor\":[" << stop.anchor.x << ',' <<
      stop.anchor.y << ',' << stop.anchor.z << "],\"target\":\"" <<
      toString(stop.target) << "\"}";
  }
  stream << "\n  ]\n}\n";
  stream.close();
  if (!stream) {
    throw std::runtime_error("failed to finish navigation authoring JSON: " + path.string());
  }
}

NavigationAuthoring loadNavigationAuthoringJson(const std::filesystem::path & path)
{
  std::error_code size_error;
  const std::uintmax_t size = std::filesystem::file_size(path, size_error);
  if (!size_error && size > kMaximumDocumentBytes) {
    throw std::runtime_error("navigation authoring JSON exceeds 4 MiB");
  }
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open navigation authoring JSON: " + path.string());
  }
  const std::string input{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  const JsonValue root = JsonParser(input).parse();
  requireType(root, JsonValue::Type::kObject, "root");
  NavigationAuthoring result;
  result.schema_version = static_cast<std::uint32_t>(
    asUnsigned(requireMember(root, "schema_version"), "schema_version"));
  if (result.schema_version != 1U) {
    throw std::runtime_error("unsupported navigation authoring schema version");
  }
  result.frame_id = asString(requireMember(root, "frame_id"), "frame_id");
  result.graph_fingerprint = asString(
    requireMember(root, "graph_fingerprint"), "graph_fingerprint");
  const JsonValue * routes = optionalMember(root, "routes");
  if (routes == nullptr) {routes = optionalMember(root, "named_routes");}
  if (routes != nullptr) {
    requireType(*routes, JsonValue::Type::kArray, "routes");
    if (routes->array.size() > kMaximumRoutes) {
      throw std::runtime_error("navigation authoring routes exceeds 10000 entries");
    }
    for (const JsonValue & value : routes->array) {
      NamedNavigationRoute route;
      route.id = asUnsigned(requireMember(value, "id"), "route.id");
      route.name = asString(requireMember(value, "name"), "route.name");
      route.target = navigationAuthoringTargetFromString(
        asString(requireMember(value, "target"), "route.target"));
      route.start_node_id = asUnsigned(
        requireMember(value, "start_node_id"), "route.start_node_id");
      route.end_node_id = asUnsigned(
        requireMember(value, "end_node_id"), "route.end_node_id");
      const auto edges_member = value.object.find("ordered_edge_ids");
      if (edges_member == value.object.end()) {
        throw std::runtime_error(
                "navigation authoring is missing required field: ordered_edge_ids");
      }
      const JsonValue & edges = edges_member->second;
      requireType(edges, JsonValue::Type::kArray, "route.ordered_edge_ids");
      if (edges.array.size() > kMaximumRouteEdges) {
        throw std::runtime_error(
                "navigation authoring route.ordered_edge_ids exceeds 100000 entries");
      }
      for (const JsonValue & edge : edges.array) {
        route.ordered_edge_ids.push_back(asUnsigned(edge, "route.ordered_edge_ids[]"));
      }
      route.validation_requested = asBoolean(
        requireMember(value, "validation_requested"), "route.validation_requested");
      route.promotion_requested = asBoolean(
        requireMember(value, "promotion_requested"), "route.promotion_requested");
      result.routes.push_back(std::move(route));
    }
  }
  const JsonValue * stop_lines = optionalMember(root, "stop_lines");
  if (stop_lines == nullptr) {stop_lines = optionalMember(root, "stops");}
  if (stop_lines != nullptr) {
    requireType(*stop_lines, JsonValue::Type::kArray, "stop_lines");
    if (stop_lines->array.size() > kMaximumStopLines) {
      throw std::runtime_error("navigation authoring stop_lines exceeds 10000 entries");
    }
    for (const JsonValue & value : stop_lines->array) {
      AuthoredStopLine stop;
      stop.id = asUnsigned(requireMember(value, "id"), "stop_line.id");
      stop.name = asString(requireMember(value, "name"), "stop_line.name");
      stop.edge_id = asUnsigned(requireMember(value, "edge_id"), "stop_line.edge_id");
      stop.s = asDouble(requireMember(value, "s"), "stop_line.s");
      stop.width_m = asDouble(requireMember(value, "width_m"), "stop_line.width_m");
      stop.anchor = asAnchor(requireMember(value, "anchor"));
      stop.target = navigationAuthoringTargetFromString(
        asString(requireMember(value, "target"), "stop_line.target"));
      result.stop_lines.push_back(std::move(stop));
    }
  }
  return result;
}

NavigationAuthoringValidationResult validateNavigationAuthoring(
  const NavigationAuthoring & authoring, const RouteGraph & graph,
  const double maximum_anchor_distance)
{
  if (!(maximum_anchor_distance > 0.0) || !std::isfinite(maximum_anchor_distance)) {
    throw std::invalid_argument("navigation authoring anchor tolerance must be finite and positive");
  }
  NavigationAuthoringValidationResult result;
  result.authoring = authoring;
  if (authoring.schema_version != 1U) {
    addError(result.errors, "unsupported_schema_version");
  }
  if (authoring.frame_id.empty()) {
    addError(result.errors, "frame_id_empty");
  } else if (authoring.frame_id != graph.frame_id) {
    addError(result.errors, "frame_id_mismatch");
  }
  if (authoring.graph_fingerprint.empty()) {
    addError(result.errors, "graph_fingerprint_empty");
  } else if (authoring.graph_fingerprint != routeGraphFingerprint(graph)) {
    addError(result.errors, "graph_fingerprint_mismatch");
  }

  std::set<std::uint64_t> route_ids;
  std::set<std::string> route_names;
  for (const NamedNavigationRoute & route : authoring.routes) {
    NamedNavigationRouteStatus status;
    status.id = route.id;
    if (route.id == 0U || !route_ids.insert(route.id).second) {
      addError(status.errors, "route_id_not_unique_nonzero");
    }
    if (route.name.empty() || route.name.size() > kMaximumNameBytes ||
      !route_names.insert(route.name).second)
    {
      addError(status.errors, "route_name_not_unique_nonempty");
    }
    if (route.start_node_id == 0U || findNode(graph, route.start_node_id) == nullptr) {
      addError(status.errors, "start_node_missing");
    }
    if (route.end_node_id == 0U || findNode(graph, route.end_node_id) == nullptr) {
      addError(status.errors, "end_node_missing");
    }
    if (route.ordered_edge_ids.empty()) {
      addError(status.errors, "ordered_edge_ids_empty");
    }
    std::set<std::uint64_t> visited_edges;
    const RouteEdge * previous = nullptr;
    for (const std::uint64_t edge_id : route.ordered_edge_ids) {
      const RouteEdge * edge = findEdge(graph, edge_id);
      if (edge == nullptr) {
        addError(status.errors, "route_edge_missing:" + std::to_string(edge_id));
        previous = nullptr;
        continue;
      }
      if (!visited_edges.insert(edge_id).second) {
        addError(status.errors, "route_edge_repeated:" + std::to_string(edge_id));
      }
      if (previous != nullptr && previous->to != edge->from) {
        addError(status.errors, "route_edge_chain_disconnected:" + std::to_string(edge_id));
      }
      previous = edge;
    }
    if (!route.ordered_edge_ids.empty()) {
      const RouteEdge * first = findEdge(graph, route.ordered_edge_ids.front());
      const RouteEdge * last = findEdge(graph, route.ordered_edge_ids.back());
      if (first != nullptr && first->from != route.start_node_id) {
        addError(status.errors, "start_node_does_not_match_first_edge");
      }
      if (last != nullptr && last->to != route.end_node_id) {
        addError(status.errors, "end_node_does_not_match_last_edge");
      }
    }
    if (route.promotion_requested && !route.validation_requested) {
      addError(status.errors, "promotion_requires_validation_request");
    }
    status.valid = status.errors.empty();
    status.promotion_eligible =
      status.valid && route.validation_requested && route.promotion_requested;
    result.route_statuses.push_back(std::move(status));
  }

  // Duplicate IDs/names invalidate every occurrence, not only the later one.
  // Lookup-by-ID must never allow the first duplicate to stand in for another
  // invalid object.
  std::map<std::uint64_t, std::size_t> route_id_counts;
  std::map<std::string, std::size_t> route_name_counts;
  for (const NamedNavigationRoute & route : authoring.routes) {
    ++route_id_counts[route.id];
    ++route_name_counts[route.name];
  }
  for (std::size_t index = 0U; index < authoring.routes.size(); ++index) {
    const NamedNavigationRoute & route = authoring.routes[index];
    const std::size_t id_count = route_id_counts[route.id];
    const std::size_t name_count = route_name_counts[route.name];
    if (id_count > 1U || name_count > 1U) {
      NamedNavigationRouteStatus & status = result.route_statuses[index];
      status.valid = false;
      status.promotion_eligible = false;
      if (id_count > 1U) {addError(status.errors, "route_id_duplicate");}
      if (name_count > 1U) {addError(status.errors, "route_name_duplicate");}
      addError(result.errors, "duplicate_route_identity");
    }
  }

  auto select_target = [&](const NavigationAuthoringTarget target) -> std::optional<std::uint64_t> {
      std::vector<std::size_t> candidates;
      for (std::size_t index = 0U; index < authoring.routes.size(); ++index) {
        if (includesTarget(authoring.routes[index].target, target) &&
          result.route_statuses[index].promotion_eligible)
        {
          candidates.push_back(index);
        }
      }
      if (candidates.size() == 1U) {
        return authoring.routes[candidates.front()].id;
      }
      if (candidates.size() > 1U) {
        const std::string target_name = toString(target);
        addError(result.errors, "multiple_promotion_requests_for_target:" + target_name);
        for (const std::size_t index : candidates) {
          addError(
            result.route_statuses[index].errors,
            "multiple_promotion_requests_for_target:" + target_name);
          result.route_statuses[index].valid = false;
          result.route_statuses[index].promotion_eligible = false;
        }
      }
      return std::nullopt;
    };
  result.selected_autoware_route_id = select_target(NavigationAuthoringTarget::kAutoware);
  result.selected_nav2_route_id = select_target(NavigationAuthoringTarget::kNav2);

  // A stale document must never select Edge IDs from another graph revision.
  if (!result.errors.empty()) {
    result.selected_autoware_route_id.reset();
    result.selected_nav2_route_id.reset();
    for (NamedNavigationRouteStatus & status : result.route_statuses) {
      status.valid = false;
      status.promotion_eligible = false;
      addError(status.errors, "authoring_document_invalid");
    }
  }

  std::set<std::uint64_t> stop_ids;
  for (const AuthoredStopLine & stop : authoring.stop_lines) {
    AuthoredStopLineStatus status;
    status.id = stop.id;
    if (stop.id == 0U || !stop_ids.insert(stop.id).second) {
      addError(status.errors, "stop_line_id_not_unique_nonzero");
    }
    if (stop.name.empty() || stop.name.size() > kMaximumNameBytes) {
      addError(status.errors, "stop_line_name_empty");
    }
    const RouteEdge * edge = findEdge(graph, stop.edge_id);
    if (edge == nullptr) {
      addError(status.errors, "stop_line_edge_missing");
    } else {
      const double length = polylineLength(edge->centerline);
      if (stop.s < -kArcTolerance || stop.s > length + kArcTolerance) {
        addError(status.errors, "stop_line_s_out_of_range");
      } else if (finite(stop.anchor)) {
        const Vec3 expected = sampleAtArcLength(*edge, stop.s);
        if (distance3d(expected, stop.anchor) > maximum_anchor_distance) {
          addError(status.errors, "stop_line_anchor_mismatch");
        }
      }
    }
    if (!(stop.width_m > 0.0) || stop.width_m > kMaximumStopLineWidth ||
      !std::isfinite(stop.width_m))
    {
      addError(status.errors, "stop_line_width_not_positive_finite");
    }
    if (!finite(stop.anchor)) {
      addError(status.errors, "stop_line_anchor_not_finite");
    }
    const bool belongs_to_compatible_route = std::any_of(
      authoring.routes.begin(), authoring.routes.end(),
      [&](const NamedNavigationRoute & route) {
        const bool target_overlap =
          route.target == NavigationAuthoringTarget::kBoth ||
          stop.target == NavigationAuthoringTarget::kBoth ||
          route.target == stop.target;
        return target_overlap &&
               std::find(
          route.ordered_edge_ids.begin(), route.ordered_edge_ids.end(), stop.edge_id) !=
               route.ordered_edge_ids.end();
      });
    if (!belongs_to_compatible_route) {
      addError(status.errors, "stop_line_edge_not_in_compatible_route");
    }
    status.valid = status.errors.empty();
    result.stop_line_statuses.push_back(std::move(status));
  }
  std::map<std::uint64_t, std::size_t> stop_id_counts;
  for (const AuthoredStopLine & stop : authoring.stop_lines) {
    ++stop_id_counts[stop.id];
  }
  for (std::size_t index = 0U; index < authoring.stop_lines.size(); ++index) {
    const AuthoredStopLine & stop = authoring.stop_lines[index];
    const std::size_t count = stop_id_counts[stop.id];
    if (count > 1U) {
      AuthoredStopLineStatus & status = result.stop_line_statuses[index];
      status.valid = false;
      addError(status.errors, "stop_line_id_duplicate");
      addError(result.errors, "duplicate_stop_line_id");
    }
  }
  for (std::size_t index = 0U; index < authoring.stop_lines.size(); ++index) {
    if (result.stop_line_statuses[index].valid) {continue;}
    const NavigationAuthoringTarget target = authoring.stop_lines[index].target;
    if (includesTarget(target, NavigationAuthoringTarget::kAutoware)) {
      result.autoware_stop_lines_valid = false;
      result.selected_autoware_route_id.reset();
      addError(result.errors, "invalid_stop_line_for_target:autoware");
    }
    if (includesTarget(target, NavigationAuthoringTarget::kNav2)) {
      result.nav2_stop_lines_valid = false;
      result.selected_nav2_route_id.reset();
      addError(result.errors, "invalid_stop_line_for_target:nav2");
    }
  }
  return result;
}

void applyOperationalGraphSafetyValidation(
  NavigationAuthoringValidationResult & validation,
  const RouteGraph & operational_graph)
{
  validation.selected_autoware_route_id.reset();
  validation.selected_nav2_route_id.reset();
  for (std::size_t index = 0U; index < validation.authoring.routes.size(); ++index) {
    const NamedNavigationRoute & route = validation.authoring.routes[index];
    NamedNavigationRouteStatus & status = validation.route_statuses[index];
    if (!status.promotion_eligible) {
      continue;
    }
    try {
      const RouteGraph selected = selectNamedNavigationRouteGraph(operational_graph, route);
      const auto impassable = std::find_if(
        selected.edges.begin(), selected.edges.end(),
        [](const RouteEdge & edge) {return !edge.passable;});
      if (impassable != selected.edges.end()) {
        throw std::invalid_argument(
                "Route Edge is not passable: " + std::to_string(impassable->id));
      }
    } catch (const std::exception & exception) {
      status.valid = false;
      status.promotion_eligible = false;
      addError(status.errors, std::string{"operational_graph_rejected:"} + exception.what());
      addError(
        validation.errors,
        "route_rejected_by_operational_graph:" + std::to_string(route.id));
    }
  }

  const auto select_target = [&](const NavigationAuthoringTarget target) {
      std::optional<std::uint64_t> selected;
      for (std::size_t index = 0U; index < validation.authoring.routes.size(); ++index) {
        const NamedNavigationRoute & route = validation.authoring.routes[index];
        if (!includesTarget(route.target, target) ||
          !validation.route_statuses[index].promotion_eligible)
        {
          continue;
        }
        if (selected) {
          addError(
            validation.errors,
            "multiple_operational_promotion_requests_for_target:" +
            std::string{toString(target)});
          return std::optional<std::uint64_t>{};
        }
        selected = route.id;
      }
      return selected;
    };
  validation.selected_autoware_route_id =
    select_target(NavigationAuthoringTarget::kAutoware);
  validation.selected_nav2_route_id = select_target(NavigationAuthoringTarget::kNav2);
  if (!validation.autoware_stop_lines_valid) {
    validation.selected_autoware_route_id.reset();
  }
  if (!validation.nav2_stop_lines_valid) {
    validation.selected_nav2_route_id.reset();
  }
}

void applyVirtualStopLineProductionPolicy(
  NavigationAuthoringValidationResult & validation,
  const NavigationAuthoringTarget target)
{
  const NamedNavigationRoute * selected = selectedNamedNavigationRoute(validation, target);
  if (selected == nullptr ||
    expectedStopLineCountForRoute(validation, target, *selected) == 0U)
  {
    return;
  }
  const std::uint64_t selected_id = selected->id;
  if (target == NavigationAuthoringTarget::kAutoware) {
    validation.selected_autoware_route_id.reset();
  } else if (target == NavigationAuthoringTarget::kNav2) {
    validation.selected_nav2_route_id.reset();
  }
  const auto status = std::find_if(
    validation.route_statuses.begin(), validation.route_statuses.end(),
    [&](const NamedNavigationRouteStatus & value) {return value.id == selected_id;});
  if (status != validation.route_statuses.end()) {
    addError(status->errors, "authored_virtual_stop_line_not_physically_verified");
    // Status is shared by both targets.  Keep it eligible if the other target
    // remains valid (for example, an Autoware-only stop on a `both` Route),
    // but invalidate it once no target can promote this Route.
    const bool selected_for_other_target =
      (validation.selected_autoware_route_id &&
      *validation.selected_autoware_route_id == selected_id) ||
      (validation.selected_nav2_route_id && *validation.selected_nav2_route_id == selected_id);
    if (!selected_for_other_target) {
      status->valid = false;
      status->promotion_eligible = false;
    }
  }
  addError(
    validation.errors,
    std::string{"authored_virtual_stop_line_not_physically_verified:"} + toString(target));
}

const NamedNavigationRoute * selectedNamedNavigationRoute(
  const NavigationAuthoringValidationResult & validation,
  const NavigationAuthoringTarget target)
{
  const std::optional<std::uint64_t> id = target == NavigationAuthoringTarget::kAutoware ?
    validation.selected_autoware_route_id : validation.selected_nav2_route_id;
  if (!id) {return nullptr;}
  const auto found = std::find_if(
    validation.authoring.routes.begin(), validation.authoring.routes.end(),
    [&](const NamedNavigationRoute & route) {return route.id == *id;});
  return found == validation.authoring.routes.end() ? nullptr : &*found;
}

bool hasPromotionRequest(
  const NavigationAuthoring & authoring, const NavigationAuthoringTarget target)
{
  return std::any_of(
    authoring.routes.begin(), authoring.routes.end(),
    [&](const NamedNavigationRoute & route) {
      return route.promotion_requested && includesTarget(route.target, target);
    });
}

RouteGraph selectNamedNavigationRouteGraph(
  const RouteGraph & graph, const NamedNavigationRoute & route)
{
  NavigationAuthoring single;
  single.frame_id = graph.frame_id;
  single.graph_fingerprint = routeGraphFingerprint(graph);
  single.routes = {route};
  single.routes.front().validation_requested = true;
  single.routes.front().promotion_requested = true;
  const NavigationAuthoringValidationResult validation =
    validateNavigationAuthoring(single, graph);
  if (validation.route_statuses.empty() || !validation.route_statuses.front().valid) {
    std::string message = "named Route is not a valid ordered chain";
    if (!validation.route_statuses.empty() && !validation.route_statuses.front().errors.empty()) {
      message += ": " + validation.route_statuses.front().errors.front();
    }
    throw std::invalid_argument(message);
  }
  RouteGraph result;
  result.frame_id = graph.frame_id;
  std::set<std::uint64_t> added_nodes;
  for (const std::uint64_t edge_id : route.ordered_edge_ids) {
    const RouteEdge * edge = findEdge(graph, edge_id);
    result.edges.push_back(*edge);
    for (const std::uint64_t node_id : {edge->from, edge->to}) {
      if (added_nodes.insert(node_id).second) {
        result.nodes.push_back(*findNode(graph, node_id));
      }
    }
  }
  return result;
}

NamedNavigationRoute remapNamedNavigationRouteAfterSemantics(
  const RouteGraph & selected_source_graph,
  const NamedNavigationRoute & authored_route,
  const SemanticRouteGraphResult & materialized)
{
  NamedNavigationRoute result = authored_route;
  result.ordered_edge_ids.clear();
  std::uint64_t previous_to = authored_route.start_node_id;
  std::size_t covered_edges = 0U;
  for (const std::uint64_t source_edge_id : authored_route.ordered_edge_ids) {
    const RouteEdge * source_edge = findEdge(selected_source_graph, source_edge_id);
    if (source_edge == nullptr) {
      throw std::runtime_error("selected_source_edge_missing_during_semantic_remap");
    }
    std::vector<SemanticRouteEdgeProvenance> provenance;
    for (const SemanticRouteEdgeProvenance & value : materialized.edge_provenance) {
      if (value.source_edge_id == source_edge_id) {provenance.push_back(value);}
    }
    std::sort(
      provenance.begin(), provenance.end(),
      [](const SemanticRouteEdgeProvenance & lhs,
        const SemanticRouteEdgeProvenance & rhs) {
        return lhs.source_start_s < rhs.source_start_s;
      });
    const double source_length = polylineLength(source_edge->centerline);
    double expected_start_s = 0.0;
    if (provenance.empty()) {
      throw std::runtime_error("selected_source_edge_removed_by_semantics");
    }
    for (const SemanticRouteEdgeProvenance & value : provenance) {
      if (std::abs(value.source_start_s - expected_start_s) > 1.0e-6 ||
        !(value.source_end_s > value.source_start_s))
      {
        throw std::runtime_error("semantic_no_entry_created_selected_route_gap");
      }
      const RouteEdge * edge = findEdge(materialized.graph, value.edge_id);
      if (edge == nullptr || !edge->passable || edge->from != previous_to) {
        throw std::runtime_error("semantic_route_is_not_one_directed_chain");
      }
      result.ordered_edge_ids.push_back(edge->id);
      previous_to = edge->to;
      expected_start_s = value.source_end_s;
      ++covered_edges;
    }
    if (std::abs(expected_start_s - source_length) > 1.0e-6) {
      throw std::runtime_error("semantic_route_does_not_cover_selected_source_edge");
    }
  }
  if (covered_edges == 0U || previous_to != authored_route.end_node_id)
  {
    throw std::runtime_error("semantic_route_is_empty_or_disconnected");
  }
  return result;
}

std::vector<AuthoredStopLine> remapResolvedStopLinesAfterSemantics(
  const RouteGraph & source_graph,
  const std::vector<AuthoredStopLine> & resolved_source_stops,
  const SemanticRouteGraphResult & materialized)
{
  (void)validateLosslessSemanticRouteGraph(source_graph, materialized);
  constexpr double arc_tolerance = 1.0e-7;
  constexpr double anchor_tolerance = 1.0e-6;
  std::vector<AuthoredStopLine> result;
  result.reserve(resolved_source_stops.size());
  for (const AuthoredStopLine & stop : resolved_source_stops) {
    const RouteEdge * source = findEdge(source_graph, stop.edge_id);
    if (source == nullptr) {
      throw std::runtime_error("resolved_stop_source_edge_missing_during_semantic_remap");
    }
    const double source_length = polylineLength(source->centerline);
    if (!std::isfinite(stop.s) || stop.s < -arc_tolerance ||
      stop.s > source_length + arc_tolerance)
    {
      throw std::runtime_error("resolved_stop_source_arc_invalid_during_semantic_remap");
    }
    const double source_s = clamp(stop.s, 0.0, source_length);
    const SemanticRouteEdgeProvenance * selected = nullptr;
    for (const SemanticRouteEdgeProvenance & provenance : materialized.edge_provenance) {
      if (provenance.source_edge_id != stop.edge_id ||
        source_s < provenance.source_start_s - arc_tolerance ||
        source_s > provenance.source_end_s + arc_tolerance)
      {
        continue;
      }
      // At an interior split, bind to the following child. This gives one
      // deterministic Lanelet while preserving exactly the same source point.
      if (source_s < provenance.source_end_s - arc_tolerance ||
        provenance.source_end_s >= source_length - arc_tolerance)
      {
        selected = &provenance;
        break;
      }
    }
    if (selected == nullptr) {
      throw std::runtime_error("resolved_stop_not_covered_by_semantic_source_intervals");
    }
    const RouteEdge * output = findEdge(materialized.graph, selected->edge_id);
    if (output == nullptr) {
      throw std::runtime_error("resolved_stop_semantic_output_edge_missing");
    }
    AuthoredStopLine remapped = stop;
    remapped.edge_id = output->id;
    remapped.s = clamp(
      source_s - selected->source_start_s, 0.0, polylineLength(output->centerline));
    remapped.anchor = sampleAtArcLength(*output, remapped.s);
    if (distance3d(remapped.anchor, stop.anchor) > anchor_tolerance) {
      throw std::runtime_error("resolved_stop_anchor_changed_during_semantic_remap");
    }
    result.push_back(std::move(remapped));
  }
  return result;
}

std::vector<AuthoredStopLine> resolveStopLinesForGraph(
  const NavigationAuthoringValidationResult & validation,
  const RouteGraph & graph,
  const NavigationAuthoringTarget target,
  const NamedNavigationRoute * selected_route,
  const double maximum_anchor_distance)
{
  if (!(maximum_anchor_distance > 0.0) || !std::isfinite(maximum_anchor_distance)) {
    throw std::invalid_argument("stop-line anchor tolerance must be finite and positive");
  }
  std::set<std::uint64_t> selected_source_edges;
  if (selected_route != nullptr) {
    selected_source_edges.insert(
      selected_route->ordered_edge_ids.begin(), selected_route->ordered_edge_ids.end());
  }
  std::vector<AuthoredStopLine> result;
  for (const AuthoredStopLine & stop : validation.authoring.stop_lines) {
    const AuthoredStopLineStatus * status = findStopStatus(validation, stop.id);
    if (status == nullptr || !status->valid || !includesTarget(stop.target, target)) {
      continue;
    }
    if (selected_route != nullptr && selected_source_edges.count(stop.edge_id) == 0U) {
      continue;
    }
    Projection projection;
    if (const RouteEdge * exact = findEdge(graph, stop.edge_id)) {
      const double length = polylineLength(exact->centerline);
      if (stop.s >= -kArcTolerance && stop.s <= length + kArcTolerance) {
        const Vec3 point = sampleAtArcLength(*exact, stop.s);
        const double distance = distance3d(point, stop.anchor);
        projection = {exact, clamp(stop.s, 0.0, length), distance, point, false};
      }
    }
    if (projection.edge == nullptr || projection.distance > maximum_anchor_distance) {
      projection = projectToGraph(stop.anchor, graph);
    }
    if (projection.edge == nullptr || projection.ambiguous ||
      projection.distance > maximum_anchor_distance)
    {
      continue;
    }
    AuthoredStopLine resolved = stop;
    resolved.edge_id = projection.edge->id;
    resolved.s = projection.s;
    resolved.anchor = projection.point;
    result.push_back(std::move(resolved));
  }
  return result;
}

std::size_t expectedStopLineCountForRoute(
  const NavigationAuthoringValidationResult & validation,
  const NavigationAuthoringTarget target,
  const NamedNavigationRoute & selected_route)
{
  std::size_t count = 0U;
  for (std::size_t index = 0U; index < validation.authoring.stop_lines.size(); ++index) {
    if (index >= validation.stop_line_statuses.size() ||
      !validation.stop_line_statuses[index].valid)
    {
      continue;
    }
    const AuthoredStopLine & stop = validation.authoring.stop_lines[index];
    if (!includesTarget(stop.target, target)) {continue;}
    if (std::find(
        selected_route.ordered_edge_ids.begin(), selected_route.ordered_edge_ids.end(),
        stop.edge_id) != selected_route.ordered_edge_ids.end())
    {
      ++count;
    }
  }
  return count;
}

void saveNavigationAuthoringStatusJson(
  const std::filesystem::path & path,
  const NavigationAuthoringValidationResult & validation)
{
  std::ofstream stream = openOutput(path);
  stream << "{\n  \"schema_version\":1,\n  \"frame_id\":\"" <<
    jsonEscape(validation.authoring.frame_id) << "\",\n  \"graph_fingerprint\":\"" <<
    jsonEscape(validation.authoring.graph_fingerprint) <<
    "\",\n  \"autoware\":{\"selected_route_id\":";
  if (validation.selected_autoware_route_id) {stream << *validation.selected_autoware_route_id;}
  else {stream << "null";}
  stream << ",\"promoted\":" << (validation.autoware_promoted ? "true" : "false") <<
    ",\"stop_lines_valid\":" <<
    (validation.autoware_stop_lines_valid ? "true" : "false") <<
    "},\n  \"nav2\":{\"selected_route_id\":";
  if (validation.selected_nav2_route_id) {stream << *validation.selected_nav2_route_id;}
  else {stream << "null";}
  stream << ",\"promoted\":" << (validation.nav2_promoted ? "true" : "false") <<
    ",\"stop_lines_valid\":" <<
    (validation.nav2_stop_lines_valid ? "true" : "false") <<
    "},\n  \"errors\":";
  writeErrors(stream, validation.errors);
  stream << ",\n  \"routes\":[";
  for (std::size_t index = 0U; index < validation.authoring.routes.size(); ++index) {
    if (index > 0U) {stream << ',';}
    const NamedNavigationRoute & route = validation.authoring.routes[index];
    const NamedNavigationRouteStatus * status = findRouteStatus(validation, route.id);
    stream << "\n    {\"id\":" << route.id << ",\"name\":\"" << jsonEscape(route.name) <<
      "\",\"target\":\"" << toString(route.target) << "\",\"valid\":" <<
      (status != nullptr && status->valid ? "true" : "false") <<
      ",\"promotion_eligible\":" <<
      (status != nullptr && status->promotion_eligible ? "true" : "false") <<
      ",\"errors\":";
    writeErrors(stream, status != nullptr ? status->errors : std::vector<std::string>{"missing_status"});
    stream << '}';
  }
  stream << "\n  ],\n  \"stop_lines\":[";
  for (std::size_t index = 0U; index < validation.authoring.stop_lines.size(); ++index) {
    if (index > 0U) {stream << ',';}
    const AuthoredStopLine & stop = validation.authoring.stop_lines[index];
    const AuthoredStopLineStatus * status = findStopStatus(validation, stop.id);
    stream << "\n    {\"id\":" << stop.id << ",\"name\":\"" << jsonEscape(stop.name) <<
      "\",\"target\":\"" << toString(stop.target) << "\",\"valid\":" <<
      (status != nullptr && status->valid ? "true" : "false") << ",\"errors\":";
    writeErrors(stream, status != nullptr ? status->errors : std::vector<std::string>{"missing_status"});
    stream << '}';
  }
  stream << "\n  ]\n}\n";
}

}  // namespace lidar_mobility_map_generator
