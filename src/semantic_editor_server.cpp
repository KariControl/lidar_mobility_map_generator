#include "lidar_mobility_map_generator/exporters.hpp"
#include "lidar_mobility_map_generator/glim_reader.hpp"
#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/pointcloud_io.hpp"
#include "lidar_mobility_map_generator/review_io.hpp"
#include "lidar_mobility_map_generator/route_editor.hpp"
#include "lidar_mobility_map_generator/semantic_editor_web.hpp"
#include "lidar_mobility_map_generator/semantic_map.hpp"
#include "lidar_mobility_map_generator/types.hpp"
#include "lidar_mobility_map_generator/vector_map_source.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

std::atomic<bool> g_running{true};
std::atomic<std::uint64_t> g_api_error_sequence{0U};

void signalHandler(int)
{
  g_running.store(false);
}

struct Options
{
  std::filesystem::path output_directory{"output"};
  std::string bind_address{"127.0.0.1"};
  std::uint16_t port{8765U};
  std::size_t maximum_points{150000U};
  bool open_browser{true};
  bool read_only{false};
  bool dump_context{false};
  std::string editor_mode{"combined"};
  bool save_complete_autoware_route{false};
  bool enable_autoware_one_click_export{false};
  std::string autoware_one_click_session;
  std::string navigation_authoring_command;
  std::string navigation_authoring_scope;
  std::filesystem::path navigation_authoring_request;
  std::string edit_token;
};

bool parseBool(const std::string & value)
{
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::invalid_argument("invalid Boolean value: " + value);
}

std::string makeEditToken()
{
  std::random_device random_device;
  std::uniform_int_distribution<unsigned int> distribution(0U, 255U);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t index = 0U; index < 24U; ++index) {
    stream << std::setw(2) << distribution(random_device);
  }
  return stream.str();
}

Options parseOptions(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto requireValue = [&](const std::string & name) {
        if (index + 1 >= argc) {
          throw std::invalid_argument(name + " requires a value");
        }
        return std::string(argv[++index]);
      };
    if (argument == "--output-directory" || argument == "-o") {
      options.output_directory = requireValue(argument);
    } else if (argument == "--bind") {
      options.bind_address = requireValue(argument);
    } else if (argument == "--port") {
      const unsigned long value = std::stoul(requireValue(argument));
      if (value > 65535UL) {
        throw std::invalid_argument("--port must be between 0 and 65535");
      }
      options.port = static_cast<std::uint16_t>(value);
    } else if (argument == "--max-points") {
      options.maximum_points = static_cast<std::size_t>(std::stoull(requireValue(argument)));
      if (options.maximum_points == 0U) {
        throw std::invalid_argument("--max-points must be positive");
      }
    } else if (argument == "--open-browser") {
      options.open_browser = parseBool(requireValue(argument));
    } else if (argument == "--read-only") {
      options.read_only = parseBool(requireValue(argument));
    } else if (argument == "--dump-context") {
      options.dump_context = true;
    } else if (argument == "--editor-mode") {
      options.editor_mode = requireValue(argument);
      if (options.editor_mode != "combined" && options.editor_mode != "vector_map" &&
        options.editor_mode != "navigation_map")
      {
        throw std::invalid_argument(
                "--editor-mode must be combined, vector_map, or navigation_map");
      }
    } else if (argument == "--save-complete-autoware-route" ||
      argument == "--save-complete-vector-map-route")
    {
      options.save_complete_autoware_route = true;
    } else if (argument == "--enable-autoware-one-click-export" ||
      argument == "--enable-vector-map-one-click-export")
    {
      options.enable_autoware_one_click_export = parseBool(requireValue(argument));
    } else if (argument == "--autoware-one-click-session" ||
      argument == "--vector-map-one-click-session")
    {
      options.autoware_one_click_session = requireValue(argument);
      if (options.autoware_one_click_session.empty() ||
        options.autoware_one_click_session.size() > 128U ||
        !std::all_of(
          options.autoware_one_click_session.begin(),
          options.autoware_one_click_session.end(),
          [](const unsigned char character) {
            return (character >= 'A' && character <= 'Z') ||
                   (character >= 'a' && character <= 'z') ||
                   (character >= '0' && character <= '9') ||
                   character == '-' || character == '_';
          }))
      {
        throw std::invalid_argument(
                argument + " must contain 1..128 ASCII letters, digits, '-' or '_'");
      }
    } else if (argument == "--validate-navigation-authoring" ||
      argument == "--save-navigation-authoring")
    {
      options.navigation_authoring_command =
        argument == "--save-navigation-authoring" ? "save" : "validate";
      options.navigation_authoring_scope = requireValue(argument + " SCOPE");
      options.navigation_authoring_request = requireValue(argument + " SCOPE PATH");
    } else if (argument == "--help" || argument == "-h") {
      std::cout
        << "Usage: semantic_map_editor [options]\n\n"
        << "  -o, --output-directory PATH  Generator output directory\n"
        << "      --bind ADDRESS           Bind address (default: 127.0.0.1)\n"
        << "      --port PORT              HTTP port; 0 selects a free port\n"
        << "      --max-points COUNT       Maximum point-cloud samples sent to the browser\n"
        << "      --open-browser BOOL      Open the editor in the default browser\n"
        << "      --read-only BOOL         Disable the save endpoint\n"
        << "      --dump-context           Print /api/context JSON and exit without HTTP\n"
        << "      --editor-mode MODE       combined, vector_map (Lanelet2), or\n"
        << "                               navigation_map (Nav2)\n"
        << "      --save-complete-vector-map-route\n"
        << "                               Save/promote the complete selected Vector Map Route\n"
        << "                               and exit\n"
        << "      --enable-vector-map-one-click-export BOOL\n"
        << "                               Allow the trusted map_ws helper export request\n"
        << "      --vector-map-one-click-session VALUE\n"
        << "                               Trusted helper handoff session (internal)\n"
        << "      --save-complete-autoware-route\n"
        << "      --enable-autoware-one-click-export BOOL\n"
        << "      --autoware-one-click-session VALUE\n"
        << "                               Compatibility aliases for older integrations\n"
        << "      --validate-navigation-authoring SCOPE PATH\n"
        << "                               Validate one scoped authoring document and exit\n"
        << "      --save-navigation-authoring SCOPE PATH\n"
        << "                               Validate/save one scoped authoring document and exit\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  return options;
}

std::string jsonEscape(const std::string & input)
{
  std::string result;
  result.reserve(input.size());
  for (const unsigned char character : input) {
    switch (character) {
      case '"': result += "\\\""; break;
      case '\\': result += "\\\\"; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default:
        if (character < 0x20U) {
          std::ostringstream encoded;
          encoded << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(character);
          result += encoded.str();
        } else {
          result.push_back(static_cast<char>(character));
        }
        break;
    }
  }
  return result;
}

std::string lower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

lmmg::RouteDirection parseRouteDirection(const std::string & value)
{
  const std::string normalized = lower(value);
  if (normalized == "one_way") {return lmmg::RouteDirection::kOneWay;}
  if (normalized == "bidirectional") {return lmmg::RouteDirection::kBidirectional;}
  throw std::invalid_argument(
          "route direction must be one_way or bidirectional, got: " + value);
}

std::string trim(const std::string & value)
{
  const std::size_t begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const std::size_t end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1U);
}

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  while (begin <= line.size()) {
    const std::size_t end = line.find('\t', begin);
    fields.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {break;}
    begin = end + 1U;
  }
  return fields;
}

void writeVec3(std::ostream & stream, const lmmg::Vec3 & point)
{
  stream << '[' << point.x << ',' << point.y << ',' << point.z << ']';
}

struct Bounds
{
  double minimum_x{std::numeric_limits<double>::infinity()};
  double minimum_y{std::numeric_limits<double>::infinity()};
  double maximum_x{-std::numeric_limits<double>::infinity()};
  double maximum_y{-std::numeric_limits<double>::infinity()};

  void include(const lmmg::Vec3 & point)
  {
    if (!lmmg::finite(point)) {
      return;
    }
    minimum_x = std::min(minimum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_x = std::max(maximum_x, point.x);
    maximum_y = std::max(maximum_y, point.y);
  }

  void finalize()
  {
    if (!std::isfinite(minimum_x) || !std::isfinite(minimum_y) ||
      !std::isfinite(maximum_x) || !std::isfinite(maximum_y))
    {
      minimum_x = minimum_y = -10.0;
      maximum_x = maximum_y = 10.0;
      return;
    }
    const double width = std::max(1.0, maximum_x - minimum_x);
    const double height = std::max(1.0, maximum_y - minimum_y);
    const double padding = 0.05 * std::max(width, height) + 0.5;
    minimum_x -= padding;
    minimum_y -= padding;
    maximum_x += padding;
    maximum_y += padding;
  }
};

struct RouteValidationSummary
{
  bool report_available{false};
  bool navigation_ready{false};
  bool vehicle_dimensions_verified{false};
  std::size_t valid_edges{0U};
  std::size_t warning_edges{0U};
  std::size_t invalid_edges{0U};
  std::vector<std::string> reasons;
  std::string explicit_status;
};

struct NavigationTargetStatus
{
  bool available{false};
  bool enabled{false};
  bool production_ready{false};
  bool closed_course_experimental_ready{false};
  std::size_t closed_course_route_edges{0U};
};

struct NavigationTargetReadinessSummary
{
  bool report_available{false};
  bool generation_complete{false};
  std::string requested_target_mode;
  NavigationTargetStatus nav2;
  NavigationTargetStatus autoware;
};

enum class NavigationAuthoringScope
{
  kEditableTopology,
  kAutowareLosslessReplay,
  kAutowareEditedTopology
};

const char * toString(const NavigationAuthoringScope scope)
{
  switch (scope) {
    case NavigationAuthoringScope::kEditableTopology:
      return "editable_topology";
    case NavigationAuthoringScope::kAutowareLosslessReplay:
      return "autoware_lossless_replay";
    case NavigationAuthoringScope::kAutowareEditedTopology:
      return "autoware_edited_topology";
  }
  throw std::invalid_argument("unknown navigation authoring scope");
}

bool isAutowareAuthoringScope(const NavigationAuthoringScope scope)
{
  return scope == NavigationAuthoringScope::kAutowareLosslessReplay ||
         scope == NavigationAuthoringScope::kAutowareEditedTopology;
}

NavigationAuthoringScope navigationAuthoringScopeFromRequestTarget(
  const std::string & request_target)
{
  const std::size_t query_start = request_target.find('?');
  if (query_start == std::string::npos) {
    // Preserve the pre-scope API for Nav2/topology authoring only.  Autoware
    // replay authoring must always opt into its explicit lossless scope.
    return NavigationAuthoringScope::kEditableTopology;
  }
  const std::string query = request_target.substr(query_start + 1U);
  std::size_t begin = 0U;
  while (begin <= query.size()) {
    const std::size_t end = query.find('&', begin);
    const std::string field = query.substr(
      begin, end == std::string::npos ? end : end - begin);
    const std::size_t separator = field.find('=');
    if (separator != std::string::npos && field.substr(0U, separator) == "scope") {
      const std::string value = field.substr(separator + 1U);
      if (value == "editable_topology") {
        return NavigationAuthoringScope::kEditableTopology;
      }
      if (value == "autoware_lossless_replay") {
        return NavigationAuthoringScope::kAutowareLosslessReplay;
      }
      if (value == "autoware_edited_topology") {
        return NavigationAuthoringScope::kAutowareEditedTopology;
      }
      throw std::invalid_argument("unknown navigation authoring scope: " + value);
    }
    if (end == std::string::npos) {break;}
    begin = end + 1U;
  }
  throw std::invalid_argument("navigation authoring request is missing the scope query parameter");
}

const char * navigationAuthoringFilename(const NavigationAuthoringScope scope)
{
  switch (scope) {
    case NavigationAuthoringScope::kEditableTopology:
      return "navigation_authoring.json";
    case NavigationAuthoringScope::kAutowareLosslessReplay:
      return "navigation_authoring_autoware_replay.json";
    case NavigationAuthoringScope::kAutowareEditedTopology:
      return "navigation_authoring_autoware_topology.json";
  }
  throw std::invalid_argument("unknown navigation authoring scope");
}

std::string unquoteYamlScalar(std::string value)
{
  value = trim(value);
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    return value.substr(1U, value.size() - 2U);
  }
  return value;
}

RouteValidationSummary loadRouteValidationSummary(
  const std::filesystem::path & path,
  const bool append_production_dimension_reason = true)
{
  RouteValidationSummary result;
  std::ifstream stream(path);
  if (!stream) {return result;}
  result.report_available = true;
  bool reading_reasons = false;
  std::string line;
  while (std::getline(stream, line)) {
    const std::string value = trim(line);
    if (value == "reasons:") {
      reading_reasons = true;
      continue;
    }
    if (reading_reasons && value.rfind("- ", 0U) == 0U) {
      std::string reason = trim(value.substr(2U));
      if (reason.size() >= 2U && reason.front() == '"' && reason.back() == '"') {
        reason = reason.substr(1U, reason.size() - 2U);
      }
      result.reasons.push_back(std::move(reason));
      continue;
    }
    const std::size_t separator = value.find(':');
    if (separator == std::string::npos) {continue;}
    reading_reasons = false;
    const std::string key = trim(value.substr(0U, separator));
    const std::string scalar = trim(value.substr(separator + 1U));
    if (key == "navigation_ready") {
      result.navigation_ready = parseBool(scalar);
    } else if (key == "vehicle_dimensions_verified") {
      result.vehicle_dimensions_verified = parseBool(scalar);
    } else if (key == "valid_edges") {
      result.valid_edges = static_cast<std::size_t>(std::stoull(scalar));
    } else if (key == "warning_edges") {
      result.warning_edges = static_cast<std::size_t>(std::stoull(scalar));
    } else if (key == "invalid_edges") {
      result.invalid_edges = static_cast<std::size_t>(std::stoull(scalar));
    } else if (key == "validation_status") {
      result.explicit_status = scalar;
      if (result.explicit_status.size() >= 2U && result.explicit_status.front() == '"' &&
        result.explicit_status.back() == '"')
      {
        result.explicit_status = result.explicit_status.substr(
          1U, result.explicit_status.size() - 2U);
      }
    }
  }
  if (append_production_dimension_reason && !result.vehicle_dimensions_verified) {
    result.reasons.push_back(
      "vehicle dimensions are not verified; edited routes cannot be activated safely");
  }
  return result;
}

std::string routeValidationStatus(const RouteValidationSummary & validation)
{
  if (!validation.explicit_status.empty()) {return validation.explicit_status;}
  if (!validation.report_available) {return "unavailable";}
  if (!validation.navigation_ready) {return "invalid";}
  if (validation.warning_edges > 0U || validation.invalid_edges > 0U) {return "warning";}
  return "valid";
}

NavigationTargetReadinessSummary loadNavigationTargetReadinessSummary(
  const std::filesystem::path & path)
{
  NavigationTargetReadinessSummary result;
  std::ifstream stream(path);
  if (!stream) {return result;}
  result.report_available = true;
  NavigationTargetStatus * target = nullptr;
  std::string line;
  while (std::getline(stream, line)) {
    const std::size_t first = line.find_first_not_of(" \t");
    if (first == std::string::npos) {continue;}
    const std::string value = trim(line);
    if (first == 0U) {
      target = nullptr;
      if (value == "nav2:") {
        target = &result.nav2;
        target->available = true;
        continue;
      }
      if (value == "autoware:") {
        target = &result.autoware;
        target->available = true;
        continue;
      }
      const std::size_t separator = value.find(':');
      if (separator == std::string::npos) {continue;}
      const std::string key = trim(value.substr(0U, separator));
      const std::string scalar = trim(value.substr(separator + 1U));
      if (key == "generation_complete") {
        result.generation_complete = parseBool(scalar);
      } else if (key == "requested_target_mode") {
        result.requested_target_mode = unquoteYamlScalar(scalar);
      }
      continue;
    }
    if (first != 2U || target == nullptr) {continue;}
    const std::size_t separator = value.find(':');
    if (separator == std::string::npos) {continue;}
    const std::string key = trim(value.substr(0U, separator));
    const std::string scalar = trim(value.substr(separator + 1U));
    if (key == "enabled") {
      target->enabled = parseBool(scalar);
    } else if (key == "production_ready") {
      target->production_ready = parseBool(scalar);
    } else if (key == "closed_course_experimental_ready") {
      target->closed_course_experimental_ready = parseBool(scalar);
    } else if (key == "closed_course_route_edges") {
      target->closed_course_route_edges = static_cast<std::size_t>(std::stoull(scalar));
    }
  }
  return result;
}

bool isNewerThan(
  const std::filesystem::path & candidate,
  const std::filesystem::path & reference)
{
  std::error_code error;
  if (!std::filesystem::exists(candidate, error) || error) {return false;}
  if (!std::filesystem::exists(reference, error) || error) {return false;}
  const auto candidate_time = std::filesystem::last_write_time(candidate, error);
  if (error) {return false;}
  const auto reference_time = std::filesystem::last_write_time(reference, error);
  return !error && candidate_time > reference_time;
}

void writeUnvalidatedSemanticOutputs(
  const std::filesystem::path & rules_path,
  const std::filesystem::path & graph_path)
{
  {
    std::ofstream rules(rules_path);
    if (!rules) {throw std::runtime_error("failed to invalidate semantic route rules");}
    rules << "semantic_rules_version: 2\n"
          << "operational_ready: false\n"
          << "validation_status: unvalidated\n"
          << "reason: \"rerun generate_vector_map to validate edited routes\"\n"
          << "edges: []\nsegments: []\n";
  }
  {
    std::ofstream graph(graph_path);
    if (!graph) {throw std::runtime_error("failed to invalidate semantic route graph");}
    graph << "{\"type\":\"FeatureCollection\","
          << "\"name\":\"route_graph_semantic\","
          << "\"operational_ready\":false,"
          << "\"validation_status\":\"unvalidated\","
          << "\"reason\":\"rerun generate_vector_map to validate edited routes\","
          << "\"features\":[]}\n";
  }
}

const lmmg::RouteEdge * findRouteEdge(
  const lmmg::RouteGraph & graph, const std::uint64_t edge_id)
{
  const auto found = std::find_if(
    graph.edges.begin(), graph.edges.end(),
    [edge_id](const lmmg::RouteEdge & edge) {return edge.id == edge_id;});
  return found == graph.edges.end() ? nullptr : &*found;
}

lmmg::Vec3 pointAtRouteArc(const lmmg::RouteEdge & edge, double arc_s)
{
  if (edge.centerline.empty()) {return {};}
  arc_s = std::max(0.0, std::min(lmmg::polylineLength(edge.centerline), arc_s));
  double cumulative = 0.0;
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const double segment = lmmg::distance3d(
      edge.centerline[index - 1U], edge.centerline[index]);
    if (arc_s <= cumulative + segment || index + 1U == edge.centerline.size()) {
      const double ratio = segment > 1.0e-12 ? (arc_s - cumulative) / segment : 0.0;
      const double clamped_ratio = std::max(0.0, std::min(1.0, ratio));
      return edge.centerline[index - 1U] +
             (edge.centerline[index] - edge.centerline[index - 1U]) * clamped_ratio;
    }
    cumulative += segment;
  }
  return edge.centerline.back();
}

void validateExactSemanticSpanGeometry(
  const lmmg::SemanticMap & map, const lmmg::RouteGraph & graph)
{
  lmmg::validateSemanticMap(map, &graph);
  constexpr double anchor_tolerance_m = 1.0e-6;
  for (const lmmg::SemanticFeature & feature : map.features) {
    for (const lmmg::RouteEdgeSpan & span : feature.route_edge_spans) {
      const lmmg::RouteEdge * edge = findRouteEdge(graph, span.edge_id);
      if (edge == nullptr) {
        // validateSemanticMap() reports this with the feature ID. Keep this
        // branch defensive in case that validation contract changes.
        throw std::invalid_argument(
                "semantic span references an unavailable authoring edge " +
                std::to_string(span.edge_id));
      }
      if (!span.start_anchor || !span.end_anchor) {
        throw std::invalid_argument(
                "semantic span must include its exact boundary anchors; feature ID=" +
                std::to_string(feature.id));
      }
      const lmmg::Vec3 expected_start = pointAtRouteArc(*edge, span.start_s);
      const lmmg::Vec3 expected_end = pointAtRouteArc(*edge, span.end_s);
      if (lmmg::distance3d(expected_start, *span.start_anchor) > anchor_tolerance_m ||
        lmmg::distance3d(expected_end, *span.end_anchor) > anchor_tolerance_m)
      {
        throw std::invalid_argument(
                "semantic span distance and saved boundary coordinates differ on authoring "
                "edge " + std::to_string(span.edge_id) + "; feature ID=" +
                std::to_string(feature.id));
      }
    }
  }
}

void writeNavigationAuthoring(
  std::ostream & stream, const lmmg::NavigationAuthoring & document)
{
  stream << std::setprecision(12);
  stream << "{\"schema_version\":" << document.schema_version
         << ",\"frame_id\":\"" << jsonEscape(document.frame_id) << "\""
         << ",\"graph_fingerprint\":\"" << jsonEscape(document.graph_fingerprint) << "\""
         << ",\"routes\":[";
  for (std::size_t index = 0U; index < document.routes.size(); ++index) {
    if (index > 0U) {stream << ',';}
    const lmmg::NamedNavigationRoute & route = document.routes[index];
    stream << "{\"id\":" << route.id
           << ",\"name\":\"" << jsonEscape(route.name)
           << "\",\"target\":\"" << lmmg::toString(route.target)
           << "\",\"start_node_id\":" << route.start_node_id
           << ",\"end_node_id\":" << route.end_node_id
           << ",\"ordered_edge_ids\":[";
    for (std::size_t edge_index = 0U;
      edge_index < route.ordered_edge_ids.size(); ++edge_index)
    {
      if (edge_index > 0U) {stream << ',';}
      stream << route.ordered_edge_ids[edge_index];
    }
    stream << "],\"validation_requested\":"
           << (route.validation_requested ? "true" : "false")
           << ",\"promotion_requested\":"
           << (route.promotion_requested ? "true" : "false") << '}';
  }
  stream << "],\"stop_lines\":[";
  for (std::size_t index = 0U; index < document.stop_lines.size(); ++index) {
    if (index > 0U) {stream << ',';}
    const lmmg::AuthoredStopLine & line = document.stop_lines[index];
    stream << "{\"id\":" << line.id
           << ",\"name\":\"" << jsonEscape(line.name)
           << "\",\"edge_id\":" << line.edge_id
           << ",\"s\":" << line.s
           << ",\"width_m\":" << line.width_m
           << ",\"anchor\":";
    writeVec3(stream, line.anchor);
    stream << ",\"target\":\"" << lmmg::toString(line.target) << "\"}";
  }
  stream << "]}";
}

std::string navigationAuthoringJson(const lmmg::NavigationAuthoring & document)
{
  std::ostringstream stream;
  writeNavigationAuthoring(stream, document);
  return stream.str();
}

struct NavigationApiValidation
{
  lmmg::NavigationAuthoringValidationResult core;
  bool structural_valid{false};
  bool stale{false};
  bool promotion_eligible{false};
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

NavigationApiValidation makeNavigationApiValidation(
  const lmmg::NavigationAuthoring & document,
  const lmmg::RouteGraph & graph,
  const bool graph_navigation_ready,
  const NavigationAuthoringScope scope,
  const bool graph_available = true,
  const std::string & graph_unavailable_reason = {})
{
  NavigationApiValidation result;
  result.core = lmmg::validateNavigationAuthoring(document, graph);
  result.errors = result.core.errors;
  result.stale = document.graph_fingerprint != lmmg::routeGraphFingerprint(graph);
  bool all_entities_valid = true;
  for (const lmmg::NamedNavigationRouteStatus & status : result.core.route_statuses) {
    all_entities_valid = all_entities_valid && status.valid;
    for (const std::string & error : status.errors) {
      result.errors.push_back("route " + std::to_string(status.id) + ": " + error);
    }
  }
  for (const lmmg::AuthoredStopLineStatus & status : result.core.stop_line_statuses) {
    all_entities_valid = all_entities_valid && status.valid;
    for (const std::string & error : status.errors) {
      result.errors.push_back("stop line " + std::to_string(status.id) + ": " + error);
    }
  }
  const lmmg::NavigationAuthoringTarget required_target =
    isAutowareAuthoringScope(scope) ?
    lmmg::NavigationAuthoringTarget::kAutoware : lmmg::NavigationAuthoringTarget::kNav2;
  const auto require_scope_target = [&](const lmmg::NavigationAuthoringTarget target,
    const std::string & entity) {
      if (target != required_target) {
        result.errors.push_back(
          entity + " target must be " + lmmg::toString(required_target) +
          " in " + toString(scope) + " scope; cross-scope/both authoring is forbidden");
        all_entities_valid = false;
      }
    };
  for (const lmmg::NamedNavigationRoute & route : document.routes) {
    require_scope_target(route.target, "route " + std::to_string(route.id));
  }
  for (const lmmg::AuthoredStopLine & line : document.stop_lines) {
    require_scope_target(line.target, "stop line " + std::to_string(line.id));
  }
  if (!graph_available) {
    result.errors.push_back(
      std::string(toString(scope)) + " graph is unavailable: " + graph_unavailable_reason);
    all_entities_valid = false;
  }
  result.structural_valid = result.core.errors.empty() && result.errors.empty() &&
    all_entities_valid;
  const bool selected_for_promotion =
    result.core.selected_autoware_route_id.has_value() ||
    result.core.selected_nav2_route_id.has_value();
  result.promotion_eligible = result.structural_valid && graph_navigation_ready &&
    selected_for_promotion;
  if (!graph_navigation_ready && !document.routes.empty()) {
    result.warnings.push_back(
      "current graph is not navigation_ready; Generator safety validation must run");
  }
  if (result.stale) {
    result.warnings.push_back(
      "graph fingerprint mismatch: review against the current graph before saving");
  }
  return result;
}

std::string navigationValidationJson(const NavigationApiValidation & validation)
{
  std::ostringstream stream;
  stream << "{\"structural_valid\":"
         << (validation.structural_valid ? "true" : "false")
         << ",\"stale\":" << (validation.stale ? "true" : "false")
         << ",\"promotion_eligible\":"
         << (validation.promotion_eligible ? "true" : "false") << ",\"errors\":[";
  for (std::size_t index = 0U; index < validation.errors.size(); ++index) {
    if (index > 0U) {stream << ',';}
    stream << '"' << jsonEscape(validation.errors[index]) << '"';
  }
  stream << "],\"warnings\":[";
  for (std::size_t index = 0U; index < validation.warnings.size(); ++index) {
    if (index > 0U) {stream << ',';}
    stream << '"' << jsonEscape(validation.warnings[index]) << '"';
  }
  stream << "]}";
  return stream.str();
}

std::string navigationAuthoringResponseJson(
  const lmmg::NavigationAuthoring & document,
  const NavigationApiValidation & validation,
  const NavigationAuthoringScope scope,
  const bool graph_available,
  const bool exact_lossless,
  const std::string & source_artifact,
  const bool saved,
  const bool generated_outputs_stale = false)
{
  return std::string{"{\"ok\":true,\"saved\":"} + (saved ? "true" : "false") +
         ",\"scope\":\"" + toString(scope) + "\",\"allowed_target\":\"" +
         (isAutowareAuthoringScope(scope) ? "autoware" : "nav2") +
         "\",\"graph_available\":" + (graph_available ? "true" : "false") +
         ",\"exact_lossless\":" + (exact_lossless ? "true" : "false") +
         ",\"source_artifact\":\"" + jsonEscape(source_artifact) +
         "\",\"document\":" + navigationAuthoringJson(document) +
         ",\"validation\":" + navigationValidationJson(validation) +
         (generated_outputs_stale ? ",\"generated_outputs_stale\":true" : "") + "}";
}

lmmg::NavigationAuthoring loadNavigationAuthoringRequest(
  const std::filesystem::path & request_path, const std::string & request_body)
{
  if (request_body.empty()) {
    throw std::invalid_argument("navigation authoring document is empty");
  }
  if (request_body.size() > 4U * 1024U * 1024U) {
    throw std::invalid_argument("navigation authoring document exceeds 4 MiB");
  }
  {
    std::ofstream stream(request_path, std::ios::binary);
    if (!stream) {throw std::runtime_error("failed to create navigation request file");}
    stream.write(request_body.data(), static_cast<std::streamsize>(request_body.size()));
    if (!stream) {throw std::runtime_error("failed to write navigation request file");}
  }
  try {
    lmmg::NavigationAuthoring result = lmmg::loadNavigationAuthoringJson(request_path);
    std::filesystem::remove(request_path);
    return result;
  } catch (...) {
    std::filesystem::remove(request_path);
    throw;
  }
}

lmmg::SemanticMap updateStableRouteSpanAnchors(
  const lmmg::SemanticMap & source,
  const lmmg::RouteGraph & old_graph,
  const lmmg::RouteGraph & new_graph)
{
  lmmg::SemanticMap result = source;
  for (lmmg::SemanticFeature & feature : result.features) {
    for (lmmg::RouteEdgeSpan & span : feature.route_edge_spans) {
      const lmmg::RouteEdge * old_edge = findRouteEdge(old_graph, span.edge_id);
      const lmmg::RouteEdge * new_edge = findRouteEdge(new_graph, span.edge_id);
      if (old_edge == nullptr || new_edge == nullptr) {continue;}
      const double old_length = lmmg::polylineLength(old_edge->centerline);
      const double new_length = lmmg::polylineLength(new_edge->centerline);
      if (!(old_length > 1.0e-9) || !(new_length > 1.0e-9)) {continue;}
      const double scale = new_length / old_length;
      span.start_s = std::max(0.0, std::min(new_length, span.start_s * scale));
      span.end_s = std::max(0.0, std::min(new_length, span.end_s * scale));
      span.start_anchor = pointAtRouteArc(*new_edge, span.start_s);
      span.end_anchor = pointAtRouteArc(*new_edge, span.end_s);
    }
  }
  return result;
}

std::vector<std::uint64_t> semanticFeaturesWithMissingRouteTargets(
  const lmmg::SemanticMap & map,
  const lmmg::RouteGraph & graph)
{
  std::vector<std::uint64_t> result;
  for (const lmmg::SemanticFeature & feature : map.features) {
    bool missing = false;
    for (const lmmg::RouteEdgeSpan & span : feature.route_edge_spans) {
      missing = missing || findRouteEdge(graph, span.edge_id) == nullptr;
    }
    if (feature.route_edge_spans.empty()) {
      for (const std::uint64_t edge_id : feature.route_edge_ids) {
        missing = missing || findRouteEdge(graph, edge_id) == nullptr;
      }
    }
    if (missing) {result.push_back(feature.id);}
  }
  return result;
}

struct EditorData
{
  std::vector<lmmg::PointXYZI> sampled_points;
  std::size_t original_point_count{0U};
  std::vector<lmmg::TimedPose> raw_trajectory;
  std::vector<lmmg::TimedPose> processed_trajectory;
  lmmg::RouteGraph generated_graph;
  lmmg::RouteGraph graph;
  lmmg::RouteGraph operational_graph;
  lmmg::RouteEditOverlay route_edits;
  lmmg::EditedRouteGraph edited_graph;
  lmmg::SemanticMap semantics;
  lmmg::NavigationAuthoring navigation_authoring;
  std::string navigation_authoring_load_error;
  lmmg::RouteGraph autoware_lossless_replay_graph;
  lmmg::NavigationAuthoring autoware_navigation_authoring;
  std::string autoware_navigation_authoring_load_error;
  lmmg::NavigationAuthoring autoware_topology_navigation_authoring;
  std::string autoware_topology_navigation_authoring_load_error;
  bool autoware_lossless_replay_available{false};
  std::string autoware_lossless_replay_reason{
    "lossless replay review artifacts have not been loaded"};
  lmmg::VectorMapSourceSelection vector_map_source;
  std::string vector_map_source_load_error;
  Bounds bounds;
  bool production_validation_available{false};
  std::string route_validation_status{"unvalidated"};
  bool navigation_ready{false};
  bool vehicle_dimensions_verified{false};
  std::size_t valid_route_edges{0U};
  std::size_t warning_route_edges{0U};
  std::size_t invalid_route_edges{0U};
  std::vector<std::string> route_validation_reasons;
  bool closed_course_validation_available{false};
  bool closed_course_validation_stale{false};
  std::string closed_course_validation_status{"unavailable"};
  bool closed_course_graph_ready{false};
  std::size_t closed_course_valid_edges{0U};
  std::size_t closed_course_warning_edges{0U};
  std::size_t closed_course_invalid_edges{0U};
  std::vector<std::string> closed_course_validation_reasons;
  NavigationTargetReadinessSummary target_readiness;
  bool target_readiness_stale{false};
};

class EditorRepository
{
public:
  explicit EditorRepository(Options options)
  : options_(std::move(options)), editor_html_(lmmg::kSemanticEditorHtml)
  {
    const auto replaceEmbeddedValue = [this](
      const std::string & placeholder, const std::string & value) {
        std::size_t replacements = 0U;
        std::size_t position = 0U;
        while ((position = editor_html_.find(placeholder, position)) != std::string::npos) {
          editor_html_.replace(position, placeholder.size(), value);
          position += value.size();
          ++replacements;
        }
        if (replacements == 0U) {
          throw std::runtime_error(
                  "embedded editor placeholder is missing: " + placeholder);
        }
      };
    replaceEmbeddedValue("__LMMG_CSRF_TOKEN__", options_.edit_token);
    replaceEmbeddedValue("__LMMG_EDITOR_MODE__", options_.editor_mode);
    reload();
  }

  [[nodiscard]] const std::string & editorHtml() const
  {
    return editor_html_;
  }

  [[nodiscard]] bool acceptsEditToken(const std::string & token) const
  {
    return !options_.edit_token.empty() && token == options_.edit_token;
  }

  void reload()
  {
    if (!std::filesystem::exists(options_.output_directory)) {
      throw std::runtime_error(
              "output directory does not exist: " + options_.output_directory.string());
    }

    EditorData loaded;
    const std::filesystem::path generated_geometry_path =
      options_.output_directory / "review_geometry_generated.tsv";
    const std::filesystem::path geometry_path = std::filesystem::exists(generated_geometry_path) ?
      generated_geometry_path : options_.output_directory / "review_geometry.tsv";
    if (std::filesystem::exists(geometry_path)) {
      loaded.generated_graph = lmmg::loadReviewGeometryTsv(geometry_path);
      const std::filesystem::path edits_path = options_.output_directory / "route_edits.tsv";
      lmmg::RouteEditSession edit_session = std::filesystem::exists(edits_path) ?
        lmmg::RouteEditSession(
        loaded.generated_graph, lmmg::loadRouteEditOverlayTsv(edits_path)) :
        lmmg::RouteEditSession(loaded.generated_graph);
      loaded.route_edits = edit_session.overlay();
      loaded.edited_graph = edit_session.editedGraph();
      loaded.graph = loaded.edited_graph.graph;
    } else {
      loaded.generated_graph.frame_id = "map";
      loaded.graph.frame_id = "map";
      loaded.edited_graph.graph.frame_id = "map";
      loaded.route_edits.frame_id = "map";
      std::cerr << "Warning: review_geometry.tsv is missing; route-edge editing is unavailable\n";
    }

    const RouteValidationSummary validation = loadRouteValidationSummary(
      options_.output_directory / "route_validation_report.yaml");
    loaded.production_validation_available = validation.report_available;
    loaded.navigation_ready = validation.navigation_ready;
    loaded.vehicle_dimensions_verified = validation.vehicle_dimensions_verified;
    loaded.valid_route_edges = validation.valid_edges;
    loaded.warning_route_edges = validation.warning_edges;
    loaded.invalid_route_edges = validation.invalid_edges;
    loaded.route_validation_reasons = validation.reasons;
    if (!validation.report_available) {
      loaded.route_validation_status = "unvalidated";
      loaded.route_validation_reasons.push_back(
        "route_validation_report.yaml is missing; rerun generate_vector_map");
    } else {
      loaded.route_validation_status = routeValidationStatus(validation);
    }

    const std::filesystem::path closed_course_report_path =
      options_.output_directory / "route_validation_closed_course_report.yaml";
    const RouteValidationSummary closed_course_validation =
      loadRouteValidationSummary(closed_course_report_path, false);
    loaded.closed_course_validation_available = closed_course_validation.report_available;
    loaded.closed_course_validation_status = routeValidationStatus(closed_course_validation);
    loaded.closed_course_graph_ready = closed_course_validation.navigation_ready;
    loaded.closed_course_valid_edges = closed_course_validation.valid_edges;
    loaded.closed_course_warning_edges = closed_course_validation.warning_edges;
    loaded.closed_course_invalid_edges = closed_course_validation.invalid_edges;
    loaded.closed_course_validation_reasons = closed_course_validation.reasons;
    const std::filesystem::path route_edits_path =
      options_.output_directory / "route_edits.tsv";
    loaded.closed_course_validation_stale =
      validation.explicit_status == "unvalidated" ||
      isNewerThan(route_edits_path, closed_course_report_path);

    const std::filesystem::path target_readiness_path =
      options_.output_directory / "navigation_target_readiness.yaml";
    loaded.target_readiness = loadNavigationTargetReadinessSummary(target_readiness_path);
    loaded.target_readiness_stale = loaded.target_readiness.report_available &&
      (!loaded.target_readiness.generation_complete ||
      validation.explicit_status == "unvalidated" ||
      isNewerThan(route_edits_path, target_readiness_path) ||
      isNewerThan(
        options_.output_directory / "semantic_features.tsv", target_readiness_path) ||
      isNewerThan(
        options_.output_directory / "semantic_features_autoware_topology.tsv",
        target_readiness_path) ||
      isNewerThan(
        options_.output_directory / "vector_map_source.tsv", target_readiness_path) ||
      isNewerThan(
        options_.output_directory / "navigation_authoring.json", target_readiness_path) ||
      isNewerThan(
        options_.output_directory / "navigation_authoring_autoware_replay.json",
        target_readiness_path) ||
      isNewerThan(
        options_.output_directory / "navigation_authoring_autoware_topology.json",
        target_readiness_path));
    const std::filesystem::path validated_geometry_path =
      options_.output_directory / "review_geometry_validated.tsv";
    if (validation.report_available && validation.explicit_status != "unvalidated" &&
      std::filesystem::exists(validated_geometry_path))
    {
      loaded.operational_graph = lmmg::loadReviewGeometryTsv(validated_geometry_path);
    } else {
      loaded.operational_graph.frame_id = loaded.graph.frame_id;
    }

    // Navigation authoring is intentionally split by graph identity. Nav2,
    // the lossless recorded-trajectory Vector Map, and the edited-topology
    // Vector Map each keep an independent document. There is no implicit
    // projection between these scopes.
    const std::filesystem::path replay_source_path =
      options_.output_directory / "review_geometry_closed_course_replay_candidate.tsv";
    const std::filesystem::path autoware_replay_path =
      options_.output_directory / "review_geometry_autoware_replay_candidate.tsv";
    loaded.autoware_lossless_replay_graph.frame_id = loaded.graph.frame_id.empty() ?
      "map" : loaded.graph.frame_id;
    if (!std::filesystem::exists(replay_source_path) ||
      !std::filesystem::exists(autoware_replay_path))
    {
      loaded.autoware_lossless_replay_reason =
        "review_geometry_closed_course_replay_candidate.tsv and "
        "review_geometry_autoware_replay_candidate.tsv are both required";
    } else {
      try {
        const lmmg::RouteGraph source_replay = lmmg::loadReviewGeometryTsv(replay_source_path);
        lmmg::RouteGraph autoware_replay = lmmg::loadReviewGeometryTsv(autoware_replay_path);
        const std::string source_fingerprint = lmmg::routeGraphFingerprint(source_replay);
        const std::string autoware_fingerprint = lmmg::routeGraphFingerprint(autoware_replay);
        if (source_replay.edges.empty() || autoware_replay.edges.empty()) {
          loaded.autoware_lossless_replay_reason =
            "lossless replay graph is empty";
        } else if (source_replay.frame_id != autoware_replay.frame_id) {
          loaded.autoware_lossless_replay_reason =
            "closed-course replay and Autoware replay frames differ";
        } else if (source_fingerprint != autoware_fingerprint) {
          loaded.autoware_lossless_replay_reason =
            "Autoware replay differs from the lossless chronological replay; "
            "authoring is blocked instead of projecting or shortening the Route";
        } else {
          loaded.autoware_lossless_replay_graph = std::move(autoware_replay);
          loaded.autoware_lossless_replay_available = true;
          loaded.autoware_lossless_replay_reason.clear();
        }
      } catch (const std::exception & exception) {
        loaded.autoware_lossless_replay_reason =
          std::string{"failed to load lossless replay artifacts: "} + exception.what();
      }
    }

    loaded.vector_map_source.source = lmmg::VectorMapCenterlineSource::kRecordedTrajectory;
    loaded.vector_map_source.frame_id = loaded.autoware_lossless_replay_graph.frame_id.empty() ?
      (loaded.graph.frame_id.empty() ? "map" : loaded.graph.frame_id) :
      loaded.autoware_lossless_replay_graph.frame_id;
    loaded.vector_map_source.graph_fingerprint = lmmg::routeGraphFingerprint(
      loaded.autoware_lossless_replay_graph);
    const std::filesystem::path vector_map_source_path =
      options_.output_directory / "vector_map_source.tsv";
    if (std::filesystem::exists(vector_map_source_path)) {
      try {
        loaded.vector_map_source = lmmg::loadVectorMapSourceSelection(
          vector_map_source_path);
        lmmg::validateVectorMapSourceSelection(
          loaded.vector_map_source, loaded.autoware_lossless_replay_graph,
          loaded.graph);
      } catch (const std::exception & exception) {
        loaded.vector_map_source_load_error = exception.what();
      }
    }

    loaded.navigation_authoring.frame_id = loaded.graph.frame_id.empty() ?
      "map" : loaded.graph.frame_id;
    loaded.navigation_authoring.graph_fingerprint = lmmg::routeGraphFingerprint(loaded.graph);
    const std::filesystem::path navigation_authoring_path =
      options_.output_directory / "navigation_authoring.json";
    if (std::filesystem::exists(navigation_authoring_path)) {
      try {
        loaded.navigation_authoring = lmmg::loadNavigationAuthoringJson(
          navigation_authoring_path);
      } catch (const std::exception & exception) {
        loaded.navigation_authoring_load_error = exception.what();
      }
    }
    loaded.autoware_navigation_authoring.frame_id =
      loaded.autoware_lossless_replay_graph.frame_id.empty() ?
      "map" : loaded.autoware_lossless_replay_graph.frame_id;
    loaded.autoware_navigation_authoring.graph_fingerprint =
      lmmg::routeGraphFingerprint(loaded.autoware_lossless_replay_graph);
    const std::filesystem::path autoware_navigation_authoring_path =
      options_.output_directory / "navigation_authoring_autoware_replay.json";
    if (std::filesystem::exists(autoware_navigation_authoring_path)) {
      try {
        loaded.autoware_navigation_authoring = lmmg::loadNavigationAuthoringJson(
          autoware_navigation_authoring_path);
      } catch (const std::exception & exception) {
        loaded.autoware_navigation_authoring_load_error = exception.what();
      }
    }
    loaded.autoware_topology_navigation_authoring.frame_id =
      loaded.graph.frame_id.empty() ? "map" : loaded.graph.frame_id;
    loaded.autoware_topology_navigation_authoring.graph_fingerprint =
      lmmg::routeGraphFingerprint(loaded.graph);
    const std::filesystem::path autoware_topology_navigation_authoring_path =
      options_.output_directory / "navigation_authoring_autoware_topology.json";
    if (std::filesystem::exists(autoware_topology_navigation_authoring_path)) {
      try {
        loaded.autoware_topology_navigation_authoring =
          lmmg::loadNavigationAuthoringJson(autoware_topology_navigation_authoring_path);
      } catch (const std::exception & exception) {
        loaded.autoware_topology_navigation_authoring_load_error = exception.what();
      }
    }

    const std::filesystem::path pointcloud_path = options_.output_directory / "pointcloud_map.pcd";
    if (std::filesystem::exists(pointcloud_path)) {
      const std::vector<lmmg::PointXYZI> all_points = lmmg::loadPointCloudFile(pointcloud_path);
      loaded.original_point_count = all_points.size();
      const std::size_t stride = std::max<std::size_t>(
        1U, (all_points.size() + options_.maximum_points - 1U) / options_.maximum_points);
      loaded.sampled_points.reserve((all_points.size() + stride - 1U) / stride);
      for (std::size_t index = 0U; index < all_points.size(); index += stride) {
        loaded.sampled_points.push_back(all_points[index]);
      }
    } else {
      std::cerr << "Warning: pointcloud_map.pcd is missing\n";
    }

    const auto loadTrajectory = [&](const char * filename) {
        const std::filesystem::path path = options_.output_directory / filename;
        return std::filesystem::exists(path) ? lmmg::loadTumTrajectory(path) :
               std::vector<lmmg::TimedPose>{};
      };
    loaded.raw_trajectory = loadTrajectory("trajectory_raw.tum");
    loaded.processed_trajectory = loadTrajectory("trajectory_processed.tum");

    const bool edited_vector_map_source = options_.editor_mode == "vector_map" &&
      loaded.vector_map_source.source == lmmg::VectorMapCenterlineSource::kEditedTopology;
    const std::filesystem::path semantic_path = options_.output_directory /
      (edited_vector_map_source ? "semantic_features_autoware_topology.tsv" :
      "semantic_features.tsv");
    if (std::filesystem::exists(semantic_path)) {
      const lmmg::RouteGraph & semantic_graph =
        options_.editor_mode == "vector_map" && !edited_vector_map_source &&
        loaded.autoware_lossless_replay_available ?
        loaded.autoware_lossless_replay_graph : loaded.graph;
      loaded.semantics = lmmg::loadSemanticMapTsv(semantic_path, &semantic_graph);
    } else {
      loaded.semantics.frame_id = loaded.graph.frame_id.empty() ? "map" : loaded.graph.frame_id;
    }
    if (!loaded.graph.frame_id.empty() && loaded.semantics.frame_id != loaded.graph.frame_id) {
      throw std::runtime_error(
              "semantic map frame '" + loaded.semantics.frame_id +
              "' differs from route graph frame '" + loaded.graph.frame_id + "'");
    }

    for (const lmmg::PointXYZI & point : loaded.sampled_points) {
      loaded.bounds.include(point.position());
    }
    for (const lmmg::RouteEdge & edge : loaded.graph.edges) {
      for (const lmmg::Vec3 & point : edge.centerline) {
        loaded.bounds.include(point);
      }
    }
    for (const lmmg::RouteEdge & edge : loaded.autoware_lossless_replay_graph.edges) {
      for (const lmmg::Vec3 & point : edge.centerline) {
        loaded.bounds.include(point);
      }
    }
    for (const lmmg::TimedPose & pose : loaded.raw_trajectory) {
      loaded.bounds.include(pose.world_from_body.translation);
    }
    for (const lmmg::TimedPose & pose : loaded.processed_trajectory) {
      loaded.bounds.include(pose.world_from_body.translation);
    }
    for (const lmmg::SemanticFeature & feature : loaded.semantics.features) {
      loaded.bounds.include(feature.position);
      for (const lmmg::Vec3 & point : feature.polygon) {
        loaded.bounds.include(point);
      }
    }
    loaded.bounds.finalize();
    data_ = std::move(loaded);
    context_cache_ = makeContextJson();
  }

  [[nodiscard]] const std::string & contextJson() const
  {
    return context_cache_;
  }

  [[nodiscard]] std::string semanticGeoJson() const
  {
    const std::filesystem::path path =
      options_.output_directory / semanticFeaturesGeoJsonFilename();
    if (!std::filesystem::exists(path)) {
      return "{\"type\":\"FeatureCollection\",\"features\":[]}";
    }
    std::ifstream stream(path);
    std::ostringstream content;
    content << stream.rdbuf();
    return content.str();
  }

  [[nodiscard]] std::string saveVectorMapSource(const std::string & request_body)
  {
    requireVectorMapOperation("Vector Map centerline source selection");
    if (options_.editor_mode != "vector_map") {
      throw std::runtime_error(
              "Vector Map centerline source selection is available only in the "
              "dedicated vector-map editor");
    }
    if (options_.read_only) {
      throw std::runtime_error("editor is running in read-only mode");
    }
    const lmmg::VectorMapCenterlineSource source =
      lmmg::vectorMapCenterlineSourceFromString(trim(request_body));
    if (source == lmmg::VectorMapCenterlineSource::kRecordedTrajectory &&
      !data_.autoware_lossless_replay_available)
    {
      throw std::runtime_error(
              "the recorded trajectory cannot be selected: " +
              data_.autoware_lossless_replay_reason);
    }
    const lmmg::RouteGraph & graph =
      source == lmmg::VectorMapCenterlineSource::kEditedTopology ?
      data_.graph : data_.autoware_lossless_replay_graph;
    if (graph.edges.empty()) {
      throw std::runtime_error("the selected Vector Map centerline graph is empty");
    }
    lmmg::VectorMapSourceSelection selection;
    selection.source = source;
    selection.frame_id = graph.frame_id;
    selection.graph_fingerprint = lmmg::routeGraphFingerprint(graph);
    lmmg::validateVectorMapSourceSelection(
      selection, data_.autoware_lossless_replay_graph, data_.graph);
    const std::filesystem::path destination =
      options_.output_directory / "vector_map_source.tsv";
    const std::filesystem::path temporary =
      options_.output_directory / ".vector_map_source.tsv.tmp";
    const std::filesystem::path rollback_temporary =
      options_.output_directory / ".vector_map_source.tsv.rollback.tmp";
    std::optional<std::string> previous_contents;
    if (std::filesystem::exists(destination)) {
      std::ifstream previous_stream(destination, std::ios::binary);
      if (!previous_stream) {
        throw std::runtime_error("failed to read the existing Vector Map centerline source");
      }
      std::ostringstream previous;
      previous << previous_stream.rdbuf();
      if (!previous_stream.eof() && previous_stream.fail()) {
        throw std::runtime_error("failed to finish reading the existing Vector Map centerline source");
      }
      previous_contents = previous.str();
    }
    lmmg::saveVectorMapSourceSelection(temporary, selection);
    atomicReplace(temporary, destination);
    try {
      // Reload also validates the semantic and target-route documents belonging
      // to the newly selected graph. Do not leave a half-switched editor on disk.
      reload();
    } catch (...) {
      const std::exception_ptr reload_failure = std::current_exception();
      try {
        if (previous_contents) {
          std::ofstream rollback(rollback_temporary, std::ios::binary);
          if (!rollback) {
            throw std::runtime_error("failed to create the centerline-source rollback file");
          }
          rollback.write(
            previous_contents->data(),
            static_cast<std::streamsize>(previous_contents->size()));
          rollback.close();
          if (!rollback) {
            throw std::runtime_error("failed to finish the centerline-source rollback file");
          }
          atomicReplace(rollback_temporary, destination);
        } else {
          std::error_code remove_error;
          std::filesystem::remove(destination, remove_error);
          if (remove_error) {
            throw std::runtime_error(
                    "failed to remove the rejected centerline-source selection: " +
                    remove_error.message());
          }
        }
      } catch (const std::exception & rollback_error) {
        throw std::runtime_error(
                std::string{"failed to activate the selected Vector Map centerline and "}
                + "could not restore the previous selection: " + rollback_error.what());
      }
      std::rethrow_exception(reload_failure);
    }
    return context_cache_;
  }

  [[nodiscard]] std::string navigationAuthoringResponse(
    const NavigationAuthoringScope scope) const
  {
    requireNavigationScopeAllowed(scope);
    const lmmg::NavigationAuthoring & document = navigationAuthoringForScope(scope);
    const lmmg::RouteGraph & graph = navigationGraphForScope(scope);
    const bool graph_available = navigationGraphAvailable(scope);
    const bool graph_ready = navigationGraphReady(scope);
    const std::string & load_error = navigationAuthoringLoadErrorForScope(scope);
    NavigationApiValidation validation = makeNavigationApiValidation(
      document, graph, graph_ready, scope, graph_available,
      navigationGraphUnavailableReason(scope));
    if (!load_error.empty()) {
      validation.errors.insert(
        validation.errors.begin(),
        std::string{"failed to load existing "} + navigationAuthoringFilename(scope) +
        ": " + load_error);
      validation.structural_valid = false;
      validation.promotion_eligible = false;
    }
    return navigationAuthoringResponseJson(
      document, validation, scope, graph_available,
      scope == NavigationAuthoringScope::kAutowareLosslessReplay && graph_available,
      navigationGraphSourceArtifact(scope), false);
  }

  [[nodiscard]] std::string validateNavigationAuthoringRequest(
    const std::string & request_body, const NavigationAuthoringScope scope) const
  {
    requireNavigationScopeAllowed(scope);
    const lmmg::RouteGraph & graph = navigationGraphForScope(scope);
    const bool graph_available = navigationGraphAvailable(scope);
    const bool graph_ready = navigationGraphReady(scope);
    const lmmg::NavigationAuthoring document = loadNavigationAuthoringRequest(
      options_.output_directory / ".navigation_authoring.request.json", request_body);
    const NavigationApiValidation validation = makeNavigationApiValidation(
      document, graph, graph_ready, scope, graph_available,
      navigationGraphUnavailableReason(scope));
    return navigationAuthoringResponseJson(
      document, validation, scope, graph_available,
      scope == NavigationAuthoringScope::kAutowareLosslessReplay && graph_available,
      navigationGraphSourceArtifact(scope), false);
  }

  [[nodiscard]] std::string saveNavigationAuthoringRequest(
    const std::string & request_body, const NavigationAuthoringScope scope)
  {
    requireNavigationScopeAllowed(scope);
    if (options_.read_only) {
      throw std::runtime_error("editor is running in read-only mode");
    }
    const lmmg::RouteGraph & graph = navigationGraphForScope(scope);
    const bool graph_available = navigationGraphAvailable(scope);
    const bool graph_ready = navigationGraphReady(scope);
    lmmg::NavigationAuthoring document = loadNavigationAuthoringRequest(
      options_.output_directory /
      (std::string{"."} + navigationAuthoringFilename(scope) + ".request"), request_body);
    const NavigationApiValidation validation = makeNavigationApiValidation(
      document, graph, graph_ready, scope, graph_available,
      navigationGraphUnavailableReason(scope));
    if (!validation.structural_valid) {
      std::ostringstream message;
      message << "navigation authoring structural validation failed";
      for (const std::string & error : validation.errors) {
        message << "; " << error;
      }
      throw std::invalid_argument(message.str());
    }
    const std::filesystem::path temporary =
      options_.output_directory /
      (std::string{"."} + navigationAuthoringFilename(scope) + ".tmp");
    lmmg::saveNavigationAuthoringJson(temporary, document);
    atomicReplace(
      temporary, options_.output_directory / navigationAuthoringFilename(scope));
    navigationAuthoringForScope(scope) = std::move(document);
    clearNavigationAuthoringLoadErrorForScope(scope);
    data_.target_readiness_stale = true;
    context_cache_ = makeContextJson();
    const lmmg::NavigationAuthoring & saved_document = navigationAuthoringForScope(scope);
    return navigationAuthoringResponseJson(
      saved_document, validation, scope, graph_available,
      scope == NavigationAuthoringScope::kAutowareLosslessReplay && graph_available,
      navigationGraphSourceArtifact(scope), true, true);
  }

  [[nodiscard]] std::string saveCompleteAutowareReplayRoute(
    const bool allow_unvalidated_edited_topology = false)
  {
    requireVectorMapOperation("complete Vector Map Mission output");
    if (options_.read_only) {
      throw std::runtime_error("editor is running in read-only mode");
    }
    requireCurrentVectorMapSource();
    const NavigationAuthoringScope scope = selectedVectorMapAuthoringScope();
    const std::string & load_error = navigationAuthoringLoadErrorForScope(scope);
    if (!load_error.empty()) {
      throw std::runtime_error(
              std::string{"existing "} + navigationAuthoringFilename(scope) +
              " is invalid: " + load_error);
    }

    const lmmg::RouteGraph & graph = selectedVectorMapGraph();
    const std::string graph_fingerprint = lmmg::routeGraphFingerprint(graph);
    const std::filesystem::path authoring_path =
      options_.output_directory / navigationAuthoringFilename(scope);
    const bool recorded_trajectory_source =
      scope == NavigationAuthoringScope::kAutowareLosslessReplay;
    // A GUI-created Edge is deliberately marked non-passable until the
    // Generator reruns the vehicle-footprint and point-cloud checks.  Permit
    // that state only for this pre-regeneration authoring handoff.  Recorded
    // replay remains strict, and Generator promotion/staging still require the
    // edited Edge to appear in the validated operational graph.
    const bool require_passable_edges = recorded_trajectory_source ||
      !allow_unvalidated_edited_topology;
    const char * complete_route_name = recorded_trajectory_source ?
      "走行軌跡全体" : "編集した道路中心線全体";
    const std::string authoring_name = navigationAuthoringFilename(scope);
    const std::string source_name = recorded_trajectory_source ?
      "current lossless recorded trajectory" : "current edited topology";
    const auto is_reserved_recorded_trajectory_name = [&](const std::string & name) {
        // Accept names written by older versions so that opening and exporting
        // an existing map migrates it instead of creating a duplicate Route.
        return recorded_trajectory_source ?
               (name == "autoware_full_replay" ||
               name == "recorded_trajectory_full" || name == complete_route_name) :
               (name == "autoware_full_topology" ||
               name == "edited_topology_full" || name == complete_route_name);
      };
    bool migrate_stale_tool_route = false;
    if (std::filesystem::exists(authoring_path) &&
      (navigationAuthoringForScope(scope).frame_id != graph.frame_id ||
      navigationAuthoringForScope(scope).graph_fingerprint != graph_fingerprint))
    {
      const lmmg::NavigationAuthoring & stale = navigationAuthoringForScope(scope);
      if (stale.frame_id != graph.frame_id) {
        throw std::runtime_error(
                "existing " + authoring_name + " is stale for the " + source_name +
                " and uses a different frame; automatic one-click migration is forbidden");
      }
      if (!stale.stop_lines.empty()) {
        throw std::runtime_error(
                "existing " + authoring_name + " is stale for the " + source_name +
                " and contains stop lines; automatic one-click migration is forbidden");
      }
      if (stale.routes.size() != 1U) {
        throw std::runtime_error(
                "existing " + authoring_name + " is stale for the " + source_name +
                " and must contain exactly one Route; "
                "custom/additional or ambiguous reserved Routes require operator review");
      }
      const lmmg::NamedNavigationRoute & route = stale.routes.front();
      if (route.name != complete_route_name) {
        throw std::runtime_error(
                "existing " + authoring_name + " is stale for the " + source_name +
                ", but its only Route is not the unambiguous tool-reserved " +
                complete_route_name + " Route");
      }
      if (route.target != lmmg::NavigationAuthoringTarget::kAutoware) {
        throw std::runtime_error(
                "existing " + authoring_name + " is stale for the " + source_name +
                ", but the " + complete_route_name + " Route target is not autoware; "
                "cross-target migration is forbidden");
      }
      std::vector<std::uint64_t> old_edge_ids = route.ordered_edge_ids;
      std::sort(old_edge_ids.begin(), old_edge_ids.end());
      const bool invalid_edge_identity = old_edge_ids.empty() ||
        old_edge_ids.front() == 0U ||
        std::adjacent_find(old_edge_ids.begin(), old_edge_ids.end()) != old_edge_ids.end();
      if (stale.graph_fingerprint.empty() || route.id == 0U ||
        route.start_node_id == 0U || route.end_node_id == 0U ||
        route.start_node_id == route.end_node_id || invalid_edge_identity ||
        !route.validation_requested || !route.promotion_requested)
      {
        throw std::runtime_error(
                "existing " + authoring_name + " is stale for the " + source_name +
                ", but the tool-reserved " + complete_route_name +
                " Route identity is invalid; automatic migration is forbidden");
      }
      // This is the only stale shape that is safe to migrate without rebinding
      // user intent: one tool-owned complete Route and no other semantics.
      migrate_stale_tool_route = true;
    }
    std::vector<std::uint64_t> complete_edge_ids;
    complete_edge_ids.reserve(graph.edges.size());
    for (const lmmg::RouteEdge & edge : graph.edges) {
      complete_edge_ids.push_back(edge.id);
    }

    lmmg::NavigationAuthoring document = navigationAuthoringForScope(scope);
    document.schema_version = 1U;
    document.frame_id = graph.frame_id;
    document.graph_fingerprint = graph_fingerprint;
    if (migrate_stale_tool_route) {
      const std::uint64_t route_id = document.routes.front().id;
      document.routes.front() = lmmg::makeCompleteOpenChainNavigationRoute(
        graph, route_id, complete_route_name,
        lmmg::NavigationAuthoringTarget::kAutoware,
        require_passable_edges);
    }

    lmmg::NamedNavigationRoute * existing_complete = nullptr;
    for (lmmg::NamedNavigationRoute & route : document.routes) {
      if (route.target != lmmg::NavigationAuthoringTarget::kAutoware) {
        throw std::runtime_error(
                "the selected Vector Map target-route document contains a Route for "
                "another map type");
      }
      if (route.ordered_edge_ids == complete_edge_ids) {
        if (existing_complete != nullptr) {
          throw std::runtime_error(
                  "multiple complete Routes already exist for the selected Vector Map source");
        }
        existing_complete = &route;
      }
    }

    if (existing_complete == nullptr) {
      for (lmmg::NamedNavigationRoute & route : document.routes) {
        if (route.promotion_requested) {
          // This endpoint explicitly selects the complete current centerline.
          // Keep a user-authored Route as a draft instead of forcing deletion.
          route.promotion_requested = false;
        }
        if (is_reserved_recorded_trajectory_name(route.name)) {
          throw std::runtime_error(
                  std::string{"the reserved complete-Route name '"} + complete_route_name +
                  "' is already used by a different chain");
        }
      }
      std::uint64_t route_id = 1U;
      for (const lmmg::NamedNavigationRoute & route : document.routes) {
        if (route.id == std::numeric_limits<std::uint64_t>::max()) {
          throw std::runtime_error("navigation Route ID space is exhausted");
        }
        route_id = std::max(route_id, route.id + 1U);
      }
      document.routes.push_back(lmmg::makeCompleteOpenChainNavigationRoute(
          graph, route_id, complete_route_name,
          lmmg::NavigationAuthoringTarget::kAutoware,
          require_passable_edges));
      existing_complete = &document.routes.back();
    } else {
      for (lmmg::NamedNavigationRoute & route : document.routes) {
        if (&route != existing_complete && route.promotion_requested) {
          route.promotion_requested = false;
        }
      }
      const std::uint64_t route_id = existing_complete->id;
      const std::string route_name =
        is_reserved_recorded_trajectory_name(existing_complete->name) ?
        complete_route_name : existing_complete->name;
      *existing_complete = lmmg::makeCompleteOpenChainNavigationRoute(
        graph, route_id, route_name,
        lmmg::NavigationAuthoringTarget::kAutoware,
        require_passable_edges);
    }

    return saveNavigationAuthoringRequest(
      navigationAuthoringJson(document),
      scope);
  }

  [[nodiscard]] std::string requestAutowareOneClickExport()
  {
    requireVectorMapOperation("one-click Lanelet2 map output");
    const lmmg::RouteGraph & graph = selectedVectorMapGraph();
    const auto [marker, temporary] = prepareTrustedVectorMapExportRequest();
    std::string authoring_response;
    try {
      authoring_response = saveCompleteAutowareReplayRoute(true);
      atomicReplace(temporary, marker);
    } catch (...) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      throw;
    }
    g_running.store(false);
    return std::string{"{\"ok\":true,\"export_requested\":true,"}
           + "\"route_edge_count\":" +
           std::to_string(graph.edges.size()) +
           ",\"centerline_source\":\"" +
           lmmg::toString(data_.vector_map_source.source) + "\"" +
           ",\"authoring\":" + authoring_response + "}";
  }

  [[nodiscard]] std::string requestCurrentVectorMapRouteExport()
  {
    requireVectorMapOperation("current target Route vector-map output");
    if (options_.read_only) {
      throw std::runtime_error("editor is running in read-only mode");
    }
    requireCurrentVectorMapSource();
    const NavigationAuthoringScope scope = selectedVectorMapAuthoringScope();
    const std::string & load_error = navigationAuthoringLoadErrorForScope(scope);
    if (!load_error.empty()) {
      throw std::runtime_error(
              std::string{"existing "} + navigationAuthoringFilename(scope) +
              " is invalid: " + load_error);
    }

    const lmmg::NavigationAuthoring & document = navigationAuthoringForScope(scope);
    const lmmg::RouteGraph & graph = selectedVectorMapGraph();
    const NavigationApiValidation validation = makeNavigationApiValidation(
      document, graph, false, scope,
      true, {});
    const std::size_t promoted_route_count = static_cast<std::size_t>(std::count_if(
        document.routes.begin(), document.routes.end(),
        [](const lmmg::NamedNavigationRoute & route) {
          return route.promotion_requested;
        }));
    if (promoted_route_count != 1U) {
      throw std::runtime_error(
              "current vector-map target Route must contain exactly one promotion request; found " +
              std::to_string(promoted_route_count));
    }
    if (validation.stale) {
      throw std::runtime_error(
              "current vector-map target Route is stale for the selected centerline source; "
              "review and save it against the current graph before regeneration");
    }
    if (!validation.structural_valid) {
      std::ostringstream message;
      message << "current vector-map target Route structural validation failed";
      for (const std::string & error : validation.errors) {
        message << "; " << error;
      }
      throw std::runtime_error(message.str());
    }
    if (!validation.core.selected_autoware_route_id) {
      throw std::runtime_error(
              "current vector-map target Route is not validated and promoted for vector-map output");
    }
    const std::uint64_t selected_id = *validation.core.selected_autoware_route_id;
    const auto selected = std::find_if(
      document.routes.begin(), document.routes.end(),
      [&](const lmmg::NamedNavigationRoute & route) {return route.id == selected_id;});
    if (selected == document.routes.end()) {
      throw std::runtime_error("current vector-map target Route selection is inconsistent");
    }

    const auto [marker, temporary] = prepareTrustedVectorMapExportRequest();
    try {
      atomicReplace(temporary, marker);
    } catch (...) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      throw;
    }
    g_running.store(false);
    return std::string{"{\"ok\":true,\"export_requested\":true,"}
           + "\"route_id\":" + std::to_string(selected->id) +
           ",\"route_name\":\"" + jsonEscape(selected->name) + "\"" +
           ",\"route_edge_count\":" +
           std::to_string(selected->ordered_edge_ids.size()) +
           ",\"centerline_source\":\"" +
           lmmg::toString(data_.vector_map_source.source) + "\"}";
  }

  [[nodiscard]] std::pair<std::size_t, std::string> save(const std::string & tsv)
  {
    if (options_.read_only) {
      throw std::runtime_error("editor is running in read-only mode");
    }
    if (tsv.size() > 32U * 1024U * 1024U) {
      throw std::runtime_error("semantic map request exceeds 32 MiB");
    }

    const std::filesystem::path request_path =
      options_.output_directory / ".semantic_features.request.tsv";
    {
      std::ofstream request(request_path, std::ios::binary);
      if (!request) {
        throw std::runtime_error("failed to create temporary semantic request file");
      }
      request.write(tsv.data(), static_cast<std::streamsize>(tsv.size()));
    }

    const lmmg::RouteGraph & semantic_graph = semanticAuthoringGraph();
    lmmg::SemanticMap candidate;
    try {
      // An editor save is not a stale-graph migration. Preserve the submitted
      // Edge/s values and reject a client bug when their coordinates are not
      // the exact points at those arc lengths. Geometry remapping remains an
      // explicit Generator/reload operation.
      candidate = lmmg::loadSemanticMapTsv(request_path);
      validateExactSemanticSpanGeometry(candidate, semantic_graph);
      std::filesystem::remove(request_path);
    } catch (...) {
      std::filesystem::remove(request_path);
      throw;
    }
    if (!semantic_graph.frame_id.empty() && candidate.frame_id != semantic_graph.frame_id) {
      throw std::runtime_error(
              "semantic map frame must match route graph frame '" + semantic_graph.frame_id + "'");
    }

    const auto temporary = [&](const char * filename) {
        return options_.output_directory / (std::string(".") + filename + ".tmp");
      };
    const auto target = [&](const char * filename) {
        return options_.output_directory / filename;
      };

    const char * semantic_tsv_filename = semanticFeaturesFilename();
    const char * semantic_geojson_filename = semanticFeaturesGeoJsonFilename();
    const std::filesystem::path tsv_temp = temporary(semantic_tsv_filename);
    const std::filesystem::path geojson_temp = temporary(semantic_geojson_filename);
    const std::filesystem::path rules_temp = temporary("semantic_route_rules.yaml");
    const std::filesystem::path graph_temp = temporary("route_graph_semantic.geojson");
    const std::filesystem::path preview_rules_temp = temporary(
      "semantic_route_rules_preview.yaml");
    const std::filesystem::path preview_graph_temp = temporary(
      "route_graph_semantic_preview.geojson");

    lmmg::saveSemanticMapTsv(tsv_temp, candidate);
    lmmg::saveSemanticMapGeoJson(geojson_temp, candidate, semantic_graph);
    lmmg::saveSemanticRouteRulesYaml(preview_rules_temp, candidate, semantic_graph);
    lmmg::saveSemanticRouteGraphGeoJson(preview_graph_temp, candidate, semantic_graph);
    if (options_.editor_mode == "vector_map" ||
      data_.route_validation_status == "unvalidated")
    {
      // Vector Map semantics require the Generator's audited conversion to
      // the selected Lanelet geometry. Never leave an older planner-facing
      // derivative looking current after a GUI save.
      writeUnvalidatedSemanticOutputs(rules_temp, graph_temp);
    } else {
      const lmmg::SemanticGraphFilterResult operational =
        lmmg::filterSemanticMapForGraph(candidate, data_.operational_graph);
      lmmg::saveSemanticRouteRulesYaml(rules_temp, operational.map, data_.operational_graph);
      lmmg::saveSemanticRouteGraphGeoJson(graph_temp, operational.map, data_.operational_graph);
      for (const std::string & diagnostic : operational.diagnostics) {
        std::cerr << "Semantic operational filter: " << diagnostic << '\n';
      }
    }

    atomicReplace(tsv_temp, target(semantic_tsv_filename));
    atomicReplace(geojson_temp, target(semantic_geojson_filename));
    atomicReplace(rules_temp, target("semantic_route_rules.yaml"));
    atomicReplace(graph_temp, target("route_graph_semantic.geojson"));
    atomicReplace(preview_rules_temp, target("semantic_route_rules_preview.yaml"));
    atomicReplace(preview_graph_temp, target("route_graph_semantic_preview.geojson"));

    data_.semantics = std::move(candidate);
    data_.target_readiness_stale = true;
    context_cache_ = makeContextJson();
    return {data_.semantics.features.size(), featuresJson()};
  }

  [[nodiscard]] std::string editRoute(const std::string & request_body)
  {
    if (options_.read_only) {
      throw std::runtime_error("editor is running in read-only mode");
    }
    if (data_.generated_graph.edges.empty()) {
      throw std::runtime_error("generated Route Graph is unavailable");
    }
    ensureGeneratedAuditArtifacts();
    if (request_body.size() > 32U * 1024U * 1024U) {
      throw std::runtime_error("route edit request exceeds 32 MiB");
    }
    std::istringstream input(request_body);
    std::string line;
    if (!std::getline(input, line)) {
      throw std::runtime_error("route edit request is empty");
    }
    const std::vector<std::string> fields = splitTabs(trim(line));
    if (fields.empty()) {throw std::runtime_error("route edit command is empty");}

    const std::string command = lower(fields[0]);
    lmmg::RouteEditOverlay requested_overlay = data_.route_edits;
    if (command == "undo") {
      if (fields.size() != 1U) {throw std::runtime_error("undo takes no arguments");}
      if (requested_overlay.operations.empty()) {
        throw std::runtime_error("there is no Route operation to undo");
      }
      requested_overlay.operations.pop_back();
    } else if (command == "reset") {
      if (fields.size() != 1U) {throw std::runtime_error("reset takes no arguments");}
      requested_overlay = lmmg::RouteEditSession(data_.generated_graph).overlay();
    } else if (command == "clear") {
      if (fields.size() != 1U) {throw std::runtime_error("clear takes no arguments");}
      requested_overlay = lmmg::RouteEditSession(data_.generated_graph).overlay();
    }
    lmmg::RouteEditSession session(data_.generated_graph, std::move(requested_overlay));
    if (command == "undo" || command == "reset") {
      // The replayed overlay is the requested edit.
    } else if (command == "clear") {
      // One overlay operation keeps Undo intuitive while the immutable base
      // fingerprint and Reset continue to protect/restore generated geometry.
      session.clearGraph();
    } else if (command == "add_node") {
      if (fields.size() != 4U) {throw std::runtime_error("add_node requires x, y, z");}
      const double x = parseRouteEditCoordinate(fields[1], "x");
      const double y = parseRouteEditCoordinate(fields[2], "y");
      static_cast<void>(session.addNode({x, y, parseRouteEditElevation(fields[3], x, y)}));
    } else if (command == "move_node") {
      if (fields.size() != 5U) {throw std::runtime_error("move_node requires ID, x, y, z");}
      const double x = parseRouteEditCoordinate(fields[2], "x");
      const double y = parseRouteEditCoordinate(fields[3], "y");
      session.moveNode(
        static_cast<std::uint64_t>(std::stoull(fields[1])),
        {x, y, parseRouteEditElevation(fields[4], x, y)});
    } else if (command == "delete_node") {
      if (fields.size() != 3U) {throw std::runtime_error("delete_node requires ID, cascade");}
      session.deleteNode(
        static_cast<std::uint64_t>(std::stoull(fields[1])), parseBool(fields[2]));
    } else if (command == "add_edge") {
      if (fields.size() != 4U) {
        throw std::runtime_error("add_edge requires from ID, to ID, direction");
      }
      std::vector<lmmg::Vec3> polyline;
      while (std::getline(input, line)) {
        if (trim(line).empty()) {continue;}
        const std::vector<std::string> point = splitTabs(trim(line));
        if (point.size() != 4U || lower(point[0]) != "point") {
          throw std::runtime_error("add_edge geometry records must be POINT, x, y, z");
        }
        const double x = parseRouteEditCoordinate(point[1], "POINT x");
        const double y = parseRouteEditCoordinate(point[2], "POINT y");
        polyline.push_back({x, y, parseRouteEditElevation(point[3], x, y)});
      }
      const lmmg::RouteDirection direction = parseRouteDirection(fields[3]);
      static_cast<void>(session.addEdge(
        static_cast<std::uint64_t>(std::stoull(fields[1])),
        static_cast<std::uint64_t>(std::stoull(fields[2])),
        std::move(polyline), direction));
    } else if (command == "delete_edge") {
      if (fields.size() != 3U) {
        throw std::runtime_error("delete_edge requires ID, include_reverse");
      }
      session.deleteEdge(
        static_cast<std::uint64_t>(std::stoull(fields[1])), parseBool(fields[2]));
    } else if (command == "split_edge") {
      if (fields.size() != 3U) {throw std::runtime_error("split_edge requires ID, arc distance");}
      static_cast<void>(session.splitEdge(
        static_cast<std::uint64_t>(std::stoull(fields[1])), std::stod(fields[2])));
    } else if (command == "set_direction") {
      if (fields.size() != 3U) {throw std::runtime_error("set_direction requires ID, direction");}
      const lmmg::RouteDirection direction = parseRouteDirection(fields[2]);
      static_cast<void>(session.setEdgeDirection(
        static_cast<std::uint64_t>(std::stoull(fields[1])), direction));
    } else {
      throw std::runtime_error("unknown route edit command: " + fields[0]);
    }

    lmmg::EditedRouteGraph edited = session.editedGraph();
    const bool separate_replay_semantics = usesLosslessReplaySemanticGraph();
    std::optional<lmmg::SemanticMap> inactive_topology_semantics;
    if (separate_replay_semantics) {
      const std::filesystem::path topology_semantics_path =
        options_.output_directory / "semantic_features_autoware_topology.tsv";
      if (std::filesystem::exists(topology_semantics_path)) {
        inactive_topology_semantics = lmmg::loadSemanticMapTsv(
          topology_semantics_path, &data_.graph);
      }
    }
    if (!separate_replay_semantics &&
      (command == "delete_edge" || command == "delete_node" || command == "set_direction" ||
      command == "reset" || command == "clear" || command == "undo"))
    {
      const std::vector<std::uint64_t> affected = semanticFeaturesWithMissingRouteTargets(
        data_.semantics, edited.graph);
      if (!affected.empty()) {
        std::ostringstream message;
        message << "Route edit would remove edges referenced by semantic feature(s): ";
        for (std::size_t index = 0U; index < affected.size(); ++index) {
          if (index > 0U) {message << ", ";}
          message << affected[index];
        }
        message << ". Delete or retarget those semantics first; no Route edit was saved.";
        throw std::runtime_error(message.str());
      }
    }
    if (inactive_topology_semantics &&
      (command == "delete_edge" || command == "delete_node" ||
      command == "set_direction" || command == "reset" || command == "clear" ||
      command == "undo"))
    {
      const std::vector<std::uint64_t> affected = semanticFeaturesWithMissingRouteTargets(
        *inactive_topology_semantics, edited.graph);
      if (!affected.empty()) {
        std::ostringstream message;
        message << "Route edit would remove edges referenced by edited-topology semantic "
                << "feature(s): ";
        for (std::size_t index = 0U; index < affected.size(); ++index) {
          if (index > 0U) {message << ", ";}
          message << affected[index];
        }
        message << ". Select the edited-topology Vector Map source and delete or retarget "
                << "those semantics first; no Route edit was saved.";
        throw std::runtime_error(message.str());
      }
    }
    lmmg::SemanticMap remapped_semantics = data_.semantics;
    if (!separate_replay_semantics) {
      const lmmg::SemanticMap stable_semantics = updateStableRouteSpanAnchors(
        data_.semantics, data_.graph, edited.graph);
      remapped_semantics = lmmg::remapSemanticMapToGraph(stable_semantics, edited.graph);
    }
    std::optional<lmmg::SemanticMap> remapped_inactive_topology_semantics;
    if (inactive_topology_semantics) {
      const lmmg::SemanticMap stable_semantics = updateStableRouteSpanAnchors(
        *inactive_topology_semantics, data_.graph, edited.graph);
      remapped_inactive_topology_semantics =
        lmmg::remapSemanticMapToGraph(stable_semantics, edited.graph);
    }
    const auto temporary = [&](const char * filename) {
        return options_.output_directory / (std::string(".") + filename + ".tmp");
      };
    const auto target = [&](const char * filename) {
        return options_.output_directory / filename;
      };
    const std::filesystem::path edits_tsv_temp = temporary("route_edits.tsv");
    const std::filesystem::path edits_geojson_temp = temporary("route_edits.geojson");
    const std::filesystem::path edited_graph_temp = temporary("route_graph_edited.geojson");
    const std::filesystem::path edited_metadata_temp = temporary(
      "route_graph_edited_metadata.yaml");
    const std::filesystem::path edited_corridors_temp = temporary(
      "drivable_corridors_edited.geojson");
    const std::filesystem::path edited_review_temp = temporary("review_geometry_edited.tsv");
    const char * semantic_tsv_filename = semanticFeaturesFilename();
    const char * semantic_geojson_filename = semanticFeaturesGeoJsonFilename();
    const std::filesystem::path semantic_tsv_temp = temporary(semantic_tsv_filename);
    const std::filesystem::path semantic_geojson_temp = temporary(semantic_geojson_filename);
    const std::filesystem::path semantic_rules_temp = temporary("semantic_route_rules.yaml");
    const std::filesystem::path semantic_graph_temp = temporary("route_graph_semantic.geojson");
    const std::filesystem::path semantic_preview_rules_temp = temporary(
      "semantic_route_rules_preview.yaml");
    const std::filesystem::path semantic_preview_graph_temp = temporary(
      "route_graph_semantic_preview.geojson");
    const std::filesystem::path inactive_topology_semantic_tsv_temp = temporary(
      "semantic_features_autoware_topology.tsv");
    const std::filesystem::path inactive_topology_semantic_geojson_temp = temporary(
      "semantic_features_autoware_topology.geojson");
    const std::filesystem::path validation_report_temp = temporary(
      "route_validation_report.yaml");

    lmmg::RouteGraph invalidated_graph;
    invalidated_graph.frame_id = edited.graph.frame_id;
    const std::filesystem::path route_temp = temporary("route_graph.geojson");
    const std::filesystem::path route_validated_temp = temporary(
      "route_graph_validated.geojson");
    const std::filesystem::path metadata_temp = temporary("route_graph_metadata.yaml");
    const std::filesystem::path metadata_validated_temp = temporary(
      "route_graph_validated_metadata.yaml");
    const std::filesystem::path corridors_temp = temporary("drivable_corridors.geojson");
    const std::filesystem::path corridors_validated_temp = temporary(
      "drivable_corridors_validated.geojson");
    const std::filesystem::path review_temp = temporary("review_geometry.tsv");
    const std::filesystem::path review_validated_temp = temporary(
      "review_geometry_validated.tsv");
    const std::filesystem::path lanelet_temp = temporary("lanelet2_map.osm");
    const std::filesystem::path lanelet_validated_temp = temporary(
      "lanelet2_map_validated.osm");

    lmmg::saveRouteEditOverlayTsv(edits_tsv_temp, session.overlay());
    lmmg::saveRouteEditOverlayGeoJson(edits_geojson_temp, session.overlay());
    lmmg::saveEditedRouteGraphGeoJson(edited_graph_temp, edited);
    lmmg::saveRouteGraphMetadataYaml(edited_metadata_temp, edited.graph);
    lmmg::saveCorridorsGeoJson(edited_corridors_temp, edited.graph);
    lmmg::RouteGraph unvalidated_review_graph = edited.graph;
    for (lmmg::RouteEdge & edge : unvalidated_review_graph.edges) {
      edge.passable = false;
      if (std::find(
          edge.validation_errors.begin(), edge.validation_errors.end(),
          "route_edits_unvalidated") == edge.validation_errors.end())
      {
        edge.validation_errors.push_back("route_edits_unvalidated");
      }
    }
    lmmg::saveReviewGeometryTsv(edited_review_temp, unvalidated_review_graph);
    {
      std::ofstream report(validation_report_temp);
      if (!report) {throw std::runtime_error("failed to create validation placeholder");}
      report << "route_validation_version: 1\n"
             << "validation_status: unvalidated\n"
             << "navigation_ready: false\n"
             << "vehicle_dimensions_verified: false\n"
             << "valid_edges: 0\nwarning_edges: 0\ninvalid_edges: 0\n"
             << "operational_nodes: 0\noperational_edges: 0\n"
             << "reasons:\n"
             << "  - \"Route edits are not safety-validated; rerun generate_vector_map\"\n";
    }
    lmmg::saveSemanticMapTsv(semantic_tsv_temp, remapped_semantics);
    const lmmg::RouteGraph & saved_semantic_graph = separate_replay_semantics ?
      data_.autoware_lossless_replay_graph : edited.graph;
    lmmg::saveSemanticMapGeoJson(
      semantic_geojson_temp, remapped_semantics, saved_semantic_graph);
    lmmg::saveSemanticRouteRulesYaml(
      semantic_preview_rules_temp, remapped_semantics, saved_semantic_graph);
    lmmg::saveSemanticRouteGraphGeoJson(
      semantic_preview_graph_temp, remapped_semantics, saved_semantic_graph);
    writeUnvalidatedSemanticOutputs(semantic_rules_temp, semantic_graph_temp);
    if (remapped_inactive_topology_semantics) {
      lmmg::saveSemanticMapTsv(
        inactive_topology_semantic_tsv_temp, *remapped_inactive_topology_semantics);
      lmmg::saveSemanticMapGeoJson(
        inactive_topology_semantic_geojson_temp,
        *remapped_inactive_topology_semantics, edited.graph);
    }

    lmmg::saveRouteGraphGeoJson(route_temp, invalidated_graph);
    lmmg::saveRouteGraphGeoJson(route_validated_temp, invalidated_graph);
    lmmg::saveRouteGraphMetadataYaml(metadata_temp, invalidated_graph);
    lmmg::saveRouteGraphMetadataYaml(metadata_validated_temp, invalidated_graph);
    lmmg::saveCorridorsGeoJson(corridors_temp, invalidated_graph);
    lmmg::saveCorridorsGeoJson(corridors_validated_temp, invalidated_graph);
    lmmg::saveReviewGeometryTsv(review_temp, invalidated_graph);
    lmmg::saveReviewGeometryTsv(review_validated_temp, invalidated_graph);
    lmmg::saveLanelet2Osm(lanelet_temp, invalidated_graph, lmmg::Lanelet2Config{});
    lmmg::saveLanelet2Osm(lanelet_validated_temp, invalidated_graph, lmmg::Lanelet2Config{});

    // Invalidate the deployment gate and canonical route first. A process that
    // observes an interrupted multi-file update therefore fails closed.
    atomicReplace(validation_report_temp, target("route_validation_report.yaml"));
    atomicReplace(route_temp, target("route_graph.geojson"));
    atomicReplace(route_validated_temp, target("route_graph_validated.geojson"));
    atomicReplace(metadata_temp, target("route_graph_metadata.yaml"));
    atomicReplace(metadata_validated_temp, target("route_graph_validated_metadata.yaml"));
    atomicReplace(corridors_temp, target("drivable_corridors.geojson"));
    atomicReplace(corridors_validated_temp, target("drivable_corridors_validated.geojson"));
    atomicReplace(review_temp, target("review_geometry.tsv"));
    atomicReplace(review_validated_temp, target("review_geometry_validated.tsv"));
    atomicReplace(lanelet_temp, target("lanelet2_map.osm"));
    atomicReplace(lanelet_validated_temp, target("lanelet2_map_validated.osm"));
    atomicReplace(semantic_rules_temp, target("semantic_route_rules.yaml"));
    atomicReplace(semantic_graph_temp, target("route_graph_semantic.geojson"));

    atomicReplace(edits_tsv_temp, target("route_edits.tsv"));
    atomicReplace(edits_geojson_temp, target("route_edits.geojson"));
    atomicReplace(edited_graph_temp, target("route_graph_edited.geojson"));
    atomicReplace(edited_metadata_temp, target("route_graph_edited_metadata.yaml"));
    atomicReplace(edited_corridors_temp, target("drivable_corridors_edited.geojson"));
    atomicReplace(edited_review_temp, target("review_geometry_edited.tsv"));
    atomicReplace(semantic_tsv_temp, target(semantic_tsv_filename));
    atomicReplace(semantic_geojson_temp, target(semantic_geojson_filename));
    atomicReplace(
      semantic_preview_rules_temp, target("semantic_route_rules_preview.yaml"));
    atomicReplace(
      semantic_preview_graph_temp, target("route_graph_semantic_preview.geojson"));
    if (remapped_inactive_topology_semantics) {
      atomicReplace(
        inactive_topology_semantic_tsv_temp,
        target("semantic_features_autoware_topology.tsv"));
      atomicReplace(
        inactive_topology_semantic_geojson_temp,
        target("semantic_features_autoware_topology.geojson"));
    }

    data_.route_edits = session.overlay();
    data_.edited_graph = std::move(edited);
    data_.graph = data_.edited_graph.graph;
    data_.operational_graph = std::move(invalidated_graph);
    data_.semantics = std::move(remapped_semantics);
    data_.route_validation_status = "unvalidated";
    data_.navigation_ready = false;
    data_.vehicle_dimensions_verified = false;
    data_.valid_route_edges = 0U;
    data_.warning_route_edges = 0U;
    data_.invalid_route_edges = 0U;
    data_.route_validation_reasons = {
      "Route edits are not safety-validated; rerun generate_vector_map"};
    data_.closed_course_validation_stale = true;
    data_.target_readiness_stale = true;
    context_cache_ = makeContextJson();
    return context_cache_;
  }

private:
  [[nodiscard]] static double parseRouteEditCoordinate(
    const std::string & token, const char * coordinate_name)
  {
    const std::string value = trim(token);
    std::size_t consumed = 0U;
    double coordinate = 0.0;
    try {
      coordinate = std::stod(value, &consumed);
    } catch (const std::exception &) {
      throw std::runtime_error(
              std::string{"route edit "} + coordinate_name + " must be a finite number");
    }
    if (consumed != value.size() || !std::isfinite(coordinate)) {
      throw std::runtime_error(
              std::string{"route edit "} + coordinate_name + " must be a finite number");
    }
    return coordinate;
  }

  [[nodiscard]] std::optional<double> nearestRouteReferenceElevation(
    const double x, const double y) const
  {
    constexpr double maximum_distance_m = 2.0;
    constexpr double minimum_segment_length_squared = 1.0e-12;
    double best_distance_squared = maximum_distance_m * maximum_distance_m;
    double best_z = 0.0;
    bool found = false;

    const auto considerPoint = [&](const lmmg::Vec3 & point) {
        if (!lmmg::finite(point)) {return;}
        const double dx = x - point.x;
        const double dy = y - point.y;
        const double distance_squared = dx * dx + dy * dy;
        if (distance_squared <= best_distance_squared) {
          best_distance_squared = distance_squared;
          best_z = point.z;
          found = true;
        }
      };
    const auto considerSegment = [&](const lmmg::Vec3 & begin, const lmmg::Vec3 & end) {
        if (!lmmg::finite(begin) || !lmmg::finite(end)) {return;}
        const double dx = end.x - begin.x;
        const double dy = end.y - begin.y;
        const double length_squared = dx * dx + dy * dy;
        if (!(length_squared > minimum_segment_length_squared)) {
          considerPoint(begin);
          return;
        }
        const double ratio = std::max(
          0.0, std::min(1.0, ((x - begin.x) * dx + (y - begin.y) * dy) / length_squared));
        const double projected_x = begin.x + ratio * dx;
        const double projected_y = begin.y + ratio * dy;
        const double offset_x = x - projected_x;
        const double offset_y = y - projected_y;
        const double distance_squared = offset_x * offset_x + offset_y * offset_y;
        if (distance_squared <= best_distance_squared) {
          best_distance_squared = distance_squared;
          best_z = begin.z + ratio * (end.z - begin.z);
          found = true;
        }
      };

    for (const lmmg::RouteNode & node : data_.graph.nodes) {
      considerPoint(node.position);
    }
    for (const lmmg::RouteEdge & edge : data_.graph.edges) {
      for (const lmmg::Vec3 & point : edge.centerline) {
        considerPoint(point);
      }
      for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
        considerSegment(edge.centerline[index - 1U], edge.centerline[index]);
      }
    }
    for (const lmmg::TimedPose & pose : data_.processed_trajectory) {
      considerPoint(pose.world_from_body.translation);
    }
    for (std::size_t index = 1U; index < data_.processed_trajectory.size(); ++index) {
      considerSegment(
        data_.processed_trajectory[index - 1U].world_from_body.translation,
        data_.processed_trajectory[index].world_from_body.translation);
    }
    return found ? std::optional<double>{best_z} : std::nullopt;
  }

  [[nodiscard]] std::optional<double> localPointCloudElevation(
    const double x, const double y) const
  {
    constexpr double sample_radius_m = 1.0;
    constexpr double lower_quantile = 0.15;
    constexpr std::size_t minimum_samples = 3U;
    const double maximum_distance_squared = sample_radius_m * sample_radius_m;
    std::vector<double> elevations;
    for (const lmmg::PointXYZI & point : data_.sampled_points) {
      if (!point.finite()) {continue;}
      const double dx = x - static_cast<double>(point.x);
      const double dy = y - static_cast<double>(point.y);
      if (dx * dx + dy * dy <= maximum_distance_squared) {
        elevations.push_back(static_cast<double>(point.z));
      }
    }
    if (elevations.size() < minimum_samples) {
      return std::nullopt;
    }
    std::sort(elevations.begin(), elevations.end());
    const std::size_t index = static_cast<std::size_t>(std::floor(
      lower_quantile * static_cast<double>(elevations.size() - 1U)));
    return elevations[index];
  }

  [[nodiscard]] double parseRouteEditElevation(
    const std::string & token, const double x, const double y) const
  {
    if (lower(trim(token)) != "auto") {
      return parseRouteEditCoordinate(token, "z");
    }
    if (const std::optional<double> route_elevation = nearestRouteReferenceElevation(x, y)) {
      return *route_elevation;
    }
    if (const std::optional<double> cloud_elevation = localPointCloudElevation(x, y)) {
      return *cloud_elevation;
    }
    std::ostringstream message;
    message << std::setprecision(9)
            << "automatic route elevation is unavailable at x=" << x << ", y=" << y
            << "; click within 2 m of the existing route or processed trajectory, "
            << "or on a locally sampled point-cloud surface";
    throw std::runtime_error(message.str());
  }

  [[nodiscard]] bool usesEditedTopologyVectorMapSource() const
  {
    return options_.editor_mode == "vector_map" &&
           data_.vector_map_source.source ==
           lmmg::VectorMapCenterlineSource::kEditedTopology;
  }

  [[nodiscard]] NavigationAuthoringScope selectedVectorMapAuthoringScope() const
  {
    return data_.vector_map_source.source ==
           lmmg::VectorMapCenterlineSource::kEditedTopology ?
           NavigationAuthoringScope::kAutowareEditedTopology :
           NavigationAuthoringScope::kAutowareLosslessReplay;
  }

  [[nodiscard]] const lmmg::RouteGraph & navigationGraphForScope(
    const NavigationAuthoringScope scope) const
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      return data_.autoware_lossless_replay_graph;
    }
    return data_.graph;
  }

  [[nodiscard]] bool navigationGraphAvailable(const NavigationAuthoringScope scope) const
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      return data_.autoware_lossless_replay_available;
    }
    return !data_.graph.edges.empty();
  }

  [[nodiscard]] bool navigationGraphReady(const NavigationAuthoringScope scope) const
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      return data_.autoware_lossless_replay_available &&
             data_.target_readiness.autoware.enabled &&
             data_.target_readiness.autoware.closed_course_experimental_ready;
    }
    if (scope == NavigationAuthoringScope::kAutowareEditedTopology) {
      return data_.closed_course_graph_ready;
    }
    return data_.navigation_ready;
  }

  [[nodiscard]] std::string navigationGraphUnavailableReason(
    const NavigationAuthoringScope scope) const
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      return data_.autoware_lossless_replay_reason;
    }
    return data_.graph.edges.empty() ? "editable topology is empty" : std::string{};
  }

  [[nodiscard]] const char * navigationGraphSourceArtifact(
    const NavigationAuthoringScope scope) const
  {
    return scope == NavigationAuthoringScope::kAutowareLosslessReplay ?
           "review_geometry_autoware_replay_candidate.tsv" :
           "route_graph_edited.geojson";
  }

  [[nodiscard]] const lmmg::NavigationAuthoring & navigationAuthoringForScope(
    const NavigationAuthoringScope scope) const
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      return data_.autoware_navigation_authoring;
    }
    if (scope == NavigationAuthoringScope::kAutowareEditedTopology) {
      return data_.autoware_topology_navigation_authoring;
    }
    return data_.navigation_authoring;
  }

  [[nodiscard]] lmmg::NavigationAuthoring & navigationAuthoringForScope(
    const NavigationAuthoringScope scope)
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      return data_.autoware_navigation_authoring;
    }
    if (scope == NavigationAuthoringScope::kAutowareEditedTopology) {
      return data_.autoware_topology_navigation_authoring;
    }
    return data_.navigation_authoring;
  }

  [[nodiscard]] const std::string & navigationAuthoringLoadErrorForScope(
    const NavigationAuthoringScope scope) const
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      return data_.autoware_navigation_authoring_load_error;
    }
    if (scope == NavigationAuthoringScope::kAutowareEditedTopology) {
      return data_.autoware_topology_navigation_authoring_load_error;
    }
    return data_.navigation_authoring_load_error;
  }

  void clearNavigationAuthoringLoadErrorForScope(const NavigationAuthoringScope scope)
  {
    if (scope == NavigationAuthoringScope::kAutowareLosslessReplay) {
      data_.autoware_navigation_authoring_load_error.clear();
    } else if (scope == NavigationAuthoringScope::kAutowareEditedTopology) {
      data_.autoware_topology_navigation_authoring_load_error.clear();
    } else {
      data_.navigation_authoring_load_error.clear();
    }
  }

  void requireCurrentVectorMapSource() const
  {
    if (!data_.vector_map_source_load_error.empty()) {
      throw std::runtime_error(
              "vector_map_source.tsv is invalid: " + data_.vector_map_source_load_error);
    }
    if (data_.vector_map_source.source ==
      lmmg::VectorMapCenterlineSource::kRecordedTrajectory &&
      !data_.autoware_lossless_replay_available)
    {
      throw std::runtime_error(
              "the recorded-trajectory Vector Map source is unavailable: " +
              data_.autoware_lossless_replay_reason);
    }
    lmmg::validateVectorMapSourceSelection(
      data_.vector_map_source, data_.autoware_lossless_replay_graph, data_.graph);
  }

  [[nodiscard]] const lmmg::RouteGraph & selectedVectorMapGraph() const
  {
    requireCurrentVectorMapSource();
    return data_.vector_map_source.source ==
           lmmg::VectorMapCenterlineSource::kEditedTopology ?
           data_.graph : data_.autoware_lossless_replay_graph;
  }

  [[nodiscard]] const char * semanticFeaturesFilename() const
  {
    return usesEditedTopologyVectorMapSource() ?
           "semantic_features_autoware_topology.tsv" : "semantic_features.tsv";
  }

  [[nodiscard]] const char * semanticFeaturesGeoJsonFilename() const
  {
    return usesEditedTopologyVectorMapSource() ?
           "semantic_features_autoware_topology.geojson" : "semantic_features.geojson";
  }

  [[nodiscard]] bool usesLosslessReplaySemanticGraph() const
  {
    return options_.editor_mode == "vector_map" &&
           !usesEditedTopologyVectorMapSource();
  }

  [[nodiscard]] const lmmg::RouteGraph & semanticAuthoringGraph() const
  {
    if (usesLosslessReplaySemanticGraph()) {
      requireCurrentVectorMapSource();
      if (!data_.autoware_lossless_replay_available) {
        throw std::runtime_error(
                "lossless replay is unavailable for vector-map semantic authoring: " +
                data_.autoware_lossless_replay_reason);
      }
      return data_.autoware_lossless_replay_graph;
    }
    if (options_.editor_mode == "vector_map") {
      requireCurrentVectorMapSource();
    }
    return data_.graph;
  }

  [[nodiscard]] bool vectorMapOperationsAllowed() const
  {
    return options_.editor_mode != "navigation_map";
  }

  void requireVectorMapOperation(const std::string & operation) const
  {
    if (!vectorMapOperationsAllowed()) {
      throw std::runtime_error(
              operation + " is unavailable in the 2D navigation-map editor");
    }
  }

  [[nodiscard]] std::pair<std::filesystem::path, std::filesystem::path>
  prepareTrustedVectorMapExportRequest() const
  {
    if (!options_.enable_autoware_one_click_export ||
      options_.autoware_one_click_session.empty() ||
      options_.autoware_one_click_session == "disabled")
    {
      throw std::runtime_error(
              "one-click export is unavailable in this launch; use the trusted "
              "map_ws dataset review helper");
    }
    const lmmg::RouteGraph & graph = selectedVectorMapGraph();
    const std::filesystem::path marker =
      options_.output_directory / ".autoware_one_click_export_requested";
    const std::filesystem::path temporary =
      options_.output_directory / ".autoware_one_click_export_requested.tmp";
    {
      std::ofstream stream(temporary, std::ios::binary);
      if (!stream) {
        throw std::runtime_error("failed to create the one-click export request");
      }
      stream << "LMMG_AUTOWARE_ONE_CLICK_EXPORT\t1\n"
             << "SESSION\t" << options_.autoware_one_click_session << "\n"
             << "SOURCE\t" << lmmg::toString(data_.vector_map_source.source) << "\n"
             << "GRAPH_FINGERPRINT\t" << lmmg::routeGraphFingerprint(graph) << "\n";
      stream.close();
      if (!stream) {
        throw std::runtime_error("failed to finish the one-click export request");
      }
    }
    return {marker, temporary};
  }

  void requireNavigationScopeAllowed(const NavigationAuthoringScope scope) const
  {
    if (options_.editor_mode == "vector_map" &&
      scope != selectedVectorMapAuthoringScope())
    {
      throw std::runtime_error(
              "the requested Mission source is not the selected Vector Map centerline source");
    }
    if (options_.editor_mode == "navigation_map" &&
      scope != NavigationAuthoringScope::kEditableTopology)
    {
      throw std::runtime_error(
              "lossless Lanelet2 replay Mission authoring is unavailable in the "
              "2D navigation-map editor");
    }
  }

  void ensureGeneratedAuditArtifacts()
  {
    const auto preserve = [&](const char * filename, const auto & writer) {
        const std::filesystem::path destination = options_.output_directory / filename;
        if (std::filesystem::exists(destination)) {return;}
        const std::filesystem::path temporary_path =
          options_.output_directory / (std::string(".") + filename + ".migration.tmp");
        writer(temporary_path);
        atomicReplace(temporary_path, destination);
      };
    preserve(
      "review_geometry_generated.tsv",
      [&](const std::filesystem::path & path) {
        lmmg::saveReviewGeometryTsv(path, data_.generated_graph);
      });
    preserve(
      "route_graph_generated.geojson",
      [&](const std::filesystem::path & path) {
        lmmg::saveRouteGraphGeoJson(path, data_.generated_graph);
      });
    preserve(
      "route_graph_generated_metadata.yaml",
      [&](const std::filesystem::path & path) {
        lmmg::saveRouteGraphMetadataYaml(path, data_.generated_graph);
      });
    preserve(
      "drivable_corridors_generated.geojson",
      [&](const std::filesystem::path & path) {
        lmmg::saveCorridorsGeoJson(path, data_.generated_graph);
      });
    preserve(
      "lanelet2_map_generated.osm",
      [&](const std::filesystem::path & path) {
        lmmg::saveLanelet2Osm(path, data_.generated_graph, lmmg::Lanelet2Config{});
      });
  }

  static void atomicReplace(
    const std::filesystem::path & source,
    const std::filesystem::path & destination)
  {
    std::error_code error;
    std::filesystem::rename(source, destination, error);
    if (!error) {
      return;
    }
    std::filesystem::remove(destination, error);
    error.clear();
    std::filesystem::rename(source, destination, error);
    if (error) {
      throw std::runtime_error(
              "failed to replace " + destination.string() + ": " + error.message());
    }
  }

  void writeFeatures(std::ostream & stream) const
  {
    stream << '[';
    for (std::size_t feature_index = 0U;
      feature_index < data_.semantics.features.size(); ++feature_index)
    {
      if (feature_index > 0U) {
        stream << ',';
      }
      const lmmg::SemanticFeature & feature = data_.semantics.features[feature_index];
      stream << "{\"id\":" << feature.id
             << ",\"type\":\"" << lmmg::toString(feature.type) << "\""
             << ",\"geometry\":\"" << lmmg::toString(feature.geometry) << "\""
             << ",\"enabled\":" << (feature.enabled ? "true" : "false")
             << ",\"name\":\"" << jsonEscape(feature.name) << "\""
             << ",\"notes\":\"" << jsonEscape(feature.notes) << "\""
             << ",\"position\":";
      writeVec3(stream, feature.position);
      stream << ",\"yaw\":" << feature.yaw
             << ",\"value\":" << feature.value
             << ",\"extent\":" << feature.extent
             << ",\"edges\":[";
      for (std::size_t index = 0U; index < feature.route_edge_ids.size(); ++index) {
        if (index > 0U) {stream << ',';}
        stream << feature.route_edge_ids[index];
      }
      stream << "],\"spans\":[";
      for (std::size_t index = 0U; index < feature.route_edge_spans.size(); ++index) {
        if (index > 0U) {stream << ',';}
        const lmmg::RouteEdgeSpan & span = feature.route_edge_spans[index];
        stream << "{\"edge_id\":" << span.edge_id
               << ",\"start_s\":" << span.start_s
               << ",\"end_s\":" << span.end_s;
        if (span.start_anchor && span.end_anchor) {
          stream << ",\"start_anchor\":";
          writeVec3(stream, *span.start_anchor);
          stream << ",\"end_anchor\":";
          writeVec3(stream, *span.end_anchor);
        }
        stream << '}';
      }
      stream << "],\"polygon\":[";
      for (std::size_t index = 0U; index < feature.polygon.size(); ++index) {
        if (index > 0U) {stream << ',';}
        writeVec3(stream, feature.polygon[index]);
      }
      stream << "]}";
    }
    stream << ']';
  }

  [[nodiscard]] std::string featuresJson() const
  {
    std::ostringstream stream;
    stream << std::setprecision(12);
    writeFeatures(stream);
    return stream.str();
  }

  [[nodiscard]] std::string makeContextJson() const
  {
    std::ostringstream stream;
    stream << std::setprecision(12);
    const auto writeReasons = [&stream](const std::vector<std::string> & reasons) {
        stream << '[';
        for (std::size_t index = 0U; index < reasons.size(); ++index) {
          if (index > 0U) {stream << ',';}
          stream << '"' << jsonEscape(reasons[index]) << '"';
        }
        stream << ']';
      };
    const auto writeTarget = [&stream](const NavigationTargetStatus & target) {
        stream << "{\"available\":" << (target.available ? "true" : "false")
               << ",\"enabled\":" << (target.enabled ? "true" : "false")
               << ",\"production_ready\":" <<
          (target.production_ready ? "true" : "false")
               << ",\"closed_course_experimental_ready\":" <<
          (target.closed_course_experimental_ready ? "true" : "false")
               << ",\"closed_course_route_edges\":" <<
          target.closed_course_route_edges << '}';
      };
    const auto writeSelectableNavigationGraph =
      [&stream](
      const lmmg::RouteGraph & graph, const bool available, const bool exact_lossless,
      const char * allowed_target, const char * source_artifact,
      const std::string & unavailable_reason) {
        stream << "{\"available\":" << (available ? "true" : "false")
               << ",\"exact_lossless\":" << (exact_lossless ? "true" : "false")
               << ",\"allowed_target\":\"" << allowed_target << '"'
               << ",\"source_artifact\":\"" << source_artifact << '"'
               << ",\"unavailable_reason\":\"" << jsonEscape(unavailable_reason) << '"'
               << ",\"graph_fingerprint\":\"" <<
          jsonEscape(lmmg::routeGraphFingerprint(graph)) << "\",\"nodes\":[";
        for (std::size_t index = 0U; index < graph.nodes.size(); ++index) {
          if (index > 0U) {stream << ',';}
          const lmmg::RouteNode & node = graph.nodes[index];
          stream << "{\"id\":" << node.id << ",\"type\":\"" <<
            lmmg::toString(node.type) << "\",\"position\":";
          writeVec3(stream, node.position);
          stream << '}';
        }
        stream << "],\"edges\":[";
        for (std::size_t index = 0U; index < graph.edges.size(); ++index) {
          if (index > 0U) {stream << ',';}
          const lmmg::RouteEdge & edge = graph.edges[index];
          stream << "{\"id\":" << edge.id << ",\"from\":" << edge.from
                 << ",\"to\":" << edge.to
                 << ",\"passable\":" << (edge.passable ? "true" : "false")
                 << ",\"length\":" << lmmg::polylineLength(edge.centerline)
                 << ",\"reverse_of\":";
          if (edge.reverse_of) {stream << *edge.reverse_of;} else {stream << "null";}
          stream << ",\"points\":[";
          for (std::size_t point = 0U; point < edge.centerline.size(); ++point) {
            if (point > 0U) {stream << ',';}
            writeVec3(stream, edge.centerline[point]);
          }
          stream << "]}";
        }
        stream << "]}";
      };
    bool vector_map_source_valid = false;
    std::string vector_map_source_reason;
    try {
      requireCurrentVectorMapSource();
      vector_map_source_valid = true;
    } catch (const std::exception & exception) {
      vector_map_source_reason = exception.what();
    }
    const NavigationAuthoringScope selected_vector_scope =
      selectedVectorMapAuthoringScope();
    const lmmg::RouteGraph & selected_vector_graph =
      navigationGraphForScope(selected_vector_scope);
    const bool selected_vector_graph_available =
      navigationGraphAvailable(selected_vector_scope);
    stream << "{\"frame_id\":\"" << jsonEscape(data_.semantics.frame_id) << "\""
           << ",\"editor_mode\":\"" << jsonEscape(options_.editor_mode) << "\""
           << ",\"semantic_graph_scope\":\"" <<
      (options_.editor_mode == "vector_map" ? toString(selected_vector_scope) :
      "editable_topology") << "\""
           << ",\"semantic_graph_available\":" <<
      (options_.editor_mode == "vector_map" ?
      (vector_map_source_valid ? "true" : "false") :
      (!data_.graph.edges.empty() ? "true" : "false"))
           << ",\"read_only\":" << (options_.read_only ? "true" : "false")
           << ",\"vector_map_source\":{\"source\":\"" <<
      lmmg::toString(data_.vector_map_source.source) << "\""
           << ",\"frame_id\":\"" << jsonEscape(data_.vector_map_source.frame_id) << "\""
           << ",\"graph_fingerprint\":\"" <<
      jsonEscape(data_.vector_map_source.graph_fingerprint) << "\""
           << ",\"current_graph_fingerprint\":\"" <<
      jsonEscape(lmmg::routeGraphFingerprint(selected_vector_graph)) << "\""
           << ",\"navigation_scope\":\"" << toString(selected_vector_scope) << "\""
           << ",\"graph_available\":" <<
      (selected_vector_graph_available ? "true" : "false")
           << ",\"valid\":" << (vector_map_source_valid ? "true" : "false")
           << ",\"reason\":\"" << jsonEscape(vector_map_source_reason) << "\"}"
           << ",\"autoware_one_click_export\":{"
           << "\"launcher_enabled\":" <<
      (vectorMapOperationsAllowed() && options_.enable_autoware_one_click_export &&
      !options_.autoware_one_click_session.empty() &&
      options_.autoware_one_click_session != "disabled" ? "true" : "false")
           << ",\"available\":" <<
      (vectorMapOperationsAllowed() && options_.enable_autoware_one_click_export &&
      !options_.autoware_one_click_session.empty() &&
      options_.autoware_one_click_session != "disabled" && !options_.read_only &&
      vector_map_source_valid ? "true" : "false")
           << ",\"reason\":\"" << jsonEscape(
      !vectorMapOperationsAllowed() ?
      std::string{"not available in the 2D navigation-map editor"} :
      options_.read_only ? std::string{"editor is read-only"} :
      !options_.enable_autoware_one_click_export ||
      options_.autoware_one_click_session.empty() ||
      options_.autoware_one_click_session == "disabled" ?
      std::string{"launch with the trusted map_ws dataset review helper"} :
      !vector_map_source_valid ? vector_map_source_reason : std::string{}) << "\"}"
           << ",\"route_graph_fingerprint\":\"" <<
      jsonEscape(lmmg::routeGraphFingerprint(data_.graph)) << "\""
           << ",\"navigation_graphs\":{\"editable_topology\":";
    writeSelectableNavigationGraph(
      data_.graph, !data_.graph.edges.empty(), false, "nav2",
      "route_graph_edited.geojson",
      data_.graph.edges.empty() ? "editable topology is empty" : std::string{});
    stream << ",\"autoware_lossless_replay\":";
    writeSelectableNavigationGraph(
      data_.autoware_lossless_replay_graph,
      data_.autoware_lossless_replay_available,
      data_.autoware_lossless_replay_available,
      "autoware", "review_geometry_autoware_replay_candidate.tsv",
      data_.autoware_lossless_replay_reason);
    stream << ",\"autoware_edited_topology\":";
    writeSelectableNavigationGraph(
      data_.graph, !data_.graph.edges.empty(), false, "autoware",
      "route_graph_edited.geojson",
      data_.graph.edges.empty() ? "editable topology is empty" : std::string{});
    stream << '}'
           << ",\"route_edit_operation_count\":" << data_.route_edits.operations.size()
           << ",\"route_validation_status\":\"" <<
      jsonEscape(data_.route_validation_status) << "\""
           << ",\"navigation_ready\":" << (data_.navigation_ready ? "true" : "false")
           << ",\"vehicle_dimensions_verified\":" <<
      (data_.vehicle_dimensions_verified ? "true" : "false")
           << ",\"valid_route_edges\":" << data_.valid_route_edges
           << ",\"warning_route_edges\":" << data_.warning_route_edges
           << ",\"invalid_route_edges\":" << data_.invalid_route_edges
           << ",\"route_validation_reasons\":[";
    for (std::size_t index = 0U; index < data_.route_validation_reasons.size(); ++index) {
      if (index > 0U) {stream << ',';}
      stream << '"' << jsonEscape(data_.route_validation_reasons[index]) << '"';
    }
    stream << ']'
           << ",\"production_route_validation\":{"
           << "\"available\":" <<
      (data_.production_validation_available ? "true" : "false")
           << ",\"status\":\"" << jsonEscape(data_.route_validation_status) << "\""
           << ",\"graph_ready\":" << (data_.navigation_ready ? "true" : "false")
           << ",\"valid_edges\":" << data_.valid_route_edges
           << ",\"warning_edges\":" << data_.warning_route_edges
           << ",\"invalid_edges\":" << data_.invalid_route_edges
           << ",\"reasons\":";
    writeReasons(data_.route_validation_reasons);
    stream << '}'
           << ",\"closed_course_route_validation\":{"
           << "\"available\":" <<
      (data_.closed_course_validation_available ? "true" : "false")
           << ",\"stale\":" <<
      (data_.closed_course_validation_stale ? "true" : "false")
           << ",\"status\":\"" <<
      jsonEscape(data_.closed_course_validation_status) << "\""
           << ",\"graph_ready\":" <<
      (data_.closed_course_graph_ready ? "true" : "false")
           << ",\"valid_edges\":" << data_.closed_course_valid_edges
           << ",\"warning_edges\":" << data_.closed_course_warning_edges
           << ",\"invalid_edges\":" << data_.closed_course_invalid_edges
           << ",\"reasons\":";
    writeReasons(data_.closed_course_validation_reasons);
    stream << '}'
           << ",\"navigation_target_readiness\":{"
           << "\"available\":" <<
      (data_.target_readiness.report_available ? "true" : "false")
           << ",\"generation_complete\":" <<
      (data_.target_readiness.generation_complete ? "true" : "false")
           << ",\"stale\":" << (data_.target_readiness_stale ? "true" : "false")
           << ",\"requested_target_mode\":\"" <<
      jsonEscape(data_.target_readiness.requested_target_mode) << "\""
           << ",\"nav2\":";
    writeTarget(data_.target_readiness.nav2);
    stream << ",\"autoware\":";
    writeTarget(data_.target_readiness.autoware);
    stream << '}'
           << ",\"point_count_original\":" << data_.original_point_count
           << ",\"bounds\":{"
           << "\"min_x\":" << data_.bounds.minimum_x << ','
           << "\"min_y\":" << data_.bounds.minimum_y << ','
           << "\"max_x\":" << data_.bounds.maximum_x << ','
           << "\"max_y\":" << data_.bounds.maximum_y << "}"
           << ",\"points\":[";
    for (std::size_t index = 0U; index < data_.sampled_points.size(); ++index) {
      if (index > 0U) {stream << ',';}
      const lmmg::PointXYZI & point = data_.sampled_points[index];
      stream << '[' << point.x << ',' << point.y << ',' << point.z << ']';
    }
    stream << "],\"trajectory_raw\":[";
    for (std::size_t index = 0U; index < data_.raw_trajectory.size(); ++index) {
      if (index > 0U) {stream << ',';}
      writeVec3(stream, data_.raw_trajectory[index].world_from_body.translation);
    }
    stream << "],\"trajectory_processed\":[";
    for (std::size_t index = 0U; index < data_.processed_trajectory.size(); ++index) {
      if (index > 0U) {stream << ',';}
      writeVec3(stream, data_.processed_trajectory[index].world_from_body.translation);
    }
    stream << "],\"nodes\":[";
    for (std::size_t index = 0U; index < data_.graph.nodes.size(); ++index) {
      if (index > 0U) {stream << ',';}
      const lmmg::RouteNode & node = data_.graph.nodes[index];
      const auto metadata = data_.edited_graph.node_metadata.find(node.id);
      stream << "{\"id\":" << node.id << ",\"type\":\""
             << lmmg::toString(node.type) << "\",\"provenance\":\""
             << (metadata == data_.edited_graph.node_metadata.end() ? "generated" :
      lmmg::toString(metadata->second.provenance))
             << "\",\"validation_status\":\""
             << (metadata == data_.edited_graph.node_metadata.end() ? "unvalidated" :
      lmmg::toString(metadata->second.validation_status))
             << "\",\"position\":";
      writeVec3(stream, node.position);
      stream << '}';
    }
    stream << "],\"edges\":[";
    for (std::size_t edge_index = 0U; edge_index < data_.graph.edges.size(); ++edge_index) {
      if (edge_index > 0U) {stream << ',';}
      const lmmg::RouteEdge & edge = data_.graph.edges[edge_index];
      const auto metadata = data_.edited_graph.edge_metadata.find(edge.id);
      stream << "{\"id\":" << edge.id
             << ",\"from\":" << edge.from
             << ",\"to\":" << edge.to
             << ",\"passable\":" << (edge.passable ? "true" : "false")
             << ",\"speed\":" << edge.recommended_speed_mps
             << ",\"length\":" << lmmg::polylineLength(edge.centerline)
             << ",\"provenance\":\""
             << (metadata == data_.edited_graph.edge_metadata.end() ? "generated" :
      lmmg::toString(metadata->second.provenance))
             << "\",\"validation_status\":\""
             << (metadata == data_.edited_graph.edge_metadata.end() ? "unvalidated" :
      lmmg::toString(metadata->second.validation_status)) << '"'
             << ",\"reverse_of\":";
      if (edge.reverse_of) {
        stream << *edge.reverse_of;
      } else {
        stream << "null";
      }
      stream << ",\"points\":[";
      for (std::size_t point_index = 0U; point_index < edge.centerline.size(); ++point_index) {
        if (point_index > 0U) {stream << ',';}
        writeVec3(stream, edge.centerline[point_index]);
      }
      stream << "]}";
    }
    stream << "],\"features\":";
    writeFeatures(stream);
    stream << '}';
    return stream.str();
  }

  Options options_;
  EditorData data_;
  std::string context_cache_;
  std::string editor_html_;
};

struct HttpRequest
{
  std::string method;
  std::string target;
  std::map<std::string, std::string> headers;
  std::string body;
};

void sendAll(const int socket, const std::string & data)
{
  std::size_t offset = 0U;
  while (offset < data.size()) {
    const ssize_t sent = ::send(
      socket, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error("socket send failed: " + std::string(std::strerror(errno)));
    }
    if (sent == 0) {
      throw std::runtime_error("socket closed while sending response");
    }
    offset += static_cast<std::size_t>(sent);
  }
}

HttpRequest readRequest(const int socket)
{
  constexpr std::size_t maximum_header_size = 1024U * 1024U;
  constexpr std::size_t maximum_body_size = 32U * 1024U * 1024U;
  std::string buffer;
  buffer.reserve(8192U);
  char chunk[8192];
  std::size_t header_end = std::string::npos;
  while (header_end == std::string::npos) {
    const ssize_t received = ::recv(socket, chunk, sizeof(chunk), 0);
    if (received < 0) {
      if (errno == EINTR) {continue;}
      throw std::runtime_error("socket receive failed: " + std::string(std::strerror(errno)));
    }
    if (received == 0) {
      throw std::runtime_error("client closed connection before sending request headers");
    }
    buffer.append(chunk, static_cast<std::size_t>(received));
    if (buffer.size() > maximum_header_size) {
      throw std::runtime_error("HTTP request headers exceed 1 MiB");
    }
    header_end = buffer.find("\r\n\r\n");
  }

  HttpRequest request;
  std::istringstream headers(buffer.substr(0U, header_end));
  std::string request_line;
  std::getline(headers, request_line);
  if (!request_line.empty() && request_line.back() == '\r') {
    request_line.pop_back();
  }
  std::istringstream request_line_stream(request_line);
  std::string version;
  request_line_stream >> request.method >> request.target >> version;
  if (request.method.empty() || request.target.empty() || version.rfind("HTTP/", 0U) != 0U) {
    throw std::runtime_error("invalid HTTP request line");
  }
  std::string line;
  while (std::getline(headers, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const std::size_t separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    request.headers[lower(trim(line.substr(0U, separator)))] = trim(line.substr(separator + 1U));
  }
  std::size_t content_length = 0U;
  const auto content_length_header = request.headers.find("content-length");
  if (content_length_header != request.headers.end()) {
    content_length = static_cast<std::size_t>(std::stoull(content_length_header->second));
    if (content_length > maximum_body_size) {
      throw std::runtime_error("HTTP request body exceeds 32 MiB");
    }
  }
  const std::size_t body_begin = header_end + 4U;
  if (buffer.size() > body_begin) {
    request.body = buffer.substr(body_begin);
  }
  while (request.body.size() < content_length) {
    const ssize_t received = ::recv(socket, chunk, sizeof(chunk), 0);
    if (received < 0) {
      if (errno == EINTR) {continue;}
      throw std::runtime_error("socket receive failed: " + std::string(std::strerror(errno)));
    }
    if (received == 0) {
      throw std::runtime_error("client closed connection before completing request body");
    }
    request.body.append(chunk, static_cast<std::size_t>(received));
  }
  if (request.body.size() > content_length) {
    request.body.resize(content_length);
  }
  return request;
}

std::string makeResponse(
  const int status,
  const std::string & status_text,
  const std::string & content_type,
  const std::string & body,
  const std::vector<std::pair<std::string, std::string>> & extra_headers = {})
{
  std::ostringstream stream;
  stream << "HTTP/1.1 " << status << ' ' << status_text << "\r\n"
         << "Content-Type: " << content_type << "\r\n"
         << "Content-Length: " << body.size() << "\r\n"
         << "Cache-Control: no-store\r\n"
         << "X-Content-Type-Options: nosniff\r\n"
         << "X-Frame-Options: DENY\r\n"
         << "Referrer-Policy: no-referrer\r\n"
         << "Cross-Origin-Resource-Policy: same-origin\r\n"
         << "Connection: close\r\n";
  for (const auto & header : extra_headers) {
    stream << header.first << ": " << header.second << "\r\n";
  }
  stream << "\r\n" << body;
  return stream.str();
}

std::string errorJson(const std::string & message)
{
  return "{\"ok\":false,\"error\":\"" + jsonEscape(message) + "\"}";
}

std::string reportedApiErrorJson(
  const std::string & request_path, const std::string & message)
{
  const std::uint64_t sequence = g_api_error_sequence.fetch_add(1U) + 1U;
  std::ostringstream identifier;
  identifier << "LMMG-" << static_cast<unsigned long long>(::getpid()) << '-'
             << std::setw(6) << std::setfill('0') << sequence;
  const std::string correlation_id = identifier.str();
  std::cerr << "semantic_map_editor API error [" << correlation_id << "] "
            << request_path << ": " << message << '\n';
  return "{\"ok\":false,\"error\":\"" + jsonEscape(message) +
         "\",\"correlation_id\":\"" + jsonEscape(correlation_id) + "\"}";
}

void handleClient(const int client_socket, EditorRepository & repository)
{
  try {
    const HttpRequest request = readRequest(client_socket);
    const std::string path = request.target.substr(0U, request.target.find('?'));
    if (request.method == "GET" && (path == "/" || path == "/index.html")) {
      sendAll(client_socket, makeResponse(
        200, "OK", "text/html; charset=utf-8", repository.editorHtml()));
    } else if (request.method == "GET" && path == "/api/context") {
      if (request.target.find("reload=1") != std::string::npos) {
        repository.reload();
      }
      sendAll(client_socket, makeResponse(
        200, "OK", "application/json; charset=utf-8", repository.contextJson()));
    } else if (request.method == "GET" && path == "/api/health") {
      sendAll(client_socket, makeResponse(
        200, "OK", "application/json; charset=utf-8", "{\"ok\":true}"));
    } else if (request.method == "GET" && path == "/api/semantic.geojson") {
      sendAll(client_socket, makeResponse(
        200, "OK", "application/geo+json; charset=utf-8", repository.semanticGeoJson(),
          {{"Content-Disposition", "inline; filename=semantic_features.geojson"}}));
    } else if (request.method == "GET" && path == "/api/navigation-authoring") {
      try {
        const NavigationAuthoringScope scope =
          navigationAuthoringScopeFromRequestTarget(request.target);
        sendAll(client_socket, makeResponse(
          200, "OK", "application/json; charset=utf-8",
          repository.navigationAuthoringResponse(scope)));
      } catch (const std::exception & exception) {
        sendAll(client_socket, makeResponse(
          400, "Bad Request", "application/json; charset=utf-8",
          reportedApiErrorJson(path, exception.what())));
      }
    } else if (request.method == "POST" &&
      (path == "/api/navigation-authoring" ||
      path == "/api/navigation-authoring/validate"))
    {
      const auto token = request.headers.find("x-lmmg-token");
      if (token == request.headers.end() || !repository.acceptsEditToken(token->second)) {
        sendAll(client_socket, makeResponse(
          403, "Forbidden", "application/json; charset=utf-8",
          reportedApiErrorJson(path, "invalid editor session token")));
        ::close(client_socket);
        return;
      }
      try {
        const NavigationAuthoringScope scope =
          navigationAuthoringScopeFromRequestTarget(request.target);
        const std::string body = path == "/api/navigation-authoring/validate" ?
          repository.validateNavigationAuthoringRequest(request.body, scope) :
          repository.saveNavigationAuthoringRequest(request.body, scope);
        sendAll(client_socket, makeResponse(
          200, "OK", "application/json; charset=utf-8", body));
      } catch (const std::exception & exception) {
        sendAll(client_socket, makeResponse(
          400, "Bad Request", "application/json; charset=utf-8",
          reportedApiErrorJson(path, exception.what())));
      }
    } else if (request.method == "POST" && path == "/api/save") {
      const auto token = request.headers.find("x-lmmg-token");
      if (token == request.headers.end() || !repository.acceptsEditToken(token->second)) {
        sendAll(client_socket, makeResponse(
          403, "Forbidden", "application/json; charset=utf-8",
          reportedApiErrorJson(path, "invalid editor session token")));
        ::close(client_socket);
        return;
      }
      try {
        const auto saved = repository.save(request.body);
        const std::string body =
          "{\"ok\":true,\"count\":" + std::to_string(saved.first) +
          ",\"features\":" + saved.second + "}";
        sendAll(client_socket, makeResponse(
          200, "OK", "application/json; charset=utf-8", body));
      } catch (const std::exception & exception) {
        sendAll(client_socket, makeResponse(
          400, "Bad Request", "application/json; charset=utf-8",
          reportedApiErrorJson(path, exception.what())));
      }
    } else if (request.method == "POST" && path == "/api/vector-map-source") {
      const auto token = request.headers.find("x-lmmg-token");
      if (token == request.headers.end() || !repository.acceptsEditToken(token->second)) {
        sendAll(client_socket, makeResponse(
          403, "Forbidden", "application/json; charset=utf-8",
          reportedApiErrorJson(path, "invalid editor session token")));
        ::close(client_socket);
        return;
      }
      try {
        const std::string context = repository.saveVectorMapSource(request.body);
        sendAll(client_socket, makeResponse(
          200, "OK", "application/json; charset=utf-8",
          "{\"ok\":true,\"context\":" + context + "}"));
      } catch (const std::exception & exception) {
        sendAll(client_socket, makeResponse(
          400, "Bad Request", "application/json; charset=utf-8",
          reportedApiErrorJson(path, exception.what())));
      }
    } else if (request.method == "POST" &&
      (path == "/api/autoware-full-replay-route" ||
      path == "/api/autoware-one-click-export" ||
      path == "/api/vector-map-current-route-export"))
    {
      const auto token = request.headers.find("x-lmmg-token");
      if (token == request.headers.end() || !repository.acceptsEditToken(token->second)) {
        sendAll(client_socket, makeResponse(
          403, "Forbidden", "application/json; charset=utf-8",
          reportedApiErrorJson(path, "invalid editor session token")));
        ::close(client_socket);
        return;
      }
      try {
        const std::string body = path == "/api/autoware-one-click-export" ?
          repository.requestAutowareOneClickExport() :
          path == "/api/vector-map-current-route-export" ?
          repository.requestCurrentVectorMapRouteExport() :
          repository.saveCompleteAutowareReplayRoute();
        sendAll(client_socket, makeResponse(
          200, "OK", "application/json; charset=utf-8", body));
      } catch (const std::exception & exception) {
        sendAll(client_socket, makeResponse(
          400, "Bad Request", "application/json; charset=utf-8",
          reportedApiErrorJson(path, exception.what())));
      }
    } else if (request.method == "POST" && path == "/api/route-edit") {
      const auto token = request.headers.find("x-lmmg-token");
      if (token == request.headers.end() || !repository.acceptsEditToken(token->second)) {
        sendAll(client_socket, makeResponse(
          403, "Forbidden", "application/json; charset=utf-8",
          reportedApiErrorJson(path, "invalid editor session token")));
        ::close(client_socket);
        return;
      }
      try {
        const std::string context = repository.editRoute(request.body);
        sendAll(client_socket, makeResponse(
          200, "OK", "application/json; charset=utf-8",
          "{\"ok\":true,\"context\":" + context + "}"));
      } catch (const std::exception & exception) {
        sendAll(client_socket, makeResponse(
          400, "Bad Request", "application/json; charset=utf-8",
          reportedApiErrorJson(path, exception.what())));
      }
    } else if (request.method == "GET" && path == "/favicon.ico") {
      sendAll(client_socket, makeResponse(204, "No Content", "image/x-icon", ""));
    } else {
      sendAll(client_socket, makeResponse(
        404, "Not Found", "application/json; charset=utf-8", errorJson("not found")));
    }
  } catch (const std::exception & exception) {
    try {
      sendAll(client_socket, makeResponse(
        400, "Bad Request", "application/json; charset=utf-8",
        reportedApiErrorJson("request", exception.what())));
    } catch (...) {
      // The peer may already be gone.
    }
  }
  ::close(client_socket);
}

int createServerSocket(
  const std::string & bind_address,
  const std::uint16_t requested_port,
  std::uint16_t * actual_port)
{
  const int server = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server < 0) {
    throw std::runtime_error("failed to create server socket: " +
        std::string(std::strerror(errno)));
  }
  int reuse = 1;
  if (::setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
    ::close(server);
    throw std::runtime_error("failed to configure server socket");
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(requested_port);
  if (::inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1) {
    ::close(server);
    throw std::invalid_argument("--bind must be an IPv4 address");
  }
  if (::bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    const std::string message = std::strerror(errno);
    ::close(server);
    throw std::runtime_error("failed to bind HTTP server: " + message);
  }
  if (::listen(server, 16) != 0) {
    const std::string message = std::strerror(errno);
    ::close(server);
    throw std::runtime_error("failed to listen on HTTP socket: " + message);
  }
  sockaddr_in bound{};
  socklen_t bound_length = sizeof(bound);
  if (::getsockname(server, reinterpret_cast<sockaddr *>(&bound), &bound_length) != 0) {
    ::close(server);
    throw std::runtime_error("failed to query HTTP server port");
  }
  *actual_port = ntohs(bound.sin_port);
  return server;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    Options options = parseOptions(argc, argv);
    options.edit_token = makeEditToken();
    EditorRepository repository(options);
    if (options.dump_context) {
      std::cout << repository.contextJson() << '\n';
      return 0;
    }
    if (options.save_complete_autoware_route) {
      std::cout << repository.saveCompleteAutowareReplayRoute() << '\n';
      return 0;
    }
    if (!options.navigation_authoring_command.empty()) {
      std::ifstream request_stream(options.navigation_authoring_request, std::ios::binary);
      if (!request_stream) {
        throw std::runtime_error(
                "failed to open navigation authoring request: " +
                options.navigation_authoring_request.string());
      }
      const std::string request_body{
        std::istreambuf_iterator<char>(request_stream),
        std::istreambuf_iterator<char>()};
      const NavigationAuthoringScope scope = navigationAuthoringScopeFromRequestTarget(
        std::string{"?scope="} + options.navigation_authoring_scope);
      const std::string response = options.navigation_authoring_command == "save" ?
        repository.saveNavigationAuthoringRequest(request_body, scope) :
        repository.validateNavigationAuthoringRequest(request_body, scope);
      std::cout << response << '\n';
      return 0;
    }
    std::uint16_t actual_port = 0U;
    const int server = createServerSocket(options.bind_address, options.port, &actual_port);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string url =
      "http://" + (options.bind_address == "0.0.0.0" ? std::string("127.0.0.1") :
      options.bind_address) + ':' + std::to_string(actual_port) + '/';
    std::cout << "Semantic map editor: " << url << '\n'
              << "Output directory: " << std::filesystem::absolute(options.output_directory) << '\n'
              << "Press Ctrl+C to stop.\n";
    if (options.bind_address != "127.0.0.1") {
      std::cerr <<
        "Warning: the editor is not bound to loopback; do not expose it to untrusted networks.\n";
    }
    if (options.open_browser) {
      std::thread([url]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
          const std::string command = "xdg-open '" + url + "' >/dev/null 2>&1";
          const int result = std::system(command.c_str());
          if (result == -1) {
            std::cerr << "Warning: failed to start xdg-open for " << url << '\n';
          }
        }).detach();
    }

    while (g_running.load()) {
      fd_set descriptors;
      FD_ZERO(&descriptors);
      FD_SET(server, &descriptors);
      timeval timeout{};
      timeout.tv_sec = 0;
      timeout.tv_usec = 250000;
      const int ready = ::select(server + 1, &descriptors, nullptr, nullptr, &timeout);
      if (ready < 0) {
        if (errno == EINTR) {continue;}
        throw std::runtime_error("select failed: " + std::string(std::strerror(errno)));
      }
      if (ready == 0) {
        continue;
      }
      sockaddr_in client_address{};
      socklen_t client_length = sizeof(client_address);
      const int client = ::accept(
        server, reinterpret_cast<sockaddr *>(&client_address), &client_length);
      if (client < 0) {
        if (errno == EINTR) {continue;}
        std::cerr << "Warning: accept failed: " << std::strerror(errno) << '\n';
        continue;
      }
      handleClient(client, repository);
    }
    ::close(server);
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "semantic_map_editor: " << exception.what() << '\n';
    return 1;
  }
}
