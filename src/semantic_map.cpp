#include "lidar_mobility_map_generator/semantic_map.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

std::ofstream openOutput(const std::filesystem::path & path)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to create output file: " + path.string());
  }
  stream << std::setprecision(12);
  return stream;
}

std::string escapeField(const std::string & input)
{
  std::string result;
  result.reserve(input.size());
  for (const char character : input) {
    switch (character) {
      case '\\': result += "\\\\"; break;
      case '\t': result += "\\t"; break;
      case '\r': result += "\\r"; break;
      case '\n': result += "\\n"; break;
      default: result.push_back(character); break;
    }
  }
  return result;
}

std::string unescapeField(const std::string & input)
{
  std::string result;
  result.reserve(input.size());
  bool escaped = false;
  for (const char character : input) {
    if (!escaped) {
      if (character == '\\') {
        escaped = true;
      } else {
        result.push_back(character);
      }
      continue;
    }
    switch (character) {
      case 't': result.push_back('\t'); break;
      case 'r': result.push_back('\r'); break;
      case 'n': result.push_back('\n'); break;
      case '\\': result.push_back('\\'); break;
      default:
        result.push_back('\\');
        result.push_back(character);
        break;
    }
    escaped = false;
  }
  if (escaped) {
    result.push_back('\\');
  }
  return result;
}

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> result;
  std::size_t begin = 0U;
  while (begin <= line.size()) {
    const std::size_t end = line.find('\t', begin);
    result.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return result;
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

std::string yamlQuote(const std::string & input)
{
  std::string result{"\""};
  for (const char character : input) {
    if (character == '\\' || character == '"') {
      result.push_back('\\');
    }
    if (character == '\n') {
      result += "\\n";
    } else {
      result.push_back(character);
    }
  }
  result.push_back('"');
  return result;
}

void writeCoordinate3d(std::ostream & stream, const Vec3 & point)
{
  stream << '[' << point.x << ',' << point.y << ',' << point.z << ']';
}

std::vector<std::uint64_t> parseEdgeIds(const std::string & value)
{
  std::vector<std::uint64_t> result;
  std::size_t begin = 0U;
  while (begin < value.size()) {
    const std::size_t end = value.find(',', begin);
    const std::string token = value.substr(begin, end == std::string::npos ? end : end - begin);
    if (!token.empty()) {
      result.push_back(static_cast<std::uint64_t>(std::stoull(token)));
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return result;
}

std::string joinEdgeIds(const std::vector<std::uint64_t> & ids)
{
  std::ostringstream stream;
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << ids[index];
  }
  return stream.str();
}

bool finiteScalar(const double value)
{
  return std::isfinite(value);
}

bool parseSemanticEnabled(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (value == "1" || value == "true" || value == "yes") {return true;}
  if (value == "0" || value == "false" || value == "no") {return false;}
  throw std::invalid_argument(
          "semantic enabled field must be 0/1, true/false, or yes/no, got: " + value);
}

bool pointInPolygon(const Vec3 & point, const std::vector<Vec3> & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0U, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
    const Vec3 & a = polygon[i];
    const Vec3 & b = polygon[j];
    const bool y_crosses = (a.y > point.y) != (b.y > point.y);
    if (!y_crosses) {
      continue;
    }
    const double denominator = b.y - a.y;
    if (std::abs(denominator) < 1.0e-15) {
      continue;
    }
    const double x_at_y = (b.x - a.x) * (point.y - a.y) / denominator + a.x;
    if (point.x < x_at_y) {
      inside = !inside;
    }
  }
  return inside;
}

double orientation(const Vec3 & a, const Vec3 & b, const Vec3 & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment(const Vec3 & a, const Vec3 & b, const Vec3 & point)
{
  constexpr double epsilon = 1.0e-9;
  return std::abs(orientation(a, b, point)) <= epsilon &&
         point.x >= std::min(a.x, b.x) - epsilon &&
         point.x <= std::max(a.x, b.x) + epsilon &&
         point.y >= std::min(a.y, b.y) - epsilon &&
         point.y <= std::max(a.y, b.y) + epsilon;
}

double cross2d(const Vec3 & lhs, const Vec3 & rhs)
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

bool segmentsIntersect2d(const Vec3 & a, const Vec3 & b, const Vec3 & c, const Vec3 & d)
{
  constexpr double epsilon = 1.0e-9;
  const double ab_c = orientation(a, b, c);
  const double ab_d = orientation(a, b, d);
  const double cd_a = orientation(c, d, a);
  const double cd_b = orientation(c, d, b);
  if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
    ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon)))
  {
    return true;
  }
  return (std::abs(ab_c) <= epsilon && onSegment(a, b, c)) ||
         (std::abs(ab_d) <= epsilon && onSegment(a, b, d)) ||
         (std::abs(cd_a) <= epsilon && onSegment(c, d, a)) ||
         (std::abs(cd_b) <= epsilon && onSegment(c, d, b));
}

bool isSimpleNonzeroPolygon(const std::vector<Vec3> & polygon)
{
  double twice_area = 0.0;
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    const Vec3 & a = polygon[index];
    const Vec3 & b = polygon[(index + 1U) % polygon.size()];
    if (std::hypot(b.x - a.x, b.y - a.y) <= 1.0e-9) {return false;}
    twice_area += a.x * b.y - b.x * a.y;
  }
  if (std::abs(twice_area) <= 1.0e-10) {return false;}
  for (std::size_t first = 0U; first < polygon.size(); ++first) {
    const std::size_t first_next = (first + 1U) % polygon.size();
    for (std::size_t second = first + 1U; second < polygon.size(); ++second) {
      const std::size_t second_next = (second + 1U) % polygon.size();
      if (first == second || first_next == second || second_next == first) {continue;}
      if (segmentsIntersect2d(
          polygon[first], polygon[first_next], polygon[second], polygon[second_next]))
      {
        return false;
      }
    }
  }
  return true;
}

Vec3 interpolate(const Vec3 & from, const Vec3 & to, const double ratio)
{
  return from + (to - from) * ratio;
}

std::vector<double> cumulativeLengths(const std::vector<Vec3> & points)
{
  std::vector<double> result(points.size(), 0.0);
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result[index] = result[index - 1U] + distance3d(points[index - 1U], points[index]);
  }
  return result;
}

double routeEdgeLength(const RouteEdge & edge)
{
  return polylineLength(edge.centerline);
}

Vec3 pointAtArcLength(
  const std::vector<Vec3> & points,
  const std::vector<double> & cumulative,
  const double distance)
{
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1U || distance <= 0.0) {
    return points.front();
  }
  if (distance >= cumulative.back()) {
    return points.back();
  }
  const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), distance);
  const std::size_t index = static_cast<std::size_t>(upper - cumulative.begin());
  const double segment_length = cumulative[index] - cumulative[index - 1U];
  if (!(segment_length > 1.0e-12)) {
    return points[index];
  }
  const double ratio = (distance - cumulative[index - 1U]) / segment_length;
  return interpolate(points[index - 1U], points[index], ratio);
}

std::vector<Vec3> slicePolyline(
  const std::vector<Vec3> & points,
  const double start_s,
  const double end_s)
{
  if (points.size() < 2U || !(end_s > start_s)) {
    return {};
  }
  const std::vector<double> cumulative = cumulativeLengths(points);
  const double length = cumulative.back();
  const double begin = clamp(start_s, 0.0, length);
  const double end = clamp(end_s, 0.0, length);
  if (!(end > begin + 1.0e-12)) {
    return {};
  }
  std::vector<Vec3> result;
  result.push_back(pointAtArcLength(points, cumulative, begin));
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    if (cumulative[index] > begin + 1.0e-10 && cumulative[index] < end - 1.0e-10) {
      result.push_back(points[index]);
    }
  }
  const Vec3 last = pointAtArcLength(points, cumulative, end);
  if (distance3d(result.back(), last) > 1.0e-10) {
    result.push_back(last);
  } else {
    result.back() = last;
  }
  return result;
}

void addIntersectionParameters(
  const Vec3 & a,
  const Vec3 & b,
  const Vec3 & c,
  const Vec3 & d,
  std::vector<double> & parameters)
{
  constexpr double epsilon = 1.0e-10;
  const Vec3 r = b - a;
  const Vec3 s = d - c;
  const Vec3 q = c - a;
  const double denominator = cross2d(r, s);
  if (std::abs(denominator) > epsilon) {
    const double t = cross2d(q, s) / denominator;
    const double u = cross2d(q, r) / denominator;
    if (t >= -epsilon && t <= 1.0 + epsilon && u >= -epsilon && u <= 1.0 + epsilon) {
      parameters.push_back(clamp(t, 0.0, 1.0));
    }
    return;
  }
  if (std::abs(cross2d(q, r)) > epsilon) {
    return;
  }
  const double squared_length = r.x * r.x + r.y * r.y;
  if (!(squared_length > epsilon)) {
    return;
  }
  for (const Vec3 & endpoint : {c, d}) {
    const double t = ((endpoint.x - a.x) * r.x + (endpoint.y - a.y) * r.y) / squared_length;
    if (t >= -epsilon && t <= 1.0 + epsilon) {
      parameters.push_back(clamp(t, 0.0, 1.0));
    }
  }
}

bool pointOnPolygonBoundary(const Vec3 & point, const std::vector<Vec3> & polygon)
{
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    if (onSegment(polygon[index], polygon[(index + 1U) % polygon.size()], point)) {
      return true;
    }
  }
  return false;
}

std::vector<RouteEdgeSpan> mergeSpans(std::vector<RouteEdgeSpan> spans)
{
  constexpr double epsilon = 1.0e-8;
  std::sort(
    spans.begin(), spans.end(),
    [](const RouteEdgeSpan & lhs, const RouteEdgeSpan & rhs) {
      if (lhs.edge_id != rhs.edge_id) {return lhs.edge_id < rhs.edge_id;}
      if (lhs.start_s != rhs.start_s) {return lhs.start_s < rhs.start_s;}
      return lhs.end_s < rhs.end_s;
    });
  std::vector<RouteEdgeSpan> result;
  for (const RouteEdgeSpan & span : spans) {
    if (!(span.end_s > span.start_s + epsilon)) {
      continue;
    }
    if (!result.empty() && result.back().edge_id == span.edge_id &&
      span.start_s <= result.back().end_s + epsilon)
    {
      result.back().end_s = std::max(result.back().end_s, span.end_s);
    } else {
      result.push_back(span);
    }
  }
  return result;
}

std::vector<RouteEdgeSpan> polygonSpans(
  const RouteEdge & edge,
  const std::vector<Vec3> & polygon)
{
  if (edge.centerline.size() < 2U || polygon.size() < 3U) {
    return {};
  }
  std::vector<RouteEdgeSpan> result;
  const std::vector<double> cumulative = cumulativeLengths(edge.centerline);
  for (std::size_t line_index = 1U; line_index < edge.centerline.size(); ++line_index) {
    const Vec3 & a = edge.centerline[line_index - 1U];
    const Vec3 & b = edge.centerline[line_index];
    const double segment_length = cumulative[line_index] - cumulative[line_index - 1U];
    if (!(segment_length > 1.0e-12)) {
      continue;
    }
    std::vector<double> parameters{0.0, 1.0};
    for (std::size_t polygon_index = 0U; polygon_index < polygon.size(); ++polygon_index) {
      addIntersectionParameters(
        a, b, polygon[polygon_index], polygon[(polygon_index + 1U) % polygon.size()], parameters);
    }
    std::sort(parameters.begin(), parameters.end());
    parameters.erase(
      std::unique(
        parameters.begin(), parameters.end(),
        [](const double lhs, const double rhs) {return std::abs(lhs - rhs) < 1.0e-10;}),
      parameters.end());
    for (std::size_t index = 1U; index < parameters.size(); ++index) {
      const double t0 = parameters[index - 1U];
      const double t1 = parameters[index];
      if (!(t1 > t0 + 1.0e-12)) {
        continue;
      }
      const Vec3 midpoint = interpolate(a, b, 0.5 * (t0 + t1));
      if (pointInPolygon(midpoint, polygon) || pointOnPolygonBoundary(midpoint, polygon)) {
        result.push_back({
          edge.id,
          cumulative[line_index - 1U] + t0 * segment_length,
          cumulative[line_index - 1U] + t1 * segment_length,
          std::nullopt, std::nullopt});
      }
    }
  }
  return mergeSpans(std::move(result));
}

const RouteEdge * findEdge(const RouteGraph & graph, std::uint64_t id);

std::vector<RouteEdgeSpan> affectedSpans(
  const SemanticFeature & feature,
  const RouteGraph & graph)
{
  std::vector<RouteEdgeSpan> result;
  if (feature.geometry == SemanticGeometryType::kPolygon) {
    for (const RouteEdge & edge : graph.edges) {
      std::vector<RouteEdgeSpan> spans = polygonSpans(edge, feature.polygon);
      result.insert(result.end(), spans.begin(), spans.end());
    }
  } else if (feature.geometry == SemanticGeometryType::kRouteEdges) {
    if (!feature.route_edge_spans.empty()) {
      result = feature.route_edge_spans;
    } else {
      for (const std::uint64_t edge_id : feature.route_edge_ids) {
        const RouteEdge * edge = findEdge(graph, edge_id);
        if (edge != nullptr) {
          result.push_back({
            edge_id, 0.0, routeEdgeLength(*edge), std::nullopt, std::nullopt});
        }
      }
    }
  }
  return mergeSpans(std::move(result));
}

const RouteEdge * findEdge(const RouteGraph & graph, const std::uint64_t id)
{
  const auto found = std::find_if(
    graph.edges.begin(), graph.edges.end(),
    [id](const RouteEdge & edge) {return edge.id == id;});
  return found == graph.edges.end() ? nullptr : &*found;
}

struct EdgeProjection
{
  double distance{std::numeric_limits<double>::infinity()};
  double arc_s{0.0};
};

EdgeProjection projectToEdge(const Vec3 & point, const RouteEdge & edge)
{
  EdgeProjection best;
  const std::vector<double> cumulative = cumulativeLengths(edge.centerline);
  for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
    const Vec3 & a = edge.centerline[index - 1U];
    const Vec3 & b = edge.centerline[index];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    const double squared_length = dx * dx + dy * dy + dz * dz;
    const double ratio = squared_length > 1.0e-15 ?
      clamp(
      ((point.x - a.x) * dx + (point.y - a.y) * dy + (point.z - a.z) * dz) /
      squared_length, 0.0, 1.0) :
      0.0;
    const Vec3 projected = interpolate(a, b, ratio);
    const double distance = distance3d(point, projected);
    if (distance < best.distance) {
      best.distance = distance;
      best.arc_s = cumulative[index - 1U] +
        ratio * (cumulative[index] - cumulative[index - 1U]);
    }
  }
  return best;
}

struct DirectedPath
{
  std::vector<const RouteEdge *> edges;
  double length{0.0};
  bool ambiguous{false};
};

std::optional<DirectedPath> shortestDirectedPath(
  const RouteGraph & graph,
  const std::uint64_t from,
  const std::uint64_t to,
  const std::set<std::uint64_t> & excluded_edges)
{
  if (from == to) {return DirectedPath{};}
  std::map<std::uint64_t, double> distance;
  std::map<std::uint64_t, unsigned int> ways;
  std::map<std::uint64_t, const RouteEdge *> predecessor;
  std::set<std::uint64_t> visited;
  distance[from] = 0.0;
  ways[from] = 1U;
  while (true) {
    std::uint64_t current = 0U;
    double current_distance = std::numeric_limits<double>::infinity();
    for (const auto & entry : distance) {
      if (visited.count(entry.first) == 0U && entry.second < current_distance) {
        current = entry.first;
        current_distance = entry.second;
      }
    }
    if (current == 0U) {break;}
    visited.insert(current);
    if (current == to) {break;}
    for (const RouteEdge & edge : graph.edges) {
      if (edge.from != current || excluded_edges.count(edge.id) != 0U) {continue;}
      const double candidate = current_distance + std::max(routeEdgeLength(edge), 1.0e-6);
      const auto known = distance.find(edge.to);
      if (known == distance.end() || candidate < known->second - 1.0e-7) {
        distance[edge.to] = candidate;
        ways[edge.to] = ways[current];
        predecessor[edge.to] = &edge;
      } else if (std::abs(candidate - known->second) <= 1.0e-7) {
        ways[edge.to] = std::min(2U, ways[edge.to] + ways[current]);
      }
    }
  }
  const auto total = distance.find(to);
  if (total == distance.end()) {return std::nullopt;}
  DirectedPath result;
  result.length = total->second;
  result.ambiguous = ways[to] > 1U;
  std::uint64_t current = to;
  while (current != from) {
    const auto step = predecessor.find(current);
    if (step == predecessor.end()) {return std::nullopt;}
    result.edges.push_back(step->second);
    current = step->second->from;
  }
  std::reverse(result.edges.begin(), result.edges.end());
  return result;
}

unsigned int countDirectedPathsWithinLength(
  const RouteGraph & graph,
  const std::uint64_t from,
  const std::uint64_t to,
  const std::set<std::uint64_t> & excluded_edges,
  const double maximum_length)
{
  if (from == to) {return 1U;}
  if (!(maximum_length >= 0.0)) {return 0U;}
  unsigned int count = 0U;
  std::set<std::uint64_t> visited_nodes{from};
  const auto visit = [&](const auto & self, const std::uint64_t node, const double length) -> void {
      if (count >= 2U) {return;}
      for (const RouteEdge & edge : graph.edges) {
        if (edge.from != node || excluded_edges.count(edge.id) != 0U) {continue;}
        const double next_length = length + std::max(routeEdgeLength(edge), 1.0e-6);
        if (next_length > maximum_length + 1.0e-9) {continue;}
        if (edge.to == to) {
          ++count;
          if (count >= 2U) {return;}
        } else if (visited_nodes.insert(edge.to).second) {
          self(self, edge.to, next_length);
          visited_nodes.erase(edge.to);
        }
      }
    };
  visit(visit, from, 0.0);
  return count;
}

RouteEdgeSpan anchoredSpan(
  const RouteEdge & edge,
  const double start_s,
  const double end_s,
  const std::optional<Vec3> & start_anchor = std::nullopt,
  const std::optional<Vec3> & end_anchor = std::nullopt)
{
  const std::vector<double> cumulative = cumulativeLengths(edge.centerline);
  RouteEdgeSpan result;
  result.edge_id = edge.id;
  result.start_s = start_s;
  result.end_s = end_s;
  result.start_anchor = start_anchor ? start_anchor :
    std::optional<Vec3>{pointAtArcLength(edge.centerline, cumulative, start_s)};
  result.end_anchor = end_anchor ? end_anchor :
    std::optional<Vec3>{pointAtArcLength(edge.centerline, cumulative, end_s)};
  return result;
}

std::string spanRouteKey(const std::vector<RouteEdgeSpan> & spans)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  for (const RouteEdgeSpan & span : spans) {
    stream << span.edge_id << ':' << span.start_s << ':' << span.end_s << ';';
  }
  return stream.str();
}

std::optional<std::vector<RouteEdgeSpan>> remapAnchoredSpan(
  const RouteEdgeSpan & span,
  const RouteGraph & graph,
  const double maximum_anchor_distance)
{
  if (!span.start_anchor || !span.end_anchor) {
    return findEdge(graph, span.edge_id) != nullptr ?
           std::optional<std::vector<RouteEdgeSpan>>{{span}} : std::nullopt;
  }
  struct ProjectionCandidate
  {
    const RouteEdge * edge{nullptr};
    EdgeProjection projection;
  };
  std::vector<ProjectionCandidate> starts;
  std::vector<ProjectionCandidate> ends;
  for (const RouteEdge & edge : graph.edges) {
    const EdgeProjection start = projectToEdge(*span.start_anchor, edge);
    const EdgeProjection end = projectToEdge(*span.end_anchor, edge);
    if (start.distance <= maximum_anchor_distance) {starts.push_back({&edge, start});}
    if (end.distance <= maximum_anchor_distance) {ends.push_back({&edge, end});}
  }

  std::vector<RouteEdgeSpan> best_spans;
  double best_score = std::numeric_limits<double>::infinity();
  double second_best_score = std::numeric_limits<double>::infinity();
  bool best_path_ambiguous = false;
  std::set<std::string> candidate_keys;
  const double old_length = span.end_s - span.start_s;
  const double maximum_length_error = std::max(0.25, 0.05 * old_length);
  for (const ProjectionCandidate & start : starts) {
    for (const ProjectionCandidate & end : ends) {
      if (start.edge->id != end.edge->id && start.edge->reverse_of &&
        *start.edge->reverse_of == end.edge->id)
      {
        // One old directed span cannot legitimately become an immediate
        // forward-then-reverse U-turn on the same physical corridor.
        continue;
      }
      std::vector<RouteEdgeSpan> route;
      double route_length = 0.0;
      bool path_ambiguous = false;
      if (start.edge->id == end.edge->id) {
        if (!(end.projection.arc_s > start.projection.arc_s + 1.0e-6)) {continue;}
        route.push_back(anchoredSpan(
          *start.edge, start.projection.arc_s, end.projection.arc_s,
          span.start_anchor, span.end_anchor));
        route_length = end.projection.arc_s - start.projection.arc_s;
      } else {
        const std::set<std::uint64_t> excluded{start.edge->id, end.edge->id};
        const std::optional<DirectedPath> middle = shortestDirectedPath(
          graph, start.edge->to, end.edge->from, excluded);
        if (!middle) {continue;}
        path_ambiguous = middle->ambiguous;
        const double start_length = routeEdgeLength(*start.edge);
        const double start_partial = start_length - start.projection.arc_s;
        const double end_partial = end.projection.arc_s;
        const double maximum_middle_length = old_length + maximum_length_error -
          start_partial - end_partial;
        if (countDirectedPathsWithinLength(
            graph, start.edge->to, end.edge->from, excluded, maximum_middle_length) > 1U)
        {
          path_ambiguous = true;
        }
        if (start_length - start.projection.arc_s > 1.0e-6) {
          route.push_back(anchoredSpan(
            *start.edge, start.projection.arc_s, start_length,
            span.start_anchor, std::nullopt));
          route_length += start_length - start.projection.arc_s;
        }
        for (const RouteEdge * edge : middle->edges) {
          const double length = routeEdgeLength(*edge);
          route.push_back(anchoredSpan(*edge, 0.0, length));
          route_length += length;
        }
        if (end.projection.arc_s > 1.0e-6) {
          route.push_back(anchoredSpan(
            *end.edge, 0.0, end.projection.arc_s,
            std::nullopt, span.end_anchor));
          route_length += end.projection.arc_s;
        }
      }
      if (route.empty()) {continue;}
      // Anchors alone cannot prove that a long alternate branch represents the
      // original edited interval. Bound longitudinal drift and reject rather
      // than silently moving a restriction onto a detour.
      if (std::abs(route_length - old_length) > maximum_length_error) {continue;}
      const std::string key = spanRouteKey(route);
      if (!candidate_keys.insert(key).second) {continue;}
      const double score = start.projection.distance + end.projection.distance +
        0.05 * std::abs(route_length - old_length);
      if (score < best_score) {
        second_best_score = best_score;
        best_score = score;
        best_spans = std::move(route);
        best_path_ambiguous = path_ambiguous;
      } else if (score < second_best_score) {
        second_best_score = score;
      }
    }
  }
  if (best_spans.empty() || best_path_ambiguous || second_best_score - best_score < 0.05) {
    return std::nullopt;
  }
  return best_spans;
}

}  // namespace

const char * toString(const SemanticFeatureType type)
{
  switch (type) {
    case SemanticFeatureType::kStop: return "stop";
    case SemanticFeatureType::kWait: return "wait";
    case SemanticFeatureType::kDock: return "dock";
    case SemanticFeatureType::kCharger: return "charger";
    case SemanticFeatureType::kDoor: return "door";
    case SemanticFeatureType::kSpeedLimit: return "speed_limit";
    case SemanticFeatureType::kNoEntry: return "no_entry";
  }
  return "unknown";
}

const char * toString(const SemanticGeometryType type)
{
  switch (type) {
    case SemanticGeometryType::kPoint: return "point";
    case SemanticGeometryType::kRouteEdges: return "route_edges";
    case SemanticGeometryType::kPolygon: return "polygon";
  }
  return "unknown";
}

SemanticFeatureType semanticFeatureTypeFromString(const std::string & value)
{
  if (value == "stop") {return SemanticFeatureType::kStop;}
  if (value == "wait") {return SemanticFeatureType::kWait;}
  if (value == "dock") {return SemanticFeatureType::kDock;}
  if (value == "charger") {return SemanticFeatureType::kCharger;}
  if (value == "door") {return SemanticFeatureType::kDoor;}
  if (value == "speed_limit") {return SemanticFeatureType::kSpeedLimit;}
  if (value == "no_entry") {return SemanticFeatureType::kNoEntry;}
  throw std::runtime_error("unknown semantic feature type: " + value);
}

SemanticGeometryType semanticGeometryTypeFromString(const std::string & value)
{
  if (value == "point") {return SemanticGeometryType::kPoint;}
  if (value == "route_edges") {return SemanticGeometryType::kRouteEdges;}
  if (value == "polygon") {return SemanticGeometryType::kPolygon;}
  throw std::runtime_error("unknown semantic geometry type: " + value);
}

void validateSemanticMap(const SemanticMap & map, const RouteGraph * graph)
{
  if (map.frame_id.empty()) {
    throw std::invalid_argument("semantic map frame_id must not be empty");
  }
  std::set<std::uint64_t> ids;
  std::set<std::uint64_t> graph_edge_ids;
  if (graph != nullptr) {
    if (graph->frame_id != map.frame_id) {
      throw std::invalid_argument(
              "semantic map frame '" + map.frame_id + "' differs from route graph frame '" +
              graph->frame_id + "'");
    }
    for (const RouteEdge & edge : graph->edges) {
      graph_edge_ids.insert(edge.id);
    }
  }
  for (const SemanticFeature & feature : map.features) {
    if (feature.id == 0U || !ids.insert(feature.id).second) {
      throw std::invalid_argument(
              "semantic feature IDs must be unique and non-zero; invalid ID=" +
              std::to_string(feature.id));
    }
    if (!finite(feature.position) || !finiteScalar(feature.yaw) ||
      !finiteScalar(feature.value) || !finiteScalar(feature.extent))
    {
      throw std::invalid_argument(
              "semantic feature contains non-finite values; ID=" +
              std::to_string(feature.id));
    }
    const bool point_type =
      feature.type == SemanticFeatureType::kStop ||
      feature.type == SemanticFeatureType::kWait ||
      feature.type == SemanticFeatureType::kDock ||
      feature.type == SemanticFeatureType::kCharger ||
      feature.type == SemanticFeatureType::kDoor;
    if (point_type && feature.geometry != SemanticGeometryType::kPoint) {
      throw std::invalid_argument(
              "point semantic must use point geometry; ID=" + std::to_string(feature.id));
    }
    if (!point_type && feature.geometry == SemanticGeometryType::kPoint) {
      throw std::invalid_argument(
              "speed_limit/no_entry must use route_edges or polygon geometry; ID=" +
              std::to_string(feature.id));
    }
    if (feature.type == SemanticFeatureType::kSpeedLimit && !(feature.value > 0.0)) {
      throw std::invalid_argument(
              "speed_limit value must be positive; ID=" + std::to_string(feature.id));
    }
    if (feature.type == SemanticFeatureType::kDoor && !(feature.extent > 0.0)) {
      throw std::invalid_argument(
              "door width must be positive; ID=" + std::to_string(feature.id));
    }
    if (feature.geometry == SemanticGeometryType::kRouteEdges && feature.route_edge_ids.empty()) {
      if (feature.route_edge_spans.empty()) {
        throw std::invalid_argument(
                "route_edges semantic has no edge references; ID=" + std::to_string(feature.id));
      }
    }
    if (feature.geometry != SemanticGeometryType::kRouteEdges &&
      !feature.route_edge_spans.empty())
    {
      throw std::invalid_argument(
              "only route_edges semantics may contain edge spans; ID=" +
              std::to_string(feature.id));
    }
    for (const RouteEdgeSpan & span : feature.route_edge_spans) {
      if (span.edge_id == 0U || !finiteScalar(span.start_s) || !finiteScalar(span.end_s) ||
        span.start_s < 0.0 || !(span.end_s > span.start_s))
      {
        throw std::invalid_argument(
                "semantic feature contains invalid route edge span; ID=" +
                std::to_string(feature.id));
      }
      if (span.start_anchor.has_value() != span.end_anchor.has_value() ||
        (span.start_anchor && (!finite(*span.start_anchor) || !finite(*span.end_anchor))))
      {
        throw std::invalid_argument(
                "semantic feature span anchors must be a finite pair; ID=" +
                std::to_string(feature.id));
      }
    }
    if (feature.geometry == SemanticGeometryType::kPolygon) {
      if (feature.polygon.size() < 3U) {
        throw std::invalid_argument(
                "polygon semantic requires at least three vertices; ID=" +
                std::to_string(feature.id));
      }
      for (const Vec3 & vertex : feature.polygon) {
        if (!finite(vertex)) {
          throw std::invalid_argument(
                  "polygon semantic contains non-finite vertex; ID=" +
                  std::to_string(feature.id));
        }
      }
      if (!isSimpleNonzeroPolygon(feature.polygon)) {
        throw std::invalid_argument(
                "polygon semantic must be a simple polygon with non-zero XY area; ID=" +
                std::to_string(feature.id));
      }
    }
    if (graph != nullptr) {
      for (const std::uint64_t edge_id : feature.route_edge_ids) {
        if (graph_edge_ids.count(edge_id) == 0U) {
          throw std::invalid_argument(
                  "semantic feature references unknown route edge " +
                  std::to_string(edge_id) + "; feature ID=" + std::to_string(feature.id));
        }
      }
      for (const RouteEdgeSpan & span : feature.route_edge_spans) {
        const RouteEdge * edge = findEdge(*graph, span.edge_id);
        if (edge == nullptr) {
          throw std::invalid_argument(
                  "semantic feature span references unknown route edge " +
                  std::to_string(span.edge_id) + "; feature ID=" +
                  std::to_string(feature.id));
        }
        const double length = routeEdgeLength(*edge);
        const double tolerance = std::max(1.0e-8, 1.0e-8 * length);
        if (span.end_s > length + tolerance) {
          throw std::invalid_argument(
                  "semantic feature span exceeds route edge length " +
                  std::to_string(span.edge_id) + "; feature ID=" +
                  std::to_string(feature.id));
        }
      }
    }
  }
}

void saveSemanticMapTsv(
  const std::filesystem::path & path,
  const SemanticMap & map)
{
  validateSemanticMap(map);
  std::ofstream stream = openOutput(path);
  stream << "LMMG_SEMANTICS\t2\n";
  stream << "FRAME\t" << escapeField(map.frame_id) << '\n';
  for (const SemanticFeature & feature : map.features) {
    std::vector<std::uint64_t> edge_ids = feature.route_edge_ids;
    for (const RouteEdgeSpan & span : feature.route_edge_spans) {
      if (std::find(edge_ids.begin(), edge_ids.end(), span.edge_id) == edge_ids.end()) {
        edge_ids.push_back(span.edge_id);
      }
    }
    stream << "FEATURE\t" << feature.id << '\t'
           << toString(feature.type) << '\t'
           << toString(feature.geometry) << '\t'
           << (feature.enabled ? 1 : 0) << '\t'
           << feature.position.x << '\t' << feature.position.y << '\t'
           << feature.position.z << '\t' << feature.yaw << '\t'
           << feature.value << '\t' << feature.extent << '\t'
           << escapeField(feature.name) << '\t'
           << escapeField(feature.notes) << '\t'
           << joinEdgeIds(edge_ids) << '\n';
    for (std::size_t index = 0U; index < feature.route_edge_spans.size(); ++index) {
      const RouteEdgeSpan & span = feature.route_edge_spans[index];
      stream << "SPAN\t" << feature.id << '\t' << index << '\t'
             << span.edge_id << '\t' << span.start_s << '\t' << span.end_s;
      if (span.start_anchor && span.end_anchor) {
        stream << '\t' << span.start_anchor->x << '\t' << span.start_anchor->y << '\t'
               << span.start_anchor->z << '\t' << span.end_anchor->x << '\t'
               << span.end_anchor->y << '\t' << span.end_anchor->z;
      }
      stream << '\n';
    }
    for (std::size_t index = 0U; index < feature.polygon.size(); ++index) {
      const Vec3 & vertex = feature.polygon[index];
      stream << "VERTEX\t" << feature.id << '\t' << index << '\t'
             << vertex.x << '\t' << vertex.y << '\t' << vertex.z << '\n';
    }
  }
}

SemanticMap loadSemanticMapTsv(
  const std::filesystem::path & path,
  const RouteGraph * graph)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open semantic map: " + path.string());
  }
  SemanticMap result;
  std::map<std::uint64_t, std::size_t> feature_indices;
  std::map<std::uint64_t, std::map<std::size_t, Vec3>> vertices;
  std::map<std::uint64_t, std::map<std::size_t, RouteEdgeSpan>> spans;
  bool header_seen = false;
  unsigned int format_version = 0U;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(stream, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const std::vector<std::string> fields = splitTabs(line);
    if (fields.empty()) {
      continue;
    }
    try {
      // Accept files produced before the project was renamed.  The legacy
      // PCVM header used the same versioned record layout; newly saved files
      // are always normalized to the current LMMG header by
      // saveSemanticMapTsv().
      if (fields[0] == "LMMG_SEMANTICS" || fields[0] == "PCVM_SEMANTICS") {
        if (fields.size() != 2U || (fields[1] != "1" && fields[1] != "2")) {
          throw std::runtime_error("unsupported semantic map format version");
        }
        format_version = static_cast<unsigned int>(std::stoul(fields[1]));
        header_seen = true;
      } else if (fields[0] == "FRAME") {
        if (fields.size() != 2U) {
          throw std::runtime_error("FRAME record must contain two fields");
        }
        result.frame_id = unescapeField(fields[1]);
      } else if (fields[0] == "FEATURE") {
        if (fields.size() != 14U) {
          throw std::runtime_error(
                  "FEATURE record must contain 14 fields, got " +
                  std::to_string(fields.size()));
        }
        SemanticFeature feature;
        feature.id = static_cast<std::uint64_t>(std::stoull(fields[1]));
        feature.type = semanticFeatureTypeFromString(fields[2]);
        feature.geometry = semanticGeometryTypeFromString(fields[3]);
        feature.enabled = parseSemanticEnabled(fields[4]);
        feature.position = {std::stod(fields[5]), std::stod(fields[6]), std::stod(fields[7])};
        feature.yaw = std::stod(fields[8]);
        feature.value = std::stod(fields[9]);
        feature.extent = std::stod(fields[10]);
        feature.name = unescapeField(fields[11]);
        feature.notes = unescapeField(fields[12]);
        feature.route_edge_ids = parseEdgeIds(fields[13]);
        if (feature_indices.count(feature.id) != 0U) {
          throw std::runtime_error("duplicate FEATURE ID " + std::to_string(feature.id));
        }
        feature_indices[feature.id] = result.features.size();
        result.features.push_back(std::move(feature));
      } else if (fields[0] == "VERTEX") {
        if (fields.size() != 6U) {
          throw std::runtime_error("VERTEX record must contain six fields");
        }
        const std::uint64_t id = static_cast<std::uint64_t>(std::stoull(fields[1]));
        const std::size_t index = static_cast<std::size_t>(std::stoull(fields[2]));
        if (vertices[id].count(index) != 0U) {
          throw std::runtime_error(
                  "duplicate VERTEX index " + std::to_string(index) + " for feature " +
                  std::to_string(id));
        }
        vertices[id][index] = {std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5])};
      } else if (fields[0] == "SPAN") {
        if (format_version < 2U) {
          throw std::runtime_error("SPAN records require semantic map format version 2");
        }
        if (fields.size() != 6U && fields.size() != 12U) {
          throw std::runtime_error("SPAN record must contain six or twelve fields");
        }
        const std::uint64_t id = static_cast<std::uint64_t>(std::stoull(fields[1]));
        const std::size_t index = static_cast<std::size_t>(std::stoull(fields[2]));
        RouteEdgeSpan span{
          static_cast<std::uint64_t>(std::stoull(fields[3])),
          std::stod(fields[4]), std::stod(fields[5]), std::nullopt, std::nullopt};
        if (fields.size() == 12U) {
          span.start_anchor = Vec3{
            std::stod(fields[6]), std::stod(fields[7]), std::stod(fields[8])};
          span.end_anchor = Vec3{
            std::stod(fields[9]), std::stod(fields[10]), std::stod(fields[11])};
        }
        if (spans[id].count(index) != 0U) {
          throw std::runtime_error(
                  "duplicate SPAN index " + std::to_string(index) + " for feature " +
                  std::to_string(id));
        }
        spans[id][index] = std::move(span);
      } else {
        throw std::runtime_error("unknown semantic map record: " + fields[0]);
      }
    } catch (const std::exception & exception) {
      throw std::runtime_error(
              path.string() + ':' + std::to_string(line_number) + ": " + exception.what());
    }
  }
  if (!header_seen) {
    throw std::runtime_error("semantic map header is missing: " + path.string());
  }
  for (const auto & entry : vertices) {
    const auto feature = feature_indices.find(entry.first);
    if (feature == feature_indices.end()) {
      throw std::runtime_error(
              "VERTEX references unknown semantic feature " + std::to_string(entry.first));
    }
    std::vector<Vec3> ordered;
    ordered.reserve(entry.second.size());
    std::size_t expected = 0U;
    for (const auto & vertex : entry.second) {
      if (vertex.first != expected) {
        throw std::runtime_error(
                "polygon vertex indices must be contiguous for feature " +
                std::to_string(entry.first));
      }
      ordered.push_back(vertex.second);
      ++expected;
    }
    result.features[feature->second].polygon = std::move(ordered);
  }
  for (const auto & entry : spans) {
    const auto feature = feature_indices.find(entry.first);
    if (feature == feature_indices.end()) {
      throw std::runtime_error(
              "SPAN references unknown semantic feature " + std::to_string(entry.first));
    }
    std::vector<RouteEdgeSpan> ordered;
    ordered.reserve(entry.second.size());
    std::size_t expected = 0U;
    for (const auto & span : entry.second) {
      if (span.first != expected) {
        throw std::runtime_error(
                "route edge span indices must be contiguous for feature " +
                std::to_string(entry.first));
      }
      ordered.push_back(span.second);
      ++expected;
    }
    result.features[feature->second].route_edge_spans = std::move(ordered);
  }
  // Reject malformed intrinsic values before anchors are allowed to rebind an
  // ID or interval. Otherwise a negative/zero source span could be laundered
  // into a valid-looking interval by the geometry remapper.
  validateSemanticMap(result);
  if (graph != nullptr) {
    result = remapSemanticMapToGraph(result, *graph);
  }
  validateSemanticMap(result, graph);
  return result;
}

void saveSemanticMapGeoJson(
  const std::filesystem::path & path,
  const SemanticMap & map,
  const RouteGraph & graph)
{
  validateSemanticMap(map, &graph);
  std::ofstream stream = openOutput(path);
  stream << "{\n  \"type\":\"FeatureCollection\",\n"
         << "  \"name\":\"semantic_features\",\n"
         << "  \"frame_id\":\"" << jsonEscape(map.frame_id) << "\",\n"
         << "  \"features\":[\n";
  bool first_feature = true;
  for (const SemanticFeature & feature : map.features) {
    if (!first_feature) {
      stream << ",\n";
    }
    first_feature = false;
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"id\":" << feature.id << ','
           << "\"semantic_type\":\"" << toString(feature.type) << "\","
           << "\"geometry_mode\":\"" << toString(feature.geometry) << "\","
           << "\"enabled\":" << (feature.enabled ? "true" : "false") << ','
           << "\"name\":\"" << jsonEscape(feature.name) << "\","
           << "\"notes\":\"" << jsonEscape(feature.notes) << "\","
           << "\"yaw_rad\":" << feature.yaw << ','
           << "\"value\":" << feature.value << ','
           << "\"extent_m\":" << feature.extent << ','
           << "\"route_edge_ids\":[";
    for (std::size_t index = 0U; index < feature.route_edge_ids.size(); ++index) {
      if (index > 0U) {stream << ',';}
      stream << feature.route_edge_ids[index];
    }
    stream << "],\"route_edge_spans\":[";
    for (std::size_t index = 0U; index < feature.route_edge_spans.size(); ++index) {
      if (index > 0U) {stream << ',';}
      const RouteEdgeSpan & span = feature.route_edge_spans[index];
      stream << "{\"edge_id\":" << span.edge_id
             << ",\"start_s\":" << span.start_s
             << ",\"end_s\":" << span.end_s;
      if (span.start_anchor && span.end_anchor) {
        stream << ",\"start_anchor\":";
        writeCoordinate3d(stream, *span.start_anchor);
        stream << ",\"end_anchor\":";
        writeCoordinate3d(stream, *span.end_anchor);
      }
      stream << '}';
    }
    stream << "]},\"geometry\":{";
    if (feature.geometry == SemanticGeometryType::kPoint) {
      stream << "\"type\":\"Point\",\"coordinates\":";
      writeCoordinate3d(stream, feature.position);
    } else if (feature.geometry == SemanticGeometryType::kPolygon) {
      stream << "\"type\":\"Polygon\",\"coordinates\":[[";
      for (std::size_t index = 0U; index < feature.polygon.size(); ++index) {
        if (index > 0U) {stream << ',';}
        writeCoordinate3d(stream, feature.polygon[index]);
      }
      if (!feature.polygon.empty()) {
        stream << ',';
        writeCoordinate3d(stream, feature.polygon.front());
      }
      stream << "]]";
    } else {
      stream << "\"type\":\"MultiLineString\",\"coordinates\":[";
      bool first_line = true;
      for (const RouteEdgeSpan & span : affectedSpans(feature, graph)) {
        const RouteEdge * edge = findEdge(graph, span.edge_id);
        if (edge == nullptr) {
          continue;
        }
        const std::vector<Vec3> geometry = slicePolyline(
          edge->centerline, span.start_s, span.end_s);
        if (geometry.size() < 2U) {
          continue;
        }
        if (!first_line) {stream << ',';}
        first_line = false;
        stream << '[';
        for (std::size_t index = 0U; index < geometry.size(); ++index) {
          if (index > 0U) {stream << ',';}
          writeCoordinate3d(stream, geometry[index]);
        }
        stream << ']';
      }
      stream << ']';
    }
    stream << "}}";
  }
  stream << "\n  ]\n}\n";
}

std::vector<EdgeSemanticRule> deriveEdgeSemanticRules(
  const SemanticMap & map,
  const RouteGraph & graph)
{
  const std::vector<EdgeSemanticSegmentRule> segments =
    deriveEdgeSemanticSegmentRules(map, graph);
  std::vector<EdgeSemanticRule> result;
  result.reserve(graph.edges.size());
  std::unordered_map<std::uint64_t, std::size_t> indices;
  std::vector<bool> has_segment;
  has_segment.reserve(graph.edges.size());
  for (const RouteEdge & edge : graph.edges) {
    EdgeSemanticRule rule;
    rule.edge_id = edge.id;
    rule.base_passable = edge.passable;
    rule.effective_passable = edge.passable;
    rule.base_speed_limit_mps = edge.recommended_speed_mps;
    rule.effective_speed_limit_mps = edge.recommended_speed_mps;
    indices[edge.id] = result.size();
    result.push_back(rule);
    has_segment.push_back(false);
  }
  // This is a conservative whole-edge summary retained for version-1
  // consumers. Operational exports use deriveEdgeSemanticSegmentRules().
  for (const EdgeSemanticSegmentRule & segment : segments) {
    const auto found = indices.find(segment.edge_id);
    if (found == indices.end()) {continue;}
    EdgeSemanticRule & rule = result[found->second];
    rule.no_entry = rule.no_entry || segment.no_entry;
    if (segment.no_entry) {
      rule.effective_passable = false;
    }
    const std::size_t rule_index = found->second;
    if (!has_segment[rule_index] || !(rule.effective_speed_limit_mps > 0.0)) {
      rule.effective_speed_limit_mps = segment.effective_speed_limit_mps;
    } else if (segment.effective_speed_limit_mps > 0.0) {
      rule.effective_speed_limit_mps = std::min(
        rule.effective_speed_limit_mps, segment.effective_speed_limit_mps);
    }
    has_segment[rule_index] = true;
    for (const std::uint64_t source_id : segment.source_feature_ids) {
      if (std::find(
          rule.source_feature_ids.begin(), rule.source_feature_ids.end(), source_id) ==
        rule.source_feature_ids.end())
      {
        rule.source_feature_ids.push_back(source_id);
      }
    }
  }
  return result;
}

std::vector<EdgeSemanticSegmentRule> deriveEdgeSemanticSegmentRules(
  const SemanticMap & map,
  const RouteGraph & graph)
{
  validateSemanticMap(map, &graph);
  struct Restriction
  {
    const SemanticFeature * feature{nullptr};
    std::vector<RouteEdgeSpan> spans;
  };
  std::vector<Restriction> restrictions;
  for (const SemanticFeature & feature : map.features) {
    if (!feature.enabled ||
      (feature.type != SemanticFeatureType::kSpeedLimit &&
      feature.type != SemanticFeatureType::kNoEntry))
    {
      continue;
    }
    std::vector<RouteEdgeSpan> spans = affectedSpans(feature, graph);
    if (!spans.empty()) {
      restrictions.push_back({&feature, std::move(spans)});
    }
  }

  std::vector<EdgeSemanticSegmentRule> result;
  for (const RouteEdge & edge : graph.edges) {
    const double length = routeEdgeLength(edge);
    if (!(length > 1.0e-10)) {
      continue;
    }
    std::vector<double> breaks{0.0, length};
    for (const Restriction & restriction : restrictions) {
      for (const RouteEdgeSpan & span : restriction.spans) {
        if (span.edge_id == edge.id) {
          breaks.push_back(clamp(span.start_s, 0.0, length));
          breaks.push_back(clamp(span.end_s, 0.0, length));
        }
      }
    }
    std::sort(breaks.begin(), breaks.end());
    breaks.erase(
      std::unique(
        breaks.begin(), breaks.end(),
        [](const double lhs, const double rhs) {return std::abs(lhs - rhs) < 1.0e-8;}),
      breaks.end());
    for (std::size_t index = 1U; index < breaks.size(); ++index) {
      const double start_s = breaks[index - 1U];
      const double end_s = breaks[index];
      if (!(end_s > start_s + 1.0e-9)) {
        continue;
      }
      EdgeSemanticSegmentRule rule;
      rule.edge_id = edge.id;
      rule.start_s = start_s;
      rule.end_s = end_s;
      rule.base_passable = edge.passable;
      rule.effective_passable = edge.passable;
      rule.base_speed_limit_mps = edge.recommended_speed_mps;
      rule.effective_speed_limit_mps = edge.recommended_speed_mps;
      bool has_authored_speed_limit = false;
      const double midpoint = 0.5 * (start_s + end_s);
      for (const Restriction & restriction : restrictions) {
        const bool applies = std::any_of(
          restriction.spans.begin(), restriction.spans.end(),
          [&](const RouteEdgeSpan & span) {
            return span.edge_id == edge.id && midpoint >= span.start_s - 1.0e-8 &&
                   midpoint <= span.end_s + 1.0e-8;
          });
        if (!applies) {continue;}
        rule.source_feature_ids.push_back(restriction.feature->id);
        if (restriction.feature->type == SemanticFeatureType::kNoEntry) {
          rule.no_entry = true;
          rule.effective_passable = false;
        } else if (!has_authored_speed_limit) {
          rule.effective_speed_limit_mps = restriction.feature->value;
          has_authored_speed_limit = true;
        } else {
          // The generated RouteEdge speed is the default used outside an
          // authored interval, not an upper bound on an explicit map speed
          // limit.  Within an interval, preserve the user's value exactly;
          // only multiple overlapping authored limits are combined by taking
          // the most restrictive positive value.
          rule.effective_speed_limit_mps = std::min(
            rule.effective_speed_limit_mps, restriction.feature->value);
        }
      }
      result.push_back(std::move(rule));
    }
  }
  return result;
}

RouteEdgeSpan reverseRouteEdgeSpan(
  const RouteEdgeSpan & span,
  const RouteGraph & graph)
{
  const RouteEdge * source = findEdge(graph, span.edge_id);
  if (source == nullptr) {
    throw std::invalid_argument("unknown route edge " + std::to_string(span.edge_id));
  }
  if (!source->reverse_of) {
    throw std::invalid_argument(
            "route edge has no reverse_of counterpart: " + std::to_string(span.edge_id));
  }
  const RouteEdge * reverse = findEdge(graph, *source->reverse_of);
  if (reverse == nullptr) {
    throw std::invalid_argument(
            "reverse_of references unknown route edge " + std::to_string(*source->reverse_of));
  }
  const double source_length = routeEdgeLength(*source);
  const double reverse_length = routeEdgeLength(*reverse);
  const double scale = source_length > 1.0e-12 ? reverse_length / source_length : 1.0;
  return {
    reverse->id,
    clamp((source_length - span.end_s) * scale, 0.0, reverse_length),
    clamp((source_length - span.start_s) * scale, 0.0, reverse_length),
    span.end_anchor, span.start_anchor};
}

SemanticMap remapSemanticMapToGraph(
  const SemanticMap & map,
  const RouteGraph & graph,
  const double maximum_anchor_distance)
{
  if (!(maximum_anchor_distance > 0.0) || !std::isfinite(maximum_anchor_distance)) {
    throw std::invalid_argument("maximum semantic anchor distance must be finite and positive");
  }
  SemanticMap result = map;
  for (SemanticFeature & feature : result.features) {
    if (feature.route_edge_spans.empty()) {continue;}
    std::vector<RouteEdgeSpan> remapped;
    for (const RouteEdgeSpan & span : feature.route_edge_spans) {
      const std::optional<std::vector<RouteEdgeSpan>> route = remapAnchoredSpan(
        span, graph, maximum_anchor_distance);
      if (!route) {
        throw std::runtime_error(
                "cannot uniquely geometry-remap route edge span for feature " +
                std::to_string(feature.id) + "; old edge ID=" +
                std::to_string(span.edge_id));
      }
      remapped.insert(remapped.end(), route->begin(), route->end());
    }
    feature.route_edge_spans = std::move(remapped);
    feature.route_edge_ids.clear();
    for (const RouteEdgeSpan & span : feature.route_edge_spans) {
      if (std::find(
          feature.route_edge_ids.begin(), feature.route_edge_ids.end(), span.edge_id) ==
        feature.route_edge_ids.end())
      {
        feature.route_edge_ids.push_back(span.edge_id);
      }
    }
  }
  validateSemanticMap(result, &graph);
  return result;
}

bool semanticRouteSpansUseGraphCoordinates(
  const SemanticMap & map,
  const RouteGraph & graph,
  const double maximum_anchor_error)
{
  if (!(maximum_anchor_error >= 0.0) || !std::isfinite(maximum_anchor_error) ||
    map.frame_id != graph.frame_id)
  {
    return false;
  }
  for (const SemanticFeature & feature : map.features) {
    if (feature.geometry != SemanticGeometryType::kRouteEdges) {continue;}
    if (feature.route_edge_spans.empty()) {
      // A legacy whole-edge reference does not prove which graph geometry its
      // Edge ID belongs to when editable and replay graphs reuse IDs.
      return false;
    }
    for (const RouteEdgeSpan & span : feature.route_edge_spans) {
      const RouteEdge * edge = findEdge(graph, span.edge_id);
      if (edge == nullptr || !span.start_anchor || !span.end_anchor) {return false;}
      const std::vector<double> cumulative = cumulativeLengths(edge->centerline);
      if (cumulative.empty() || span.start_s < 0.0 ||
        span.end_s > cumulative.back() + 1.0e-8 || !(span.end_s > span.start_s))
      {
        return false;
      }
      const Vec3 expected_start = pointAtArcLength(
        edge->centerline, cumulative, span.start_s);
      const Vec3 expected_end = pointAtArcLength(
        edge->centerline, cumulative, span.end_s);
      if (distance3d(expected_start, *span.start_anchor) > maximum_anchor_error ||
        distance3d(expected_end, *span.end_anchor) > maximum_anchor_error)
      {
        return false;
      }
    }
  }
  return true;
}

SemanticGraphFilterResult filterSemanticMapForGraph(
  const SemanticMap & map,
  const RouteGraph & graph)
{
  if (map.frame_id != graph.frame_id) {
    throw std::invalid_argument(
            "cannot filter semantic map frame '" + map.frame_id +
            "' for route graph frame '" + graph.frame_id + "'");
  }
  SemanticGraphFilterResult result;
  result.map.frame_id = map.frame_id;
  for (const SemanticFeature & source : map.features) {
    SemanticFeature feature = source;
    if (feature.geometry == SemanticGeometryType::kRouteEdges) {
      feature.route_edge_ids.clear();
      feature.route_edge_spans.clear();
      if (!source.route_edge_spans.empty()) {
        for (const RouteEdgeSpan & span : source.route_edge_spans) {
          const RouteEdge * edge = findEdge(graph, span.edge_id);
          if (edge == nullptr) {
            result.diagnostics.push_back(
              "feature " + std::to_string(source.id) + " span on edited edge " +
              std::to_string(span.edge_id) + " was excluded: edge is absent from the "
              "validated operational graph");
            continue;
          }
          const double length = routeEdgeLength(*edge);
          if (span.start_s < 0.0 || span.end_s > length + std::max(1.0e-8, 1.0e-8 * length) ||
            !(span.end_s > span.start_s))
          {
            result.diagnostics.push_back(
              "feature " + std::to_string(source.id) + " span on edge " +
              std::to_string(span.edge_id) + " was excluded: interval is outside the "
              "validated edge geometry");
            continue;
          }
          feature.route_edge_spans.push_back(span);
          if (std::find(
              feature.route_edge_ids.begin(), feature.route_edge_ids.end(), span.edge_id) ==
            feature.route_edge_ids.end())
          {
            feature.route_edge_ids.push_back(span.edge_id);
          }
        }
      } else {
        for (const std::uint64_t edge_id : source.route_edge_ids) {
          if (findEdge(graph, edge_id) != nullptr) {
            feature.route_edge_ids.push_back(edge_id);
          } else {
            result.diagnostics.push_back(
              "feature " + std::to_string(source.id) + " whole-edge target " +
              std::to_string(edge_id) + " was excluded: edge is absent from the validated "
              "operational graph");
          }
        }
      }
      if (feature.route_edge_ids.empty() && feature.route_edge_spans.empty()) {
        result.excluded_feature_ids.push_back(source.id);
        result.diagnostics.push_back(
          "feature " + std::to_string(source.id) +
          " was excluded from operational semantic outputs: no validated target remains");
        continue;
      }
    } else if (feature.geometry == SemanticGeometryType::kPoint) {
      const auto old_end = feature.route_edge_ids.end();
      feature.route_edge_ids.erase(
        std::remove_if(
          feature.route_edge_ids.begin(), old_end,
          [&](const std::uint64_t edge_id) {return findEdge(graph, edge_id) == nullptr;}),
        old_end);
      feature.route_edge_spans.clear();
    }
    result.map.features.push_back(std::move(feature));
  }
  validateSemanticMap(result.map, &graph);
  return result;
}

void saveSemanticRouteRulesYaml(
  const std::filesystem::path & path,
  const SemanticMap & map,
  const RouteGraph & graph)
{
  const std::vector<EdgeSemanticRule> rules = deriveEdgeSemanticRules(map, graph);
  const std::vector<EdgeSemanticSegmentRule> segments =
    deriveEdgeSemanticSegmentRules(map, graph);
  std::ofstream stream = openOutput(path);
  stream << "format_version: 2\n";
  stream << "frame_id: " << yamlQuote(map.frame_id) << "\n";
  stream << "semantic_source: \"semantic_features.tsv\"\n";
  stream << "# edges is a conservative version-1 summary; segments is operational.\n";
  stream << "edges:\n";
  for (const EdgeSemanticRule & rule : rules) {
    stream << "  " << rule.edge_id << ":\n"
           << "    base_passable: " << (rule.base_passable ? "true" : "false") << "\n"
           << "    no_entry: " << (rule.no_entry ? "true" : "false") << "\n"
           << "    effective_passable: " << (rule.effective_passable ? "true" : "false") << "\n"
           << "    base_speed_limit_mps: " << rule.base_speed_limit_mps << "\n"
           << "    effective_speed_limit_mps: " << rule.effective_speed_limit_mps << "\n"
           << "    source_feature_ids: [";
    for (std::size_t index = 0U; index < rule.source_feature_ids.size(); ++index) {
      if (index > 0U) {stream << ", ";}
      stream << rule.source_feature_ids[index];
    }
    stream << "]\n";
  }
  stream << "segments:\n";
  for (const EdgeSemanticSegmentRule & rule : segments) {
    stream << "  - source_edge_id: " << rule.edge_id << "\n"
           << "    start_s: " << rule.start_s << "\n"
           << "    end_s: " << rule.end_s << "\n"
           << "    base_passable: " << (rule.base_passable ? "true" : "false") << "\n"
           << "    no_entry: " << (rule.no_entry ? "true" : "false") << "\n"
           << "    effective_passable: " <<
      (rule.effective_passable ? "true" : "false") << "\n"
           << "    base_speed_limit_mps: " << rule.base_speed_limit_mps << "\n"
           << "    effective_speed_limit_mps: " << rule.effective_speed_limit_mps << "\n"
           << "    source_feature_ids: [";
    for (std::size_t index = 0U; index < rule.source_feature_ids.size(); ++index) {
      if (index > 0U) {stream << ", ";}
      stream << rule.source_feature_ids[index];
    }
    stream << "]\n";
  }
}

void saveSemanticRouteGraphGeoJson(
  const std::filesystem::path & path,
  const SemanticMap & map,
  const RouteGraph & graph)
{
  const std::vector<EdgeSemanticSegmentRule> rules =
    deriveEdgeSemanticSegmentRules(map, graph);
  struct DerivedSegment
  {
    EdgeSemanticSegmentRule rule;
    const RouteEdge * source{nullptr};
    std::vector<Vec3> geometry;
    std::uint64_t from{0U};
    std::uint64_t to{0U};
    std::uint64_t id{0U};
    std::optional<std::uint64_t> reverse_of;
  };

  std::uint64_t maximum_id = 0U;
  for (const RouteNode & node : graph.nodes) {maximum_id = std::max(maximum_id, node.id);}
  for (const RouteEdge & edge : graph.edges) {maximum_id = std::max(maximum_id, edge.id);}
  if (maximum_id == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("route graph ID space is exhausted");
  }
  std::uint64_t next_id = maximum_id + 1U;

  using PositionKey = std::tuple<long long, long long, long long>;
  const auto positionKey = [](const Vec3 & point) {
      constexpr double precision = 1.0e6;
      return PositionKey{
        std::llround(point.x * precision), std::llround(point.y * precision),
        std::llround(point.z * precision)};
    };
  using SplitNodeKey = std::tuple<std::uint64_t, PositionKey>;
  std::map<SplitNodeKey, std::uint64_t> split_node_by_source;
  std::vector<RouteNode> derived_nodes = graph.nodes;
  const auto ensureNode = [&](const RouteEdge & source, const Vec3 & point) {
      // Coincident but topologically unrelated routes (for example a bridge
      // crossing) must not acquire a junction merely because semantic split
      // positions share XYZ. Forward/reverse pairs intentionally share their
      // physical-source key so their split endpoints remain paired.
      const std::uint64_t physical_source = source.reverse_of ?
        std::min(source.id, *source.reverse_of) : source.id;
      const SplitNodeKey key{physical_source, positionKey(point)};
      const auto found = split_node_by_source.find(key);
      if (found != split_node_by_source.end()) {return found->second;}
      if (next_id == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("route graph ID space is exhausted");
      }
      const std::uint64_t id = next_id++;
      split_node_by_source[key] = id;
      derived_nodes.push_back({id, point, RouteNodeType::kNormal});
      return id;
    };

  std::unordered_map<std::uint64_t, std::size_t> segment_count;
  for (const EdgeSemanticSegmentRule & rule : rules) {++segment_count[rule.edge_id];}
  std::vector<DerivedSegment> segments;
  segments.reserve(rules.size());
  for (const EdgeSemanticSegmentRule & rule : rules) {
    const RouteEdge * source = findEdge(graph, rule.edge_id);
    if (source == nullptr) {continue;}
    std::vector<Vec3> geometry = slicePolyline(
      source->centerline, rule.start_s, rule.end_s);
    if (geometry.size() < 2U) {continue;}
    const double length = routeEdgeLength(*source);
    const std::uint64_t from = rule.start_s <= 1.0e-8 ? source->from :
      ensureNode(*source, geometry.front());
    const std::uint64_t to = rule.end_s >= length - 1.0e-8 ? source->to :
      ensureNode(*source, geometry.back());
    segments.push_back({rule, source, std::move(geometry), from, to, 0U, std::nullopt});
  }
  for (DerivedSegment & segment : segments) {
    const double length = routeEdgeLength(*segment.source);
    const bool unsplit = segment_count[segment.source->id] == 1U &&
      segment.rule.start_s <= 1.0e-8 && segment.rule.end_s >= length - 1.0e-8;
    if (unsplit) {
      segment.id = segment.source->id;
    } else {
      if (next_id == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("route graph ID space is exhausted");
      }
      segment.id = next_id++;
    }
  }
  for (DerivedSegment & segment : segments) {
    if (!segment.source->reverse_of) {continue;}
    const RouteEdge * reverse_source = findEdge(graph, *segment.source->reverse_of);
    if (reverse_source == nullptr) {continue;}
    const double source_length = routeEdgeLength(*segment.source);
    const double reverse_length = routeEdgeLength(*reverse_source);
    const double scale = source_length > 1.0e-12 ? reverse_length / source_length : 1.0;
    const double expected_start = (source_length - segment.rule.end_s) * scale;
    const double expected_end = (source_length - segment.rule.start_s) * scale;
    const auto reverse = std::find_if(
      segments.begin(), segments.end(),
      [&](const DerivedSegment & candidate) {
        return candidate.source->id == reverse_source->id &&
               std::abs(candidate.rule.start_s - expected_start) < 1.0e-6 &&
               std::abs(candidate.rule.end_s - expected_end) < 1.0e-6;
      });
    if (reverse != segments.end() && reverse->rule.effective_passable &&
      segment.from == reverse->to && segment.to == reverse->from)
    {
      segment.reverse_of = reverse->id;
    }
  }

  std::map<std::uint64_t, std::set<std::uint64_t>> neighbours;
  for (const DerivedSegment & segment : segments) {
    if (!segment.rule.effective_passable) {continue;}
    neighbours[segment.from].insert(segment.to);
    neighbours[segment.to].insert(segment.from);
  }
  for (RouteNode & node : derived_nodes) {
    const auto found = neighbours.find(node.id);
    const std::size_t degree = found == neighbours.end() ? 0U : found->second.size();
    node.type = degree <= 1U ? RouteNodeType::kEndpoint :
      (degree > 2U ? RouteNodeType::kJunction : RouteNodeType::kNormal);
  }

  std::ofstream stream = openOutput(path);
  stream << "{\n  \"type\": \"FeatureCollection\",\n"
         << "  \"name\": \"route_graph_semantic\",\n"
         << "  \"source\": \"route_graph.geojson\",\n"
         << "  \"generated_graph_unchanged\": true,\n"
         << "  \"features\": [\n";
  bool first = true;
  auto separator = [&]() {
      if (!first) {stream << ",\n";}
      first = false;
  };
  for (const RouteNode & node : derived_nodes) {
    const auto found = neighbours.find(node.id);
    if (found == neighbours.end() || found->second.empty()) {continue;}
    separator();
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"id\":" << node.id << ','
           << "\"frame\":\"" << jsonEscape(graph.frame_id) << "\","
           << "\"metadata\":{\"node_type\":\"" << toString(node.type) << "\","
           << "\"z\":" << node.position.z << "}},"
           << "\"geometry\":{\"type\":\"Point\",\"coordinates\":["
           << node.position.x << ',' << node.position.y << "]}}";
  }
  for (const DerivedSegment & segment : segments) {
    if (!segment.rule.effective_passable) {
      continue;
    }
    separator();
    stream << "    {\"type\":\"Feature\",\"properties\":{"
           << "\"id\":" << segment.id << ','
           << "\"startid\":" << segment.from << ','
           << "\"endid\":" << segment.to << ','
           << "\"cost\":" << (segment.rule.end_s - segment.rule.start_s) << ','
           << "\"overridable\":true,"
           << "\"metadata\":{"
           << "\"class\":\"semantic_route\","
           << "\"provenance\":\"generated_route_graph+semantic_features\","
           << "\"validation_status\":\"not_safety_validated\","
           << "\"passable\":true,"
           << "\"source_edge_id\":" << segment.source->id << ','
           << "\"source_start_s\":" << segment.rule.start_s << ','
           << "\"source_end_s\":" << segment.rule.end_s << ','
           << "\"abs_speed_limit\":" << segment.rule.effective_speed_limit_mps << ','
           << "\"base_speed_limit\":" << segment.rule.base_speed_limit_mps << ','
           << "\"semantic_override\":"
           << (!segment.rule.source_feature_ids.empty() ? "true" : "false") << ','
           << "\"minimum_safe_width\":" << segment.source->minimum_safe_width << ','
           << "\"confidence\":" << segment.source->confidence;
    if (segment.reverse_of) {
      stream << ",\"reverse_of\":" << *segment.reverse_of;
    }
    stream << "}},\"geometry\":{\"type\":\"MultiLineString\",\"coordinates\":[[";
    for (std::size_t index = 0U; index < segment.geometry.size(); ++index) {
      if (index > 0U) {stream << ',';}
      stream << '[' << segment.geometry[index].x << ',' << segment.geometry[index].y << ']';
    }
    stream << "]]}}";
  }
  stream << "\n  ]\n}\n";
}

std::uint64_t nextSemanticFeatureId(const SemanticMap & map)
{
  std::uint64_t maximum = 0U;
  for (const SemanticFeature & feature : map.features) {
    maximum = std::max(maximum, feature.id);
  }
  if (maximum == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("semantic feature ID space is exhausted");
  }
  return maximum + 1U;
}

}  // namespace lidar_mobility_map_generator
