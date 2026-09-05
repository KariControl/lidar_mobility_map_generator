#include "lidar_mobility_map_generator/transform_graph.hpp"

#include <algorithm>
#include <queue>
#include <unordered_set>
#include <utility>

namespace lidar_mobility_map_generator
{

std::string normalizeFrameId(std::string frame_id)
{
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
}

void StaticTransformGraph::add(
  const std::string & parent_frame_value,
  const std::string & child_frame_value,
  const Transform & parent_from_child)
{
  const std::string parent = normalizeFrameId(parent_frame_value);
  const std::string child = normalizeFrameId(child_frame_value);
  if (parent.empty() || child.empty() || parent == child ||
    !finite(parent_from_child.translation) ||
    !parent_from_child.rotation.isFinite() ||
    parent_from_child.rotation.squaredNorm() < 1.0e-15)
  {
    return;
  }
  adjacency_[parent].push_back({child, parent_from_child});
  adjacency_[child].push_back({parent, parent_from_child.inverse()});
}

std::optional<Transform> StaticTransformGraph::resolve(
  const std::string & target_frame_value,
  const std::string & source_frame_value) const
{
  const std::string target = normalizeFrameId(target_frame_value);
  const std::string source = normalizeFrameId(source_frame_value);
  if (target.empty() || source.empty()) {
    return std::nullopt;
  }
  if (target == source) {
    return Transform{};
  }
  if (adjacency_.find(target) == adjacency_.end() || adjacency_.find(source) == adjacency_.end()) {
    return std::nullopt;
  }

  struct State
  {
    std::string frame;
    Transform target_from_frame;
  };
  std::queue<State> queue;
  std::unordered_set<std::string> visited;
  queue.push({target, Transform{}});
  visited.insert(target);

  while (!queue.empty()) {
    const State current = queue.front();
    queue.pop();
    const auto adjacency = adjacency_.find(current.frame);
    if (adjacency == adjacency_.end()) {
      continue;
    }
    for (const Edge & edge : adjacency->second) {
      if (!visited.insert(edge.neighbor).second) {
        continue;
      }
      const Transform target_from_neighbor = current.target_from_frame * edge.current_from_neighbor;
      if (edge.neighbor == source) {
        return target_from_neighbor;
      }
      queue.push({edge.neighbor, target_from_neighbor});
    }
  }
  return std::nullopt;
}

}  // namespace lidar_mobility_map_generator
