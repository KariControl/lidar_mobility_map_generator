#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace lidar_mobility_map_generator
{

[[nodiscard]] std::string normalizeFrameId(std::string frame_id);

class StaticTransformGraph
{
public:
  void add(
    const std::string & parent_frame,
    const std::string & child_frame,
    const Transform & parent_from_child);

  [[nodiscard]] std::optional<Transform> resolve(
    const std::string & target_frame,
    const std::string & source_frame) const;

private:
  struct Edge
  {
    std::string neighbor;
    Transform current_from_neighbor;
  };

  std::unordered_map<std::string, std::vector<Edge>> adjacency_;
};

}  // namespace lidar_mobility_map_generator
