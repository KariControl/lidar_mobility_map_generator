#pragma once

#include "lidar_mobility_map_generator/types.hpp"

#include <cstdint>
#include <filesystem>
#include <string>

namespace lidar_mobility_map_generator
{

enum class VectorMapCenterlineSource
{
  kRecordedTrajectory,
  kEditedTopology
};

struct VectorMapSourceSelection
{
  std::uint32_t version{1U};
  VectorMapCenterlineSource source{VectorMapCenterlineSource::kRecordedTrajectory};
  std::string frame_id{"map"};
  std::string graph_fingerprint;
};

[[nodiscard]] const char * toString(VectorMapCenterlineSource source);
[[nodiscard]] VectorMapCenterlineSource vectorMapCenterlineSourceFromString(
  const std::string & value);

void saveVectorMapSourceSelection(
  const std::filesystem::path & path,
  const VectorMapSourceSelection & selection);

[[nodiscard]] VectorMapSourceSelection loadVectorMapSourceSelection(
  const std::filesystem::path & path);

// The fingerprint binds the operator's explicit source choice to the exact
// graph that was visible in the editor. A regeneration must never reinterpret
// the same Edge IDs against different geometry.
void validateVectorMapSourceSelection(
  const VectorMapSourceSelection & selection,
  const RouteGraph & recorded_trajectory,
  const RouteGraph & edited_topology);

// Validate the GUI handoff against the raw graph that the operator actually
// edited, then return the corresponding graph that passed regeneration safety
// checks. Edited centerlines may be densified during those checks, so their
// validated fingerprint is intentionally not used as the authoring identity.
[[nodiscard]] const RouteGraph & validateAndSelectVectorMapSourceGraph(
  const VectorMapSourceSelection & selection,
  const RouteGraph & recorded_trajectory,
  const RouteGraph & raw_edited_topology,
  const RouteGraph & validated_edited_topology);

}  // namespace lidar_mobility_map_generator
