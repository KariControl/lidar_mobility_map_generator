#pragma once

#include "lidar_mobility_map_generator/route_editor.hpp"
#include "lidar_mobility_map_generator/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

// RouteEdge currently has no dedicated provenance field.  Observed-route
// edges carry this diagnostic in validation_errors and keep confidence at
// zero so a measured traversal can never be mistaken for a clearance proof.
inline constexpr char kObservedDrivenRouteDiagnostic[] =
  "observed_driven_route_not_safety_validated";

// Build a directional replay chain from the processed trajectory.
//
// Unlike buildRouteGraph(), this function never merges revisits, crossings, or
// a loop's coincident start/end positions.  Every positive-length trajectory
// segment is retained exactly once and in observation order.  Edges are split
// at configured direction-change boundaries and as needed to satisfy
// topology.maximum_edge_length.  topology.minimum_edge_length and
// topology.generate_reverse_edges are intentionally not applied: dropping a
// short observed span or adding a reverse copy would violate the chronological
// replay contract.
[[nodiscard]] RouteGraph buildObservedDrivenRouteGraph(
  const std::vector<TimedPose> & processed_trajectory,
  const TopologyConfig & topology,
  const std::string & frame_id,
  double recommended_speed_mps);

// A terminal localization correction can look like a short reverse traversal
// in map coordinates.  Never alter the lossless observed graph implicitly:
// this derivative is only for Autoware's forward-only Lanelet replay and only
// acts after an operator has explicitly verified the auxiliary localization
// evidence for the concrete data set.
inline constexpr double kAutowareTerminalCuspMinimumHeadingJumpDeg = 150.0;
inline constexpr double kAutowareTerminalSettlingMaximumLength = 1.0;
inline constexpr double kAutowareTerminalSettlingMaximumLengthRatio = 0.01;
inline constexpr double kAutowareTerminalSettlingMaximumBodyYawChangeDeg = 15.0;

struct AutowareReplayCandidateResult
{
  RouteGraph graph;
  bool explicit_verification{false};
  bool terminal_tail_omitted{false};
  double source_length{0.0};
  double retained_length{0.0};
  double omitted_length{0.0};
  double omitted_length_ratio{0.0};
  double connection_heading_jump_deg{0.0};
  double terminal_body_yaw_change_deg{0.0};
  std::size_t source_edges{0U};
  std::size_t retained_edges{0U};
  std::size_t omitted_edges{0U};
  std::string reason{"not_evaluated"};
  std::string verification_provenance{
    "lanelet2.terminal_localization_settling_verified"};
};

// Returns a copy of chronological_replay unless every fail-closed condition
// for a verified terminal settling tail is satisfied.  The returned graph is
// intentionally independent from Nav2 and route_graph_observed_driven.
[[nodiscard]] AutowareReplayCandidateResult materializeAutowareReplayCandidate(
  const RouteGraph & chronological_replay,
  const std::vector<TimedPose> & processed_trajectory,
  bool terminal_localization_settling_verified);

struct ObservedDrivenCandidateResult
{
  RouteGraph graph;
  std::size_t observed_evidence_edges{0U};
  std::size_t independently_validated_edges{0U};
  std::size_t excluded_edges{0U};
  double retained_length{0.0};
};

// Build the supervised replay candidate after route edits and closed-course
// validation. Unchanged/split observed edges are retained even when an
// estimated footprint margin fails: an actual traversal is passage evidence,
// but the original validation errors remain attached. Moved/reversed/manual
// geometry has no such evidence and is included only after independent
// closed-course validation. Generated reverse copies are never inferred.
[[nodiscard]] ObservedDrivenCandidateResult materializeObservedDrivenCandidate(
  const RouteValidationResult & closed_course_validation);

}  // namespace lidar_mobility_map_generator
