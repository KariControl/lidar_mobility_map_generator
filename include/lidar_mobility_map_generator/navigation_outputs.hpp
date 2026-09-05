#pragma once

#include "lidar_mobility_map_generator/config.hpp"
#include "lidar_mobility_map_generator/nav2_route_export.hpp"
#include "lidar_mobility_map_generator/pipeline.hpp"
#include "lidar_mobility_map_generator/route_editor.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace lidar_mobility_map_generator
{

struct NavigationTargetReadiness
{
  bool nav2_enabled{false};
  bool nav2_map_server_compatible{false};
  bool nav2_route_server_compatible{false};
  bool nav2_navigation_ready{false};
  std::vector<std::string> nav2_reasons;
  bool nav2_closed_course_experimental_ready{false};
  std::vector<std::string> nav2_experimental_reasons;
  std::vector<std::string> nav2_experimental_warnings;

  bool autoware_enabled{false};
  bool autoware_map_loader_compatible{false};
  bool autoware_navigation_ready{false};
  std::vector<std::string> autoware_reasons;
  bool autoware_closed_course_experimental_ready{false};
  std::vector<std::string> autoware_experimental_reasons;
  std::vector<std::string> autoware_experimental_warnings;
};

[[nodiscard]] NavigationTargetReadiness evaluateNavigationTargetReadiness(
  const MappingDataset & dataset,
  const PipelineResult & pipeline,
  const RouteValidationResult & route_validation,
  const ApplicationConfig & config,
  const RouteValidationResult * closed_course_route_validation = nullptr,
  const RouteValidationResult * autoware_closed_course_route_validation = nullptr);

void saveNavigationTargetReadinessYaml(
  const std::filesystem::path & path,
  const NavigationTargetReadiness & readiness,
  const MappingDataset & dataset,
  const PipelineResult & pipeline,
  const RouteValidationResult & route_validation,
  const ApplicationConfig & config,
  const RouteValidationResult * closed_course_route_validation = nullptr,
  const RouteValidationResult * autoware_closed_course_route_validation = nullptr,
  const std::string & autoware_centerline_source = "recorded_trajectory");

}  // namespace lidar_mobility_map_generator
