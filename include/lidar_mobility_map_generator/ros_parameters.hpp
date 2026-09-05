#pragma once

#include "lidar_mobility_map_generator/config.hpp"

#include <rclcpp/node.hpp>

namespace lidar_mobility_map_generator
{

[[nodiscard]] ApplicationConfig loadApplicationConfig(rclcpp::Node & node);

}  // namespace lidar_mobility_map_generator
