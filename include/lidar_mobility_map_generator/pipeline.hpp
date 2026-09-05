#pragma once

#include "lidar_mobility_map_generator/occupancy_grid.hpp"
#include "lidar_mobility_map_generator/types.hpp"

namespace lidar_mobility_map_generator
{

struct PipelineResult
{
  GenerationResult generation;
  TraversabilityGridResult grids;
};

[[nodiscard]] PipelineResult runVectorMapPipeline(
  const MappingDataset & dataset,
  const GeneratorConfig & config);

}  // namespace lidar_mobility_map_generator
