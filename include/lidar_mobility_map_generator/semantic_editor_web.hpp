#pragma once

namespace lidar_mobility_map_generator
{

// Embedded HTML/JavaScript served by semantic_map_editor. The definition
// lives in one translation unit so users of this public header do not parse
// the full web application.
extern const char kSemanticEditorHtml[];

}  // namespace lidar_mobility_map_generator
