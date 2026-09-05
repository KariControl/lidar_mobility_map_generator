#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/occupancy_grid.hpp"
#include "lidar_mobility_map_generator/route_editor.hpp"
#include "lidar_mobility_map_generator/route_graph.hpp"
#include "lidar_mobility_map_generator/vector_map_source.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

lmmg::RouteGraph graph(const double y)
{
  lmmg::RouteGraph result;
  result.frame_id = "map";
  result.nodes = {{1U, {0.0, y, 0.0}}, {2U, {2.0, y, 0.0}}};
  lmmg::RouteEdge edge;
  edge.id = 3U;
  edge.from = 1U;
  edge.to = 2U;
  edge.centerline = {result.nodes[0].position, result.nodes[1].position};
  edge.passable = true;
  result.edges.push_back(edge);
  return result;
}

lmmg::RouteGraph curvedGraph()
{
  lmmg::RouteGraph result = graph(1.0);
  result.edges.front().centerline = {
    result.nodes.front().position,
    {0.7, 1.05, 0.0},
    {1.3, 1.05, 0.0},
    result.nodes.back().position};
  result.edges.front().passable = false;
  return result;
}

}  // namespace

int main()
{
  try {
    const lmmg::RouteGraph recorded = graph(0.0);
    const lmmg::RouteGraph edited = curvedGraph();
    lmmg::VectorMapSourceSelection selection;
    selection.source = lmmg::VectorMapCenterlineSource::kEditedTopology;
    selection.frame_id = edited.frame_id;
    selection.graph_fingerprint = lmmg::routeGraphFingerprint(edited);

    const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "lmmg_vector_map_source_test.tsv";
    lmmg::saveVectorMapSourceSelection(path, selection);
    const lmmg::VectorMapSourceSelection loaded =
      lmmg::loadVectorMapSourceSelection(path);
    std::filesystem::remove(path);
    lmmg::validateVectorMapSourceSelection(loaded, recorded, edited);
    if (loaded.source != lmmg::VectorMapCenterlineSource::kEditedTopology ||
      loaded.graph_fingerprint != selection.graph_fingerprint)
    {
      throw std::runtime_error("Vector Map source selection did not round-trip");
    }

    bool stale_rejected = false;
    try {
      lmmg::validateVectorMapSourceSelection(loaded, recorded, graph(2.0));
    } catch (const std::exception &) {
      stale_rejected = true;
    }
    if (!stale_rejected) {
      throw std::runtime_error("stale edited-topology selection was accepted");
    }

    // Mirror the GUI -> Generator contract. The GUI fingerprints the sparse
    // raw curve. Clearance validation densifies it for bounded collision
    // sampling, which must not make the operator's unchanged selection stale.
    lmmg::RouteGraph validated_edited = edited;
    lmmg::OccupancyGrid2D obstacles(-2.0, -2.0, 0.10, 61U, 61U);
    lmmg::OccupancyGrid2D unknown(-2.0, -2.0, 0.10, 61U, 61U);
    lmmg::TraversabilityConfig traversability;
    traversability.ray_step = 0.05;
    traversability.maximum_corridor_half_width = 0.25;
    traversability.minimum_safe_center_width = 0.20;
    traversability.unknown_space_policy = "occupied";
    lmmg::computeRouteClearance(
      validated_edited, obstacles, unknown, traversability, 0.5);
    if (lmmg::routeGraphFingerprint(validated_edited) == selection.graph_fingerprint ||
      validated_edited.edges.front().centerline.size() <=
      edited.edges.front().centerline.size())
    {
      throw std::runtime_error("test curve was not densified into a distinct validated graph");
    }
    const lmmg::RouteGraph & selected_validated =
      lmmg::validateAndSelectVectorMapSourceGraph(
      loaded, recorded, edited, validated_edited);
    if (&selected_validated != &validated_edited) {
      throw std::runtime_error("edited source did not select the safety-validated graph");
    }

    lmmg::RouteGraph substituted_validated = validated_edited;
    substituted_validated.edges.front().centerline[1U].y += 0.01;
    bool substituted_validated_rejected = false;
    try {
      static_cast<void>(lmmg::validateAndSelectVectorMapSourceGraph(
          loaded, recorded, edited, substituted_validated));
    } catch (const std::exception &) {
      substituted_validated_rejected = true;
    }
    if (!substituted_validated_rejected) {
      throw std::runtime_error(
              "validated graph geometry unrelated to the operator-bound raw graph was accepted");
    }

    substituted_validated = validated_edited;
    substituted_validated.edges.front().to = 999U;
    bool substituted_topology_rejected = false;
    try {
      static_cast<void>(lmmg::validateAndSelectVectorMapSourceGraph(
          loaded, recorded, edited, substituted_validated));
    } catch (const std::exception &) {
      substituted_topology_rejected = true;
    }
    if (!substituted_topology_rejected) {
      throw std::runtime_error(
              "validated graph topology unrelated to the operator-bound raw graph was accepted");
    }

    lmmg::NamedNavigationRoute route =
      lmmg::makeCompleteOpenChainNavigationRoute(
      edited, 7U, "edited topology", lmmg::NavigationAuthoringTarget::kAutoware,
      false);
    lmmg::NavigationAuthoring authoring;
    authoring.frame_id = edited.frame_id;
    authoring.graph_fingerprint = selection.graph_fingerprint;
    authoring.routes.push_back(std::move(route));
    lmmg::NavigationAuthoringValidationResult authoring_validation =
      lmmg::validateNavigationAuthoring(authoring, edited);
    lmmg::applyOperationalGraphSafetyValidation(
      authoring_validation, selected_validated);
    if (!authoring_validation.selected_autoware_route_id) {
      std::string detail;
      for (const std::string & error : selected_validated.edges.front().validation_errors) {
        detail += ":" + error;
      }
      throw std::runtime_error(
              "raw-authored Mission was not promoted through the densified safety graph" +
              detail);
    }

    lmmg::RouteGraph changed_raw = edited;
    changed_raw.edges.front().centerline[1U].y += 0.01;
    bool post_selection_edit_rejected = false;
    try {
      static_cast<void>(lmmg::validateAndSelectVectorMapSourceGraph(
          loaded, recorded, changed_raw, validated_edited));
    } catch (const std::exception &) {
      post_selection_edit_rejected = true;
    }
    if (!post_selection_edit_rejected) {
      throw std::runtime_error("raw edit after source selection was not rejected as stale");
    }
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
