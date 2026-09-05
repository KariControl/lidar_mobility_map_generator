#include "lidar_mobility_map_generator/body_passage_planning.hpp"
#include "lidar_mobility_map_generator/exporters.hpp"
#include "lidar_mobility_map_generator/glim_reader.hpp"
#include "lidar_mobility_map_generator/nav2_experimental.hpp"
#include "lidar_mobility_map_generator/navigation_authoring.hpp"
#include "lidar_mobility_map_generator/navigation_outputs.hpp"
#include "lidar_mobility_map_generator/navigation_promotion.hpp"
#include "lidar_mobility_map_generator/nav2_route_export.hpp"
#include "lidar_mobility_map_generator/observed_route_graph.hpp"
#include "lidar_mobility_map_generator/pipeline.hpp"
#include "lidar_mobility_map_generator/pointcloud_io.hpp"
#include "lidar_mobility_map_generator/ros_parameters.hpp"
#include "lidar_mobility_map_generator/rosbag_reader.hpp"
#include "lidar_mobility_map_generator/review_io.hpp"
#include "lidar_mobility_map_generator/route_editor.hpp"
#include "lidar_mobility_map_generator/semantic_map.hpp"
#include "lidar_mobility_map_generator/semantic_route_graph.hpp"
#include "lidar_mobility_map_generator/vector_map_source.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace lmmg = lidar_mobility_map_generator;

namespace
{

std::string commaSeparatedIds(const std::vector<std::uint64_t> & ids)
{
  std::string result;
  for (const std::uint64_t id : ids) {
    if (!result.empty()) {
      result += ',';
    }
    result += std::to_string(id);
  }
  return result;
}

const char * publicMapType(const lmmg::OutputConfig & output)
{
  if (output.target_mode == "autoware") {
    return "vector_map";
  }
  if (output.target_mode == "nav2") {
    return "navigation_map";
  }
  return "both";
}

void markGenerationInProgress(
  const std::filesystem::path & output, const lmmg::ApplicationConfig & config)
{
  const std::filesystem::path readiness = output / "navigation_target_readiness.yaml";
  const std::filesystem::path temporary = readiness.string() + ".lmmg.tmp";
  std::ofstream stream(temporary, std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("failed to invalidate navigation readiness: " + temporary.string());
  }
  stream << "schema_version: 3\n"
         << "generation_complete: false\n"
         << "requested_target_mode: \"" << config.output.target_mode << "\"\n"
         << "canonical_outputs_fail_closed: true\n"
         << "nav2:\n"
         << "  enabled: " << (config.output.nav2Enabled() ? "true" : "false") << '\n'
         << "  navigation_ready: false\n"
         << "  closed_course_experimental_ready: false\n"
         << "  reasons: [\"generation_in_progress\"]\n"
         << "autoware:\n"
         << "  enabled: " << (config.output.autowareEnabled() ? "true" : "false") << '\n'
         << "  navigation_ready: false\n"
         << "  closed_course_experimental_ready: false\n"
         << "  reasons: [\"generation_in_progress\"]\n";
  stream.close();
  if (!stream) {
    throw std::runtime_error("failed to finish readiness invalidation: " + temporary.string());
  }
  std::error_code rename_error;
  std::filesystem::rename(temporary, readiness, rename_error);
  if (rename_error) {
    std::error_code cleanup_error;
    std::filesystem::remove(temporary, cleanup_error);
    throw std::runtime_error(
            "failed to atomically invalidate navigation readiness: " +
            rename_error.message());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("lidar_mobility_map_generator");
  try {
    const lmmg::ApplicationConfig config = lmmg::loadApplicationConfig(*node);
    const std::filesystem::path output = config.output.directory;
    const std::filesystem::path existing_autoware_stage =
      output / "autoware_closed_course_experimental_map";
    if (!config.output.autowareEnabled() && std::filesystem::exists(existing_autoware_stage)) {
      throw std::runtime_error(
              "the selected map type excludes Vector Map, but the existing staged runtime "
              "directory would become stale; use a new output.directory or explicitly "
              "archive/remove autoware_closed_course_experimental_map first");
    }
    RCLCPP_INFO(
      node->get_logger(), "Reading input type: %s; output map type: %s",
      config.input_type.c_str(), publicMapType(config.output));
    // Invalidate any prior successful readiness before input I/O or the
    // pipeline can fail.  Consumers and the staging helper must never combine
    // an old ready=true marker with partially regenerated target artifacts.
    std::filesystem::create_directories(output);
    markGenerationInProgress(output, config);

    lmmg::MappingDataset dataset;
    if (config.input_type == "rosbag2") {
      dataset = lmmg::readRosbagDataset(
        config.rosbag2, config.extrinsics, config.generator.map_builder);
    } else {
      dataset = lmmg::readGlimDataset(
        config.glim, config.extrinsics, config.generator.map_builder);
    }
    if (dataset.world_frame != config.output.frame_id) {
      dataset.warnings.push_back(
        "input world frame '" + dataset.world_frame + "' was relabelled to navigation output "
        "frame '" + config.output.frame_id +
        "'; coordinates are unchanged and runtime localization must provide the matching datum");
      dataset.world_frame = config.output.frame_id;
    }
    if (!config.extrinsics.verified) {
      dataset.warnings.push_back(
        "LiDAR-to-body extrinsics are unverified; base-relative footprint, clearance, "
        "and route geometry are diagnostic only");
    }
    RCLCPP_INFO(
      node->get_logger(), "Input loaded: %zu map points, %zu trajectory poses",
      dataset.map_points.size(), dataset.trajectory.size());

    const lmmg::PipelineResult result = lmmg::runVectorMapPipeline(dataset, config.generator);
    lmmg::RouteGraph empty_navigation_graph;
    empty_navigation_graph.frame_id = dataset.world_frame;

    if (config.output.save_pointcloud_map) {
      lmmg::saveBinaryPcd(output / "pointcloud_map.pcd", dataset.map_points);
    }
    lmmg::saveTrajectoryTum(output / "trajectory_raw.tum", dataset.trajectory);
    lmmg::saveTrajectoryTum(
      output / "trajectory_processed.tum", result.generation.processed_trajectory);
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_generated.geojson", result.generation.graph);
    lmmg::saveCorridorsGeoJson(
      output / "drivable_corridors_generated.geojson", result.generation.graph);
    lmmg::saveRouteGraphMetadataYaml(
      output / "route_graph_generated_metadata.yaml", result.generation.graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_generated.tsv", result.generation.graph);
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_observed_driven.geojson",
      result.generation.observed_route_graph);
    lmmg::saveRouteGraphMetadataYaml(
      output / "route_graph_observed_driven_metadata.yaml",
      result.generation.observed_route_graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_observed_driven.tsv",
      result.generation.observed_route_graph);

    // Human route edits are an overlay; the generated graph above remains an
    // immutable audit artifact. Replaying an existing overlay is fingerprint-
    // guarded, then every edited edge is revalidated against this run's grids
    // and platform constraints before entering the operational graph.
    const std::filesystem::path route_edits_path = output / "route_edits.tsv";
    std::optional<lmmg::RouteEditSession> route_edit_session;
    if (std::filesystem::exists(route_edits_path)) {
      try {
        route_edit_session.emplace(
          result.generation.graph, lmmg::loadRouteEditOverlayTsv(route_edits_path));
      } catch (const std::exception & exception) {
        std::filesystem::path backup = output / "route_edits.incompatible.tsv";
        for (std::size_t suffix = 1U; std::filesystem::exists(backup); ++suffix) {
          backup = output / ("route_edits.incompatible." + std::to_string(suffix) + ".tsv");
        }
        std::error_code rename_error;
        std::filesystem::rename(route_edits_path, backup, rename_error);
        if (rename_error) {
          throw std::runtime_error(
                  "route edit overlay is incompatible and could not be backed up: " +
                  std::string(exception.what()) + " (backup error: " +
                  rename_error.message() + ")");
        }
        RCLCPP_WARN(
          node->get_logger(),
          "Route edit overlay is incompatible with the regenerated graph. It was moved to %s: %s",
          backup.string().c_str(), exception.what());
      }
    }
    if (!route_edit_session) {
      route_edit_session.emplace(result.generation.graph);
    }
    lmmg::saveRouteEditOverlayTsv(route_edits_path, route_edit_session->overlay());
    lmmg::saveRouteEditOverlayGeoJson(
      output / "route_edits.geojson", route_edit_session->overlay());
    // The GUI binds vector_map_source.tsv to the exact raw edited graph that
    // the operator saw. Validation below intentionally densifies centerlines
    // for bounded collision/clearance sampling, so keep the raw graph as the
    // immutable handoff identity and use the validated copy only downstream.
    const lmmg::EditedRouteGraph raw_edited_route = route_edit_session->editedGraph();
    lmmg::RouteValidationOptions production_validation_options;
    production_validation_options.use_orientation_aware_obstacle_footprint = true;
    const lmmg::RouteValidationResult route_validation = lmmg::validateEditedRouteGraph(
      raw_edited_route, result.grids.inflated_grid, result.grids.unknown_grid,
      config.generator, production_validation_options, &result.grids.obstacle_grid);
    lmmg::RouteValidationOptions closed_course_validation_options;
    closed_course_validation_options.require_verified_vehicle_dimensions = false;
    closed_course_validation_options.use_orientation_aware_obstacle_footprint = true;
    closed_course_validation_options.use_orientation_aware_unknown_footprint = true;
    closed_course_validation_options.include_clearance_in_unknown_footprint = false;
    const lmmg::RouteValidationResult closed_course_route_validation =
      lmmg::validateEditedRouteGraph(
      raw_edited_route, result.grids.inflated_grid,
      result.grids.unknown_grid, config.generator, closed_course_validation_options,
      &result.grids.obstacle_grid);
    lmmg::VectorMapSourceSelection vector_map_source;
    vector_map_source.frame_id = result.generation.observed_route_graph.frame_id;
    vector_map_source.graph_fingerprint = lmmg::routeGraphFingerprint(
      result.generation.observed_route_graph);
    const std::filesystem::path vector_map_source_path =
      output / "vector_map_source.tsv";
    const lmmg::RouteGraph * validated_vector_map_source_graph =
      &result.generation.observed_route_graph;
    if (config.output.autowareEnabled() && std::filesystem::exists(vector_map_source_path)) {
      vector_map_source = lmmg::loadVectorMapSourceSelection(vector_map_source_path);
      validated_vector_map_source_graph =
        &lmmg::validateAndSelectVectorMapSourceGraph(
        vector_map_source, result.generation.observed_route_graph,
        raw_edited_route.graph, closed_course_route_validation.operational_graph);
    }
    const bool edited_topology_vector_map = config.output.autowareEnabled() &&
      vector_map_source.source == lmmg::VectorMapCenterlineSource::kEditedTopology;
    RCLCPP_INFO(
      node->get_logger(), "Vector Map centerline source: %s",
      lmmg::toString(vector_map_source.source));
    lmmg::saveRouteValidationReportYaml(
      output / "route_validation_closed_course_report.yaml",
      closed_course_route_validation, config.generator);
    lmmg::saveEditedRouteGraphGeoJson(
      output / "route_graph_edited.geojson", route_validation.edited);
    lmmg::saveRouteGraphMetadataYaml(
      output / "route_graph_edited_metadata.yaml", route_validation.edited.graph);
    lmmg::saveCorridorsGeoJson(
      output / "drivable_corridors_edited.geojson", route_validation.edited.graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_edited.tsv", route_validation.edited.graph);
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_validated.geojson", route_validation.operational_graph);
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph.geojson", route_validation.operational_graph);
    lmmg::saveRouteGraphMetadataYaml(
      output / "route_graph_validated_metadata.yaml", route_validation.operational_graph);
    lmmg::saveRouteGraphMetadataYaml(
      output / "route_graph_metadata.yaml", route_validation.operational_graph);
    lmmg::saveCorridorsGeoJson(
      output / "drivable_corridors_validated.geojson", route_validation.operational_graph);
    lmmg::saveCorridorsGeoJson(
      output / "drivable_corridors.geojson", route_validation.operational_graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_validated.tsv", route_validation.operational_graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry.tsv", route_validation.operational_graph);
    lmmg::saveRouteValidationReportYaml(
      output / "route_validation_report.yaml", route_validation, config.generator);
    if (!route_validation.operational_ready) {
      RCLCPP_WARN(
        node->get_logger(),
        "Route edit validation produced no operational edge; route_graph_validated.geojson is empty");
    }

    // Semantics are authored against the complete edited graph so invalid
    // routes and their annotations remain reviewable. Planner-facing semantic
    // derivatives are filtered onto the validated operational graph below.
    // If regeneration changes Route Edge IDs, preserve an incompatible layer
    // as a backup rather than silently applying it to another route.
    const std::filesystem::path semantic_path = output / "semantic_features.tsv";
    lmmg::SemanticMap semantic_map;
    lmmg::SemanticMap authored_semantic_map;
    bool semantics_use_lossless_replay_coordinates = false;
    bool semantic_layer_ready = true;
    if (std::filesystem::exists(semantic_path)) {
      try {
        authored_semantic_map = lmmg::loadSemanticMapTsv(semantic_path);
        semantics_use_lossless_replay_coordinates =
          config.output.autowareEnabled() &&
          lmmg::semanticRouteSpansUseGraphCoordinates(
          authored_semantic_map, result.generation.observed_route_graph);
        semantic_map = lmmg::remapSemanticMapToGraph(
          authored_semantic_map, route_validation.edited.graph);
      } catch (const std::exception & exception) {
        std::filesystem::path backup = output / "semantic_features.incompatible.tsv";
        for (std::size_t suffix = 1U; std::filesystem::exists(backup); ++suffix) {
          backup = output /
            ("semantic_features.incompatible." + std::to_string(suffix) + ".tsv");
        }
        std::error_code rename_error;
        std::filesystem::rename(semantic_path, backup, rename_error);
        if (rename_error) {
          semantic_layer_ready = false;
          RCLCPP_WARN(
            node->get_logger(),
            "Semantic layer is incompatible with the regenerated graph and could not be "
            "backed up; leaving it untouched: %s (backup error: %s)",
            exception.what(), rename_error.message().c_str());
        } else {
          semantic_map.frame_id = route_validation.edited.graph.frame_id;
          authored_semantic_map = semantic_map;
          semantics_use_lossless_replay_coordinates = false;
          lmmg::saveSemanticMapTsv(semantic_path, semantic_map);
          RCLCPP_WARN(
            node->get_logger(),
            "Semantic layer is incompatible with the regenerated graph. The old layer was "
            "moved to %s and a new empty active layer was created: %s",
            backup.string().c_str(), exception.what());
        }
      }
    } else {
      semantic_map.frame_id = route_validation.edited.graph.frame_id;
      authored_semantic_map = semantic_map;
      lmmg::saveSemanticMapTsv(semantic_path, semantic_map);
    }
    if (semantic_layer_ready) {
      try {
        lmmg::saveSemanticMapGeoJson(
          output / "semantic_features.geojson", semantic_map, route_validation.edited.graph);
        lmmg::saveSemanticRouteRulesYaml(
          output / "semantic_route_rules_preview.yaml", semantic_map,
          route_validation.edited.graph);
        lmmg::saveSemanticRouteGraphGeoJson(
          output / "route_graph_semantic_preview.geojson", semantic_map,
          route_validation.edited.graph);
        const lmmg::SemanticGraphFilterResult operational_semantics =
          lmmg::filterSemanticMapForGraph(semantic_map, route_validation.operational_graph);
        lmmg::saveSemanticRouteRulesYaml(
          output / "semantic_route_rules.yaml", operational_semantics.map,
          route_validation.operational_graph);
        lmmg::saveSemanticRouteGraphGeoJson(
          output / "route_graph_semantic.geojson", operational_semantics.map,
          route_validation.operational_graph);
        for (const std::string & diagnostic : operational_semantics.diagnostics) {
          RCLCPP_WARN(node->get_logger(), "Semantic operational filter: %s", diagnostic.c_str());
        }
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          node->get_logger(), "Failed to refresh semantic derived outputs: %s",
          exception.what());
      }
    }

    // Keep the editable, de-duplicated physical topology as a separate
    // candidate. It is useful for arbitrary user-authored routing, but a loop
    // or revisit can legitimately branch and therefore does not encode the
    // temporal order in which the mapping vehicle drove the course.
    const lmmg::ObservedDrivenCandidateResult topology_candidate =
      lmmg::materializeObservedDrivenCandidate(closed_course_route_validation);

    // Named Routes are authored against the complete edited graph.  The
    // fingerprint prevents stable-looking Edge IDs from being replayed onto a
    // different regeneration.  Promotion then has an independent second gate
    // against either the production operational graph or the explicitly
    // labelled closed-course observed-passage graph.
    const std::filesystem::path navigation_authoring_path =
      output / "navigation_authoring.json";
    const bool navigation_authoring_present =
      std::filesystem::exists(navigation_authoring_path);
    bool navigation_authoring_load_failed = false;
    lmmg::NavigationAuthoring navigation_authoring;
    navigation_authoring.frame_id = raw_edited_route.graph.frame_id;
    navigation_authoring.graph_fingerprint =
      lmmg::routeGraphFingerprint(raw_edited_route.graph);
    if (navigation_authoring_present) {
      try {
        navigation_authoring =
          lmmg::loadNavigationAuthoringJson(navigation_authoring_path);
      } catch (const std::exception & exception) {
        navigation_authoring_load_failed = true;
        RCLCPP_ERROR(
          node->get_logger(),
          "navigation_authoring.json is malformed; all authored promotion is blocked: %s",
          exception.what());
      }
    }
    lmmg::NavigationAuthoringValidationResult edited_authoring_validation =
      lmmg::validateNavigationAuthoring(
      navigation_authoring, raw_edited_route.graph);
    if (navigation_authoring_load_failed) {
      edited_authoring_validation.errors.push_back("navigation_authoring_load_failed");
      edited_authoring_validation.selected_autoware_route_id.reset();
      edited_authoring_validation.selected_nav2_route_id.reset();
    }
    lmmg::NavigationAuthoringValidationResult production_authoring_input =
      edited_authoring_validation;
    lmmg::applyOperationalGraphSafetyValidation(
      production_authoring_input, route_validation.operational_graph);
    lmmg::applyVirtualStopLineProductionPolicy(
      production_authoring_input, lmmg::NavigationAuthoringTarget::kAutoware);
    lmmg::applyVirtualStopLineProductionPolicy(
      production_authoring_input, lmmg::NavigationAuthoringTarget::kNav2);
    lmmg::NavigationAuthoringValidationResult closed_course_authoring_input =
      edited_authoring_validation;
    lmmg::applyOperationalGraphSafetyValidation(
      closed_course_authoring_input, topology_candidate.graph);
    const bool production_autoware_authoring_requested =
      navigation_authoring_load_failed ||
      (navigation_authoring_present && lmmg::hasPromotionRequest(
        navigation_authoring, lmmg::NavigationAuthoringTarget::kAutoware));
    const bool nav2_authoring_requested = navigation_authoring_load_failed ||
      (navigation_authoring_present && lmmg::hasPromotionRequest(
        navigation_authoring, lmmg::NavigationAuthoringTarget::kNav2));

    lmmg::SemanticRouteGraphResult closed_course_topology_route;
    closed_course_topology_route.graph.frame_id = topology_candidate.graph.frame_id;
    if (semantic_layer_ready) {
      const lmmg::SemanticGraphFilterResult candidate_semantics =
        lmmg::filterSemanticMapForGraph(semantic_map, topology_candidate.graph);
      closed_course_topology_route = lmmg::materializeSemanticRouteGraph(
        topology_candidate.graph, candidate_semantics.map);
      for (const std::string & diagnostic : candidate_semantics.diagnostics) {
        RCLCPP_WARN(node->get_logger(), "Semantic topology filter: %s", diagnostic.c_str());
      }
    }

    lmmg::NavigationPromotionResult navigation_promotion =
      lmmg::materializeNavigationPromotions(
      production_authoring_input, closed_course_authoring_input,
      route_validation.operational_graph, topology_candidate.graph,
      empty_navigation_graph, semantic_layer_ready ? &semantic_map : nullptr,
      nav2_authoring_requested, production_autoware_authoring_requested,
      semantic_layer_ready ? &closed_course_topology_route.graph : nullptr);

    // User-authored Lanelet centerlines have an explicit source selection and
    // independent semantic/Mission files. They must never be confused with
    // the immutable recorded-trajectory replay or with Nav2 authoring.
    // Unlike the recorded-trajectory candidate, this source has no
    // observed-passage exception: only edges that passed the configured
    // vehicle/obstacle/unknown-space checks may become drivable Lanelets.
    const lmmg::RouteGraph & edited_vector_validated_source_graph =
      *validated_vector_map_source_graph;
    lmmg::SemanticMap edited_vector_semantics;
    edited_vector_semantics.frame_id = raw_edited_route.graph.frame_id;
    lmmg::SemanticRouteGraphResult edited_vector_semantic_route;
    edited_vector_semantic_route.graph.frame_id =
      edited_vector_validated_source_graph.frame_id;
    lmmg::NavigationAuthoring edited_vector_authoring;
    edited_vector_authoring.frame_id = raw_edited_route.graph.frame_id;
    edited_vector_authoring.graph_fingerprint =
      lmmg::routeGraphFingerprint(raw_edited_route.graph);
    lmmg::NavigationAuthoringValidationResult edited_vector_authoring_validation;
    lmmg::NavigationPromotionResult edited_vector_promotion;
    edited_vector_promotion.closed_course_autoware_graph.frame_id = dataset.world_frame;
    bool edited_vector_authoring_present = false;
    bool edited_vector_authoring_requested = false;
    if (edited_topology_vector_map) {
      const std::filesystem::path edited_vector_semantic_path =
        output / "semantic_features_autoware_topology.tsv";
      if (std::filesystem::exists(edited_vector_semantic_path)) {
        edited_vector_semantics = lmmg::loadSemanticMapTsv(
          edited_vector_semantic_path, &route_validation.edited.graph);
      }
      const lmmg::SemanticGraphFilterResult filtered =
        lmmg::filterSemanticMapForGraph(
        edited_vector_semantics, edited_vector_validated_source_graph);
      edited_vector_semantic_route = lmmg::materializeSemanticRouteGraph(
        edited_vector_validated_source_graph, filtered.map);
      for (const std::string & diagnostic : filtered.diagnostics) {
        RCLCPP_WARN(
          node->get_logger(), "Edited Lanelet semantic filter: %s", diagnostic.c_str());
      }
      lmmg::saveSemanticMapGeoJson(
        output / "semantic_features_autoware_topology.geojson",
        edited_vector_semantics, route_validation.edited.graph);
      lmmg::saveSemanticMapTsv(
        edited_vector_semantic_path, edited_vector_semantics);
      lmmg::saveRouteGraphGeoJson(
        output / "route_graph_autoware_topology_source.geojson",
        edited_vector_validated_source_graph);
      lmmg::saveRouteGraphGeoJson(
        output / "route_graph_autoware_topology_semantic_candidate.geojson",
        edited_vector_semantic_route.graph);
      lmmg::saveReviewGeometryTsv(
        output / "review_geometry_autoware_topology_semantic_candidate.tsv",
        edited_vector_semantic_route.graph);

      const std::filesystem::path edited_vector_authoring_path =
        output / "navigation_authoring_autoware_topology.json";
      edited_vector_authoring_present =
        std::filesystem::exists(edited_vector_authoring_path);
      if (edited_vector_authoring_present) {
        edited_vector_authoring = lmmg::loadNavigationAuthoringJson(
          edited_vector_authoring_path);
      }
      edited_vector_authoring_validation = lmmg::validateNavigationAuthoring(
        edited_vector_authoring, raw_edited_route.graph);
      for (const lmmg::NamedNavigationRoute & route : edited_vector_authoring.routes) {
        if (route.target != lmmg::NavigationAuthoringTarget::kAutoware) {
          edited_vector_authoring_validation.errors.push_back(
            "edited Vector Map Route target must be autoware");
          edited_vector_authoring_validation.selected_autoware_route_id.reset();
        }
      }
      for (const lmmg::AuthoredStopLine & line : edited_vector_authoring.stop_lines) {
        if (line.target != lmmg::NavigationAuthoringTarget::kAutoware) {
          edited_vector_authoring_validation.errors.push_back(
            "edited Vector Map stop-line target must be autoware");
          edited_vector_authoring_validation.selected_autoware_route_id.reset();
        }
      }
      lmmg::applyOperationalGraphSafetyValidation(
        edited_vector_authoring_validation, edited_vector_validated_source_graph);
      edited_vector_authoring_requested = edited_vector_authoring_present &&
        lmmg::hasPromotionRequest(
        edited_vector_authoring, lmmg::NavigationAuthoringTarget::kAutoware);
      lmmg::NavigationAuthoringValidationResult no_production =
        edited_vector_authoring_validation;
      no_production.selected_autoware_route_id.reset();
      edited_vector_promotion = lmmg::materializeNavigationPromotions(
        no_production, edited_vector_authoring_validation,
        empty_navigation_graph, edited_vector_validated_source_graph,
        empty_navigation_graph, &edited_vector_semantics,
        false, edited_vector_authoring_requested,
        &edited_vector_semantic_route.graph);
      edited_vector_authoring_validation =
        edited_vector_promotion.closed_course_validation;
    }

    lmmg::NavigationAuthoringValidationResult & production_authoring_validation =
      navigation_promotion.production_validation;
    lmmg::NavigationAuthoringValidationResult & closed_course_authoring_validation =
      navigation_promotion.closed_course_validation;
    lmmg::RouteGraph & production_nav2_graph =
      navigation_promotion.production_nav2_graph;
    lmmg::RouteGraph & production_autoware_graph =
      navigation_promotion.production_autoware_graph;
    lmmg::RouteGraph & authored_closed_nav2_graph =
      navigation_promotion.closed_course_nav2_graph;

    const lmmg::NamedNavigationRoute * production_nav2_named_route =
      navigation_promotion.production_nav2_route ?
      &*navigation_promotion.production_nav2_route : nullptr;
    const lmmg::NamedNavigationRoute * production_autoware_named_route =
      navigation_promotion.production_autoware_route ?
      &*navigation_promotion.production_autoware_route : nullptr;
    const lmmg::NamedNavigationRoute * closed_nav2_named_route =
      navigation_promotion.closed_course_nav2_route ?
      &*navigation_promotion.closed_course_nav2_route : nullptr;
    const lmmg::NamedNavigationRoute * production_nav2_export_route =
      production_nav2_named_route;
    const lmmg::NamedNavigationRoute * production_autoware_export_route =
      production_autoware_named_route;
    const lmmg::NamedNavigationRoute * closed_nav2_export_route =
      closed_nav2_named_route;

    const std::vector<lmmg::AuthoredStopLine> & production_autoware_stops =
      navigation_promotion.production_autoware_stop_lines;
    const std::vector<lmmg::AuthoredStopLine> & closed_nav2_stops =
      navigation_promotion.closed_course_nav2_stop_lines;
    const lmmg::NamedNavigationRoute * edited_vector_named_route =
      edited_vector_promotion.closed_course_autoware_route ?
      &*edited_vector_promotion.closed_course_autoware_route : nullptr;
    const std::vector<lmmg::AuthoredStopLine> & edited_vector_stops =
      edited_vector_promotion.closed_course_autoware_stop_lines;
    const lmmg::NamedNavigationRoute * edited_vector_source_named_route =
      lmmg::selectedNamedNavigationRoute(
      edited_vector_authoring_validation,
      lmmg::NavigationAuthoringTarget::kAutoware);
    const bool edited_vector_route_promotable = edited_topology_vector_map &&
      edited_vector_authoring_requested && edited_vector_named_route != nullptr &&
      !edited_vector_promotion.closed_course_autoware_graph.edges.empty();
    // A named Route is a mission selection inside the map; it must not trim
    // unrelated validated Lanelets from the map itself.  Keep the complete
    // edited topology as the Lanelet2 source and attach the selected mission
    // separately through Lanelet2AuthoringOptions below.
    const lmmg::RouteGraph & edited_vector_export_graph =
      edited_vector_semantic_route.graph;

    // Planner maps and mission selections are separate artifacts. Both
    // Autoware and Nav2 graphs above intentionally retain every eligible map
    // Edge; keep exact ordered-chain sidecars instead of making downstream
    // consumers infer a mission from the complete map.
    lmmg::RouteGraph production_nav2_selected_mission = empty_navigation_graph;
    if (production_nav2_export_route != nullptr && !production_nav2_graph.edges.empty()) {
      production_nav2_selected_mission = lmmg::selectNamedNavigationRouteGraph(
        production_nav2_graph, *production_nav2_export_route);
    }
    lmmg::RouteGraph closed_nav2_selected_mission = empty_navigation_graph;
    if (closed_nav2_export_route != nullptr && !authored_closed_nav2_graph.edges.empty()) {
      closed_nav2_selected_mission = lmmg::selectNamedNavigationRouteGraph(
        authored_closed_nav2_graph, *closed_nav2_export_route);
    }
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_nav2_production_selected_mission.geojson",
      production_nav2_selected_mission);
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_nav2_selected_mission.geojson",
      closed_nav2_selected_mission);

    lmmg::RouteGraph production_autoware_selected_mission = empty_navigation_graph;
    if (production_autoware_export_route != nullptr &&
      !production_autoware_graph.edges.empty())
    {
      production_autoware_selected_mission = lmmg::selectNamedNavigationRouteGraph(
        production_autoware_graph, *production_autoware_export_route);
    }
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_autoware_production_selected_mission.geojson",
      production_autoware_selected_mission);
    lmmg::RouteGraph edited_vector_selected_mission = empty_navigation_graph;
    if (edited_vector_source_named_route != nullptr &&
      !edited_vector_validated_source_graph.edges.empty())
    {
      edited_vector_selected_mission = lmmg::selectNamedNavigationRouteGraph(
        edited_vector_validated_source_graph, *edited_vector_source_named_route);
    }
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_autoware_topology_selected_mission.geojson",
      edited_vector_selected_mission);

    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_closed_course_topology_candidate.geojson",
      closed_course_topology_route.graph);
    lmmg::saveRouteGraphMetadataYaml(
      output / "route_graph_closed_course_topology_candidate_metadata.yaml",
      closed_course_topology_route.graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_closed_course_topology_candidate.tsv",
      closed_course_topology_route.graph);

    // The default supervised-replay candidate is the lossless chronological
    // graph. Unlike the physical topology graph it never merges a revisit,
    // crossing, or a loop's coincident first/last positions, so Autoware sees
    // one open directed chain covering the measured trajectory in order.
    // Semantics authored on the physical graph carry geometric anchors; remap
    // those anchors onto the replay chain and fail closed if the mapping is
    // ambiguous instead of silently losing a speed/no-entry restriction.
    lmmg::SemanticRouteGraphResult closed_course_semantic_route;
    // Nav2 does not consume this Lanelet-only representation. Keep its
    // default source as the raw observed replay so an Autoware-only semantic
    // failure or vehicle turning-radius requirement cannot destroy an
    // otherwise complete Nav2 bundle.
    closed_course_semantic_route.graph = result.generation.observed_route_graph;
    if (config.output.autowareEnabled()) {
      if (!semantic_layer_ready) {
        throw std::runtime_error(
                "semantic layer is unavailable; refusing to export a Vector Map "
                "that could silently lose GUI-authored semantics");
      }
      try {
        // The dedicated vector-map GUI authors bounded semantics directly on
        // the immutable lossless replay graph. Preserve those submitted arc
        // values byte-for-byte through Lanelet segmentation. Routing them via
        // the editable topology and projecting back moves boundaries by tiny
        // floating-point amounts, breaking the exact authored seam contract.
        lmmg::SemanticMap replay_semantics =
          semantics_use_lossless_replay_coordinates ? authored_semantic_map : semantic_map;
        for (lmmg::SemanticFeature & feature : replay_semantics.features) {
          if (feature.geometry != lmmg::SemanticGeometryType::kRouteEdges) {
            // Point/polygon semantics are geometric; a stale physical Edge ID
            // must not make validation against the temporal graph fail.
            feature.route_edge_ids.clear();
            continue;
          }
          if (!feature.route_edge_spans.empty()) {
            continue;
          }
          // Upgrade a legacy whole-edge rule to an anchored span before
          // remapping. The web editor already writes anchored v2 spans.
          for (const std::uint64_t edge_id : feature.route_edge_ids) {
            const lmmg::RouteEdge * source = nullptr;
            for (const lmmg::RouteEdge & edge : route_validation.edited.graph.edges) {
              if (edge.id == edge_id) {
                source = &edge;
                break;
              }
            }
            if (source == nullptr || source->centerline.size() < 2U) {
              throw std::runtime_error(
                      "semantic feature references a missing physical Route Edge " +
                      std::to_string(edge_id));
            }
            lmmg::RouteEdgeSpan span;
            span.edge_id = source->id;
            span.start_s = 0.0;
            span.end_s = lmmg::polylineLength(source->centerline);
            span.start_anchor = source->centerline.front();
            span.end_anchor = source->centerline.back();
            feature.route_edge_spans.push_back(std::move(span));
          }
        }
        if (semantics_use_lossless_replay_coordinates) {
          lmmg::validateSemanticMap(
            replay_semantics, &result.generation.observed_route_graph);
        } else {
          replay_semantics = lmmg::remapSemanticMapToGraph(
            replay_semantics, result.generation.observed_route_graph);
        }
        closed_course_semantic_route = lmmg::materializeSemanticRouteGraph(
          result.generation.observed_route_graph, replay_semantics);
        (void)lmmg::validateLosslessSemanticRouteGraph(
          result.generation.observed_route_graph, closed_course_semantic_route);
      } catch (const std::exception & exception) {
        throw std::runtime_error(
                std::string{
          "observed replay semantics are not a lossless ordered segmentation; "
          "Vector Map export is blocked: "} + exception.what());
      }
      // This is a representational Lanelet derivative only. Keep it separately
      // named: the canonical replay artifacts below must retain the raw observed
      // Edge IDs/order/geometry even when a speed span splits one raw Edge.
      lmmg::saveRouteGraphGeoJson(
        output / "route_graph_autoware_semantic_lanelet_candidate.geojson",
        closed_course_semantic_route.graph);
      lmmg::saveRouteGraphMetadataYaml(
        output / "route_graph_autoware_semantic_lanelet_candidate_metadata.yaml",
        closed_course_semantic_route.graph);
      lmmg::saveReviewGeometryTsv(
        output / "review_geometry_autoware_semantic_lanelet_candidate.tsv",
        closed_course_semantic_route.graph);
    }
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_closed_course_replay_candidate.geojson",
      result.generation.observed_route_graph);
    lmmg::saveRouteGraphMetadataYaml(
      output / "route_graph_closed_course_replay_candidate_metadata.yaml",
      result.generation.observed_route_graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_closed_course_replay_candidate.tsv",
      result.generation.observed_route_graph);

    // Keep the chronological replay above lossless for audit and Nav2.  A
    // short map-frame correction at the terminal stop may only be removed
    // from this separately named Autoware derivative after the data-set-
    // specific verification parameter and every fixed geometric gate agree.
    const lmmg::RouteGraph & effective_nav2_closed_course_graph =
      nav2_authoring_requested ? authored_closed_nav2_graph :
      result.generation.observed_route_graph;
    // Keep the recorded-trajectory candidate lossless and immutable. It is
    // the default Vector Map source. An explicitly selected edited topology
    // is handled later through separate semantics, authoring, validation, and
    // output sidecars, so it cannot silently alter this replay evidence.
    const lmmg::RouteGraph & effective_autoware_closed_course_source =
      result.generation.observed_route_graph;
    const lmmg::AutowareReplayCandidateResult autoware_replay_candidate =
      lmmg::materializeAutowareReplayCandidate(
      effective_autoware_closed_course_source,
      result.generation.processed_trajectory,
      false);
    lmmg::AutowareReplayCandidateResult exported_autoware_replay_candidate =
      autoware_replay_candidate;
    if (!config.output.autowareEnabled()) {
      exported_autoware_replay_candidate.graph = empty_navigation_graph;
    }
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_autoware_replay_candidate.geojson",
      exported_autoware_replay_candidate.graph);
    lmmg::saveAutowareReplayCandidateMetadataYaml(
      output / "route_graph_autoware_replay_candidate_metadata.yaml",
      exported_autoware_replay_candidate);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_autoware_replay_candidate.tsv",
      exported_autoware_replay_candidate.graph);
    const lmmg::RouteBodyPassagePlanningReport body_passage_planning_report =
      lmmg::evaluateRouteBodyPassagePlanning(
        exported_autoware_replay_candidate.graph,
        result.generation.processed_trajectory,
        result.grids.obstacle_grid,
      result.grids.unknown_grid,
      config.generator.robot,
      closed_course_route_validation);
    lmmg::saveRouteBodyPassagePlanningReportJson(
      output / "route_body_passage_planning_report.json",
      body_passage_planning_report);
    if (autoware_replay_candidate.terminal_tail_omitted) {
      RCLCPP_WARN(
        node->get_logger(),
        "Vector Map replay omitted %.3f m (%0.3f%%) of explicitly verified "
        "terminal localization settling; Navigation Map replay remains lossless",
        autoware_replay_candidate.omitted_length,
        100.0 * autoware_replay_candidate.omitted_length_ratio);
    } else if (autoware_replay_candidate.explicit_verification) {
      RCLCPP_WARN(
        node->get_logger(),
        "Terminal localization settling was explicitly verified, but the fixed "
        "Vector Map trim gates did not match (%s); retaining the full replay",
        autoware_replay_candidate.reason.c_str());
    }

    // Autoware authoring has its own lossless-replay document.  The editable
    // topology document remains the Nav2/legacy source because its Edge IDs
    // can legitimately differ after revisit de-duplication.  If the dedicated
    // document exists, it is authoritative: a malformed or stale document
    // must fail closed and must never fall back to the topology document.
    const std::filesystem::path autoware_replay_authoring_path =
      output / "navigation_authoring_autoware_replay.json";
    const bool autoware_replay_authoring_present =
      std::filesystem::exists(autoware_replay_authoring_path);
    bool autoware_replay_authoring_load_failed = false;
    bool autoware_replay_authoring_scope_failed = false;
    lmmg::NavigationAuthoring autoware_replay_authoring;
    autoware_replay_authoring.frame_id = autoware_replay_candidate.graph.frame_id;
    autoware_replay_authoring.graph_fingerprint =
      lmmg::routeGraphFingerprint(autoware_replay_candidate.graph);
    if (autoware_replay_authoring_present) {
      try {
        autoware_replay_authoring =
          lmmg::loadNavigationAuthoringJson(autoware_replay_authoring_path);
        for (const lmmg::NamedNavigationRoute & route :
          autoware_replay_authoring.routes)
        {
          if (route.target != lmmg::NavigationAuthoringTarget::kAutoware) {
            autoware_replay_authoring_scope_failed = true;
          }
        }
        for (const lmmg::AuthoredStopLine & stop :
          autoware_replay_authoring.stop_lines)
        {
          if (stop.target != lmmg::NavigationAuthoringTarget::kAutoware) {
            autoware_replay_authoring_scope_failed = true;
          }
        }
        if (autoware_replay_authoring_scope_failed) {
          RCLCPP_ERROR(
            node->get_logger(),
            "navigation_authoring_autoware_replay.json contains a non-Vector-Map "
            "target; cross-scope or 'both' promotion is blocked");
        }
      } catch (const std::exception & exception) {
        autoware_replay_authoring_load_failed = true;
        RCLCPP_ERROR(
          node->get_logger(),
          "navigation_authoring_autoware_replay.json is malformed; Vector Map "
          "route promotion is blocked without legacy fallback: %s",
          exception.what());
      }
    }
    const bool autoware_replay_authoring_requested =
      autoware_replay_authoring_load_failed ||
      autoware_replay_authoring_scope_failed ||
      (autoware_replay_authoring_present &&
      lmmg::hasPromotionRequest(
        autoware_replay_authoring, lmmg::NavigationAuthoringTarget::kAutoware));

    // Re-validate only the dedicated Autoware document against the exact
    // lossless replay graph. The editable-topology document is never copied
    // into this scope; when the dedicated document is absent the complete map
    // remains exportable but deliberately has no promoted Autoware Mission.
    lmmg::NavigationAuthoringValidationResult autoware_replay_authoring_validation =
      lmmg::validateNavigationAuthoring(
      autoware_replay_authoring, autoware_replay_candidate.graph);
    if (autoware_replay_authoring_load_failed) {
      autoware_replay_authoring_validation.errors.push_back(
        "navigation_authoring_autoware_replay_load_failed");
      autoware_replay_authoring_validation.selected_autoware_route_id.reset();
    }
    if (autoware_replay_authoring_scope_failed) {
      autoware_replay_authoring_validation.errors.push_back(
        "navigation_authoring_autoware_replay_scope_violation");
      autoware_replay_authoring_validation.selected_autoware_route_id.reset();
    }
    lmmg::applyOperationalGraphSafetyValidation(
      autoware_replay_authoring_validation, autoware_replay_candidate.graph);
    std::optional<lmmg::NamedNavigationRoute> exact_replay_autoware_route;
    std::vector<lmmg::AuthoredStopLine> exact_replay_autoware_stops;
    const lmmg::NamedNavigationRoute * replay_selected_route =
      lmmg::selectedNamedNavigationRoute(
      autoware_replay_authoring_validation,
      lmmg::NavigationAuthoringTarget::kAutoware);
    if (autoware_replay_authoring_requested && replay_selected_route != nullptr &&
      autoware_replay_authoring_validation.autoware_stop_lines_valid)
    {
      try {
        // Defensive chain selection proves that every ordered reference is an
        // Edge of the same full replay map without changing that map.
        (void)lmmg::selectNamedNavigationRouteGraph(
          autoware_replay_candidate.graph, *replay_selected_route);
        exact_replay_autoware_route = *replay_selected_route;
        exact_replay_autoware_stops = lmmg::resolveStopLinesForGraph(
          autoware_replay_authoring_validation,
          autoware_replay_candidate.graph,
          lmmg::NavigationAuthoringTarget::kAutoware,
          &*exact_replay_autoware_route);
        const std::size_t expected_stops = lmmg::expectedStopLineCountForRoute(
          autoware_replay_authoring_validation,
          lmmg::NavigationAuthoringTarget::kAutoware,
          *exact_replay_autoware_route);
        if (exact_replay_autoware_stops.size() != expected_stops) {
          throw std::runtime_error(
                  "lossless_replay_stop_line_resolution_incomplete");
        }
      } catch (const std::exception & exception) {
        exact_replay_autoware_route.reset();
        exact_replay_autoware_stops.clear();
        autoware_replay_authoring_validation.errors.push_back(
          std::string{"autoware_lossless_replay_promotion_failed:"} +
          exception.what());
        autoware_replay_authoring_validation.selected_autoware_route_id.reset();
      }
    }
    const lmmg::NamedNavigationRoute * exact_replay_autoware_export_route =
      exact_replay_autoware_route ? &*exact_replay_autoware_route : nullptr;
    lmmg::RouteGraph exact_replay_autoware_selected_mission = empty_navigation_graph;
    if (config.output.autowareEnabled() &&
      exact_replay_autoware_export_route != nullptr)
    {
      exact_replay_autoware_selected_mission = lmmg::selectNamedNavigationRouteGraph(
        autoware_replay_candidate.graph, *exact_replay_autoware_export_route);
    }
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_autoware_selected_mission.geojson",
      exact_replay_autoware_selected_mission);

    // Lanelet relations may be split at arbitrary GUI speed-span boundaries,
    // while the accepted Mission document intentionally remains expressed in
    // immutable raw replay Edge IDs. Expand only the OSM representation and
    // keep an auditable sidecar for that one-to-many mapping.
    std::optional<lmmg::NamedNavigationRoute> semantic_lanelet_autoware_route;
    std::vector<lmmg::AuthoredStopLine> semantic_lanelet_autoware_stops;
    lmmg::RouteGraph semantic_lanelet_autoware_selected_mission =
      empty_navigation_graph;
    if (config.output.autowareEnabled() &&
      exact_replay_autoware_export_route != nullptr)
    {
      semantic_lanelet_autoware_route = lmmg::remapNamedNavigationRouteAfterSemantics(
        autoware_replay_candidate.graph,
        *exact_replay_autoware_export_route,
        closed_course_semantic_route);
      semantic_lanelet_autoware_stops =
        lmmg::remapResolvedStopLinesAfterSemantics(
        autoware_replay_candidate.graph,
        exact_replay_autoware_stops,
        closed_course_semantic_route);
      semantic_lanelet_autoware_selected_mission =
        lmmg::selectNamedNavigationRouteGraph(
        closed_course_semantic_route.graph,
        *semantic_lanelet_autoware_route);
    }
    if (config.output.autowareEnabled()) {
      lmmg::saveRouteGraphGeoJson(
        output / "route_graph_autoware_semantic_lanelet_selected_mission.geojson",
        semantic_lanelet_autoware_selected_mission);
    }

    if (autoware_replay_authoring_requested) {
      if (exact_replay_autoware_export_route == nullptr) {
        autoware_replay_authoring_validation.errors.push_back(
          "autoware_named_route_not_exact_lossless_replay_reference");
      }
    }

    lmmg::RouteValidationResult closed_course_candidate_validation;
    closed_course_candidate_validation.edited.graph = effective_nav2_closed_course_graph;
    closed_course_candidate_validation.operational_graph = effective_nav2_closed_course_graph;
    closed_course_candidate_validation.operational_ready =
      !effective_nav2_closed_course_graph.edges.empty();
    lmmg::RouteValidationResult autoware_candidate_validation;
    if (edited_topology_vector_map) {
      autoware_candidate_validation = closed_course_route_validation;
      autoware_candidate_validation.operational_graph = edited_vector_export_graph;
      autoware_candidate_validation.operational_ready =
        !edited_vector_export_graph.edges.empty();
    } else {
      autoware_candidate_validation.edited.graph = autoware_replay_candidate.graph;
      autoware_candidate_validation.operational_graph = autoware_replay_candidate.graph;
      autoware_candidate_validation.operational_ready =
        !autoware_replay_candidate.graph.edges.empty();
    }
    lmmg::RouteValidationResult nav2_navigation_validation = route_validation;
    if (nav2_authoring_requested) {
      nav2_navigation_validation.edited.graph = production_nav2_graph;
      nav2_navigation_validation.operational_graph = production_nav2_graph;
      nav2_navigation_validation.operational_ready = !production_nav2_graph.edges.empty();
    }
    lmmg::RouteValidationResult autoware_navigation_validation = route_validation;
    if (production_autoware_authoring_requested) {
      autoware_navigation_validation.edited.graph = production_autoware_graph;
      autoware_navigation_validation.operational_graph = production_autoware_graph;
      autoware_navigation_validation.operational_ready =
        !production_autoware_graph.edges.empty();
    }
    const lmmg::NavigationTargetReadiness nav2_evaluated_readiness =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, result, nav2_navigation_validation, config,
      &closed_course_candidate_validation, &autoware_candidate_validation);
    const lmmg::NavigationTargetReadiness autoware_evaluated_readiness =
      lmmg::evaluateNavigationTargetReadiness(
      dataset, result, autoware_navigation_validation, config,
      &closed_course_candidate_validation, &autoware_candidate_validation);
    lmmg::NavigationTargetReadiness target_readiness = nav2_evaluated_readiness;
    target_readiness.autoware_enabled = autoware_evaluated_readiness.autoware_enabled;
    target_readiness.autoware_map_loader_compatible =
      autoware_evaluated_readiness.autoware_map_loader_compatible;
    target_readiness.autoware_navigation_ready =
      autoware_evaluated_readiness.autoware_navigation_ready;
    target_readiness.autoware_reasons = autoware_evaluated_readiness.autoware_reasons;
    target_readiness.autoware_closed_course_experimental_ready =
      autoware_evaluated_readiness.autoware_closed_course_experimental_ready;
    target_readiness.autoware_experimental_reasons =
      autoware_evaluated_readiness.autoware_experimental_reasons;
    target_readiness.autoware_experimental_warnings =
      autoware_evaluated_readiness.autoware_experimental_warnings;
    const auto append_readiness_reason = [](
      std::vector<std::string> & reasons, const std::string & reason) {
        if (std::find(reasons.begin(), reasons.end(), reason) == reasons.end()) {
          reasons.push_back(reason);
        }
      };
    if (nav2_authoring_requested &&
      (production_nav2_named_route == nullptr || production_nav2_graph.edges.empty()))
    {
      target_readiness.nav2_navigation_ready = false;
      append_readiness_reason(
        target_readiness.nav2_reasons, "navigation_authoring_not_promotable");
    }
    if (nav2_authoring_requested &&
      (closed_nav2_named_route == nullptr || effective_nav2_closed_course_graph.edges.empty()))
    {
      target_readiness.nav2_closed_course_experimental_ready = false;
      append_readiness_reason(
        target_readiness.nav2_experimental_reasons,
        "navigation_authoring_not_valid_for_closed_course");
    }
    if (production_autoware_authoring_requested &&
      (production_autoware_named_route == nullptr || production_autoware_graph.edges.empty()))
    {
      target_readiness.autoware_navigation_ready = false;
      append_readiness_reason(
        target_readiness.autoware_reasons, "navigation_authoring_not_promotable");
    }
    if (edited_topology_vector_map) {
      if (!edited_vector_route_promotable) {
        target_readiness.autoware_closed_course_experimental_ready = false;
        append_readiness_reason(
          target_readiness.autoware_experimental_reasons,
          "edited_lanelet_target_route_not_valid_for_closed_course");
      }
    } else if (autoware_replay_authoring_requested &&
      (exact_replay_autoware_export_route == nullptr ||
      autoware_replay_candidate.graph.edges.empty()))
    {
      target_readiness.autoware_closed_course_experimental_ready = false;
      append_readiness_reason(
        target_readiness.autoware_experimental_reasons,
        "navigation_authoring_not_valid_for_closed_course");
    }
    production_authoring_validation.nav2_promoted =
      nav2_authoring_requested && production_nav2_named_route != nullptr &&
      target_readiness.nav2_navigation_ready;
    production_authoring_validation.autoware_promoted =
      production_autoware_authoring_requested &&
      production_autoware_named_route != nullptr &&
      target_readiness.autoware_navigation_ready;
    closed_course_authoring_validation.nav2_promoted =
      nav2_authoring_requested && closed_nav2_named_route != nullptr &&
      target_readiness.nav2_closed_course_experimental_ready;
    autoware_replay_authoring_validation.autoware_promoted =
      autoware_replay_authoring_requested &&
      exact_replay_autoware_export_route != nullptr &&
      target_readiness.autoware_closed_course_experimental_ready;
    edited_vector_authoring_validation.autoware_promoted =
      edited_vector_route_promotable &&
      target_readiness.autoware_closed_course_experimental_ready;
    // Always emit the separately named closed-course candidate bundle. Its
    // artifact gates are independent from the production outputs below, while
    // per-session deployment acknowledgements deliberately remain false until
    // collected by an operator for the actual run.
    lmmg::Nav2ClosedCourseControls nav2_closed_course_controls;
    nav2_closed_course_controls.enabled = config.output.nav2Enabled();
    if (config.input_type == "rosbag2") {
      nav2_closed_course_controls.base_frame = config.rosbag2.base_frame;
      nav2_closed_course_controls.obstacle_pointcloud_topic =
        config.rosbag2.pointcloud_topic;
    }
    const lmmg::Nav2ClosedCourseAssessment nav2_closed_course_assessment =
      lmmg::saveNav2ClosedCourseExperimentalBundle(
    {
      output / "nav2_map_closed_course_experimental.pgm",
      output / "nav2_map_closed_course_experimental.yaml",
      output / "nav2_route_graph_closed_course_experimental.geojson",
      output / "nav2_waypoints_closed_course_experimental.yaml",
      output / "nav2_closed_course_experimental_params.yaml",
      output / "nav2_closed_course_experimental_readiness.yaml"
      },
      result, effective_nav2_closed_course_graph, config,
      nav2_closed_course_controls,
      nav2_authoring_requested ? closed_nav2_export_route : nullptr,
      nav2_authoring_requested && closed_nav2_export_route != nullptr ?
      &closed_nav2_stops : nullptr);
    RCLCPP_INFO(
      node->get_logger(),
      "Navigation Map closed-course bundle: artifact_ready=%s deployment_ready=%s "
      "(%zu waypoint routes, %zu waypoints)",
      nav2_closed_course_assessment.closed_course_artifact_ready ? "true" : "false",
      nav2_closed_course_assessment.closed_course_deployment_ready ? "true" : "false",
      nav2_closed_course_assessment.waypoint_routes,
      nav2_closed_course_assessment.waypoints);

    lmmg::saveNav2TrinaryPgm(
      output / "nav2_map_generated.pgm",
      result.grids.obstacle_grid, result.grids.observed_free_grid,
      result.grids.unknown_grid, !config.output.nav2Enabled());
    lmmg::saveOccupancyGridYaml(
      output / "nav2_map_generated.yaml", "nav2_map_generated.pgm",
      result.grids.obstacle_grid);
    lmmg::saveNav2TrinaryPgm(
      output / "nav2_map.pgm",
      result.grids.obstacle_grid, result.grids.observed_free_grid,
      result.grids.unknown_grid, !target_readiness.nav2_navigation_ready);
    lmmg::saveOccupancyGridYaml(
      output / "nav2_map.yaml", "nav2_map.pgm", result.grids.obstacle_grid);
    lmmg::saveNav2RouteGraphGeoJson(
      output / "nav2_route_graph_generated.geojson",
      config.output.nav2Enabled() ? result.generation.graph : empty_navigation_graph,
      config.output.nav2_route_max_chord_error,
      config.output.nav2_route_max_segment_length);
    lmmg::saveNav2RouteGraphGeoJson(
      output / "nav2_route_graph.geojson",
      target_readiness.nav2_navigation_ready ?
      production_nav2_graph : empty_navigation_graph,
      config.output.nav2_route_max_chord_error,
      config.output.nav2_route_max_segment_length,
      target_readiness.nav2_navigation_ready ? production_nav2_export_route : nullptr);
    lmmg::saveNav2RouteGraphGeoJson(
      output / "nav2_route_graph_selected.geojson",
      config.output.nav2Enabled() && production_nav2_export_route != nullptr ?
      production_nav2_selected_mission : empty_navigation_graph,
      config.output.nav2_route_max_chord_error,
      config.output.nav2_route_max_segment_length,
      production_nav2_export_route);

    if (config.output.save_debug_grids) {
      lmmg::saveBinaryPcd(
        output / "obstacle_points_classified.pcd",
        result.grids.classified_obstacle_points);
      result.grids.obstacle_grid.savePgm(output / "obstacles.pgm");
      result.grids.inflated_grid.savePgm(output / "obstacles_inflated.pgm");
      result.grids.observed_free_grid.savePgm(output / "observed_free.pgm");
      result.grids.unknown_grid.savePgm(output / "unknown.pgm");
      lmmg::saveOccupancyGridYaml(
        output / "obstacles.yaml", "obstacles.pgm", result.grids.obstacle_grid);
      lmmg::saveOccupancyGridYaml(
        output / "obstacles_inflated.yaml", "obstacles_inflated.pgm",
        result.grids.inflated_grid);
      lmmg::saveOccupancyGridYaml(
        output / "observed_free.yaml", "observed_free.pgm",
        result.grids.observed_free_grid);
      lmmg::saveOccupancyGridYaml(
        output / "unknown.yaml", "unknown.pgm", result.grids.unknown_grid);
    }

    lmmg::Lanelet2AuthoringOptions production_lanelet_authoring;
    if (production_autoware_export_route != nullptr &&
      !production_autoware_graph.edges.empty())
    {
      production_lanelet_authoring.named_route = production_autoware_export_route;
      production_lanelet_authoring.resolved_stop_lines = production_autoware_stops;
    }
    lmmg::Lanelet2AuthoringOptions experimental_lanelet_authoring;
    if (config.output.autowareEnabled() &&
      !closed_course_semantic_route.graph.edges.empty())
    {
      experimental_lanelet_authoring.semantic_source_graph =
        &autoware_replay_candidate.graph;
      experimental_lanelet_authoring.semantic_edge_provenance =
        &closed_course_semantic_route.edge_provenance;
      if (semantic_lanelet_autoware_route) {
        experimental_lanelet_authoring.named_route =
          &*semantic_lanelet_autoware_route;
        experimental_lanelet_authoring.resolved_stop_lines =
          semantic_lanelet_autoware_stops;
      }
    }
    lmmg::Lanelet2AuthoringOptions edited_vector_lanelet_authoring;
    if (edited_topology_vector_map && !edited_vector_export_graph.edges.empty()) {
      edited_vector_lanelet_authoring.semantic_source_graph =
        &edited_vector_validated_source_graph;
      edited_vector_lanelet_authoring.semantic_edge_provenance =
        &edited_vector_semantic_route.edge_provenance;
      if (edited_vector_named_route != nullptr) {
        edited_vector_lanelet_authoring.named_route = edited_vector_named_route;
        edited_vector_lanelet_authoring.resolved_stop_lines = edited_vector_stops;
      }
    }
    const lmmg::RouteGraph & selected_experimental_vector_graph =
      edited_topology_vector_map ?
      edited_vector_export_graph : closed_course_semantic_route.graph;
    const lmmg::Lanelet2AuthoringOptions & selected_experimental_authoring =
      edited_topology_vector_map ?
      edited_vector_lanelet_authoring : experimental_lanelet_authoring;
    lmmg::saveRouteGraphGeoJson(
      output / "route_graph_autoware_selected_source.geojson",
      selected_experimental_vector_graph);
    lmmg::saveReviewGeometryTsv(
      output / "review_geometry_autoware_selected_source.tsv",
      selected_experimental_vector_graph);

    // Lanelet2 and its swept-boundary validator are Autoware products. A
    // Nav2-only small robot may legitimately rotate in place with a zero
    // minimum turning radius; never feed that valid Nav2 configuration into
    // the Autoware vehicle-envelope exporter.
    if (config.output.autowareEnabled() && config.output.save_lanelet2) {
      lmmg::saveLanelet2Osm(
        output / "lanelet2_map_generated.osm",
        config.output.autowareEnabled() ?
        result.generation.graph : empty_navigation_graph,
        config.generator.lanelet2);
      lmmg::saveMapProjectorInfo(output / "map_projector_info.yaml");
      lmmg::saveLanelet2Osm(
        output / "lanelet2_map_validated.osm",
        config.output.autowareEnabled() ?
        route_validation.operational_graph : empty_navigation_graph,
        config.generator.lanelet2);
      lmmg::ClosedCourseLanelet2ExportOptions experimental_lanelet_options;
      experimental_lanelet_options.estimated_vehicle_width = config.generator.robot.width;
      experimental_lanelet_options.estimated_front_extent =
        config.generator.robot.front_extent;
      experimental_lanelet_options.estimated_rear_extent =
        config.generator.robot.rear_extent;
      experimental_lanelet_options.estimated_minimum_turning_radius =
        config.generator.robot.minimum_turning_radius;
      experimental_lanelet_options.lateral_clearance_margin =
        config.generator.robot.clearance_margin;
      experimental_lanelet_options.vehicle_profile = config.generator.robot.profile;
      experimental_lanelet_options.vehicle_base_reference =
        config.generator.robot.base_reference;
      experimental_lanelet_options.vehicle_dimensions_evidence_source =
        config.generator.robot.dimensions_source;
      experimental_lanelet_options.vehicle_dimensions_evidence_confidence =
        config.generator.robot.dimensions_confidence;
      experimental_lanelet_options.vehicle_dimensions_verified =
        config.generator.robot.dimensions_verified;
      experimental_lanelet_options.experimental_ready =
        config.output.autowareEnabled() &&
        target_readiness.autoware_closed_course_experimental_ready;
      experimental_lanelet_options.centerline_source =
        lmmg::toString(vector_map_source.source);
      // Generated maps contain only measured/authored route geometry.  The
      // vehicle swept-envelope exporter still extends the two boundary caps
      // by the configured rear/front extents, but it must not invent passable
      // predecessor/successor Lanelets for planner endpoint context.
      experimental_lanelet_options.test_only_add_open_route_planning_support = false;
      lmmg::ClosedCourseLanelet2ExportOptions topology_lanelet_options =
        experimental_lanelet_options;
      // A physical topology with user edits can contain branches or cycles.
      // Export it for explicit route authoring, but do not inherit the
      // chronological replay readiness claim.
      topology_lanelet_options.experimental_ready = false;
      const lmmg::ClosedCourseLanelet2ExportSummary topology_lanelet_summary =
        lmmg::saveClosedCourseExperimentalLanelet2Osm(
        output / "lanelet2_map_closed_course_topology_candidate.osm",
        config.output.autowareEnabled() ?
        closed_course_topology_route.graph : empty_navigation_graph,
        config.generator.lanelet2, topology_lanelet_options);
      RCLCPP_INFO(
        node->get_logger(),
        "Editable topology Lanelet2 candidate exported %zu/%zu physical edges",
        topology_lanelet_summary.exported_physical_edges,
        topology_lanelet_summary.source_physical_edges);
      const lmmg::ClosedCourseLanelet2ExportSummary experimental_lanelet_summary =
        lmmg::saveClosedCourseExperimentalLanelet2Osm(
        output / "lanelet2_map_closed_course_experimental.osm",
        config.output.autowareEnabled() ?
        selected_experimental_vector_graph : empty_navigation_graph,
        config.generator.lanelet2, experimental_lanelet_options,
        selected_experimental_authoring);
      if (!experimental_lanelet_summary.synthetic_planning_support.empty()) {
        throw std::runtime_error(
                "generated closed-course Lanelet2 map contains forbidden unobserved "
                "synthetic planning-support geometry");
      }
      RCLCPP_INFO(
        node->get_logger(),
        "%s Lanelet2 candidate covered %zu/%zu physical edges as %zu semantic "
        "Lanelet segments (%.3f/%.3f m coverage); experimental_ready=%s",
        edited_topology_vector_map ? "User-authored" : "Observed-driven replay",
        experimental_lanelet_summary.exported_physical_edges,
        experimental_lanelet_summary.source_physical_edges,
        experimental_lanelet_summary.exported_lanelet_segments,
        experimental_lanelet_summary.exported_length,
        experimental_lanelet_summary.source_length,
        target_readiness.autoware_closed_course_experimental_ready ? "true" : "false");
      if (experimental_lanelet_summary.terminal_support_applied) {
        const std::string terminal_support_ids = commaSeparatedIds(
          experimental_lanelet_summary.terminal_support_edge_ids);
        RCLCPP_INFO(
          node->get_logger(),
          "Closed-course Vector Map terminal support composited %.3f m from semantic "
          "topology Edge(s) %s into final named Route Edge %llu; named Route IDs unchanged",
          experimental_lanelet_summary.terminal_support_length_m,
          terminal_support_ids.c_str(),
          static_cast<unsigned long long>(
            experimental_lanelet_summary.terminal_support_named_edge_id));
      }
      lmmg::saveLanelet2Osm(
        output / "lanelet2_map.osm",
        target_readiness.autoware_navigation_ready ?
        production_autoware_graph : empty_navigation_graph,
        config.generator.lanelet2,
        target_readiness.autoware_navigation_ready,
        production_lanelet_authoring);
    }
    lmmg::saveGenerationReport(
      output / "generation_report.yaml", dataset, result, config,
      &autoware_replay_candidate);
    lmmg::saveNavigationAuthoringStatusJson(
      output / "navigation_authoring_status.json",
      production_authoring_validation);
    lmmg::saveNavigationAuthoringStatusJson(
      output / "navigation_authoring_closed_course_status.json",
      edited_topology_vector_map ?
      edited_vector_authoring_validation : autoware_replay_authoring_validation);
    lmmg::saveNavigationAuthoringStatusJson(
      output / "navigation_authoring_autoware_topology_status.json",
      edited_vector_authoring_validation);
    // This is the completion marker consumed by staging. Keep it last so a
    // failed/partial run cannot combine new target files with old readiness.
    lmmg::saveNavigationTargetReadinessYaml(
      output / "navigation_target_readiness.yaml", target_readiness,
      dataset, result, autoware_navigation_validation, config,
      &closed_course_candidate_validation, &autoware_candidate_validation,
      lmmg::toString(vector_map_source.source));

    const lmmg::GenerationStatistics & statistics = result.generation.statistics;
    RCLCPP_INFO(
      node->get_logger(),
      "Generated %zu nodes, %zu directed edges (%zu physical); %zu physical routes failed "
      "clearance checks",
      statistics.route_nodes, statistics.route_edges, statistics.physical_route_edges,
      statistics.impassable_physical_edges);
    RCLCPP_INFO(node->get_logger(), "Output directory: %s", output.string().c_str());
    for (const std::string & warning : result.generation.warnings) {
      RCLCPP_WARN(node->get_logger(), "%s", warning.c_str());
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
}
