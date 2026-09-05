#!/usr/bin/env python3
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest


SCRIPT = pathlib.Path(sys.argv[1]).resolve()
sys.argv = [sys.argv[0]]


class AutowareStageTest(unittest.TestCase):
    @staticmethod
    def sha256(path: pathlib.Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def route_graph(edges: list[tuple[int, int, int, float, float]]) -> str:
        return json.dumps(
            {
                "type": "FeatureCollection",
                "features": [
                    {
                        "type": "Feature",
                        "properties": {
                            "id": edge_id,
                            "startid": start_id,
                            "endid": end_id,
                        },
                        "geometry": {
                            "type": "MultiLineString",
                            "coordinates": [[[start_x, 0.0], [end_x, 0.0]]],
                        },
                    }
                    for edge_id, start_id, end_id, start_x, end_x in edges
                ],
            }
        )

    @staticmethod
    def lanelet_osm(two_lanelets: bool = False) -> str:
        nodes = [
            (1, 0, 1), (2, 1, 1), (3, 2, 1),
            (4, 0, -1), (5, 1, -1), (6, 2, -1),
            (7, 0, 0), (8, 1, 0), (9, 2, 0),
        ]
        node_xml = "".join(
            f'<node id="{identifier}" lat="0" lon="0">'
            f'<tag k="local_x" v="{x}"/><tag k="local_y" v="{y}"/>'
            '<tag k="ele" v="0"/></node>'
            for identifier, x, y in nodes
        )
        if two_lanelets:
            ways = [(10, [1, 2]), (11, [4, 5]), (12, [7, 8]),
                    (13, [2, 3]), (14, [5, 6]), (15, [8, 9])]
            relations = [(20, 10, 11, 12, 77), (21, 13, 14, 15, 78)]
        else:
            ways = [(10, [1, 3]), (11, [4, 6]), (12, [7, 9])]
            relations = [(20, 10, 11, 12, 77)]
        way_xml = "".join(
            f'<way id="{identifier}">' +
            "".join(f'<nd ref="{node}"/>' for node in references) +
            '<tag k="type" v="virtual"/></way>'
            for identifier, references in ways
        )
        relation_xml = "".join(
            f'<relation id="{identifier}"><member type="way" ref="{left}" role="left"/>'
            f'<member type="way" ref="{right}" role="right"/>'
            f'<member type="way" ref="{center}" role="centerline"/>'
            '<tag k="type" v="lanelet"/><tag k="subtype" v="road"/>'
            '<tag k="location" v="urban"/><tag k="one_way" v="yes"/>'
            '<tag k="participant:vehicle" v="yes"/><tag k="speed_limit" v="0.9"/>'
            f'<tag k="route_edge_id" v="{route_edge_id}"/>'
            '</relation>'
            for identifier, left, right, center, route_edge_id in relations
        )
        return f'<?xml version="1.0"?><osm version="0.6">{node_xml}{way_xml}{relation_xml}</osm>\n'

    def make_output(self, ready: bool) -> pathlib.Path:
        root = pathlib.Path(self.temporary.name) / ("ready" if ready else "blocked")
        root.mkdir()
        (root / "navigation_target_readiness.yaml").write_text(
            "schema_version: 3\ngeneration_complete: true\n"
            "requested_target_mode: \"autoware\"\nautoware:\n"
            "  enabled: true\n"
            f"  closed_course_experimental_ready: {'true' if ready else 'false'}\n",
            encoding="utf-8",
        )
        (root / "map_projector_info.yaml").write_text(
            "projector_type: Local\n", encoding="utf-8"
        )
        (root / "pointcloud_map.pcd").write_text(
            "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
            encoding="ascii",
        )
        (root / "lanelet2_map_closed_course_experimental.osm").write_text(
            self.lanelet_osm(), encoding="utf-8")
        (root / "trajectory_processed.tum").write_text(
            "0.0 0 0 0 0 0 0 1\n1.0 1 0 0 0 0 0 1\n"
            "2.0 2 0 0 0 0 0 1\n", encoding="ascii")
        (root / "route_graph_autoware_replay_candidate_metadata.yaml").write_text(
            "frame_id: \"map\"\nnodes: {}\nedges: {}\n"
            "autoware_replay_derivative:\n"
            "  scope: \"autoware_lanelet_only\"\n"
            "  terminal_localization_settling_verified: true\n"
            "  terminal_tail_omitted: true\n"
            "  omitted_planar_length_m: 0.45\n",
            encoding="utf-8",
        )
        (root / "route_graph_autoware_replay_candidate.geojson").write_text(
            self.route_graph([(77, 1, 2, 0.0, 2.0)]),
            encoding="utf-8",
        )
        (root / "route_graph_closed_course_replay_candidate.geojson").write_text(
            self.route_graph([(77, 1, 2, 0.0, 2.0)]),
            encoding="utf-8",
        )
        (root / "route_graph_autoware_selected_mission.geojson").write_text(
            self.route_graph([(77, 1, 2, 0.0, 2.0)]),
            encoding="utf-8",
        )
        return root

    def make_named_output(self, *, maximal_fixture: bool = True) -> pathlib.Path:
        root = self.make_output(True)
        full_edges = [
            (77, 1, 2, 0.0, 1.0),
            (78, 2, 1 if maximal_fixture else 3, 1.0, 2.0),
        ]
        selected_edges = (
            full_edges
            if maximal_fixture
            else full_edges[:1]
        )
        selected_edge_ids = [edge[0] for edge in selected_edges]
        lanelet = root / "lanelet2_map_closed_course_experimental.osm"
        lanelet_text = self.lanelet_osm(two_lanelets=True)
        for order, edge_id in enumerate(selected_edge_ids):
            marker = f'<tag k="route_edge_id" v="{edge_id}"/></relation>'
            lanelet_text = lanelet_text.replace(
                marker,
                f'<tag k="route_edge_id" v="{edge_id}"/>'
                '<tag k="named_route_id" v="42"/>'
                '<tag k="named_route_name" v="short stage route"/>'
                '<tag k="named_route_target" v="autoware"/>'
                f'<tag k="named_route_order" v="{order}"/></relation>',
                1,
            )
        lanelet.write_text(lanelet_text, encoding="utf-8")
        (root / "route_graph_autoware_replay_candidate.geojson").write_text(
            self.route_graph(full_edges),
            encoding="utf-8",
        )
        (root / "route_graph_closed_course_replay_candidate.geojson").write_text(
            self.route_graph(full_edges),
            encoding="utf-8",
        )
        (root / "route_graph_autoware_selected_mission.geojson").write_text(
            self.route_graph(selected_edges),
            encoding="utf-8",
        )
        (root / "navigation_authoring_autoware_replay.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "frame_id": "map",
                    "graph_fingerprint": "0123456789abcdef",
                    "routes": [
                        {
                            "id": 42,
                            "name": "short stage route",
                            "target": "autoware",
                            "start_node_id": 1,
                            "end_node_id": 1 if maximal_fixture else 2,
                            "ordered_edge_ids": selected_edge_ids,
                            "validation_requested": True,
                            "promotion_requested": True,
                        }
                    ],
                    "stop_lines": [],
                }
            ),
            encoding="utf-8",
        )
        (root / "navigation_authoring_closed_course_status.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "frame_id": "map",
                    "graph_fingerprint": "0123456789abcdef",
                    "autoware": {
                        "selected_route_id": 42,
                        "promoted": True,
                        "stop_lines_valid": True,
                    },
                    "nav2": {
                        "selected_route_id": None,
                        "promoted": False,
                        "stop_lines_valid": True,
                    },
                    "errors": [],
                    "routes": [
                        {
                            "id": 42,
                            "target": "autoware",
                            "valid": True,
                            "promotion_eligible": True,
                        }
                    ],
                    "stop_lines": [],
                }
            ),
            encoding="utf-8",
        )
        (root / "semantic_features.tsv").write_text(
            "LMMG_SEMANTICS\t2\nFRAME\tmap\n",
            encoding="utf-8",
        )
        (root / "semantic_features.geojson").write_text(
            '{"type":"FeatureCollection","features":[]}\n',
            encoding="utf-8",
        )
        (root / "route_graph_autoware_semantic_lanelet_candidate.geojson").write_text(
            (root / "route_graph_autoware_replay_candidate.geojson").read_text(
                encoding="utf-8"
            ),
            encoding="utf-8",
        )
        (root / "review_geometry_autoware_semantic_lanelet_candidate.tsv").write_text(
            "VERSION\t2\nFRAME\tmap\n",
            encoding="utf-8",
        )
        if maximal_fixture:
            authoring = root / "navigation_authoring_autoware_replay.json"
            semantic = root / "semantic_features.tsv"
            full_graph = root / "route_graph_autoware_replay_candidate.geojson"
            (root / "maximal_autoware_authoring_fixture_manifest.json").write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "kind": "gui_equivalent_exact_autoware_replay_authoring_fixture",
                        "scenario": "full_map_route",
                        "scope": "autoware_lossless_replay",
                        "operational_claim": False,
                        "source": {
                            "route_graph": full_graph.name,
                            "route_graph_sha256": self.sha256(full_graph),
                            "graph_fingerprint_recomputed": True,
                        },
                        "route_selection": {
                            "algorithm": "exact_chronological_replay_file_order_all_edges",
                            "edge_id_order_assumed": False,
                            "source_order_preserved": True,
                            "is_cycle": False,
                            "full_ordered_edge_ids": selected_edge_ids,
                            "selected_mission_edge_ids": selected_edge_ids,
                            "excluded_terminal_support_edge_ids": [],
                            "full_edge_count": len(selected_edge_ids),
                            "selected_edge_count": len(selected_edge_ids),
                            "excluded_terminal_support_edge_count": 0,
                            "selected_to_full_arc_length_ratio": 1.0,
                        },
                        "terminal_planning_support_preflight": {
                            "production_ready": False,
                            "deployment_ready": False,
                            "synthetic_support_is_part_of_named_route": False,
                        },
                        "stop_line": {
                            "classification": "closed_course_virtual_unsurveyed",
                            "physical_stop_line_verified": False,
                        },
                        "authoring": {
                            "file": authoring.name,
                            "sha256": self.sha256(authoring),
                            "schema_version": 1,
                            "gui_schema_equivalent": True,
                            "scope": "autoware_lossless_replay",
                        },
                        "semantic_authoring": {
                            "file": semantic.name,
                            "sha256": self.sha256(semantic),
                            "schema_version": 2,
                            "gui_schema_equivalent": True,
                        },
                    }
                )
                + "\n",
                encoding="utf-8",
            )
        return root

    def make_manual_output(self) -> pathlib.Path:
        root = self.make_named_output(maximal_fixture=False)
        (root / "navigation_target_readiness.yaml").write_text(
            "schema_version: 3\ngeneration_complete: true\n"
            "requested_target_mode: \"autoware\"\nautoware:\n"
            "  enabled: true\n  centerline_source: \"edited_topology\"\n"
            "  closed_course_experimental_ready: true\n",
            encoding="utf-8",
        )
        (root / "vector_map_source.tsv").write_text(
            "LMMG_VECTOR_MAP_SOURCE\t1\n"
            "SOURCE\tedited_topology\n"
            "FRAME\tmap\n"
            "GRAPH_FINGERPRINT\t0123456789abcdef\n",
            encoding="utf-8",
        )
        renames = {
            "route_graph_autoware_replay_candidate.geojson":
                "route_graph_autoware_topology_source.geojson",
            "route_graph_autoware_selected_mission.geojson":
                "route_graph_autoware_topology_selected_mission.geojson",
            "navigation_authoring_autoware_replay.json":
                "navigation_authoring_autoware_topology.json",
            "navigation_authoring_closed_course_status.json":
                "navigation_authoring_autoware_topology_status.json",
            "semantic_features.tsv": "semantic_features_autoware_topology.tsv",
            "semantic_features.geojson":
                "semantic_features_autoware_topology.geojson",
            "route_graph_autoware_semantic_lanelet_candidate.geojson":
                "route_graph_autoware_topology_semantic_candidate.geojson",
            "review_geometry_autoware_semantic_lanelet_candidate.tsv":
                "review_geometry_autoware_topology_semantic_candidate.tsv",
        }
        for source, target in renames.items():
            (root / source).rename(root / target)
        (root / "route_graph_closed_course_replay_candidate.geojson").unlink()
        (root / "route_graph_autoware_replay_candidate_metadata.yaml").unlink()
        (root / "trajectory_processed.tum").write_text(
            "0.0 0 5 0 0 0 0 1\n1.0 1 5 0 0 0 0 1\n",
            encoding="ascii",
        )

        lanelet = root / "lanelet2_map_closed_course_experimental.osm"
        text = lanelet.read_text(encoding="utf-8")
        for node_id, original, extended in (
            (1, "0", "-0.15"), (4, "0", "-0.15"),
            (3, "2", "2.15"), (6, "2", "2.15"),
        ):
            text = text.replace(
                f'<node id="{node_id}" lat="0" lon="0"><tag k="local_x" v="{original}"/>',
                f'<node id="{node_id}" lat="0" lon="0"><tag k="local_x" v="{extended}"/>',
            )
        metadata = (
            '<tag k="boundary_model" v="trajectory_derived_estimated_drivable_corridor"/>'
            '<tag k="estimated_vehicle_width_m" v="0.8"/>'
            '<tag k="estimated_front_extent_m" v="0.1"/>'
            '<tag k="estimated_rear_extent_m" v="0.1"/>'
            '<tag k="vehicle_minimum_turning_radius_m" v="1.0"/>'
            '<tag k="estimated_lateral_margin_m" v="0.1"/>'
            '<tag k="estimated_boundary_interpolation_guard_m" v="0.05"/>'
            '<tag k="estimated_effective_lateral_margin_m" v="0.15"/>'
            '<tag k="estimated_longitudinal_endpoint_guard_m" v="0.05"/>'
            '<tag k="estimated_boundary_algorithm" v="oriented_rectangular_swept_envelope"/>'
            '<tag k="vehicle_profile" v="car"/>'
            '<tag k="vehicle_base_reference" v="rear_axle_ground_projection"/>'
            '<tag k="vehicle_dimensions_evidence_source" v="catalog_estimated"/>'
            '<tag k="vehicle_dimensions_evidence_confidence" v="medium"/>'
            '<tag k="vehicle_dimensions_verified" v="no"/>'
            '<tag k="centerline_source" v="edited_topology"/>'
            '<tag k="provenance" v="user_authored_centerline"/>'
            '<tag k="observed_driven" v="no"/>'
            '<tag k="validation_status" '
            'v="user_authored_vehicle_footprint_validated_candidate"/>'
        )
        lanelet.write_text(text.replace("</relation>", metadata + "</relation>"),
                           encoding="utf-8")
        return root

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_stages_only_expected_autoware_filenames(self) -> None:
        root = self.make_output(True)
        subprocess.run([str(SCRIPT), str(root)], check=True, capture_output=True, text=True)
        stage = root / "autoware_closed_course_experimental_map"
        self.assertEqual(
            sorted(path.name for path in stage.iterdir()),
            [
                "EXPERIMENTAL_ONLY.yaml",
                "autoware_candidate_acceptance.json",
                "lanelet2_map.osm",
                "map_projector_info.yaml",
                "pointcloud_map.pcd",
                "route_graph_autoware_replay_candidate.geojson",
                "route_graph_autoware_replay_candidate_metadata.yaml",
                "route_graph_autoware_selected_mission.geojson",
                "route_graph_closed_course_replay_candidate.geojson",
            ],
        )
        self.assertEqual(
            (root / "route_graph_autoware_selected_mission.geojson").read_bytes(),
            (stage / "route_graph_autoware_selected_mission.geojson").read_bytes(),
        )
        self.assertEqual(
            (root / "autoware_candidate_acceptance.json").read_bytes(),
            (stage / "autoware_candidate_acceptance.json").read_bytes(),
        )
        self.assertEqual(
            os.stat(root / "pointcloud_map.pcd").st_ino,
            os.stat(stage / "pointcloud_map.pcd").st_ino,
        )
        manifest = (stage / "EXPERIMENTAL_ONLY.yaml").read_text(encoding="utf-8")
        self.assertIn("schema_version: 2", manifest)
        self.assertIn("production_ready: false", manifest)
        self.assertIn("deployment_ready: false", manifest)
        self.assertIn(
            'acceptance_scope: "static_format_geometry_coverage_only"', manifest
        )
        self.assertIn("accepted_for_autoware_motion_test: false", manifest)
        self.assertIn("lanelet_relations: 1", manifest)
        self.assertIn(
            'autoware_replay_candidate_metadata: '
            '"route_graph_autoware_replay_candidate_metadata.yaml"', manifest
        )
        self.assertIn("terminal_localization_settling_verified: true", manifest)
        self.assertIn("terminal_tail_omitted: true", manifest)
        self.assertIn("omitted_planar_length_m: 0.45", manifest)
        self.assertIn(
            'full_map_source_graph: "route_graph_autoware_replay_candidate.geojson"',
            manifest,
        )
        self.assertRegex(
            manifest,
            r'full_map_source_graph_sha256: "[0-9a-f]{64}"',
        )
        self.assertIn(
            f'full_map_source_graph_sha256: "{self.sha256(stage / "route_graph_autoware_replay_candidate.geojson")}"',
            manifest,
        )
        self.assertEqual(
            (root / "route_graph_autoware_replay_candidate_metadata.yaml").read_text(
                encoding="utf-8"
            ),
            (stage / "route_graph_autoware_replay_candidate_metadata.yaml").read_text(
                encoding="utf-8"
            ),
        )

    def test_named_route_stages_independent_acceptance_evidence(self) -> None:
        root = self.make_named_output()
        subprocess.run([str(SCRIPT), str(root)], check=True, capture_output=True, text=True)
        stage = root / "autoware_closed_course_experimental_map"
        self.assertEqual(
            sorted(path.name for path in stage.iterdir()),
            [
                "EXPERIMENTAL_ONLY.yaml",
                "autoware_candidate_acceptance.json",
                "lanelet2_map.osm",
                "map_projector_info.yaml",
                "maximal_autoware_authoring_fixture_manifest.json",
                "navigation_authoring_autoware_replay.json",
                "navigation_authoring_closed_course_status.json",
                "pointcloud_map.pcd",
                "review_geometry_autoware_semantic_lanelet_candidate.tsv",
                "route_graph_autoware_replay_candidate.geojson",
                "route_graph_autoware_replay_candidate_metadata.yaml",
                "route_graph_autoware_selected_mission.geojson",
                "route_graph_autoware_semantic_lanelet_candidate.geojson",
                "route_graph_closed_course_replay_candidate.geojson",
                "semantic_features.geojson",
                "semantic_features.tsv",
            ],
        )
        manifest = (stage / "EXPERIMENTAL_ONLY.yaml").read_text(encoding="utf-8")
        self.assertIn("schema_version: 7", manifest)
        self.assertIn("named_route:", manifest)
        self.assertIn("id: 42", manifest)
        self.assertIn('name: "short stage route"', manifest)
        self.assertIn("ordered_edges: 2", manifest)
        self.assertIn("authored_stop_lines: 0", manifest)
        self.assertIn('coverage_reference: "named_route_source_graph"', manifest)
        self.assertIn(
            'full_map_source_graph: "route_graph_autoware_replay_candidate.geojson"',
            manifest,
        )
        self.assertIn(
            'mission_source_graph: "route_graph_autoware_selected_mission.geojson"',
            manifest,
        )
        self.assertIn('authoring_scope: "autoware_lossless_replay"', manifest)
        self.assertIn(
            'navigation_authoring: "navigation_authoring_autoware_replay.json"',
            manifest,
        )
        self.assertIn(
            'maximal_authoring_fixture_manifest: "maximal_autoware_authoring_fixture_manifest.json"',
            manifest,
        )
        self.assertIn("artifact_sha256:", manifest)
        self.assertNotIn('  "EXPERIMENTAL_ONLY.yaml":', manifest)
        for artifact in stage.iterdir():
            if artifact.name != "EXPERIMENTAL_ONLY.yaml":
                self.assertIn(
                    f'  "{artifact.name}": "{self.sha256(artifact)}"',
                    manifest,
                )
        self.assertRegex(
            manifest,
            r'mission_source_graph_sha256: "[0-9a-f]{64}"',
        )
        self.assertIn(
            f'mission_source_graph_sha256: "{self.sha256(stage / "route_graph_autoware_selected_mission.geojson")}"',
            manifest,
        )
        acceptance = json.loads(
            (stage / "autoware_candidate_acceptance.json").read_text(encoding="utf-8")
        )
        self.assertTrue(acceptance["accepted"])
        self.assertEqual(acceptance["named_route"]["ordered_edge_ids"], [77, 78])
        full_map_graph = json.loads(
            (stage / "route_graph_autoware_replay_candidate.geojson").read_text(
                encoding="utf-8"
            )
        )
        mission_graph = json.loads(
            (stage / "route_graph_autoware_selected_mission.geojson").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(len(full_map_graph["features"]), 2)
        self.assertEqual(len(mission_graph["features"]), 2)

    def test_gui_named_route_without_maximal_fixture_stages_schema_5(self) -> None:
        root = self.make_named_output(maximal_fixture=False)
        subprocess.run([str(SCRIPT), str(root)], check=True, capture_output=True, text=True)
        stage = root / "autoware_closed_course_experimental_map"
        self.assertFalse(
            (stage / "maximal_autoware_authoring_fixture_manifest.json").exists()
        )
        manifest = (stage / "EXPERIMENTAL_ONLY.yaml").read_text(encoding="utf-8")
        self.assertIn("schema_version: 5", manifest)
        self.assertIn("named_route:", manifest)
        self.assertIn("ordered_edges: 1", manifest)
        self.assertNotIn("maximal_authoring_fixture_manifest:", manifest)
        self.assertIn(
            '  "navigation_authoring_autoware_replay.json":', manifest
        )
        acceptance = json.loads(
            (stage / "autoware_candidate_acceptance.json").read_text(encoding="utf-8")
        )
        self.assertEqual(acceptance["named_route"]["ordered_edge_ids"], [77])

    def test_user_authored_topology_stages_source_specific_evidence(self) -> None:
        root = self.make_manual_output()
        subprocess.run([str(SCRIPT), str(root)], check=True, capture_output=True, text=True)
        stage = root / "autoware_closed_course_experimental_map"
        names = {path.name for path in stage.iterdir()}
        self.assertIn("route_graph_autoware_topology_source.geojson", names)
        self.assertIn("route_graph_autoware_topology_selected_mission.geojson", names)
        self.assertIn("navigation_authoring_autoware_topology.json", names)
        self.assertIn("navigation_authoring_autoware_topology_status.json", names)
        self.assertIn("semantic_features_autoware_topology.tsv", names)
        self.assertIn("semantic_features_autoware_topology.geojson", names)
        self.assertIn("vector_map_source.tsv", names)
        self.assertNotIn("route_graph_autoware_replay_candidate.geojson", names)
        self.assertNotIn("route_graph_closed_course_replay_candidate.geojson", names)
        self.assertNotIn("route_graph_autoware_replay_candidate_metadata.yaml", names)
        manifest = (stage / "EXPERIMENTAL_ONLY.yaml").read_text(encoding="utf-8")
        self.assertIn("schema_version: 8", manifest)
        self.assertIn('centerline_source: "edited_topology"', manifest)
        self.assertIn("user_authored: true", manifest)
        self.assertIn(
            'runtime_route_geometry: "user_authored_vehicle_footprint_validated_lanelet_centerlines"',
            manifest,
        )
        self.assertIn(
            'full_map_source_graph: "route_graph_autoware_topology_source.geojson"',
            manifest,
        )
        self.assertIn('authoring_scope: "autoware_edited_topology"', manifest)
        acceptance = json.loads(
            (stage / "autoware_candidate_acceptance.json").read_text(encoding="utf-8")
        )
        self.assertTrue(acceptance["accepted"], acceptance["errors"])
        self.assertEqual(acceptance["centerline_source"], "edited_topology")
        self.assertEqual(acceptance["metrics"]["trajectory_centerline_coverage"], 0.0)
        self.assertEqual(acceptance["metrics"]["acceptance_centerline_coverage"], 1.0)

    def test_invalid_maximal_fixture_falls_back_to_schema_5(self) -> None:
        root = self.make_named_output(maximal_fixture=False)
        (root / "maximal_autoware_authoring_fixture_manifest.json").write_text(
            '{"schema_version":1,"kind":"unexpected"}\n', encoding="utf-8"
        )
        result = subprocess.run(
            [str(SCRIPT), str(root)], check=True, capture_output=True, text=True
        )
        stage = root / "autoware_closed_course_experimental_map"
        manifest = (stage / "EXPERIMENTAL_ONLY.yaml").read_text(encoding="utf-8")
        self.assertIn("schema_version: 5", manifest)
        self.assertFalse(
            (stage / "maximal_autoware_authoring_fixture_manifest.json").exists()
        )
        self.assertIn("maximal authoring fixture manifest ignored", result.stderr)

    def test_named_route_refuses_missing_selected_mission_sidecar(self) -> None:
        root = self.make_named_output()
        (root / "route_graph_autoware_selected_mission.geojson").unlink()
        result = subprocess.run(
            [str(SCRIPT), str(root)], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("required artifact is missing", result.stderr)
        self.assertFalse((root / "autoware_closed_course_experimental_map").exists())

    def test_named_route_never_substitutes_topology_authoring(self) -> None:
        root = self.make_named_output()
        dedicated = root / "navigation_authoring_autoware_replay.json"
        dedicated.rename(root / "navigation_authoring.json")
        result = subprocess.run(
            [str(SCRIPT), str(root)], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 4)
        report = json.loads(
            (root / "autoware_candidate_acceptance.json").read_text(encoding="utf-8")
        )
        self.assertIn(
            "missing_navigation_authoring",
            [error["code"] for error in report["errors"]],
        )
        self.assertFalse((root / "autoware_closed_course_experimental_map").exists())

    def test_named_route_refuses_cross_scope_replay_authoring(self) -> None:
        root = self.make_named_output()
        path = root / "navigation_authoring_autoware_replay.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        document["routes"][0]["target"] = "nav2"
        path.write_text(json.dumps(document), encoding="utf-8")
        result = subprocess.run(
            [str(SCRIPT), str(root)], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 4)
        report = json.loads(
            (root / "autoware_candidate_acceptance.json").read_text(encoding="utf-8")
        )
        self.assertIn(
            "cross_scope_navigation_authoring",
            [error["code"] for error in report["errors"]],
        )
        self.assertFalse((root / "autoware_closed_course_experimental_map").exists())

    def test_refuses_nav2_only_target(self) -> None:
        root = self.make_output(True)
        (root / "navigation_target_readiness.yaml").write_text(
            "schema_version: 3\ngeneration_complete: true\n"
            "requested_target_mode: \"nav2\"\nautoware:\n"
            "  enabled: false\n  closed_course_experimental_ready: false\n",
            encoding="utf-8",
        )
        result = subprocess.run(
            [str(SCRIPT), str(root)], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 3)
        self.assertIn("not selected", result.stderr)

    def test_refuses_incomplete_generation_before_target_artifacts(self) -> None:
        root = self.make_output(True)
        (root / "navigation_target_readiness.yaml").write_text(
            "schema_version: 3\ngeneration_complete: false\n"
            "requested_target_mode: \"autoware\"\nautoware:\n"
            "  enabled: true\n  closed_course_experimental_ready: false\n",
            encoding="utf-8",
        )
        (root / "lanelet2_map_closed_course_experimental.osm").unlink()
        result = subprocess.run(
            [str(SCRIPT), str(root)], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 3)
        self.assertIn("generation is incomplete", result.stderr)

    def test_refuses_candidate_metadata_with_missing_evidence_values(self) -> None:
        root = self.make_output(True)
        (root / "route_graph_autoware_replay_candidate_metadata.yaml").write_text(
            "autoware_replay_derivative:\n"
            "  terminal_localization_settling_verified: true\n",
            encoding="utf-8",
        )
        result = subprocess.run([str(SCRIPT), str(root)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 4)
        self.assertIn("lacks valid terminal-settling evidence", result.stderr)
        self.assertFalse((root / "autoware_closed_course_experimental_map").exists())

    def test_refuses_failed_experimental_readiness(self) -> None:
        root = self.make_output(False)
        result = subprocess.run([str(SCRIPT), str(root)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 3)
        self.assertFalse((root / "autoware_closed_course_experimental_map").exists())

    def test_ready_claim_does_not_hide_an_invalid_export(self) -> None:
        root = self.make_output(True)
        lanelet = root / "lanelet2_map_closed_course_experimental.osm"
        lanelet.write_text(
            lanelet.read_text(encoding="utf-8").replace(
                '<nd ref="3"/>', '<nd ref="999"/>', 1),
            encoding="utf-8",
        )
        result = subprocess.run([str(SCRIPT), str(root)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 4)
        self.assertIn("failed post-export", result.stderr)
        self.assertFalse((root / "autoware_closed_course_experimental_map").exists())

    def test_refuses_coordinate_only_lanelet_join_without_shared_ids(self) -> None:
        root = self.make_output(True)
        lanelet = root / "lanelet2_map_closed_course_experimental.osm"
        text = self.lanelet_osm(two_lanelets=True)
        duplicate = (
            '<node id="10" lat="0" lon="0"><tag k="local_x" v="1"/>'
            '<tag k="local_y" v="1"/><tag k="ele" v="0"/></node>'
        )
        text = text.replace("</osm>", duplicate + "</osm>")
        text = text.replace(
            '<way id="13"><nd ref="2"/>',
            '<way id="13"><nd ref="10"/>',
            1,
        )
        lanelet.write_text(text, encoding="utf-8")
        result = subprocess.run(
            [str(SCRIPT), str(root)], capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 4)
        report = json.loads(
            (root / "autoware_candidate_acceptance.json").read_text(encoding="utf-8")
        )
        self.assertIn(
            "unshared_lanelet_boundary_endpoint",
            [error["code"] for error in report["errors"]],
        )
        self.assertFalse((root / "autoware_closed_course_experimental_map").exists())

    def test_replace_refreshes_an_existing_bundle(self) -> None:
        root = self.make_output(True)
        subprocess.run([str(SCRIPT), str(root)], check=True, capture_output=True, text=True)
        source = root / "lanelet2_map_closed_course_experimental.osm"
        source.write_text(self.lanelet_osm(two_lanelets=True), encoding="utf-8")
        updated_graph = self.route_graph(
            [(77, 1, 2, 0.0, 1.0), (78, 2, 3, 1.0, 2.0)]
        )
        (root / "route_graph_autoware_replay_candidate.geojson").write_text(
            updated_graph, encoding="utf-8"
        )
        (root / "route_graph_closed_course_replay_candidate.geojson").write_text(
            updated_graph, encoding="utf-8"
        )
        subprocess.run(
            [str(SCRIPT), str(root), "--replace"],
            check=True,
            capture_output=True,
            text=True,
        )
        stage = root / "autoware_closed_course_experimental_map"
        self.assertIn("relation id=\"21\"", (stage / "lanelet2_map.osm").read_text())
        self.assertIn(
            "lanelet_relations: 2",
            (stage / "EXPERIMENTAL_ONLY.yaml").read_text(encoding="utf-8"),
        )
        self.assertFalse(any("previous" in path.name for path in root.iterdir()))

    def test_runs_configured_real_lanelet_smoke_checker(self) -> None:
        root = self.make_output(True)
        checker = pathlib.Path(self.temporary.name) / "lanelet-smoke"
        checker.write_text(
            "#!/usr/bin/env bash\n"
            "echo lanelets=1\n"
            "echo AUTOWARE_LANELET_SMOKE=PASS\n",
            encoding="utf-8",
        )
        checker.chmod(0o755)
        environment = os.environ.copy()
        environment["LMMG_AUTOWARE_LANELET_SMOKE_EXECUTABLE"] = str(checker)
        environment["LMMG_REQUIRE_AUTOWARE_LANELET_SMOKE"] = "true"
        subprocess.run(
            [str(SCRIPT), str(root)], check=True, capture_output=True, text=True,
            env=environment,
        )
        stage = root / "autoware_closed_course_experimental_map"
        self.assertTrue((stage / "autoware_lanelet_smoke_report.txt").is_file())
        manifest = (stage / "EXPERIMENTAL_ONLY.yaml").read_text(encoding="utf-8")
        self.assertIn("lanelet2_local_loader_verified: true", manifest)
        self.assertIn("routing_graph_verified: true", manifest)


if __name__ == "__main__":
    unittest.main()
