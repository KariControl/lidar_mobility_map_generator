#!/usr/bin/env python3
"""Fixture tests for the public Vector Map edit smoke auditor."""

from __future__ import annotations

import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / "scripts" / "check_vector_map_edit_smoke.py"


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class VectorMapEditSmokeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.output = pathlib.Path(self.temporary.name) / "rosbag_velodyne"
        self.output.mkdir()

    def write_fixture(
        self,
        *,
        positive_gap_m: int = 0,
        promoted_edges: list[int] | None = None,
        malformed_osm: bool = False,
        malformed_acceptance: bool = False,
        lanelet_slow_speed: str = "0.3",
        candidate_accepted: bool = True,
        stale_stage: bool = False,
        centerline_source: str = "recorded_trajectory",
        canonicalization_drift: bool = False,
    ) -> None:
        self.assertIn(centerline_source, ("recorded_trajectory", "edited_topology"))
        edited_topology = centerline_source == "edited_topology"
        graph = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [float(index * 10), 0.0, 0.0],
                    },
                    "properties": {"id": index + 1},
                }
                for index in range(4)
            ]
            + [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "MultiLineString",
                        "coordinates": [
                            [
                                [float(ordinal * 10), 0.0, 0.0],
                                [float((ordinal + 1) * 10), 0.0, 0.0],
                            ]
                        ],
                    },
                    "properties": {"id": edge_id, "cost": 10.0},
                }
                for ordinal, edge_id in enumerate((10, 11, 12))
            ],
        }
        graph_name = (
            "route_graph_autoware_topology_source.geojson"
            if edited_topology
            else "route_graph_autoware_replay_candidate.geojson"
        )
        graph_path = self.output / graph_name
        graph_path.write_text(json.dumps(graph, indent=2) + "\n", encoding="utf-8")

        slow_start = positive_gap_m
        fast_end_s = "9.999999999996" if canonicalization_drift else "10"
        semantics = "\n".join(
            [
                "LMMG_SEMANTICS\t2",
                "FRAME\tmap",
                "FEATURE\t1\tspeed_limit\troute_edges\t1\t0\t0\t0\t0\t0.90\t0\tfast\t\t10,11",
                "SPAN\t1\t0\t10\t2\t10\t2\t0\t0\t10\t0\t0",
                f"SPAN\t1\t1\t11\t0\t{fast_end_s}\t10\t0\t0\t20\t0\t0",
                "FEATURE\t2\tspeed_limit\troute_edges\t1\t0\t0\t0\t0\t0.30\t0\tslow\t\t12",
                f"SPAN\t2\t0\t12\t{slow_start}\t8\t{20 + slow_start}\t0\t0\t28\t0\t0",
                "",
            ]
        )
        semantics_name = (
            "semantic_features_autoware_topology.tsv"
            if edited_topology
            else "semantic_features.tsv"
        )
        semantics_path = self.output / semantics_name
        semantics_path.write_text(semantics, encoding="utf-8")

        route_edges = promoted_edges if promoted_edges is not None else [10, 11, 12]
        authoring = {
            "schema_version": 1,
            "frame_id": "map",
            "graph_fingerprint": "0123456789abcdef",
            "routes": [
                {
                    "id": 1,
                    "name": "走行軌跡全体",
                    "target": "autoware",
                    "start_node_id": 1,
                    "end_node_id": 4,
                    "ordered_edge_ids": route_edges,
                    "validation_requested": True,
                    "promotion_requested": True,
                }
            ],
            "stop_lines": [
                {
                    "id": 7,
                    "name": "stop_user_01",
                    "edge_id": 12,
                    "s": 4.0,
                    "width_m": 2.0,
                    "anchor": [24.0, 0.0, 0.0],
                    "target": "autoware",
                }
            ],
        }
        authoring_name = (
            "navigation_authoring_autoware_topology.json"
            if edited_topology
            else "navigation_authoring_autoware_replay.json"
        )
        authoring_path = self.output / authoring_name
        authoring_path.write_text(
            json.dumps(authoring, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        canonical_osm = self.output / "lanelet2_map_closed_course_experimental.osm"
        if malformed_osm:
            osm = "<osm><relation>"
        else:
            osm = f"""<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="fixture">
  <node id="1" lat="0" lon="0"/>
  <node id="2" lat="0" lon="0.00001"/>
  <way id="300">
    <nd ref="1"/><nd ref="2"/>
    <tag k="type" v="stop_line"/>
    <tag k="subtype" v="solid"/>
    <tag k="name" v="stop_user_01"/>
    <tag k="authored_stop_line_id" v="7"/>
    <tag k="source_route_edge_id" v="12"/>
    <tag k="source_route_edge_s_m" v="4"/>
  </way>
  <relation id="200">
    <tag k="type" v="lanelet"/>
    <tag k="source_route_edge_id" v="10"/>
    <tag k="source_start_s_m" v="2"/>
    <tag k="source_end_s_m" v="10"/>
    <tag k="generator_speed_limit_mps" v="0.9"/>
    <tag k="semantic_segment" v="yes"/>
  </relation>
  <relation id="201">
    <tag k="type" v="lanelet"/>
    <tag k="source_route_edge_id" v="11"/>
    <tag k="source_start_s_m" v="0"/>
    <tag k="source_end_s_m" v="10"/>
    <tag k="generator_speed_limit_mps" v="0.9"/>
    <tag k="semantic_segment" v="yes"/>
  </relation>
  <relation id="202">
    <member type="relation" ref="400" role="regulatory_element"/>
    <tag k="type" v="lanelet"/>
    <tag k="source_route_edge_id" v="12"/>
    <tag k="source_start_s_m" v="{slow_start}"/>
    <tag k="source_end_s_m" v="8"/>
    <tag k="generator_speed_limit_mps" v="{lanelet_slow_speed}"/>
    <tag k="semantic_segment" v="yes"/>
  </relation>
  <relation id="400">
    <member type="way" ref="300" role="ref_line"/>
    <tag k="type" v="regulatory_element"/>
    <tag k="subtype" v="traffic_sign"/>
    <tag k="sign_type" v="stop_sign"/>
    <tag k="name" v="stop_user_01"/>
    <tag k="authored_stop_line_id" v="7"/>
  </relation>
</osm>
"""
        canonical_osm.write_text(osm, encoding="utf-8")

        stage = self.output / "autoware_closed_course_experimental_map"
        stage.mkdir()
        (stage / semantics_name).write_bytes(semantics_path.read_bytes())
        (stage / authoring_name).write_bytes(authoring_path.read_bytes())
        (stage / "lanelet2_map.osm").write_bytes(canonical_osm.read_bytes())
        if stale_stage:
            with (stage / semantics_name).open("a", encoding="utf-8") as stream:
                stream.write("# stale stage\n")

        acceptance = {
            "format_version": 1,
            "accepted": candidate_accepted,
            "accepted_for_autoware_motion_test": False,
            "production_ready": False,
            "deployment_ready": False,
            "centerline_source": centerline_source,
            "errors": [] if candidate_accepted else ["fixture rejection"],
            "input": {
                "lanelet2_map_sha256": sha256(canonical_osm),
                (
                    "semantic_features_autoware_topology_tsv_sha256"
                    if edited_topology
                    else "semantic_features_tsv_sha256"
                ): sha256(semantics_path),
                "navigation_authoring_sha256": sha256(authoring_path),
                "full_map_source_graph_sha256": sha256(graph_path),
            },
            "counts": {
                "authored_stop_lines": 1,
                "authored_stop_regulatory_elements": 1,
                "lanelets": 3,
            },
            "named_route": {
                "id": 1,
                "name": "走行軌跡全体",
                "target": "autoware",
                "ordered_edge_ids": route_edges,
            },
        }
        acceptance_path = stage / "autoware_candidate_acceptance.json"
        if malformed_acceptance:
            acceptance_path.write_text("{\n", encoding="utf-8")
        else:
            acceptance_path.write_text(
                json.dumps(acceptance, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

    def run_audit(self, stop_selector: str = "stop_user_01") -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                str(self.output),
                "--stop-line-id",
                stop_selector,
            ],
            check=False,
            text=True,
            capture_output=True,
        )

    def assert_rejected(self, result: subprocess.CompletedProcess[str], text: str) -> None:
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertFalse(report["accepted"])
        self.assertIn(text, "\n".join(report["errors"]))

    def test_accepts_exact_boundary_full_route_stop_and_fresh_stage(self) -> None:
        self.write_fixture()
        before = {
            path.relative_to(self.output): sha256(path)
            for path in self.output.rglob("*")
            if path.is_file()
        }

        result = self.run_audit()

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["accepted"])
        self.assertEqual(report["route"]["ordered_edge_ids"], [10, 11, 12])
        self.assertEqual(report["speed_transition"]["positive_gap_m"], 0.0)
        self.assertEqual(report["speed_transition"]["overlap_m"], 0.0)
        self.assertTrue(report["speed_transition"]["anchors_equal"])
        self.assertEqual(report["stop_line"]["name"], "stop_user_01")
        after = {
            path.relative_to(self.output): sha256(path)
            for path in self.output.rglob("*")
            if path.is_file()
        }
        self.assertEqual(before, after, "the read-only audit changed an artifact")

    def test_numeric_stop_line_id_is_also_accepted(self) -> None:
        self.write_fixture()
        result = self.run_audit("7")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_accepts_edited_topology_artifact_names_and_source_binding(self) -> None:
        self.write_fixture(centerline_source="edited_topology")
        result = self.run_audit()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["centerline_source"], "edited_topology")
        self.assertTrue(report["route"]["complete_selected_source_exact_order"])

    def test_accepts_sub_nanometre_serialization_drift_at_exact_boundary(self) -> None:
        self.write_fixture(canonicalization_drift=True)
        result = self.run_audit()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["speed_transition"]["positive_gap_m"], 0.0)
        self.assertEqual(report["speed_transition"]["overlap_m"], 0.0)

    def test_rejects_positive_default_speed_gap(self) -> None:
        self.write_fixture(positive_gap_m=1)
        self.assert_rejected(self.run_audit(), "positive default-speed gap")

    def test_rejects_shortened_promoted_route(self) -> None:
        self.write_fixture(promoted_edges=[10, 11])
        self.assert_rejected(self.run_audit(), "does not contain every selected-source Edge")

    def test_rejects_missing_requested_stop_line(self) -> None:
        self.write_fixture()
        self.assert_rejected(self.run_audit("missing_stop"), "exactly one stop line matching")

    def test_rejects_malformed_lanelet2(self) -> None:
        self.write_fixture(malformed_osm=True)
        self.assert_rejected(self.run_audit(), "invalid canonical Lanelet2 OSM")

    def test_rejects_lanelet_speed_different_from_semantic_span(self) -> None:
        self.write_fixture(lanelet_slow_speed="0.5")
        self.assert_rejected(self.run_audit(), "Lanelet2 speed differs")

    def test_rejects_stale_stage_copy(self) -> None:
        self.write_fixture(stale_stage=True)
        self.assert_rejected(self.run_audit(), "staged semantic TSV is stale")

    def test_rejects_candidate_acceptance_failure(self) -> None:
        self.write_fixture(candidate_accepted=False)
        self.assert_rejected(self.run_audit(), "candidate acceptance is not accepted")

    def test_rejects_malformed_candidate_acceptance_json(self) -> None:
        self.write_fixture(malformed_acceptance=True)
        self.assert_rejected(self.run_audit(), "invalid candidate acceptance JSON")


if __name__ == "__main__":
    unittest.main()
