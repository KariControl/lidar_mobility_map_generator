#!/usr/bin/env python3

import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path


EDITOR = Path(sys.argv[1]).resolve()
WEB_SOURCE = Path(sys.argv[2]).resolve()


def write_route_graph(
    path: Path,
    edge_id: int,
    start_x: float,
    end_x: float,
    z: float = 0.0,
) -> None:
    length = end_x - start_x
    path.write_text(
        "\n".join(
            [
                "VERSION\t2",
                "FRAME\tmap",
                f"NODE\t1\tendpoint\t{start_x}\t0\t{z}",
                f"NODE\t2\tendpoint\t{end_x}\t0\t{z}",
                f"EDGE\t{edge_id}\t1\t2\t-\t1\t{length}\t2\t0\t1\t1\t2\t1\t",
                (
                    f"SAMPLE\t{edge_id}\t0\t{start_x}\t0\t{z}"
                    f"\t{start_x}\t1\t{z}\t{start_x}\t-1\t{z}\t1\t1\t1\t1"
                ),
                (
                    f"SAMPLE\t{edge_id}\t1\t{end_x}\t0\t{z}"
                    f"\t{end_x}\t1\t{z}\t{end_x}\t-1\t{z}\t1\t1\t1\t1"
                ),
                "",
            ]
        ),
        encoding="utf-8",
    )


def write_ascii_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    path.write_text(
        "\n".join(
            [
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                f"WIDTH {len(points)}",
                "HEIGHT 1",
                f"POINTS {len(points)}",
                "DATA ascii",
                *(f"{x} {y} {z}" for x, y, z in points),
                "",
            ]
        ),
        encoding="utf-8",
    )


def write_editor_fixture(root: Path) -> None:
    write_route_graph(root / "review_geometry_generated.tsv", 11, 100.0, 106.0, 6.0)
    for name in (
        "review_geometry_closed_course_replay_candidate.tsv",
        "review_geometry_autoware_replay_candidate.tsv",
    ):
        write_route_graph(root / name, 101, 0.0, 20.0, 1.0)


class EditorServer:
    def __init__(
        self,
        root: Path,
        maximum_points: int = 100,
        one_click_session: str | None = None,
    ) -> None:
        self.root = root
        self.maximum_points = maximum_points
        self.one_click_session = one_click_session
        self.process: subprocess.Popen[str] | None = None
        self.base_url = ""
        self.token = ""

    def __enter__(self):
        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except PermissionError as error:
            if os.environ.get("LMMG_REQUIRE_WEB_EDITOR_TESTS") == "1":
                raise AssertionError(
                    f"required local-socket editor test cannot run: {error}"
                ) from error
            raise unittest.SkipTest(f"local sockets are unavailable: {error}") from error
        with listener:
            listener.bind(("127.0.0.1", 0))
            port = listener.getsockname()[1]
        self.base_url = f"http://127.0.0.1:{port}"
        command = [
            str(EDITOR),
            "--output-directory",
            str(self.root),
            "--editor-mode",
            "vector_map",
            "--bind",
            "127.0.0.1",
            "--port",
            str(port),
            "--max-points",
            str(self.maximum_points),
            "--open-browser",
            "false",
        ]
        if self.one_click_session is not None:
            command.extend(
                [
                    "--enable-vector-map-one-click-export",
                    "true",
                    "--vector-map-one-click-session",
                    self.one_click_session,
                ]
            )
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        html = None
        for _ in range(100):
            if self.process.poll() is not None:
                raise AssertionError(self.process.stderr.read())
            try:
                with urllib.request.urlopen(self.base_url + "/", timeout=0.25) as response:
                    html = response.read().decode("utf-8")
                break
            except (OSError, urllib.error.URLError):
                time.sleep(0.02)
        if html is None:
            raise AssertionError("semantic editor HTTP server did not start")
        match = re.search(r"const csrfToken='([0-9a-f]{48})'", html)
        if match is None:
            raise AssertionError("semantic editor page did not contain its CSRF token")
        self.token = match.group(1)
        return self

    def __exit__(self, _exception_type, _exception, _traceback) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            self.process.wait(timeout=3.0)
        if self.process is not None and self.process.stderr is not None:
            self.process.stderr.close()

    def get_json(self, path: str) -> dict:
        with urllib.request.urlopen(self.base_url + path, timeout=2.0) as response:
            return json.loads(response.read().decode("utf-8"))

    def post_json(self, path: str, body: str, content_type: str = "text/plain") -> dict:
        request = urllib.request.Request(
            self.base_url + path,
            data=body.encode("utf-8"),
            headers={
                "Content-Type": f"{content_type}; charset=utf-8",
                "X-LMMG-Token": self.token,
            },
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return json.loads(response.read().decode("utf-8"))


class VectorMapManualEditorTest(unittest.TestCase):
    def test_source_selector_activates_the_separate_autoware_topology_scope(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_editor_fixture(root)

            with EditorServer(root) as editor:
                initial = editor.get_json("/api/context")
                self.assertEqual(
                    initial["vector_map_source"]["source"], "recorded_trajectory"
                )
                self.assertEqual(
                    initial["vector_map_source"]["navigation_scope"],
                    "autoware_lossless_replay",
                )

                unauthorized = urllib.request.Request(
                    editor.base_url + "/api/vector-map-source",
                    data=b"edited_topology",
                    method="POST",
                )
                with self.assertRaises(urllib.error.HTTPError) as denied:
                    urllib.request.urlopen(unauthorized, timeout=2.0)
                self.assertEqual(denied.exception.code, 403)
                self.assertFalse((root / "vector_map_source.tsv").exists())

                switched = editor.post_json(
                    "/api/vector-map-source", "edited_topology"
                )["context"]
                source = switched["vector_map_source"]
                self.assertEqual(source["source"], "edited_topology")
                self.assertEqual(
                    source["navigation_scope"], "autoware_edited_topology"
                )
                self.assertTrue(source["valid"])
                self.assertEqual(
                    switched["semantic_graph_scope"], "autoware_edited_topology"
                )
                self.assertEqual(
                    [
                        edge["id"]
                        for edge in switched["navigation_graphs"][
                            "autoware_edited_topology"
                        ]["edges"]
                    ],
                    [11],
                )
                self.assertEqual(
                    [
                        edge["id"]
                        for edge in switched["navigation_graphs"][
                            "autoware_lossless_replay"
                        ]["edges"]
                    ],
                    [101],
                )

                topology = editor.get_json(
                    "/api/navigation-authoring?scope=autoware_edited_topology"
                )
                self.assertEqual(topology["allowed_target"], "autoware")
                self.assertFalse(topology["exact_lossless"])
                self.assertEqual(
                    topology["document"]["graph_fingerprint"],
                    source["graph_fingerprint"],
                )

                invalid_document = {
                    "schema_version": 1,
                    "frame_id": "map",
                    "graph_fingerprint": source["graph_fingerprint"],
                    "routes": [
                        {
                            "id": 1,
                            "name": "manual",
                            "target": "nav2",
                            "start_node_id": 1,
                            "end_node_id": 2,
                            "ordered_edge_ids": [11],
                            "validation_requested": True,
                            "promotion_requested": True,
                        }
                    ],
                    "stop_lines": [],
                }
                with self.assertRaises(urllib.error.HTTPError) as rejected:
                    editor.post_json(
                        "/api/navigation-authoring?scope=autoware_edited_topology",
                        json.dumps(invalid_document),
                        "application/json",
                    )
                self.assertEqual(rejected.exception.code, 400)
                self.assertFalse(
                    (root / "navigation_authoring_autoware_topology.json").exists()
                )

                invalid_document["routes"][0]["target"] = "autoware"
                saved = editor.post_json(
                    "/api/navigation-authoring?scope=autoware_edited_topology",
                    json.dumps(invalid_document),
                    "application/json",
                )
                self.assertTrue(saved["saved"])
                self.assertTrue(saved["validation"]["structural_valid"])
                self.assertEqual(saved["document"]["routes"][0]["name"], "manual")
                self.assertTrue(
                    (root / "navigation_authoring_autoware_topology.json").is_file()
                )
                self.assertFalse(
                    (root / "navigation_authoring_autoware_replay.json").exists()
                )

            selection = (root / "vector_map_source.tsv").read_text(encoding="utf-8")
            self.assertIn("SOURCE\tedited_topology\n", selection)

    def test_complete_recorded_route_keeps_its_stable_storage_name(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_editor_fixture(root)

            with EditorServer(root) as editor:
                saved = editor.post_json(
                    "/api/autoware-full-replay-route", "{}", "application/json"
                )
                self.assertTrue(saved["saved"])
                self.assertEqual(
                    saved["document"]["routes"][0]["name"], "走行軌跡全体"
                )

            stored = json.loads(
                (root / "navigation_authoring_autoware_replay.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(stored["routes"][0]["name"], "走行軌跡全体")

    def test_one_click_output_uses_the_selected_edited_topology(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_editor_fixture(root)

            with EditorServer(root, one_click_session="public-test-session") as editor:
                switched = editor.post_json(
                    "/api/vector-map-source", "edited_topology"
                )["context"]
                fingerprint = switched["vector_map_source"]["graph_fingerprint"]
                replay_draft = root / "navigation_authoring_autoware_replay.json"
                replay_draft.write_bytes(b"replay draft must remain unchanged\n")

                exported = editor.post_json(
                    "/api/autoware-one-click-export", "{}", "application/json"
                )
                self.assertTrue(exported["export_requested"])
                self.assertEqual(exported["centerline_source"], "edited_topology")
                self.assertEqual(exported["route_edge_count"], 1)
                self.assertEqual(editor.process.wait(timeout=3.0), 0)

                marker = (
                    root / ".autoware_one_click_export_requested"
                ).read_text(encoding="utf-8")
                self.assertIn("SESSION\tpublic-test-session\n", marker)
                self.assertIn("SOURCE\tedited_topology\n", marker)
                self.assertIn(f"GRAPH_FINGERPRINT\t{fingerprint}\n", marker)
                topology = json.loads(
                    (
                        root / "navigation_authoring_autoware_topology.json"
                    ).read_text(encoding="utf-8")
                )
                self.assertEqual(topology["graph_fingerprint"], fingerprint)
                self.assertEqual(topology["routes"][0]["target"], "autoware")
                self.assertEqual(
                    topology["routes"][0]["name"], "編集した道路中心線全体"
                )
                self.assertEqual(topology["routes"][0]["ordered_edge_ids"], [11])
                self.assertTrue(topology["routes"][0]["promotion_requested"])
                self.assertEqual(
                    replay_draft.read_bytes(),
                    b"replay draft must remain unchanged\n",
                )

    def test_one_click_accepts_a_new_unvalidated_manual_open_chain(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_editor_fixture(root)

            with EditorServer(root, one_click_session="manual-chain-session") as editor:
                editor.post_json("/api/route-edit", "clear")
                first = editor.post_json(
                    "/api/route-edit", "add_node\t100\t0\t6"
                )["context"]
                first_id = next(
                    node["id"]
                    for node in first["nodes"]
                    if node["provenance"] == "manual"
                )
                second = editor.post_json(
                    "/api/route-edit", "add_node\t106\t0\t6"
                )["context"]
                second_id = next(
                    node["id"]
                    for node in second["nodes"]
                    if node["provenance"] == "manual" and node["id"] != first_id
                )
                edited = editor.post_json(
                    "/api/route-edit",
                    (
                        f"add_edge\t{first_id}\t{second_id}\tone_way\n"
                        "POINT\t100\t0\t6\n"
                        "POINT\t106\t0\t6"
                    ),
                )["context"]
                manual_edges = [
                    edge
                    for edge in edited["edges"]
                    if edge["provenance"] == "manual"
                ]
                self.assertEqual(len(manual_edges), 1)
                self.assertFalse(manual_edges[0]["passable"])

                switched = editor.post_json(
                    "/api/vector-map-source", "edited_topology"
                )["context"]
                fingerprint = switched["vector_map_source"]["graph_fingerprint"]
                with self.assertRaises(urllib.error.HTTPError) as not_one_click:
                    editor.post_json(
                        "/api/autoware-full-replay-route", "{}", "application/json"
                    )
                self.assertEqual(not_one_click.exception.code, 400)
                self.assertFalse(
                    (root / "navigation_authoring_autoware_topology.json").exists()
                )
                exported = editor.post_json(
                    "/api/autoware-one-click-export", "{}", "application/json"
                )
                self.assertTrue(exported["export_requested"])
                self.assertEqual(exported["centerline_source"], "edited_topology")
                self.assertEqual(exported["route_edge_count"], 1)
                self.assertEqual(editor.process.wait(timeout=3.0), 0)

                topology = json.loads(
                    (
                        root / "navigation_authoring_autoware_topology.json"
                    ).read_text(encoding="utf-8")
                )
                self.assertEqual(topology["graph_fingerprint"], fingerprint)
                self.assertEqual(
                    topology["routes"][0]["ordered_edge_ids"],
                    [manual_edges[0]["id"]],
                )
                self.assertTrue(topology["routes"][0]["validation_requested"])
                self.assertTrue(topology["routes"][0]["promotion_requested"])
                marker = (
                    root / ".autoware_one_click_export_requested"
                ).read_text(encoding="utf-8")
                self.assertIn("SESSION\tmanual-chain-session\n", marker)
                self.assertIn("SOURCE\tedited_topology\n", marker)
                self.assertIn(f"GRAPH_FINGERPRINT\t{fingerprint}\n", marker)

    def test_auto_elevation_uses_route_trajectory_and_cloud_and_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_editor_fixture(root)
            (root / "trajectory_processed.tum").write_text(
                "0.0 0 0 1 0 0 0 1\n"
                "1.0 20 0 11 0 0 0 1\n",
                encoding="utf-8",
            )
            write_ascii_pcd(
                root / "pointcloud_map.pcd",
                [
                    (29.8, -0.1, 4.0),
                    (29.9, 0.1, 4.0),
                    (30.0, 0.0, 4.0),
                    (30.1, -0.1, 4.0),
                    (30.2, 0.1, 20.0),
                ],
            )

            with EditorServer(root) as editor:
                def add_node(x: float, y: float = 0.0) -> dict:
                    return editor.post_json(
                        "/api/route-edit", f"add_node\t{x}\t{y}\tauto"
                    )["context"]

                def manual_z(context: dict, x: float) -> float:
                    matches = [
                        node
                        for node in context["nodes"]
                        if node["provenance"] == "manual"
                        and abs(node["position"][0] - x) < 1.0e-6
                    ]
                    self.assertEqual(len(matches), 1)
                    return matches[0]["position"][2]

                context = add_node(103.0)
                self.assertAlmostEqual(manual_z(context, 103.0), 6.0, places=6)
                context = add_node(5.0)
                self.assertAlmostEqual(manual_z(context, 5.0), 3.5, places=6)
                context = add_node(30.0)
                self.assertAlmostEqual(manual_z(context, 30.0), 4.0, places=6)

                edits_before = (root / "route_edits.tsv").read_bytes()
                node_count_before = len(context["nodes"])
                with self.assertRaises(urllib.error.HTTPError) as rejected:
                    add_node(80.0, 80.0)
                self.assertEqual(rejected.exception.code, 400)
                self.assertEqual(
                    (root / "route_edits.tsv").read_bytes(), edits_before
                )
                after = editor.get_json("/api/context")
                self.assertEqual(len(after["nodes"]), node_count_before)

    def test_public_web_page_contains_the_source_controls(self):
        html = WEB_SOURCE.read_text(encoding="utf-8")
        for required in (
            'id="vectorMapSourceSelect"',
            'value="recorded_trajectory"',
            'value="edited_topology"',
            'value="autoware_edited_topology"',
            "/api/vector-map-source",
            "autoware_edited_topology",
        ):
            self.assertTrue(required in html, f"missing web editor contract: {required}")

    def test_public_web_localizes_only_builtin_route_names_for_display(self):
        html = WEB_SOURCE.read_text(encoding="utf-8")
        mapping = re.search(
            r"const builtinRouteNameKeys=new Map\(\[(.*?)\]\);", html
        )
        self.assertIsNotNone(mapping)
        self.assertEqual(
            re.findall(r"\['([^']+)','([^']+)'\]", mapping.group(1)),
            [
                ("走行軌跡全体", "builtin_route_recorded_full"),
                ("編集した道路中心線全体", "builtin_route_edited_full"),
            ],
        )
        for required in (
            "builtin_route_recorded_full:'Complete recorded trajectory'",
            "builtin_route_edited_full:'Complete edited road centerline'",
            "function displayRouteName(name){const raw=String(name??'');"
            "const key=builtinRouteNameKeys.get(raw);return key?t(key):raw}",
            "setNavigationRouteNameInput(route.name)",
            "input.dataset.storedRouteName=raw",
            "return input.dataset.storedRouteName??input.value.trim()",
            "delete event.target.dataset.storedRouteName",
            "row.children[1].textContent=displayRouteName(route.name)",
            "name:displayRouteName(json.route_name||json.route_id)",
        ):
            self.assertIn(required, html)

    def test_public_web_status_distinguishes_strict_and_selected_source_checks(self):
        html = WEB_SOURCE.read_text(encoding="utf-8")
        for required in (
            "production_gate:'Strict real-vehicle check:",
            "production_gate:'実車走行向けの厳格な確認：",
            "closed_course_gate_edited:'Regenerated edited-centerline check:",
            "closed_course_gate_edited:'再生成した道路中心線の確認結果：",
            "closedGateKey=vectorMapSource()==='edited_topology'?"
            "'closed_course_gate_edited':'closed_course_gate'",
            "t(closedGateKey,{state:candidateStateLabel(closedState),"
            "operational:closedOperational,total:closedTotal})",
        ):
            self.assertIn(required, html)
        self.assertNotIn("production_gate:'道路中心線の確認結果：", html)
        self.assertNotIn("closed_course_gate:'走行軌跡の確認結果：", html)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
