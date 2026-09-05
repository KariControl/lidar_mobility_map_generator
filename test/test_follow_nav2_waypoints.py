#!/usr/bin/env python3

import importlib.util
import pathlib
import subprocess
import sys
import tempfile
import unittest


SCRIPT = pathlib.Path(sys.argv[1]).resolve()
SPEC = importlib.util.spec_from_file_location("follow_nav2_waypoints", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class FollowNav2WaypointsTest(unittest.TestCase):
    def write(self, text: str) -> pathlib.Path:
        temporary = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        self.addCleanup(pathlib.Path(temporary.name).unlink, missing_ok=True)
        temporary.write(text)
        temporary.close()
        return pathlib.Path(temporary.name)

    def fixture(self, routes: str) -> pathlib.Path:
        return self.write(
            "schema_version: 2\n"
            'format: "lmmg_nav2_follow_waypoints_routes"\n'
            'action_type: "nav2_msgs/action/FollowWaypoints"\n'
            'frame_id: "map"\nexperimental_only: true\nartifact_ready: true\n'
            f"routes:\n{routes}"
        )

    def test_dry_run_builds_planar_goal(self):
        path = self.fixture(
            "  - route_id: 7\n    closed_loop: false\n    source_edge_ids: [3]\n"
            "    waypoints:\n      - {x: 1.0, y: 2.0, z: 8.0, yaw: 0.0}\n"
            "      - {x: 2.0, y: 2.0, z: -4.0, yaw: 1.0}\n"
        )
        result = subprocess.run(
            [sys.executable, str(SCRIPT), str(path), "--dry-run"],
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertIn('"action_type": "nav2_msgs/action/FollowWaypoints"', result.stdout)
        self.assertIn('"goal_z_is_planar": true', result.stdout)
        self.assertIn('"waypoints": 2', result.stdout)

    def test_multiple_routes_require_explicit_selection(self):
        path = self.fixture(
            "  - route_id: 1\n    waypoints: [{x: 0, y: 0, yaw: 0}, {x: 1, y: 0, yaw: 0}]\n"
            "  - route_id: 2\n    waypoints: [{x: 0, y: 1, yaw: 0}, {x: 1, y: 1, yaw: 0}]\n"
        )
        result = subprocess.run(
            [sys.executable, str(SCRIPT), str(path), "--dry-run"],
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("--route-id", result.stderr)
        selected = subprocess.run(
            [sys.executable, str(SCRIPT), str(path), "--dry-run", "--route-id", "2"],
            text=True,
            capture_output=True,
        )
        self.assertEqual(selected.returncode, 0)
        self.assertIn('"route_id": 2', selected.stdout)

    def test_unready_artifact_is_rejected(self):
        path = self.write(
            'format: "lmmg_nav2_follow_waypoints_routes"\nframe_id: map\n'
            "artifact_ready: false\nroutes: []\n"
        )
        result = subprocess.run(
            [sys.executable, str(SCRIPT), str(path), "--dry-run"],
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("artifact_ready is not true", result.stderr)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
