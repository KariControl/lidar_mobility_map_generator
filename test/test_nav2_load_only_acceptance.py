#!/usr/bin/env python3
"""Static boundary checks for the Nav2 alpha load-only acceptance path."""

from __future__ import annotations

import ast
import pathlib
import subprocess
import sys
import unittest


HOST_RUNNER = pathlib.Path(sys.argv[1]).resolve()
CONTAINER_RUNNER = pathlib.Path(sys.argv[2]).resolve()
PROBE = pathlib.Path(sys.argv[3]).resolve()


class Nav2LoadOnlyAcceptanceStaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.host = HOST_RUNNER.read_text(encoding="utf-8")
        cls.container = CONTAINER_RUNNER.read_text(encoding="utf-8")
        cls.probe = PROBE.read_text(encoding="utf-8")

    def test_scripts_have_valid_syntax(self) -> None:
        for script in (HOST_RUNNER, CONTAINER_RUNNER):
            subprocess.run(["bash", "-n", str(script)], check=True)
        ast.parse(self.probe, filename=str(PROBE))

    def test_container_starts_only_map_and_route_consumers(self) -> None:
        self.assertIn("ros2 run nav2_map_server map_server", self.container)
        self.assertIn("ros2 run nav2_route route_server", self.container)
        self.assertIn('"${lmmg_waypoint_args[@]}" --dry-run', self.container)
        for forbidden in (
            "ros2 run nav2_waypoint_follower",
            "ros2 run nav2_planner",
            "ros2 run nav2_controller",
            "ros2 run nav2_bt_navigator",
            "/compute_route",
            "/follow_waypoints",
        ):
            self.assertNotIn(forbidden, self.container)

    def test_probe_has_no_planning_or_action_api(self) -> None:
        self.assertIn('transition(node, "map_server"', self.probe)
        self.assertIn('transition(node, "route_server"', self.probe)
        self.assertIn('"planning": False', self.probe)
        self.assertIn('"action_execution": False', self.probe)
        self.assertIn('"robot_motion": False', self.probe)
        for forbidden in (
            "from nav2_msgs.action",
            "from rclpy.action",
            "ActionClient",
            "ActionServer",
            "ComputeRoute",
            "NavigateToPose",
            "send_goal",
        ):
            self.assertNotIn(forbidden, self.probe)

    def test_host_runner_is_offline_read_only_and_uses_fresh_reports(self) -> None:
        self.assertIn("--network none", self.host)
        self.assertIn("--read-only", self.host)
        self.assertIn('"${lmmg_artifacts}:/artifacts:ro"', self.host)
        self.assertIn("report directory must be fresh", self.host)
        self.assertIn("run_nav2_load_only_acceptance.sh /artifacts /reports", self.host)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
