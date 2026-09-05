#!/usr/bin/env python3

import importlib.util
import json
import os
import re
import tempfile
import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("ROS_LOG_DIR", str(Path(tempfile.gettempdir()) / "lmmg_test_ros_logs"))


def load_review_launch():
    path = PACKAGE_ROOT / "launch" / "review.launch.py"
    spec = importlib.util.spec_from_file_location("lmmg_review_launch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ReviewLaunchAutoFitTest(unittest.TestCase):
    def test_navigation_mode_shows_only_trinary_map_and_route_overlays(self):
        module = load_review_launch()
        generated = Path(
            module._mode_rviz_config(
                PACKAGE_ROOT / "rviz" / "review.rviz", "navigation_map"
            )
        )
        try:
            text = generated.read_text(encoding="utf-8")
            self.assertIn("/review_vector_map/navigation_map", text)
            navigation_block = re.search(
                r"(?ms)^    - .*?/review_vector_map/navigation_map\s*$.*?(?=^    - |^  Enabled:)",
                text,
            )
            self.assertIsNotNone(navigation_block)
            self.assertRegex(navigation_block.group(0), r"(?m)^      Enabled: true$")
            self.assertRegex(navigation_block.group(0), r"(?m)^      Value: true$")
            self.assertIn("/review_vector_map/trajectory_processed", text)
            self.assertIn("/review_vector_map/route_graph", text)
            self.assertIn("/review_vector_map/semantic_features", text)
            self.assertIn("/review_vector_map/issues", text)
            self.assertNotIn("/review_vector_map/lanelet2", text)
            self.assertNotIn("Lanelet2 Experimental / Review", text)
            self.assertNotIn("/review_vector_map/pointcloud_map", text)
            self.assertNotIn("/review_vector_map/trajectory_raw", text)
            self.assertNotIn("/review_vector_map/corridors", text)
            self.assertNotIn("/review_vector_map/obstacles\n", text)
            self.assertNotIn("/review_vector_map/obstacles_inflated", text)
            self.assertNotIn("/review_vector_map/observed_free", text)
            self.assertNotIn("/review_vector_map/unknown", text)

            # This is deliberately a parsed-document assertion.  A previous
            # implementation preserved the expected topic strings but joined
            # two Display blocks without a newline.  RViz consequently opened
            # with zero subscriptions even though the text-only checks passed.
            document = yaml.safe_load(text)
            manager = document["Visualization Manager"]
            displays = manager["Displays"]
            self.assertEqual(len(displays), 5)
            self.assertEqual(
                {
                    display["Topic"]["Value"]
                    for display in displays
                },
                {
                    "/review_vector_map/navigation_map",
                    "/review_vector_map/trajectory_processed",
                    "/review_vector_map/route_graph",
                    "/review_vector_map/semantic_features",
                    "/review_vector_map/issues",
                },
            )
            self.assertTrue(all(display["Enabled"] for display in displays))
            self.assertTrue(all(display["Value"] for display in displays))
            self.assertEqual(len(manager["Tools"]), 5)
        finally:
            generated.unlink(missing_ok=True)

    def test_vector_and_combined_modes_keep_lanelet2_display(self):
        module = load_review_launch()
        base = PACKAGE_ROOT / "rviz" / "review.rviz"
        for mode in ("vector_map", "combined"):
            self.assertEqual(module._mode_rviz_config(base, mode), str(base))

    def test_dedicated_launches_select_lanelet2_publication_by_map_type(self):
        vector_launch = (PACKAGE_ROOT / "launch" / "edit_vector_map.launch.py").read_text(
            encoding="utf-8"
        )
        navigation_launch = (
            PACKAGE_ROOT / "launch" / "edit_navigation_map.launch.py"
        ).read_text(encoding="utf-8")
        shared_launch = (PACKAGE_ROOT / "launch" / "edit_and_review.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"publish_lanelet2": "true"', vector_launch)
        self.assertIn('"publish_lanelet2": "false"', navigation_launch)
        self.assertIn('"publish_navigation_map": "false"', vector_launch)
        self.assertIn('"publish_navigation_map": "true"', navigation_launch)
        self.assertIn('"review_mode": editor_mode', shared_launch)
        self.assertIn('"publish_lanelet2": publish_lanelet2', shared_launch)
        self.assertIn(
            '"publish_navigation_map": publish_navigation_map', shared_launch
        )
        self.assertIn('"open_browser": LaunchConfiguration("open_browser")', vector_launch)
        self.assertIn(
            '"open_browser": LaunchConfiguration("open_browser")', navigation_launch
        )
        self.assertIn('open_browser = LaunchConfiguration("open_browser")', shared_launch)
        self.assertIn('DeclareLaunchArgument("open_browser", default_value="true")', shared_launch)

    def test_review_node_publishes_selected_trinary_navigation_map(self):
        source = (PACKAGE_ROOT / "src" / "review_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('"~/navigation_map", latched_qos', source)
        self.assertIn('"nav2_map_closed_course_experimental.yaml"', source)
        self.assertIn('"nav2_map_generated.yaml"', source)
        self.assertIn('"nav2_map.yaml"', source)
        self.assertIn("message.data = loaded.occupancy_values", source)

    def test_far_origin_grid_centers_top_down_view(self):
        module = load_review_launch()
        with tempfile.TemporaryDirectory(prefix="lmmg_review_test_") as directory:
            output = Path(directory)
            (output / "obstacles.pgm").write_text(
                "P2\n4 2\n255\n0 0 0 0\n0 0 0 0\n", encoding="ascii"
            )
            (output / "obstacles.yaml").write_text(
                "image: obstacles.pgm\n"
                "resolution: 0.5\n"
                "origin: [-268320.0, -57682.0, 0.0]\n",
                encoding="utf-8",
            )
            generated = Path(
                module._auto_fit_rviz_config(PACKAGE_ROOT / "rviz" / "review.rviz", output)
            )
            try:
                text = generated.read_text(encoding="utf-8")
                self.assertRegex(text, r"(?m)^      X: -268319(?:\.0+)?$")
                self.assertRegex(text, r"(?m)^      Y: -57681\.5$")
                scale_match = re.search(r"(?m)^      Scale: ([0-9.eE+-]+)$", text)
                self.assertIsNotNone(scale_match)
                self.assertGreater(float(scale_match.group(1)), 0.0)
            finally:
                generated.unlink(missing_ok=True)

    def test_trajectory_is_used_when_grid_is_absent(self):
        module = load_review_launch()
        with tempfile.TemporaryDirectory(prefix="lmmg_review_test_") as directory:
            output = Path(directory)
            (output / "trajectory_processed.tum").write_text(
                "0.0 10.0 -4.0 0 0 0 0 1\n"
                "1.0 14.0 6.0 0 0 0 0 1\n",
                encoding="utf-8",
            )
            generated = Path(
                module._auto_fit_rviz_config(PACKAGE_ROOT / "rviz" / "review.rviz", output)
            )
            try:
                text = generated.read_text(encoding="utf-8")
                self.assertRegex(text, r"(?m)^      X: 12(?:\.0+)?$")
                self.assertRegex(text, r"(?m)^      Y: 1(?:\.0+)?$")
            finally:
                generated.unlink(missing_ok=True)

    def test_vector_map_auto_fit_prefers_trajectory_over_large_grid(self):
        module = load_review_launch()
        with tempfile.TemporaryDirectory(prefix="lmmg_review_test_") as directory:
            output = Path(directory)
            (output / "obstacles.pgm").write_text(
                "P2\n100 100\n255\n" + "0 " * 10000 + "\n", encoding="ascii"
            )
            (output / "obstacles.yaml").write_text(
                "image: obstacles.pgm\n"
                "resolution: 1.0\n"
                "origin: [-50.0, -50.0, 0.0]\n",
                encoding="utf-8",
            )
            (output / "trajectory_processed.tum").write_text(
                "0.0 10.0 -2.0 0 0 0 0 1\n"
                "1.0 20.0 2.0 0 0 0 0 1\n",
                encoding="utf-8",
            )
            generated = Path(
                module._auto_fit_rviz_config(
                    PACKAGE_ROOT / "rviz" / "review.rviz",
                    output,
                    prefer_trajectory=True,
                )
            )
            try:
                text = generated.read_text(encoding="utf-8")
                self.assertRegex(text, r"(?m)^      X: 15(?:\.0+)?$")
                self.assertRegex(text, r"(?m)^      Y: 0(?:\.0+)?$")
                scale_match = re.search(r"(?m)^      Scale: ([0-9.eE+-]+)$", text)
                self.assertIsNotNone(scale_match)
                self.assertEqual(float(scale_match.group(1)), 20.0)
            finally:
                generated.unlink(missing_ok=True)

    def test_acceptance_history_bounds_and_frame_override(self):
        module = load_review_launch()
        with tempfile.TemporaryDirectory(prefix="lmmg_review_result_") as directory:
            output = Path(directory)
            report = output / "acceptance.json"
            report.write_text(
                json.dumps(
                    {
                        "stop_line": {
                            "odometry_history": [
                                {"x": 100.0, "y": -50.0},
                                {"x": 124.0, "y": -48.0},
                            ]
                        }
                    }
                ),
                encoding="utf-8",
            )
            bounds = module._acceptance_history_bounds(report)
            self.assertEqual(bounds, (95.0, 129.0, -55.0, -43.0))
            generated = Path(
                module._auto_fit_rviz_config(
                    PACKAGE_ROOT / "rviz" / "review.rviz",
                    output,
                    preferred_bounds=bounds,
                    frame_id="odom_precision",
                )
            )
            try:
                text = generated.read_text(encoding="utf-8")
                self.assertRegex(text, r"(?m)^      X: 112(?:\.0+)?$")
                self.assertRegex(text, r"(?m)^      Y: -49(?:\.0+)?$")
                self.assertIn("    Fixed Frame: odom_precision", text)
            finally:
                generated.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
