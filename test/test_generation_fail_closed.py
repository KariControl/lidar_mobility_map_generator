#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

EXECUTABLE = Path(sys.argv[1])
PARAMS_FILE = Path(sys.argv[2])


class GenerationFailClosedTest(unittest.TestCase):
    def test_input_failure_invalidates_prior_readiness(self):
        with tempfile.TemporaryDirectory(prefix="lmmg_generation_failure_") as temporary:
            root = Path(temporary)
            output = root / "output"
            output.mkdir()
            (output / "navigation_target_readiness.yaml").write_text(
                "schema_version: 3\n"
                "generation_complete: true\n"
                "requested_target_mode: \"both\"\n"
                "nav2:\n  enabled: true\n  navigation_ready: true\n"
                "autoware:\n  enabled: true\n  navigation_ready: true\n",
                encoding="utf-8",
            )
            environment = os.environ.copy()
            environment["ROS_LOG_DIR"] = str(root / "ros_logs")
            (root / "ros_logs").mkdir()
            completed = subprocess.run(
                [
                    str(EXECUTABLE),
                    "--ros-args",
                    "--params-file",
                    str(PARAMS_FILE),
                    "-p",
                    f"input.glim.map_path:={root / 'missing.ply'}",
                    "-p",
                    f"input.glim.trajectory_path:={root / 'missing.tum'}",
                    "-p",
                    f"output.directory:={output}",
                ],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=20,
                env=environment,
            )
            self.assertNotEqual(completed.returncode, 0, completed.stdout)
            readiness = (output / "navigation_target_readiness.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn("generation_complete: false", readiness, completed.stdout)
            self.assertIn('reasons: ["generation_in_progress"]', readiness)
            self.assertNotIn("navigation_ready: true", readiness)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
