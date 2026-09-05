#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


EXPECTED_HASHES = {
    "map.ply": "9d88e2f4e3a443e87455c3b2a676113704062b761d65a164945d88e1c2af77e3",
    "traj_lidar.txt": "39d638b30e883f3e1b1b239036fb4712f69de10f192a33c64865445f7f0d7a61",
    "FIXTURE.json": "61f6e0f1576efc75dbb48fcd8481d0a2b4ae8f4975c05bd32f5e5e811e2a1ecc",
}


class RosconDemoFixtureTest(unittest.TestCase):
    EXPECTED_SHA256 = {
        "map.ply": "9d88e2f4e3a443e87455c3b2a676113704062b761d65a164945d88e1c2af77e3",
        "traj_lidar.txt": "39d638b30e883f3e1b1b239036fb4712f69de10f192a33c64865445f7f0d7a61",
        "FIXTURE.json": "61f6e0f1576efc75dbb48fcd8481d0a2b4ae8f4975c05bd32f5e5e811e2a1ecc",
    }

    @classmethod
    def setUpClass(cls) -> None:
        if len(sys.argv) != 3:
            raise RuntimeError("expected fixture-generator and output-checker paths")
        cls.generator = Path(sys.argv[1]).resolve()
        cls.checker = Path(sys.argv[2]).resolve()
        del sys.argv[1:]

    @staticmethod
    def _hash(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def test_fixture_is_deterministic_and_location_free(self) -> None:
        with tempfile.TemporaryDirectory(prefix="lmmg_roscon_fixture_") as temporary:
            output = Path(temporary) / "fixture"
            command = [
                sys.executable,
                str(self.generator),
                "--output-directory",
                str(output),
            ]
            subprocess.run(command, check=True, capture_output=True, text=True)
            paths = [output / "map.ply", output / "traj_lidar.txt", output / "FIXTURE.json"]
            first_hashes = [self._hash(path) for path in paths]
            self.assertEqual(
                [EXPECTED_HASHES[path.name] for path in paths], first_hashes
            )
            self.assertEqual(
                first_hashes,
                [self.EXPECTED_SHA256[path.name] for path in paths],
            )

            refused = subprocess.run(command, check=False, capture_output=True, text=True)
            self.assertEqual(refused.returncode, 2)

            subprocess.run(command + ["--replace"], check=True, capture_output=True, text=True)
            self.assertEqual(first_hashes, [self._hash(path) for path in paths])

            manifest = json.loads((output / "FIXTURE.json").read_text(encoding="utf-8"))
            self.assertTrue(manifest["location_free"])
            self.assertFalse(manifest["contains_recorded_personal_or_facility_data"])
            self.assertEqual(manifest["point_count"], 38201)
            self.assertEqual(manifest["trajectory_pose_count"], 121)

    def test_checker_fails_closed_on_incomplete_output(self) -> None:
        with tempfile.TemporaryDirectory(prefix="lmmg_roscon_check_") as temporary:
            checked = subprocess.run(
                [sys.executable, str(self.checker), temporary],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(checked.returncode, 1)
            self.assertIn("ROSCON_DEMO_CHECK=FAIL", checked.stdout)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
