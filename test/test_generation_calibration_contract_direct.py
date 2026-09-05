#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest


CONTRACT_TOOL = Path(sys.argv[1]).resolve()
PUBLIC_WORKFLOW = Path(sys.argv[2]).resolve()
sys.path.insert(0, str(CONTRACT_TOOL.parent))
spec = importlib.util.spec_from_file_location("generation_contract_direct", CONTRACT_TOOL)
assert spec is not None and spec.loader is not None
contract_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract_tool)
sys.argv = [sys.argv[0]]


VEHICLE_YAML = """
/**:
  ros__parameters:
    wheel_radius: 0.3
    wheel_width: 0.2
    wheel_base: 1.0
    wheel_tread: 1.0
    front_overhang: 0.5
    rear_overhang: 0.4
    left_overhang: 0.1
    right_overhang: 0.1
    vehicle_height: 1.2
    max_steer_angle: 0.4636476090008061
"""


class DirectCalibrationContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.output = self.root / "public_dataset"
        self.output.mkdir()
        self.params = self.root / "generator.params.yaml"
        self.params.write_text("node:\n  ros__parameters:\n    input.type: glim\n", encoding="utf-8")
        self.vehicle = self.output / "target_vehicle_info.param.yaml"
        self.vehicle.write_text(textwrap.dedent(VEHICLE_YAML), encoding="utf-8")
        self.vehicle_model = contract_tool.resolve(self.vehicle)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_report(self, input_lines: str, input_type: str) -> Path:
        derived = self.vehicle_model["derived_map_parameters"]
        report = self.output / "generation_report.yaml"
        input_block = textwrap.indent(textwrap.dedent(input_lines).strip(), "  ")
        report.write_text(
            f"""schema_version: 6
generation_completed: true
input_type: "{input_type}"
input:
{input_block}
extrinsics:
  source: "parameters"
  calibration_source: "catalog_estimated"
  calibration_confidence: "medium"
  verified: false
  translation: [0.77, 0, 1.694]
  quaternion_xyzw: [0, 0, -0.00523596, 0.99998629]
parameters:
  robot_profile: "custom"
  robot_base_reference: "rear_axle_ground_projection"
  robot_footprint_model: "rectangle"
  robot_width: {derived['width_m']!r}
  robot_front_extent: {derived['front_extent_m']!r}
  robot_rear_extent: {derived['rear_extent_m']!r}
  clearance_margin: 0
  robot_minimum_collision_height: 0
  robot_maximum_collision_height: {derived['vehicle_height_m']!r}
  robot_minimum_turning_radius: {derived['minimum_turning_radius_m']!r}
  robot_allow_in_place_rotation: false
  robot_allow_reverse_motion: true
  robot_dimensions_source: "catalog_estimated"
  robot_dimensions_confidence: "medium"
  robot_dimensions_verified: false
  free_space_evidence_mode: "combined"
  observed_trajectory_clearance_radius: 0
  trajectory_free_space_model: "footprint"
  trajectory_footprint_erosion_margin: 0
  nav2_free_space_verified: false
  lanelet2_physical_boundaries_verified: false
  lanelet2_terminal_localization_settling_verified: false
  output_map_type: "vector_map"
""",
            encoding="utf-8",
        )
        return report

    def create_contract(self, report: Path) -> dict:
        result = subprocess.run(
            [
                sys.executable,
                str(CONTRACT_TOOL),
                "create-direct",
                "--dataset",
                self.output.name,
                "--map-type",
                "vector_map",
                "--generation-report",
                str(report),
                "--generator-parameters",
                str(self.params),
                "--input-path-base",
                str(self.root),
                "--target-vehicle-info",
                str(self.vehicle),
                "--acquisition-vehicle-is-target",
                "true",
                "--output",
                str(self.output / "acquisition_vehicle_target_contract.json"),
                "--sha256-output",
                str(self.output / "acquisition_vehicle_target_contract.sha256"),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(
            (self.output / "acquisition_vehicle_target_contract.json").read_text(
                encoding="utf-8"
            )
        )

    def verify(
        self, *, verify_inputs: bool, output_format: str = "json"
    ) -> subprocess.CompletedProcess[str]:
        command = [
            sys.executable,
            str(CONTRACT_TOOL),
            "verify-direct",
            "--expected-dataset",
            self.output.name,
            "--expected-map-type",
            "vector_map",
            "--contract",
            str(self.output / "acquisition_vehicle_target_contract.json"),
            "--sha256",
            str(self.output / "acquisition_vehicle_target_contract.sha256"),
            "--generator-parameters",
            str(self.params),
            "--target-vehicle-info",
            str(self.vehicle),
            "--format",
            output_format,
        ]
        if verify_inputs:
            command.append("--verify-inputs")
        return subprocess.run(command, text=True, capture_output=True, check=False)

    def test_public_glim_contract_binds_params_and_rehashes_inputs(self) -> None:
        inputs = self.root / "inputs"
        inputs.mkdir()
        map_path = inputs / "map.ply"
        trajectory = inputs / "trajectory.tum"
        map_path.write_bytes(b"ply\nmap payload\n")
        trajectory.write_text("0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n", encoding="utf-8")
        report = self.write_report(
            'type: "glim"\nmap_path: "inputs/map.ply"\n'
            'trajectory_path: "inputs/trajectory.tum"\ntrajectory_frame: "sensor"',
            "glim",
        )
        document = self.create_contract(report)
        self.assertEqual(document["schema_version"], 2)
        self.assertEqual(document["generator_parameters"]["path"], str(self.params))
        self.assertEqual(set(document["input"]["files"]), {"map", "trajectory"})
        self.assertEqual(self.verify(verify_inputs=True).returncode, 0)

        self.params.write_text(self.params.read_text(encoding="utf-8") + "# tamper\n")
        lightweight = self.verify(verify_inputs=False)
        self.assertEqual(lightweight.returncode, 2)
        self.assertIn("parameter YAML", lightweight.stderr)

    def test_public_rosbag_binds_every_storage_file_in_name_order(self) -> None:
        bag = self.root / "bag"
        bag.mkdir()
        (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
        (bag / "z.db3").write_bytes(b"z storage")
        (bag / "a.mcap").write_bytes(b"a storage")
        (bag / "ignored.txt").write_text("not storage\n")
        report = self.write_report(
            'type: "rosbag2"\nbag_path: "bag"\npose_reference_frame: "base"',
            "rosbag2",
        )
        document = self.create_contract(report)
        files = document["input"]["files"]
        self.assertEqual(list(files), ["metadata", "storage_0000", "storage_0001"])
        self.assertEqual(Path(files["storage_0000"]["path"]).name, "a.mcap")
        self.assertEqual(Path(files["storage_0001"]["path"]).name, "z.db3")
        self.assertEqual(self.verify(verify_inputs=True).returncode, 0)

        (bag / "z.db3").write_bytes(b"changed storage")
        tampered = self.verify(verify_inputs=True)
        self.assertEqual(tampered.returncode, 2)
        self.assertIn("differs from contract", tampered.stderr)

    def test_tsv_restores_numeric_values_as_ros_doubles(self) -> None:
        inputs = self.root / "inputs"
        inputs.mkdir()
        (inputs / "map.ply").write_bytes(b"ply\nmap payload\n")
        (inputs / "trajectory.tum").write_text(
            "0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n", encoding="utf-8"
        )
        report = self.write_report(
            'type: "glim"\nmap_path: "inputs/map.ply"\n'
            'trajectory_path: "inputs/trajectory.tum"\ntrajectory_frame: "sensor"',
            "glim",
        )
        document = self.create_contract(report)
        # Preserve the actual regression shape: generation_report.yaml may
        # parse zero-valued array elements and numeric scalars as integers in
        # the stored contract.
        self.assertIsInstance(
            document["lidar_base_extrinsics"]["translation_xyz_m"][1], int
        )
        self.assertIsInstance(
            document["generation_controls"][
                "observed_trajectory_clearance_radius_m"
            ],
            int,
        )
        self.assertIsInstance(document["vehicle_model"]["clearance_margin_m"], int)

        restored = self.verify(verify_inputs=False, output_format="tsv")
        self.assertEqual(restored.returncode, 0, restored.stderr)
        values = dict(line.split("\t", 1) for line in restored.stdout.splitlines())
        self.assertEqual(
            values["EXTRINSICS_TRANSLATION"], "[0.77,0.0,1.694]"
        )
        self.assertEqual(
            values["EXTRINSICS_QUATERNION_XYZW"],
            "[0.0,0.0,-0.00523596,0.99998629]",
        )
        numeric_scalars = {
            "ROBOT_WIDTH": document["vehicle_model"]["width_m"],
            "FRONT_EXTENT": document["vehicle_model"]["front_extent_m"],
            "REAR_EXTENT": document["vehicle_model"]["rear_extent_m"],
            "CLEARANCE_MARGIN": document["vehicle_model"]["clearance_margin_m"],
            "COLLISION_MIN_HEIGHT": document["vehicle_model"][
                "minimum_collision_height_m"
            ],
            "COLLISION_MAX_HEIGHT": document["vehicle_model"][
                "maximum_collision_height_m"
            ],
            "MINIMUM_TURNING_RADIUS": document["vehicle_model"][
                "minimum_turning_radius_m"
            ],
            "TRAJECTORY_CLEARANCE_RADIUS": document["generation_controls"][
                "observed_trajectory_clearance_radius_m"
            ],
            "TRAJECTORY_FOOTPRINT_EROSION_MARGIN": document[
                "generation_controls"
            ]["trajectory_footprint_erosion_margin_m"],
        }
        for key, source_value in numeric_scalars.items():
            with self.subTest(key=key):
                self.assertEqual(values[key], repr(float(source_value)))
                self.assertIsInstance(json.loads(values[key]), float)

    def test_public_input_symlink_is_rejected(self) -> None:
        inputs = self.root / "inputs"
        inputs.mkdir()
        real_map = inputs / "real.ply"
        real_map.write_bytes(b"ply\n")
        (inputs / "map.ply").symlink_to(real_map)
        (inputs / "trajectory.tum").write_text(
            "0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n", encoding="utf-8"
        )
        report = self.write_report(
            'type: "glim"\nmap_path: "inputs/map.ply"\n'
            'trajectory_path: "inputs/trajectory.tum"\ntrajectory_frame: "sensor"',
            "glim",
        )
        result = subprocess.run(
            [
                sys.executable,
                str(CONTRACT_TOOL),
                "create-direct",
                "--dataset",
                self.output.name,
                "--map-type",
                "vector_map",
                "--generation-report",
                str(report),
                "--generator-parameters",
                str(self.params),
                "--input-path-base",
                str(self.root),
                "--target-vehicle-info",
                str(self.vehicle),
                "--acquisition-vehicle-is-target",
                "false",
                "--output",
                str(self.output / "contract.json"),
                "--sha256-output",
                str(self.output / "contract.sha256"),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("symlink", result.stderr)

    def test_fixed_schema_v1_remains_recognized(self) -> None:
        # Existing fixed-dataset contracts remain schema v1; direct mode is a
        # separate schema and does not reinterpret their fixed_input_lock.
        self.assertEqual(contract_tool.SCHEMA_VERSION, 1)
        self.assertEqual(contract_tool.DIRECT_SCHEMA_VERSION, 2)
        source = CONTRACT_TOOL.read_text(encoding="utf-8")
        self.assertIn('"fixed_input_lock"', source)
        self.assertIn("verify_fixed_dataset_input", source)

    def test_public_workflow_contract_order_and_shell_syntax(self) -> None:
        syntax = subprocess.run(
            ["bash", "-n", str(PUBLIC_WORKFLOW)],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(syntax.returncode, 0, syntax.stderr)
        text = PUBLIC_WORKFLOW.read_text(encoding="utf-8")
        self.assertIn("LMMG_ACQUISITION_VEHICLE_IS_TARGET", text)
        self.assertIn("route_body_passage_planning_report.json", text)
        self.assertIn("create-direct", text)
        self.assertIn("verify-direct", text)
        generate_body = text[text.index("lmmg_generate() {") :]
        self.assertLess(
            generate_body.index("lmmg_record_contract\n"),
            generate_body.index('"${lmmg_binding_verifier}"'),
        )
        self.assertIn("lmmg_verify_contract true", text)

    def test_one_click_marker_binds_selected_vector_map_source(self) -> None:
        def extract_function(path: Path, name: str) -> Path:
            source = path.read_text(encoding="utf-8")
            begin = source.index(f"{name}() {{")
            end = source.index("\n}\n", begin) + 3
            extracted = self.root / f"{name}.sh"
            extracted.write_text(source[begin:end], encoding="utf-8")
            return extracted

        def validate(
            function_file: Path,
            function_name: str,
            marker_text: str,
            selection_text: str | None,
        ) -> int:
            marker = self.root / f"{function_name}.marker"
            marker.write_text(marker_text, encoding="utf-8")
            selection = self.root / f"{function_name}.selection"
            if selection_text is None:
                selection.unlink(missing_ok=True)
            else:
                selection.write_text(selection_text, encoding="utf-8")
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1"; "$2" "$3" expected-session "$4"',
                    "marker-test",
                    str(function_file),
                    function_name,
                    str(marker),
                    str(selection),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            return result.returncode

        marker = (
            "LMMG_AUTOWARE_ONE_CLICK_EXPORT\t1\n"
            "SESSION\texpected-session\n"
            "SOURCE\tedited_topology\n"
            "GRAPH_FINGERPRINT\t0123456789abcdef\n"
        )
        selection = (
            "LMMG_VECTOR_MAP_SOURCE\t1\n"
            "SOURCE\tedited_topology\n"
            "FRAME\tmap\n"
            "GRAPH_FINGERPRINT\t0123456789abcdef\n"
        )
        functions = [
            (extract_function(PUBLIC_WORKFLOW, "lmmg_validate_marker"),
             "lmmg_validate_marker"),
        ]
        for function_file, function_name in functions:
            with self.subTest(function=function_name, case="matching_manual"):
                self.assertEqual(validate(function_file, function_name, marker, selection), 0)
            with self.subTest(function=function_name, case="missing_source"):
                self.assertNotEqual(
                    validate(
                        function_file,
                        function_name,
                        marker.replace("SOURCE\tedited_topology\n", ""),
                        selection,
                    ),
                    0,
                )
            with self.subTest(function=function_name, case="selection_mismatch"):
                self.assertNotEqual(
                    validate(
                        function_file,
                        function_name,
                        marker,
                        selection.replace("SOURCE\tedited_topology", "SOURCE\trecorded_trajectory"),
                    ),
                    0,
                )
            with self.subTest(function=function_name, case="manual_without_selection"):
                self.assertNotEqual(validate(function_file, function_name, marker, None), 0)
            with self.subTest(function=function_name, case="legacy_recorded_default"):
                self.assertEqual(
                    validate(
                        function_file,
                        function_name,
                        marker.replace("edited_topology", "recorded_trajectory"),
                        None,
                    ),
                    0,
                )


if __name__ == "__main__":
    unittest.main()
