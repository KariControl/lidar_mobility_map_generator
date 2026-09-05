#!/usr/bin/env python3
"""Create and verify the map-generation calibration/vehicle contract.

The contract records the exact fixed input dataset, target vehicle file, body
geometry, LiDAR-to-body transform, and evidence claims that reached the map
generator.  A GUI regeneration restores these values only after verifying the
contract hash and all bindings.  Presence of numeric values never promotes an
unverified or inferred claim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import tempfile
from typing import Mapping

from resolve_autoware_vehicle_info import VehicleInfoError, resolve

try:
    import map_ws_provenance_contract as fixed_provenance
except ModuleNotFoundError as error:
    if error.name != "map_ws_provenance_contract":
        raise
    fixed_provenance = None


KIND = "lmmg_generation_calibration_vehicle_contract"
SCHEMA_VERSION = 1
DIRECT_SCHEMA_VERSION = 2
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
EVIDENCE_SOURCES = {"unknown", "measured", "catalog_estimated", "inferred"}
EVIDENCE_CONFIDENCES = {"unknown", "low", "medium", "high"}
MAP_TYPES = {"vector_map", "navigation_map", "both"}


class CalibrationContractError(ValueError):
    pass


class FixedProvenanceUnavailableError(ValueError):
    """Stable exception type when local fixed-campaign support is absent."""


FixedProvenanceError = (
    fixed_provenance.ContractError
    if fixed_provenance is not None
    else FixedProvenanceUnavailableError
)


def _fail(message: str) -> None:
    raise CalibrationContractError(message)


def _require_fixed_provenance():
    if fixed_provenance is None:
        raise FixedProvenanceUnavailableError(
            "fixed map_ws campaign support is not included in the public release"
        )
    return fixed_provenance


def _sha256_file(path: Path) -> str:
    """Hash a stable regular file without following a symlink."""

    try:
        before = path.lstat()
    except OSError as error:
        _fail(f"could not inspect file for hashing: {path}: {error}")
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        _fail(f"file to hash must be a regular non-symlink file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
        after = path.lstat()
    except OSError as error:
        _fail(f"could not hash file: {path}: {error}")
    identity_before = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    identity_after = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
    if identity_before != identity_after:
        _fail(f"file changed while being hashed: {path}")
    return digest.hexdigest()


def _parse_yaml_scalar(text: str) -> object:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    if text in {"true", "True"}:
        return True
    if text in {"false", "False"}:
        return False
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text.strip("\"'")


def _load_yaml_scalars(path: Path) -> dict[tuple[str, ...], object]:
    """Read the scalar subset used by generation_report.yaml."""

    result: dict[tuple[str, ...], object] = {}
    stack: list[tuple[int, str]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        match = re.match(r"^\s*([^:#][^:]*):(?:\s*(.*))?$", raw)
        if match is None:
            continue
        while stack and stack[-1][0] >= indent:
            stack.pop()
        key = match.group(1).strip()
        value = (match.group(2) or "").strip()
        nested = tuple(item[1] for item in stack) + (key,)
        if value:
            result[nested] = _parse_yaml_scalar(value)
        else:
            stack.append((indent, key))
    return result


def _exact_keys(value: object, expected: set[str], context: str) -> dict:
    if not isinstance(value, dict):
        _fail(f"{context} must be an object")
    actual = set(value)
    if actual != expected:
        _fail(
            f"{context} keys differ: missing={sorted(expected - actual)!r}, "
            f"unexpected={sorted(actual - expected)!r}"
        )
    return value


def _string(value: object, context: str) -> str:
    if not isinstance(value, str) or not value or "\n" in value or "\r" in value:
        _fail(f"{context} must be a non-empty single-line string")
    return value


def _boolean(value: object, context: str) -> bool:
    if not isinstance(value, bool):
        _fail(f"{context} must be boolean")
    return value


def _finite(value: object, context: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(f"{context} must be numeric")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "positive and " if positive else ""
        _fail(f"{context} must be {qualifier}finite")
    return result


def _sha256(value: object, context: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        _fail(f"{context} must be a lowercase SHA-256")
    return value


def _canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _scalar(scalars: Mapping[tuple[str, ...], object], path: tuple[str, ...]) -> object:
    try:
        return scalars[path]
    except KeyError:
        _fail(f"generation report is missing {'.'.join(path)}")


def _report_vehicle(scalars: Mapping[tuple[str, ...], object]) -> dict:
    parameters = ("parameters",)
    return {
        "profile": _scalar(scalars, parameters + ("robot_profile",)),
        "base_reference": _scalar(scalars, parameters + ("robot_base_reference",)),
        "footprint_model": _scalar(scalars, parameters + ("robot_footprint_model",)),
        "width_m": _scalar(scalars, parameters + ("robot_width",)),
        "front_extent_m": _scalar(scalars, parameters + ("robot_front_extent",)),
        "rear_extent_m": _scalar(scalars, parameters + ("robot_rear_extent",)),
        "clearance_margin_m": _scalar(scalars, parameters + ("clearance_margin",)),
        "minimum_collision_height_m": _scalar(
            scalars, parameters + ("robot_minimum_collision_height",)
        ),
        "maximum_collision_height_m": _scalar(
            scalars, parameters + ("robot_maximum_collision_height",)
        ),
        "minimum_turning_radius_m": _scalar(
            scalars, parameters + ("robot_minimum_turning_radius",)
        ),
        "allow_in_place_rotation": _scalar(
            scalars, parameters + ("robot_allow_in_place_rotation",)
        ),
        "allow_reverse_motion": _scalar(
            scalars, parameters + ("robot_allow_reverse_motion",)
        ),
        "dimensions_source": _scalar(
            scalars, parameters + ("robot_dimensions_source",)
        ),
        "dimensions_confidence": _scalar(
            scalars, parameters + ("robot_dimensions_confidence",)
        ),
        "dimensions_verified": _scalar(
            scalars, parameters + ("robot_dimensions_verified",)
        ),
    }


def _report_extrinsics(scalars: Mapping[tuple[str, ...], object]) -> dict:
    return {
        "source": _scalar(scalars, ("extrinsics", "source")),
        "calibration_source": _scalar(
            scalars, ("extrinsics", "calibration_source")
        ),
        "calibration_confidence": _scalar(
            scalars, ("extrinsics", "calibration_confidence")
        ),
        "verified": _scalar(scalars, ("extrinsics", "verified")),
        "translation_xyz_m": _scalar(scalars, ("extrinsics", "translation")),
        "quaternion_xyzw": _scalar(
            scalars, ("extrinsics", "quaternion_xyzw")
        ),
    }


def _report_generation_controls(
    scalars: Mapping[tuple[str, ...], object], input_type: str
) -> dict:
    reference_path = (
        ("input", "trajectory_frame")
        if input_type == "glim"
        else ("input", "pose_reference_frame")
    )
    parameters = ("parameters",)
    return {
        "pose_reference_frame": _scalar(scalars, reference_path),
        "free_space_evidence_mode": _scalar(
            scalars, parameters + ("free_space_evidence_mode",)
        ),
        "observed_trajectory_clearance_radius_m": _scalar(
            scalars, parameters + ("observed_trajectory_clearance_radius",)
        ),
        "trajectory_free_space_model": _scalar(
            scalars, parameters + ("trajectory_free_space_model",)
        ),
        "trajectory_footprint_erosion_margin_m": _scalar(
            scalars, parameters + ("trajectory_footprint_erosion_margin",)
        ),
        "nav2_free_space_verified": _scalar(
            scalars, parameters + ("nav2_free_space_verified",)
        ),
        "lanelet2_physical_boundaries_verified": _scalar(
            scalars, parameters + ("lanelet2_physical_boundaries_verified",)
        ),
        "terminal_localization_settling_verified": _scalar(
            scalars,
            parameters + ("lanelet2_terminal_localization_settling_verified",),
        ),
    }


def _public_input_binding(evidence: dict) -> dict:
    files = {}
    for role, item in sorted(evidence["files"].items()):
        files[role] = {
            "path": item["path"],
            "size_bytes": item["size_bytes"],
            "sha256": item["sha256"],
        }
    binding = {
        "dataset": evidence["dataset"],
        "input_type": evidence["input_type"],
        "files": files,
    }
    return {**binding, "binding_sha256": _canonical_sha256(binding)}


def _absolute_path(path: Path, base: Path | None = None) -> Path:
    if path.is_absolute():
        return Path(os.path.abspath(os.fspath(path)))
    origin = base if base is not None else Path.cwd()
    return Path(os.path.abspath(os.fspath(origin / path)))


def _reject_symlink_components(path: Path, context: str) -> None:
    """Reject a direct/public input reached through any symlink component."""

    absolute = _absolute_path(path)
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        try:
            metadata = current.lstat()
        except OSError as error:
            _fail(f"{context} is missing: {current}: {error}")
        if stat.S_ISLNK(metadata.st_mode):
            _fail(f"{context} must not contain a symlink: {current}")


def _direct_regular_file(path: Path, context: str) -> dict:
    absolute = _absolute_path(path)
    _reject_symlink_components(absolute, context)
    try:
        metadata = absolute.lstat()
    except OSError as error:
        _fail(f"{context} is missing: {absolute}: {error}")
    if not stat.S_ISREG(metadata.st_mode):
        _fail(f"{context} must be a regular file: {absolute}")
    digest = _sha256_file(absolute)
    # sha256_file verifies identity before and after hashing.  Read the size
    # only after that stable hash has completed.
    size = absolute.lstat().st_size
    if size <= 0:
        _fail(f"{context} must not be empty: {absolute}")
    return {"path": str(absolute), "size_bytes": size, "sha256": digest}


def _direct_source_path(raw: object, base: Path, context: str) -> Path:
    text = _string(raw, context)
    return _absolute_path(Path(text), base)


def direct_input_evidence(
    report_scalars: Mapping[tuple[str, ...], object],
    dataset: str,
    path_resolution_base: Path,
) -> dict:
    """Hash every direct GLIM/rosbag2 source named by generation_report.yaml."""

    base = _absolute_path(path_resolution_base)
    _reject_symlink_components(base, "input path-resolution directory")
    try:
        metadata = base.lstat()
    except OSError as error:
        _fail(f"input path-resolution directory is missing: {base}: {error}")
    if not stat.S_ISDIR(metadata.st_mode):
        _fail(f"input path-resolution base is not a directory: {base}")

    input_type = _string(_scalar(report_scalars, ("input_type",)), "input_type")
    files: dict[str, dict]
    if input_type == "glim":
        map_path = _direct_source_path(
            _scalar(report_scalars, ("input", "map_path")), base, "input.map_path"
        )
        trajectory_path = _direct_source_path(
            _scalar(report_scalars, ("input", "trajectory_path")),
            base,
            "input.trajectory_path",
        )
        files = {
            "map": _direct_regular_file(map_path, "GLIM map"),
            "trajectory": _direct_regular_file(
                trajectory_path, "GLIM trajectory"
            ),
        }
    elif input_type == "rosbag2":
        bag_path = _direct_source_path(
            _scalar(report_scalars, ("input", "bag_path")), base, "input.bag_path"
        )
        _reject_symlink_components(bag_path, "rosbag2 directory")
        try:
            bag_metadata = bag_path.lstat()
        except OSError as error:
            _fail(f"rosbag2 directory is missing: {bag_path}: {error}")
        if not stat.S_ISDIR(bag_metadata.st_mode):
            _fail(f"rosbag2 input is not a directory: {bag_path}")
        metadata_path = bag_path / "metadata.yaml"
        storage_paths = sorted(
            (
                item
                for item in bag_path.iterdir()
                if item.name != "metadata.yaml"
                and item.suffix.lower() in {".mcap", ".db3"}
            ),
            key=lambda item: item.name,
        )
        if not storage_paths:
            _fail(f"rosbag2 directory has no .mcap or .db3 storage file: {bag_path}")
        files = {"metadata": _direct_regular_file(metadata_path, "rosbag2 metadata")}
        for index, storage_path in enumerate(storage_paths):
            files[f"storage_{index:04d}"] = _direct_regular_file(
                storage_path, "rosbag2 storage"
            )
    else:
        _fail(f"direct/public input type must be glim or rosbag2, found {input_type!r}")
    return {
        "dataset": dataset,
        "input_type": input_type,
        "files": files,
        "verified": True,
    }


def _direct_input_binding(evidence: dict, path_resolution_base: Path) -> dict:
    files = {
        role: {
            "path": item["path"],
            "size_bytes": item["size_bytes"],
            "sha256": item["sha256"],
        }
        for role, item in sorted(evidence["files"].items())
    }
    binding = {
        "source_mode": "direct_generation_report",
        "path_resolution_base": str(_absolute_path(path_resolution_base)),
        "dataset": evidence["dataset"],
        "input_type": evidence["input_type"],
        "files": files,
    }
    return {**binding, "binding_sha256": _canonical_sha256(binding)}


def _assert_report_input_matches_evidence(
    scalars: Mapping[tuple[str, ...], object],
    evidence: dict,
    path_resolution_base: Path | None = None,
) -> None:
    def reported_path(value: object, context: str) -> Path:
        return _absolute_path(Path(_string(value, context)), path_resolution_base)

    if evidence["input_type"] == "glim":
        expected = {
            "map": Path(evidence["files"]["map"]["path"]).resolve(strict=True),
            "trajectory": Path(
                evidence["files"]["trajectory"]["path"]
            ).resolve(strict=True),
        }
        reported = {
            "map": reported_path(
                _scalar(scalars, ("input", "map_path")), "input.map_path"
            ).resolve(strict=True),
            "trajectory": reported_path(
                _scalar(scalars, ("input", "trajectory_path")),
                "input.trajectory_path",
            ).resolve(strict=True),
        }
        if reported != expected:
            _fail("generation report GLIM paths differ from the fixed input binding")
        return
    bag = reported_path(
        _scalar(scalars, ("input", "bag_path")), "input.bag_path"
    ).resolve(strict=True)
    bound_parents = {
        Path(item["path"]).resolve(strict=True).parent
        for item in evidence["files"].values()
    }
    if bound_parents != {bag}:
        _fail("generation report rosbag path differs from the fixed input binding")


def _vehicle_binding(vehicle: dict, stored_filename: str) -> dict:
    binding = {
        "stored_file": stored_filename,
        "sha256": vehicle["source"]["sha256"],
        "base_reference": vehicle["base_reference"],
        "footprint_model": vehicle["footprint_model"],
        "derived_map_parameters": vehicle["derived_map_parameters"],
    }
    return {**binding, "binding_sha256": _canonical_sha256(binding)}


def _same_number(left: object, right: object) -> bool:
    return math.isclose(
        _finite(left, "left comparison value"),
        _finite(right, "right comparison value"),
        rel_tol=1.0e-9,
        abs_tol=1.0e-9,
    )


def _assert_vehicle_matches_report(vehicle: dict, report_vehicle: dict) -> None:
    derived = vehicle["derived_map_parameters"]
    comparisons = {
        "width_m": "width_m",
        "front_extent_m": "front_extent_m",
        "rear_extent_m": "rear_extent_m",
        "minimum_turning_radius_m": "minimum_turning_radius_m",
        "vehicle_height_m": "maximum_collision_height_m",
    }
    for vehicle_key, report_key in comparisons.items():
        if not _same_number(derived[vehicle_key], report_vehicle[report_key]):
            _fail(
                f"target vehicle {vehicle_key} does not match generation report "
                f"{report_key}"
            )
    if report_vehicle["base_reference"] != vehicle["base_reference"]:
        _fail("target vehicle base reference does not match generation report")
    if report_vehicle["footprint_model"] != vehicle["footprint_model"]:
        _fail("target vehicle footprint model does not match generation report")


def build_contract(
    *,
    dataset: str,
    map_type: str,
    generation_report: Path,
    report_scalars: Mapping[tuple[str, ...], object],
    input_evidence: dict,
    input_lock_path: Path,
    target_vehicle: dict | None,
    target_vehicle_filename: str | None,
    acquisition_vehicle_is_target: bool,
) -> dict:
    if map_type not in MAP_TYPES:
        _fail(f"unsupported map type: {map_type}")
    report_map_type = _scalar(report_scalars, ("parameters", "output_map_type"))
    if report_map_type != map_type:
        _fail(
            f"generation report map type differs: expected={map_type}, "
            f"actual={report_map_type}"
        )
    input_type = _string(_scalar(report_scalars, ("input_type",)), "input_type")
    if input_type != input_evidence.get("input_type"):
        _fail("generation report input type differs from fixed input verification")
    if dataset != input_evidence.get("dataset") or input_evidence.get("verified") is not True:
        _fail("fixed input verification does not prove the selected dataset")
    _assert_report_input_matches_evidence(report_scalars, input_evidence)

    vehicle_model = _report_vehicle(report_scalars)
    target_binding = None
    if map_type in {"vector_map", "both"}:
        if target_vehicle is None or target_vehicle_filename is None:
            _fail("Vector Map contract requires target_vehicle_info.param.yaml")
        _assert_vehicle_matches_report(target_vehicle, vehicle_model)
        target_binding = _vehicle_binding(target_vehicle, target_vehicle_filename)

    report_path = generation_report.resolve(strict=True)
    lock_path = input_lock_path.resolve(strict=True)
    contract = {
        "schema_version": SCHEMA_VERSION,
        "kind": KIND,
        "dataset": dataset,
        "map_type": map_type,
        "target_vehicle_info_sha256": (
            target_binding["sha256"] if target_binding is not None else "none"
        ),
        "generation_report": {
            "stored_file": generation_report.name,
            "sha256": _sha256_file(report_path),
        },
        "input": {
            "fixed_input_lock": {
                "path": str(lock_path),
                "sha256": _sha256_file(lock_path),
            },
            **_public_input_binding(input_evidence),
        },
        "target_vehicle": target_binding,
        "vehicle_model": vehicle_model,
        "lidar_base_extrinsics": _report_extrinsics(report_scalars),
        "generation_controls": _report_generation_controls(
            report_scalars, input_type
        ),
        "acquisition_vehicle_is_target": acquisition_vehicle_is_target,
    }
    validate_contract_document(contract)
    return contract


def build_direct_contract(
    *,
    dataset: str,
    map_type: str,
    generation_report: Path,
    report_scalars: Mapping[tuple[str, ...], object],
    input_evidence: dict,
    path_resolution_base: Path,
    generator_parameters_path: Path,
    target_vehicle: dict | None,
    target_vehicle_filename: str | None,
    acquisition_vehicle_is_target: bool,
) -> dict:
    """Build the schema-v2 contract used by the public arbitrary-input helper."""

    if map_type not in MAP_TYPES:
        _fail(f"unsupported map type: {map_type}")
    report_map_type = _scalar(report_scalars, ("parameters", "output_map_type"))
    if report_map_type != map_type:
        _fail(
            f"generation report map type differs: expected={map_type}, "
            f"actual={report_map_type}"
        )
    input_type = _string(_scalar(report_scalars, ("input_type",)), "input_type")
    if input_type != input_evidence.get("input_type"):
        _fail("generation report input type differs from direct input verification")
    if dataset != input_evidence.get("dataset") or input_evidence.get("verified") is not True:
        _fail("direct input verification does not prove the selected dataset")
    _assert_report_input_matches_evidence(
        report_scalars, input_evidence, path_resolution_base
    )

    vehicle_model = _report_vehicle(report_scalars)
    target_binding = None
    if map_type in {"vector_map", "both"}:
        if target_vehicle is None or target_vehicle_filename is None:
            _fail("Vector Map contract requires target_vehicle_info.param.yaml")
        _assert_vehicle_matches_report(target_vehicle, vehicle_model)
        target_binding = _vehicle_binding(target_vehicle, target_vehicle_filename)

    report_path = _absolute_path(generation_report)
    parameters_binding = _direct_regular_file(
        generator_parameters_path, "generator parameter YAML"
    )
    contract = {
        "schema_version": DIRECT_SCHEMA_VERSION,
        "kind": KIND,
        "dataset": dataset,
        "map_type": map_type,
        "target_vehicle_info_sha256": (
            target_binding["sha256"] if target_binding is not None else "none"
        ),
        "generation_report": {
            "stored_file": generation_report.name,
            "sha256": _sha256_file(report_path),
        },
        "generator_parameters": parameters_binding,
        "input": _direct_input_binding(input_evidence, path_resolution_base),
        "target_vehicle": target_binding,
        "vehicle_model": vehicle_model,
        "lidar_base_extrinsics": _report_extrinsics(report_scalars),
        "generation_controls": _report_generation_controls(
            report_scalars, input_type
        ),
        "acquisition_vehicle_is_target": acquisition_vehicle_is_target,
    }
    validate_contract_document(contract)
    return contract


def _validate_evidence(
    source: object, confidence: object, verified: object, context: str
) -> None:
    if source not in EVIDENCE_SOURCES:
        _fail(f"{context} source is invalid")
    if confidence not in EVIDENCE_CONFIDENCES:
        _fail(f"{context} confidence is invalid")
    if (source == "unknown") != (confidence == "unknown"):
        _fail(f"{context} source/confidence must both be unknown or both known")
    is_verified = _boolean(verified, f"{context}.verified")
    if is_verified and (source != "measured" or confidence != "high"):
        _fail(f"{context} verified=true requires measured/high evidence")


def validate_contract_document(value: object) -> dict:
    if not isinstance(value, dict):
        _fail("contract must be an object")
    schema_version = value.get("schema_version")
    common_keys = {
        "schema_version",
        "kind",
        "dataset",
        "map_type",
        "target_vehicle_info_sha256",
        "generation_report",
        "input",
        "target_vehicle",
        "vehicle_model",
        "lidar_base_extrinsics",
        "generation_controls",
        "acquisition_vehicle_is_target",
    }
    if schema_version == SCHEMA_VERSION:
        expected_root_keys = common_keys
    elif schema_version == DIRECT_SCHEMA_VERSION:
        expected_root_keys = common_keys | {"generator_parameters"}
    else:
        _fail("unsupported calibration contract schema or kind")
    root = _exact_keys(
        value,
        expected_root_keys,
        "contract",
    )
    if root["kind"] != KIND:
        _fail("unsupported calibration contract schema or kind")
    _string(root["dataset"], "dataset")
    if root["map_type"] not in MAP_TYPES:
        _fail("map_type is invalid")
    _boolean(root["acquisition_vehicle_is_target"], "acquisition_vehicle_is_target")

    report = _exact_keys(root["generation_report"], {"stored_file", "sha256"}, "report")
    if report["stored_file"] != "generation_report.yaml":
        _fail("generation report filename is not canonical")
    _sha256(report["sha256"], "generation_report.sha256")

    if schema_version == SCHEMA_VERSION:
        input_value = _exact_keys(
            root["input"],
            {
                "fixed_input_lock",
                "dataset",
                "input_type",
                "files",
                "binding_sha256",
            },
            "input",
        )
    else:
        parameters = _exact_keys(
            root["generator_parameters"],
            {"path", "size_bytes", "sha256"},
            "generator_parameters",
        )
        parameters_path = _string(parameters["path"], "generator_parameters.path")
        if not Path(parameters_path).is_absolute():
            _fail("generator_parameters.path must be absolute")
        if (
            not isinstance(parameters["size_bytes"], int)
            or isinstance(parameters["size_bytes"], bool)
            or parameters["size_bytes"] <= 0
        ):
            _fail("generator_parameters.size_bytes must be a positive integer")
        _sha256(parameters["sha256"], "generator_parameters.sha256")
        input_value = _exact_keys(
            root["input"],
            {
                "source_mode",
                "path_resolution_base",
                "dataset",
                "input_type",
                "files",
                "binding_sha256",
            },
            "input",
        )
        if input_value["source_mode"] != "direct_generation_report":
            _fail("direct input source_mode is invalid")
        resolution_base = _string(
            input_value["path_resolution_base"], "input.path_resolution_base"
        )
        if not Path(resolution_base).is_absolute():
            _fail("input.path_resolution_base must be absolute")
    if input_value["dataset"] != root["dataset"]:
        _fail("input dataset differs from contract dataset")
    if input_value["input_type"] not in {"glim", "rosbag2"}:
        _fail("input type is invalid")
    if schema_version == SCHEMA_VERSION:
        lock = _exact_keys(
            input_value["fixed_input_lock"], {"path", "sha256"}, "fixed_input_lock"
        )
        _string(lock["path"], "fixed_input_lock.path")
        _sha256(lock["sha256"], "fixed_input_lock.sha256")
    files = input_value["files"]
    if not isinstance(files, dict) or not files:
        _fail("input.files must be a non-empty object")
    for role, item in files.items():
        _string(role, "input file role")
        document = _exact_keys(item, {"path", "size_bytes", "sha256"}, f"input.{role}")
        _string(document["path"], f"input.{role}.path")
        if not isinstance(document["size_bytes"], int) or document["size_bytes"] <= 0:
            _fail(f"input.{role}.size_bytes must be a positive integer")
        _sha256(document["sha256"], f"input.{role}.sha256")
        if schema_version == DIRECT_SCHEMA_VERSION and not Path(
            document["path"]
        ).is_absolute():
            _fail(f"input.{role}.path must be absolute")
    if schema_version == SCHEMA_VERSION:
        input_binding = {
            "dataset": input_value["dataset"],
            "input_type": input_value["input_type"],
            "files": files,
        }
    else:
        if input_value["input_type"] == "glim":
            if set(files) != {"map", "trajectory"}:
                _fail("direct GLIM input must bind exactly map and trajectory")
        else:
            storage_roles = sorted(role for role in files if role.startswith("storage_"))
            if not storage_roles:
                _fail("direct rosbag2 input must bind at least one storage file")
            expected_storage_roles = [
                f"storage_{index:04d}" for index in range(len(storage_roles))
            ]
            if set(files) != {"metadata", *expected_storage_roles}:
                _fail("direct rosbag2 input roles are not canonical")
            if Path(files["metadata"]["path"]).name != "metadata.yaml":
                _fail("direct rosbag2 metadata filename is not canonical")
            storage_names = [Path(files[role]["path"]).name for role in storage_roles]
            if storage_names != sorted(storage_names) or any(
                Path(name).suffix.lower() not in {".mcap", ".db3"}
                for name in storage_names
            ):
                _fail("direct rosbag2 storage order or suffix is invalid")
        input_binding = {
            "source_mode": input_value["source_mode"],
            "path_resolution_base": input_value["path_resolution_base"],
            "dataset": input_value["dataset"],
            "input_type": input_value["input_type"],
            "files": files,
        }
    if _sha256(input_value["binding_sha256"], "input.binding_sha256") != _canonical_sha256(
        input_binding
    ):
        _fail("input binding SHA-256 mismatch")

    vehicle = _exact_keys(
        root["vehicle_model"],
        {
            "profile",
            "base_reference",
            "footprint_model",
            "width_m",
            "front_extent_m",
            "rear_extent_m",
            "clearance_margin_m",
            "minimum_collision_height_m",
            "maximum_collision_height_m",
            "minimum_turning_radius_m",
            "allow_in_place_rotation",
            "allow_reverse_motion",
            "dimensions_source",
            "dimensions_confidence",
            "dimensions_verified",
        },
        "vehicle_model",
    )
    for key in ("profile", "base_reference", "footprint_model"):
        _string(vehicle[key], f"vehicle_model.{key}")
    for key in ("width_m", "front_extent_m", "rear_extent_m"):
        _finite(vehicle[key], f"vehicle_model.{key}", positive=True)
    for key in (
        "clearance_margin_m",
        "minimum_collision_height_m",
        "minimum_turning_radius_m",
    ):
        if _finite(vehicle[key], f"vehicle_model.{key}") < 0.0:
            _fail(f"vehicle_model.{key} must be nonnegative")
    if _finite(
        vehicle["maximum_collision_height_m"],
        "vehicle_model.maximum_collision_height_m",
    ) <= _finite(
        vehicle["minimum_collision_height_m"],
        "vehicle_model.minimum_collision_height_m",
    ):
        _fail("vehicle collision-height interval is invalid")
    _boolean(vehicle["allow_in_place_rotation"], "allow_in_place_rotation")
    _boolean(vehicle["allow_reverse_motion"], "allow_reverse_motion")
    _validate_evidence(
        vehicle["dimensions_source"],
        vehicle["dimensions_confidence"],
        vehicle["dimensions_verified"],
        "vehicle dimensions",
    )

    extrinsics = _exact_keys(
        root["lidar_base_extrinsics"],
        {
            "source",
            "calibration_source",
            "calibration_confidence",
            "verified",
            "translation_xyz_m",
            "quaternion_xyzw",
        },
        "lidar_base_extrinsics",
    )
    if schema_version == SCHEMA_VERSION and extrinsics["source"] != "parameters":
        _fail("dataset helper calibration contract requires parameter extrinsics")
    if schema_version == DIRECT_SCHEMA_VERSION and extrinsics["source"] not in {
        "parameters",
        "tf_static",
    }:
        _fail("direct/public calibration contract has an invalid extrinsics source")
    _validate_evidence(
        extrinsics["calibration_source"],
        extrinsics["calibration_confidence"],
        extrinsics["verified"],
        "LiDAR extrinsics",
    )
    translation = extrinsics["translation_xyz_m"]
    quaternion = extrinsics["quaternion_xyzw"]
    if not isinstance(translation, list) or len(translation) != 3:
        _fail("extrinsics translation requires three values")
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        _fail("extrinsics quaternion requires four values")
    translation_values = [
        _finite(item, "extrinsics translation value") for item in translation
    ]
    quaternion_values = [
        _finite(item, "extrinsics quaternion value") for item in quaternion
    ]
    del translation_values
    if sum(item * item for item in quaternion_values) <= 1.0e-15:
        _fail("extrinsics quaternion must be nonzero")
    if vehicle["dimensions_verified"] and not extrinsics["verified"]:
        _fail("verified vehicle dimensions require verified LiDAR extrinsics")

    controls = _exact_keys(
        root["generation_controls"],
        {
            "pose_reference_frame",
            "free_space_evidence_mode",
            "observed_trajectory_clearance_radius_m",
            "trajectory_free_space_model",
            "trajectory_footprint_erosion_margin_m",
            "nav2_free_space_verified",
            "lanelet2_physical_boundaries_verified",
            "terminal_localization_settling_verified",
        },
        "generation_controls",
    )
    if controls["pose_reference_frame"] not in {"sensor", "base"}:
        _fail("pose reference frame is invalid")
    _string(controls["free_space_evidence_mode"], "free_space_evidence_mode")
    _string(controls["trajectory_free_space_model"], "trajectory_free_space_model")
    if _finite(
        controls["observed_trajectory_clearance_radius_m"],
        "observed trajectory clearance radius",
    ) < 0.0:
        _fail("observed trajectory clearance radius must be nonnegative")
    if _finite(
        controls["trajectory_footprint_erosion_margin_m"],
        "trajectory footprint erosion margin",
    ) < 0.0:
        _fail("trajectory footprint erosion margin must be nonnegative")
    for key in (
        "nav2_free_space_verified",
        "lanelet2_physical_boundaries_verified",
        "terminal_localization_settling_verified",
    ):
        _boolean(controls[key], f"generation_controls.{key}")
    if (
        controls["nav2_free_space_verified"]
        or controls["lanelet2_physical_boundaries_verified"]
    ) and not (vehicle["dimensions_verified"] and extrinsics["verified"]):
        _fail("planner-facing verification requires verified dimensions and extrinsics")

    target = root["target_vehicle"]
    if root["map_type"] in {"vector_map", "both"}:
        target = _exact_keys(
            target,
            {
                "stored_file",
                "sha256",
                "base_reference",
                "footprint_model",
                "derived_map_parameters",
                "binding_sha256",
            },
            "target_vehicle",
        )
        if target["stored_file"] != "target_vehicle_info.param.yaml":
            _fail("target vehicle filename is not canonical")
        _sha256(target["sha256"], "target_vehicle.sha256")
        if root["target_vehicle_info_sha256"] != target["sha256"]:
            _fail("top-level target vehicle SHA-256 differs from its binding")
        target_without_hash = {
            key: value
            for key, value in target.items()
            if key != "binding_sha256"
        }
        target_binding_sha = _sha256(
            target["binding_sha256"], "target_vehicle.binding_sha256"
        )
        if target_binding_sha != _canonical_sha256(target_without_hash):
            _fail("target vehicle binding SHA-256 mismatch")
    else:
        if target is not None:
            _fail("Navigation Map-only contract must not bind an Autoware vehicle file")
        if root["target_vehicle_info_sha256"] != "none":
            _fail("Navigation Map-only contract target vehicle SHA must be none")
    return root


def _atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent, text=True
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def write_contract(contract_path: Path, sha256_path: Path, contract: dict) -> None:
    validate_contract_document(contract)
    text = json.dumps(contract, indent=2, sort_keys=True) + "\n"
    digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
    _atomic_text(contract_path, text)
    _atomic_text(sha256_path, digest + "\n")


def load_hashed_contract(contract_path: Path, sha256_path: Path) -> dict:
    try:
        expected = sha256_path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as error:
        _fail(f"could not read calibration contract/hash: {error}")
    _sha256(expected, "contract sidecar hash")
    # sha256_file also rejects symlinks and non-regular files and detects a
    # file that changes while it is being hashed.
    actual = _sha256_file(contract_path)
    _sha256_file(sha256_path)
    if actual != expected:
        _fail(f"calibration contract SHA-256 mismatch: expected={expected}, actual={actual}")
    try:
        raw = contract_path.read_bytes()
        expected_after = sha256_path.read_text(encoding="ascii").strip()
        actual_after = _sha256_file(contract_path)
        _sha256_file(sha256_path)
        if (
            expected_after != expected
            or actual_after != actual
            or hashlib.sha256(raw).hexdigest() != actual
        ):
            _fail("calibration contract or SHA-256 sidecar changed while being read")
        document = json.loads(raw)
    except (UnicodeError, json.JSONDecodeError) as error:
        _fail(f"invalid calibration contract JSON: {error}")
    return validate_contract_document(document)


def verify_bindings(
    contract: dict,
    *,
    output_directory: Path,
    workspace_root: Path,
    input_lock_path: Path,
    expected_dataset: str,
    expected_map_type: str,
    target_vehicle_path: Path | None,
    verify_inputs: bool,
) -> None:
    if contract["schema_version"] != SCHEMA_VERSION:
        _fail("fixed-dataset verification requires a schema-v1 contract")
    if contract["dataset"] != expected_dataset:
        _fail("calibration contract dataset differs from requested dataset")
    if contract["map_type"] != expected_map_type:
        _fail("calibration contract map type differs from requested map type")
    actual_lock = input_lock_path.resolve(strict=True)
    lock_binding = contract["input"]["fixed_input_lock"]
    if _sha256_file(actual_lock) != lock_binding["sha256"]:
        _fail("fixed input lock SHA-256 differs from calibration contract")
    if verify_inputs:
        evidence = _require_fixed_provenance().verify_fixed_dataset_input(
            workspace_root, expected_dataset, actual_lock
        )
        if _public_input_binding(evidence) != {
            key: contract["input"][key]
            for key in ("dataset", "input_type", "files", "binding_sha256")
        }:
            _fail("current dataset input binding differs from calibration contract")

    report_path = output_directory.resolve(strict=True) / contract["generation_report"][
        "stored_file"
    ]
    if _sha256_file(report_path) != contract["generation_report"]["sha256"]:
        _fail("generation_report.yaml SHA-256 differs from calibration contract")

    target = contract["target_vehicle"]
    if target is not None:
        if target_vehicle_path is None:
            _fail("target vehicle path is required by calibration contract")
        resolved_path = target_vehicle_path.resolve(strict=True)
        if _sha256_file(resolved_path) != target["sha256"]:
            _fail("target vehicle YAML SHA-256 differs from calibration contract")
        vehicle = resolve(resolved_path)
        if _vehicle_binding(vehicle, target["stored_file"]) != target:
            _fail("target vehicle derived binding differs from calibration contract")
        _assert_vehicle_matches_report(vehicle, contract["vehicle_model"])


def verify_direct_bindings(
    contract: dict,
    *,
    output_directory: Path,
    expected_dataset: str,
    expected_map_type: str,
    generator_parameters_path: Path,
    target_vehicle_path: Path | None,
    verify_inputs: bool,
) -> None:
    if contract["schema_version"] != DIRECT_SCHEMA_VERSION:
        _fail("direct/public verification requires a schema-v2 contract")
    if contract["dataset"] != expected_dataset:
        _fail("calibration contract dataset differs from requested dataset")
    if contract["map_type"] != expected_map_type:
        _fail("calibration contract map type differs from requested map type")

    parameters = contract["generator_parameters"]
    actual_parameters = _direct_regular_file(
        generator_parameters_path, "generator parameter YAML"
    )
    if actual_parameters != parameters:
        _fail("generator parameter YAML path, size, or SHA-256 differs from contract")

    report_path = output_directory.resolve(strict=True) / contract["generation_report"][
        "stored_file"
    ]
    if _sha256_file(report_path) != contract["generation_report"]["sha256"]:
        _fail("generation_report.yaml SHA-256 differs from calibration contract")

    target = contract["target_vehicle"]
    if target is not None:
        if target_vehicle_path is None:
            _fail("target vehicle path is required by calibration contract")
        actual_target = _direct_regular_file(
            target_vehicle_path, "target vehicle YAML"
        )
        if actual_target["sha256"] != target["sha256"]:
            _fail("target vehicle YAML SHA-256 differs from calibration contract")
        vehicle = resolve(Path(actual_target["path"]))
        if _vehicle_binding(vehicle, target["stored_file"]) != target:
            _fail("target vehicle derived binding differs from calibration contract")
        _assert_vehicle_matches_report(vehicle, contract["vehicle_model"])
    elif target_vehicle_path is not None:
        _fail("Navigation Map-only direct contract must not receive a target vehicle YAML")

    if verify_inputs:
        scalars = _load_yaml_scalars(report_path)
        evidence = direct_input_evidence(
            scalars,
            expected_dataset,
            Path(contract["input"]["path_resolution_base"]),
        )
        if _direct_input_binding(
            evidence, Path(contract["input"]["path_resolution_base"])
        ) != contract["input"]:
            _fail("current direct GLIM/rosbag2 input binding differs from contract")
        if _sha256_file(report_path) != contract["generation_report"]["sha256"]:
            _fail("generation_report.yaml changed while direct inputs were verified")


def restoration_rows(contract: dict) -> list[tuple[str, str]]:
    validate_contract_document(contract)
    vehicle = contract["vehicle_model"]
    extrinsics = contract["lidar_base_extrinsics"]
    controls = contract["generation_controls"]

    def boolean(value: bool) -> str:
        return "true" if value else "false"

    def ros_double(value: object) -> str:
        # ROS 2 infers scalar override types from their YAML token too.  repr
        # on a float keeps a decimal point for integral values (0.0, 1.0)
        # while retaining Python's shortest round-trip representation.
        return repr(float(value))

    def ros_double_array(values: list[object]) -> str:
        # ROS 2 infers the element type of a command-line sequence from its
        # YAML tokens.  generation_report.yaml may serialize a zero-valued
        # double as ``0``; passing that spelling back alongside ``0.77`` makes
        # a mixed integer/double sequence which ROS 2 rejects.  Convert every
        # validated numeric element to float so integral values retain ``.0``.
        return json.dumps(
            [float(value) for value in values],
            separators=(",", ":"),
            allow_nan=False,
        )

    target = contract["target_vehicle"]
    return [
        ("MAP_TYPE", contract["map_type"]),
        ("TARGET_VEHICLE_INFO_SHA256", target["sha256"] if target else "none"),
        ("PLATFORM_PROFILE", vehicle["profile"]),
        ("BASE_REFERENCE", vehicle["base_reference"]),
        ("FOOTPRINT_MODEL", vehicle["footprint_model"]),
        ("ROBOT_WIDTH", ros_double(vehicle["width_m"])),
        ("FRONT_EXTENT", ros_double(vehicle["front_extent_m"])),
        ("REAR_EXTENT", ros_double(vehicle["rear_extent_m"])),
        ("CLEARANCE_MARGIN", ros_double(vehicle["clearance_margin_m"])),
        (
            "COLLISION_MIN_HEIGHT",
            ros_double(vehicle["minimum_collision_height_m"]),
        ),
        (
            "COLLISION_MAX_HEIGHT",
            ros_double(vehicle["maximum_collision_height_m"]),
        ),
        (
            "MINIMUM_TURNING_RADIUS",
            ros_double(vehicle["minimum_turning_radius_m"]),
        ),
        ("ALLOW_IN_PLACE_ROTATION", boolean(vehicle["allow_in_place_rotation"])),
        ("ALLOW_REVERSE_MOTION", boolean(vehicle["allow_reverse_motion"])),
        ("DIMENSIONS_SOURCE", vehicle["dimensions_source"]),
        ("DIMENSIONS_CONFIDENCE", vehicle["dimensions_confidence"]),
        ("DIMENSIONS_VERIFIED", boolean(vehicle["dimensions_verified"])),
        ("EXTRINSICS_SOURCE", extrinsics["source"]),
        ("EXTRINSICS_CALIBRATION_SOURCE", extrinsics["calibration_source"]),
        ("EXTRINSICS_CALIBRATION_CONFIDENCE", extrinsics["calibration_confidence"]),
        ("EXTRINSICS_VERIFIED", boolean(extrinsics["verified"])),
        (
            "EXTRINSICS_TRANSLATION",
            ros_double_array(extrinsics["translation_xyz_m"]),
        ),
        (
            "EXTRINSICS_QUATERNION_XYZW",
            ros_double_array(extrinsics["quaternion_xyzw"]),
        ),
        ("POSE_REFERENCE_FRAME", controls["pose_reference_frame"]),
        ("FREE_SPACE_EVIDENCE_MODE", controls["free_space_evidence_mode"]),
        (
            "TRAJECTORY_CLEARANCE_RADIUS",
            ros_double(controls["observed_trajectory_clearance_radius_m"]),
        ),
        ("TRAJECTORY_FREE_SPACE_MODEL", controls["trajectory_free_space_model"]),
        (
            "TRAJECTORY_FOOTPRINT_EROSION_MARGIN",
            ros_double(controls["trajectory_footprint_erosion_margin_m"]),
        ),
        ("NAV2_FREE_SPACE_VERIFIED", boolean(controls["nav2_free_space_verified"])),
        (
            "LANELET2_PHYSICAL_BOUNDARIES_VERIFIED",
            boolean(controls["lanelet2_physical_boundaries_verified"]),
        ),
        (
            "TERMINAL_LOCALIZATION_SETTLING_VERIFIED",
            boolean(controls["terminal_localization_settling_verified"]),
        ),
        (
            "ACQUISITION_VEHICLE_IS_TARGET",
            boolean(contract["acquisition_vehicle_is_target"]),
        ),
    ]


def _parse_bool(text: str, context: str) -> bool:
    if text == "true":
        return True
    if text == "false":
        return False
    _fail(f"{context} must be true or false")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    create = commands.add_parser("create")
    create.add_argument("--workspace-root", type=Path, required=True)
    create.add_argument("--input-lock", type=Path, required=True)
    create.add_argument("--dataset", required=True)
    create.add_argument("--map-type", choices=sorted(MAP_TYPES), required=True)
    create.add_argument("--generation-report", type=Path, required=True)
    create.add_argument("--target-vehicle-info", type=Path)
    create.add_argument("--acquisition-vehicle-is-target", required=True)
    create.add_argument("--output", type=Path, required=True)
    create.add_argument("--sha256-output", type=Path, required=True)

    create_direct = commands.add_parser("create-direct")
    create_direct.add_argument("--dataset", required=True)
    create_direct.add_argument("--map-type", choices=sorted(MAP_TYPES), required=True)
    create_direct.add_argument("--generation-report", type=Path, required=True)
    create_direct.add_argument("--generator-parameters", type=Path, required=True)
    create_direct.add_argument("--input-path-base", type=Path, required=True)
    create_direct.add_argument("--target-vehicle-info", type=Path)
    create_direct.add_argument("--acquisition-vehicle-is-target", required=True)
    create_direct.add_argument("--output", type=Path, required=True)
    create_direct.add_argument("--sha256-output", type=Path, required=True)

    verify = commands.add_parser("verify")
    verify.add_argument("--workspace-root", type=Path, required=True)
    verify.add_argument("--input-lock", type=Path, required=True)
    verify.add_argument("--expected-dataset", required=True)
    verify.add_argument("--expected-map-type", choices=sorted(MAP_TYPES), required=True)
    verify.add_argument("--contract", type=Path, required=True)
    verify.add_argument("--sha256", type=Path, required=True)
    verify.add_argument("--target-vehicle-info", type=Path)
    verify.add_argument("--verify-inputs", action="store_true")
    verify.add_argument("--format", choices=("json", "tsv"), default="json")

    verify_direct = commands.add_parser("verify-direct")
    verify_direct.add_argument("--expected-dataset", required=True)
    verify_direct.add_argument(
        "--expected-map-type", choices=sorted(MAP_TYPES), required=True
    )
    verify_direct.add_argument("--contract", type=Path, required=True)
    verify_direct.add_argument("--sha256", type=Path, required=True)
    verify_direct.add_argument("--generator-parameters", type=Path, required=True)
    verify_direct.add_argument("--target-vehicle-info", type=Path)
    verify_direct.add_argument("--verify-inputs", action="store_true")
    verify_direct.add_argument("--format", choices=("json", "tsv"), default="json")

    args = parser.parse_args()
    try:
        if args.command == "create":
            report = args.generation_report.resolve(strict=True)
            scalars = _load_yaml_scalars(report)
            evidence = _require_fixed_provenance().verify_fixed_dataset_input(
                args.workspace_root, args.dataset, args.input_lock
            )
            target = resolve(args.target_vehicle_info) if args.target_vehicle_info else None
            contract = build_contract(
                dataset=args.dataset,
                map_type=args.map_type,
                generation_report=report,
                report_scalars=scalars,
                input_evidence=evidence,
                input_lock_path=args.input_lock,
                target_vehicle=target,
                target_vehicle_filename=(
                    args.target_vehicle_info.name if args.target_vehicle_info else None
                ),
                acquisition_vehicle_is_target=_parse_bool(
                    args.acquisition_vehicle_is_target,
                    "acquisition vehicle identity",
                ),
            )
            write_contract(args.output, args.sha256_output, contract)
            return 0

        if args.command == "create-direct":
            report = _absolute_path(args.generation_report)
            # Hash/report parsing rejects a symlinked or concurrently changing
            # generation report before any contract is written.
            _reject_symlink_components(report, "generation report")
            report_sha_before = _sha256_file(report)
            scalars = _load_yaml_scalars(report)
            if _sha256_file(report) != report_sha_before:
                _fail("generation report changed while the direct contract was created")
            input_base = _absolute_path(args.input_path_base)
            evidence = direct_input_evidence(scalars, args.dataset, input_base)
            if args.target_vehicle_info:
                target_path = Path(
                    _direct_regular_file(
                        args.target_vehicle_info, "target vehicle YAML"
                    )["path"]
                )
                target = resolve(target_path)
            else:
                target = None
            contract = build_direct_contract(
                dataset=args.dataset,
                map_type=args.map_type,
                generation_report=report,
                report_scalars=scalars,
                input_evidence=evidence,
                path_resolution_base=input_base,
                generator_parameters_path=args.generator_parameters,
                target_vehicle=target,
                target_vehicle_filename=(
                    args.target_vehicle_info.name if args.target_vehicle_info else None
                ),
                acquisition_vehicle_is_target=_parse_bool(
                    args.acquisition_vehicle_is_target,
                    "acquisition vehicle identity",
                ),
            )
            write_contract(args.output, args.sha256_output, contract)
            return 0

        contract = load_hashed_contract(args.contract, args.sha256)
        if args.command == "verify":
            verify_bindings(
                contract,
                output_directory=args.contract.parent,
                workspace_root=args.workspace_root,
                input_lock_path=args.input_lock,
                expected_dataset=args.expected_dataset,
                expected_map_type=args.expected_map_type,
                target_vehicle_path=args.target_vehicle_info,
                verify_inputs=args.verify_inputs,
            )
        else:
            verify_direct_bindings(
                contract,
                output_directory=args.contract.parent,
                expected_dataset=args.expected_dataset,
                expected_map_type=args.expected_map_type,
                generator_parameters_path=args.generator_parameters,
                target_vehicle_path=args.target_vehicle_info,
                verify_inputs=args.verify_inputs,
            )
        if args.format == "tsv":
            for key, value in restoration_rows(contract):
                print(f"{key}\t{value}")
        else:
            print(json.dumps(contract, indent=2, sort_keys=True))
        return 0
    except (
        CalibrationContractError,
        VehicleInfoError,
        FixedProvenanceError,
        OSError,
        UnicodeError,
        json.JSONDecodeError,
    ) as error:
        print(f"error: calibration contract: {error}", file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
