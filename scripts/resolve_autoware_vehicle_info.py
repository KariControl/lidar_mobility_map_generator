#!/usr/bin/env python3
"""Read one Autoware vehicle_info.param.yaml and derive map footprint values.

The map builder uses ``base_link`` at the rear-axle ground projection for a
Vector Map target.  This helper deliberately derives the footprint from the
same ROS parameter file that Autoware uses; it never guesses a vehicle from a
LiDAR or dataset name.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

import yaml


class VehicleInfoError(RuntimeError):
    """Raised when a vehicle-info document is missing or ambiguous."""


VEHICLE_INFO_KEYS = (
    "wheel_radius",
    "wheel_width",
    "wheel_base",
    "wheel_tread",
    "front_overhang",
    "rear_overhang",
    "left_overhang",
    "right_overhang",
    "vehicle_height",
    "max_steer_angle",
)


def _number(value: Any, label: str, *, allow_zero: bool = False) -> float:
    if isinstance(value, bool):
        raise VehicleInfoError(f"{label} must be a number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise VehicleInfoError(f"{label} must be a number") from error
    if not math.isfinite(result) or result < 0.0 or (result == 0.0 and not allow_zero):
        qualifier = "nonnegative" if allow_zero else "positive"
        raise VehicleInfoError(f"{label} must be a finite {qualifier} number")
    return result


def _parameter_blocks(document: Any, path: str = "$") -> list[tuple[str, dict[str, Any]]]:
    found: list[tuple[str, dict[str, Any]]] = []
    if isinstance(document, dict):
        parameters = document.get("ros__parameters")
        if isinstance(parameters, dict) and any(
            key in parameters for key in VEHICLE_INFO_KEYS
        ):
            found.append((f"{path}.ros__parameters", parameters))
        for key, value in document.items():
            if key != "ros__parameters":
                found.extend(_parameter_blocks(value, f"{path}.{key}"))
    elif isinstance(document, list):
        for index, value in enumerate(document):
            found.extend(_parameter_blocks(value, f"{path}[{index}]"))
    return found


def resolve(path: Path) -> dict[str, Any]:
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise VehicleInfoError(f"could not read vehicle-info YAML: {error}") from error
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as error:
        raise VehicleInfoError(f"could not parse vehicle-info YAML: {error}") from error

    blocks = _parameter_blocks(document)
    if not blocks:
        raise VehicleInfoError(
            "vehicle-info YAML has no ros__parameters block containing vehicle fields"
        )
    if len(blocks) != 1:
        locations = ", ".join(location for location, _ in blocks)
        raise VehicleInfoError(
            f"vehicle-info YAML is ambiguous; found {len(blocks)} parameter blocks: "
            f"{locations}"
        )
    location, parameters = blocks[0]
    missing = [key for key in VEHICLE_INFO_KEYS if key not in parameters]
    if missing:
        raise VehicleInfoError(
            f"{location} lacks required vehicle parameters: {', '.join(missing)}"
        )

    values: dict[str, float] = {}
    nonnegative = {
        "front_overhang",
        "rear_overhang",
        "left_overhang",
        "right_overhang",
    }
    for key in VEHICLE_INFO_KEYS:
        values[key] = _number(
            parameters[key], f"{location}.{key}", allow_zero=key in nonnegative
        )
    if not 0.0 < values["max_steer_angle"] < math.pi / 2.0:
        raise VehicleInfoError(
            f"{location}.max_steer_angle must be between 0 and pi/2 radians"
        )

    width = (
        values["wheel_tread"]
        + values["left_overhang"]
        + values["right_overhang"]
    )
    front = values["wheel_base"] + values["front_overhang"]
    rear = values["rear_overhang"]
    radius = values["wheel_base"] / math.tan(values["max_steer_angle"])
    for value, label in (
        (width, "derived width"),
        (front, "derived front extent"),
        (rear, "derived rear extent"),
        (radius, "derived minimum turning radius"),
    ):
        _number(value, label)

    return {
        "schema_version": 1,
        "source": {
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "parameter_block": location,
        },
        "base_reference": "rear_axle_ground_projection",
        "footprint_model": "rectangle",
        "vehicle_info": values,
        "derived_map_parameters": {
            "width_m": width,
            "front_extent_m": front,
            "rear_extent_m": rear,
            "minimum_turning_radius_m": radius,
            "vehicle_height_m": values["vehicle_height"],
        },
    }


def _atomic_write(path: Path, content: str) -> None:
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vehicle_info_yaml", type=Path)
    parser.add_argument("--format", choices=("json", "tsv"), default="json")
    parser.add_argument("--audit-output", type=Path)
    args = parser.parse_args()
    try:
        result = resolve(args.vehicle_info_yaml)
        serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if args.audit_output is not None:
            _atomic_write(args.audit_output, serialized)
        if args.format == "json":
            print(serialized, end="")
        else:
            derived = result["derived_map_parameters"]
            for key in (
                "width_m",
                "front_extent_m",
                "rear_extent_m",
                "minimum_turning_radius_m",
                "vehicle_height_m",
            ):
                # ROS 2 command-line parameter overrides infer their type from
                # the token.  Preserve a decimal point for mathematically
                # integral values (for example rear_extent=1.0); emitting "1"
                # would be inferred as an integer and rejected by a parameter
                # declared as double.
                print(f"{key}\t{float(derived[key])!r}")
            print(f"sha256\t{result['source']['sha256']}")
    except (VehicleInfoError, OSError) as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
