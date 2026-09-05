#!/usr/bin/env python3
"""Fail closed unless one vehicle_info YAML matches every vehicle tag in an OSM."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from resolve_autoware_vehicle_info import VehicleInfoError, resolve


TAG_TO_DERIVED_KEY = {
    "estimated_vehicle_width_m": "width_m",
    "estimated_front_extent_m": "front_extent_m",
    "estimated_rear_extent_m": "rear_extent_m",
    "vehicle_minimum_turning_radius_m": "minimum_turning_radius_m",
}


class BindingError(RuntimeError):
    pass


def _atomic_json(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent, text=True
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def verify(vehicle_yaml: Path, osm: Path) -> dict:
    vehicle = resolve(vehicle_yaml)
    try:
        osm_raw = osm.read_bytes()
        root = ET.fromstring(osm_raw)
    except (OSError, ET.ParseError) as error:
        raise BindingError(f"could not read Lanelet2 OSM: {error}") from error

    expected = vehicle["derived_map_parameters"]
    observed: dict[str, list[float]] = {key: [] for key in TAG_TO_DERIVED_KEY}
    for tag in root.iter("tag"):
        key = tag.get("k")
        if key not in observed:
            continue
        raw = tag.get("v")
        try:
            value = float(raw) if raw is not None else math.nan
        except ValueError as error:
            raise BindingError(f"OSM tag {key} is not numeric: {raw!r}") from error
        if not math.isfinite(value):
            raise BindingError(f"OSM tag {key} is not finite: {raw!r}")
        observed[key].append(value)

    checks: dict[str, dict] = {}
    for tag_key, derived_key in TAG_TO_DERIVED_KEY.items():
        values = observed[tag_key]
        if not values:
            raise BindingError(f"Lanelet2 OSM has no {tag_key} vehicle tag")
        wanted = float(expected[derived_key])
        # Exported decimal text is compared as a number.  The 1e-9 bound only
        # permits floating-point serialization noise; it is not a map margin.
        mismatches = [value for value in values if not math.isclose(
            value, wanted, rel_tol=1.0e-9, abs_tol=1.0e-9
        )]
        if mismatches:
            unique = sorted(set(mismatches))
            raise BindingError(
                f"target vehicle/OSM mismatch for {tag_key}: "
                f"vehicle_info={wanted:.17g}, OSM={unique}"
            )
        checks[tag_key] = {
            "expected": wanted,
            "occurrences": len(values),
            "unique_observed": sorted(set(values)),
        }

    return {
        "schema_version": 1,
        "accepted": True,
        "binding": "target_vehicle_info_to_lanelet2_vehicle_tags",
        "vehicle_info_sha256": vehicle["source"]["sha256"],
        "lanelet2_map_sha256": hashlib.sha256(osm_raw).hexdigest(),
        "base_reference": vehicle["base_reference"],
        "derived_map_parameters": expected,
        "checks": checks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle-yaml", type=Path, required=True)
    parser.add_argument("--osm", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = verify(args.vehicle_yaml, args.osm)
        _atomic_json(args.output, result)
    except (BindingError, VehicleInfoError, OSError) as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
