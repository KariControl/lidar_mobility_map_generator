#!/usr/bin/env python3
"""
Create a deterministic, location-free GLIM-style demo fixture.

The fixture is generated rather than recorded so it contains no facility,
person, vehicle-registration, GNSS, or rosbag metadata.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile


SCHEMA_VERSION = 1


def _samples(first: float, last: float, step: float):
    count = int(round((last - first) / step))
    for index in range(count + 1):
        yield first + index * step


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent, text=True
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _point_cloud() -> list[tuple[float, float, float, float]]:
    points: list[tuple[float, float, float, float]] = []

    # A directly observed flat floor provides explicit FREE evidence.
    for x in _samples(-1.5, 13.5, 0.05):
        for y in _samples(-2.5, 2.5, 0.05):
            points.append((x, y, 0.0, 15.0))

    # Two walls bound the synthetic closed course.
    for x in _samples(-1.5, 13.5, 0.05):
        for z in _samples(0.10, 1.20, 0.10):
            points.append((x, -2.45, z, 80.0))
            points.append((x, 2.45, z, 80.0))

    # An off-route pillar makes obstacle classification visible without
    # blocking the demonstrated path.
    for angle_index in range(48):
        angle = 2.0 * math.pi * angle_index / 48.0
        for z in _samples(0.10, 1.20, 0.10):
            points.append(
                (7.0 + 0.20 * math.cos(angle), 1.55 + 0.20 * math.sin(angle), z, 100.0)
            )
    return points


def _ply(points: list[tuple[float, float, float, float]]) -> str:
    header = [
        "ply",
        "format ascii 1.0",
        "comment synthetic location-free ROSCon demo fixture",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
        "property float intensity",
        "end_header",
    ]
    rows = [f"{x:.6f} {y:.6f} {z:.6f} {intensity:.1f}" for x, y, z, intensity in points]
    return "\n".join(header + rows) + "\n"


def _trajectory() -> str:
    rows = []
    x_values = list(_samples(0.0, 12.0, 0.10))
    for index, x in enumerate(x_values):
        y = 0.35 * math.sin(0.45 * x)
        derivative = 0.35 * 0.45 * math.cos(0.45 * x)
        yaw = math.atan2(derivative, 1.0)
        stamp = 1000.0 + 0.10 * index
        rows.append(
            f"{stamp:.6f} {x:.6f} {y:.6f} 0.000000 "
            f"0.000000 0.000000 {math.sin(0.5 * yaw):.9f} {math.cos(0.5 * yaw):.9f}"
        )
    return "\n".join(rows) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument(
        "--replace",
        action="store_true",
        help="replace only the three known generated fixture files",
    )
    args = parser.parse_args()

    output = args.output_directory.resolve()
    targets = [output / "map.ply", output / "traj_lidar.txt", output / "FIXTURE.json"]
    existing = [path for path in targets if path.exists()]
    if existing and not args.replace:
        parser.error(
            "fixture already exists; pass --replace to replace only: "
            + ", ".join(str(path) for path in existing)
        )

    points = _point_cloud()
    _atomic_text(output / "map.ply", _ply(points))
    _atomic_text(output / "traj_lidar.txt", _trajectory())
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "kind": "deterministic_synthetic_glim_fixture",
        "coordinate_frame": "map",
        "trajectory_frame": "base",
        "location_free": True,
        "contains_recorded_personal_or_facility_data": False,
        "point_count": len(points),
        "trajectory_pose_count": 121,
        "license": "Apache-2.0",
    }
    _atomic_text(output / "FIXTURE.json", json.dumps(manifest, indent=2) + "\n")
    print(f"ROSCon synthetic input: {output}")
    print(f"points={len(points)} trajectory_poses=121")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
