#!/usr/bin/env python3
"""Fail-closed structural check for the synthetic ROSCon demo output."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    position = 0
    while len(tokens) < 4:
        while position < len(data) and chr(data[position]).isspace():
            position += 1
        if position < len(data) and data[position] == ord("#"):
            position = data.find(b"\n", position)
            _require(position >= 0, f"unterminated PGM comment: {path}")
            continue
        end = position
        while end < len(data) and not chr(data[end]).isspace():
            end += 1
        _require(end > position, f"incomplete PGM header: {path}")
        tokens.append(data[position:end])
        position = end
    _require(tokens[0] == b"P5", f"expected binary P5 PGM: {path}")
    width, height, maximum = map(int, tokens[1:])
    _require(maximum == 255, f"unexpected PGM maximum value: {maximum}")
    while position < len(data) and chr(data[position]).isspace():
        position += 1
    pixels = data[position:]
    _require(len(pixels) == width * height, f"PGM pixel count mismatch: {path}")
    return width, height, pixels


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()
    output = args.output_directory.resolve()

    required = [
        "generation_report.yaml",
        "pointcloud_map.pcd",
        "trajectory_processed.tum",
        "route_graph_closed_course_replay_candidate.geojson",
        "lanelet2_map_closed_course_experimental.osm",
        "nav2_map_closed_course_experimental.pgm",
        "nav2_map_closed_course_experimental.yaml",
        "nav2_route_graph_closed_course_experimental.geojson",
        "nav2_waypoints_closed_course_experimental.yaml",
        "navigation_target_readiness.yaml",
    ]
    for name in required:
        path = output / name
        _require(path.is_file() and path.stat().st_size > 0, f"missing output: {path}")

    report = (output / "generation_report.yaml").read_text(encoding="utf-8")
    _require("generation_completed: true" in report, "generation did not complete")
    version_match = re.search(r"(?m)^generator_version:\s*([^\s]+)", report)
    _require(version_match is not None, "generation report has no generator version")

    graph = json.loads(
        (output / "nav2_route_graph_closed_course_experimental.geojson").read_text(
            encoding="utf-8"
        )
    )
    features = graph.get("features", [])
    edges = [
        feature
        for feature in features
        if feature.get("geometry", {}).get("type") in {"LineString", "MultiLineString"}
    ]
    _require(edges, "Nav2 closed-course Route graph has no edges")

    root = ET.parse(output / "lanelet2_map_closed_course_experimental.osm").getroot()
    lanelets = []
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") == "lanelet":
            lanelets.append(relation)
    _require(lanelets, "Autoware closed-course OSM has no Lanelet relations")

    width, height, pixels = _read_pgm(output / "nav2_map_closed_course_experimental.pgm")
    values = set(pixels)
    _require(values <= {0, 205, 254}, f"non-trinary Nav2 values: {sorted(values)}")
    _require(0 in values and 254 in values, "Nav2 map lacks obstacle or explicit FREE cells")

    readiness = (output / "navigation_target_readiness.yaml").read_text(encoding="utf-8")
    _require("generation_complete: true" in readiness, "target readiness is incomplete")
    _require(
        "physical_boundaries_verified: false" in readiness,
        "demo unexpectedly claims surveyed physical boundaries",
    )

    print("ROSCON_DEMO_CHECK=PASS")
    print(f"generator_version={version_match.group(1)}")
    print(f"lanelets={len(lanelets)} nav2_edges={len(edges)} grid={width}x{height}")
    print("scope=synthetic_offline_generation_and_structural_acceptance")
    print("not_verified=localization,control,live_obstacle_avoidance,physical_boundaries")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, ET.ParseError, json.JSONDecodeError) as error:
        print(f"ROSCON_DEMO_CHECK=FAIL: {error}")
        raise SystemExit(1)
