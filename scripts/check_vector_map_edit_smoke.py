#!/usr/bin/env python3
"""Read-only smoke audit for one edited Vector Map output directory.

The audit is deliberately narrower than the full Autoware acceptance suite.  It
checks the user-facing edit handoff for either supported Vector Map centerline
source: one complete selected-source Route, one exact 0.90 -> 0.30 m/s
boundary, one requested stop line, and the staged Lanelet2/candidate artifacts
produced by the subsequent regeneration.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from typing import Any


FAST_SPEED = Decimal("0.90")
SLOW_SPEED = Decimal("0.30")
DISTANCE_TOLERANCE_M = Decimal("0.00000001")


class AuditError(RuntimeError):
    """Raised when an artifact violates the smoke-test contract."""


@dataclass(frozen=True)
class RouteSpan:
    feature_id: int
    index: int
    edge_id: int
    start_s: Decimal
    end_s: Decimal
    start_anchor: tuple[Decimal, Decimal, Decimal] | None
    end_anchor: tuple[Decimal, Decimal, Decimal] | None


@dataclass
class SemanticFeature:
    feature_id: int
    feature_type: str
    geometry_type: str
    enabled: bool
    value: Decimal
    name: str
    declared_edge_ids: list[int]
    spans: list[RouteSpan]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditError(message)


def distance_equal(first: Decimal, second: Decimal) -> bool:
    return abs(first - second) <= DISTANCE_TOLERANCE_M


def decimal_value(value: Any, label: str) -> Decimal:
    try:
        result = value if isinstance(value, Decimal) else Decimal(str(value))
    except (InvalidOperation, ValueError, TypeError) as error:
        raise AuditError(f"{label} is not a decimal number") from error
    if not result.is_finite():
        raise AuditError(f"{label} must be finite")
    return result


def integer_value(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise AuditError(f"{label} is not an integer")
    try:
        result = int(value)
    except (ValueError, TypeError) as error:
        raise AuditError(f"{label} is not an integer") from error
    if str(result) != str(value) and not isinstance(value, int):
        raise AuditError(f"{label} is not an exact integer")
    return result


def require_file(path: pathlib.Path, label: str) -> pathlib.Path:
    if not path.is_file():
        raise AuditError(f"missing {label}: {path}")
    return path


def load_json(path: pathlib.Path, label: str) -> dict[str, Any]:
    require_file(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"), parse_float=Decimal)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AuditError(f"invalid {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise AuditError(f"{label} root must be a JSON object: {path}")
    return value


def sha256_path(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise AuditError(f"could not hash {path}: {error}") from error
    return digest.hexdigest()


def parse_edge_id_list(text: str, label: str) -> list[int]:
    if text == "":
        return []
    result: list[int] = []
    for item in text.split(","):
        try:
            edge_id = int(item)
        except ValueError as error:
            raise AuditError(f"{label} contains an invalid Edge ID") from error
        if edge_id <= 0 or edge_id in result:
            raise AuditError(f"{label} contains a duplicate or non-positive Edge ID")
        result.append(edge_id)
    return result


def parse_semantic_features(path: pathlib.Path) -> dict[int, SemanticFeature]:
    require_file(path, "semantic feature TSV")
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise AuditError(f"could not read semantic feature TSV: {error}") from error

    features: dict[int, SemanticFeature] = {}
    span_rows: dict[int, dict[int, RouteSpan]] = {}
    header_count = 0
    frame_count = 0
    for line_number, line in enumerate(lines, start=1):
        if not line or line.startswith("#"):
            continue
        fields = line.split("\t")
        record = fields[0]
        if record == "LMMG_SEMANTICS":
            require(
                fields == ["LMMG_SEMANTICS", "2"],
                f"semantic TSV line {line_number}: expected LMMG_SEMANTICS version 2",
            )
            header_count += 1
        elif record == "FRAME":
            require(
                len(fields) == 2 and fields[1] == "map",
                f"semantic TSV line {line_number}: FRAME must be map",
            )
            frame_count += 1
        elif record == "FEATURE":
            require(
                len(fields) == 14,
                f"semantic TSV line {line_number}: FEATURE must have 14 fields",
            )
            feature_id = integer_value(fields[1], f"FEATURE ID at line {line_number}")
            require(
                feature_id > 0 and feature_id not in features,
                f"semantic TSV line {line_number}: duplicate or non-positive FEATURE ID",
            )
            require(
                fields[4] in ("0", "1"),
                f"semantic TSV line {line_number}: enabled must be 0 or 1",
            )
            # Parse every numeric field so malformed non-target features do not
            # make a syntactically invalid semantic map look acceptable.
            for index, label in ((5, "x"), (6, "y"), (7, "z"), (8, "yaw"), (10, "extent")):
                decimal_value(fields[index], f"FEATURE {feature_id} {label}")
            feature = SemanticFeature(
                feature_id=feature_id,
                feature_type=fields[2],
                geometry_type=fields[3],
                enabled=fields[4] == "1",
                value=decimal_value(fields[9], f"FEATURE {feature_id} value"),
                name=fields[11],
                declared_edge_ids=parse_edge_id_list(
                    fields[13], f"FEATURE {feature_id} Edge list"
                ),
                spans=[],
            )
            features[feature_id] = feature
        elif record == "SPAN":
            require(
                len(fields) in (6, 12),
                f"semantic TSV line {line_number}: SPAN must have 6 or 12 fields",
            )
            feature_id = integer_value(fields[1], f"SPAN feature ID at line {line_number}")
            index = integer_value(fields[2], f"SPAN index at line {line_number}")
            edge_id = integer_value(fields[3], f"SPAN Edge ID at line {line_number}")
            require(
                feature_id in features,
                f"semantic TSV line {line_number}: SPAN precedes its FEATURE",
            )
            require(
                index >= 0 and edge_id > 0 and index not in span_rows.setdefault(feature_id, {}),
                f"semantic TSV line {line_number}: invalid or duplicate SPAN identity",
            )
            start_s = decimal_value(fields[4], f"SPAN {feature_id}:{index} start_s")
            end_s = decimal_value(fields[5], f"SPAN {feature_id}:{index} end_s")
            require(
                start_s >= 0 and end_s > start_s,
                f"semantic TSV line {line_number}: SPAN bounds must be positive and ordered",
            )
            start_anchor = None
            end_anchor = None
            if len(fields) == 12:
                anchors = tuple(
                    decimal_value(item, f"SPAN {feature_id}:{index} anchor")
                    for item in fields[6:12]
                )
                start_anchor = anchors[0:3]
                end_anchor = anchors[3:6]
            span_rows[feature_id][index] = RouteSpan(
                feature_id,
                index,
                edge_id,
                start_s,
                end_s,
                start_anchor,
                end_anchor,
            )
        elif record == "VERTEX":
            require(
                len(fields) == 6,
                f"semantic TSV line {line_number}: VERTEX must have 6 fields",
            )
            feature_id = integer_value(fields[1], f"VERTEX feature ID at line {line_number}")
            index = integer_value(fields[2], f"VERTEX index at line {line_number}")
            require(
                feature_id in features and index >= 0,
                f"semantic TSV line {line_number}: invalid VERTEX identity",
            )
            for item in fields[3:6]:
                decimal_value(item, f"VERTEX {feature_id}:{index} coordinate")
        else:
            raise AuditError(f"semantic TSV line {line_number}: unknown record {record!r}")

    require(header_count == 1, "semantic TSV must contain one LMMG_SEMANTICS header")
    require(frame_count == 1, "semantic TSV must contain one FRAME record")
    for feature_id, rows in span_rows.items():
        indices = sorted(rows)
        require(
            indices == list(range(len(indices))),
            f"FEATURE {feature_id} SPAN indices must be contiguous from zero",
        )
        features[feature_id].spans = [rows[index] for index in indices]
    return features


def parse_route_graph(path: pathlib.Path) -> tuple[list[int], dict[int, Decimal]]:
    graph = load_json(path, "selected centerline-source Route graph")
    require(graph.get("type") == "FeatureCollection", "Route graph must be a FeatureCollection")
    raw_features = graph.get("features")
    require(isinstance(raw_features, list), "Route graph features must be an array")
    edge_ids: list[int] = []
    edge_lengths: dict[int, Decimal] = {}
    for ordinal, feature in enumerate(raw_features):
        require(isinstance(feature, dict), f"Route graph feature {ordinal} must be an object")
        geometry = feature.get("geometry")
        properties = feature.get("properties")
        require(isinstance(geometry, dict), f"Route graph feature {ordinal} has no geometry")
        if geometry.get("type") != "MultiLineString":
            continue
        require(isinstance(properties, dict), f"Route Edge {ordinal} has no properties")
        edge_id = integer_value(properties.get("id"), f"Route Edge {ordinal} ID")
        length = decimal_value(properties.get("cost"), f"Route Edge {edge_id} cost")
        require(
            edge_id > 0 and edge_id not in edge_lengths,
            "Route Edge IDs must be unique and positive",
        )
        require(length > 0, f"Route Edge {edge_id} cost must be positive")
        coordinates = geometry.get("coordinates")
        require(
            isinstance(coordinates, list) and coordinates,
            f"Route Edge {edge_id} has no coordinates",
        )
        edge_ids.append(edge_id)
        edge_lengths[edge_id] = length
    require(edge_ids, "Route graph contains no MultiLineString Edge")
    return edge_ids, edge_lengths


def parse_full_route(
    authoring: dict[str, Any], full_edge_ids: list[int], route_name: str | None
) -> dict[str, Any]:
    require(authoring.get("schema_version") == 1, "navigation authoring schema_version must be 1")
    require(authoring.get("frame_id") == "map", "navigation authoring frame_id must be map")
    routes = authoring.get("routes")
    require(isinstance(routes, list), "navigation authoring routes must be an array")
    promoted = [
        route
        for route in routes
        if isinstance(route, dict) and route.get("promotion_requested") is True
    ]
    require(len(promoted) == 1, "exactly one Route must have promotion_requested=true")
    route = promoted[0]
    actual_name = route.get("name")
    require(isinstance(actual_name, str) and actual_name, "promoted Route must have a name")
    if route_name is not None:
        require(actual_name == route_name, f"promoted Route name must be {route_name!r}")
    require(route.get("target") == "autoware", "promoted Route target must be autoware")
    require(route.get("validation_requested") is True, "promoted Route must request validation")
    ordered = route.get("ordered_edge_ids")
    require(isinstance(ordered, list), "promoted Route ordered_edge_ids must be an array")
    ordered_ids = [integer_value(item, "promoted Route Edge ID") for item in ordered]
    require(
        ordered_ids == full_edge_ids,
        "promoted Route does not contain every selected-source Edge in source order",
    )
    return route


def route_arc_helpers(
    edge_ids: list[int], edge_lengths: dict[int, Decimal]
) -> tuple[dict[int, Decimal], Decimal]:
    prefixes: dict[int, Decimal] = {}
    total = Decimal(0)
    for edge_id in edge_ids:
        prefixes[edge_id] = total
        total += edge_lengths[edge_id]
    return prefixes, total


def validate_span_chain(
    feature: SemanticFeature,
    edge_ids: list[int],
    edge_lengths: dict[int, Decimal],
    prefixes: dict[int, Decimal],
) -> tuple[Decimal, Decimal]:
    require(feature.spans, f"speed FEATURE {feature.feature_id} has no SPAN")
    declared = []
    arcs: list[tuple[Decimal, Decimal]] = []
    for span in feature.spans:
        require(
            span.edge_id in prefixes,
            f"SPAN references Edge {span.edge_id} outside the full Route",
        )
        require(
            span.end_s <= edge_lengths[span.edge_id] + DISTANCE_TOLERANCE_M,
            f"SPAN {span.feature_id}:{span.index} exceeds Edge {span.edge_id}",
        )
        if span.edge_id not in declared:
            declared.append(span.edge_id)
        arcs.append((prefixes[span.edge_id] + span.start_s, prefixes[span.edge_id] + span.end_s))
    require(
        feature.declared_edge_ids == declared,
        f"FEATURE {feature.feature_id} declared Edge list differs from its SPAN list",
    )
    for left, right in zip(arcs, arcs[1:]):
        require(
            distance_equal(left[1], right[0]),
            f"speed FEATURE {feature.feature_id} contains an internal gap or overlap",
        )
    return arcs[0][0], arcs[-1][1]


def select_speed_feature(
    features: dict[int, SemanticFeature], speed: Decimal
) -> SemanticFeature:
    selected = [
        feature
        for feature in features.values()
        if feature.enabled
        and feature.feature_type == "speed_limit"
        and feature.geometry_type == "route_edges"
        and feature.value == speed
    ]
    require(len(selected) == 1, f"expected exactly one enabled {speed} m/s speed FEATURE")
    return selected[0]


def select_stop_line(
    authoring: dict[str, Any], selector: str, edge_lengths: dict[int, Decimal]
) -> tuple[dict[str, Any], Decimal]:
    stops = authoring.get("stop_lines")
    require(isinstance(stops, list), "navigation authoring stop_lines must be an array")
    matched = [
        stop
        for stop in stops
        if isinstance(stop, dict)
        and (str(stop.get("id")) == selector or stop.get("name") == selector)
    ]
    require(len(matched) == 1, f"expected exactly one stop line matching {selector!r}")
    stop = matched[0]
    stop_id = integer_value(stop.get("id"), "stop line ID")
    edge_id = integer_value(stop.get("edge_id"), "stop line Edge ID")
    s = decimal_value(stop.get("s"), "stop line s")
    width = decimal_value(stop.get("width_m"), "stop line width_m")
    require(stop_id > 0, "stop line ID must be positive")
    require(isinstance(stop.get("name"), str) and stop["name"], "stop line name must be non-empty")
    require(stop.get("target") == "autoware", "stop line target must be autoware")
    require(edge_id in edge_lengths, "stop line is outside the promoted full Route")
    require(Decimal(0) <= s <= edge_lengths[edge_id], "stop line s is outside its Edge")
    require(width > 0, "stop line width_m must be positive")
    anchor = stop.get("anchor")
    require(
        isinstance(anchor, list) and len(anchor) == 3,
        "stop line anchor must have three coordinates",
    )
    for coordinate in anchor:
        decimal_value(coordinate, "stop line anchor coordinate")
    return stop, s


def parse_osm(path: pathlib.Path, label: str) -> ET.Element:
    require_file(path, label)
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as error:
        raise AuditError(f"invalid {label}: {path}: {error}") from error
    require(root.tag == "osm", f"{label} root element must be osm")
    return root


def tags(element: ET.Element) -> dict[str, str]:
    result: dict[str, str] = {}
    for tag in element.findall("tag"):
        key = tag.get("k")
        value = tag.get("v")
        require(
            key is not None and value is not None and key not in result,
            "OSM contains an invalid or duplicate tag",
        )
        result[key] = value
    return result


def lanelet_intervals(root: ET.Element) -> list[dict[str, Any]]:
    intervals: list[dict[str, Any]] = []
    lanelet_count = 0
    relation_ids: set[int] = set()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue
        lanelet_count += 1
        relation_id = integer_value(relation.get("id"), "Lanelet relation ID")
        require(relation_id not in relation_ids, "Lanelet relation IDs must be unique")
        relation_ids.add(relation_id)
        required = {
            "source_route_edge_id",
            "source_start_s_m",
            "source_end_s_m",
            "generator_speed_limit_mps",
        }
        if not required.issubset(relation_tags):
            continue
        start = decimal_value(relation_tags["source_start_s_m"], "Lanelet source_start_s_m")
        end = decimal_value(relation_tags["source_end_s_m"], "Lanelet source_end_s_m")
        require(end > start >= 0, "Lanelet source interval is invalid")
        intervals.append(
            {
                "relation_id": relation_id,
                "edge_id": integer_value(
                    relation_tags["source_route_edge_id"],
                    "Lanelet source Edge ID",
                ),
                "start": start,
                "end": end,
                "speed": decimal_value(
                    relation_tags["generator_speed_limit_mps"],
                    "Lanelet generator speed",
                ),
                "semantic_segment": relation_tags.get("semantic_segment"),
            }
        )
    require(lanelet_count > 0, "Lanelet2 OSM contains no Lanelet relation")
    return intervals


def validate_lanelet_speed_coverage(
    feature: SemanticFeature, intervals: list[dict[str, Any]]
) -> int:
    covered_relations: set[int] = set()
    for span in feature.spans:
        overlaps = sorted(
            (
                item
                for item in intervals
                if item["edge_id"] == span.edge_id
                and item["end"] > span.start_s - DISTANCE_TOLERANCE_M
                and item["start"] < span.end_s + DISTANCE_TOLERANCE_M
            ),
            key=lambda item: item["start"],
        )
        require(overlaps, f"Lanelet2 has no segment for speed SPAN {span.feature_id}:{span.index}")
        cursor = span.start_s
        for item in overlaps:
            overlap_start = max(span.start_s, item["start"])
            overlap_end = min(span.end_s, item["end"])
            require(
                distance_equal(overlap_start, cursor),
                "Lanelet2 speed segmentation contains a gap or overlap",
            )
            require(
                item["speed"] == feature.value,
                "Lanelet2 speed differs from semantic FEATURE value",
            )
            require(
                item["semantic_segment"] == "yes",
                "authored speed Lanelet lacks semantic_segment=yes",
            )
            cursor = overlap_end
            covered_relations.add(item["relation_id"])
        require(
            distance_equal(cursor, span.end_s),
            "Lanelet2 speed segmentation does not cover the complete SPAN",
        )
    return len(covered_relations)


def validate_osm_stop_line(root: ET.Element, stop: dict[str, Any], stop_s: Decimal) -> None:
    stop_id = str(stop["id"])
    stop_name = stop["name"]
    stop_edge = str(stop["edge_id"])
    ways: list[tuple[ET.Element, dict[str, str]]] = []
    for way in root.findall("way"):
        way_tags = tags(way)
        if (
            way_tags.get("type") == "stop_line"
            and way_tags.get("authored_stop_line_id") == stop_id
            and way_tags.get("name") == stop_name
        ):
            ways.append((way, way_tags))
    require(len(ways) == 1, "Lanelet2 must contain exactly one matching stop-line way")
    stop_way, stop_tags = ways[0]
    require(
        stop_tags.get("source_route_edge_id") == stop_edge,
        "Lanelet2 stop-line Edge differs from authoring",
    )
    require(
        distance_equal(
            decimal_value(stop_tags.get("source_route_edge_s_m"), "Lanelet2 stop-line s"),
            stop_s,
        ),
        "Lanelet2 stop-line s differs from authoring",
    )

    regulatory: list[ET.Element] = []
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if (
            relation_tags.get("type") == "regulatory_element"
            and relation_tags.get("subtype") == "traffic_sign"
            and relation_tags.get("sign_type") == "stop_sign"
            and relation_tags.get("authored_stop_line_id") == stop_id
            and relation_tags.get("name") == stop_name
        ):
            regulatory.append(relation)
    require(
        len(regulatory) == 1,
        "Lanelet2 must contain exactly one matching stop regulatory element",
    )
    way_id = stop_way.get("id")
    require(
        any(
            member.get("type") == "way"
            and member.get("ref") == way_id
            and member.get("role") == "ref_line"
            for member in regulatory[0].findall("member")
        ),
        "stop regulatory element does not reference its stop-line way",
    )
    regulatory_id = regulatory[0].get("id")
    require(
        any(
            member.get("type") == "relation"
            and member.get("ref") == regulatory_id
            and member.get("role") == "regulatory_element"
            for relation in root.findall("relation")
            if tags(relation).get("type") == "lanelet"
            for member in relation.findall("member")
        ),
        "no Lanelet references the stop regulatory element",
    )


def validate_acceptance(
    acceptance: dict[str, Any],
    canonical_osm: pathlib.Path,
    semantics: pathlib.Path,
    authoring_path: pathlib.Path,
    graph_path: pathlib.Path,
    route_name: str,
    edge_ids: list[int],
    centerline_source: str,
) -> None:
    require(acceptance.get("format_version") == 1, "candidate acceptance format_version must be 1")
    require(acceptance.get("accepted") is True, "candidate acceptance is not accepted")
    require(acceptance.get("errors") == [], "candidate acceptance contains errors")
    require(
        acceptance.get("centerline_source") in (None, centerline_source),
        "candidate acceptance centerline source differs",
    )
    named_route = acceptance.get("named_route")
    require(isinstance(named_route, dict), "candidate acceptance has no named_route")
    require(named_route.get("name") == route_name, "candidate acceptance Route name differs")
    require(
        named_route.get("ordered_edge_ids") == edge_ids,
        "candidate acceptance Route Edge order differs",
    )
    counts = acceptance.get("counts")
    require(isinstance(counts, dict), "candidate acceptance counts must be an object")
    require(
        integer_value(counts.get("authored_stop_lines"), "candidate authored_stop_lines") >= 1,
        "candidate acceptance reports no authored stop line",
    )
    inputs = acceptance.get("input")
    require(isinstance(inputs, dict), "candidate acceptance input must be an object")
    semantics_hash_key = (
        "semantic_features_autoware_topology_tsv_sha256"
        if centerline_source == "edited_topology"
        else "semantic_features_tsv_sha256"
    )
    expected_hashes = {
        "lanelet2_map_sha256": sha256_path(canonical_osm),
        semantics_hash_key: sha256_path(semantics),
        "navigation_authoring_sha256": sha256_path(authoring_path),
        "full_map_source_graph_sha256": sha256_path(graph_path),
    }
    for key, expected in expected_hashes.items():
        require(inputs.get(key) == expected, f"candidate acceptance {key} is missing or stale")


def audit(
    output: pathlib.Path, stop_selector: str, route_name: str | None
) -> dict[str, Any]:
    canonical_osm = require_file(
        output / "lanelet2_map_closed_course_experimental.osm", "canonical Lanelet2 OSM"
    )
    staged = output / "autoware_closed_course_experimental_map"
    staged_osm = require_file(staged / "lanelet2_map.osm", "staged Lanelet2 OSM")
    acceptance_path = require_file(
        staged / "autoware_candidate_acceptance.json", "candidate acceptance JSON"
    )
    acceptance = load_json(acceptance_path, "candidate acceptance JSON")
    centerline_source = acceptance.get("centerline_source", "recorded_trajectory")
    require(
        centerline_source in ("recorded_trajectory", "edited_topology"),
        "candidate acceptance has an unsupported centerline source",
    )
    if centerline_source == "edited_topology":
        semantics_name = "semantic_features_autoware_topology.tsv"
        authoring_name = "navigation_authoring_autoware_topology.json"
        graph_name = "route_graph_autoware_topology_source.geojson"
    else:
        semantics_name = "semantic_features.tsv"
        authoring_name = "navigation_authoring_autoware_replay.json"
        graph_name = "route_graph_autoware_replay_candidate.geojson"
    semantics = require_file(output / semantics_name, "semantic feature TSV")
    authoring_path = require_file(output / authoring_name, "navigation authoring JSON")
    graph_path = require_file(output / graph_name, "selected centerline-source Route graph")
    staged_semantics = require_file(staged / semantics_name, "staged semantic feature TSV")
    staged_authoring = require_file(staged / authoring_name, "staged navigation authoring JSON")
    require(semantics.read_bytes() == staged_semantics.read_bytes(), "staged semantic TSV is stale")
    require(
        authoring_path.read_bytes() == staged_authoring.read_bytes(),
        "staged navigation authoring is stale",
    )
    require(canonical_osm.read_bytes() == staged_osm.read_bytes(), "staged Lanelet2 OSM is stale")

    features = parse_semantic_features(semantics)
    edge_ids, edge_lengths = parse_route_graph(graph_path)
    authoring = load_json(authoring_path, "navigation authoring JSON")
    route = parse_full_route(authoring, edge_ids, route_name)
    actual_route_name = str(route["name"])
    prefixes, total_length = route_arc_helpers(edge_ids, edge_lengths)

    fast = select_speed_feature(features, FAST_SPEED)
    slow = select_speed_feature(features, SLOW_SPEED)
    _, fast_end = validate_span_chain(fast, edge_ids, edge_lengths, prefixes)
    slow_start, _ = validate_span_chain(slow, edge_ids, edge_lengths, prefixes)
    boundary_delta = slow_start - fast_end
    if boundary_delta > DISTANCE_TOLERANCE_M:
        raise AuditError(
            "positive default-speed gap between 0.90 and 0.30 m/s spans: "
            f"{boundary_delta} m"
        )
    if boundary_delta < -DISTANCE_TOLERANCE_M:
        raise AuditError(f"0.90 and 0.30 m/s spans overlap by {-boundary_delta} m")
    boundary_arc = (fast_end + slow_start) / Decimal(2)
    fast_anchor = fast.spans[-1].end_anchor
    slow_anchor = slow.spans[0].start_anchor
    if fast_anchor is not None and slow_anchor is not None:
        require(
            all(distance_equal(left, right) for left, right in zip(fast_anchor, slow_anchor)),
            "speed boundary anchors differ despite an equal Route arc",
        )

    stop, stop_s = select_stop_line(authoring, stop_selector, edge_lengths)
    stop_arc = prefixes[integer_value(stop["edge_id"], "stop Edge ID")] + stop_s

    canonical_root = parse_osm(canonical_osm, "canonical Lanelet2 OSM")
    parse_osm(staged_osm, "staged Lanelet2 OSM")
    intervals = lanelet_intervals(canonical_root)
    fast_lanelets = validate_lanelet_speed_coverage(fast, intervals)
    slow_lanelets = validate_lanelet_speed_coverage(slow, intervals)
    validate_osm_stop_line(canonical_root, stop, stop_s)

    validate_acceptance(
        acceptance,
        canonical_osm,
        semantics,
        authoring_path,
        graph_path,
        actual_route_name,
        edge_ids,
        centerline_source,
    )

    return {
        "schema_version": 1,
        "kind": "lmmg_vector_map_edit_smoke",
        "accepted": True,
        "errors": [],
        "output_directory": str(output),
        "centerline_source": centerline_source,
        "route": {
            "id": route.get("id"),
            "name": actual_route_name,
            "edge_count": len(edge_ids),
            "ordered_edge_ids": edge_ids,
            "complete_selected_source_exact_order": True,
            "length_m": float(total_length),
        },
        "speed_transition": {
            "first_feature_id": fast.feature_id,
            "first_limit_mps": float(FAST_SPEED),
            "second_feature_id": slow.feature_id,
            "second_limit_mps": float(SLOW_SPEED),
            "boundary_route_arc_m": float(boundary_arc),
            "positive_gap_m": 0.0,
            "overlap_m": 0.0,
            "anchors_equal": all(
                distance_equal(left, right)
                for left, right in zip(fast_anchor, slow_anchor)
            )
            if fast_anchor is not None and slow_anchor is not None
            else None,
            "lanelet_relations": fast_lanelets + slow_lanelets,
        },
        "stop_line": {
            "selector": stop_selector,
            "id": stop["id"],
            "name": stop["name"],
            "edge_id": stop["edge_id"],
            "edge_s_m": float(stop_s),
            "route_arc_m": float(stop_arc),
            "lanelet2_materialized": True,
        },
        "artifacts": {
            "semantic_features_tsv": str(semantics),
            "navigation_authoring": str(authoring_path),
            "route_graph": str(graph_path),
            "canonical_lanelet2_osm": str(canonical_osm),
            "staged_lanelet2_osm": str(staged_osm),
            "candidate_acceptance": str(acceptance_path),
            "stage_matches_canonical": True,
        },
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Read-only smoke audit for a regenerated Vector Map: exact selected-source "
            "Route, adjacent 0.90/0.30 m/s spans, one stop line, and staged Lanelet2."
        )
    )
    result.add_argument("output_directory", type=pathlib.Path)
    result.add_argument(
        "--stop-line-id",
        required=True,
        help="stop-line name or numeric authoring ID (for example stop_user_01)",
    )
    result.add_argument(
        "--route-name",
        help=(
            "optional promoted selected-source Route name; when omitted, the "
            "single promoted Route is checked regardless of display language"
        ),
    )
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    output = args.output_directory.resolve()
    try:
        require(output.is_dir(), f"output directory does not exist: {output}")
        report = audit(output, args.stop_line_id, args.route_name)
    except (AuditError, OSError) as error:
        report = {
            "schema_version": 1,
            "kind": "lmmg_vector_map_edit_smoke",
            "accepted": False,
            "errors": [str(error)],
            "output_directory": str(output),
        }
        json.dump(report, sys.stdout, ensure_ascii=False, indent=2, sort_keys=True)
        sys.stdout.write("\n")
        return 1
    json.dump(report, sys.stdout, ensure_ascii=False, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
