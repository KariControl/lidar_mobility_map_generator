#!/usr/bin/env python3
"""Validate a generated closed-course Lanelet2 candidate without ROS dependencies."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path
from typing import Any, Iterable, NamedTuple


DEFAULT_MINIMUM_COVERAGE = 0.99
DEFAULT_COVERAGE_DISTANCE_M = 0.50
GEOMETRY_EPSILON = 1.0e-8
MINIMUM_LANELET_CENTERLINE_LENGTH_M = 0.50
MAXIMUM_LANELET_CONNECTION_HEADING_JUMP_DEG = 90.0
TERMINAL_SUPPORT_TANGENT_SPAN_M = 0.50
TERMINAL_SUPPORT_MAXIMUM_HEADING_JUMP_DEG = 30.0
SWEPT_FOOTPRINT_POSE_STEP_M = 0.10
SWEPT_FOOTPRINT_GRID_STEP_M = 0.10
SWEPT_FOOTPRINT_TANGENT_SPAN_M = 0.25
SWEPT_FOOTPRINT_SPATIAL_CELL_M = 2.0
POLYGON_BOUNDARY_DISTANCE_M = 1.0e-7
POLYGON_BOUNDARY_DISTANCE_SQUARED_M2 = 1.0e-14
SYNTHETIC_PLANNING_SUPPORT_SOURCE = "deterministic_kinematic_staging_search"
SYNTHETIC_PLANNING_SUPPORT_CONTRACT_VERSION = 2
SYNTHETIC_PLANNING_SUPPORT_OUTER_POSE_ISOLATION_SCOPE = (
    "nonadjacent_raw_route_centerlines"
)
SYNTHETIC_PLANNING_SUPPORT_OUTER_POSE_ISOLATION_DERIVATION = (
    "vehicle_footprint_circumradius_plus_endpoint_allowance"
)
SYNTHETIC_PLANNING_SUPPORT_SEARCH_STEP_M = 0.25
SYNTHETIC_PLANNING_SUPPORT_PATH_SAMPLE_SPACING_M = 0.10
SYNTHETIC_PLANNING_SUPPORT_FAMILIES_PER_LENGTH = 49
SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_POOL_LIMIT = 512
SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_PAIR_EVALUATION_LIMIT = 512
SYNTHETIC_PLANNING_SUPPORT_RADIUS_MULTIPLIERS = (1.0, 1.5, 2.0)
SYNTHETIC_PLANNING_SUPPORT_TURN_ANGLES_RAD = tuple(
    math.radians(value) for value in (15, 30, 45, 60, 75, 90, 120, 150)
)
SYNTHETIC_PLANNING_SUPPORT_TAGS = (
    "synthetic_planning_support",
    "synthetic_test_staging",
    "surveyed",
    "deployment_ready",
    "support_is_part_of_raw_counts",
    "support_is_part_of_named_route",
    "support_is_raw_coverage",
    "planning_support_contract_version",
    "planning_support_role",
    "planning_support_source",
    "planning_support_estimated",
    "planning_support_geometry_kind",
    "planning_support_adjacent_output_edge_id",
    "planning_support_adjacent_source_edge_id",
    "planning_support_raw_endpoint_node_id",
    "planning_support_source_edge_length_m",
    "planning_support_raw_endpoint_s_m",
    "planning_support_raw_endpoint_x",
    "planning_support_raw_endpoint_y",
    "planning_support_raw_endpoint_z",
    "planning_support_synthetic_endpoint_x",
    "planning_support_synthetic_endpoint_y",
    "planning_support_synthetic_endpoint_z",
    "planning_support_tangent_x",
    "planning_support_tangent_y",
    "planning_support_outer_tangent_x",
    "planning_support_outer_tangent_y",
    "planning_support_centerline_planar_length_m",
    "planning_support_centerline_3d_length_m",
    "planning_support_endpoint_allowance_m",
    "planning_support_required_boundary_beyond_raw_endpoint_m",
    "planning_support_actual_left_boundary_beyond_raw_endpoint_m",
    "planning_support_actual_right_boundary_beyond_raw_endpoint_m",
    "planning_support_search_step_m",
    "planning_support_path_sample_spacing_m",
    "planning_support_search_max_length_m",
    "planning_support_selected_candidate_index",
    "planning_support_candidate_count_tested",
    "planning_support_individually_valid_candidate_rank",
    "planning_support_rejected_kinematic_candidates",
    "planning_support_rejected_invalid_geometry_candidates",
    "planning_support_rejected_outer_raw_overlap_candidates",
    "planning_support_rejected_insufficient_outer_pose_isolation_candidates",
    "planning_support_rejected_raw_polygon_reentry_candidates",
    "planning_support_rejected_nonadjacent_transition_candidates",
    "planning_support_turn_radius_m",
    "planning_support_turn_angle_rad",
    "planning_support_straight_length_m",
    "planning_support_maximum_curvature_inv_m",
    "planning_support_actual_maximum_curvature_inv_m",
    "planning_support_kinematic_valid",
    "planning_support_outer_endpoint_unique",
    "planning_support_outer_endpoint_route_polygon_edge_ids",
    "planning_support_outer_footprint_raw_overlap_edge_ids",
    "planning_support_outer_pose_isolation_scope",
    "planning_support_outer_pose_isolation_derivation",
    "planning_support_required_outer_pose_nonadjacent_raw_centerline_isolation_m",
    "planning_support_actual_outer_pose_nonadjacent_raw_centerline_isolation_m",
    "planning_support_outer_pose_nonadjacent_raw_centerline_count",
    "planning_support_outer_pose_nearest_nonadjacent_raw_centerline_edge_ids",
    "planning_support_raw_overlap_single_transition",
    "planning_support_raw_overlap_transition_length_m",
    "planning_support_nonadjacent_raw_overlap_edge_ids",
    "planning_support_nonadjacent_raw_overlap_transition_length_m",
    "planning_support_maximum_nonadjacent_raw_overlap_transition_length_m",
    "planning_support_outer_footprint_contained",
    "planning_support_connection_footprint_contained",
    "planning_support_candidate_pool_limit",
    "planning_support_head_candidate_pool_size",
    "planning_support_tail_candidate_pool_size",
    "planning_support_candidate_pair_evaluation_limit",
    "planning_support_candidate_pairs_tested",
    "planning_support_selected_candidate_pair_rank",
    "planning_support_rejected_final_boundary_pairs",
    "planning_support_rejected_final_outer_membership_pairs",
    "planning_support_rejected_final_transition_pairs",
    "planning_support_rejected_final_containment_pairs",
    "planning_support_collision_scope",
)
SYNTHETIC_PLANNING_SUPPORT_INTEGER_TAGS = (
    "planning_support_adjacent_output_edge_id",
    "planning_support_adjacent_source_edge_id",
    "planning_support_raw_endpoint_node_id",
    "planning_support_selected_candidate_index",
    "planning_support_candidate_count_tested",
    "planning_support_individually_valid_candidate_rank",
    "planning_support_rejected_kinematic_candidates",
    "planning_support_rejected_invalid_geometry_candidates",
    "planning_support_rejected_outer_raw_overlap_candidates",
    "planning_support_rejected_insufficient_outer_pose_isolation_candidates",
    "planning_support_rejected_raw_polygon_reentry_candidates",
    "planning_support_rejected_nonadjacent_transition_candidates",
    "planning_support_candidate_pool_limit",
    "planning_support_head_candidate_pool_size",
    "planning_support_tail_candidate_pool_size",
    "planning_support_candidate_pair_evaluation_limit",
    "planning_support_candidate_pairs_tested",
    "planning_support_selected_candidate_pair_rank",
    "planning_support_rejected_final_boundary_pairs",
    "planning_support_rejected_final_outer_membership_pairs",
    "planning_support_rejected_final_transition_pairs",
    "planning_support_rejected_final_containment_pairs",
    "planning_support_outer_pose_nonadjacent_raw_centerline_count",
)
SYNTHETIC_PLANNING_SUPPORT_FLOAT_TAGS = (
    "planning_support_source_edge_length_m",
    "planning_support_raw_endpoint_s_m",
    "planning_support_raw_endpoint_x",
    "planning_support_raw_endpoint_y",
    "planning_support_raw_endpoint_z",
    "planning_support_synthetic_endpoint_x",
    "planning_support_synthetic_endpoint_y",
    "planning_support_synthetic_endpoint_z",
    "planning_support_tangent_x",
    "planning_support_tangent_y",
    "planning_support_outer_tangent_x",
    "planning_support_outer_tangent_y",
    "planning_support_centerline_planar_length_m",
    "planning_support_centerline_3d_length_m",
    "planning_support_endpoint_allowance_m",
    "planning_support_required_boundary_beyond_raw_endpoint_m",
    "planning_support_actual_left_boundary_beyond_raw_endpoint_m",
    "planning_support_actual_right_boundary_beyond_raw_endpoint_m",
    "planning_support_search_step_m",
    "planning_support_path_sample_spacing_m",
    "planning_support_search_max_length_m",
    "planning_support_turn_radius_m",
    "planning_support_turn_angle_rad",
    "planning_support_straight_length_m",
    "planning_support_maximum_curvature_inv_m",
    "planning_support_actual_maximum_curvature_inv_m",
    "planning_support_required_outer_pose_nonadjacent_raw_centerline_isolation_m",
    "planning_support_actual_outer_pose_nonadjacent_raw_centerline_isolation_m",
    "planning_support_raw_overlap_transition_length_m",
    "planning_support_nonadjacent_raw_overlap_transition_length_m",
    "planning_support_maximum_nonadjacent_raw_overlap_transition_length_m",
)
SYNTHETIC_PLANNING_SUPPORT_REJECTION_TAGS = (
    "planning_support_rejected_kinematic_candidates",
    "planning_support_rejected_invalid_geometry_candidates",
    "planning_support_rejected_outer_raw_overlap_candidates",
    "planning_support_rejected_insufficient_outer_pose_isolation_candidates",
    "planning_support_rejected_raw_polygon_reentry_candidates",
    "planning_support_rejected_nonadjacent_transition_candidates",
)
SYNTHETIC_PLANNING_SUPPORT_PAIR_REJECTION_TAGS = (
    "planning_support_rejected_final_boundary_pairs",
    "planning_support_rejected_final_outer_membership_pairs",
    "planning_support_rejected_final_transition_pairs",
    "planning_support_rejected_final_containment_pairs",
)
SPEED_PATTERN = re.compile(
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
    r"(?:\s*(m/s|mps|km/h|kmh|kph|mph))?\s*",
    re.IGNORECASE,
)

Point = tuple[float, float, float]
Segment = tuple[Point, Point]
PolygonBounds = tuple[float, float, float, float]
SpatialCell = tuple[int, int]
VALID_CENTERLINE_SOURCES = ("recorded_trajectory", "edited_topology")


class _PreparedPolygonEdge(NamedTuple):
    first: Point
    second: Point
    dx: float
    dy: float
    denominator: float
    ray_dx: float
    ray_dy: float
    bounds: PolygonBounds


class _PreparedPolygon(NamedTuple):
    points: tuple[Point, ...]
    bounds: PolygonBounds
    edges: tuple[_PreparedPolygonEdge, ...]
    boundary_spatial_index: dict[SpatialCell, tuple[int, ...]]
    ray_y_spatial_index: dict[int, tuple[int, ...]]


def _is_synthetic_planning_support(lanelet: dict[str, Any]) -> bool:
    tags = lanelet["tags"]
    return any(key in tags for key in SYNTHETIC_PLANNING_SUPPORT_TAGS)


def _issue(report: dict[str, Any], code: str, message: str) -> None:
    issue = {"code": code, "message": message}
    if issue not in report["errors"]:
        report["errors"].append(issue)


def _warning(report: dict[str, Any], code: str, message: str) -> None:
    warning = {"code": code, "message": message}
    if warning not in report["warnings"]:
        report["warnings"].append(warning)


def _element_tags(element: ET.Element) -> dict[str, str]:
    return {
        child.get("k", ""): child.get("v", "")
        for child in element.findall("tag")
        if child.get("k") is not None
    }


def _read_readiness_centerline_source(
    path: Path, report: dict[str, Any]
) -> str | None:
    if not path.exists():
        return None
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        _issue(report, "malformed_navigation_readiness", str(error))
        return None
    in_autoware = False
    for line in lines:
        if line == "autoware:":
            in_autoware = True
            continue
        if in_autoware and line and not line.startswith(" "):
            break
        if in_autoware:
            match = re.fullmatch(r'  centerline_source:\s*"?([^"\s]+)"?\s*', line)
            if match:
                return match.group(1)
    return None


def _read_vector_map_source_selection(
    path: Path, report: dict[str, Any]
) -> dict[str, str] | None:
    if not path.exists():
        return None
    report["input"]["vector_map_source"] = str(path)
    if path.is_file() and not path.is_symlink():
        report["input"]["vector_map_source_sha256"] = _sha256(path)
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        _issue(report, "malformed_vector_map_source", str(error))
        return None
    fields: dict[str, str] = {}
    header_seen = False
    for line in lines:
        if not line.strip():
            continue
        parts = line.split("\t")
        if len(parts) != 2 or not parts[0] or not parts[1]:
            _issue(
                report,
                "malformed_vector_map_source",
                "vector_map_source.tsv contains a malformed record",
            )
            return None
        key, value = parts
        if key == "LMMG_VECTOR_MAP_SOURCE":
            if header_seen or value != "1":
                _issue(
                    report,
                    "malformed_vector_map_source",
                    "vector_map_source.tsv requires exactly one version-1 header",
                )
                return None
            header_seen = True
            continue
        if key in fields:
            _issue(
                report,
                "malformed_vector_map_source",
                f"vector_map_source.tsv duplicates {key}",
            )
            return None
        fields[key] = value
    if (
        not header_seen
        or set(fields) != {"SOURCE", "FRAME", "GRAPH_FINGERPRINT"}
        or fields["SOURCE"] not in VALID_CENTERLINE_SOURCES
        or not fields["FRAME"]
        or re.fullmatch(r"[0-9a-f]{16}", fields["GRAPH_FINGERPRINT"]) is None
    ):
        _issue(
            report,
            "malformed_vector_map_source",
            "vector_map_source.tsv has an incomplete source/frame/fingerprint binding",
        )
        return None
    return fields


def _resolve_centerline_source(
    directory: Path,
    map_details: dict[str, Any],
    report: dict[str, Any],
) -> str:
    """Bind source selection, readiness, and every exported Lanelet tag."""
    selection_path = directory / "vector_map_source.tsv"
    readiness_path = directory / "navigation_target_readiness.yaml"
    selection = _read_vector_map_source_selection(selection_path, report)
    if selection is not None:
        report["vector_map_source_selection"] = dict(selection)
    readiness_source = _read_readiness_centerline_source(readiness_path, report)
    lanelet_sources = {
        lanelet["tags"].get("centerline_source")
        for lanelet in map_details["lanelets"]
        if "centerline_source" in lanelet["tags"]
    }
    explicit_sources = set(lanelet_sources)
    if selection is not None:
        explicit_sources.add(selection["SOURCE"])
    if readiness_source is not None:
        explicit_sources.add(readiness_source)
    invalid_sources = sorted(
        source for source in explicit_sources if source not in VALID_CENTERLINE_SOURCES
    )
    if invalid_sources:
        _issue(
            report,
            "invalid_centerline_source",
            f"unsupported Vector Map centerline source values: {invalid_sources}",
        )
    valid_sources = {
        source for source in explicit_sources if source in VALID_CENTERLINE_SOURCES
    }
    if len(valid_sources) > 1:
        _issue(
            report,
            "centerline_source_mismatch",
            "vector_map_source.tsv, navigation readiness, and Lanelet tags disagree: "
            f"{sorted(valid_sources)}",
        )
    source = next(iter(valid_sources), "recorded_trajectory")
    report["centerline_source"] = source
    report["user_authored"] = source == "edited_topology"
    report["input"]["navigation_target_readiness"] = str(readiness_path)
    if readiness_path.is_file() and not readiness_path.is_symlink():
        report["input"]["navigation_target_readiness_sha256"] = _sha256(
            readiness_path
        )

    if source == "edited_topology":
        if selection is None:
            _issue(
                report,
                "missing_vector_map_source",
                "edited_topology requires vector_map_source.tsv",
            )
        if readiness_source != source:
            _issue(
                report,
                "missing_or_mismatched_readiness_centerline_source",
                "edited_topology requires the same Autoware readiness centerline_source",
            )
        if len(lanelet_sources) != 1 or source not in lanelet_sources or any(
            lanelet["tags"].get("centerline_source") != source
            for lanelet in map_details["lanelets"]
        ):
            _issue(
                report,
                "missing_or_mismatched_lanelet_centerline_source",
                "every user-authored Lanelet must declare centerline_source=edited_topology",
            )
        for lanelet in map_details["lanelets"]:
            tags = lanelet["tags"]
            if (
                tags.get("provenance") != "user_authored_centerline"
                or tags.get("observed_driven") != "no"
                or tags.get("validation_status")
                != "user_authored_vehicle_footprint_validated_candidate"
            ):
                _issue(
                    report,
                    "invalid_user_authored_lanelet_provenance",
                    f"lanelet {lanelet['label']} lacks user-authored validation provenance",
                )
    return source


def _parse_element_id(
    element: ET.Element, kind: str, report: dict[str, Any]
) -> int | None:
    raw_id = element.get("id")
    try:
        return int(raw_id)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        _issue(report, "malformed_id", f"{kind} has invalid id {raw_id!r}")
        return None


def _point_distance_xy(lhs: Point, rhs: Point) -> float:
    return math.hypot(lhs[0] - rhs[0], lhs[1] - rhs[1])


def _polyline_length_xy(points: list[Point]) -> float:
    return sum(
        _point_distance_xy(points[index - 1], points[index])
        for index in range(1, len(points))
    )


def _polyline_length_xyz(points: list[Point]) -> float:
    return sum(
        math.dist(points[index - 1], points[index])
        for index in range(1, len(points))
    )


def _maximum_discrete_polyline_curvature_xy(points: list[Point]) -> float:
    """Recompute the final sampled-centerline circumcircle curvature."""
    maximum = 0.0
    for first, middle, last in zip(points, points[1:], points[2:]):
        first_vector = (middle[0] - first[0], middle[1] - first[1])
        second_vector = (last[0] - middle[0], last[1] - middle[1])
        chord = (last[0] - first[0], last[1] - first[1])
        denominator = (
            math.hypot(*first_vector)
            * math.hypot(*second_vector)
            * math.hypot(*chord)
        )
        if denominator <= GEOMETRY_EPSILON:
            return math.inf
        cross = first_vector[0] * second_vector[1] - first_vector[1] * second_vector[0]
        maximum = max(maximum, 2.0 * abs(cross) / denominator)
    return maximum


def _joined_polyline(parts: Iterable[list[Point]]) -> list[Point]:
    result: list[Point] = []
    for points in parts:
        if not points:
            continue
        start = 0
        if result:
            # Semantic children are exact slices of one 3-D source arc.  An
            # XY-only seam check could hide an elevation discontinuity while
            # still rendering as a connected Lanelet in a top-down view.
            if math.dist(result[-1], points[0]) > GEOMETRY_EPSILON:
                raise ValueError("polyline parts do not share a directed endpoint")
            start = 1
        result.extend(points[start:])
    return result


def _directed_endpoint_tangent_xy(
    points: list[Point], *, at_end: bool, minimum_span_m: float
) -> tuple[float, float] | None:
    """Match the generator's stable directed endpoint-tangent calculation."""
    if len(points) < 2:
        return None
    if at_end:
        previous = len(points) - 1
        span = 0.0
        while previous > 0 and span < minimum_span_m:
            span += _point_distance_xy(points[previous], points[previous - 1])
            previous -= 1
        dx = points[-1][0] - points[previous][0]
        dy = points[-1][1] - points[previous][1]
    else:
        next_index = 0
        span = 0.0
        while next_index + 1 < len(points) and span < minimum_span_m:
            span += _point_distance_xy(points[next_index], points[next_index + 1])
            next_index += 1
        dx = points[next_index][0] - points[0][0]
        dy = points[next_index][1] - points[0][1]
    length = math.hypot(dx, dy)
    if length <= GEOMETRY_EPSILON:
        return None
    return dx / length, dy / length


def _heading_change_deg(
    lhs: tuple[float, float], rhs: tuple[float, float]
) -> float:
    lhs_norm = math.hypot(*lhs)
    rhs_norm = math.hypot(*rhs)
    if lhs_norm <= GEOMETRY_EPSILON or rhs_norm <= GEOMETRY_EPSILON:
        return math.inf
    # OSM decimal serialization changes the norm of an otherwise identical
    # unit tangent by a few 1e-13.  Normalize before acos so that this harmless
    # representation error is not amplified to ~1e-5 degrees.  The caller
    # independently hard-gates the serialized norm and component geometry.
    cosine = max(
        -1.0,
        min(
            1.0,
            (lhs[0] * rhs[0] + lhs[1] * rhs[1]) / (lhs_norm * rhs_norm),
        ),
    )
    return math.degrees(math.acos(cosine))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _start_direction_xy(points: list[Point]) -> tuple[float, float] | None:
    if len(points) < 2:
        return None
    start = points[0]
    for point in points[1:]:
        dx = point[0] - start[0]
        dy = point[1] - start[1]
        length = math.hypot(dx, dy)
        if length > GEOMETRY_EPSILON:
            return dx / length, dy / length
    return None


def _end_direction_xy(points: list[Point]) -> tuple[float, float] | None:
    if len(points) < 2:
        return None
    end = points[-1]
    for point in reversed(points[:-1]):
        dx = end[0] - point[0]
        dy = end[1] - point[1]
        length = math.hypot(dx, dy)
        if length > GEOMETRY_EPSILON:
            return dx / length, dy / length
    return None


def _point_at_fraction(points: list[Point], fraction: float) -> Point:
    cumulative = [0.0]
    for index in range(1, len(points)):
        cumulative.append(
            cumulative[-1] + _point_distance_xy(points[index - 1], points[index])
        )
    total = cumulative[-1]
    if total <= GEOMETRY_EPSILON:
        return points[0]
    distance = min(1.0, max(0.0, fraction)) * total
    for index in range(1, len(points)):
        if distance <= cumulative[index] + GEOMETRY_EPSILON:
            interval = cumulative[index] - cumulative[index - 1]
            if interval <= GEOMETRY_EPSILON:
                return points[index]
            ratio = (distance - cumulative[index - 1]) / interval
            start = points[index - 1]
            end = points[index]
            return (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
                start[2] + (end[2] - start[2]) * ratio,
            )
    return points[-1]


def _orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, point: Point) -> bool:
    return (
        abs(_orientation(a, b, point)) <= GEOMETRY_EPSILON
        and min(a[0], b[0]) - GEOMETRY_EPSILON
        <= point[0]
        <= max(a[0], b[0]) + GEOMETRY_EPSILON
        and min(a[1], b[1]) - GEOMETRY_EPSILON
        <= point[1]
        <= max(a[1], b[1]) + GEOMETRY_EPSILON
    )


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)
    if (
        ((ab_c > GEOMETRY_EPSILON and ab_d < -GEOMETRY_EPSILON)
         or (ab_c < -GEOMETRY_EPSILON and ab_d > GEOMETRY_EPSILON))
        and ((cd_a > GEOMETRY_EPSILON and cd_b < -GEOMETRY_EPSILON)
             or (cd_a < -GEOMETRY_EPSILON and cd_b > GEOMETRY_EPSILON))
    ):
        return True
    return (
        (abs(ab_c) <= GEOMETRY_EPSILON and _on_segment(a, b, c))
        or (abs(ab_d) <= GEOMETRY_EPSILON and _on_segment(a, b, d))
        or (abs(cd_a) <= GEOMETRY_EPSILON and _on_segment(c, d, a))
        or (abs(cd_b) <= GEOMETRY_EPSILON and _on_segment(c, d, b))
    )


def _polygon_area(polygon: list[Point]) -> float:
    return 0.5 * sum(
        polygon[index][0] * polygon[(index + 1) % len(polygon)][1]
        - polygon[(index + 1) % len(polygon)][0] * polygon[index][1]
        for index in range(len(polygon))
    )


def _polygon_self_intersects(polygon: list[Point]) -> bool:
    edge_count = len(polygon)
    for first in range(edge_count):
        first_next = (first + 1) % edge_count
        for second in range(first + 1, edge_count):
            second_next = (second + 1) % edge_count
            if first == second or first_next == second or second_next == first:
                continue
            if _segments_intersect(
                polygon[first],
                polygon[first_next],
                polygon[second],
                polygon[second_next],
            ):
                return True
    return False


def _prepare_polygon_xy(polygon: list[Point]) -> _PreparedPolygon:
    """Precompute exact-predicate coefficients and conservative edge indexes."""
    if not polygon:
        # Match the legacy predicate, which indexed polygon[-1] for an empty input.
        raise IndexError("list index out of range")
    points = tuple(polygon)
    bounds = (
        min(point[0] for point in points),
        min(point[1] for point in points),
        max(point[0] for point in points),
        max(point[1] for point in points),
    )
    edges: list[_PreparedPolygonEdge] = []
    for index, first in enumerate(points):
        second = points[(index + 1) % len(points)]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        edges.append(
            _PreparedPolygonEdge(
                first=first,
                second=second,
                dx=dx,
                dy=dy,
                denominator=dx * dx + dy * dy,
                # Compute these in the same operand order as the legacy ray test.
                ray_dx=first[0] - second[0],
                ray_dy=first[1] - second[1],
                bounds=(
                    min(first[0], second[0]),
                    min(first[1], second[1]),
                    max(first[0], second[0]),
                    max(first[1], second[1]),
                ),
            )
        )

    mutable_boundary_index: dict[SpatialCell, list[int]] = {}
    for edge_index, edge in enumerate(edges):
        minimum_cell_x = math.floor(
            (edge.bounds[0] - POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        minimum_cell_y = math.floor(
            (edge.bounds[1] - POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        maximum_cell_x = math.floor(
            (edge.bounds[2] + POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        maximum_cell_y = math.floor(
            (edge.bounds[3] + POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        for cell_x in range(minimum_cell_x, maximum_cell_x + 1):
            for cell_y in range(minimum_cell_y, maximum_cell_y + 1):
                mutable_boundary_index.setdefault((cell_x, cell_y), []).append(
                    edge_index
                )

    # The legacy ray loop visits the closing edge first, followed by edges 0..N-2.
    # Retaining that order also makes any future diagnostic short-circuit stable.
    ray_edge_order = (len(edges) - 1,) + tuple(range(len(edges) - 1))
    mutable_ray_y_index: dict[int, list[int]] = {}
    for edge_index in ray_edge_order:
        edge = edges[edge_index]
        minimum_cell_y = math.floor(
            (edge.bounds[1] - POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        maximum_cell_y = math.floor(
            (edge.bounds[3] + POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        for cell_y in range(minimum_cell_y, maximum_cell_y + 1):
            mutable_ray_y_index.setdefault(cell_y, []).append(edge_index)

    return _PreparedPolygon(
        points=points,
        bounds=bounds,
        edges=tuple(edges),
        boundary_spatial_index={
            cell: tuple(indices) for cell, indices in mutable_boundary_index.items()
        },
        ray_y_spatial_index={
            cell: tuple(indices) for cell, indices in mutable_ray_y_index.items()
        },
    )


def _point_to_prepared_edge_distance_squared_xy(
    point: Point, edge: _PreparedPolygonEdge
) -> float:
    """Use precomputed segment coefficients with the legacy distance arithmetic."""
    if edge.denominator <= GEOMETRY_EPSILON * GEOMETRY_EPSILON:
        return (point[0] - edge.first[0]) ** 2 + (point[1] - edge.first[1]) ** 2
    ratio = min(
        1.0,
        max(
            0.0,
            (
                (point[0] - edge.first[0]) * edge.dx
                + (point[1] - edge.first[1]) * edge.dy
            )
            / edge.denominator,
        ),
    )
    nearest_x = edge.first[0] + ratio * edge.dx
    nearest_y = edge.first[1] + ratio * edge.dy
    return (point[0] - nearest_x) ** 2 + (point[1] - nearest_y) ** 2


def _point_in_prepared_polygon_xy(
    point: tuple[float, float], polygon: _PreparedPolygon
) -> bool:
    """Return true on the interior/boundary using conservative edge candidates."""
    cell_x = math.floor(point[0] / SWEPT_FOOTPRINT_SPATIAL_CELL_M)
    cell_y = math.floor(point[1] / SWEPT_FOOTPRINT_SPATIAL_CELL_M)
    candidate = (point[0], point[1], 0.0)
    for edge_index in polygon.boundary_spatial_index.get((cell_x, cell_y), ()):
        if (
            _point_to_prepared_edge_distance_squared_xy(
                candidate, polygon.edges[edge_index]
            )
            <= POLYGON_BOUNDARY_DISTANCE_SQUARED_M2
        ):
            return True

    inside = False
    for edge_index in polygon.ray_y_spatial_index.get(cell_y, ()):
        edge = polygon.edges[edge_index]
        if (edge.second[1] > point[1]) != (edge.first[1] > point[1]):
            intersection_x = (
                edge.ray_dx * (point[1] - edge.second[1])
                / edge.ray_dy
                + edge.second[0]
            )
            if point[0] < intersection_x:
                inside = not inside
    return inside


def _point_in_polygon_xy(point: tuple[float, float], polygon: list[Point]) -> bool:
    """Return true on either the interior or boundary of a simple polygon."""
    return _point_in_prepared_polygon_xy(point, _prepare_polygon_xy(polygon))


def _prepare_polygon_union_xy(
    polygons: Iterable[list[Point]],
) -> tuple[list[_PreparedPolygon], dict[SpatialCell, list[int]]]:
    prepared = [_prepare_polygon_xy(polygon) for polygon in polygons]
    spatial_index: dict[SpatialCell, list[int]] = {}
    for polygon_index, polygon in enumerate(prepared):
        minimum_cell_x = math.floor(
            (polygon.bounds[0] - POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        minimum_cell_y = math.floor(
            (polygon.bounds[1] - POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        maximum_cell_x = math.floor(
            (polygon.bounds[2] + POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        maximum_cell_y = math.floor(
            (polygon.bounds[3] + POLYGON_BOUNDARY_DISTANCE_M)
            / SWEPT_FOOTPRINT_SPATIAL_CELL_M
        )
        for cell_x in range(minimum_cell_x, maximum_cell_x + 1):
            for cell_y in range(minimum_cell_y, maximum_cell_y + 1):
                spatial_index.setdefault((cell_x, cell_y), []).append(polygon_index)
    return prepared, spatial_index


def _point_covered_by_lanelets(
    point: tuple[float, float],
    polygons: list[_PreparedPolygon],
    spatial_index: dict[SpatialCell, list[int]],
) -> bool:
    cell = (
        math.floor(point[0] / SWEPT_FOOTPRINT_SPATIAL_CELL_M),
        math.floor(point[1] / SWEPT_FOOTPRINT_SPATIAL_CELL_M),
    )
    for polygon_index in spatial_index.get(cell, ()):
        polygon = polygons[polygon_index]
        bounds = polygon.bounds
        if not (
            bounds[0] - 1.0e-7 <= point[0] <= bounds[2] + 1.0e-7
            and bounds[1] - 1.0e-7 <= point[1] <= bounds[3] + 1.0e-7
        ):
            continue
        if _point_in_prepared_polygon_xy(point, polygon):
            return True
    return False


def _position_at_arc(
    points: list[Point], cumulative: list[float], arc: float, *, closed: bool
) -> Point:
    length = cumulative[-1]
    if closed:
        arc %= length
    else:
        arc = min(length, max(0.0, arc))
    for index in range(1, len(points)):
        if arc <= cumulative[index] + GEOMETRY_EPSILON:
            interval = cumulative[index] - cumulative[index - 1]
            if interval <= GEOMETRY_EPSILON:
                return points[index]
            ratio = (arc - cumulative[index - 1]) / interval
            first = points[index - 1]
            second = points[index]
            return (
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
                first[2] + (second[2] - first[2]) * ratio,
            )
    return points[-1]


@lru_cache(maxsize=32)
def _footprint_grid_offsets(
    front: float,
    rear: float,
    width: float,
    spacing: float,
) -> tuple[tuple[float, float], ...]:
    longitudinal_count = max(1, math.ceil((front + rear) / spacing))
    lateral_count = max(1, math.ceil(width / spacing))
    return tuple(
        (
            -rear + (front + rear) * index / longitudinal_count,
            -0.5 * width + width * lateral_index / lateral_count,
        )
        for index in range(longitudinal_count + 1)
        for lateral_index in range(lateral_count + 1)
    )


def _footprint_grid_points(
    center: Point,
    yaw: float,
    front: float,
    rear: float,
    width: float,
    spacing: float,
) -> Iterable[tuple[float, float]]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    for longitudinal, lateral in _footprint_grid_offsets(
        front, rear, width, spacing
    ):
        yield (
            center[0] + longitudinal * cosine - lateral * sine,
            center[1] + longitudinal * sine + lateral * cosine,
        )


def _parse_speed_mps(value: str) -> float | None:
    match = SPEED_PATTERN.fullmatch(value)
    if match is None:
        return None
    speed = float(match.group(1))
    if not math.isfinite(speed):
        return None
    # Lanelet2 interprets a unitless speed_limit as km/h. Keep that default
    # here so the acceptance report matches TrafficRules' SI result.
    unit = (match.group(2) or "km/h").lower()
    if unit in ("km/h", "kmh", "kph"):
        speed /= 3.6
    elif unit == "mph":
        speed *= 0.44704
    return speed


def _read_trajectory(path: Path, report: dict[str, Any]) -> list[Point]:
    if not path.is_file():
        _issue(report, "missing_trajectory", f"missing trajectory file: {path}")
        return []
    points: list[Point] = []
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, raw_line in enumerate(stream, start=1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                fields = line.split()
                if len(fields) != 8:
                    _issue(
                        report,
                        "malformed_trajectory",
                        f"trajectory line {line_number} must contain 8 TUM fields",
                    )
                    continue
                try:
                    values = [float(field) for field in fields]
                except ValueError:
                    _issue(
                        report,
                        "malformed_trajectory",
                        f"trajectory line {line_number} contains a non-numeric field",
                    )
                    continue
                if not all(math.isfinite(value) for value in values):
                    _issue(
                        report,
                        "nonfinite_trajectory",
                        f"trajectory line {line_number} contains a non-finite value",
                    )
                    continue
                points.append((values[1], values[2], values[3]))
    except OSError as error:
        _issue(report, "trajectory_read_error", f"failed to read trajectory: {error}")
        return []
    if len(points) < 2:
        _issue(report, "trajectory_too_short", "trajectory must contain at least two poses")
    return points


def _way_points(
    way_id: int,
    ways: dict[int, tuple[int, ...]],
    nodes: dict[int, Point | None],
) -> list[Point]:
    references = ways.get(way_id)
    if references is None:
        return []
    points: list[Point] = []
    for reference in references:
        point = nodes.get(reference)
        if point is None:
            return []
        points.append(point)
    return points


def _read_lanelet_map(
    path: Path, report: dict[str, Any]
) -> tuple[list[Segment], list[tuple[int, int]], dict[str, Any]]:
    details: dict[str, Any] = {
        "nodes": {},
        "ways": {},
        "way_tags": {},
        "relations": {},
        "lanelets": [],
    }
    if not path.is_file():
        _issue(report, "missing_lanelet_map", f"missing Lanelet2 map: {path}")
        return [], [], details
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as error:
        _issue(report, "malformed_xml", f"failed to parse Lanelet2 map: {error}")
        return [], [], details
    if root.tag != "osm":
        _issue(report, "malformed_osm", f"expected <osm> root, got <{root.tag}>")

    nodes: dict[int, Point | None] = {}
    for element in root.findall("node"):
        node_id = _parse_element_id(element, "node", report)
        if node_id is None:
            continue
        if node_id in nodes:
            _issue(report, "duplicate_id", f"duplicate node id {node_id}")
            continue
        tags = _element_tags(element)
        missing = [key for key in ("local_x", "local_y", "ele") if key not in tags]
        if missing:
            _issue(
                report,
                "missing_local_geometry",
                f"node {node_id} lacks local coordinate tags: {', '.join(missing)}",
            )
            nodes[node_id] = None
            continue
        try:
            point = (float(tags["local_x"]), float(tags["local_y"]), float(tags["ele"]))
        except ValueError:
            point = (math.nan, math.nan, math.nan)
        if not all(math.isfinite(value) for value in point):
            _issue(
                report,
                "nonfinite_local_geometry",
                f"node {node_id} has non-finite local coordinates",
            )
            nodes[node_id] = None
        else:
            nodes[node_id] = point

    ways: dict[int, tuple[int, ...]] = {}
    way_tags: dict[int, dict[str, str]] = {}
    for element in root.findall("way"):
        way_id = _parse_element_id(element, "way", report)
        if way_id is None:
            continue
        if way_id in ways:
            _issue(report, "duplicate_id", f"duplicate way id {way_id}")
            continue
        references: list[int] = []
        for nd in element.findall("nd"):
            try:
                reference = int(nd.get("ref"))  # type: ignore[arg-type]
            except (TypeError, ValueError):
                _issue(report, "invalid_node_ref", f"way {way_id} has malformed node ref")
                continue
            references.append(reference)
            if reference not in nodes:
                _issue(
                    report,
                    "invalid_node_ref",
                    f"way {way_id} references missing node {reference}",
                )
        ways[way_id] = tuple(references)
        way_tags[way_id] = _element_tags(element)

    relations: dict[int, dict[str, Any]] = {}
    for element in root.findall("relation"):
        relation_id = _parse_element_id(element, "relation", report)
        if relation_id is None:
            continue
        if relation_id in relations:
            _issue(report, "duplicate_id", f"duplicate relation id {relation_id}")
            continue
        relation_members: list[tuple[str, int, str]] = []
        for member in element.findall("member"):
            try:
                reference = int(member.get("ref"))  # type: ignore[arg-type]
            except (TypeError, ValueError):
                continue
            relation_members.append(
                (member.get("type", ""), reference, member.get("role", ""))
            )
        relations[relation_id] = {
            "tags": _element_tags(element),
            "members": relation_members,
        }

    details["nodes"] = nodes
    details["ways"] = ways
    details["way_tags"] = way_tags
    details["relations"] = relations

    lanelet_count = 0
    centerline_segments: list[Segment] = []
    endpoint_edges: list[tuple[int, int]] = []
    # relation, center start/end, left start/end, right start/end. Lanelet2
    # following topology is identity-based: equal coordinates with different
    # OSM node IDs do not form a routable predecessor/successor connection.
    lanelet_endpoints: list[tuple[str, int, int, int, int, int, int]] = []
    lanelet_centerlines: list[tuple[str, int, int, list[Point]]] = []
    minimum_width = math.inf
    minimum_centerline_length = math.inf
    minimum_speed = math.inf
    required_roles = ("left", "right", "centerline")
    for relation in root.findall("relation"):
        tags = _element_tags(relation)
        if tags.get("type") != "lanelet":
            continue
        lanelet_count += 1
        relation_id = _parse_element_id(relation, "relation", report)
        relation_label = str(relation_id) if relation_id is not None else "<invalid>"
        members: dict[str, list[int]] = {role: [] for role in required_roles}
        regulatory_relations: list[int] = []
        for member in relation.findall("member"):
            role = member.get("role", "")
            if role == "regulatory_element":
                if member.get("type") != "relation":
                    _issue(
                        report,
                        "malformed_regulatory_element_member",
                        f"lanelet {relation_label} regulatory_element is not a relation",
                    )
                    continue
                try:
                    regulatory_reference = int(member.get("ref"))  # type: ignore[arg-type]
                except (TypeError, ValueError):
                    _issue(
                        report,
                        "invalid_regulatory_element_ref",
                        f"lanelet {relation_label} has malformed regulatory_element ref",
                    )
                    continue
                regulatory_relations.append(regulatory_reference)
                if regulatory_reference not in relations:
                    _issue(
                        report,
                        "invalid_regulatory_element_ref",
                        f"lanelet {relation_label} references missing regulatory element "
                        f"{regulatory_reference}",
                    )
                continue
            if role not in members:
                continue
            if member.get("type") != "way":
                _issue(
                    report,
                    "malformed_lanelet_member",
                    f"lanelet {relation_label} member {role} is not a way",
                )
                continue
            try:
                reference = int(member.get("ref"))  # type: ignore[arg-type]
            except (TypeError, ValueError):
                _issue(
                    report,
                    "invalid_way_ref",
                    f"lanelet {relation_label} has malformed {role} way ref",
                )
                continue
            members[role].append(reference)
            if reference not in ways:
                _issue(
                    report,
                    "invalid_way_ref",
                    f"lanelet {relation_label} references missing {role} way {reference}",
                )
        complete = True
        for role in required_roles:
            if len(members[role]) != 1:
                _issue(
                    report,
                    "missing_lanelet_member",
                    f"lanelet {relation_label} requires exactly one {role} way",
                )
                complete = False

        speed_text = tags.get("speed_limit")
        speed_mps = _parse_speed_mps(speed_text) if speed_text is not None else None
        if speed_text is None:
            _issue(
                report,
                "missing_speed_limit",
                f"lanelet {relation_label} has no speed_limit tag",
            )
        elif speed_mps is None:
            _issue(
                report,
                "invalid_speed_limit",
                f"lanelet {relation_label} has malformed speed_limit {speed_text!r}",
            )
        elif speed_mps <= 0.0:
            _issue(
                report,
                "nonpositive_speed_limit",
                f"lanelet {relation_label} speed_limit must be positive",
            )
        else:
            minimum_speed = min(minimum_speed, speed_mps)

        if not complete or any(members[role][0] not in ways for role in required_roles):
            continue
        left_id = members["left"][0]
        right_id = members["right"][0]
        center_id = members["centerline"][0]
        if len({left_id, right_id, center_id}) != 3:
            _issue(
                report,
                "malformed_lanelet_member",
                f"lanelet {relation_label} must use distinct member ways",
            )
            continue
        left = _way_points(left_id, ways, nodes)
        right = _way_points(right_id, ways, nodes)
        center = _way_points(center_id, ways, nodes)
        valid_geometry = True
        for role, points in (("left", left), ("right", right), ("centerline", center)):
            if len(points) < 2 or _polyline_length_xy(points) <= GEOMETRY_EPSILON:
                _issue(
                    report,
                    "degenerate_way_geometry",
                    f"lanelet {relation_label} {role} way is missing or degenerate",
                )
                valid_geometry = False
        if not valid_geometry:
            continue

        centerline_length = _polyline_length_xy(center)
        minimum_centerline_length = min(minimum_centerline_length, centerline_length)
        if (
            centerline_length + GEOMETRY_EPSILON
            < MINIMUM_LANELET_CENTERLINE_LENGTH_M
        ):
            _issue(
                report,
                "subminimum_lanelet_centerline_length",
                f"lanelet {relation_label} centerline length "
                f"{centerline_length:.6f} m is below the required "
                f"{MINIMUM_LANELET_CENTERLINE_LENGTH_M:.6f} m",
            )

        polygon = left + list(reversed(right))
        consecutive_duplicate = any(
            _point_distance_xy(polygon[index], polygon[(index + 1) % len(polygon)])
            <= GEOMETRY_EPSILON
            for index in range(len(polygon))
        )
        if consecutive_duplicate or abs(_polygon_area(polygon)) <= GEOMETRY_EPSILON:
            _issue(
                report,
                "degenerate_boundary_polygon",
                f"lanelet {relation_label} boundary polygon has zero area or a collapsed edge",
            )
        if _polygon_self_intersects(polygon):
            _issue(
                report,
                "self_intersecting_boundary_polygon",
                f"lanelet {relation_label} boundary polygon self-intersects",
            )

        lanelet_minimum_width = min(
            _point_distance_xy(
                _point_at_fraction(left, sample / 100.0),
                _point_at_fraction(right, sample / 100.0),
            )
            for sample in range(101)
        )
        minimum_width = min(minimum_width, lanelet_minimum_width)
        if lanelet_minimum_width <= GEOMETRY_EPSILON:
            _issue(
                report,
                "nonpositive_lanelet_width",
                f"lanelet {relation_label} has zero or negative effective width",
            )

        center_references = ways[center_id]
        left_references = ways[left_id]
        right_references = ways[right_id]
        endpoint_edges.append((center_references[0], center_references[-1]))
        lanelet_endpoints.append(
            (
                relation_label,
                center_references[0], center_references[-1],
                left_references[0], left_references[-1],
                right_references[0], right_references[-1],
            )
        )
        lanelet_centerlines.append(
            (relation_label, center_references[0], center_references[-1], center)
        )
        details["lanelets"].append(
            {
                "relation_id": relation_id,
                "label": relation_label,
                "tags": tags,
                "left": left,
                "right": right,
                "center": center,
                "left_refs": left_references,
                "right_refs": right_references,
                "center_refs": center_references,
                "regulatory_relations": regulatory_relations,
            }
        )
        centerline_segments.extend(zip(center[:-1], center[1:]))

    for predecessor_index, predecessor in enumerate(lanelet_endpoints):
        for successor_index, successor in enumerate(lanelet_endpoints):
            if predecessor_index == successor_index or predecessor[2] != successor[1]:
                continue
            if predecessor[4] != successor[3] or predecessor[6] != successor[5]:
                _issue(
                    report,
                    "unshared_lanelet_boundary_endpoint",
                    f"lanelets {predecessor[0]} -> {successor[0]} share a centerline "
                    "endpoint but not both left/right boundary endpoint node IDs",
                )

    maximum_heading_jump_deg = 0.0
    for predecessor in lanelet_centerlines:
        for successor in lanelet_centerlines:
            if predecessor[0] == successor[0] or predecessor[2] != successor[1]:
                continue
            incoming = _end_direction_xy(predecessor[3])
            outgoing = _start_direction_xy(successor[3])
            if incoming is None or outgoing is None:
                continue
            cosine = min(1.0, max(-1.0, incoming[0] * outgoing[0] + incoming[1] * outgoing[1]))
            heading_jump_deg = math.degrees(math.acos(cosine))
            maximum_heading_jump_deg = max(maximum_heading_jump_deg, heading_jump_deg)
            if (
                heading_jump_deg
                > MAXIMUM_LANELET_CONNECTION_HEADING_JUMP_DEG + 1.0e-9
            ):
                _issue(
                    report,
                    "excessive_lanelet_connection_heading_jump",
                    f"lanelets {predecessor[0]} -> {successor[0]} contain a "
                    f"{heading_jump_deg:.6f} degree connection, above the allowed "
                    f"{MAXIMUM_LANELET_CONNECTION_HEADING_JUMP_DEG:.6f} degrees",
                )
            if heading_jump_deg + 1.0e-9 >= 150.0:
                _issue(
                    report,
                    "forward_replay_cusp",
                    f"lanelets {predecessor[0]} -> {successor[0]} contain a "
                    f"{heading_jump_deg:.6f} degree forward-incompatible cusp",
                )

    if lanelet_count == 0:
        _issue(report, "no_lanelets", "map contains no Lanelet relations")
    report["counts"].update(
        {
            "nodes": len(nodes),
            "ways": len(ways),
            "lanelets": lanelet_count,
            "centerline_segments": len(centerline_segments),
        }
    )
    report["metrics"]["minimum_lanelet_width_m"] = (
        minimum_width if math.isfinite(minimum_width) else None
    )
    report["metrics"]["minimum_lanelet_centerline_length_m"] = (
        minimum_centerline_length
        if math.isfinite(minimum_centerline_length)
        else None
    )
    report["metrics"]["required_minimum_lanelet_centerline_length_m"] = (
        MINIMUM_LANELET_CENTERLINE_LENGTH_M
    )
    report["metrics"]["minimum_speed_limit_mps"] = (
        minimum_speed if math.isfinite(minimum_speed) else None
    )
    report["metrics"]["maximum_lanelet_heading_jump_deg"] = maximum_heading_jump_deg
    report["metrics"]["allowed_maximum_lanelet_heading_jump_deg"] = (
        MAXIMUM_LANELET_CONNECTION_HEADING_JUMP_DEG
    )
    return centerline_segments, endpoint_edges, details


ESTIMATED_BOUNDARY_MODEL = "trajectory_derived_estimated_drivable_corridor"
ESTIMATED_BOUNDARY_ALGORITHM = "oriented_rectangular_swept_envelope"
ESTIMATED_LONGITUDINAL_ENDPOINT_GUARD_M = 0.05
ESTIMATED_BOUNDARY_NUMERIC_TAGS = (
    "estimated_vehicle_width_m",
    "estimated_front_extent_m",
    "estimated_rear_extent_m",
    "vehicle_minimum_turning_radius_m",
    "estimated_lateral_margin_m",
    "estimated_boundary_interpolation_guard_m",
    "estimated_effective_lateral_margin_m",
    "estimated_longitudinal_endpoint_guard_m",
)
ESTIMATED_BOUNDARY_STRING_TAGS = (
    "vehicle_profile",
    "vehicle_base_reference",
    "vehicle_dimensions_evidence_source",
    "vehicle_dimensions_evidence_confidence",
    "vehicle_dimensions_verified",
)


def _ordered_lanelet_centerline_components(
    lanelets: list[dict[str, Any]],
) -> list[tuple[list[Point], bool, list[str]]]:
    """Join every unbranched Lanelet component in its directed OSM topology."""
    starts: dict[int, list[int]] = {}
    ends: dict[int, list[int]] = {}
    for index, lanelet in enumerate(lanelets):
        starts.setdefault(lanelet["center_refs"][0], []).append(index)
        ends.setdefault(lanelet["center_refs"][-1], []).append(index)
    ambiguous = any(len(indices) != 1 for indices in (*starts.values(), *ends.values()))
    if ambiguous:
        # A branch has more than one valid directed traversal. Test each Edge
        # independently rather than choosing a conveniently short branch.
        return [
            (lanelet["center"], False, [lanelet["label"]])
            for lanelet in lanelets
        ]

    remaining = set(range(len(lanelets)))
    components: list[tuple[list[Point], bool, list[str]]] = []
    while remaining:
        heads = sorted(
            index
            for index in remaining
            if not any(
                predecessor in remaining
                for predecessor in ends.get(lanelets[index]["center_refs"][0], [])
            )
        )
        current = heads[0] if heads else min(remaining)
        ordered: list[int] = []
        while current in remaining:
            ordered.append(current)
            remaining.remove(current)
            successors = [
                successor
                for successor in starts.get(
                    lanelets[current]["center_refs"][-1], []
                )
                if successor in remaining
            ]
            if len(successors) != 1:
                break
            current = successors[0]
        closed = (
            lanelets[ordered[-1]]["center_refs"][-1]
            == lanelets[ordered[0]]["center_refs"][0]
        )
        try:
            centerline = _joined_polyline(
                lanelets[index]["center"] for index in ordered
            )
        except ValueError:
            components.extend(
                (
                    lanelets[index]["center"],
                    False,
                    [lanelets[index]["label"]],
                )
                for index in ordered
            )
            continue
        components.append(
            (centerline, closed, [lanelets[index]["label"] for index in ordered])
        )
    return components


def _validate_configured_swept_footprint(
    map_details: dict[str, Any], report: dict[str, Any]
) -> None:
    """Hard-gate the tagged estimated footprint+clearance geometry contract."""
    lanelets = map_details["lanelets"]
    experimental = [
        lanelet
        for lanelet in lanelets
        if lanelet["tags"].get("boundary_model") == ESTIMATED_BOUNDARY_MODEL
    ]
    if not experimental:
        return
    report["counts"]["estimated_swept_boundary_lanelets"] = len(experimental)
    if len(experimental) != len(lanelets):
        _issue(
            report,
            "mixed_closed_course_boundary_models",
            "a closed-course estimated map must tag every road Lanelet with the same "
            "estimated swept-envelope boundary contract",
        )

    required_tags = (
        *ESTIMATED_BOUNDARY_NUMERIC_TAGS,
        *ESTIMATED_BOUNDARY_STRING_TAGS,
        "estimated_boundary_algorithm",
    )
    parsed: list[dict[str, float]] = []
    parsed_strings: list[dict[str, str]] = []
    for lanelet in experimental:
        tags = lanelet["tags"]
        missing = [key for key in required_tags if key not in tags]
        if missing:
            _issue(
                report,
                "missing_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} lacks {', '.join(missing)}",
            )
            continue
        if tags["estimated_boundary_algorithm"] != ESTIMATED_BOUNDARY_ALGORITHM:
            _issue(
                report,
                "unsupported_estimated_boundary_algorithm",
                f"lanelet {lanelet['label']} uses unsupported estimated boundary "
                f"algorithm {tags['estimated_boundary_algorithm']!r}",
            )
            continue
        string_values = {
            key: tags[key] for key in ESTIMATED_BOUNDARY_STRING_TAGS
        }
        if (
            string_values["vehicle_profile"]
            not in ("custom", "small_robot", "car", "yaris")
            or string_values["vehicle_base_reference"]
            not in ("unspecified", "body_center", "rear_axle_ground_projection")
            or string_values["vehicle_dimensions_evidence_source"]
            not in ("unknown", "measured", "catalog_estimated", "inferred")
            or string_values["vehicle_dimensions_evidence_confidence"]
            not in ("unknown", "low", "medium", "high")
            or string_values["vehicle_dimensions_verified"] not in ("yes", "no")
        ):
            _issue(
                report,
                "invalid_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} has invalid vehicle profile/base/evidence metadata",
            )
            continue
        source_unknown = (
            string_values["vehicle_dimensions_evidence_source"] == "unknown"
        )
        confidence_unknown = (
            string_values["vehicle_dimensions_evidence_confidence"] == "unknown"
        )
        verified = string_values["vehicle_dimensions_verified"] == "yes"
        if source_unknown != confidence_unknown or (
            verified
            and (
                string_values["vehicle_dimensions_evidence_source"] != "measured"
                or string_values["vehicle_dimensions_evidence_confidence"] != "high"
            )
        ):
            _issue(
                report,
                "invalid_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} has inconsistent vehicle dimension evidence",
            )
            continue
        try:
            values = {
                key: float(tags[key]) for key in ESTIMATED_BOUNDARY_NUMERIC_TAGS
            }
        except ValueError:
            _issue(
                report,
                "invalid_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} has non-numeric swept-envelope metadata",
            )
            continue
        if not all(math.isfinite(value) for value in values.values()):
            _issue(
                report,
                "invalid_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} has non-finite swept-envelope metadata",
            )
            continue
        if (
            values["estimated_vehicle_width_m"] <= 0.0
            or values["estimated_front_extent_m"] <= 0.0
            or values["estimated_rear_extent_m"] <= 0.0
            or values["vehicle_minimum_turning_radius_m"] <= 0.0
            or values["estimated_lateral_margin_m"] < 0.0
            or values["estimated_boundary_interpolation_guard_m"] < 0.0
            or values["estimated_longitudinal_endpoint_guard_m"] <= 0.0
        ):
            _issue(
                report,
                "invalid_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} has invalid swept-envelope dimensions",
            )
            continue
        if not math.isclose(
            values["estimated_longitudinal_endpoint_guard_m"],
            ESTIMATED_LONGITUDINAL_ENDPOINT_GUARD_M,
            abs_tol=1.0e-12,
        ):
            _issue(
                report,
                "invalid_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} longitudinal endpoint guard does not "
                f"equal the audited {ESTIMATED_LONGITUDINAL_ENDPOINT_GUARD_M:g} m",
            )
            continue
        expected_effective_margin = (
            values["estimated_lateral_margin_m"]
            + values["estimated_boundary_interpolation_guard_m"]
        )
        if not math.isclose(
            values["estimated_effective_lateral_margin_m"],
            expected_effective_margin,
            abs_tol=1.0e-9,
        ):
            _issue(
                report,
                "invalid_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} effective lateral margin does not equal "
                "configured margin plus interpolation guard",
            )
            continue
        parsed.append(values)
        parsed_strings.append(string_values)
    if len(parsed) != len(experimental) or len(parsed_strings) != len(experimental):
        return

    reference = parsed[0]
    for lanelet, values in zip(experimental[1:], parsed[1:]):
        if any(
            not math.isclose(values[key], reference[key], abs_tol=1.0e-9)
            for key in ESTIMATED_BOUNDARY_NUMERIC_TAGS
        ):
            _issue(
                report,
                "inconsistent_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} has a different swept-envelope contract",
            )
    reference_strings = parsed_strings[0]
    for lanelet, values in zip(experimental[1:], parsed_strings[1:]):
        if values != reference_strings:
            _issue(
                report,
                "inconsistent_estimated_boundary_metadata",
                f"lanelet {lanelet['label']} has a different "
                "vehicle profile/base/evidence contract",
            )
    if any(
        issue["code"] == "inconsistent_estimated_boundary_metadata"
        for issue in report["errors"]
    ):
        return

    width_with_clearance = (
        reference["estimated_vehicle_width_m"]
        + 2.0 * reference["estimated_lateral_margin_m"]
    )
    front = reference["estimated_front_extent_m"]
    rear = reference["estimated_rear_extent_m"]
    # Only observed/raw Lanelets may satisfy the swept-footprint contract.
    # Forbidden synthetic endpoint geometry must never turn an otherwise
    # failing raw boundary cap into an accepted candidate.
    swept_lanelets = [
        lanelet for lanelet in lanelets
        if not _is_synthetic_planning_support(lanelet)
    ]
    polygons, polygon_spatial_index = _prepare_polygon_union_xy(
        lanelet["left"] + list(reversed(lanelet["right"]))
        for lanelet in swept_lanelets
    )
    outgoing: dict[int, int] = {}
    incoming: dict[int, int] = {}
    for lanelet in swept_lanelets:
        outgoing[lanelet["center_refs"][0]] = (
            outgoing.get(lanelet["center_refs"][0], 0) + 1
        )
        incoming[lanelet["center_refs"][-1]] = (
            incoming.get(lanelet["center_refs"][-1], 0) + 1
        )
    branch_nodes = sorted(
        node_id
        for node_id in set(outgoing) | set(incoming)
        if outgoing.get(node_id, 0) > 1 or incoming.get(node_id, 0) > 1
    )
    report["counts"]["swept_footprint_branch_nodes"] = len(branch_nodes)
    if branch_nodes:
        # Each predecessor/successor combination needs a separate oriented
        # overhang sweep through a branch. Until that contract is exported,
        # accepting per-Edge interiors would leave those transitions unproved.
        _issue(
            report,
            "branched_swept_footprint_contract_unsupported",
            "estimated swept-envelope acceptance fails closed for branched "
            f"Lanelet topology; unproved branch node IDs: {branch_nodes}",
        )
        return
    components = _ordered_lanelet_centerline_components(swept_lanelets)
    sampled = 0
    failed = 0
    closed_components = 0
    first_failure: tuple[Point, list[str]] | None = None
    for points, closed, labels in components:
        cumulative = [0.0]
        for first, second in zip(points, points[1:]):
            cumulative.append(cumulative[-1] + _point_distance_xy(first, second))
        length = cumulative[-1]
        if length <= GEOMETRY_EPSILON:
            continue
        if closed:
            closed_components += 1
            pose_count = max(1, math.ceil(length / SWEPT_FOOTPRINT_POSE_STEP_M))
            arcs = [length * index / pose_count for index in range(pose_count)]
        else:
            # Open replay components include both measured terminal base poses.
            # Their boundary contract carries rear/front endpoint overhangs,
            # so audit the complete [0,L] arc instead of deleting both ends.
            pose_count = max(
                1, math.ceil(length / SWEPT_FOOTPRINT_POSE_STEP_M)
            )
            arcs = [
                length * index / pose_count
                for index in range(pose_count + 1)
            ]
        for arc in arcs:
            center = _position_at_arc(points, cumulative, arc, closed=closed)
            before = _position_at_arc(
                points,
                cumulative,
                arc - SWEPT_FOOTPRINT_TANGENT_SPAN_M,
                closed=closed,
            )
            after = _position_at_arc(
                points,
                cumulative,
                arc + SWEPT_FOOTPRINT_TANGENT_SPAN_M,
                closed=closed,
            )
            yaw = math.atan2(after[1] - before[1], after[0] - before[0])
            sampled += 1
            fits = all(
                _point_covered_by_lanelets(point, polygons, polygon_spatial_index)
                for point in _footprint_grid_points(
                    center,
                    yaw,
                    front,
                    rear,
                    width_with_clearance,
                    SWEPT_FOOTPRINT_GRID_STEP_M,
                )
            )
            if not fits:
                failed += 1
                if first_failure is None:
                    first_failure = (center, labels)

    report["counts"]["swept_footprint_components"] = len(components)
    report["counts"]["swept_footprint_closed_components"] = closed_components
    report["counts"]["swept_footprint_pose_samples"] = sampled
    report["counts"]["swept_footprint_passed_pose_samples"] = sampled - failed
    report["counts"]["swept_footprint_failed_pose_samples"] = failed
    report["metrics"].update(
        {
            "estimated_vehicle_width_m": reference["estimated_vehicle_width_m"],
            "vehicle_profile": reference_strings["vehicle_profile"],
            "vehicle_base_reference": reference_strings["vehicle_base_reference"],
            "vehicle_dimensions_evidence_source": reference_strings[
                "vehicle_dimensions_evidence_source"
            ],
            "vehicle_dimensions_evidence_confidence": reference_strings[
                "vehicle_dimensions_evidence_confidence"
            ],
            "vehicle_dimensions_verified": (
                reference_strings["vehicle_dimensions_verified"] == "yes"
            ),
            "estimated_front_extent_m": front,
            "estimated_rear_extent_m": rear,
            "vehicle_minimum_turning_radius_m": reference[
                "vehicle_minimum_turning_radius_m"
            ],
            "estimated_lateral_margin_m": reference["estimated_lateral_margin_m"],
            "estimated_boundary_interpolation_guard_m": reference[
                "estimated_boundary_interpolation_guard_m"
            ],
            "estimated_longitudinal_endpoint_guard_m": reference[
                "estimated_longitudinal_endpoint_guard_m"
            ],
            "swept_footprint_width_with_clearance_m": width_with_clearance,
            "swept_footprint_pose_step_m": SWEPT_FOOTPRINT_POSE_STEP_M,
            "swept_footprint_grid_step_m": SWEPT_FOOTPRINT_GRID_STEP_M,
            "configured_swept_footprint_inside_lanelet_union": failed == 0,
        }
    )
    if failed:
        location = ""
        if first_failure is not None:
            location = (
                f"; first failure at ({first_failure[0][0]:.6f}, "
                f"{first_failure[0][1]:.6f}) in Lanelets {first_failure[1]}"
            )
        _issue(
            report,
            "configured_swept_footprint_not_contained",
            f"{failed} of {sampled} full-map centerline sweep poses place the tagged "
            f"vehicle footprint plus lateral clearance outside the Lanelet union{location}",
        )


def _component_count(endpoint_edges: list[tuple[int, int]]) -> int:
    adjacency: dict[int, set[int]] = {}
    for start, end in endpoint_edges:
        adjacency.setdefault(start, set()).add(end)
        adjacency.setdefault(end, set()).add(start)
    remaining = set(adjacency)
    components = 0
    while remaining:
        components += 1
        stack = [remaining.pop()]
        while stack:
            node = stack.pop()
            for neighbour in adjacency[node]:
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    stack.append(neighbour)
    return components


def _trajectory_samples(points: list[Point], maximum_step: float) -> list[Point]:
    if not points:
        return []
    result = [points[0]]
    for start, end in zip(points[:-1], points[1:]):
        length = _point_distance_xy(start, end)
        if length <= GEOMETRY_EPSILON:
            continue
        intervals = max(1, math.ceil(length / maximum_step))
        for index in range(1, intervals + 1):
            ratio = index / intervals
            result.append(
                (
                    start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio,
                    start[2] + (end[2] - start[2]) * ratio,
                )
            )
    return result


def _point_segment_distance_squared_xy(point: Point, segment: Segment) -> float:
    start, end = segment
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= GEOMETRY_EPSILON * GEOMETRY_EPSILON:
        return (point[0] - start[0]) ** 2 + (point[1] - start[1]) ** 2
    ratio = min(
        1.0,
        max(
            0.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / denominator,
        ),
    )
    nearest_x = start[0] + ratio * dx
    nearest_y = start[1] + ratio * dy
    return (point[0] - nearest_x) ** 2 + (point[1] - nearest_y) ** 2


def _point_to_segment_coverage(
    trajectory: list[Point], centerline_segments: list[Segment], distance_limit: float
) -> tuple[float, int, int]:
    if not trajectory or not centerline_segments:
        return 0.0, 0, 0
    samples = _trajectory_samples(trajectory, max(1.0e-3, 0.5 * distance_limit))
    cell_size = max(distance_limit, 0.05)
    spatial_index: dict[tuple[int, int], list[int]] = {}
    for segment_index, (start, end) in enumerate(centerline_segments):
        minimum_x = math.floor((min(start[0], end[0]) - distance_limit) / cell_size)
        maximum_x = math.floor((max(start[0], end[0]) + distance_limit) / cell_size)
        minimum_y = math.floor((min(start[1], end[1]) - distance_limit) / cell_size)
        maximum_y = math.floor((max(start[1], end[1]) + distance_limit) / cell_size)
        for cell_x in range(minimum_x, maximum_x + 1):
            for cell_y in range(minimum_y, maximum_y + 1):
                spatial_index.setdefault((cell_x, cell_y), []).append(segment_index)
    squared_limit = distance_limit * distance_limit
    covered = 0
    for point in samples:
        cell = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
        if any(
            _point_segment_distance_squared_xy(point, centerline_segments[index])
            <= squared_limit
            for index in spatial_index.get(cell, ())
        ):
            covered += 1
    return covered / len(samples), covered, len(samples)


NAMED_ROUTE_IDENTITY_TAGS = (
    "named_route_id",
    "named_route_name",
    "named_route_target",
    "named_route_order",
)

NAMED_ROUTE_TAGS = (
    *NAMED_ROUTE_IDENTITY_TAGS,
    "route_edge_id",
)


def _unsigned_integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value if 0 <= value <= (1 << 64) - 1 else None
    if isinstance(value, str) and re.fullmatch(r"0|[1-9][0-9]*", value):
        parsed = int(value)
        return parsed if parsed <= (1 << 64) - 1 else None
    return None


def _canonical_unsigned_id_list(value: Any) -> list[int] | None:
    """Parse the exporter CSV form and reject alternate/ambiguous encodings."""
    if not isinstance(value, str):
        return None
    if value == "":
        return []
    if re.fullmatch(r"[1-9][0-9]*(?:,[1-9][0-9]*)*", value) is None:
        return None
    result = [int(item) for item in value.split(",")]
    if result != sorted(set(result)) or any(item > (1 << 64) - 1 for item in result):
        return None
    return result


def _planning_support_expected_search_index(
    values: dict[str, float], integers: dict[str, int], geometry_kind: str,
    minimum_turning_radius_m: float,
) -> int | None:
    """Recompute the deterministic analytic-length/family enumeration index."""
    allowance = values["planning_support_endpoint_allowance_m"]
    length = (
        values["planning_support_straight_length_m"]
        + values["planning_support_turn_radius_m"]
        * values["planning_support_turn_angle_rad"]
    )
    step = values["planning_support_search_step_m"]
    if step <= 0.0:
        return None
    length_step_value = (length - allowance) / step
    length_step = round(length_step_value)
    if length_step < 0 or abs(length_step_value - length_step) > 1.0e-6:
        return None
    if geometry_kind == "straight":
        family_index = 0
    else:
        radius = values["planning_support_turn_radius_m"]
        angle = values["planning_support_turn_angle_rad"]
        radius_index = next(
            (
                index for index, multiplier in enumerate(
                    SYNTHETIC_PLANNING_SUPPORT_RADIUS_MULTIPLIERS
                )
                if math.isclose(
                    radius,
                    minimum_turning_radius_m * multiplier,
                    rel_tol=0.0,
                    abs_tol=1.0e-7,
                )
            ),
            None,
        )
        angle_index = next(
            (
                index for index, candidate in enumerate(
                    SYNTHETIC_PLANNING_SUPPORT_TURN_ANGLES_RAD
                )
                if math.isclose(angle, candidate, rel_tol=0.0, abs_tol=1.0e-7)
            ),
            None,
        )
        if radius_index is None or angle_index is None:
            return None
        side_index = 0 if geometry_kind == "left_arc_straight" else (
            1 if geometry_kind == "right_arc_straight" else None
        )
        if side_index is None:
            return None
        family_index = 1 + (radius_index * 8 + angle_index) * 2 + side_index
    expected = (
        length_step * SYNTHETIC_PLANNING_SUPPORT_FAMILIES_PER_LENGTH
        + family_index
    )
    return expected if integers["planning_support_selected_candidate_index"] == expected else None


def _planning_support_chord_deficit_limit(
    analytic_length: float,
    radius: float,
    angle: float,
    sampled_length: float,
    piece_count: int,
) -> float:
    """Bound analytic arc length lost by the deterministic inscribed polyline."""
    if angle <= 0.0:
        return 1.0e-6
    if (
        radius <= 0.0
        or analytic_length <= 0.0
        or sampled_length <= 0.0
        or piece_count <= 0
    ):
        return -1.0
    # The generator chooses the piece count before serializing its 12-digit
    # analytic tags.  Re-applying ceil() to the rounded tag can turn an exact
    # 0.10 m boundary into N+1 pieces and make the valid N-piece chord deficit
    # exceed a spuriously tighter bound.  Use the independently checked OSM
    # piece count and the reconstructed search-grid length instead.
    step = sampled_length / piece_count
    # The extra step covers the one sample interval that can straddle the
    # analytic arc-to-straight seam.  This remains far below the 0.25 m search
    # grid and therefore cannot make a neighbouring length family ambiguous.
    return ((radius * angle + step) * step * step) / (24.0 * radius * radius) + 1.0e-6


def _read_json_object(
    path: Path, report: dict[str, Any], missing_code: str, malformed_code: str
) -> dict[str, Any] | None:
    if not path.is_file():
        _issue(report, missing_code, f"missing named-Route evidence artifact: {path}")
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        _issue(report, malformed_code, f"failed to parse {path.name}: {error}")
        return None
    if not isinstance(value, dict):
        _issue(report, malformed_code, f"{path.name} root must be an object")
        return None
    return value


def _named_route_from_lanelets(
    map_details: dict[str, Any], report: dict[str, Any]
) -> dict[str, Any] | None:
    lanelets = map_details["lanelets"]
    # route_edge_id is ordinary lineage metadata on every generated replay
    # Lanelet. It becomes part of the named-Route contract only after at least
    # one actual named_route_* identity/order tag is present. Treating the
    # lineage tag alone as promotion evidence makes an un-authored replay look
    # like a malformed named Route.
    metadata_detected = any(
        any(key in lanelet["tags"] for key in NAMED_ROUTE_IDENTITY_TAGS)
        for lanelet in lanelets
    )
    if not metadata_detected:
        report["counts"]["named_route_lanelets"] = 0
        report["counts"]["non_named_route_lanelets"] = len(lanelets)
        return None

    named_lanelets = [
        lanelet
        for lanelet in lanelets
        if any(key in lanelet["tags"] for key in NAMED_ROUTE_IDENTITY_TAGS)
    ]
    report["counts"]["named_route_lanelets"] = len(named_lanelets)
    report["counts"]["non_named_route_lanelets"] = len(lanelets) - len(
        named_lanelets
    )
    records: list[dict[str, Any]] = []
    for lanelet in named_lanelets:
        tags = lanelet["tags"]
        missing = [key for key in NAMED_ROUTE_TAGS if key not in tags]
        if missing:
            _issue(
                report,
                "incomplete_named_route_metadata",
                f"lanelet {lanelet['label']} lacks {', '.join(missing)}",
            )
            continue
        route_id = _unsigned_integer(tags["named_route_id"])
        order = _unsigned_integer(tags["named_route_order"])
        edge_id = _unsigned_integer(tags["route_edge_id"])
        source_edge_id = _unsigned_integer(tags.get("source_route_edge_id", tags["route_edge_id"]))
        name = tags["named_route_name"]
        target = tags["named_route_target"]
        if route_id is None or route_id == 0:
            _issue(
                report,
                "invalid_named_route_id",
                f"lanelet {lanelet['label']} has invalid named_route_id",
            )
        if order is None:
            _issue(
                report,
                "invalid_named_route_order",
                f"lanelet {lanelet['label']} has invalid named_route_order",
            )
        if edge_id is None or edge_id == 0:
            _issue(
                report,
                "invalid_named_route_edge_id",
                f"lanelet {lanelet['label']} has invalid route_edge_id",
            )
        if source_edge_id is None or source_edge_id == 0:
            _issue(
                report,
                "invalid_named_route_source_edge_id",
                f"lanelet {lanelet['label']} has invalid source_route_edge_id",
            )
        if not name or len(name.encode("utf-8")) > 256:
            _issue(
                report,
                "invalid_named_route_name",
                f"lanelet {lanelet['label']} has an empty or overlong Route name",
            )
        if target not in ("autoware", "both"):
            _issue(
                report,
                "invalid_named_route_target",
                f"lanelet {lanelet['label']} target {target!r} does not include Autoware",
            )
        if route_id is None or order is None or edge_id is None or source_edge_id is None:
            continue
        records.append(
            {
                "route_id": route_id,
                "name": name,
                "target": target,
                "order": order,
                "edge_id": edge_id,
                "source_edge_id": source_edge_id,
                "lanelet": lanelet,
            }
        )

    result: dict[str, Any] = {"detected": True, "records": []}
    if not records:
        return result
    route_id = records[0]["route_id"]
    name = records[0]["name"]
    target = records[0]["target"]
    for record in records:
        if (
            record["route_id"] != route_id
            or record["name"] != name
            or record["target"] != target
        ):
            _issue(
                report,
                "inconsistent_named_route_identity",
                "all named-Route Lanelets must have one Route id, name, and target",
            )
    ordered = sorted(records, key=lambda record: record["order"])
    orders = [record["order"] for record in ordered]
    edge_ids = [record["edge_id"] for record in ordered]
    source_edge_ids: list[int] = []
    for record in ordered:
        if not source_edge_ids or source_edge_ids[-1] != record["source_edge_id"]:
            source_edge_ids.append(record["source_edge_id"])
    if len(set(source_edge_ids)) != len(source_edge_ids):
        _issue(
            report,
            "named_route_source_edge_reordered",
            "a raw Mission Edge appears in more than one semantic Lanelet run",
        )
    if orders != list(range(len(named_lanelets))):
        _issue(
            report,
            "invalid_named_route_order",
            "named_route_order on the selected subset must be exactly "
            f"0..{max(0, len(named_lanelets) - 1)}, got {orders}",
        )
    if len(set(edge_ids)) != len(edge_ids):
        _issue(report, "duplicate_named_route_edge", "named Route contains duplicate Edge IDs")
    if len(records) != len(named_lanelets):
        _issue(
            report,
            "incomplete_named_route_metadata",
            "every selected named-Route Lanelet must carry complete metadata",
        )
    for predecessor, successor in zip(ordered[:-1], ordered[1:]):
        previous = predecessor["lanelet"]
        following = successor["lanelet"]
        if (
            previous["center_refs"][-1] != following["center_refs"][0]
            or previous["left_refs"][-1] != following["left_refs"][0]
            or previous["right_refs"][-1] != following["right_refs"][0]
        ):
            _issue(
                report,
                "named_route_order_not_a_directed_chain",
                f"named Route order {predecessor['order']} -> {successor['order']} "
                "does not share all directed endpoint node IDs",
            )
    result.update(
        {
            "id": route_id,
            "name": name,
            "target": target,
            "records": ordered,
            "edge_ids": edge_ids,
            "source_edge_ids": source_edge_ids,
        }
    )
    return result


def _audit_named_route_terminal_membership(
    named_route: dict[str, Any],
    map_details: dict[str, Any],
    report: dict[str, Any],
) -> None:
    """Report non-unique Lanelet membership at the named Route terminals."""
    records = named_route.get("records", [])
    if not records:
        return
    first_lanelet = records[0]["lanelet"]
    last_lanelet = records[-1]["lanelet"]
    start = first_lanelet["center"][0]
    goal = last_lanelet["center"][-1]
    memberships: dict[str, list[str]] = {"start": [], "goal": []}
    for lanelet in map_details["lanelets"]:
        polygon = lanelet["left"] + list(reversed(lanelet["right"]))
        prepared = _prepare_polygon_xy(polygon)
        if _point_in_prepared_polygon_xy((start[0], start[1]), prepared):
            memberships["start"].append(lanelet["label"])
        if _point_in_prepared_polygon_xy((goal[0], goal[1]), prepared):
            memberships["goal"].append(lanelet["label"])
    expected = {
        "start": first_lanelet["label"],
        "goal": last_lanelet["label"],
    }
    unexpected = {
        terminal: [
            label for label in labels if label != expected[terminal]
        ]
        for terminal, labels in memberships.items()
    }
    report["named_route_terminal_membership"] = {
        "start_lanelet_ids": memberships["start"],
        "goal_lanelet_ids": memberships["goal"],
        "expected_start_lanelet_id": expected["start"],
        "expected_goal_lanelet_id": expected["goal"],
        "unique": not unexpected["start"] and not unexpected["goal"],
    }
    report["counts"]["named_route_start_lanelet_memberships"] = len(
        memberships["start"]
    )
    report["counts"]["named_route_goal_lanelet_memberships"] = len(
        memberships["goal"]
    )
    if unexpected["start"] or unexpected["goal"]:
        _warning(
            report,
            "ambiguous_named_route_terminal_lanelet_membership",
            "named Route terminal points also lie in other Lanelet polygons; "
            f"start={memberships['start']}, goal={memberships['goal']}. "
            "This static warning does not block staging, but start/goal Lanelet "
            "selection must be checked in the target planner.",
        )


def _read_source_route_graph(
    path: Path, report: dict[str, Any]
) -> dict[int, dict[str, Any]]:
    document = _read_json_object(
        path, report, "missing_named_route_source_graph", "malformed_named_route_source_graph"
    )
    if document is None:
        return {}
    if document.get("type") != "FeatureCollection" or not isinstance(
        document.get("features"), list
    ):
        _issue(
            report,
            "malformed_named_route_source_graph",
            "source Route Graph must be a GeoJSON FeatureCollection",
        )
        return {}
    edges: dict[int, dict[str, Any]] = {}
    for feature in document["features"]:
        if not isinstance(feature, dict):
            continue
        geometry = feature.get("geometry")
        properties = feature.get("properties")
        if not isinstance(geometry, dict) or geometry.get("type") != "MultiLineString":
            continue
        if not isinstance(properties, dict):
            _issue(
                report,
                "malformed_named_route_source_edge",
                "source Route Edge lacks properties",
            )
            continue
        edge_id = _unsigned_integer(properties.get("id"))
        start_id = _unsigned_integer(properties.get("startid"))
        end_id = _unsigned_integer(properties.get("endid"))
        try:
            source_arc_length = float(properties.get("cost"))
        except (TypeError, ValueError):
            source_arc_length = math.nan
        coordinates = geometry.get("coordinates")
        if (
            edge_id is None
            or edge_id == 0
            or start_id is None
            or end_id is None
            or not isinstance(coordinates, list)
            or len(coordinates) != 1
            or not isinstance(coordinates[0], list)
            or len(coordinates[0]) < 2
        ):
            _issue(
                report,
                "malformed_named_route_source_edge",
                f"source Route Edge {properties.get('id')!r} is malformed",
            )
            continue
        points: list[Point] = []
        malformed_point = False
        for coordinate in coordinates[0]:
            if not isinstance(coordinate, list) or len(coordinate) < 2:
                malformed_point = True
                break
            try:
                values = [float(coordinate[0]), float(coordinate[1])]
                if len(coordinate) >= 3:
                    values.append(float(coordinate[2]))
                else:
                    values.append(0.0)
            except (TypeError, ValueError):
                malformed_point = True
                break
            if not all(math.isfinite(value) for value in values):
                malformed_point = True
                break
            points.append((values[0], values[1], values[2]))
        if malformed_point or _polyline_length_xy(points) <= GEOMETRY_EPSILON:
            _issue(
                report,
                "malformed_named_route_source_edge",
                f"source Route Edge {edge_id} has invalid geometry",
            )
            continue
        if not math.isfinite(source_arc_length) or source_arc_length <= 0.0:
            # Backward-compatible static fixtures may predate the serialized
            # RouteEdge::length (`cost`) contract. Generated artifacts always
            # carry it; for legacy XY-only evidence the planar arc is exact.
            source_arc_length = _polyline_length_xyz(points)
        if edge_id in edges:
            _issue(
                report,
                "duplicate_named_route_source_edge",
                f"source Route Graph contains duplicate Edge {edge_id}",
            )
            continue
        metadata = properties.get("metadata")
        passable = isinstance(metadata, dict) and metadata.get("passable") is True
        edges[edge_id] = {
            "start": start_id,
            "end": end_id,
            "points": points,
            "arc_length": source_arc_length,
            "passable": passable,
        }
    if not edges:
        _issue(report, "empty_named_route_source_graph", "source Route Graph has no Edges")
    return edges


def _ordered_replay_signature(
    path: Path,
    report: dict[str, Any],
    *,
    missing_code: str,
    malformed_code: str,
) -> list[tuple[int, int, int, float | None, tuple[tuple[float, ...], ...]]]:
    """Read chronological Edge identity/order without rebuilding topology."""
    document = _read_json_object(path, report, missing_code, malformed_code)
    if document is None:
        return []
    features = document.get("features")
    if document.get("type") != "FeatureCollection" or not isinstance(features, list):
        _issue(report, malformed_code, f"{path.name} is not a GeoJSON FeatureCollection")
        return []
    signature: list[
        tuple[int, int, int, float | None, tuple[tuple[float, ...], ...]]
    ] = []
    seen: set[int] = set()
    for feature in features:
        if not isinstance(feature, dict):
            continue
        geometry = feature.get("geometry")
        properties = feature.get("properties")
        if not isinstance(geometry, dict) or not isinstance(properties, dict):
            continue
        geometry_type = geometry.get("type")
        if geometry_type not in ("LineString", "MultiLineString"):
            continue
        edge_id = _unsigned_integer(properties.get("id"))
        start_id = _unsigned_integer(properties.get("startid"))
        end_id = _unsigned_integer(properties.get("endid"))
        cost: float | None = None
        if "cost" in properties:
            try:
                cost = float(properties["cost"])
            except (TypeError, ValueError):
                cost = math.nan
            if not math.isfinite(cost) or cost <= 0.0:
                _issue(
                    report,
                    malformed_code,
                    f"{path.name} Edge {edge_id} has invalid 3-D arc cost",
                )
                continue
        coordinates = geometry.get("coordinates")
        if geometry_type == "MultiLineString":
            coordinates = (
                coordinates[0]
                if isinstance(coordinates, list) and len(coordinates) == 1
                else None
            )
        if (
            edge_id is None
            or edge_id == 0
            or start_id is None
            or end_id is None
            or edge_id in seen
            or not isinstance(coordinates, list)
            or len(coordinates) < 2
        ):
            _issue(report, malformed_code, f"{path.name} has a malformed replay Edge")
            continue
        points: list[tuple[float, ...]] = []
        malformed = False
        for coordinate in coordinates:
            if not isinstance(coordinate, list) or len(coordinate) < 2:
                malformed = True
                break
            try:
                point = tuple(float(value) for value in coordinate)
            except (TypeError, ValueError):
                malformed = True
                break
            if not all(math.isfinite(value) for value in point):
                malformed = True
                break
            points.append(point)
        if malformed:
            _issue(
                report,
                malformed_code,
                f"{path.name} Edge {edge_id} has malformed geometry",
            )
            continue
        seen.add(edge_id)
        signature.append((edge_id, start_id, end_id, cost, tuple(points)))
    if not signature:
        _issue(report, malformed_code, f"{path.name} has no replay Edges")
    return signature


def _validate_lossless_replay_identity(
    directory: Path, report: dict[str, Any]
) -> None:
    """Forbid Autoware authoring from replacing/resegmenting observed replay."""
    lossless_path = directory / "route_graph_closed_course_replay_candidate.geojson"
    autoware_path = directory / "route_graph_autoware_replay_candidate.geojson"
    if not lossless_path.exists() and not autoware_path.exists():
        return
    report["input"]["lossless_replay_source_graph"] = str(lossless_path)
    report["input"]["autoware_replay_source_graph"] = str(autoware_path)
    if lossless_path.is_file():
        report["input"]["lossless_replay_source_graph_sha256"] = _sha256(lossless_path)
    if autoware_path.is_file():
        report["input"]["autoware_replay_source_graph_sha256"] = _sha256(autoware_path)
    lossless = _ordered_replay_signature(
        lossless_path,
        report,
        missing_code="missing_lossless_replay_source_graph",
        malformed_code="malformed_lossless_replay_source_graph",
    )
    autoware = _ordered_replay_signature(
        autoware_path,
        report,
        missing_code="missing_autoware_replay_source_graph",
        malformed_code="malformed_autoware_replay_source_graph",
    )
    report["counts"]["lossless_replay_edges"] = len(lossless)
    report["counts"]["autoware_replay_edges"] = len(autoware)
    lossless_ids = [record[0] for record in lossless]
    autoware_ids = [record[0] for record in autoware]
    report["metrics"]["lossless_replay_exact_match"] = bool(lossless) and lossless == autoware
    if lossless_ids != autoware_ids:
        _issue(
            report,
            "autoware_replay_shortened_or_reordered",
            "Autoware replay must preserve every chronological Edge ID in exact "
            f"serialized order: lossless={lossless_ids}, autoware={autoware_ids}",
        )
        return
    if [(record[1], record[2]) for record in lossless] != [
        (record[1], record[2]) for record in autoware
    ]:
        _issue(
            report,
            "autoware_replay_topology_changed",
            "Autoware replay changed chronological start/end Node identity",
        )
    if [record[3] for record in lossless] != [record[3] for record in autoware]:
        _issue(
            report,
            "autoware_replay_arc_length_changed",
            "Autoware replay changed chronological RouteEdge 3-D arc cost",
        )
    if [record[4] for record in lossless] != [record[4] for record in autoware]:
        _issue(
            report,
            "autoware_replay_geometry_changed",
            "Autoware replay changed/resegmented chronological centerline geometry",
        )


TERMINAL_SUPPORT_TAGS = (
    "autoware_terminal_support",
    "terminal_support_edge_ids",
    "terminal_support_length_m",
    "named_route_source_length_m",
    "terminal_support_source",
)


def _parse_terminal_support_ids(value: str) -> list[int] | None:
    if not re.fullmatch(r"[1-9][0-9]*(?:,[1-9][0-9]*)*", value):
        return None
    result = [int(part) for part in value.split(",")]
    if len(set(result)) != len(result) or any(item > (1 << 64) - 1 for item in result):
        return None
    return result


def _terminal_support_composite(
    directory: Path,
    records: list[dict[str, Any]],
    source_edges: dict[int, dict[str, Any]],
    named_edge_ids: list[int],
    report: dict[str, Any],
    coverage_distance_m: float,
) -> tuple[int, list[Point]] | None:
    tagged = [
        record
        for record in records
        if any(key in record["lanelet"]["tags"] for key in TERMINAL_SUPPORT_TAGS)
    ]
    if not tagged:
        report["terminal_support"] = {"present": False}
        return None
    if len(tagged) != 1:
        _issue(
            report,
            "invalid_terminal_support_cardinality",
            "exactly one final named-Route Lanelet may carry terminal support",
        )
        return None
    record = tagged[0]
    tags = record["lanelet"]["tags"]
    missing = [key for key in TERMINAL_SUPPORT_TAGS if key not in tags]
    if missing:
        _issue(
            report,
            "incomplete_terminal_support_metadata",
            f"terminal-support Lanelet lacks {', '.join(missing)}",
        )
        return None
    if record is not records[-1] or record["order"] != len(records) - 1:
        _issue(
            report,
            "terminal_support_not_on_final_lanelet",
            "terminal support must extend only the final named-Route Lanelet",
        )
    if tags["autoware_terminal_support"] != "yes" or tags[
        "terminal_support_source"
    ] != "closed_course_semantic_topology":
        _issue(
            report,
            "invalid_terminal_support_provenance",
            "terminal support must be an explicit closed-course semantic-topology extension",
        )
    support_ids = _parse_terminal_support_ids(tags["terminal_support_edge_ids"])
    if not support_ids:
        _issue(
            report,
            "invalid_terminal_support_edge_ids",
            "terminal_support_edge_ids must be a non-empty ordered uint64 list",
        )
        return None
    try:
        tagged_support_length = float(tags["terminal_support_length_m"])
        tagged_named_length = float(tags["named_route_source_length_m"])
    except ValueError:
        tagged_support_length = math.nan
        tagged_named_length = math.nan
    if not math.isfinite(tagged_support_length) or tagged_support_length <= 0.0:
        _issue(
            report,
            "invalid_terminal_support_length",
            "terminal_support_length_m must be finite and positive",
        )
    if not math.isfinite(tagged_named_length) or tagged_named_length <= 0.0:
        _issue(
            report,
            "invalid_terminal_named_source_length",
            "named_route_source_length_m must be finite and positive",
        )

    topology_path = directory / "route_graph_closed_course_topology_candidate.geojson"
    report["input"]["terminal_support_source_graph"] = str(topology_path)
    topology_edges = _read_source_route_graph(topology_path, report)
    if not topology_edges:
        return None
    final_edge_id = record["edge_id"]
    final_source = source_edges.get(final_edge_id)
    topology_final = topology_edges.get(final_edge_id)
    if final_source is None or topology_final is None:
        _issue(
            report,
            "terminal_support_missing_named_tail",
            f"terminal support source lacks named final Edge {final_edge_id}",
        )
        return None
    final_source_coverage, _, _ = _point_to_segment_coverage(
        final_source["points"],
        list(zip(topology_final["points"][:-1], topology_final["points"][1:])),
        coverage_distance_m,
    )
    topology_final_coverage, _, _ = _point_to_segment_coverage(
        topology_final["points"],
        list(zip(final_source["points"][:-1], final_source["points"][1:])),
        coverage_distance_m,
    )
    if min(final_source_coverage, topology_final_coverage) + 1.0e-12 < 0.99:
        _issue(
            report,
            "terminal_support_named_tail_mismatch",
            "topology source does not reproduce the named final Edge geometry",
        )

    named_ids = set(named_edge_ids)
    visited = set(named_edge_ids)
    expected_parts = [final_source["points"]]
    current_node = final_source["end"]
    support_length = 0.0
    maximum_join_heading_change_deg = 0.0
    previous_points = final_source["points"]
    for index, support_id in enumerate(support_ids):
        support = topology_edges.get(support_id)
        if support is None:
            _issue(
                report,
                "terminal_support_edge_missing",
                f"terminal support source lacks Edge {support_id}",
            )
            return None
        if support_id in visited or support_id in named_ids:
            _issue(
                report,
                "terminal_support_edge_reused",
                f"terminal support Edge {support_id} overlaps or cycles into the named Route",
            )
        if not support["passable"]:
            _issue(
                report,
                "terminal_support_edge_not_passable",
                f"terminal support Edge {support_id} is not passable in semantic topology",
            )
        if support["start"] != current_node:
            _issue(
                report,
                "terminal_support_not_a_directed_chain",
                f"terminal support Edge {support_id} does not start at node {current_node}",
            )
        outgoing = sorted(
            edge_id
            for edge_id, edge in topology_edges.items()
            if edge["start"] == current_node
            and edge["passable"]
            and edge_id not in visited
        )
        if outgoing != [support_id]:
            _issue(
                report,
                "terminal_support_successor_not_unique",
                f"terminal support step {index} expected unique Edge {support_id}, got {outgoing}",
            )
        previous_tangent = _directed_endpoint_tangent_xy(
            previous_points,
            at_end=True,
            minimum_span_m=TERMINAL_SUPPORT_TANGENT_SPAN_M,
        )
        support_tangent = _directed_endpoint_tangent_xy(
            support["points"],
            at_end=False,
            minimum_span_m=TERMINAL_SUPPORT_TANGENT_SPAN_M,
        )
        if previous_tangent is None or support_tangent is None:
            _issue(
                report,
                "terminal_support_tangent_invalid",
                f"terminal support Edge {support_id} has no stable directed endpoint tangent",
            )
        else:
            heading_change_deg = _heading_change_deg(
                previous_tangent, support_tangent
            )
            maximum_join_heading_change_deg = max(
                maximum_join_heading_change_deg, heading_change_deg
            )
            if (
                heading_change_deg
                > TERMINAL_SUPPORT_MAXIMUM_HEADING_JUMP_DEG + 1.0e-9
            ):
                _issue(
                    report,
                    "terminal_support_heading_discontinuity",
                    f"terminal support Edge {support_id} changes heading by "
                    f"{heading_change_deg:.6f} degrees",
                )
        expected_parts.append(support["points"])
        support_length += _polyline_length_xy(support["points"])
        current_node = support["end"]
        visited.add(support_id)
        previous_points = support["points"]
    try:
        composite = _joined_polyline(expected_parts)
    except ValueError as error:
        _issue(report, "terminal_support_geometry_gap", str(error))
        return None
    named_length = _polyline_length_xy(final_source["points"])
    length_tolerance = max(1.0e-6, 1.0e-6 * (named_length + support_length))
    if abs(tagged_support_length - support_length) > length_tolerance:
        _issue(
            report,
            "terminal_support_length_mismatch",
            f"tagged support length {tagged_support_length:.9f} differs from "
            f"source {support_length:.9f}",
        )
    if abs(tagged_named_length - named_length) > length_tolerance:
        _issue(
            report,
            "terminal_named_source_length_mismatch",
            f"tagged named length {tagged_named_length:.9f} differs from "
            f"source {named_length:.9f}",
        )
    report["terminal_support"] = {
        "present": True,
        "final_named_edge_id": final_edge_id,
        "support_edge_ids": support_ids,
        "support_length_m": support_length,
        "named_route_source_length_m": named_length,
        "composite_length_m": _polyline_length_xy(composite),
        "source": "closed_course_semantic_topology",
        "source_graph_sha256": _sha256(topology_path),
        "support_is_part_of_named_route": False,
        "maximum_join_heading_change_deg": maximum_join_heading_change_deg,
        "maximum_allowed_join_heading_change_deg": (
            TERMINAL_SUPPORT_MAXIMUM_HEADING_JUMP_DEG
        ),
        "tangent_span_m": TERMINAL_SUPPORT_TANGENT_SPAN_M,
    }
    return final_edge_id, composite


def _validate_authored_stop_lines(
    named_route: dict[str, Any],
    authored_route: dict[str, Any],
    authoring: dict[str, Any],
    map_details: dict[str, Any],
    report: dict[str, Any],
    coverage_distance_m: float,
) -> None:
    raw_stops = authoring.get("stop_lines", [])
    if not isinstance(raw_stops, list):
        _issue(report, "malformed_navigation_authoring", "stop_lines must be an array")
        raw_stops = []
    raw_authored_edge_ids = authored_route.get("ordered_edge_ids", [])
    authored_edge_ids = {
        edge_id
        for value in raw_authored_edge_ids
        if (edge_id := _unsigned_integer(value)) not in (None, 0)
    } if isinstance(raw_authored_edge_ids, list) else set()
    expected: dict[int, dict[str, Any]] = {}
    for stop in raw_stops:
        if not isinstance(stop, dict) or stop.get("target") not in ("autoware", "both"):
            continue
        stop_id = _unsigned_integer(stop.get("id"))
        edge_id = _unsigned_integer(stop.get("edge_id"))
        if edge_id not in authored_edge_ids:
            continue
        if stop_id is None or stop_id == 0 or stop_id in expected:
            _issue(
                report,
                "invalid_authored_stop_line",
                f"selected Route has invalid or duplicate stop-line id {stop.get('id')!r}",
            )
            continue
        expected[stop_id] = stop

    ways: dict[int, tuple[int, ...]] = map_details["ways"]
    nodes: dict[int, Point | None] = map_details["nodes"]
    way_tags: dict[int, dict[str, str]] = map_details["way_tags"]
    actual_ways: dict[int, list[int]] = {}
    for way_id, tags in way_tags.items():
        if tags.get("audit_source") != "navigation_authoring_gui" and \
                "authored_stop_line_id" not in tags:
            continue
        stop_id = _unsigned_integer(tags.get("authored_stop_line_id"))
        if stop_id is None or stop_id == 0:
            _issue(
                report,
                "invalid_authored_stop_line_way",
                f"authored stop-line Way {way_id} lacks a valid authored_stop_line_id",
            )
            continue
        actual_ways.setdefault(stop_id, []).append(way_id)

    actual_relations: dict[int, list[int]] = {}
    for relation_id, relation in map_details["relations"].items():
        tags = relation["tags"]
        if tags.get("audit_source") != "navigation_authoring_gui" and \
                "authored_stop_line_id" not in tags:
            continue
        stop_id = _unsigned_integer(tags.get("authored_stop_line_id"))
        if stop_id is None or stop_id == 0:
            _issue(
                report,
                "invalid_authored_stop_regulatory_element",
                f"authored regulatory relation {relation_id} lacks a valid stop-line id",
            )
            continue
        actual_relations.setdefault(stop_id, []).append(relation_id)

    if set(actual_ways) != set(expected) or set(actual_relations) != set(expected):
        _issue(
            report,
            "authored_stop_line_set_mismatch",
            "exported authored stop-line Ways/Regulatory Elements do not exactly match "
            "the selected Route request",
        )

    for stop_id, stop in expected.items():
        stop_way_ids = actual_ways.get(stop_id, [])
        regulatory_ids = actual_relations.get(stop_id, [])
        if len(stop_way_ids) != 1 or len(regulatory_ids) != 1:
            _issue(
                report,
                "authored_stop_line_cardinality",
                f"stop line {stop_id} requires exactly one Way and one Regulatory Element",
            )
            continue
        way_id = stop_way_ids[0]
        regulatory_id = regulatory_ids[0]
        tags = way_tags[way_id]
        relation = map_details["relations"][regulatory_id]
        relation_tags = relation["tags"]
        if (
            tags.get("type") != "stop_line"
            or tags.get("subtype") != "solid"
            or tags.get("name") != stop.get("name")
            or tags.get("virtual") != "yes"
            or tags.get("autogenerated") != "yes"
            or tags.get("audit_source") != "navigation_authoring_gui"
            or tags.get("physical_stop_sign_observed") != "no"
            or tags.get("navigation_target") != stop.get("target")
        ):
            _issue(
                report,
                "invalid_authored_stop_line_way",
                f"stop line {stop_id} Way lacks required name/virtual audit semantics",
            )
        if (
            relation_tags.get("type") != "regulatory_element"
            or relation_tags.get("subtype") != "traffic_sign"
            or relation_tags.get("sign_type") != "stop_sign"
            or relation_tags.get("name") != stop.get("name")
            or relation_tags.get("virtual") != "yes"
            or relation_tags.get("autogenerated") != "yes"
            or relation_tags.get("audit_source") != "navigation_authoring_gui"
            or relation_tags.get("physical_stop_sign_observed") != "no"
        ):
            _issue(
                report,
                "invalid_authored_stop_regulatory_element",
                f"stop line {stop_id} is not a virtual TrafficSign(stop_sign)",
            )
        ref_lines = [
            reference
            for member_type, reference, role in relation["members"]
            if member_type == "way" and role == "ref_line"
        ]
        if ref_lines != [way_id]:
            _issue(
                report,
                "invalid_authored_stop_ref_line",
                f"stop line {stop_id} Regulatory Element must reference its Way as ref_line",
            )
        affected_lanelets = [
            lanelet
            for lanelet in named_route["records"]
            if regulatory_id in lanelet["lanelet"]["regulatory_relations"]
        ]
        if len(affected_lanelets) != 1:
            _issue(
                report,
                "invalid_authored_stop_lanelet_reference",
                f"stop line {stop_id} must be referenced by exactly one selected Lanelet",
            )
        stop_points = _way_points(way_id, ways, nodes)
        if len(stop_points) != 2 or _polyline_length_xy(stop_points) <= GEOMETRY_EPSILON:
            _issue(
                report,
                "invalid_authored_stop_line_geometry",
                f"stop line {stop_id} has missing or degenerate geometry",
            )
            continue
        try:
            width = float(stop.get("width_m"))
            authored_s = float(stop.get("s"))
            anchor_values = stop.get("anchor")
            anchor = (
                float(anchor_values[0]),
                float(anchor_values[1]),
                float(anchor_values[2]),
            )
        except (TypeError, ValueError, IndexError):
            _issue(
                report,
                "invalid_authored_stop_line",
                f"stop line {stop_id} has malformed width or anchor",
            )
            continue
        if (
            not math.isfinite(width)
            or width <= 0.0
            or not math.isfinite(authored_s)
            or authored_s < 0.0
            or not all(math.isfinite(value) for value in anchor)
        ):
            _issue(
                report,
                "invalid_authored_stop_line",
                f"stop line {stop_id} has non-finite width or anchor",
            )
            continue
        exported_width = _polyline_length_xy(stop_points)
        try:
            tagged_width = float(tags.get("authored_width_m", "nan"))
            tagged_s = float(tags.get("route_edge_s_m", "nan"))
        except ValueError:
            tagged_width = math.nan
            tagged_s = math.nan
        if (
            not math.isfinite(tagged_width)
            or abs(tagged_width - width) > max(1.0e-9, 1.0e-9 * width)
            or not math.isfinite(tagged_s)
            or tagged_s < 0.0
        ):
            _issue(
                report,
                "invalid_authored_stop_line_audit_geometry",
                f"stop line {stop_id} has invalid persisted width/s audit tags",
            )
        if abs(exported_width - width) > max(1.0e-6, 1.0e-6 * width):
            _issue(
                report,
                "authored_stop_line_width_mismatch",
                f"stop line {stop_id} width {exported_width:.9f} differs from {width:.9f}",
            )
        midpoint = _point_at_fraction(stop_points, 0.5)
        if _point_distance_xy(midpoint, anchor) > coverage_distance_m:
            _issue(
                report,
                "authored_stop_line_anchor_mismatch",
                f"stop line {stop_id} midpoint is not near its independent authored anchor",
            )
        if len(affected_lanelets) == 1:
            affected = affected_lanelets[0]
            exported_edge_id = _unsigned_integer(tags.get("route_edge_id"))
            if exported_edge_id != affected["edge_id"]:
                _issue(
                    report,
                    "authored_stop_line_route_order_mismatch",
                    f"stop line {stop_id} Edge does not match affected named Route order "
                    f"{affected['order']}",
                )
            affected_tags = affected["lanelet"]["tags"]
            if "source_route_edge_id" in affected_tags:
                source_edge_id = _unsigned_integer(tags.get("source_route_edge_id"))
                try:
                    source_s = float(tags.get("source_route_edge_s_m", "nan"))
                except ValueError:
                    source_s = math.nan
                expected_source_edge_id = _unsigned_integer(stop.get("edge_id"))
                if (
                    source_edge_id != expected_source_edge_id
                    or not math.isfinite(source_s)
                    or abs(source_s - authored_s)
                    > max(1.0e-7, 1.0e-7 * max(1.0, authored_s))
                ):
                    _issue(
                        report,
                        "authored_stop_line_source_lineage_mismatch",
                        f"stop line {stop_id} does not preserve its raw source Edge/s",
                    )
            attached_center = affected["lanelet"]["center"]
            attached_segments = list(zip(attached_center[:-1], attached_center[1:]))
            if not attached_segments or min(
                _point_segment_distance_squared_xy(anchor, segment)
                for segment in attached_segments
            ) > coverage_distance_m * coverage_distance_m:
                _issue(
                    report,
                    "authored_stop_line_affected_lanelet_mismatch",
                    f"stop line {stop_id} anchor is not on its affected named Route Lanelet",
                )
            directed_segments = [
                segment
                for segment in attached_segments
                if _point_distance_xy(segment[0], segment[1]) > GEOMETRY_EPSILON
            ]
            if directed_segments:
                nearest_segment = min(
                    directed_segments,
                    key=lambda segment: _point_segment_distance_squared_xy(midpoint, segment),
                )
                tangent_x = nearest_segment[1][0] - nearest_segment[0][0]
                tangent_y = nearest_segment[1][1] - nearest_segment[0][1]
                stop_x = stop_points[-1][0] - stop_points[0][0]
                stop_y = stop_points[-1][1] - stop_points[0][1]
                tangent_length = math.hypot(tangent_x, tangent_y)
                stop_length = math.hypot(stop_x, stop_y)
                alignment = abs(tangent_x * stop_x + tangent_y * stop_y) / (
                    tangent_length * stop_length
                )
                if alignment > 0.25:
                    _issue(
                        report,
                        "authored_stop_line_not_transverse",
                        f"stop line {stop_id} is not transverse to named Route order "
                        f"{affected['order']}",
                    )
    report["counts"]["authored_stop_lines"] = len(expected)
    report["counts"]["authored_stop_regulatory_elements"] = sum(
        len(values) for values in actual_relations.values()
    )


def _validate_full_map_source(
    directory: Path,
    map_details: dict[str, Any],
    report: dict[str, Any],
    minimum_coverage: float,
    coverage_distance_m: float,
    source_path: Path | None = None,
) -> dict[int, dict[str, Any]]:
    """Hard-gate selected raw graph coverage, allowing lossless Lanelet splits."""
    full_map_source_path = source_path or (
        directory / "route_graph_autoware_replay_candidate.geojson"
    )
    report["input"]["full_map_source_graph"] = str(full_map_source_path)
    if full_map_source_path.is_file():
        report["input"]["full_map_source_graph_sha256"] = _sha256(full_map_source_path)
    full_map_edges = _read_source_route_graph(full_map_source_path, report)
    lanelets_by_source: dict[int, list[dict[str, Any]]] = {}
    output_edge_ids: set[int] = set()
    serialized_source_runs: list[int] = []
    previous_source_id: int | None = None
    lineage_keys = (
        "source_route_edge_id",
        "source_start_s_m",
        "source_end_s_m",
        "source_edge_length_m",
    )
    for lanelet in map_details["lanelets"]:
        if _is_synthetic_planning_support(lanelet):
            continue
        tags = lanelet["tags"]
        output_edge_id = _unsigned_integer(tags.get("route_edge_id"))
        if output_edge_id is None or output_edge_id == 0:
            _issue(
                report,
                "invalid_full_map_route_edge_id",
                f"lanelet {lanelet['label']} has no valid route_edge_id lineage",
            )
            continue
        if output_edge_id in output_edge_ids:
            _issue(
                report,
                "duplicate_full_map_output_edge_id",
                f"Lanelet output Edge ID {output_edge_id} is duplicated",
            )
        output_edge_ids.add(output_edge_id)
        lineage_present = [key in tags for key in lineage_keys]
        if any(lineage_present) and not all(lineage_present):
            _issue(
                report,
                "incomplete_semantic_source_lineage",
                f"lanelet {lanelet['label']} must carry all semantic source lineage tags",
            )
            continue
        source_edge_id = (
            _unsigned_integer(tags.get("source_route_edge_id"))
            if all(lineage_present)
            else output_edge_id
        )
        if source_edge_id is None or source_edge_id == 0:
            _issue(
                report,
                "invalid_semantic_source_edge_id",
                f"lanelet {lanelet['label']} has invalid source_route_edge_id",
            )
            continue
        start_s: float | None = None
        end_s: float | None = None
        tagged_source_length: float | None = None
        if all(lineage_present):
            try:
                start_s = float(tags["source_start_s_m"])
                end_s = float(tags["source_end_s_m"])
                tagged_source_length = float(tags["source_edge_length_m"])
            except ValueError:
                pass
            if (
                start_s is None
                or end_s is None
                or not math.isfinite(start_s)
                or not math.isfinite(end_s)
                or start_s < 0.0
                or end_s <= start_s
                or tagged_source_length is None
                or not math.isfinite(tagged_source_length)
                or tagged_source_length <= 0.0
            ):
                _issue(
                    report,
                    "invalid_semantic_source_interval",
                    f"lanelet {lanelet['label']} has invalid source arc interval",
                )
                continue
            try:
                generator_speed = float(tags.get("generator_speed_limit_mps", "nan"))
            except ValueError:
                generator_speed = math.nan
            lanelet_speed = _parse_speed_mps(tags.get("speed_limit", ""))
            if (
                not math.isfinite(generator_speed)
                or generator_speed <= 0.0
                or lanelet_speed is None
                or abs(generator_speed - lanelet_speed)
                > max(1.0e-9, 1.0e-9 * generator_speed)
            ):
                _issue(
                    report,
                    "semantic_effective_speed_mismatch",
                    f"lanelet {lanelet['label']} has inconsistent effective speed tags",
                )
        lanelet["source_edge_id"] = source_edge_id
        lanelet["source_start_s"] = start_s
        lanelet["source_end_s"] = end_s
        lanelet["source_edge_length"] = tagged_source_length
        lanelets_by_source.setdefault(source_edge_id, []).append(lanelet)
        if source_edge_id != previous_source_id:
            serialized_source_runs.append(source_edge_id)
            previous_source_id = source_edge_id
    report["counts"]["full_map_source_edges"] = len(full_map_edges)
    report["counts"]["full_map_lanelet_edges"] = len(lanelets_by_source)
    report["counts"]["full_map_lanelet_segments"] = sum(
        len(matches) for matches in lanelets_by_source.values()
    )
    report["counts"]["synthetic_planning_support_lanelets"] = sum(
        _is_synthetic_planning_support(lanelet)
        for lanelet in map_details["lanelets"]
    )
    if set(full_map_edges) != set(lanelets_by_source):
        _issue(
            report,
            "full_map_source_edge_set_mismatch",
            "full-map source and exported Lanelet Edge sets differ: "
            f"source={sorted(full_map_edges)}, exported={sorted(lanelets_by_source)}",
        )
    source_order = list(full_map_edges)
    if serialized_source_runs != source_order:
        _issue(
            report,
            "semantic_source_edge_order_mismatch",
            "Lanelet source Edge runs must retain exact raw serialized order: "
            f"source={source_order}, lanelets={serialized_source_runs}",
        )

    minimum_full_source_coverage = 1.0
    minimum_full_lanelet_coverage = 1.0
    maximum_length_delta = 0.0
    maximum_3d_arc_length_delta = 0.0
    full_edge_comparisons = 0
    for edge_id, full_edge in full_map_edges.items():
        matches = lanelets_by_source.get(edge_id, [])
        if not matches:
            continue
        source_points = full_edge["points"]
        source_arc_length = float(full_edge["arc_length"])
        lineage_mode = all(
            lanelet.get("source_start_s") is not None
            and lanelet.get("source_end_s") is not None
            for lanelet in matches
        )
        if not lineage_mode and len(matches) != 1:
            _issue(
                report,
                "duplicate_full_map_route_edge",
                f"legacy full-map Edge {edge_id} must export exactly one Lanelet",
            )
            continue
        expected_start_s = 0.0
        if lineage_mode:
            for lanelet in matches:
                start_s = float(lanelet["source_start_s"])
                end_s = float(lanelet["source_end_s"])
                tolerance = max(1.0e-7, 1.0e-7 * source_arc_length)
                if abs(float(lanelet["source_edge_length"]) - source_arc_length) > tolerance:
                    _issue(
                        report,
                        "semantic_source_length_mismatch",
                        f"source Edge {edge_id} tagged L differs from raw RouteEdge cost",
                    )
                if abs(start_s - expected_start_s) > tolerance:
                    _issue(
                        report,
                        "semantic_source_interval_gap_or_overlap",
                        f"source Edge {edge_id} expected interval start "
                        f"{expected_start_s:.12g}, got {start_s:.12g}",
                    )
                if end_s > source_arc_length + tolerance:
                    _issue(
                        report,
                        "semantic_source_interval_out_of_range",
                        f"source Edge {edge_id} interval ends beyond L={source_arc_length:.12g}",
                    )
                expected_start_s = end_s
            if abs(expected_start_s - source_arc_length) > max(
                1.0e-7, 1.0e-7 * source_arc_length
            ):
                _issue(
                    report,
                    "semantic_source_interval_incomplete",
                    f"source Edge {edge_id} intervals end at {expected_start_s:.12g}, "
                    f"not L={source_arc_length:.12g}",
                )
        try:
            lanelet_points = _joined_polyline(lanelet["center"] for lanelet in matches)
        except ValueError as error:
            _issue(
                report,
                "semantic_source_reconstruction_disconnected",
                f"source Edge {edge_id}: {error}",
            )
            continue
        lanelet_segments = list(zip(lanelet_points[:-1], lanelet_points[1:]))
        source_segments = list(zip(source_points[:-1], source_points[1:]))
        source_coverage, _, _ = _point_to_segment_coverage(
            source_points, lanelet_segments, coverage_distance_m
        )
        lanelet_coverage, _, _ = _point_to_segment_coverage(
            lanelet_points, source_segments, coverage_distance_m
        )
        minimum_full_source_coverage = min(
            minimum_full_source_coverage, source_coverage
        )
        terminal_composite = any(
            key in lanelet["tags"]
            for lanelet in matches
            for key in TERMINAL_SUPPORT_TAGS
        )
        if not terminal_composite:
            minimum_full_lanelet_coverage = min(
                minimum_full_lanelet_coverage, lanelet_coverage
            )
            source_length = _polyline_length_xy(source_points)
            lanelet_length = _polyline_length_xy(lanelet_points)
            length_delta = abs(source_length - lanelet_length)
            maximum_length_delta = max(maximum_length_delta, length_delta)
            length_tolerance = max(1.0e-6, 1.0e-6 * source_length)
            if length_delta > length_tolerance:
                _issue(
                    report,
                    "full_map_edge_length_changed",
                    f"Lanelet segments changed full-map Edge {edge_id} "
                    f"arc length by {length_delta:.9g} m",
                )
            # GeoJSON deliberately remains planar for browser tooling, while
            # RouteEdge::length is serialized as `cost`.  OSM `ele` therefore
            # provides the independent 3-D reconstruction needed to prove a
            # semantic split neither shortens nor stretches the source arc.
            lanelet_3d_arc_length = _polyline_length_xyz(lanelet_points)
            arc_length_delta = abs(lanelet_3d_arc_length - source_arc_length)
            maximum_3d_arc_length_delta = max(
                maximum_3d_arc_length_delta, arc_length_delta
            )
            arc_length_tolerance = max(1.0e-7, 1.0e-7 * source_arc_length)
            if arc_length_delta > arc_length_tolerance:
                _issue(
                    report,
                    "full_map_edge_3d_arc_length_changed",
                    f"Lanelet segments changed full-map Edge {edge_id} 3-D arc "
                    f"length by {arc_length_delta:.9g} m",
                )
            if (
                _point_distance_xy(source_points[0], lanelet_points[0]) > 1.0e-6
                or _point_distance_xy(source_points[-1], lanelet_points[-1]) > 1.0e-6
            ):
                _issue(
                    report,
                    "full_map_edge_orientation_mismatch",
                    f"Lanelet segments do not preserve exact endpoints "
                    f"of full-map Edge {edge_id}",
                )
        full_edge_comparisons += 1
    if full_edge_comparisons == 0:
        minimum_full_source_coverage = 0.0
        minimum_full_lanelet_coverage = 0.0
    report["metrics"]["full_map_source_centerline_coverage"] = (
        minimum_full_source_coverage
    )
    report["metrics"]["full_map_lanelet_centerline_source_coverage"] = (
        minimum_full_lanelet_coverage
    )
    report["metrics"]["full_map_maximum_centerline_length_delta_m"] = (
        maximum_length_delta
    )
    report["metrics"]["full_map_maximum_3d_arc_length_delta_m"] = (
        maximum_3d_arc_length_delta
    )
    if min(
        minimum_full_source_coverage, minimum_full_lanelet_coverage
    ) + 1.0e-12 < minimum_coverage:
        _issue(
            report,
            "insufficient_full_map_source_coverage",
            "bidirectional full-map Lanelet/source coverage "
            f"{min(minimum_full_source_coverage, minimum_full_lanelet_coverage):.6f} "
            f"is below {minimum_coverage:.6f}",
        )
    return full_map_edges


def _validate_synthetic_open_route_planning_support(
    map_path: Path,
    map_details: dict[str, Any],
    full_map_edges: dict[int, dict[str, Any]],
    named_route: dict[str, Any] | None,
    report: dict[str, Any],
) -> None:
    """Audit synthetic head/tail context without counting it as raw replay."""
    lanelets = map_details["lanelets"]
    support_lanelets = [
        lanelet for lanelet in lanelets if _is_synthetic_planning_support(lanelet)
    ]
    raw_lanelets = [
        lanelet for lanelet in lanelets if not _is_synthetic_planning_support(lanelet)
    ]
    source_ids = list(full_map_edges)
    if not support_lanelets:
        report["planning_support"] = {"present": False}
        return
    # Generated Vector Maps must contain only measured or explicitly authored
    # route geometry.  Legacy head/tail staging Lanelets are unobserved,
    # passable geometry and are rejected regardless of whether the map has no
    # selected Mission or selects the exact complete open replay.  Continue the
    # detailed legacy audit below so a rejected artifact remains diagnosable.
    _issue(
        report,
        "unobserved_synthetic_open_route_planning_support",
        "the Lanelet2 map contains forbidden unobserved synthetic head/tail "
        "planning-support geometry",
    )
    if len(support_lanelets) != 2:
        _issue(
            report,
            "invalid_synthetic_planning_support_cardinality",
            f"expected exactly one head and one tail Lanelet, got {len(support_lanelets)}",
        )

    for lanelet in lanelets:
        raw_count = lanelet["tags"].get("synthetic_planning_support_count")
        try:
            tagged_count = int(raw_count) if raw_count is not None else None
        except ValueError:
            tagged_count = None
        if tagged_count != len(support_lanelets):
            _issue(
                report,
                "inconsistent_synthetic_planning_support_count",
                f"lanelet {lanelet['label']} declares support count {raw_count!r}, "
                f"expected {len(support_lanelets)}",
            )

    by_role: dict[str, dict[str, Any]] = {}
    canonical_records: list[dict[str, Any]] = []
    role_evidence: dict[str, dict[str, Any]] = {}
    for lanelet in support_lanelets:
        tags = lanelet["tags"]
        missing = [key for key in SYNTHETIC_PLANNING_SUPPORT_TAGS if key not in tags]
        if missing:
            _issue(
                report,
                "incomplete_synthetic_planning_support_metadata",
                f"support lanelet {lanelet['label']} lacks {', '.join(missing)}",
            )
            continue
        role = tags["planning_support_role"]
        if role not in ("head", "tail") or role in by_role:
            _issue(
                report,
                "invalid_synthetic_planning_support_roles",
                f"support lanelet {lanelet['label']} has duplicate/invalid role {role!r}",
            )
            continue
        by_role[role] = lanelet
        if (
            tags["synthetic_planning_support"] != "yes"
            or tags["synthetic_test_staging"] != "yes"
            or tags["surveyed"] != "no"
            or tags["deployment_ready"] != "no"
            or tags["support_is_part_of_raw_counts"] != "no"
            or tags["support_is_part_of_named_route"] != "no"
            or tags["support_is_raw_coverage"] != "no"
            or tags["planning_support_contract_version"]
            != str(SYNTHETIC_PLANNING_SUPPORT_CONTRACT_VERSION)
            or tags["planning_support_source"]
            != SYNTHETIC_PLANNING_SUPPORT_SOURCE
            or tags["planning_support_estimated"] != "yes"
            or tags.get("provenance")
            != "synthetic_test_kinematic_staging"
            or tags.get("observed_driven") != "no"
            or tags.get("validation_status")
            != "estimated_synthetic_test_staging"
            or tags.get("production_ready") != "no"
            or tags.get("physical_boundaries_verified") != "no"
            or tags["planning_support_kinematic_valid"] != "yes"
            or tags["planning_support_outer_endpoint_unique"] != "yes"
            or tags["planning_support_raw_overlap_single_transition"] != "yes"
            or tags["planning_support_outer_footprint_contained"] != "yes"
            or tags["planning_support_connection_footprint_contained"] != "yes"
            or tags["planning_support_outer_pose_isolation_scope"]
            != SYNTHETIC_PLANNING_SUPPORT_OUTER_POSE_ISOLATION_SCOPE
            or tags["planning_support_outer_pose_isolation_derivation"]
            != SYNTHETIC_PLANNING_SUPPORT_OUTER_POSE_ISOLATION_DERIVATION
            or tags["planning_support_collision_scope"]
            != "route_polygon_single_transition_only"
        ):
            _issue(
                report,
                "invalid_synthetic_planning_support_provenance",
                f"support lanelet {lanelet['label']} has unsupported provenance/status",
            )
        forbidden = (
            "source_route_edge_id",
            "source_start_s_m",
            "source_end_s_m",
            "source_edge_length_m",
            *NAMED_ROUTE_IDENTITY_TAGS,
        )
        if any(key in tags for key in forbidden):
            _issue(
                report,
                "synthetic_planning_support_claims_raw_or_named_membership",
                f"support lanelet {lanelet['label']} must not count as raw/named Route data",
            )
        canonical_records.append(
            {
                "relation_id": lanelet["relation_id"],
                "route_edge_id": tags.get("route_edge_id"),
                "tags": {key: tags[key] for key in sorted(SYNTHETIC_PLANNING_SUPPORT_TAGS)},
                "center_refs": list(lanelet["center_refs"]),
                "left_refs": list(lanelet["left_refs"]),
                "right_refs": list(lanelet["right_refs"]),
                "center": lanelet["center"],
                "left": lanelet["left"],
                "right": lanelet["right"],
            }
        )

    if set(by_role) != {"head", "tail"}:
        _issue(
            report,
            "invalid_synthetic_planning_support_roles",
            f"support roles must be exactly head/tail, got {sorted(by_role)}",
        )

    raw_output_ids = {
        _unsigned_integer(lanelet["tags"].get("route_edge_id"))
        for lanelet in raw_lanelets
    }
    for role, support in by_role.items():
        tags = support["tags"]
        adjacent = raw_lanelets[0] if role == "head" and raw_lanelets else (
            raw_lanelets[-1] if raw_lanelets else None
        )
        if adjacent is None or not source_ids:
            _issue(
                report,
                "synthetic_planning_support_missing_raw_endpoint",
                f"{role} support has no adjacent raw Lanelet",
            )
            continue
        source_id = source_ids[0] if role == "head" else source_ids[-1]
        source = full_map_edges[source_id]
        raw_endpoint = adjacent["center"][0 if role == "head" else -1]
        expected_tangent = _directed_endpoint_tangent_xy(
            source["points"],
            at_end=role == "tail",
            minimum_span_m=TERMINAL_SUPPORT_TANGENT_SPAN_M,
        )
        integer_keys = ("route_edge_id", *SYNTHETIC_PLANNING_SUPPORT_INTEGER_TAGS)
        integers = {key: _unsigned_integer(tags.get(key)) for key in integer_keys}
        identity_keys = (
            "route_edge_id",
            "planning_support_adjacent_output_edge_id",
            "planning_support_adjacent_source_edge_id",
            "planning_support_raw_endpoint_node_id",
        )
        if (
            any(value is None for value in integers.values())
            or any(integers[key] == 0 for key in identity_keys)
        ):
            _issue(
                report,
                "invalid_synthetic_planning_support_ids",
                f"{role} support has malformed uint64 IDs/counts",
            )
            continue
        integers = {key: int(value) for key, value in integers.items()}
        if integers["route_edge_id"] in raw_output_ids:
            _issue(
                report,
                "synthetic_planning_support_edge_id_collision",
                f"{role} support reuses a raw output Edge ID",
            )
        expected_adjacent_output = _unsigned_integer(
            adjacent["tags"].get("route_edge_id")
        )
        expected_endpoint_node = source["start"] if role == "head" else source["end"]
        if (
            integers["planning_support_adjacent_output_edge_id"]
            != expected_adjacent_output
            or integers["planning_support_adjacent_source_edge_id"] != source_id
            or integers["planning_support_raw_endpoint_node_id"]
            != expected_endpoint_node
        ):
            _issue(
                report,
                "synthetic_planning_support_lineage_mismatch",
                f"{role} support is not bound to the exact raw endpoint",
            )
        if role == "head":
            shares_endpoint = (
                support["center_refs"][-1] == adjacent["center_refs"][0]
                and support["left_refs"][-1] == adjacent["left_refs"][0]
                and support["right_refs"][-1] == adjacent["right_refs"][0]
            )
            synthetic_endpoint = support["center"][0]
        else:
            shares_endpoint = (
                support["center_refs"][0] == adjacent["center_refs"][-1]
                and support["left_refs"][0] == adjacent["left_refs"][-1]
                and support["right_refs"][0] == adjacent["right_refs"][-1]
            )
            synthetic_endpoint = support["center"][-1]
        if not shares_endpoint:
            _issue(
                report,
                "synthetic_planning_support_not_topologically_connected",
                f"{role} support does not share all three endpoint node IDs",
            )
        if len(support["center"]) < 2:
            _issue(
                report,
                "synthetic_planning_support_centerline_degenerate",
                f"{role} support has fewer than two centerline points",
            )

        numeric_keys = SYNTHETIC_PLANNING_SUPPORT_FLOAT_TAGS
        try:
            values = {key: float(tags[key]) for key in numeric_keys}
        except ValueError:
            values = {}
        if not values or not all(math.isfinite(value) for value in values.values()):
            _issue(
                report,
                "invalid_synthetic_planning_support_numeric_metadata",
                f"{role} support contains non-finite numeric metadata",
            )
            continue
        tagged_raw_endpoint = (
            values["planning_support_raw_endpoint_x"],
            values["planning_support_raw_endpoint_y"],
            values["planning_support_raw_endpoint_z"],
        )
        tagged_synthetic_endpoint = (
            values["planning_support_synthetic_endpoint_x"],
            values["planning_support_synthetic_endpoint_y"],
            values["planning_support_synthetic_endpoint_z"],
        )
        tolerance = 1.0e-7
        if (
            math.dist(tagged_raw_endpoint, raw_endpoint) > tolerance
            or math.dist(tagged_synthetic_endpoint, synthetic_endpoint) > tolerance
        ):
            _issue(
                report,
                "synthetic_planning_support_endpoint_tag_mismatch",
                f"{role} support endpoint tags differ from serialized OSM geometry",
            )
        tangent = (
            values["planning_support_tangent_x"],
            values["planning_support_tangent_y"],
        )
        outer_tangent = (
            values["planning_support_outer_tangent_x"],
            values["planning_support_outer_tangent_y"],
        )
        tangent_norm = math.hypot(*tangent)
        outer_tangent_norm = math.hypot(*outer_tangent)
        geometry_kind = tags["planning_support_geometry_kind"]
        if (
            expected_tangent is None
            or abs(tangent_norm - 1.0) > tolerance
            or abs(outer_tangent_norm - 1.0) > tolerance
            or geometry_kind
            not in {"straight", "left_arc_straight", "right_arc_straight"}
        ):
            _issue(
                report,
                "invalid_synthetic_planning_support_tangent",
                f"{role} support tangent/family is degenerate",
            )
        else:
            tangent_delta = _heading_change_deg(expected_tangent, tangent)
            raw_chord = (
                _end_direction_xy(support["center"])
                if role == "head"
                else _start_direction_xy(support["center"])
            )
            outer_chord = (
                _start_direction_xy(support["center"])
                if role == "head"
                else _end_direction_xy(support["center"])
            )
            radius = values["planning_support_turn_radius_m"]
            chord_angle_tolerance_deg = 1.0e-5
            if geometry_kind != "straight" and radius > 0.0:
                chord_angle_tolerance_deg += math.degrees(
                    math.asin(
                        min(
                            1.0,
                            values["planning_support_path_sample_spacing_m"]
                            / (2.0 * radius),
                        )
                    )
                )
            chord_delta = max(
                _heading_change_deg(tangent, raw_chord)
                if raw_chord is not None else math.inf,
                _heading_change_deg(outer_tangent, outer_chord)
                if outer_chord is not None else math.inf,
            )
            if tangent_delta > 1.0e-5 or chord_delta > chord_angle_tolerance_deg:
                _issue(
                    report,
                    "synthetic_planning_support_tangent_mismatch",
                    f"{role} support analytic/chord tangent audit failed: raw="
                    f"{tangent_delta:.9g} deg, chord={chord_delta:.9g} deg",
                )
        planar_length = _polyline_length_xy(support["center"])
        arc_3d = _polyline_length_xyz(support["center"])
        allowance = values["planning_support_endpoint_allowance_m"]
        source_length = float(source["arc_length"])
        expected_s = 0.0 if role == "head" else source_length
        if (
            allowance <= 0.0
            or planar_length + tolerance < allowance
            or abs(values["planning_support_centerline_planar_length_m"] - planar_length)
            > tolerance
            or abs(values["planning_support_centerline_3d_length_m"] - arc_3d)
            > tolerance
            or abs(values["planning_support_source_edge_length_m"] - source_length)
            > max(tolerance, 1.0e-7 * source_length)
            or abs(values["planning_support_raw_endpoint_s_m"] - expected_s)
            > max(tolerance, 1.0e-7 * source_length)
        ):
            _issue(
                report,
                "synthetic_planning_support_length_or_arc_mismatch",
                f"{role} support length/source-arc contract changed",
            )
        search_step = values["planning_support_search_step_m"]
        sample_spacing = values["planning_support_path_sample_spacing_m"]
        search_maximum = values["planning_support_search_max_length_m"]
        turning_radius = values["planning_support_turn_radius_m"]
        turn_angle = values["planning_support_turn_angle_rad"]
        straight_length = values["planning_support_straight_length_m"]
        maximum_curvature = values["planning_support_maximum_curvature_inv_m"]
        actual_maximum_curvature = values[
            "planning_support_actual_maximum_curvature_inv_m"
        ]
        recomputed_actual_maximum_curvature = (
            _maximum_discrete_polyline_curvature_xy(support["center"])
        )
        minimum_turning_radius = float(
            tags.get("vehicle_minimum_turning_radius_m", "nan")
        )
        expected_index = _planning_support_expected_search_index(
            values, integers, geometry_kind, minimum_turning_radius
        ) if math.isfinite(minimum_turning_radius) else None
        analytic_length = straight_length + turning_radius * turn_angle
        length_step_value = (
            (analytic_length - allowance) / search_step
            if search_step > 0.0 else math.nan
        )
        length_step = round(length_step_value) if math.isfinite(length_step_value) else -1
        sampled_length = (
            allowance + length_step * search_step if length_step >= 0 else math.nan
        )
        expected_piece_count = (
            max(1, math.ceil(sampled_length / sample_spacing - 1.0e-9))
            if math.isfinite(sampled_length) and sample_spacing > 0.0
            else -1
        )
        actual_piece_count = len(support["center"]) - 1
        chord_deficit = analytic_length - planar_length
        chord_deficit_limit = _planning_support_chord_deficit_limit(
            analytic_length,
            turning_radius,
            turn_angle,
            sampled_length,
            actual_piece_count,
        )
        expected_curvature = (
            0.0
            if geometry_kind == "straight"
            else (1.0 / turning_radius if turning_radius > 0.0 else math.inf)
        )
        pair_lengths = [
            _point_distance_xy(first, second)
            for first, second in zip(support["center"], support["center"][1:])
        ]
        if (
            not math.isclose(
                search_step,
                SYNTHETIC_PLANNING_SUPPORT_SEARCH_STEP_M,
                rel_tol=0.0,
                abs_tol=tolerance,
            )
            or not math.isclose(
                sample_spacing,
                SYNTHETIC_PLANNING_SUPPORT_PATH_SAMPLE_SPACING_M,
                rel_tol=0.0,
                abs_tol=tolerance,
            )
            or search_maximum + tolerance < analytic_length
            or expected_index is None
            or actual_piece_count != expected_piece_count
            or integers["planning_support_candidate_count_tested"]
            != integers["planning_support_selected_candidate_index"] + 1
            or integers["planning_support_individually_valid_candidate_rank"] < 1
            or sum(integers[key] for key in SYNTHETIC_PLANNING_SUPPORT_REJECTION_TAGS)
            + integers["planning_support_individually_valid_candidate_rank"] - 1
            != integers["planning_support_selected_candidate_index"]
            or integers["planning_support_candidate_pool_limit"]
            != SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_POOL_LIMIT
            or not 1 <= integers["planning_support_head_candidate_pool_size"]
            <= SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_POOL_LIMIT
            or not 1 <= integers["planning_support_tail_candidate_pool_size"]
            <= SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_POOL_LIMIT
            or integers["planning_support_individually_valid_candidate_rank"]
            > integers[
                "planning_support_head_candidate_pool_size"
                if role == "head" else "planning_support_tail_candidate_pool_size"
            ]
            or integers["planning_support_candidate_pair_evaluation_limit"]
            != SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_PAIR_EVALUATION_LIMIT
            or integers["planning_support_candidate_pairs_tested"]
            != integers["planning_support_selected_candidate_pair_rank"]
            or not 1 <= integers["planning_support_selected_candidate_pair_rank"]
            <= SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_PAIR_EVALUATION_LIMIT
            or sum(
                integers[key]
                for key in SYNTHETIC_PLANNING_SUPPORT_PAIR_REJECTION_TAGS
            ) + 1 != integers["planning_support_selected_candidate_pair_rank"]
            or not math.isfinite(minimum_turning_radius)
            or minimum_turning_radius <= 0.0
            or straight_length < -tolerance
            or chord_deficit < -tolerance
            or chord_deficit > chord_deficit_limit
            or abs(maximum_curvature - expected_curvature) > tolerance
            or maximum_curvature > 1.0 / minimum_turning_radius + tolerance
            or actual_maximum_curvature
            > 1.0 / minimum_turning_radius + 1.0e-6
            or abs(
                actual_maximum_curvature - recomputed_actual_maximum_curvature
            ) > 1.0e-6
            or (geometry_kind == "straight" and (
                abs(turning_radius) > tolerance
                or abs(turn_angle) > tolerance
                or _heading_change_deg(tangent, outer_tangent) > 1.0e-5
            ))
            or any(
                length <= GEOMETRY_EPSILON or length > sample_spacing + 1.0e-6
                for length in pair_lengths
            )
        ):
            _issue(
                report,
                "invalid_synthetic_planning_support_search_audit",
                f"{role} support does not reproduce the deterministic v2 search index/family",
            )

        outer_polygon_edge_ids = _canonical_unsigned_id_list(
            tags["planning_support_outer_endpoint_route_polygon_edge_ids"]
        )
        outer_raw_overlap_ids = _canonical_unsigned_id_list(
            tags["planning_support_outer_footprint_raw_overlap_edge_ids"]
        )
        nonadjacent_ids = _canonical_unsigned_id_list(
            tags["planning_support_nonadjacent_raw_overlap_edge_ids"]
        )
        nearest_nonadjacent_centerline_ids = _canonical_unsigned_id_list(
            tags[
                "planning_support_outer_pose_nearest_nonadjacent_raw_centerline_edge_ids"
            ]
        )
        support_edge_id = integers["route_edge_id"]
        containing_relation_ids = sorted(
            int(candidate["relation_id"])
            for candidate in lanelets
            if _point_in_polygon_xy(
                (synthetic_endpoint[0], synthetic_endpoint[1]),
                candidate["left"] + list(reversed(candidate["right"])),
            )
        )
        if (
            outer_polygon_edge_ids != [support_edge_id]
            or outer_raw_overlap_ids != []
            or nonadjacent_ids is None
            or integers["planning_support_adjacent_output_edge_id"]
            in (nonadjacent_ids or [])
            or containing_relation_ids != [int(support["relation_id"])]
        ):
            _issue(
                report,
                "synthetic_planning_support_outer_pose_not_unique",
                f"{role} outer pose is not an order-independent support-only polygon winner",
            )

        front = float(tags.get("estimated_front_extent_m", "nan"))
        rear = float(tags.get("estimated_rear_extent_m", "nan"))
        vehicle_width = float(tags.get("estimated_vehicle_width_m", "nan"))
        lateral_margin = float(tags.get("estimated_lateral_margin_m", "nan"))
        maximum_longitudinal_extent = max(front, rear)
        expected_required_isolation = (
            math.hypot(
                maximum_longitudinal_extent,
                0.5 * vehicle_width + lateral_margin,
            )
            + allowance
            if all(
                math.isfinite(value)
                for value in (front, rear, vehicle_width, lateral_margin, allowance)
            )
            else math.nan
        )
        adjacent_output_id = integers["planning_support_adjacent_output_edge_id"]
        competing_centerlines: list[tuple[int, list[Point]]] = []
        for raw_lanelet in raw_lanelets:
            raw_output_id = _unsigned_integer(
                raw_lanelet["tags"].get("route_edge_id")
            )
            if raw_output_id is None or raw_output_id == adjacent_output_id:
                continue
            competing_centerlines.append((raw_output_id, raw_lanelet["center"]))
        minimum_centerline_distance = math.inf
        expected_nearest_centerline_ids: list[int] = []
        for raw_output_id, centerline in competing_centerlines:
            edge_distance = math.sqrt(
                min(
                    _point_segment_distance_squared_xy(
                        synthetic_endpoint, segment
                    )
                    for segment in zip(centerline[:-1], centerline[1:])
                )
            )
            if edge_distance + 1.0e-9 < minimum_centerline_distance:
                minimum_centerline_distance = edge_distance
                expected_nearest_centerline_ids = [raw_output_id]
            elif abs(edge_distance - minimum_centerline_distance) <= 1.0e-9:
                expected_nearest_centerline_ids.append(raw_output_id)
        expected_nearest_centerline_ids = sorted(
            set(expected_nearest_centerline_ids)
        )
        expected_actual_isolation = (
            minimum_centerline_distance
            if competing_centerlines
            else expected_required_isolation
        )
        reported_required_isolation = values[
            "planning_support_required_outer_pose_nonadjacent_raw_centerline_isolation_m"
        ]
        reported_actual_isolation = values[
            "planning_support_actual_outer_pose_nonadjacent_raw_centerline_isolation_m"
        ]
        if (
            not math.isfinite(expected_required_isolation)
            or not math.isfinite(expected_actual_isolation)
            or abs(reported_required_isolation - expected_required_isolation)
            > tolerance
            or abs(reported_actual_isolation - expected_actual_isolation) > tolerance
            or integers["planning_support_outer_pose_nonadjacent_raw_centerline_count"]
            != len(competing_centerlines)
            or nearest_nonadjacent_centerline_ids
            != expected_nearest_centerline_ids
            or reported_actual_isolation + tolerance < reported_required_isolation
        ):
            _issue(
                report,
                "synthetic_planning_support_outer_pose_insufficiently_isolated",
                f"{role} outer pose is not separated from nonadjacent raw "
                "centerlines by the vehicle-derived minimum",
            )
        raw_transition = values["planning_support_raw_overlap_transition_length_m"]
        nonadjacent_transition = values[
            "planning_support_nonadjacent_raw_overlap_transition_length_m"
        ]
        maximum_nonadjacent_transition = values[
            "planning_support_maximum_nonadjacent_raw_overlap_transition_length_m"
        ]
        front = float(tags.get("estimated_front_extent_m", "nan"))
        rear = float(tags.get("estimated_rear_extent_m", "nan"))
        expected_maximum_nonadjacent_transition = (
            front + rear + 2.0 * minimum_turning_radius
        )
        expected_required = (rear if role == "head" else front) + allowance
        required = values[
            "planning_support_required_boundary_beyond_raw_endpoint_m"
        ]
        outward = (-tangent[0], -tangent[1]) if role == "head" else tangent
        left_outer = support["left"][0 if role == "head" else -1]
        right_outer = support["right"][0 if role == "head" else -1]
        actual_left = (
            (left_outer[0] - raw_endpoint[0]) * outward[0]
            + (left_outer[1] - raw_endpoint[1]) * outward[1]
        )
        actual_right = (
            (right_outer[0] - raw_endpoint[0]) * outward[0]
            + (right_outer[1] - raw_endpoint[1]) * outward[1]
        )
        if (
            not math.isfinite(front)
            or not math.isfinite(rear)
            or abs(required - expected_required) > tolerance
            or abs(
                maximum_nonadjacent_transition
                - expected_maximum_nonadjacent_transition
            ) > tolerance
            or raw_transition < -tolerance
            or raw_transition > planar_length + tolerance
            or nonadjacent_transition < -tolerance
            or nonadjacent_transition > raw_transition + tolerance
            or nonadjacent_transition > maximum_nonadjacent_transition + tolerance
            or ((nonadjacent_ids == []) != (abs(nonadjacent_transition) <= tolerance))
            or abs(
                values["planning_support_actual_left_boundary_beyond_raw_endpoint_m"]
                - actual_left
            ) > tolerance
            or abs(
                values["planning_support_actual_right_boundary_beyond_raw_endpoint_m"]
                - actual_right
            ) > tolerance
            or (
                geometry_kind == "straight"
                and min(actual_left, actual_right) + tolerance < expected_required
            )
        ):
            _issue(
                report,
                "synthetic_planning_support_endpoint_footprint_not_contained",
                f"{role} support does not provide the configured overhang + allowance",
            )
        role_evidence[role] = {
            "geometry_kind": geometry_kind,
            "selected_candidate_index": integers[
                "planning_support_selected_candidate_index"
            ],
            "candidate_count_tested": integers[
                "planning_support_candidate_count_tested"
            ],
            "individually_valid_candidate_rank": integers[
                "planning_support_individually_valid_candidate_rank"
            ],
            "candidate_pool_limit": integers[
                "planning_support_candidate_pool_limit"
            ],
            "head_candidate_pool_size": integers[
                "planning_support_head_candidate_pool_size"
            ],
            "tail_candidate_pool_size": integers[
                "planning_support_tail_candidate_pool_size"
            ],
            "candidate_pair_evaluation_limit": integers[
                "planning_support_candidate_pair_evaluation_limit"
            ],
            "candidate_pairs_tested": integers[
                "planning_support_candidate_pairs_tested"
            ],
            "selected_candidate_pair_rank": integers[
                "planning_support_selected_candidate_pair_rank"
            ],
            "pair_rejection_counts": {
                key: integers[key]
                for key in SYNTHETIC_PLANNING_SUPPORT_PAIR_REJECTION_TAGS
            },
            "centerline_planar_length_m": planar_length,
            "outer_lanelet_relation_id": int(support["relation_id"]),
            "outer_route_polygon_edge_ids": outer_polygon_edge_ids,
            "required_outer_pose_nonadjacent_raw_centerline_isolation_m": (
                reported_required_isolation
            ),
            "actual_outer_pose_nonadjacent_raw_centerline_isolation_m": (
                reported_actual_isolation
            ),
            "outer_pose_nearest_nonadjacent_raw_centerline_edge_ids": (
                nearest_nonadjacent_centerline_ids
            ),
            "nonadjacent_raw_overlap_edge_ids": nonadjacent_ids,
            "nonadjacent_raw_overlap_transition_length_m": nonadjacent_transition,
            "maximum_nonadjacent_raw_overlap_transition_length_m": (
                maximum_nonadjacent_transition
            ),
        }

    if set(role_evidence) == {"head", "tail"}:
        pair_fields = (
            "candidate_pool_limit",
            "head_candidate_pool_size",
            "tail_candidate_pool_size",
            "candidate_pair_evaluation_limit",
            "candidate_pairs_tested",
            "selected_candidate_pair_rank",
            "pair_rejection_counts",
        )
        if any(
            role_evidence["head"][field] != role_evidence["tail"][field]
            for field in pair_fields
        ):
            _issue(
                report,
                "inconsistent_synthetic_planning_support_pair_audit",
                "head/tail support does not carry one identical final pair-search audit",
            )

    canonical_records.sort(key=lambda record: record["relation_id"])
    contract_sha256 = hashlib.sha256(
        json.dumps(
            canonical_records,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    report["input"]["lanelet2_map_sha256"] = _sha256(map_path)
    report["planning_support"] = {
        "present": bool(support_lanelets),
        "contract_version": SYNTHETIC_PLANNING_SUPPORT_CONTRACT_VERSION,
        "source": SYNTHETIC_PLANNING_SUPPORT_SOURCE,
        "test_only": True,
        "relation_ids": {
            role: by_role[role]["relation_id"]
            for role in ("head", "tail") if role in by_role
        },
        "route_edge_ids": {
            role: _unsigned_integer(by_role[role]["tags"].get("route_edge_id"))
            for role in ("head", "tail") if role in by_role
        },
        "raw_edge_ids": source_ids,
        "raw_edge_count": len(source_ids),
        "support_lanelet_count": len(support_lanelets),
        "support_geometry_tags_sha256": contract_sha256,
        "lanelet2_map_sha256": _sha256(map_path),
        "support_is_part_of_raw_counts": False,
        "support_is_part_of_named_route": False,
        "support_is_raw_coverage": False,
        "collision_scope": "route_polygon_single_transition_only",
        "roles": role_evidence,
        "production_ready": False,
        "deployment_ready": False,
    }


def _validate_named_route(
    directory: Path,
    named_route: dict[str, Any],
    map_details: dict[str, Any],
    centerline_segments: list[Segment],
    report: dict[str, Any],
    minimum_coverage: float,
    coverage_distance_m: float,
    centerline_source: str = "recorded_trajectory",
) -> None:
    report["coverage_reference"] = "named_route_source_graph"
    # The replay-candidate graph is the complete map graph.  A promoted named
    # Route has a separate, exact mission-subset sidecar so validating the
    # mission can never imply that the full map should be trimmed to it.
    edited_topology = centerline_source == "edited_topology"
    source_path = directory / (
        "route_graph_autoware_topology_selected_mission.geojson"
        if edited_topology
        else "route_graph_autoware_selected_mission.geojson"
    )
    # A promoted Autoware Mission is authored against the exact chronological
    # replay, not against the editable/de-duplicated topology used by Nav2.
    # Never substitute navigation_authoring.json here: a self-consistent old
    # topology document can still describe a shorter or synthetically closed
    # graph while using plausible-looking Edge IDs.
    authoring_path = directory / (
        "navigation_authoring_autoware_topology.json"
        if edited_topology
        else "navigation_authoring_autoware_replay.json"
    )
    status_path = directory / (
        "navigation_authoring_autoware_topology_status.json"
        if edited_topology
        else "navigation_authoring_closed_course_status.json"
    )
    authoring_scope = (
        "autoware_edited_topology"
        if edited_topology
        else "autoware_lossless_replay"
    )
    report["input"]["named_route_source_graph"] = str(source_path)
    if source_path.is_file():
        report["input"]["named_route_source_graph_sha256"] = _sha256(source_path)
    report["input"]["navigation_authoring"] = str(authoring_path)
    report["input"]["navigation_authoring_scope"] = authoring_scope
    if authoring_path.is_file():
        report["input"]["navigation_authoring_sha256"] = _sha256(authoring_path)
    report["input"]["navigation_authoring_status"] = str(status_path)
    if status_path.is_file():
        report["input"]["navigation_authoring_status_sha256"] = _sha256(status_path)
    records = named_route.get("records", [])
    if not records or "id" not in named_route:
        _issue(
            report,
            "invalid_named_route_metadata",
            "named Route metadata could not establish one ordered Route",
        )
        return

    source_edges = _read_source_route_graph(source_path, report)
    lanelet_edge_ids = named_route["edge_ids"]
    source_edge_ids = named_route.get("source_edge_ids", lanelet_edge_ids)
    if set(source_edges) != set(source_edge_ids):
        _issue(
            report,
            "named_route_source_edge_set_mismatch",
            "selected-mission source Edge set must exactly equal the exported "
            f"named-Route source subset: source={sorted(source_edges)}, "
            f"exported={source_edge_ids}",
        )
    if list(source_edges) != source_edge_ids:
        _issue(
            report,
            "named_route_source_edge_order_mismatch",
            "selected-mission source Edge order differs from semantic Lanelet lineage: "
            f"source={list(source_edges)}, exported={source_edge_ids}",
        )
    for previous_id, following_id in zip(source_edge_ids[:-1], source_edge_ids[1:]):
        if (
            previous_id in source_edges
            and following_id in source_edges
            and source_edges[previous_id]["end"] != source_edges[following_id]["start"]
        ):
            _issue(
                report,
                "named_route_source_not_a_directed_chain",
                f"source Route Edges {previous_id} -> {following_id} are disconnected",
            )

    terminal_composite = _terminal_support_composite(
        directory,
        records,
        source_edges,
        source_edge_ids,
        report,
        coverage_distance_m,
    )

    minimum_source_coverage = 1.0
    minimum_lanelet_coverage = 1.0
    valid_edge_comparisons = 0
    grouped_records: list[tuple[int, list[dict[str, Any]]]] = []
    for record in records:
        source_edge_id = record["source_edge_id"]
        if not grouped_records or grouped_records[-1][0] != source_edge_id:
            grouped_records.append((source_edge_id, []))
        grouped_records[-1][1].append(record)
    for edge_id, source_records in grouped_records:
        if edge_id not in source_edges:
            continue
        source_points = source_edges[edge_id]["points"]
        if terminal_composite is not None and edge_id == terminal_composite[0]:
            source_points = terminal_composite[1]
        try:
            lanelet_points = _joined_polyline(
                record["lanelet"]["center"] for record in source_records
            )
        except ValueError as error:
            _issue(
                report,
                "named_route_semantic_reconstruction_disconnected",
                f"source Edge {edge_id}: {error}",
            )
            continue
        lanelet_segments = list(zip(lanelet_points[:-1], lanelet_points[1:]))
        source_segments = list(zip(source_points[:-1], source_points[1:]))
        source_coverage, _, _ = _point_to_segment_coverage(
            source_points, lanelet_segments, coverage_distance_m
        )
        lanelet_coverage, _, _ = _point_to_segment_coverage(
            lanelet_points, source_segments, coverage_distance_m
        )
        minimum_source_coverage = min(minimum_source_coverage, source_coverage)
        minimum_lanelet_coverage = min(minimum_lanelet_coverage, lanelet_coverage)
        valid_edge_comparisons += 1
        if (
            _point_distance_xy(source_points[0], lanelet_points[0]) > coverage_distance_m
            or _point_distance_xy(source_points[-1], lanelet_points[-1])
            > coverage_distance_m
        ):
            _issue(
                report,
                "named_route_edge_orientation_mismatch",
                f"Lanelet orders {source_records[0]['order']}.."
                f"{source_records[-1]['order']} do not preserve source Edge "
                f"{edge_id} direction",
            )
    if valid_edge_comparisons == 0:
        minimum_source_coverage = 0.0
        minimum_lanelet_coverage = 0.0
    report["metrics"]["source_route_centerline_coverage"] = minimum_source_coverage
    report["metrics"]["lanelet_centerline_source_coverage"] = minimum_lanelet_coverage
    report["metrics"]["acceptance_centerline_coverage"] = min(
        minimum_source_coverage, minimum_lanelet_coverage
    )
    # This count now describes the dedicated selected-mission source sidecar.
    report["counts"]["named_route_source_edges"] = len(source_edges)
    report["counts"]["selected_named_route_source_edges"] = len(source_edge_ids)
    report["counts"]["selected_named_route_lanelet_segments"] = len(lanelet_edge_ids)
    if min(minimum_source_coverage, minimum_lanelet_coverage) + 1.0e-12 < minimum_coverage:
        _issue(
            report,
            "insufficient_named_route_source_coverage",
            "bidirectional Lanelet/source Route coverage "
            f"{min(minimum_source_coverage, minimum_lanelet_coverage):.6f} is below "
            f"{minimum_coverage:.6f}",
        )

    authoring = _read_json_object(
        authoring_path, report, "missing_navigation_authoring", "malformed_navigation_authoring"
    )
    status = _read_json_object(
        status_path,
        report,
        "missing_navigation_authoring_status",
        "malformed_navigation_authoring_status",
    )
    if authoring is None or status is None:
        return
    authoring_fingerprint = authoring.get("graph_fingerprint")
    if authoring.get("schema_version") != 1 or not isinstance(
        authoring_fingerprint, str
    ) or re.fullmatch(r"[0-9a-f]{16}", authoring_fingerprint) is None:
        _issue(
            report,
            "malformed_navigation_authoring",
            "Autoware authoring requires schema v1 and a 16-digit lowercase "
            "graph fingerprint",
        )
    authoring_frame = authoring.get("frame_id")
    status_fingerprint = status.get("graph_fingerprint")
    status_frame = status.get("frame_id")
    if edited_topology:
        source_selection = report.get("vector_map_source_selection")
        if not isinstance(source_selection, dict) or (
            source_selection.get("SOURCE") != "edited_topology"
            or source_selection.get("FRAME") != authoring_frame
            or source_selection.get("GRAPH_FINGERPRINT") != authoring_fingerprint
        ):
            _issue(
                report,
                "vector_map_source_authoring_identity_mismatch",
                "the user-authored Mission must use the exact frame and graph "
                "fingerprint saved in vector_map_source.tsv",
            )
    if (
        not isinstance(authoring_frame, str)
        or not authoring_frame
        or status.get("schema_version") != 1
        or status_frame != authoring_frame
        or status_fingerprint != authoring_fingerprint
    ):
        _issue(
            report,
            "navigation_authoring_status_identity_mismatch",
            "Autoware authoring/status schema, frame, and graph fingerprint "
            "must match exactly",
        )
    routes = authoring.get("routes")
    if not isinstance(routes, list):
        routes = []
        _issue(report, "malformed_navigation_authoring", "routes must be an array")
    elif any(
        not isinstance(route, dict) or route.get("target") != "autoware"
        for route in routes
    ):
        _issue(
            report,
            "cross_scope_navigation_authoring",
            f"{authoring_path.name} may contain only "
            "target=autoware Routes",
        )
    stops = authoring.get("stop_lines")
    if not isinstance(stops, list):
        _issue(report, "malformed_navigation_authoring", "stop_lines must be an array")
    elif any(
        not isinstance(stop, dict) or stop.get("target") != "autoware"
        for stop in stops
    ):
        _issue(
            report,
            "cross_scope_navigation_authoring",
            f"{authoring_path.name} may contain only "
            "target=autoware stop lines",
        )
    authored_matches = [
        route
        for route in routes
        if isinstance(route, dict) and _unsigned_integer(route.get("id")) == named_route["id"]
    ]
    if len(authored_matches) != 1:
        _issue(
            report,
            "named_route_authoring_identity_mismatch",
            "exported named Route does not match exactly one authoring Route",
        )
        return
    authored_route = authored_matches[0]
    if (
        authored_route.get("name") != named_route["name"]
        or authored_route.get("target") != named_route["target"]
        or authored_route.get("validation_requested") is not True
        or authored_route.get("promotion_requested") is not True
    ):
        _issue(
            report,
            "named_route_authoring_identity_mismatch",
            "Route name/target/request flags differ from independent authoring evidence",
        )
    autoware_status = status.get("autoware")
    nav2_status = status.get("nav2")
    route_statuses = status.get("routes")
    status_matches = (
        [
            route_status
            for route_status in route_statuses
            if isinstance(route_status, dict)
            and _unsigned_integer(route_status.get("id")) == named_route["id"]
        ]
        if isinstance(route_statuses, list)
        else []
    )
    if (
        not isinstance(autoware_status, dict)
        or _unsigned_integer(autoware_status.get("selected_route_id")) != named_route["id"]
        or autoware_status.get("promoted") is not True
        or len(status_matches) != 1
        or status_matches[0].get("valid") is not True
        or status_matches[0].get("promotion_eligible") is not True
    ):
        _issue(
            report,
            "named_route_closed_course_status_mismatch",
            "closed-course status does not select and promote this valid named Route",
        )
    if (
        not isinstance(nav2_status, dict)
        or nav2_status.get("selected_route_id") is not None
        or nav2_status.get("promoted") is not False
        or not isinstance(status.get("errors"), list)
        or status.get("errors")
        or (
            isinstance(route_statuses, list)
            and any(
                not isinstance(route_status, dict)
                or route_status.get("target") != "autoware"
                for route_status in route_statuses
            )
        )
        or (
            isinstance(status.get("stop_lines"), list)
            and any(
                not isinstance(stop_status, dict)
                or stop_status.get("target") != "autoware"
                for stop_status in status["stop_lines"]
            )
        )
    ):
        _issue(
            report,
            "cross_scope_navigation_authoring_status",
            "Autoware status must contain only clean Autoware-scope "
            "promotion evidence",
        )
    ordered_edge_ids = authored_route.get("ordered_edge_ids")
    if (
        not isinstance(ordered_edge_ids, list)
        or not ordered_edge_ids
        or any(_unsigned_integer(edge_id) in (None, 0) for edge_id in ordered_edge_ids)
    ):
        _issue(
            report,
            "malformed_navigation_authoring",
            "selected authoring Route has invalid ordered_edge_ids",
        )
    else:
        authored_edge_ids = [int(edge_id) for edge_id in ordered_edge_ids]
        if set(authored_edge_ids) != set(source_edge_ids):
            _issue(
                report,
                "named_route_authoring_edge_set_mismatch",
                "exported named-Route Edge set differs from independent authoring "
                f"evidence: exported={source_edge_ids}, authored={authored_edge_ids}",
            )
        elif authored_edge_ids != source_edge_ids:
            _issue(
                report,
                "named_route_authoring_edge_order_mismatch",
                "exported named-Route Edge order differs from independent authoring "
                f"evidence: exported={source_edge_ids}, authored={authored_edge_ids}",
            )
    if source_edges and source_edge_ids:
        first_source = source_edges.get(source_edge_ids[0])
        last_source = source_edges.get(source_edge_ids[-1])
        if (
            first_source is not None
            and _unsigned_integer(authored_route.get("start_node_id")) != first_source["start"]
        ) or (
            last_source is not None
            and _unsigned_integer(authored_route.get("end_node_id")) != last_source["end"]
        ):
            _issue(
                report,
                "named_route_endpoint_mismatch",
                "source Route chain endpoints differ from authoring start/end Nodes",
            )
    _validate_authored_stop_lines(
        named_route,
        authored_route,
        authoring,
        map_details,
        report,
        coverage_distance_m,
    )
    report["named_route"] = {
        "id": named_route["id"],
        "name": named_route["name"],
        "target": named_route["target"],
        "ordered_edge_ids": source_edge_ids,
        "ordered_lanelet_segment_ids": lanelet_edge_ids,
        "authoring_scope": authoring_scope,
    }


def validate_candidate(
    output_dir: str | Path,
    minimum_coverage: float = DEFAULT_MINIMUM_COVERAGE,
    coverage_distance_m: float = DEFAULT_COVERAGE_DISTANCE_M,
) -> dict[str, Any]:
    """Return a JSON-serializable acceptance report for one generated output directory."""
    if not 0.0 <= minimum_coverage <= 1.0:
        raise ValueError("minimum_coverage must be between 0 and 1")
    if not math.isfinite(coverage_distance_m) or coverage_distance_m <= 0.0:
        raise ValueError("coverage_distance_m must be finite and positive")
    directory = Path(output_dir)
    map_path = directory / "lanelet2_map_closed_course_experimental.osm"
    trajectory_path = directory / "trajectory_processed.tum"
    report: dict[str, Any] = {
        "format_version": 1,
        "acceptance_scope": "static_format_geometry_coverage_only",
        "accepted": False,
        # This validator intentionally has no authority to approve vehicle
        # motion.  The Docker runner applies the independent closed-course
        # planning-test gate after binding the acquisition/target vehicle and
        # physical Route-validation evidence.
        "accepted_for_autoware_motion_test": False,
        "production_ready": False,
        "deployment_ready": False,
        "input": {
            "output_dir": str(directory),
            "lanelet2_map": str(map_path),
            "trajectory": str(trajectory_path),
        },
        "thresholds": {
            "minimum_trajectory_centerline_coverage": minimum_coverage,
            "maximum_centerline_distance_m": coverage_distance_m,
            "swept_footprint_pose_step_m": SWEPT_FOOTPRINT_POSE_STEP_M,
            "swept_footprint_grid_step_m": SWEPT_FOOTPRINT_GRID_STEP_M,
        },
        "counts": {},
        "metrics": {},
        "errors": [],
        "warnings": [],
    }
    if map_path.is_symlink():
        _issue(
            report,
            "lanelet_map_not_regular_file",
            f"Lanelet2 acceptance input must not be a symlink: {map_path}",
        )
    elif map_path.is_file():
        # Bind every acceptance report to the canonical OSM, including the
        # ordinary no-synthetic-support path.  Previously this hash was added
        # only as a side effect of planning-support validation.
        report["input"]["lanelet2_map_sha256"] = _sha256(map_path)
    # Record the exact GUI semantic map used to derive semantic Lanelet
    # children.  The runtime provenance gate requires these hashes for a
    # promoted full-route campaign; keeping them in the candidate report
    # prevents a self-consistent OSM/stage copy from hiding a swapped TSV.
    for semantic_name in (
        "semantic_features.tsv",
        "semantic_features.geojson",
        "semantic_features_autoware_topology.tsv",
        "semantic_features_autoware_topology.geojson",
    ):
        semantic_path = directory / semantic_name
        report["input"][semantic_name.replace(".", "_")] = str(semantic_path)
        if semantic_path.is_file() and not semantic_path.is_symlink():
            report["input"][f"{semantic_name.replace('.', '_')}_sha256"] = _sha256(
                semantic_path
            )
    centerline_segments, endpoint_edges, map_details = _read_lanelet_map(map_path, report)
    centerline_source = _resolve_centerline_source(directory, map_details, report)
    replay_sidecar_present = any(
        (directory / name).exists()
        for name in (
            "route_graph_closed_course_replay_candidate.geojson",
            "route_graph_autoware_replay_candidate.geojson",
        )
    )
    estimated_replay_map = any(
        lanelet["tags"].get("boundary_model") == ESTIMATED_BOUNDARY_MODEL
        for lanelet in map_details["lanelets"]
    )
    full_map_edges: dict[int, dict[str, Any]] = {}
    if centerline_source == "recorded_trajectory" and (
        replay_sidecar_present or estimated_replay_map
    ):
        _validate_lossless_replay_identity(directory, report)
        full_map_edges = _validate_full_map_source(
            directory,
            map_details,
            report,
            minimum_coverage,
            coverage_distance_m,
        )
    elif centerline_source == "edited_topology":
        full_map_edges = _validate_full_map_source(
            directory,
            map_details,
            report,
            minimum_coverage,
            coverage_distance_m,
            source_path=directory
            / "route_graph_autoware_topology_source.geojson",
        )
    _validate_configured_swept_footprint(map_details, report)
    trajectory = _read_trajectory(trajectory_path, report)
    report["counts"]["trajectory_poses"] = len(trajectory)

    components = _component_count(endpoint_edges)
    report["metrics"]["centerline_endpoint_components"] = components
    if endpoint_edges and components != 1:
        _issue(
            report,
            "disconnected_centerline_graph",
            f"Lanelet centerline endpoint graph has {components} components",
        )

    trajectory_coverage, covered_samples, total_samples = _point_to_segment_coverage(
        trajectory, centerline_segments, coverage_distance_m
    )
    # Full processed-trajectory coverage is a map-level acceptance gate even
    # when a named Route selects only a mission subset.  Route authoring must
    # never turn a short route-only OSM into an accepted replacement for the
    # complete reviewed Lanelet map.
    report["metrics"]["trajectory_centerline_coverage"] = trajectory_coverage
    report["counts"]["covered_trajectory_samples"] = covered_samples
    report["counts"]["trajectory_coverage_samples"] = total_samples
    if (
        centerline_source == "recorded_trajectory"
        and trajectory
        and centerline_segments
        and trajectory_coverage + 1.0e-12 < minimum_coverage
    ):
        _issue(
            report,
            "insufficient_trajectory_coverage",
            f"full processed-trajectory centerline coverage "
            f"{trajectory_coverage:.6f} is below {minimum_coverage:.6f}",
        )

    named_route = _named_route_from_lanelets(map_details, report)
    if full_map_edges:
        _validate_synthetic_open_route_planning_support(
            map_path, map_details, full_map_edges, named_route, report
        )
    if named_route is None:
        if centerline_source == "edited_topology":
            report["coverage_reference"] = "edited_topology_source_graph"
            report["metrics"]["acceptance_centerline_coverage"] = min(
                report["metrics"].get(
                    "full_map_source_centerline_coverage", 0.0
                ),
                report["metrics"].get(
                    "full_map_lanelet_centerline_source_coverage", 0.0
                ),
            )
        else:
            report["coverage_reference"] = "full_processed_trajectory"
            report["metrics"]["acceptance_centerline_coverage"] = trajectory_coverage
    else:
        _audit_named_route_terminal_membership(named_route, map_details, report)
        # Retain the historical diagnostic fields for downstream report
        # readers, but the same value is now enforced above as a hard gate.
        # Named-Route set/order/direction/coverage is validated independently.
        report["metrics"][
            "full_trajectory_centerline_coverage_diagnostic"
        ] = trajectory_coverage
        report["counts"]["full_trajectory_covered_samples_diagnostic"] = covered_samples
        report["counts"]["full_trajectory_coverage_samples_diagnostic"] = total_samples
        _validate_named_route(
            directory,
            named_route,
            map_details,
            centerline_segments,
            report,
            minimum_coverage,
            coverage_distance_m,
            centerline_source,
        )
    report["accepted"] = not report["errors"]
    return report


def _write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate a generated Autoware closed-course Lanelet2 candidate."
    )
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--minimum-coverage",
        type=float,
        default=DEFAULT_MINIMUM_COVERAGE,
        help="minimum trajectory-to-centerline coverage fraction (default: 0.99)",
    )
    parser.add_argument(
        "--coverage-distance-m",
        type=float,
        default=DEFAULT_COVERAGE_DISTANCE_M,
        help="maximum XY distance from a trajectory sample to a centerline (default: 0.50)",
    )
    parser.add_argument(
        "--report",
        "--report-path",
        dest="report",
        type=Path,
        help="optional JSON acceptance report path",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    arguments = _argument_parser().parse_args(argv)
    try:
        report = validate_candidate(
            arguments.output_dir,
            minimum_coverage=arguments.minimum_coverage,
            coverage_distance_m=arguments.coverage_distance_m,
        )
        if arguments.report is not None:
            _write_report(arguments.report, report)
    except (OSError, ValueError) as error:
        print(f"validation configuration/error: {error}", file=sys.stderr)
        return 2
    summary = {
        "accepted": report["accepted"],
        "acceptance_scope": report["acceptance_scope"],
        "accepted_for_autoware_motion_test": report[
            "accepted_for_autoware_motion_test"
        ],
        # Missing acceptance coverage means the selected evidence contract
        # could not be established.  Reporting 0.0 here falsely looks like a
        # measured geometry result, so preserve the distinction as JSON null.
        "coverage": report["metrics"].get("acceptance_centerline_coverage"),
        "coverage_reference": report.get("coverage_reference"),
        "errors": len(report["errors"]),
        "warnings": len(report["warnings"]),
    }
    diagnostic_coverage = report["metrics"].get(
        "full_trajectory_centerline_coverage_diagnostic"
    )
    if diagnostic_coverage is not None:
        summary["full_trajectory_coverage_diagnostic"] = diagnostic_coverage
    if arguments.report is not None:
        summary["report"] = str(arguments.report)
    print(json.dumps(summary, sort_keys=True))
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    sys.exit(main())
