#!/usr/bin/env python3

import hashlib
import importlib.util
import json
import math
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(sys.argv[1]).resolve()
sys.dont_write_bytecode = True


def load_validator():
    spec = importlib.util.spec_from_file_location("validate_autoware_candidate", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


VALIDATOR = load_validator()


def legacy_point_in_polygon_xy(point, polygon):
    """Exact pre-index predicate retained as a differential-test reference."""
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        candidate = (point[0], point[1], 0.0)
        if (
            VALIDATOR._point_segment_distance_squared_xy(
                candidate, (first, second)
            )
            <= 1.0e-14
        ):
            return True
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if (current[1] > point[1]) != (previous[1] > point[1]):
            intersection_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if point[0] < intersection_x:
                inside = not inside
        previous = current
    return inside


def legacy_point_covered_by_polygons(point, polygons):
    for polygon in polygons:
        bounds = (
            min(item[0] for item in polygon),
            min(item[1] for item in polygon),
            max(item[0] for item in polygon),
            max(item[1] for item in polygon),
        )
        if not (
            bounds[0] - 1.0e-7 <= point[0] <= bounds[2] + 1.0e-7
            and bounds[1] - 1.0e-7 <= point[1] <= bounds[3] + 1.0e-7
        ):
            continue
        if legacy_point_in_polygon_xy(point, polygon):
            return True
    return False


def node(node_id, x, y, *, local_x=None):
    x_value = str(x if local_x is None else local_x)
    return f"""  <node id="{node_id}">
    <tag k="local_x" v="{x_value}"/>
    <tag k="local_y" v="{y}"/>
    <tag k="ele" v="0"/>
  </node>"""


def way(way_id, references):
    refs = "\n".join(f'    <nd ref="{reference}"/>' for reference in references)
    return f"  <way id=\"{way_id}\">\n{refs}\n  </way>"


def relation(
    relation_id,
    left,
    right,
    center,
    speed="0.25 m/s",
    omit_role=None,
    extra_members="",
    extra_tags="",
):
    members = []
    for role, reference in (("left", left), ("right", right), ("centerline", center)):
        if role != omit_role:
            members.append(f'    <member type="way" ref="{reference}" role="{role}"/>')
    members_text = "\n".join(members) + extra_members
    return f"""  <relation id="{relation_id}">
{members_text}
    <tag k="type" v="lanelet"/>
    <tag k="speed_limit" v="{speed}"/>
{extra_tags}
  </relation>"""


def candidate_osm(
    *,
    first_speed="0.25 m/s",
    omit_role=None,
    invalid_center_ref=False,
    nonfinite_node=False,
    crossing_boundaries=False,
    zero_width=False,
    disconnected=False,
    cusp=False,
    route_edge_tags=False,
    second_length=5.0,
    second_heading_deg=0.0,
):
    first_left_end_y = -1 if crossing_boundaries else 1
    first_right_end_y = 1 if crossing_boundaries else -1
    if zero_width:
        first_right_end_y = first_left_end_y
    second_heading = math.radians(second_heading_deg)
    second_center_x = 5.0 + second_length * math.cos(second_heading)
    second_center_y = second_length * math.sin(second_heading)
    second_left_x = second_center_x - math.sin(second_heading)
    second_left_y = second_center_y + math.cos(second_heading)
    second_right_x = second_center_x + math.sin(second_heading)
    second_right_y = second_center_y - math.cos(second_heading)
    if second_length == 5.0 and second_heading_deg == 0.0:
        # Keep the long-standing fixture text stable; several metadata tests
        # deliberately replace these exact endpoint elements.
        second_center_x, second_center_y = 10, 0
        second_left_x, second_left_y = 10, 1
        second_right_x, second_right_y = 10, -1
    nodes = [
        node(1, 0, 1),
        node(2, 5, first_left_end_y),
        node(3, 0, 1 if zero_width else -1, local_x="nan" if nonfinite_node else None),
        node(4, 5, first_right_end_y),
        node(5, 0, 0),
        node(6, 5, 0),
        node(7, 0 if cusp else second_left_x, 1 if cusp else second_left_y),
        node(8, 0 if cusp else second_right_x, -1 if cusp else second_right_y),
        node(9, 0 if cusp else second_center_x, 0 if cusp else second_center_y),
    ]
    if disconnected:
        nodes.append(node(10, 5, 0))
    first_center_nodes = (5, 999) if invalid_center_ref else (5, 6)
    second_center_nodes = (10 if disconnected else 6, 9)
    ways = [
        way(10, (1, 2)),
        way(11, (3, 4)),
        way(12, first_center_nodes),
        way(20, (2, 7)),
        way(21, (4, 8)),
        way(22, second_center_nodes),
    ]
    first_extra_tags = (
        '    <tag k="route_edge_id" v="10"/>' if route_edge_tags else ""
    )
    second_extra_tags = (
        '    <tag k="route_edge_id" v="11"/>' if route_edge_tags else ""
    )
    relations = [
        relation(
            100, 10, 11, 12, first_speed, omit_role,
            extra_tags=first_extra_tags,
        ),
        relation(200, 20, 21, 22, extra_tags=second_extra_tags),
    ]
    return "<osm version=\"0.6\">\n" + "\n".join(nodes + ways + relations) + "\n</osm>\n"


def estimated_boundary_candidate_osm(*, width="1.5", include_guard=True):
    tags = [
        '    <tag k="boundary_model" '
        'v="trajectory_derived_estimated_drivable_corridor"/>',
        f'    <tag k="estimated_vehicle_width_m" v="{width}"/>',
        '    <tag k="estimated_front_extent_m" v="2.0"/>',
        '    <tag k="estimated_rear_extent_m" v="1.0"/>',
        '    <tag k="vehicle_minimum_turning_radius_m" v="4.8"/>',
        '    <tag k="estimated_lateral_margin_m" v="0.2"/>',
        '    <tag k="estimated_boundary_algorithm" '
        'v="oriented_rectangular_swept_envelope"/>',
        '    <tag k="vehicle_profile" v="car"/>',
        '    <tag k="vehicle_base_reference" v="rear_axle_ground_projection"/>',
        '    <tag k="vehicle_dimensions_evidence_source" v="catalog_estimated"/>',
        '    <tag k="vehicle_dimensions_evidence_confidence" v="medium"/>',
        '    <tag k="vehicle_dimensions_verified" v="no"/>',
    ]
    if include_guard:
        tags.extend(
            [
                '    <tag k="estimated_boundary_interpolation_guard_m" v="0.05"/>',
                '    <tag k="estimated_effective_lateral_margin_m" v="0.25"/>',
                '    <tag k="estimated_longitudinal_endpoint_guard_m" v="0.05"/>',
            ]
        )
    metadata = "\n".join(tags)
    osm = candidate_osm(route_edge_tags=True).replace(
        "\n  </relation>", f"\n{metadata}\n  </relation>"
    )
    # The open replay contract includes the base poses at x=0 and x=10.
    # Preserve centerlines while extending only boundary end caps by the
    # tagged 1.0 m rear / 2.0 m front extents plus the 0.05 m guard.
    osm = osm.replace(node(1, 0, 1), node(1, -1.05, 1))
    osm = osm.replace(node(3, 0, -1), node(3, -1.05, -1))
    osm = osm.replace(node(7, 10, 1), node(7, 12.05, 1))
    osm = osm.replace(node(8, 10, -1), node(8, 12.05, -1))
    return osm


def semantic_speed_candidate_osm(tamper=""):
    """Two raw replay Edges, with the first losslessly split for speed."""
    def semantic_node(node_id, x, y, z=0.0):
        return f'''  <node id="{node_id}">
    <tag k="local_x" v="{x}"/>
    <tag k="local_y" v="{y}"/>
    <tag k="ele" v="{z}"/>
  </node>'''

    split_z = 0.25 if tamper == "3d_length" else 0.0
    nodes = [
        semantic_node(1, -1.05, 1), semantic_node(13, 2, 1),
        semantic_node(2, 5, 1), semantic_node(3, -1.05, -1),
        semantic_node(14, 2, -1), semantic_node(4, 5, -1),
        semantic_node(5, 0, 0), semantic_node(15, 2, 0, split_z),
        semantic_node(6, 5, 0), semantic_node(7, 12.05, 1),
        semantic_node(8, 12.05, -1), semantic_node(9, 10, 0),
    ]
    ways = [
        way(10, (1, 13)), way(11, (3, 14)), way(12, (5, 15)),
        way(13, (13, 2)), way(14, (14, 4)), way(15, (15, 6)),
        way(20, (2, 7)), way(21, (4, 8)), way(22, (6, 9)),
    ]
    metadata = """    <tag k="boundary_model" v="trajectory_derived_estimated_drivable_corridor"/>
    <tag k="estimated_vehicle_width_m" v="1.5"/>
    <tag k="estimated_front_extent_m" v="2.0"/>
    <tag k="estimated_rear_extent_m" v="1.0"/>
    <tag k="vehicle_minimum_turning_radius_m" v="4.8"/>
    <tag k="estimated_lateral_margin_m" v="0.2"/>
    <tag k="estimated_boundary_algorithm" v="oriented_rectangular_swept_envelope"/>
    <tag k="vehicle_profile" v="car"/>
    <tag k="vehicle_base_reference" v="rear_axle_ground_projection"/>
    <tag k="vehicle_dimensions_evidence_source" v="catalog_estimated"/>
    <tag k="vehicle_dimensions_evidence_confidence" v="medium"/>
    <tag k="vehicle_dimensions_verified" v="no"/>
    <tag k="estimated_boundary_interpolation_guard_m" v="0.05"/>
    <tag k="estimated_effective_lateral_margin_m" v="0.25"/>
    <tag k="estimated_longitudinal_endpoint_guard_m" v="0.05"/>"""
    second_start = 2.0
    source_length = 5.0
    if tamper == "gap":
        second_start = 2.1
    elif tamper == "overlap":
        second_start = 1.9
    elif tamper == "source_length":
        source_length = 4.9

    def semantic_tags(output_id, source_id, start_s, end_s, length, speed):
        return f'''    <tag k="route_edge_id" v="{output_id}"/>
    <tag k="source_route_edge_id" v="{source_id}"/>
    <tag k="source_start_s_m" v="{start_s}"/>
    <tag k="source_end_s_m" v="{end_s}"/>
    <tag k="source_edge_length_m" v="{length}"/>
    <tag k="semantic_segment" v="yes"/>
    <tag k="generator_speed_limit_mps" v="{speed}"/>
{metadata}'''

    relations = [
        relation(
            100, 10, 11, 12, speed="1.8",
            extra_tags=semantic_tags(100, 10, 0, 2, source_length, 0.5),
        ),
        relation(
            101, 13, 14, 15, speed="0.72",
            extra_tags=semantic_tags(
                101, 10, second_start, 5, source_length, 0.2
            ),
        ),
        relation(
            102, 20, 21, 22, speed="1.8",
            extra_tags=semantic_tags(102, 11, 0, 5, 5, 0.5),
        ),
    ]
    if tamper == "reorder":
        relations = [relations[2], relations[0], relations[1]]
    return "<osm version=\"0.6\">\n" + "\n".join(
        nodes + ways + relations
    ) + "\n</osm>\n"


def semantic_speed_source_graph():
    document = named_source_graph()
    for feature in document["features"]:
        feature["properties"]["cost"] = 5.0
    return document


def write_semantic_speed_candidate(directory, tamper=""):
    write_candidate(directory, osm=semantic_speed_candidate_osm(tamper))
    replay = json.dumps(semantic_speed_source_graph())
    for name in (
        "route_graph_autoware_replay_candidate.geojson",
        "route_graph_closed_course_replay_candidate.geojson",
    ):
        (directory / name).write_text(replay, encoding="utf-8")


def trajectory(y=0.0):
    # Includes segment-interior samples and a complete out-and-back traversal.
    xs = (0.0, 2.5, 5.0, 7.5, 10.0, 7.5, 5.0, 2.5, 0.0)
    return "".join(
        f"{index}.0 {x} {y} 0 0 0 0 1\n" for index, x in enumerate(xs)
    )


def write_candidate(directory, osm=None, tum=None):
    directory.mkdir(parents=True, exist_ok=True)
    osm_text = candidate_osm() if osm is None else osm
    (directory / "lanelet2_map_closed_course_experimental.osm").write_text(
        osm_text, encoding="utf-8"
    )
    (directory / "trajectory_processed.tum").write_text(
        trajectory() if tum is None else tum, encoding="utf-8"
    )
    if VALIDATOR.ESTIMATED_BOUNDARY_MODEL in osm_text:
        replay = json.dumps(named_source_graph())
        (directory / "route_graph_autoware_replay_candidate.geojson").write_text(
            replay, encoding="utf-8"
        )
        (directory / "route_graph_closed_course_replay_candidate.geojson").write_text(
            replay, encoding="utf-8"
        )


def named_candidate_osm(*, stop=True):
    first_tags = """    <tag k="named_route_id" v="42"/>
    <tag k="named_route_name" v="short course"/>
    <tag k="named_route_target" v="autoware"/>
    <tag k="named_route_order" v="0"/>
    <tag k="route_edge_id" v="10"/>"""
    second_tags = first_tags.replace('v="0"', 'v="1"', 1).replace(
        'route_edge_id" v="10"', 'route_edge_id" v="11"'
    )
    nodes = [
        node(1, 0, 1), node(2, 5, 1), node(3, 0, -1), node(4, 5, -1),
        node(5, 0, 0), node(6, 5, 0), node(7, 10, 1), node(8, 10, -1),
        node(9, 10, 0),
    ]
    ways = [
        way(10, (1, 2)), way(11, (3, 4)), way(12, (5, 6)),
        way(20, (2, 7)), way(21, (4, 8)), way(22, (6, 9)),
    ]
    regulatory_member = ""
    extra_relations = []
    if stop:
        nodes.extend([node(30, 7, -1.25), node(31, 7, 1.25)])
        ways.append(
            way(30, (30, 31)).replace(
                "</way>",
                """    <tag k="type" v="stop_line"/>
    <tag k="subtype" v="solid"/>
    <tag k="name" v="gate stop"/>
    <tag k="virtual" v="yes"/>
    <tag k="autogenerated" v="yes"/>
    <tag k="audit_source" v="navigation_authoring_gui"/>
    <tag k="physical_stop_sign_observed" v="no"/>
    <tag k="authored_stop_line_id" v="7"/>
    <tag k="route_edge_id" v="11"/>
    <tag k="route_edge_s_m" v="2"/>
    <tag k="authored_width_m" v="2.5"/>
    <tag k="navigation_target" v="autoware"/>
  </way>""",
            )
        )
        regulatory_member = (
            '\n    <member type="relation" ref="300" role="regulatory_element"/>'
        )
        extra_relations.append(
            """  <relation id="300">
    <member type="way" ref="30" role="ref_line"/>
    <tag k="type" v="regulatory_element"/>
    <tag k="subtype" v="traffic_sign"/>
    <tag k="sign_type" v="stop_sign"/>
    <tag k="name" v="gate stop"/>
    <tag k="virtual" v="yes"/>
    <tag k="autogenerated" v="yes"/>
    <tag k="audit_source" v="navigation_authoring_gui"/>
    <tag k="physical_stop_sign_observed" v="no"/>
    <tag k="authored_stop_line_id" v="7"/>
  </relation>"""
        )
    relations = [
        relation(100, 10, 11, 12, extra_tags=first_tags),
        relation(
            200,
            20,
            21,
            22,
            extra_members=regulatory_member,
            extra_tags=second_tags,
        ),
        *extra_relations,
    ]
    return "<osm version=\"0.6\">\n" + "\n".join(nodes + ways + relations) + "\n</osm>\n"


def named_candidate_with_synthetic_planning_support(*, tamper=""):
    common = """    <tag k="boundary_model" v="trajectory_derived_estimated_drivable_corridor"/>
    <tag k="estimated_vehicle_width_m" v="1.5"/>
    <tag k="estimated_front_extent_m" v="2.0"/>
    <tag k="estimated_rear_extent_m" v="1.0"/>
    <tag k="vehicle_minimum_turning_radius_m" v="4.8"/>
    <tag k="estimated_lateral_margin_m" v="0.2"/>
    <tag k="estimated_boundary_interpolation_guard_m" v="0.05"/>
    <tag k="estimated_effective_lateral_margin_m" v="0.25"/>
    <tag k="estimated_longitudinal_endpoint_guard_m" v="0.05"/>
    <tag k="estimated_boundary_algorithm" v="oriented_rectangular_swept_envelope"/>
    <tag k="vehicle_profile" v="car"/>
    <tag k="vehicle_base_reference" v="rear_axle_ground_projection"/>
    <tag k="vehicle_dimensions_evidence_source" v="catalog_estimated"/>
    <tag k="vehicle_dimensions_evidence_confidence" v="medium"/>
    <tag k="vehicle_dimensions_verified" v="no"/>
    <tag k="synthetic_planning_support_count" v="2"/>"""
    raw_first = """    <tag k="named_route_id" v="42"/>
    <tag k="named_route_name" v="short course"/>
    <tag k="named_route_target" v="autoware"/>
    <tag k="named_route_order" v="0"/>
    <tag k="route_edge_id" v="10"/>""" + "\n" + common
    raw_second = raw_first.replace('v="0"', 'v="1"', 1).replace(
        'route_edge_id" v="10"', 'route_edge_id" v="11"'
    )

    def support_tags(role, edge_id, adjacent_output, adjacent_source, node_id,
                     raw_s, raw_x, synthetic_x, required, actual):
        center_length = "0.4" if tamper == f"{role}_length" else "0.5"
        support_role = "tail" if tamper == "duplicate_role" and role == "head" else role
        required_isolation = math.hypot(2.0, 0.75 + 0.2) + 0.5
        actual_isolation = 2.0 if tamper == f"{role}_isolation" else 5.5
        nearest_nonadjacent = 11 if role == "head" else 10
        return f'''    <tag k="route_edge_id" v="{edge_id}"/>
    <tag k="synthetic_planning_support" v="yes"/>
    <tag k="synthetic_test_staging" v="yes"/>
    <tag k="surveyed" v="no"/>
    <tag k="deployment_ready" v="no"/>
    <tag k="support_is_part_of_raw_counts" v="no"/>
    <tag k="support_is_part_of_named_route" v="no"/>
    <tag k="support_is_raw_coverage" v="no"/>
    <tag k="planning_support_contract_version" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_CONTRACT_VERSION}"/>
    <tag k="planning_support_role" v="{support_role}"/>
    <tag k="planning_support_source" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_SOURCE}"/>
    <tag k="planning_support_estimated" v="yes"/>
    <tag k="planning_support_geometry_kind" v="straight"/>
    <tag k="planning_support_adjacent_output_edge_id" v="{adjacent_output}"/>
    <tag k="planning_support_adjacent_source_edge_id" v="{adjacent_source}"/>
    <tag k="planning_support_raw_endpoint_node_id" v="{node_id}"/>
    <tag k="planning_support_source_edge_length_m" v="5"/>
    <tag k="planning_support_raw_endpoint_s_m" v="{raw_s}"/>
    <tag k="planning_support_raw_endpoint_x" v="{raw_x}"/>
    <tag k="planning_support_raw_endpoint_y" v="0"/>
    <tag k="planning_support_raw_endpoint_z" v="0"/>
    <tag k="planning_support_synthetic_endpoint_x" v="{synthetic_x}"/>
    <tag k="planning_support_synthetic_endpoint_y" v="0"/>
    <tag k="planning_support_synthetic_endpoint_z" v="0"/>
    <tag k="planning_support_tangent_x" v="1"/>
    <tag k="planning_support_tangent_y" v="0"/>
    <tag k="planning_support_outer_tangent_x" v="1"/>
    <tag k="planning_support_outer_tangent_y" v="0"/>
    <tag k="planning_support_centerline_planar_length_m" v="{center_length}"/>
    <tag k="planning_support_centerline_3d_length_m" v="0.5"/>
    <tag k="planning_support_endpoint_allowance_m" v="0.5"/>
    <tag k="planning_support_required_boundary_beyond_raw_endpoint_m" v="{required}"/>
    <tag k="planning_support_actual_left_boundary_beyond_raw_endpoint_m" v="{actual}"/>
    <tag k="planning_support_actual_right_boundary_beyond_raw_endpoint_m" v="{actual}"/>
    <tag k="planning_support_search_step_m" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_SEARCH_STEP_M}"/>
    <tag k="planning_support_path_sample_spacing_m" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_PATH_SAMPLE_SPACING_M}"/>
    <tag k="planning_support_search_max_length_m" v="0.5"/>
    <tag k="planning_support_selected_candidate_index" v="0"/>
    <tag k="planning_support_candidate_count_tested" v="1"/>
    <tag k="planning_support_individually_valid_candidate_rank" v="1"/>
    <tag k="planning_support_rejected_kinematic_candidates" v="0"/>
    <tag k="planning_support_rejected_invalid_geometry_candidates" v="0"/>
    <tag k="planning_support_rejected_outer_raw_overlap_candidates" v="0"/>
    <tag k="planning_support_rejected_insufficient_outer_pose_isolation_candidates" v="0"/>
    <tag k="planning_support_rejected_raw_polygon_reentry_candidates" v="0"/>
    <tag k="planning_support_rejected_nonadjacent_transition_candidates" v="0"/>
    <tag k="planning_support_turn_radius_m" v="0"/>
    <tag k="planning_support_turn_angle_rad" v="0"/>
    <tag k="planning_support_straight_length_m" v="0.5"/>
    <tag k="planning_support_maximum_curvature_inv_m" v="0"/>
    <tag k="planning_support_actual_maximum_curvature_inv_m" v="0"/>
    <tag k="planning_support_kinematic_valid" v="yes"/>
    <tag k="planning_support_outer_endpoint_unique" v="yes"/>
    <tag k="planning_support_outer_endpoint_route_polygon_edge_ids" v="{edge_id}"/>
    <tag k="planning_support_outer_footprint_raw_overlap_edge_ids" v=""/>
    <tag k="planning_support_outer_pose_isolation_scope" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_OUTER_POSE_ISOLATION_SCOPE}"/>
    <tag k="planning_support_outer_pose_isolation_derivation" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_OUTER_POSE_ISOLATION_DERIVATION}"/>
    <tag k="planning_support_required_outer_pose_nonadjacent_raw_centerline_isolation_m" v="{required_isolation}"/>
    <tag k="planning_support_actual_outer_pose_nonadjacent_raw_centerline_isolation_m" v="{actual_isolation}"/>
    <tag k="planning_support_outer_pose_nonadjacent_raw_centerline_count" v="1"/>
    <tag k="planning_support_outer_pose_nearest_nonadjacent_raw_centerline_edge_ids" v="{nearest_nonadjacent}"/>
    <tag k="planning_support_raw_overlap_single_transition" v="yes"/>
    <tag k="planning_support_raw_overlap_transition_length_m" v="0"/>
    <tag k="planning_support_nonadjacent_raw_overlap_edge_ids" v=""/>
    <tag k="planning_support_nonadjacent_raw_overlap_transition_length_m" v="0"/>
    <tag k="planning_support_maximum_nonadjacent_raw_overlap_transition_length_m" v="12.6"/>
    <tag k="planning_support_outer_footprint_contained" v="yes"/>
    <tag k="planning_support_connection_footprint_contained" v="yes"/>
    <tag k="planning_support_candidate_pool_limit" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_POOL_LIMIT}"/>
    <tag k="planning_support_head_candidate_pool_size" v="1"/>
    <tag k="planning_support_tail_candidate_pool_size" v="1"/>
    <tag k="planning_support_candidate_pair_evaluation_limit" v="{VALIDATOR.SYNTHETIC_PLANNING_SUPPORT_CANDIDATE_PAIR_EVALUATION_LIMIT}"/>
    <tag k="planning_support_candidate_pairs_tested" v="1"/>
    <tag k="planning_support_selected_candidate_pair_rank" v="1"/>
    <tag k="planning_support_rejected_final_boundary_pairs" v="0"/>
    <tag k="planning_support_rejected_final_outer_membership_pairs" v="0"/>
    <tag k="planning_support_rejected_final_transition_pairs" v="0"/>
    <tag k="planning_support_rejected_final_containment_pairs" v="0"/>
    <tag k="planning_support_collision_scope" v="route_polygon_single_transition_only"/>
    <tag k="provenance" v="synthetic_test_kinematic_staging"/>
    <tag k="observed_driven" v="no"/>
    <tag k="validation_status" v="estimated_synthetic_test_staging"/>
    <tag k="production_ready" v="no"/>
    <tag k="physical_boundaries_verified" v="no"/>
{common}'''

    head_center_x = -0.4 if tamper == "head_geometry" else -0.5
    head_left_x = -1.4 if tamper == "head_boundary" else -1.55
    nodes = [
        node(1, 0, 1), node(2, 5, 1), node(3, 0, -1), node(4, 5, -1),
        node(5, 0, 0), node(6, 5, 0), node(7, 10, 1), node(8, 10, -1),
        node(9, 10, 0), node(40, head_left_x, 1), node(41, -1.55, -1),
        node(42, head_center_x, 0), node(43, 12.55, 1), node(44, 12.55, -1),
        node(45, 10.5, 0),
        *(
            node(45 + index, head_center_x + (0.0 - head_center_x) * index / 5.0, 0)
            for index in range(1, 5)
        ),
        *(node(49 + index, 10.0 + 0.1 * index, 0) for index in range(1, 5)),
        node(30, 7, -1.25), node(31, 7, 1.25),
    ]
    ways = [
        way(40, (40, 1)), way(41, (41, 3)),
        way(42, (42, 46, 47, 48, 49, 5)),
        way(10, (1, 2)), way(11, (3, 4)), way(12, (5, 6)),
        way(20, (2, 7)), way(21, (4, 8)), way(22, (6, 9)),
        way(43, (7, 43)), way(44, (8, 44)),
        way(45, (9, 50, 51, 52, 53, 45)),
        way(30, (30, 31)).replace(
            "</way>",
            """    <tag k="type" v="stop_line"/>
    <tag k="subtype" v="solid"/>
    <tag k="name" v="gate stop"/>
    <tag k="virtual" v="yes"/>
    <tag k="autogenerated" v="yes"/>
    <tag k="audit_source" v="navigation_authoring_gui"/>
    <tag k="physical_stop_sign_observed" v="no"/>
    <tag k="authored_stop_line_id" v="7"/>
    <tag k="route_edge_id" v="11"/>
    <tag k="route_edge_s_m" v="2"/>
    <tag k="authored_width_m" v="2.5"/>
    <tag k="navigation_target" v="autoware"/>
  </way>""",
        ),
    ]
    relations = [
        relation(400, 40, 41, 42, extra_tags=support_tags(
            "head", 14, 10, 10, 1, 0, 0, -0.5, 1.5, 1.55)),
        relation(100, 10, 11, 12, extra_tags=raw_first),
        relation(
            200, 20, 21, 22,
            extra_members='\n    <member type="relation" ref="300" role="regulatory_element"/>',
            extra_tags=raw_second,
        ),
        relation(500, 43, 44, 45, extra_tags=support_tags(
            "tail", 15, 11, 11, 3, 5, 10, 10.5, 2.5, 2.55)),
        """  <relation id="300">
    <member type="way" ref="30" role="ref_line"/>
    <tag k="type" v="regulatory_element"/>
    <tag k="subtype" v="traffic_sign"/>
    <tag k="sign_type" v="stop_sign"/>
    <tag k="name" v="gate stop"/>
    <tag k="virtual" v="yes"/>
    <tag k="autogenerated" v="yes"/>
    <tag k="audit_source" v="navigation_authoring_gui"/>
    <tag k="physical_stop_sign_observed" v="no"/>
    <tag k="authored_stop_line_id" v="7"/>
  </relation>""",
    ]
    if tamper == "missing_head":
        relations.pop(0)
    return "<osm version=\"0.6\">\n" + "\n".join(nodes + ways + relations) + "\n</osm>\n"


def named_source_graph(second_y=0.0):
    return {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"id": 10, "startid": 1, "endid": 2},
                "geometry": {
                    "type": "MultiLineString",
                    "coordinates": [[[0.0, 0.0], [5.0, 0.0]]],
                },
            },
            {
                "type": "Feature",
                "properties": {"id": 11, "startid": 2, "endid": 3},
                "geometry": {
                    "type": "MultiLineString",
                    "coordinates": [[[5.0, second_y], [10.0, second_y]]],
                },
            },
        ],
    }


def named_source_graph_with_reversed_second_edge():
    document = named_source_graph()
    document["features"][1]["geometry"]["coordinates"][0].reverse()
    return document


def terminal_support_topology_graph(
    *, passable=True, support_y=0.0, support_end=None
):
    document = named_source_graph()
    for feature in document["features"]:
        feature["properties"]["metadata"] = {"passable": True}
    if support_end is None:
        support_end = [15.0, support_y]
    document["features"].append(
        {
            "type": "Feature",
            "properties": {
                "id": 12,
                "startid": 3,
                "endid": 4,
                "metadata": {"passable": passable},
            },
            "geometry": {
                "type": "MultiLineString",
                "coordinates": [[[10.0, support_y], support_end]],
            },
        }
    )
    return document


def named_candidate_with_terminal_support():
    osm = named_candidate_osm()
    osm = osm.replace(
        node(9, 10, 0),
        node(9, 10, 0) + "\n" + node(40, 15, 1) + "\n"
        + node(41, 15, -1) + "\n" + node(42, 15, 0),
    )
    osm = osm.replace(way(20, (2, 7)), way(20, (2, 7, 40)))
    osm = osm.replace(way(21, (4, 8)), way(21, (4, 8, 41)))
    osm = osm.replace(way(22, (6, 9)), way(22, (6, 9, 42)))
    marker = '    <tag k="route_edge_id" v="11"/>'
    prefix, separator, suffix = osm.rpartition(marker)
    assert separator
    osm = prefix + (
        marker + '\n'
        '    <tag k="autoware_terminal_support" v="yes"/>\n'
        '    <tag k="terminal_support_edge_ids" v="12"/>\n'
        '    <tag k="terminal_support_length_m" v="5"/>\n'
        '    <tag k="named_route_source_length_m" v="5"/>\n'
        '    <tag k="terminal_support_source" '
        'v="closed_course_semantic_topology"/>'
    ) + suffix
    return osm


def full_map_trajectory():
    xs = (0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 12.5, 10.0, 7.5,
          5.0, 2.5, 0.0)
    return "".join(
        f"{index}.0 {x} 0 0 0 0 0 1\n" for index, x in enumerate(xs)
    )


def named_candidate_with_non_route_lanelet():
    extra = "\n".join(
        [
            node(13, 15, 1),
            node(14, 15, -1),
            node(15, 15, 0),
            way(40, (7, 13)),
            way(41, (8, 14)),
            way(42, (9, 15)),
            relation(
                400,
                40,
                41,
                42,
                extra_tags='    <tag k="route_edge_id" v="12"/>',
            ),
        ]
    )
    return named_candidate_osm().replace("</osm>", extra + "\n</osm>")


def source_graph_with_non_route_edge():
    document = named_source_graph()
    document["features"].append(
        {
            "type": "Feature",
            "properties": {"id": 12, "startid": 3, "endid": 4},
            "geometry": {
                "type": "MultiLineString",
                "coordinates": [[[10.0, 0.0], [15.0, 0.0]]],
            },
        }
    )
    return document


def write_named_candidate(
    directory,
    *,
    osm=None,
    source_graph=None,
    full_source_graph=None,
    topology_graph=None,
    tum=None,
):
    write_candidate(
        directory,
        osm=named_candidate_osm() if osm is None else osm,
        tum=trajectory() if tum is None else tum,
    )
    (directory / "route_graph_autoware_replay_candidate.geojson").write_text(
        json.dumps(
            named_source_graph() if full_source_graph is None else full_source_graph
        ),
        encoding="utf-8",
    )
    (directory / "route_graph_closed_course_replay_candidate.geojson").write_text(
        (directory / "route_graph_autoware_replay_candidate.geojson").read_text(
            encoding="utf-8"
        ),
        encoding="utf-8",
    )
    (directory / "route_graph_autoware_selected_mission.geojson").write_text(
        json.dumps(named_source_graph() if source_graph is None else source_graph),
        encoding="utf-8",
    )
    (directory / "navigation_authoring_autoware_replay.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "frame_id": "map",
                "graph_fingerprint": "0123456789abcdef",
                "routes": [
                    {
                        "id": 42,
                        "name": "short course",
                        "target": "autoware",
                        "start_node_id": 1,
                        "end_node_id": 3,
                        "ordered_edge_ids": [10, 11],
                        "validation_requested": True,
                        "promotion_requested": True,
                    }
                ],
                "stop_lines": [
                    {
                        "id": 7,
                        "name": "gate stop",
                        "edge_id": 11,
                        "s": 2.0,
                        "width_m": 2.5,
                        "anchor": [7.0, 0.0, 0.0],
                        "target": "autoware",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    (directory / "navigation_authoring_closed_course_status.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "frame_id": "map",
                "graph_fingerprint": "0123456789abcdef",
                "autoware": {
                    "selected_route_id": 42,
                    "promoted": True,
                    "stop_lines_valid": True,
                },
                "nav2": {
                    "selected_route_id": None,
                    "promoted": False,
                    "stop_lines_valid": True,
                },
                "errors": [],
                "routes": [
                    {
                        "id": 42,
                        "name": "short course",
                        "target": "autoware",
                        "valid": True,
                        "promotion_eligible": True,
                    }
                ],
                "stop_lines": [
                    {
                        "id": 7,
                        "name": "gate stop",
                        "target": "autoware",
                        "valid": True,
                        "errors": [],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    if topology_graph is not None:
        (directory / "route_graph_closed_course_topology_candidate.geojson").write_text(
            json.dumps(topology_graph), encoding="utf-8"
        )


def manual_topology_candidate_osm():
    metadata = """    <tag k="boundary_model" v="trajectory_derived_estimated_drivable_corridor"/>
    <tag k="estimated_vehicle_width_m" v="0.8"/>
    <tag k="estimated_front_extent_m" v="0.1"/>
    <tag k="estimated_rear_extent_m" v="0.1"/>
    <tag k="vehicle_minimum_turning_radius_m" v="1.0"/>
    <tag k="estimated_lateral_margin_m" v="0.1"/>
    <tag k="estimated_boundary_interpolation_guard_m" v="0.05"/>
    <tag k="estimated_effective_lateral_margin_m" v="0.15"/>
    <tag k="estimated_longitudinal_endpoint_guard_m" v="0.05"/>
    <tag k="estimated_boundary_algorithm" v="oriented_rectangular_swept_envelope"/>
    <tag k="vehicle_profile" v="car"/>
    <tag k="vehicle_base_reference" v="rear_axle_ground_projection"/>
    <tag k="vehicle_dimensions_evidence_source" v="catalog_estimated"/>
    <tag k="vehicle_dimensions_evidence_confidence" v="medium"/>
    <tag k="vehicle_dimensions_verified" v="no"/>
    <tag k="centerline_source" v="edited_topology"/>
    <tag k="provenance" v="user_authored_centerline"/>
    <tag k="observed_driven" v="no"/>
    <tag k="validation_status" v="user_authored_vehicle_footprint_validated_candidate"/>"""
    osm = named_candidate_osm(stop=False).replace(
        "\n  </relation>", f"\n{metadata}\n  </relation>"
    )
    # Centerlines remain exactly [0,10], while the swept-boundary caps cover
    # the tagged 0.1 m rear/front extents and 0.05 m endpoint guard.
    osm = osm.replace(node(1, 0, 1), node(1, -0.15, 1))
    osm = osm.replace(node(3, 0, -1), node(3, -0.15, -1))
    osm = osm.replace(node(7, 10, 1), node(7, 10.15, 1))
    osm = osm.replace(node(8, 10, -1), node(8, 10.15, -1))
    return osm


def write_manual_topology_candidate(directory):
    write_candidate(
        directory,
        osm=manual_topology_candidate_osm(),
        # Deliberately unrelated to the user-authored centerlines. Manual maps
        # are bound to their raw edited topology, not full acquisition replay.
        tum=trajectory(y=5.0),
    )
    for replay_name in (
        "route_graph_autoware_replay_candidate.geojson",
        "route_graph_closed_course_replay_candidate.geojson",
    ):
        replay_path = directory / replay_name
        if replay_path.exists():
            replay_path.unlink()
    source = json.dumps(named_source_graph())
    (directory / "route_graph_autoware_topology_source.geojson").write_text(
        source, encoding="utf-8"
    )
    (directory / "route_graph_autoware_topology_selected_mission.geojson").write_text(
        source, encoding="utf-8"
    )
    (directory / "vector_map_source.tsv").write_text(
        "LMMG_VECTOR_MAP_SOURCE\t1\n"
        "SOURCE\tedited_topology\n"
        "FRAME\tmap\n"
        "GRAPH_FINGERPRINT\t0123456789abcdef\n",
        encoding="utf-8",
    )
    (directory / "navigation_target_readiness.yaml").write_text(
        "schema_version: 3\ngeneration_complete: true\n"
        "requested_target_mode: \"autoware\"\nautoware:\n"
        "  enabled: true\n  centerline_source: \"edited_topology\"\n"
        "  closed_course_experimental_ready: true\n",
        encoding="utf-8",
    )
    authoring = {
        "schema_version": 1,
        "frame_id": "map",
        "graph_fingerprint": "0123456789abcdef",
        "routes": [
            {
                "id": 42,
                "name": "short course",
                "target": "autoware",
                "start_node_id": 1,
                "end_node_id": 3,
                "ordered_edge_ids": [10, 11],
                "validation_requested": True,
                "promotion_requested": True,
            }
        ],
        "stop_lines": [],
    }
    (directory / "navigation_authoring_autoware_topology.json").write_text(
        json.dumps(authoring), encoding="utf-8"
    )
    status = {
        "schema_version": 1,
        "frame_id": "map",
        "graph_fingerprint": "0123456789abcdef",
        "autoware": {
            "selected_route_id": 42,
            "promoted": True,
            "stop_lines_valid": True,
        },
        "nav2": {
            "selected_route_id": None,
            "promoted": False,
            "stop_lines_valid": True,
        },
        "errors": [],
        "routes": [
            {
                "id": 42,
                "name": "short course",
                "target": "autoware",
                "valid": True,
                "promotion_eligible": True,
            }
        ],
        "stop_lines": [],
    }
    (directory / "navigation_authoring_autoware_topology_status.json").write_text(
        json.dumps(status), encoding="utf-8"
    )
    (directory / "semantic_features_autoware_topology.tsv").write_text(
        "LMMG_SEMANTICS\t2\nFRAME\tmap\n", encoding="utf-8"
    )
    (directory / "semantic_features_autoware_topology.geojson").write_text(
        '{"type":"FeatureCollection","features":[]}\n', encoding="utf-8"
    )


class ValidateAutowareCandidateTest(unittest.TestCase):
    def test_named_route_terminal_overlap_is_a_visible_nonblocking_warning(self):
        first = {
            "label": "100",
            "left": [(0.0, 1.0, 0.0), (5.0, 1.0, 0.0)],
            "right": [(0.0, -1.0, 0.0), (5.0, -1.0, 0.0)],
            "center": [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
        }
        last = {
            "label": "200",
            "left": [(-0.5, 1.0, 0.0), (1.0, 1.0, 0.0)],
            "right": [(-0.5, -1.0, 0.0), (1.0, -1.0, 0.0)],
            "center": [(0.5, 0.0, 0.0), (0.0, 0.0, 0.0)],
        }
        named_route = {
            "records": [{"lanelet": first}, {"lanelet": last}],
        }
        report = {"counts": {}, "warnings": []}
        VALIDATOR._audit_named_route_terminal_membership(
            named_route, {"lanelets": [first, last]}, report
        )
        self.assertFalse(report["named_route_terminal_membership"]["unique"])
        self.assertEqual(report["counts"]["named_route_start_lanelet_memberships"], 2)
        self.assertEqual(report["counts"]["named_route_goal_lanelet_memberships"], 2)
        self.assertEqual(
            [warning["code"] for warning in report["warnings"]],
            ["ambiguous_named_route_terminal_lanelet_membership"],
        )

    def test_manual_topology_uses_raw_graph_not_full_trajectory_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_manual_topology_candidate(output)
            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertEqual(result["centerline_source"], "edited_topology")
            self.assertTrue(result["user_authored"])
            self.assertEqual(result["coverage_reference"], "named_route_source_graph")
            self.assertEqual(result["counts"]["full_map_source_edges"], 2)
            self.assertEqual(result["counts"]["full_map_lanelet_edges"], 2)
            self.assertEqual(
                result["metrics"]["full_map_source_centerline_coverage"], 1.0
            )
            self.assertEqual(result["metrics"]["acceptance_centerline_coverage"], 1.0)
            self.assertEqual(result["metrics"]["trajectory_centerline_coverage"], 0.0)
            self.assertNotIn(
                "insufficient_trajectory_coverage",
                {error["code"] for error in result["errors"]},
            )
            self.assertEqual(
                result["input"]["navigation_authoring_scope"],
                "autoware_edited_topology",
            )
            self.assertEqual(result["named_route"]["ordered_edge_ids"], [10, 11])

            source_path = output / "route_graph_autoware_topology_source.geojson"
            reversed_source = json.loads(source_path.read_text(encoding="utf-8"))
            reversed_source["features"][1]["geometry"]["coordinates"][0].reverse()
            source_path.write_text(json.dumps(reversed_source), encoding="utf-8")
            failed = VALIDATOR.validate_candidate(output)
            self.assertFalse(failed["accepted"])
            self.assertIn(
                "full_map_edge_orientation_mismatch",
                {error["code"] for error in failed["errors"]},
            )

    def test_manual_topology_binds_source_selection_and_lanelet_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_manual_topology_candidate(output)
            selection_path = output / "vector_map_source.tsv"
            selection_path.write_text(
                selection_path.read_text(encoding="utf-8").replace(
                    "0123456789abcdef", "fedcba9876543210"
                ),
                encoding="utf-8",
            )
            mismatched = VALIDATOR.validate_candidate(output)
            self.assertFalse(mismatched["accepted"])
            self.assertIn(
                "vector_map_source_authoring_identity_mismatch",
                {error["code"] for error in mismatched["errors"]},
            )

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_manual_topology_candidate(output)
            osm_path = output / "lanelet2_map_closed_course_experimental.osm"
            osm_path.write_text(
                osm_path.read_text(encoding="utf-8").replace(
                    'provenance" v="user_authored_centerline',
                    'provenance" v="observed_driven_trajectory',
                    1,
                ),
                encoding="utf-8",
            )
            mismatched = VALIDATOR.validate_candidate(output)
            self.assertFalse(mismatched["accepted"])
            self.assertIn(
                "invalid_user_authored_lanelet_provenance",
                {error["code"] for error in mismatched["errors"]},
            )

    def test_indexed_point_in_polygon_matches_legacy_predicate(self):
        polygons = {
            "convex_ccw": [
                (-2.0, -2.0, 0.0),
                (2.0, -2.0, 0.0),
                (2.0, 2.0, 0.0),
                (-2.0, 2.0, 0.0),
            ],
            "concave_ccw": [
                (-4.0, -4.0, 0.0),
                (0.0, -4.0, 0.0),
                (0.0, -2.0, 0.0),
                (-2.0, -2.0, 0.0),
                (-2.0, 0.0, 0.0),
                (-4.0, 0.0, 0.0),
            ],
            "cell_boundary_negative": [
                (-4.0, -2.0, 0.0),
                (-2.0, -2.0, 0.0),
                (-2.0, 2.0, 0.0),
                (-4.0, 2.0, 0.0),
            ],
        }
        polygons["convex_cw"] = list(reversed(polygons["convex_ccw"]))
        polygons["concave_cw"] = list(reversed(polygons["concave_ccw"]))

        coordinates = [value * 0.25 for value in range(-18, 19)]
        points = {(x, y) for x in coordinates for y in coordinates}
        threshold = 1.0e-7
        threshold_values = (
            -threshold,
            math.nextafter(-threshold, -math.inf),
            math.nextafter(-threshold, math.inf),
            0.0,
            math.nextafter(threshold, -math.inf),
            threshold,
            math.nextafter(threshold, math.inf),
        )
        for boundary in (-4.0, -2.0, 0.0, 2.0, 4.0):
            for delta in threshold_values:
                # Exercise horizontal/vertical edges, exact vertices, the 1e-7
                # boundary threshold, negative cells, and both sides of 2 m cells.
                points.add((boundary + delta, -2.0))
                points.add((boundary + delta, 0.0))
                points.add((-2.0, boundary + delta))
                points.add((0.0, boundary + delta))
            points.add((math.nextafter(boundary, -math.inf), 0.5))
            points.add((math.nextafter(boundary, math.inf), 0.5))

        for name, polygon in polygons.items():
            prepared = VALIDATOR._prepare_polygon_xy(polygon)
            boundary_order = {edge: edge for edge in range(len(polygon))}
            ray_order = {
                edge: order
                for order, edge in enumerate(
                    (len(polygon) - 1,) + tuple(range(len(polygon) - 1))
                )
            }
            for candidates in prepared.boundary_spatial_index.values():
                self.assertEqual(
                    list(candidates),
                    sorted(candidates, key=boundary_order.__getitem__),
                )
            for candidates in prepared.ray_y_spatial_index.values():
                self.assertEqual(
                    list(candidates), sorted(candidates, key=ray_order.__getitem__)
                )
            for point in points:
                with self.subTest(polygon=name, point=point):
                    expected = legacy_point_in_polygon_xy(point, polygon)
                    self.assertEqual(
                        VALIDATOR._point_in_prepared_polygon_xy(point, prepared),
                        expected,
                    )
                    self.assertEqual(
                        VALIDATOR._point_in_polygon_xy(point, polygon), expected
                    )

    def test_indexed_adjacent_polygon_union_matches_legacy_shared_boundary(self):
        polygons = [
            [
                (-4.0, -2.0, 0.0),
                (-2.0, -2.0, 0.0),
                (-2.0, 2.0, 0.0),
                (-4.0, 2.0, 0.0),
            ],
            [
                (-2.0, -2.0, 0.0),
                (0.0, -2.0, 0.0),
                (0.0, 2.0, 0.0),
                (-2.0, 2.0, 0.0),
            ],
        ]
        prepared, spatial_index = VALIDATOR._prepare_polygon_union_xy(polygons)
        offsets = (
            math.nextafter(-1.0e-7, -math.inf),
            -1.0e-7,
            math.nextafter(-1.0e-7, math.inf),
            0.0,
            math.nextafter(1.0e-7, -math.inf),
            1.0e-7,
            math.nextafter(1.0e-7, math.inf),
        )
        points = {
            (-2.0 + offset, y)
            for offset in offsets
            for y in (-2.0, -1.0, 0.0, 1.0, 2.0)
        }
        points.update(
            {
                (-4.0, 0.0),
                (0.0, 0.0),
                (-4.25, 0.0),
                (0.25, 0.0),
                (-3.0, math.nextafter(2.0, math.inf)),
                (-1.0, math.nextafter(-2.0, -math.inf)),
            }
        )
        for point in points:
            with self.subTest(point=point):
                self.assertEqual(
                    VALIDATOR._point_covered_by_lanelets(
                        point, prepared, spatial_index
                    ),
                    legacy_point_covered_by_polygons(point, polygons),
                )

    def test_cached_footprint_offsets_match_legacy_nested_grid_order(self):
        cases = (
            (3.35, 0.6, 1.995, 0.1),
            (2.0, 1.0, 1.5, 0.1),
            (0.2, 0.2, 0.2, 0.2),
        )
        VALIDATOR._footprint_grid_offsets.cache_clear()
        for front, rear, width, spacing in cases:
            longitudinal_count = max(1, math.ceil((front + rear) / spacing))
            lateral_count = max(1, math.ceil(width / spacing))
            expected = tuple(
                (
                    -rear + (front + rear) * index / longitudinal_count,
                    -0.5 * width + width * lateral_index / lateral_count,
                )
                for index in range(longitudinal_count + 1)
                for lateral_index in range(lateral_count + 1)
            )
            with self.subTest(case=(front, rear, width, spacing)):
                first = VALIDATOR._footprint_grid_offsets(
                    front, rear, width, spacing
                )
                second = VALIDATOR._footprint_grid_offsets(
                    front, rear, width, spacing
                )
                self.assertEqual(first, expected)
                self.assertIs(first, second)

    def test_support_chord_bound_uses_pre_serialization_piece_count(self):
        radius = 4.8
        generated_angle = math.radians(75.0)
        serialized_angle = 1.308996939
        sampled_length = 6.5
        serialized_straight = 0.21681469282
        serialized_analytic_length = (
            serialized_straight + radius * serialized_angle
        )
        pieces = 65
        arc_length = radius * generated_angle
        points = []
        for piece in range(pieces + 1):
            arc = sampled_length * piece / pieces
            if arc <= arc_length:
                points.append(
                    (
                        radius * math.sin(arc / radius),
                        radius * (1.0 - math.cos(arc / radius)),
                    )
                )
            else:
                points.append(
                    (
                        radius * math.sin(generated_angle)
                        + math.cos(generated_angle) * (arc - arc_length),
                        radius * (1.0 - math.cos(generated_angle))
                        + math.sin(generated_angle) * (arc - arc_length),
                    )
                )
        planar_length = sum(
            math.dist(first, second)
            for first, second in zip(points, points[1:])
        )
        deficit = serialized_analytic_length - planar_length
        limit = VALIDATOR._planning_support_chord_deficit_limit(
            serialized_analytic_length,
            radius,
            serialized_angle,
            sampled_length,
            pieces,
        )
        rounded_tag_pieces = math.ceil(serialized_analytic_length / 0.1)
        wrong_step = serialized_analytic_length / rounded_tag_pieces
        old_limit = (
            (radius * serialized_angle + wrong_step)
            * wrong_step
            * wrong_step
            / (24.0 * radius * radius)
            + 1.0e-6
        )
        self.assertEqual(rounded_tag_pieces, 66)
        self.assertGreater(deficit, old_limit)
        self.assertLessEqual(deficit, limit)

    def test_unitless_lanelet_speed_is_kilometres_per_hour(self):
        self.assertAlmostEqual(VALIDATOR._parse_speed_mps("0.9"), 0.25)
        self.assertAlmostEqual(VALIDATOR._parse_speed_mps("0.9 km/h"), 0.25)
        self.assertAlmostEqual(VALIDATOR._parse_speed_mps("0.9 kph"), 0.25)
        self.assertAlmostEqual(VALIDATOR._parse_speed_mps("0.25 m/s"), 0.25)

    def test_valid_connected_out_and_back_candidate_and_cli_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "candidate"
            write_candidate(output)
            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertEqual(
                result["acceptance_scope"], "static_format_geometry_coverage_only"
            )
            self.assertFalse(result["accepted_for_autoware_motion_test"])
            self.assertFalse(result["production_ready"])
            self.assertFalse(result["deployment_ready"])
            self.assertEqual(
                result["input"]["lanelet2_map_sha256"],
                hashlib.sha256(
                    (output / "lanelet2_map_closed_course_experimental.osm").read_bytes()
                ).hexdigest(),
            )
            self.assertEqual(result["counts"]["lanelets"], 2)
            self.assertEqual(result["metrics"]["centerline_endpoint_components"], 1)
            self.assertAlmostEqual(result["metrics"]["trajectory_centerline_coverage"], 1.0)
            self.assertAlmostEqual(result["metrics"]["minimum_lanelet_width_m"], 2.0)

            report_path = Path(temporary) / "reports" / "acceptance.json"
            completed = subprocess.run(
                [sys.executable, str(SCRIPT_PATH), str(output), "--report", str(report_path)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            summary = json.loads(completed.stdout)
            self.assertTrue(summary["accepted"])
            self.assertFalse(summary["accepted_for_autoware_motion_test"])
            self.assertEqual(Path(summary["report"]), report_path)
            self.assertTrue(json.loads(report_path.read_text(encoding="utf-8"))["accepted"])

    def test_replay_lineage_edge_ids_do_not_imply_named_route_promotion(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "candidate"
            write_candidate(output, osm=candidate_osm(route_edge_tags=True))
            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertEqual(result["coverage_reference"], "full_processed_trajectory")
            self.assertNotIn("named_route", result)

    def test_estimated_boundary_metadata_hard_gates_full_swept_footprint(self):
        cases = (
            ("contained", estimated_boundary_candidate_osm(), True, None),
            (
                "outside",
                estimated_boundary_candidate_osm(width="2.2"),
                False,
                "configured_swept_footprint_not_contained",
            ),
            (
                "missing_guard",
                estimated_boundary_candidate_osm(include_guard=False),
                False,
                "missing_estimated_boundary_metadata",
            ),
            (
                "wrong_longitudinal_guard",
                estimated_boundary_candidate_osm().replace(
                    'estimated_longitudinal_endpoint_guard_m" v="0.05"',
                    'estimated_longitudinal_endpoint_guard_m" v="0.04"',
                ),
                False,
                "invalid_estimated_boundary_metadata",
            ),
            (
                "endpoint_narrowed",
                estimated_boundary_candidate_osm().replace(
                    'local_x" v="-1.05"', 'local_x" v="-0.5"'
                ),
                False,
                "configured_swept_footprint_not_contained",
            ),
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, osm, accepted, expected_code in cases:
                with self.subTest(name=name):
                    output = root / name
                    write_candidate(output, osm=osm)
                    result = VALIDATOR.validate_candidate(output)
                    self.assertEqual(result["accepted"], accepted, result["errors"])
                    if expected_code is not None:
                        self.assertIn(
                            expected_code,
                            {item["code"] for item in result["errors"]},
                        )
            passed = VALIDATOR.validate_candidate(root / "contained")
            self.assertEqual(passed["counts"]["swept_footprint_pose_samples"], 101)
            self.assertEqual(
                passed["counts"]["swept_footprint_passed_pose_samples"], 101
            )
            self.assertEqual(
                passed["counts"]["swept_footprint_failed_pose_samples"], 0
            )
            self.assertTrue(
                passed["metrics"][
                    "configured_swept_footprint_inside_lanelet_union"
                ]
            )
            self.assertEqual(
                passed["metrics"]["estimated_longitudinal_endpoint_guard_m"],
                0.05,
            )

    def test_estimated_boundary_sweep_fails_closed_for_branch_transitions(self):
        tags = {
            "boundary_model": VALIDATOR.ESTIMATED_BOUNDARY_MODEL,
            "estimated_vehicle_width_m": "1.5",
            "estimated_front_extent_m": "2.0",
            "estimated_rear_extent_m": "1.0",
            "vehicle_minimum_turning_radius_m": "4.8",
            "estimated_lateral_margin_m": "0.2",
            "estimated_boundary_interpolation_guard_m": "0.05",
            "estimated_effective_lateral_margin_m": "0.25",
            "estimated_longitudinal_endpoint_guard_m": "0.05",
            "estimated_boundary_algorithm": VALIDATOR.ESTIMATED_BOUNDARY_ALGORITHM,
            "vehicle_profile": "car",
            "vehicle_base_reference": "rear_axle_ground_projection",
            "vehicle_dimensions_evidence_source": "catalog_estimated",
            "vehicle_dimensions_evidence_confidence": "medium",
            "vehicle_dimensions_verified": "no",
        }

        def lanelet(label, start_ref, end_ref, start, end):
            return {
                "label": label,
                "tags": dict(tags),
                "center_refs": (start_ref, end_ref),
                "center": [start, end],
                "left": [
                    (start[0], start[1] + 1.0, 0.0),
                    (end[0], end[1] + 1.0, 0.0),
                ],
                "right": [
                    (start[0], start[1] - 1.0, 0.0),
                    (end[0], end[1] - 1.0, 0.0),
                ],
            }

        details = {
            "lanelets": [
                lanelet("100", 1, 2, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0)),
                lanelet("200", 2, 3, (5.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
                lanelet("300", 2, 4, (5.0, 0.0, 0.0), (5.0, 5.0, 0.0)),
            ]
        }
        report = {"counts": {}, "metrics": {}, "errors": []}
        VALIDATOR._validate_configured_swept_footprint(details, report)
        self.assertEqual(report["counts"]["swept_footprint_branch_nodes"], 1)
        self.assertIn(
            "branched_swept_footprint_contract_unsupported",
            {item["code"] for item in report["errors"]},
        )

    def test_unauthed_replay_still_hard_gates_full_map_identity_and_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "candidate"
            write_candidate(output, osm=estimated_boundary_candidate_osm())

            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertNotIn("named_route", result)
            self.assertEqual(result["counts"]["lossless_replay_edges"], 2)
            self.assertEqual(result["counts"]["autoware_replay_edges"], 2)
            self.assertEqual(result["counts"]["full_map_source_edges"], 2)
            self.assertEqual(result["counts"]["full_map_lanelet_edges"], 2)
            self.assertEqual(result["counts"]["full_map_lanelet_segments"], 2)
            self.assertTrue(result["metrics"]["lossless_replay_exact_match"])
            self.assertEqual(
                result["metrics"]["full_map_maximum_3d_arc_length_delta_m"],
                0.0,
            )
            self.assertEqual(
                result["metrics"]["full_map_source_centerline_coverage"], 1.0
            )
            self.assertEqual(
                result["metrics"][
                    "full_map_lanelet_centerline_source_coverage"
                ],
                1.0,
            )

            autoware_path = (
                output / "route_graph_autoware_replay_candidate.geojson"
            )
            autoware = json.loads(autoware_path.read_text(encoding="utf-8"))
            autoware["features"][0]["geometry"]["coordinates"][0][1][0] = 4.75
            autoware_path.write_text(json.dumps(autoware), encoding="utf-8")
            tampered = VALIDATOR.validate_candidate(output)
            self.assertFalse(tampered["accepted"])
            self.assertFalse(
                tampered["metrics"]["lossless_replay_exact_match"]
            )
            self.assertIn(
                "autoware_replay_geometry_changed",
                {item["code"] for item in tampered["errors"]},
            )

    def test_partial_speed_span_is_a_lossless_one_to_many_lanelet_export(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "candidate"
            write_semantic_speed_candidate(output)
            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertEqual(result["counts"]["full_map_source_edges"], 2)
            self.assertEqual(result["counts"]["full_map_lanelet_edges"], 2)
            self.assertEqual(result["counts"]["full_map_lanelet_segments"], 3)
            self.assertEqual(
                result["metrics"]["full_map_maximum_centerline_length_delta_m"],
                0.0,
            )
            self.assertEqual(
                result["metrics"]["full_map_maximum_3d_arc_length_delta_m"],
                0.0,
            )
            report = {"counts": {}, "metrics": {}, "errors": []}
            _, _, details = VALIDATOR._read_lanelet_map(
                output / "lanelet2_map_closed_course_experimental.osm", report
            )
            speeds = [
                float(lanelet["tags"]["generator_speed_limit_mps"])
                for lanelet in details["lanelets"]
            ]
            self.assertEqual(speeds, [0.5, 0.2, 0.5])
            self.assertEqual(
                [
                    int(lanelet["tags"]["source_route_edge_id"])
                    for lanelet in details["lanelets"]
                ],
                [10, 10, 11],
            )

    def test_semantic_lanelet_gap_overlap_reorder_and_length_tampering_fails(self):
        cases = {
            "gap": "semantic_source_interval_gap_or_overlap",
            "overlap": "semantic_source_interval_gap_or_overlap",
            "reorder": "semantic_source_edge_order_mismatch",
            "source_length": "semantic_source_length_mismatch",
            "3d_length": "full_map_edge_3d_arc_length_changed",
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for tamper, expected_code in cases.items():
                with self.subTest(tamper=tamper):
                    output = root / tamper
                    write_semantic_speed_candidate(output, tamper)
                    result = VALIDATOR.validate_candidate(output)
                    self.assertFalse(result["accepted"])
                    self.assertIn(
                        expected_code,
                        {item["code"] for item in result["errors"]},
                        result["errors"],
                    )

    def test_partial_named_route_metadata_still_fails_without_fake_zero_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "candidate"
            partial_named_route = candidate_osm(route_edge_tags=True).replace(
                '    <tag k="route_edge_id" v="10"/>',
                '    <tag k="named_route_id" v="42"/>\n'
                '    <tag k="route_edge_id" v="10"/>',
                1,
            )
            write_candidate(output, osm=partial_named_route)
            result = VALIDATOR.validate_candidate(output)
            codes = {error["code"] for error in result["errors"]}
            self.assertFalse(result["accepted"])
            self.assertIn("incomplete_named_route_metadata", codes)

            completed = subprocess.run(
                [sys.executable, str(SCRIPT_PATH), str(output)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 1, completed.stderr)
            summary = json.loads(completed.stdout)
            self.assertIsNone(summary["coverage"])
            self.assertEqual(summary["full_trajectory_coverage_diagnostic"], 1.0)

    def test_each_required_rejection(self):
        cases = {
            "missing_member": (
                candidate_osm(omit_role="centerline"),
                trajectory(),
                "missing_lanelet_member",
            ),
            "invalid_ref": (
                candidate_osm(invalid_center_ref=True),
                trajectory(),
                "invalid_node_ref",
            ),
            "nonfinite_geometry": (
                candidate_osm(nonfinite_node=True),
                trajectory(),
                "nonfinite_local_geometry",
            ),
            "self_intersection": (
                candidate_osm(crossing_boundaries=True),
                trajectory(),
                "self_intersecting_boundary_polygon",
            ),
            "nonpositive_width": (
                candidate_osm(zero_width=True),
                trajectory(),
                "nonpositive_lanelet_width",
            ),
            "nonpositive_speed": (
                candidate_osm(first_speed="0 m/s"),
                trajectory(),
                "nonpositive_speed_limit",
            ),
            "disconnected": (
                candidate_osm(disconnected=True),
                trajectory(),
                "disconnected_centerline_graph",
            ),
            "insufficient_coverage": (
                candidate_osm(),
                trajectory(y=5.0),
                "insufficient_trajectory_coverage",
            ),
            "forward_replay_cusp": (
                candidate_osm(cusp=True),
                "0.0 0 0 0 0 0 0 1\n1.0 5 0 0 0 0 0 1\n"
                "2.0 0 0 0 0 0 0 1\n",
                "forward_replay_cusp",
            ),
            "excessive_connection_heading_jump": (
                candidate_osm(second_heading_deg=111.7),
                "0.0 0 0 0 0 0 0 1\n1.0 5 0 0 0 0 0 1\n"
                "2.0 3.150263 4.645665 0 0 0 0 1\n",
                "excessive_lanelet_connection_heading_jump",
            ),
            "subminimum_lanelet": (
                candidate_osm(second_length=0.097733674),
                "0.0 0 0 0 0 0 0 1\n1.0 5 0 0 0 0 0 1\n"
                "2.0 5.097733674 0 0 0 0 0 1\n",
                "subminimum_lanelet_centerline_length",
            ),
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, (osm, tum, expected_code) in cases.items():
                with self.subTest(name=name):
                    output = root / name
                    write_candidate(output, osm=osm, tum=tum)
                    result = VALIDATOR.validate_candidate(output)
                    codes = {error["code"] for error in result["errors"]}
                    self.assertFalse(result["accepted"])
                    self.assertIn(expected_code, codes, result["errors"])

    def test_default_coverage_is_99_percent_and_configurable(self):
        self.assertEqual(VALIDATOR.DEFAULT_MINIMUM_COVERAGE, 0.99)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            partial = (
                "0.0 0 0 0 0 0 0 1\n"
                "1.0 5 0 0 0 0 0 1\n"
                "2.0 10 0 0 0 0 0 1\n"
                "3.0 12 0 0 0 0 0 1\n"
            )
            write_candidate(output, tum=partial)
            strict = VALIDATOR.validate_candidate(output)
            relaxed = VALIDATOR.validate_candidate(output, minimum_coverage=0.75)
            self.assertFalse(strict["accepted"])
            self.assertTrue(relaxed["accepted"], relaxed["errors"])

    def test_named_route_is_an_independent_subset_of_a_full_coverage_map(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_named_candidate(
                output,
                osm=named_candidate_with_non_route_lanelet(),
                full_source_graph=source_graph_with_non_route_edge(),
                tum=full_map_trajectory(),
            )
            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertEqual(result["coverage_reference"], "named_route_source_graph")
            self.assertEqual(result["counts"]["lanelets"], 3)
            self.assertEqual(result["counts"]["named_route_lanelets"], 2)
            self.assertEqual(result["counts"]["non_named_route_lanelets"], 1)
            self.assertEqual(result["counts"]["named_route_source_edges"], 2)
            self.assertEqual(result["counts"]["full_map_source_edges"], 3)
            self.assertEqual(result["counts"]["full_map_lanelet_edges"], 3)
            self.assertEqual(
                Path(result["input"]["named_route_source_graph"]).name,
                "route_graph_autoware_selected_mission.geojson",
            )
            self.assertEqual(result["metrics"]["trajectory_centerline_coverage"], 1.0)
            self.assertEqual(
                result["metrics"]["full_map_source_centerline_coverage"], 1.0
            )
            self.assertEqual(
                result["metrics"]["full_map_lanelet_centerline_source_coverage"],
                1.0,
            )
            self.assertRegex(
                result["input"]["full_map_source_graph_sha256"], r"^[0-9a-f]{64}$"
            )
            self.assertRegex(
                result["input"]["named_route_source_graph_sha256"],
                r"^[0-9a-f]{64}$",
            )
            self.assertEqual(result["metrics"]["acceptance_centerline_coverage"], 1.0)
            self.assertEqual(result["named_route"]["ordered_edge_ids"], [10, 11])
            self.assertEqual(result["counts"]["authored_stop_lines"], 1)
            self.assertEqual(result["counts"]["authored_stop_regulatory_elements"], 1)

    def test_named_route_cannot_hide_full_map_source_omission_or_geometry_loss(self):
        cases = {
            "edge_omission": (
                named_source_graph(),
                "full_map_source_edge_set_mismatch",
            ),
            "geometry_loss": (
                source_graph_with_non_route_edge(),
                "insufficient_full_map_source_coverage",
            ),
        }
        # Move only the full-map source continuation away from the exported
        # third Lanelet. The selected two-Edge Mission remains unchanged and
        # must not be able to make this map pass.
        cases["geometry_loss"][0]["features"][-1]["geometry"]["coordinates"] = [
            [[10.0, 3.0], [15.0, 3.0]]
        ]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, (full_source, expected_code) in cases.items():
                with self.subTest(name=name):
                    output = root / name
                    write_named_candidate(
                        output,
                        osm=named_candidate_with_non_route_lanelet(),
                        full_source_graph=full_source,
                        tum=full_map_trajectory(),
                    )
                    result = VALIDATOR.validate_candidate(output)
                    self.assertFalse(result["accepted"])
                    self.assertIn(
                        expected_code,
                        {error["code"] for error in result["errors"]},
                    )

    def test_named_route_only_map_fails_full_processed_trajectory_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_named_candidate(output, tum=full_map_trajectory())
            result = VALIDATOR.validate_candidate(output)
            codes = {error["code"] for error in result["errors"]}
            self.assertFalse(result["accepted"])
            self.assertIn("insufficient_trajectory_coverage", codes)
            self.assertEqual(result["metrics"]["acceptance_centerline_coverage"], 1.0)
            self.assertLess(result["metrics"]["trajectory_centerline_coverage"], 0.99)
            self.assertEqual(result["named_route"]["ordered_edge_ids"], [10, 11])

    def test_named_route_authoring_edge_set_and_order_are_independent_gates(self):
        cases = {
            "set": ([10, 12], "named_route_authoring_edge_set_mismatch"),
            "order": ([11, 10], "named_route_authoring_edge_order_mismatch"),
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, (ordered_ids, expected_code) in cases.items():
                with self.subTest(name=name):
                    output = root / name
                    write_named_candidate(output)
                    authoring_path = (
                        output / "navigation_authoring_autoware_replay.json"
                    )
                    authoring = json.loads(authoring_path.read_text(encoding="utf-8"))
                    authoring["routes"][0]["ordered_edge_ids"] = ordered_ids
                    authoring_path.write_text(json.dumps(authoring), encoding="utf-8")
                    result = VALIDATOR.validate_candidate(output)
                    self.assertFalse(result["accepted"])
                    self.assertIn(
                        expected_code,
                        {error["code"] for error in result["errors"]},
                    )

    def test_exact_full_open_named_route_accepts_without_synthetic_support(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_named_candidate(output)
            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertEqual(result["planning_support"], {"present": False})
            self.assertEqual(result["counts"]["synthetic_planning_support_lanelets"], 0)

    def test_named_route_accepts_audited_measured_terminal_support_without_synthetic_support(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            write_named_candidate(
                output,
                osm=named_candidate_with_terminal_support(),
                topology_graph=terminal_support_topology_graph(),
            )
            result = VALIDATOR.validate_candidate(output)
            self.assertTrue(result["accepted"], result["errors"])
            self.assertEqual(result["planning_support"], {"present": False})
            self.assertTrue(result["terminal_support"]["present"])
            self.assertEqual(result["terminal_support"]["final_named_edge_id"], 11)
            self.assertEqual(result["terminal_support"]["support_edge_ids"], [12])
            self.assertAlmostEqual(result["terminal_support"]["support_length_m"], 5.0)
            self.assertFalse(
                result["terminal_support"]["support_is_part_of_named_route"]
            )
            self.assertEqual(result["metrics"]["acceptance_centerline_coverage"], 1.0)

    def test_synthetic_open_route_support_is_always_rejected_and_tamper_evident(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "rejected"
            write_named_candidate(
                output, osm=named_candidate_with_synthetic_planning_support()
            )
            result = VALIDATOR.validate_candidate(output)
            self.assertFalse(result["accepted"])
            self.assertIn(
                "unobserved_synthetic_open_route_planning_support",
                {error["code"] for error in result["errors"]},
            )
            self.assertTrue(result["planning_support"]["present"])
            self.assertEqual(
                result["planning_support"]["relation_ids"],
                {"head": 400, "tail": 500},
            )
            self.assertEqual(result["counts"]["full_map_source_edges"], 2)
            self.assertEqual(result["counts"]["full_map_lanelet_segments"], 2)
            self.assertEqual(result["counts"]["lanelets"], 4)
            self.assertFalse(
                result["planning_support"]["support_is_part_of_raw_counts"]
            )
            self.assertRegex(
                result["planning_support"]["support_geometry_tags_sha256"],
                r"^[0-9a-f]{64}$",
            )
            for tamper, expected in {
                "missing_head": "invalid_synthetic_planning_support_cardinality",
                "head_length": "synthetic_planning_support_length_or_arc_mismatch",
                "head_geometry": "synthetic_planning_support_endpoint_tag_mismatch",
                "head_boundary": "synthetic_planning_support_endpoint_footprint_not_contained",
                "head_isolation": "synthetic_planning_support_outer_pose_insufficiently_isolated",
                "duplicate_role": "invalid_synthetic_planning_support_roles",
            }.items():
                with self.subTest(tamper=tamper):
                    candidate = Path(temporary) / tamper
                    write_named_candidate(
                        candidate,
                        osm=named_candidate_with_synthetic_planning_support(
                            tamper=tamper
                        ),
                    )
                    failed = VALIDATOR.validate_candidate(candidate)
                    self.assertFalse(failed["accepted"])
                    self.assertIn(
                        "unobserved_synthetic_open_route_planning_support",
                        {item["code"] for item in failed["errors"]},
                    )
                    self.assertIn(
                        expected, {item["code"] for item in failed["errors"]}
                    )

    def test_initial_unselected_open_replay_rejects_synthetic_endpoint_support(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            osm = named_candidate_with_synthetic_planning_support()
            osm = re.sub(
                r'^\s*<tag k="named_route_(?:id|name|target|order)"[^>]*/>\n?',
                "",
                osm,
                flags=re.MULTILINE,
            )
            write_candidate(output, osm=osm)
            result = VALIDATOR.validate_candidate(output)
            self.assertFalse(result["accepted"])
            self.assertIn(
                "unobserved_synthetic_open_route_planning_support",
                {error["code"] for error in result["errors"]},
            )
            self.assertNotIn("named_route", result)
            self.assertTrue(result["planning_support"]["present"])

    def test_terminal_support_tampering_fails_closed(self):
        cases = {
            "not_passable": (
                named_candidate_with_terminal_support(),
                terminal_support_topology_graph(passable=False),
                "terminal_support_edge_not_passable",
            ),
            "geometry": (
                named_candidate_with_terminal_support(),
                terminal_support_topology_graph(support_y=2.0),
                "terminal_support_geometry_gap",
            ),
            "wrong_id": (
                named_candidate_with_terminal_support().replace(
                    'terminal_support_edge_ids" v="12"',
                    'terminal_support_edge_ids" v="13"',
                ),
                terminal_support_topology_graph(),
                "terminal_support_edge_missing",
            ),
            "heading": (
                named_candidate_with_terminal_support(),
                terminal_support_topology_graph(support_end=[10.0, 5.0]),
                "terminal_support_heading_discontinuity",
            ),
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, (osm, topology, expected) in cases.items():
                with self.subTest(name=name):
                    output = root / name
                    write_named_candidate(
                        output, osm=osm, topology_graph=topology
                    )
                    result = VALIDATOR.validate_candidate(output)
                    self.assertFalse(result["accepted"])
                    self.assertIn(
                        expected, {item["code"] for item in result["errors"]}
                    )

    def test_named_route_rejects_source_order_identity_and_stop_corruption(self):
        cases = {
            "source_set": (
                named_candidate_osm(),
                source_graph_with_non_route_edge(),
                "named_route_source_edge_set_mismatch",
            ),
            "source_geometry": (
                named_candidate_osm(),
                named_source_graph(second_y=3.0),
                "insufficient_named_route_source_coverage",
            ),
            "source_direction": (
                named_candidate_osm(),
                named_source_graph_with_reversed_second_edge(),
                "named_route_edge_orientation_mismatch",
            ),
            "duplicate_order": (
                named_candidate_osm().replace(
                    '<tag k="named_route_order" v="1"/>',
                    '<tag k="named_route_order" v="0"/>',
                    1,
                ),
                named_source_graph(),
                "invalid_named_route_order",
            ),
            "route_name": (
                named_candidate_osm().replace(
                    '<tag k="named_route_name" v="short course"/>',
                    '<tag k="named_route_name" v="wrong course"/>',
                    1,
                ),
                named_source_graph(),
                "inconsistent_named_route_identity",
            ),
            "stop_sign": (
                named_candidate_osm().replace(
                    '<tag k="sign_type" v="stop_sign"/>',
                    '<tag k="sign_type" v="yield"/>',
                    1,
                ),
                named_source_graph(),
                "invalid_authored_stop_regulatory_element",
            ),
            "stop_ref_line": (
                named_candidate_osm().replace('role="ref_line"', 'role="refers"', 1),
                named_source_graph(),
                "invalid_authored_stop_ref_line",
            ),
            "stop_lanelet_attachment": (
                named_candidate_osm().replace(
                    '\n    <member type="relation" ref="300" role="regulatory_element"/>',
                    "",
                    1,
                ),
                named_source_graph(),
                "invalid_authored_stop_lanelet_reference",
            ),
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, (osm, source_graph, expected_code) in cases.items():
                with self.subTest(name=name):
                    output = root / name
                    write_named_candidate(output, osm=osm, source_graph=source_graph)
                    result = VALIDATOR.validate_candidate(output)
                    codes = {error["code"] for error in result["errors"]}
                    self.assertFalse(result["accepted"])
                    self.assertIn(expected_code, codes, result["errors"])

    def test_named_route_requires_independent_source_and_selected_status(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            missing_source = root / "missing_source"
            write_named_candidate(missing_source)
            (missing_source / "route_graph_autoware_selected_mission.geojson").unlink()
            result = VALIDATOR.validate_candidate(missing_source)
            self.assertIn(
                "missing_named_route_source_graph",
                {error["code"] for error in result["errors"]},
            )

            stale_status = root / "stale_status"
            write_named_candidate(stale_status)
            status_path = stale_status / "navigation_authoring_closed_course_status.json"
            status = json.loads(status_path.read_text(encoding="utf-8"))
            status["autoware"]["promoted"] = False
            status_path.write_text(json.dumps(status), encoding="utf-8")
            result = VALIDATOR.validate_candidate(stale_status)
            self.assertIn(
                "named_route_closed_course_status_mismatch",
                {error["code"] for error in result["errors"]},
            )


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
