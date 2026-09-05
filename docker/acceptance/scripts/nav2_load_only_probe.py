#!/usr/bin/env python3
"""Load-only acceptance probe for generated Nav2 artifacts.

This probe configures and activates Map Server and Route Server, observes the
published occupancy grid, and records the separate waypoint-file dry run.  It
does not request a route, send an action goal, start a planner/controller, or
move a robot.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import sys
import threading
import time
from typing import Any, Callable

import yaml


class LoadOnlyError(RuntimeError):
    """Raised when an artifact cannot be loaded within the alpha scope."""


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: pathlib.Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise LoadOnlyError(f"failed to read {label}: {error}") from error
    if not isinstance(value, dict):
        raise LoadOnlyError(f"{label} root must be an object")
    return value


def load_yaml(path: pathlib.Path, label: str) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise LoadOnlyError(f"failed to read {label}: {error}") from error
    if not isinstance(value, dict):
        raise LoadOnlyError(f"{label} root must be a mapping")
    return value


def positive_finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise LoadOnlyError(f"{label} must be a positive finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise LoadOnlyError(f"{label} must be a positive finite number") from error
    if not math.isfinite(result) or result <= 0.0:
        raise LoadOnlyError(f"{label} must be a positive finite number")
    return result


def inspect_artifacts(root: pathlib.Path, waypoint_dry_run: pathlib.Path) -> dict[str, Any]:
    readiness = load_yaml(
        root / "nav2_closed_course_experimental_readiness.yaml", "readiness"
    )
    artifact = readiness.get("artifact")
    compatibility = readiness.get("compatibility")
    if not isinstance(artifact, dict) or not isinstance(compatibility, dict):
        raise LoadOnlyError("readiness artifact/compatibility sections are missing")
    for key in ("ready", "static_map_ready", "follow_waypoints_ready", "route_server_ready"):
        if artifact.get(key) is not True:
            raise LoadOnlyError(f"readiness.artifact.{key} is not true")
    for key in ("map_server", "follow_waypoints_action_goal", "route_server_geojson"):
        if compatibility.get(key) is not True:
            raise LoadOnlyError(f"readiness.compatibility.{key} is not true")

    map_yaml = load_yaml(
        root / "nav2_map_closed_course_experimental.yaml", "Map Server YAML"
    )
    image_name = map_yaml.get("image")
    if not isinstance(image_name, str) or pathlib.Path(image_name).name != image_name:
        raise LoadOnlyError("Map Server YAML image must be a bundle-local file name")
    map_image = (root / image_name).resolve(strict=True)
    try:
        map_image.relative_to(root)
    except ValueError as error:
        raise LoadOnlyError("Map Server image escapes the artifact directory") from error
    positive_finite(map_yaml.get("resolution"), "map resolution")

    graph = load_json(
        root / "nav2_route_graph_closed_course_experimental.geojson",
        "Route Server GeoJSON",
    )
    features = graph.get("features")
    if graph.get("type") != "FeatureCollection" or not isinstance(features, list):
        raise LoadOnlyError("Route Server GeoJSON is not a FeatureCollection")
    point_count = 0
    edge_count = 0
    for feature in features:
        if not isinstance(feature, dict) or not isinstance(feature.get("geometry"), dict):
            continue
        geometry_type = feature["geometry"].get("type")
        if geometry_type == "Point":
            point_count += 1
        elif geometry_type in ("LineString", "MultiLineString"):
            edge_count += 1
    if point_count < 2 or edge_count < 1:
        raise LoadOnlyError("Route Server GeoJSON contains no loadable route graph")

    waypoints = load_yaml(
        root / "nav2_waypoints_closed_course_experimental.yaml", "waypoint YAML"
    )
    routes = waypoints.get("routes")
    if (
        waypoints.get("schema_version") != 2
        or waypoints.get("format") != "lmmg_nav2_follow_waypoints_routes"
        or waypoints.get("frame_id") != "map"
        or waypoints.get("artifact_ready") is not True
        or not isinstance(routes, list)
        or not routes
    ):
        raise LoadOnlyError("waypoint YAML schema/frame/readiness is invalid")
    dry_run = load_json(waypoint_dry_run, "waypoint dry-run result")
    if (
        dry_run.get("action_type") != "nav2_msgs/action/FollowWaypoints"
        or dry_run.get("frame_id") != "map"
        or dry_run.get("goal_z_is_planar") is not True
        or not isinstance(dry_run.get("waypoints"), int)
        or dry_run["waypoints"] < 2
    ):
        raise LoadOnlyError("waypoint dry-run did not produce a valid planar summary")

    artifact_paths = {
        "nav2_map_closed_course_experimental.yaml": (
            root / "nav2_map_closed_course_experimental.yaml"
        ),
        image_name: map_image,
        "nav2_route_graph_closed_course_experimental.geojson": (
            root / "nav2_route_graph_closed_course_experimental.geojson"
        ),
        "nav2_waypoints_closed_course_experimental.yaml": (
            root / "nav2_waypoints_closed_course_experimental.yaml"
        ),
        "nav2_closed_course_experimental_params.yaml": (
            root / "nav2_closed_course_experimental_params.yaml"
        ),
        "nav2_closed_course_experimental_readiness.yaml": (
            root / "nav2_closed_course_experimental_readiness.yaml"
        ),
    }
    return {
        "sha256": {
            name: sha256_file(path) for name, path in sorted(artifact_paths.items())
        },
        "map_yaml": str(artifact_paths["nav2_map_closed_course_experimental.yaml"]),
        "map_image": str(map_image),
        "map_resolution": float(map_yaml["resolution"]),
        "route_graph": str(root / "nav2_route_graph_closed_course_experimental.geojson"),
        "route_nodes_declared": point_count,
        "route_edges_declared": edge_count,
        "waypoint_yaml": str(root / "nav2_waypoints_closed_course_experimental.yaml"),
        "waypoint_dry_run": dry_run,
    }


def wait_until(predicate: Callable[[], bool], deadline: float, label: str) -> None:
    while not predicate():
        if time.monotonic() >= deadline:
            raise LoadOnlyError(f"timed out waiting for {label}")
        time.sleep(0.05)


def run_probe(timeout: float) -> dict[str, Any]:
    try:
        import rclpy
        from lifecycle_msgs.msg import Transition
        from lifecycle_msgs.srv import ChangeState
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as error:
        raise LoadOnlyError(f"required ROS 2/Nav2 Python modules are unavailable: {error}") from error

    class ProbeNode(Node):
        def __init__(self) -> None:
            super().__init__("lmmg_nav2_load_only_probe")
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.map_message: Any = None
            self.map_event = threading.Event()
            self.subscription = self.create_subscription(
                OccupancyGrid, "/map", self.on_map, qos
            )

        def on_map(self, message: Any) -> None:
            self.map_message = message
            self.map_event.set()

    def transition(node: Any, name: str, transition_id: int, deadline: float) -> None:
        client = node.create_client(ChangeState, f"/{name}/change_state")
        wait_until(
            lambda: client.wait_for_service(timeout_sec=0.1),
            deadline,
            f"{name} lifecycle service",
        )
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        wait_until(future.done, deadline, f"{name} lifecycle transition {transition_id}")
        response = future.result()
        if response is None or response.success is not True:
            raise LoadOnlyError(f"{name} rejected lifecycle transition {transition_id}")
        node.destroy_client(client)

    rclpy.init(args=None)
    node = ProbeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    deadline = time.monotonic() + timeout
    try:
        transition(node, "map_server", Transition.TRANSITION_CONFIGURE, deadline)
        transition(node, "map_server", Transition.TRANSITION_ACTIVATE, deadline)
        wait_until(node.map_event.is_set, deadline, "Map Server occupancy grid")
        transition(node, "route_server", Transition.TRANSITION_CONFIGURE, deadline)
        transition(node, "route_server", Transition.TRANSITION_ACTIVATE, deadline)

        message = node.map_message
        if message is None:
            raise LoadOnlyError("Map Server published no occupancy grid")
        width = int(message.info.width)
        height = int(message.info.height)
        values = [int(value) for value in message.data]
        if (
            message.header.frame_id != "map"
            or width <= 0
            or height <= 0
            or len(values) != width * height
            or not set(values).issubset({-1, 0, 100})
        ):
            raise LoadOnlyError("Map Server published an invalid trinary map")
        return {
            "map_server": {
                "configured": True,
                "activated": True,
                "frame_id": message.header.frame_id,
                "width": width,
                "height": height,
                "resolution": float(message.info.resolution),
                "occupied_cells": values.count(100),
                "free_cells": values.count(0),
                "unknown_cells": values.count(-1),
            },
            "route_server": {"configured": True, "activated": True},
        }
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-dir", type=pathlib.Path, required=True)
    parser.add_argument("--waypoint-dry-run", type=pathlib.Path, required=True)
    parser.add_argument("--report", type=pathlib.Path, required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    report: dict[str, Any] = {
        "schema_version": 1,
        "kind": "lmmg_nav2_alpha_load_only_acceptance",
        "accepted": False,
        "scope": {
            "map_server_load": True,
            "route_server_load": True,
            "waypoint_yaml_dry_run": True,
            "planning": False,
            "action_execution": False,
            "robot_motion": False,
        },
        "errors": [],
    }
    try:
        if not math.isfinite(args.timeout) or args.timeout <= 0.0:
            raise LoadOnlyError("timeout must be a positive finite number")
        root = args.artifact_dir.resolve(strict=True)
        report["artifacts"] = inspect_artifacts(root, args.waypoint_dry_run)
        report["runtime"] = run_probe(args.timeout)
        report["accepted"] = True
    except (LoadOnlyError, FileNotFoundError, OSError) as error:
        report["errors"].append(str(error))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if not report["accepted"]:
        print(f"error: {report['errors'][0]}", file=sys.stderr)
        return 1
    print(f"Nav2 alpha load-only acceptance passed: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
