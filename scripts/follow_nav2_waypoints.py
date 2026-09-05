#!/usr/bin/env python3
"""Send a generated route as a real Nav2 FollowWaypoints action goal."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time
from typing import Any

import yaml


SUPPORTED_FORMATS = {
    "lmmg_nav2_follow_waypoints_routes",
    # Read v0.7 files for migration, but new output never writes this label.
    "lmmg_nav2_jazzy_waypoint_routes",
}


class WaypointFileError(ValueError):
    """Raised when an artifact cannot safely become an action goal."""


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise WaypointFileError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise WaypointFileError(f"{name} must be a finite number") from error
    if not math.isfinite(result):
        raise WaypointFileError(f"{name} must be a finite number")
    return result


def load_route(path: pathlib.Path, route_id: int | None) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise WaypointFileError(f"failed to read waypoint artifact: {error}") from error
    if not isinstance(document, dict):
        raise WaypointFileError("waypoint artifact root must be a mapping")
    if document.get("format") not in SUPPORTED_FORMATS:
        raise WaypointFileError(f"unsupported waypoint format: {document.get('format')!r}")
    if document.get("artifact_ready") is not True:
        raise WaypointFileError("artifact_ready is not true")
    frame_id = document.get("frame_id")
    if frame_id != "map":
        raise WaypointFileError("frame_id must be 'map' for Nav2")
    routes = document.get("routes")
    if not isinstance(routes, list) or not routes:
        raise WaypointFileError("waypoint artifact contains no routes")
    if route_id is None:
        if len(routes) != 1:
            identifiers = [route.get("route_id") for route in routes if isinstance(route, dict)]
            raise WaypointFileError(
                f"artifact has {len(routes)} routes; select one with --route-id from {identifiers}"
            )
        selected = routes[0]
    else:
        selected = next(
            (route for route in routes if isinstance(route, dict) and route.get("route_id") == route_id),
            None,
        )
        if selected is None:
            raise WaypointFileError(f"route_id {route_id} does not exist")
    if not isinstance(selected, dict):
        raise WaypointFileError("selected route must be a mapping")
    waypoints = selected.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise WaypointFileError("selected route must contain at least two waypoints")
    normalized: list[dict[str, float]] = []
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise WaypointFileError(f"waypoint {index} must be a mapping")
        normalized.append(
            {
                "x": _finite_number(waypoint.get("x"), f"waypoint {index}.x"),
                "y": _finite_number(waypoint.get("y"), f"waypoint {index}.y"),
                # Nav2's occupancy-grid navigation is planar. Source map Z is
                # audit metadata only and must not become a goal height.
                "z": 0.0,
                "yaw": _finite_number(waypoint.get("yaw"), f"waypoint {index}.yaw"),
            }
        )
    return {
        "frame_id": frame_id,
        "experimental_only": document.get("experimental_only") is True,
        "route_id": selected.get("route_id"),
        "closed_loop": selected.get("closed_loop") is True,
        "source_edge_ids": selected.get("source_edge_ids", []),
        "waypoints": normalized,
    }


def send_goal(args: argparse.Namespace, route: dict[str, Any]) -> int:
    if route["experimental_only"] and not args.acknowledge_experimental:
        raise WaypointFileError(
            "sending this experimental route requires --acknowledge-experimental"
        )
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import FollowWaypoints
        from rclpy.action import ActionClient
        from rclpy.node import Node
    except ImportError as error:
        raise WaypointFileError(
            "Nav2 Python messages are unavailable; install/source nav2_msgs and rclpy"
        ) from error

    rclpy.init(args=None)
    node = Node("lmmg_follow_waypoints_client")
    client = ActionClient(node, FollowWaypoints, args.action_name)
    try:
        if not client.wait_for_server(timeout_sec=args.server_timeout):
            raise WaypointFileError(
                f"FollowWaypoints action server {args.action_name!r} was not available "
                f"within {args.server_timeout:.1f} s"
            )
        goal = FollowWaypoints.Goal()
        goal.number_of_loops = args.number_of_loops
        goal.goal_index = args.goal_index
        stamp = node.get_clock().now().to_msg()
        for waypoint in route["waypoints"]:
            pose = PoseStamped()
            pose.header.frame_id = route["frame_id"]
            pose.header.stamp = stamp
            pose.pose.position.x = waypoint["x"]
            pose.pose.position.y = waypoint["y"]
            pose.pose.position.z = 0.0
            half_yaw = 0.5 * waypoint["yaw"]
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            goal.poses.append(pose)

        def feedback(message: Any) -> None:
            print(f"current_waypoint={message.feedback.current_waypoint}", flush=True)

        future = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.server_timeout)
        if not future.done() or future.result() is None or not future.result().accepted:
            raise WaypointFileError("FollowWaypoints goal was rejected")
        result_future = future.result().get_result_async()
        deadline = None if args.result_timeout <= 0.0 else time.monotonic() + args.result_timeout
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.25)
            if deadline is not None and time.monotonic() >= deadline:
                future.result().cancel_goal_async()
                raise WaypointFileError("timed out waiting for FollowWaypoints result")
        if not result_future.done() or result_future.result() is None:
            raise WaypointFileError("FollowWaypoints ended without a result")
        wrapped = result_future.result()
        result = wrapped.result
        print(
            json.dumps(
                {
                    "status": int(wrapped.status),
                    "error_code": int(getattr(result, "error_code", 0)),
                    "error_msg": str(getattr(result, "error_msg", "")),
                    "missed_waypoints": len(getattr(result, "missed_waypoints", [])),
                },
                sort_keys=True,
            )
        )
        return 0 if wrapped.status == GoalStatus.STATUS_SUCCEEDED else 5
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send one generated LMMG route to Nav2's FollowWaypoints action"
    )
    parser.add_argument("waypoint_file", type=pathlib.Path)
    parser.add_argument("--route-id", type=int)
    parser.add_argument("--action-name", default="/follow_waypoints")
    parser.add_argument("--number-of-loops", type=int, default=0)
    parser.add_argument("--goal-index", type=int, default=0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=0.0)
    parser.add_argument("--acknowledge-experimental", action="store_true")
    parser.add_argument(
        "--dry-run", action="store_true", help="validate and summarize without importing ROS"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.number_of_loops < 0 or args.goal_index < 0:
        parser.error("--number-of-loops and --goal-index must be nonnegative")
    if args.server_timeout <= 0.0 or args.result_timeout < 0.0:
        parser.error("timeouts must be positive (result timeout may be zero for unlimited)")
    try:
        route = load_route(args.waypoint_file, args.route_id)
        if args.goal_index >= len(route["waypoints"]):
            raise WaypointFileError("--goal-index is outside the selected waypoint route")
        if args.dry_run:
            print(
                json.dumps(
                    {
                        "action_type": "nav2_msgs/action/FollowWaypoints",
                        "route_id": route["route_id"],
                        "frame_id": route["frame_id"],
                        "closed_loop": route["closed_loop"],
                        "waypoints": len(route["waypoints"]),
                        "goal_z_is_planar": all(wp["z"] == 0.0 for wp in route["waypoints"]),
                    },
                    sort_keys=True,
                )
            )
            return 0
        return send_goal(args, route)
    except WaypointFileError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
