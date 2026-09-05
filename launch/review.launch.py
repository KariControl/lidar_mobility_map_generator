import json
import math
import re
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _enabled(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _grid_bounds(output_directory):
    metadata = output_directory / "obstacles.yaml"
    if not metadata.is_file():
        return None
    values = {}
    for line in metadata.read_text(encoding="utf-8").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip().strip('"')
    try:
        resolution = float(values["resolution"])
        origin = [float(value) for value in values["origin"].strip("[]").split(",")]
        image = Path(values["image"])
        if not image.is_absolute():
            image = metadata.parent / image
        # Only the four ASCII header tokens are needed for both P2 and P5 PGM.
        header = image.read_bytes()[:4096].decode("latin-1", errors="ignore")
        tokens = []
        for line in header.splitlines():
            clean = line.split("#", 1)[0]
            tokens.extend(clean.split())
            if len(tokens) >= 4:
                break
        if tokens[0] not in ("P2", "P5"):
            return None
        width = int(tokens[1])
        height = int(tokens[2])
        return (
            origin[0],
            origin[0] + width * resolution,
            origin[1],
            origin[1] + height * resolution,
        )
    except (KeyError, OSError, ValueError, IndexError):
        return None


def _trajectory_bounds(output_directory):
    for name in ("trajectory_processed.tum", "trajectory_raw.tum"):
        path = output_directory / name
        if not path.is_file():
            continue
        xs = []
        ys = []
        try:
            for line in path.read_text(encoding="utf-8").splitlines():
                fields = line.split()
                if len(fields) >= 3:
                    x = float(fields[1])
                    y = float(fields[2])
                    if math.isfinite(x) and math.isfinite(y):
                        xs.append(x)
                        ys.append(y)
        except (OSError, ValueError):
            continue
        if xs:
            return min(xs), max(xs), min(ys), max(ys)
    return None


def _acceptance_history_bounds(path):
    if not path or not path.is_file():
        return None
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        history = document["stop_line"]["odometry_history"]
        coordinates = [
            (float(sample["x"]), float(sample["y"])) for sample in history
        ]
        coordinates = [
            (x, y) for x, y in coordinates if math.isfinite(x) and math.isfinite(y)
        ]
        if not coordinates:
            return None
        margin = 5.0
        xs = [coordinate[0] for coordinate in coordinates]
        ys = [coordinate[1] for coordinate in coordinates]
        return (
            min(xs) - margin,
            max(xs) + margin,
            min(ys) - margin,
            max(ys) + margin,
        )
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return None


def _write_rviz_config(text):
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", prefix="lmmg_review_", suffix=".rviz", delete=False
    ) as generated:
        generated.write(text)
        return generated.name


def _auto_fit_rviz_config(
    base_config,
    output_directory,
    preferred_bounds=None,
    frame_id="",
    prefer_trajectory=False,
):
    bounds = (
        preferred_bounds
        or (
            _trajectory_bounds(output_directory)
            if prefer_trajectory
            else _grid_bounds(output_directory)
        )
        or (
            _grid_bounds(output_directory)
            if prefer_trajectory
            else _trajectory_bounds(output_directory)
        )
    )
    if bounds is None:
        get_logger("review.launch").warning(
            "RViz auto-fit could not determine generated-map bounds; using the supplied view"
        )
        if not frame_id:
            return str(base_config)
        text = base_config.read_text(encoding="utf-8")
        text = re.sub(
            r"(?m)^    Fixed Frame:.*$", f"    Fixed Frame: {frame_id}", text
        )
        return _write_rviz_config(text)
    min_x, max_x, min_y, max_y = bounds
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)
    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)
    # review.rviz opens at 1500 x 900. TopDownOrtho Scale is pixels/metre.
    scale = max(0.05, min(20.0, 0.85 * min(1500.0 / span_x, 900.0 / span_y)))

    text = base_config.read_text(encoding="utf-8")
    text = re.sub(r"(?m)^      X:.*$", f"      X: {center_x:.9g}", text)
    text = re.sub(r"(?m)^      Y:.*$", f"      Y: {center_y:.9g}", text)
    if not re.search(r"(?m)^      X:", text):
        text = text.replace(
            "      Name: Current View\n",
            f"      Name: Current View\n      X: {center_x:.9g}\n      Y: {center_y:.9g}\n",
            1,
        )
    text = re.sub(r"(?m)^      Scale:.*$", f"      Scale: {scale:.9g}", text, count=1)
    if frame_id:
        text = re.sub(
            r"(?m)^    Fixed Frame:.*$", f"    Fixed Frame: {frame_id}", text
        )
    generated_path = _write_rviz_config(text)
    get_logger("review.launch").info(
        f"RViz auto-fit center=({center_x:.3f}, {center_y:.3f}) scale={scale:.3f}"
    )
    return generated_path


def _frame_rviz_config(base_config, frame_id):
    text = base_config.read_text(encoding="utf-8")
    text = re.sub(
        r"(?m)^    Fixed Frame:.*$", f"    Fixed Frame: {frame_id}", text
    )
    return _write_rviz_config(text)


def _mode_rviz_config(base_config, review_mode):
    """Build the small, useful display set for a dedicated map editor.

    The bundled file remains a combined/debug configuration.  In the 2D
    navigation-map editor, showing the point cloud, raw diagnostic masks, and
    Lanelet2 at the same time is both noisy and potentially misleading.  Keep
    only the trinary navigation map and the overlays needed to judge its route.
    Matching topics makes this independent of human-facing display names.
    """
    if review_mode not in ("combined", "vector_map", "navigation_map"):
        raise ValueError(
            "review_mode must be combined, vector_map, or navigation_map"
        )
    if review_mode != "navigation_map":
        return str(base_config)

    text = base_config.read_text(encoding="utf-8")
    navigation_topics = {
        "/review_vector_map/navigation_map",
        "/review_vector_map/trajectory_processed",
        "/review_vector_map/route_graph",
        "/review_vector_map/semantic_features",
        "/review_vector_map/issues",
    }
    # Only split list items inside ``Visualization Manager/Displays``.  A
    # document-wide ``^    -`` expression also matches Tools and Panels, and a
    # replacement can accidentally remove the newline between two retained
    # displays.  RViz then accepts the file argument but creates no Display
    # subscriptions because the generated YAML is invalid.
    lines = text.splitlines(keepends=True)
    try:
        displays_header = next(
            index
            for index, line in enumerate(lines)
            if line.rstrip("\r\n") == "  Displays:"
        )
        displays_end = next(
            index
            for index in range(displays_header + 1, len(lines))
            if re.match(r"^  [^\s]", lines[index])
        )
    except StopIteration as error:
        raise ValueError("RViz configuration has no complete Displays section") from error

    item_starts = [
        index
        for index in range(displays_header + 1, displays_end)
        if lines[index].startswith("    - ")
    ]
    if not item_starts:
        raise ValueError("RViz Displays section is empty")

    item_ends = item_starts[1:] + [displays_end]
    retained = []
    retained_topics = set()
    for begin, end in zip(item_starts, item_ends):
        block = "".join(lines[begin:end])
        topic = re.search(
            r"(?m)^        Value:[ \t]*(/review_vector_map/[^\s]+)[ \t]*$",
            block,
        )
        if topic is None or topic.group(1) not in navigation_topics:
            continue
        retained_topics.add(topic.group(1))
        # Every retained layer must be enabled so RViz creates all five
        # subscriptions.  Six-space indentation identifies the Display-level
        # fields and does not alter Topic/Update Topic values.
        block = re.sub(
            r"(?m)^      Enabled:[ \t]*(?:true|false)[ \t]*$",
            "      Enabled: true",
            block,
        )
        block = re.sub(
            r"(?m)^      Value:[ \t]*(?:true|false)[ \t]*$",
            "      Value: true",
            block,
        )
        if not block.endswith(("\n", "\r")):
            block += "\n"
        retained.append(block)

    missing_topics = sorted(navigation_topics - retained_topics)
    if missing_topics:
        raise ValueError(
            "RViz configuration is missing navigation-map displays for: "
            + ", ".join(missing_topics)
        )

    filtered = (
        "".join(lines[: item_starts[0]])
        + "".join(retained)
        + "".join(lines[displays_end:])
    )
    return _write_rviz_config(filtered)


def _start_rviz(
    context,
    start_rviz,
    rviz_config,
    output_directory,
    frame_id,
    view_bounds_file,
    review_mode,
):
    if not _enabled(start_rviz.perform(context)):
        return []
    config_path = Path(rviz_config.perform(context))
    output_path = Path(output_directory.perform(context))
    requested_frame = frame_id.perform(context)
    requested_mode = review_mode.perform(context)
    config_path = Path(
        _mode_rviz_config(config_path, requested_mode)
    )
    bounds_path = view_bounds_file.perform(context)
    preferred_bounds = _acceptance_history_bounds(Path(bounds_path)) if bounds_path else None
    auto_fit = _enabled(LaunchConfiguration("auto_fit_view").perform(context))
    if auto_fit:
        config_path = Path(
            _auto_fit_rviz_config(
                config_path,
                output_path,
                preferred_bounds,
                requested_frame,
                prefer_trajectory=requested_mode == "vector_map",
            )
        )
    elif requested_frame:
        config_path = Path(_frame_rviz_config(config_path, requested_frame))
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="vector_map_review_rviz",
            output="screen",
            arguments=["-d", str(config_path)],
            on_exit=EmitEvent(event=Shutdown(reason="RViz closed")),
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare("lidar_mobility_map_generator")
    default_params = PathJoinSubstitution([package_share, "config", "review.yaml"])
    default_rviz = PathJoinSubstitution([package_share, "rviz", "review.rviz"])

    params_file = LaunchConfiguration("params_file")
    output_directory = LaunchConfiguration("output_directory")
    frame_id = LaunchConfiguration("frame_id")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    publish_lanelet2 = LaunchConfiguration("publish_lanelet2")
    publish_navigation_map = LaunchConfiguration("publish_navigation_map")

    reviewer = Node(
        package="lidar_mobility_map_generator",
        executable="lidar_mobility_map_review",
        name="review_vector_map",
        output="screen",
        parameters=[
            params_file,
            {
                "output_directory": output_directory,
                "frame_id": frame_id,
                "publish.lanelet2": ParameterValue(
                    publish_lanelet2, value_type=bool
                ),
                "publish.navigation_map": ParameterValue(
                    publish_navigation_map, value_type=bool
                ),
            },
        ],
    )

    def stop_after_reviewer(event, _context):
        if event.returncode != 0:
            raise RuntimeError(
                f"review_vector_map failed with exit code {event.returncode}"
            )
        return [EmitEvent(event=Shutdown(reason="review_vector_map stopped"))]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Review-node parameter YAML file",
            ),
            DeclareLaunchArgument(
                "output_directory",
                description="Directory produced by generate_vector_map",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="map",
                description="Optional frame override; empty uses the generated frame",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the supplied review configuration",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz configuration file",
            ),
            DeclareLaunchArgument(
                "review_mode",
                default_value="combined",
                description="combined, vector_map, or navigation_map RViz layers",
            ),
            DeclareLaunchArgument(
                "publish_lanelet2",
                default_value="true",
                description="Publish the Lanelet2 review layer",
            ),
            DeclareLaunchArgument(
                "publish_navigation_map",
                default_value="true",
                description="Publish the trinary 2D navigation-map review layer",
            ),
            DeclareLaunchArgument(
                "auto_fit_view",
                default_value="true",
                description="Center and scale the RViz top-down view from generated grid bounds",
            ),
            DeclareLaunchArgument(
                "view_bounds_file",
                default_value="",
                description="Optional JSON file whose saved XY history sets the view",
            ),
            reviewer,
            OpaqueFunction(
                function=_start_rviz,
                args=[
                    start_rviz,
                    rviz_config,
                    output_directory,
                    frame_id,
                    LaunchConfiguration("view_bounds_file"),
                    LaunchConfiguration("review_mode"),
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=reviewer, on_exit=stop_after_reviewer)
            ),
        ]
    )
