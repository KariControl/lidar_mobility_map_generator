from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("lidar_mobility_map_generator")
    default_params = PathJoinSubstitution([package_share, "config", "review.yaml"])
    default_rviz = PathJoinSubstitution([package_share, "rviz", "review.rviz"])
    review_launch = PathJoinSubstitution([package_share, "launch", "review.launch.py"])

    output_directory = LaunchConfiguration("output_directory")
    params_file = LaunchConfiguration("params_file")
    frame_id = LaunchConfiguration("frame_id")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    auto_fit_view = LaunchConfiguration("auto_fit_view")
    editor_port = LaunchConfiguration("editor_port")
    max_points = LaunchConfiguration("max_points")
    open_browser = LaunchConfiguration("open_browser")
    editor_mode = LaunchConfiguration("editor_mode")
    publish_lanelet2 = LaunchConfiguration("publish_lanelet2")
    publish_navigation_map = LaunchConfiguration("publish_navigation_map")
    enable_autoware_one_click_export = LaunchConfiguration(
        "enable_autoware_one_click_export"
    )
    autoware_one_click_session = LaunchConfiguration(
        "autoware_one_click_session"
    )

    reviewer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(review_launch),
        launch_arguments={
            "params_file": params_file,
            "output_directory": output_directory,
            "frame_id": frame_id,
            "start_rviz": start_rviz,
            "rviz_config": rviz_config,
            "auto_fit_view": auto_fit_view,
            "review_mode": editor_mode,
            "publish_lanelet2": publish_lanelet2,
            "publish_navigation_map": publish_navigation_map,
        }.items(),
    )
    editor = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "lidar_mobility_map_generator",
            "semantic_map_editor",
            "--output-directory",
            output_directory,
            "--port",
            editor_port,
            "--max-points",
            max_points,
            "--editor-mode",
            editor_mode,
            "--open-browser",
            open_browser,
            "--enable-autoware-one-click-export",
            enable_autoware_one_click_export,
            "--autoware-one-click-session",
            autoware_one_click_session,
        ],
        output="screen",
    )

    def stop_after_required_process(name):
        def handler(event, _context):
            if event.returncode != 0:
                raise RuntimeError(f"{name} failed with exit code {event.returncode}")
            return [EmitEvent(event=Shutdown(reason=f"{name} stopped"))]

        return handler

    return LaunchDescription(
        [
            DeclareLaunchArgument("output_directory"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("auto_fit_view", default_value="true"),
            DeclareLaunchArgument("editor_port", default_value="8765"),
            DeclareLaunchArgument("max_points", default_value="150000"),
            DeclareLaunchArgument("open_browser", default_value="true"),
            DeclareLaunchArgument(
                "editor_mode",
                default_value="combined",
                description="combined, vector_map, or navigation_map",
            ),
            DeclareLaunchArgument("publish_lanelet2", default_value="true"),
            DeclareLaunchArgument("publish_navigation_map", default_value="true"),
            DeclareLaunchArgument(
                "enable_autoware_one_click_export", default_value="false"
            ),
            DeclareLaunchArgument("autoware_one_click_session", default_value="disabled"),
            reviewer,
            editor,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=editor,
                    on_exit=stop_after_required_process("semantic_map_editor"),
                )
            ),
        ]
    )
