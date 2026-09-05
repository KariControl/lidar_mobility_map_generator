from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("lidar_mobility_map_generator")
    default_params = PathJoinSubstitution([package_share, "config", "review.yaml"])
    default_rviz = PathJoinSubstitution([package_share, "rviz", "review.rviz"])
    shared_editor = PathJoinSubstitution(
        [package_share, "launch", "edit_and_review.launch.py"]
    )

    arguments = {
        "output_directory": LaunchConfiguration("output_directory"),
        "params_file": LaunchConfiguration("params_file"),
        "frame_id": LaunchConfiguration("frame_id"),
        "start_rviz": LaunchConfiguration("start_rviz"),
        "rviz_config": LaunchConfiguration("rviz_config"),
        "auto_fit_view": LaunchConfiguration("auto_fit_view"),
        "editor_port": LaunchConfiguration("editor_port"),
        "max_points": LaunchConfiguration("max_points"),
        "open_browser": LaunchConfiguration("open_browser"),
        "editor_mode": "navigation_map",
        "publish_lanelet2": "false",
        "publish_navigation_map": "true",
        "enable_autoware_one_click_export": "false",
        "autoware_one_click_session": "disabled",
    }

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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(shared_editor),
                launch_arguments=arguments.items(),
            ),
        ]
    )
