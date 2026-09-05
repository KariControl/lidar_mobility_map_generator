from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    output_directory = LaunchConfiguration("output_directory")
    bind = LaunchConfiguration("bind")
    port = LaunchConfiguration("port")
    max_points = LaunchConfiguration("max_points")
    open_browser = LaunchConfiguration("open_browser")
    read_only = LaunchConfiguration("read_only")
    editor_mode = LaunchConfiguration("editor_mode")
    enable_autoware_one_click_export = LaunchConfiguration(
        "enable_autoware_one_click_export"
    )
    autoware_one_click_session = LaunchConfiguration(
        "autoware_one_click_session"
    )

    editor = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "lidar_mobility_map_generator",
            "semantic_map_editor",
            "--output-directory",
            output_directory,
            "--bind",
            bind,
            "--port",
            port,
            "--max-points",
            max_points,
            "--open-browser",
            open_browser,
            "--read-only",
            read_only,
            "--editor-mode",
            editor_mode,
            "--enable-autoware-one-click-export",
            enable_autoware_one_click_export,
            "--autoware-one-click-session",
            autoware_one_click_session,
        ],
        output="screen",
    )

    def propagate_editor_failure(event, _context):
        if event.returncode != 0:
            raise RuntimeError(
                f"semantic_map_editor failed with exit code {event.returncode}"
            )
        return []

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_directory",
                description="Directory produced by generate_vector_map",
            ),
            DeclareLaunchArgument("bind", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("max_points", default_value="150000"),
            DeclareLaunchArgument("open_browser", default_value="true"),
            DeclareLaunchArgument("read_only", default_value="false"),
            DeclareLaunchArgument(
                "editor_mode",
                default_value="combined",
                description="combined, vector_map, or navigation_map",
            ),
            DeclareLaunchArgument(
                "enable_autoware_one_click_export", default_value="false"
            ),
            DeclareLaunchArgument("autoware_one_click_session", default_value="disabled"),
            editor,
            RegisterEventHandler(
                OnProcessExit(target_action=editor, on_exit=propagate_editor_failure)
            ),
        ]
    )
