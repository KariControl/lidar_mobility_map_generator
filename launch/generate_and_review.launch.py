from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("lidar_mobility_map_generator")
    default_review_params = PathJoinSubstitution(
        [package_share, "config", "review.yaml"]
    )
    default_rviz = PathJoinSubstitution([package_share, "rviz", "review.rviz"])

    generator_params = LaunchConfiguration("generator_params_file")
    review_params = LaunchConfiguration("review_params_file")
    output_directory = LaunchConfiguration("output_directory")
    frame_id = LaunchConfiguration("frame_id")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    generator = Node(
        package="lidar_mobility_map_generator",
        executable="lidar_mobility_map_generator",
        name="lidar_mobility_map_generator",
        output="screen",
        parameters=[generator_params],
    )
    reviewer = Node(
        package="lidar_mobility_map_generator",
        executable="lidar_mobility_map_review",
        name="review_vector_map",
        output="screen",
        parameters=[
            review_params,
            {
                "output_directory": output_directory,
                "frame_id": frame_id,
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="vector_map_review_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )

    def start_review_after_success(event, _context):
        if event.returncode != 0:
            # Raising from the event handler makes `ros2 launch` return a
            # failure status as well as preventing stale output from opening.
            raise RuntimeError(
                "map generation failed with exit code "
                f"{event.returncode}; review was not started"
            )
        return [reviewer, rviz]

    def stop_after_reviewer(event, _context):
        if event.returncode != 0:
            raise RuntimeError(
                f"review_vector_map failed with exit code {event.returncode}"
            )
        return [EmitEvent(event=Shutdown(reason="review_vector_map stopped"))]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "generator_params_file",
                description="Generator parameter YAML file",
            ),
            DeclareLaunchArgument(
                "review_params_file",
                default_value=default_review_params,
                description="Review-node parameter YAML file",
            ),
            DeclareLaunchArgument(
                "output_directory",
                description=(
                    "Must match output.directory in the generator parameter file"
                ),
            ),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            generator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=generator,
                    on_exit=start_review_after_success,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=reviewer, on_exit=stop_after_reviewer)
            ),
        ]
    )
