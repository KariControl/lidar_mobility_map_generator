from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="Absolute path to the generator parameter YAML file",
            ),
            Node(
                package="lidar_mobility_map_generator",
                executable="lidar_mobility_map_generator",
                name="lidar_mobility_map_generator",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
