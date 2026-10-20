# pyright: reportMissingImports=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sim_camera_TEAM_9"),
        "config",
        "sim_camera_tb4.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=config,
            description="TB4 synthetic camera parameters",
        ),
        Node(
            package="sim_camera_TEAM_9",
            executable="sim_camera",
            namespace="tb402",
            name="sim_camera",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": True},
            ],
        ),
    ])