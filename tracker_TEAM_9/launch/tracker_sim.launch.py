# pyright: reportMissingImports=false

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("tracker_TEAM_9")

    default_config = os.path.join(
        package_share,
        "config",
        "tracker_sim.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_config,
            description="Path to the simulation tracker parameters",
        ),
        Node(
            package="tracker_TEAM_9",
            executable="tracker_two_tb4",
            namespace="tb3_0",
            name="tracker_two_tb4",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": True},
            ],
        ),
    ])