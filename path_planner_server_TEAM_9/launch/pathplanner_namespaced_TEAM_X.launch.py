import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context):

    robot_namespace = LaunchConfiguration("namespace").perform(context)

    config_dir = os.path.join(get_package_share_directory("path_planner_server_TEAM_X"),"config")

    planner_server_yaml = os.path.join(config_dir,.....)
    controller_yaml     = os.path.join(config_dir,.....)
    bt_navigator_yaml   = os.path.join(config_dir,.....)
    recovery_yaml       = os.path.join(config_dir,.....)
        


    return [

        # Write all required nodes.
        .............
        .............
        ............
        .............


        Node(
            namespace=robot_namespace,
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_pathplanner",
            output="screen",
            parameters=[
                {"autostart": True},
                {"bond_timeout": 0.0},
                {
                    
                    "node_names": [
                        ..........
                        ..........
                        ..........
                        ..........
                    ]
                }
            ]
        )
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace"
        ),

        OpaqueFunction(function=launch_setup)

    ])