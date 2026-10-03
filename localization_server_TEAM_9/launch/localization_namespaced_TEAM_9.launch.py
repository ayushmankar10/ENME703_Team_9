import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context):

    robot_namespace = LaunchConfiguration("namespace").perform(context)

    nav2_yaml = os.path.join(get_package_share_directory("localization_server_TEAM_9"),"config",f"{robot_namespace}_amcl_config.yaml")
   
    return [

        Node(
            namespace=robot_namespace, # your namespace
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_yaml]
        ),

        Node(
            namespace=robot_namespace, # your namespace
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"bond_timeout": 0.0},
                {"node_names": ["amcl"]}
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