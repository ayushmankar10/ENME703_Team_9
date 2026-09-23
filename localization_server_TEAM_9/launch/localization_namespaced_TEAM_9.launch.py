import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context):

    robot_namespace = LaunchConfiguration("namespace").perform(context)

    nav2_yaml = os.path.join(get_package_share_directory("localization_server"),"config",f"{robot_namespace}_amcl_config.yaml")
   
    return [

        Node(
            ........, # your namespace
            package=........
            executable=........,
            name=........,
            output=........,
            parameters=[........]
        ),

        Node(
            ........, # your namespace
            package=........,
            executable=........,
            name=........,
            output=........,
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"bond_timeout": 0.0},
                {"node_names": [........]}
            ]
        )
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value=" ",
            description="Robot namespace"
        ),

        OpaqueFunction(function=launch_setup)

    ])