
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context):

    robot_namespace = LaunchConfiguration("namespace").perform(context)
    base_frame = LaunchConfiguration("base_frame_id").perform(context)
    odom_frame = LaunchConfiguration("odom_frame_id").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)

    pkg_share = get_package_share_directory("localization_server_TEAM_9")

    nav2_yaml = os.path.join(
        pkg_share, "config", f"{robot_namespace}_amcl_config.yaml"
    )
    map_yaml = os.path.join(pkg_share, "maps", "map.yaml")

    return [

        Node(
            namespace=robot_namespace,
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"yaml_filename": map_yaml}
            ],
            remappings=[("map", "/map")]
        ),

        Node(
            namespace=robot_namespace,
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                nav2_yaml,
                {
                    "base_frame_id": base_frame,
                    "odom_frame_id": odom_frame,
                    "scan_topic": scan_topic,
                    "global_frame_id": "map",
                    "map_topic": "/map",
                }
            ]
        ),

        Node(
            namespace=robot_namespace,
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"bond_timeout": 0.0},
                {"node_names": ["map_server", "amcl"]}
            ]
        )
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="tb402",
            description="Namespace selecting an existing AMCL config"
        ),

        DeclareLaunchArgument(
            "base_frame_id",
            default_value="tb402/base_link",
            description="TF frame of the robot base"
        ),

        DeclareLaunchArgument(
            "odom_frame_id",
            default_value="tb402/odom",
            description="TF odometry frame of the selected robot"
        ),

        DeclareLaunchArgument(
            "scan_topic",
            default_value="scan_copy",
            description="Laser scan topic for the selected robot"
        ),

        OpaqueFunction(function=launch_setup)

    ])
