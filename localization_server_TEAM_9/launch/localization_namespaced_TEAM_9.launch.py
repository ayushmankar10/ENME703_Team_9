# pyright: reportMissingImports=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robot = LaunchConfiguration("namespace").perform(context).strip("/")
    if not robot:
        raise RuntimeError("Supply namespace:=tb402 or your robot namespace")

    base_frame = LaunchConfiguration("base_frame_id").perform(context)
    odom_frame = LaunchConfiguration("odom_frame_id").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )
    start_map = (
        LaunchConfiguration("start_map").perform(context).lower() == "true"
    )

    share = get_package_share_directory("localization_server_TEAM_9")
    config = os.path.join(share, "config", f"{robot}_amcl_config.yaml")
    map_yaml = LaunchConfiguration("map").perform(context)
    if not map_yaml:
        map_yaml = os.path.join(share, "maps", "map.yaml")

    if not os.path.isfile(config):
        raise RuntimeError(f"AMCL config not found: {config}")

    actions = []
    managed_nodes = []

    if start_map:
        if not os.path.isfile(map_yaml):
            raise RuntimeError(f"Map not found: {map_yaml}")

        actions.append(
            Node(
                namespace=robot,
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "yaml_filename": map_yaml,
                    "frame_id": "map",
                }],
                remappings=[("map", "/map")],
            )
        )
        managed_nodes.append("map_server")

    actions.append(
        Node(
            namespace=robot,
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "base_frame_id": base_frame or f"{robot}/base_link",
                    "odom_frame_id": odom_frame or f"{robot}/odom",
                    "scan_topic": scan_topic,
                    "global_frame_id": "map",
                    "map_topic": "/map",
                },
            ],
        )
    )
    managed_nodes.append("amcl")

    actions.append(
        Node(
            namespace=robot,
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": managed_nodes,
            }],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="tb402"),
        DeclareLaunchArgument("base_frame_id", default_value=""),
        DeclareLaunchArgument("odom_frame_id", default_value=""),
        DeclareLaunchArgument("scan_topic", default_value="scan_copy"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "start_map",
            default_value="false",
            description="Enable on one robot only if no shared map server exists",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Map YAML; empty uses this package's maps/map.yaml",
        ),
        OpaqueFunction(function=launch_setup),
    ])