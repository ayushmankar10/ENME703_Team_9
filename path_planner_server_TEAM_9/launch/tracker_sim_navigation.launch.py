import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context):
    localization_share = get_package_share_directory(
        "localization_server_TEAM_9"
    )
    planner_share = get_package_share_directory(
        "path_planner_server_TEAM_9"
    )

    map_file = LaunchConfiguration("map").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    if not map_file:
        map_file = os.path.join(
            localization_share, "maps", "map.yaml"
        )

    if not os.path.isfile(map_file):
        raise RuntimeError(f"Map YAML not found: {map_file}")

    behavior_xml = os.path.join(
        planner_share, "config", "behavior.xml"
    )

    actions = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_file,
                "use_sim_time": True,
                "frame_id": "map",
                "topic_name": "map",
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ),
    ]

    for robot in ("tb3_0", "tb3_1"):
        amcl_yaml = os.path.join(
            localization_share,
            "config",
            f"{robot}_amcl_config.yaml",
        )

        if not os.path.isfile(amcl_yaml):
            raise RuntimeError(f"AMCL config not found: {amcl_yaml}")

        actions.append(
            Node(
                package="nav2_amcl",
                executable="amcl",
                namespace=robot,
                name="amcl",
                output="screen",
                parameters=[amcl_yaml, {"use_sim_time": True}],
            )
        )

        navigation_nodes = [
            ("nav2_planner", "planner_server", "planner_server"),
            ("nav2_controller", "controller_server", "controller"),
            ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
            ("nav2_behaviors", "behavior_server", "recovery"),
        ]

        for package, executable, config_name in navigation_nodes:
            config_file = os.path.join(
                planner_share,
                "config",
                f"{robot}_{config_name}.yaml",
            )

            if not os.path.isfile(config_file):
                raise RuntimeError(
                    f"Navigation config not found: {config_file}"
                )

            parameters = [
                config_file,
                {"use_sim_time": True},
            ]

            if executable == "bt_navigator":
                parameters.append({
                    "default_nav_to_pose_bt_xml": behavior_xml,
                })

            actions.append(
                Node(
                    package=package,
                    executable=executable,
                    namespace=robot,
                    name=executable,
                    output="screen",
                    parameters=parameters,
                )
            )

        actions.extend([
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=robot,
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": ["amcl"],
                }],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=robot,
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": [
                        "planner_server",
                        "controller_server",
                        "bt_navigator",
                        "behavior_server",
                    ],
                }],
            ),
        ])

    rviz_arguments = []

    if rviz_config:
        if not os.path.isfile(rviz_config):
            raise RuntimeError(
                f"RViz config not found: {rviz_config}"
            )
        rviz_arguments = ["-d", rviz_config]

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="team9_rviz",
            output="screen",
            arguments=rviz_arguments,
            parameters=[{"use_sim_time": True}],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Optional absolute path to the map YAML",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Optional absolute path to an RViz config",
        ),
        OpaqueFunction(function=launch_setup),
    ])