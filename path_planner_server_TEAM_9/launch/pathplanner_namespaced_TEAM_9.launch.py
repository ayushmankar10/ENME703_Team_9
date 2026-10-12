import os

from ament_index_python.packages import get_package_share_directory  # pyright: ignore[reportMissingImports]

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # pyright: ignore[reportMissingImports]
from launch.substitutions import LaunchConfiguration  # pyright: ignore[reportMissingImports]

from launch_ros.actions import Node  # pyright: ignore[reportMissingImports]

def launch_setup(context):
    robot_namespace = LaunchConfiguration("namespace").perform(context)

    if not robot_namespace:
        raise RuntimeError(
            "Supply a robot namespace, for example namespace:=tb402"
        )

    package_share = get_package_share_directory(
        "path_planner_server_TEAM_9"
    )
    config_dir = os.path.join(package_share, "config")

    planner_server_yaml = os.path.join(
        config_dir, f"{robot_namespace}_planner_server.yaml"
    )
    controller_yaml = os.path.join(
        config_dir, f"{robot_namespace}_controller.yaml"
    )
    bt_navigator_yaml = os.path.join(
        config_dir, f"{robot_namespace}_bt_navigator.yaml"
    )
    recovery_yaml = os.path.join(
        config_dir, f"{robot_namespace}_recovery.yaml"
    )
    behavior_xml = os.path.join(config_dir, "behavior.xml")

    return [
        Node(
            namespace=robot_namespace,
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[planner_server_yaml],
        ),
        Node(
            namespace=robot_namespace,
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[controller_yaml],
        ),
        Node(
            namespace=robot_namespace,
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                bt_navigator_yaml,
                {"default_nav_to_pose_bt_xml": behavior_xml},
            ],
        ),
        Node(
            namespace=robot_namespace,
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[recovery_yaml],
        ),
        Node(
            namespace=robot_namespace,
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_pathplanner",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "bond_timeout": 0.0,
                    "node_names": [
                        "planner_server",
                        "controller_server",
                        "bt_navigator",
                        "behavior_server",
                    ],
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )