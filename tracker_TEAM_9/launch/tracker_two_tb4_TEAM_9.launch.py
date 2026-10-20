from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    chaser = LaunchConfiguration("chaser")
    enemy = LaunchConfiguration("enemy")
    global_frame = LaunchConfiguration("global_frame")
    chaser_base_frame = LaunchConfiguration("chaser_base_frame")
    enemy_base_frame = LaunchConfiguration("enemy_base_frame")
    camera_topic = LaunchConfiguration("camera_topic")
    nav_action_topic = LaunchConfiguration("nav_action_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "chaser",
                default_value="tb402",
                description="Namespace of the chasing robot",
            ),
            DeclareLaunchArgument(
                "enemy",
                default_value="tb407",
                description="Namespace of the robot being tracked",
            ),
            DeclareLaunchArgument(
                "global_frame",
                default_value="map",
                description="Shared global reference frame",
            ),
            DeclareLaunchArgument(
                "chaser_base_frame",
                default_value="",
                description="Base frame of the chaser (empty = {chaser}/base_link)",
            ),
            DeclareLaunchArgument(
                "enemy_base_frame",
                default_value="",
                description="Base frame of the enemy (empty = {enemy}/base_link)",
            ),
            DeclareLaunchArgument(
                "camera_topic",
                default_value="",
                description="Camera topic for capture (empty = default Oak-D topic)",
            ),
            DeclareLaunchArgument(
                "nav_action_topic",
                default_value="",
                description="Action topic for navigation (empty = /{chaser}/navigate_to_pose)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (/clock) if true",
            ),
            Node(
                package="tracker_TEAM_9",
                executable="tracker_two_tb4",
                name="tracker_two_tb4",
                output="screen",
                parameters=[
                    {
                        "chaser": chaser,
                        "enemy": enemy,
                        "global_frame": global_frame,
                        "chaser_base_frame": chaser_base_frame,
                        "enemy_base_frame": enemy_base_frame,
                        "camera_topic": camera_topic,
                        "nav_action_topic": nav_action_topic,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )