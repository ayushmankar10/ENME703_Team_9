from launch import LaunchDescription  # pyright: ignore[reportMissingImports]
from launch.actions import DeclareLaunchArgument  # pyright: ignore[reportMissingImports]
from launch.substitutions import LaunchConfiguration  # pyright: ignore[reportMissingImports]
from launch_ros.actions import Node  # pyright: ignore[reportMissingImports]


def generate_launch_description():
    chaser = LaunchConfiguration("chaser")
    enemy = LaunchConfiguration("enemy")
    global_frame = LaunchConfiguration("global_frame")

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
            Node(
                package="tracker_TEAM_9",
                executable="tracker_two_tb4",
                namespace=chaser,
                name="tracker_two_tb4",
                output="screen",
                parameters=[
                    {
                        "chaser": chaser,
                        "enemy": enemy,
                        "global_frame": global_frame,
                        "use_sim_time": False,
                    }
                ],
            ),
        ]
    )