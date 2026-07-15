from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = PathJoinSubstitution(
        [get_package_share_directory("pid_goal_controller"), "config", "pid_goal_controller.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            Node(
                package="pid_goal_controller",
                executable="pid_goal_controller",
                name="pid_goal_controller",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
