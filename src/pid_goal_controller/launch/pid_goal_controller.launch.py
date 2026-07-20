from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    global_frame = LaunchConfiguration("global_frame")
    base_frame = LaunchConfiguration("base_frame")
    goal_topic = LaunchConfiguration("goal_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    error_topic = LaunchConfiguration("error_topic")
    distance_topic = LaunchConfiguration("distance_topic")
    config_path = PathJoinSubstitution(
        [get_package_share_directory("pid_goal_controller"), "config", "pid_goal_controller.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "global_frame", default_value="map", description="Global localization frame"
            ),
            DeclareLaunchArgument(
                "base_frame", default_value="base_link", description="Robot body frame"
            ),
            DeclareLaunchArgument(
                "goal_topic", default_value="/goal_pose", description="Goal pose topic"
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic", default_value="/cmd_vel", description="Velocity command topic"
            ),
            DeclareLaunchArgument(
                "error_topic", default_value="/pid_error", description="PID error debug topic"
            ),
            DeclareLaunchArgument(
                "distance_topic",
                default_value="/pid_distance",
                description="Goal distance debug topic",
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
                        "global_frame": global_frame,
                        "base_frame": base_frame,
                        "goal_topic": goal_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "error_topic": error_topic,
                        "distance_topic": distance_topic,
                    },
                ],
            ),
        ]
    )
