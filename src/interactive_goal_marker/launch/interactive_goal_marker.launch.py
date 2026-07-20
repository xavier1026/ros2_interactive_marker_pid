from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_id = LaunchConfiguration("frame_id")
    server_name = LaunchConfiguration("server_name")
    goal_topic = LaunchConfiguration("goal_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument("frame_id", default_value="map", description="Goal frame ID"),
            DeclareLaunchArgument(
                "server_name",
                default_value="goal_marker",
                description="Interactive Marker Server name",
            ),
            DeclareLaunchArgument(
                "goal_topic", default_value="/goal_pose", description="Published goal topic"
            ),
            Node(
                package="interactive_goal_marker",
                executable="interactive_goal_marker",
                name="interactive_goal_marker",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_id": frame_id,
                        "server_name": server_name,
                        "goal_topic": goal_topic,
                    }
                ],
            ),
        ]
    )
