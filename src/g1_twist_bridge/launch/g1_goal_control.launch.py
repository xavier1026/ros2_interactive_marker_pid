import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    global_frame = LaunchConfiguration("global_frame")
    base_frame = LaunchConfiguration("base_frame")
    armed = LaunchConfiguration("armed")
    start_marker = LaunchConfiguration("start_marker")
    start_pid = LaunchConfiguration("start_pid")
    start_bridge = LaunchConfiguration("start_bridge")
    start_rviz = LaunchConfiguration("start_rviz")

    pid_config = os.path.join(
        get_package_share_directory("pid_goal_controller"),
        "config",
        "pid_goal_controller.yaml",
    )

    rviz_config = os.path.join(
        get_package_share_directory("g1_twist_bridge"),
        "rviz",
        "g1_goal_control.rviz",
    )

    bridge_config = os.path.join(
        get_package_share_directory("g1_twist_bridge"),
        "config",
        "g1_twist_bridge.yaml",
    )

    marker_node = Node(
        package="interactive_goal_marker",
        executable="interactive_goal_marker",
        name="interactive_goal_marker",
        output="screen",
        condition=IfCondition(start_marker),
        parameters=[
            {
                "use_sim_time": False,
                "frame_id": frame_id,
                "server_name": "goal_marker",
                "goal_topic": "/goal_pose",
            }
        ],
    )

    pid_node = Node(
        package="pid_goal_controller",
        executable="pid_goal_controller",
        name="pid_goal_controller",
        output="screen",
        condition=IfCondition(start_pid),
        parameters=[
            pid_config,
            {
                "use_sim_time": False,
                "global_frame": global_frame,
                "base_frame": base_frame,
                "goal_topic": "/goal_pose",
                "cmd_vel_topic": "/cmd_vel",

                # 实机初次调试采用较低速度。
                "max_v": 0.10,
                "max_w": 0.15,

                # G1 实机不要把容差设置得过小，否则容易反复调整。
                "pos_tolerance": 0.10,
                "yaw_tolerance": 0.10,
            },
        ],
    )

    bridge_node = Node(
        package="g1_twist_bridge",
        executable="g1_twist_bridge",
        name="g1_twist_bridge",
        output="screen",
        condition=IfCondition(start_bridge),
        parameters=[
            bridge_config,
            {
                "armed": ParameterValue(armed, value_type=bool),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frame_id",
                default_value="map",
                description="Interactive Marker target coordinate frame",
            ),
            DeclareLaunchArgument(
                "global_frame",
                default_value="map",
                description="Global coordinate frame used by PID",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot body frame used by PID",
            ),
            DeclareLaunchArgument(
                "armed",
                default_value="false",
                description="Whether g1_twist_bridge may send commands",
            ),
            DeclareLaunchArgument(
                "start_marker",
                default_value="true",
                description="Whether to start interactive_goal_marker",
            ),
            DeclareLaunchArgument(
                "start_pid",
                default_value="true",
                description="Whether to start pid_goal_controller",
            ),
            DeclareLaunchArgument(
                "start_bridge",
                default_value="true",
                description="Whether to start g1_twist_bridge",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Whether to start RViz2",
            ),
            marker_node,
            pid_node,
            bridge_node,
            rviz_node,
        ]
    )
