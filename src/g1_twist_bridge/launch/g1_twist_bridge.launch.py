from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("g1_twist_bridge")
    default_config = f"{package_share}/config/g1_twist_bridge.yaml"

    config_file = LaunchConfiguration("config_file")
    armed = LaunchConfiguration("armed")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the g1_twist_bridge parameter file",
            ),
            DeclareLaunchArgument(
                "armed",
                default_value="false",
                description="Allow non-zero commands to reach the Unitree sport API",
            ),
            Node(
                package="g1_twist_bridge",
                executable="g1_twist_bridge",
                name="g1_twist_bridge",
                output="screen",
                parameters=[
                    config_file,
                    {"armed": ParameterValue(armed, value_type=bool)},
                ],
            ),
        ]
    )
