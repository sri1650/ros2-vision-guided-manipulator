"""Launch the RGB-D perception baseline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("vgm_perception"), "config", "perception.yaml"]
    )
    config_argument = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Perception parameter YAML file",
    )
    node = Node(
        package="vgm_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )
    return LaunchDescription([config_argument, node])
