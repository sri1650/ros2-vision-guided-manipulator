"""Run a self-contained MoveIt demo against ros2_control mock hardware."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():
    description_share = Path(get_package_share_directory("vgm_description"))
    moveit_share = Path(get_package_share_directory("vgm_moveit_config"))
    robot_description = xacro.process_file(
        str(description_share / "urdf" / "vgm_arm.urdf.xacro"),
        mappings={"use_gazebo": "false"},
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[str(description_share / "config" / "ros2_controllers.yaml")],
        remappings=[("~/robot_description", "/robot_description")],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / "launch" / "move_group.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "allow_trajectory_execution": "true",
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(moveit_share / "config" / "moveit.rviz")],
        parameters=[
            {"robot_description": robot_description},
            {
                "robot_description_semantic": (
                    moveit_share / "config" / "vgm_arm.srdf"
                ).read_text(encoding="utf-8")
            },
            {
                "robot_description_kinematics": _read_yaml(
                    moveit_share / "config" / "kinematics.yaml"
                )
            },
        ],
    )

    return LaunchDescription(
        [
            world_to_base,
            robot_state_publisher,
            ros2_control,
            joint_state_broadcaster,
            arm_controller,
            gripper_controller,
            move_group,
            rviz,
        ]
    )


def _read_yaml(path: Path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)
