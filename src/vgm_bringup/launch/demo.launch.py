"""Bring up the manipulation stack with Gazebo or mock ros2_control hardware."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    bringup = Path(get_package_share_directory("vgm_bringup"))
    description = Path(get_package_share_directory("vgm_description"))
    moveit = Path(get_package_share_directory("vgm_moveit_config"))
    perception = Path(get_package_share_directory("vgm_perception"))

    use_sim = LaunchConfiguration("use_sim")
    plan_only = LaunchConfiguration("plan_only")
    allow_execution = LaunchConfiguration("allow_execution")
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(description / "urdf" / "vgm_arm.urdf.xacro"),
                " use_gazebo:=",
                use_sim,
            ]
        ),
        value_type=str,
    )
    common_robot_parameters = [
        {"robot_description": robot_description},
        {
            "robot_description_semantic": (
                moveit / "config" / "vgm_arm.srdf"
            ).read_text(encoding="utf-8")
        },
        {
            "robot_description_kinematics": yaml.safe_load(
                (moveit / "config" / "kinematics.yaml").read_text(encoding="utf-8")
            )
        },
        {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("plan_only", default_value="true"),
            DeclareLaunchArgument("allow_execution", default_value="false"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": f"-r {bringup / 'worlds' / 'tabletop.sdf'}"}.items(),
                condition=IfCondition(use_sim),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "vgm_arm", "-topic", "robot_description"],
                condition=IfCondition(use_sim),
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": str(description / "config" / "gz_bridge.yaml")}],
                condition=IfCondition(use_sim),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[str(description / "config" / "ros2_controllers.yaml")],
                remappings=[("~/robot_description", "/robot_description")],
                condition=UnlessCondition(use_sim),
                output="screen",
            ),
            *[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"],
                    output="screen",
                )
                for name in (
                    "joint_state_broadcaster",
                    "arm_controller",
                    "gripper_controller",
                )
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit / "launch" / "move_group.launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim,
                    "allow_trajectory_execution": allow_execution,
                }.items(),
            ),
            Node(
                package="vgm_perception",
                executable="rgbd_pose_estimator",
                parameters=[
                    str(perception / "config" / "perception.yaml"),
                    {"output_frame": "world", "use_sim_time": use_sim},
                ],
                output="screen",
            ),
            Node(
                package="vgm_motion",
                executable="motion_server",
                parameters=[
                    str(bringup / "config" / "task.yaml"),
                    *common_robot_parameters,
                    {"execution_enabled": ParameterValue(allow_execution, value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                package="vgm_task_manager",
                executable="task_manager",
                parameters=[
                    str(bringup / "config" / "task.yaml"),
                    {"plan_only": ParameterValue(plan_only, value_type=bool)},
                    {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
                ],
                output="screen",
            ),
        ]
    )
