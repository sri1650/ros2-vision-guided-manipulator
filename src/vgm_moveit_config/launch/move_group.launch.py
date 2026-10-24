"""Start MoveIt 2 for the VGM arm; robot/controller bringup is separate."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def _load_yaml(path: Path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _moveit_parameters(use_sim_time):
    description_share = Path(get_package_share_directory("vgm_description"))
    config_share = Path(get_package_share_directory("vgm_moveit_config")) / "config"

    robot_description = xacro.process_file(
        str(description_share / "urdf" / "vgm_arm.urdf.xacro"),
        mappings={"use_gazebo": "false"},
    ).toxml()
    semantic = (config_share / "vgm_arm.srdf").read_text(encoding="utf-8")

    ompl = _load_yaml(config_share / "ompl_planning.yaml")
    controllers = _load_yaml(config_share / "moveit_controllers.yaml")
    sensors = _load_yaml(config_share / "sensors_3d.yaml")

    return [
        {"robot_description": robot_description},
        {"robot_description_semantic": semantic},
        {
            "robot_description_kinematics": _load_yaml(
                config_share / "kinematics.yaml"
            )
        },
        {
            "robot_description_planning": _load_yaml(
                config_share / "joint_limits.yaml"
            )
        },
        {
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl,
        },
        controllers,
        sensors,
        {
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "planning_scene_monitor_options": {
                "name": "planning_scene_monitor",
                "robot_description": "robot_description",
                "joint_state_topic": "/joint_states",
                "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
                "publish_planning_scene_topic": "/planning_scene",
                "monitored_planning_scene_topic": "/monitored_planning_scene",
                "wait_for_initial_state_timeout": 10.0,
            },
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "octomap_frame": "world",
            "octomap_resolution": 0.015,
        },
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    allow_execution = LaunchConfiguration("allow_trajectory_execution")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "allow_trajectory_execution", default_value="true"
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=_moveit_parameters(use_sim_time)
                + [
                    {
                        "allow_trajectory_execution": ParameterValue(
                            allow_execution, value_type=bool
                        )
                    }
                ],
            ),
        ]
    )
