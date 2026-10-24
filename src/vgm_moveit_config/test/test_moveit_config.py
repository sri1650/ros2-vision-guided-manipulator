"""Cross-file consistency tests for the hand-authored MoveIt configuration."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


CONFIG_ROOT = Path(__file__).resolve().parents[1] / "config"
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]


def _yaml(name: str):
    with (CONFIG_ROOT / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_srdf_declares_chain_gripper_and_end_effector():
    root = ET.parse(CONFIG_ROOT / "vgm_arm.srdf").getroot()
    groups = {group.attrib["name"]: group for group in root.findall("group")}

    chain = groups["manipulator"].find("chain")
    assert chain.attrib == {"base_link": "base_link", "tip_link": "tool0"}
    assert {
        joint.attrib["name"] for joint in groups["gripper"].findall("joint")
    } == {"left_finger_joint", "right_finger_joint"}

    end_effector = root.find("end_effector")
    assert end_effector.attrib["group"] == "gripper"
    assert end_effector.attrib["parent_group"] == "manipulator"


def test_controller_and_limit_joint_sets_are_consistent():
    controllers = _yaml("moveit_controllers.yaml")
    manager = controllers["moveit_simple_controller_manager"]
    assert manager["arm_controller"]["joints"] == ARM_JOINTS
    assert manager["gripper_controller"]["joints"] == ["left_finger_joint"]

    limits = _yaml("joint_limits.yaml")["joint_limits"]
    assert set(ARM_JOINTS).issubset(limits)
    assert all(limits[joint]["max_velocity"] > 0 for joint in ARM_JOINTS)


def test_planning_and_depth_sensor_configuration_is_bounded():
    ompl = _yaml("ompl_planning.yaml")
    assert ompl["manipulator"]["default_planner_config"] in ompl["planner_configs"]
    assert 0.0 < ompl["manipulator"]["longest_valid_segment_fraction"] <= 0.05

    sensors = _yaml("sensors_3d.yaml")
    assert sensors["depth_camera"]["point_cloud_topic"] == "/camera/depth/points"
    assert 0.0 < sensors["depth_camera"]["max_range"] <= 3.0
