"""Structural checks for the expanded robot description."""

import xml.etree.ElementTree as ET
from pathlib import Path

import xacro


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_ROOT / "urdf" / "vgm_arm.urdf.xacro"


def _expanded_root(use_gazebo: bool = False) -> ET.Element:
    document = xacro.process_file(
        str(XACRO_FILE), mappings={"use_gazebo": str(use_gazebo).lower()}
    )
    return ET.fromstring(document.toxml())


def test_six_axis_chain_and_parallel_gripper_are_present():
    root = _expanded_root()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    for index in range(1, 7):
        assert joints[f"joint_{index}"].attrib["type"] == "revolute"

    assert joints["left_finger_joint"].attrib["type"] == "prismatic"
    mimic = joints["right_finger_joint"].find("mimic")
    assert mimic is not None
    assert mimic.attrib["joint"] == "left_finger_joint"
    assert joints["tcp_joint"].find("child").attrib["link"] == "tcp_link"


def test_camera_and_control_interfaces_are_wired_for_simulation():
    root = _expanded_root(use_gazebo=True)
    links = {link.attrib["name"] for link in root.findall("link")}
    assert "camera_mast_link" in links
    assert "camera_color_optical_frame" in links
    assert "camera_depth_optical_frame" in links

    controlled_joints = {
        joint.attrib["name"] for joint in root.findall("./ros2_control/joint")
    }
    assert controlled_joints == {
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
        "left_finger_joint",
    }
    assert root.find(".//sensor[@name='rgbd_camera']") is not None
    assert root.find(".//plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']") is not None


def test_all_moving_joints_have_limits():
    root = _expanded_root()
    for joint in root.findall("joint"):
        if joint.attrib["type"] in {"revolute", "prismatic"}:
            limit = joint.find("limit")
            assert limit is not None, joint.attrib["name"]
            assert float(limit.attrib["lower"]) <= float(limit.attrib["upper"])
            assert float(limit.attrib["velocity"]) > 0.0
            assert float(limit.attrib["effort"]) > 0.0
