# Simulation setup

## Supported baseline

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2 for Jazzy
- Gazebo Harmonic through `ros_gz`
- `ros2_control` and `gz_ros2_control`

Start with `plan_only:=true` and inspect `/tf`, `/joint_states`, `/perception/objects`, `/pick_place/_action/status`, and the MoveIt planning scene.

## Known initial limitation

The repository contains the models, configuration, and launch composition, but the first graphical Gazebo/MoveIt run must be performed on a compatible Linux host. macOS is suitable for editing and unit tests, not for claiming this ROS/Gazebo integration has run.

Capture the eventual acceptance evidence as screenshots, a short demo video, rosbag data, and the exact commit/OS/dependency versions used.

