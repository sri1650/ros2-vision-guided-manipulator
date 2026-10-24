# Demonstration walkthrough

Build and launch in plan-only mode:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch vgm_bringup demo.launch.py plan_only:=true
```

Useful observations:

```bash
ros2 topic echo /perception/objects
ros2 action info /pick_place
ros2 topic hz /joint_states
ros2 run tf2_tools view_frames
```

Success means a valid object becomes a selected target, the request passes workspace validation, and MoveIt reports a plan-ready result without executing it. Test failure paths by removing valid depth, lowering confidence, moving the object outside the workspace, and cancelling an active goal.

