# Verification strategy

## Automated evidence

| Layer | Evidence |
|---|---|
| Geometry | pixel/depth projection, median filtering, invalid-depth rejection |
| Task policy | class/confidence selection and guarded lifecycle transitions |
| Motion policy | workspace boundaries and plan/execute authorisation |
| Configuration | launch syntax, plan-only defaults, YAML and URDF/SRDF checks |
| Integration | clean ROS 2 Jazzy dependency resolution, package build, and package tests in CI |

## End-to-end simulation acceptance criteria

An acceptance run is complete only when recorded evidence shows:

1. all required nodes reach a healthy state;
2. transforms form one connected tree;
3. an object pose is published in a calibrated frame;
4. MoveIt accepts the robot model and planning scene;
5. a collision-free plan reaches pre-grasp, transfer, and retreat targets;
6. invalid depth, unreachable targets, collisions, and cancellation produce controlled failures;
7. the plan-only default never sends a controller command.

This initial version does not claim that acceptance run has been completed.

## Hardware evidence required later

- camera intrinsic and hand-eye calibration reports;
- encoder/joint-state timing and controller watchdog measurements;
- pose accuracy and repeatability over a documented sample size;
- collision model and payload verification;
- emergency-stop, safe-torque-off, guard, and recovery tests;
- cycle-time and failure-recovery results.

