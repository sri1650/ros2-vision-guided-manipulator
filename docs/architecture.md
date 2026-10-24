# Architecture

## Design goals

The stack separates perception, task policy, motion planning, and actuation so each boundary can be tested or replaced independently. Camera algorithms never command joints, while motion execution requires both a valid plan and an explicit execution flag.

## Runtime components

| Component | Input | Output | Failure behaviour |
|---|---|---|---|
| RGB-D pose estimator | colour, aligned depth, camera info | `/perception/objects` | drops invalid/stale depth and publishes no target |
| Task manager | detected objects | `/pick_place` goal | waits if confidence or class policy is not met |
| Motion server | `PickPlace` goal | staged feedback/result | rejects invalid geometry or unauthorised execution |
| MoveIt 2 | targets, scene, robot state | collision-checked plan | returns planning failure without controller command |
| ros2_control | validated trajectory | joint commands/status | hardware-specific controller owns low-level faults |

## Coordinate frames

- `world`: stationary cell reference.
- `base_link`: arm mounting datum.
- `camera_link` / `camera_optical_frame`: calibrated RGB-D sensor frames.
- `tool0`: planning end-effector reference.
- `left_finger` and `right_finger`: gripper contact geometry.

Perception reports poses in the image header frame. The motion boundary must transform them into `world` and reject stale or unavailable transforms before planning. Static transforms in simulation are not a substitute for hand-eye calibration on hardware.

## Pick/place sequence

```mermaid
sequenceDiagram
  participant P as Perception
  participant T as Task manager
  participant M as Motion server
  participant V as MoveIt 2
  participant C as Controller
  P->>T: DetectedObjectArray
  T->>M: PickPlace(target, place_pose, execute)
  M->>M: Validate confidence, size, workspace
  M->>V: Plan pre-grasp and approach
  V-->>M: Collision-checked plan
  M->>V: Plan transfer and retreat
  V-->>M: Collision-checked plan
  alt plan only
    M-->>T: Ready; execution not requested
  else execution authorised
    M->>C: Execute trajectories
    C-->>M: Result
    M-->>T: Success or fault
  end
```

## Extension points

- Replace colour segmentation through the detector adapter without changing the message contract.
- Add grasp scoring before the motion action client.
- Replace the demonstration arm by regenerating the description and MoveIt configuration while preserving application interfaces.
- Connect a real controller only after joint names, limits, feedback rates, safety I/O, and recovery semantics are verified.

