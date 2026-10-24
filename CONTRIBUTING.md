# Contributing

Use ROS 2 Jazzy on Ubuntu 24.04. Keep algorithmic logic independent from ROS nodes where practical, add tests for behavioural changes, and run:

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Submit focused changes with clear validation evidence. Do not describe simulation-only behaviour as hardware validated.

