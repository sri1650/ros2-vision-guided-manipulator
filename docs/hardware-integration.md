# Hardware integration plan

## Required adaptations

1. Replace the example robot description with the manufacturer's calibrated URDF.
2. Map joint names, limits, transmissions, and controllers to the real driver.
3. Calibrate camera intrinsics, depth scale, and camera-to-base transform.
4. Measure network latency, joint-state rate, and timestamp synchronisation.
5. Build the real planning scene, payload model, tool centre point, and grasp offsets.
6. Add independent safety-rated guarding, emergency stop, and safe torque off.
7. Commission at reduced speed with supervised, incremental tests.

## Release gate

Execution must remain disabled until the responsible engineer signs off robot limits, transforms, collision geometry, controller direction, stopping behaviour, payload, and risk controls. Software topic names or a successful simulation are not evidence of machinery safety.

