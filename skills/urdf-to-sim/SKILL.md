---
name: urdf-to-sim
description: Author or repair URDF/xacro so the robot behaves in Isaac Sim, Gazebo Harmonic and MuJoCo - inertia, collisions, joint limits, drives, sensors, conversion.
---

# URDF that survives every simulator

- Run `urdf_validate` first. Inertia must be positive definite and satisfy the triangle inequality; masses >= 0.01 kg.
  Use primitive formulas (box: m(y^2+z^2)/12 ...) at the collision geometry's center, not mesh guesses.
- Every link with collision needs `<inertial>`. Collision: primitives or convex hulls, never the visual mesh.
- Revolute/prismatic joints need limits with effort and velocity > 0, else Isaac drives and Gazebo controllers do nothing.
- One root link (`base_link`, optional `base_footprint` via fixed joint). Frames follow REP-103/105.
- **Isaac Sim**: import with the URDF importer (`isaacsim.asset.importer.urdf`); set drive type and stiffness/damping
  (position control: high stiffness; velocity control for wheels: stiffness 0, damping > 0). Merge fixed joints for speed.
- **Gazebo Harmonic**: `<gazebo>` blocks with `gz-sim-*` plugin filenames; `gz_ros2_control` for ros2_control; bridge topics with `ros_gz_bridge`.
- **MuJoCo**: convert (`mujoco` compiles URDF with `<mujoco><compiler meshdir=.../></mujoco>`), then `mujoco_check` for NaN qacc.
- After changes, update controllers YAML joint names and launch files in the same change, then re-run the sim check.
