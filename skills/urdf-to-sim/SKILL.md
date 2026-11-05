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

## Isaac Sim 6 URDF import: make it stable before you trust it (learned on Unitree G1 + BrainCo Revo2, Nov 2026)
- Isaac Sim 6 uses `isaacsim.asset.importer.urdf` 3.x (urdf-usd-converter): `URDFImporter(URDFImporterConfig(urdf_path=..., usd_path=<dir>, fix_base=True|False)).import_urdf()` returns the .usda path.
- Remove a `world` link + `floating` joint from the URDF first; set `fix_base` explicitly instead.
- The importer leaves every joint drive at stiffness 0 and damping 0: arms and fingers hang limp. Apply angular drives with target 0 (current pose). Working start values: legs/waist 400/20 (max force 300), shoulder/elbow 150/8 (100), wrist 40/2 (20), fingers 5/0.5 (5); armature 0.05 / 0.03 / 0.01 / 0.005.
- Dexterous hands ship 1 g fingertip links with ~1e-7 inertia on mimic joints. They go NaN within a second. Floor link mass at 0.02 kg and inertia at max(20x, 2e-6).
- Articulation: solver 32 position / 4 velocity iterations, self-collision off; physics scene at 120 Hz.
- Gate: `simwright isaac check 3` must report zero issues before the scene is called done.
