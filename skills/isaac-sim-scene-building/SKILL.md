---
name: isaac-sim-scene-building
description: Build or modify an Isaac Sim scene (robot, props, sensors, lights, physics) through the live session and prove it works with physics checks and a rendered capture.
---

# Building an Isaac Sim scene that actually works

Use with a running Isaac Sim (`isaac_status`; start with `isaac_launch`). NVIDIA's own skills (isaac-sim-remote,
isaac-sim-validator, physics-simulation, usd-articulation ...) are discovered from `$ISAAC_SIM_DIR/skills`; prefer them for deep procedures.

## Procedure (one capability at a time, verify before adding the next)
1. **Orient**: `isaac_scene` (up axis, meters per unit, existing PhysicsScene, lights, articulations). Never assume an empty stage.
2. **Ground + physics**: ensure exactly one PhysicsScene and a ground with a collider. Units: Isaac assets are meters, Z-up.
3. **Robot**: reference the robot USD (or import URDF with `isaacsim.asset.importer.urdf`) above the floor. Then gate:
   `isaac_physics_check(seconds=3)` must report no NaN, no body through the floor, no explosion.
4. **Props / world**: use `world_compose` for rooms (real assets, aisles, doorways) or `isaac_asset_search` for single assets.
   Re-run the physics check after adding props (interpenetration at spawn is the #1 cause of explosions).
5. **Sensors**: attach cameras/lidars, then confirm one frame of non-zero data.
6. **Lighting**: DomeLight >= 100 or DistantLight >= 500, else captures are black. `RayTracedLighting` while iterating.
7. **Verify visually**: `isaac_capture` (frame > ~150 KB means not black) and `isaac_record` for motion.

## Code conventions inside Kit
- Isaac Sim 5/6: `isaacsim.*` namespaces (not `omni.isaac.*`); Isaac Sim 6 prefers the Core Experimental API.
- Keep each `isaac_exec` snippet small (< 200 lines); write bigger scripts to files in the workspace and exec them.
- Use top-level `await omni.kit.app.get_app().next_update_async()` to let the stage settle after edits.

## Done means
Physics check passes, capture is lit and framed on the robot, and the scene is saved (`omni.usd.get_context().save_as_stage(path)`)
to a path recorded in SIMWRIGHT.md.

## Captures
- `simwright isaac capture <abs path>` waits until the PNG is fully written; when the terminal sits in an unwritable folder (C:\Windows\System32) captures go to ~/.simwright/captures.
- Look at the frame after every build: placement bugs (overhangs, giant props, wrong facing) show up there first.

## Saving and verifying (learned Nov 2026)
- A stage edit is not done until the saved file passes: save, reopen the file, run `isaac_scene` and `isaac_physics_check`. A run once reported "real xArm7 robots, gravity fixed, saved", and the reopened stage had empty xArm containers and a broken G1 reference.
- Relative references resolve from the layer that authors them: a scene in `~/robotics/scenes/` referencing `../assets/usd/x.usda` means `~/robotics/assets/usd/x.usda`. Check `scene health.broken_references` after every save; prefer the exact path the importer returned.
- UsdPhysics gravity direction (0,0,0) means "negative up axis" and a negative magnitude (even -inf) means "Earth gravity". These are valid defaults; do not call them corrupt. Set explicit values only if the user wants them.
- `isaac_scene` → `health` reports robot-named prims with no articulation and no joints: a placeholder or a failed load, not a robot.
