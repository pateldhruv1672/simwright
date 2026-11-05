---
name: isaaclab-policy-lifecycle
description: Train, monitor, evaluate, export, register and deploy reinforcement-learning policies with Isaac Lab (2.x and 3.x) using Simwright's lab tools.
---

# Policy lifecycle in Isaac Lab

1. **Pick the task**: `lab_tasks` (filter by robot, e.g. "Anymal", "Franka", "G1"). For custom tasks, confirm the gym id is
   registered (external projects must be pip-installed into Isaac Lab's Python: `isaaclab.bat -p -m pip install -e <ext>`).
2. **Smoke test first**: `lab_train(task, num_envs=64, max_iterations=10)`; check `lab_jobs` shows iterations advancing and no Traceback.
3. **Full run**: `lab_train(task, num_envs=4096 (fits 16 GB for locomotion; cameras need far fewer), max_iterations=...)`.
   Monitor `lab_jobs`: mean reward should trend up and mean episode length should stop collapsing. Stop early with `lab_stop` if reward is NaN.
4. **Evaluate**: `lab_play(task, checkpoint=<model_X.pt>, video=true)` and watch the clip. A rising reward curve is not proof of a usable gait.
5. **Export**: Isaac Lab 2.x `play.py` writes `exported/policy.pt` and `policy.onnx` next to the checkpoint.
   Isaac Lab 3.x: `lab_export(task, checkpoint)` runs the LEAPP exporter (ONNX/JIT + IO metadata + initial state) for manager-based RSL-RL tasks.
6. **Register**: `policy_register(name, task, checkpoint, notes, metrics={"mean_reward":..., "iterations":...})`.
7. **Deploy**: `policy_deploy(name, dest)` copies the exported bundle + env.yaml + manifest to a ROS 2 package, robot folder, or
   Isaac Sim project. For running the policy inside Isaac Sim, follow NVIDIA's policy-deployment skill reference
   (`isaac-sim-robot-navigation/references/policy-deployment.md`) when present.

## Version gotchas
- Isaac Lab 3.0 needs Isaac Sim 6.2 + Python 3.12 and uses XYZW quaternions (2.x used WXYZ). Convert hand-written rotations.
- Windows: use `isaaclab.bat`; logs land in `<IsaacLab>/logs/<library>/<experiment>/<timestamp>/`.
- Record the winning command line, seed and commit in SIMWRIGHT.md under Policies.
