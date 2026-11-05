---
name: windows-wsl-docker-workstation
description: What runs where on a Windows physical-AI workstation (Isaac Sim/Lab native, ROS 2 in WSL or Docker, Gazebo in WSL), path mapping, GPU access and DDS networking.
---

# Windows + WSL + Docker map

| Piece | Where | Target |
|---|---|---|
| Isaac Sim 5.x/6.x, Isaac Lab | native Windows (RTX GPU) | `host` |
| ROS 2, colcon, Nav2, MoveIt | WSL 2 Ubuntu (or native Windows ROS 2 via Pixi with Isaac Sim 6) | `wsl` |
| Gazebo Harmonic | WSL 2 (WSLg shows the GUI) | `wsl` |
| Reproducible builds / CI images | Docker Desktop (WSL 2 backend) | `ros` (docker exec) |
| MuJoCo | anywhere `pip install mujoco` works | `host` or `wsl` |

- Paths: `C:\ws\robot` is `/mnt/c/ws/robot` in WSL. Building on `/mnt/c` is slow; prefer the workspace inside WSL
  (`\\wsl$\Ubuntu\home\you\ws`) and open it from Windows when needed.
- GPU in Docker: Docker Desktop + WSL 2 exposes CUDA (`docker run --gpus all ...`); verify with `simwright doctor --deep`.
- DDS between Isaac Sim (Windows) and ROS 2 (WSL): same `ROS_DOMAIN_ID`, same RMW (FastDDS default), and WSL mirrored networking
  (`[wsl2]\nnetworkingMode=mirrored` in `%USERPROFILE%\.wslconfig`, then `wsl --shutdown`). Verify with `ros2 topic list` in WSL
  while Isaac Sim publishes `/clock`. If discovery still fails, use a FastDDS discovery server.
- Targets live in `~/.simwright/config.json`; `simwright config targets.wsl.name Ubuntu-24.04` selects the distro.
