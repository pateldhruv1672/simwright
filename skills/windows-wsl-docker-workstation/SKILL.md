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

## Isaac Sim on Windows <-> ROS 2 in WSL (verified Nov 4 2026, Isaac Sim 6.0.1, Humble, mirrored networking)
Run `simwright ros bridge setup` once, then `simwright isaac launch --ros`, `simwright ros bridge graph`, `simwright ros bridge check`. Each of these was needed:
- Isaac Sim must start with its bundled libraries for the same distro as WSL (`ROS_DISTRO=humble`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `exts/isaacsim.ros2.core/humble/lib` on PATH) and `--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge`. Fast DDS reads its XML profile once per process: profile changes need an Isaac Sim restart.
- Shared memory cannot cross Windows <-> WSL: UDP-only transport (`useBuiltinTransports=false`).
- Multicast from Windows does not reach WSL in mirrored mode; unicast to 127.0.0.1 does. Discovery through `initialPeersList` 127.0.0.1.
- IP fragments do not survive the hop either: anything over one packet (a 67-joint JointState, images) was lost ("A message was lost"). `maxMessageSize` 1400 makes DDS fragment instead; 4 MB socket buffers.
- WSL's Hyper-V firewall blocks inbound by default: rule `Simwright-DDS-WSL-In-d<domain>` (UDP 7400+250*domain .. +249, from this PC only).
- Same ROS_DOMAIN_ID both sides (setup reads it from WSL's ~/.bashrc).
- Use `ros2 topic list --no-daemon` and explicit types in checks; the daemon lags right after discovery.
- Commands go back through `/<robot>/joint_command` (sensor_msgs/JointState, radians) into an articulation controller.
