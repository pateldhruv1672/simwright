# Simwright

AI copilot for robotics simulation engineering. Come with prompts; Simwright handles ROS 2, Gazebo Harmonic,
MuJoCo, Isaac Sim and Isaac Lab on a Windows + WSL + Docker workstation (Linux works too).

## Pieces

| Piece | What it does |
|---|---|
| `simwright` CLI / REPL | `/init`, `/skill add`, live ROS 2 inspection, colcon recovery, URDF checks, Isaac Sim control, Isaac Lab training, world composition. Every command is shown before it runs; ESC pauses into step mode. |
| Targets | Commands run on `host` (PowerShell), `wsl` (ROS sourced) or `ros` (Docker container). |
| Isaac Sim link | NVIDIA's `isaacsim.code_editor.python_server` (127.0.0.1:8226) + the `simwright.copilot` Kit extension (prompt panel, scene/physics/capture helpers). |
| Isaac Lab | background training jobs, reward monitoring, runs on disk, play/export (LEAPP on 3.x), policy registry, deploy bundles. |
| Skills | Agent Skills format (`<name>/SKILL.md`). Team skills in `~/.simwright/skills` or `<repo>/.skills`; NVIDIA's Isaac Sim skills from `$ISAAC_SIM_DIR/skills` and `npx skills add nvidia/skills`. |
| `simwright-mcp` | the same tools over MCP for Claude Code, Cursor, Codex. |
| Mana | through Mana Bridge, Mana runs Simwright on your PC from chat and keeps the project's spec, runs and policies in your workspace. |

## Quick start (Windows)

```powershell
& ([scriptblock]::Create((irm '<url>/install.ps1'))) -Package '<url>/simwright-0.1.0-py3-none-any.whl'
cd C:\ws\my_robot
simwright init
simwright doctor
simwright isaac launch
simwright
```

Project context: `SIMWRIGHT.md` (also reads `DRIFT.md`, `CLAUDE.md`, `AGENTS.md`) at the workspace root, read before every step.

## SimReady assets

Simwright searches [SimReady Central](https://huggingface.co/spaces/nvidia/simready-central), NVIDIA's index of simulation-ready OpenUSD assets on Hugging Face, and imports them at their pinned revision:

```powershell
simwright simready search "electric kettle" --max_size 0.5
simwright simready place <url> --on /World/Kitchen/countertop --x 2.4 --y 4.7
simwright simready list      # cached packages with source revision and license
```

Packages are cached under `~/.simwright/simready/` with their license and validation reports. Set `HF_TOKEN` for gated contributor datasets.
