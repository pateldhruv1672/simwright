# Simwright

**An AI copilot for robot simulation.** Describe what you want; Simwright builds and runs it in Isaac Sim, Isaac Lab, ROS 2, Gazebo and MuJoCo on your own workstation, checks the result (physics, renders, reward curves, the live ROS graph) and fixes what broke.

Use it from the assistant you already have: **Claude Code, Codex, Cursor, VS Code, Claude Desktop or ChatGPT** (over MCP), from its own terminal copilot, or from a panel inside Isaac Sim.

```text
you   > Build a kitchen with a Unitree G1 standing at the table, utensils on it, and check the physics
agent > imports the G1 URDF, tunes drives, places SimReady assets on the tabletop, runs 3 s of physics,
        captures a frame, reports "83 bodies stable, nothing off the table"
```

- **Isaac Sim:** builds scenes through NVIDIA's own `python_server`, physics checks (NaN, falling, explosions), captures and recordings, URDF import with a stability gate.
- **Real assets:** searches [SimReady Central](https://huggingface.co/spaces/nvidia/simready-central) and places simulation-ready OpenUSD at metric scale, pinned to an exact revision, with its license recorded.
- **Isaac Lab:** finds your installs (2.x and 3.x, conda envs included), trains in the background, tracks reward, plays, exports, registers and deploys policies.
- **ROS 2:** live graph with QoS and type mismatches, publisher → controller → joint tracing, `colcon` build repair across `CMakeLists.txt`, `package.xml` and `setup.py`.
- **Worlds:** rooms laid out the way a person would: chairs at tables, screens facing people, clear doorways, robot on measured free floor.
- **Safety:** every command is shown before it runs, destructive shell shapes are refused over MCP, write tools are labelled so your assistant asks first.

> Status: early (0.x). Developed and verified on Windows 11 + RTX 5080 (16 GB) with Isaac Sim 6.0.1, Isaac Lab 3.0.0 and 2.3.2, WSL Ubuntu 22.04 + ROS 2 Humble, Docker 29.5. Linux should work and is less tested. Issues and PRs welcome.

---

## Contents

1. [Requirements](#requirements)
2. [Install](#install)
3. [Quick start](#quick-start)
4. [Connect your AI assistant](#connect-your-ai-assistant) · [Claude Code](#claude-code) · [Codex](#codex) · [Cursor / VS Code / Claude Desktop](#cursor-vs-code-claude-desktop) · [ChatGPT](#chatgpt) · [Built-in copilot](#built-in-terminal-copilot) · [Isaac Sim panel](#isaac-sim-panel)
5. [What to ask](#what-to-ask)
6. [Safety model](#safety-model)
7. [Feature guide](#feature-guide)
8. [Configuration](#configuration)
9. [Troubleshooting](#troubleshooting)
10. [Development](#development)
11. [License](#license)

---

## Requirements

| | Needed for | Notes |
|---|---|---|
| Windows 11 or Ubuntu 22.04/24.04 | everything | macOS works for ROS/URDF/MuJoCo tools only (no Isaac) |
| NVIDIA RTX GPU | Isaac Sim / Isaac Lab | 16 GB+ VRAM for Isaac Lab; Windows driver 580.88 or newer |
| [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/) 5.1+ | scenes, physics, panel | standalone zip or `pip install isaacsim` |
| [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) 2.3+ | training | 3.x pairs with Isaac Sim 6, 2.x with Isaac Sim 5 |
| WSL 2 (Windows) | ROS 2, colcon, Gazebo | set `networkingMode=mirrored` in `.wslconfig` so DDS reaches Windows |
| Docker Desktop | optional reproducible builds | WSL 2 backend |
| Python 3.12 | Simwright | installed for you by [uv](https://docs.astral.sh/uv/) |
| Git | installing from GitHub | `winget install Git.Git` |

You only need the pieces you use: Simwright reports what it found and skips the rest.

## Install

**Windows (PowerShell, no admin):**

```powershell
irm https://raw.githubusercontent.com/pateldhruv1672/simwright/main/install/install.ps1 | iex
```

This installs uv if missing, installs Simwright from this repository as an isolated tool (`simwright`, `simwright-mcp` on your PATH) and runs `simwright doctor`.

**Linux / macOS, or Windows by hand:**

```bash
uv tool install --python 3.12 "simwright @ git+https://github.com/pateldhruv1672/simwright"
# upgrade later:  uv tool install --force --python 3.12 "simwright @ git+https://github.com/pateldhruv1672/simwright"
```

Upgrading on Windows: stop any running `simwright serve` first (the installer does this for you); Windows cannot replace `simwright.exe` while it runs.

## Quick start

```powershell
cd C:\ws\my_robot            # your robot workspace (any folder works)
simwright init               # scans packages, URDF/xacro, launch files, worlds, USD, Isaac Lab tasks; writes SIMWRIGHT.md
simwright doctor             # GPU, WSL, Docker, Isaac Sim, Isaac Lab, ROS 2, model connection
simwright isaac launch       # Isaac Sim with NVIDIA's python_server and the Simwright panel
simwright isaac check 3      # 3 s of physics: anything falling, exploding or NaN?
simwright connect claude     # or codex | cursor | vscode | claude-desktop | chatgpt
```

`doctor` finds Isaac Sim and Isaac Lab installs on its own (including Isaac Lab checkouts installed into conda environments). If it picks the wrong one: `simwright config isaac_sim_dir <path>` / `simwright config isaac_lab_dir <path>`.

## Connect your AI assistant

Simwright exposes 40 tools over the [Model Context Protocol](https://modelcontextprotocol.io). Your assistant provides the model; Simwright provides the hands. `simwright connect <client>` writes the client's config for you (existing entries are kept, a `.bak` backup is written first). Add `--print` to see the config without writing it, or `--read-only` to expose only the 19 tools that never change anything.

| Client | Command | Transport |
|---|---|---|
| Claude Code | `simwright connect claude` | stdio |
| Codex (CLI, IDE extension, desktop app) | `simwright connect codex` | stdio |
| Cursor | `simwright connect cursor` | stdio |
| VS Code (Copilot agent mode) | `simwright connect vscode` | stdio |
| Claude Desktop | `simwright connect claude-desktop` | stdio |
| ChatGPT | `simwright mcp --http --tunnel` | Streamable HTTP |

Run `connect` from inside your robot workspace (or pass `--root`): the server operates on that folder.

### Claude Code

```powershell
cd C:\ws\my_robot
simwright connect claude
claude mcp list              # simwright: connected
```

Manual equivalent: `claude mcp add --transport stdio --scope user simwright -- simwright-mcp --root C:\ws\my_robot`. Claude Code asks before each tool call according to your permission settings.

### Codex

```powershell
simwright connect codex
codex mcp list               # or /mcp inside Codex
```

This adds to `~/.codex/config.toml`:

```toml
[mcp_servers.simwright]
command = 'C:\Users\you\.local\bin\simwright-mcp.exe'
args = ['--root', 'C:\ws\my_robot']
startup_timeout_sec = 30
tool_timeout_sec = 900       # Isaac steps can take minutes; Codex defaults to 60 s
```

### Cursor, VS Code, Claude Desktop

`simwright connect cursor` writes `~/.cursor/mcp.json`, `simwright connect vscode` writes `<workspace>/.vscode/mcp.json`, `simwright connect claude-desktop` writes `claude_desktop_config.json` (restart Claude Desktop afterwards). The entry is always:

```json
{ "command": "simwright-mcp", "args": ["--root", "C:\\ws\\my_robot"] }
```

### ChatGPT

ChatGPT runs in OpenAI's cloud, so it needs a public HTTPS URL; Simwright serves MCP over Streamable HTTP behind a secret token and can open a free [Cloudflare quick tunnel](https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/do-more-with-tunnels/trycloudflare/) for you.

```powershell
winget install Cloudflare.cloudflared      # once
cd C:\ws\my_robot
simwright mcp --http --tunnel              # add --read-only to start safe
```

It prints something like:

```text
MCP server URL:  https://quiet-river-1234.trycloudflare.com/t/Xk2…token…/mcp
Authentication:  No authentication   (the token is in the URL; keep it private)
```

In ChatGPT on the web: **Settings → Apps → Advanced settings → Developer mode**, then **Create** (newer accounts: **Customize → Plugins → Add → Create MCP App**). Paste the URL, set **Authentication: No authentication**, accept the warning, create. Availability of developer mode depends on your ChatGPT plan and, for Business/Enterprise, your workspace admin.

- ChatGPT asks for confirmation before tools marked as writes (Simwright labels every tool).
- The token stays the same across restarts (`~/.simwright/mcp_http.json`); quick-tunnel hostnames change each start, so update the URL in ChatGPT, or use a named Cloudflare tunnel, ngrok or Tailscale Funnel pointing at `http://127.0.0.1:8765`.
- Rotate the token any time with `simwright mcp --http --new-token`. Anyone holding the full URL can use the tools that are exposed; prefer `--read-only` unless you need writes.
- Other remote MCP clients can send the token as `Authorization: Bearer <token>` to `http://127.0.0.1:8765/mcp` instead of putting it in the path.

### Built-in terminal copilot

No assistant? Simwright has its own agent loop for any OpenAI-compatible model:

```powershell
setx SIMWRIGHT_API_KEY sk-...                    # OpenAI, or any compatible provider
setx SIMWRIGHT_BASE_URL https://api.example/v1   # only for non-OpenAI endpoints
simwright config model gpt-5.4                   # any model name your endpoint serves
simwright                                        # interactive; type /help
simwright chat "why does my robot not move on /cmd_vel?"
```

`/mode step` confirms every command, `/mode auto` runs uninterrupted (press ESC to pause into step mode), `/image room.jpg` attaches a photo to match a layout to.

### Isaac Sim panel

`simwright isaac launch` starts Isaac Sim with a **Simwright** window: type a prompt, approve or skip each command. The panel talks to a local server you run in your workspace:

```powershell
simwright serve          # uses the same model settings as the built-in copilot
```

Launching Isaac Sim yourself works too: enable `isaacsim.code_editor.python_server` in the Extensions window and every Simwright tool works; the panel additionally needs the folder printed by `simwright isaac ext-path` added under Extensions → Settings → Extension Search Paths.

<details><summary>Using it from Mana</summary>

Simwright started in <a href="https://mana.space">Mana</a>, which can drive it from chat through Mana Bridge (a paired terminal on your PC), keep a console with status, training runs and team skills, and provide models without an API key (it writes <code>~/.simwright/mana.json</code>). None of this is required: everything above works without Mana.
</details>

## What to ask

- “Compose a 12 × 9 m office with six desks, a meeting table and a Nova Carter, build it in Isaac Sim and check the physics.”
- “Import `urdf/my_arm.urdf` into Isaac Sim, make it hold its pose, and show me a frame.”
- “Find a SimReady kettle under 30 cm and put it on the counter.”
- “Train `Isaac-Velocity-Flat-Anymal-D-v0` for 1500 iterations and tell me when the reward plateaus.”
- “My robot doesn’t move when I publish `/cmd_vel`. Find out why.”
- “Fix my colcon build.”
- “Export the best checkpoint, register it as `anymal-flat-v1` and deploy it to `C:\deploy`.”

## Safety model

| Layer | What it does |
|---|---|
| Show before run | Every shell command is printed with its target (host, WSL, Docker) before it runs. |
| Approval modes | Built-in copilot: `auto` (default; ESC pauses into step), `step` (confirm anything that is not read-only), `ask` (confirm every command). `simwright config approval step`. Over MCP, your assistant's own approval settings decide. |
| Dangerous shapes | Recursive deletes, disk formatting (`format`, `mkfs`, `dd`), `git reset --hard` / `git clean -f` / force-push, Docker removals and prunes, `shutdown`, `wsl --unregister`: refused outright over MCP and in CI; in the terminal you must type `yes`. |
| MCP annotations | Each tool declares read-only / destructive / open-world, so Claude Code, Codex and ChatGPT can ask before writes. |
| Read-only server | `simwright-mcp --read-only` exposes only tools that never change anything. |
| Remote access | HTTP mode refuses requests without the token; `/healthz` is the only open path. |

## Feature guide

Every subcommand prints JSON, so scripts and agents can use the CLI directly.

| Area | Commands |
|---|---|
| Workspace | `simwright init` · `simwright status --json` · `simwright doctor [--deep]` |
| Isaac Sim | `simwright isaac launch [--headless]` · `ext-path` · `status` · `scene` · `check <seconds>` · `capture [path]` · `record` · `assets <keywords>` · `exec <script.py>` |
| SimReady assets | `simwright simready search "office chair" --max_size 1.2` · `fetch <url>` · `place <url> --on /World/table --x 1 --y 2 --yaw 90` · `list` |
| Isaac Lab | `simwright lab installs` · `tasks <filter>` · `train <task> --num_envs 4096 --max_iterations 1500` · `jobs` · `stop <id>` · `runs` · `play <task>` · `export <task> <checkpoint>` · `register <name> <task> <checkpoint>` · `policies` · `deploy <name> <dest>` |
| ROS 2 | `simwright ros graph` · `simwright ros trace /cmd_vel` |
| Build | `simwright build [packages]` (diagnoses failures and proposes coordinated fixes) |
| Models | `simwright urdf <file>` · `simwright mujoco <scene.xml>` · `simwright gazebo <world.sdf>` |
| Worlds | `simwright world spec.json [--no-apply]` |
| Skills | `simwright skill add ./team-skills/ekf-tuning.md [--repo]` · `skill list` · `skill nvidia` |
| MCP | `simwright mcp [--http --tunnel --read-only --port 8765 --new-token]` · `simwright connect <client> [--print --read-only]` |

**Project context.** `SIMWRIGHT.md` at the workspace root (also `DRIFT.md`, `CLAUDE.md`, `AGENTS.md`) is read before every step: packages, robots, frames, topics, conventions. `simwright init` writes a starter; edit it freely.

**Skills.** Procedures in the open [Agent Skills](https://agentskills.io) format (`<name>/SKILL.md`). Precedence: repo (`<repo>/.skills`) > user (`~/.simwright/skills`, `~/.claude/skills`) > NVIDIA (`$ISAAC_SIM_DIR/skills`, or `npx skills add nvidia/skills`) > the seven built-ins (scene building, Isaac Lab policy lifecycle, ROS 2 graph debugging, colcon recovery, URDF to sim, world composition, Windows/WSL/Docker workstation).

**Execution targets.** Commands run on `host` (PowerShell or bash), `wsl` (a WSL distro with ROS sourced) or `ros` (a Docker container via `docker exec`). Windows paths are mapped to `/mnt/c/...` automatically.

**SimReady assets.** Search and package manifests come from the public SimReady Central API; files download straight from the contributor's Hugging Face dataset at the indexed revision into `~/.simwright/simready/<org>__<dataset>/<revision>/`. Set `HF_TOKEN` for gated datasets. NVIDIA's own assets are CC-BY-4.0: attribute them when you publish.

## Project memory

Each workspace keeps its own memory in `.simwright/`, shared by the Isaac Sim panel, the terminal copilot and MCP clients:

| File | What |
|---|---|
| `notes.md` | dated notes: decisions, saved stage paths, conventions, next steps. Edit it by hand too. |
| `conversation.json` | the current panel / terminal conversation, restored after restarts |
| `history/` | archived conversations; **New chat** (panel) or `/new` (terminal) summarizes the old one into `notes.md` first |
| `session.jsonl` | every file written, command run and scene change |
| `panel_events.jsonl` | what the panel shows when it opens |

MCP clients get the same memory through the `project_memory` and `project_note` tools.

## Placement and mounting

Objects are set down the way a person would: `isaac_place` / `simready_place` lower the object onto the **real surface under its footprint** (a ray cast through the support's triangles), so a lamp or a back panel on a table can't lift it, a raised floor is respected, and placement inside moved or scaled groups lands at the requested world position. `isaac_mount` attaches cameras, sensors and tools to a robot link so they move with it, touching the chosen face of the link. `isaac_placement_audit` reports anything floating or sunk, with the exact gap.

```powershell
simwright isaac exec check.py      # or ask: "audit placement under /World/Warehouse"
```

## Configuration

`simwright config` prints everything; `simwright config <key> <value>` sets one key (dotted keys for nested values).

| Key | Default | Meaning |
|---|---|---|
| `isaac_sim_dir` | auto | Isaac Sim install (folder with `isaac-sim.bat` / `isaac-sim.sh`) |
| `isaac_lab_dir` / `isaac_lab_python` | auto | Isaac Lab checkout and the Python that installed it |
| `targets.wsl.name` | default distro | WSL distro for ROS 2 / colcon |
| `targets.ros.name` | `ros2_dev` | Docker container for the `ros` target |
| `ros_target`, `build_target` | `wsl` on Windows | where ROS commands and builds run |
| `python_server.port` | `8226` | Isaac Sim python_server port |
| `approval` | `auto` | `auto`, `step` or `ask` |
| `model`, `base_url` | `gpt-5.4`, OpenAI | built-in copilot model (key from `SIMWRIGHT_API_KEY` or `OPENAI_API_KEY`) |
| `lab.rl_library` | `rsl_rl` | Isaac Lab RL library for train/play/export |

Files live in `~/.simwright/`: `config.json`, `skills/`, `simready/`, `jobs.json`, `policies.json`, `captures/`, `mcp_http.json` (HTTP token). Per workspace: `SIMWRIGHT.md` and `.simwright/session.jsonl` (what was built, so later steps don't hallucinate paths).

## Troubleshooting

| Symptom | Fix |
|---|---|
| `failed to copy ... simwright.exe ... used by another process` | Stop `simwright serve` (or close the Isaac Sim panel's server window), then rerun the installer. |
| `Isaac Sim session: not running` | `simwright isaac launch`, or enable `isaacsim.code_editor.python_server` in your own Isaac Sim. |
| Isaac Lab picked the wrong version | `simwright lab installs` lists what was found; `simwright config isaac_lab_dir <path>`. Lab 3.x needs Isaac Sim 6, 2.x needs 5.x. |
| Robot arms sag or fingers explode after a URDF import | Drives import with zero stiffness and tiny finger masses go NaN; ask for “make it hold its pose” (the `urdf-to-sim` skill has working gains), then `simwright isaac check 3`. |
| ChatGPT: “does not implement OAuth” | Set Authentication to **No authentication**; the token is in the URL. |
| ChatGPT can't reach the server | The quick-tunnel URL changed after a restart; paste the new one. Check `https://<tunnel>/healthz`. |
| Codex tool calls time out | Use `simwright connect codex` (sets `tool_timeout_sec = 900`) or raise it in `config.toml`. |
| ROS 2 in WSL can't see Isaac Sim topics | Add `networkingMode=mirrored` to `%USERPROFILE%\.wslconfig`, `wsl --shutdown`, and use the same `ROS_DOMAIN_ID`. |
| Built-in copilot: “No model connected” | Set `SIMWRIGHT_API_KEY` (and `SIMWRIGHT_BASE_URL` for non-OpenAI endpoints), or use an assistant over MCP instead. |

## Development

```bash
git clone https://github.com/pateldhruv1672/simwright && cd simwright
uv sync
uv run pytest -q
uv build --wheel
```

| Path | What |
|---|---|
| `src/simwright/tools.py` | the toolbox: one set of tools shared by MCP, the CLI agent and the panel |
| `src/simwright/mcp_server.py`, `connect.py` | MCP server (stdio + HTTP) and client config writers |
| `src/simwright/isaac.py`, `kit_helpers.py` | Isaac Sim link (python_server client) and helpers that run inside Kit |
| `src/simwright/lab.py`, `simready.py`, `world.py`, `ros2.py`, `colcon.py`, `urdf.py` | feature modules |
| `kit_exts/simwright.copilot` | the Isaac Sim extension (panel) |
| `skills/` | built-in skills |
| `install/install.ps1` | Windows installer |
| `tools/` | maintainer automation (test-gated publishing used by the author's Mana workspace) |

Contributions are welcome: open an issue describing the robot, simulator versions and what you asked for, and include `simwright doctor` output.

## License

Apache-2.0 (see [LICENSE](LICENSE)). Isaac Sim, Isaac Lab and NVIDIA skills are covered by NVIDIA's own licenses; SimReady assets carry their own licenses, recorded per package by `simwright simready list`.
