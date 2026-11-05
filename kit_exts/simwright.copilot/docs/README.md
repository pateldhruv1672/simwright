# Simwright Copilot (Isaac Sim extension)

Adds a **Simwright** window to Isaac Sim: type a prompt, watch the steps, approve commands.
The agent itself runs in `simwright serve` (started in your robot workspace); this extension talks to it on 127.0.0.1.
It also exposes `simwright.copilot.helpers` (scene summary, physics check, capture, asset search, room builder)
to NVIDIA's `isaacsim.code_editor.python_server`, which Simwright, Mana and MCP clients use to drive the open stage.

Enable: `simwright isaac launch` does it for you, or add this folder's parent to Window > Extensions > Settings > Extension Search Paths and enable "Simwright Copilot".
