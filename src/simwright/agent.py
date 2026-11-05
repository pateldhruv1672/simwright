"""The local agent loop: any OpenAI-compatible model + the Simwright toolbox, with approvals and ESC pause.

The same tools are reachable from Mana (through Mana Bridge) and from MCP clients; this loop is for
running Simwright standalone or inside CI.
"""
from __future__ import annotations
import base64, json, time, mimetypes, os, pathlib, platform, sys
from . import config, context, skills
from .tools import Toolbox

SYSTEM = """You are Simwright, an AI copilot for robotics simulation engineering (ROS 2, Gazebo Harmonic, MuJoCo,
Isaac Sim, Isaac Lab, URDF, MJCF, SDF, USD). You work on the user's machine through tools; every command
is shown to the user before it runs.

Rules
- Ground every step in the workspace: list/search/read before you edit. Never invent file paths, topic names or frames;
  use the project context, the session log and live inspection (ros2_graph, isaac_scene).
- Make coordinated changes: when a joint, link, topic or frame changes, update URDF/xacro, controllers YAML,
  launch files, bridges (ros_gz / Isaac ROS 2 bridge) and docs together.
- Never report a scene change as done from your own edit: after saving a stage, reopen the saved file and run isaac_scene (its `health` lists broken references and robots without articulations) and isaac_physics_check; report only what those show.
- UsdPhysics gravity direction (0,0,0) and a negative magnitude are schema defaults (Earth gravity along the negative up axis), not corruption.
- When the user says 'fix only with my approval' or uses step mode, propose first; isaac_exec always needs a one-sentence `why`.
- Close the loop: after building or changing a sim, RUN it and read the result back (colcon_build, mujoco_check,
  gazebo_check, isaac_physics_check, isaac_capture/isaac_record). A clean exit code is not proof the scene is right.
- Follow skills: when a skill below matches the task, read it with skill_read and follow its procedure and acceptance checks.
- Isaac Sim 5/6 uses the isaacsim.* namespace (not omni.isaac.*). Isaac Lab 3 uses XYZW quaternions; 2.x uses WXYZ.
- Windows workstation: Isaac Sim / Isaac Lab run natively on Windows (target 'host'); ROS 2 and colcon run in the
  ROS target (WSL or Docker). Paths like C:\\ws map to /mnt/c/ws inside WSL.
- Keep answers short: what you did, what you verified, what is left.
"""


def build_system(root: pathlib.Path, toolbox: Toolbox, prompt: str) -> str:
    cfg = config.load()
    name, ctx = context.load_context(root)
    parts = [SYSTEM, f"\nMachine: {platform.system()} {platform.release()}; workspace: {root}",
             "Targets: " + ", ".join(f"{k}={v.get('kind')}{(':' + v['name']) if v.get('name') else ''}" for k, v in cfg["targets"].items()),
             f"ROS target: {cfg.get('ros_target')}; build target: {cfg.get('build_target')}"]
    parts.append(f"\n## Project context ({name})\n{ctx[:20000]}" if ctx else
                 "\n## Project context\nNone yet. Suggest `/init` to scan the workspace and write SIMWRIGHT.md.")
    mem = toolbox.session.summary()
    if mem:
        parts.append("\n## Session memory\n" + mem)
    all_sk = skills.discover(root)
    if all_sk:
        parts.append("\n## Skills (read with skill_read)\n" + "\n".join(f"- {s.name} [{s.scope}]: {s.description[:160]}" for s in all_sk[:120]))
        rel = skills.relevant(all_sk, prompt, k=2)
        for s in rel:
            parts.append(f"\n## Skill loaded for this request: {s.name}\n{s.body()[:12000]}")
    return "\n".join(parts)


def model_endpoint(cfg: dict) -> tuple[str, str]:
    """(api key, base url). Your own key wins; otherwise the Mana relay written by Mana to ~/.simwright/mana.json."""
    key = os.environ.get("SIMWRIGHT_API_KEY") or os.environ.get("OPENAI_API_KEY")
    if key:
        return key, cfg.get("base_url") or os.environ.get("SIMWRIGHT_BASE_URL") or ""
    m = mana_relay()
    if m and not m.get("expired"):
        return m["token"], m["base_url"]
    return "", ""


def mana_relay() -> dict | None:
    p = config.HOME / "mana.json"
    try:
        m = json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return None
    if not m.get("token") or not m.get("base_url"):
        return None
    m["expired"] = bool(m.get("expires") and time.time() > m["expires"])
    return m


class Agent:
    def __init__(self, root: pathlib.Path, toolbox: Toolbox, emit=None, model: str | None = None):
        from openai import OpenAI
        cfg = config.load()
        key, base = model_endpoint(cfg)
        if not key:
            raise RuntimeError("No model connected. In Mana, say \"connect Simwright to Mana models\" (no API key needed), "
                               "or set SIMWRIGHT_API_KEY (and SIMWRIGHT_BASE_URL for non-OpenAI endpoints).")
        self.client = OpenAI(api_key=key, base_url=base or None)
        self.model = model or cfg.get("model")
        self.root, self.tb = root, toolbox
        self.emit = emit or self._print
        self.messages: list[dict] = []
        self.images: list[str] = []

    @staticmethod
    def _print(ev: dict):
        if ev["type"] == "text":
            print("\n" + ev["text"], flush=True)
        elif ev["type"] == "tool":
            print(f"\n\u2022 {ev['name']}({ev['args_brief']})", flush=True)
        elif ev["type"] == "tool_result" and not ev.get("ok", True):
            print(f"  \u2717 {ev.get('error', '')[:300]}", flush=True)

    def attach_image(self, path: str):
        p = pathlib.Path(path).expanduser()
        mime = mimetypes.guess_type(p.name)[0] or "image/png"
        self.images.append(f"data:{mime};base64," + base64.b64encode(p.read_bytes()).decode())

    def turn(self, prompt: str, max_steps: int = 40) -> str:
        system = build_system(self.root, self.tb, prompt)
        if not self.messages:
            self.messages.append({"role": "system", "content": system})
        else:
            self.messages[0] = {"role": "system", "content": system}
        content = [{"type": "text", "text": prompt}] + [{"type": "image_url", "image_url": {"url": u}} for u in self.images]
        self.images = []
        self.messages.append({"role": "user", "content": content if len(content) > 1 else prompt})
        keys = self.tb.runner.approver.keys
        final = ""
        with keys.listening():
            for _ in range(max_steps):
                if self._cancelled():
                    break
                resp = self.client.chat.completions.create(model=self.model, messages=self.messages, tools=self.tb.schemas())
                msg = resp.choices[0].message
                entry = {"role": "assistant", "content": msg.content or ""}
                if msg.tool_calls:
                    entry["tool_calls"] = [{"id": c.id, "type": "function", "function": {"name": c.function.name, "arguments": c.function.arguments}} for c in msg.tool_calls]
                self.messages.append(entry)
                if msg.content:
                    final = msg.content
                    self.emit({"type": "text", "text": msg.content})
                if not msg.tool_calls:
                    break
                for c in msg.tool_calls:
                    if self._cancelled():  # keep the transcript valid: answer every pending call
                        self.messages.append({"role": "tool", "tool_call_id": c.id, "content": json.dumps({"ok": False, "skipped": True, "note": "stopped by the user"})})
                        continue
                    try:
                        args = json.loads(c.function.arguments or "{}")
                    except json.JSONDecodeError:
                        args = {}
                    brief = ", ".join(f"{k}={str(v)[:60]!r}" for k, v in args.items() if k not in ("content",))
                    self.emit({"type": "tool", "name": c.function.name, "args_brief": brief})
                    res = self.tb.call(c.function.name, args)
                    self.emit({"type": "tool_result", "name": c.function.name, "ok": res.get("ok", True), "error": res.get("error", "")})
                    text = json.dumps(res, default=str)
                    if len(text) > 14000:
                        text = text[:14000] + '..."(truncated)"'
                    self.messages.append({"role": "tool", "tool_call_id": c.id, "content": text})
        self._trim()
        return final

    def _cancelled(self) -> bool:
        ev = getattr(self, "cancel", None)
        if ev is not None and ev.is_set():
            if not getattr(self, "_told_stop", False):
                self._told_stop = True
                self.emit({"type": "text", "text": "Stopped at your request."})
            return True
        self._told_stop = False
        return False

    def _trim(self, keep: int = 60):
        if len(self.messages) > keep:
            head, tail = self.messages[:1], self.messages[-keep:]
            while tail and tail[0]["role"] == "tool":
                tail = tail[1:]
            self.messages = head + tail
