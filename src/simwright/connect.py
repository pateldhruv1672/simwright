"""`simwright connect <client>`: register the Simwright MCP server with an AI coding assistant.

claude    Claude Code   (runs `claude mcp add --scope user`, or prints it)
codex     Codex CLI / IDE / ChatGPT desktop app  (~/.codex/config.toml, tool timeout raised for long sim steps)
cursor    Cursor        (~/.cursor/mcp.json)
vscode    VS Code       (<workspace>/.vscode/mcp.json)
claude-desktop          (claude_desktop_config.json)
chatgpt   ChatGPT       (remote: starts the HTTP server + tunnel and prints the connector URL)
Existing entries for other servers are preserved; a backup (*.bak) is written before any edit.
"""
from __future__ import annotations
import json, os, pathlib, platform, re, shutil, subprocess, sys

CLIENTS = ("claude", "codex", "cursor", "vscode", "claude-desktop", "chatgpt")


def server_command(root: pathlib.Path, read_only: bool = False) -> tuple[str, list[str]]:
    exe = shutil.which("simwright-mcp")
    args = ["--root", str(root)] + (["--read-only"] if read_only else [])
    if exe:
        return str(pathlib.Path(exe).resolve()), args
    return sys.executable, ["-m", "simwright.mcp_server"] + args


def _backup(p: pathlib.Path):
    if p.exists():
        shutil.copyfile(p, p.with_name(p.name + ".bak"))


def _merge_json(p: pathlib.Path, key: str, entry: dict, name: str = "simwright") -> dict:
    data = {}
    if p.exists():
        txt = p.read_text(encoding="utf-8").strip()
        if txt:
            data = json.loads(txt)  # refuse to clobber a file we cannot parse
    _backup(p)
    data.setdefault(key, {})[name] = entry
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return {"ok": True, "file": str(p)}


def _toml_str(s: str) -> str:
    return "'" + s + "'" if "'" not in s else json.dumps(s)


def codex_block(cmd: str, args: list[str]) -> str:
    return ("[mcp_servers.simwright]\n"
            f"command = {_toml_str(cmd)}\n"
            f"args = [{', '.join(_toml_str(a) for a in args)}]\n"
            "startup_timeout_sec = 30\n"
            "tool_timeout_sec = 900\n")


def write_codex(p: pathlib.Path, cmd: str, args: list[str]) -> dict:
    old = p.read_text(encoding="utf-8") if p.exists() else ""
    out, skip = [], False
    for line in old.splitlines(keepends=True):
        h = re.match(r"^\s*\[([^\]]+)\]\s*$", line)
        if h:
            name = h.group(1).strip()
            skip = name == "mcp_servers.simwright" or name.startswith("mcp_servers.simwright.")
        if not skip:
            out.append(line)
    text = "".join(out).rstrip()
    text = (text + "\n\n" if text else "") + codex_block(cmd, args)
    _backup(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding="utf-8")
    return {"ok": True, "file": str(p)}


def claude_desktop_path() -> pathlib.Path:
    if os.name == "nt":
        return pathlib.Path(os.environ.get("APPDATA", pathlib.Path.home() / "AppData/Roaming")) / "Claude" / "claude_desktop_config.json"
    if platform.system() == "Darwin":
        return pathlib.Path.home() / "Library/Application Support/Claude/claude_desktop_config.json"
    return pathlib.Path.home() / ".config/Claude/claude_desktop_config.json"


def connect(client: str, root: pathlib.Path, read_only: bool = False, print_only: bool = False, home: pathlib.Path | None = None) -> dict:
    home = home or pathlib.Path.home()
    cmd, args = server_command(root, read_only)
    if client == "claude":
        line = ["claude", "mcp", "add", "--transport", "stdio", "--scope", "user", "simwright", "--", cmd, *args]
        if print_only or not shutil.which("claude"):
            return {"ok": True, "run_this": subprocess.list2cmdline(line), "note": "" if print_only else "Claude Code CLI not found on PATH"}
        subprocess.run(["claude", "mcp", "remove", "--scope", "user", "simwright"], capture_output=True, text=True)
        r = subprocess.run(line, capture_output=True, text=True)
        return {"ok": r.returncode == 0, "ran": subprocess.list2cmdline(line), "output": (r.stdout + r.stderr).strip()[-500:],
                "check": "claude mcp list"}
    if client == "codex":
        p = home / ".codex" / "config.toml"
        if print_only:
            return {"ok": True, "file": str(p), "add_this": codex_block(cmd, args)}
        return {**write_codex(p, cmd, args), "check": "codex mcp list  (or /mcp inside Codex)"}
    if client == "cursor":
        entry = {"command": cmd, "args": args}
        p = home / ".cursor" / "mcp.json"
        return {"ok": True, "file": str(p), "add_this": {"mcpServers": {"simwright": entry}}} if print_only else _merge_json(p, "mcpServers", entry)
    if client == "vscode":
        entry = {"type": "stdio", "command": cmd, "args": args}
        p = root / ".vscode" / "mcp.json"
        return {"ok": True, "file": str(p), "add_this": {"servers": {"simwright": entry}}} if print_only else _merge_json(p, "servers", entry)
    if client == "claude-desktop":
        entry = {"command": cmd, "args": args}
        p = claude_desktop_path()
        if print_only:
            return {"ok": True, "file": str(p), "add_this": {"mcpServers": {"simwright": entry}}}
        return {**_merge_json(p, "mcpServers", entry), "note": "restart Claude Desktop"}
    if client == "chatgpt":
        return {"ok": True, "run_this": "simwright-mcp --http --tunnel --root " + subprocess.list2cmdline([str(root)])
                + ("  --read-only" if read_only else ""),
                "then": "In ChatGPT (web): Settings > Apps > Advanced > Developer mode, then Create (newer UI: Customize > Plugins > Add > "
                        "Create MCP App). Paste the printed URL, Authentication: No authentication. ChatGPT asks before write actions."}
    raise ValueError(f"unknown client {client!r}; choose one of {', '.join(CLIENTS)}")



def connect_docs(client: str, root: pathlib.Path, print_only: bool = False, home: pathlib.Path | None = None, url: str | None = None) -> dict:
    """Register NVIDIA's Isaac Sim docs MCP server (HTTP) next to Simwright in an assistant."""
    from . import docs
    home = home or pathlib.Path.home()
    url = url or docs.url()
    name = "isaac-sim-mcp"
    if client == "claude":
        line = ["claude", "mcp", "add", "--transport", "http", "--scope", "user", name, url]
        if print_only or not shutil.which("claude"):
            return {"ok": True, "run_this": subprocess.list2cmdline(line)}
        subprocess.run(["claude", "mcp", "remove", "--scope", "user", name], capture_output=True, text=True)
        r = subprocess.run(line, capture_output=True, text=True)
        return {"ok": r.returncode == 0, "ran": subprocess.list2cmdline(line), "output": (r.stdout + r.stderr).strip()[-400:]}
    if client == "codex":
        p = home / ".codex" / "config.toml"
        block = f"[mcp_servers.{name}]\nurl = {_toml_str(url)}\n"
        if print_only:
            return {"ok": True, "file": str(p), "add_this": block}
        old = p.read_text(encoding="utf-8") if p.exists() else ""
        if f"[mcp_servers.{name}]" in old:
            return {"ok": True, "file": str(p), "note": "already registered"}
        _backup(p); p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(old.rstrip() + ("\n\n" if old.strip() else "") + block, encoding="utf-8")
        return {"ok": True, "file": str(p)}
    if client == "cursor":
        p = home / ".cursor" / "mcp.json"
        e = {"url": url}
        return {"ok": True, "file": str(p), "add_this": {"mcpServers": {name: e}}} if print_only else _merge_json(p, "mcpServers", e, name)
    if client == "vscode":
        p = root / ".vscode" / "mcp.json"
        e = {"type": "http", "url": url}
        return {"ok": True, "file": str(p), "add_this": {"servers": {name: e}}} if print_only else _merge_json(p, "servers", e, name)
    return {"ok": False, "note": f"{client}: NVIDIA's docs server is HTTP on localhost; Simwright already proxies it through its isaac_docs tool."}
