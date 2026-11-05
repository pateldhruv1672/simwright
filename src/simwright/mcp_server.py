r"""Simwright MCP server: the Simwright toolbox for Claude Code, Codex, Cursor, VS Code, Claude Desktop and ChatGPT.

Local clients (stdio):   simwright-mcp --root C:\path\to\ws          (or: simwright connect claude|codex|cursor|vscode)
ChatGPT (remote HTTP):   simwright-mcp --http --tunnel --root C:\path\to\ws
                         prints  https://<tunnel>/t/<token>/mcp  to paste into ChatGPT (Authentication: none)

Every tool carries MCP annotations (read-only / destructive) so clients can ask before write actions.
Dangerous shell shapes are always refused. --read-only exposes only tools that never change anything.
"""
from __future__ import annotations
import argparse, inspect, json, os, pathlib, re, secrets, shutil, subprocess, sys, threading, time

READ_ONLY = {"read_file", "list_dir", "search_files", "ros2_graph", "ros2_trace", "urdf_validate", "colcon_diagnose",
             "isaac_status", "isaac_scene", "isaac_asset_search", "simready_search", "simready_catalog", "lab_tasks",
             "lab_jobs", "lab_runs", "policy_list", "skill_list", "skill_read", "doctor", "project_memory", "isaac_placement_audit"}  # isaac_physics_repair previews by default but can write
DESTRUCTIVE = {"run_command", "write_file", "edit_file", "isaac_exec", "lab_stop", "policy_deploy"}
OPEN_WORLD = {"run_command", "simready_search", "simready_fetch", "simready_place"}

INSTRUCTIONS = """Simwright drives robot simulation on the user's machine: Isaac Sim, Isaac Lab, ROS 2, Gazebo, MuJoCo.
Before acting, read project_memory (notes, earlier work) and the project context (read_file SIMWRIGHT.md), and the matching skill.
Record decisions and saved stage paths with project_note.
Place objects with isaac_place / simready_place (surface ray cast), mount on links with isaac_mount, verify with isaac_placement_audit.
Never invent paths, topics, frames or prim paths: list, search or inspect first (ros2_graph, isaac_scene).
Close the loop: after changing a sim run the check (isaac_physics_check, isaac_capture, lab_jobs, ros2_trace) and read it.
Real assets: simready_search then simready_place (metric scale, pinned revision, license recorded).
Isaac Lab training runs as a background job: start with lab_train, then poll lab_jobs."""

HTTP_STATE = "mcp_http.json"


def _annotations(name: str):
    from mcp.types import ToolAnnotations
    vals = {"readOnlyHint": name in READ_ONLY, "destructiveHint": name in DESTRUCTIVE,
            "idempotentHint": name in READ_ONLY, "openWorldHint": name in OPEN_WORLD}
    try:
        return ToolAnnotations.model_validate(vals)
    except Exception:  # field names differ across mcp versions
        return ToolAnnotations(**{re.sub(r"([A-Z])", lambda m: "_" + m.group(1).lower(), k): v for k, v in vals.items()})


def build(root: pathlib.Path, read_only: bool = False):
    try:  # mcp 2.x
        from mcp.server.mcpserver import MCPServer as Server
    except ImportError:  # mcp 1.x
        from mcp.server.fastmcp import FastMCP as Server
    from .approval import AutoApprover
    from .targets import Runner
    from .tools import TOOLS, Toolbox
    from . import __version__
    tb = Toolbox(root, Runner(AutoApprover(), echo=False))
    try:
        mcp = Server("simwright", instructions=INSTRUCTIONS, version=__version__)
    except TypeError:
        mcp = Server("simwright", instructions=INSTRUCTIONS)
    names = []
    for name, desc in TOOLS.items():
        if read_only and name not in READ_ONLY:
            continue
        fn = getattr(tb, name)

        def make(fn=fn, name=name):
            def call(**kwargs):
                return tb.call(name, kwargs)
            call.__name__ = name
            call.__doc__ = desc
            call.__signature__ = inspect.signature(fn)
            call.__annotations__ = dict(getattr(fn, "__annotations__", {}))
            return call
        try:
            mcp.add_tool(make(), name=name, description=desc, annotations=_annotations(name))
        except TypeError:
            mcp.add_tool(make(), name=name, description=desc)
        names.append(name)
    return mcp, tb, names


# ---------------- HTTP (ChatGPT and other remote clients) ----------------
def http_token(rotate: bool = False) -> str:
    from . import config
    st = config.read_state(HTTP_STATE, {}) or {}
    if rotate or not st.get("token"):
        st = {"token": secrets.token_urlsafe(24), "created": int(time.time())}
        config.write_state(HTTP_STATE, st)
    return st["token"]


def guard(app, token: str):
    """ASGI wrapper: /healthz is open; everything else needs the token as `Authorization: Bearer`, `?token=`
    or a /t/<token>/ path prefix (ChatGPT's 'No authentication' mode can only carry it in the URL)."""
    import hmac
    from urllib.parse import parse_qs

    async def wrapped(scope, receive, send):
        if scope["type"] == "lifespan":
            return await app(scope, receive, send)
        if scope["type"] not in ("http", "websocket"):
            return await app(scope, receive, send)
        path = scope.get("path", "")
        if path.rstrip("/") == "/healthz":
            body = b'{"ok": true, "server": "simwright"}'
            await send({"type": "http.response.start", "status": 200, "headers": [(b"content-type", b"application/json")]})
            return await send({"type": "http.response.body", "body": body})
        got = ""
        m = re.match(r"^/t/([^/]+)(/.*)?$", path)
        if m:
            got, rest = m.group(1), m.group(2) or "/"
            scope = dict(scope, path=rest, raw_path=rest.encode())
        if not got:
            hdr = dict(scope.get("headers") or []).get(b"authorization", b"").decode()
            got = hdr[7:].strip() if hdr.lower().startswith("bearer ") else ""
        if not got:
            got = (parse_qs((scope.get("query_string") or b"").decode()).get("token") or [""])[0]
        if not (got and hmac.compare_digest(got.encode(), token.encode())):
            await send({"type": "http.response.start", "status": 401,
                        "headers": [(b"content-type", b"application/json"), (b"www-authenticate", b"Bearer")]})
            return await send({"type": "http.response.body", "body": b'{"error": "missing or wrong Simwright MCP token"}'})
        return await app(scope, receive, send)
    return wrapped


def http_app(mcp, token: str):
    kw = {"streamable_http_path": "/mcp"}
    try:
        from mcp.server.transport_security import TransportSecuritySettings
        # Host-header checks would reject tunnel hostnames; the token is the access control instead.
        kw["transport_security"] = TransportSecuritySettings(enable_dns_rebinding_protection=False)
    except ImportError:
        pass
    try:
        app = mcp.streamable_http_app(**kw)
    except TypeError:  # mcp 1.x: configured through settings
        mcp.settings.streamable_http_path = "/mcp"
        app = mcp.streamable_http_app()
    return guard(app, token)


def start_tunnel(port: int, timeout: float = 40.0) -> tuple[subprocess.Popen | None, str]:
    """Cloudflare quick tunnel (no account): returns (process, https URL) or (None, reason)."""
    exe = shutil.which("cloudflared")
    if not exe:
        hint = "winget install Cloudflare.cloudflared" if os.name == "nt" else "see https://developers.cloudflare.com/cloudflared/"
        return None, f"cloudflared not found ({hint}); or expose port {port} with ngrok / Tailscale Funnel"
    p = subprocess.Popen([exe, "tunnel", "--no-autoupdate", "--url", f"http://127.0.0.1:{port}"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    found = {"url": ""}

    def read():
        for line in p.stdout:
            m = re.search(r"https://[a-z0-9-]+\.trycloudflare\.com", line)
            if m and not found["url"]:
                found["url"] = m.group(0)
    threading.Thread(target=read, daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < timeout and not found["url"] and p.poll() is None:
        time.sleep(0.3)
    return (p, found["url"]) if found["url"] else (None, "cloudflared started but printed no URL; check your network")


def serve_http(root: pathlib.Path, host: str = "127.0.0.1", port: int = 8765, read_only: bool = False,
               tunnel: bool = False, rotate: bool = False):
    import uvicorn
    mcp, _, names = build(root, read_only)
    token = http_token(rotate)
    app = http_app(mcp, token)
    local = f"http://{host}:{port}"
    print(f"Simwright MCP over HTTP: {len(names)} tools{' (read-only)' if read_only else ''}, workspace {root}", file=sys.stderr)
    print(f"  local (Bearer token or URL):  {local}/t/{token}/mcp", file=sys.stderr)
    procs = []
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    if tunnel:
        def later():
            while not server.started:
                time.sleep(0.2)
            p, url = start_tunnel(port)
            if p:
                procs.append(p)
                print("\nChatGPT: Settings > Apps (Developer mode) > Create, or Customize > Plugins > Add > Create MCP App", file=sys.stderr)
                print(f"  MCP server URL:  {url}/t/{token}/mcp", file=sys.stderr)
                print("  Authentication:  No authentication   (the token is in the URL; keep it private)", file=sys.stderr)
                print("  Quick-tunnel URLs change on every start; rotate the token with --new-token if it leaks.\n", file=sys.stderr)
                from . import config as cfgmod
                cfgmod.write_state(HTTP_STATE, {**(cfgmod.read_state(HTTP_STATE, {}) or {}), "tunnel_url": url, "port": port})
            else:
                print(f"\nTunnel not started: {url}", file=sys.stderr)
        threading.Thread(target=later, daemon=True).start()
    try:
        server.run()
    finally:
        for p in procs:
            p.terminate()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="simwright-mcp", description="Simwright tools over MCP (stdio by default)")
    ap.add_argument("--root", default="", help="robot workspace (default: detected from the current folder)")
    ap.add_argument("--read-only", action="store_true", help="expose only tools that never change anything")
    ap.add_argument("--http", action="store_true", help="serve Streamable HTTP (for ChatGPT and remote clients)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--tunnel", action="store_true", help="with --http: open a Cloudflare quick tunnel and print the ChatGPT URL")
    ap.add_argument("--new-token", action="store_true", help="with --http: rotate the access token")
    a = ap.parse_args(argv)
    from .context import find_root
    root = find_root(a.root or None, exact=bool(a.root))
    if a.http:
        return serve_http(root, a.host, a.port, a.read_only, a.tunnel, a.new_token)
    real = sys.stdout
    sys.stdout = sys.stderr  # tool chatter must not corrupt the stdio protocol
    mcp, _, _ = build(root, a.read_only)
    sys.stdout = real
    mcp.run()


if __name__ == "__main__":
    main()
