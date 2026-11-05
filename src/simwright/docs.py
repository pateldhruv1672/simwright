"""NVIDIA Isaac Sim MCP server (kit-usd-agents) as a knowledge source: real extension, API, code-example and settings
lookups before Simwright writes Isaac Sim code. It is a documentation server (5 read-only tools), not a live-sim controller.
Default endpoint http://localhost:9904/mcp (Docker); set `simwright config isaac_docs_url <url>` to change it."""
from __future__ import annotations
import asyncio, json
from . import config

DEFAULT_URL = "http://localhost:9904/mcp"
KINDS = {
    "code": "search_isaac_sim_code_examples",
    "extensions": "search_isaac_sim_extensions",
    "extension": "get_isaac_sim_extension_details",
    "settings": "search_isaac_sim_settings",
    "instructions": "get_isaac_sim_instructions",
}
SETUP_HINT = ("NVIDIA's Isaac Sim docs server is not running. Set it up once with `simwright docs setup` "
              "(Docker + an NVIDIA API key from build.nvidia.com/settings/api-keys), or point Simwright at another "
              "endpoint with `simwright config isaac_docs_url <url>`.")


def url() -> str:
    return config.load().get("isaac_docs_url") or DEFAULT_URL


def _args_for(schema: dict, query: str) -> dict:
    """Fill the tool's own input schema: the query goes to its first string (or string-array) parameter."""
    props = (schema or {}).get("properties") or {}
    req = (schema or {}).get("required") or list(props)
    for name in list(req) + [p for p in props if p not in req]:
        t = (props.get(name) or {}).get("type")
        if t == "string" or t is None:
            return {name: query}
        if t == "array":
            return {name: [q.strip() for q in query.split(",") if q.strip()]}
    return {}


def _text(result) -> str:
    out = []
    for c in getattr(result, "content", None) or []:
        out.append(getattr(c, "text", None) or json.dumps(getattr(c, "model_dump", lambda: str(c))(), default=str))
    return "\n".join(out)


async def _call(endpoint: str, tool: str | None, query: str, timeout: float) -> dict:
    import anyio
    with anyio.fail_after(timeout + 15):
        return await _call_inner(endpoint, tool, query, timeout)


async def _call_inner(endpoint: str, tool: str | None, query: str, timeout: float) -> dict:
    from mcp import ClientSession
    try:   # mcp 2.x
        from mcp.client.streamable_http import streamable_http_client as _client
        ctx = _client(endpoint)
    except ImportError:   # mcp 1.x
        from mcp.client.streamable_http import streamablehttp_client as _client
        ctx = _client(endpoint, timeout=timeout)
    async with ctx as streams:
        read, write = streams[0], streams[1]
        async with ClientSession(read, write) as s:
            await s.initialize()
            tools = {t.name: t for t in (await s.list_tools()).tools}
            if tool is None:
                return {"ok": True, "tools": sorted(tools)}
            if tool not in tools:
                return {"ok": False, "error": f"the docs server has no tool {tool}", "tools": sorted(tools)}
            t = tools[tool]
            schema = getattr(t, "input_schema", None) or getattr(t, "inputSchema", None) or {}
            r = await s.call_tool(tool, _args_for(schema, query) if query else {})
            err = getattr(r, "is_error", None)
            if err is None:
                err = getattr(r, "isError", False)
            return {"ok": not err, "tool": tool, "result": _text(r)[:14000]}


def _root(e: BaseException) -> str:
    while getattr(e, "exceptions", None):
        e = e.exceptions[0]
    return f"{type(e).__name__}: {e}"[:400]


def search(query: str = "", kind: str = "code", endpoint: str | None = None, timeout: float = 60) -> dict:
    tool = KINDS.get(kind)
    if not tool:
        return {"ok": False, "error": f"kind must be one of {sorted(KINDS)}"}
    ep = endpoint or url()
    try:
        return {"endpoint": ep, **asyncio.run(_call(ep, tool, query, timeout))}
    except Exception as e:
        return {"ok": False, "endpoint": ep, "error": _root(e), "fix": SETUP_HINT}


def status(endpoint: str | None = None) -> dict:
    ep = endpoint or url()
    try:
        r = asyncio.run(_call(ep, None, "", 15))
        return {"ok": True, "endpoint": ep, "tools": r.get("tools", [])}
    except Exception as e:
        return {"ok": False, "endpoint": ep, "error": _root(e), "fix": SETUP_HINT}


def env_file():
    return config.state_file("isaac_docs.env")


def setup_commands(windows: bool, repo_dir: str, env_path: str) -> list[tuple[str, str]]:
    """Commands to build and run NVIDIA's Isaac Sim MCP container (Docker), in order, with what each does."""
    q = (lambda p: f'"{p}"')
    mcp_dir = repo_dir + ("\\source\\mcp\\isaacsim_mcp" if windows else "/source/mcp/isaacsim_mcp")
    if windows:
        return [
            ("git lfs version", "Git LFS is required: the Isaac Sim index files are LFS objects"),
            (f"if (!(Test-Path {q(repo_dir)})) {{ git clone --depth 1 https://github.com/NVIDIA-Omniverse/kit-usd-agents.git {q(repo_dir)} }}; "
             f"git -C {q(repo_dir)} lfs install --local; git -C {q(repo_dir)} lfs pull", "get NVIDIA's kit-usd-agents source with its index files"),
            ("uv python install 3.12; uv tool install poetry", "Python 3.12 and Poetry for the wheel build"),
            (f"$py = (uv python find 3.12); $env:POETRY_KEYRING_ENABLED='false'; "
             f"Push-Location {q(repo_dir + chr(92) + 'source' + chr(92) + 'aiq' + chr(92) + 'isaacsim_fns')}; poetry env use $py; Pop-Location; "
             f"Push-Location {q(mcp_dir)}; poetry env use $py; Pop-Location", "pin Poetry to Python 3.12 in both packages"),
            (f"Push-Location {q(mcp_dir)}; $env:POETRY_KEYRING_ENABLED='false'; .\\build-docker.bat; Pop-Location", "build the isaacsim-mcp image (10-15 min the first time)"),
            (f"docker rm -f isaacsim-mcp 2>$null; docker run -d --name isaacsim-mcp --restart unless-stopped -p 9904:9904 --env-file {q(env_path)} isaacsim-mcp:latest",
             "run it on http://localhost:9904/mcp, restarting with Docker"),
        ]
    return [
        ("git lfs version", "Git LFS is required"),
        (f"[ -d {q(repo_dir)} ] || git clone --depth 1 https://github.com/NVIDIA-Omniverse/kit-usd-agents.git {q(repo_dir)}; git -C {q(repo_dir)} lfs install --local; git -C {q(repo_dir)} lfs pull", "get the source"),
        ("uv python install 3.12 && uv tool install poetry", "Python 3.12 and Poetry"),
        (f"cd {q(mcp_dir)} && POETRY_KEYRING_ENABLED=false ./build-docker.sh", "build the image"),
        (f"docker rm -f isaacsim-mcp 2>/dev/null; docker run -d --name isaacsim-mcp --restart unless-stopped -p 9904:9904 --env-file {q(env_path)} isaacsim-mcp:latest", "run it"),
    ]
