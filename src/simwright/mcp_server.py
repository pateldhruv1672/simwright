"""MCP server (stdio): exposes the Simwright toolbox to Claude Code, Cursor, Codex, VS Code, ...

  claude mcp add simwright -- simwright-mcp --root C:\\path\\to\\ws
Commands run without interactive prompts (your MCP client shows each tool call); dangerous shell
shapes are refused.
"""
from __future__ import annotations
import argparse, pathlib, sys


def build(root: pathlib.Path):
    try:  # mcp 2.x
        from mcp.server.mcpserver import MCPServer as FastMCP
    except ImportError:  # mcp 1.x
        from mcp.server.fastmcp import FastMCP
    from .approval import AutoApprover
    from .targets import Runner
    from .tools import TOOLS, Toolbox
    tb = Toolbox(root, Runner(AutoApprover(), echo=False))
    mcp = FastMCP("simwright")
    for name, desc in TOOLS.items():
        fn = getattr(tb, name)

        def make(fn=fn, name=name):
            def call(**kwargs):
                return tb.call(name, kwargs)
            call.__name__ = name
            call.__doc__ = desc
            import inspect
            call.__signature__ = inspect.signature(fn)
            call.__annotations__ = {k: v for k, v in getattr(fn, "__annotations__", {}).items()}
            return call
        mcp.add_tool(make(), name=name, description=desc)
    return mcp, tb


def main(argv=None):
    ap = argparse.ArgumentParser(prog="simwright-mcp")
    ap.add_argument("--root", default=".")
    a = ap.parse_args(argv)
    from .context import find_root
    import io
    sys.stdout = sys.stderr  # tool chatter must not corrupt the stdio protocol
    mcp, _ = build(find_root(a.root))
    sys.stdout = sys.__stdout__
    mcp.run()


if __name__ == "__main__":
    main()
