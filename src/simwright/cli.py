"""simwright: command line and interactive REPL.

  simwright                 interactive copilot in the current workspace (/help for commands)
  simwright init            scan the workspace, write SIMWRIGHT.md
  simwright skill add F     install a team skill (--repo to share it through the repo)
  simwright doctor          check GPU, WSL, Docker, Isaac Sim, Isaac Lab, ROS 2, Mana Bridge
  simwright ros graph       live ROS 2 graph with QoS problems
  simwright isaac launch    start Isaac Sim with the python_server + Simwright panel
  simwright lab train TASK  train a policy in Isaac Lab (background job)
  simwright world spec.json compose a room and build it in Isaac Sim
  simwright mcp             MCP server for Claude Code / Cursor / Codex
  simwright status --json   machine summary (used by the Mana console)
"""
from __future__ import annotations
import argparse, json, os, pathlib, shlex, subprocess, sys
from . import __version__, config, context, skills
from .approval import Approver


def _out(obj):
    print(json.dumps(obj, indent=2, default=str))


def _tb(root, mode=None):
    from .targets import Runner
    from .tools import Toolbox
    approver = Approver(mode or config.load().get("approval", "auto"))
    return Toolbox(root, Runner(approver))


def cmd_init(root, force=False):
    out, info, wrote = context.init(root, force)
    counts = {k: len(v) for k, v in info.items() if isinstance(v, list)}
    print(("Wrote " if wrote else "Kept existing ") + str(out))
    print("Found: " + ", ".join(f"{v} {k}" for k, v in counts.items() if v) if counts else "Found nothing yet (empty workspace?)")
    if not wrote:
        print("Use --force to regenerate (your edits would be replaced).")


def cmd_skill(root, args):
    if args.action == "add":
        s = skills.add(args.path, root, args.repo, args.name)
        print(f"Installed skill '{s.name}' -> {s.path} ({s.scope}). Used automatically from the next prompt.")
    elif args.action == "list":
        for s in skills.discover(root):
            print(f"{s.name:38s} [{s.scope}] {s.description[:90]}")
    elif args.action == "remove":
        print("removed" if skills.remove(args.path, root) else "not found (only user/repo skills can be removed)")
    elif args.action == "nvidia":
        cmd = ["npx", "skills@latest", "add", "nvidia/skills", "--agent", "claude-code", "--yes"] + (["--skill", args.path] if args.path else ["--list"])
        print("> " + " ".join(cmd))
        subprocess.run(cmd, cwd=str(root), shell=os.name == "nt")


def cmd_doctor(args):
    from .doctor import run_doctor
    r = run_doctor(deep=args.deep)
    if args.json:
        return _out(r)
    for c in r["checks"]:
        print(f"{'OK ' if c['ok'] else '-- '} {c['check']:26s} {c['detail'][:110]}")
        if not c["ok"] and c.get("fix"):
            print(f"    fix: {c['fix']}")


def cmd_status(root):
    from .doctor import run_doctor
    from . import lab
    from .isaac import find_isaac_sim
    _out({"version": __version__, "root": str(root), "doctor": run_doctor(), "isaac_sim": find_isaac_sim(),
          "lab_version": lab.Lab().version(), "lab": {"dir": str(lab.Lab().dir or ""), "python": lab.Lab().python,
                                                       "installs": lab.discover()},
          "runs": lab.Lab().runs(15), "jobs": [
              {k: v for k, v in j.items() if k not in ("tail",)} for j in lab.job_status()],
          "policies": lab.policies(), "skills": [{"name": s.name, "scope": s.scope} for s in skills.discover(root)],
          "context_file": context.load_context(root)[0],
          "simready": [{k: r.get(k) for k in ("url", "package", "dataset", "revision", "license", "local_usd", "fetched_at")} for r in _simready_catalog()]})


def _simready_catalog():
    try:
        from . import simready
        return simready.catalog()[-50:]
    except Exception:
        return []


HELP = """Slash commands
  /init [--force]            scan workspace, write SIMWRIGHT.md
  /skill add <file> [--repo] install a skill   (or: "add this skill: ./x.md")
  /skills                    list skills
  /doctor                    machine checks
  /ros graph | /ros trace <topic>
  /urdf <file>   /build [pkgs]   /mujoco <xml>   /gazebo <world>
  /isaac launch|status|scene|check|capture|record|assets <kw>
  /world <spec.json>
  /simready search <query> [--limit=8 --max_size=0.5] | fetch <url> | place <url> [--on=/World/table --x= --y= --yaw= --prim=] | list
  /lab installs|tasks|train <task>|jobs|runs|play <task>|export <task> <ckpt>|policies
  /mode auto|step|ask        auto = run uninterrupted (ESC pauses), step = confirm each command
  /image <path>              attach a photo (e.g. of a room to match)
  /exit
Anything else is a prompt for the copilot."""


def repl(root):
    tb = _tb(root)
    name, _ = context.load_context(root)
    print(f"Simwright {__version__}  workspace: {root}")
    print(f"context: {name or 'none (run /init)'}  skills: {len(skills.discover(root))}  mode: {tb.runner.approver.mode}  (ESC pauses, /help)")
    agent = None
    while True:
        try:
            line = input("\nsimwright> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        nl = skills.parse_nl_add(line)
        if nl:
            s = skills.add(nl[0], root, nl[1])
            print(f"Installed skill '{s.name}' -> {s.path}")
            continue
        if line.startswith("/"):
            parts = shlex.split(line[1:], posix=os.name != "nt")
            c, rest = parts[0], parts[1:]
            try:
                if c in ("exit", "quit"):
                    return
                elif c == "help":
                    print(HELP)
                elif c == "init":
                    cmd_init(root, "--force" in rest)
                elif c == "skill" and rest[:1] == ["add"]:
                    s = skills.add(rest[1], root, "--repo" in rest)
                    print(f"Installed skill '{s.name}' -> {s.path}")
                elif c == "skills":
                    cmd_skill(root, argparse.Namespace(action="list"))
                elif c == "doctor":
                    cmd_doctor(argparse.Namespace(deep=False, json=False))
                elif c == "mode":
                    tb.runner.approver.mode = rest[0] if rest else "auto"
                    print("mode:", tb.runner.approver.mode)
                elif c == "image":
                    if agent is None:
                        from .agent import Agent
                        agent = Agent(root, tb)
                    agent.attach_image(rest[0])
                    print("attached; it goes with your next prompt")
                else:
                    _out(dispatch(tb, c, rest))
            except Exception as e:
                print(f"error: {type(e).__name__}: {e}")
            continue
        try:
            if agent is None:
                from .agent import Agent
                agent = Agent(root, tb)
            agent.turn(line)
        except Exception as e:
            print(f"error: {e}")


def dispatch(tb, c, rest):
    """Shared by the REPL and argparse subcommands."""
    a = rest + [""] * 4
    if c == "ros":
        return tb.ros2_trace(a[1]) if a[0] == "trace" else tb.ros2_graph()
    if c == "urdf":
        return tb.urdf_validate(a[0])
    if c == "build":
        return tb.colcon_build(" ".join(rest))
    if c == "mujoco":
        return tb.mujoco_check(a[0])
    if c == "gazebo":
        return tb.gazebo_check(a[0])
    if c == "isaac":
        sub = a[0] or "status"
        return {"launch": lambda: tb.isaac_launch("--headless" in rest), "status": tb.isaac_status, "scene": tb.isaac_scene,
                "check": lambda: tb.isaac_physics_check(float(a[1] or 3)), "capture": lambda: tb.isaac_capture(a[1] or ".simwright/captures/frame.png"),
                "record": lambda: tb.isaac_record(float(a[1] or 4)), "assets": lambda: tb.isaac_asset_search(rest[1:]),
                "exec": lambda: tb.isaac_exec(pathlib.Path(a[1]).read_text() if a[1] != "-" else sys.stdin.read())}[sub]()
    if c == "world":
        spec = json.loads(pathlib.Path(a[0]).read_text())
        r = tb.world_compose(spec, pathlib.Path(a[0]).stem, apply="--no-apply" not in rest)
        print(r.pop("map"))
        return r
    if c == "simready":
        sub = a[0]
        pos, kw, items, i = [], {}, list(rest[1:]), 0
        while i < len(items):
            x = items[i]
            if x.startswith("--"):
                k, eq, v = x[2:].partition("=")
                if not eq and i + 1 < len(items) and not items[i + 1].startswith("--"):
                    v, i = items[i + 1], i + 1
                kw[k.replace("-", "_")] = v if (eq or v) else True
            else:
                pos.append(x)
            i += 1
        if sub == "search":
            return tb.simready_search(" ".join(pos), int(kw.get("limit", 8)), float(kw.get("max_size", 0)),
                                      float(kw.get("min_size", 0)), kw.get("dataset", ""))
        if sub in ("fetch", "get"):
            return tb.simready_fetch(pos[0])
        if sub == "place":
            import re as _re
            stem = _re.sub(r"\W+", "_", pos[0].replace("\\", "/").rsplit("/", 1)[-1].rsplit(".", 1)[0])
            return tb.simready_place(pos[0], kw.get("prim", "/World/SimReady/" + stem), float(kw.get("x", 0)), float(kw.get("y", 0)),
                                     float(kw.get("yaw", 0)), kw.get("on", ""), kw.get("physics", "auto"), use_xy=("x" in kw or "y" in kw))
        if sub in ("list", "catalog"):
            return tb.simready_catalog()
        raise ValueError("simready search|fetch|place|list")
    if c == "lab":
        sub = a[0]
        if sub == "installs":
            from . import lab as labm
            L = labm.Lab()
            return {"ok": True, "selected": str(L.dir or ""), "python": L.python, "installs": labm.discover()}
        if sub == "tasks":
            return tb.lab_tasks(a[1])
        if sub == "train":
            kw = _kv(rest[2:])
            return tb.lab_train(a[1], int(kw.get("num_envs", 0)), int(kw.get("max_iterations", 0)), "no_headless" not in kw)
        if sub == "jobs":
            return tb.lab_jobs(a[1])
        if sub == "stop":
            return tb.lab_stop(a[1])
        if sub == "runs":
            return tb.lab_runs()
        if sub == "play":
            kw = _kv(rest[2:])
            return tb.lab_play(a[1], kw.get("checkpoint", ""), int(kw.get("num_envs", 16)), "video" in kw)
        if sub == "export":
            return tb.lab_export(a[1], a[2])
        if sub == "register":
            return tb.policy_register(a[1], a[2], a[3], " ".join(rest[4:]))
        if sub == "policies":
            return tb.policy_list()
        if sub == "deploy":
            return tb.policy_deploy(a[1], a[2])
    raise ValueError(f"unknown command /{c} {' '.join(rest)} (try /help)")


def _kv(args):
    out, i = {}, 0
    while i < len(args):
        k = args[i].lstrip("-").replace("-", "_")
        if i + 1 < len(args) and not args[i + 1].startswith("--"):
            out[k] = args[i + 1]
            i += 2
        else:
            out[k] = True
            i += 1
    return out


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if os.name == "nt":
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass
    ap = argparse.ArgumentParser(prog="simwright", description="AI copilot for robot simulation")
    ap.add_argument("--root", default=None, help="workspace root (default: detected from cwd)")
    ap.add_argument("--version", action="version", version=__version__)
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("init"); p.add_argument("--force", action="store_true")
    p = sub.add_parser("skill"); p.add_argument("action", choices=["add", "list", "remove", "nvidia"]); p.add_argument("path", nargs="?", default="")
    p.add_argument("--repo", action="store_true"); p.add_argument("--name")
    p = sub.add_parser("doctor"); p.add_argument("--deep", action="store_true"); p.add_argument("--json", action="store_true")
    sub.add_parser("status").add_argument("--json", action="store_true")
    p = sub.add_parser("config"); p.add_argument("key", nargs="?"); p.add_argument("value", nargs="?")
    sub.add_parser("targets")
    sub.add_parser("mcp")
    p = sub.add_parser("serve"); p.add_argument("--port", type=int)
    p = sub.add_parser("chat"); p.add_argument("prompt", nargs="+"); p.add_argument("--mode", default=None)
    for name in ("ros", "urdf", "build", "mujoco", "gazebo", "isaac", "world", "lab", "simready"):
        p = sub.add_parser(name); p.add_argument("rest", nargs=argparse.REMAINDER)
    a = ap.parse_args(argv)
    root = context.find_root(a.root)
    if a.cmd is None:
        return repl(root)
    if a.cmd == "init":
        return cmd_init(root, a.force)
    if a.cmd == "skill":
        return cmd_skill(root, a)
    if a.cmd == "doctor":
        return cmd_doctor(a)
    if a.cmd == "status":
        return cmd_status(root)
    if a.cmd == "config":
        if not a.key:
            return _out(config.load())
        try:
            val = json.loads(a.value)
        except (TypeError, json.JSONDecodeError):
            val = a.value
        config.set_value(a.key, val)
        return print(f"{a.key} = {val!r}")
    if a.cmd == "targets":
        return _out(config.load()["targets"])
    if a.cmd == "mcp":
        from .mcp_server import main as mcp_main
        return mcp_main(["--root", str(root)])
    if a.cmd == "serve":
        from .serve import serve
        return serve(root, a.port)
    if a.cmd == "chat":
        from .agent import Agent
        return Agent(root, _tb(root, a.mode)).turn(" ".join(a.prompt))
    res = dispatch(_tb(root), a.cmd, a.rest)
    if res is not None:
        _out(res)


if __name__ == "__main__":
    main()
