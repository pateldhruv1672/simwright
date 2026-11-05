"""The toolbox: one set of robotics tools shared by the CLI agent, the MCP server and the Isaac Sim panel."""
from __future__ import annotations
import difflib, fnmatch, inspect, json, os, pathlib, re, time, typing
from . import config, colcon, lab as labmod, ros2, sims, skills, urdf, world
from .isaac import KitClient, KitError, find_isaac_sim, frames_to_mp4, launch as kit_launch
from .memory import ProjectMemory, Session
from .targets import Runner, Target

TOOLS: dict[str, str] = {}


def tool(desc: str):
    def deco(fn):
        TOOLS[fn.__name__] = desc
        return fn
    return deco


def _pid_alive(pid: int) -> bool:
    if not pid:
        return False
    if os.name == "nt":
        import ctypes
        h = ctypes.windll.kernel32.OpenProcess(0x1000, False, int(pid))  # PROCESS_QUERY_LIMITED_INFORMATION
        if not h:
            return False
        code = ctypes.c_ulong()
        ok = ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
        ctypes.windll.kernel32.CloseHandle(h)
        return bool(ok) and code.value == 259  # STILL_ACTIVE
    try:
        os.kill(int(pid), 0)
        return True
    except OSError:
        return False


def crash_report(wait: float = 0.0, marker: pathlib.Path | None = None) -> dict | None:
    """If the last physics check never finished and its Isaac Sim process is gone, say so (with the backup)."""
    f = marker or (pathlib.Path.home() / ".simwright" / "physics_check.json")
    end = time.time() + wait
    while True:
        try:
            m = json.loads(f.read_text())
        except Exception:
            return None
        if m.get("state") != "running":
            return None
        if not _pid_alive(m.get("kit_pid", 0)):
            return {"crash": "Isaac Sim exited during a physics check", "stage": m.get("stage"), "backup": m.get("backup"),
                    "at": time.strftime("%Y-%m-%d %H:%M", time.localtime(m.get("started", 0)))}
        if time.time() >= end:
            return None
        time.sleep(1)


class Toolbox:
    def __init__(self, root: pathlib.Path, runner: Runner | None = None):
        self.root = root
        self.session = Session(root)
        self.memory = ProjectMemory(root)
        self.runner = runner or Runner(log=self.session.log)
        if self.runner.log is None:
            self.runner.log = self.session.log
        self.cfg = config.load()
        self.kit = KitClient.from_config()

    # ---------- helpers ----------
    def _abs(self, p: str) -> pathlib.Path:
        q = pathlib.Path(p).expanduser()
        return q if q.is_absolute() else (self.root / q)

    def _out(self, rel: str) -> pathlib.Path:
        """Output files (captures, recordings) go under the workspace's .simwright, unless the shell sits somewhere
        unwritable such as C:\\Windows\\System32; then under ~/.simwright."""
        q = pathlib.Path(rel).expanduser()
        if q.is_absolute():
            return q
        rel_tail = rel[len(".simwright/"):] if rel.startswith(".simwright/") else rel
        root = self.root.resolve()
        bad = "windows" in [part.lower() for part in root.parts[:2]] or not os.access(root, os.W_OK)
        return (config.HOME / rel_tail) if bad else (root / rel)

    def _rel(self, p: pathlib.Path) -> str:
        try:
            return p.resolve().relative_to(self.root.resolve()).as_posix()
        except ValueError:
            return str(p)

    def schema(self, name: str) -> dict:
        fn = getattr(self, name)
        props, req = {}, []
        hints = typing.get_type_hints(fn)
        for pname, param in inspect.signature(fn).parameters.items():
            t = hints.get(pname, str)
            origin = typing.get_origin(t)
            if origin is typing.Union:
                t = [a for a in typing.get_args(t) if a is not type(None)][0]
            js = {str: "string", int: "integer", float: "number", bool: "boolean", dict: "object", list: "array"}.get(t, "string")
            props[pname] = {"type": js}
            if js == "array":
                props[pname]["items"] = {"type": "string"}
            if param.default is inspect.Parameter.empty:
                req.append(pname)
        return {"type": "function", "function": {"name": name, "description": TOOLS[name],
                                                 "parameters": {"type": "object", "properties": props, "required": req}}}

    def schemas(self) -> list[dict]:
        return [self.schema(n) for n in TOOLS]

    def call(self, name: str, args: dict) -> dict:
        if name not in TOOLS:
            return {"ok": False, "error": f"unknown tool {name}"}
        try:
            out = getattr(self, name)(**(args or {}))
            return out if isinstance(out, dict) else {"ok": True, "result": out}
        except KitError as e:
            return {"ok": False, "error": str(e)}
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ---------- shell & files ----------
    @tool("Run a shell command. target: 'host' (Windows PowerShell / local shell), 'wsl' (Ubuntu with ROS sourced) or 'ros' (Docker container); see `simwright targets`. The user sees every command before it runs.")
    def run_command(self, command: str, target: str = "host", cwd: str = "", description: str = "", timeout: int = 1800) -> dict:
        cwd = str(self._abs(cwd)) if cwd else str(self.root)
        return self.runner.run(command, target, cwd, description=description, timeout=timeout).brief()

    @tool("Read a text file (line range). Use before editing; never guess file contents.")
    def read_file(self, path: str, start: int = 1, end: int = 400) -> dict:
        p = self._abs(path)
        lines = p.read_text(encoding="utf-8", errors="replace").splitlines()
        return {"ok": True, "path": self._rel(p), "total_lines": len(lines),
                "text": "\n".join(f"{i}: {l}" for i, l in enumerate(lines[start - 1:end], start))}

    @tool("Create or overwrite a file with full content. Shows a diff for approval. Give `why` so the session remembers it.")
    def write_file(self, path: str, content: str, why: str = "") -> dict:
        p = self._abs(path)
        old = p.read_text(encoding="utf-8", errors="replace") if p.exists() else ""
        diff = list(difflib.unified_diff(old.splitlines(), content.splitlines(), self._rel(p), self._rel(p), lineterm="", n=2))
        added = sum(1 for d in diff if d.startswith("+") and not d.startswith("+++"))
        removed = sum(1 for d in diff if d.startswith("-") and not d.startswith("---"))
        import sys
        print("\n".join(diff[:80]) + ("\n  ..." if len(diff) > 80 else ""), file=sys.stdout if self.runner.echo else sys.stderr)
        if self.runner.approver.decide("file", f"write {self._rel(p)} (+{added}/-{removed})", why) != "run":
            return {"ok": False, "skipped": True}
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content, encoding="utf-8")
        self.session.log({"type": "write", "path": self._rel(p), "why": why, "added": added, "removed": removed})
        return {"ok": True, "path": self._rel(p), "added": added, "removed": removed, "created": not old}

    @tool("Replace an exact snippet in a file (must match once). Prefer this over write_file for small changes.")
    def edit_file(self, path: str, old: str, new: str, why: str = "") -> dict:
        p = self._abs(path)
        text = p.read_text(encoding="utf-8", errors="replace")
        n = text.count(old)
        if n != 1:
            return {"ok": False, "error": f"snippet found {n} times; include more context so it matches exactly once"}
        return self.write_file(path, text.replace(old, new), why)

    @tool("List a directory tree (skips build/install/log/.git).")
    def list_dir(self, path: str = ".", depth: int = 2) -> dict:
        base = self._abs(path)
        out = []
        def walk(d, lvl):
            for c in sorted(d.iterdir(), key=lambda x: (x.is_file(), x.name.lower())):
                if c.name in ("build", "install", "log", ".git", "__pycache__", "node_modules", ".venv"):
                    continue
                out.append("  " * lvl + c.name + ("/" if c.is_dir() else ""))
                if c.is_dir() and lvl + 1 < depth and len(out) < 600:
                    walk(c, lvl + 1)
        walk(base, 0)
        return {"ok": True, "path": self._rel(base), "tree": "\n".join(out[:600])}

    @tool("Search files for a regex (like grep -rn). glob filters filenames, e.g. '*.launch.py'.")
    def search_files(self, pattern: str, glob: str = "*", max_hits: int = 80) -> dict:
        rx = re.compile(pattern)
        hits = []
        from .context import walk
        for f in walk(self.root):
            if not fnmatch.fnmatch(f.name, glob) or f.stat().st_size > 1_000_000:
                continue
            try:
                for i, line in enumerate(f.read_text(encoding="utf-8", errors="ignore").splitlines(), 1):
                    if rx.search(line):
                        hits.append(f"{self._rel(f)}:{i}: {line.strip()[:200]}")
                        if len(hits) >= max_hits:
                            return {"ok": True, "hits": hits, "truncated": True}
            except Exception:
                pass
        return {"ok": True, "hits": hits}

    # ---------- ROS 2 ----------
    def _ros(self, target: str) -> str:
        return target or self.cfg.get("ros_target", "wsl")

    @tool("Inspect the running ROS 2 graph: nodes, topics with types, publishers/subscribers with QoS, controllers, and QoS/type incompatibilities.")
    def ros2_graph(self, target: str = "") -> dict:
        r = self.runner.capture(ros2.graph_script(), self._ros(target), description="inspect ROS 2 graph", timeout=300)
        g = ros2.parse_graph(r.out)
        topics = {k: {"type": v["type"], "pubs": sum(e["kind"].upper().startswith("PUB") for e in v["endpoints"]),
                      "subs": sum(e["kind"].upper().startswith("SUB") for e in v["endpoints"])} for k, v in g["topics"].items()}
        self._last_graph = g
        return {"ok": not g["error"], "error": g["error"], "distro": g["distro"], "nodes": g["nodes"], "topics": topics,
                "controllers": g["controllers"], "issues": g["issues"], "truncated": g["truncated"]}

    @tool("Trace a command chain from a topic (e.g. /cmd_vel): publishers, subscribers, what those subscribers publish, active controllers, QoS problems.")
    def ros2_trace(self, topic: str, target: str = "") -> dict:
        r = self.runner.capture(ros2.graph_script(), self._ros(target), description=f"trace {topic}", timeout=300)
        return {"ok": True, **ros2.trace(ros2.parse_graph(r.out), topic)}

    @tool("Validate a URDF (or xacro, expanded in the ROS target): inertia positive-definite and triangle inequality, masses, joint limits, tree structure.")
    def urdf_validate(self, path: str, target: str = "") -> dict:
        p = self._abs(path)
        text = None
        if p.suffix == ".xacro" or p.name.endswith(".urdf.xacro"):
            t = Target.get(self._ros(target))
            r = self.runner.capture(f"xacro {t.path(str(p))}", t, description="expand xacro")
            if r.ok and "<robot" in r.out:
                text = r.out[r.out.index("<"):] if "<" in r.out else r.out
        res = urdf.validate(str(p), text)
        res["path"] = self._rel(p)
        return res

    @tool("Build the colcon workspace in the build target, then diagnose failures (missing deps, CMake/package.xml/setup.py errors) with the coordinated fix.")
    def colcon_build(self, packages: str = "", args: str = "--symlink-install", target: str = "") -> dict:
        t = Target.get(target or self.cfg.get("build_target", "wsl"))
        sel = f" --packages-select {packages}" if packages else ""
        r = self.runner.run(f"colcon build {args}{sel} --event-handlers console_cohesion+ 2>&1", t, str(self.root),
                            description="colcon build", timeout=7200)
        diag = colcon.diagnose(r.out)
        return {"ok": r.ok, "exit": r.exit, **diag, "tail": "\n".join(r.out.splitlines()[-40:])}

    @tool("Diagnose a colcon build log you already have (text).")
    def colcon_diagnose(self, log_text: str) -> dict:
        return colcon.diagnose(log_text)

    @tool("Load and step a MuJoCo MJCF headless; reports NaN in qacc/qpos, solver warnings, penetration, velocities.")
    def mujoco_check(self, model: str, steps: int = 2000, target: str = "host") -> dict:
        t = Target.get(target)
        r = self.runner.capture(sims.mujoco_command(t.path(str(self._abs(model))), steps), t, description="MuJoCo check")
        return sims.interpret_mujoco(r.out)

    @tool("Run a Gazebo Harmonic world headless for N iterations and report plugin/model/SDF errors.")
    def gazebo_check(self, world_file: str, iterations: int = 2000, target: str = "") -> dict:
        t = Target.get(self._ros(target))
        r = self.runner.capture(sims.gazebo_command(t.path(str(self._abs(world_file))), iterations), t, description="Gazebo check", timeout=300)
        return {**sims.interpret_gazebo(r.out), "tail": "\n".join(r.out.splitlines()[-25:])}

    # ---------- Isaac Sim ----------
    @tool("Is Isaac Sim installed and is a session reachable (python_server)?")
    def isaac_status(self) -> dict:
        inst = find_isaac_sim(self.cfg)
        try:
            ping = self.kit.ping()
        except KitError as e:
            ping = {"ok": False, "error": str(e)}
        except (TimeoutError, OSError) as e:
            ping = {"ok": False, "error": f"Isaac Sim is running but not responding ({type(e).__name__}): its main thread is busy "
                    "(a long script or physics step) or hung. Wait, or close it if it stays like this; unsaved work would be lost."}
        out = {"ok": True, "install": inst, "session": ping}
        rep = crash_report()
        if rep:
            out["last_physics_check"] = rep
        return out

    @tool("Launch Isaac Sim with the python_server and the Simwright extension (headless optional). Waits until the app is ready.")
    def isaac_launch(self, headless: bool = False) -> dict:
        if self.runner.approver.decide("host", "launch Isaac Sim" + (" headless" if headless else ""), "") != "run":
            return {"ok": False, "skipped": True}
        s = kit_launch(headless=headless)
        self.kit = KitClient.from_config()
        return {"ok": bool(s.get("ready")), **{k: v for k, v in s.items() if k != "token"}}

    @tool("Execute Python inside the running Isaac Sim (Kit). Top-level await allowed. Print what you need back. Always fill `why` with one plain sentence saying what the code changes (or 'read-only: ...'); the user approves from it.")
    def isaac_exec(self, code: str, why: str = "", timeout: int = 120) -> dict:
        lines = [l for l in code.strip().splitlines() if l.strip() and not re.match(r"\s*(import|from)\s", l)]
        head = (lines[0] if lines else (code.strip().splitlines() or [""])[0])[:100]
        label = (why.strip() or f"python ({len(code.splitlines())} lines): {head}")[:200]
        if self.runner.approver.decide("isaac-sim", label, f"{len(code.splitlines())} lines of Kit Python", detail=code[:4000]) != "run":
            return {"ok": False, "skipped": True, "note": "the user skipped this step; do not retry it another way"}
        r = self.kit.exec(code, timeout)
        if why.strip() and not why.strip().lower().startswith("read-only"):
            self.session.log({"type": "scene", "what": why.strip()[:200] + ("" if r.get("status") == "ok" else " (failed)")})
        return {"ok": r.get("status") == "ok", "output": r.get("output", "")[-6000:], "result": r.get("result"),
                "error": (f"{r.get('ename')}: {r.get('evalue')}" if r.get("status") != "ok" else ""),
                "traceback": "".join(r.get("traceback", []))[-3000:]}

    @tool("Summarize the open Isaac Sim stage: prim types, articulations, rigid bodies, lights, cameras, physics scene, timeline.")
    def isaac_scene(self) -> dict:
        return {"ok": True, **self.kit.call("scene_summary()")}

    @tool("Play the simulation for N seconds and verify physics: NaN poses, bodies through the floor, explosions, missing PhysicsScene, lighting. "
          "First runs a preflight for setups known to crash PhysX (CCD on a GPU scene, nested articulation roots, negative mass) and refuses to "
          "play while any are present unless force=true; backs up the stage with its unsaved edits before playing.")
    def isaac_physics_check(self, seconds: float = 3.0, force: bool = False) -> dict:
        prev = crash_report()
        try:
            r = self.kit.call(f"await physics_check({float(seconds)}, force={bool(force)})", timeout=seconds * 30 + 90)
        except Exception as e:
            rep = crash_report(wait=8)
            if rep:
                self.session.log({"type": "scene", "what": f"Isaac Sim crashed during a physics check; backup {rep.get('backup')}"})
                return {"ok": False, "crashed": True, **rep,
                        "next": "Relaunch Isaac Sim (simwright isaac launch), open the backup (it has the edits made before the crash), "
                                "run isaac_scene and fix the preflight issues before playing again."}
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}
        if prev and isinstance(r, dict):
            r["previous_crash"] = prev
        return r

    @tool("Save a viewport screenshot (PNG) to a path on this machine.")
    def isaac_capture(self, path: str = ".simwright/captures/frame.png") -> dict:
        p = str(self._out(path)).replace("\\", "/")
        return self.kit.call(f"await capture({json.dumps(p)})", timeout=180)

    # ---------- project memory (shared by the panel, the terminal copilot and MCP clients) ----------
    @tool("Read this project's memory: notes (decisions, saved stages, conventions, next steps), what Simwright changed "
          "(files, scene edits, commands) and earlier conversations. Read it before continuing earlier work.")
    def project_memory(self) -> dict:
        return {"ok": True, "notes": self.memory.notes(), "session": self.session.summary(), "conversations": self.memory.history(10)}

    @tool("Add a dated note to the project memory (.simwright/notes.md): a decision, a saved stage path, a convention, "
          "or what to do next. Keep it one or two factual sentences with real paths.")
    def project_note(self, text: str, kind: str = "note") -> dict:
        return {"ok": True, "added": self.memory.add_note(text, kind)}

    # ---------- placement and mounting ----------
    @tool("Place any USD (file path or URL) in the open Isaac Sim stage the way a person would: units and up axis "
          "fixed, footprint centered on (x, y) in meters, lowered onto the real surface under it (ray cast through the "
          "support's mesh, so lamps or back panels don't lift it). `on` = support prim (table, shelf, pedestal), else "
          "the floor; `z` forces the bottom height. Works inside moved/scaled groups. Check supported_fraction and warning.")
    def isaac_place(self, url: str, prim_path: str, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, on: str = "",
                    z: float = -1.0, physics: str = "auto", use_xy: bool = True) -> dict:
        xy = f"{float(x)}, {float(y)}" if (use_xy or not on) else "None, None"
        zz = "None" if z is None or z < 0 else str(float(z))
        r = self.kit.call(f"place_usd({json.dumps(prim_path)}, {json.dumps(url.replace(chr(92), '/'))}, {xy}, {float(yaw)}, "
                          f"{zz}, {json.dumps(on or None)}, {json.dumps(physics)})", timeout=300)
        if isinstance(r, dict) and r.get("ok"):
            self.session.log({"type": "scene", "what": f"placed {prim_path} from {url.rsplit('/', 1)[-1]} bottom {r.get('bottom_m')} m"
                              + (f" on {on}" if on else "")})
        return r

    @tool("Mount something on a robot link so it moves with it (wrist camera, sensor, tool, gripper adapter). It is "
          "created under the link and aligned to a face of the link's own bounds (`face` +x/-x/+y/-y/+z/-z), touching it "
          "(+ `gap` m); `offset` [x,y,z] m in the link frame, `rpy` [deg] in the item frame. url='camera' makes a USD "
          "camera; `look` (+x, -z, ...) points it along a link axis. Mounted rigid bodies join the link (no nesting).")
    def isaac_mount(self, name: str, url: str, link: str, face: str = "+z", offset: list = None, rpy: list = None,
                    look: str = "", gap: float = 0.0, kind: str = "attach") -> dict:
        off = [float(v) for v in (offset or [0, 0, 0])][:3]
        rp = [float(v) for v in (rpy or [0, 0, 0])][:3]
        u = url if url == "camera" else url.replace(chr(92), "/")
        r = self.kit.call(f"mount({json.dumps(name)}, {json.dumps(u)}, {json.dumps(link)}, {json.dumps(face)}, {off}, {rp}, "
                          f"{json.dumps(look or None)}, {float(gap)}, {json.dumps(kind)})", timeout=120)
        if isinstance(r, dict) and r.get("ok"):
            self.session.log({"type": "scene", "what": f"mounted {name} on {link} face {face}" + (f" looking {look}" if look else "")})
        return r

    @tool("Find and fix setups that make PhysX unstable or crash in the open stage: CCD on a GPU scene, articulation roots nested "
          "inside another robot (e.g. an xArm gripper), limp drives (stiffness 0 and damping 0), links lighter than 20 g, low solver "
          "iterations, self-collision. apply=false previews the changes; apply=true writes them as overrides in the scene file "
          "(referenced robot assets stay untouched) after a backup. Save the stage afterwards to keep them.")
    def isaac_physics_repair(self, scope: str = "/World", apply: bool = False, mass_floor: float = 0.02) -> dict:
        if apply and self.runner.approver.decide("isaac-sim", f"physics repair under {scope}", "writes overrides into the open scene") != "run":
            return {"ok": False, "skipped": True}
        pre = "_bk = _backup_stage('physics-repair')\n" if apply else ""
        r = self.kit.call(f"physics_repair({json.dumps(scope)}, apply={bool(apply)}, mass_floor={float(mass_floor)})", timeout=120)
        if apply and isinstance(r, dict):
            try:
                r["backup"] = self.kit.call("_backup_stage('after-physics-repair')", timeout=60).get("path")
            except Exception:
                pass
            self.session.log({"type": "scene", "what": f"physics repair under {scope}: {r.get('summary')}"})
        return r

    @tool("Audit placement in the open stage: for each object `depth` levels under `scope`, is its bottom resting on "
          "the real surface below it? Reports floating (gap > 1 cm) and sunk objects with exact gaps. Read-only, time-bounded.")
    def isaac_placement_audit(self, scope: str = "/World", depth: int = 2, max_items: int = 120) -> dict:
        return self.kit.call(f"placement_audit({json.dumps(scope)}, {int(depth)}, {int(max_items)})", timeout=120)

    # ---------- SimReady Central (NVIDIA's index of simulation-ready OpenUSD assets on Hugging Face) ----------
    @tool("Search SimReady Central for simulation-ready USD assets (physics, materials, real-world scale). Returns name, "
          "url, dataset, revision, real size in meters [x,y,z] and a thumbnail. Optional max_size_m/min_size_m filter "
          "the largest side; dataset restricts to one HF dataset, e.g. nvidia/simready-assets.")
    def simready_search(self, query: str, limit: int = 8, max_size_m: float = 0.0, min_size_m: float = 0.0, dataset: str = "") -> dict:
        from . import simready
        return simready.search(query, limit, max_size_m or None, min_size_m or None, dataset)

    @tool("Download a SimReady asset's whole package (USD, physics sidecar, materials, license, validation reports) at "
          "the revision the index pinned, into the local cache. Returns local_usd and the license.")
    def simready_fetch(self, url: str) -> dict:
        from . import simready
        return simready.fetch(url)

    @tool("Place a SimReady asset (url from simready_search, fetched automatically) or any local USD into the open "
          "Isaac Sim stage at metric scale. Rests on the floor at (x, y) or on top of prim `on` (e.g. /World/Kitchen/table; "
          "x/y default to its center). yaw in degrees. physics: auto (keep the asset's own), dynamic, static, none.")
    def simready_place(self, url: str, prim_path: str, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, on: str = "",
                       physics: str = "auto", use_xy: bool = True) -> dict:
        from . import simready
        usd = simready.local(url).replace("\\", "/")
        xy = f"{float(x)}, {float(y)}" if (use_xy or not on) else "None, None"
        r = self.kit.call(f"place_usd({json.dumps(prim_path)}, {json.dumps(usd)}, {xy}, {float(yaw)}, None, "
                          f"{json.dumps(on or None)}, {json.dumps(physics)})", timeout=300)
        if isinstance(r, dict):
            r["local_usd"] = usd
            if r.get("ok"):
                self.session.log({"type": "scene", "what": f"placed SimReady {usd.rsplit('/', 1)[-1]} at {prim_path}, bottom {r.get('bottom_m')} m"
                                  + (f" on {on}" if on else "")})
        return r

    @tool("List SimReady packages already downloaded on this machine, with source revision and license.")
    def simready_catalog(self) -> dict:
        from . import simready
        return {"ok": True, "cache": str(simready.cache_root()), "assets": simready.catalog()}

    @tool("Play and record the viewport to an MP4 (needs ffmpeg) for verification videos.")
    def isaac_record(self, seconds: float = 4.0, fps: int = 15, name: str = "") -> dict:
        folder = self._out(f".simwright/captures/{name or time.strftime('%Y%m%d-%H%M%S')}")
        r = self.kit.call(f"await record({json.dumps(str(folder).replace(chr(92), '/'))}, {float(seconds)}, {int(fps)})",
                          timeout=seconds * 60 + 120)
        if r.get("ok"):
            r["video"] = frames_to_mp4(str(folder), fps)
        return r

    @tool("Search the Isaac asset library (robots, props, environments) by keywords, e.g. ['chair'] or ['carter'].")
    def isaac_asset_search(self, keywords: list) -> dict:
        return self.kit.call(f"asset_search({json.dumps(keywords)})", timeout=300)

    @tool("Compose a whole room and (apply=true) build it in Isaac Sim with real assets. spec = {\"room\": {\"width\": m, \"depth\": m, \"height\": m, \"doors\": [{\"wall\": \"south|north|east|west\", \"offset\": m, \"width\": m}]}, \"items\": [{\"kind\": K, \"count\": n, \"with\": [\"chair\", \"monitor\"] (desks), \"chairs\": n (tables), \"against_wall\": bool}], \"robot\": {\"name\": str, \"footprint\": [w, d], \"on\": \"floor|desk\", \"search\": [keywords], \"usd\": url}}. K in desk, chair, monitor, table, round_table, shelf, rack, cabinet, sofa, plant, counter, fridge, workbench, pallet, box, conveyor. Chairs face desks, screens face sitters, aisles 0.9 m, doorways clear, robot on the freest floor.")
    def world_compose(self, spec: dict, name: str = "world", apply: bool = True) -> dict:
        problems = world.validate_spec(spec)
        if problems:
            return {"ok": False, "error": "bad spec: " + "; ".join(problems)}
        layout = world.solve(spec)
        out = self._abs(f".simwright/worlds/{name}.json")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(layout, indent=2))
        self.session.log({"type": "write", "path": self._rel(out), "why": "world layout"})
        res = {"ok": layout["checks"]["ok"], "layout_file": self._rel(out), "checks": layout["checks"],
               "unplaced": layout["unplaced"], "placements": len(layout["placements"]), "map": world.ascii_map(layout)}
        if apply:
            if self.runner.approver.decide("isaac-sim", f"build layout '{name}' ({len(layout['placements'])} items)", "") == "run":
                res["isaac"] = self.kit.call(f"build_layout({json.dumps(layout)})", timeout=900)
        return res

    # ---------- Isaac Lab ----------
    @tool("List registered Isaac Lab tasks (gym ids), optionally filtered by keyword.")
    def lab_tasks(self, keyword: str = "") -> dict:
        return {"ok": True, "tasks": labmod.Lab(self.cfg).list_tasks(self.runner, keyword)}

    @tool("Start Isaac Lab training in the background (RSL-RL by default). Returns a job id; monitor with lab_jobs.")
    def lab_train(self, task: str, num_envs: int = 0, max_iterations: int = 0, headless: bool = True, seed: int = -1, extra_args: str = "") -> dict:
        L = labmod.Lab(self.cfg)
        L.require()
        cmd = L.train_cmd(task, num_envs or None, max_iterations or None, headless, None if seed < 0 else seed, extra_args)
        return labmod.start_job(self.runner, cmd, "host", str(L.dir), name=f"train {task}", meta={"task": task, "kind": "train"})

    @tool("Status of training jobs: iteration, mean reward curve, errors, alive.")
    def lab_jobs(self, job_id: str = "") -> dict:
        return {"ok": True, "jobs": [{k: v for k, v in j.items() if k != "cmd"} for j in labmod.job_status(job_id or None)]}

    @tool("Stop a running training job.")
    def lab_stop(self, job_id: str) -> dict:
        if self.runner.approver.decide("host", f"stop job {job_id}", "") != "run":
            return {"ok": False, "skipped": True}
        return {"ok": labmod.stop_job(job_id)}

    @tool("List Isaac Lab runs on disk (logs/<library>/<experiment>/<run>) with checkpoints and exported files.")
    def lab_runs(self) -> dict:
        return {"ok": True, "runs": labmod.Lab(self.cfg).runs()}

    @tool("Play a trained policy in Isaac Lab (visual evaluation; video=true records a clip headless). Also exports policy.pt/onnx in Isaac Lab 2.x.")
    def lab_play(self, task: str, checkpoint: str = "", num_envs: int = 16, video: bool = False) -> dict:
        L = labmod.Lab(self.cfg)
        L.require()
        return labmod.start_job(self.runner, L.play_cmd(task, checkpoint or None, num_envs, video), "host", str(L.dir),
                                name=f"play {task}", meta={"task": task, "kind": "play"})

    @tool("Export a checkpoint for deployment (Isaac Lab 3.x LEAPP export: ONNX/JIT + IO metadata).")
    def lab_export(self, task: str, checkpoint: str) -> dict:
        L = labmod.Lab(self.cfg)
        L.require()
        r = self.runner.run(L.export_cmd(task, checkpoint), "host", str(L.dir), description="export policy", timeout=3600)
        return r.brief(60)

    @tool("Register a checkpoint as a named policy in the local registry (keeps task, run, exported files, notes, metrics).")
    def policy_register(self, name: str, task: str, checkpoint: str, notes: str = "", metrics: dict = None) -> dict:
        return {"ok": True, **labmod.register_policy(name, task, checkpoint, notes, metrics or {})}

    @tool("List registered policies.")
    def policy_list(self) -> dict:
        return {"ok": True, "policies": labmod.policies()}

    @tool("Deploy a registered policy's exported bundle to a folder (e.g. a ROS 2 package share dir or a robot sync folder) with a manifest.")
    def policy_deploy(self, name: str, dest: str) -> dict:
        if self.runner.approver.decide("file", f"deploy policy {name} -> {dest}", "") != "run":
            return {"ok": False, "skipped": True}
        return labmod.deploy_policy(name, str(self._abs(dest)))

    # ---------- skills & context ----------
    @tool("List available skills (team, user, NVIDIA Isaac skills, built-in) with descriptions.")
    def skill_list(self) -> dict:
        return {"ok": True, "skills": [{"name": s.name, "scope": s.scope, "description": s.description[:200]} for s in skills.discover(self.root)]}

    @tool("Read a skill's full SKILL.md before following it.")
    def skill_read(self, name: str) -> dict:
        for s in skills.discover(self.root):
            if s.name == name:
                return {"ok": True, "name": name, "scope": s.scope, "path": str(s.path), "text": s.body()[:30000]}
        return {"ok": False, "error": f"no skill named {name}"}

    @tool("Install a skill from a markdown file or folder. repo=true installs into <repo>/.skills so teammates get it.")
    def skill_add(self, path: str, repo: bool = False) -> dict:
        s = skills.add(str(self._abs(path)), self.root, repo)
        return {"ok": True, "name": s.name, "installed_at": str(s.path), "scope": s.scope}

    @tool("Check this machine: GPU/driver, WSL distros, Docker (+GPU), Isaac Sim, Isaac Lab, ROS 2 in the ROS target, Mana Bridge.")
    def doctor(self, deep: bool = False) -> dict:
        from .doctor import run_doctor
        return run_doctor(self.runner, deep)
