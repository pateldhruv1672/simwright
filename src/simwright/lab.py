"""Isaac Lab: tasks, training jobs, runs, policy registry, export and deploy.

Works with Isaac Lab 2.x and 3.x. Script paths are configurable in ~/.simwright/config.json -> lab.
"""
from __future__ import annotations
import json, os, pathlib, re, shutil, subprocess, sys, time, uuid
from . import config
from .targets import Runner, Target

JOBS = "jobs.json"
POLICIES = "policies.json"


def _ver(text: str) -> str:
    m = re.search(r"(\d+\.\d+\.\d+)", text or "")
    return m.group(1) if m else ""


def _lab_version(d: pathlib.Path) -> str:
    for f in (d / "VERSION", d / "source" / "isaaclab" / "config" / "extension.toml"):
        if f.exists():
            v = _ver(f.read_text(errors="ignore"))
            if v:
                return v
    return ""


def conda_env_dirs() -> list[pathlib.Path]:
    """Every conda env we can see without running conda (fast, read-only)."""
    home = pathlib.Path.home()
    roots = [home / ".conda" / "envs", home / "anaconda3" / "envs", home / "miniconda3" / "envs",
             home / "miniforge3" / "envs", home / "mambaforge" / "envs",
             pathlib.Path(r"C:\ProgramData\anaconda3\envs"), pathlib.Path(r"C:\ProgramData\miniconda3\envs")]
    for var in ("CONDA_EXE", "CONDA_PREFIX"):
        v = os.environ.get(var)
        if v:
            base = pathlib.Path(v).parent.parent if var == "CONDA_EXE" else pathlib.Path(v)
            roots += [base / "envs", base.parent]
    seen, out = set(), []
    for r in roots:
        try:
            kids = [k for k in r.iterdir() if k.is_dir()] if r.is_dir() else []
        except OSError:
            kids = []
        for k in kids:
            key = str(k).lower()
            if key not in seen and _env_python(k):
                seen.add(key)
                out.append(k)
    return out


def _env_python(env: pathlib.Path) -> pathlib.Path | None:
    for c in (env / "python.exe", env / "bin" / "python"):
        if c.exists():
            return c
    return None


def _site_packages(env: pathlib.Path) -> list[pathlib.Path]:
    return [env / "Lib" / "site-packages"] + sorted((env / "lib").glob("python3*/site-packages"))


def env_packages(env: pathlib.Path) -> dict:
    """isaaclab source dir + isaacsim version installed in a conda env, from dist-info metadata."""
    info = {"env": env.name, "prefix": str(env), "python": str(_env_python(env) or "")}
    for sp in _site_packages(env):
        if not sp.is_dir():
            continue
        for di in sp.glob("isaacsim-*.dist-info"):
            info.setdefault("isaacsim", _ver(di.name))
        for di in list(sp.glob("isaaclab-*.dist-info")) + list(sp.glob("isaaclab-*.egg-info")):
            du = di / "direct_url.json"
            if du.exists():
                try:
                    url = json.loads(du.read_text()).get("url", "")
                    src = urllib_path(url)
                    if src:
                        info["isaaclab_src"] = src
                except Exception:
                    pass
            info.setdefault("isaaclab_pkg", _ver(di.name))
        for egg in sp.glob("isaaclab*.egg-link"):
            info.setdefault("isaaclab_src", egg.read_text(errors="ignore").splitlines()[0].strip())
    return info


def urllib_path(url: str) -> str:
    if not url.startswith("file:"):
        return ""
    from urllib.parse import urlparse, unquote
    u = urlparse(url)
    path = unquote(u.path)
    if re.match(r"^/[A-Za-z]:", path):
        path = path[1:]
    return path


def _same(a: str, b: pathlib.Path) -> bool:
    try:
        return pathlib.Path(a).resolve() == b.resolve() or str(pathlib.Path(a).resolve()).lower().startswith(str(b.resolve()).lower() + os.sep)
    except OSError:
        return False


def discover(cfg: dict | None = None) -> list[dict]:
    """All Isaac Lab checkouts on this machine, each with the conda env (if any) that installed it."""
    cfg = cfg or config.load()
    home = pathlib.Path.home()
    launcher = "isaaclab.bat" if os.name == "nt" else "isaaclab.sh"
    cands = [cfg.get("isaac_lab_dir"), os.environ.get("ISAAC_LAB_DIR")]
    for base in (home, home / "robotics", home / "code", home / "src", home / "projects", home / "dev",
                 home / "Documents", pathlib.Path.cwd(), pathlib.Path(r"C:\\")):
        try:
            cands += [str(d) for d in base.glob("*[Ii]saac*[Ll]ab*") if d.is_dir()]
        except OSError:
            pass
    envs = [env_packages(e) for e in conda_env_dirs()]
    out, seen = [], set()
    for c in cands:
        if not c:
            continue
        d = pathlib.Path(c)
        if not (d / launcher).exists() or str(d.resolve()).lower() in seen:
            continue
        seen.add(str(d.resolve()).lower())
        env = next((e for e in envs if e.get("isaaclab_src") and _same(e["isaaclab_src"], d)), None)
        link = d / "_isaac_sim"
        out.append({"dir": str(d), "version": _lab_version(d), "env": env["env"] if env else "",
                    "python": env["python"] if env else "", "isaacsim": (env or {}).get("isaacsim", ""),
                    "isaac_sim_link": str(link.resolve()) if link.exists() else "",
                    "broken_hint": "broken" in d.name.lower()})
    return out


def _major(v: str) -> int:
    try:
        return int(v.split(".")[0])
    except (ValueError, IndexError):
        return -1


def choose(installs: list[dict], isaac_sim_version: str = "") -> dict | None:
    """Isaac Lab 3.x pairs with Isaac Sim 6.x, 2.x with 5.x. Prefer a matching, installed (env found), non-broken checkout."""
    if not installs:
        return None
    want = {6: 3, 5: 2, 4: 1}.get(_major(isaac_sim_version), None)
    def score(i):
        return ((want is not None and _major(i["version"]) == want), bool(i["python"] or i["isaac_sim_link"]),
                not i["broken_hint"], tuple(int(x) for x in (i["version"] or "0.0.0").split(".")))
    return sorted(installs, key=score)[-1]


class Lab:
    def __init__(self, cfg: dict | None = None):
        self.cfg = cfg or config.load()
        self.lib = self.cfg["lab"]["rl_library"]
        self.python = self.cfg.get("isaac_lab_python") or ""
        self.dir = self._find()

    def _find(self) -> pathlib.Path | None:
        launcher = "isaaclab.bat" if os.name == "nt" else "isaaclab.sh"
        explicit = self.cfg.get("isaac_lab_dir") or os.environ.get("ISAAC_LAB_DIR")
        if explicit and (pathlib.Path(explicit) / launcher).exists():
            if not self.python:
                hit = next((i for i in discover(self.cfg) if _same(i["dir"], pathlib.Path(explicit))), None)
                self.python = (hit or {}).get("python", "")
            return pathlib.Path(explicit)
        try:
            from .isaac import find_isaac_sim
            simv = (find_isaac_sim(self.cfg) or {}).get("version", "")
        except Exception:
            simv = ""
        pick = choose(discover(self.cfg), simv)
        if not pick:
            return None
        self.python = self.python or pick["python"]
        return pathlib.Path(pick["dir"])

    def require(self):
        if not self.dir:
            raise RuntimeError("Isaac Lab not found. `simwright config isaac_lab_dir C:\\path\\to\\IsaacLab`")

    @property
    def launcher(self) -> str:
        return str(self.dir / ("isaaclab.bat" if os.name == "nt" else "isaaclab.sh"))

    def version(self) -> str:
        return _lab_version(self.dir) if self.dir else ""

    def script(self, key: str) -> str:
        return self.cfg["lab"][key].format(lib=self.lib)

    def _cmd(self, key: str, args: str) -> str:
        prefix = "& " if os.name == "nt" else ""
        script = str(self.dir / self.script(key))
        script = f'"{script}"' if " " in script else script
        if self.python:  # the conda env that installed this checkout: same as `isaaclab -p` with it activated
            return f'{prefix}"{self.python}" {script} {args}'.strip()
        q = f'"{self.launcher}"' if " " in self.launcher else self.launcher
        return f"{prefix}{q} -p {script} {args}".strip()

    def train_cmd(self, task: str, num_envs: int | None = None, max_iterations: int | None = None,
                  headless: bool = True, seed: int | None = None, extra: str = "") -> str:
        a = f"--task {task}"
        a += " --headless" if headless else ""
        a += f" --num_envs {num_envs}" if num_envs else ""
        a += f" --max_iterations {max_iterations}" if max_iterations else ""
        a += f" --seed {seed}" if seed is not None else ""
        return self._cmd("train", (a + " " + extra).strip())

    def play_cmd(self, task: str, checkpoint: str | None = None, num_envs: int = 16, video: bool = False, extra: str = "") -> str:
        a = f"--task {task} --num_envs {num_envs}" + (f' --checkpoint "{checkpoint}"' if checkpoint else "")
        a += " --video --video_length 300 --headless" if video else ""
        return self._cmd("play", (a + " " + extra).strip())

    def export_cmd(self, task: str, checkpoint: str, extra: str = "") -> str:
        return self._cmd("export", f'--task {task} --checkpoint "{checkpoint}" --headless {extra}'.strip())

    def list_tasks(self, runner: Runner, keyword: str = "") -> list[str]:
        self.require()
        r = runner.capture(self._cmd("list_envs", ""), "host", str(self.dir), timeout=900, description="list Isaac Lab tasks")
        tasks = sorted(set(re.findall(r"\b((?:Isaac|Template)-[\w\-]+-v\d+)\b", r.out)))
        return [t for t in tasks if keyword.lower() in t.lower()]

    # ---------- runs ----------
    def runs(self, limit: int = 30) -> list[dict]:
        if not self.dir:
            return []
        out = []
        logs = self.dir / "logs"
        for lib_dir in (logs.iterdir() if logs.is_dir() else []):
            if not lib_dir.is_dir():
                continue
            for exp in lib_dir.iterdir():
                for run in (exp.iterdir() if exp.is_dir() else []):
                    if not run.is_dir():
                        continue
                    ckpts = sorted(run.glob("model_*.pt"), key=lambda p: int(re.sub(r"\D", "", p.stem) or 0))
                    exported = sorted(str(p.relative_to(run)) for p in run.rglob("*") if p.suffix in (".onnx", ".pt") and "export" in str(p.parent).lower())
                    env_yaml = run / "params" / "env.yaml"
                    task = ""
                    agent = run / "params" / "agent.yaml"
                    if agent.exists():
                        m = re.search(r"experiment_name:\s*(\S+)", agent.read_text(errors="ignore"))
                        task = m.group(1) if m else ""
                    out.append({"library": lib_dir.name, "experiment": exp.name, "run": run.name, "path": str(run),
                                "checkpoints": len(ckpts), "latest": str(ckpts[-1]) if ckpts else "",
                                "latest_iter": int(re.sub(r"\D", "", ckpts[-1].stem) or 0) if ckpts else 0,
                                "exported": exported[:10], "has_env_yaml": env_yaml.exists(),
                                "modified": max((p.stat().st_mtime for p in run.iterdir()), default=run.stat().st_mtime)})
        out.sort(key=lambda r: -r["modified"])
        return out[:limit]


# ---------- background jobs (training) ----------
def _alive(pid: int) -> bool:
    if os.name == "nt":
        import ctypes
        h = ctypes.windll.kernel32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
        if not h:
            return False
        code = ctypes.c_ulong()
        ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
        ctypes.windll.kernel32.CloseHandle(h)
        return code.value == 259  # STILL_ACTIVE
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def start_job(runner: Runner, cmd: str, target: str = "host", cwd: str | None = None, name: str = "", meta: dict | None = None) -> dict:
    t = Target.get(target)
    if runner.approver.decide(t.key, cmd, f"background job: {name}") != "run":
        return {"ok": False, "skipped": True}
    jid = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:4]
    log = config.state_file(f"jobs/{jid}.log")
    log.parent.mkdir(parents=True, exist_ok=True)
    f = open(log, "wb")
    flags = (0x00000200 | 0x08000000) if os.name == "nt" else 0  # new process group, no window
    proc = subprocess.Popen(t.argv(cmd, cwd), stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            cwd=cwd if t.kind == "host" else None, creationflags=flags,
                            start_new_session=(os.name != "nt"))
    job = {"id": jid, "name": name, "cmd": cmd, "target": target, "cwd": cwd, "pid": proc.pid, "log": str(log),
           "started": time.time(), **(meta or {})}
    jobs = config.read_state(JOBS, []) or []
    jobs.append(job)
    config.write_state(JOBS, jobs[-200:])
    if runner.log:
        runner.log({"type": "job", "id": jid, "cmd": cmd})
    return {"ok": True, **job}


def job_status(jid: str | None = None) -> list[dict]:
    jobs = config.read_state(JOBS, []) or []
    if jid:
        jobs = [j for j in jobs if j["id"].startswith(jid)]
    out = []
    for j in jobs[-20:]:
        text = pathlib.Path(j["log"]).read_text(errors="ignore") if pathlib.Path(j["log"]).exists() else ""
        it = re.findall(r"Learning iteration\s+(\d+)\s*/\s*(\d+)", text)
        rew = re.findall(r"Mean reward:\s*([-\d.eE+]+)", text)
        eplen = re.findall(r"Mean episode length:\s*([-\d.eE+]+)", text)
        run_dir = re.findall(r"\[INFO\] Logging experiment in directory:\s*(\S+)", text) + re.findall(r"Exact experiment name requested from command line:\s*(\S+)", text)
        err = re.findall(r"(?m)^(\w*Error: .+)$", text)
        alive = _alive(j["pid"])
        rewards = [float(x) for x in rew[-200:]]
        out.append({**j, "alive": alive, "iteration": int(it[-1][0]) if it else None, "max_iterations": int(it[-1][1]) if it else None,
                    "mean_reward": rewards[-1] if rewards else None, "reward_curve": rewards[-50:],
                    "mean_episode_length": float(eplen[-1]) if eplen else None, "run_hint": run_dir[-1] if run_dir else "",
                    "last_error": err[-1][:300] if err else "", "state": "running" if alive else ("failed" if err else "finished"),
                    "tail": "\n".join(text.splitlines()[-15:])})
    return out


def stop_job(jid: str) -> bool:
    for j in config.read_state(JOBS, []) or []:
        if j["id"].startswith(jid) and _alive(j["pid"]):
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(j["pid"]), "/T", "/F"], capture_output=True)
            else:
                import signal
                os.killpg(os.getpgid(j["pid"]), signal.SIGTERM)
            return True
    return False


# ---------- policy registry ----------
def policies() -> list[dict]:
    return config.read_state(POLICIES, []) or []


def register_policy(name: str, task: str, checkpoint: str, notes: str = "", metrics: dict | None = None) -> dict:
    ck = pathlib.Path(checkpoint)
    if not ck.exists():
        raise FileNotFoundError(checkpoint)
    run = ck.parent
    exported = [str(p) for p in run.rglob("*") if p.suffix in (".onnx", ".pt", ".yaml", ".safetensors") and "export" in str(p).lower()]
    entry = {"name": name, "task": task, "checkpoint": str(ck), "run": str(run), "exported": exported, "notes": notes,
             "metrics": metrics or {}, "registered": time.time(), "deployments": []}
    items = [p for p in policies() if p["name"] != name] + [entry]
    config.write_state(POLICIES, items)
    return entry


def deploy_policy(name: str, dest: str) -> dict:
    """Copy a policy's exported bundle (ONNX/JIT + env/IO yaml) to `dest` with a manifest."""
    pol = next((p for p in policies() if p["name"] == name), None)
    if not pol:
        raise KeyError(f"no policy named {name}")
    if not pol["exported"]:
        raise RuntimeError("policy has no exported files yet: run `simwright lab export` first")
    d = pathlib.Path(dest) / name
    d.mkdir(parents=True, exist_ok=True)
    copied = []
    for f in pol["exported"]:
        if pathlib.Path(f).exists():
            shutil.copy2(f, d / pathlib.Path(f).name)
            copied.append(pathlib.Path(f).name)
    env_yaml = pathlib.Path(pol["run"]) / "params" / "env.yaml"
    if env_yaml.exists():
        shutil.copy2(env_yaml, d / "env.yaml")
        copied.append("env.yaml")
    manifest = {"name": name, "task": pol["task"], "source_checkpoint": pol["checkpoint"], "files": copied,
                "deployed": time.strftime("%Y-%m-%dT%H:%M:%S")}
    (d / "policy.json").write_text(json.dumps(manifest, indent=2))
    items = policies()
    for p in items:
        if p["name"] == name:
            p["deployments"].append({"dest": str(d), "at": time.time()})
    config.write_state(POLICIES, items)
    return {"ok": True, "dest": str(d), **manifest}
