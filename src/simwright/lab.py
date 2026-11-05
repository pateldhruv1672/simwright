"""Isaac Lab: tasks, training jobs, runs, policy registry, export and deploy.

Works with Isaac Lab 2.x and 3.x. Script paths are configurable in ~/.simwright/config.json -> lab.
"""
from __future__ import annotations
import json, os, pathlib, re, shutil, subprocess, sys, time, uuid
from . import config
from .targets import Runner, Target

JOBS = "jobs.json"
POLICIES = "policies.json"


class Lab:
    def __init__(self, cfg: dict | None = None):
        self.cfg = cfg or config.load()
        self.lib = self.cfg["lab"]["rl_library"]
        self.dir = self._find()

    def _find(self) -> pathlib.Path | None:
        for c in [self.cfg.get("isaac_lab_dir"), os.environ.get("ISAAC_LAB_DIR"), r"C:\IsaacLab",
                  str(pathlib.Path.home() / "IsaacLab"), str(pathlib.Path.cwd() / "IsaacLab")]:
            if c and (pathlib.Path(c) / ("isaaclab.bat" if os.name == "nt" else "isaaclab.sh")).exists():
                return pathlib.Path(c)
        return None

    def require(self):
        if not self.dir:
            raise RuntimeError("Isaac Lab not found. `simwright config isaac_lab_dir C:\\path\\to\\IsaacLab`")

    @property
    def launcher(self) -> str:
        return str(self.dir / ("isaaclab.bat" if os.name == "nt" else "isaaclab.sh"))

    def version(self) -> str:
        if not self.dir:
            return ""
        for f in (self.dir / "VERSION", self.dir / "source" / "isaaclab" / "config" / "extension.toml"):
            if f.exists():
                m = re.search(r'(\d+\.\d+\.\d+)', f.read_text())
                if m:
                    return m.group(1)
        return ""

    def script(self, key: str) -> str:
        return self.cfg["lab"][key].format(lib=self.lib)

    def _cmd(self, key: str, args: str) -> str:
        q = f'"{self.launcher}"' if " " in self.launcher else self.launcher
        prefix = "& " if os.name == "nt" else ""
        return f"{prefix}{q} -p {self.script(key)} {args}".strip()

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
