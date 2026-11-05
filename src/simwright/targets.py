"""Execution targets: the Windows host, a WSL distro, or a Docker container.

Every command goes through run(), which shows it, asks the Approver, streams output
and returns a compact result for the agent.
"""
from __future__ import annotations
import os, re, shlex, subprocess, sys, threading, time
from dataclasses import dataclass, field
from . import config
from .approval import Approver


@dataclass
class Target:
    key: str
    kind: str = "host"          # host | wsl | docker
    name: str = ""              # WSL distro or container name
    setup: str = ""             # sourced before every command (ROS environment, venv, ...)

    @classmethod
    def get(cls, key: str | None = None) -> "Target":
        cfg = config.load()
        key = key or "host"
        spec = cfg["targets"].get(key)
        if spec is None:
            raise KeyError(f"unknown target '{key}'. Known: {', '.join(cfg['targets'])}")
        return cls(key=key, kind=spec.get("kind", "host"), name=spec.get("name", ""), setup=spec.get("setup", ""))

    @property
    def posix(self) -> bool:
        return self.kind in ("wsl", "docker") or not config.IS_WINDOWS

    def path(self, p: str | None) -> str | None:
        """Translate a host path for this target (C:\\ws -> /mnt/c/ws inside WSL)."""
        if not p:
            return p
        if self.kind == "wsl":
            return to_wsl_path(p)
        return p

    def argv(self, cmd: str, cwd: str | None = None) -> list[str]:
        cwd = self.path(cwd)
        if self.kind == "host":
            if config.IS_WINDOWS:
                pre = f"Set-Location -LiteralPath '{cwd}'; " if cwd else ""
                shell = "pwsh" if config.which("pwsh") else "powershell"
                return [shell, "-NoLogo", "-NoProfile", "-NonInteractive", "-Command",
                        "$ProgressPreference='SilentlyContinue'; " + pre + cmd]
            pre = f"cd {shlex.quote(cwd)} && " if cwd else ""
            return ["bash", "-lc", (self.setup + "; " if self.setup else "") + pre + cmd]
        body = (self.setup + "; " if self.setup else "") + (f"cd {shlex.quote(cwd)} && " if cwd else "") + cmd
        if self.kind == "wsl":
            return ["wsl.exe"] + (["-d", self.name] if self.name else []) + ["--", "bash", "-lc", body]
        if self.kind == "docker":
            return ["docker", "exec", "-i", self.name, "bash", "-lc", body]
        raise ValueError(f"unknown target kind {self.kind}")


def to_wsl_path(p: str) -> str:
    m = re.match(r"^([A-Za-z]):[\\/](.*)$", p)
    if not m:
        return p.replace("\\", "/")
    return f"/mnt/{m.group(1).lower()}/" + m.group(2).replace("\\", "/")


@dataclass
class Result:
    ok: bool
    exit: int | None
    out: str
    skipped: bool = False
    took_s: float = 0.0
    cmd: str = ""
    target: str = ""

    def brief(self, tail: int = 120) -> dict:
        lines = self.out.splitlines()
        text = "\n".join(lines[-tail:])
        if len(lines) > tail:
            text = f"... ({len(lines) - tail} earlier lines omitted)\n" + text
        return {"ok": self.ok, "exit": self.exit, "skipped": self.skipped, "took_s": round(self.took_s, 2), "output": text}


class Runner:
    def __init__(self, approver: Approver | None = None, echo: bool = True, log=None):
        self.approver = approver or Approver(config.load().get("approval", "auto"))
        self.echo = echo
        self.log = log          # callable(dict) for session memory

    def run(self, cmd: str, target: str | Target | None = None, cwd: str | None = None, description: str = "",
            timeout: float | None = 1800, allow_dangerous: bool = False, env: dict | None = None) -> Result:
        t = target if isinstance(target, Target) else Target.get(target)
        decision = self.approver.decide(t.key, cmd, description, allow_dangerous)
        if decision != "run":
            res = Result(False, None, "skipped by user", skipped=True, cmd=cmd, target=t.key)
            self._log(res)
            return res
        start = time.time()
        argv = t.argv(cmd, cwd)
        try:
            proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                    env={**os.environ, **(env or {})}, cwd=(cwd if t.kind == "host" and cwd else None))
        except FileNotFoundError as e:
            res = Result(False, None, f"cannot start {argv[0]}: {e}", cmd=cmd, target=t.key)
            self._log(res)
            return res
        chunks: list[str] = []

        def pump():
            for raw in iter(proc.stdout.readline, b""):
                line = decode(raw)
                chunks.append(line)
                if self.echo:
                    sys.stdout.write("  " + line)
                    sys.stdout.flush()

        th = threading.Thread(target=pump, daemon=True)
        th.start()
        try:
            code = proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            code = None
            chunks.append(f"\n[simwright] timed out after {timeout}s\n")
        th.join(timeout=2)
        res = Result(code == 0, code, "".join(chunks), cmd=cmd, target=t.key, took_s=time.time() - start)
        self._log(res)
        return res

    def capture(self, cmd: str, target=None, cwd=None, timeout=120, description="") -> Result:
        """Run quietly (still shown and approved) and return output."""
        echo, self.echo = self.echo, False
        try:
            return self.run(cmd, target, cwd, description=description, timeout=timeout)
        finally:
            self.echo = echo

    def _log(self, res: Result):
        if self.log:
            self.log({"type": "command", "target": res.target, "cmd": res.cmd, "exit": res.exit, "skipped": res.skipped})


def decode(raw: bytes) -> str:
    """wsl.exe prints UTF-16LE for its own messages; everything else is UTF-8 (or the console codepage)."""
    if raw.count(b"\x00") > len(raw) // 3:
        try:
            return raw.decode("utf-16-le", errors="replace").replace("\x00", "")
        except Exception:
            pass
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("cp1252", errors="replace")
