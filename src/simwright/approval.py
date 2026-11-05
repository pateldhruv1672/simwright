"""Command approval: every command is shown before it runs.

Modes
  auto  run uninterrupted; ESC pauses and switches to step mode
  step  ask before each command (Enter = run, s = skip, a = back to auto)
  ask   like step, and also re-asks for read-only commands
Dangerous shapes (rm -rf, Remove-Item -Recurse, git reset --hard, docker rm, ...)
always ask unless the caller passed allow_dangerous.
"""
from __future__ import annotations
import os, re, sys, threading, time

DANGEROUS = [
    r"\brm\s+-[a-z]*r[a-z]*f", r"\brm\s+-[a-z]*f[a-z]*r", r"Remove-Item\b.*-Recurse", r"\brmdir\s+/s",
    r"\bdel\s+/[sq]", r"\bformat\s+[a-z]:", r"\bgit\s+reset\s+--hard", r"\bgit\s+clean\s+-[a-z]*f",
    r"\bgit\s+push\b.*--force", r"\bdocker\s+(rm|rmi|system\s+prune|volume\s+rm)\b", r"\bmkfs\b",
    r"\bdd\s+if=", r"\bshutdown\b", r"\bwsl(\.exe)?\s+--unregister", r"\bsudo\s+rm\b", r">\s*/dev/sd",
]
READ_ONLY = [
    r"^(ls|dir|cat|type|head|tail|pwd|echo|whoami|hostname|uname|printenv|env|which|where|Get-ChildItem|Get-Content|Get-Command|Test-Path)\b",
    r"^git\s+(status|log|diff|show|branch|remote)\b", r"^nvidia-smi\b", r"^docker\s+(ps|images|info|inspect|logs)\b",
    r"^ros2\s+(node|topic|service|param|action|interface|pkg)\s+(list|info|type|find|show|describe|get|hz|echo)\b",
    r"^ros2\s+control\s+list_", r"^ros2\s+doctor\b", r"^colcon\s+list\b", r"^wsl(\.exe)?\s+(-l|--list|--status)\b",
]


def is_dangerous(cmd: str) -> bool:
    return any(re.search(p, cmd, re.I) for p in DANGEROUS)


def is_read_only(cmd: str) -> bool:
    first = cmd.strip()
    return bool(first) and not re.search(r"[>|;&]\s*\S", first.replace("2>/dev/null", "")) and any(re.search(p, first, re.I) for p in READ_ONLY)


class KeyWatcher:
    """Background ESC detector, active only inside `with watcher.listening():`."""

    def __init__(self):
        self.escape = threading.Event()
        self._stop = threading.Event()
        self._thread = None

    def _loop_windows(self):
        import msvcrt
        while not self._stop.is_set():
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch == "\x1b":
                    self.escape.set()
            time.sleep(0.05)

    def _loop_posix(self):
        import select, termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.1)
                if r and os.read(fd, 1) == b"\x1b":
                    self.escape.set()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def listening(self):
        watcher = self

        class _Ctx:
            def __enter__(self_inner):
                if not sys.stdin or not sys.stdin.isatty():
                    return watcher
                watcher._stop.clear()
                target = watcher._loop_windows if os.name == "nt" else watcher._loop_posix
                watcher._thread = threading.Thread(target=target, daemon=True)
                watcher._thread.start()
                return watcher

            def __exit__(self_inner, *exc):
                watcher._stop.set()
                if watcher._thread:
                    watcher._thread.join(timeout=1)
                    watcher._thread = None
                return False

        return _Ctx()


class Approver:
    """Terminal approver. Other front ends (Isaac Sim panel, MCP) subclass decide()."""

    def __init__(self, mode: str = "auto", interactive: bool | None = None):
        self.mode = mode
        self.interactive = sys.stdin.isatty() if interactive is None else interactive
        self.keys = KeyWatcher()

    def decide(self, target: str, cmd: str, description: str = "", allow_dangerous: bool = False) -> str:
        """Return 'run' or 'skip'. Always prints the command first."""
        label = f" ({description})" if description else ""
        print(f"\n\u25b6 [{target}] {cmd}{label}", flush=True)
        if self.keys.escape.is_set():
            self.keys.escape.clear()
            self.mode = "step"
            print("  paused (ESC): step mode. Enter = run, s = skip, a = resume auto", flush=True)
        danger = is_dangerous(cmd) and not allow_dangerous
        need = danger or self.mode == "ask" or (self.mode == "step" and not is_read_only(cmd))
        if not need:
            return "run"
        if not self.interactive:
            print("  refused: needs approval and no terminal is attached" + (" (dangerous command)" if danger else ""))
            return "skip"
        prompt = "  DANGEROUS. type 'yes' to run, anything else skips: " if danger else "  [Enter] run  [s] skip  [a] auto > "
        ans = input(prompt).strip().lower()
        if danger:
            return "run" if ans == "yes" else "skip"
        if ans == "a":
            self.mode = "auto"
            return "run"
        return "skip" if ans == "s" else "run"


class AutoApprover(Approver):
    """Non-interactive (CI, MCP): runs everything except dangerous commands."""

    def __init__(self):
        super().__init__("auto", interactive=False)

    def decide(self, target, cmd, description="", allow_dangerous=False):
        print(f"\u25b6 [{target}] {cmd}", file=sys.stderr, flush=True)
        if is_dangerous(cmd) and not allow_dangerous:
            return "skip"
        return "run"


class QueueApprover(Approver):
    """Approvals answered remotely (the Isaac Sim panel) through `answer()`."""

    def __init__(self, mode="step"):
        super().__init__(mode, interactive=False)
        self.pending: dict[str, dict] = {}
        self._lock = threading.Lock()
        self._n = 0
        self.events: list[dict] = []

    def decide(self, target, cmd, description="", allow_dangerous=False):
        if self.mode == "auto" and not is_dangerous(cmd):
            self.events.append({"type": "command", "target": target, "cmd": cmd})
            return "run"
        if is_read_only(cmd) and not is_dangerous(cmd):
            self.events.append({"type": "command", "target": target, "cmd": cmd})
            return "run"
        with self._lock:
            self._n += 1
            pid = f"p{self._n}"
            ev = threading.Event()
            self.pending[pid] = {"id": pid, "target": target, "cmd": cmd, "description": description,
                                 "dangerous": is_dangerous(cmd), "event": ev, "answer": None}
        self.events.append({"type": "approval", "id": pid, "target": target, "cmd": cmd, "description": description})
        ev.wait(timeout=600)
        item = self.pending.pop(pid, {})
        return "run" if item.get("answer") == "run" else "skip"

    def answer(self, pid: str, decision: str) -> bool:
        item = self.pending.get(pid)
        if not item:
            return False
        item["answer"] = "run" if decision in ("run", "approve", "yes") else "skip"
        item["event"].set()
        return True
