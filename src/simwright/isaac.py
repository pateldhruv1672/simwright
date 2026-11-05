"""Talk to a running Isaac Sim through NVIDIA's isaacsim.code_editor.python_server (127.0.0.1:8226),
and launch Isaac Sim with that server plus the simwright.copilot extension enabled."""
from __future__ import annotations
import json, os, pathlib, re, socket, subprocess, sys, time
from . import config

HELPERS = (pathlib.Path(__file__).parent / "kit_helpers.py").read_text(encoding="utf-8")
HELPERS_VERSION = (re.search(r'HELPERS_VERSION = "([^"]+)"', HELPERS) or [None, ""])[1]


class KitError(RuntimeError):
    pass


class KitClient:
    def __init__(self, host="127.0.0.1", port=8226, token="", timeout=120.0):
        self.host, self.port, self.token, self.timeout = host, int(port), token, timeout
        self._raw_only = False
        self._prefix = None

    @classmethod
    def from_config(cls) -> "KitClient":
        cfg = config.load()["python_server"]
        sess = config.read_state("kit_session.json", {}) or {}
        return cls(sess.get("host") or cfg["host"], sess.get("port") or cfg["port"], cfg.get("token") or sess.get("token", ""))

    def _send(self, payload: str, timeout: float) -> dict:
        try:
            s = socket.create_connection((self.host, self.port), timeout=5)
        except OSError as e:
            raise KitError(f"Isaac Sim python_server not reachable at {self.host}:{self.port} ({e}). "
                           "Start Isaac Sim with `simwright isaac launch`, or enable isaacsim.code_editor.python_server.")
        s.settimeout(timeout + 15)
        with s:
            s.sendall(payload.encode("utf-8"))
            s.shutdown(socket.SHUT_WR)
            buf = b""
            while True:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
        if not buf:
            raise KitError("empty reply (authentication required? set python_server.token in ~/.simwright/config.json)")
        return json.loads(buf.decode("utf-8", errors="replace"))

    def exec(self, code: str, timeout: float | None = None, context: str = "simwright") -> dict:
        timeout = timeout or self.timeout
        if not self._raw_only:
            env = {"code": code, "context": context, "timeout": float(timeout)}
            if self.token:
                env["auth_token"] = self.token
            r = self._send(json.dumps(env), timeout)
            res = str(r.get("result", ""))
            if res.startswith("{'code'") or (r.get("status") == "error" and r.get("ename") == "NameError" and "true" in str(r.get("evalue", ""))):
                self._raw_only = True  # older server without the JSON envelope
            else:
                return r
        return self._send(code, timeout)

    def call(self, expr: str, timeout: float | None = None):
        """Evaluate a helper expression (may be awaited) and return its JSON-decoded value."""
        prefix = self.helper_prefix()
        code = prefix + "\nimport json as _j\n_r = " + expr + "\nprint('@@SW ' + _j.dumps(_r, default=str))"
        r = self.exec(code, timeout)
        if r.get("status") != "ok":
            raise KitError(f"{r.get('ename')}: {r.get('evalue')}\n" + "".join(r.get("traceback", []))[-3000:])
        m = re.search(r"@@SW (.*)$", r.get("output", ""), re.S | re.M)
        return json.loads(m.group(1)) if m else r.get("output")

    def helper_prefix(self) -> str:
        """Use the installed simwright.copilot extension's helpers when present, else send them inline."""
        if self._prefix is None:
            r = self.exec("try:\n    import simwright.copilot.helpers\n    print('ext')\nexcept Exception:\n    print('none')", 30)
            self._prefix = ("import importlib, simwright.copilot.helpers as _swh\n"
                            f"if getattr(_swh, 'HELPERS_VERSION', '') != {HELPERS_VERSION!r}:\n    importlib.reload(_swh)\n"
                            "from simwright.copilot.helpers import *") if "ext" in r.get("output", "") else HELPERS
        return self._prefix

    def ping(self) -> dict:
        r = self.exec("import omni.kit.app as _a\nprint(_a.get_app().get_build_version())", timeout=15)
        return {"ok": r.get("status") == "ok", "kit": r.get("output", "").strip()}


def find_isaac_sim(cfg: dict | None = None) -> dict:
    cfg = cfg or config.load()
    cands = [cfg.get("isaac_sim_dir"), os.environ.get("ISAAC_SIM_DIR"),
             r"C:\isaacsim", r"C:\isaac-sim", str(pathlib.Path.home() / "isaacsim"), str(pathlib.Path.home() / "isaac-sim")]
    for c in cands:
        if not c:
            continue
        d = pathlib.Path(c)
        for name in ("isaac-sim.bat", "isaac-sim.sh"):
            if (d / name).exists():
                ver = (d / "VERSION").read_text().strip() if (d / "VERSION").exists() else ""
                return {"kind": "binary", "dir": str(d), "launcher": str(d / name), "version": ver,
                        "skills": str(d / "skills") if (d / "skills").is_dir() else ""}
    exe = config.which("isaacsim")
    if exe:
        return {"kind": "pip", "launcher": exe, "dir": "", "version": "", "skills": ""}
    return {}


def launch_command(headless: bool = False, extra: list[str] | None = None) -> list[str]:
    inst = find_isaac_sim()
    if not inst:
        raise KitError("Isaac Sim not found. Set it with `simwright config isaac_sim_dir <path>` or pip install isaacsim.")
    ext_dir = str(config.DATA / "kit_exts")
    args = ["--enable", "isaacsim.code_editor.python_server", "--ext-folder", ext_dir, "--enable", "simwright.copilot"]
    if headless:
        args += ["--no-window"]
    base = [inst["launcher"]] if inst["kind"] == "binary" else [inst["launcher"], "isaacsim.exp.full.kit"]
    return base + args + (extra or [])


def launch(headless=False, wait=True, timeout=600, extra=None) -> dict:
    argv = launch_command(headless, extra)
    log = config.state_file("isaac-sim.log")
    f = open(log, "wb")
    flags = 0x00000200 if os.name == "nt" else 0  # CREATE_NEW_PROCESS_GROUP
    proc = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, creationflags=flags)
    sess = {"pid": proc.pid, "argv": argv, "log": str(log), "started": time.time(), "host": "127.0.0.1", "port": 8226}
    config.write_state("kit_session.json", sess)
    if not wait:
        return sess
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(2)
        text = log.read_text(errors="ignore")
        m = re.search(r"authentication token:\s*(\S+)", text)
        if m:
            sess["token"] = m.group(1)
        if "app ready" in text.lower() or "isaac sim full app is loaded" in text.lower():
            sess["ready"] = True
            config.write_state("kit_session.json", sess)
            return sess
        if proc.poll() is not None:
            raise KitError(f"Isaac Sim exited with code {proc.returncode}; see {log}")
    sess["ready"] = False
    config.write_state("kit_session.json", sess)
    return sess


def frames_to_mp4(folder: str, fps: int = 15, out: str | None = None) -> dict:
    out = out or str(pathlib.Path(folder).with_suffix(".mp4"))
    ff = config.which("ffmpeg")
    if not ff:
        return {"ok": False, "error": "ffmpeg not on PATH (winget install Gyan.FFmpeg); frames kept", "folder": folder}
    r = subprocess.run([ff, "-y", "-framerate", str(fps), "-i", str(pathlib.Path(folder) / "frame_%05d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", out], capture_output=True, text=True)
    return {"ok": r.returncode == 0, "video": out, "error": r.stderr[-500:] if r.returncode else ""}
