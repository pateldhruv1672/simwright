"""User configuration (~/.simwright/config.json) and well-known paths."""
from __future__ import annotations
import json, os, pathlib, shutil, sys

HOME = pathlib.Path(os.environ.get("SIMWRIGHT_HOME", pathlib.Path.home() / ".simwright"))
IS_WINDOWS = os.name == "nt"
DATA = pathlib.Path(__file__).resolve().parent / "_data"
if not DATA.exists():  # running from a source checkout
    DATA = pathlib.Path(__file__).resolve().parents[2]

DEFAULTS = {
    # Where commands run. "host" = this machine (PowerShell on Windows, bash elsewhere).
    "targets": {
        "host": {"kind": "host"},
        "wsl": {"kind": "wsl", "name": "", "setup": "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true"},
        "ros": {"kind": "docker", "name": "ros2_dev", "setup": "source /opt/ros/jazzy/setup.bash 2>/dev/null || true"},
    },
    "ros_target": "wsl" if IS_WINDOWS else "host",
    "build_target": "wsl" if IS_WINDOWS else "host",
    "isaac_target": "host",
    "isaac_sim_dir": os.environ.get("ISAAC_SIM_DIR", ""),
    "isaac_lab_dir": os.environ.get("ISAAC_LAB_DIR", ""),
    "python_server": {"host": "127.0.0.1", "port": 8226, "token": ""},
    "approval": "auto",            # auto | step | ask
    "model": os.environ.get("SIMWRIGHT_MODEL", "gpt-5.4"),
    "base_url": os.environ.get("SIMWRIGHT_BASE_URL", ""),
    "serve_port": 8787,
    "lab": {
        "rl_library": "rsl_rl",
        "train": "scripts/reinforcement_learning/{lib}/train.py",
        "play": "scripts/reinforcement_learning/{lib}/play.py",
        "export": "scripts/reinforcement_learning/leapp/{lib}/export.py",
        "list_envs": "scripts/environments/list_envs.py",
    },
}


def _merge(a: dict, b: dict) -> dict:
    out = dict(a)
    for k, v in b.items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def load() -> dict:
    p = HOME / "config.json"
    cfg = DEFAULTS
    if p.exists():
        try:
            cfg = _merge(DEFAULTS, json.loads(p.read_text(encoding="utf-8")))
        except Exception as e:  # keep working with defaults, but say so
            print(f"[simwright] ignoring unreadable {p}: {e}", file=sys.stderr)
    return cfg


def save(cfg: dict) -> None:
    HOME.mkdir(parents=True, exist_ok=True)
    (HOME / "config.json").write_text(json.dumps(cfg, indent=2), encoding="utf-8")


def set_value(dotted: str, value) -> dict:
    cfg = load()
    node = cfg
    keys = dotted.split(".")
    for k in keys[:-1]:
        node = node.setdefault(k, {})
    node[keys[-1]] = value
    save(cfg)
    return cfg


def state_file(name: str) -> pathlib.Path:
    HOME.mkdir(parents=True, exist_ok=True)
    return HOME / name


def read_state(name: str, default=None):
    p = state_file(name)
    if p.exists():
        try:
            return json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            return default
    return default


def write_state(name: str, data) -> None:
    state_file(name).write_text(json.dumps(data, indent=2), encoding="utf-8")


def which(cmd: str) -> str | None:
    return shutil.which(cmd)
