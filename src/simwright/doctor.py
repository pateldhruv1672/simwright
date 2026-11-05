"""Environment checks for a Windows + WSL + Docker physical-AI workstation (also works on Linux)."""
from __future__ import annotations
import os, pathlib, re, subprocess
from . import config
from .isaac import KitClient, KitError, find_isaac_sim
from .lab import Lab
from .targets import Runner, Target, decode


def _q(argv, timeout=20):
    try:
        r = subprocess.run(argv, capture_output=True, timeout=timeout)
        return r.returncode, decode(r.stdout + r.stderr)
    except FileNotFoundError:
        return None, "not installed"
    except subprocess.TimeoutExpired:
        return None, "timed out"


def run_doctor(runner: Runner | None = None, deep: bool = False) -> dict:
    cfg = config.load()
    checks = []

    def add(name, ok, detail, fix=""):
        checks.append({"check": name, "ok": ok, "detail": detail.strip()[:400], **({"fix": fix} if fix else {})})

    code, out = _q(["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"])
    add("NVIDIA GPU", code == 0, out if code == 0 else out,
        "install the NVIDIA Studio/Game Ready driver (Isaac Sim needs an RTX GPU, driver >= 580.88 on Windows for Isaac Lab 3)")
    if config.IS_WINDOWS:
        code, out = _q(["wsl.exe", "-l", "-v"])
        distros = [l.strip("* ").split()[0] for l in out.splitlines()[1:] if l.strip()]
        add("WSL 2", code == 0 and bool(distros), ", ".join(distros) or out, "wsl --install -d Ubuntu-24.04")
    code, out = _q(["docker", "info", "--format", "{{.ServerVersion}} {{.OperatingSystem}}"])
    add("Docker", code == 0, out, "install Docker Desktop with the WSL 2 backend and start it")
    if deep and code == 0:
        c2, o2 = _q(["docker", "run", "--rm", "--gpus", "all", "nvidia/cuda:12.6.3-base-ubuntu24.04", "nvidia-smi", "-L"], timeout=300)
        add("Docker GPU", c2 == 0, o2, "enable GPU support in Docker Desktop (WSL 2 backend) / install nvidia-container-toolkit")
    inst = find_isaac_sim(cfg)
    add("Isaac Sim", bool(inst), f"{inst.get('kind')} {inst.get('version','')} {inst.get('dir','') or inst.get('launcher','')}" if inst else "not found",
        "install Isaac Sim (pip install isaacsim[all,extscache] or the standalone zip), then `simwright config isaac_sim_dir <path>`")
    if inst.get("skills"):
        n = len(list(pathlib.Path(inst["skills"]).glob("*/SKILL.md")))
        add("NVIDIA Isaac Sim skills", n > 0, f"{n} skills at {inst['skills']}")
    try:
        ping = KitClient.from_config().ping()
        add("Isaac Sim session", ping["ok"], f"python_server reachable, Kit {ping.get('kit')}")
    except KitError as e:
        add("Isaac Sim session", False, "not running", "simwright isaac launch")
    L = Lab(cfg)
    add("Isaac Lab", bool(L.dir), f"{L.version()} at {L.dir}" if L.dir else "not found",
        "git clone https://github.com/isaac-sim/IsaacLab and run isaaclab.bat --install; then `simwright config isaac_lab_dir <path>`")
    runner = runner or Runner()
    try:
        t = Target.get(cfg.get("ros_target"))
        r = runner.capture("printenv ROS_DISTRO; command -v ros2 && command -v colcon", t, description="check ROS 2", timeout=60)
        ok = r.ok and "ros2" in r.out
        add(f"ROS 2 ({t.key})", ok, r.out, "install ROS 2 (Jazzy on Ubuntu 24.04) in WSL, or set targets.ros to a Docker container; Isaac Sim 6 also supports native Windows ROS 2 via Pixi")
    except Exception as e:
        add("ROS 2", False, str(e))
    bridge = pathlib.Path.home() / ".mana-terminal"
    add("Mana Bridge", bridge.exists(), "paired" if (bridge / "config.json").exists() else ("installed" if bridge.exists() else "not installed"),
        "open Mana Bridge in Mana and run its Windows install line, then type `mana` in your robot workspace")
    return {"ok": all(c["ok"] for c in checks if c["check"] in ("NVIDIA GPU",)), "checks": checks}
