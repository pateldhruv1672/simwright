"""Project context you write once: SIMWRIGHT.md (or DRIFT.md / CLAUDE.md / AGENTS.md) at the workspace root.

`simwright init` scans the workspace and writes a starter SIMWRIGHT.md.
"""
from __future__ import annotations
import collections, json, os, pathlib, re, time
import xml.etree.ElementTree as ET

CONTEXT_FILES = ["SIMWRIGHT.md", "DRIFT.md", "CLAUDE.md", "AGENTS.md", ".simwright/CONTEXT.md"]
SKIP_DIRS = {".git", "build", "install", "log", "node_modules", ".venv", "venv", "__pycache__", ".pixi",
             "_build", "logs", "outputs", "wandb", ".cache", ".simwright"}
MARKERS = ["SIMWRIGHT.md", "DRIFT.md", "CLAUDE.md", "AGENTS.md", ".git", "colcon.meta", ".simwright"]


def find_root(start: str | os.PathLike | None = None) -> pathlib.Path:
    p = pathlib.Path(start or os.getcwd()).resolve()
    for d in [p, *p.parents]:
        if any((d / m).exists() for m in MARKERS) or (d / "src").is_dir() and any((d / "src").rglob("package.xml")):
            return d
    return p


def load_context(root: pathlib.Path) -> tuple[str, str]:
    """Return (filename, text) of the first context file found, else ('', '')."""
    for name in CONTEXT_FILES:
        f = root / name
        if f.is_file():
            return name, f.read_text(encoding="utf-8", errors="replace")
    return "", ""


def walk(root: pathlib.Path, max_files: int = 20000):
    n = 0
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS and not d.startswith(".")]
        for f in filenames:
            n += 1
            if n > max_files:
                return
            yield pathlib.Path(dirpath) / f


TOPIC_RE = re.compile(r"""["'](/[A-Za-z_][\w]*(?:/[A-Za-z_][\w]*)*)["']""")
FRAME_RE = re.compile(r"""(?:frame_id|frame|base_frame|odom_frame|map_frame|child_frame_id|parent_frame)\s*[:=]\s*["']?([A-Za-z_][\w/]*)""")
NOT_TOPICS = {"/opt", "/usr", "/home", "/tmp", "/dev", "/bin", "/etc", "/mnt", "/proc", "/root", "/var"}


def scan(root: pathlib.Path) -> dict:
    info = collections.defaultdict(list)
    topics, frames = collections.Counter(), collections.Counter()
    distros = collections.Counter()
    for f in walk(root):
        rel = f.relative_to(root).as_posix()
        name, low = f.name, f.name.lower()
        try:
            if name == "package.xml":
                info["packages"].append(_package(f, rel))
            elif low.endswith((".urdf", ".urdf.xacro", ".xacro")):
                info["urdf"].append(_urdf(f, rel))
            elif low.endswith((".sdf", ".world")):
                info["sdf"].append(rel)
            elif low.endswith((".usd", ".usda", ".usdc", ".usdz")):
                info["usd"].append(rel)
            elif low.endswith(".xml") and f.stat().st_size < 2_000_000 and "<mujoco" in f.read_text(errors="ignore")[:4000]:
                info["mjcf"].append(rel)
            elif low.endswith((".launch.py", ".launch.xml", ".launch.yaml", ".launch")):
                info["launch"].append(rel)
            elif name == "extension.toml" and "isaaclab" in f.read_text(errors="ignore"):
                info["isaaclab_ext"].append(rel)
            elif low in ("dockerfile",) or low.endswith(".dockerfile") or name == "devcontainer.json" or low.startswith("docker-compose"):
                info["docker"].append(rel)
                distros.update(re.findall(r"ros[:-](humble|iron|jazzy|kilted|rolling)", f.read_text(errors="ignore")))
            elif low.endswith(".repos") or name == "colcon.meta" or name == "pixi.toml":
                info["workspace_files"].append(rel)
            if low.endswith((".py", ".yaml", ".yml", ".launch", ".xml", ".cpp", ".hpp")) and f.stat().st_size < 400_000:
                text = f.read_text(errors="ignore")
                if low.endswith(".py") and "gym.register(" in text and "Isaac" in text or "isaaclab" in text and "gym.register(" in text:
                    info["isaaclab_tasks"] += re.findall(r"""id\s*=\s*["']([\w\-:.]+)["']""", text)
                if any(k in rel for k in ("launch", "config", "param")) or low.endswith((".launch.py", ".yaml", ".yml")):
                    for t in TOPIC_RE.findall(text):
                        if t.split("/")[1] and "/" + t.split("/")[1] not in NOT_TOPICS and len(t) < 60:
                            topics[t] += 1
                    frames.update(FRAME_RE.findall(text))
        except Exception as e:
            info["scan_errors"].append(f"{rel}: {e}")
    info["topics"] = [t for t, _ in topics.most_common(30)]
    info["frames"] = [t for t, _ in frames.most_common(20)]
    info["ros_distro"] = distros.most_common(1)[0][0] if distros else ""
    return dict(info)


def _package(f: pathlib.Path, rel: str) -> dict:
    root = ET.parse(f).getroot()
    deps = sorted({e.text.strip() for tag in ("depend", "build_depend", "exec_depend") for e in root.iter(tag) if e.text})
    bt = root.find("export/build_type")
    return {"name": (root.findtext("name") or "").strip(), "path": str(pathlib.PurePosixPath(rel).parent),
            "build_type": (bt.text.strip() if bt is not None and bt.text else "ament_cmake"), "deps": deps[:40]}


def _urdf(f: pathlib.Path, rel: str) -> dict:
    text = f.read_text(errors="ignore")
    robot = re.search(r"<robot[^>]*name=[\"']([^\"']+)", text)
    links = re.findall(r"<link\s+name=[\"']([^\"']+)", text)
    joints = re.findall(r"<joint\s+name=[\"']([^\"']+)[\"'][^>]*type=[\"']([^\"']+)", text)
    sensors = re.findall(r"<sensor[^>]*type=[\"']([^\"']+)", text)
    return {"path": rel, "robot": robot.group(1) if robot else "", "links": len(links), "joints": len(joints),
            "actuated": sum(1 for _, t in joints if t in ("revolute", "continuous", "prismatic")),
            "root_link": links[0] if links else "", "sensors": sorted(set(sensors))}


def starter_markdown(root: pathlib.Path, info: dict) -> str:
    L = [f"# {root.name}: project context for Simwright", "",
         "Simwright reads this file before every step. Keep it short and true; edit the TODOs.", "",
         f"_Generated by `simwright init` on {time.strftime('%Y-%m-%d')}. Facts below were found in the workspace; TODOs need you._", ""]
    L += ["## Robot", ""]
    if info.get("urdf"):
        for u in info["urdf"][:6]:
            L.append(f"- `{u['path']}`: robot `{u['robot'] or '?'}`, {u['links']} links, {u['joints']} joints ({u['actuated']} actuated), root `{u['root_link']}`" + (f", sensors: {', '.join(u['sensors'])}" if u['sensors'] else ""))
    else:
        L.append("- TODO: robot description (URDF/xacro path, mass, footprint, actuators)")
    L += ["- TODO: physical specs that matter (wheel radius/separation, max velocity, joint limits, payload)", ""]
    L += ["## ROS 2", "", f"- Distro: {info.get('ros_distro') or 'TODO (humble / jazzy / ...)'}"]
    if info.get("packages"):
        L.append("- Packages:")
        for p in sorted(info["packages"], key=lambda x: x["name"])[:40]:
            L.append(f"  - `{p['name']}` ({p['build_type']}) at `{p['path']}`")
    L += ["- Topics seen in launch/config files: " + (", ".join(f"`{t}`" for t in info.get("topics", [])[:20]) or "TODO"),
          "- Frames: " + (", ".join(f"`{t}`" for t in info.get("frames", [])[:12]) or "TODO (map -> odom -> base_link -> ...)"),
          "- TODO: QoS conventions (sensor data best-effort? latched maps?)", ""]
    L += ["## Simulation", ""]
    for key, label in (("usd", "USD"), ("sdf", "SDF worlds"), ("mjcf", "MJCF"), ("launch", "Launch files")):
        if info.get(key):
            more = f" (+{len(info[key]) - 8} more)" if len(info[key]) > 8 else ""
            L.append(f"- {label}: " + ", ".join(f"`{x}`" for x in info[key][:8]) + more)
    if info.get("isaaclab_ext") or info.get("isaaclab_tasks"):
        L.append("- Isaac Lab extension: " + ", ".join(f"`{x}`" for x in info.get("isaaclab_ext", [])[:4]))
        L.append("- Isaac Lab tasks: " + ", ".join(f"`{x}`" for x in sorted(set(info.get("isaaclab_tasks", [])))[:12]))
    L += ["- TODO: primary simulator (Isaac Sim / Gazebo Harmonic / MuJoCo) and versions", ""]
    L += ["## Build", "", "- `colcon build --symlink-install` in the ROS target (WSL by default on Windows)",
          "- TODO: build quirks (packages to skip, cmake args, mixins)"]
    if info.get("docker"):
        L.append("- Containers: " + ", ".join(f"`{x}`" for x in info["docker"][:6]))
    L += ["", "## Conventions", "", "- TODO: package naming (e.g. `<robot>_description`, `<robot>_bringup`)",
          "- TODO: launch hierarchy, parameter file layout, units, frame naming",
          "- TODO: what a finished task must show (video, metrics, tests)", "",
          "## Policies", "", "- TODO: Isaac Lab tasks you train, RL library, checkpoints that matter", ""]
    return "\n".join(L)


def init(root: pathlib.Path, force: bool = False) -> tuple[pathlib.Path, dict, bool]:
    info = scan(root)
    out = root / "SIMWRIGHT.md"
    (root / ".simwright").mkdir(exist_ok=True)
    (root / ".simwright" / "scan.json").write_text(json.dumps(info, indent=2))
    if out.exists() and not force:
        return out, info, False
    out.write_text(starter_markdown(root, info), encoding="utf-8")
    return out, info, True
