"""Headless checks for MuJoCo and Gazebo Harmonic, run inside a target."""
from __future__ import annotations
import json, re

MUJOCO_CHECK = r"""
import json, sys, math
try:
    import mujoco
except ImportError:
    print(json.dumps({"ok": False, "error": "mujoco not installed in this target: pip install mujoco"})); sys.exit(0)
path, steps = sys.argv[1], int(sys.argv[2])
try:
    m = mujoco.MjModel.from_xml_path(path)
except Exception as e:
    print(json.dumps({"ok": False, "stage": "load", "error": str(e)})); sys.exit(0)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
first_bad, max_pen, max_v = None, 0.0, 0.0
for i in range(steps):
    mujoco.mj_step(m, d)
    if first_bad is None and (any(map(math.isnan, d.qacc)) or any(map(math.isnan, d.qpos))):
        first_bad = i
        break
    for c in range(d.ncon):
        max_pen = max(max_pen, -d.contact[c].dist)
    if m.nv:
        max_v = max(max_v, max(abs(x) for x in d.qvel))
warn = [{"warning": mujoco.mjtWarning(k).name, "count": int(d.warning[k].number)} for k in range(len(d.warning)) if d.warning[k].number]
print(json.dumps({"ok": first_bad is None and not warn, "nq": m.nq, "nv": m.nv, "nbody": m.nbody, "timestep": m.opt.timestep,
                  "steps": steps, "nan_at_step": first_bad, "max_penetration_m": round(max_pen, 5),
                  "max_abs_qvel": round(max_v, 3), "contacts_at_end": int(d.ncon), "warnings": warn}))
"""

MUJOCO_HINTS = {
    "mjWARN_BADQACC": "NaN/Inf in qacc: check inertias (tiny masses, zero diaginertia), too-large timestep, or stiff contacts; try opt.timestep=0.001 and implicitfast integrator",
    "mjWARN_CONTACTFULL": "too many contacts: raise nconmax or simplify collision geoms",
    "mjWARN_CNSTRFULL": "constraint buffer full: raise njmax/nconmax",
    "mjWARN_INERTIA": "invalid inertia on a body: fix mass/diaginertia",
}


def mujoco_command(model_path: str, steps: int = 2000) -> str:
    return "python3 - " + json.dumps(model_path) + f" {int(steps)} <<'PY'\n" + MUJOCO_CHECK + "\nPY"


def interpret_mujoco(output: str) -> dict:
    line = next((l for l in reversed(output.splitlines()) if l.strip().startswith("{")), "")
    try:
        r = json.loads(line)
    except Exception:
        return {"ok": False, "error": "no result", "output": output[-2000:]}
    r["hints"] = [MUJOCO_HINTS[w["warning"]] for w in r.get("warnings", []) if w["warning"] in MUJOCO_HINTS]
    if r.get("nan_at_step") is not None:
        r["hints"].append(f"simulation went NaN at step {r['nan_at_step']}: " + MUJOCO_HINTS["mjWARN_BADQACC"])
    if r.get("max_penetration_m", 0) > 0.01:
        r["hints"].append("deep interpenetration (>1 cm): bodies start overlapping or solref/solimp too soft")
    return r


def gazebo_command(world: str, iterations: int = 2000) -> str:
    return f"timeout 120 gz sim -s -r --iterations {int(iterations)} -v 4 {json.dumps(world)} 2>&1 | tail -n 400"


def interpret_gazebo(output: str) -> dict:
    issues = []
    for pat, hint in [
        (r"Failed to load system plugin \[([^\]]+)\]", "plugin library not found: check GZ_SIM_SYSTEM_PLUGIN_PATH and the filename (gz-sim-... vs ignition-gazebo-...)"),
        (r"Unable to find uri\[([^\]]+)\]", "model URI not found: set GZ_SIM_RESOURCE_PATH to your models folder"),
        (r"Error Code (\d+): Msg: (.+)", "SDF error"),
        (r"\[Err\].*", "Gazebo error"),
    ]:
        for m in re.finditer(pat, output):
            issues.append({"evidence": m.group(0)[:300], "hint": hint})
    return {"ok": not issues, "issues": issues[:30]}
