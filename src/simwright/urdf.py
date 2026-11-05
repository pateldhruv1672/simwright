"""Static URDF checks that catch most 'robot explodes / falls through floor / sinks' problems."""
from __future__ import annotations
import math, pathlib, re
import xml.etree.ElementTree as ET


def _floats(s: str | None, n: int = 3, default: float = 0.0) -> list[float]:
    if not s:
        return [default] * n
    v = [float(x) for x in s.split()]
    return (v + [default] * n)[:n]


def sym_eigen(ixx, iyy, izz, ixy, ixz, iyz) -> list[float]:
    """Eigenvalues of a symmetric 3x3 (closed form)."""
    p1 = ixy ** 2 + ixz ** 2 + iyz ** 2
    if p1 < 1e-18:
        return sorted([ixx, iyy, izz])
    q = (ixx + iyy + izz) / 3
    p2 = (ixx - q) ** 2 + (iyy - q) ** 2 + (izz - q) ** 2 + 2 * p1
    p = math.sqrt(p2 / 6)
    B = [[(ixx - q) / p, ixy / p, ixz / p], [ixy / p, (iyy - q) / p, iyz / p], [ixz / p, iyz / p, (izz - q) / p]]
    detB = (B[0][0] * (B[1][1] * B[2][2] - B[1][2] * B[2][1]) - B[0][1] * (B[1][0] * B[2][2] - B[1][2] * B[2][0])
            + B[0][2] * (B[1][0] * B[2][1] - B[1][1] * B[2][0]))
    r = max(-1.0, min(1.0, detB / 2))
    phi = math.acos(r) / 3
    e1 = q + 2 * p * math.cos(phi)
    e3 = q + 2 * p * math.cos(phi + 2 * math.pi / 3)
    return sorted([e3, 3 * q - e1 - e3, e1])


def validate(path: str, text: str | None = None) -> dict:
    src = text if text is not None else pathlib.Path(path).read_text(encoding="utf-8", errors="replace")
    issues = []

    def add(sev, where, msg, fix=""):
        issues.append({"severity": sev, "where": where, "issue": msg, **({"fix": fix} if fix else {})})

    if "xacro:" in src and text is None and path.endswith(".xacro"):
        add("info", path, "xacro file: validating raw XML; run `xacro` first for exact results")
    try:
        root = ET.fromstring(re.sub(r"\sxmlns:xacro=\"[^\"]+\"", "", src))
    except ET.ParseError as e:
        return {"ok": False, "issues": [{"severity": "error", "where": path, "issue": f"XML parse error: {e}"}]}
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    children, parents = {}, {}
    total_mass = 0.0
    for name, l in links.items():
        inert = l.find("inertial")
        has_col = l.find("collision") is not None
        has_vis = l.find("visual") is not None
        if inert is None:
            if has_col or has_vis:
                add("warn", f"link {name}", "no <inertial>: Isaac Sim / Gazebo will give it a default or ignore it",
                    "add <inertial> with mass and inertia (box/cylinder formulas)")
            continue
        m = inert.find("mass")
        mass = float(m.get("value", "0")) if m is not None else 0.0
        total_mass += mass
        if mass <= 0:
            add("error", f"link {name}", f"mass {mass} <= 0", "set a positive mass")
        elif mass < 1e-3:
            add("warn", f"link {name}", f"tiny mass {mass} kg causes solver instability", "use >= 0.01 kg or merge with parent (fixed joint)")
        i = inert.find("inertia")
        if i is None:
            add("error", f"link {name}", "missing <inertia>")
            continue
        g = lambda k: float(i.get(k, "0"))
        ixx, iyy, izz, ixy, ixz, iyz = g("ixx"), g("iyy"), g("izz"), g("ixy"), g("ixz"), g("iyz")
        ev = sym_eigen(ixx, iyy, izz, ixy, ixz, iyz)
        if min(ev) <= 0:
            add("error", f"link {name}", f"inertia not positive definite (principal moments {[round(x, 6) for x in ev]})",
                "recompute inertia; off-diagonal terms are probably wrong or diagonal is zero")
        else:
            a, b, c = ev
            if a + b < c * (1 - 1e-6):
                add("error", f"link {name}", f"inertia violates triangle inequality ({a:.3g} + {b:.3g} < {c:.3g}): not a physical body",
                    "recompute from geometry; PhysX may reject or clamp it")
            if mass > 0:
                r = math.sqrt(max(ev) / mass)
                if r > 5:
                    add("warn", f"link {name}", f"radius of gyration {r:.2f} m is huge for this mass: units mixed up (g*mm^2?)")
                if r < 1e-3:
                    add("warn", f"link {name}", f"radius of gyration {r:.4f} m is tiny: inertia probably in wrong units")
        for col in l.findall("collision"):
            mesh = col.find("geometry/mesh")
            if mesh is not None and str(mesh.get("filename", "")).lower().endswith((".dae", ".obj")) and False:
                pass
    for j in joints:
        jn, jt = j.get("name"), j.get("type")
        p = j.find("parent").get("link") if j.find("parent") is not None else None
        c = j.find("child").get("link") if j.find("child") is not None else None
        if p not in links:
            add("error", f"joint {jn}", f"parent link '{p}' does not exist")
        if c not in links:
            add("error", f"joint {jn}", f"child link '{c}' does not exist")
        if c in parents:
            add("error", f"joint {jn}", f"link '{c}' has two parents ({parents[c]} and {p})")
        parents[c] = p
        children.setdefault(p, []).append(c)
        lim = j.find("limit")
        if jt in ("revolute", "prismatic"):
            if lim is None:
                add("error", f"joint {jn}", f"{jt} joint without <limit>", "add lower/upper/effort/velocity")
            else:
                lo, hi = float(lim.get("lower", "0")), float(lim.get("upper", "0"))
                if lo >= hi:
                    add("error", f"joint {jn}", f"lower {lo} >= upper {hi}")
                if float(lim.get("effort", "0")) <= 0:
                    add("warn", f"joint {jn}", "effort limit is 0: the joint cannot be driven in Isaac Sim / Gazebo")
                if float(lim.get("velocity", "0")) <= 0:
                    add("warn", f"joint {jn}", "velocity limit is 0")
        axis = j.find("axis")
        if axis is not None and jt != "fixed":
            v = _floats(axis.get("xyz"))
            n = math.sqrt(sum(x * x for x in v))
            if n == 0:
                add("error", f"joint {jn}", "zero axis")
            elif abs(n - 1) > 1e-3:
                add("warn", f"joint {jn}", f"axis not unit length ({n:.3f})")
    roots = [n for n in links if n not in parents]
    if len(roots) != 1:
        add("error", "robot", f"expected one root link, found {roots}", "every link except the root needs exactly one parent joint")
    seen, stack = set(), list(roots[:1])
    while stack:
        n = stack.pop()
        if n in seen:
            add("error", "robot", f"kinematic loop at {n}")
            break
        seen.add(n)
        stack += children.get(n, [])
    orphans = set(links) - seen
    if roots and orphans and len(roots) == 1:
        add("error", "robot", f"links not connected to root: {sorted(orphans)}")
    return {"ok": not any(i["severity"] == "error" for i in issues), "robot": root.get("name"), "links": len(links),
            "joints": len(joints), "total_mass_kg": round(total_mass, 3), "root": roots[0] if len(roots) == 1 else roots,
            "issues": issues}
