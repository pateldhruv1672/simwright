"""Simwright helpers that run INSIDE Isaac Sim (Kit Python). Loaded by the simwright.copilot extension
or sent once over the Isaac Sim python_server. Uses only USD, omni.usd, omni.timeline, omni.kit.app,
omni.physx and omni.client so it works across Isaac Sim 5.x and 6.x."""
import math, os, re, time

try:
    from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf, Sdf
except ImportError:  # no USD at all (plain unit tests): definitions only
    pass
try:
    import omni.usd, omni.timeline, omni.kit.app
except ImportError:  # outside Kit (tests use usd-core and set _TEST_STAGE)
    pass

_TEST_STAGE = None

_ASSET_CACHE = globals().get("_ASSET_CACHE") or {}


HELPERS_VERSION = "0.5.3"


def _stage():
    if _TEST_STAGE is not None:
        return _TEST_STAGE
    return omni.usd.get_context().get_stage()


def _app():
    return omni.kit.app.get_app()


def assets_root():
    for mod, fn in (("isaacsim.storage.native", "get_assets_root_path"), ("omni.isaac.nucleus", "get_assets_root_path")):
        try:
            m = __import__(mod, fromlist=[fn])
            r = getattr(m, fn)()
            if r:
                return r
        except Exception:
            pass
    return None


def scene_summary(max_list=40):
    st = _stage()
    if st is None:
        return {"stage": None}
    counts, lights, arts, cams, scenes, rigid = {}, [], [], [], [], 0
    all_joints = []
    for j in st.Traverse(Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)) if hasattr(st, "Traverse") else []:
        if j.IsA(UsdPhysics.Joint):
            jj = UsdPhysics.Joint(j)
            tg = [str(x) for x in (jj.GetBody0Rel().GetTargets() + jj.GetBody1Rel().GetTargets())]
            all_joints.append((str(j.GetPath()), tg))
    for p in st.Traverse():
        t = p.GetTypeName() or "(untyped)"
        counts[t] = counts.get(t, 0) + 1
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            root = str(p.GetPath())
            # an articulation root applied to a joint (NVIDIA xArm assets) spans its robot folder; importers may also
            # put joints in a sibling /Physics scope, so match joints by the bodies they connect
            scope = str(p.GetParent().GetPath()) if p.IsA(UsdPhysics.Joint) and p.GetParent() else root
            n = sum(1 for jp, tg in all_joints if jp == root or jp.startswith(scope + "/") or any(x == scope or x.startswith(scope + "/") for x in tg))
            arts.append({"path": root, "joints": n})
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid += 1
        if t.endswith("Light"):
            a = p.GetAttribute("inputs:intensity") or p.GetAttribute("intensity")
            lights.append({"path": str(p.GetPath()), "type": t, "intensity": float(a.Get()) if a and a.Get() is not None else None})
        if t == "Camera":
            cams.append(str(p.GetPath()))
        if t == "PhysicsScene":
            g = UsdPhysics.Scene(p)
            gd, gm = list(g.GetGravityDirectionAttr().Get() or []), g.GetGravityMagnitudeAttr().Get()
            eff_dir = "negative up axis (schema default)" if (not gd or all(v == 0 for v in gd)) else str(gd)
            eff_mag = "Earth gravity (schema default)" if (gm is None or gm < 0) else f"{gm} stage units/s^2"
            scenes.append({"path": str(p.GetPath()), "gravity_dir": gd, "gravity": gm, "effective": f"{eff_mag}, along {eff_dir}"})
    tl = omni.timeline.get_timeline_interface()
    return {"stage": st.GetRootLayer().identifier, "up_axis": UsdGeom.GetStageUpAxis(st),
            "meters_per_unit": UsdGeom.GetStageMetersPerUnit(st), "prims": sum(counts.values()),
            "types": dict(sorted(counts.items(), key=lambda kv: -kv[1])[:25]), "articulations": arts[:max_list],
            "rigid_bodies": rigid, "lights": lights[:max_list], "cameras": cams[:max_list], "physics_scenes": scenes,
            "timeline": {"playing": tl.is_playing(), "time": tl.get_current_time()}, "health": _health_brief()}


def _health_brief():
    try:
        h = stage_health()
        pf = physics_preflight()
        return {"ok": h["ok"] and pf["ok"], "issues": (pf["issues"] + h["issues"])[:20], "articulations": h["articulations"]}
    except Exception as e:
        return {"ok": None, "error": str(e)}


def _rigid_paths(limit=300):
    out = []
    for p in _stage().Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and _attr(p, "physics:rigidBodyEnabled", True):
            out.append(str(p.GetPath()))
            if len(out) >= limit:
                break
    return out


def _pose(path):
    try:
        import omni.timeline
        if not omni.timeline.get_timeline_interface().is_playing():
            raise RuntimeError("PhysX has not parsed the stage yet: use the USD pose")
        from omni.physx import get_physx_interface
        r = get_physx_interface().get_rigidbody_transformation(path)
        if r and r.get("ret_val"):
            return [float(v) for v in r["position"]]
    except Exception:
        pass
    m = UsdGeom.Xformable(_stage().GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return [t[0], t[1], t[2]]


def light_check():
    s = scene_summary()
    dome = max([l["intensity"] or 0 for l in s["lights"] if l["type"] == "DomeLight"] or [0])
    dist = max([l["intensity"] or 0 for l in s["lights"] if l["type"] == "DistantLight"] or [0])
    other = [l for l in s["lights"] if l["type"] not in ("DomeLight", "DistantLight")]
    ok = dome >= 100 or dist >= 500 or bool(other)
    return {"ok": ok, "dome": dome, "distant": dist, "other_lights": len(other),
            "hint": "" if ok else "no usable light: renders will be black. Add a DomeLight (>=100) or DistantLight (>=500)"}


ROBOT_NAME = re.compile(r"(xarm|franka|panda|ur\d|ur_|g1|h1|go2|unitree|carter|jetbot|robot|arm|humanoid|spot|anymal|allegro|shadow|kinova|iiwa)", re.I)


def _asset_items(lst):
    out = []
    for attr in ("explicitItems", "prependedItems", "appendedItems", "addedItems"):
        try:
            out += list(getattr(lst, attr) or [])
        except Exception:
            pass
    return out


def stage_health(max_prims=50000):
    """Static checks that a short physics run cannot catch: broken references/payloads, robot prims that never
    loaded an articulation, invalid gravity. Gravity direction (0,0,0) and a negative magnitude are UsdPhysics
    defaults (negative up axis, Earth gravity) and are fine."""
    st = _stage()
    issues, broken, n = [], [], 0
    seen = set()
    for prim in st.TraverseAll():
        n += 1
        if n > max_prims:
            break
        for spec in prim.GetPrimStack():
            for kind, lst in (("reference", spec.referenceList), ("payload", spec.payloadList)):
                for item in _asset_items(lst):
                    ap = getattr(item, "assetPath", "") or ""
                    if not ap:
                        continue
                    key = (str(prim.GetPath()), ap)
                    if key in seen:
                        continue
                    seen.add(key)
                    try:
                        resolved = Sdf.ComputeAssetPathRelativeToLayer(spec.layer, ap)
                    except Exception:
                        resolved = ap
                    ok = False
                    try:
                        ok = Sdf.Layer.FindOrOpen(resolved) is not None
                    except Exception:
                        ok = False
                    if not ok:
                        broken.append({"prim": str(prim.GetPath()), "kind": kind, "asset": ap, "resolved": resolved,
                                       "authored_in": spec.layer.identifier})
    for b in broken[:20]:
        issues.append({"severity": "error", "body": b["prim"], "issue": f"broken {b['kind']}: {b['asset']} (resolves to {b['resolved']})",
                       "fix": "point it at the real file; relative paths resolve from the layer that authors them (" + b["authored_in"] + ")"})
    arts = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    for p in st.Traverse():
        path = str(p.GetPath())
        if p.GetParent() and str(p.GetParent().GetPath()) in ("/", "/World") and ROBOT_NAME.search(p.GetName()):
            inside = any(a == path or a.startswith(path + "/") for a in arts)
            joints = sum(1 for d in Usd.PrimRange(p, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
                         if d.IsA(UsdPhysics.Joint))
            if not inside and joints == 0:
                issues.append({"severity": "warning", "body": path, "issue": "looks like a robot but has no articulation and no joints (empty placeholder or failed load)",
                               "fix": "reference the robot USD (or import its URDF) and check the reference resolves"})
    for p in st.Traverse():
        if p.IsA(UsdPhysics.Scene):
            sc = UsdPhysics.Scene(p)
            d = sc.GetGravityDirectionAttr().Get()
            m = sc.GetGravityMagnitudeAttr().Get()
            if d is not None and any(math.isnan(v) or math.isinf(v) for v in d):
                issues.append({"severity": "error", "body": str(p.GetPath()), "issue": f"gravity direction is not finite: {tuple(d)}",
                               "fix": "set gravityDirection to (0,0,-1) or (0,0,0) for 'negative up axis'"})
            if m is not None and (math.isnan(m) or (math.isinf(m) and m > 0)):
                issues.append({"severity": "error", "body": str(p.GetPath()), "issue": f"gravity magnitude is {m}",
                               "fix": "use 9.81 (m/s^2, scaled by metersPerUnit) or any negative value for Earth gravity"})
    return {"ok": not any(i["severity"] == "error" for i in issues), "articulations": arts[:20], "broken_references": broken[:20],
            "issues": issues, "note": "gravity (0,0,0) / negative magnitude are UsdPhysics defaults: Earth gravity along the negative up axis"}


def _attr(p, name, default=None):
    a = p.GetAttribute(name)
    if a and a.HasAuthoredValue():
        v = a.Get()
        return default if v is None else v
    return default


def physics_preflight(max_prims=60000):
    """Static checks for setups known to crash or destabilize PhysX, run before pressing Play.
    severity 'crash-risk' blocks the physics check unless forced."""
    st = _stage()
    issues = []
    scenes = [p for p in st.Traverse() if p.IsA(UsdPhysics.Scene)]
    for sc in scenes:
        gpu = bool(_attr(sc, "physxScene:enableGPUDynamics", True))          # PhysX scene default: GPU dynamics on
        bp = str(_attr(sc, "physxScene:broadphaseType", "GPU"))
        if bool(_attr(sc, "physxScene:enableCCD", False)) and (gpu or bp.upper() == "GPU"):
            issues.append({"severity": "crash-risk", "body": str(sc.GetPath()),
                           "issue": "CCD is enabled on a GPU PhysX scene (GPU dynamics / GPU broadphase do not support CCD)",
                           "fix": f"set physxScene:enableCCD = False on {sc.GetPath()} (or switch the scene to CPU dynamics + MBP broadphase if you really need CCD)"})
    if len(scenes) > 1:
        issues.append({"severity": "warning", "issue": f"{len(scenes)} PhysicsScene prims: bodies may be split across scenes",
                       "fix": "keep one PhysicsScene"})
    roots, bodies, n = [], [], 0
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    for p in st.Traverse(pred):
        n += 1
        if n > max_prims:
            issues.append({"severity": "warning", "issue": f"preflight stopped after {max_prims} prims"})
            break
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(p.GetPath()))
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and _attr(p, "physics:rigidBodyEnabled", True):
            bodies.append(p)
            mass = _attr(p, "physics:mass", None)
            if mass is not None and (mass < 0 or math.isnan(mass)):
                issues.append({"severity": "crash-risk", "body": str(p.GetPath()), "issue": f"negative or NaN mass ({mass})", "fix": "set a positive mass or remove it to use density"})
            if _attr(p, "physxRigidBody:enableCCD", False):
                issues.append({"severity": "warning", "body": str(p.GetPath()), "issue": "per-body CCD on a GPU scene is ignored or unstable", "fix": "disable physxRigidBody:enableCCD"})
            if not _attr(p, "physics:kinematicEnabled", False):
                for d in Usd.PrimRange(p, pred):
                    if d.IsA(UsdGeom.Mesh) and d.HasAPI(UsdPhysics.CollisionAPI):
                        appr = _attr(d, "physics:approximation", "none")
                        if appr in ("none", "meshSimplification"):
                            issues.append({"severity": "warning", "body": str(d.GetPath()),
                                           "issue": f"triangle-mesh collider ({appr}) on a dynamic body: PhysX cannot simulate it and falls back",
                                           "fix": "use convexHull or convexDecomposition for moving parts"})
                            break
    # nested articulation roots: a second root inside another articulation's robot
    def scope(path):
        pr = st.GetPrimAtPath(path)
        return str(pr.GetParent().GetPath()) if pr.IsA(UsdPhysics.Joint) and pr.GetParent() else path
    scopes = sorted({scope(r) for r in roots}, key=len)
    for i, a in enumerate(scopes):
        for b in scopes[i + 1:]:
            if b.startswith(a + "/"):
                issues.append({"severity": "crash-risk", "body": b,
                               "issue": f"articulation root nested inside the articulation at {a}",
                               "fix": "keep one ArticulationRootAPI per robot: remove it from the inner prim (e.g. the gripper) and connect the gripper with a fixed joint, "
                                      "or use the combined robot+gripper asset"})
    # rigid bodies nested in rigid bodies without resetting the transform stack
    bpaths = sorted(str(b.GetPath()) for b in bodies)
    art_scopes = [scope(r) for r in roots]
    in_art = lambda q: any(q == s_ or q.startswith(s_ + "/") for s_ in art_scopes)
    for i, a in enumerate(bpaths):
        for b in bpaths[i + 1:i + 40]:
            if b.startswith(a + "/") and not (in_art(a) and in_art(b)):  # links nested inside one articulation are normal
                bp = st.GetPrimAtPath(b)
                if not UsdGeom.Xformable(bp).GetResetXformStack():
                    issues.append({"severity": "warning", "body": b, "issue": f"rigid body nested inside rigid body {a}",
                                   "fix": "disable one of them, or make the inner one a child collider / connect them with a joint"})
                break
    if len(bodies) > 3000:
        issues.append({"severity": "warning", "issue": f"{len(bodies)} rigid bodies: GPU contact buffers may overflow on a 16 GB laptop GPU",
                       "fix": "make static props non-rigid (colliders only) and raise physxScene:gpuMaxRigidContactCount if needed"})
    return {"ok": not any(i["severity"] == "crash-risk" for i in issues), "rigid_bodies": len(bodies), "articulation_roots": roots[:30], "issues": issues[:40]}


# Joint-name classes -> (stiffness, damping, max force, armature). USD angular drive units (per degree), the values that
# made the G1 + Revo2 stable on Nov 4 2026. Only applied to drives left limp (stiffness 0 and damping 0).
DRIVE_PROFILES = [
    (re.compile(r"(hip|knee|ankle|waist|torso|leg|thigh|calf)", re.I), (400.0, 20.0, 300.0, 0.05)),
    (re.compile(r"(shoulder|elbow|arm|joint[1-4]$)", re.I), (150.0, 8.0, 100.0, 0.03)),
    (re.compile(r"(wrist|joint[5-7]$|head|neck)", re.I), (40.0, 2.0, 20.0, 0.01)),
    (re.compile(r"(thumb|index|middle|ring|pinky|finger|knuckle|drive_joint|grip|hand)", re.I), (5.0, 0.5, 5.0, 0.005)),
]


def _is_mimic(p):
    names = [str(a) for a in p.GetAppliedSchemas()]
    lo = p.GetMetadata("apiSchemas")
    if lo is not None:
        for attr in ("explicitItems", "prependedItems", "appendedItems", "addedItems"):
            names += [str(x) for x in (getattr(lo, attr, None) or [])]
    return any("Mimic" in n for n in names) or any("mimic" in a.GetName().lower() for a in p.GetAuthoredAttributes())


def _profile(name):
    for rx, v in DRIVE_PROFILES:
        if rx.search(name):
            return v
    return (100.0, 5.0, 50.0, 0.01)


def physics_repair(scope="/World", apply=False, mass_floor=0.02, solver=(32, 4), budget_s=6.0):
    """Find and (with apply=True) fix the setups that make PhysX unstable or crash, as overrides on the current
    edit target (the scene file); referenced robot assets are not modified.
    - CCD on a GPU PhysX scene -> off
    - an articulation root nested inside another robot (xArm gripper) -> removed, the inner part joins the outer one
    - limp drives (stiffness 0 and damping 0) -> gains by joint class, plus armature
    - links lighter than mass_floor (kg) -> raised, inertia scaled with it (same radius of gyration)
    - articulations: solver iterations, self-collision off
    Returns the list of changes (planned or applied)."""
    st = _stage()
    t0 = time.time()
    root = st.GetPrimAtPath(scope)
    if not root or not root.IsValid():
        return {"ok": False, "error": f"{scope} not found"}
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    kg = UsdPhysics.GetStageKilogramsPerUnit(st) if hasattr(UsdPhysics, "GetStageKilogramsPerUnit") else 1.0
    changes, truncated = [], False

    def setattr_(prim, name, vtype, value, why):
        a = prim.GetAttribute(name)
        old = a.Get() if a and a.HasAuthoredValue() else None
        changes.append({"prim": str(prim.GetPath()), "attr": name, "from": None if old is None else str(old), "to": str(value), "why": why})
        if apply:
            (a if a else prim.CreateAttribute(name, vtype)).Set(value)

    for sc in [p for p in st.Traverse() if p.IsA(UsdPhysics.Scene)]:
        gpu = bool(_attr(sc, "physxScene:enableGPUDynamics", True)) or str(_attr(sc, "physxScene:broadphaseType", "GPU")).upper() == "GPU"
        if _attr(sc, "physxScene:enableCCD", False) and gpu:
            setattr_(sc, "physxScene:enableCCD", Sdf.ValueTypeNames.Bool, False, "CCD is not supported by GPU PhysX")
    roots = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]

    def scope_of(p):
        return p.GetParent() if p.IsA(UsdPhysics.Joint) and p.GetParent() else p
    outer = sorted(roots, key=lambda p: len(str(scope_of(p).GetPath())))
    kept = []
    for p in outer:
        sp = str(scope_of(p).GetPath())
        host = next((k for k in kept if sp.startswith(str(scope_of(k).GetPath()) + "/")), None)
        if host is not None:
            changes.append({"prim": str(p.GetPath()), "attr": "apiSchemas", "from": "PhysicsArticulationRootAPI",
                            "to": "(removed)", "why": f"nested inside the articulation at {host.GetPath()}; it joins that robot"})
            if apply:
                p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                for api in ("PhysxArticulationAPI",):
                    if api in [str(a) for a in p.GetAppliedSchemas()]:
                        p.RemoveAppliedSchema(api)
        else:
            kept.append(p)
    for p in kept:
        it = p.GetAttribute("physxArticulation:solverPositionIterationCount")
        if not it or (it.Get() or 0) < solver[0]:
            setattr_(p, "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int, int(solver[0]), "more solver iterations for long chains and light fingers")
        iv = p.GetAttribute("physxArticulation:solverVelocityIterationCount")
        if not iv or (iv.Get() or 0) < solver[1]:
            setattr_(p, "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, int(solver[1]), "velocity iterations")
        if _attr(p, "physxArticulation:enabledSelfCollisions", True):
            setattr_(p, "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, False, "adjacent links overlap in imported meshes")
    for p in Usd.PrimRange(root):
        if time.time() - t0 > budget_s:
            truncated = True
            break
        if (p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)) and not _is_mimic(p):
            # mimic followers copy another joint: a drive on them fights the mimic constraint
            kind = "angular" if p.IsA(UsdPhysics.RevoluteJoint) else "linear"
            k = _attr(p, f"drive:{kind}:physics:stiffness", 0.0) or 0.0
            d = _attr(p, f"drive:{kind}:physics:damping", 0.0) or 0.0
            if not p.HasAPI(UsdPhysics.DriveAPI, kind) or (k == 0.0 and d == 0.0):
                ks, ds, fmax, arm = _profile(p.GetName())
                if kind == "linear":
                    ks, ds = ks * 10, ds * 10
                if apply and not p.HasAPI(UsdPhysics.DriveAPI, kind):
                    UsdPhysics.DriveAPI.Apply(p, kind)
                setattr_(p, f"drive:{kind}:physics:stiffness", Sdf.ValueTypeNames.Float, ks, "limp drive (stiffness 0, damping 0)")
                setattr_(p, f"drive:{kind}:physics:damping", Sdf.ValueTypeNames.Float, ds, "limp drive")
                if (_attr(p, f"drive:{kind}:physics:maxForce", 0.0) or 0.0) <= 0:
                    setattr_(p, f"drive:{kind}:physics:maxForce", Sdf.ValueTypeNames.Float, fmax, "no force limit")
                if (_attr(p, "physxJoint:armature", 0.0) or 0.0) <= 0:
                    setattr_(p, "physxJoint:armature", Sdf.ValueTypeNames.Float, arm, "armature stabilizes light links")
        if p.HasAPI(UsdPhysics.MassAPI):
            m = _attr(p, "physics:mass", None)
            if m is not None and 0 < m * kg < mass_floor * 0.999:
                f = mass_floor / (m * kg)
                setattr_(p, "physics:mass", Sdf.ValueTypeNames.Float, mass_floor / kg, f"{m * kg * 1000:.0f} g link: too light for the solver")
                I = _attr(p, "physics:diagonalInertia", None)
                if I is not None:
                    setattr_(p, "physics:diagonalInertia", Sdf.ValueTypeNames.Float3, Gf.Vec3f(*(float(v) * f for v in I)), "inertia scaled with the mass")
    summary = {}
    for c in changes:
        key = c["attr"].split(":")[-1] if c["attr"] != "apiSchemas" else "nested articulation root"
        summary[key] = summary.get(key, 0) + 1
    return {"ok": True, "applied": bool(apply), "changes": len(changes), "summary": summary, "details": changes[:80],
            "truncated": truncated, "edit_target": st.GetEditTarget().GetLayer().identifier,
            "note": "overrides in the edit target; referenced robot assets unchanged" + ("" if apply else "; nothing changed yet (preview)")}


def backup_stage(tag="manual"):
    """Public alias: `from helpers import *` skips underscore names."""
    return _backup_stage(tag)


# ---------------- ROS 2 bridge ----------------
def ros_bridge_enable(distro="humble", domain_id=None, profile=None, isaac_dir=None):
    """Point this Isaac Sim process at its bundled ROS 2 libraries and enable isaacsim.ros2.bridge.
    On Windows the process environment (PATH, RMW, Fast DDS profile, domain) can still be set at runtime; on Linux
    LD_LIBRARY_PATH cannot, so launch with `simwright isaac launch --ros` there."""
    import omni.kit.app
    em = omni.kit.app.get_app().get_extension_manager()
    core = None
    for e in em.get_extensions():
        if e.get("name") == "isaacsim.ros2.core" and e.get("path"):
            core = e["path"]
            break
    if not core and isaac_dir:
        core = os.path.join(isaac_dir, "exts", "isaacsim.ros2.core")
    lib = os.path.join(core, distro, "lib") if core else None
    already = em.is_extension_enabled("isaacsim.ros2.bridge")
    env_before = {k: os.environ.get(k) for k in ("ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "FASTRTPS_DEFAULT_PROFILES_FILE")}
    os.environ["ROS_DISTRO"] = distro
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    if profile:
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    if domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(int(domain_id))
    if lib and os.path.isdir(lib) and os.name == "nt":
        if lib.lower() not in os.environ.get("PATH", "").lower():
            os.environ["PATH"] = lib + ";" + os.environ.get("PATH", "")
        try:
            os.add_dll_directory(lib)
        except Exception:
            pass
    ok = em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    import omni.graph.core as og
    have = bool(og.get_node_type("isaacsim.ros2.bridge.ROS2PublishClock"))
    out = {"ok": bool(ok and have), "bridge_enabled": em.is_extension_enabled("isaacsim.ros2.bridge"), "node_types": have,
           "distro": distro, "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"), "profile": os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"),
           "lib": lib, "was_enabled": already}
    if already and env_before.get("ROS_DISTRO") not in (None, distro):
        out["warning"] = f"the bridge was already running with ROS_DISTRO={env_before.get('ROS_DISTRO')}: restart Isaac Sim with `simwright isaac launch --ros` to switch"
    return out


def _ros_name(path):
    n = path.rstrip("/").split("/")[-1]
    n = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", n).lower()
    return re.sub(r"[^a-z0-9_]+", "_", n).strip("_") or "robot"


def _robot_of(root_path):
    """The robot's own prim for an articulation root (NVIDIA assets put the root on .../root_joint or .../Geometry)."""
    st = _stage()
    p = st.GetPrimAtPath(root_path)
    while p and p.GetParent() and (p.IsA(UsdPhysics.Joint) or p.GetName() in ("Geometry", "geometry", "root_joint", "base_link")):
        p = p.GetParent()
    return p


def ros_bridge_graph(robots=None, clock=True, joint_states=True, joint_commands=True, tf=True, cameras=None,
                     domain_id=None, graph_path="/World/SimwrightROS", persist=False, width=640, height=480, camera_hz_skip=1):
    """Build an Action Graph that connects the open stage to ROS 2:
    /clock; per robot /<robot>/joint_states (and /<robot>/joint_command -> articulation controller); /tf for every robot;
    optional cameras (list of camera prim paths) -> /<camera>/rgb + camera_info. robots: articulation root paths
    (default: every articulation). persist=False puts the graph on the session layer (the scene file is unchanged)."""
    import omni.graph.core as og
    st = _stage()
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    if not og.get_node_type("isaacsim.ros2.bridge.ROS2PublishClock"):
        return {"ok": False, "error": "the ROS 2 bridge is not enabled in this Isaac Sim: run ros_bridge_enable() (simwright ros bridge enable)"}
    if robots is None:
        robots = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    cameras = list(cameras or [])
    K = og.Controller.Keys
    layer = st.GetRootLayer() if persist else st.GetSessionLayer()
    nodes, values, connects, topics, skipped = [], [], [], [], []
    with Usd.EditContext(st, layer):
        if st.GetPrimAtPath(graph_path):
            st.RemovePrim(graph_path)
        nodes += [("tick", "omni.graph.action.OnPlaybackTick"), ("time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                  ("ctx", "isaacsim.ros2.bridge.ROS2Context")]
        if domain_id is not None:
            values += [("ctx.inputs:useDomainIDEnvVar", False), ("ctx.inputs:domain_id", int(domain_id))]
        if clock:
            nodes.append(("clock", "isaacsim.ros2.bridge.ROS2PublishClock"))
            connects += [("tick.outputs:tick", "clock.inputs:execIn"), ("time.outputs:simulationTime", "clock.inputs:timeStamp"),
                         ("ctx.outputs:context", "clock.inputs:context")]
            topics.append({"topic": "/clock", "type": "rosgraph_msgs/msg/Clock", "dir": "out"})
        names = {}
        for i, r in enumerate(robots):
            rp = st.GetPrimAtPath(r)
            if not rp or not rp.IsValid():
                skipped.append({"robot": r, "why": "prim not found"})
                continue
            robot = _robot_of(r)
            nm = _ros_name(str(robot.GetPath()))
            while nm in names.values():
                nm += "_%d" % i
            names[r] = nm
            if joint_states:
                nodes.append((f"js{i}", "isaacsim.ros2.bridge.ROS2PublishJointState"))
                values += [(f"js{i}.inputs:topicName", f"/{nm}/joint_states"), (f"js{i}.inputs:stageMetersPerUnit", float(smpu))]
                connects += [("tick.outputs:tick", f"js{i}.inputs:execIn"), ("time.outputs:simulationTime", f"js{i}.inputs:timeStamp"),
                             ("ctx.outputs:context", f"js{i}.inputs:context")]
                topics.append({"topic": f"/{nm}/joint_states", "type": "sensor_msgs/msg/JointState", "dir": "out", "robot": r})
            if joint_commands:
                nodes += [(f"sub{i}", "isaacsim.ros2.bridge.ROS2SubscribeJointState"), (f"art{i}", "isaacsim.core.nodes.IsaacArticulationController")]
                values += [(f"sub{i}.inputs:topicName", f"/{nm}/joint_command")]
                connects += [("tick.outputs:tick", f"sub{i}.inputs:execIn"), ("ctx.outputs:context", f"sub{i}.inputs:context"),
                             ("tick.outputs:tick", f"art{i}.inputs:execIn"),
                             (f"sub{i}.outputs:jointNames", f"art{i}.inputs:jointNames"),
                             (f"sub{i}.outputs:positionCommand", f"art{i}.inputs:positionCommand"),
                             (f"sub{i}.outputs:velocityCommand", f"art{i}.inputs:velocityCommand"),
                             (f"sub{i}.outputs:effortCommand", f"art{i}.inputs:effortCommand")]
                topics.append({"topic": f"/{nm}/joint_command", "type": "sensor_msgs/msg/JointState", "dir": "in", "robot": r})
        if tf and names:
            nodes.append(("tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
            connects += [("tick.outputs:tick", "tf.inputs:execIn"), ("time.outputs:simulationTime", "tf.inputs:timeStamp"),
                         ("ctx.outputs:context", "tf.inputs:context")]
            topics.append({"topic": "/tf", "type": "tf2_msgs/msg/TFMessage", "dir": "out"})
        for j, cam in enumerate(cameras):
            cp = st.GetPrimAtPath(cam)
            if not cp or not cp.IsA(UsdGeom.Camera):
                skipped.append({"camera": cam, "why": "not a Camera prim"})
                continue
            cn = _ros_name(cam)
            nodes += [(f"rp{j}", "isaacsim.core.nodes.IsaacCreateRenderProduct"), (f"rgb{j}", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                      (f"info{j}", "isaacsim.ros2.bridge.ROS2CameraHelper")]
            values += [(f"rp{j}.inputs:width", int(width)), (f"rp{j}.inputs:height", int(height)),
                       (f"rgb{j}.inputs:type", "rgb"), (f"rgb{j}.inputs:topicName", f"/{cn}/rgb"), (f"rgb{j}.inputs:frameId", cn),
                       (f"rgb{j}.inputs:frameSkipCount", int(camera_hz_skip)),
                       (f"info{j}.inputs:type", "camera_info"), (f"info{j}.inputs:topicName", f"/{cn}/camera_info"), (f"info{j}.inputs:frameId", cn)]
            connects += [("tick.outputs:tick", f"rp{j}.inputs:execIn"), (f"rp{j}.outputs:execOut", f"rgb{j}.inputs:execIn"),
                         (f"rp{j}.outputs:execOut", f"info{j}.inputs:execIn"), (f"rp{j}.outputs:renderProductPath", f"rgb{j}.inputs:renderProductPath"),
                         (f"rp{j}.outputs:renderProductPath", f"info{j}.inputs:renderProductPath"),
                         ("ctx.outputs:context", f"rgb{j}.inputs:context"), ("ctx.outputs:context", f"info{j}.inputs:context")]
            topics += [{"topic": f"/{cn}/rgb", "type": "sensor_msgs/msg/Image", "dir": "out", "camera": cam},
                       {"topic": f"/{cn}/camera_info", "type": "sensor_msgs/msg/CameraInfo", "dir": "out", "camera": cam}]
        og.Controller.edit({"graph_path": graph_path, "evaluator_name": "execution"}, {K.CREATE_NODES: nodes})
        missing = []
        for addr, val in values:
            node, attr = addr.split(".", 1)
            n = og.Controller.node(f"{graph_path}/{node}")
            if n.get_attribute_exists(attr):
                og.Controller.set(og.Controller.attribute(f"{graph_path}/{node}.{attr}"), val)
            else:
                missing.append(addr)
        ok_conn = []
        for a, b in connects:
            try:
                og.Controller.connect(f"{graph_path}/{a}", f"{graph_path}/{b}")
                ok_conn.append(1)
            except Exception as e:
                missing.append(f"{a}->{b}: {e}")
        # relationship targets (articulations, TF prims, cameras) after the nodes exist
        for i, r in enumerate(robots):
            if r not in names:
                continue
            for node in ([f"js{i}"] if joint_states else []) + ([f"art{i}"] if joint_commands else []):
                rel = st.GetPrimAtPath(f"{graph_path}/{node}").GetRelationship("inputs:targetPrim")
                if rel:
                    rel.SetTargets([Sdf.Path(r)])
                else:
                    missing.append(f"{node}.inputs:targetPrim")
        if tf and names:
            rel = st.GetPrimAtPath(f"{graph_path}/tf").GetRelationship("inputs:targetPrims")
            if rel:
                rel.SetTargets([_robot_of(r).GetPath() for r in names])
            else:
                missing.append("tf.inputs:targetPrims")
        for j, cam in enumerate(cameras):
            rel = st.GetPrimAtPath(f"{graph_path}/rp{j}").GetRelationship("inputs:cameraPrim")
            if rel:
                rel.SetTargets([Sdf.Path(cam)])
    return {"ok": not missing, "graph": graph_path, "layer": "scene file" if persist else "session (not saved)",
            "domain_id": domain_id if domain_id is not None else os.environ.get("ROS_DOMAIN_ID", "0"),
            "topics": topics, "robots": names, "skipped": skipped, "problems": missing[:30],
            "note": "topics publish while the simulation is playing"}


# ---------------- teleoperation (Rokoko suit + gloves -> G1, WBC-AGILE legs) ----------------
def _teleop_rt():
    import importlib
    from simwright.copilot.teleop import runtime as rt, rokoko, retarget_g1, onnx_np
    if getattr(rt, "_SESSION", None) is None:
        for m in (rokoko, retarget_g1, onnx_np, rt):
            importlib.reload(m)
    return rt


def teleop_g1_start(robot=None, port=14043, walk=True, physics_hz=200, actor=None, hands=True, arms=True):
    return _teleop_rt().start(robot=robot, port=port, walk=walk, physics_hz=physics_hz, actor=actor, hands=hands, arms=arms)


def teleop_g1_status():
    return _teleop_rt().status()


def teleop_g1_calibrate():
    return _teleop_rt().calibrate()


def teleop_g1_command(vx=0.0, vy=0.0, wz=0.0, h=None, clear=False):
    return _teleop_rt().command(vx, vy, wz, h, clear)


def teleop_g1_stop():
    return _teleop_rt().stop()


def teleop_scene_g1(path, asset, table=True, save=True):
    """A light scene for real-time teleop: ground, lights, the G1 asset (pinned until teleop releases it), a table with
    graspable objects. GPU dynamics, 200 Hz, articulation solver 8/4 (what WBC-AGILE trained with)."""
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, PhysxSchema, Gf, Sdf
    ctx = omni.usd.get_context()
    ctx.new_stage()
    st = ctx.get_stage()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
    world = UsdGeom.Xform.Define(st, "/World"); st.SetDefaultPrim(world.GetPrim())
    scene = UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); scene.CreateGravityMagnitudeAttr(9.81)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    ps.CreateEnableGPUDynamicsAttr(True); ps.CreateBroadphaseTypeAttr("GPU"); ps.CreateSolverTypeAttr("TGS"); ps.CreateTimeStepsPerSecondAttr(200)
    ps.CreateEnableCCDAttr(False)
    g = UsdGeom.Cube.Define(st, "/World/Ground"); g.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(g).SetScale(Gf.Vec3f(40, 40, 0.1)); UsdGeom.XformCommonAPI(g).SetTranslate(Gf.Vec3d(0, 0, -0.05))
    g.CreateDisplayColorAttr([Gf.Vec3f(0.42, 0.43, 0.45)])
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    mat = UsdShade_material = None
    UsdLux.DomeLight.Define(st, "/World/Dome").CreateIntensityAttr(900)
    d = UsdLux.DistantLight.Define(st, "/World/Sun"); d.CreateIntensityAttr(2500); UsdGeom.XformCommonAPI(d).SetRotate(Gf.Vec3f(-45, 0, 30))
    robot = st.DefinePrim("/World/G1", "Xform")
    robot.GetReferences().AddReference(asset)
    UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0, 0, 0.793))   # pelvis-root asset: standing pelvis height
    objects = []
    if table:
        def box(path, size, pos, color, dynamic=False, mass=None):
            c = UsdGeom.Cube.Define(st, path); c.CreateSizeAttr(1.0)
            UsdGeom.XformCommonAPI(c).SetScale(Gf.Vec3f(*size)); UsdGeom.XformCommonAPI(c).SetTranslate(Gf.Vec3d(*pos))
            c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
            if dynamic:
                UsdPhysics.RigidBodyAPI.Apply(c.GetPrim()); UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass or 0.2)
            objects.append(path)
        box("/World/Table/Top", (0.6, 1.0, 0.04), (0.55, 0, 0.74), (0.55, 0.42, 0.3))
        for i, (x, y) in enumerate(((0.3, 0.45), (0.3, -0.45), (0.8, 0.45), (0.8, -0.45))):
            box(f"/World/Table/Leg{i}", (0.04, 0.04, 0.72), (x, y, 0.36), (0.3, 0.3, 0.3))
        box("/World/Objects/Cube", (0.05, 0.05, 0.05), (0.42, 0.12, 0.785), (0.91, 0.4, 0.29), True, 0.1)
        box("/World/Objects/Block", (0.04, 0.12, 0.04), (0.45, -0.15, 0.78), (0.2, 0.5, 0.75), True, 0.15)
        box("/World/Objects/Bin", (0.16, 0.2, 0.06), (0.62, 0.0, 0.79), (0.3, 0.6, 0.35), True, 0.4)
    for p in st.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            a = PhysxSchema.PhysxArticulationAPI.Apply(p)
            a.CreateSolverPositionIterationCountAttr(8); a.CreateSolverVelocityIterationCountAttr(4)
    if save:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        st.GetRootLayer().Export(path)
        ctx.open_stage(path)
    return {"ok": True, "path": path, "asset": asset, "objects": objects}


def _marker_path():
    return os.path.join(os.path.expanduser("~"), ".simwright", "physics_check.json")


def _backup_stage(tag="physics-check"):
    """Export the root layer (with unsaved edits) next to Simwright's state before anything risky."""
    st = _stage()
    lay = st.GetRootLayer()
    base = os.path.splitext(os.path.basename(lay.realPath or lay.identifier or "anon"))[0] or "anon"
    d = os.path.join(os.path.expanduser("~"), ".simwright", "backups")
    os.makedirs(d, exist_ok=True)
    keep = sorted(f for f in os.listdir(d) if f.startswith(base + "-"))
    for old in keep[:-8]:  # keep the 8 newest per scene
        try:
            os.remove(os.path.join(d, old))
        except OSError:
            pass
    ext = os.path.splitext(lay.realPath or "")[1] or ".usda"
    path = os.path.join(d, f"{base}-{time.strftime('%Y%m%d-%H%M%S')}-{tag}{ext if ext in ('.usd', '.usda', '.usdc') else '.usda'}")
    ok = lay.Export(path)
    return {"path": path if ok else None, "dirty": bool(lay.dirty), "stage": lay.realPath or lay.identifier}


async def physics_check(seconds=3.0, floor_z=0.0, keep_playing=False, force=False, backup=True):
    """Play, step, and report bodies that went NaN, fell through the floor or flew away.
    Before playing: preflight for known PhysX crash setups (blocks unless force=True) and a backup of the stage
    with its unsaved edits; a marker file records the run so a crash is detected afterwards."""
    import json as _json
    st = _stage()
    issues = []
    try:
        issues += stage_health()["issues"]
    except Exception as e:
        issues.append({"severity": "warning", "issue": f"static checks failed: {e}"})
    pf = physics_preflight()
    issues += pf["issues"]
    if not any(p.GetTypeName() == "PhysicsScene" for p in st.Traverse()):
        issues.append({"severity": "error", "issue": "no PhysicsScene prim: nothing will simulate",
                       "fix": "add a PhysicsScene (UsdPhysics.Scene.Define(stage, '/physicsScene'))"})
    if not pf["ok"] and not force:
        return {"ok": False, "played": False, "issues": issues[:40],
                "note": "Not played: the scene has setups known to crash PhysX (severity crash-risk). Fix them, or rerun with force=True."}
    bk = _backup_stage() if backup else {"path": None}
    tl = omni.timeline.get_timeline_interface()
    was_playing = tl.is_playing()
    try:
        os.makedirs(os.path.dirname(_marker_path()), exist_ok=True)
        with open(_marker_path(), "w") as f:
            _json.dump({"state": "running", "stage": bk.get("stage"), "backup": bk.get("path"), "kit_pid": os.getpid(),
                        "started": time.time(), "seconds": seconds}, f)
    except Exception:
        pass
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    paths = _rigid_paths()
    start = {p: _pose(p) for p in paths}
    tl.play()
    for _ in range(3):                      # let PhysX parse the stage before measuring
        await _app().next_update_async()
    t0, frames = tl.get_current_time(), 0
    wall = time.time()
    while tl.get_current_time() - t0 < seconds and time.time() - wall < seconds * 20 + 30:
        await _app().next_update_async()
        frames += 1
    end = {p: _pose(p) for p in paths}
    sim_t = max(tl.get_current_time() - t0, 1e-6)
    if not keep_playing and not was_playing:
        tl.pause()                          # finish the in-flight step before tearing PhysX down
        for _ in range(2):
            await _app().next_update_async()
        tl.stop()
        for _ in range(3):
            await _app().next_update_async()
    try:
        with open(_marker_path(), "w") as f:
            _json.dump({"state": "done", "stage": bk.get("stage"), "backup": bk.get("path"), "kit_pid": os.getpid(), "finished": time.time()}, f)
    except Exception:
        pass
    up = 2 if UsdGeom.GetStageUpAxis(st) == "Z" else 1
    for p in paths:
        a, b = start[p], end[p]
        if any(math.isnan(v) or math.isinf(v) for v in b):
            issues.append({"severity": "error", "body": p, "issue": "pose became NaN/Inf (solver blew up)",
                           "fix": "check inertia/mass, overlapping colliders at start, joint drive gains"})
            continue
        d = math.dist(a, b) * mpu
        if b[up] * mpu < floor_z - 0.5 and a[up] * mpu >= floor_z - 0.05:
            issues.append({"severity": "error", "body": p, "issue": f"fell through the floor ({b[up]*mpu:.2f} m)",
                           "fix": "ground needs a collider; robot root must start above the floor; check collision approximation"})
        elif d / sim_t > 25:
            issues.append({"severity": "error", "body": p, "issue": f"moved {d:.1f} m in {sim_t:.1f} s (exploded)",
                           "fix": "interpenetrating colliders at spawn or bad inertia"})
    return {"ok": not any(i["severity"] in ("error", "crash-risk") for i in issues), "played": True, "bodies_checked": len(paths),
            "sim_seconds": round(sim_t, 2), "frames": frames, "issues": issues[:40], "lights": light_check(), "backup": bk.get("path")}


async def capture(path, frames_before=4):
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    for _ in range(frames_before):
        await _app().next_update_async()
    vp = get_active_viewport()
    if vp is None:
        return {"ok": False, "error": "no active viewport (headless without a viewport?)"}
    path = os.path.abspath(path)
    cap = capture_viewport_to_file(vp, file_path=path, is_hdr=False)
    await cap.wait_for_result()
    size, last, stable = 0, -1, 0
    for _ in range(240):  # the PNG is written after the capture resolves; wait until its size settles
        await _app().next_update_async()
        size = os.path.getsize(path) if os.path.exists(path) else 0
        stable = stable + 1 if size and size == last else 0
        last = size
        if stable >= 3:
            break
    return {"ok": size > 0, "path": path, "bytes": size,
            "hint": "" if size > 150_000 else "small frame: probably black or empty; check lights and camera"}


async def record(folder, seconds=4.0, fps=15):
    """Play and save a PNG sequence; the host stitches it into an MP4."""
    os.makedirs(folder, exist_ok=True)
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    t0, n, next_t = tl.get_current_time(), 0, 0.0
    files = []
    while tl.get_current_time() - t0 < seconds and n < seconds * fps + 5:
        await _app().next_update_async()
        if tl.get_current_time() - t0 >= next_t:
            f = os.path.join(folder, f"frame_{n:05d}.png")
            r = await capture(f, frames_before=0)
            if r.get("ok"):
                files.append(f)
            n += 1
            next_t += 1.0 / fps
    tl.pause()
    return {"ok": bool(files), "frames": len(files), "folder": folder, "fps": fps}


def _list(url):
    import omni.client
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        return []
    return [(e.relative_path, bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)) for e in entries]


def asset_search(keywords, roots=("/Isaac/Props", "/Isaac/Environments", "/Isaac/Robots", "/Isaac/People"), max_depth=4, limit=25):
    """Find USD assets whose path contains every keyword (case-insensitive)."""
    base = assets_root()
    if not base:
        return {"ok": False, "error": "Isaac asset root not configured (Nucleus / S3 / local assets)"}
    if "index" not in _ASSET_CACHE:
        found, frontier = [], [(base + r, 0) for r in roots]
        while frontier and len(found) < 20000:
            url, depth = frontier.pop()
            for name, is_dir in _list(url):
                full = url.rstrip("/") + "/" + name
                if is_dir and depth < max_depth and not name.startswith((".", "Materials", "Textures", "textures", "materials")):
                    frontier.append((full, depth + 1))
                elif name.lower().endswith((".usd", ".usda", ".usdc", ".usdz")) and not name.startswith("."):
                    found.append(full)
        _ASSET_CACHE["index"] = found
    kws = [k.lower() for k in ([keywords] if isinstance(keywords, str) else keywords)]
    hits = [u for u in _ASSET_CACHE["index"] if all(k in u.lower() for k in kws)]
    hits.sort(key=lambda u: (len(u), u))
    return {"ok": True, "root": base, "indexed": len(_ASSET_CACHE["index"]), "hits": hits[:limit]}


def _bbox(prim):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return r


# ---------------- placement geometry (pure numpy core; USD only for gathering triangles) ----------------
def ray_down_heights(tris, pts, z_ceiling=float("inf")):
    """Highest surface z under each (x, y) in `pts`, below z_ceiling, from triangles `tris` (N,3,3) in world units.
    Vertical ray cast: a point hits a triangle when it lies inside the triangle's XY projection (barycentric test).
    Returns a list with a float per point, or None where nothing is below."""
    import numpy as np
    tris = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
    out = []
    if not len(tris):
        return [None] * len(pts)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    v0, v1 = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2]
    den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
    ok = np.abs(den) > 1e-14  # skip vertical / degenerate triangles: they are walls, not surfaces
    for x, y in pts:
        v2x, v2y = x - a[:, 0], y - a[:, 1]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = (v2x * v1[:, 1] - v1[:, 0] * v2y) / den
            v = (v0[:, 0] * v2y - v2x * v0[:, 1]) / den
            inside = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        if not inside.any():
            out.append(None)
            continue
        z = a[inside, 2] + u[inside] * (b[inside, 2] - a[inside, 2]) + v[inside] * (c[inside, 2] - a[inside, 2])
        z = z[z <= z_ceiling + 1e-9]
        out.append(float(z.max()) if len(z) else None)
    return out


def footprint_samples(mn, mx, inset=0.08, n=3):
    """n x n grid over the footprint rectangle, inset a little so edge-on contacts don't count."""
    xs = [mn[0] + (mx[0] - mn[0]) * (inset + (1 - 2 * inset) * i / (n - 1)) for i in range(n)]
    ys = [mn[1] + (mx[1] - mn[1]) * (inset + (1 - 2 * inset) * j / (n - 1)) for j in range(n)]
    return [(x, y) for x in xs for y in ys]


def rest_height(tris, mn, mx, z_ceiling=float("inf"), n=3):
    """Where an object with world footprint [mn, mx] comes to rest when lowered onto `tris`:
    the highest surface under any footprint sample (a box lands on its highest support).
    Returns (z or None, supported_fraction)."""
    hs = ray_down_heights(tris, footprint_samples(mn, mx, n=n), z_ceiling)
    hits = [h for h in hs if h is not None]
    if not hits:
        return None, 0.0
    top = max(hits)
    tol = max(1e-6, 0.005 * (abs(top) if abs(top) < 1 else 1) + 0.002)   # ~5 mm in meter-scale stages
    touching = sum(1 for h in hits if h >= top - tol)   # samples actually under the surface it rests on
    return top, touching / len(hs)


def _tris_world(prims, max_tris=400000, budget_s=6.0, exclude=None, xy_box=None):
    """World-space triangles of every Mesh under `prims` (instance proxies included), fan-triangulated.
    Bounded by max_tris and a time budget so it can never hang Kit. xy_box=(xmin,ymin,xmax,ymax) skips meshes
    whose world bounds miss it."""
    import numpy as np
    t0 = time.time()
    xc = UsdGeom.XformCache()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    chunks, n, truncated = [], 0, False
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    ex = str(exclude) if exclude else None
    for root in prims:
        for p in Usd.PrimRange(root, pred):
            if time.time() - t0 > budget_s or n > max_tris:
                truncated = True
                break
            if ex and (str(p.GetPath()) == ex or str(p.GetPath()).startswith(ex + "/")):
                continue
            if not p.IsA(UsdGeom.Mesh):
                continue
            im = UsdGeom.Imageable(p)
            if im.ComputeVisibility() == UsdGeom.Tokens.invisible or im.ComputePurpose() in (UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy):
                continue
            if xy_box is not None:
                r = cache.ComputeWorldBound(p).ComputeAlignedRange()
                if r.IsEmpty() or r.GetMax()[0] < xy_box[0] or r.GetMin()[0] > xy_box[2] or r.GetMax()[1] < xy_box[1] or r.GetMin()[1] > xy_box[3]:
                    continue
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get()
            cnt = m.GetFaceVertexCountsAttr().Get()
            idx = m.GetFaceVertexIndicesAttr().Get()
            if not pts or not cnt or not idx:
                continue
            P = np.asarray(pts, dtype=np.float64)
            M = np.asarray(xc.GetLocalToWorldTransform(p), dtype=np.float64)  # row-vector convention
            Pw = P @ M[:3, :3] + M[3, :3]
            cnt = np.asarray(cnt); idx = np.asarray(idx)
            tri = []
            o = 0
            if (cnt == 3).all():
                tri = idx.reshape(-1, 3)
            else:
                rows = []
                for c in cnt:
                    for k in range(1, c - 1):
                        rows.append((idx[o], idx[o + k], idx[o + k + 1]))
                    o += c
                tri = np.asarray(rows, dtype=np.int64) if rows else np.zeros((0, 3), np.int64)
            if len(tri):
                chunks.append(Pw[tri])
                n += len(tri)
        if truncated:
            break
    T = np.concatenate(chunks) if chunks else np.zeros((0, 3, 3))
    return T, {"triangles": int(len(T)), "truncated": truncated, "seconds": round(time.time() - t0, 2)}


def _world_to_parent_delta(prim, dworld):
    """A world-space displacement expressed in the prim's parent frame (handles moved / rotated / scaled groups)."""
    parent = prim.GetParent()
    if not parent or parent.IsPseudoRoot():
        return Gf.Vec3d(dworld)
    M = UsdGeom.XformCache().GetLocalToWorldTransform(parent)
    return M.GetInverse().TransformDir(Gf.Vec3d(dworld))


def _parent_scale(prim):
    parent = prim.GetParent()
    if not parent or parent.IsPseudoRoot():
        return 1.0
    M = UsdGeom.XformCache().GetLocalToWorldTransform(parent)
    return (Gf.Vec3d(M.GetRow3(0)).GetLength() + Gf.Vec3d(M.GetRow3(1)).GetLength() + Gf.Vec3d(M.GetRow3(2)).GetLength()) / 3.0


def place_asset(path, url, x, y, yaw_deg=0.0, fit=None, z=0.0, kind=""):
    """World-composer placement: reference `url` (units/up axis corrected), scale uniformly so its footprint fits
    `fit`=[w, d] (meters), and lower it onto the floor under (x, y) (or rest it at `z` if z > 0). Without a url,
    a box of fit=[w, d, h] stands in. Shares place_usd's math, so groups and raised floors are handled."""
    st = _stage()
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    if url:
        r = place_usd(path, url, x, y, yaw_deg, z=(z if z else None))
        if not r.get("ok") or not fit:
            if kind and r.get("ok"):
                st.GetPrimAtPath(path).CreateAttribute("simwright:kind", Sdf.ValueTypeNames.String).Set(kind)
            return r
        w, d = r["size_m"][0], r["size_m"][1]
        cur, want = sorted([w, d]), sorted([float(fit[0]), float(fit[1])])
        s_ = min(want[1] / max(cur[1], 1e-6), want[0] / max(cur[0], 1e-6))
        pr = st.GetPrimAtPath(path)
        ops = {op.GetOpName(): op for op in UsdGeom.Xformable(pr).GetOrderedXformOps()}
        sop = ops.get("xformOp:scale")
        if sop is not None and abs(s_ - 1.0) > 1e-3:
            sop.Set(Gf.Vec3f(*(v * s_ for v in sop.Get())))
            r = place_usd(path, url, x, y, yaw_deg, z=(z if z else None), target_size=max(r["size_m"]) * s_)
        r["fit_scale"] = round(s_, 4)
        if kind:
            st.GetPrimAtPath(path).CreateAttribute("simwright:kind", Sdf.ValueTypeNames.String).Set(kind)
        return r
    holder = UsdGeom.Xform.Define(st, path)
    w, d, h = (list(fit) + [0.8] * 3)[:3] if fit else (1.0, 1.0, 0.8)
    cube = UsdGeom.Cube.Define(st, path + "/box")
    cube.GetSizeAttr().Set(1.0)
    UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(w / smpu, d / smpu, h / smpu))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    api = UsdGeom.XformCommonAPI(holder)
    api.SetRotate(Gf.Vec3f(0, 0, float(yaw_deg)))
    b = _bbox(holder.GetPrim())
    mn, mx = b.GetMin(), b.GetMax()
    _ = mx
    tris, _ti = _tris_world([st.GetPseudoRoot()], budget_s=4.0, exclude=path,
                            xy_box=(x / smpu - w, y / smpu - d, x / smpu + w, y / smpu + d))
    floor, _f = rest_height(tris, (x / smpu - w / 2 / smpu, y / smpu - d / 2 / smpu), (x / smpu + w / 2 / smpu, y / smpu + d / 2 / smpu), 0.5 / smpu)
    base = (z / smpu) if z else (floor or 0.0)
    pt = Gf.Vec3d(x / smpu, y / smpu, base - mn[2] + 0.001 / smpu)
    if holder.GetPrim().GetParent() and not holder.GetPrim().GetParent().IsPseudoRoot():
        pt = UsdGeom.XformCache().GetLocalToWorldTransform(holder.GetPrim().GetParent()).GetInverse().Transform(pt)
    api.SetTranslate(pt)
    if kind:
        holder.GetPrim().CreateAttribute("simwright:kind", Sdf.ValueTypeNames.String).Set(kind)
    rr = _bbox(holder.GetPrim())
    return {"ok": True, "path": path, "url": url, "scale": 1.0, "bbox_min": [v * smpu for v in rr.GetMin()], "bbox_max": [v * smpu for v in rr.GetMax()]}

def _reference_unit_scaled(path, url, yaw_deg):
    """Reference `url` at `path` with units and up axis corrected for its parent's world scale. Returns (prim, ops)."""
    st = _stage()
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    pr = st.DefinePrim(path, "Xform")
    pr.GetReferences().ClearReferences()
    pr.GetReferences().AddReference(url)
    lay = Sdf.Layer.FindOrOpen(url)
    info = lay.pseudoRoot if lay else None
    ampu = (info.GetInfo("metersPerUnit") if info is not None and info.HasInfo("metersPerUnit") else 0.01) or 0.01
    up = info.GetInfo("upAxis") if info is not None and info.HasInfo("upAxis") else "Y"
    k = (ampu / smpu) / (_parent_scale(pr) or 1.0)   # a scaled group must not rescale the asset again
    xf = UsdGeom.Xformable(pr)
    xf.ClearXformOpOrder()
    t_op, r_op, s_op = xf.AddTranslateOp(), xf.AddRotateXYZOp(), xf.AddScaleOp()
    s_op.Set(Gf.Vec3f(k, k, k))
    r_op.Set(Gf.Vec3f(90.0 if up == "Y" else 0.0, 0.0, float(yaw_deg)))
    t_op.Set(Gf.Vec3d(0, 0, 0))
    return pr, (t_op, r_op, s_op), {"asset_mpu": ampu, "up_axis": up, "scale": k, "stage_mpu": smpu}


def _move_world(pr, t_op, dworld):
    cur = Gf.Vec3d(t_op.Get() or Gf.Vec3d(0, 0, 0))
    t_op.Set(cur + _world_to_parent_delta(pr, dworld))


def place_usd(path, url, x=None, y=None, yaw_deg=0.0, z=None, on=None, physics="auto", target_size=None, budget_s=8.0):
    """Reference a USD (local file or URL) at `path` and set it down the way a person would.
    - units and up axis corrected, also inside moved/scaled groups (positions are solved in the parent's frame)
    - (x, y) in meters: footprint center; defaults to the center of `on`
    - height: explicit `z` (meters, bottom of the object), else lowered onto the real surface under its footprint:
      the triangles of `on` (a table, shelf, pedestal) or, without `on`, the highest surface below 0.5 m (the floor).
      Ray casts through the mesh, so a lamp or back panel on the support does not lift the object.
    physics: auto (keep the asset's own; add colliders only when it has none), dynamic, static, none."""
    st = _stage()
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    pr, (t_op, r_op, s_op), unit = _reference_unit_scaled(path, url, yaw_deg)
    b = _bbox(pr)
    if b.IsEmpty():
        return {"ok": False, "path": path, "error": "asset has no renderable geometry (empty bounds)"}
    if target_size:
        ext = b.GetMax() - b.GetMin()
        k2 = (target_size / smpu) / max(ext[0], ext[1], ext[2])
        s_op.Set(Gf.Vec3f(*(v * k2 for v in s_op.Get())))
        b = _bbox(pr)
    sp = None
    if on:
        sp = st.GetPrimAtPath(on)
        if not sp or not sp.IsValid():
            return {"ok": False, "path": path, "error": f"support prim {on} not found"}
        sb = _bbox(sp)
        if x is None:
            x = (sb.GetMin()[0] + sb.GetMax()[0]) / 2 * smpu
        if y is None:
            y = (sb.GetMin()[1] + sb.GetMax()[1]) / 2 * smpu
    x, y = float(x or 0.0), float(y or 0.0)
    # 1) center the footprint on (x, y) (world), keeping the current height
    mn, mx = b.GetMin(), b.GetMax()
    _move_world(pr, t_op, Gf.Vec3d(x / smpu - (mn[0] + mx[0]) / 2, y / smpu - (mn[1] + mx[1]) / 2, 0))
    b = _bbox(pr); mn, mx = b.GetMin(), b.GetMax()
    # 2) find the rest height under the footprint
    info = {"method": "explicit z"}
    frac = None
    if z is not None:
        rest = float(z) / smpu
    else:
        pad = 0.02 / smpu
        box = (mn[0] - pad, mn[1] - pad, mx[0] + pad, mx[1] + pad)
        if sp is not None:
            tris, ti = _tris_world([sp], budget_s=budget_s, exclude=path, xy_box=box)
            rest, frac = rest_height(tris, mn, mx, _bbox(sp).GetMax()[2] + 1e-6)
            info = {"method": f"raycast onto {on}", **ti}
            if rest is None:  # footprint entirely off the support's surface
                rest = _bbox(sp).GetMax()[2]
                info["warning"] = "no surface of the support under the footprint; used its top bound"
        else:
            tris, ti = _tris_world([st.GetPseudoRoot()], budget_s=budget_s, exclude=path, xy_box=box)
            rest, frac = rest_height(tris, mn, mx, 0.5 / smpu)
            info = {"method": "raycast onto the floor (highest surface below 0.5 m)", **ti}
            if rest is None:
                rest = 0.0
                info["warning"] = "no floor mesh under the footprint; used z = 0"
    _move_world(pr, t_op, Gf.Vec3d(0, 0, rest + 0.001 / smpu - mn[2]))
    # 3) physics: keep authored physics; never nest a rigid body inside another
    every = list(Usd.PrimRange(pr, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)))  # SimReady instances its meshes
    has_body = any(p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in every)
    has_col = any(p.HasAPI(UsdPhysics.CollisionAPI) for p in every)
    added = ""
    if physics in ("static", "dynamic") or (physics == "auto" and not has_col):
        approx = "convexHull" if physics == "dynamic" else "convexDecomposition"
        for p in Usd.PrimRange(pr):  # instance proxies are read-only; only author on real prims
            if p.IsA(UsdGeom.Mesh) and not p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(p)
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(approx)
        added = "colliders"
    if physics == "dynamic" and not has_body:
        UsdPhysics.RigidBodyAPI.Apply(pr)
        UsdPhysics.MassAPI.Apply(pr).CreateDensityAttr(600.0)
        added += "+rigid body"
    rb = _bbox(pr)
    out = {"ok": True, "path": path, "url": url, **unit,
           "bbox_min": [round(v * smpu, 4) for v in rb.GetMin()], "bbox_max": [round(v * smpu, 4) for v in rb.GetMax()],
           "size_m": [round((rb.GetMax()[i] - rb.GetMin()[i]) * smpu, 4) for i in range(3)],
           "rest_on_m": round(rest * smpu, 4), "bottom_m": round(rb.GetMin()[2] * smpu, 4),
           "supported_fraction": None if frac is None else round(frac, 2), "height_method": info,
           "asset_physics": {"rigid_body": has_body, "colliders": has_col}, "added": added}
    if frac is not None and frac < 0.5:
        out["warning"] = f"only {int(frac * 100)}% of the footprint is over a surface: it overhangs or sits on an edge; move it"
    if sp is not None:
        sb = _bbox(sp)
        out["on"] = on
        out["inside_support_xy"] = bool(rb.GetMin()[0] >= sb.GetMin()[0] - 1e-3 and rb.GetMax()[0] <= sb.GetMax()[0] + 1e-3
                                        and rb.GetMin()[1] >= sb.GetMin()[1] - 1e-3 and rb.GetMax()[1] <= sb.GetMax()[1] + 1e-3)
    return out


_AXES = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}


def _rot_from_to(a, b):
    a, b = Gf.Vec3d(*a).GetNormalized(), Gf.Vec3d(*b).GetNormalized()
    return Gf.Rotation(a, b) if (a - b).GetLength() > 1e-9 else Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)


def _basis(x, y, z):
    """4x4 whose rows are the images of the local X, Y, Z axes (USD row-vector convention: p_parent = p_local * M)."""
    return Gf.Matrix4d(x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, 0, 0, 0, 1)


def _rpy(rpy):
    r = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), float(rpy[0])))
    p = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), float(rpy[1])))
    y = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(rpy[2])))
    return r * p * y   # roll, then pitch, then yaw (in the item's frame)


def mount(path_or_name, url, link, face="+z", offset=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), look=None, gap=0.0,
          kind="attach", camera_focal=18.0):
    """Mount something on a robot link so it moves with it.
    - created as a child of `link`, so its transform is in the link's frame and follows the link
    - `face` (+x/-x/+y/-y/+z/-z of the link's own bounds): the item's bottom (its -Z after the up-axis fix) is turned
      to face the link and its nearest side touches that face, centered on it; `gap` and `offset` are meters in the
      link frame, `rpy` degrees in the item frame
    - url="camera" makes a USD Camera at the face center; `look` ("+x", "-z", ...) points its view along that link axis
    - kind="attach": rigid bodies inside the item are disabled (no nested rigid bodies; its colliders join the link);
      kind="visual": physics stripped."""
    st = _stage()
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    lp = st.GetPrimAtPath(link)
    if not lp or not lp.IsValid():
        return {"ok": False, "error": f"link {link} not found"}
    name = path_or_name.split("/")[-1]
    path = f"{link}/{name}"
    L = UsdGeom.XformCache().GetLocalToWorldTransform(lp)
    lscale = (Gf.Vec3d(L.GetRow3(0)).GetLength() + Gf.Vec3d(L.GetRow3(1)).GetLength() + Gf.Vec3d(L.GetRow3(2)).GetLength()) / 3.0
    to_link = 1.0 / (smpu * lscale)               # meters -> link-local units
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    old = st.GetPrimAtPath(path)
    if old and old.IsValid():
        st.RemovePrim(path)                        # re-mounting replaces, and keeps it out of the link's bounds
    lb = cache.ComputeUntransformedBound(lp).ComputeAlignedRange()
    n = Gf.Vec3d(*_AXES[face])
    center = (lb.GetMin() + lb.GetMax()) / 2 if not lb.IsEmpty() else Gf.Vec3d(0, 0, 0)
    half = (lb.GetMax() - lb.GetMin()) / 2 if not lb.IsEmpty() else Gf.Vec3d(0, 0, 0)
    anchor = Gf.Vec3d(center[0] + n[0] * half[0], center[1] + n[1] * half[1], center[2] + n[2] * half[2])
    dot = lambda a, b: a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    if url == "camera":
        cam = UsdGeom.Camera.Define(st, path)
        cam.CreateFocalLengthAttr(float(camera_focal))
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01 / smpu, 1000.0 / smpu))
        pr = cam.GetPrim()
        v = Gf.Vec3d(*_AXES[look or face])
        up = Gf.Vec3d(0, 0, 1) if abs(v[2]) < 0.9 else Gf.Vec3d(0, 1, 0)
        zc = -v                                    # a USD camera looks down its local -Z
        xcv = Gf.Cross(up, zc).GetNormalized()
        ycv = Gf.Cross(zc, xcv)
        R = _rpy(rpy) * _basis(xcv, ycv, zc)
        pos = anchor + n * (gap * to_link) + Gf.Vec3d(*offset) * to_link
        xf = UsdGeom.Xformable(pr); xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(R * Gf.Matrix4d().SetTranslate(pos))
    else:
        pr = st.DefinePrim(path, "Xform")
        pr.GetReferences().AddReference(url)
        lay = Sdf.Layer.FindOrOpen(url)
        info = lay.pseudoRoot if lay else None
        ampu = (info.GetInfo("metersPerUnit") if info is not None and info.HasInfo("metersPerUnit") else 0.01) or 0.01
        upx = info.GetInfo("upAxis") if info is not None and info.HasInfo("upAxis") else "Y"
        k = ampu / smpu / lscale
        Ru = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90.0 if upx == "Y" else 0.0))
        a, b = Gf.Vec3d(0, 0, -1), -n                # item bottom (-Z) turned to face the link
        Rf = Gf.Matrix4d().SetRotate(Gf.Rotation(a, b)) if (a - b).GetLength() > 1e-9 else Gf.Matrix4d(1)
        M = Gf.Matrix4d().SetScale(Gf.Vec3d(k, k, k)) * Ru * _rpy(rpy) * Rf
        xf = UsdGeom.Xformable(pr); xf.ClearXformOpOrder()
        op = xf.AddTransformOp(); op.Set(M)
        rb = cache.ComputeRelativeBound(pr, lp).ComputeAlignedRange()  # item bounds in link-local units
        cmin, cmax = rb.GetMin(), rb.GetMax()
        ccen = (cmin + cmax) / 2
        along_min = min(dot(cmin, n), dot(cmax, n))  # item's side nearest the link, along the face normal
        lateral = ccen - n * dot(ccen, n)
        shift = (anchor - n * dot(anchor, n)) - lateral + n * (dot(anchor, n) - along_min + gap * to_link) + Gf.Vec3d(*offset) * to_link
        op.Set(M * Gf.Matrix4d().SetTranslate(shift))
        cache.Clear()
        for p in Usd.PrimRange(pr, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if p.IsInstanceProxy():
                continue
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            if kind == "visual" and p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    W = UsdGeom.XformCache().GetLocalToWorldTransform(pr)
    loc = UsdGeom.Xformable(pr).GetLocalTransformation()
    out = {"ok": True, "path": path, "link": link, "face": face,
           "local_translate_m": [round(v / to_link, 4) for v in loc.ExtractTranslation()],
           "world_position_m": [round(v * smpu, 4) for v in W.ExtractTranslation()], "follows_link": True}
    if url == "camera":
        out["camera_forward_world"] = [round(v, 3) for v in W.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()]
    else:
        cache.Clear()
        rb = cache.ComputeRelativeBound(pr, lp).ComputeAlignedRange()
        out["touch_gap_m"] = round((min(dot(rb.GetMin(), n), dot(rb.GetMax(), n)) - dot(anchor, n)) / to_link, 5)
        wb = _bbox(pr)
        out["world_bbox_m"] = [[round(v * smpu, 4) for v in wb.GetMin()], [round(v * smpu, 4) for v in wb.GetMax()]]
        out["physics"] = "colliders join the link" if kind == "attach" else "visual only"
    return out

def placement_audit(scope="/World", depth=2, max_items=120, budget_s=12.0, tol_m=0.01):
    """For each object `depth` levels below `scope`: is its bottom resting on the real surface under it?
    gap > tol: floating; gap < -tol: sunk into its support. Uses the same ray cast as place_usd and stops at
    budget_s (returns truncated=True) so it can never freeze Isaac Sim."""
    st = _stage()
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    root = st.GetPrimAtPath(scope)
    if not root or not root.IsValid():
        return {"ok": False, "error": f"{scope} not found"}
    t0 = time.time()
    items = [root]
    for _ in range(depth):
        items = [c for p in items for c in p.GetChildren() if c.IsActive()]
    rows, truncated = [], False
    for p in items[:max_items]:
        if time.time() - t0 > budget_s:
            truncated = True
            break
        if p.IsA(UsdGeom.Camera) or p.HasAPI(UsdLux.LightAPI):
            continue
        b = _bbox(p)
        if b.IsEmpty():
            continue
        mn, mx = b.GetMin(), b.GetMax()
        pad = 0.01 / smpu
        left = max(0.5, budget_s - (time.time() - t0))
        tris, ti = _tris_world([st.GetPseudoRoot()], budget_s=min(3.0, left), exclude=str(p.GetPath()),
                               xy_box=(mn[0] - pad, mn[1] - pad, mx[0] + pad, mx[1] + pad))
        z, frac = rest_height(tris, mn, mx, mn[2] + 0.25 / smpu)
        gap = None if z is None else (mn[2] - z) * smpu
        state = "no surface below" if gap is None else ("floating" if gap > tol_m else ("sunk" if gap < -tol_m else "resting"))
        rows.append({"path": str(p.GetPath()), "bottom_m": round(mn[2] * smpu, 4), "surface_m": None if z is None else round(z * smpu, 4),
                     "gap_m": None if gap is None else round(gap, 4), "supported_fraction": round(frac, 2), "state": state,
                     "mesh_truncated": ti["truncated"]})
    bad = [r for r in rows if r["state"] in ("floating", "sunk")]
    return {"ok": not bad, "checked": len(rows), "problems": bad[:40], "rows": rows[:max_items], "truncated": truncated,
            "seconds": round(time.time() - t0, 2), "note": "gap = bottom minus the highest surface under the footprint (within 25 cm below)"}


def build_room(width, depth, height=3.0, wall=0.1, root="/World/Room"):
    st = _stage()
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    UsdGeom.Xform.Define(st, root)
    if not any(p.GetTypeName() == "PhysicsScene" for p in st.Traverse()):
        sc = UsdPhysics.Scene.Define(st, "/World/physicsScene")
        sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        sc.CreateGravityMagnitudeAttr().Set(9.81 / mpu)
    def box(name, cx, cy, cz, sx, sy, sz):
        c = UsdGeom.Cube.Define(st, f"{root}/{name}")
        c.GetSizeAttr().Set(1.0)
        a = UsdGeom.XformCommonAPI(c)
        a.SetTranslate(Gf.Vec3d(cx / mpu, cy / mpu, cz / mpu))
        a.SetScale(Gf.Vec3f(sx / mpu, sy / mpu, sz / mpu))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        c.CreateDisplayColorAttr().Set([Gf.Vec3f(0.78, 0.78, 0.76)])
    box("floor", width / 2, depth / 2, -0.05, width, depth, 0.1)
    box("wall_s", width / 2, -wall / 2, height / 2, width + 2 * wall, wall, height)
    box("wall_n", width / 2, depth + wall / 2, height / 2, width + 2 * wall, wall, height)
    box("wall_w", -wall / 2, depth / 2, height / 2, wall, depth, height)
    box("wall_e", width + wall / 2, depth / 2, height / 2, wall, depth, height)
    if not any(p.GetTypeName() == "DomeLight" for p in st.Traverse()):
        d = UsdLux.DomeLight.Define(st, "/World/DomeLight")
        d.CreateIntensityAttr(1000.0)
    if not any(p.GetTypeName() == "DistantLight" for p in st.Traverse()):
        l = UsdLux.DistantLight.Define(st, "/World/SunLight")
        l.CreateIntensityAttr(3000.0)
        UsdGeom.XformCommonAPI(l).SetRotate(Gf.Vec3f(-45, 0, 30))
    return {"ok": True, "room": root}


def build_layout(layout, resolve_assets=True):
    """Apply a Simwright layout (from simwright.world.solve) to the open stage."""
    room = layout["room"]
    build_room(room["width"], room["depth"], room.get("height", 3.0))
    placed, missing = [], []
    for it in layout["placements"]:
        url = it.get("asset") or ""
        if not url and resolve_assets and it.get("search"):
            hits = asset_search(it["search"]).get("hits", [])
            url = hits[0] if hits else ""
        if not url:
            missing.append(it["id"])
        r = place_asset(f"/World/Layout/{it['id']}", url, it["x"], it["y"], it.get("yaw", 0.0),
                        fit=[it["w"], it["d"], it.get("h", 0.8)], z=it.get("z", 0.0), kind=it["kind"])
        placed.append({"id": it["id"], **r})
    return {"ok": True, "placed": len(placed), "fallback_boxes": missing, "items": placed}
