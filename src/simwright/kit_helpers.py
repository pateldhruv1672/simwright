"""Simwright helpers that run INSIDE Isaac Sim (Kit Python). Loaded by the simwright.copilot extension
or sent once over the Isaac Sim python_server. Uses only USD, omni.usd, omni.timeline, omni.kit.app,
omni.physx and omni.client so it works across Isaac Sim 5.x and 6.x."""
import math, os, re, time

try:
    import omni.usd, omni.timeline, omni.kit.app
    from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf, Sdf
except ImportError:  # imported outside Kit (tests): definitions only
    pass

_ASSET_CACHE = globals().get("_ASSET_CACHE") or {}


HELPERS_VERSION = "0.2.3"


def _stage():
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
        return {"ok": h["ok"], "issues": h["issues"][:15], "articulations": h["articulations"]}
    except Exception as e:
        return {"ok": None, "error": str(e)}


def _rigid_paths(limit=300):
    out = []
    for p in _stage().Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            out.append(str(p.GetPath()))
            if len(out) >= limit:
                break
    return out


def _pose(path):
    try:
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


async def physics_check(seconds=3.0, floor_z=0.0, keep_playing=False):
    """Play, step, and report bodies that went NaN, fell through the floor or flew away."""
    st = _stage()
    issues = []
    try:
        issues += stage_health()["issues"]
    except Exception as e:
        issues.append({"severity": "warning", "issue": f"static checks failed: {e}"})
    if not any(p.GetTypeName() == "PhysicsScene" for p in st.Traverse()):
        issues.append({"severity": "error", "issue": "no PhysicsScene prim: nothing will simulate",
                       "fix": "add a PhysicsScene (UsdPhysics.Scene.Define(stage, '/physicsScene'))"})
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    paths = _rigid_paths()
    tl = omni.timeline.get_timeline_interface()
    was_playing = tl.is_playing()
    start = {p: _pose(p) for p in paths}
    tl.play()
    t0, frames = tl.get_current_time(), 0
    wall = time.time()
    while tl.get_current_time() - t0 < seconds and time.time() - wall < seconds * 20 + 30:
        await _app().next_update_async()
        frames += 1
    end = {p: _pose(p) for p in paths}
    sim_t = max(tl.get_current_time() - t0, 1e-6)
    if not keep_playing and not was_playing:
        tl.stop()
        await _app().next_update_async()
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
    return {"ok": not any(i["severity"] == "error" for i in issues), "bodies_checked": len(paths), "sim_seconds": round(sim_t, 2),
            "frames": frames, "issues": issues[:40], "lights": light_check()}


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


def place_asset(path, url, x, y, yaw_deg=0.0, fit=None, z=0.0, kind=""):
    """Reference `url` under an Xform at `path`; scale so its footprint fits `fit`=[w,d] (meters); rest it on z."""
    st = _stage()
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    holder = UsdGeom.Xform.Define(st, path)
    child = st.DefinePrim(path + "/asset", "Xform")
    if url:
        child.GetReferences().AddReference(url)
    else:
        cube = UsdGeom.Cube.Define(st, path + "/asset/box")
        cube.GetSizeAttr().Set(1.0 / mpu)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    api = UsdGeom.XformCommonAPI(holder)
    api.SetRotate(Gf.Vec3f(0, 0, float(yaw_deg)))
    r = _bbox(child)
    size = r.GetSize() if not r.IsEmpty() else Gf.Vec3d(1, 1, 1)
    s = 1.0
    if fit:
        fx, fy = fit[0] / mpu, fit[1] / mpu
        cur = sorted([size[0], size[1]])
        want = sorted([fx, fy])
        if cur[1] > 0:
            s = min(want[1] / cur[1], want[0] / max(cur[0], 1e-6)) if url else 1.0
        if not url:
            UsdGeom.XformCommonAPI(UsdGeom.Xformable(child)).SetScale(Gf.Vec3f(fx, fy, (fit[2] if len(fit) > 2 else 0.8) / mpu))
    if url:
        api.SetScale(Gf.Vec3f(s, s, s))
    r = _bbox(holder.GetPrim())
    zmin = r.GetMin()[2] if not r.IsEmpty() else 0.0
    api.SetTranslate(Gf.Vec3d(x / mpu, y / mpu, z / mpu - zmin))
    if kind:
        holder.GetPrim().CreateAttribute("simwright:kind", Sdf.ValueTypeNames.String).Set(kind)
    rr = _bbox(holder.GetPrim())
    return {"path": path, "url": url, "scale": round(s, 4), "bbox_min": list(rr.GetMin()), "bbox_max": list(rr.GetMax())}


def place_usd(path, url, x=None, y=None, yaw_deg=0.0, z=None, on=None, physics="auto", target_size=None):
    """Reference a USD (local file or URL) at `path`, correct its units and up axis, rotate about Z and rest its
    bbox bottom at z (default: the floor, or the top of prim `on`), centered on (x, y) (default: the center of `on`).
    physics: auto (keep the asset's own physics, else add a static collider), dynamic, static, none."""
    st = _stage()
    smpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    pr = st.DefinePrim(path, "Xform")
    pr.GetReferences().ClearReferences()
    pr.GetReferences().AddReference(url)
    lay = Sdf.Layer.FindOrOpen(url)
    info = lay.pseudoRoot if lay else None
    ampu = (info.GetInfo("metersPerUnit") if info is not None and info.HasInfo("metersPerUnit") else 0.01) or 0.01
    up = info.GetInfo("upAxis") if info is not None and info.HasInfo("upAxis") else "Y"
    k = ampu / smpu
    xf = UsdGeom.Xformable(pr)
    xf.ClearXformOpOrder()
    t_op, r_op, s_op = xf.AddTranslateOp(), xf.AddRotateXYZOp(), xf.AddScaleOp()
    s_op.Set(Gf.Vec3f(k, k, k))
    r_op.Set(Gf.Vec3f(90.0 if up == "Y" else 0.0, 0.0, float(yaw_deg)))
    t_op.Set(Gf.Vec3d(0, 0, 0))
    b = _bbox(pr)
    if b.IsEmpty():
        return {"ok": False, "path": path, "error": "asset has no renderable geometry (empty bounds)"}
    if target_size:
        ext = b.GetMax() - b.GetMin()
        k2 = (target_size / smpu) / max(ext[0], ext[1], ext[2])
        s_op.Set(Gf.Vec3f(k * k2, k * k2, k * k2))
        b = _bbox(pr)
    support = None
    if on:
        sp = st.GetPrimAtPath(on)
        if not sp or not sp.IsValid():
            return {"ok": False, "path": path, "error": f"support prim {on} not found"}
        support = _bbox(sp)
        if x is None:
            x = (support.GetMin()[0] + support.GetMax()[0]) / 2 * smpu
        if y is None:
            y = (support.GetMin()[1] + support.GetMax()[1]) / 2 * smpu
        if z is None:
            z = support.GetMax()[2] * smpu + 0.002
    x, y, z = float(x or 0.0), float(y or 0.0), float(z or 0.0)
    mn, mx = b.GetMin(), b.GetMax()
    t_op.Set(Gf.Vec3d(x / smpu - (mn[0] + mx[0]) / 2, y / smpu - (mn[1] + mx[1]) / 2, z / smpu - mn[2]))
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
    out = {"ok": True, "path": path, "url": url, "scale": round(k, 5), "up_axis": up,
           "bbox_min": [round(v * smpu, 4) for v in rb.GetMin()], "bbox_max": [round(v * smpu, 4) for v in rb.GetMax()],
           "size_m": [round((rb.GetMax()[i] - rb.GetMin()[i]) * smpu, 4) for i in range(3)],
           "asset_physics": {"rigid_body": has_body, "colliders": has_col}, "added": added}
    if support is not None:
        out["on"] = on
        out["inside_support_xy"] = bool(rb.GetMin()[0] >= support.GetMin()[0] - 1e-3 and rb.GetMax()[0] <= support.GetMax()[0] + 1e-3
                                        and rb.GetMin()[1] >= support.GetMin()[1] - 1e-3 and rb.GetMax()[1] <= support.GetMax()[1] + 1e-3)
    return out


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
