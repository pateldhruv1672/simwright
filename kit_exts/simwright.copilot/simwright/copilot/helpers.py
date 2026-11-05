"""Simwright helpers that run INSIDE Isaac Sim (Kit Python). Loaded by the simwright.copilot extension
or sent once over the Isaac Sim python_server. Uses only USD, omni.usd, omni.timeline, omni.kit.app,
omni.physx and omni.client so it works across Isaac Sim 5.x and 6.x."""
import math, os, time

try:
    import omni.usd, omni.timeline, omni.kit.app
    from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf, Sdf
except ImportError:  # imported outside Kit (tests): definitions only
    pass

_ASSET_CACHE = globals().get("_ASSET_CACHE") or {}


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
    for p in st.Traverse():
        t = p.GetTypeName() or "(untyped)"
        counts[t] = counts.get(t, 0) + 1
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            joints = [c for c in Usd.PrimRange(p) if c.IsA(UsdPhysics.Joint)]
            arts.append({"path": str(p.GetPath()), "joints": len(joints)})
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid += 1
        if t.endswith("Light"):
            a = p.GetAttribute("inputs:intensity") or p.GetAttribute("intensity")
            lights.append({"path": str(p.GetPath()), "type": t, "intensity": float(a.Get()) if a and a.Get() is not None else None})
        if t == "Camera":
            cams.append(str(p.GetPath()))
        if t == "PhysicsScene":
            g = UsdPhysics.Scene(p)
            scenes.append({"path": str(p.GetPath()), "gravity_dir": list(g.GetGravityDirectionAttr().Get() or []),
                           "gravity": g.GetGravityMagnitudeAttr().Get()})
    tl = omni.timeline.get_timeline_interface()
    return {"stage": st.GetRootLayer().identifier, "up_axis": UsdGeom.GetStageUpAxis(st),
            "meters_per_unit": UsdGeom.GetStageMetersPerUnit(st), "prims": sum(counts.values()),
            "types": dict(sorted(counts.items(), key=lambda kv: -kv[1])[:25]), "articulations": arts[:max_list],
            "rigid_bodies": rigid, "lights": lights[:max_list], "cameras": cams[:max_list], "physics_scenes": scenes,
            "timeline": {"playing": tl.is_playing(), "time": tl.get_current_time()}}


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


async def physics_check(seconds=3.0, floor_z=0.0, keep_playing=False):
    """Play, step, and report bodies that went NaN, fell through the floor or flew away."""
    st = _stage()
    issues = []
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
    cap = capture_viewport_to_file(vp, file_path=path, is_hdr=False)
    await cap.wait_for_result()
    for _ in range(2):
        await _app().next_update_async()
    size = os.path.getsize(path) if os.path.exists(path) else 0
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
