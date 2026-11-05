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


HELPERS_VERSION = "0.3.0"


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
