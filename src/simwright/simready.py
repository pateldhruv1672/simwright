"""SimReady Central: search NVIDIA's index of simulation-ready OpenUSD assets, fetch packages, place them in Isaac Sim.

Search and package manifests come from the public SimReady Central Space (USD Search gateway,
https://huggingface.co/spaces/nvidia/simready-central). Files are downloaded straight from the contributor's
Hugging Face dataset at the revision the index registered, so an import is reproducible. Packages are cached under
~/.simwright/simready/<org>__<dataset>/<revision>/<package>/ and recorded in catalog.json with their license.
"""
from __future__ import annotations
import concurrent.futures as cf, hashlib, json, os, pathlib, re, time, urllib.parse, urllib.request
from . import config

API = os.environ.get("SIMREADY_API", "https://nvidia-simready-central.hf.space")
UA = {"User-Agent": "simwright-simready/1"}
_HF = re.compile(r"^(?:hf://|https://)huggingface\.co/datasets/([^/]+)/([^/]+)/(?:blob|resolve)/([^/]+)/(.+)$")
MARKER = "com.nvidia.simready.packaging.json"


def cache_root() -> pathlib.Path:
    return config.HOME / "simready"


def parse_url(url: str) -> dict:
    """hf:// or https blob/resolve URL -> org, dataset, revision, path (URL-decoded)."""
    m = _HF.match(url.strip())
    if not m:
        raise ValueError(f"not a Hugging Face dataset file URL: {url}")
    org, ds, rev, path = m.groups()
    return {"org": org, "dataset": ds, "revision": rev, "path": urllib.parse.unquote(path)}


def blob_url(u: dict) -> str:
    return f"https://huggingface.co/datasets/{u['org']}/{u['dataset']}/blob/{u['revision']}/{urllib.parse.quote(u['path'])}"


def resolve_url(u: dict, path: str | None = None) -> str:
    return f"https://huggingface.co/datasets/{u['org']}/{u['dataset']}/resolve/{u['revision']}/{urllib.parse.quote(path or u['path'])}"


def _http(method: str, url: str, body: dict | None = None, timeout: int = 90, params: dict | None = None):
    if params:
        url += ("&" if "?" in url else "?") + urllib.parse.urlencode({k: v for k, v in params.items() if v is not None})
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers={**UA, **({"Content-Type": "application/json"} if data else {})})
    tok = os.environ.get("HF_TOKEN")
    if tok and "huggingface.co" in url:
        req.add_header("Authorization", f"Bearer {tok}")
    for attempt in range(3):
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            if e.code in (429, 502, 503, 504) and attempt < 2:
                time.sleep(2 + 3 * attempt)
                continue
            raise RuntimeError(f"SimReady API {e.code} for {url.split('?')[0]}: {e.read().decode(errors='ignore')[:300]}")
        except urllib.error.URLError as e:
            if attempt < 2:
                time.sleep(2)
                continue
            raise RuntimeError(f"SimReady API unreachable ({e.reason}); the Space may be waking up, retry in a minute")


def _hit(src: dict, score: float | None = None, tags=None) -> dict:
    url = src.get("path") or src.get("url") or ""
    try:
        u = parse_url(url)
    except ValueError:
        return {"url": url, "score": score}
    d = src.get("usd_dimensions") or {}
    dims = None
    if d:
        dims = [round(float(d.get(f"scaled_bbox_dimension_{a}", d.get(f"bbox_dimension_{a}", 0)) or 0) * float(d.get("scene_mpu", 1) or 1), 4) for a in "xyz"]
    parts = u["path"].split("/")
    fname = parts[-1]
    folder = "/".join(parts[:-1])
    pkg = parts[0] if len(parts) > 1 else fname
    return {"name": pkg, "file": fname, "url": blob_url(u), "dataset": f"{u['org']}/{u['dataset']}", "revision": u["revision"],
            "dims_m": dims, "score": round(score, 4) if score is not None else None,
            "thumbnail": resolve_url(u, (folder + "/" if folder else "") + f".thumbs/256x256/{fname}.png"),
            **({"tags": tags} if tags else {})}


def search(query: str, limit: int = 10, max_size_m: float | None = None, min_size_m: float | None = None,
           dataset: str = "", offset: int = 0) -> dict:
    """Text search over SimReady Central. Sizes filter the largest bbox side, in meters."""
    body = {"description": query, "limit": max(1, min(int(limit), 50)), "offset": offset, "return_metadata": True,
            "return_usd_dimensions": True, "return_tags": True, "return_images": False, "deduplicate_by_hash": True}
    if dataset:
        body["filter_url_regexp"] = f".*datasets/{re.escape(dataset)}/.*"
    r = _http("POST", API + "/search_hybrid", body)
    hits = []
    for h in r.get("hits", []):
        hit = _hit(h.get("source") or {}, h.get("score"), h.get("tags"))
        dm = hit.get("dims_m")
        if dm and max_size_m and max(dm) > max_size_m:
            continue
        if dm and min_size_m and max(dm) < min_size_m:
            continue
        hits.append(hit)
    return {"ok": True, "query": query, "total": r.get("total"), "hits": hits}


def manifest(url: str) -> dict:
    """Every file in the asset's SimReady package (nearest folder with the packaging marker), with sizes."""
    u = parse_url(url)
    m = _http("GET", API + "/download/asset", params={"asset_url": blob_url(u), "manifest_only": "true"}, timeout=180)
    files = []
    for f in m.get("files", []):
        try:
            files.append({"path": parse_url(f["url"])["path"], "size": f.get("size", 0)})
        except ValueError:
            pass
    marker = next((f["path"] for f in files if f["path"].endswith("/" + MARKER) or f["path"] == MARKER), "")
    pkg_root = marker.rsplit("/", 1)[0] if "/" in marker else ""
    if not pkg_root and files:
        pkg_root = os.path.commonpath([f["path"] for f in files]) if len(files) > 1 else files[0]["path"].rsplit("/", 1)[0]
    return {"url": blob_url(u), "org": u["org"], "dataset": u["dataset"], "revision": u["revision"], "root_usd": u["path"],
            "package_root": pkg_root, "file_count": len(files), "total_size": sum(f["size"] for f in files), "files": files}


def _download(url: str, dest: pathlib.Path, size: int) -> int:
    if dest.exists() and (not size or dest.stat().st_size == size):
        return 0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    req = urllib.request.Request(url, headers=UA)
    tok = os.environ.get("HF_TOKEN")
    if tok:
        req.add_header("Authorization", f"Bearer {tok}")
    for attempt in range(3):
        try:
            with urllib.request.urlopen(req, timeout=300) as r, open(tmp, "wb") as f:
                while True:
                    b = r.read(1 << 20)
                    if not b:
                        break
                    f.write(b)
            break
        except Exception as e:
            if attempt == 2:
                raise RuntimeError(f"download failed {url}: {e}")
            time.sleep(2 + 2 * attempt)
    got = tmp.stat().st_size
    if size and got != size:
        raise RuntimeError(f"size mismatch for {dest.name}: {got} != {size}")
    os.replace(tmp, dest)
    return got


def _catalog() -> dict:
    p = cache_root() / "catalog.json"
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _license(local: pathlib.Path) -> dict:
    out = {}
    mk = local / MARKER
    if mk.exists():
        try:
            j = json.loads(mk.read_text(encoding="utf-8"))
            for k in ("license", "licenses", "name", "version", "profile", "profiles"):
                if k in j:
                    out[k] = j[k]
        except Exception:
            pass
    lic = local / ".metadata" / "LICENSE.txt"
    if lic.exists():
        out["license_text"] = next((l.strip() for l in lic.read_text(encoding="utf-8", errors="ignore").splitlines() if l.strip()), "")[:200]
        out["license_file"] = str(lic)
    return out


def fetch(url: str, workers: int = 8) -> dict:
    """Download the asset's whole SimReady package at its pinned revision; return the local root USD."""
    m = manifest(url)
    if m["total_size"] > 4 * 1024 ** 3:
        raise RuntimeError(f"package is {m['total_size'] / 1e9:.1f} GB; refusing without an explicit size override")
    base = cache_root() / f"{m['org']}__{m['dataset']}" / m["revision"][:12]
    u = {"org": m["org"], "dataset": m["dataset"], "revision": m["revision"]}
    t0, got = time.time(), 0
    with cf.ThreadPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(_download, resolve_url(u, f["path"]), base / f["path"], f["size"]) for f in m["files"]]
        for fu in cf.as_completed(futs):
            got += fu.result()
    local_pkg = base / m["package_root"] if m["package_root"] else base
    root = base / m["root_usd"]
    rec = {"url": m["url"], "dataset": f"{m['org']}/{m['dataset']}", "revision": m["revision"], "package": m["package_root"],
           "local_usd": str(root), "local_package": str(local_pkg), "files": m["file_count"], "bytes": m["total_size"],
           "fetched_at": time.strftime("%Y-%m-%dT%H:%M:%S"), **_license(local_pkg)}
    cat = _catalog()
    cat[m["url"]] = rec
    cache_root().mkdir(parents=True, exist_ok=True)
    (cache_root() / "catalog.json").write_text(json.dumps(cat, indent=2), encoding="utf-8")
    return {"ok": root.exists(), **rec, "downloaded_bytes": got, "seconds": round(time.time() - t0, 1)}


def local(url_or_path: str) -> str:
    """Local USD for a SimReady URL (fetching it if needed) or a path that already exists."""
    if os.path.exists(url_or_path):
        return str(pathlib.Path(url_or_path).resolve())
    rec = _catalog().get(blob_url(parse_url(url_or_path)))
    if rec and os.path.exists(rec["local_usd"]):
        return rec["local_usd"]
    return fetch(url_or_path)["local_usd"]


def catalog() -> list[dict]:
    return list(_catalog().values())
