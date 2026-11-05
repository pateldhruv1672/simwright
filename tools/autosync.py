"""Simwright autosync: test -> build -> publish to the Mana app -> commit -> push to GitHub.

Run by the MPM node apps/simwright/console/autosync (every 15 min). Nothing ships unless pytest passes.
Never pulls or rewrites local work: it is one-way, workspace -> app storage -> GitHub.
"""
import base64, hashlib, json, os, pathlib, re, shutil, subprocess, sys, time, zipfile

REPO = pathlib.Path("/workspace/simwright")
DIST = pathlib.Path("/cloudspace/app-storage/apps/simwright/console/dist")
GH_REPO = "pateldhruv1672/simwright"


def sh(cmd, timeout=600, env=None):
    p = subprocess.run(cmd, shell=True, cwd=REPO, capture_output=True, text=True, timeout=timeout, env=env)
    return p.returncode, (p.stdout + p.stderr).strip()


def src_hash():
    h = hashlib.sha256()
    for p in sorted(REPO.rglob("*")):
        rel = p.relative_to(REPO).as_posix()
        if p.is_file() and not re.match(r"(\.git|\.venv|dist|build|\.pytest_cache)/|.*__pycache__|.*\.pyc$", rel):
            h.update(rel.encode()); h.update(p.read_bytes())
    return h.hexdigest()[:16]


def version():
    return re.search(r'^version = "([^"]+)"', (REPO / "pyproject.toml").read_text(), re.M).group(1)


def run(uid, console_path, firebase):
    now = int(time.time() * 1000)
    state = {"at": now, "metadata": {"type": "entry", "createdAt": now, "updatedAt": now}}
    base = f"{uid}/file-system/{console_path}"
    prev = firebase.read(base + "/state/autosync") or {}
    code, out = sh(f"{REPO}/.venv/bin/python -m pytest -q", timeout=900)
    tests = (out.strip().splitlines() or ["?"])[-1]
    state["tests"] = tests
    if code != 0:
        state.update(ok=False, step="tests", error=out[-1500:])
        firebase.write(base + "/state/autosync", state)
        print("SKIP: tests failing;", tests)
        return state
    ver, sh_ = version(), src_hash()
    wheel = f"simwright-{ver}-py3-none-any.whl"
    published = False
    if sh_ != prev.get("src_hash") or not (DIST / wheel).exists():
        shutil.rmtree(REPO / "dist", ignore_errors=True)
        c2, o2 = sh("uv build --wheel", timeout=600)
        if c2 != 0:
            state.update(ok=False, step="build", error=o2[-1500:])
            firebase.write(base + "/state/autosync", state)
            print("SKIP: build failed"); return state
        DIST.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(REPO / "dist" / wheel, DIST / wheel)
        shutil.copyfile(REPO / "install" / "install.ps1", DIST / "install.ps1")
        sh(f"git archive --format=zip -o /tmp/simwright-src.zip --prefix=simwright/ HEAD")
        published = True
    # commit (only when something changed) and push
    sh("git add -A")
    dirty = sh("git diff --cached --quiet")[0] != 0
    if dirty:
        files = sh("git diff --cached --name-only")[1].splitlines()
        msg = f"Autosync {ver}: " + ", ".join(sorted({f.split('/')[-1] for f in files})[:6]) + (" ..." if len(files) > 6 else "")
        sh(f"git commit -qm {json.dumps(msg)}")
    head = sh("git rev-parse --short HEAD")[1]
    from mana_oauth import Connection
    tok = Connection("github").bind()
    hdr = base64.b64encode(f"x-access-token:{tok}".encode()).decode()
    c3, o3 = sh(f'git -c http.extraheader="AUTHORIZATION: basic {hdr}" push origin main', timeout=300)
    o3 = o3.replace(tok, "***").replace(hdr, "***")
    if c3 != 0:
        state.update(ok=False, step="push", error=o3[-800:], head=head)
        firebase.write(base + "/state/autosync", state)
        print("PUSH FAILED", o3[-300:]); return state
    if published or dirty:  # src zip of the pushed HEAD
        sh(f"git archive --format=zip -o /tmp/simwright-src.zip --prefix=simwright/ HEAD")
        shutil.copyfile("/tmp/simwright-src.zip", DIST / f"simwright-{ver}-src.zip")
    # app metadata: version, tests, files, tools, skills
    sys.path.insert(0, str(REPO / "src"))
    tools = []
    try:
        code_t, out_t = sh(f"{REPO}/.venv/bin/python -c \"import json; from simwright.tools import TOOLS; print(json.dumps(list(TOOLS)))\"")
        tools = json.loads(out_t.splitlines()[-1]) if code_t == 0 else []
    except Exception:
        pass
    skills = []
    for p in sorted((REPO / "skills").glob("*/SKILL.md")):
        m = re.search(r"^description:\s*(.+)$", p.read_text(encoding="utf-8"), re.M)
        skills.append({"name": p.parent.name, "d": (m.group(1).strip() if m else "")[:160]})
    commit = sh("git log -1 --format=%h%x09%cI%x09%s")[1].split("\t")
    upd = {"version": ver, "tests": tests + " (pytest, sandbox)", "wheel": wheel, "src_zip": f"simwright-{ver}-src.zip",
           "repo": f"https://github.com/{GH_REPO} (main) · local /workspace/simwright", "metadata/updatedAt": now,
           "last_commit": {"sha": commit[0], "at": commit[1] if len(commit) > 1 else "", "message": commit[2] if len(commit) > 2 else ""}}
    if tools:
        upd["tools"] = tools
    if skills:
        upd["skills"] = skills
    firebase.update(base + "/data/capabilities", upd)
    try:  # keep the PC's Mana model link fresh (best effort: the PC may be offline)
        import relay_login
        state["relay"] = relay_login.push()
    except Exception as e:
        state["relay"] = {"ok": False, "error": str(e)[:300]}
    state.update(ok=True, step="done", version=ver, head=head, committed=dirty, published=published, src_hash=sh_,
                 pushed=("Everything up-to-date" not in o3))
    firebase.write(base + "/state/autosync", state)
    print("OK", json.dumps({k: state[k] for k in ("version", "head", "committed", "published", "pushed", "tests")}))
    return state
