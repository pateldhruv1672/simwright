"""Skills: markdown procedures your team writes (Agent Skills format: a folder with SKILL.md).

Install with `simwright skill add ./team-skills/ekf-tuning.md` (user scope, ~/.simwright/skills/<name>/SKILL.md)
or `--repo` (<repo>/.skills/<name>/SKILL.md, shared with everyone who clones the project).
Discovered automatically: repo .skills, .claude/skills, .agents/skills; ~/.simwright/skills, ~/.claude/skills;
NVIDIA's Isaac Sim skills under $ISAAC_SIM_DIR/skills; and Simwright's built-ins.
"""
from __future__ import annotations
import os, pathlib, re, shutil
from dataclasses import dataclass
from . import config


@dataclass
class Skill:
    name: str
    description: str
    path: pathlib.Path
    scope: str

    def body(self) -> str:
        return self.path.read_text(encoding="utf-8", errors="replace")


def parse_frontmatter(text: str) -> dict:
    m = re.match(r"^---\s*\n(.*?)\n---\s*\n", text, re.S)
    meta = {}
    if m:
        for line in m.group(1).splitlines():
            if ":" in line and not line.startswith(" "):
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip().strip("\"'")
    if "description" not in meta:
        body = text[m.end():] if m else text
        para = next((p.strip() for p in re.split(r"\n\s*\n", body) if p.strip() and not p.strip().startswith("#")), "")
        meta["description"] = re.sub(r"\s+", " ", para)[:240]
    if "name" not in meta:
        h = re.search(r"^#\s+(.+)$", text, re.M)
        if h:
            meta["title"] = h.group(1).strip()
    return meta


def slug(s: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-")[:64] or "skill"


def roots(repo: pathlib.Path | None) -> list[tuple[str, pathlib.Path]]:
    cfg = config.load()
    out = []
    if repo:
        out += [("repo", repo / ".skills"), ("repo", repo / ".claude" / "skills"), ("repo", repo / ".agents" / "skills")]
    out += [("user", config.HOME / "skills"), ("user", pathlib.Path.home() / ".claude" / "skills"),
            ("user", pathlib.Path.home() / ".agents" / "skills")]
    for key in ("isaac_sim_dir", "isaac_lab_dir"):
        d = cfg.get(key) or os.environ.get(key.upper(), "")
        if d:
            out.append(("nvidia", pathlib.Path(d) / "skills"))
    out.append(("builtin", config.DATA / "skills"))
    return out


def discover(repo: pathlib.Path | None = None) -> list[Skill]:
    seen, found = set(), []
    for scope, root in roots(repo):
        if not root.is_dir():
            continue
        for f in sorted(root.glob("*/SKILL.md")):
            meta = parse_frontmatter(f.read_text(encoding="utf-8", errors="replace"))
            name = meta.get("name") or f.parent.name
            if name in seen:
                continue  # earlier scope wins: repo > user > nvidia > builtin
            seen.add(name)
            found.append(Skill(name, meta.get("description", ""), f, scope))
    return found


def add(src: str, repo: pathlib.Path | None = None, to_repo: bool = False, name: str | None = None) -> Skill:
    p = pathlib.Path(src).expanduser()
    if not p.exists():
        raise FileNotFoundError(f"no such skill file or folder: {src}")
    md = p / "SKILL.md" if p.is_dir() else p
    if not md.is_file():
        raise FileNotFoundError(f"{p} has no SKILL.md")
    text = md.read_text(encoding="utf-8", errors="replace")
    meta = parse_frontmatter(text)
    nm = slug(name or meta.get("name") or (p.name if p.is_dir() else p.stem))
    if to_repo and not repo:
        raise ValueError("--repo needs a workspace (run inside your project)")
    dest_root = (repo / ".skills") if to_repo else (config.HOME / "skills")
    dest = dest_root / nm
    if dest.exists():
        shutil.rmtree(dest)
    if p.is_dir():
        shutil.copytree(p, dest)
    else:
        dest.mkdir(parents=True)
    if not text.startswith("---"):
        desc = meta.get("description", "")
        text = f"---\nname: {nm}\ndescription: {desc}\n---\n\n" + text
    (dest / "SKILL.md").write_text(text, encoding="utf-8")
    return Skill(nm, meta.get("description", ""), dest / "SKILL.md", "repo" if to_repo else "user")


def remove(name: str, repo: pathlib.Path | None = None) -> bool:
    for scope, root in roots(repo):
        if scope in ("repo", "user") and (root / name / "SKILL.md").is_file() and root.name in ("skills", ".skills") and ".claude" not in str(root):
            shutil.rmtree(root / name)
            return True
    return False


NL_ADD = re.compile(r"^\s*(?:please\s+)?(?:add|install)\s+(?:this\s+|the\s+)?skill\s*:?\s+(\S+)(.*)$", re.I)


def parse_nl_add(text: str):
    """'add this skill: ./x.md' -> ('./x.md', to_repo)."""
    m = NL_ADD.match(text)
    if not m:
        return None
    rest = m.group(2).lower()
    return m.group(1), ("--repo" in rest or "repo" in rest)


WORD = re.compile(r"[a-z0-9]+")
STOP = set("the a an and or to of in on for with my our your this that it is are be how what when use using".split())


def relevant(skills: list[Skill], prompt: str, k: int = 3) -> list[Skill]:
    words = {w for w in WORD.findall(prompt.lower()) if w not in STOP and len(w) > 2}
    scored = []
    for s in skills:
        hay = set(WORD.findall((s.name.replace("-", " ") + " " + s.description).lower()))
        score = len(words & hay) + 2 * sum(1 for w in words if w in s.name)
        if score >= 2:
            scored.append((score, s))
    return [s for _, s in sorted(scored, key=lambda x: -x[0])[:k]]
