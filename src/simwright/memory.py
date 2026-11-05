"""Session memory: what Simwright built and ran in this workspace, so later steps do not hallucinate paths."""
from __future__ import annotations
import json, pathlib, time


class Session:
    def __init__(self, root: pathlib.Path):
        self.dir = root / ".simwright"
        self.dir.mkdir(exist_ok=True)
        self.file = self.dir / "session.jsonl"

    def log(self, event: dict):
        event = {"t": round(time.time(), 1), **event}
        with self.file.open("a", encoding="utf-8") as f:
            f.write(json.dumps(event) + "\n")

    def events(self, limit: int = 400) -> list[dict]:
        if not self.file.exists():
            return []
        lines = self.file.read_text(encoding="utf-8").splitlines()[-limit:]
        out = []
        for l in lines:
            try:
                out.append(json.loads(l))
            except Exception:
                pass
        return out

    def summary(self) -> str:
        ev = self.events()
        files, cmds = {}, []
        for e in ev:
            if e.get("type") == "write":
                files[e["path"]] = e.get("why", "")
            elif e.get("type") == "command" and not e.get("skipped"):
                cmds.append(f"[{e.get('target')}] {e.get('cmd')} -> exit {e.get('exit')}")
        out = []
        if files:
            out.append("Files Simwright created or changed in this workspace (real paths):")
            out += [f"- {p}" + (f": {w}" if w else "") for p, w in list(files.items())[-60:]]
        if cmds:
            out.append("Recent commands:")
            out += [f"- {c}" for c in cmds[-25:]]
        return "\n".join(out)
