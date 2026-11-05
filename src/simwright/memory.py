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
        files, cmds, scene = {}, [], []
        for e in ev:
            if e.get("type") == "write":
                files[e["path"]] = e.get("why", "")
            elif e.get("type") == "command" and not e.get("skipped"):
                cmds.append(f"[{e.get('target')}] {e.get('cmd')} -> exit {e.get('exit')}")
            elif e.get("type") == "scene":
                scene.append(f"{time.strftime('%m-%d %H:%M', time.localtime(e.get('t', 0)))} {e.get('what', '')}"
                             + (f" ({e['stage']})" if e.get("stage") else ""))
        out = []
        if files:
            out.append("Files Simwright created or changed in this workspace (real paths):")
            out += [f"- {p}" + (f": {w}" if w else "") for p, w in list(files.items())[-60:]]
        if cmds:
            out.append("Recent commands:")
            out += [f"- {c}" for c in cmds[-25:]]
        if scene:
            out.append("Scene changes (most recent last):")
            out += [f"- {c}" for c in scene[-30:]]
        return "\n".join(out)


NOTES_HEADER = "# Simwright project notes\n\nDecisions, scene files, robots, conventions and next steps. Written by Simwright and by you; read before every step.\n"


class ProjectMemory:
    """Durable, per-workspace memory shared by every front end (panel, terminal, MCP):
    - notes.md: dated notes (decisions, saved stages, what is next), editable by hand
    - conversation.json: the panel/terminal conversation, restored after restarts
    - history/: archived conversations with a title and summary"""

    def __init__(self, root: pathlib.Path):
        self.dir = root / ".simwright"
        self.dir.mkdir(exist_ok=True)
        self.notes_file = self.dir / "notes.md"
        self.conv_file = self.dir / "conversation.json"
        self.hist = self.dir / "history"

    # notes
    def notes(self, max_chars: int = 8000) -> str:
        if not self.notes_file.exists():
            return ""
        txt = self.notes_file.read_text(encoding="utf-8")
        return txt if len(txt) <= max_chars else "...(older notes trimmed)\n" + txt[-max_chars:]

    def add_note(self, text: str, kind: str = "note") -> str:
        text = " ".join(text.strip().split()) if "\n" not in text.strip() else text.strip()
        if not self.notes_file.exists():
            self.notes_file.write_text(NOTES_HEADER, encoding="utf-8")
        line = f"\n- {time.strftime('%Y-%m-%d %H:%M')} [{kind}] {text}"
        with self.notes_file.open("a", encoding="utf-8") as f:
            f.write(line)
        return line.strip()

    # conversation
    @staticmethod
    def _clean(messages: list[dict]) -> list[dict]:
        out = []
        for m in messages:
            if m.get("role") == "system":
                continue
            c = m.get("content")
            if isinstance(c, list):  # drop images (they are large and stale); keep the text
                c = " ".join(p.get("text", "") for p in c if isinstance(p, dict) and p.get("type") == "text") or "(image)"
                m = {**m, "content": c}
            out.append(m)
        return out

    def save_conversation(self, messages: list[dict]):
        data = {"saved": time.time(), "messages": self._clean(messages)[-80:]}
        tmp = self.conv_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(data), encoding="utf-8")
        tmp.replace(self.conv_file)

    def load_conversation(self) -> list[dict]:
        try:
            msgs = json.loads(self.conv_file.read_text(encoding="utf-8")).get("messages", [])
        except Exception:
            return []
        while msgs and msgs[0].get("role") == "tool":  # never start on an orphaned tool result
            msgs = msgs[1:]
        return msgs

    def archive(self, summary: str = "") -> dict:
        msgs = self.load_conversation()
        if not msgs:
            return {"archived": False}
        self.hist.mkdir(exist_ok=True)
        title = next((m["content"] for m in msgs if m.get("role") == "user" and isinstance(m.get("content"), str)), "conversation")[:90]
        name = time.strftime("%Y%m%d-%H%M%S") + ".json"
        (self.hist / name).write_text(json.dumps({"title": title, "summary": summary, "messages": msgs}), encoding="utf-8")
        self.conv_file.unlink(missing_ok=True)
        return {"archived": True, "file": str(self.hist / name), "title": title}

    def history(self, limit: int = 20) -> list[dict]:
        if not self.hist.exists():
            return []
        out = []
        for f in sorted(self.hist.glob("*.json"))[-limit:]:
            try:
                j = json.loads(f.read_text(encoding="utf-8"))
                out.append({"file": f.name, "title": j.get("title", ""), "summary": j.get("summary", "")[:600]})
            except Exception:
                pass
        return out
