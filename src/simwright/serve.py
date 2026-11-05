"""Local HTTP API on 127.0.0.1 for the Isaac Sim panel: ask, stream events, answer approvals."""
from __future__ import annotations
import json, pathlib, secrets, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from . import config
from .approval import QueueApprover
from .targets import Runner
from .tools import Toolbox


class State:
    def __init__(self, root):
        self.root = pathlib.Path(root)
        self.approver = QueueApprover(config.load().get("approval", "step"))
        self.tb = Toolbox(self.root, Runner(self.approver, echo=False))
        self.events = []
        self.busy = False
        self.agent = None
        self.lock = threading.Lock()
        self.queue: list[str] = []
        self.cancel = threading.Event()
        self.log_file = self.root / ".simwright" / "panel_events.jsonl"
        self._load_events()

    def _load_events(self, keep: int = 400):
        try:
            lines = self.log_file.read_text(encoding="utf-8").splitlines()[-keep:]
        except Exception:
            return
        for l in lines:
            try:
                ev = json.loads(l)
            except Exception:
                continue
            ev["n"] = len(self.events)
            ev["restored"] = True
            self.events.append(ev)

    def emit(self, ev):
        with self.lock:
            rec = {"n": len(self.events), "t": time.time(), **ev}
            self.events.append(rec)
        if ev.get("type") in ("user", "text", "tool", "command", "error", "reset", "queued", "stopping"):
            try:
                self.log_file.parent.mkdir(exist_ok=True)
                with self.log_file.open("a", encoding="utf-8") as f:
                    f.write(json.dumps({k: v for k, v in rec.items() if k not in ("n", "restored")}, default=str) + "\n")
                if self.log_file.stat().st_size > 4_000_000:  # keep the file small
                    lines = self.log_file.read_text(encoding="utf-8").splitlines()[-2000:]
                    self.log_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
            except Exception:
                pass

    def reset(self) -> dict:
        if self.busy:
            return {"ok": False, "error": "busy: stop the current request first"}
        from .agent import Agent
        try:
            if self.agent is None:
                self.agent = Agent(self.root, self.tb, emit=self.emit)
            r = self.agent.reset()
        except Exception as e:
            r = {"archived": bool(self.tb.memory.archive()), "summary": "", "error": str(e)}
        self.emit({"type": "reset", "text": "New chat. The last one was summarized into the project notes." if r.get("summary")
                   else "New chat.", "summary": r.get("summary", "")})
        return {"ok": True, **r}

    def drain_approver(self):
        while self.approver.events:
            self.emit(self.approver.events.pop(0))

    def submit(self, prompt: str) -> dict:
        """Run now, or queue behind the current request (never reject)."""
        with self.lock:
            if self.busy:
                self.queue.append(prompt)
                pos = len(self.queue)
                self.events.append({"n": len(self.events), "t": time.time(), "type": "queued", "text": prompt, "position": pos})
                return {"ok": True, "queued": pos}
            self.busy = True
        threading.Thread(target=self._run, args=(prompt,), daemon=True).start()
        return {"ok": True, "queued": 0}

    def stop(self) -> dict:
        with self.lock:
            dropped = len(self.queue)
            self.queue.clear()
        if self.busy:
            self.cancel.set()
        skipped = self.approver.skip_all()
        self.emit({"type": "stopping", "text": f"Stopping after the current step (skipped {skipped} approval(s), dropped {dropped} queued)."})
        return {"ok": True, "was_busy": self.busy, "skipped_approvals": skipped, "dropped": dropped}

    def _run(self, prompt):
        while prompt is not None:
            self.ask(prompt)
            with self.lock:
                prompt = self.queue.pop(0) if self.queue else None
                if prompt is None:
                    self.busy = False
        self.emit({"type": "idle"})

    def ask(self, prompt):
        from .agent import Agent
        self.cancel.clear()
        self.emit({"type": "user", "text": prompt})
        stop = threading.Event()

        def pump():
            while not stop.is_set():
                self.drain_approver()
                time.sleep(0.2)
        th = threading.Thread(target=pump, daemon=True)
        th.start()
        try:
            if self.agent is None:
                self.agent = Agent(self.root, self.tb, emit=self.emit)
            self.agent.cancel = self.cancel
            if getattr(self.agent, "restored", 0) and not getattr(self, "_told_restore", False):
                self._told_restore = True
                self.emit({"type": "info", "text": f"Continuing this project's conversation ({self.agent.restored} earlier messages, plus project notes)."})
            self.agent.turn(prompt)
        except Exception as e:
            self.emit({"type": "error", "text": f"{type(e).__name__}: {e}"})
        finally:
            stop.set()
            self.drain_approver()
            self.emit({"type": "done"})


def make_handler(state: State, token: str):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _auth(self):
            if self.headers.get("Authorization") != f"Bearer {token}":
                self._json(401, {"error": "bad token"})
                return False
            return True

        def _json(self, code, obj):
            body = json.dumps(obj, default=str).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not self._auth():
                return
            if self.path.startswith("/events"):
                since = int(self.path.split("since=")[-1]) if "since=" in self.path else 0
                with state.lock:
                    evs = state.events[since:]
                self._json(200, {"events": evs, "busy": state.busy, "queued": len(state.queue), "pending": [
                    {k: v for k, v in p.items() if k != "event"} for p in state.approver.pending.values()]})
            elif self.path.startswith("/history"):
                self._json(200, {"ok": True, "notes": state.tb.memory.notes(), "conversations": state.tb.memory.history(20)})
            elif self.path.startswith("/status"):
                self._json(200, {"ok": True, "root": str(state.root), "busy": state.busy, "queued": len(state.queue), "mode": state.approver.mode})
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            if not self._auth():
                return
            n = int(self.headers.get("Content-Length") or 0)
            data = json.loads(self.rfile.read(n) or b"{}")
            if self.path == "/ask":
                prompt = (data.get("prompt") or "").strip()
                if not prompt:
                    return self._json(400, {"error": "empty prompt"})
                self._json(200, state.submit(prompt))
            elif self.path == "/stop":
                self._json(200, state.stop())
            elif self.path == "/reset":
                self._json(200, state.reset())
            elif self.path == "/approve":
                self._json(200, {"ok": state.approver.answer(data.get("id", ""), data.get("decision", "skip"))})
            elif self.path == "/mode":
                state.approver.mode = data.get("mode", "step")
                self._json(200, {"ok": True, "mode": state.approver.mode})
            else:
                self._json(404, {"error": "not found"})
    return H


def serve(root, port: int | None = None):
    port = port or config.load().get("serve_port", 8787)
    token = secrets.token_urlsafe(24)
    config.write_state("serve.json", {"port": port, "token": token, "root": str(root)})
    state = State(root)
    httpd = ThreadingHTTPServer(("127.0.0.1", port), make_handler(state, token))
    print(f"Simwright serving {root} on http://127.0.0.1:{port} (token in {config.state_file('serve.json')})")
    httpd.serve_forever()
