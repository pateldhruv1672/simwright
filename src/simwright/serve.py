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

    def emit(self, ev):
        with self.lock:
            self.events.append({"n": len(self.events), "t": time.time(), **ev})

    def drain_approver(self):
        while self.approver.events:
            self.emit(self.approver.events.pop(0))

    def ask(self, prompt):
        from .agent import Agent
        self.busy = True
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
            self.agent.turn(prompt)
        except Exception as e:
            self.emit({"type": "error", "text": f"{type(e).__name__}: {e}"})
        finally:
            stop.set()
            self.drain_approver()
            self.busy = False
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
                self._json(200, {"events": evs, "busy": state.busy, "pending": [
                    {k: v for k, v in p.items() if k != "event"} for p in state.approver.pending.values()]})
            elif self.path.startswith("/status"):
                self._json(200, {"ok": True, "root": str(state.root), "busy": state.busy, "mode": state.approver.mode})
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            if not self._auth():
                return
            n = int(self.headers.get("Content-Length") or 0)
            data = json.loads(self.rfile.read(n) or b"{}")
            if self.path == "/ask":
                if state.busy:
                    return self._json(409, {"error": "busy"})
                threading.Thread(target=state.ask, args=(data.get("prompt", ""),), daemon=True).start()
                self._json(200, {"ok": True})
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
