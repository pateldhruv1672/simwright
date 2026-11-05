import asyncio, json, os, pathlib, time, urllib.request

import carb
import omni.ext

HOME = pathlib.Path(os.environ.get("SIMWRIGHT_HOME", str(pathlib.Path.home() / ".simwright")))
PS = "/exts/isaacsim.code_editor.python_server"


def _http(method, url, token, body=None, timeout=10):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


class SimwrightCopilotExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._window = None
        self._since = 0
        self._shown_approvals = set()
        self._poll = None
        self._write_session()
        asyncio.ensure_future(self._later_session())
        try:
            import omni.ui as ui
            self._ui = ui
            self._build_ui()
        except Exception as e:  # headless app without omni.ui
            carb.log_info(f"[simwright] panel disabled: {e}")

    def on_shutdown(self):
        if self._poll:
            self._poll.cancel()
        if self._window:
            self._window.destroy()
            self._window = None

    # ---------- session file for the Simwright client ----------
    def _write_session(self):
        s = carb.settings.get_settings()
        info = {"host": s.get(PS + "/host") or "127.0.0.1", "port": int(s.get(PS + "/port") or 8226),
                "token": s.get(PS + "/auth_token") or "", "kit_pid": os.getpid(), "extension_loaded": time.time(), "ready": True}
        try:
            HOME.mkdir(parents=True, exist_ok=True)
            f = HOME / "kit_session.json"
            old = json.loads(f.read_text()) if f.exists() else {}
            old.update({k: v for k, v in info.items() if v not in ("", None)})
            f.write_text(json.dumps(old, indent=2))
        except Exception as e:
            carb.log_warn(f"[simwright] could not write session file: {e}")

    async def _later_session(self):
        await asyncio.sleep(8)
        self._write_session()

    # ---------- panel ----------
    def _serve(self):
        f = HOME / "serve.json"
        if not f.exists():
            return None
        try:
            return json.loads(f.read_text())
        except Exception:
            return None

    def _build_ui(self):
        ui = self._ui
        self._window = ui.Window("Simwright", width=440, height=600)
        with self._window.frame:
            with ui.VStack(spacing=6, style={"margin": 4}):
                self._status = ui.Label("", height=18, style={"color": 0xFF9A9A9A})
                with ui.ScrollingFrame(height=ui.Fraction(1), horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                    self._log = ui.VStack(spacing=6, height=0)
                self._approvals = ui.VStack(spacing=4, height=0)
                self._prompt = ui.StringField(height=64, multiline=True)
                with ui.HStack(height=26, spacing=6):
                    ui.Button("Send", clicked_fn=self._on_send)
                    ui.Button("Check scene", clicked_fn=lambda: self._send(
                        "Check the open scene: physics stability, lighting, articulations and anything that will break. Fix only with my approval."))
                    self._mode = ui.ComboBox(1, "auto", "step", "ask", width=70)
        self._refresh_status()

    def _refresh_status(self):
        s = self._serve()
        self._status.text = (f"connected to simwright serve ({s.get('root','')})" if s else
                             "start `simwright serve` in your robot workspace to use the panel")

    def _line(self, text, color=0xFFDDDDDD):
        with self._log:
            self._ui.Label(text, word_wrap=True, height=0, style={"color": color})

    def _on_send(self):
        text = self._prompt.model.get_value_as_string().strip()
        if text:
            self._prompt.model.set_value("")
            self._send(text)

    def _send(self, text):
        s = self._serve()
        self._refresh_status()
        if not s:
            self._line("Simwright is not running. In your workspace: simwright serve", 0xFF5A8CFF)
            return
        base, tok = f"http://127.0.0.1:{s['port']}", s["token"]
        modes = ["auto", "step", "ask"]
        try:
            mode = modes[self._mode.model.get_item_value_model().as_int]
            _http("POST", base + "/mode", tok, {"mode": mode})
            _http("POST", base + "/ask", tok, {"prompt": text})
        except Exception as e:
            self._line(f"could not reach simwright serve: {e}", 0xFF5A8CFF)
            return
        if self._poll is None or self._poll.done():
            self._poll = asyncio.ensure_future(self._pump(base, tok))

    async def _pump(self, base, tok):
        loop = asyncio.get_event_loop()
        idle = 0
        while idle < 6:
            try:
                r = await loop.run_in_executor(None, _http, "GET", f"{base}/events?since={self._since}", tok)
            except Exception as e:
                self._line(f"lost connection: {e}", 0xFF5A8CFF)
                return
            for ev in r.get("events", []):
                self._since = ev["n"] + 1
                t = ev.get("type")
                if t == "user":
                    self._line("you: " + ev["text"], 0xFFB0E0FF)
                elif t == "text":
                    self._line(ev["text"])
                elif t == "tool":
                    self._line(f"\u2022 {ev['name']}({ev.get('args_brief','')})", 0xFF9A9A9A)
                elif t == "command":
                    self._line(f"\u25b6 [{ev['target']}] {ev['cmd']}", 0xFF9A9A9A)
                elif t == "error":
                    self._line(ev["text"], 0xFF5A8CFF)
            for p in r.get("pending", []):
                if p["id"] not in self._shown_approvals:
                    self._shown_approvals.add(p["id"])
                    self._approval(base, tok, p)
            idle = 0 if r.get("busy") or r.get("pending") else idle + 1
            await asyncio.sleep(0.5)

    def _approval(self, base, tok, p):
        ui = self._ui
        with self._approvals:
            row = ui.HStack(height=24, spacing=6)
            with row:
                ui.Label(("DANGEROUS " if p.get("dangerous") else "") + f"[{p['target']}] {p['cmd'][:120]}", word_wrap=True)

                def answer(d, row=row, pid=p["id"]):
                    try:
                        _http("POST", base + "/approve", tok, {"id": pid, "decision": d})
                    finally:
                        row.visible = False
                ui.Button("Run", width=50, clicked_fn=lambda: answer("run"))
                ui.Button("Skip", width=50, clicked_fn=lambda: answer("skip"))
