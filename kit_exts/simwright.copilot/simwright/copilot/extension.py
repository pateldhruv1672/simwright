import asyncio, json, os, pathlib, time, urllib.error, urllib.request

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
        self._window = ui.Window("Simwright", width=460, height=640)
        with self._window.frame:
            with ui.VStack(spacing=6, style={"margin": 4}):
                self._status = ui.Label("", height=18, style={"color": 0xFF9A9A9A})
                self._scroll = ui.ScrollingFrame(height=ui.Fraction(1), horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF)
                with self._scroll:
                    self._log = ui.VStack(spacing=6, height=0)
                self._approvals = ui.VStack(spacing=6, height=0)
                with ui.CollapsableFrame("SimReady library", collapsed=True):
                    with ui.VStack(spacing=5):
                        with ui.HStack(height=26, spacing=5):
                            self._asset_query = ui.StringField()
                            ui.Button("Search", width=65, clicked_fn=lambda: self._start_asset_search(0))
                        self._asset_status = ui.Label("Search the public catalog across publishers, not just downloaded assets.", word_wrap=True, height=30)
                        with ui.ScrollingFrame(height=180):
                            self._asset_rows = ui.VStack(spacing=6, height=0)
                        self._asset_more = ui.Button("More results", height=24, enabled=False, clicked_fn=self._more_assets)
                self._prompt = ui.StringField(height=64, multiline=True)
                with ui.HStack(height=26, spacing=6):
                    ui.Button("Send", clicked_fn=self._on_send)
                    ui.Button("Check scene", clicked_fn=lambda: self._send(
                        "Check the open scene: physics stability, lighting, articulations and anything that will break. Fix only with my approval."))
                    ui.Button("Stop", width=60, clicked_fn=self._on_stop)
                    ui.Button("New chat", width=80, clicked_fn=self._on_reset)
                    self._mode = ui.ComboBox(1, "auto", "step", "ask", width=70)
        self._refresh_status()
        s = self._serve()
        if s:  # show the project's earlier conversation right away
            self._ensure_pump(f"http://127.0.0.1:{s['port']}", s["token"])

    def _start_asset_search(self, offset=0):
        asyncio.ensure_future(self._search_assets(offset))

    def _more_assets(self):
        if getattr(self, "_asset_next", None) is not None:
            self._start_asset_search(self._asset_next)

    async def _search_assets(self, offset):
        ep = self._endpoint()
        if not ep:
            return
        query = self._asset_query.model.get_value_as_string().strip()
        if not query:
            return
        generation = getattr(self, "_asset_generation", 0) + 1
        self._asset_generation = generation
        self._asset_status.text = "Searching SimReady Central…"
        self._asset_more.enabled = False
        try:
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, lambda: _http("POST", ep[0] + "/assets/search", ep[1], {"query": query, "offset": offset}, timeout=300))
            if generation != self._asset_generation or not self._window:
                return
            if not result.get("ok"):
                raise RuntimeError(result.get("error", "search failed"))
            self._asset_next = result.get("next_offset")
            self._asset_more.enabled = self._asset_next is not None
            hits = result.get("hits", [])
            self._asset_status.text = f"{len(hits)} results at offset {offset}. Imports include package dependencies."
            self._asset_rows.clear()
            ui = self._ui
            with self._asset_rows:
                for hit in hits:
                    with ui.VStack(spacing=2, height=0):
                        ui.Label(hit.get("name", "Asset").replace("_", " "), word_wrap=True, height=26)
                        dims = hit.get("dims_m")
                        detail = hit.get("dataset", "") + (" · " + " × ".join(f"{v:.2f}" for v in dims) + " m" if dims else " · size unknown")
                        ui.Label(detail, word_wrap=True, height=24)
                        url = hit.get("url", "")
                        ui.Button("Add to scene…", height=24, clicked_fn=lambda u=url: self._send("Use simready_place to add this asset to the current scene. Inspect the scene and choose a free, supported spot; preserve existing objects. Check physics and capture the result. Asset: " + u))
        except Exception as e:
            if generation == self._asset_generation and self._window:
                self._asset_status.text = "Library unavailable: " + self._explain(e)

    def _on_reset(self):
        ep = self._endpoint()
        if not ep:
            return
        try:
            r = _http("POST", ep[0] + "/reset", ep[1], {}, timeout=120)
            if not r.get("ok"):
                self._line(r.get("error", "could not start a new chat"), 0xFF5A8CFF)
                return
        except Exception as e:
            self._line(self._explain(e), 0xFF5A8CFF)
            return
        try:
            self._log.clear()
        except Exception:
            pass
        self._ensure_pump(*ep)

    def _refresh_status(self, busy=None, queued=0):
        s = self._serve()
        if not s:
            self._status.text = "start `simwright serve` in your robot workspace to use the panel"
            return
        state = "" if busy is None else ("working" + (f" ({queued} queued)" if queued else "") + " · " if busy else "idle · ")
        self._status.text = f"{state}simwright serve: {s.get('root', '')}"

    def _line(self, text, color=0xFFDDDDDD):
        with self._log:
            self._ui.Label(text, word_wrap=True, height=0, style={"color": color})
        asyncio.ensure_future(self._scroll_end())

    async def _scroll_end(self):
        import omni.kit.app
        await omni.kit.app.get_app().next_update_async()
        try:
            self._scroll.scroll_y = self._scroll.scroll_y_max
        except Exception:
            pass

    def _on_send(self):
        text = self._prompt.model.get_value_as_string().strip()
        if text:
            self._prompt.model.set_value("")
            self._send(text)

    def _endpoint(self):
        s = self._serve()
        self._refresh_status()
        if not s:
            self._line("Simwright is not running. In your workspace: simwright serve", 0xFF5A8CFF)
            return None
        return f"http://127.0.0.1:{s['port']}", s["token"]

    def _explain(self, e):
        if isinstance(e, urllib.error.HTTPError):
            try:
                body = e.read().decode()[:200]
            except Exception:
                body = ""
            return f"simwright serve answered {e.code} {body}".strip()
        if isinstance(e, urllib.error.URLError):
            return "simwright serve is not running (start it in your workspace: simwright serve)"
        return f"{type(e).__name__}: {e}"

    def _send(self, text):
        ep = self._endpoint()
        if not ep:
            return
        base, tok = ep
        modes = ["auto", "step", "ask"]
        try:
            mode = modes[self._mode.model.get_item_value_model().as_int]
            _http("POST", base + "/mode", tok, {"mode": mode})
            r = _http("POST", base + "/ask", tok, {"prompt": text})
        except Exception as e:
            self._line(self._explain(e), 0xFF5A8CFF)
            return
        if r.get("queued"):
            self._line(f"queued #{r['queued']} (runs after the current request): {text}", 0xFF9AB8D0)
        self._ensure_pump(base, tok)

    def _on_stop(self):
        ep = self._endpoint()
        if not ep:
            return
        try:
            r = _http("POST", ep[0] + "/stop", ep[1], {})
            if not r.get("was_busy"):
                self._line("nothing is running", 0xFF9A9A9A)
        except Exception as e:
            self._line(self._explain(e), 0xFF5A8CFF)
        self._ensure_pump(*ep)

    def _ensure_pump(self, base, tok):
        if self._poll is None or self._poll.done():
            self._poll = asyncio.ensure_future(self._pump(base, tok))

    async def _pump(self, base, tok):
        loop = asyncio.get_event_loop()
        idle = 0
        while idle < 6:
            try:
                r = await loop.run_in_executor(None, _http, "GET", f"{base}/events?since={self._since}", tok)
            except Exception as e:
                self._line("lost connection: " + self._explain(e), 0xFF5A8CFF)
                return
            evs = r.get("events", [])
            if evs and evs[0].get("restored") and self._since == 0:
                self._line("earlier in this project", 0xFF7A7A7A)
            for ev in evs:
                self._since = ev["n"] + 1
                t = ev.get("type")
                old = ev.get("restored")
                if t == "user":
                    self._line("you: " + ev["text"], 0xFF7FA6BF if old else 0xFFB0E0FF)
                elif t == "text":
                    self._line(ev["text"], 0xFF9A9A9A if old else 0xFFDDDDDD)
                elif t in ("info", "reset"):
                    self._line(ev.get("text", ""), 0xFF9AB8D0)
                    if ev.get("summary"):
                        self._line(ev["summary"], 0xFF9A9A9A)
                elif t == "tool":
                    self._line(f"- {ev['name']}({ev.get('args_brief', '')})", 0xFF9A9A9A)
                elif t == "tool_result" and ev.get("ok") is False and ev.get("error"):
                    self._line(f"  ! {ev['name']}: {str(ev['error'])[:300]}", 0xFF5A8CFF)
                elif t == "command":
                    self._line(f"> [{ev['target']}] {ev['cmd']}", 0xFF9A9A9A)
                elif t == "approval":
                    self._line(f"needs your approval below: {ev.get('cmd', '')}", 0xFF8CC8FF)
                elif t in ("stopping",):
                    self._line(ev.get("text", "stopping"), 0xFF9AB8D0)
                elif t == "error":
                    self._line(ev["text"], 0xFF5A8CFF)
            for p in r.get("pending", []):
                if p["id"] not in self._shown_approvals:
                    self._shown_approvals.add(p["id"])
                    self._approval(base, tok, p)
            self._refresh_status(bool(r.get("busy")), int(r.get("queued") or 0))
            idle = 0 if r.get("busy") or r.get("pending") or r.get("queued") else idle + 1
            await asyncio.sleep(0.5)

    def _approval(self, base, tok, p):
        ui = self._ui
        with self._approvals:
            box = ui.VStack(spacing=4, height=0)
            with box:
                head = ("DANGEROUS  " if p.get("dangerous") else "") + f"[{p['target']}] {p['cmd']}"
                ui.Label(head, word_wrap=True, height=0, style={"color": 0xFF8CC8FF if not p.get("dangerous") else 0xFF5A8CFF})
                if p.get("description"):
                    ui.Label(p["description"], word_wrap=True, height=0, style={"color": 0xFF9A9A9A})
                if p.get("detail"):
                    with ui.CollapsableFrame("Show code", collapsed=True, height=0):
                        ui.Label(p["detail"][:4000], word_wrap=True, height=0, style={"color": 0xFFCFCFCF, "font_size": 13})

                def answer(d, box=box, pid=p["id"]):
                    try:
                        _http("POST", base + "/approve", tok, {"id": pid, "decision": d})
                    except Exception as e:
                        self._line(self._explain(e), 0xFF5A8CFF)
                    finally:
                        box.visible = False
                with ui.HStack(height=24, spacing=6):
                    ui.Button("Run", width=70, clicked_fn=lambda: answer("run"))
                    ui.Button("Skip", width=70, clicked_fn=lambda: answer("skip"))
