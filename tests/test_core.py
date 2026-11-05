import json, pathlib, socket, threading, textwrap
import pytest
from simwright import colcon, context, ros2, skills, urdf, world, config
from simwright.approval import AutoApprover, Approver, is_dangerous, is_read_only
from simwright.targets import Runner, to_wsl_path, Target

GOOD_URDF = """<robot name="bot">
 <link name="base_link"><inertial><mass value="5"/><inertia ixx="0.1" iyy="0.1" izz="0.15" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.4 0.3 0.1"/></geometry></collision></link>
 <link name="wheel"><inertial><mass value="0.5"/><inertia ixx="0.001" iyy="0.001" izz="0.0015" ixy="0" ixz="0" iyz="0"/></inertial></link>
 <joint name="wheel_joint" type="continuous"><parent link="base_link"/><child link="wheel"/><axis xyz="0 1 0"/></joint>
</robot>"""

BAD_URDF = """<robot name="bad">
 <link name="base_link"><inertial><mass value="5"/><inertia ixx="0.01" iyy="0.01" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link>
 <link name="arm"><inertial><mass value="0.0001"/><inertia ixx="1" iyy="1" izz="1" ixy="2" ixz="0" iyz="0"/></inertial></link>
 <link name="loose"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
 <joint name="j1" type="revolute"><parent link="base_link"/><child link="arm"/><axis xyz="0 0 2"/><limit lower="1" upper="0" effort="0" velocity="1"/></joint>
</robot>"""


def make_ws(tmp_path):
    ws = tmp_path / "ws"
    (ws / "src/bot_description/urdf").mkdir(parents=True)
    (ws / "src/bot_description/package.xml").write_text("""<package format="3"><name>bot_description</name><depend>xacro</depend><depend>robot_state_publisher</depend><export><build_type>ament_cmake</build_type></export></package>""")
    (ws / "src/bot_description/urdf/bot.urdf").write_text(GOOD_URDF)
    (ws / "src/bot_bringup/launch").mkdir(parents=True)
    (ws / "src/bot_bringup/package.xml").write_text("<package format='3'><name>bot_bringup</name><export><build_type>ament_python</build_type></export></package>")
    (ws / "src/bot_bringup/launch/sim.launch.py").write_text("remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]\nparams={'frame_id': 'base_link', 'odom_frame': 'odom'}\n")
    (ws / "Dockerfile").write_text("FROM ros:jazzy\n")
    (ws / "scene.xml").write_text("<mujoco model='m'><worldbody/></mujoco>")
    return ws


def test_init_scan(tmp_path):
    ws = make_ws(tmp_path)
    assert context.find_root(ws / "src" / "bot_bringup") == ws
    out, info, wrote = context.init(ws)
    text = out.read_text()
    assert wrote and "bot_description" in text and "jazzy" in text and "/cmd_vel" in text and "base_link" in text
    assert info["mjcf"] == ["scene.xml"] and info["urdf"][0]["joints"] == 1
    assert context.init(ws)[2] is False  # keeps user edits
    name, ctx = context.load_context(ws)
    assert name == "SIMWRIGHT.md"


def test_skills(tmp_path):
    ws = make_ws(tmp_path)
    f = tmp_path / "ekf-tuning.md"
    f.write_text("# EKF tuning\n\nHow our team tunes robot_localization EKF covariance for wheel odometry and IMU fusion.\n")
    s = skills.add(str(f), ws)
    assert s.path == config.HOME / "skills" / "ekf-tuning" / "SKILL.md" and s.path.read_text().startswith("---")
    r = skills.add(str(f), ws, to_repo=True, name="ekf")
    assert r.path == ws / ".skills" / "ekf" / "SKILL.md"
    names = {x.name: x.scope for x in skills.discover(ws)}
    assert names["ekf"] == "repo" and names["ekf-tuning"] == "user" and names["isaaclab-policy-lifecycle"] == "builtin"
    assert skills.parse_nl_add("add this skill: ./team-skills/ekf-tuning.md") == ("./team-skills/ekf-tuning.md", False)
    assert skills.parse_nl_add("add skill ./x.md to the repo")[1] is True
    rel = skills.relevant(skills.discover(ws), "tune the EKF covariance for imu odometry fusion")
    assert rel and rel[0].name.startswith("ekf")
    rel = skills.relevant(skills.discover(ws), "train a locomotion policy in isaac lab and export it")
    assert rel[0].name == "isaaclab-policy-lifecycle"


def test_urdf():
    good = urdf.validate("bot.urdf", GOOD_URDF)
    assert good["ok"] and good["total_mass_kg"] == 5.5, good
    bad = urdf.validate("bad.urdf", BAD_URDF)
    issues = " | ".join(i["issue"] for i in bad["issues"])
    assert not bad["ok"]
    for needle in ("triangle inequality", "not positive definite", "tiny mass", "lower 1.0 >= upper 0.0", "effort limit is 0",
                   "axis not unit", "no <inertial>", "expected one root"):
        assert needle in issues, (needle, issues)
    assert sorted(urdf.sym_eigen(2, 3, 4, 0.5, 0.2, 0.1))[0] > 0


def test_colcon():
    log = textwrap.dedent("""
        Starting >>> bot_control
        --- stderr: bot_control
        CMake Error at CMakeLists.txt:12 (find_package):
          By not providing "Findcontrol_toolbox.cmake" in CMAKE_MODULE_PATH this project has
          Could not find a package configuration file provided by "control_toolbox"
        ---
        Failed   <<< bot_control [1.2s, exited with code 1]
        Starting >>> bot_nav
        --- stderr: bot_nav
        /ws/src/bot_nav/src/planner.cpp:3:10: fatal error: nav_msgs/msg/path.hpp: No such file or directory
        ---
        Failed   <<< bot_nav [0.8s, exited with code 1]
        Summary: 3 packages finished [2.1s]
          2 packages failed: bot_control bot_nav
    """)
    d = colcon.diagnose(log)
    assert d["failed_packages"] == ["bot_control", "bot_nav"]
    assert d["root_cause"]["kind"] == "missing_cmake_dep" and d["root_cause"]["package"] == "bot_control"
    assert "<depend>control_toolbox</depend>" in d["root_cause"]["fix"]
    inc = [x for x in d["diagnoses"] if x["kind"] == "missing_include"][0]
    assert inc["package"] == "bot_nav" and "nav_msgs" in inc["fix"]


GRAPH = """@@DISTRO jazzy
@@NODES
/camera_driver
/perception
/teleop
/diff_cont
@@TOPICS
/image_raw [sensor_msgs/msg/Image]
/cmd_vel [geometry_msgs/msg/Twist]
@@TOPIC /image_raw
Type: sensor_msgs/msg/Image

Publisher count: 1

Node name: camera_driver
Node namespace: /
Topic type: sensor_msgs/msg/Image
Endpoint type: PUBLISHER
GID: 01.0f
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): KEEP_LAST (5)
  Durability: VOLATILE

Subscription count: 1

Node name: perception
Node namespace: /
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
GID: 01.0f
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE
@@TOPIC /cmd_vel
Node name: teleop
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
@@CONTROLLERS
diff_cont  diff_drive_controller/DiffDriveController  active
@@END
"""


def test_ros2_graph():
    g = ros2.parse_graph(GRAPH)
    assert g["distro"] == "jazzy" and len(g["nodes"]) == 4 and g["controllers"]
    kinds = {(i["topic"], i["issue"]) for i in g["issues"]}
    assert ("/image_raw", "QoS incompatible: reliability") in kinds
    assert ("/cmd_vel", "no subscribers") in kinds
    t = ros2.trace(g, "/cmd_vel")
    assert t["publishers"] == ["/teleop"] and t["subscribers"] == []
    miss = ros2.trace(g, "/robot1/cmd_vel")
    assert not miss["found"] and "/cmd_vel" in miss["similar"]


def test_world_office():
    spec = {"room": {"width": 10, "depth": 8, "doors": [{"wall": "south", "offset": 1.0, "width": 1.0}]},
            "items": [{"kind": "desk", "count": 4, "with": ["chair", "monitor"]}, {"kind": "table", "count": 1, "chairs": 6},
                      {"kind": "shelf", "count": 3}, {"kind": "plant", "count": 2}],
            "robot": {"name": "carter", "footprint": [0.7, 0.5], "on": "floor"}}
    L = world.solve(spec)
    assert L["checks"]["ok"], L["checks"]
    assert not L["unplaced"], L["unplaced"]
    P = {p["id"]: p for p in L["placements"]}
    desks = [p for p in P.values() if p["kind"] == "desk"]
    assert len(desks) == 4
    for d in desks:
        chair = [p for p in P.values() if p.get("on") == d["id"] and p["kind"] == "chair"][0]
        mon = [p for p in P.values() if p.get("on") == d["id"] and p["kind"] == "monitor"][0]
        assert (chair["yaw"] - d["yaw"]) % 360 == 180          # chair faces its desk
        assert mon["yaw"] == d["yaw"] and mon["z"] == 0.75     # screen on the desk, facing the sitter
        import math
        fx, fy = math.cos(math.radians(d["yaw"])), math.sin(math.radians(d["yaw"]))
        assert (chair["x"] - d["x"]) * fx + (chair["y"] - d["y"]) * fy > 0.3   # chair in front of the desk
    robot = [p for p in P.values() if p["kind"] == "robot"][0]
    assert robot["z"] == 0 and robot["clearance_m"] >= 0.7
    door = world.Rect(0.8, 0, 2.2, 1.5)
    assert all(not world._rect(p["x"], p["y"], p["w"], p["d"], p["yaw"]).hits(door) for p in P.values() if p["z"] == 0 and p["kind"] != "robot")
    print(world.ascii_map(L))


def test_world_desk_robot():
    L = world.solve({"room": {"width": 5, "depth": 4}, "items": [{"kind": "desk", "count": 1, "with": ["chair", "monitor"]}],
                     "robot": {"name": "franka", "footprint": [0.3, 0.3], "on": "desk"}})
    r = [p for p in L["placements"] if p["kind"] == "robot"][0]
    assert r["z"] == 0.75 and r["on"].startswith("desk")


def test_runner_and_approval(tmp_path):
    r = Runner(AutoApprover(), echo=False)
    res = r.run("echo hello && exit 3", "host", str(tmp_path))
    assert res.exit == 3 and "hello" in res.out
    assert r.run("rm -rf /tmp/x", "host").skipped
    assert is_dangerous("Remove-Item -Recurse -Force C:\\ws") and is_dangerous("git reset --hard")
    assert is_read_only("ros2 topic list") and not is_read_only("ros2 topic list > out.txt")
    assert to_wsl_path(r"C:\Users\me\ws") == "/mnt/c/Users/me/ws"
    t = Target("wsl", "wsl", "Ubuntu-24.04", "source /opt/ros/jazzy/setup.bash")
    argv = t.argv("colcon build", r"C:\ws")
    assert argv[:4] == ["wsl.exe", "-d", "Ubuntu-24.04", "--"] and "cd /mnt/c/ws" in argv[-1]
    d = Target("ros", "docker", "ros2_dev").argv("ros2 node list")
    assert d[:4] == ["docker", "exec", "-i", "ros2_dev"]


def fake_python_server(envelope=True):
    srv = socket.socket(); srv.bind(("127.0.0.1", 0)); srv.listen(5)
    port = srv.getsockname()[1]
    def loop():
        while True:
            c, _ = srv.accept()
            buf = b""
            while True:
                d = c.recv(4096)
                if not d:
                    break
                buf += d
            text = buf.decode()
            if envelope and text.startswith("{"):
                code = json.loads(text)["code"]
            elif text.startswith("{"):
                c.sendall(json.dumps({"status": "ok", "output": "", "result": "{'code': ...}"}).encode()); c.close(); continue
            else:
                code = text
            import io, contextlib
            out = io.StringIO()
            try:
                with contextlib.redirect_stdout(out):
                    exec(code.replace("simwright.copilot.helpers", "nope_module") if "await" not in code else "print('@@SW {\"ok\": true}')", {})
                c.sendall(json.dumps({"status": "ok", "output": out.getvalue()}).encode())
            except Exception as e:
                c.sendall(json.dumps({"status": "error", "ename": type(e).__name__, "evalue": str(e), "traceback": []}).encode())
            c.close()
    threading.Thread(target=loop, daemon=True).start()
    return port


@pytest.mark.parametrize("envelope", [True, False])
def test_kit_client(envelope):
    from simwright.isaac import KitClient
    k = KitClient("127.0.0.1", fake_python_server(envelope))
    r = k.exec("print(1+1)")
    assert r["status"] == "ok" and r["output"].strip() == "2"
    assert k._raw_only is (not envelope)
    assert k.call("await physics_check(1.0)") == {"ok": True}


def test_tool_schemas_and_mcp(tmp_path):
    from simwright.tools import Toolbox, TOOLS
    from simwright.approval import AutoApprover
    tb = Toolbox(make_ws(tmp_path), Runner(AutoApprover(), echo=False))
    sch = tb.schemas()
    json.dumps(sch)
    assert len(sch) == len(TOOLS) >= 30
    w = tb.call("write_file", {"path": "notes/a.txt", "content": "x\n", "why": "test"})
    assert w["ok"] and "notes/a.txt" in tb.session.summary()
    assert tb.call("edit_file", {"path": "notes/a.txt", "old": "x", "new": "y"})["ok"]
    assert tb.call("search_files", {"pattern": "cmd_vel", "glob": "*.py"})["hits"]
    v = tb.call("urdf_validate", {"path": "src/bot_description/urdf/bot.urdf"})
    assert v["ok"]
    from simwright.mcp_server import build
    import asyncio
    mcp, _, _ = build(make_ws(tmp_path / "b"))
    names = {t.name for t in asyncio.run(mcp.list_tools())}
    assert {"ros2_graph", "isaac_physics_check", "lab_train", "world_compose", "skill_add"} <= names


def test_lab_discovery(tmp_path, monkeypatch):
    from simwright import lab
    home = tmp_path / "home"
    launcher = "isaaclab.bat" if __import__("os").name == "nt" else "isaaclab.sh"
    for name, ver in (("IsaacLab", "2.3.2"), ("IsaacLab-beta", "3.0.0"), ("IsaacLab_5_1_broken", "2.3.0")):
        d = home / "robotics" / name; d.mkdir(parents=True)
        (d / launcher).write_text(""); (d / "VERSION").write_text(ver)
    def env(name, src, simv):
        e = home / ".conda" / "envs" / name
        sp = e / "lib" / "python3.12" / "site-packages"; sp.mkdir(parents=True)
        (e / "bin").mkdir(); (e / "bin" / "python").write_text("")
        di = sp / "isaaclab-1.0.0.dist-info"; di.mkdir()
        (di / "direct_url.json").write_text(json.dumps({"url": (home / "robotics" / src / "source" / "isaaclab").as_uri(), "dir_info": {"editable": True}}))
        (sp / f"isaacsim-{simv}.0.dist-info").mkdir()
    env("env_isaaclab", "IsaacLab", "5.1.0"); env("env_isaaclab_beta", "IsaacLab-beta", "6.0.1")
    monkeypatch.setattr(pathlib.Path, "home", classmethod(lambda cls: home))
    monkeypatch.delenv("CONDA_PREFIX", raising=False); monkeypatch.delenv("CONDA_EXE", raising=False)
    monkeypatch.delenv("ISAAC_LAB_DIR", raising=False)
    cfg = config._merge(config.DEFAULTS, {"isaac_lab_dir": ""})
    found = lab.discover(cfg)
    assert {i["version"] for i in found} == {"2.3.2", "3.0.0", "2.3.0"}
    beta = next(i for i in found if i["version"] == "3.0.0")
    assert beta["env"] == "env_isaaclab_beta" and beta["isaacsim"] == "6.0.1"
    assert lab.choose(found, "6.0.1-rc.7")["version"] == "3.0.0"
    assert lab.choose(found, "5.1.0")["version"] == "2.3.2"
    L = lab.Lab.__new__(lab.Lab); L.cfg = cfg; L.lib = "rsl_rl"; L.python = beta["python"]; L.dir = pathlib.Path(beta["dir"])
    cmd = L.train_cmd("Isaac-Cartpole-v0", max_iterations=10)
    assert beta["python"] in cmd and "train.py" in cmd and str(L.dir) in cmd and "isaaclab.sh" not in cmd and "isaaclab.bat" not in cmd


def test_simready(tmp_path, monkeypatch):
    from simwright import simready
    monkeypatch.setattr(config, "HOME", tmp_path / "home")
    U = "https://huggingface.co/datasets/nvidia/simready-assets/blob/abc123def4567/kettle_a01/simready_usd/sm_kettle_a01_01.usd"
    def fake_http(method, url, body=None, timeout=90, params=None):
        if url.endswith("/search_hybrid"):
            assert body["description"] == "kettle" and body["return_usd_dimensions"]
            return {"total": 2, "hits": [
                {"score": 1.2, "source": {"path": "hf://huggingface.co/datasets/nvidia/simready-assets/resolve/abc123def4567/kettle_a01/simready_usd/sm_kettle_a01_01.usd",
                 "usd_dimensions": {"scaled_bbox_dimension_x": 0.16, "scaled_bbox_dimension_y": 0.22, "scaled_bbox_dimension_z": 0.25, "scene_mpu": 1}}},
                {"score": 1.1, "source": {"path": "hf://huggingface.co/datasets/x/y/resolve/r1/big/a.usd",
                 "usd_dimensions": {"scaled_bbox_dimension_x": 2.0, "scaled_bbox_dimension_y": 1, "scaled_bbox_dimension_z": 1, "scene_mpu": 1}}}]}
        assert url.endswith("/download/asset") and params["manifest_only"] == "true"
        base = "https://huggingface.co/datasets/nvidia/simready-assets/blob/abc123def4567/kettle_a01/"
        return {"files": [{"url": base + "com.nvidia.simready.packaging.json", "size": 30},
                          {"url": base + ".metadata/LICENSE.txt", "size": 31},
                          {"url": base + "simready_usd/sm_kettle_a01_01.usd", "size": 5}]}
    payload = {"com.nvidia.simready.packaging.json": b'{"license": "CC-BY-4.0", "name": "x"}' + b" " * 0,
               "LICENSE.txt": b"Attribution 4.0 International\n" , "sm_kettle_a01_01.usd": b"#usda"}
    def fake_dl(url, dest, size):
        dest.parent.mkdir(parents=True, exist_ok=True)
        data = payload[dest.name]
        dest.write_bytes(data)
        return len(data)
    monkeypatch.setattr(simready, "_http", fake_http)
    monkeypatch.setattr(simready, "_download", fake_dl)
    r = simready.search("kettle", max_size_m=0.5)
    assert len(r["hits"]) == 1 and r["hits"][0]["dims_m"] == [0.16, 0.22, 0.25] and r["hits"][0]["name"] == "kettle_a01"
    assert r["hits"][0]["url"] == U and r["hits"][0]["thumbnail"].endswith("/simready_usd/.thumbs/256x256/sm_kettle_a01_01.usd.png")
    m = simready.manifest(U)
    assert m["package_root"] == "kettle_a01" and m["file_count"] == 3
    f = simready.fetch(U)
    assert f["ok"] and f["license"] == "CC-BY-4.0" and f["license_text"].startswith("Attribution") and f["revision"] == "abc123def4567"
    assert simready.local(U) == f["local_usd"] and simready.catalog()[0]["package"] == "kettle_a01"


def test_output_dir_avoids_system32(tmp_path, monkeypatch):
    from simwright.tools import Toolbox
    monkeypatch.setattr(config, "HOME", tmp_path / "home")
    tb = Toolbox.__new__(Toolbox)
    tb.root = pathlib.Path("/Windows/System32")
    assert str(tb._out(".simwright/captures/f.png")).startswith(str(tmp_path / "home"))
    tb.root = tmp_path
    assert tb._out(".simwright/captures/f.png") == tmp_path / ".simwright/captures/f.png"
    assert tb._out("/abs/x.png") == pathlib.Path("/abs/x.png")


def test_model_endpoint(tmp_path, monkeypatch):
    import time as _t
    from simwright import agent
    monkeypatch.setattr(config, "HOME", tmp_path)
    for v in ("SIMWRIGHT_API_KEY", "OPENAI_API_KEY", "SIMWRIGHT_BASE_URL"):
        monkeypatch.delenv(v, raising=False)
    cfg = config._merge(config.DEFAULTS, {})
    assert agent.model_endpoint(cfg) == ("", "")
    (tmp_path / "mana.json").write_text(json.dumps({"base_url": "https://relay/v1", "token": "t0k", "expires": _t.time() + 99}))
    assert agent.model_endpoint(cfg) == ("t0k", "https://relay/v1")
    (tmp_path / "mana.json").write_text(json.dumps({"base_url": "https://relay/v1", "token": "t0k", "expires": _t.time() - 1}))
    assert agent.model_endpoint(cfg) == ("", "")
    monkeypatch.setenv("SIMWRIGHT_API_KEY", "own")
    assert agent.model_endpoint(cfg)[0] == "own"


def test_every_module_imports():
    import importlib, pkgutil, simwright
    for m in pkgutil.iter_modules(simwright.__path__):
        if m.name not in ("kit_helpers",):
            importlib.import_module(f"simwright.{m.name}")


def test_connect_writers(tmp_path, monkeypatch):
    from simwright import connect
    monkeypatch.setattr(connect, "server_command", lambda root, ro=False: ("C:/Users/me/.local/bin/simwright-mcp.exe", ["--root", str(root)] + (["--read-only"] if ro else [])))
    home, root = tmp_path / "home", tmp_path / "ws"
    (home / ".codex").mkdir(parents=True); root.mkdir()
    (home / ".codex" / "config.toml").write_text('model = "gpt-5.4"\n\n[mcp_servers.other]\ncommand = "x"\n\n[mcp_servers.simwright]\ncommand = "old"\n\n[mcp_servers.simwright.env]\nA = "1"\n\n[profiles.fast]\nmodel = "m"\n')
    for _ in range(2):  # idempotent
        r = connect.connect("codex", root, home=home)
    txt = (home / ".codex" / "config.toml").read_text()
    import tomllib
    cfg = tomllib.loads(txt)
    assert cfg["mcp_servers"]["other"]["command"] == "x" and cfg["profiles"]["fast"]["model"] == "m" and cfg["model"] == "gpt-5.4"
    assert cfg["mcp_servers"]["simwright"]["command"].endswith("simwright-mcp.exe") and cfg["mcp_servers"]["simwright"]["tool_timeout_sec"] == 900
    assert "env" not in cfg["mcp_servers"]["simwright"] and txt.count("[mcp_servers.simwright]") == 1
    assert (home / ".codex" / "config.toml.bak").exists()
    (home / ".cursor").mkdir()
    (home / ".cursor" / "mcp.json").write_text(json.dumps({"mcpServers": {"github": {"command": "gh"}}}))
    connect.connect("cursor", root, read_only=True, home=home)
    cj = json.loads((home / ".cursor" / "mcp.json").read_text())
    assert cj["mcpServers"]["github"] == {"command": "gh"} and cj["mcpServers"]["simwright"]["args"][-1] == "--read-only"
    connect.connect("vscode", root, home=home)
    assert json.loads((root / ".vscode" / "mcp.json").read_text())["servers"]["simwright"]["type"] == "stdio"
    assert "Developer mode" in connect.connect("chatgpt", root, home=home)["then"]


def test_mcp_http_guard_and_annotations(tmp_path, monkeypatch):
    from starlette.testclient import TestClient
    from simwright import mcp_server
    monkeypatch.setattr(config, "HOME", tmp_path)
    mcp, _, names = mcp_server.build(tmp_path)
    ro = mcp_server.build(tmp_path, read_only=True)[2]
    assert set(ro) <= mcp_server.READ_ONLY and "run_command" not in ro and "run_command" in names
    assert mcp_server._annotations("run_command").destructive_hint and mcp_server._annotations("read_file").read_only_hint
    tok = mcp_server.http_token()
    assert mcp_server.http_token() == tok and mcp_server.http_token(rotate=True) != tok
    tok = mcp_server.http_token()
    init = {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18", "capabilities": {},
            "clientInfo": {"name": "t", "version": "1"}}}
    hdr = {"Accept": "application/json, text/event-stream", "Content-Type": "application/json"}
    with TestClient(mcp_server.http_app(mcp, tok), base_url="https://abc-def.trycloudflare.com") as c:
        assert c.get("/healthz").status_code == 200
        assert c.post("/mcp", json=init, headers=hdr).status_code == 401
        assert c.post("/t/nope/mcp", json=init, headers=hdr).status_code == 401
        assert c.post(f"/t/{tok}/mcp", json=init, headers=hdr).status_code == 200
        assert c.post("/mcp", json=init, headers={**hdr, "Authorization": f"Bearer {tok}"}).status_code == 200
        assert c.post(f"/mcp?token={tok}", json=init, headers=hdr).status_code == 200


def test_find_root_ignores_global_dir(tmp_path, monkeypatch):
    home = tmp_path / "home"; (home / ".simwright").mkdir(parents=True)
    ws = home / "robotics" / "ws"; ws.mkdir(parents=True)
    monkeypatch.setattr(pathlib.Path, "home", classmethod(lambda cls: home))
    monkeypatch.setattr(config, "HOME", home / ".simwright")
    monkeypatch.chdir(ws)
    assert context.find_root() == ws                      # not the home folder
    (home / "robotics" / "SIMWRIGHT.md").write_text("# ctx")
    assert context.find_root() == home / "robotics"       # nearest real marker
    assert context.find_root(str(ws), exact=True) == ws   # explicit --root wins


def test_serve_queue_and_stop(tmp_path, monkeypatch):
    import threading, time as _t
    from simwright import serve, agent as agentmod
    monkeypatch.setattr(config, "HOME", tmp_path / "home")
    ran = []

    class FakeAgent:
        def __init__(self, root, tb, emit=None):
            self.emit = emit
        def turn(self, prompt):
            ran.append(prompt)
            for _ in range(100):
                if getattr(self, "cancel", None) is not None and self.cancel.is_set():
                    self.emit({"type": "text", "text": "Stopped at your request."})
                    return
                _t.sleep(0.01)
    monkeypatch.setattr(agentmod, "Agent", FakeAgent)
    st = serve.State(tmp_path)
    assert st.submit("first")["queued"] == 0
    assert st.submit("second")["queued"] == 1 and st.submit("third")["queued"] == 2
    _t.sleep(0.05)
    r = st.stop()
    assert r["dropped"] == 2 and r["was_busy"]
    for _ in range(200):
        if not st.busy:
            break
        _t.sleep(0.01)
    assert not st.busy and ran == ["first"]
    types = [e["type"] for e in st.events]
    assert types.count("queued") == 2 and "stopping" in types and "idle" in types
    assert any(e.get("text") == "Stopped at your request." for e in st.events)
    assert st.submit("after")["queued"] == 0       # cancel flag is reset for the next request
    for _ in range(300):
        if not st.busy:
            break
        _t.sleep(0.01)
    assert ran == ["first", "after"]


# ---------------- placement and mount geometry (real USD via usd-core) ----------------
def _box_mesh(stage, path, mn, mx):
    from pxr import UsdGeom, Gf, Vt
    m = UsdGeom.Mesh.Define(stage, path)
    x0, y0, z0 = mn; x1, y1, z1 = mx
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateExtentAttr([Gf.Vec3f(*mn), Gf.Vec3f(*mx)])
    return m


def _asset(tmp_path, name, boxes, mpu=1.0, up="Z"):
    from pxr import Usd, UsdGeom
    f = tmp_path / f"{name}.usda"
    st = Usd.Stage.CreateNew(str(f))
    UsdGeom.SetStageMetersPerUnit(st, mpu); UsdGeom.SetStageUpAxis(st, up)
    root = UsdGeom.Xform.Define(st, "/Root"); st.SetDefaultPrim(root.GetPrim())
    for i, (mn, mx) in enumerate(boxes):
        _box_mesh(st, f"/Root/b{i}", mn, mx)
    st.Save()
    return str(f)


def _helpers_with_stage():
    from pxr import Usd, UsdGeom
    from simwright import kit_helpers as kh
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0); UsdGeom.SetStageUpAxis(st, "Z")
    UsdGeom.Xform.Define(st, "/World")
    kh._TEST_STAGE = st
    return kh, st


def test_ray_down_heights_pure():
    from simwright.kit_helpers import ray_down_heights, rest_height
    floor = [[(0, 0, 0), (10, 0, 0), (10, 10, 0)], [(0, 0, 0), (10, 10, 0), (0, 10, 0)]]
    shelf = [[(2, 2, 1), (4, 2, 1), (4, 4, 1)], [(2, 2, 1), (4, 4, 1), (2, 4, 1)]]
    wall = [[(5, 5, 0), (5, 6, 0), (5, 5, 3)]]                     # vertical: never a surface
    h = ray_down_heights(floor + shelf + wall, [(3, 3), (8, 8), (20, 20)])
    assert h[0] == 1.0 and h[1] == 0.0 and h[2] is None
    assert ray_down_heights(floor + shelf, [(3, 3)], z_ceiling=0.5) == [0.0]
    z, frac = rest_height(floor + shelf, (3.5, 3.5, 2), (4.5, 4.5, 3))   # half over the shelf edge
    assert z == 1.0 and 0.3 < frac < 0.7


def test_place_usd_uses_the_surface_not_the_bounds(tmp_path):
    kh, st = _helpers_with_stage()
    try:
        # table in centimetres, Y-up: top at 75 cm, with a 60 cm lamp standing in one corner
        table = _asset(tmp_path, "table", [((-60, 72, -40), (60, 75, 40)), ((-60, 0, -40), (-55, 72, -35)),
                                           ((50, 75, 25), (58, 135, 33))], mpu=0.01, up="Y")
        mug = _asset(tmp_path, "mug", [((-0.04, -0.04, 0), (0.04, 0.04, 0.1))])
        _box_mesh(st, "/World/floor", (-5, -5, -0.1), (5, 5, 0))
        rt = kh.place_usd("/World/table", table, 0, 0)
        assert abs(rt["bottom_m"] - 0.0) < 0.003 and abs(rt["size_m"][2] - 1.35) < 0.01   # Y-up cm asset stands upright, 1.35 m with lamp
        r = kh.place_usd("/World/mug", mug, -0.2, 0.0, on="/World/table")
        assert abs(r["bottom_m"] - 0.75) < 0.003, r                     # on the tabletop, not on the lamp (1.35 m)
        assert r["supported_fraction"] == 1.0 and r["inside_support_xy"]
        r2 = kh.place_usd("/World/mug2", mug, 0.54, -0.29, on="/World/table")  # over the lamp (Y-up +Z faces -Y)
        assert r2["bottom_m"] > 1.3                                     # footprint overlaps the lamp: it rests on top of it
    finally:
        kh._TEST_STAGE = None


def test_place_usd_inside_moved_scaled_group_and_raised_floor(tmp_path):
    from pxr import UsdGeom, Gf
    kh, st = _helpers_with_stage()
    try:
        g = UsdGeom.Xform.Define(st, "/World/Group")
        g.AddTranslateOp().Set(Gf.Vec3d(5, 0, 0.3)); g.AddRotateZOp().Set(30.0); g.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
        _box_mesh(st, "/World/platform", (-10, -10, 0), (10, 10, 0.2))      # raised floor at 0.2 m
        crate = _asset(tmp_path, "crate", [((-0.25, -0.25, 0), (0.25, 0.25, 0.5))])
        r = kh.place_usd("/World/Group/crate", crate, 2.0, 1.0)
        assert abs(r["bottom_m"] - 0.2) < 0.003, r                      # lands on the platform, not at z = 0 or the group's 0.3
        cx = (r["bbox_min"][0] + r["bbox_max"][0]) / 2; cy = (r["bbox_min"][1] + r["bbox_max"][1]) / 2
        assert abs(cx - 2.0) < 0.003 and abs(cy - 1.0) < 0.003          # world target despite the group transform
        assert abs(r["size_m"][2] - 0.5) < 0.003                        # the group's 0.5 scale did not shrink it
        r = kh.place_usd("/World/Group/crate2", crate, 2.0, 1.0, z=1.25)
        assert abs(r["bottom_m"] - 1.25) < 0.003
    finally:
        kh._TEST_STAGE = None


def test_mount_follows_link_and_touches_face(tmp_path):
    from pxr import UsdGeom, Gf
    kh, st = _helpers_with_stage()
    try:
        arm = UsdGeom.Xform.Define(st, "/World/arm")
        arm.AddTranslateOp().Set(Gf.Vec3d(1, 2, 1)); arm.AddRotateYOp().Set(90.0)        # link x-axis now points down (-Z world)
        link = UsdGeom.Xform.Define(st, "/World/arm/wrist")
        _box_mesh(st, "/World/arm/wrist/geo", (-0.03, -0.03, -0.05), (0.03, 0.03, 0.05))
        cam = kh.mount("wrist_cam", "camera", "/World/arm/wrist", face="+x", look="+x")
        assert cam["path"] == "/World/arm/wrist/wrist_cam"
        fw = cam["camera_forward_world"]
        assert abs(fw[2] + 1) < 1e-3                                   # looks along the link's +X = world down
        assert abs(cam["local_translate_m"][0] - 0.03) < 1e-4          # on the +X face of the link
        cam_box = _asset(tmp_path, "cambox", [((-0.02, -0.01, 0), (0.02, 0.01, 0.025))], mpu=1.0)
        m = kh.mount("d435", cam_box, "/World/arm/wrist", face="+z")
        assert abs(m["touch_gap_m"]) < 1e-4, m                         # sits exactly on the +Z face
        assert abs(m["local_translate_m"][2] - 0.05) < 1e-4
        m2 = kh.mount("d435", cam_box, "/World/arm/wrist", face="+z", gap=0.005)   # re-mount replaces
        assert abs(m2["touch_gap_m"] - 0.005) < 1e-4 and len([c for c in st.GetPrimAtPath("/World/arm/wrist").GetChildren() if c.GetName() == "d435"]) == 1
    finally:
        kh._TEST_STAGE = None


def test_placement_audit_flags_floating(tmp_path):
    from pxr import UsdGeom, Gf
    kh, st = _helpers_with_stage()
    try:
        _box_mesh(st, "/World/floor/geo", (-5, -5, -0.1), (5, 5, 0))
        _box_mesh(st, "/World/things/ok/geo", (0, 0, 0), (0.3, 0.3, 0.3))
        _box_mesh(st, "/World/things/floater/geo", (1, 1, 0.08), (1.3, 1.3, 0.38))
        _box_mesh(st, "/World/things/sunk/geo", (2, 2, -0.05), (2.3, 2.3, 0.25))
        a = kh.placement_audit("/World/things", depth=1)
        by = {r["path"].split("/")[-1]: r for r in a["rows"]}
        assert by["ok"]["state"] == "resting" and by["floater"]["state"] == "floating" and abs(by["floater"]["gap_m"] - 0.08) < 1e-3
        assert by["sunk"]["state"] in ("sunk", "no surface below") and not a["ok"]
    finally:
        kh._TEST_STAGE = None



def test_place_asset_fit_and_floor(tmp_path):
    kh, st = _helpers_with_stage()
    try:
        _box_mesh(st, "/World/floor", (-5, -5, -0.1), (5, 5, 0.05))
        desk = _asset(tmp_path, "desk", [((-80, 0, -40), (80, 75, 40))], mpu=0.01, up="Y")   # 1.6 x 0.8 m, Y-up cm
        r = kh.place_asset("/World/desk", desk, 1.0, 1.0, yaw_deg=90, fit=[1.2, 0.6])
        assert abs(r["bottom_m"] - 0.05) < 0.003 and abs(max(r["size_m"][:2]) - 1.2) < 0.01 and abs(min(r["size_m"][:2]) - 0.6) < 0.01, r
        b = kh.place_asset("/World/box", "", 2.0, 2.0, fit=[0.5, 0.5, 0.4])
        assert abs(b["bbox_min"][2] - 0.05) < 0.003
    finally:
        kh._TEST_STAGE = None



def test_project_memory_roundtrip(tmp_path, monkeypatch):
    from simwright.memory import ProjectMemory
    pm = ProjectMemory(tmp_path)
    msgs = [{"role": "system", "content": "sys"}, {"role": "tool", "tool_call_id": "x", "content": "{}"},
            {"role": "user", "content": [{"type": "text", "text": "build a kitchen"}, {"type": "image_url", "image_url": {"url": "data:..."}}]},
            {"role": "assistant", "content": "done, saved ~/robotics/scenes/kitchen.usd"}]
    pm.save_conversation(msgs)
    back = pm.load_conversation()
    assert back[0]["role"] == "user" and back[0]["content"] == "build a kitchen" and len(back) == 2   # no system, no orphan tool, no image
    pm.add_note("saved kitchen at ~/robotics/scenes/kitchen.usd", "scene")
    assert "kitchen.usd" in pm.notes()
    a = pm.archive("- built kitchen")
    assert a["archived"] and pm.load_conversation() == [] and pm.history()[0]["title"] == "build a kitchen"


def test_serve_events_persist_across_restart(tmp_path, monkeypatch):
    from simwright import serve
    monkeypatch.setattr(config, "HOME", tmp_path / "home")
    st = serve.State(tmp_path)
    st.emit({"type": "user", "text": "hello"}); st.emit({"type": "text", "text": "hi there"}); st.emit({"type": "done"})
    st2 = serve.State(tmp_path)
    kinds = [(e["type"], e.get("text")) for e in st2.events]
    assert ("user", "hello") in kinds and ("text", "hi there") in kinds and all(e.get("restored") for e in st2.events)
    assert [e["n"] for e in st2.events] == list(range(len(st2.events)))



def test_physics_preflight_flags_crash_setups(tmp_path):
    from pxr import UsdGeom, UsdPhysics, Sdf, Gf
    kh, st = _helpers_with_stage()
    try:
        sc = UsdPhysics.Scene.Define(st, "/World/PhysicsScene").GetPrim()
        assert kh.physics_preflight()["ok"]
        sc.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
        r = kh.physics_preflight()
        assert not r["ok"] and any("CCD" in i["issue"] for i in r["issues"])
        sc.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(False)
        sc.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set("MBP")
        assert kh.physics_preflight()["ok"]                      # CCD is fine on a CPU scene
        # xArm-style: root on the arm's root_joint and again on the gripper's root_joint
        for p in ("/World/Robots/XArm/root_joint", "/World/Robots/XArm/gripper/root_joint"):
            j = UsdPhysics.FixedJoint.Define(st, p).GetPrim()
            UsdPhysics.ArticulationRootAPI.Apply(j)
        r = kh.physics_preflight()
        assert not r["ok"] and any("nested" in i["issue"] and i["body"].endswith("/gripper") for i in r["issues"])
        # separate robots side by side are fine
        st.RemovePrim("/World/Robots/XArm/gripper")
        j = UsdPhysics.FixedJoint.Define(st, "/World/Robots/XArm2/root_joint").GetPrim(); UsdPhysics.ArticulationRootAPI.Apply(j)
        assert kh.physics_preflight()["ok"]
        # triangle-mesh collider on a dynamic body is a warning, not a block
        b = UsdGeom.Xform.Define(st, "/World/box").GetPrim(); UsdPhysics.RigidBodyAPI.Apply(b)
        m = _box_mesh(st, "/World/box/geo", (0, 0, 0), (1, 1, 1)).GetPrim(); UsdPhysics.CollisionAPI.Apply(m)
        r = kh.physics_preflight()
        assert r["ok"] and any("triangle-mesh" in i["issue"] for i in r["issues"])
    finally:
        kh._TEST_STAGE = None


def test_crash_report_detects_dead_kit(tmp_path):
    import json as _j, os as _os
    from simwright.tools import crash_report
    f = tmp_path / "physics_check.json"
    f.write_text(_j.dumps({"state": "running", "kit_pid": 999999, "backup": "/b/scene-x.usda", "stage": "/s/scene.usd", "started": 1}))
    rep = crash_report(marker=f)
    assert rep and rep["backup"] == "/b/scene-x.usda"
    f.write_text(_j.dumps({"state": "running", "kit_pid": _os.getpid(), "started": 1}))
    assert crash_report(marker=f) is None                      # still alive: not a crash
    f.write_text(_j.dumps({"state": "done", "kit_pid": 999999}))
    assert crash_report(marker=f) is None



def test_physics_repair_fixes_overrides_only(tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
    # robot asset: arm articulation (root on a fixed joint), gripper with its own nested root, limp drives, a 1 g fingertip
    f = tmp_path / "robot.usda"
    a = Usd.Stage.CreateNew(str(f)); UsdGeom.SetStageMetersPerUnit(a, 1.0); UsdGeom.SetStageUpAxis(a, "Z")
    r = UsdGeom.Xform.Define(a, "/Robot"); a.SetDefaultPrim(r.GetPrim())
    for b in ("base", "link7", "gripper/base", "gripper/tip"):
        p = UsdGeom.Xform.Define(a, f"/Robot/{b}").GetPrim(); UsdPhysics.RigidBodyAPI.Apply(p)
        m = UsdPhysics.MassAPI.Apply(p); m.CreateMassAttr(0.001 if b.endswith("tip") else 1.0)
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-7, 1e-7, 1e-7) if b.endswith("tip") else Gf.Vec3f(0.01, 0.01, 0.01))
    rj = UsdPhysics.FixedJoint.Define(a, "/Robot/root_joint"); rj.CreateBody1Rel().SetTargets(["/Robot/base"]); UsdPhysics.ArticulationRootAPI.Apply(rj.GetPrim())
    for name, b0, b1 in (("elbow_joint", "/Robot/base", "/Robot/link7"), ("finger_joint", "/Robot/gripper/base", "/Robot/gripper/tip")):
        j = UsdPhysics.RevoluteJoint.Define(a, f"/Robot/{name}"); j.CreateBody0Rel().SetTargets([b0]); j.CreateBody1Rel().SetTargets([b1])
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular"); d.CreateStiffnessAttr(0.0); d.CreateDampingAttr(0.0)
    gj = UsdPhysics.FixedJoint.Define(a, "/Robot/gripper/root_joint"); gj.CreateBody0Rel().SetTargets(["/Robot/link7"]); gj.CreateBody1Rel().SetTargets(["/Robot/gripper/base"])
    UsdPhysics.ArticulationRootAPI.Apply(gj.GetPrim())
    a.Save()
    before = f.read_text()
    kh, st = _helpers_with_stage()
    try:
        sc = UsdPhysics.Scene.Define(st, "/World/PhysicsScene").GetPrim()
        sc.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
        st.DefinePrim("/World/Robots/XArm", "Xform").GetReferences().AddReference(str(f))
        assert not kh.physics_preflight()["ok"]
        prev = kh.physics_repair("/World")
        assert prev["changes"] > 0 and not prev["applied"]
        assert st.GetPrimAtPath("/World/Robots/XArm/gripper/root_joint").HasAPI(UsdPhysics.ArticulationRootAPI)   # preview changes nothing
        res = kh.physics_repair("/World", apply=True)
        assert res["summary"].get("nested articulation root") == 1
        assert not st.GetPrimAtPath("/World/Robots/XArm/gripper/root_joint").HasAPI(UsdPhysics.ArticulationRootAPI)
        assert st.GetPrimAtPath("/World/Robots/XArm/root_joint").HasAPI(UsdPhysics.ArticulationRootAPI)          # outer root kept
        fj = st.GetPrimAtPath("/World/Robots/XArm/finger_joint")
        assert fj.GetAttribute("drive:angular:physics:stiffness").Get() == 5.0 and abs(fj.GetAttribute("physxJoint:armature").Get() - 0.005) < 1e-6
        assert st.GetPrimAtPath("/World/Robots/XArm/elbow_joint").GetAttribute("drive:angular:physics:stiffness").Get() == 150.0
        tip = st.GetPrimAtPath("/World/Robots/XArm/gripper/tip")
        assert abs(tip.GetAttribute("physics:mass").Get() - 0.02) < 1e-6 and abs(tip.GetAttribute("physics:diagonalInertia").Get()[0] - 2e-6) < 1e-9
        assert sc.GetAttribute("physxScene:enableCCD").Get() is False
        assert kh.physics_preflight()["ok"]
        assert kh.physics_repair("/World")["changes"] == 0        # idempotent
        assert f.read_text() == before                            # the robot asset file is untouched
    finally:
        kh._TEST_STAGE = None



def test_physics_repair_skips_mimic_followers(tmp_path):
    from pxr import UsdPhysics, UsdGeom
    kh, st = _helpers_with_stage()
    try:
        for b in ("a", "b"):
            UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(st, f"/World/g/{b}").GetPrim())
        j = UsdPhysics.RevoluteJoint.Define(st, "/World/g/follower_finger_joint")
        j.CreateBody0Rel().SetTargets(["/World/g/a"]); j.CreateBody1Rel().SetTargets(["/World/g/b"])
        j.GetPrim().AddAppliedSchema("PhysxMimicJointAPI:rotZ")
        r = kh.physics_repair("/World", apply=True)
        assert not any(c["prim"].endswith("follower_finger_joint") for c in r["details"])
    finally:
        kh._TEST_STAGE = None



def test_preflight_ignores_links_inside_one_articulation(tmp_path):
    from pxr import UsdPhysics, UsdGeom
    kh, st = _helpers_with_stage()
    try:
        UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
        g = UsdGeom.Xform.Define(st, "/World/G1/Geometry").GetPrim(); UsdPhysics.ArticulationRootAPI.Apply(g)
        for p in ("/World/G1/Geometry/pelvis", "/World/G1/Geometry/pelvis/hip", "/World/loose", "/World/loose/inner"):
            UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(st, p).GetPrim())
        bodies = [i["body"] for i in kh.physics_preflight()["issues"] if "nested inside rigid body" in i["issue"]]
        assert bodies == ["/World/loose/inner"]
    finally:
        kh._TEST_STAGE = None



def _standin_docs_server(port):
    """A local server with NVIDIA's Isaac Sim MCP tool names and argument shapes."""
    import threading, time as _t, uvicorn
    try:
        from mcp.server.mcpserver import MCPServer as _S
    except ImportError:
        from mcp.server.fastmcp import FastMCP as _S
    m = _S("isaac-sim-docs-standin")

    @m.tool()
    def search_isaac_sim_code_examples(request: str, top_k: int = 5) -> str:
        return f"example for: {request} -> from isaacsim.core.api import World"

    @m.tool()
    def get_isaac_sim_extension_details(extension_ids: list[str]) -> str:
        return "details: " + ",".join(extension_ids)

    @m.tool()
    def get_isaac_sim_instructions() -> str:
        return "use isaacsim.* namespaces"
    srv = uvicorn.Server(uvicorn.Config(m.streamable_http_app(), host="127.0.0.1", port=port, log_level="error"))
    th = threading.Thread(target=srv.run, daemon=True); th.start()
    for _ in range(100):
        if srv.started:
            break
        _t.sleep(0.05)
    return srv


def test_isaac_docs_client_against_standin():
    import socket
    from simwright import docs
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    srv = _standin_docs_server(port)
    try:
        ep = f"http://127.0.0.1:{port}/mcp"
        st = docs.status(ep)
        assert st["ok"] and "search_isaac_sim_code_examples" in st["tools"], st
        r = docs.search("spawn a franka and step physics", "code", ep)
        assert r["ok"] and "spawn a franka" in r["result"], r
        r = docs.search("isaacsim.sensors.camera, isaacsim.core.api", "extension", ep)
        assert "isaacsim.sensors.camera,isaacsim.core.api" in r["result"], r
        assert "isaacsim" in docs.search("", "instructions", ep)["result"]
        bad = docs.search("x", "code", "http://127.0.0.1:1/mcp")
        assert not bad["ok"] and "simwright docs setup" in bad["fix"]
    finally:
        srv.should_exit = True



def test_connect_isaac_docs_entries(tmp_path):
    import json as _j
    from simwright.connect import connect_docs
    r = connect_docs("cursor", tmp_path, home=tmp_path, url="http://localhost:9904/mcp")
    assert _j.loads((tmp_path / ".cursor" / "mcp.json").read_text())["mcpServers"]["isaac-sim-mcp"]["url"] == "http://localhost:9904/mcp"
    connect_docs("codex", tmp_path, home=tmp_path, url="http://localhost:9904/mcp")
    connect_docs("codex", tmp_path, home=tmp_path, url="http://localhost:9904/mcp")
    toml = (tmp_path / ".codex" / "config.toml").read_text()
    assert toml.count("[mcp_servers.isaac-sim-mcp]") == 1
    assert "--transport http" in connect_docs("claude", tmp_path, print_only=True, url="http://x/mcp")["run_this"]



def test_rosbridge_pieces(tmp_path, monkeypatch):
    from simwright import rosbridge as rb
    assert rb.dds_ports(0) == (7400, 7649) and rb.dds_ports(31) == (15150, 15399)
    xml = rb.profile_xml()
    assert "<useBuiltinTransports>false</useBuiltinTransports>" in xml and "<address>127.0.0.1</address>" in xml
    import xml.dom.minidom as md
    md.parseString(xml)                                           # well-formed
    assert "initialPeersList" not in rb.profile_xml([])
    env = rb.launch_env(r"C:\isaac-sim", {"distro": "humble", "domain_id": 31, "profile": "p.xml"}, base_env={"PATH": "x"})
    key = "PATH" if __import__("os").name == "nt" else "LD_LIBRARY_PATH"
    assert env["ROS_DISTRO"] == "humble" and env["ROS_DOMAIN_ID"] == "31" and env["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert "isaacsim.ros2.core" in env[key] and env["FASTRTPS_DEFAULT_PROFILES_FILE"] == "p.xml"
    assert rb.wsl_domain_from_bashrc("source /opt/ros/humble/setup.bash\nexport ROS_DOMAIN_ID=31\n") == 31
    assert rb.wsl_domain_from_bashrc("# export ROS_DOMAIN_ID=4") is None
    assert "15150-15399" in rb.firewall_ps(31) and "-RemoteAddresses $ips" in rb.firewall_ps(31)
    out = 'noise\nSWJSON {"topics": "/clock /parameter_events /rosout /g1/joint_states ", "clock_sec": "12", "rates": {"/clock": "59.8","/g1/joint_states": "60.1"}}\n'
    r = rb.parse_check(out)
    assert r["ok"] and r["from_isaac_sim"] == ["/clock", "/g1/joint_states"] and r["rates_hz"]["/clock"] == 59.8
    bad = rb.parse_check('SWJSON {"topics": "/rosout", "clock_sec": "", "rates": {"/clock": "0"}}')
    assert not bad["ok"]
    s = rb.check_script(31, ["/clock", "/a/joint_states"])
    assert "ROS_DOMAIN_ID=31" in s and "SWJSON" in s



def test_kit_call_expressions_are_python(tmp_path, monkeypatch):
    """Every expression a tool sends to Isaac Sim must compile as Python (no JSON null/true/false)."""
    import ast
    from simwright.tools import Toolbox
    sent = []

    class FakeKit:
        def call(self, expr, timeout=None):
            ast.parse("async def _f():\n    _r = " + expr.replace("\n", "\n    "))
            sent.append(expr)
            return {"ok": True, "topics": []}

    tb = Toolbox(tmp_path)
    tb.kit = FakeKit()
    tb.runner.approver.mode = "auto"
    tb.isaac_place("C:/a.usd", "/World/a")
    tb.isaac_place("C:/a.usd", "/World/a", on="/World/table", z=0.5)
    tb.isaac_mount("cam", "camera", "/World/arm/link7")
    tb.isaac_mount("cam", "C:/c.usd", "/World/arm/link7", look="+x", offset=[0, 0, 0.01])
    tb.ros_bridge_graph()
    tb.ros_bridge_graph(robots=["/World/r"], cameras=["/World/cam"], persist=True)
    tb.isaac_physics_repair("/World")
    tb.isaac_placement_audit("/World")
    assert len(sent) >= 9 and not any(" null" in s or "(null" in s for s in sent)
