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
    mcp, _ = build(make_ws(tmp_path / "b"))
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
