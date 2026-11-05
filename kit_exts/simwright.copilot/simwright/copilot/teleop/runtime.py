"""Rokoko suit + gloves -> Unitree G1 (29 DoF + Revo2 hands) teleoperation inside the running Isaac Sim.

Legs: NVIDIA WBC-AGILE velocity-height policy (50 Hz) driven by commands from the performer's hips.
Waist, arms, hands: retargeted from the suit and gloves. Runs in a physics-step callback on Kit's main thread.
All stage changes (released world pin, physics rate) go on the session layer and are undone by stop().
"""
from __future__ import annotations
import json, math, os, time, traceback
import numpy as np

from . import rokoko, retarget_g1 as rg
from .onnx_np import NpGraph

DATA = os.path.join(os.path.dirname(__file__), "data", "agile_g1_velocity_height")
_SESSION = None


def _quat_to_mat(x, y, z, w):
    return rokoko.quat_to_mat((x, y, z, w))


class G1Teleop:
    def __init__(self, robot=None, port=rokoko.DEFAULT_PORT, walk=True, physics_hz=200, actor=None, hands=True, arms=True):
        import omni.usd
        self.stage = omni.usd.get_context().get_stage()
        self.robot = robot or self._find_robot()
        self.walk, self.physics_hz, self.hands, self.arms = walk, int(physics_hz), hands, arms
        self.port, self.actor = port, actor
        self.desc = json.load(open(os.path.join(DATA, "descriptor.json")))
        self.policy = NpGraph(os.path.join(DATA, "policy.npz"))
        z = np.load(os.path.join(DATA, "initial_values.npz"))
        self.init_state = {k: z[k].astype(np.float32) for k in z.files}
        self.state = {k: v.copy() for k, v in self.init_state.items()}
        self.decim = max(1, round(self.physics_hz / self.desc["frequency"]))
        self.retarget = rg.G1Retarget()
        self.rx = None; self.sub = None; self.view = None; self.art = None
        self.overrides = []
        self.steps = 0; self.policy_steps = 0; self.t_start = 0.0; self.last_policy_ms = 0.0; self.last_retarget_ms = 0.0
        self.cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "h": self.desc["default_pelvis_height"]}
        self.manual = None
        self.errors = []; self.fallen = False; self.info = {}
        self.calibrate_at = None

    # ---------- setup ----------
    def _find_robot(self):
        from pxr import UsdPhysics
        roots = [p for p in self.stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        waists = [str(p.GetPath()) for p in self.stage.Traverse() if p.GetName() == "waist_yaw_joint"]
        for p in roots:
            base = str(p.GetParent().GetPath())
            if any(w.startswith(base + "/") for w in waists):
                return str(p.GetPath())
        raise RuntimeError("no G1 articulation found (needs a waist_yaw_joint next to an articulation root)")

    def _override(self, prim_path, attr, value, type_name=None):
        from pxr import Usd, Sdf
        layer = self.stage.GetSessionLayer()
        with Usd.EditContext(self.stage, layer):
            prim = self.stage.GetPrimAtPath(prim_path)
            a = prim.GetAttribute(attr)
            if not a:
                a = prim.CreateAttribute(attr, type_name or Sdf.ValueTypeNames.Int)
            self.overrides.append((prim_path, attr))
            a.Set(value)

    def _restore(self):
        from pxr import Sdf
        layer = self.stage.GetSessionLayer()
        for prim_path, attr in reversed(self.overrides):
            spec = layer.GetPrimAtPath(prim_path)
            if spec and attr in spec.attributes:
                spec.RemoveProperty(spec.attributes[attr])
        self.overrides = []

    def _world_pins(self):
        from pxr import UsdPhysics
        base = self.robot.rsplit("/", 1)[0] if self.robot.endswith("/Geometry") else self.robot
        pins = []
        for p in self.stage.Traverse():
            if not str(p.GetPath()).startswith(base) or not p.IsA(UsdPhysics.FixedJoint):
                continue
            fj = UsdPhysics.FixedJoint(p)
            b0 = [str(x) for x in fj.GetBody0Rel().GetTargets()]; b1 = [str(x) for x in fj.GetBody1Rel().GetTargets()]
            def rigid(path):
                q = self.stage.GetPrimAtPath(path)
                return bool(q) and q.HasAPI(UsdPhysics.RigidBodyAPI)
            if (not b0 or not rigid(b0[0])) and b1 and b1[0].endswith("pelvis"):
                pins.append(str(p.GetPath()))
        return pins

    def start(self):
        import omni.timeline, omni.physx
        from pxr import Sdf, UsdPhysics, PhysxSchema
        tl = omni.timeline.get_timeline_interface()
        if tl.is_playing():
            tl.stop()
        self.pins = self._world_pins() if self.walk else []
        for pin in self.pins:
            self._override(pin, "physics:jointEnabled", False, Sdf.ValueTypeNames.Bool)
        for p in self.stage.Traverse():
            if p.IsA(UsdPhysics.Scene):
                self._override(str(p.GetPath()), "physxScene:timeStepsPerSecond", self.physics_hz, Sdf.ValueTypeNames.UInt)
                break
        self.rx = rokoko.Receiver(self.port, actor=self.actor)
        self.sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
        self.t_start = time.time()
        tl.play()
        return self.status()

    def stop(self):
        import omni.timeline
        try:
            if self.sub is not None:
                self.sub.unsubscribe() if hasattr(self.sub, "unsubscribe") else None
        except Exception:
            pass
        self.sub = None
        omni.timeline.get_timeline_interface().stop()
        self._restore()
        if self.rx:
            self.rx.close()
        self.view = self.art = None
        return {"ok": True, "stopped": True, "restored": "world pin and physics rate back to the scene's values"}

    def _bind(self):
        import omni.physics.tensors as pt
        self.view = pt.create_simulation_view("numpy")
        self.view.set_subspace_roots("/")
        last = None
        for path in (self.robot, self.robot.rsplit("/", 1)[0] + "/Geometry/pelvis"):
            try:
                a = self.view.create_articulation_view(path)
                if a is not None and a.count > 0:
                    self.art = a; break
            except Exception as e:
                last = e
        if self.art is None:
            raise RuntimeError(f"cannot bind articulation at {self.robot}: {last}")
        names = list(self.art.shared_metatype.dof_names)
        self.dof = {n: i for i, n in enumerate(names)}
        miss = [n for n in self.desc["joint_names"] if n not in self.dof]
        if miss:
            raise RuntimeError(f"robot is missing policy joints: {miss}")
        self.pol_idx = np.array([self.dof[n] for n in self.desc["joint_names"]])
        self.leg_idx = np.array([self.dof[n] for n in self.desc["leg_joint_names"]])
        self.idx0 = np.array([0], dtype=np.int32)
        n = len(names)
        kp = self.art.get_dof_stiffnesses().copy(); kd = self.art.get_dof_dampings().copy()
        kp[0, self.pol_idx] = self.desc["default_joint_stiffness"]; kd[0, self.pol_idx] = self.desc["default_joint_damping"]
        self.art.set_dof_stiffnesses(kp.astype(np.float32), self.idx0); self.art.set_dof_dampings(kd.astype(np.float32), self.idx0)
        self.targets = self.art.get_dof_position_targets().copy().astype(np.float32)
        q = self.art.get_dof_positions().copy()
        q0 = np.array(self.desc["default_joint_pos"], np.float32)
        q[0, self.pol_idx] = q0; self.targets[0, self.pol_idx] = q0
        self.art.set_dof_positions(q.astype(np.float32), self.idx0)
        self.art.set_dof_velocities(np.zeros_like(q, dtype=np.float32), self.idx0)
        self.art.set_dof_position_targets(self.targets, self.idx0)
        self.state = {k: v.copy() for k, v in self.init_state.items()}

    # ---------- control ----------
    def _policy(self, cmd):
        q = self.art.get_dof_positions()[0]; qd = self.art.get_dof_velocities()[0]
        tr = self.art.get_root_transforms()[0]; vel = self.art.get_root_velocities()[0]
        Rw = _quat_to_mat(*tr[3:7])
        ang_b = Rw.T @ vel[3:6]
        feeds = {"base_velocity": np.array([[cmd["vx"], cmd["vy"], cmd["wz"], cmd["h"]]], np.float32),
                 "robot_root_ang_vel_b": ang_b[None].astype(np.float32), "robot_root_quat_w": tr[None, 3:7].astype(np.float32),
                 "robot_joint_pos": q[self.pol_idx][None].astype(np.float32), "robot_joint_vel": qd[self.pol_idx][None].astype(np.float32)}
        feeds.update(self.state)
        t0 = time.perf_counter()
        out = self.policy(feeds)
        self.last_policy_ms = (time.perf_counter() - t0) * 1000
        for k in self.state:
            self.state[k] = out[k[:-3] + "_out"].astype(np.float32)
        if self.policy_steps == 0 or self.policy_steps % 50 == 0:
            kp = self.art.get_dof_stiffnesses().copy(); kd = self.art.get_dof_dampings().copy()
            kp[0, self.leg_idx] = out["joint_pos_kp_gains"][0]; kd[0, self.leg_idx] = out["joint_pos_kd_gains"][0]
            self.art.set_dof_stiffnesses(kp.astype(np.float32), self.idx0); self.art.set_dof_dampings(kd.astype(np.float32), self.idx0)
        self.targets[0, self.leg_idx] = out["joint_pos"][0]
        up = Rw[:, 2]
        self.info = {**{k: v for k, v in self.info.items() if k in ("calibration", "arm_err_mm")}, "root_xy": [round(float(tr[0]), 3), round(float(tr[1]), 3)], "yaw_deg": round(math.degrees(math.atan2(Rw[1, 0], Rw[0, 0])), 1), "pelvis_z": round(float(tr[2]), 3), "tilt_deg": round(math.degrees(math.acos(max(-1, min(1, up[2])))), 1)}
        if tr[2] < 0.35 or up[2] < 0.5:
            self.fallen = True

    def _on_step(self, dt):
        try:
            self.steps += 1
            if self.art is None:
                self._bind()
                return
            if self.steps % self.decim:
                return
            self.rx.poll()
            a = self.rx.actor()
            fresh = a is not None and self.rx.age() < 0.3 and a["meta"]["body"]
            if self.calibrate_at and fresh:
                self.info["calibration"] = self.retarget.calibrate(a["joints"]); self.calibrate_at = None
            cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "h": self.cmd["h"]}
            if fresh and self.retarget.cal is not None:
                t0 = time.perf_counter()
                o = self.retarget.update(a["joints"], 1.0 / self.desc["frequency"], gloves=(a["meta"]["left_hand"] and self.hands, a["meta"]["right_hand"] and self.hands))
                self.last_retarget_ms = (time.perf_counter() - t0) * 1000
                for name, v in o["targets"].items():
                    if name in self.dof and (self.arms or not ("shoulder" in name or "elbow" in name or "wrist" in name)):
                        self.targets[0, self.dof[name]] = v
                cmd = o["cmd"]; self.info["arm_err_mm"] = {s: round(e["wrist_err"] * 1000, 1) for s, e in o["arm_err"].items()}
            if self.manual is not None:
                cmd = dict(self.manual)
            self.cmd = cmd
            if self.walk and not self.fallen:
                self._policy(cmd)
                self.policy_steps += 1
            self.art.set_dof_position_targets(self.targets, self.idx0)
        except Exception as e:
            self.errors = (self.errors + [f"{type(e).__name__}: {e} | {traceback.format_exc()[-400:]}"])[-5:]

    def status(self):
        el = max(time.time() - self.t_start, 1e-6) if self.t_start else 0
        return {"running": self.sub is not None, "robot": self.robot, "walk": self.walk, "physics_hz": self.physics_hz, "decimation": self.decim,
                "released_pins": getattr(self, "pins", []), "physics_steps": self.steps, "policy_steps": self.policy_steps,
                "policy_hz": round(self.policy_steps / el, 1) if el else 0, "policy_ms": round(self.last_policy_ms, 2), "retarget_ms": round(self.last_retarget_ms, 2),
                "rokoko": self.rx.status() if self.rx else None, "calibrated": self.retarget.cal is not None, "cmd": {k: round(v, 3) for k, v in self.cmd.items()},
                "manual": self.manual, "fallen": self.fallen, **self.info, "errors": self.errors}


def start(**kw):
    global _SESSION
    if _SESSION is not None:
        try:
            _SESSION.stop()
        except Exception:
            pass
    _SESSION = G1Teleop(**kw)
    return _SESSION.start()


def stop():
    global _SESSION
    if _SESSION is None:
        return {"ok": True, "note": "not running"}
    r = _SESSION.stop(); _SESSION = None
    return r


def status():
    return _SESSION.status() if _SESSION else {"running": False}


def calibrate():
    if _SESSION is None:
        return {"ok": False, "error": "teleop is not running"}
    _SESSION.calibrate_at = time.time()
    return {"ok": True, "note": "stand in the calibration pose: upper arms at your sides, elbows bent 90 degrees, forearms forward, palms facing each other, fingers straight"}


def command(vx=0.0, vy=0.0, wz=0.0, h=None, clear=False):
    """Manual walk command (testing without the suit, or a joystick). clear=True hands control back to the suit."""
    if _SESSION is None:
        return {"ok": False, "error": "teleop is not running"}
    if clear:
        _SESSION.manual = None
    else:
        lim = rg.CMD_LIMITS
        _SESSION.manual = {"vx": float(np.clip(vx, *lim["vx"])), "vy": float(np.clip(vy, *lim["vy"])), "wz": float(np.clip(wz, *lim["wz"])),
                           "h": float(np.clip(h if h is not None else _SESSION.desc["default_pelvis_height"], *lim["h"]))}
    return {"ok": True, "manual": _SESSION.manual}
