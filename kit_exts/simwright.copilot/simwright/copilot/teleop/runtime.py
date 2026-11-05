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
        import omni.kit.app
        self.sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
        self.upd = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update, name="simwright.teleop.bind")
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
        try:
            if getattr(self, "upd", None) is not None:
                self.upd.unsubscribe()
        except Exception:
            pass
        self.upd = None
        omni.timeline.get_timeline_interface().stop()
        self._restore()
        if self.rx:
            self.rx.close()
        self.view = self.art = None
        return {"ok": True, "stopped": True, "restored": "world pin and physics rate back to the scene's values"}

    def _get_sim_view(self):
        """Isaac Sim 6 creates the physics views itself on Play (after attaching the stage and warming up);
        use that shared view instead of creating our own."""
        try:
            from isaacsim.core.simulation_manager import SimulationManager
            v = SimulationManager.get_physics_sim_view()
            if v is not None:
                return v, SimulationManager.get_backend()
        except Exception as e:
            self.last_bind_error = f"SimulationManager: {e}"
        return None, None

    def _bind(self):
        self.view, backend = self._get_sim_view()
        if self.view is None:
            return False
        self.backend = backend or "torch"
        if self.backend == "torch":
            import torch
            self.torch = torch
        art = None; last = None
        for path in (self.robot, self.robot.rsplit("/", 1)[0] + "/Geometry/pelvis", self.robot.rsplit("/", 1)[0] + "/pelvis"):
            try:
                a = self.view.create_articulation_view(path)
                if a is not None and a.count > 0:
                    art = a; break
            except Exception as e:
                last = e
        if art is None:
            raise RuntimeError(f"cannot bind articulation at {self.robot}: {last}")
        self.art = art
        q_raw = self.art.get_dof_positions(); g_raw = self.art.get_dof_stiffnesses()
        self.dev = getattr(q_raw, "device", None); self.gdev = getattr(g_raw, "device", None)
        names = list(self.art.shared_metatype.dof_names)
        self.dof = {n: i for i, n in enumerate(names)}
        miss = [n for n in self.desc["joint_names"] if n not in self.dof]
        if miss:
            self.art = None
            raise RuntimeError(f"robot is missing policy joints: {miss}")
        self.pol_idx = np.array([self.dof[n] for n in self.desc["joint_names"]])
        self.leg_idx = np.array([self.dof[n] for n in self.desc["leg_joint_names"]])
        self.idx0 = self.torch.tensor([0], dtype=self.torch.int32, device=self.dev) if self.backend == "torch" else np.array([0], dtype=np.int32)
        kp = self._np(g_raw); kd = self._np(self.art.get_dof_dampings())
        kp[0, self.pol_idx] = self.desc["default_joint_stiffness"]; kd[0, self.pol_idx] = self.desc["default_joint_damping"]
        self.art.set_dof_stiffnesses(self._t(kp, gain=True), self.idx0); self.art.set_dof_dampings(self._t(kd, gain=True), self.idx0)
        self.targets = self._np(self.art.get_dof_position_targets())
        q = self._np(q_raw)
        q0 = np.array(self.desc["default_joint_pos"], np.float32)
        q[0, self.pol_idx] = q0; self.targets[0, self.pol_idx] = q0
        self.art.set_dof_positions(self._t(q), self.idx0)
        self.art.set_dof_velocities(self._t(np.zeros_like(q)), self.idx0)
        self.art.set_dof_position_targets(self._t(self.targets), self.idx0)
        self.state = {k: v.copy() for k, v in self.init_state.items()}
        self.bound_at = time.time(); self.bind_steps = self.steps
        return True

    def _np(self, x):
        if hasattr(x, "detach"):
            return x.detach().cpu().numpy().astype(np.float32).copy()
        return np.array(x, dtype=np.float32, copy=True)

    def _t(self, a, gain=False):
        a = np.ascontiguousarray(a, dtype=np.float32)
        if self.backend == "torch":
            return self.torch.as_tensor(a, device=self.gdev if gain else self.dev)
        return a

    def _on_update(self, e):
        """App-update tick (not a physics step): bind once the simulation is up; at most 2 tries/s, give up after 60."""
        if self.art is not None or self.sub is None or getattr(self, "bind_gave_up", False):
            return
        now = time.time()
        if now - getattr(self, "last_bind_try", 0.0) < 0.5:
            return
        self.last_bind_try = now
        self.bind_tries = getattr(self, "bind_tries", 0) + 1
        try:
            import omni.timeline
            if omni.timeline.get_timeline_interface().is_playing():
                self._bind()
        except Exception as ex:
            self.last_bind_error = f"{type(ex).__name__}: {ex}"
        if self.art is None and self.bind_tries >= 60:
            self.bind_gave_up = True
            self.errors = (self.errors + [f"could not bind the robot after 30 s: {getattr(self, 'last_bind_error', 'no physics view')}"])[-5:]

    # ---------- control ----------
    def _policy(self, cmd):
        q = self._np(self.art.get_dof_positions())[0]; qd = self._np(self.art.get_dof_velocities())[0]
        tr = self._np(self.art.get_root_transforms())[0]; vel = self._np(self.art.get_root_velocities())[0]
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
            kp = self._np(self.art.get_dof_stiffnesses()); kd = self._np(self.art.get_dof_dampings())
            kp[0, self.leg_idx] = out["joint_pos_kp_gains"][0]; kd[0, self.leg_idx] = out["joint_pos_kd_gains"][0]
            self.art.set_dof_stiffnesses(self._t(kp, gain=True), self.idx0); self.art.set_dof_dampings(self._t(kd, gain=True), self.idx0)
        self.targets[0, self.leg_idx] = out["joint_pos"][0]
        up = Rw[:, 2]
        self.info = {**{k: v for k, v in self.info.items() if k in ("calibration", "arm_err_mm")}, "root_xy": [round(float(tr[0]), 3), round(float(tr[1]), 3)], "yaw_deg": round(math.degrees(math.atan2(Rw[1, 0], Rw[0, 0])), 1), "pelvis_z": round(float(tr[2]), 3), "tilt_deg": round(math.degrees(math.acos(max(-1, min(1, up[2])))), 1)}
        if tr[2] < 0.35 or up[2] < 0.5:
            self.fallen = True

    def _on_step(self, dt):
        try:
            self.steps += 1
            if self.art is None:
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
            self.art.set_dof_position_targets(self._t(self.targets), self.idx0)
        except Exception as e:
            msg = f"{type(e).__name__}: {e}"
            self.err_count = getattr(self, "err_count", 0) + 1
            if msg not in self.errors:
                self.errors = (self.errors + [msg + " | " + traceback.format_exc()[-300:]])[-5:]
            if self.err_count > 200:          # a persistent fault: stop driving the robot instead of erroring every step
                self.walk = False; self.fault = msg

    def status(self):
        el = max(time.time() - self.t_start, 1e-6) if self.t_start else 0
        try:
            import omni.timeline
            sim_t = omni.timeline.get_timeline_interface().get_current_time()
        except Exception:
            sim_t = None
        return {"running": self.sub is not None, "bound": self.art is not None, "sim_time": round(sim_t, 2) if sim_t is not None else None,
                "wall_time": round(el, 2), "physics_hz_measured": round(self.steps / el, 1) if el else 0, "robot": self.robot, "walk": self.walk, "physics_hz": self.physics_hz, "decimation": self.decim,
                "released_pins": getattr(self, "pins", []), "physics_steps": self.steps, "policy_steps": self.policy_steps,
                "policy_hz": round(self.policy_steps / el, 1) if el else 0, "policy_ms": round(self.last_policy_ms, 2), "retarget_ms": round(self.last_retarget_ms, 2),
                "bind_tries": getattr(self, "bind_tries", 0), "bind_error": getattr(self, "last_bind_error", None), "backend": getattr(self, "backend", None),
                "fault": getattr(self, "fault", None), "rokoko": self.rx.status() if self.rx else None, "calibrated": self.retarget.cal is not None, "cmd": {k: round(v, 3) for k, v in self.cmd.items()},
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
