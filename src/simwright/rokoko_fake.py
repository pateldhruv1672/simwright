"""Synthetic Rokoko Studio stream for testing teleop without a suit: same UDP JSON (v3 or v4) Studio sends.

The virtual performer's arms are driven by known G1 arm joint angles (scaled to human proportions), so the retargeting
can be checked against ground truth. Script: calibration pose, wave, reach, fists, squat, step forward and back, turn.
"""
from __future__ import annotations
import json, math, socket, time
import numpy as np
from . import retarget_g1 as rg
from .rokoko import M

HUMAN = {"upper": 0.30, "fore": 0.26, "hip_z": 0.95, "shoulder_z": 1.42, "shoulder_y": 0.19}
_C0 = rg._rpy(0.3, -0.2, 0.5)      # arbitrary bone-axis conventions: the retargeting must not depend on them
_H0 = rg._rpy(-0.4, 0.1, 0.2)
_HAND0 = {"left": rg._rpy(0.2, 0.3, -0.1), "right": rg._rpy(-0.3, 0.2, 0.4)}


def _mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2; w = 0.25 * s
        x, y, z = (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; x = 0.25 * s
            w, y, z = (R[2, 1] - R[1, 2]) / s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; y = 0.25 * s
            w, x, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; z = 0.25 * s
            w, x, y = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s
    return {"x": float(x), "y": float(y), "z": float(z), "w": float(w)}


def _out(p, R):
    """robot frame -> Rokoko frame (inverse of rokoko.to_robot)."""
    pl = M.T @ np.asarray(p, float)
    return {"position": {"x": float(pl[0]), "y": float(pl[1]), "z": float(pl[2])}, "rotation": _mat_to_quat(M.T @ R @ M)}


def pose(t: float, script: bool = True) -> dict:
    """Ground-truth performer state at time t (robot frame)."""
    s = {"q": {"left": np.zeros(7), "right": np.zeros(7)}, "waist": (0.0, 0.0, 0.0), "curl": {"left": 0.0, "right": 0.0},
         "hip": np.array([0.0, 0.0, HUMAN["hip_z"]]), "yaw": 0.0}
    if not script or t < 3.0:
        return s
    u = t - 3.0
    k = lambda a, b: max(0.0, min(1.0, (u - a) / (b - a))) if b > a else 0.0
    bump = lambda a, b: math.sin(math.pi * k(a, b))
    # 0-4 s: right arm raises forward and waves
    s["q"]["right"] = np.array([-1.3 * bump(0, 4), -0.2 * bump(0, 4), 0.4 * math.sin(4 * u) * bump(0, 4), 0.6 * bump(0, 4), 0, 0, 0])
    # 4-8 s: both arms reach forward, hands rotate, waist yaws
    for side, sg in (("left", 1), ("right", -1)):
        s["q"][side] = s["q"][side] + np.array([-0.9 * bump(4, 8), 0.15 * sg * bump(4, 8), 0, -0.5 * bump(4, 8), 0.8 * bump(4, 8), 0.3 * bump(4, 8), 0])
    s["waist"] = (0.4 * bump(4, 8), 0.0, 0.15 * bump(4, 8))
    # 8-11 s: fists
    s["curl"] = {"left": bump(8, 11), "right": bump(8, 11)}
    # 11-14 s: squat
    s["hip"] = s["hip"] + np.array([0, 0, -0.25 * bump(11, 14)])
    # 14-20 s: step forward then back
    s["hip"] = s["hip"] + np.array([0.30 * bump(14, 20), 0, 0])
    # 20-25 s: turn left
    s["yaw"] = 0.8 * bump(20, 25)
    return s


def skeleton(st: dict) -> dict:
    """Joint name -> (position, rotation) in the robot frame for one performer state."""
    Ry = rg._axis([0, 0, 1], st["yaw"])
    P = Ry                                           # pelvis frame
    yw, rl, pt = st["waist"]
    T = P @ rg._axis([0, 0, 1], yw) @ rg._axis([1, 0, 0], rl) @ rg._axis([0, 1, 0], pt)   # torso frame (G1 waist chain)
    hip = st["hip"]
    chest_c = hip + P @ np.array([0, 0, 0.12]) + T @ np.array([0, 0, 0.30])
    j = {"hip": (hip.copy(), P @ _H0), "spine": (hip + P @ np.array([0, 0, 0.12]), T @ _C0), "chest": (chest_c, T @ _C0),
         "neck": (chest_c + T @ np.array([0, 0, 0.18]), T @ _C0), "head": (chest_c + T @ np.array([0, 0, 0.28]), T @ _C0)}
    for side, sg in (("left", 1), ("right", -1)):
        q = st["q"][side]
        S0, E0, W0, R0 = rg.arm_fk(side, np.zeros(7))
        S, E, W, Rh = rg.arm_fk(side, q)
        L1, L2 = rg.ARM_LEN[side]
        sh = chest_c + T @ np.array([0, sg * HUMAN["shoulder_y"], 0.08])
        el = sh + T @ ((E - S) / L1 * HUMAN["upper"])
        wr = el + T @ ((W - E) / L2 * HUMAN["fore"])
        Rhand = T @ Rh @ R0.T @ _HAND0[side]
        hs = side
        j[f"{hs}Shoulder"] = (sh - T @ np.array([0, sg * 0.05, 0]), T @ _C0)
        j[f"{hs}UpperArm"] = (sh, T @ _C0); j[f"{hs}LowerArm"] = (el, T @ _C0); j[f"{hs}Hand"] = (wr, Rhand)
        # fingers in the hand frame: forward along the robot hand's x, curling about its y; palm normal ~ -sg*y side
        Rw = T @ Rh                                   # hand frame (robot convention) for geometry
        c = st["curl"][side]
        for n, (F, off) in enumerate((("Index", 0.025), ("Middle", 0.008), ("Ring", -0.01), ("Little", -0.027))):
            base = wr + Rw @ np.array([0.09, 0.0, off])
            pts = [base]; ang = 0.0; d = 0.045
            for seg in range(3):
                ang += c * 1.0
                pts.append(pts[-1] + Rw @ (d * np.array([math.cos(ang), 0, 0]) + d * np.array([0, -sg * math.sin(ang), 0])))
                d *= 0.75
            for name, p in zip(("Proximal", "Medial", "Distal", "Tip"), pts):
                j[f"{hs}{F}{name}"] = (p, Rhand)
        tb = wr + Rw @ np.array([0.03, -sg * 0.02, 0.03])
        tpts = [tb]; d = 0.04
        for seg in range(3):
            a = 0.5 * c * (seg + 1)
            tpts.append(tpts[-1] + Rw @ (d * np.array([math.cos(a), -sg * 0.5 * math.sin(a) - sg * 0.6 * c, 0.6 * (1 - c)])))
            d *= 0.8
        for name, p in zip(("Proximal", "Medial", "Distal", "Tip"), tpts):
            j[f"{hs}Thumb{name}"] = (p, Rhand)
        j[f"{hs}UpLeg"] = (hip + P @ np.array([0, sg * 0.09, -0.05]), P @ _H0)
        j[f"{hs}Leg"] = (hip + P @ np.array([0, sg * 0.09, -0.47]), P @ _H0)
        j[f"{hs}Foot"] = (np.array([hip[0], hip[1], 0.08]) + P @ np.array([0, sg * 0.09, 0]), P @ _H0)
    return j


def frame_json(t: float, version: int = 3, script: bool = True, name: str = "Simwright test performer") -> bytes:
    j = skeleton(pose(t, script))
    meta = {"hasGloves": True, "hasLeftGlove": True, "hasRightGlove": True, "hasBody": True, "hasFace": False}
    if version == 4:
        inv = {v: k for k, v in __import__("simwright.rokoko", fromlist=["_V4"])._V4.items()}
        joints = [{"name": inv.get(n, n), "parent": -1, **_out(*v)} for n, v in j.items()]
        scene = {"timestamp": t, "actors": [], "newtons": [{"name": name, "color": [200, 90, 70],
                 "meta": {"hasHands": True, "hasLeftHand": True, "hasRightHand": True, "hasBody": True, "hasFace": False},
                 "dimensions": {"totalHeight": 1.75, "hipHeight": 0.95}, "joints": joints}], "props": [], "characters": []}
    else:
        body = {n: _out(*v) for n, v in j.items()}
        scene = {"timestamp": t, "actors": [{"name": name, "color": {"r": 200, "g": 90, "b": 70}, "meta": meta,
                 "dimensions": {"totalHeight": 175, "hipHeight": 95}, "body": body}], "props": []}
    return json.dumps({"version": 3, "fps": 60, "scene": scene}).encode()


def stream(seconds: float = 40.0, port: int = 14043, host: str = "127.0.0.1", version: int = 3, fps: int = 60, loop: bool = False) -> dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time(); n = 0
    while True:
        t = time.time() - t0
        if t > seconds:
            if not loop:
                break
            t0 = time.time(); t = 0.0
        sock.sendto(frame_json(t, version), (host, port)); n += 1
        time.sleep(max(0.0, t0 + n / fps - time.time()) if not loop else 1.0 / fps)
    return {"ok": True, "frames": n, "seconds": seconds, "port": port, "format": f"json-v{version}"}
