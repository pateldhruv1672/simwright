"""Rokoko (Smartsuit Pro + Smartgloves) -> Unitree G1 29-DoF + BrainCo Revo2 joint targets and AGILE walk commands.

Calibration pose = the G1's zero pose: stand upright, upper arms hanging at your sides, elbows bent 90 degrees with the
forearms pointing straight forward, palms facing each other, fingers straight, thumbs relaxed. Everything after that is
measured relative to it, so Rokoko's own bone axes never matter.

Arms: a 7-joint damped least-squares solve per arm matching the elbow and wrist positions (scaled to the G1 arm lengths)
and the hand orientation, using the joint frames of g1_29dof_mode_15_brainco_hand.urdf.
"""
from __future__ import annotations
import math
import numpy as np

ARM = {"left": [{"name": "left_shoulder_pitch_joint", "xyz": [0.0039563, 0.10022, 0.24778], "rpy": [0.27931, 5.4949e-05, -0.00019159], "axis": [0.0, 1.0, 0.0], "lim": [-3.0892, 2.6704]}, {"name": "left_shoulder_roll_joint", "xyz": [0.0, 0.038, -0.013831], "rpy": [-0.27925, 0.0, 0.0], "axis": [1.0, 0.0, 0.0], "lim": [-1.5882, 2.2515]}, {"name": "left_shoulder_yaw_joint", "xyz": [0.0, 0.00624, -0.1032], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "lim": [-2.618, 2.618]}, {"name": "left_elbow_joint", "xyz": [0.015783, 0.0, -0.080518], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 1.0, 0.0], "lim": [-1.0472, 2.0944]}, {"name": "left_wrist_roll_joint", "xyz": [0.1, 0.00188791, -0.01], "rpy": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0], "lim": [-1.972222054, 1.972222054]}, {"name": "left_wrist_pitch_joint", "xyz": [0.038, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 1.0, 0.0], "lim": [-1.614429558, 1.614429558]}, {"name": "left_wrist_yaw_joint", "xyz": [0.046, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "lim": [-1.614429558, 1.614429558]}], "right": [{"name": "right_shoulder_pitch_joint", "xyz": [0.0039563, -0.10021, 0.24778], "rpy": [-0.27931, 5.4949e-05, 0.00019159], "axis": [0.0, 1.0, 0.0], "lim": [-3.0892, 2.6704]}, {"name": "right_shoulder_roll_joint", "xyz": [0.0, -0.038, -0.013831], "rpy": [0.27925, 0.0, 0.0], "axis": [1.0, 0.0, 0.0], "lim": [-2.2515, 1.5882]}, {"name": "right_shoulder_yaw_joint", "xyz": [0.0, -0.00624, -0.1032], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "lim": [-2.618, 2.618]}, {"name": "right_elbow_joint", "xyz": [0.015783, 0.0, -0.080518], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 1.0, 0.0], "lim": [-1.0472, 2.0944]}, {"name": "right_wrist_roll_joint", "xyz": [0.1, -0.00188791, -0.01], "rpy": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0], "lim": [-1.972222054, 1.972222054]}, {"name": "right_wrist_pitch_joint", "xyz": [0.038, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 1.0, 0.0], "lim": [-1.614429558, 1.614429558]}, {"name": "right_wrist_yaw_joint", "xyz": [0.046, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0], "lim": [-1.614429558, 1.614429558]}]}
WAIST_LIM = {"waist_yaw_joint": [-2.618, 2.618], "waist_roll_joint": [-0.52, 0.52], "waist_pitch_joint": [-0.52, 0.52]}
HAND_LIM = {"left_thumb_metacarpal_joint": [0.0, 1.5184], "left_thumb_proximal_joint": [0.0, 1.0472], "left_index_proximal_joint": [0.0, 1.4661], "left_middle_proximal_joint": [0.0, 1.4661], "left_ring_proximal_joint": [0.0, 1.4661], "left_pinky_proximal_joint": [0.0, 1.4661], "right_thumb_metacarpal_joint": [0.0, 1.5184], "right_thumb_proximal_joint": [0.0, 1.0472], "right_index_proximal_joint": [0.0, 1.4661], "right_middle_proximal_joint": [0.0, 1.4661], "right_ring_proximal_joint": [0.0, 1.4661], "right_pinky_proximal_joint": [0.0, 1.4661]}
DEFAULT_PELVIS_HEIGHT = 0.72          # AGILE unitree_g1.DEFAULT_PELVIS_HEIGHT
CMD_LIMITS = {"vx": (-0.5, 1.0), "vy": (-0.4, 0.4), "wz": (-0.8, 0.8), "h": (0.45, DEFAULT_PELVIS_HEIGHT)}  # inside AGILE's training ranges
FINGERS = (("index", "Index"), ("middle", "Middle"), ("ring", "Ring"), ("pinky", "Little"))


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis(ax, q):
    ax = np.asarray(ax, float); ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


_FIXED = {s: [(np.array(j["xyz"]), _rpy(*j["rpy"]), np.array(j["axis"], float)) for j in ARM[s]] for s in ARM}
LIMITS = {s: np.array([j["lim"] for j in ARM[s]]) for s in ARM}
JOINTS = {s: [j["name"] for j in ARM[s]] for s in ARM}


def arm_fk(side, q, with_axes=False):
    """Frames in the torso_link frame: shoulder, elbow, wrist positions and the hand (wrist_yaw_link) rotation.
    with_axes also returns every joint's origin and axis (for the geometric Jacobian)."""
    R = np.eye(3); p = np.zeros(3); pts = []; axes = []
    for i, (xyz, Rf, ax) in enumerate(_FIXED[side]):
        p = p + R @ xyz
        Rj = R @ Rf
        pts.append(p.copy()); axes.append(Rj @ (ax / np.linalg.norm(ax)))
        R = Rj @ _axis(ax, q[i])
    out = (pts[0], pts[3], pts[5], R)
    return out + (pts, axes) if with_axes else out


def _log(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2)); a = math.acos(c)
    if a < 1e-6:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (a / (2 * math.sin(a))) if a < math.pi - 1e-4 else v * 0.5 * math.pi / max(np.linalg.norm(v) / 2, 1e-6)


_Z = {s: arm_fk(s, np.zeros(7)) for s in ARM}
ARM_LEN = {s: (np.linalg.norm(_Z[s][1] - _Z[s][0]), np.linalg.norm(_Z[s][2] - _Z[s][1])) for s in ARM}


def solve_arm(side, elbow_t, wrist_t, hand_R_t=None, q0=None, iters=10, w_elbow=1.0, w_wrist=2.0, w_rot=0.25, reg=0.001):
    """Damped least squares with the geometric Jacobian (one FK per iteration, ~1 ms per arm)."""
    q = np.zeros(7) if q0 is None else np.array(q0, float)
    qp = q.copy(); lo, hi = LIMITS[side][:, 0], LIMITS[side][:, 1]
    sr = math.sqrt(reg)
    for _ in range(iters):
        _, e, w, Rh, pts, axes = arm_fk(side, q, with_axes=True)
        rows, Js = [w_elbow * (e - elbow_t), w_wrist * (w - wrist_t)], []
        Je = np.zeros((3, 7)); Jw = np.zeros((3, 7))
        for i in range(7):
            if i < 3:
                Je[:, i] = np.cross(axes[i], e - pts[i])
            if i < 5:
                Jw[:, i] = np.cross(axes[i], w - pts[i])
        Js = [w_elbow * Je, w_wrist * Jw]
        if hand_R_t is not None:
            rows.append(-w_rot * _log(hand_R_t @ Rh.T)); Js.append(w_rot * np.column_stack(axes))
        rows.append(sr * (q - qp)); Js.append(sr * np.eye(7))
        r, Jm = np.concatenate(rows), np.vstack(Js)
        step = np.linalg.solve(Jm.T @ Jm + 1e-4 * np.eye(7), -Jm.T @ r)
        q = np.clip(q + np.clip(step, -0.5, 0.5), lo, hi)
        if np.linalg.norm(step) < 1e-5:
            break
    _, e, w, _ = arm_fk(side, q)
    return q, {"elbow_err": float(np.linalg.norm(e - elbow_t)), "wrist_err": float(np.linalg.norm(w - wrist_t))}


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _heading_frame(left, right):
    y = np.array(left, float) - np.array(right, float); y[2] = 0.0; y = _unit(y)
    z = np.array([0.0, 0.0, 1.0]); x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _angle(a, b):
    a, b = _unit(a), _unit(b)
    return math.acos(max(-1.0, min(1.0, float(a @ b))))


def _yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


def _zxy(R):
    """R = Rz(yaw) Rx(roll) Ry(pitch): the G1 waist chain order."""
    roll = math.asin(max(-1.0, min(1.0, R[2, 1])))
    return math.atan2(-R[0, 1], R[1, 1]), roll, math.atan2(-R[2, 0], R[2, 2])


def finger_flex(j, side, F):
    """Sum of bend angles along one finger from joint positions (radians)."""
    names = [f"{side}{F}{s}" for s in ("Metacarpal", "Proximal", "Medial", "Distal", "Tip")]
    pts = [j[n][0] for n in names if n in j]
    if len(pts) < 3:
        pts = [j[f"{side}Hand"][0]] + pts if f"{side}Hand" in j else pts
    if len(pts) < 3:
        return None
    segs = [pts[i + 1] - pts[i] for i in range(len(pts) - 1)]
    return sum(_angle(segs[i], segs[i + 1]) for i in range(len(segs) - 1))


def thumb_metrics(j, side):
    hand = j.get(f"{side}Hand"); t = [j.get(f"{side}Thumb{s}") for s in ("Proximal", "Medial", "Distal", "Tip")]
    t = [x[0] for x in t if x is not None]
    idx, lit = j.get(f"{side}IndexProximal"), j.get(f"{side}LittleProximal")
    if hand is None or len(t) < 3 or idx is None or lit is None:
        return None, None
    segs = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    flex = sum(_angle(segs[i], segs[i + 1]) for i in range(len(segs) - 1))
    a, b = idx[0] - hand[0], lit[0] - hand[0]
    n = _unit(np.cross(a, b) if side == "left" else np.cross(b, a))   # palm normal (towards the palm side)
    opp = math.asin(max(-1.0, min(1.0, float(_unit(segs[0]) @ n))))
    return flex, opp


class G1Retarget:
    """Stateful mapping. calibrate(joints) once in the calibration pose, then update(joints) every control step."""

    def __init__(self, walk_deadzone=0.12, walk_gain=2.0, turn_deadzone=0.35, turn_gain=1.2, smooth=0.35, max_speed=4.0):
        self.cal = None
        self.q_arm = {"left": np.zeros(7), "right": np.zeros(7)}
        self.out = {}
        self.walk_deadzone, self.walk_gain, self.turn_deadzone, self.turn_gain = walk_deadzone, walk_gain, turn_deadzone, turn_gain
        self.smooth, self.max_speed = smooth, max_speed

    @staticmethod
    def needed(j):
        return [n for n in ("hip", "chest", "leftUpperArm", "leftLowerArm", "leftHand", "rightUpperArm", "rightLowerArm", "rightHand") if n not in j]

    def calibrate(self, j):
        miss = self.needed(j)
        if miss:
            raise ValueError(f"missing Rokoko joints: {miss}")
        T = _heading_frame(j["leftUpperArm"][0], j["rightUpperArm"][0])
        cal = {"T": T, "chest": j["chest"][1], "hip": j["hip"][1], "hip_pos": j["hip"][0].copy(),
               "hand": {s: j[f"{s}Hand"][1] for s in ("left", "right")}, "flex": {}, "thumb": {}}
        for s in ("left", "right"):
            for f, F in FINGERS:
                v = finger_flex(j, s, F)
                if v is not None:
                    cal["flex"][(s, f)] = v
            cal["thumb"][s] = thumb_metrics(j, s)
        self.cal = cal
        self.q_arm = {"left": np.zeros(7), "right": np.zeros(7)}
        self.out = {}
        return {"ok": True, "hip_height": float(cal["hip_pos"][2]), "fingers": len(cal["flex"])}

    def update(self, j, dt=0.02, gloves=(True, True)):
        if self.cal is None:
            raise RuntimeError("calibrate first")
        c = self.cal
        dC = j["chest"][1] @ c["chest"].T if "chest" in j else np.eye(3)
        dH = j["hip"][1] @ c["hip"].T if "hip" in j else np.eye(3)
        T, P = dC @ c["T"], dH @ c["T"]
        tgt = {}
        # waist
        yw, rl, pt = _zxy(P.T @ T)
        for name, v in zip(("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"), (yw, rl, pt)):
            lo, hi = WAIST_LIM[name]; tgt[name] = float(np.clip(v, lo, hi))
        info = {}
        # arms: directions in the current torso frame, scaled to the G1 arm
        for s in ("left", "right"):
            S, E, W = (j[f"{s}{b}"][0] for b in ("UpperArm", "LowerArm", "Hand"))
            u, f = T.T @ _unit(E - S), T.T @ _unit(W - E)
            s0 = _Z[s][0]; L1, L2 = ARM_LEN[s]
            e_t = s0 + L1 * u; w_t = e_t + L2 * f
            dR = j[f"{s}Hand"][1] @ c["hand"][s].T
            R_t = T.T @ dR @ c["T"] @ _Z[s][3]
            q, err = solve_arm(s, e_t, w_t, R_t, self.q_arm[s])
            self.q_arm[s] = q; info[s] = err
            tgt.update(dict(zip(JOINTS[s], q.tolist())))
            # hands
            if gloves[0 if s == "left" else 1]:
                for fn, F in FINGERS:
                    v = finger_flex(j, s, F)
                    if v is not None and (s, fn) in c["flex"]:
                        k = float(np.clip((v - c["flex"][(s, fn)]) / 2.4, 0.0, 1.0))
                        name = f"{s}_{fn}_proximal_joint"; lo, hi = HAND_LIM[name]
                        tgt[name] = lo + k * (hi - lo)
                flex, opp = thumb_metrics(j, s)
                f0, o0 = c["thumb"][s]
                if flex is not None and f0 is not None:
                    name = f"{s}_thumb_proximal_joint"; lo, hi = HAND_LIM[name]
                    tgt[name] = lo + float(np.clip((flex - f0) / 1.4, 0, 1)) * (hi - lo)
                    name = f"{s}_thumb_metacarpal_joint"; lo, hi = HAND_LIM[name]
                    tgt[name] = lo + float(np.clip((opp - o0) / 0.9, 0, 1)) * (hi - lo)
        # smoothing + speed limit
        for k, v in tgt.items():
            prev = self.out.get(k, v)
            v = prev + (1 - self.smooth) * (v - prev)
            v = prev + float(np.clip(v - prev, -self.max_speed * dt, self.max_speed * dt))
            self.out[k] = v
        # walk commands from the hips (in the calibration heading frame)
        d = c["T"].T @ (j["hip"][0] - c["hip_pos"])
        def dz(x, zone, gain, lim):
            if abs(x) <= zone:
                return 0.0
            return float(np.clip(gain * (x - math.copysign(zone, x)), *lim))
        dyaw = math.atan2(math.sin(_yaw(P) - _yaw(c["T"])), math.cos(_yaw(P) - _yaw(c["T"])))
        h = DEFAULT_PELVIS_HEIGHT * float(j["hip"][0][2] / max(c["hip_pos"][2], 1e-3))
        cmd = {"vx": dz(d[0], self.walk_deadzone, self.walk_gain, CMD_LIMITS["vx"]), "vy": dz(d[1], self.walk_deadzone, self.walk_gain, CMD_LIMITS["vy"]),
               "wz": dz(dyaw, self.turn_deadzone, self.turn_gain, CMD_LIMITS["wz"]), "h": float(np.clip(h, *CMD_LIMITS["h"]))}
        return {"targets": dict(self.out), "cmd": cmd, "arm_err": info}
