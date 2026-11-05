"""Rokoko Studio live data (Smartsuit Pro + Smartgloves) for Simwright.

Rokoko Studio > Livestreaming > Custom: protocol UDP, data format "JSON v3" or "JSON v4" (the LZ4 variants work
when the `lz4` package is importable), forward IP 127.0.0.1, port 14043. One complete JSON frame per datagram.

Everything returned here is converted from Rokoko's left-handed, Y-up, Z-forward frame (metres, quaternions xyzw,
world space) into the robot convention: right-handed, Z-up, X-forward, Y-left.
"""
from __future__ import annotations
import json, math, socket, time
import numpy as np

DEFAULT_PORT = 14043
COMMAND_PORT = 14053
LZ4_MAGIC = b"\x04\x22\x4d\x18"

# Rokoko (x right, y up, z forward; left-handed)  ->  robot (x forward, y left, z up; right-handed)
M = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

# JSON v4 "Newton" joint names -> the v3 humanoid names used throughout Simwright
_V4 = {"Hips": "hip", "Spine1": "spine", "Spine4": "chest", "Neck": "neck", "Head": "head",
       "LeftShoulder": "leftShoulder", "LeftArm": "leftUpperArm", "LeftForeArm": "leftLowerArm", "LeftHand": "leftHand",
       "RightShoulder": "rightShoulder", "RightArm": "rightUpperArm", "RightForeArm": "rightLowerArm", "RightHand": "rightHand",
       "LeftThigh": "leftUpLeg", "LeftShin": "leftLeg", "LeftFoot": "leftFoot", "LeftToe": "leftToe",
       "RightThigh": "rightUpLeg", "RightShin": "rightLeg", "RightFoot": "rightFoot", "RightToe": "rightToe"}
_FINGERS = {1: "Thumb", 2: "Index", 3: "Middle", 4: "Ring", 5: "Little"}
for _side in ("Left", "Right"):
    for _n, _f in _FINGERS.items():
        if _n == 1:   # Newton thumb: Metacarpal, Proximal, Distal, Tip  ->  v3 thumb Proximal, Medial, Distal, Tip
            for a, b in (("Metacarpal", "Proximal"), ("Proximal", "Medial"), ("Distal", "Distal"), ("Tip", "Tip")):
                _V4[f"{_side}Finger1{a}"] = f"{_side.lower()}Thumb{b}"
        else:
            for a in ("Metacarpal", "Proximal", "Medial", "Distal", "Tip"):
                _V4[f"{_side}Finger{_n}{a}"] = f"{_side.lower()}{_f}{a}"


def quat_to_mat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_robot(pos, quat):
    """Rokoko world pose -> robot world pose (position in m, 3x3 rotation)."""
    p = M @ np.array([pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)], float) if isinstance(pos, dict) else M @ np.asarray(pos, float)
    q = (quat.get("x", 0.0), quat.get("y", 0.0), quat.get("z", 0.0), quat.get("w", 1.0)) if isinstance(quat, dict) else tuple(quat)
    return p, M @ quat_to_mat(q) @ M.T


def decode(datagram: bytes) -> dict:
    if datagram[:4] == LZ4_MAGIC:
        try:
            import lz4.frame
        except ImportError:
            raise ValueError("Rokoko is sending LZ4-compressed JSON; pick plain 'JSON v3' or 'JSON v4' in Studio's Custom streaming, "
                             "or install the lz4 package")
        datagram = lz4.frame.decompress(datagram)
    return json.loads(datagram.decode("utf-8", errors="replace"))


def parse(frame: dict) -> dict:
    """Normalise a v3 or v4 frame: {timestamp, fps, format, actors: [{name, meta, joints: {name: (pos, R)}, dims}]}."""
    scene = frame.get("scene") or {}
    actors = []
    if scene.get("newtons"):
        for n in scene["newtons"]:
            meta = n.get("meta") or {}
            joints = {}
            for j in n.get("joints") or []:
                name = _V4.get(j.get("name"), j.get("name"))
                joints[name] = to_robot(j.get("position") or {}, j.get("rotation") or {})
            actors.append({"name": n.get("name", "actor"), "joints": joints, "dims": n.get("dimensions") or {},
                           "meta": {"body": bool(meta.get("hasBody", True)), "left_hand": bool(meta.get("hasLeftHand", meta.get("hasHands"))),
                                    "right_hand": bool(meta.get("hasRightHand", meta.get("hasHands"))), "face": bool(meta.get("hasFace"))}})
        fmt = "json-v4"
    else:
        for a in scene.get("actors") or []:
            meta = a.get("meta") or {}
            joints = {}
            for name, b in (a.get("body") or {}).items():
                if isinstance(b, dict) and "rotation" in b:
                    joints[name] = to_robot(b.get("position") or {}, b.get("rotation") or {})
            actors.append({"name": a.get("name", "actor"), "joints": joints, "dims": a.get("dimensions") or {},
                           "meta": {"body": bool(meta.get("hasBody", True)), "left_hand": bool(meta.get("hasLeftGlove", meta.get("hasGloves"))),
                                    "right_hand": bool(meta.get("hasRightGlove", meta.get("hasGloves"))), "face": bool(meta.get("hasFace"))}})
        fmt = "json-v3"
    return {"timestamp": scene.get("timestamp"), "fps": frame.get("fps"), "format": fmt, "actors": actors}


class Receiver:
    """Non-blocking UDP receiver. poll() drains the socket and keeps only the newest frame (never blocks)."""

    def __init__(self, port: int = DEFAULT_PORT, host: str = "0.0.0.0", actor: str | None = None):
        self.port, self.actor_name = port, actor
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.latest = None
        self.latest_t = 0.0
        self.frames = 0
        self.errors = 0
        self.last_error = ""
        self._times = []

    def poll(self):
        got = None
        for _ in range(256):
            try:
                data, _addr = self.sock.recvfrom(1 << 17)
            except (BlockingIOError, socket.timeout):
                break
            except OSError as e:
                if getattr(e, "winerror", None) == 10035:
                    break
                self.errors += 1; self.last_error = str(e); break
            got = data
        if got is not None:
            try:
                f = parse(decode(got))
                self.latest, self.latest_t = f, time.time()
                self.frames += 1
                self._times = (self._times + [self.latest_t])[-120:]
            except Exception as e:
                self.errors += 1; self.last_error = str(e)[:300]
        return self.latest

    def actor(self):
        f = self.latest
        if not f or not f["actors"]:
            return None
        if self.actor_name:
            for a in f["actors"]:
                if a["name"] == self.actor_name:
                    return a
        return f["actors"][0]

    def age(self) -> float:
        return time.time() - self.latest_t if self.latest_t else float("inf")

    def fps(self) -> float:
        t = self._times
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 2 and t[-1] > t[0] else 0.0

    def status(self) -> dict:
        a = self.actor()
        return {"port": self.port, "frames": self.frames, "fps": round(self.fps(), 1), "age_s": None if self.age() == float("inf") else round(self.age(), 3),
                "format": (self.latest or {}).get("format"), "actors": [x["name"] for x in (self.latest or {}).get("actors", [])],
                "body": bool(a and a["meta"]["body"]), "left_glove": bool(a and a["meta"]["left_hand"]), "right_glove": bool(a and a["meta"]["right_hand"]),
                "joints": len(a["joints"]) if a else 0, "errors": self.errors, "last_error": self.last_error}

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def command(path: str, api_key: str = "1234", host: str = "127.0.0.1", port: int = COMMAND_PORT, body: dict | None = None, timeout=5.0) -> dict:
    """Rokoko Studio Command API, e.g. command('calibrate'), command('recording/start'). Enable it in Studio settings."""
    import urllib.request
    url = f"http://{host}:{port}/v1/{api_key}/{path.strip('/')}"
    req = urllib.request.Request(url, data=json.dumps(body or {}).encode(), method="POST", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            txt = r.read().decode(errors="replace")
            try:
                return {"ok": True, "response": json.loads(txt)}
            except Exception:
                return {"ok": True, "response": txt}
    except Exception as e:
        return {"ok": False, "error": str(e), "hint": "enable the Command API in Rokoko Studio (Settings) and use the API key it shows"}
