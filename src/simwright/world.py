"""Compose whole rooms the way a person would: chairs at their desks facing the screen, shelves against walls,
doorways kept clear, and the robot dropped on measured free floor (or a free desk surface).

solve(spec) -> layout with placements (meters, z-up, yaw in degrees = direction the item's front faces).
"""
from __future__ import annotations
import math, random

# footprint w x d x h in meters, plus how to find a real asset in the Isaac library
CATALOG = {
    "desk": {"size": (1.4, 0.7, 0.75), "search": ["desk"], "wall": True},
    "chair": {"size": (0.55, 0.55, 0.9), "search": ["chair"]},
    "monitor": {"size": (0.6, 0.2, 0.45), "search": ["monitor"], "surface": True},
    "table": {"size": (1.8, 0.9, 0.75), "search": ["table"]},
    "round_table": {"size": (1.2, 1.2, 0.75), "search": ["table"]},
    "shelf": {"size": (1.0, 0.4, 2.0), "search": ["shelf"], "wall": True},
    "rack": {"size": (2.6, 1.0, 3.0), "search": ["rack"], "wall": True},
    "cabinet": {"size": (0.8, 0.5, 1.1), "search": ["cabinet"], "wall": True},
    "sofa": {"size": (2.0, 0.9, 0.85), "search": ["sofa"], "wall": True},
    "plant": {"size": (0.45, 0.45, 1.2), "search": ["plant"], "corner": True},
    "counter": {"size": (2.4, 0.65, 0.92), "search": ["counter"], "wall": True},
    "fridge": {"size": (0.8, 0.75, 1.8), "search": ["fridge"], "wall": True},
    "workbench": {"size": (1.8, 0.8, 0.9), "search": ["table"], "wall": True},
    "pallet": {"size": (1.2, 1.0, 0.15), "search": ["pallet"]},
    "box": {"size": (0.6, 0.4, 0.4), "search": ["box"]},
    "conveyor": {"size": (3.0, 0.8, 0.9), "search": ["conveyor"]},
}
AISLE = 0.9          # walkable clearance between furniture groups
DOOR_KEEPOUT = 1.5   # clear floor in front of a doorway
WALL_GAP = 0.05


class Rect:
    def __init__(self, x0, y0, x1, y1, tag=""):
        self.x0, self.y0, self.x1, self.y1, self.tag = min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), tag

    def inflate(self, m):
        return Rect(self.x0 - m, self.y0 - m, self.x1 + m, self.y1 + m, self.tag)

    def hits(self, o, eps=1e-6):
        return self.x0 < o.x1 - eps and o.x0 < self.x1 - eps and self.y0 < o.y1 - eps and o.y0 < self.y1 - eps

    def inside(self, W, D):
        return self.x0 >= -1e-6 and self.y0 >= -1e-6 and self.x1 <= W + 1e-6 and self.y1 <= D + 1e-6

    def dist(self, x, y):
        dx = max(self.x0 - x, 0, x - self.x1)
        dy = max(self.y0 - y, 0, y - self.y1)
        return math.hypot(dx, dy)


def _rect(cx, cy, w, d, yaw):
    """Axis-aligned footprint. w = width across the item, d = depth along the direction it faces (yaw)."""
    ex, ey = (w, d) if int(round(yaw)) % 180 == 90 else (d, w)
    return Rect(cx - ex / 2, cy - ey / 2, cx + ex / 2, cy + ey / 2)


def _front(yaw):
    a = math.radians(yaw)
    return round(math.cos(a), 6), round(math.sin(a), 6)


class Solver:
    def __init__(self, spec):
        r = spec["room"]
        self.W, self.D, self.H = float(r["width"]), float(r["depth"]), float(r.get("height", 3.0))
        self.spec = spec
        self.blocks: list[Rect] = []      # furniture footprints
        self.keepout: list[Rect] = []     # door clearances
        self.placements, self.unplaced = [], []
        self.n = 0
        for door in r.get("doors", []):
            self.keepout.append(self._door_rect(door))

    def _door_rect(self, door):
        w, off, wall = float(door.get("width", 0.9)), float(door.get("offset", 1.0)), door.get("wall", "south")
        k = DOOR_KEEPOUT
        if wall == "south":
            return Rect(off - 0.2, 0, off + w + 0.2, k, "door")
        if wall == "north":
            return Rect(off - 0.2, self.D - k, off + w + 0.2, self.D, "door")
        if wall == "west":
            return Rect(0, off - 0.2, k, off + w + 0.2, "door")
        return Rect(self.W - k, off - 0.2, self.W, off + w + 0.2, "door")

    def free(self, r: Rect, clearance=0.0, ignore_keepout=False):
        if not r.inside(self.W, self.D):
            return False
        if any(r.inflate(clearance).hits(b) for b in self.blocks):
            return False
        return ignore_keepout or not any(r.hits(k) for k in self.keepout)

    def add(self, kind, cx, cy, yaw, z=0.0, size=None, parent=None, extra=None):
        cat = CATALOG.get(kind, {"size": (0.6, 0.6, 0.8), "search": [kind]})
        w, d, h = size or cat["size"]
        self.n += 1
        pid = f"{kind}_{self.n:02d}"
        p = {"id": pid, "kind": kind, "x": round(cx, 3), "y": round(cy, 3), "z": round(z, 3), "yaw": yaw % 360,
             "w": w, "d": d, "h": h, "search": cat["search"], "asset": "", **({"on": parent} if parent else {}), **(extra or {})}
        self.placements.append(p)
        if z == 0.0:
            self.blocks.append(_rect(cx, cy, w, d, yaw).inflate(0.0))
            self.blocks[-1].tag = pid
        return p

    # ----- groups -----
    def group_footprint(self, item):
        kind = item["kind"]
        w, d, _ = CATALOG.get(kind, {"size": (0.6, 0.6, 0.8)})["size"]
        if kind in ("desk", "workbench") and "chair" in item.get("with", ["chair"]):
            return w, d + 0.85  # chair + legroom in front
        if kind == "round_table":
            return w + 1.2, d + 1.2
        if kind == "table":
            return self._table_len(item, w), d + 1.2
        return w, d

    @staticmethod
    def _table_len(item, w):
        """Long tables grow so every chair gets 0.65 m of edge."""
        per_side = (int(item.get("chairs", 4)) + 1) // 2
        return max(w, per_side * 0.65)

    def place_group(self, item, cx, cy, yaw):
        """yaw = direction the group's 'front' faces (desk: toward the person sitting at it)."""
        kind = item["kind"]
        w, d, h = CATALOG.get(kind, {"size": (0.6, 0.6, 0.8)})["size"]
        fx, fy = _front(yaw)
        gw, gd = self.group_footprint(item)
        # main piece sits at the back of the group footprint
        back = (gd - d) / 2
        mx, my = cx - fx * back, cy - fy * back
        if kind == "table":
            w = self._table_len(item, w)
            main = self.add(kind, mx, my, yaw, size=(w, d, h))
        else:
            main = self.add(kind, mx, my, yaw)
        if kind in ("desk", "workbench"):
            if "monitor" in item.get("with", []):
                bx, by = mx - fx * (d / 2 - 0.15), my - fy * (d / 2 - 0.15)
                self.add("monitor", bx, by, yaw, z=h, parent=main["id"])  # screen faces the sitter
            if "chair" in item.get("with", ["chair"]):
                cxx, cyy = mx + fx * (d / 2 + 0.4), my + fy * (d / 2 + 0.4)
                self.add("chair", cxx, cyy, (yaw + 180) % 360, parent=main["id"])  # chair faces the desk
        if kind in ("table", "round_table"):
            chairs = int(item.get("chairs", 4))
            pos = []
            if kind == "round_table":
                rr = w / 2 + 0.35
                for i in range(chairs):
                    a = 2 * math.pi * i / chairs
                    pos.append((mx + rr * math.cos(a), my + rr * math.sin(a), math.degrees(a) + 180))
            else:
                per_side = (chairs + 1) // 2
                long_ax = (fy, -fx) if True else None
                for side in (1, -1):
                    n = per_side if side == 1 else chairs - per_side
                    for i in range(n):
                        t = (i + 0.5) / n - 0.5
                        ox, oy = long_ax[0] * t * w, long_ax[1] * t * w
                        sx, sy = fx * side * (d / 2 + 0.38), fy * side * (d / 2 + 0.38)
                        cyaw = (yaw + (180 if side == 1 else 0)) % 360
                        pos.append((mx + ox + sx, my + oy + sy, cyaw))
            for (x, y, a) in pos:
                self.add("chair", x, y, a, parent=main["id"])
        return main

    # ----- strategies -----
    def along_walls(self, item):
        w, d = self.group_footprint(item)
        step = 0.1
        for wall, yaw in (("north", 270), ("east", 180), ("west", 0), ("south", 90)):
            span = self.W if wall in ("north", "south") else self.D
            s = WALL_GAP + w / 2
            while s <= span - w / 2 - WALL_GAP:
                if wall == "north":
                    cx, cy = s, self.D - WALL_GAP - d / 2
                elif wall == "south":
                    cx, cy = s, WALL_GAP + d / 2
                elif wall == "west":
                    cx, cy = WALL_GAP + d / 2, s
                else:
                    cx, cy = self.W - WALL_GAP - d / 2, s
                r = _rect(cx, cy, w, d, yaw)
                if self.free(r, clearance=0.05) and self._aisle_ok(r):
                    return self.place_group(item, cx, cy, yaw)
                s += step
        return None

    def _aisle_ok(self, r):
        """Keep a walkable aisle: the group may touch walls but needs AISLE clearance to other groups' fronts."""
        return all(not r.inflate(AISLE / 2).hits(b.inflate(AISLE / 2)) or self._touches_wall(b) and self._touches_wall(r) and not r.inflate(0.05).hits(b) for b in self.blocks)

    def _touches_wall(self, r):
        return r.x0 < 0.2 or r.y0 < 0.2 or r.x1 > self.W - 0.2 or r.y1 > self.D - 0.2

    def interior(self, item):
        w, d = self.group_footprint(item)
        best = None
        for yaw in (90, 0):
            y = AISLE + d / 2
            while y <= self.D - AISLE - d / 2 + 1e-6:
                x = AISLE + w / 2
                while x <= self.W - AISLE - w / 2 + 1e-6:
                    r = _rect(x, y, w, d, yaw)
                    if self.free(r, clearance=AISLE):
                        score = -abs(x - self.W / 2) * 0.1 - abs(y - self.D / 2) * 0.1
                        if best is None or score > best[0]:
                            best = (score, x, y, yaw)
                    x += 0.2
                y += 0.2
            if best:
                break
        if not best:
            return None
        return self.place_group(item, best[1], best[2], best[3])

    def corner(self, item):
        w, d, _ = CATALOG.get(item["kind"], {"size": (0.5, 0.5, 1)})["size"]
        for cx, cy in ((w / 2 + 0.1, d / 2 + 0.1), (self.W - w / 2 - 0.1, d / 2 + 0.1),
                       (w / 2 + 0.1, self.D - d / 2 - 0.1), (self.W - w / 2 - 0.1, self.D - d / 2 - 0.1)):
            r = _rect(cx, cy, w, d, 0)
            if self.free(r, clearance=0.1):
                return self.add(item["kind"], cx, cy, 0)
        return self.along_walls(item)

    # ----- robot -----
    def place_robot(self, robot):
        fw, fd = robot.get("footprint", [0.6, 0.5])
        radius = math.hypot(fw, fd) / 2
        if robot.get("on") in ("desk", "table", "workbench"):
            hosts = [p for p in self.placements if p["kind"] in ("desk", "table", "workbench", "counter") and "on" not in p]
            for host in hosts:
                used = [q for q in self.placements if q.get("on") == host["id"] and q["z"] > 0]
                fx, fy = _front(host["yaw"])
                # front half of the surface, offset sideways away from the monitor
                lx, ly = -fy, fx
                for side in (0.3, -0.3, 0.0):
                    x = host["x"] + fx * 0.1 + lx * side * host["w"]
                    y = host["y"] + fy * 0.1 + ly * side * host["w"]
                    r = Rect(x - fw / 2, y - fd / 2, x + fw / 2, y + fd / 2)
                    surf = _rect(host["x"], host["y"], host["w"], host["d"], host["yaw"])
                    inside = r.x0 >= surf.x0 and r.x1 <= surf.x1 and r.y0 >= surf.y0 and r.y1 <= surf.y1
                    clash = any(_rect(q["x"], q["y"], q["w"], q["d"], q["yaw"]).hits(r) for q in used)
                    if inside and not clash:
                        return self.add("robot", x, y, host["yaw"], z=host["h"], size=(fw, fd, robot.get("height", 0.3)),
                                        parent=host["id"], extra={"asset": robot.get("usd", ""), "search": robot.get("search", [robot.get("name", "robot")])})
            self.unplaced.append({"kind": "robot", "reason": "no desk/table surface with enough free area"})
            return None
        best = None
        y = radius + 0.1
        while y < self.D - radius - 0.1:
            x = radius + 0.1
            while x < self.W - radius - 0.1:
                walls = min(x, y, self.W - x, self.D - y)
                obs = min([b.dist(x, y) for b in self.blocks] + [walls])
                in_door = any(k.dist(x, y) < radius for k in self.keepout)
                if obs > radius + 0.3 and not in_door and (best is None or obs > best[0]):
                    best = (obs, x, y)
                x += 0.1
            y += 0.1
        if not best:
            self.unplaced.append({"kind": "robot", "reason": "no free floor patch large enough"})
            return None
        return self.add("robot", best[1], best[2], 0, size=(fw, fd, robot.get("height", 0.4)),
                        extra={"clearance_m": round(best[0], 2), "asset": robot.get("usd", ""),
                               "search": robot.get("search", [robot.get("name", "robot")])})

    def solve(self):
        items = []
        for it in self.spec.get("items", []):
            for _ in range(int(it.get("count", 1))):
                items.append(dict(it))
        order = lambda it: (0 if CATALOG.get(it["kind"], {}).get("wall") and it.get("against_wall", True) else
                            2 if CATALOG.get(it["kind"], {}).get("corner") else 1)
        for it in sorted(items, key=order):
            cat = CATALOG.get(it["kind"], {})
            placed = None
            if it.get("against_wall", cat.get("wall")):
                placed = self.along_walls(it) or self.interior(it)
            elif cat.get("corner"):
                placed = self.corner(it)
            else:
                placed = self.interior(it) or self.along_walls(it)
            if not placed:
                self.unplaced.append({"kind": it["kind"], "reason": "no space that keeps aisles and doorways clear"})
        if self.spec.get("robot"):
            self.place_robot(self.spec["robot"])
        return {"room": {"width": self.W, "depth": self.D, "height": self.H, "doors": self.spec["room"].get("doors", [])},
                "placements": self.placements, "unplaced": self.unplaced, "checks": self.checks()}

    def checks(self):
        floor = [p for p in self.placements if p["z"] == 0.0]
        overlaps = []
        for i, a in enumerate(floor):
            ra = _rect(a["x"], a["y"], a["w"], a["d"], a["yaw"])
            if not ra.inside(self.W, self.D):
                overlaps.append(f"{a['id']} outside room")
            for b in floor[i + 1:]:
                if ra.hits(_rect(b["x"], b["y"], b["w"], b["d"], b["yaw"]), eps=0.01):
                    overlaps.append(f"{a['id']} overlaps {b['id']}")
            if a["kind"] != "robot" and any(ra.hits(k) for k in self.keepout):
                overlaps.append(f"{a['id']} blocks a doorway")
        used = sum(p["w"] * p["d"] for p in floor)
        return {"ok": not overlaps, "problems": overlaps, "floor_used_pct": round(100 * used / (self.W * self.D), 1)}


def validate_spec(spec: dict) -> list[str]:
    out = []
    room = spec.get("room") if isinstance(spec, dict) else None
    if not isinstance(room, dict) or not all(isinstance(room.get(k), (int, float)) and room.get(k) > 0 for k in ("width", "depth")):
        out.append("room needs numeric width and depth in meters")
    for i, it in enumerate(spec.get("items", []) if isinstance(spec, dict) else []):
        if it.get("kind") not in CATALOG:
            out.append(f"items[{i}].kind '{it.get('kind')}' unknown; use one of {', '.join(CATALOG)}")
    for d in (room or {}).get("doors", []):
        if d.get("wall") not in ("south", "north", "east", "west"):
            out.append("door wall must be south, north, east or west")
    return out


def solve(spec: dict) -> dict:
    return Solver(spec).solve()


def ascii_map(layout: dict, cell: float = 0.25) -> str:
    W, D = layout["room"]["width"], layout["room"]["depth"]
    cols, rows = int(W / cell), int(D / cell)
    grid = [["." for _ in range(cols)] for _ in range(rows)]
    sym = {"desk": "D", "chair": "c", "table": "T", "round_table": "T", "shelf": "S", "rack": "R", "plant": "p",
           "sofa": "F", "cabinet": "K", "robot": "@", "monitor": "m", "counter": "C", "fridge": "Z", "workbench": "W"}
    for p in sorted(layout["placements"], key=lambda p: p["z"]):
        r = _rect(p["x"], p["y"], p["w"], p["d"], p["yaw"])
        for yy in range(rows):
            for xx in range(cols):
                x, y = (xx + 0.5) * cell, (yy + 0.5) * cell
                if r.x0 <= x <= r.x1 and r.y0 <= y <= r.y1:
                    grid[yy][xx] = sym.get(p["kind"], "#")
    return "\n".join("".join(row) for row in reversed(grid))
