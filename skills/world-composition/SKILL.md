---
name: world-composition
description: Compose rooms (office, warehouse, kitchen, lab) from real assets with sensible placement - chairs at desks, screens facing people, clear doorways, robot on measured free floor or a desk.
---

# Composing worlds like a person would

Call `world_compose(spec, name, apply=true)`. Spec (meters):

```json
{"room": {"width": 10, "depth": 8, "height": 3,
          "doors": [{"wall": "south", "offset": 1.0, "width": 1.0}]},
 "items": [{"kind": "desk", "count": 4, "with": ["chair", "monitor"]},
           {"kind": "table", "count": 1, "chairs": 6},
           {"kind": "shelf", "count": 3},
           {"kind": "plant", "count": 2}],
 "robot": {"name": "carter", "footprint": [0.7, 0.5], "on": "floor", "search": ["carter"]}}
```

Kinds: desk, chair, monitor, table, round_table, shelf, rack, cabinet, sofa, plant, counter, fridge, workbench, pallet, box, conveyor.
`"on": "desk"` puts a tabletop robot (arm) on a free desk surface instead.

Rules the solver enforces: wall pieces go against walls; groups keep 0.9 m aisles; 1.5 m in front of every door stays clear;
chairs face their desk/table; monitors sit at the back of the desk facing the chair; the robot spawns where floor clearance is largest.
Check `checks.ok` and `unplaced` before building. Assets are searched in the Isaac library by keyword and scaled to the footprint;
when none match, a labeled box with a collider stands in (reported as `fallback_boxes`) - tell the user.

From a photo: describe the room's dimensions, door positions and furniture groups from the image into this spec, state your
estimates, then compose. After building: `isaac_physics_check` and `isaac_capture` from a high oblique view.

## Real assets from SimReady Central
- `simwright simready search "<object>" --max_size 0.5` returns simulation-ready USD (physics, materials, metric scale) with its real size in meters; prefer it over hand-made boxes and over unscaled art assets.
- `simwright simready place <url> --on /World/Kitchen/table --x 3.1 --y 2.2 --yaw 90` downloads the whole package at its pinned revision, fixes units and up axis and rests it on the support's top (or the floor). It reports `inside_support_xy`; false means it overhangs: move it.
- Keep SimReady physics as authored (`--physics auto`); add colliders or a rigid body only for assets without them.
- Note each asset's license (catalog: `simwright simready list`); NVIDIA's own set is CC-BY-4.0 and needs attribution.
## Placement checks that caught real mistakes
- Items on a table: every bbox inside the tabletop in x and y, no two footprints overlapping (5 mm tolerance), bottoms at top + 2-3 mm.
- Appliances: no intersections with counters; check model heights (a 0.4 m "dishwasher" is a broken asset, drop it).
- Long thin utensils point toward the diner; verify the long axis after rotation from the bbox, not from the yaw you asked for.

## Placement math (0.3): set things down, don't compute heights by hand
- `isaac_place` / `simready_place` lower an object onto the real surface under its footprint: they ray cast through the support's triangles. A bounding-box top is wrong whenever the support has something taller on it (lamp, monitor, back panel, cable) and made objects float.
- Give `on` for tables, shelves, pedestals and mounts; without it the object lands on the floor (highest surface below 0.5 m). Pass `z` only when the height is specified.
- Positions are solved in the parent's frame, so placing under a moved, rotated or scaled group (`/World/WarehouseScene/...`) lands at the requested world (x, y). Never write a world position into a child's local translate yourself.
- Read the result: `bottom_m`, `rest_on_m`, `supported_fraction` (< 0.5 means it hangs off an edge) and `warning`.
- After building, run `isaac_placement_audit` on the scope: every object should be `resting` (gap within 1 cm).
