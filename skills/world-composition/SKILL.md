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
