# Warehouse kit

Scene generators for warehouse test worlds. Every generator runs only when you
press Generate (or an agent calls `generator.run`); its output is saved with the
project, so reopening shows the same scene.

| Generator | What it builds |
| --- | --- |
| `warehouse` | Room, rows of racks with boxes on the shelves, aisle regions, and a doorway. The robot's origin stays inside an aisle |
| `rack` | One rack (`wire_rack`, `pallet_rack`, `cubby`, or `custom`) with boxes |
| `pallet` | Euro or US pallets with a simple collision box and an optional load |
| `bin` | An open bin with items inside |
| `box` | A grid stack of cartons |
| `conveyor` | A belt conveyor on legs |
| `safety_barrier` | A run of barrier posts and rails |

Enabling the kit adds only `warehouse`. The others are parts (`"part": true`):
add one from the Add list in the kit's panel or with `extension.add_instance` when you
want it on its own.

Shared code lives in `src/kit.hpp`; the helpers come from
`sdk/include/robotlens/procgen.hpp`. Change settings first; edit code only for new
behavior. After a change, build, press Reload, then Generate, then Validate.
