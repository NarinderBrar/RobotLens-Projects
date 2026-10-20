# Room kit

Scene generators for indoor rooms. Every generator runs only when you press
Generate (or an agent calls `generator.run`); its output is saved with the
project, so reopening shows the same scene.

| Generator | What it builds |
| --- | --- |
| `room` | Floor and four walls with an optional doorway and a keep-out region through it |
| `wall` | One wall with an optional doorway |
| `doorway` | A free-standing door frame with a keep-out region |
| `table` | A table with legs and a `top` support surface |
| `obstacle` | Boxes, cylinders, or L-shapes scattered in an area, keeping the origin clear |
| `door` | A hinged door in a frame, with a swing keep-out region |
| `window` | A wall with a framed window opening |

Enabling the kit adds only `room`. The others are parts (`"part": true`): add one
from the Add list in the kit's panel or with `extension.add_instance` when you want it
on its own.

Shared code lives in `src/kit.hpp`; the helpers come from
`sdk/include/robotlens/procgen.hpp`.
