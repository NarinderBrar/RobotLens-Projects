# Kitchen kit

Scene generators for kitchen test worlds. Every generator runs only when you
press Generate (or an agent calls `generator.run`); its output is saved with the
project, so reopening shows the same scene.

| Generator | What it builds |
| --- | --- |
| `kitchen` | Room with cabinet runs, counters, appliances, and a walkway in a galley, L-shaped, or island layout |
| `cabinet` | One cabinet with its front style and optional counter |
| `appliance` | One appliance placed by position and yaw |

Enabling the kit adds only `kitchen`. `cabinet` and `appliance` are parts
(`"part": true`): add one with `extension.add_instance` when you want it on its own.

Presets in `presets/kitchen/` (`small_galley`, `l_shaped`, `family_island`) were used
for the atlas worlds `01`–`03`; `worlds/atlas/manifest.json` records their seeds.

Rebuild after editing `src/` with `robotlens-extension-harness build <this folder>`,
then press Reload in Window → Extensions.
