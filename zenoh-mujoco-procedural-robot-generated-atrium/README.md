# Procedural Atlas

RobotLens project: `.robotlens/zenoh-mujoco-procedural-robot-generated-atrium.xml`. The project uses Zenoh and the in-process MuJoCo backend. The active scene is the atrium; the other nine generated worlds are saved in `worlds/atlas/` as standalone MJCF files. Each world has a matching capability report. `worlds/atlas/manifest.json` records the extension preset and seed used for each file.

The scene packages in `extensions/` were created from RobotLens's room, warehouse, and kitchen templates. Their WebAssembly modules were built and validated. Generation and robot authoring were performed through the RobotLens editor commands exposed by MCP.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py         # setup, then test
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The test checks that:

- the project opens
- communication starts and connects
- the simulation starts and reaches Running
- a viewport screenshot is saved to `evidence/quickstart/quickstart.png`

It then stops navigation, the simulation, and communication, in that order. `--keep-running` leaves the simulation and RobotLens running for a look.
The shared code is in `../tools/quickstart.py`.

## Environments

| File | Scene | Objects |
| --- | --- | ---: |
| `01_small_galley.xml` | Small galley kitchen | 295 |
| `02_l_shaped.xml` | L-shaped kitchen | 200 |
| `03_family_island.xml` | Family kitchen with island | 175 |
| `04_small_warehouse.xml` | Compact warehouse | 164 |
| `05_pallet_warehouse.xml` | Pallet warehouse | 358 |
| `06_retail_aisle.xml` | Retail shelving aisles | 413 |
| `07_cluttered_warehouse.xml` | Cluttered warehouse | 521 |
| `08_office.xml` | Office room | 7 |
| `09_corridor.xml` | Long corridor | 7 |
| `10_atrium.xml` | Tall atrium room | 7 |

## Robots

The `extensions/robot-gallery` package defines 20 structural families. Its WebAssembly module was built and validated, then enabled in RobotLens. MCP created one configured extension instance per family and generated the 20 robots recorded in `robots/gallery-manifest.json`. Each robot passed `robot.validate` after generation and after reopening the saved project. The project contains exactly those 20 gallery documents, each with its extension package, configuration, and seed in the saved metadata.

`robots/legacy-authored-robots.json` preserves the 16 earlier built-in recipe snapshots. Their saved documents were compared with the archive before removal. The archive retains their IDs and geometry if they need to be restored later.

The generated robots are editable geometry and kinematic layouts. MCP editing, undo and redo, explicit regeneration, and duplication of a generated robot were exercised in the live project. Focused CTest suites and native Inspector, picking, and gizmo acceptance checks remain for the user's verification.

These are editable geometry and kinematic layouts. A name such as SCARA, drone, or exoskeleton does not imply a controller, physical safety validation, or complete domain dynamics.
