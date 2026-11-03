# RobotLens-Projects

Each example opens from its project directory and stores its scene in
`.robotlens/`. Asset, simulation and navigation paths are relative to that
project directory. Shared robot mesh paths are relative to the model file.
Keep the complete folder tree together when moving this repository.

From an example directory, run its existing workflow with:

```bash
python3 scripts/quickstart.py
```

The workflow checks prerequisites, opens the project in RobotLens, runs its
scenario, records evidence, and stops its simulation and any application session
it starts.
Results are written to `evidence/quickstart/`. Aggregate run reports are kept in
the ignored `reports/` directory.

Run every project's quickstart sequentially from this repository:

```bash
python3 tools/run_all_projects.py
```

This uses the existing built RobotLens and sibling `RobotLens-Automation`
harness. Close any running RobotLens session first. Each run gets its own
`reports/quickstarts-<timestamp>/` directory with Markdown/JSON summaries and
per-project console logs and reports. Failed scenarios remain failed; the batch
continues unless cleanup fails. The exit code is 0 when all projects pass,
1 for failed checks, 2 for startup/cleanup errors, and 130 when interrupted.

List projects or rerun selected ones:

```bash
python3 tools/run_all_projects.py --list
python3 tools/run_all_projects.py --projects ros-mujoco-panda-ground-plane zenoh-mujoco-panda-ground-plane
```

Use `--timeout SECONDS` to change the 1,800-second per-project limit,
`--stop-on-failure` to stop after a failed workflow, and `--report-dir PATH`
to select a new report directory. The runner finds the repository from its
own file location, so the command also works after moving the folder tree.

Python virtual environments retain absolute launcher paths. After moving one,
recreate it or update its activation scripts and entry-point interpreter paths.
Generated build/install directories must be regenerated rather than rebased.
