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

Python virtual environments retain absolute launcher paths. After moving one,
recreate it or update its activation scripts and entry-point interpreter paths.
Generated build/install directories must be regenerated rather than rebased.
