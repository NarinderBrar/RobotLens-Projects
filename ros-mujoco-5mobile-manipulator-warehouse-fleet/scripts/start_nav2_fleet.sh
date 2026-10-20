#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
mcp=(python3 scripts/mcp.py)
names=(Mobile_manipulator Mobile_manipulator_copy Mobile_manipulator_copy_2 Mobile_manipulator_copy_3 Mobile_manipulator_copy_4)
for name in "${names[@]}"; do
    alias=$("${mcp[@]}" scene.find "{\"target\":\"Robots/$name\"}" | python3 -c 'import json,sys; print((lambda d: (d.get("candidates") or d.get("items"))[0]["alias"])(json.load(sys.stdin)))')
    "${mcp[@]}" robot.navigation.start "{\"target\":\"$alias\"}"
done
