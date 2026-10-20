#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
mcp=(python3 scripts/mcp.py)
names=(Mobile_manipulator Mobile_manipulator_copy Mobile_manipulator_copy_2 Mobile_manipulator_copy_3 Mobile_manipulator_copy_4)
goals=("6.5 0 0" "-3.6 4.5 1.57079632679" "3.6 -4.5 -1.57079632679" "6.5 4.5 0" "-6.5 -4.5 3.14159265359")
for index in $(seq 0 4); do
    alias=$("${mcp[@]}" scene.find "{\"target\":\"Robots/${names[$index]}\"}" | python3 -c 'import json,sys; print((lambda d: (d.get("candidates") or d.get("items"))[0]["alias"])(json.load(sys.stdin)))')
    read -r x y yaw <<< "${goals[$index]}"
    "${mcp[@]}" robot.navigation.goal "{\"target\":\"$alias\",\"x\":$x,\"y\":$y,\"yaw\":$yaw}"
done
for index in $(seq 0 4); do
    alias=$("${mcp[@]}" scene.find "{\"target\":\"Robots/${names[$index]}\"}" | python3 -c 'import json,sys; print((lambda d: (d.get("candidates") or d.get("items"))[0]["alias"])(json.load(sys.stdin)))')
    "${mcp[@]}" robot.navigation.status "{\"target\":\"$alias\"}"
done
