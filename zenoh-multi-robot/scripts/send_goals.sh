#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
mcp=(python3 scripts/mcp.py)
names=(Mobile_manipulator Mobile_manipulator_copy Mobile_manipulator_copy_2 Mobile_manipulator_copy_3 Mobile_manipulator_copy_4)
goals=("6.5 0 0" "-3.6 4.5 1.57079632679" "3.6 -4.5 -1.57079632679" "6.5 4.5 0" "-6.5 -4.5 3.14159265359")
declare -A aliases
for index in $(seq 0 4); do
    aliases[${names[$index]}]=$("${mcp[@]}" scene.find "{\"target\":\"Robots/${names[$index]}\"}" | python3 -c 'import json,sys; print((lambda d: (d.get("candidates") or d.get("items"))[0]["alias"])(json.load(sys.stdin)))')
    read -r x y yaw <<< "${goals[$index]}"
    "${mcp[@]}" robot.navigation.goal "{\"target\":\"${aliases[${names[$index]}]}\",\"x\":$x,\"y\":$y,\"yaw\":$yaw}" >/dev/null
done
while :; do
    done_count=0
    printf '%-8s %-12s %-16s\n' Robot State Last_pose
    for index in $(seq 0 4); do
        result=$("${mcp[@]}" robot.navigation.status "{\"target\":\"${aliases[${names[$index]}]}\"}")
        row=$(printf '%s' "$result" | python3 -c 'import json,sys; d=json.load(sys.stdin).get("data",{}); print(d.get("state","unknown"), json.dumps(d.get("last_pose")))')
        read -r state pose <<< "$row"
        printf '%-8s %-12s %-16s\n' "robot$((index + 1))" "$state" "$pose"
        [[ $state == succeeded || $state == failed || $state == cancelled ]] && ((done_count+=1))
    done
    ((done_count == 5)) && break
    sleep 1
done
