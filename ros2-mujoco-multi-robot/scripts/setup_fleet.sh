#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
python3 scripts/generate_warehouse_map.py >/dev/null
mcp=(python3 scripts/mcp.py)
project_file="$project_root/.robotlens/ros2-mujoco-multi-robot.xml"
"${mcp[@]}" project.open "{\"path\":\"$project_file\"}" >/dev/null
for _ in $(seq 1 60); do
    "${mcp[@]}" project.status '{}' 2>/dev/null | grep -q '"state": "opened"' && break
    sleep 2
done
names=(Mobile_manipulator Mobile_manipulator_copy Mobile_manipulator_copy_2 Mobile_manipulator_copy_3 Mobile_manipulator_copy_4)
find_robot() {
    "${mcp[@]}" scene.find "{\"target\":\"Robots/$1\"}" | python3 -c 'import json,sys; d=json.load(sys.stdin); matches=[c["alias"] for c in (d.get("candidates") or d.get("items") or []) if c.get("path", "").rstrip("/").split("/")[-1] == sys.argv[1]]; print(matches[0] if len(matches) == 1 else "")' "$1"
}
for name in "${names[@]}"; do
    [[ -n $(find_robot "$name") ]] && continue
    "${mcp[@]}" model.import '{"path":"robots/Mobile_manipulator.urdf"}' >/dev/null
    for _ in $(seq 1 60); do
        "${mcp[@]}" model.status '{}' 2>/dev/null | grep -qE '"busy": false|"state": "idle"' && break
        sleep 2
    done
done
declare -A aliases
poses=("-6.5 0 0" "-3.6 -4.5 90" "3.6 4.5 -90" "-6.5 4.5 0" "6.5 -4.5 180")
for index in $(seq 0 4); do
    name=${names[$index]}
    aliases[$name]=$(find_robot "$name")
    [[ -n ${aliases[$name]} ]] || { printf 'Missing or ambiguous robot: %s\n' "$name" >&2; exit 1; }
    read -r x y yaw <<< "${poses[$index]}"
    robot=$((index + 1))
    "${mcp[@]}" scene.move "{\"target\":\"${aliases[$name]}\",\"x\":$x,\"y\":$y}" >/dev/null
    "${mcp[@]}" scene.rotate "{\"target\":\"${aliases[$name]}\",\"x\":0,\"y\":0,\"z\":$yaw}" >/dev/null
    "${mcp[@]}" robot.navigation.configure "{\"target\":\"${aliases[$name]}\",\"stack_kind\":\"nav2\",\"namespace_prefix\":\"/robot$robot\",\"nav2\":{\"launch_package\":\"\",\"launch_file\":\"nav2_config/nav2_robot_launch.py\",\"map_yaml\":\"nav2_config/warehouse_navigation_map.yaml\",\"params_yaml\":\"nav2_config/nav2_params.yaml\",\"args\":[\"start_x:=$x\",\"start_y:=$y\",\"start_yaw:=$yaw\"],\"attach_only\":false}}" >/dev/null
done
mapfile -t lidars < <("${mcp[@]}" scene.find '{"target":"2D LiDAR"}' | python3 -c 'import json,sys; seen=set(); [print(("dup" if c["path"].split("/")[1] in seen else "keep"), c["alias"], c["path"].split("/")[1]) or seen.add(c["path"].split("/")[1]) for c in (lambda d: d.get("candidates") or d.get("items") or [])(json.load(sys.stdin))]')
duplicates=()
for item in "${lidars[@]}"; do
    read -r kind alias owner <<< "$item"
    [[ $kind == dup ]] && duplicates+=("\"$alias\"")
done
if ((${#duplicates[@]})); then
    targets=$(IFS=,; echo "${duplicates[*]}")
    "${mcp[@]}" scene.delete "{\"target\":[$targets]}" >/dev/null
fi
for item in "${lidars[@]}"; do
    read -r kind alias owner <<< "$item"
    [[ $kind == keep ]] || continue
    lidar_topic=""
    for index in "${!names[@]}"; do
        [[ ${names[$index]} == "$owner" ]] && lidar_topic="/robot$((index + 1))/scan"
    done
    [[ -n $lidar_topic ]] || { printf 'Unknown LiDAR owner: %s\n' "$owner" >&2; exit 1; }
    "${mcp[@]}" scene.set_properties "{\"target\":\"$alias\",\"properties\":{\"lidar_simulation\":\"ray_based\",\"range_min\":0.1,\"enabled\":true,\"topic\":\"$lidar_topic\"}}" >/dev/null
    "${mcp[@]}" scene.move "{\"target\":\"$alias\",\"x\":0,\"y\":0,\"z\":0.04}" >/dev/null
    "${mcp[@]}" scene.rotate "{\"target\":\"$alias\",\"x\":0,\"y\":0,\"z\":0}" >/dev/null
done
"${mcp[@]}" comms.start '{}' >/dev/null
"${mcp[@]}" simulation.play '{}' >/dev/null
for name in "${names[@]}"; do
    "${mcp[@]}" robot.simulation_bridge "{\"target\":\"${aliases[$name]}\",\"publish_states\":true,\"accept_commands\":true}" >/dev/null
done
printf 'Fleet configured with five enabled namespaced LiDARs. Run scripts/start_nav2_fleet.sh.\n'
