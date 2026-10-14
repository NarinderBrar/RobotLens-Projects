#!/usr/bin/env bash
set -euo pipefail

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
stack_bin=${ZMS_BIN:-$project_dir/../../motion-stacks/build/system}
log_dir=$project_dir/evidence/logs
mkdir -p "$log_dir"

"$stack_bin/zms_navigation" --check-config "$project_dir/stacks/navigation.json"
"$stack_bin/zms_manipulation" --check-config "$project_dir/stacks/manipulation.json"

"$stack_bin/zms_navigation" --config "$project_dir/stacks/navigation.json" >"$log_dir/navigation.log" 2>&1 &
navigation_pid=$!
"$stack_bin/zms_manipulation" --config "$project_dir/stacks/manipulation.json" >"$log_dir/manipulation.log" 2>&1 &
manipulation_pid=$!

stop_stacks() {
    kill -TERM "$navigation_pid" "$manipulation_pid" 2>/dev/null || true
    wait "$navigation_pid" "$manipulation_pid" 2>/dev/null || true
}
trap stop_stacks EXIT INT TERM
printf 'navigation pid %s, manipulation pid %s; press Ctrl+C to stop\n' "$navigation_pid" "$manipulation_pid"
wait
